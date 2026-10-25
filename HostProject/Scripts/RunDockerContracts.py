"""Portable IAmSpeed CI: one immutable UE image plus an exact plugin commit.

No host Unreal installation, private UBT manifest, automatic pull, or service
management. Docker source/input is read-only; generated files stay in one run.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import shutil
import subprocess
import sys
import tarfile
import uuid
# Isolated Python (-I) omits the script directory from sys.path.
# Bind the helper to the same immutable source directory explicitly.
import importlib.util
_cadence_spec = importlib.util.spec_from_file_location("IAmSpeedCadenceDebt", Path(__file__).resolve().with_name("CadenceDebt.py"))
_cadence_module = importlib.util.module_from_spec(_cadence_spec)
_cadence_spec.loader.exec_module(_cadence_module)
check_cadence_policy = _cadence_module.check_policy
project_cadence = _cadence_module.project_cadence
_matrix_spec = importlib.util.spec_from_file_location("IAmSpeedCollisionMatrix042", Path(__file__).resolve().with_name("CollisionMatrix042.py"))
_matrix_module = importlib.util.module_from_spec(_matrix_spec)
_matrix_spec.loader.exec_module(_matrix_module)
validate_collision_matrix = _matrix_module.validate_artifacts
diagnose_collision_matrix = _matrix_module.diagnose_artifacts

RETAINED_TESTS = {'SkyLeague.Camera.GenericInputLifecycle'}
FILTER = 'IAmSpeed.' + ''.join('+' + name for name in sorted(RETAINED_TESTS))
REQUIRED = {
    'IAmSpeed.Collision.SubBodyMatrix.SensorPeers',
    'IAmSpeed.Collision.SubBodyMatrix.CylinderPeers',
    'IAmSpeed.ContactDetection.SweeperCandidates',
    'IAmSpeed.Collision.Cylinder.MovingCCD',
    'IAmSpeed.Collision.Torus.WorldProvider',
}
ROOTS = ('/opt/unreal', '/opt/UnrealEngine', '/opt/unreal-engine', '/home/ue4/UnrealEngine')
ACCEPTED_DEBT_IDS = {
    'IAmSpeed.Camera.NetworkInputBoundary',
    'IAmSpeed.PhysicalLaws.BoxEquilibrium.CanonicalDrops',
}


def save(path, data):
    Path(path).write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def validate_report(exit_code, log, report, required=REQUIRED, expected_paths=None):
    if exit_code != 0:
        raise ValueError('Editor exited with code ' + str(exit_code))
    # A completed queue does not prove that shutdown was crash-free. Exit 1
    # may also accompany accepted debt, so check fatal diagnostics independently.
    crash = re.search(r'\b(?:Fatal error[!:]|Unhandled Exception:|Unhandled signal\b|Assertion failed:|Segmentation fault\b|LowLevelFatalError\b)', log, re.IGNORECASE)
    if crash:
        raise ValueError('Native crash diagnostic: ' + crash.group(0))
    # Startup/shutdown ensures can occur outside any automation report row.
    # A successful queue or an accepted test debt must not hide them.
    ensure = re.search(r'\bEnsure condition failed:|===\s*Handled ensure\s*:?\s*===', log, re.IGNORECASE)
    if ensure:
        raise ValueError('Native ensure diagnostic: ' + ensure.group(0))
    markers = re.findall(r'Automation Test Queue Empty (\d+) tests performed\.', log)
    if len(markers) != 1 or int(markers[0]) <= 0:
        raise ValueError('Missing, empty, or ambiguous automation completion')
    count = int(markers[0])
    tests = report.get('tests', [])
    paths = [t.get('fullTestPath', '') for t in tests]
    if len(tests) != count or len(set(paths)) != count:
        raise ValueError('Queue/report cardinality or unique identity mismatch')
    prefixes = [p.rstrip('.') for p in FILTER.split('+')]
    if any(not any(p == prefix or p.startswith(prefix + '.') for prefix in prefixes) for p in paths):
        raise ValueError('Report contains a foreign test family')
    if any(t.get('state') != 'Success' or int(t.get('warnings', 0)) or int(t.get('errors', 0)) for t in tests):
        raise ValueError('Tests contain failures, warnings or incomplete states')
    for key, value in [('succeeded', count), ('succeededWithWarnings', 0), ('failed', 0), ('notRun', 0), ('inProcess', 0)]:
        if report.get(key) != value:
            raise ValueError('Invalid automation aggregate: ' + key)
    if not set(required).issubset(paths):
        raise ValueError('Required SubBody/geometry contract missing: ' + str(sorted(set(required) - set(paths))))
    if expected_paths is not None and set(paths) != set(expected_paths):
        raise ValueError('Exact checkout test manifest differs: missing=' + str(sorted(set(expected_paths)-set(paths))) + ', unexpected=' + str(sorted(set(paths)-set(expected_paths))))
    return {'tests': count, 'identities': sorted(paths)}


def source_test_manifest(root):
    # This checkout declares static SIMPLE tests. Refuse a future dynamic test
    # registration until its enumeration is implemented, rather than omit it.
    names = []
    pattern = r'IMPLEMENT_SIMPLE_AUTOMATION_TEST\s*\(\s*\w+\s*,\s*"([^"]+)"'
    for path in Path(root).rglob('*.cpp'):
        code = path.read_text(encoding='utf-8-sig')
        # The Docker target is Linux. OwnedFrameAudit's eight Windows-only
        # native sidecar tests are not compiled there. Track the exact guarded
        # regions; never remove a report failure or infer eligibility from the
        # runtime result. Unknown platform guards around tests fail closed.
        frames, filtered = [], []
        for line in code.splitlines(keepends=True):
            directive = re.match(r'^\s*#\s*(if|ifdef|ifndef|elif|else|endif)\b(.*)', line)
            if directive:
                op, expression = directive.groups()
                if op in ('if', 'ifdef', 'ifndef'):
                    windows = expression.strip() == 'WITH_DEV_AUTOMATION_TESTS && PLATFORM_WINDOWS'
                    frames.append(dict(skip=windows, windows=windows,
                                       unknown_platform='PLATFORM_' in expression and not windows))
                elif op in ('else', 'elif') and frames:
                    frame = frames[-1]
                    if frame['windows']:
                        if op == 'elif':
                            raise ValueError('Windows test guard needs explicit Linux elif handling: ' + str(path))
                        frame['skip'] = not frame['skip']
                    elif 'PLATFORM_' in expression:
                        frame['unknown_platform'] = True
                elif op == 'endif' and frames:
                    frames.pop()
            skipped = any(frame['skip'] for frame in frames)
            if not skipped and 'IMPLEMENT_SIMPLE_AUTOMATION_TEST' in line and any(frame['unknown_platform'] for frame in frames):
                raise ValueError('Platform test registration needs explicit Linux enumeration: ' + str(path))
            filtered.append('\n' if skipped else line)
        code = ''.join(filtered)
        if re.search(r'\b(?:IMPLEMENT_COMPLEX_AUTOMATION_TEST|IMPLEMENT_CUSTOM_SIMPLE_AUTOMATION_TEST|BEGIN_DEFINE_SPEC|DEFINE_SPEC)\s*\(', code):
            raise ValueError('Dynamic test registration needs explicit enumeration: ' + str(path))
        matches = re.findall(pattern, code)
        if len(matches) != len(re.findall(r'\bIMPLEMENT_SIMPLE_AUTOMATION_TEST\s*\(', code)):
            raise ValueError('Unrecognized static test identity: ' + str(path))
        if any(not name.startswith('IAmSpeed.') and name not in RETAINED_TESTS for name in matches):
            raise ValueError('Repository test needs an explicit namespace/filter declaration: ' + str(path))
        names.extend(matches)
    if len(names) != len(set(names)) or not REQUIRED.issubset(names):
        raise ValueError('Source test manifest is incomplete or has duplicate identities')
    return sorted(names)


def accepted_debt_policy():
    policy = json.loads(Path(__file__).with_name('accepted-test-debt.json').read_text(encoding='utf-8'))
    if (policy.get('schema') != 'iamspeed.accepted-test-debt/v1'
            or set(policy.get('tests', {})) != ACCEPTED_DEBT_IDS
            or policy.get('authority_date') != '2026-10-20'
            or policy.get('execute_tests') is not True
            or policy.get('preserve_raw_failures') is not True):
        raise ValueError('Expected the exact two human-accepted test debts; no wildcard or skipped test')
    return policy


def validate_report_with_debt(exit_code, log, report, policy, required=REQUIRED, expected_paths=None):
    """Assess integration separately; never rewrite the raw UE report or test result.

    Only completed Fail rows for the two explicit debts may be nonblocking.
    Incomplete tests, another failure, non-debt warnings and a crash stay blocking.
    """
    if report.get('succeededWithWarnings') != 0 or any(
            row.get('warnings') != 0 or any(e.get('event', {}).get('type') == 'Warning'
                for e in row.get('entries', [])) for row in report.get('tests', [])):
        raise ValueError('Raw warnings remain blocking, including accepted Fail rows')
    if set(policy.get('tests', {})) != ACCEPTED_DEBT_IDS:
        raise ValueError('Debt identities differ from human authorization')
    accepted = [row for row in report.get('tests', [])
                if row.get('fullTestPath') in ACCEPTED_DEBT_IDS and row.get('state') == 'Fail']
    if exit_code not in (0, 1) or (exit_code != 0 and not accepted):
        raise ValueError('Native exit is not a completed accepted-debt test failure: ' + str(exit_code))
    # A private assessment copy lets the existing strict manifest/completion/
    # aggregate checks apply unchanged to every other row. Raw artifacts stay Fail.
    assessment = copy.deepcopy(report)
    for row in assessment.get('tests', []):
        if row.get('fullTestPath') in ACCEPTED_DEBT_IDS and row.get('state') == 'Fail':
            row.update(state='Success', warnings=0, errors=0)
    assessment['failed'] = report.get('failed', -1) - len(accepted)
    assessment['succeeded'] = report.get('succeeded', -1) + len(accepted)
    validated = validate_report(0, log, assessment, required, expected_paths)
    return dict(validated, state='PASS_WITH_ACCEPTED_DEBT' if accepted else 'PASS',
                accepted_debt=[dict(test=row['fullTestPath'], raw_state=row['state'],
                                    errors=row.get('errors', 0), warnings=row.get('warnings', 0),
                                    reason=policy['tests'][row['fullTestPath']]['reason']) for row in accepted],
                raw_editor_exit=exit_code, raw_report_unchanged=True,
                full_ci_qualified=not accepted)


def additive_cadence_policy():
    policy = json.loads(Path(__file__).with_name('accepted-cadence-debt.json').read_text(encoding='utf-8'))
    check_cadence_policy(policy)
    return policy


def validate_report_with_additive_debt(exit_code, log, report, policy, cadence_policy,
                                     evidence_root, required=REQUIRED, expected_paths=None):
    # Default off: preserve the exact pre-existing two-Fail-debt contract.
    if not check_cadence_policy(cadence_policy):
        return validate_report_with_debt(exit_code, log, report, policy, required, expected_paths)
    # Cadence cannot hide a new diagnostic. The already-authorized v1 completed
    # Fail identities retain their unchanged existing assessment below.
    for row in report.get('tests', []):
        old_debt = row.get('fullTestPath') in ACCEPTED_DEBT_IDS and row.get('state') == 'Fail'
        if not old_debt and (type(row.get('errors')) is not int or row['errors'] < 0):
            raise ValueError('Missing or invalid raw error counter')
        if not old_debt and any(entry.get('event', {}).get('type') in ('Error', 'Fatal')
                                for entry in row.get('entries', [])):
            raise ValueError('New raw error remains blocking outside existing v1 debt')
    assessment, events = project_cadence(report, evidence_root)
    result = validate_report_with_debt(exit_code, log, assessment, policy, required, expected_paths)
    if events:
        counts = {}
        for event in events:
            counts[event['test']] = counts.get(event['test'], 0) + 1
        result['accepted_debt'].extend(dict(test=test, kind='additive_observer_cadence',
                                           warning_count=count, raw_state='Success')
                                       for test, count in sorted(counts.items()))
        result.update(state='PASS_WITH_ACCEPTED_DEBT', full_ci_qualified=False)
    result.update(accepted_cadence=events, accepted_cadence_count=len(events),
                  cadence_policy=cadence_policy, raw_report_unchanged=True)
    return result


def extract_archive(archive, destination):
    destination = Path(destination)
    destination.mkdir()
    with tarfile.open(archive) as source:
        for member in source:
            relative = PurePosixPath(member.name)
            windows = PureWindowsPath(member.name)
            if relative.is_absolute() or windows.drive or windows.root or '\\' in member.name or '..' in relative.parts or member.issym() or member.islnk():
                raise ValueError('Unsafe source archive member: ' + member.name)
            target = destination / relative
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            elif member.isfile():
                target.parent.mkdir(parents=True, exist_ok=True)
                with source.extractfile(member) as incoming, target.open('wb') as output:
                    shutil.copyfileobj(incoming, output)
            else:
                raise ValueError('Unsupported source archive member: ' + member.name)


def engine_root():
    candidates = [os.environ.get(n) for n in ('UE_ROOT', 'UE_ENGINE_ROOT', 'UNREAL_ENGINE_ROOT')]
    candidates.extend(ROOTS)
    for candidate in candidates:
        if candidate and (Path(candidate) / 'Engine/Build/Build.version').is_file():
            return Path(candidate)
    raise RuntimeError('The UE image must expose UE_ROOT or a standard Engine root; no host Engine is used')


def run_logged(command, log, timeout, cwd, env):
    with Path(log).open('wb') as output:
        result = subprocess.run(command, cwd=cwd, env=env, stdout=output, stderr=subprocess.STDOUT, timeout=timeout)
    return result.returncode



def cpuset_ids(spec):
    if type(spec) is not str or not re.fullmatch(r'[0-9]+(?:-[0-9]+)?(?:,[0-9]+(?:-[0-9]+)?)*', spec):
        raise ValueError('CPU set must contain only nonnegative CPU IDs or ascending ranges')
    selected = set()
    for item in spec.split(','):
        ends = item.split('-')
        first, last = int(ends[0]), int(ends[-1])
        if first > last or last > 65535 or last-first > 4095:
            raise ValueError('Invalid or unbounded CPU range')
        values = set(range(first, last+1))
        if selected.intersection(values):
            raise ValueError('Duplicate CPU IDs are ambiguous')
        selected.update(values)
    return sorted(selected)


def canonical_cpuset(spec, cpus):
    selected = cpuset_ids(spec)
    if len(selected) != cpus:
        raise ValueError('Explicit CPU set cardinality must match the unchanged CPU quota')
    return ','.join(map(str, selected))


def cpu_affinity_evidence(spec, cpus):
    selected = cpuset_ids(canonical_cpuset(spec, cpus))
    if not hasattr(os, 'sched_getaffinity') or sorted(os.sched_getaffinity(0)) != selected:
        raise ValueError('Linux effective affinity does not match the explicit CPU set')
    rows = []
    for cpu in selected:
        topology = Path('/sys/devices/system/cpu/cpu' + str(cpu) + '/topology')
        package = int((topology / 'physical_package_id').read_text().strip())
        core = int((topology / 'core_id').read_text().strip())
        die_file = topology / 'die_id'
        die = int(die_file.read_text().strip()) if die_file.is_file() else None
        siblings = cpuset_ids((topology / 'thread_siblings_list').read_text().strip())
        if package < 0 or core < 0 or (die is not None and die < 0) or cpu not in siblings:
            raise ValueError('Unknown Linux guest core topology')
        if set(siblings).intersection(selected) != {cpu}:
            raise ValueError('Explicit CPU set selects SMT siblings of the same guest core')
        rows.append(dict(cpu=cpu, package=package, die=die, core=core, thread_siblings=siblings))
    if len({(r['package'], r['die'], r['core']) for r in rows}) != len(rows):
        raise ValueError('Explicit CPU set does not select distinct Linux guest cores')
    return dict(state='VERIFIED_EXPLICIT_DISTINCT_LINUX_GUEST_CORES', effective_affinity=selected,
                topology=rows, quota_cpus=cpus, exclusivity_claimed=False,
                Windows_host_physical_mapping='UNKNOWN')


def work_tmpfs_evidence(work):
    # This is a transport envelope, not a physical acceptance threshold.
    fs = os.statvfs(work)
    if fs.f_blocks * fs.f_frsize != 14 * 1024**3:
        raise RuntimeError('Native work requires the bounded 14 GiB tmpfs')
    if not hasattr(os, 'ST_NOEXEC') or fs.f_flag & os.ST_NOEXEC:
        raise RuntimeError('Executable native work tmpfs was not proved')
    mounts = [line for line in Path('/proc/self/mountinfo').read_text().splitlines()
              if len(line.split(' - ')[0].split()) > 4
              and line.split(' - ')[0].split()[4] == str(work)]
    if len(mounts) != 1 or mounts[0].partition(' - ')[2].split()[0] != 'tmpfs':
        raise RuntimeError('Native work is not the exact private Linux tmpfs mount')
    return dict(capacity_bytes=fs.f_blocks * fs.f_frsize, mountinfo=mounts[0], noexec=False)


def artifact_files(work):
    # Preserve raw failures too. Build caches/DDC/home are disposable, while
    # source.tar and the exact committed input remain in the host run metadata.
    roots = ('Logs', 'Automation', 'CollisionMatrix042', 'user/Saved', 'HostProject/Saved',
             'HostProject/Binaries', 'HostProject/ExternalPlugins/IAmSpeed/Binaries',
             'HostProject/ExternalPlugins/IAmSpeed/Saved')
    files = list(work.glob('*.json'))
    for name in roots:
        root = work / name
        if root.is_symlink():
            raise RuntimeError('Symlink export root is not dereferenced: ' + name)
        if root.exists():
            entries = list(root.rglob('*'))
            if any(path.is_symlink() for path in entries):
                raise RuntimeError('Symlink in native export tree: ' + name)
            files.extend(path for path in entries if path.is_file())
    files = sorted(set(files))
    for path in files:
        if path.is_symlink() or not path.is_file() or any(
                parent.is_symlink() for parent in path.parents if parent != work.parent):
            raise RuntimeError('Unsafe native export source: ' + str(path))
    return files


def export_artifacts(work, export, editor_started, editor_call_finished, native_timeout=False):
    # subprocess.run returns/reaps its direct child, including on timeout;
    # this marker does not establish closure of arbitrary descendant writers.
    if editor_started and not editor_call_finished:
        raise RuntimeError('Native Editor call has not finished; export is forbidden')
    files = artifact_files(work)
    for path in files:
        target = export / path.relative_to(work)
        if any(parent.is_symlink() for parent in target.parents if parent != export.parent):
            raise RuntimeError('Symlink native export destination: ' + str(target))
        if target.exists() or target.is_symlink():
            raise RuntimeError('Native export would overwrite retained host evidence: ' + str(target))
    before = [dict(path=path.relative_to(work).as_posix(), bytes=path.stat().st_size,
                   sha256=digest(path)) for path in files]
    for path in files:
        target = export / path.relative_to(work)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
    after = [dict(path=path.relative_to(work).as_posix(), bytes=path.stat().st_size,
                  sha256=digest(path)) for path in files]
    copied = [dict(path=row['path'], bytes=(export/row['path']).stat().st_size,
                   sha256=digest(export/row['path'])) for row in before]
    if before != after or before != copied:
        raise RuntimeError('Native export source/destination byte identity differs')
    if artifact_files(work) != files:
        raise RuntimeError('Native export source file set changed during export')
    save(export / 'export-manifest.json', dict(state='VERIFIED_AFTER_NATIVE_CALL', files=before,
         editor_started=editor_started, editor_call_finished=editor_call_finished,
         source_before_equals_after=True, destination_verified=True,
         source_file_set_before_equals_after=True, native_call_closure_scope='DIRECT_CHILD_ONLY',
         export_completeness='UNKNOWN_TIMEOUT_DIRECT_CHILD_ONLY' if native_timeout else
                             'REINVENTORIED_SELECTED_ROOTS_AFTER_DIRECT_CHILD',
         excluded=['home', 'config', 'cache', 'dotnet', 'DDC', 'Intermediate', 'immutable image SDK']))


def inside():
    output = Path('/work')
    editor_started = editor_call_finished = False
    result = {'state': 'FAIL', 'source_sha': os.environ['IAMSPEED_SOURCE_SHA'], 'image_id': os.environ['IAMSPEED_IMAGE_ID']}
    try:
        if hasattr(os, 'geteuid') and os.geteuid() == 0:
            raise RuntimeError('Unreal must run as a non-root user supplied by the image')
        save(output / 'work-tmpfs.json', work_tmpfs_evidence(output))
        spec = os.environ.get('IAMSPEED_CPUSET_CPUS')
        affinity = cpu_affinity_evidence(spec, int(os.environ['IAMSPEED_CPUSET_QUOTA'])) if spec else {'state': 'NOT_REQUESTED'}
        save(output / 'cpu-affinity.json', affinity)
        engine = engine_root()
        version = json.loads((engine / 'Engine/Build/Build.version').read_text())
        if tuple(version.get(k) for k in ('MajorVersion', 'MinorVersion', 'PatchVersion')) != (5, 8, 2):
            raise RuntimeError('The immutable image does not contain UE 5.8.2')
        if version.get('CompatibleChangelist') != 55116800:
            raise RuntimeError('UE 5.8.2 compatible changelist differs from the physical contract pin')
        build = engine / 'Engine/Build/BatchFiles/Linux/Build.sh'
        editor = next((engine / ('Engine/Binaries/Linux/' + name) for name in ('UnrealEditor-Cmd', 'UnrealEditor') if (engine / ('Engine/Binaries/Linux/' + name)).is_file()), None)
        if not build.is_file() or editor is None:
            raise RuntimeError('Image requires a prebuilt Linux Editor and its matching build toolchain')
        project = output / 'HostProject'
        shutil.copytree('/input/HostProject', project, ignore=shutil.ignore_patterns('Scripts', 'Binaries', 'Intermediate', 'Saved'))
        plugin = project / 'ExternalPlugins/IAmSpeed'
        shutil.copytree('/input', plugin, ignore=shutil.ignore_patterns('HostProject', '.git', 'Binaries', 'Intermediate', 'Saved', 'Artifacts'))
        expected = source_test_manifest(plugin / 'Source')
        if _matrix_module.IDENTITY not in expected:
            raise RuntimeError('The native single-world collision matrix042 is missing from the exact source test manifest')
        save(output / 'expected-tests.json', expected)
        env = os.environ.copy()
        for name, directory in [('HOME', 'home'), ('XDG_CONFIG_HOME', 'config'), ('XDG_CACHE_HOME', 'cache'), ('DOTNET_CLI_HOME', 'dotnet')]:
            path = output / directory; path.mkdir(exist_ok=True); env[name] = str(path)
        # Keep .NET FIFOs and native temporary files on Linux tmpfs, not the host bind.
        env['TMPDIR'] = '/tmp'
        # Linux Editor probes its user Documents filesystem during automation.
        # Keep that real directory inside this run, including on headless hosts.
        (output / 'home/Documents').mkdir(parents=True, exist_ok=True)
        for name, value in {'UE-LocalDataCachePath': str(output / 'DDC'), 'UE-SharedDataCachePath': 'None', 'DOTNET_SKIP_FIRST_TIME_EXPERIENCE': '1', 'DOTNET_CLI_TELEMETRY_OPTOUT': '1'}.items():
            env[name] = value
        logs = output / 'Logs'; logs.mkdir()
        report = output / 'Automation'; report.mkdir()
        project_file = project / 'IAmSpeedHostProject.uproject'
        command = ['bash', str(build), 'IAmSpeedHostProjectEditor', 'Linux', 'Development', '-Project=' + str(project_file), '-UsePrecompiled', '-NoUBA', '-NoXGE', '-NoHotReload', '-MaxParallelActions=2', '-Log=' + str(logs / 'ubt.log')]
        save(output / 'build-command.json', command)
        code = run_logged(command, logs / 'build.stdout.log', 5400, project, env)
        result['build_exit'] = code
        if code:
            raise RuntimeError('HostProject compilation failed; original log preserved')
        # One Automation command queues SoftQuit after RunTests. Unlike TestExit,
        # this preserves report error status and performs normal Linux cleanup.
        command = [str(editor), str(project_file), '-NullRHI', '-SLPortableHeadlessSDL', '-sdlvideodriver=dummy', '-unattended', '-MULTIHOME=127.0.0.1', '-NoEOS', '-nosound', '-nop4', '-nosplash', '-NoCompile', '-NoCompileEditor', '-NoLiveCoding', '-NoShaderCompile', '-notraceserver', '-NoTrace', '-ini:Engine:[ConsoleVariables]:HomeScreen.EnableHomeScreen=0', '-ini:EditorSettings:[/Script/UnrealEd.AnalyticsPrivacySettings]:bSendUsageData=False', '-stdout', '-FullStdOutLogOutput', '-UserDir=' + str(output / 'user'), '-abslog=' + str(logs / 'editor.log'), '-ReportExportPath=' + str(report), '-ExecCmds=Automation RunTests ' + FILTER + '; SoftQuit']
        # The native test creates its own fresh evidence directory and runs
        # the complete matrix in one world. No smoke-only option is passed.
        command.append('-IAmSpeedCollisionMatrix042Output=' + str(output / 'CollisionMatrix042'))
        # Stock cache-write opt-out: gathering/reads remain available. Preserve
        # the same C++ startup locale used by the private Windows validation.
        command.extend(['-NoAssetRegistryCacheWrite', '-locale=en-US', '-DisablePython'])
        save(output / 'test-command.json', command)
        # The embedded Python home may be relative to the Editor executable.
        # Keep project/output arguments absolute and resolve that SDK layout.
        editor_started = True
        try:
            code = run_logged(command, logs / 'tests.stdout.log', 1800, editor.parent, env)
        finally:
            # Direct child only: timeout does not prove descendant-writer drain.
            editor_call_finished = True
        result['editor_exit'] = code
        try:
            diagnostics = diagnose_collision_matrix(output / 'CollisionMatrix042')
        except Exception as exc:
            diagnostics = dict(state='DIAGNOSTICS_UNAVAILABLE', reason=str(exc),
                               qualification_granted=False)
        save(output / 'collision-matrix042-diagnostics.json', diagnostics)
        matrix_error = None
        try:
            matrix_result = validate_collision_matrix(output / 'CollisionMatrix042')
        except Exception as exc:
            matrix_error = str(exc)
            matrix_result = dict(state='FAIL', reason=matrix_error, native_report_not_waived=True)
        save(output / 'collision-matrix042-assessment.json', matrix_result)
        policy = accepted_debt_policy()
        cadence_policy = additive_cadence_policy()
        raw_report = json.loads((report / 'index.json').read_text(encoding='utf-8-sig'))
        save(output / 'debt-assessment.json', dict(state='PENDING_VALIDATION', policy=policy, cadence_policy=cadence_policy,
             accepted_failures=[row['fullTestPath'] for row in raw_report.get('tests', [])
                               if row.get('state') == 'Fail' and row.get('fullTestPath') in ACCEPTED_DEBT_IDS],
             raw_editor_exit=code, raw_report_unchanged=True, full_ci_qualified=False))
        result.update(validate_report_with_additive_debt(code, (logs / 'tests.stdout.log').read_text(encoding='utf-8-sig', errors='replace'), raw_report, policy, cadence_policy, output, expected_paths=expected))
        if matrix_error is not None:
            raise RuntimeError('Collision matrix042 artifact contract failed: ' + matrix_error)
        result['collision_matrix042'] = matrix_result
        result.update(engine_version=version, engine_root_inside_image=str(engine))
    except Exception as exc:
        result.update(state='FAIL', reason=str(exc), full_ci_qualified=False)
        if isinstance(exc, subprocess.TimeoutExpired):
            result.update(state='FAIL', native_timeout=True, full_ci_qualified=False,
                          export_completeness='UNKNOWN_TIMEOUT_DIRECT_CHILD_ONLY')
    result.update(editor_started=editor_started, editor_call_finished=editor_call_finished)
    try:
        if (output / 'debt-assessment.json').exists():
            assessment = json.loads((output / 'debt-assessment.json').read_text())
            assessment.update(state=result['state'], reason=result.get('reason'),
                              accepted_debt=result.get('accepted_debt', []),
                              accepted_cadence=result.get('accepted_cadence', []),
                              accepted_cadence_count=result.get('accepted_cadence_count', 0),
                              full_ci_qualified=result.get('full_ci_qualified', False))
            save(output / 'debt-assessment.json', assessment)
        save(output / 'container-result.json', result)
    except Exception as exc:
        result.update(state='FAIL', evidence_error=str(exc), full_ci_qualified=False)
    try:
        export_artifacts(output, Path('/output'), editor_started, editor_call_finished,
                         native_timeout=result.get('native_timeout', False))
    except Exception as exc:
        # Preserve the original Editor/build exit and raw files already copied.
        result.update(state='FAIL', export_error=str(exc), full_ci_qualified=False)
        save(Path('/output') / 'container-result.json', result)
        return 1
    # Also covers a full tmpfs which prevented saving the local result above.
    if result.get('evidence_error'):
        save(Path('/output') / 'container-result.json', result)
    return 0 if result['state'] in ('PASS', 'PASS_WITH_ACCEPTED_DEBT') else 1


def docker_args(image_id, source, output, owner, sha, cpus=2, cpuset_cpus=None):
    if type(cpus) is not int or cpus not in (2, 4, 8, 12):
        raise ValueError("Native CI CPU cap must be exactly 2, 4, 8 or 12")
    cpuset = canonical_cpuset(cpuset_cpus, cpus) if cpuset_cpus is not None else None
    if output.resolve()==source.resolve() or output.resolve() in source.resolve().parents:
        raise ValueError('Read-only source must not be reachable through the writable output mount')
    for path in (source, output):
        if any(char in str(path) for char in (',', '\n', '\r')):
            raise ValueError('Docker bind paths cannot contain commas or newlines')
    bootstrap = 'for p in python3 /opt/unreal/Engine/Binaries/ThirdParty/Python3/Linux/bin/python3 /opt/UnrealEngine/Engine/Binaries/ThirdParty/Python3/Linux/bin/python3; do if command -v "$p" >/dev/null 2>&1; then exec "$p" -I /input/HostProject/Scripts/RunDockerContracts.py --inside; fi; done; echo "Python3 missing from UE image" >&2; exit 127'
    # Source-built UE writes its early Trace.uba before parsing -Log. Keep
    # those image-internal writes in this container's private disposable layer;
    # the immutable image and the read-only exact source bind stay unchanged.
    command = ['docker', 'create', '--pull', 'never', '--name', 'iamspeed-' + owner, '--label', 'iamspeed.owner=' + owner, '--network', 'none', '--cpus', str(cpus), '--memory', '24g', '--memory-swap', '24g', '--shm-size', '2g', '--pids-limit', '512', '--ulimit', 'nice=30:30', '--tmpfs', '/tmp:rw,nosuid,nodev,exec,mode=1777,size=1024m', '--tmpfs', '/work:rw,nosuid,nodev,exec,mode=1777,size=14g', '--mount', 'type=bind,source=' + str(source) + ',target=/input,readonly', '--mount', 'type=bind,source=' + str(output) + ',target=/output', '--workdir', '/work', '--env', 'IAMSPEED_SOURCE_SHA=' + sha, '--env', 'IAMSPEED_IMAGE_ID=' + image_id, '--env', 'HOME=/work/home', '--entrypoint', '/bin/bash']
    if cpuset is not None:
        command.extend(['--cpuset-cpus', cpuset, '--env', 'IAMSPEED_CPUSET_CPUS=' + cpuset, '--env', 'IAMSPEED_CPUSET_QUOTA=' + str(cpus)])
    if hasattr(os, 'getuid'):
        if os.getuid() == 0:
            raise ValueError('Run the host launcher as an ordinary user')
        command.extend(['--user', str(os.getuid()) + ':' + str(os.getgid())])
    return command + [image_id, '-c', bootstrap]


def inspect_owned(container, owner, image_id):
    if not re.fullmatch('[0-9a-f]{64}', container):
        raise RuntimeError('Docker did not return a full container ID')
    observed = json.loads(subprocess.check_output(['docker', 'inspect', container], text=True))[0]
    if observed.get('Id') != container or observed.get('Config', {}).get('Labels', {}).get('iamspeed.owner') != owner or observed.get('Image') != image_id:
        raise RuntimeError('Container identity does not match this run')
    return observed


def nice_limit_evidence(observed):
    limits = observed.get('HostConfig', {}).get('Ulimits')
    if not isinstance(limits, list) or any(not isinstance(row, dict) for row in limits):
        raise RuntimeError('Container nice limit readback is missing or malformed')
    nice = [row for row in limits if row.get('Name') == 'nice']
    if len(nice) != 1 or any(type(nice[0].get(key)) is not int or nice[0][key] != 30 for key in ('Soft', 'Hard')):
        raise RuntimeError('Container nice limit must be exactly soft30/hard30')
    return dict(nice[0])


def host(args):
    image = args.image or os.environ.get('IAMSPEED_UE_IMAGE', '')
    if not image:
        raise RuntimeError('Set IAMSPEED_UE_IMAGE to the already available UE 5.8.2 Docker image')
    plugin = Path(__file__).resolve().parents[2]
    sha = subprocess.check_output(['git', '-C', str(plugin), 'rev-parse', 'HEAD'], text=True).strip()
    expected = args.expected_sha or os.environ.get('IAMSPEED_EXPECTED_SHA', sha)
    if not re.fullmatch('[0-9a-f]{40}', expected) or sha != expected:
        raise RuntimeError('Checkout does not match the requested exact IAmSpeed SHA')
    dirty = subprocess.check_output(['git', '-C', str(plugin), 'status', '--porcelain'], text=True)
    if dirty.strip():
        raise RuntimeError('CI requires the exact committed checkout; preserve dirty changes and commit the candidate first')
    info = json.loads(subprocess.check_output(['docker', 'image', 'inspect', image], text=True))[0]
    if info.get('Os') != 'linux' or not re.fullmatch('sha256:[0-9a-f]{64}', info.get('Id', '')):
        raise RuntimeError('A local immutable Linux UE image is required; no automatic pull')
    owner = uuid.uuid4().hex
    output = (Path(args.output) if args.output else plugin / 'Artifacts/PhysicalContracts' / owner).resolve()
    output.mkdir(parents=True, exist_ok=False)
    save(output / 'image.json', {k: info.get(k) for k in ('Id', 'RepoDigests', 'Os', 'Architecture')})
    source_tar = output / 'source.tar'
    with source_tar.open('wb') as stream:
        subprocess.run(['git', '-c', 'core.autocrlf=false', '-C', str(plugin), 'archive', '--format=tar', sha], stdout=stream, check=True)
    source = output.parent / (output.name + '-input-' + owner)
    extract_archive(source_tar, source)
    launch = {'source_sha': sha, 'source_archive_sha256': digest(source_tar), 'image_ref': image, 'image_id': info['Id'], 'owner': owner, 'state': 'STARTING', 'generated_root': str(output), 'read_only_input_root': str(source)}
    save(output / 'launch.json', launch)
    command = docker_args(info['Id'], source, output, owner, sha, args.cpus, args.cpuset_cpus)
    save(output / 'docker-create.json', command)
    container = subprocess.check_output(command, text=True).strip()
    launch['container_id'] = container; save(output / 'launch.json', launch)
    try:
        launch['nice_limit'] = nice_limit_evidence(inspect_owned(container, owner, info['Id']))
        save(output / 'launch.json', launch)
        subprocess.run(['docker', 'start', container], check=True)
        launch['state'] = 'RUNNING'
        save(output / 'launch.json', launch)
        status = subprocess.check_output(['docker', 'wait', container], text=True, timeout=7500).strip()
        with (output / 'container.stdout.log').open('wb') as stream:
            subprocess.run(['docker', 'logs', container], stdout=stream, stderr=subprocess.STDOUT, check=True)
        observed = inspect_owned(container, owner, info['Id'])
        save(output / 'container-state.json', observed['State'])
        if observed['State']['Running']:
            raise RuntimeError('Container identity or final closure does not match this run')
        if digest(source_tar) != launch['source_archive_sha256']:
            raise RuntimeError('Source archive changed during the run')
        result = json.loads((output / 'container-result.json').read_text())
        if status != '0' or observed['State']['ExitCode'] != 0 or result.get('state') not in ('PASS', 'PASS_WITH_ACCEPTED_DEBT') or result.get('source_sha') != sha or result.get('image_id') != info['Id']:
            raise RuntimeError('Container/report exit or exact candidate mismatch; preserve failures')
        launch.update(state=result['state'], native_container_closed=True, tests=result['tests'],
                      accepted_debt=result.get('accepted_debt', []),
                      accepted_cadence_count=result.get('accepted_cadence_count', 0),
                      full_ci_qualified=result.get('full_ci_qualified', False))
    except Exception as exc:
        launch.update(state='FAIL', reason=str(exc))
        try:
            observed = inspect_owned(container, owner, info['Id'])
            if observed['State']['Running']:
                # This exact task-owned, non-interactive CI container exceeded
                # its finite run. Verify identity immediately before stopping.
                subprocess.run(['docker', 'stop', '--time', '30', container], check=True, timeout=60)
                observed = inspect_owned(container, owner, info['Id'])
                launch['controlled_stop'] = True
            launch['native_container_closed'] = not observed['State']['Running']
            save(output / 'container-state.json', observed['State'])
            with (output / 'container.stdout.log').open('wb') as stream:
                subprocess.run(['docker', 'logs', container], stdout=stream, stderr=subprocess.STDOUT, timeout=30)
        except Exception as cleanup:
            launch['closure_unverified'] = str(cleanup)
        raise
    finally:
        save(output / 'launch.json', launch)
        if os.environ.get('GITHUB_OUTPUT'):
            with open(os.environ['GITHUB_OUTPUT'], 'a', encoding='utf-8') as stream:
                stream.write('output_dir=' + str(output) + '\n')
        print(json.dumps(launch))
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--inside', action='store_true')
    parser.add_argument('--image')
    parser.add_argument('--expected-sha')
    parser.add_argument('--output')
    parser.add_argument('--cpus', type=int, choices=(2, 4, 8, 12), default=2)
    parser.add_argument('--cpuset-cpus', help='Explicit Docker Linux CPU IDs, one distinct guest core per --cpus; validated inside before build')
    args = parser.parse_args()
    return inside() if args.inside else host(args)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, subprocess.SubprocessError, OSError) as exc:
        print('IAmSpeed Docker CI failed: ' + str(exc), file=sys.stderr)
        raise SystemExit(1)
