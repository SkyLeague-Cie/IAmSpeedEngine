"""Portable IAmSpeed CI: one immutable UE image plus an exact plugin commit.

No host Unreal installation, private UBT manifest, automatic pull, or service
management. Docker source/input is read-only; generated files stay in one run.
"""
import argparse
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


def inside():
    output = Path('/output')
    result = {'state': 'FAIL', 'source_sha': os.environ['IAMSPEED_SOURCE_SHA'], 'image_id': os.environ['IAMSPEED_IMAGE_ID']}
    try:
        if hasattr(os, 'geteuid') and os.geteuid() == 0:
            raise RuntimeError('Unreal must run as a non-root user supplied by the image')
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
        save(output / 'expected-tests.json', expected)
        env = os.environ.copy()
        for name, directory in [('HOME', 'home'), ('TMPDIR', 'tmp'), ('XDG_CONFIG_HOME', 'config'), ('XDG_CACHE_HOME', 'cache'), ('DOTNET_CLI_HOME', 'dotnet')]:
            path = output / directory; path.mkdir(exist_ok=True); env[name] = str(path)
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
        command = [str(editor), str(project_file), '-NullRHI', '-unattended', '-nosound', '-nop4', '-nosplash', '-NoCompile', '-NoCompileEditor', '-NoLiveCoding', '-NoShaderCompile', '-notraceserver', '-NoTrace', '-ini:EditorSettings:[/Script/UnrealEd.AnalyticsPrivacySettings]:bSendUsageData=False', '-stdout', '-FullStdOutLogOutput', '-UserDir=' + str(output / 'user'), '-abslog=' + str(logs / 'editor.log'), '-ReportExportPath=' + str(report), '-ExecCmds=Automation RunTests ' + FILTER, '-TestExit=Automation Test Queue Empty']
        save(output / 'test-command.json', command)
        code = run_logged(command, logs / 'tests.stdout.log', 1800, project, env)
        result['editor_exit'] = code
        result.update(validate_report(code, (logs / 'tests.stdout.log').read_text(encoding='utf-8-sig', errors='replace'), json.loads((report / 'index.json').read_text(encoding='utf-8-sig')), expected_paths=expected))
        result.update(state='PASS', engine_version=version, engine_root_inside_image=str(engine))
    except Exception as exc:
        result['reason'] = str(exc)
    save(output / 'container-result.json', result)
    return 0 if result['state'] == 'PASS' else 1


def docker_args(image_id, source, output, owner, sha):
    if output.resolve()==source.resolve() or output.resolve() in source.resolve().parents:
        raise ValueError('Read-only source must not be reachable through the writable output mount')
    for path in (source, output):
        if any(char in str(path) for char in (',', '\n', '\r')):
            raise ValueError('Docker bind paths cannot contain commas or newlines')
    bootstrap = 'for p in python3 /opt/unreal/Engine/Binaries/ThirdParty/Python3/Linux/bin/python3 /opt/UnrealEngine/Engine/Binaries/ThirdParty/Python3/Linux/bin/python3; do if command -v "$p" >/dev/null 2>&1; then exec "$p" -I /input/HostProject/Scripts/RunDockerContracts.py --inside; fi; done; echo "Python3 missing from UE image" >&2; exit 127'
    # Source-built UE writes its early Trace.uba before parsing -Log. Keep
    # those image-internal writes in this container's private disposable layer;
    # the immutable image and the read-only exact source bind stay unchanged.
    command = ['docker', 'create', '--pull', 'never', '--name', 'iamspeed-' + owner, '--label', 'iamspeed.owner=' + owner, '--network', 'none', '--cpus', '2', '--memory', '12g', '--memory-swap', '12g', '--pids-limit', '512', '--tmpfs', '/tmp:rw,nosuid,nodev,size=1024m', '--mount', 'type=bind,source=' + str(source) + ',target=/input,readonly', '--mount', 'type=bind,source=' + str(output) + ',target=/output', '--workdir', '/output', '--env', 'IAMSPEED_SOURCE_SHA=' + sha, '--env', 'IAMSPEED_IMAGE_ID=' + image_id, '--env', 'HOME=/output/home', '--entrypoint', '/bin/bash']
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
        subprocess.run(['git', '-C', str(plugin), 'archive', '--format=tar', sha], stdout=stream, check=True)
    source = output.parent / (output.name + '-input-' + owner)
    extract_archive(source_tar, source)
    launch = {'source_sha': sha, 'source_archive_sha256': digest(source_tar), 'image_ref': image, 'image_id': info['Id'], 'owner': owner, 'state': 'STARTING', 'generated_root': str(output), 'read_only_input_root': str(source)}
    save(output / 'launch.json', launch)
    command = docker_args(info['Id'], source, output, owner, sha)
    save(output / 'docker-create.json', command)
    container = subprocess.check_output(command, text=True).strip()
    launch['container_id'] = container; save(output / 'launch.json', launch)
    try:
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
        if status != '0' or observed['State']['ExitCode'] != 0 or result.get('state') != 'PASS' or result.get('source_sha') != sha or result.get('image_id') != info['Id']:
            raise RuntimeError('Container/report exit or exact candidate mismatch; preserve failures')
        launch.update(state='PASS', native_container_closed=True, tests=result['tests'])
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
    args = parser.parse_args()
    return inside() if args.inside else host(args)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, subprocess.SubprocessError, OSError) as exc:
        print('IAmSpeed Docker CI failed: ' + str(exc), file=sys.stderr)
        raise SystemExit(1)
