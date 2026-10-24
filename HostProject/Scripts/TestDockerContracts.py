"""Portable launcher regression tests; no Docker or Unreal process is started."""
import ast
import copy
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch
import RunDockerContracts as ci


class Contracts(unittest.TestCase):
    def setUp(self):
        self.paths = sorted(ci.REQUIRED)
        self.log = 'Automation Test Queue Empty 5 tests performed.'
        self.report = dict(succeeded=5, succeededWithWarnings=0, failed=0,
                           notRun=0, inProcess=0,
                           tests=[dict(fullTestPath=p, state='Success', warnings=0, errors=0) for p in self.paths])

    def test_complete_physical_contracts(self):
        self.assertEqual(ci.validate_report(0, self.log, self.report)['tests'], 5)

    def test_incomplete_and_foreign_reports_fail(self):
        cases = []
        for key, value in [('state', 'InProcess'), ('warnings', 1), ('errors', 1),
                           ('fullTestPath', 'IAmSpeedImpostor.Collision.Test')]:
            report = copy.deepcopy(self.report); report['tests'][0][key] = value; cases.append(report)
        report = copy.deepcopy(self.report); report['tests'][0] = report['tests'][1]; cases.append(report)
        report = copy.deepcopy(self.report); report['tests'].pop(); cases.append(report)
        report = copy.deepcopy(self.report); report['failed'] = 1; cases.append(report)
        report = copy.deepcopy(self.report); report['tests'][0]['fullTestPath'] = 'IAmSpeed.Collision.Unrelated'; cases.append(report)
        for report in cases:
            with self.subTest(report=report), self.assertRaises(ValueError):
                ci.validate_report(0, self.log, report)

    def test_missing_completion_and_crash_fail(self):
        for log in ['', 'Automation Test Queue Empty 0 tests performed.', self.log * 2]:
            with self.subTest(log=log), self.assertRaises(ValueError):
                ci.validate_report(0, log, self.report)
        with self.assertRaises(ValueError):
            ci.validate_report(3, self.log, self.report)
        with self.assertRaises(ValueError):
            ci.validate_report(0, self.log, self.report, expected_paths=self.paths + ['IAmSpeed.Simulation.Missing'])

    def test_soft_quit_completion_never_overrides_exit_or_raw_report(self):
        log = self.log + '\n**** TEST COMPLETE. EXIT CODE: 0 ****'
        self.assertEqual(ci.validate_report(0, log, self.report)['tests'], 5)
        for code in (1, 3, 124, 255):
            with self.subTest(exit=code), self.assertRaises(ValueError):
                ci.validate_report(code, log, self.report)
        for key, value in [('state', 'Fail'), ('warnings', 1), ('errors', 1),
                           ('state', 'NotRun'), ('state', 'InProcess')]:
            raw = copy.deepcopy(self.report); raw['tests'][0][key] = value
            original = copy.deepcopy(raw)
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                ci.validate_report(0, log, raw)
            self.assertEqual(raw, original)

    def test_exact_source_manifest(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as tmp:
            p = Path(tmp) / 'Tests.cpp'
            p.write_text('\n'.join('IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest%d, "%s", EAutomationTestFlags::EditorContext)' % (i, name) for i, name in enumerate(self.paths)))
            self.assertEqual(ci.source_test_manifest(tmp), self.paths)
            p.write_text(p.read_text() + '\n#if WITH_DEV_AUTOMATION_TESTS && PLATFORM_WINDOWS\nIMPLEMENT_SIMPLE_AUTOMATION_TEST(FWindows, "IAmSpeed.WindowsOnly", 0)\n#endif\n')
            self.assertEqual(ci.source_test_manifest(tmp), self.paths)
            p.write_text(p.read_text() + '\nIMPLEMENT_COMPLEX_AUTOMATION_TEST(FDynamic, "IAmSpeed.Dynamic", 0)')
            with self.assertRaises(ValueError):
                ci.source_test_manifest(tmp)

    def test_archive_blocks_escape_and_links(self):
        for name, kind in [('../outside', tarfile.REGTYPE), ('/outside', tarfile.REGTYPE),
                           ('link', tarfile.SYMTYPE), ('hard', tarfile.LNKTYPE)]:
            with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as tmp:
                archive = Path(tmp) / 'source.tar'
                with tarfile.open(archive, 'w') as stream:
                    member = tarfile.TarInfo(name); member.type = kind; member.linkname = '../outside'
                    stream.addfile(member)
                with self.assertRaises(ValueError):
                    ci.extract_archive(archive, Path(tmp) / 'source')

    def test_archive_extracts_exact_bytes(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as tmp:
            archive = Path(tmp) / 'source.tar'
            with tarfile.open(archive, 'w') as stream:
                member = tarfile.TarInfo('Source/Test.cpp'); member.size = 4
                stream.addfile(member, io.BytesIO(b'test'))
            ci.extract_archive(archive, Path(tmp) / 'source')
            self.assertEqual((Path(tmp) / 'source/Source/Test.cpp').read_bytes(), b'test')

    def test_no_host_engine_network_or_pull(self):
        with patch.object(ci.os, 'getuid', return_value=1000, create=True), patch.object(ci.os, 'getgid', return_value=1000, create=True):
            args = ci.docker_args('sha256:' + 'a' * 64, Path('/source'), Path('/output'), 'owner', 'b' * 40)
        for required in ['--network', 'none', '--pull', 'never', '1000:1000']:
            self.assertIn(required, args)
        mounts = [args[i + 1] for i, value in enumerate(args[:-1]) if value == '--mount']
        self.assertEqual(len(mounts), 2)
        self.assertIn('target=/input,readonly', mounts[0])
        self.assertIn('target=/output', mounts[1])
        self.assertFalse(any('docker.sock' in arg for arg in args))
        with self.assertRaises(ValueError):
            ci.docker_args('image', Path('/output/source'), Path('/output'), 'owner', 'sha')

    def test_identity_mismatch_prevents_stop(self):
        observed = dict(Id='a' * 64, Image='sha256:' + 'b' * 64,
                        Config=dict(Labels={'iamspeed.owner': 'other'}))
        with patch.object(ci.subprocess, 'check_output', return_value=json.dumps([observed])):
            with self.assertRaises(RuntimeError):
                ci.inspect_owned('a' * 64, 'owner', observed['Image'])

    def debt_report(self):
        report = copy.deepcopy(self.report)
        report['tests'] += [dict(fullTestPath=name, state='Fail', errors=4, warnings=0)
                            for name in sorted(ci.ACCEPTED_DEBT_IDS)]
        report['failed'] = 2
        return report, 'Automation Test Queue Empty 7 tests performed.'

    def test_exact_accepted_debts_are_visible_and_do_not_mutate_raw_report(self):
        report, log = self.debt_report(); original = copy.deepcopy(report)
        decision = ci.validate_report_with_debt(1, log, report, ci.accepted_debt_policy(),
                                                expected_paths=self.paths + sorted(ci.ACCEPTED_DEBT_IDS))
        self.assertEqual(decision['state'], 'PASS_WITH_ACCEPTED_DEBT')
        self.assertFalse(decision['full_ci_qualified'])
        self.assertEqual({d['test'] for d in decision['accepted_debt']}, ci.ACCEPTED_DEBT_IDS)
        self.assertEqual(report, original)
        with self.assertRaises(ValueError):
            ci.validate_report(1, log, report)

    def test_debt_does_not_accept_other_failures_warnings_or_missing_tests(self):
        report, log = self.debt_report(); policy = ci.accepted_debt_policy()
        for key, value in [('state', 'Fail'), ('warnings', 1), ('errors', 1)]:
            changed = copy.deepcopy(report); changed['tests'][0][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                ci.validate_report_with_debt(1, log, changed, policy)
        for key in ('notRun', 'inProcess', 'failed'):
            changed = copy.deepcopy(report); changed[key] += 1
            with self.subTest(key=key), self.assertRaises(ValueError):
                ci.validate_report_with_debt(1, log, changed, policy)
        for state in ('NotRun', 'InProcess'):
            changed = copy.deepcopy(report); changed['tests'][-1]['state'] = state
            with self.assertRaises(ValueError):
                ci.validate_report_with_debt(1, log, changed, policy)
        with self.assertRaises(ValueError):
            ci.validate_report_with_debt(1, log, report, policy, expected_paths=self.paths)

    def test_debt_never_accepts_crashes_missing_completion_or_wildcards(self):
        report, log = self.debt_report(); policy = ci.accepted_debt_policy()
        for code in (-11, 3, 124, 134, 255):
            with self.subTest(code=code), self.assertRaises(ValueError):
                ci.validate_report_with_debt(code, log, report, policy)
        with self.assertRaises(ValueError):
            ci.validate_report_with_debt(1, '', report, policy)
        policy['tests']['IAmSpeed.*'] = dict(reason='forbidden')
        with self.assertRaises(ValueError):
            ci.validate_report_with_debt(1, log, report, policy)

    def test_completed_debt_report_with_crash_diagnostics_is_rejected(self):
        report, log = self.debt_report(); policy = ci.accepted_debt_policy()
        for marker in ('Fatal error!', 'Unhandled Exception: EXCEPTION_ACCESS_VIOLATION',
                       'Unhandled signal 11', 'Assertion failed: NativeInvariant',
                       'Segmentation fault (core dumped)'):
            for code in (0, 1):
                with self.subTest(marker=marker, code=code), self.assertRaises(ValueError):
                    ci.validate_report_with_debt(code, log + '\n' + marker, report, policy)

    def test_boundary_ensure_rejects_accepted_debt_without_mutating_report(self):
        report, log = self.debt_report(); original = copy.deepcopy(report)
        log += '\nEnsure condition failed: CompleteInputFrame'
        with self.assertRaisesRegex(ValueError, 'Native ensure diagnostic'):
            ci.validate_report_with_debt(1, log, report, ci.accepted_debt_policy())
        self.assertEqual(report, original)

    def test_startup_ensure_blocks_clean_queue_and_debt_at_both_exit_codes(self):
        for marker in ('Ensure condition failed: !PyStatus_Exception(Status)',
                       '=== Handled ensure: ===', '=== Handled ensure ==='):
            for code in (0, 1):
                for debt in (False, True):
                    report, log = self.debt_report() if debt else (copy.deepcopy(self.report), self.log)
                    original = copy.deepcopy(report)
                    raw_log = marker + '\nPython startup failed before automation\n' + log
                    with self.subTest(marker=marker, code=code, debt=debt), self.assertRaises(ValueError):
                        ci.validate_report_with_debt(code, raw_log, report, ci.accepted_debt_policy())
                    self.assertEqual(report, original)
                    self.assertTrue(raw_log.startswith(marker))

    def test_clean_report_with_fatal_tail_is_rejected(self):
        with self.assertRaises(ValueError):
            ci.validate_report(0, self.log + '\nFatal error!', self.report)

    def test_fixed_debt_is_clean_pass_and_no_debt_exit_one_is_rejected(self):
        policy = ci.accepted_debt_policy()
        result = ci.validate_report_with_debt(0, self.log, self.report, policy)
        self.assertEqual(result['state'], 'PASS')
        self.assertEqual(result['accepted_debt'], [])
        self.assertTrue(result['full_ci_qualified'])
        with self.assertRaises(ValueError):
            ci.validate_report_with_debt(1, self.log, self.report, policy)



class RawDebtWarningRegression108(unittest.TestCase):
    def test_accepted_fail_warning_is_rejected_without_mutation(self):
        import copy
        import importlib.util
        from pathlib import Path
        source = Path(__file__).with_name('RunDockerContracts.py')
        spec = importlib.util.spec_from_file_location('strict_debt_108', source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        paths = sorted(module.REQUIRED | module.ACCEPTED_DEBT_IDS)
        rows = [dict(fullTestPath=name, state='Fail' if name in module.ACCEPTED_DEBT_IDS else 'Success',
                     errors=int(name in module.ACCEPTED_DEBT_IDS), warnings=0, entries=[]) for name in paths]
        report = dict(tests=rows, succeeded=len(paths)-2, failed=2, notRun=0, inProcess=0, succeededWithWarnings=0)
        log = 'Automation Test Queue Empty '+str(len(paths))+' tests performed.'
        module.validate_report_with_debt(0, log, report, module.accepted_debt_policy(), expected_paths=paths)
        next(row for row in rows if row['state']=='Fail')['warnings'] = 1
        original = copy.deepcopy(report)
        with self.assertRaises(ValueError):
            module.validate_report_with_debt(0, log, report, module.accepted_debt_policy(), expected_paths=paths)
        self.assertEqual(report, original)


class EditorLoopbackContract1573(unittest.TestCase):
    def test_real_editor_argv_satisfies_loopback_reference_contract(self):
        # Evaluate the actual argument-list expression used by inside(), not a
        # second hand-maintained command. No inside(), Git, Docker or UE runs.
        tree = ast.parse(Path(ci.__file__).read_text(encoding='utf-8-sig'))
        inside = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'inside')
        commands = [node.value for node in ast.walk(inside)
                    if isinstance(node, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == 'command' for target in node.targets)
                    and isinstance(node.value, ast.List) and node.value.elts
                    and isinstance(node.value.elts[0], ast.Call)
                    and isinstance(node.value.elts[0].func, ast.Name) and node.value.elts[0].func.id == 'str'
                    and len(node.value.elts[0].args) == 1
                    and isinstance(node.value.elts[0].args[0], ast.Name) and node.value.elts[0].args[0].id == 'editor']
        self.assertEqual(len(commands), 1, 'One unambiguous native Editor argv must remain inspectable')
        args = eval(compile(ast.Expression(body=commands[0]), str(ci.__file__), 'eval'),
                    {'__builtins__': {'str': str}, 'FILTER': ci.FILTER,
                     'editor': Path('/fixture/UnrealEditor-Cmd'), 'project_file': Path('/fixture/Host.uproject'),
                     'output': Path('/fixture/output'), 'logs': Path('/fixture/logs'), 'report': Path('/fixture/report')})
        binds = [arg.partition('=')[2] for arg in args
                 if arg.lstrip('-').partition('=')[0].upper() == 'MULTIHOME']
        self.assertEqual(binds, ['127.0.0.1'], 'Mapped reference gate requires exactly one IPv4 loopback binding')
        exec_commands = [arg.partition('=')[2] for arg in args
                         if arg.lstrip('-').partition('=')[0].lower() == 'execcmds']
        self.assertEqual(exec_commands, ['Automation RunTests ' + ci.FILTER + '; SoftQuit'])
        # Keep RunTests and SoftQuit in one Automation queue. An independent
        # console Quit could exit before test completion.
        self.assertEqual(exec_commands[0].removeprefix('Automation ').split(';'),
                         ['RunTests ' + ci.FILTER, ' SoftQuit'])
        self.assertFalse(any(arg.lstrip('-').partition('=')[0].lower() == 'testexit' for arg in args))
        self.assertFalse(any(arg.lower() in ('quit', 'softquit', '-quit', '-softquit') for arg in args))
        self.assertIn('-NullRHI', args)
        self.assertIn('-nosound', args)



class DockerSharedMemoryContract1581(unittest.TestCase):
    def test_uba_shared_memory_is_bounded_inside_existing_isolation(self):
        # docker_args is the real launcher builder; no Docker process starts.
        with patch.object(ci.os, 'getuid', return_value=1000, create=True), patch.object(ci.os, 'getgid', return_value=1000, create=True):
            args = ci.docker_args('sha256:' + 'a' * 64, Path('/source'), Path('/output'), 'owner', 'b' * 40)
        def value(flag):
            self.assertEqual(args.count(flag), 1, flag + ' must be unambiguous')
            return args[args.index(flag) + 1]
        self.assertEqual(value('--shm-size'), '2g')
        self.assertEqual(value('--memory'), '24g')
        self.assertEqual(value('--memory-swap'), '24g')
        self.assertEqual(value('--cpus'), '2')
        self.assertEqual(value('--pids-limit'), '512')
        self.assertLessEqual(int(value('--shm-size')[:-1]), int(value('--memory')[:-1]))
        self.assertEqual(value('--network'), 'none')
        self.assertEqual(value('--pull'), 'never')
        for forbidden in ('--ipc', '--privileged', '--cap-add', '--security-opt'):
            self.assertNotIn(forbidden, args, 'Do not relax IPC, ptrace or seccomp for UBA')
        self.assertFalse(any('SYS_PTRACE' in arg or 'seccomp=unconfined' in arg for arg in args))





class ExplicitNativeCpuEnvelope1617(unittest.TestCase):
    def test_requested_cpu_cap_preserves_all_other_resource_bounds(self):
        with patch.object(ci.os, 'getuid', return_value=1000, create=True), patch.object(ci.os, 'getgid', return_value=1000, create=True):
            baseline = ci.docker_args('sha256:' + 'a' * 64, Path('/source'), Path('/output'), 'owner', 'b' * 40)
            for cap in (4, 8, 12):
                candidate = ci.docker_args('sha256:' + 'a' * 64, Path('/source'), Path('/output'), 'owner', 'b' * 40, cap)
                expected = baseline.copy(); expected[expected.index('--cpus') + 1] = str(cap)
                self.assertEqual(candidate, expected)
            for invalid in (0, 1, -1, 3, 13, 2.0, True, '12'):
                with self.assertRaises(ValueError):
                    ci.docker_args('sha256:' + 'a' * 64, Path('/source'), Path('/output'), 'owner', 'b' * 40, invalid)



class ExplicitLinuxCpuTopologyProposal(unittest.TestCase):
    def test_cardinality_duplicates_and_unbounded_cpu_ranges_fail_closed(self):
        self.assertEqual(ci.canonical_cpuset('0,2,4,6', 4), '0,2,4,6')
        for spec in ('0-2', '0,2,4,6,8', '0,0,2,4', '3-1', '-1', '0, 2', '0-999999999'):
            with self.assertRaises(ValueError):
                ci.canonical_cpuset(spec, 4)

    def topology(self, text_by_path, actual):
        def read(path):
            return text_by_path[str(path).replace('\\', '/')]
        return patch.object(Path, 'read_text', read), patch.object(Path, 'is_file', lambda path: False), patch.object(ci.os, 'sched_getaffinity', return_value=set(actual), create=True)

    def test_effective_affinity_mismatch_is_not_a_performance_result(self):
        with patch.object(ci.os, 'sched_getaffinity', return_value={0, 1}, create=True):
            with self.assertRaises(ValueError):
                ci.cpu_affinity_evidence('0,2', 2)

    def test_smt_sibling_selection_refused_and_distinct_cores_recorded(self):
        files = {}
        for cpu, core, siblings in [(0, 0, '0-1'), (1, 0, '0-1'), (2, 1, '2-3')]:
            prefix = '/sys/devices/system/cpu/cpu'+str(cpu)+'/topology/'
            files.update({prefix+'physical_package_id': '0', prefix+'core_id': str(core), prefix+'thread_siblings_list': siblings})
        a,b,c = self.topology(files, [0, 1])
        with a,b,c:
            with self.assertRaises(ValueError):
                ci.cpu_affinity_evidence('0,1', 2)
        a,b,c = self.topology(files, [0, 2])
        with a,b,c:
            evidence = ci.cpu_affinity_evidence('0,2', 2)
        self.assertEqual(evidence['effective_affinity'], [0,2])
        self.assertFalse(evidence['exclusivity_claimed'])
        self.assertEqual(evidence['Windows_host_physical_mapping'], 'UNKNOWN')

    def test_single_bounded_executable_sticky_tmpfs_and_explicit_mask(self):
        with patch.object(ci.os, 'getuid', return_value=1000, create=True), patch.object(ci.os, 'getgid', return_value=1000, create=True):
            args = ci.docker_args('sha256:'+'a'*64, Path('/source'), Path('/output'), 'owner', 'b'*40, 2, '0,2')
        self.assertEqual(args.count('--tmpfs'), 2)
        self.assertIn('/work:rw,nosuid,nodev,exec,mode=1777,size=14g', args)
        self.assertEqual(args[args.index('--tmpfs')+1], '/tmp:rw,nosuid,nodev,exec,mode=1777,size=1024m')
        self.assertEqual(args[args.index('--cpus')+1], '2')
        self.assertEqual(args[args.index('--cpuset-cpus')+1], '0,2')
        self.assertEqual(args[args.index('--user')+1], '1000:1000')


class UnrealNiceLimitContract(unittest.TestCase):
    def test_real_launcher_sets_one_bounded_nice_limit_without_extra_privileges(self):
        with patch.object(ci.os, 'getuid', return_value=1000, create=True), patch.object(ci.os, 'getgid', return_value=1000, create=True):
            args = ci.docker_args('sha256:' + 'a' * 64, Path('/source'), Path('/output'), 'owner', 'b' * 40, 12, '0,2,4,6,8,10,12,14,16,18,20,22')
        self.assertEqual(args.count('--ulimit'), 1)
        self.assertEqual(args[args.index('--ulimit') + 1], 'nice=30:30')
        self.assertEqual(args[args.index('--user') + 1], '1000:1000')
        for forbidden in ('--cap-add', '--privileged', '--pid'):
            self.assertNotIn(forbidden, args)

    def test_nice_limit_readback_requires_exact_unique_integer_soft_and_hard(self):
        good = {'HostConfig': {'Ulimits': [{'Name': 'nice', 'Soft': 30, 'Hard': 30}]}}
        original = copy.deepcopy(good)
        self.assertEqual(ci.nice_limit_evidence(good), good['HostConfig']['Ulimits'][0])
        self.assertEqual(good, original)
        bad_limits = [None, [], [{'Name': 'other', 'Soft': 30, 'Hard': 30}],
                      [{'Name': 'nice', 'Soft': 29, 'Hard': 30}],
                      [{'Name': 'nice', 'Soft': 30, 'Hard': 31}],
                      [{'Name': 'nice', 'Soft': '30', 'Hard': 30}],
                      [{'Name': 'nice', 'Soft': 30.0, 'Hard': 30}],
                      [{'Name': 'nice', 'Soft': True, 'Hard': 30}],
                      [{'Name': 'nice', 'Soft': 30}],
                      good['HostConfig']['Ulimits'] * 2, ['not-a-limit']]
        for limits in bad_limits:
            with self.subTest(limits=limits), self.assertRaises(RuntimeError):
                ci.nice_limit_evidence({'HostConfig': {'Ulimits': limits}})
        with self.assertRaises(RuntimeError):
            ci.nice_limit_evidence({})

class LinuxWorkExportTransport(unittest.TestCase):
    def run_fixture(self, root, mode):
        work, export, engine = (root/name for name in ('work', 'export', 'engine'))
        work.mkdir(); export.mkdir()
        (export/'launch.json').write_bytes(b'host metadata must remain exact')
        version = engine/'Engine/Build/Build.version'; version.parent.mkdir(parents=True)
        version.write_text(json.dumps(dict(MajorVersion=5, MinorVersion=8, PatchVersion=2,
                                          CompatibleChangelist=55116800)))
        build=engine/'Engine/Build/BatchFiles/Linux/Build.sh'; build.parent.mkdir(parents=True); build.touch()
        editor=engine/'Engine/Binaries/Linux/UnrealEditor-Cmd'; editor.parent.mkdir(parents=True); editor.touch()
        calls=[]; paths=sorted(ci.REQUIRED)
        def mapped(value):
            return {'/work':work, '/output':export}.get(str(value), Path(value))
        def copy_source(source, target, **kwargs):
            Path(target).mkdir(parents=True)
            if str(source)=='/input/HostProject':
                (Path(target)/'IAmSpeedHostProject.uproject').write_bytes(b'{}')
        def native(command, log, timeout, cwd, env):
            # Test doubles only. Real exporter and strict report validator run.
            self.assertEqual(list(export.iterdir()),[export/'launch.json'])
            self.assertTrue(str(log).startswith(str(work)))
            self.assertTrue(env['HOME'].startswith(str(work)))
            calls.append(command)
            if len(calls)==1:
                Path(log).write_bytes(b'original build stdout')
                return 43 if mode=='build_failure' else 0
            report=dict(succeeded=len(paths),succeededWithWarnings=0,failed=0,notRun=0,inProcess=0,
                        tests=[dict(fullTestPath=p,state='Success',warnings=0,errors=0)for p in paths])
            if mode=='warning':report['tests'][0]['warnings']=1
            (work/'Automation/index.json').write_text(json.dumps(report))
            Path(log).write_text('Automation Test Queue Empty '+str(len(paths))+' tests performed.')
            for relative in ('HostProject/Saved/IAmSpeed/raw-failed.pack',
                             'user/Saved/Logs/native.log', 'HostProject/Binaries/Linux/module.so'):
                path=work/relative;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(b'RAW\x00WITNESS')
            if mode=='timeout':raise ci.subprocess.TimeoutExpired(command,timeout)
            return 1 if mode=='editor_failure' else 0
        with patch.object(ci,'Path',side_effect=mapped), patch.object(ci.os,'geteuid',return_value=10001,create=True), \
             patch.dict(ci.os.environ,{'IAMSPEED_SOURCE_SHA':'b'*40,'IAMSPEED_IMAGE_ID':'sha256:'+'a'*64},clear=True), \
             patch.object(ci,'work_tmpfs_evidence',return_value={'fixture':True}), \
             patch.object(ci,'engine_root',return_value=engine), patch.object(ci.shutil,'copytree',side_effect=copy_source), \
             patch.object(ci,'source_test_manifest',return_value=paths),patch.object(ci,'run_logged',side_effect=native):
            code=ci.inside()
        result=json.loads((export/'container-result.json').read_text())
        self.assertEqual((export/'launch.json').read_bytes(),b'host metadata must remain exact')
        self.assertEqual(json.loads((export/'export-manifest.json').read_text())['destination_verified'],True)
        return code,result,work,export,calls

    def test_success_failure_warning_timeout_preserve_raw_after_editor(self):
        for mode in ('success','warning','editor_failure','timeout'):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory(dir=Path(__file__).parent)as tmp:
                code,result,work,export,calls=self.run_fixture(Path(tmp),mode)
                self.assertEqual(code,0 if mode=='success'else 1)
                self.assertEqual(result['state'],'PASS'if mode=='success'else'FAIL')
                self.assertTrue(result['editor_call_finished']);self.assertEqual(len(calls),2)
                for relative in ('Automation/index.json','Logs/tests.stdout.log','HostProject/Saved/IAmSpeed/raw-failed.pack','HostProject/Binaries/Linux/module.so'):
                    self.assertEqual((work/relative).read_bytes(),(export/relative).read_bytes())
                if mode=='timeout':self.assertIn('timed out',result['reason'])
                else:self.assertEqual(result['editor_exit'],1 if mode=='editor_failure'else 0)

    def test_build_failure_preserves_original_code_and_logs_without_editor(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent)as tmp:
            code,result,work,export,calls=self.run_fixture(Path(tmp),'build_failure')
            self.assertEqual((code,result['build_exit'],len(calls)),(1,43,1))
            self.assertFalse(result['editor_started']);self.assertFalse(result['editor_call_finished'])
            self.assertEqual((export/'Logs/build.stdout.log').read_bytes(),b'original build stdout')

    def test_active_editor_blocks_export_before_any_file_access(self):
        with patch.object(Path,'glob',side_effect=AssertionError('must not enumerate active files')):
            with self.assertRaisesRegex(RuntimeError,'has not finished'):
                ci.export_artifacts(Path('unused-work'),Path('unused-export'),True,False)

    def test_export_refuses_collision_and_retains_existing_metadata(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent)as tmp:
            work,export=(Path(tmp)/name for name in ('work','export'));work.mkdir();export.mkdir()
            (work/'raw.json').write_bytes(b'new');(export/'raw.json').write_bytes(b'original')
            with self.assertRaisesRegex(RuntimeError,'overwrite retained'):
                ci.export_artifacts(work,export,False,False)
            self.assertEqual((export/'raw.json').read_bytes(),b'original')

    def test_corrupted_export_fails_independent_byte_verification(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent)as tmp:
            work,export=(Path(tmp)/name for name in ('work','export'));work.mkdir();export.mkdir()
            (work/'raw.json').write_bytes(b'original native report')
            with patch.object(ci.shutil,'copyfile',side_effect=lambda source,target:Path(target).write_bytes(b'corrupted')):
                with self.assertRaisesRegex(RuntimeError,'byte identity differs'):
                    ci.export_artifacts(work,export,True,True)
            self.assertFalse((export/'export-manifest.json').exists())


if __name__ == '__main__':
    unittest.main()
