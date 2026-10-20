"""Portable launcher regression tests; no Docker or Unreal process is started."""
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

    def test_boundary_ensure_remains_visible_as_accepted_debt(self):
        report, log = self.debt_report(); original = copy.deepcopy(report)
        log += '\nEnsure condition failed: CompleteInputFrame'
        result = ci.validate_report_with_debt(1, log, report, ci.accepted_debt_policy())
        self.assertEqual(result['state'], 'PASS_WITH_ACCEPTED_DEBT')
        self.assertEqual(report, original)
        self.assertFalse(result['full_ci_qualified'])

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

if __name__ == '__main__':
    unittest.main()
