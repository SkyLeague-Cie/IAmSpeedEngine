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
        for required in ['--read-only', '--network', 'none', '--pull', 'never', '1000:1000']:
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


if __name__ == '__main__':
    unittest.main()
