"""Pure host rejection tests: no Docker, Editor, or real debt activation."""
import copy
import datetime as dt
import json
import os
from pathlib import Path
import tempfile
import unittest

import CadenceDebt as debt
import RunDockerContracts as ci


class AdditiveCadence(unittest.TestCase):
    def setUp(self):
        test_dir = str(Path(__file__).resolve().parent)
        if os.name == 'nt' and not test_dir.startswith('\\\\?\\'):
            test_dir = '\\\\?\\' + test_dir
        self.tmp = tempfile.TemporaryDirectory(dir=test_dir)
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.policy = json.loads(Path(__file__).with_name('accepted-cadence-debt.json').read_text())
        # Synthetic decision only, not saved or used for a real native report.
        self.policy['activation'] = dict(enabled=True, trigger='CI_GREEN', at_utc='2026-10-24T00:00:00Z', decision_sha256='a'*64)
        self.old = ci.accepted_debt_policy()
        self.native = '/work/user/Saved/IAmSpeed/QualificationEightCars1534/'+'a'*32+'/Fast0-W8'
        self.header = 'Fast=0 Requested=8 Host=2 Hz=300 Driver=RealTimeSimulation Finished=1 Failed=0 FailureLine=0 Steps=90 Scope=IsolatedNativeFixtureFullStadiumUNKNOWN'
        self.rows(self.native, self.header)
        self.message = 'SpeedPhysicsLog: [RealTimeSimulationCadence] Physics Thread is too slow: Frame=50 ActualStartIntervalMs=3.908272 TargetStartIntervalMs=2.525274 BaseStartIntervalMs=3.333333 PhaseErrorMs=1.382998 RecoveryDebtMs=2.165782'
        self.report = dict(succeeded=0, succeededWithWarnings=1, failed=0, notRun=0, inProcess=0,
                           tests=[dict(fullTestPath=debt.EIGHT, state='Success', warnings=1, errors=0,
                                       entries=[self.event('Warning', self.message), self.evidence(self.native)])])
        self.log = 'Automation Test Queue Empty 1 tests performed.'

    def rows(self, native, header, count=90):
        path = self.root.joinpath(*Path(native).parts[2:], 'rows.tsv')
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(header+'\nN\tFrame\tPublication\tComplete\n'+''.join(f'{n}\t{n}\t{n+1}\t1\n' for n in range(count)))
        return path

    @staticmethod
    def event(kind, message):
        return {'event': {'type': kind, 'message': message, 'context': 'log'}}

    def evidence(self, path, saved=1):
        return self.event('Info', f'[NativeAnalyticContacts1368Evidence] Path={path} Saved={saved}')

    def assess(self, report=None, log=None, policy=None, code=0):
        return ci.validate_report_with_additive_debt(code, self.log if log is None else log,
                    self.report if report is None else report, self.old,
                    self.policy if policy is None else policy, self.root,
                    required=set(), expected_paths=[row['fullTestPath'] for row in (report or self.report)['tests']])

    def test_disabled_policy_stays_strict_fail(self):
        inactive = ci.additive_cadence_policy()
        # Exercise the disabled behavior even after Root's later versioned
        # activation. The prepared candidate's false default is pinned in its
        # source receipt, not an assertion which would break that future CI.
        inactive['activation'] = dict(enabled=False, trigger=None, at_utc=None, decision_sha256=None)
        with self.assertRaises(ValueError): self.assess(policy=inactive)

    def test_accepts_only_separate_assessment_preserves_raw_and_log(self):
        before = copy.deepcopy(self.report); original_log = self.log
        result = self.assess()
        self.assertEqual(result['state'], 'PASS_WITH_ACCEPTED_DEBT')
        self.assertFalse(result['full_ci_qualified'])
        self.assertEqual(result['accepted_cadence_count'], 1)
        self.assertEqual(result['accepted_cadence'][0]['raw_event']['message'], self.message)
        self.assertEqual(self.report, before); self.assertEqual(self.log, original_log)

    def test_all_matches_counted_and_metrics_not_capped(self):
        report = copy.deepcopy(self.report)
        second = self.message.replace('Frame=50 ', 'Frame=51 ').replace('3.908272', '30.000000')
        report['tests'][0]['entries'].insert(1, self.event('Warning', second))
        report['tests'][0]['warnings'] = 2
        result = self.assess(report)
        self.assertEqual(result['accepted_cadence_count'], 2)
        self.assertEqual([x['frame'] for x in result['accepted_cadence']], [50,51])
        self.assertEqual(result['accepted_cadence'][1]['metrics']['ActualStartIntervalMs'], '30.000000')

    def test_grounded_other_real_scene_and_w12(self):
        native = '/work/user/Saved/IAmSpeed/Qualification1368/'+'b'*32+'/Fast0-W12'
        self.rows(native, self.header.replace('Requested=8 ', 'Requested=12 '))
        report = copy.deepcopy(self.report)
        report['tests'][0].update(fullTestPath=debt.GROUND)
        report['tests'][0]['entries'][-1] = self.evidence(native)
        self.assertEqual(self.assess(report)['accepted_cadence'][0]['cohort']['requested_lanes'],12)

    def test_other_warning_and_too_fast_rejected(self):
        for message in (self.message.replace('too slow','too fast'), 'Other warning', self.message+' extra', self.message.replace('Frame=50','Frame=91')):
            with self.subTest(message=message):
                report=copy.deepcopy(self.report); report['tests'][0]['entries'][0]=self.event('Warning',message)
                with self.assertRaises(ValueError):self.assess(report)

    def test_new_errors_never_become_cadence_debt(self):
        for declared in (0,1):
            report=copy.deepcopy(self.report);row=report['tests'][0];row['errors']=declared
            row['entries'].insert(0,self.event('Error','new failure'))
            with self.assertRaises(ValueError):self.assess(report)
        report=copy.deepcopy(self.report);report['tests'][0]['errors']=1
        with self.assertRaises(ValueError):self.assess(report)

    def test_other_test_identity_rejected(self):
        report=copy.deepcopy(self.report);report['tests'][0]['fullTestPath']='IAmSpeed.Other.Test'
        with self.assertRaises(ValueError):self.assess(report)

    def test_ambiguous_test_and_cohort_identity_rejected(self):
        report=copy.deepcopy(self.report);report['tests'].append(copy.deepcopy(report['tests'][0]))
        with self.assertRaises(ValueError):self.assess(report)
        report=copy.deepcopy(self.report);report['tests'][0]['entries'].append(self.evidence(self.native))
        with self.assertRaises(ValueError):self.assess(report)

    def test_missing_saved0_foreign_traversal_and_fast_evidence_refused(self):
        for native,saved in ((self.native,0),(self.native.replace('Fast0','Fast1'),1),
                              (self.native.replace('W8','W3'),1),(self.native.replace('/work/','/other/'),1),
                              (self.native.replace('/Fast0','/../Fast0'),1)):
            report=copy.deepcopy(self.report);report['tests'][0]['entries'][-1]=self.evidence(native,saved)
            with self.subTest(native=native,saved=saved),self.assertRaises(ValueError):self.assess(report)
        report=copy.deepcopy(self.report);report['tests'][0]['entries'].pop()
        with self.assertRaises(ValueError):self.assess(report)

    def test_incomplete_or_fast_rows_refused(self):
        for header,count in ((self.header,89),(self.header.replace('Finished=1','Finished=0'),90),
                             (self.header.replace('Failed=0','Failed=1'),90),
                             (self.header.replace('Fast=0','Fast=1'),90)):
            self.rows(self.native,header,count)
            with self.subTest(header=header,count=count),self.assertRaises(ValueError):self.assess()

    def test_incomplete_counts_and_exit_stay_blocking(self):
        for key,value in [('inProcess',1),('notRun',1),('failed',1),('succeededWithWarnings',0),('succeeded',1)]:
            report=copy.deepcopy(self.report);report[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):self.assess(report)
        for code in (1,3,137):
            with self.subTest(code=code),self.assertRaises(ValueError):self.assess(code=code)
        report=copy.deepcopy(self.report);report['tests'][0]['state']='InProcess'
        with self.assertRaises(ValueError):self.assess(report)

    def test_unreported_duplicate_and_invalid_metric_warnings_rejected(self):
        report=copy.deepcopy(self.report);report['tests'][0]['warnings']=0
        with self.assertRaises(ValueError):self.assess(report)
        report=copy.deepcopy(self.report);report['tests'][0]['entries'].insert(1,copy.deepcopy(report['tests'][0]['entries'][0]));report['tests'][0]['warnings']=2
        with self.assertRaises(ValueError):self.assess(report)
        for value in ('nan','inf','-1','x'):
            report=copy.deepcopy(self.report);report['tests'][0]['entries'][0]=self.event('Warning',self.message.replace('3.908272',value))
            with self.subTest(value=value),self.assertRaises(ValueError):self.assess(report)

    def test_existing_v1_fail_debts_remain_additive_and_unchanged(self):
        before=copy.deepcopy(self.old);report=copy.deepcopy(self.report)
        for test in sorted(ci.ACCEPTED_DEBT_IDS):
            report['tests'].append(dict(fullTestPath=test,state='Fail',errors=1,warnings=0,entries=[self.event('Error','historical authorized failure')]))
        report['failed']=2
        result=self.assess(report,log='Automation Test Queue Empty 3 tests performed.',code=1)
        self.assertEqual(len(result['accepted_debt']),3);self.assertEqual(result['accepted_cadence_count'],1)
        self.assertEqual(self.old,before);self.assertEqual(report['failed'],2)
        report['tests'][1]['entries'].append(self.event('Warning','new warning'));report['tests'][1]['warnings']=1
        with self.assertRaises(ValueError):self.assess(report,log='Automation Test Queue Empty 3 tests performed.',code=1)

    def test_raw_fatal_and_ensure_still_block(self):
        for extra in ('Fatal error: crash','Ensure condition failed: broken','=== Handled ensure: ==='):
            with self.subTest(extra=extra),self.assertRaises(ValueError):self.assess(log=self.log+'\n'+extra)

    def test_policy_mutation_and_premature_activation_rejected(self):
        for key,value in [('normal_lanes',[1,2,4,8,12,24]),('frames',91),('preserve_existing_v1_debt',False)]:
            policy=copy.deepcopy(self.policy);policy[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):self.assess(policy=policy)
        policy=copy.deepcopy(self.policy);policy['activation'].update(trigger='HUMAN_DEADLINE_2026_10_24',at_utc='2026-10-24T20:29:59Z')
        with self.assertRaises(ValueError):debt.check_policy(policy,now=debt.DEADLINE)
        policy['activation']['at_utc']='2026-10-24T20:30:00Z'
        self.assertTrue(debt.check_policy(policy,now=debt.DEADLINE))
        with self.assertRaises(ValueError):debt.check_policy(policy,now=debt.DEADLINE-dt.timedelta(seconds=1))


if __name__=='__main__':unittest.main()
