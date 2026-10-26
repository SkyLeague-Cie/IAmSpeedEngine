"""Portable finite-debt contracts; synthetic inputs are never native evidence.

The production module rejects this corpus. Targeted mocks only replace the
source/policy binding and allow the explicitly tagged parser fixture. Evidence,
fault enumeration and report projection run their real implementations.
"""
import argparse
from contextlib import ExitStack, contextmanager
import copy
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch
import uuid


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(filename))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


debt = load('MatrixDebtUnderTest', 'CollisionMatrixDebt.py')
fixture_module = load('MatrixFixtureUnderTest', 'TestCollisionMatrix042.py')
OUTPUT = None


def write_rows(path, rows):
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


class FiniteMatrixDebtContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = OUTPUT/uuid.uuid4().hex
        fixture_module.fixture(cls.root)
        rows = list(debt.gate.records(cls.root/'cases.csv'))
        for row in rows:
            row.update(driver_generation=1 if row['case']=='1' else 2, terminal_failure=0)
        rows[0].update(iteration_exhausted=1, frequency_pass=0, translation_pass=0)
        write_rows(cls.root/'cases.csv', rows)
        summary = json.loads((cls.root/'summary.json').read_text())
        summary.update(failed=1, frequency_failures=1, translation_failures=1, iteration_limit_cases=1)
        (cls.root/'summary.json').write_text(json.dumps(summary))
        execution = dict(completed_cases=4800, attempted_cases=4800, expected_frames=100200,
                         actual_frames=100200, drivers_created=2, terminal_failure_cases=0,
                         worlds_created=1, production_terminal_latches_cleared=False)
        (cls.root/'execution.json').write_text(json.dumps(execution))
        cls.policy = dict(spec_sha256=debt.sha256(cls.root/'spec.json'),
                          cases={'1':['execution.iteration_exhausted', 'native.frequency_pass',
                                      'native.translation_pass']}, iteration_frames=[1])
        dt = struct.unpack('f', struct.pack('f', 1./1440))[0]
        cls.iteration_message = ('LogTemp: [SpeedSolverIterationLimit] Frame=1 Iterations=24 MaxIterations=24 '
                                'ComponentSweeps=48 StaticQueries=4 AuthorityAttempts=4 ResolvedEvents=24 '
                                'TimePassed=%.17g Dt=%.17g StepMs=0.2' % (dt,dt))
        cls.report = dict(failed=1, succeeded=1, tests=[
            dict(fullTestPath=debt.gate.IDENTITY, state='Fail', warnings=0, errors=3, entries=[
                dict(event=dict(type='Info', message='Synthetic fixture only')),
                dict(event=dict(type='Error', message=cls.iteration_message)),
                dict(event=dict(type='Error', message='CollisionMatrix042 case=1 Sphere/AnalyticPlane variant=0 Hz=1440 offset=0 native=0 frequency=0 translation=0 oracle=1')),
                dict(event=dict(type='Error', message='all collision and invariance gates: The two values are not equal.'))]),
            dict(fullTestPath='IAmSpeed.OtherContract', state='Success', warnings=0, errors=0,
                 entries=[dict(event=dict(type='Info',message='Unrelated test unchanged'))])])

    @contextmanager
    def corpus(self, *, transform=None, json_update=None, source=None, allow_synthetic=True):
        original_records = debt.gate.records
        original_read = Path.read_text
        original_validate = debt.gate.validate_artifacts
        def records(path):
            for index,row in enumerate(original_records(path)):
                candidate = copy.deepcopy(row)
                if transform:
                    candidate = transform(Path(path).name,index,candidate)
                if candidate is not None:
                    yield candidate
        def read(path,*args,**kwargs):
            text = original_read(path,*args,**kwargs)
            if json_update and Path(path).parent==self.root and Path(path).suffix=='.json':
                value = json.loads(text)
                json_update(Path(path).name,value)
                return json.dumps(value)
            return text
        def validate(directory,**kwargs):
            return original_validate(directory,allow_synthetic=allow_synthetic,**kwargs)
        with ExitStack() as stack:
            stack.enter_context(patch.object(debt,'load_policy',return_value=copy.deepcopy(self.policy)))
            stack.enter_context(patch.object(debt,'source_tree',return_value=source or debt.SOURCE_TREE))
            stack.enter_context(patch.object(debt.gate,'records',side_effect=records))
            stack.enter_context(patch.object(Path,'read_text',read))
            stack.enter_context(patch.object(debt.gate,'validate_artifacts',side_effect=validate))
            yield

    def assess(self):
        return debt.validate_accepted_matrix(self.root,self.root/'unused-source-binding')

    def project(self, report=None, code=1):
        return debt.project_matrix_report(code,copy.deepcopy(report or self.report),
                                           {'state':'PASS_WITH_ACCEPTED_DEBT'},self.root)

    def test_admitted_faults_keep_strict_fail_and_raw_bytes(self):
        before = {p.name:debt.sha256(p) for p in self.root.iterdir() if p.is_file()}
        with self.corpus():
            result = self.assess()
        self.assertEqual(result['state'],'PASS_WITH_ACCEPTED_DEBT')
        self.assertEqual(result['accepted_case_count'],1)
        self.assertEqual(result['strict_assessment']['state'],'FAIL')
        self.assertFalse(result['physical_matrix_qualified'])
        self.assertEqual(result['evidence']['cases'],4800)
        self.assertEqual(result['evidence']['frames'],100200)
        self.assertEqual(result['observed_source_tree'],debt.SOURCE_TREE)
        self.assertEqual(result['frozen_source_tree'],debt.SOURCE_TREE)
        self.assertEqual(before,{p.name:debt.sha256(p) for p in self.root.iterdir() if p.is_file()})

    def test_production_gate_rejects_synthetic_evidence(self):
        with self.corpus(allow_synthetic=False),self.assertRaisesRegex(ValueError,'Synthetic parser'):
            self.assess()

    def test_another_source_tree_is_not_covered(self):
        with self.corpus(source='0'*40),self.assertRaisesRegex(ValueError,'another Source tree'):
            self.assess()

    def test_admitted_replay_source_is_observed_without_relabelling_the_freeze(self):
        replay=debt.ADMITTED_SOURCE_TREES[1]
        with self.corpus(source=replay):
            result=self.assess()
        self.assertEqual(result['observed_source_tree'],replay)
        self.assertEqual(result['frozen_source_tree'],debt.SOURCE_TREE)
        self.assertNotEqual(result['observed_source_tree'],result['frozen_source_tree'])

    def test_modified_policy_is_rejected_before_interpretation(self):
        with patch.object(debt,'sha256',return_value='0'*64),self.assertRaisesRegex(ValueError,'policy identity'):
            debt.load_policy()

    def test_actual_policy_load_is_pinned_and_finite(self):
        policy = debt.load_policy()
        self.assertEqual(len(policy['cases']),1880)
        self.assertEqual(policy['source_tree'],debt.SOURCE_TREE)
        self.assertEqual(len(policy['iteration_frames']),len(set(policy['iteration_frames'])))

    def test_spec_changes_remain_blocking(self):
        with self.corpus(),patch.object(debt,'sha256',return_value='0'*64),self.assertRaisesRegex(ValueError,'specification'):
            self.assess()

    def test_missing_case_remains_blocking(self):
        def alter(name,index,row):
            return None if name=='cases.csv' and index==5 else row
        with self.corpus(transform=alter),self.assertRaisesRegex(ValueError,'case IDs'):
            self.assess()

    def test_case_identity_cannot_move_to_another_scenario(self):
        def alter(name,index,row):
            if name=='cases.csv' and index==1:
                row['variant']='1'
            return row
        with self.corpus(transform=alter),self.assertRaisesRegex(ValueError,'Case/scenario identity'):
            self.assess()

    def test_clock_failure_is_never_debt(self):
        def alter(name,index,row):
            if name=='cases.csv' and index==0:
                row['clock_valid']='0'
            return row
        with self.corpus(transform=alter),self.assertRaisesRegex(ValueError,'clock gate failed'):
            self.assess()

    def test_incomplete_execution_is_never_debt(self):
        def update(name,value):
            if name=='execution.json':
                value['completed_cases']=4799
        with self.corpus(json_update=update),self.assertRaisesRegex(ValueError,'Incomplete native execution'):
            self.assess()

    def test_terminal_failure_is_never_debt(self):
        def alter(name,index,row):
            if name=='cases.csv' and index==0:
                row['terminal_failure']='1'
            return row
        with self.corpus(transform=alter),self.assertRaisesRegex(ValueError,'Terminal simulation'):
            self.assess()

    def test_new_physical_category_in_admitted_case_is_rejected(self):
        def alter(name,index,row):
            if name=='end-states.csv' and index==0:
                row['velocity_a_x']='10000'
            return row
        with self.corpus(transform=alter),self.assertRaisesRegex(ValueError,'Unadmitted matrix case/category'):
            self.assess()

    def test_new_native_red_flag_in_another_case_is_rejected(self):
        def alter(name,index,row):
            if name=='cases.csv' and index==1:
                row['oracle_pass']='0'
            return row
        with self.corpus(transform=alter),self.assertRaisesRegex(ValueError,'Unadmitted matrix case/category'):
            self.assess()

    def test_summary_cannot_understate_failures(self):
        def update(name,value):
            if name=='summary.json':
                value['failed']=0
        with self.corpus(json_update=update),self.assertRaisesRegex(ValueError,'aggregate mismatch'):
            self.assess()

    def test_projection_is_copy_only_and_other_test_is_untouched(self):
        original=copy.deepcopy(self.report)
        with self.corpus():
            projected,accepted=self.project(self.report)
        self.assertTrue(accepted)
        self.assertEqual(self.report,original)
        self.assertEqual(projected['tests'][1],original['tests'][1])
        self.assertEqual(projected['tests'][0]['state'],'Success')
        self.assertEqual(projected['tests'][0]['errors'],0)
        self.assertEqual(projected['failed'],0)
        self.assertEqual(projected['succeeded'],2)

    def test_unknown_native_error_is_not_projected(self):
        report=copy.deepcopy(self.report)
        report['tests'][0]['entries'].append(dict(event=dict(type='Error',message='Unexpected new failure')))
        report['tests'][0]['errors']+=1
        with self.corpus(),self.assertRaisesRegex(ValueError,'Unadmitted matrix native error'):
            self.project(report)

    def test_warning_is_not_projected(self):
        report=copy.deepcopy(self.report)
        report['tests'][0]['entries'].append(dict(event=dict(type='Warning',message='New warning')))
        with self.corpus(),self.assertRaisesRegex(ValueError,'event severity'):
            self.project(report)

    def test_unadmitted_iteration_frame_is_not_projected(self):
        report=copy.deepcopy(self.report)
        report['tests'][0]['entries'][1]['event']['message']=self.iteration_message.replace('Frame=1 ','Frame=2 ')
        with self.corpus(),self.assertRaisesRegex(ValueError,'Unadmitted or duplicate iteration'):
            self.project(report)

    def test_duplicate_iteration_diagnostic_is_rejected(self):
        report=copy.deepcopy(self.report)
        report['tests'][0]['entries'].append(copy.deepcopy(report['tests'][0]['entries'][1]))
        report['tests'][0]['errors']+=1
        with self.corpus(),self.assertRaisesRegex(ValueError,'Unadmitted or duplicate iteration'):
            self.project(report)

    def test_missing_iteration_diagnostic_is_rejected(self):
        report=copy.deepcopy(self.report)
        del report['tests'][0]['entries'][1]
        report['tests'][0]['errors']-=1
        with self.corpus(),self.assertRaisesRegex(ValueError,'Missing iteration-limit diagnostic'):
            self.project(report)

    def test_negative_diagnostic_time_is_rejected(self):
        report=copy.deepcopy(self.report)
        report['tests'][0]['entries'][1]['event']['message']=self.iteration_message.replace('StepMs=0.2','StepMs=-0.2')
        with self.corpus(),self.assertRaisesRegex(ValueError,'diagnostic physical clock'):
            self.project(report)

    def test_error_counter_cannot_hide_missing_entries(self):
        report=copy.deepcopy(self.report)
        report['tests'][0]['errors']=2
        with self.corpus(),self.assertRaisesRegex(ValueError,'counter mismatch'):
            self.project(report)

    def test_crash_exit_is_never_accepted(self):
        with self.corpus(),self.assertRaisesRegex(ValueError,'crash exit'):
            self.project(code=3)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output-root',type=Path,
                        default=Path(__file__).resolve().parents[2]/'Artifacts/Matrix042DebtParser')
    args,remaining=parser.parse_known_args()
    OUTPUT=args.output_root.resolve()
    OUTPUT.mkdir(parents=True,exist_ok=True)
    unittest.main(argv=[sys.argv[0],*remaining])
