"""Artifact gate tests only. No Unreal/Docker or physical qualification.

The synthetic corpus is explicitly tagged and rejected by the production gate.
Each invocation retains a unique output directory; no recursive cleanup occurs.
"""
import argparse
import copy
import csv
import importlib.util
import itertools
import json
import math
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch
import uuid

loader = importlib.util.spec_from_file_location('CollisionMatrix042', Path(__file__).with_name('CollisionMatrix042.py'))
gate = importlib.util.module_from_spec(loader)
loader.loader.exec_module(gate)
OUTPUT = None


def write_rows(path, rows):
    with path.open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)


def fixture(root):
    root.mkdir(parents=True, exist_ok=False)
    spec = dict(schema=1, rates_hz=list(gate.RATES), reference_hz=1440,
                shape_a=list(gate.SHAPES_A), shape_b=list(gate.SHAPES_B), variants=8,
                translations_cm=[list(x) for x in gate.TRANSLATIONS], duration_seconds=.05,
                position_grid_cm=.01, geometric_cm=.02, normal_radians=.002,
                translation_cm=.0001, translation_time_s=1e-6, translation_normal_radians=2e-6,
                translation_velocity_cm_s=.01, translation_angular_rad_s=.001, synthetic_parser_fixture=True)
    cases, initial, end, events = [], [], [], []
    frame = 0
    for ident, (a,b,v,h,t) in enumerate(itertools.product(gate.SHAPES_A,gate.SHAPES_B,range(8),gate.RATES,range(5)),1):
        dt = struct.unpack('f',struct.pack('f',1./h))[0]
        local = math.floor(.021/dt); elapsed = .021-local*dt
        point_bound=.02+math.sqrt(3)*.005*(h//20+1440//20)
        speed=300. if v==1 else 4600. if v==2 else 1200.
        cases.append(dict(case=ident,shape_a=a,shape_b=b,variant=v,hz=h,translation=t,
            complete=1,clock_valid=1,actual_delta_valid=1,hits=1,impact_seconds=.021,
            first_global_frame=frame,frames=h//20,world_id=17,iteration_exhausted=0,
            observer_overflow=0,real_wheel_hit=int(a=='Wheel'),max_delta_error_ms=0,
            frequency_pass=1,translation_pass=1,oracle_pass=1,
            point_x=0,point_y=0,point_z=0,normal_x=0,normal_y=0,normal_z=1,
            point_bound_cm=point_bound,time_bound_s=1e-6+point_bound/(speed-75.)))
        frame += h//20
        row = dict(case=ident)
        for prefix in ('a','b','environment'):
            for axis,shift in zip('xyz',gate.TRANSLATIONS[t]): row[prefix+'_'+axis]=shift
        for prefix in ('a','b'):
            for component in ('qx','qy','qz','qw','vx','vy','vz','wx','wy','wz'):
                row[prefix+'_'+component]=int(component=='qw')
        row.update(mass_a=18 if a=='Wheel' else 10,mass_b=0 if b in ('AnalyticPlane','RingStatic','CylinderPrescribed') else 10)
        initial.append(row)
        final=dict(case=ident,sample_seconds=.05)
        event=dict(case=ident,local_frame=local,elapsed_in_frame=elapsed,query_toi=elapsed,
                   point_x=0,point_y=0,point_z=0,normal_x=0,normal_y=0,normal_z=1,feature_a=1,feature_b=2)
        for actor in ('a','b'):
            for axis in 'xyz':final['position_'+actor+'_'+axis]=0
            for quantity in ('velocity','angular'):
                for axis in 'xyz': final[quantity+'_'+actor+'_'+axis]=event[quantity+'_'+actor+'_'+axis]=0
        end.append(final);events.append(event)
    summary=dict(worlds_created_by_harness=1,driver='AFastSimulation',fast_batch=True,
        highest_hz=1440,expected_cases=len(cases),actual_cases=len(cases),frames=frame,
        incomplete=False,export_valid=True,wheel_coverage=True,wheel_cases=len(cases)//3,wheel_hit_cases=len(cases)//3,
        failed=0,frequency_failures=0,translation_failures=0,oracle_failures=0,iteration_limit_cases=0,observer_overflow_cases=0)
    for name,data in (('spec.json',spec),('summary.json',summary)):
        with (root/name).open('x',encoding='utf-8') as stream: json.dump(data,stream)
    for name,rows in (('cases.csv',cases),('initial-states.csv',initial),('end-states.csv',end),('events.csv',events)):
        write_rows(root/name,rows)
    write_rows(root/'support-samples.csv',[dict(case=3201,local_frame=0,
        sample_seconds=struct.unpack('f',struct.pack('f',1./1440))[0],query_parameter=.5,
        point_x=0,point_y=0,point_z=0,normal_x=0,normal_y=0,normal_z=1)])


class ArtifactContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root=OUTPUT/uuid.uuid4().hex
        fixture(cls.root)

    def validate(self):
        return gate.validate_artifacts(self.root,allow_synthetic=True)

    def altered_rows(self, filename, alter):
        original=gate.records
        def rows(path):
            for index,row in enumerate(original(path)):
                if Path(path).name==filename:
                    row=copy.deepcopy(row)
                    row=alter(index,row)
                    if row is None: continue
                yield row
        return patch.object(gate,'records',rows)

    def test_complete_format_contract_is_not_physics_execution(self):
        result=self.validate()
        self.assertEqual(result['cases'],4800)
        self.assertEqual(result['frames'],100200)
        self.assertEqual(result['physical_execution_provenance'],'REQUIRES_EDITOR_EXIT_AND_AUTOMATION_REPORT')

    def test_synthetic_input_is_rejected_in_ci_mode(self):
        with self.assertRaisesRegex(ValueError,'Synthetic parser'):
            gate.validate_artifacts(self.root)

    def test_translation_drift_cannot_hide_behind_native_pass_flag(self):
        def change(i,r):
            if i==1:r['point_x']='0.01'
            return r
        with self.altered_rows('cases.csv',change),self.altered_rows('events.csv',change):
            with self.assertRaisesRegex(ValueError,'Translation geometry'):self.validate()

    def test_lower_frequency_drift_cannot_hide_behind_native_pass_flag(self):
        def change(i,r):
            if i==5:r['point_x']='3'
            return r
        with self.altered_rows('cases.csv',change),self.altered_rows('events.csv',change):
            with self.assertRaisesRegex(ValueError,'Frequency geometry'):self.validate()

    def test_raw_first_contact_must_match_summary(self):
        def change(i,r):
            if i==1:r['point_x']='0.01'
            return r
        with self.altered_rows('events.csv',change):
            with self.assertRaisesRegex(ValueError,'First contact differs'):self.validate()

    def test_post_impact_angular_drift_is_independently_rejected(self):
        def change(i,r):
            if i==1:r['angular_b_x']='0.005'
            return r
        with self.altered_rows('events.csv',change):
            with self.assertRaisesRegex(ValueError,'Translation immediate'):self.validate()

    def test_common_time_velocity_drift_is_independently_rejected(self):
        def change(i,r):
            if i==1:r['velocity_a_x']='0.05'
            return r
        with self.altered_rows('end-states.csv',change):
            with self.assertRaisesRegex(ValueError,'Translation common-time'):self.validate()

    def test_missing_case(self):
        with self.altered_rows('cases.csv',lambda i,r:None if i==0 else r):
            with self.assertRaisesRegex(ValueError,'case IDs'):self.validate()

    def test_world_recreation(self):
        def change(i,r):
            if i==1:r['world_id']='18'
            return r
        with self.altered_rows('cases.csv',change):
            with self.assertRaisesRegex(ValueError,'World identity'):self.validate()

    def test_hidden_frame_reset(self):
        def change(i,r):
            if i==1:r['first_global_frame']='0'
            return r
        with self.altered_rows('cases.csv',change):
            with self.assertRaisesRegex(ValueError,'frame discontinuity'):self.validate()

    def test_wrong_real_physics_delta(self):
        def change(i,r):
            if i==0:r['actual_delta_valid']='0'
            return r
        with self.altered_rows('cases.csv',change):
            with self.assertRaisesRegex(ValueError,'clock gate'):self.validate()

    def test_environment_not_translated(self):
        def change(i,r):
            if i==1:r['environment_x']='0'
            return r
        with self.altered_rows('initial-states.csv',change):
            with self.assertRaisesRegex(ValueError,'common translation'):self.validate()

    def test_velocity_changed_under_translation(self):
        def change(i,r):
            if i==1:r['a_vx']='2'
            return r
        with self.altered_rows('initial-states.csv',change):
            with self.assertRaisesRegex(ValueError,'Non-position input'):self.validate()

    def test_impact_time_is_in_seconds_within_episode(self):
        def change(i,r):
            if i==0:r['elapsed_in_frame']='1'
            return r
        with self.altered_rows('events.csv',change):
            with self.assertRaisesRegex(ValueError,'outside the actual physical frame'):self.validate()

    def test_non_unit_normal(self):
        def change(i,r):
            if i==0:r['normal_z']='2'
            return r
        with self.altered_rows('events.csv',change):
            with self.assertRaisesRegex(ValueError,'not unit'):self.validate()

    def test_failed_native_gate_cannot_be_waived(self):
        def change(i,r):
            if i==0:r['translation_pass']='0'
            return r
        with self.altered_rows('cases.csv',change):
            with self.assertRaisesRegex(ValueError,'physical gate failed'):self.validate()

    def test_fake_wheel_coverage(self):
        def change(i,r):
            if r['shape_a']=='Wheel':r['real_wheel_hit']='0'
            return r
        with self.altered_rows('cases.csv',change):
            with self.assertRaisesRegex(ValueError,'not actually contacted'):self.validate()

    def test_diagnostics_separate_incomplete_episodes(self):
        def change(i,r):
            if i==0:r.update(complete='0',frames='0')
            return r
        with self.altered_rows('cases.csv',change):
            result=gate.diagnose_artifacts(self.root)
        self.assertEqual(result['state'],'DIAGNOSTIC_ONLY_NO_QUALIFICATION')
        self.assertEqual(result['summary']['completed'],4799)
        self.assertEqual(result['summary']['failures']['execution.incomplete'],1)
        self.assertGreater(result['summary']['failures']['frequency.not_comparable'],0)

    def test_diagnostics_recompute_response_despite_native_pass(self):
        def change(i,r):
            if i==40:r['velocity_a_z']='1200'
            return r
        with self.altered_rows('events.csv',change):
            result=gate.diagnose_artifacts(self.root)
        self.assertGreater(result['summary']['failures']['frequency.immediate_velocity'],0)

    def test_diagnostics_detect_non_equivalent_initial_conditions(self):
        def change(i,r):
            if i==1:r['a_z']=str(float(r['a_z'])+1)
            return r
        with self.altered_rows('initial-states.csv',change):
            result=gate.diagnose_artifacts(self.root)
        self.assertEqual(result['summary']['failures']['input.common_translation_changed'],1)

    def test_diagnostics_missing_reference_is_not_a_numeric_delta(self):
        def change(i,r):return None if i==0 else r
        with self.altered_rows('events.csv',change):
            result=gate.diagnose_artifacts(self.root)
        self.assertEqual(result['summary']['failures']['contact.missing'],1)
        self.assertGreater(result['summary']['failures']['frequency.reference_contact_missing'],0)
        self.assertNotIn('frequency.point',result['summary']['failures'])

    def test_support_query_parameter_is_not_a_CCD_time(self):
        def change(i,r):r['query_parameter']='1000000000';return r
        with self.altered_rows('support-samples.csv',change):
            result=gate.diagnose_artifacts(self.root)
            self.validate()
        self.assertEqual(result['wheel_support']['samples'],1)
        self.assertIn('not a CCD impact time',result['wheel_support']['semantics'])

    def test_support_frame_time_is_validated_independently(self):
        def change(i,r):r['sample_seconds']='.049';return r
        with self.altered_rows('support-samples.csv',change):
            with self.assertRaisesRegex(ValueError,'wrong frame-end time'):self.validate()

    def test_support_pairing_uses_highest_rate_and_common_time_without_a_new_PASS(self):
        original=gate.records
        def rows(path):
            for row in original(path):
                yield row
                if Path(path).name=='support-samples.csv':
                    second=copy.deepcopy(row);second.update(case='3206',
                        sample_seconds=str(struct.unpack('f',struct.pack('f',1./720))[0]),point_x='3')
                    yield second
        with patch.object(gate,'records',side_effect=rows):
            result=gate.diagnose_artifacts(self.root)
        pairs=result['wheel_support']['paired_comparisons']['frequency']
        self.assertEqual(pairs['state'],'MEASUREMENTS_ONLY_NO_NEW_THRESHOLD_OR_PASS')
        self.assertEqual(pairs['maxima']['first_sample_point_cm'],dict(value=3.,case=3206,reference_case=3201))
        self.assertGreater(pairs['maxima']['first_sample_time_delta_s']['value'],0)
        self.assertEqual(pairs['maxima']['common50ms_velocity_a']['value'],0)

    def test_support_comparison_rejects_a_wrong_common_time_sample(self):
        def change(i,r):
            if r['case']=='3201':r['sample_seconds']='.049'
            return r
        with self.altered_rows('end-states.csv',change):
            result=gate.diagnose_artifacts(self.root)
        pairs=result['wheel_support']['paired_comparisons']['frequency']
        self.assertEqual(pairs['compared_cases'],0)
        self.assertEqual(pairs['unavailable_cases'],1)
        self.assertFalse(pairs['maxima'])

    def test_stationary_prescribed_owner_cannot_inherit_free_body_motion(self):
        def change(i,r):
            if r['case']=='1281':r['velocity_b_x']='20000'
            return r
        with self.altered_rows('end-states.csv',change):
            with self.assertRaisesRegex(ValueError,'prescribed fixture was deflected'):self.validate()
            result=gate.diagnose_artifacts(self.root)
        self.assertEqual(result['summary']['failures']['fixture.prescribed_owner_deflected'],1)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output-root',type=Path,default=Path(__file__).resolve().parents[2]/'Artifacts/Matrix042Parser')
    args,remaining=parser.parse_known_args()
    OUTPUT=args.output_root.resolve();OUTPUT.mkdir(parents=True,exist_ok=True)
    unittest.main(argv=[sys.argv[0],*remaining])
