"""Strict coverage/evidence gate for the native single-world collision matrix.

This validates artifacts, not physical execution provenance by itself. The CI
runner must also retain the real Editor exit, exact Automation report and source
identity. No debt policy or warning waiver applies to this new matrix.
"""
import csv
from collections import Counter, defaultdict
import itertools
import json
import math
from pathlib import Path
import struct

IDENTITY = 'IAmSpeed.PhysicalLaws.CollisionMatrix042.FastBatch'
RATES = (1440, 720, 360, 300, 240, 120, 100, 60)
SHAPES_A = ('Sphere', 'Box', 'Wheel')
SHAPES_B = ('AnalyticPlane', 'RingStatic', 'Sphere', 'Box', 'CylinderPrescribed')
TRANSLATIONS = ((0., 0., 0.), (1000., -2000., 3000.),
                (-1048576., 2097152., -524288.), (1e8, -1e8, 1e8), (.003, -.004, .001))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def number(row, key):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('Missing or malformed numeric field: ' + key) from exc
    require(math.isfinite(value), 'Non-finite field: ' + key)
    return value


def integer(row, key):
    value = number(row, key)
    require(value.is_integer(), 'Non-integer field: ' + key)
    return int(value)


def records(path):
    require(path.is_file() and path.stat().st_size <= 512 * 1024**2,
            'Missing or oversized matrix artifact: ' + path.name)
    with path.open(encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream)
        require(reader.fieldnames and len(reader.fieldnames) == len(set(reader.fieldnames)),
                'Duplicate or empty CSV header: ' + path.name)
        for row in reader:
            require(None not in row and all(value is not None for value in row.values()),
                    'Malformed CSV width: ' + path.name)
            yield row


def by_case(path):
    result = {}
    for row in records(path):
        ident = integer(row, 'case')
        require(ident not in result, 'Duplicate case: ' + path.name)
        result[ident] = row
    return result


def vector(row, keys):
    return tuple(number(row, key) for key in keys)


def distance(a, b):
    return math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))


def angle(a, b):
    la, lb = distance(a, (0.,)*3), distance(b, (0.,)*3)
    require(abs(la-1) < 1e-5 and abs(lb-1) < 1e-5, 'ImpactNormal is not unit length')
    return math.acos(max(-1., min(1., sum(x*y for x, y in zip(a, b))/(la*lb))))


def xyz(row, prefix):
    return vector(row, tuple(prefix+'_'+axis for axis in 'xyz'))


def compare_response(event, reference, end, end_reference, velocity_bound, angular_bound, label):
    for actor in ('a', 'b'):
        for quantity, bound in (('velocity', velocity_bound), ('angular', angular_bound)):
            prefix = quantity+'_'+actor
            require(distance(xyz(event, prefix), xyz(reference, prefix)) <= bound,
                    label+' immediate post-impact '+prefix+' differs')
            require(distance(xyz(end, prefix), xyz(end_reference, prefix)) <= bound,
                    label+' common-time post-impact '+prefix+' differs')


def validate_support_samples(root, cases):
    counts=Counter()
    for sample in records(root/'support-samples.csv'):
        ident=integer(sample,'case')
        require(ident in cases and cases[ident]['shape_a']=='Wheel','Support sample has no Wheel case')
        frame=integer(sample,'local_frame')
        require(0<=frame<integer(cases[ident],'frames'),'Support sample outside episode')
        expected=(frame+1)*struct.unpack('f',struct.pack('f',1./integer(cases[ident],'hz')))[0]
        require(abs(number(sample,'sample_seconds')-expected)<1e-10,'Support sample has wrong frame-end time')
        for field in sample: number(sample,field)
        angle(xyz(sample,'normal'),xyz(sample,'normal'))
        counts[ident]+=1
    return counts


def validate_artifacts(directory, *, allow_synthetic=False):
    root = Path(directory)
    spec = json.loads((root/'spec.json').read_text(encoding='utf-8-sig'))
    summary = json.loads((root/'summary.json').read_text(encoding='utf-8-sig'))
    require(allow_synthetic or not spec.get('synthetic_parser_fixture'),
            'Synthetic parser fixtures are not native CI evidence')
    require(spec.get('rates_hz') == list(RATES) and spec.get('reference_hz') == max(RATES),
            'Frequency set or highest-frequency reference changed')
    require(spec.get('shape_a') == list(SHAPES_A) and spec.get('shape_b') == list(SHAPES_B),
            'Shape-role coverage changed')
    require(spec.get('translations_cm') == [list(x) for x in TRANSLATIONS]
            and spec.get('variants') == 8 and spec.get('duration_seconds') == .05,
            'Translation, variant or physical-time coverage changed')
    for key, value in {'position_grid_cm': .01, 'geometric_cm': .02, 'normal_radians': .002,
                       'translation_cm': .0001, 'translation_time_s': 1e-6,
                       'translation_normal_radians': 2e-6, 'translation_velocity_cm_s': .01,
                       'translation_angular_rad_s': .001}.items():
        require(spec.get(key) == value, 'Predeclared tolerance changed: ' + key)
    expected = set(itertools.product(SHAPES_A, SHAPES_B, range(8), RATES, range(5)))
    require(summary.get('worlds_created_by_harness') == 1
            and summary.get('driver') == 'AFastSimulation' and summary.get('fast_batch') is True,
            'Matrix did not report one world and the real FastBatch driver')
    require(summary.get('expected_cases') == summary.get('actual_cases') == len(expected)
            and summary.get('incomplete') is False and summary.get('export_valid') is True,
            'Incomplete native matrix or export')
    cases, initial, end = (by_case(root/name) for name in ('cases.csv', 'initial-states.csv', 'end-states.csv'))
    require(set(cases) == set(initial) == set(end) == set(range(1, len(expected)+1)),
            'Missing, extra or non-contiguous evidence case IDs')
    keyed, next_frame, worlds = {}, 0, set()
    for ident, row in sorted(cases.items()):
        key = (row['shape_a'], row['shape_b'], integer(row, 'variant'), integer(row, 'hz'), integer(row, 'translation'))
        require(key not in keyed and key in expected, 'Duplicate or undeclared scenario')
        keyed[key] = ident
        require(integer(row, 'first_global_frame') == next_frame, 'Inter-case frame discontinuity')
        require(integer(row, 'frames') == key[3]//20, 'Wrong finite physical duration')
        next_frame += integer(row, 'frames'); worlds.add(integer(row, 'world_id'))
        for field in ('complete', 'clock_valid', 'actual_delta_valid'):
            require(integer(row, field) == 1, 'Native execution/clock gate failed: ' + field)
        for field in ('iteration_exhausted', 'observer_overflow'):
            require(integer(row, field) == 0, 'Native budget/observation exhausted: ' + field)
        require(integer(row, 'hits') > 0 and number(row, 'max_delta_error_ms') < 1e-5,
                'Missing collision or wrong actual physics delta')
        require(abs(number(end[ident], 'sample_seconds')-.05) < 1e-12,
                'End states were not sampled at the common physical time')
        if key[1]=='CylinderPrescribed':
            for prefix in ('position_b','velocity_b','angular_b'):
                require(xyz(end[ident],prefix)==(0.,0.,0.),
                        'Stationary prescribed fixture was deflected: '+prefix)
    require(len(worlds) == 1 and set(keyed) == expected and summary.get('frames') == next_frame,
            'World identity, complete coverage or frame aggregate mismatch')
    counts, first = {ident: 0 for ident in cases}, {}
    for event in records(root/'events.csv'):
        ident = integer(event, 'case'); require(ident in cases, 'Unknown event case')
        row = cases[ident]; hz = integer(row, 'hz'); frame = integer(event, 'local_frame')
        require(0 <= frame < integer(row, 'frames'), 'Contact outside episode')
        delta = struct.unpack('f', struct.pack('f', 1./hz))[0]
        elapsed = number(event, 'elapsed_in_frame'); query_toi = number(event, 'query_toi')
        require(0 <= elapsed <= delta+1e-7 and 0 <= query_toi <= delta+1e-7,
                'Contact time is outside the actual physical frame')
        for field in event:
            number(event, field)
        normal = vector(event, ('normal_x', 'normal_y', 'normal_z'))
        angle(normal, normal)
        counts[ident] += 1
        if ident not in first:
            first[ident] = event
            require(abs(number(row, 'impact_seconds')-(frame*delta+elapsed)) < 1e-10,
                    'First impact seconds do not match the native event')
    for ident, row in cases.items():
        require(counts[ident] == integer(row, 'hits'), 'Contact stream truncated or duplicated')
        key = (row['shape_a'], row['shape_b'], integer(row, 'variant'), integer(row, 'hz'), integer(row, 'translation'))
        origin = initial[keyed[key[:-1]+(0,)]]; this = initial[ident]
        translation = TRANSLATIONS[key[-1]]
        for prefix in ('a', 'b', 'environment'):
            for axis, shift in zip('xyz', translation):
                field = prefix+'_'+axis
                require(abs(number(this, field)-shift-number(origin, field)) < 2e-7,
                        'Actors and environment did not receive the same common translation')
        for field in ('a_qx','a_qy','a_qz','a_qw','a_vx','a_vy','a_vz','a_wx','a_wy','a_wz',
                      'b_qx','b_qy','b_qz','b_qw','b_vx','b_vy','b_vz','b_wx','b_wy','b_wz','mass_a','mass_b'):
            require(number(this, field) == number(origin, field), 'Non-position input changed under translation')
        for field in ('frequency_pass','translation_pass','oracle_pass'):
            require(integer(row, field) == 1, 'Native physical gate failed: ' + field)
        require(key[0] != 'Wheel' or integer(row, 'real_wheel_hit') == 1, 'Wheel was not actually contacted')
        event = first[ident]
        for quantity in ('point', 'normal'):
            require(distance(xyz(row, quantity), xyz(event, quantity)) < 1e-10,
                    'First contact differs from its case summary: '+quantity)
        origin_id = keyed[key[:-1]+(0,)]
        reference_id = keyed[key[:3]+(max(RATES), key[-1])]
        reference, origin_event = first[reference_id], first[origin_id]
        reference_row, origin_row = cases[reference_id], cases[origin_id]
        speed = 300. if key[2] == 1 else 4600. if key[2] == 2 else 1200.
        point_bound = .02+math.sqrt(3)*.005*(integer(row,'frames')+integer(reference_row,'frames'))
        time_bound = 1e-6+point_bound/(speed-75.)
        require(abs(number(row,'point_bound_cm')-point_bound)<1e-10
                and abs(number(row,'time_bound_s')-time_bound)<1e-10,
                'Reported frequency bounds differ from the frozen specification')
        point_error = distance(xyz(event,'point'),xyz(reference,'point'))
        time_error = abs(number(row,'impact_seconds')-number(reference_row,'impact_seconds'))
        require(point_error <= point_bound and time_error <= time_bound
                and angle(xyz(event,'normal'),xyz(reference,'normal')) <= .002,
                'Frequency geometry or TOI invariance failed despite native pass flag')
        require(distance(xyz(event,'point'),xyz(origin_event,'point')) <= .0001
                and abs(number(row,'impact_seconds')-number(origin_row,'impact_seconds')) <= 1e-6
                and angle(xyz(event,'normal'),xyz(origin_event,'normal')) <= 2e-6,
                'Translation geometry or TOI invariance failed despite native pass flag')
        for feature in ('feature_a','feature_b'):
            require(integer(event,feature)==integer(reference,feature)==integer(origin_event,feature),
                    'First contact feature changed under frequency or translation')
        compare_response(event,reference,end[ident],end[reference_id],.02+.002*speed,.01+.002*speed/12.5,'Frequency')
        compare_response(event,origin_event,end[ident],end[origin_id],.01,.001,'Translation')
        if key[0]=='Sphere' and key[1]=='AnalyticPlane' and key[2] not in (3,4):
            require(abs(number(row,'impact_seconds')-.021) <= 1e-6+(.02+.005*integer(row,'frames'))/speed
                    and abs(number(event,'point_z')) <= .02
                    and angle(xyz(event,'normal'),(0.,0.,1.)) <= 1e-6
                    and abs(number(event,'velocity_a_z')) <= .02,
                    'Independent Sphere/Plane oracle failed despite native pass flag')
    require(summary.get('wheel_coverage') is True
            and summary.get('wheel_cases') == summary.get('wheel_hit_cases') == len(expected)//3,
            'Incomplete actual Wheel contact coverage')
    require(all(summary.get(k) == 0 for k in ('failed','frequency_failures','translation_failures',
                'oracle_failures','iteration_limit_cases','observer_overflow_cases')),
            'Native failure aggregate remains nonzero')
    validate_support_samples(root,cases)
    return dict(state='ARTIFACT_CONTRACT_OK', cases=len(cases), frames=next_frame, worlds=1,
                native_identity=IDENTITY, highest_hz=max(RATES), physical_execution_provenance='REQUIRES_EDITOR_EXIT_AND_AUTOMATION_REPORT')


def diagnose_artifacts(directory):
    """Recompute all comparison failures, including when the strict gate rejects.

    Diagnostic categories can overlap. Missing contacts and incomplete episodes
    are kept separate from measured differences. This never grants a PASS or
    changes the predeclared tolerances, and does not establish native provenance.
    """
    root = Path(directory)
    cases, initial, end = (by_case(root/name) for name in
                           ('cases.csv', 'initial-states.csv', 'end-states.csv'))
    require(set(cases) == set(initial) == set(end), 'Diagnostic case ID mismatch')
    first, event_counts = {}, Counter()
    for event in records(root/'events.csv'):
        ident = integer(event, 'case')
        require(ident in cases, 'Unknown diagnostic event case')
        first.setdefault(ident, event)
        event_counts[ident] += 1
    def key(row):
        return (row['shape_a'], row['shape_b'], integer(row, 'variant'),
                integer(row, 'hz'), integer(row, 'translation'))
    keyed = {key(row): ident for ident, row in cases.items()}
    require(len(keyed) == len(cases), 'Duplicate diagnostic scenario')
    faults, metrics = defaultdict(set), {}
    non_positions = tuple(prefix+'_'+field for prefix in ('a', 'b')
        for field in ('qx','qy','qz','qw','vx','vy','vz','wx','wy','wz'))+('mass_a','mass_b')
    for ident, row in cases.items():
        tags = faults[ident]
        completed = integer(row, 'complete') == 1
        if not completed: tags.add('execution.incomplete')
        if integer(row, 'frames') != integer(row, 'hz')//20: tags.add('execution.duration')
        for field in ('clock_valid','actual_delta_valid'):
            if integer(row, field) != 1: tags.add('execution.'+field)
        for field in ('iteration_exhausted','observer_overflow'):
            if integer(row, field): tags.add('execution.'+field)
        if 'terminal_failure' in row and integer(row, 'terminal_failure'):
            tags.add('execution.terminal')
        if event_counts[ident] != integer(row, 'hits'): tags.add('evidence.event_count')
        if row['shape_b']=='CylinderPrescribed' and any(xyz(end[ident],f)!=(0.,0.,0.) for f in ('position_b','velocity_b','angular_b')):
            tags.add('fixture.prescribed_owner_deflected')
        if ident not in first: tags.add('contact.missing')
        if row['shape_a'] == 'Wheel' and not integer(row, 'real_wheel_hit'):
            tags.add('contact.wheel_not_observed')
        scenario = key(row)
        origin_id = keyed.get(scenario[:-1]+(0,))
        reference_id = keyed.get(scenario[:3]+(max(RATES), scenario[-1]))
        if origin_id is not None:
            residual = max(abs(number(initial[ident],prefix+'_'+axis)-shift-
                               number(initial[origin_id],prefix+'_'+axis))
                for prefix in ('a','b','environment')
                for axis,shift in zip('xyz',TRANSLATIONS[scenario[-1]]))
            metrics[ident] = {'initial_translation_residual_cm':residual}
            if residual >= 2e-7: tags.add('input.common_translation_changed')
            if any(number(initial[ident],f) != number(initial[origin_id],f) for f in non_positions):
                tags.add('input.non_position_changed')
        else:
            tags.add('evidence.origin_missing')
        for label, other_id in (('frequency',reference_id),('translation',origin_id)):
            if other_id is None or other_id not in first:
                tags.add(label+'.reference_contact_missing'); continue
            if ident not in first or not completed or integer(cases[other_id],'complete') != 1:
                tags.add(label+'.not_comparable'); continue
            event, reference = first[ident], first[other_id]
            speed = 300. if scenario[2] == 1 else 4600. if scenario[2] == 2 else 1200.
            point_bound = .02+math.sqrt(3)*.005*(integer(row,'frames')+integer(cases[other_id],'frames'))
            bounds = (point_bound,1e-6+point_bound/(speed-75.),.002,
                      .02+.002*speed,.01+.002*speed/12.5) if label=='frequency' else (.0001,1e-6,2e-6,.01,.001)
            point_error = distance(xyz(event,'point'),xyz(reference,'point'))
            time_error = abs(number(row,'impact_seconds')-number(cases[other_id],'impact_seconds'))
            metrics.setdefault(ident,{}).update({label+'_point_error_cm':point_error,
                                                label+'_time_error_s':time_error})
            if point_error > bounds[0]: tags.add(label+'.point')
            if time_error > bounds[1]: tags.add(label+'.toi')
            try:
                if angle(xyz(event,'normal'),xyz(reference,'normal')) > bounds[2]: tags.add(label+'.normal')
            except ValueError:
                tags.add('evidence.non_unit_normal')
            if any(integer(event,f) != integer(reference,f) for f in ('feature_a','feature_b')):
                tags.add(label+'.feature')
            for actor in ('a','b'):
                for quantity,bound in (('velocity',bounds[3]),('angular',bounds[4])):
                    prefix = quantity+'_'+actor
                    if distance(xyz(event,prefix),xyz(reference,prefix)) > bound:
                        tags.add(label+'.immediate_'+quantity)
                    if distance(xyz(end[ident],prefix),xyz(end[other_id],prefix)) > bound:
                        tags.add(label+'.common_time_'+quantity)
        if scenario[0:2] == ('Sphere','AnalyticPlane') and scenario[2] not in (3,4):
            if ident not in first: tags.add('oracle.missing_contact')
            else:
                event=first[ident]
                speed=300. if scenario[2]==1 else 4600. if scenario[2]==2 else 1200.
                if abs(number(row,'impact_seconds')-.021) > 1e-6+(.02+.005*integer(row,'frames'))/speed:
                    tags.add('oracle.toi')
                if abs(number(event,'point_z')) > .02: tags.add('oracle.point')
                try:
                    if angle(xyz(event,'normal'),(0.,0.,1.)) > 1e-6: tags.add('oracle.normal')
                except ValueError: tags.add('evidence.non_unit_normal')
                if abs(number(event,'velocity_a_z')) > .02: tags.add('oracle.response')
    def aggregate(ids):
        ids=list(ids)
        return dict(cases=len(ids),completed=sum(integer(cases[i],'complete') for i in ids),
            contacts=sum(i in first for i in ids),frames=sum(integer(cases[i],'frames') for i in ids),
            cases_with_diagnostic_faults=sum(bool(faults[i]) for i in ids),
            failures=dict(sorted(Counter(tag for i in ids for tag in faults[i]).items())))
    def grouped(dimension):
        groups=defaultdict(list)
        for ident,row in cases.items(): groups[dimension(row)].append(ident)
        return {name:aggregate(ids) for name,ids in sorted(groups.items())}
    support_path=root/'support-samples.csv'
    support_counts=validate_support_samples(root,cases) if support_path.is_file() else Counter()
    support_first={}
    if support_path.is_file():
        for sample in records(support_path):support_first.setdefault(integer(sample,'case'),sample)
    support_comparisons={}
    def common_time_complete(ident):
        row=cases[ident]
        return (all(integer(row,f)==1 for f in ('complete','clock_valid','actual_delta_valid'))
                and integer(row,'frames')==integer(row,'hz')//20
                and abs(number(end[ident],'sample_seconds')-.05)<1e-12
                and 'fixture.prescribed_owner_deflected' not in faults[ident])
    for label in ('frequency','translation'):
        maxima={};compared=0;unavailable=0
        for ident,sample in support_first.items():
            scenario=key(cases[ident])
            other_id=keyed.get(scenario[:3]+(max(RATES),scenario[-1])if label=='frequency' else scenario[:-1]+(0,))
            if other_id not in support_first or not common_time_complete(ident) or not common_time_complete(other_id):
                unavailable+=1;continue
            reference=support_first[other_id];compared+=1
            values=dict(first_sample_point_cm=distance(xyz(sample,'point'),xyz(reference,'point')),
                first_sample_normal_radians=angle(xyz(sample,'normal'),xyz(reference,'normal')),
                first_sample_time_delta_s=abs(number(sample,'sample_seconds')-number(reference,'sample_seconds')))
            for actor in ('a','b'):
                for quantity in ('velocity','angular'):
                    values['common50ms_'+quantity+'_'+actor]=distance(xyz(end[ident],quantity+'_'+actor),xyz(end[other_id],quantity+'_'+actor))
            for metric,value in values.items():
                if metric not in maxima or value>maxima[metric]['value']:
                    maxima[metric]=dict(value=value,case=ident,reference_case=other_id)
        support_comparisons[label]=dict(compared_cases=compared,unavailable_cases=unavailable,maxima=maxima,
            state='MEASUREMENTS_ONLY_NO_NEW_THRESHOLD_OR_PASS',
            time_semantics='Difference between first frame-end support samples, not a CCD TOI error',
            state_semantics='Both actors compared at common50ms; first support observations can occur at different times')
    return dict(schema=1,state='DIAGNOSTIC_ONLY_NO_QUALIFICATION',categories_may_overlap=True,
        tolerances_unchanged=True,missing_contact_is_not_a_measured_invariance_delta=True,
        incomplete_case_is_not_a_completed_physics_comparison=True,
        summary=aggregate(cases),world_ids=sorted({integer(r,'world_id') for r in cases.values()}),
        driver_generations=sorted({integer(r,'driver_generation') for r in cases.values() if 'driver_generation' in r}),
        by_pair=grouped(lambda r:r['shape_a']+'/'+r['shape_b']),
        by_rate=grouped(lambda r:r['hz']),by_translation=grouped(lambda r:r['translation']),
        wheel_support=dict(artifact_present=support_path.is_file(),samples=sum(support_counts.values()),cases=len(support_counts),
            cases_without_CCD_contact=sum(i not in first for i in support_counts),
            by_peer=dict(sorted(Counter(cases[i]['shape_b'] for i in support_counts).items())),
            paired_comparisons=support_comparisons,
            semantics='Native frame-end suspension response; query_parameter is not a CCD impact time; no CCD gate is waived'),
        largest_initial_translation_residual_cm=max((m.get('initial_translation_residual_cm',0) for m in metrics.values()),default=0),
        first_fault_cases=[dict(case=i,scenario=key(cases[i]),faults=sorted(faults[i]),metrics=metrics.get(i,{}))
                           for i in sorted(cases) if faults[i]][:24])
