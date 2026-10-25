"""Strict coverage/evidence gate for the native single-world collision matrix.

This validates artifacts, not physical execution provenance by itself. The CI
runner must also retain the real Editor exit, exact Automation report and source
identity. No debt policy or warning waiver applies to this new matrix.
"""
import csv
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
    return dict(state='ARTIFACT_CONTRACT_OK', cases=len(cases), frames=next_frame, worlds=1,
                native_identity=IDENTITY, highest_hz=max(RATES), physical_execution_provenance='REQUIRES_EDITOR_EXIT_AND_AUTOMATION_REPORT')
