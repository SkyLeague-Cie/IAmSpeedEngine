"""Opt-in assessment of a named observer-cadence debt; never rewrites raw evidence."""
import copy
import datetime as dt
from decimal import Decimal, InvalidOperation
import hashlib
from pathlib import Path, PurePosixPath
import re

EIGHT = 'IAmSpeed.Simulation.FrameParallel.NativeAnalyticEightCars1552'
GROUND = 'IAmSpeed.Simulation.FrameParallel.NativeAnalyticGrounded1368'
FAMILIES = {
    EIGHT: ['QualificationEightCars1534'],
    GROUND: ['Qualification1368', 'QualificationLow16_1403', 'QualificationLow8_1403',
             'QualificationIdle50Cells64_1403', 'QualificationIdle50Cells16_1403',
             'QualificationIdle50Cells8_1403'],
}
LANES = [1, 2, 4, 8, 12]
METRICS = ['ActualStartIntervalMs', 'TargetStartIntervalMs', 'BaseStartIntervalMs',
           'PhaseErrorMs', 'RecoveryDebtMs']
MESSAGE = re.compile(r'SpeedPhysicsLog: \[RealTimeSimulationCadence\] Physics Thread is too slow: Frame=(\d+) '
                     + ' '.join(name + r'=([^ ]+)' for name in METRICS))
EVIDENCE = re.compile(r'\[NativeAnalyticContacts1368Evidence\] Path=(\S+) Saved=([01])')
DEADLINE = dt.datetime(2026, 10, 24, 20, 30, tzinfo=dt.timezone.utc)


def check_policy(policy, now=None):
    if (policy.get('schema') != 'iamspeed.additive-cadence-debt/v1'
            or policy.get('tests') != FAMILIES or policy.get('normal_lanes') != LANES
            or policy.get('frames') != 90 or policy.get('hz') != 300
            or policy.get('preserve_raw_evidence') is not True
            or policy.get('preserve_existing_v1_debt') is not True):
        raise ValueError('Cadence policy scope differs from the two authorized fixture identities')
    activation = policy.get('activation', {})
    if activation.get('enabled') is False:
        return False
    if activation.get('enabled') is not True:
        raise ValueError('Cadence debt activation must be explicit')
    if not re.fullmatch(r'[0-9a-f]{64}', activation.get('decision_sha256', '')):
        raise ValueError('Cadence debt requires a versioned decision receipt hash')
    try:
        at = dt.datetime.fromisoformat(activation['at_utc'].replace('Z', '+00:00'))
        if at.utcoffset() != dt.timedelta(0):
            raise ValueError('Activation must be UTC')
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('Invalid cadence activation timestamp') from exc
    if activation.get('trigger') not in ('CI_GREEN', 'HUMAN_DEADLINE_2026_10_24'):
        raise ValueError('Unknown cadence activation trigger')
    if activation['trigger'] == 'HUMAN_DEADLINE_2026_10_24' and at < DEADLINE:
        raise ValueError('Human deadline has not been reached')
    if (now or dt.datetime.now(dt.timezone.utc)) < at:
        raise ValueError('Cadence activation is in the future')
    return True


def cohort_proof(evidence_root, test, native_path, saved):
    # The native output root is /work; host retrospective assessment remaps only
    # the exact relative path, never an arbitrary path supplied by a log line.
    parts = PurePosixPath(native_path).parts
    if (saved != '1' or len(parts) != 8 or parts[:5] != ('/', 'work', 'user', 'Saved', 'IAmSpeed')
            or parts[5] not in FAMILIES[test] or not re.fullmatch(r'[0-9A-Fa-f]{32}', parts[6])):
        raise ValueError('Untrusted or foreign cadence cohort path')
    match = re.fullmatch(r'Fast0-W(1|2|4|8|12)', parts[-1])
    if not match:
        raise ValueError('Cadence debt admits only the explicit Normal lane cohorts')
    root = Path(evidence_root).resolve(strict=True)
    rows = root.joinpath(*parts[2:], 'rows.tsv')
    for parent in (rows, *rows.parents):
        if parent == root.parent:
            break
        if parent.is_symlink():
            raise ValueError('Aliased cadence evidence path')
    actual = rows.resolve(strict=True)
    if not actual.is_relative_to(root) or not actual.is_file() or actual.stat().st_size > 64 * 1024:
        raise ValueError('Cadence row evidence missing, outside root, or oversized')
    raw = actual.read_bytes()
    lines = raw.decode('utf-8-sig').splitlines()
    header = re.fullmatch(r'Fast=0 Requested=(\d+) Host=2 Hz=300 Driver=RealTimeSimulation Finished=1 Failed=0 FailureLine=0 Steps=90 Scope=IsolatedNativeFixtureFullStadiumUNKNOWN', lines[0]) if lines else None
    if not header or header[1] != match[1] or len(lines) != 92:
        raise ValueError('Cadence cohort is incomplete, non-Normal or ambiguous')
    fields = lines[1].split('\t')
    if len(fields) != len(set(fields)) or not {'N', 'Frame', 'Publication', 'Complete'}.issubset(fields):
        raise ValueError('Malformed cadence row fields')
    for index, line in enumerate(lines[2:]):
        values = line.split('\t')
        row = dict(zip(fields, values))
        if len(values) != len(fields) or [row[key] for key in ('N', 'Frame', 'Publication', 'Complete')] != [str(index), str(index), str(index + 1), '1']:
            raise ValueError('Incomplete or inconsistent cadence frame rows')
    return dict(native_path=native_path, relative_path=actual.relative_to(root).as_posix(),
                sha256=hashlib.sha256(raw).hexdigest(), requested_lanes=int(match[1]),
                family=parts[5], normal=True, frames=90, hz=300, rows_header=lines[0])


def project_cadence(report, evidence_root):
    """Make a private assessment copy and an exhaustive list of accepted events.

    All other validation, including the original two Fail debts, stays in the
    existing validator. A warning is tied to the next synchronous Saved1 cohort
    export in the SAME test; its actual complete90-frame rows prove mode/lanes.
    """
    assessed = copy.deepcopy(report)
    accepted = []
    warned_rows = 0
    paths = [row.get('fullTestPath') for row in report.get('tests', [])]
    if len(paths) != len(set(paths)):
        raise ValueError('Ambiguous cadence test identity')
    for row in assessed.get('tests', []):
        test = row.get('fullTestPath')
        warnings = row.get('warnings')
        if type(warnings) is not int or warnings < 0:
            raise ValueError('Missing or invalid warning counter')
        entries = row.get('entries', [])
        actual = [i for i, entry in enumerate(entries) if entry.get('event', {}).get('type') == 'Warning']
        if warnings != len(actual):
            raise ValueError('Raw warning count differs from preserved events')
        if not actual:
            continue
        if test not in FAMILIES or row.get('state') != 'Success' or row.get('errors') != 0:
            raise ValueError('Non-cadence warning, failed or incomplete cadence test')
        if any(entry.get('event', {}).get('type') in ('Error', 'Fatal') for entry in entries):
            raise ValueError('New error in cadence fixture')
        pending, matched, seen_paths, frames, scopes = [], set(), set(), {}, set()
        for i, entry in enumerate(entries):
            event = entry.get('event', {})
            message = event.get('message', '')
            if event.get('type') == 'Warning':
                match = MESSAGE.fullmatch(message)
                if not match or not 1 <= int(match[1]) <= 90:
                    raise ValueError('Only exact too-slow cadence warnings are additive debt')
                try:
                    numbers = [Decimal(x) for x in match.groups()[1:]]
                except InvalidOperation as exc:
                    raise ValueError('Malformed cadence metric') from exc
                if any(not value.is_finite() or value < 0 for value in numbers) or not numbers[0] > numbers[1] or numbers[2] <= 0:
                    raise ValueError('Invalid too-slow cadence metrics')
                pending.append((i, int(match[1]), dict(zip(METRICS, match.groups()[1:]))))
            evidence = EVIDENCE.fullmatch(message)
            if evidence:
                path, saved = evidence.groups()
                if path in seen_paths:
                    raise ValueError('Ambiguous repeated cadence cohort identity')
                seen_paths.add(path)
                if pending:
                    proof = cohort_proof(evidence_root, test, path, saved)
                    scope = (proof['family'], proof['requested_lanes'])
                    if scope in scopes:
                        raise ValueError('More than one source cohort claims the same scene/lane identity')
                    scopes.add(scope)
                    cohort_frames = frames.setdefault(path, set())
                    for warning_index, frame, values in pending:
                        if frame in cohort_frames:
                            raise ValueError('Duplicate cadence event for the same cohort/frame')
                        cohort_frames.add(frame)
                        accepted.append(dict(test=test, entry_index=warning_index, frame=frame,
                                             raw_event=copy.deepcopy(entries[warning_index]['event']),
                                             metrics=values, cohort=proof))
                        matched.add(warning_index)
                    pending.clear()
        if pending or len(matched) != warnings:
            raise ValueError('Cadence warnings lack unique completed cohort evidence')
        # Six source scenes for1368, one for1552; all retain five Normal lanes.
        if len(frames) > len(FAMILIES[test]) * len(LANES):
            raise ValueError('Cadence cohort cardinality exceeds the source fixture')
        row['entries'] = [entry for i, entry in enumerate(entries) if i not in matched]
        row['warnings'] = 0
        warned_rows += 1
    if report.get('succeededWithWarnings') != warned_rows:
        raise ValueError('Cadence warning aggregate mismatch')
    assessed['succeededWithWarnings'] = 0
    assessed['succeeded'] = report.get('succeeded', -1) + warned_rows
    return assessed, accepted
