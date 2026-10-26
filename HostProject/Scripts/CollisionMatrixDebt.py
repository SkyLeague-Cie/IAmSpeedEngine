"""Finite human admission of the frozen 2026-10-26 matrix faults.

Integration assessment only: raw artifacts, native errors and strict validator
are never rewritten. Source, spec, case/category and diagnostic frames are bound
to the observed candidate. New faults and broken evidence remain blocking.
"""
from collections import Counter
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import struct

_spec = importlib.util.spec_from_file_location('MatrixDebtGate', Path(__file__).with_name('CollisionMatrix042.py'))
gate = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gate)
SOURCE_TREE = '9f5b085e66c7435d260556b26856a7d22227fcfe'
FROZEN_COMMIT = '4beb89bb1c58ecb7c65b9fc1e2d9ba77efe6b47a'
ADMITTED_SOURCE_TREES = (SOURCE_TREE, '9f0ff5635dc6c9a75d076b833b9ecbf5a84286aa')
POLICY_SHA256 = '7293e53b4f44a5d46414ba9fb478570d8177c01daffa3df5e49210dd5bc539be'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_tree(directory):
    """Git tree identity of ordinary 100644 Source files, without requiring Git."""
    root = Path(directory)
    gate.require(root.is_dir() and not root.is_symlink(), 'Missing or linked source root')
    entries = []
    for path in root.iterdir():
        gate.require(not path.is_symlink(), 'Linked source is not frozen evidence')
        if path.is_dir():
            mode, digest, sort_name = '40000', source_tree(path), path.name+'/'
        else:
            gate.require(path.is_file(), 'Nonregular source entry')
            raw = path.read_bytes()
            mode = '100644'
            digest = hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()
            sort_name = path.name
        entries.append((sort_name.encode(), mode, path.name, digest))
    raw = b''.join(mode.encode()+b' '+name.encode()+b'\0'+bytes.fromhex(digest)
                   for _,mode,name,digest in sorted(entries))
    return hashlib.sha1(b'tree '+str(len(raw)).encode()+b'\0'+raw).hexdigest()


def load_policy():
    path = Path(__file__).with_name('accepted-collision-matrix042-debt.json')
    gate.require(sha256(path) == POLICY_SHA256, 'Matrix debt policy identity changed')
    policy = json.loads(path.read_text())
    gate.require(policy['schema'] == 'iamspeed.collision-matrix042-debt/v1'
                 and policy['frozen_commit'] == FROZEN_COMMIT
                 and policy['source_tree'] == SOURCE_TREE
                 and policy['admitted_source_trees'] == list(ADMITTED_SOURCE_TREES)
                 and policy['native_identity'] == gate.IDENTITY
                 and len(policy['cases']) == 1880
                 and policy['preserve_raw_failures'] is True,
                 'Matrix debt scope differs from human authorization')
    return policy


def case_faults(root):
    diagnostics = gate.diagnose_artifacts(root, include_all_fault_cases=True)
    faults = {str(r['case']):set(r['faults']) for r in diagnostics['all_fault_cases']}
    cases = gate.by_case(Path(root)/'cases.csv')
    for ident,row in cases.items():
        for flag in ('frequency_pass','translation_pass','oracle_pass'):
            if gate.integer(row,flag) == 0:
                faults.setdefault(str(ident),set()).add('native.'+flag)
    return cases, {i:sorted(tags) for i,tags in faults.items()}, diagnostics


def validate_accepted_matrix(directory, plugin_source):
    root = Path(directory)
    policy = load_policy()
    observed_source = source_tree(plugin_source)
    gate.require(observed_source in ADMITTED_SOURCE_TREES, 'Matrix debt belongs to another Source tree')
    gate.require(sha256(root/'spec.json') == policy['spec_sha256'], 'Frozen matrix specification changed')
    evidence = gate.validate_artifacts(root, _evidence_only=True)
    cases, faults, diagnostics = case_faults(root)
    unexpected = {i:sorted(set(tags)-set(policy['cases'].get(i,[])))
                  for i,tags in faults.items() if set(tags)-set(policy['cases'].get(i,[]))}
    gate.require(not unexpected, 'Unadmitted matrix case/category faults: '+str(unexpected))
    summary = json.loads((root/'summary.json').read_text(encoding='utf-8-sig'))
    # Reconcile every native aggregate with its actual rows, including debts.
    expected = {name:sum(gate.integer(r,flag)==0 for r in cases.values()) for name,flag in
                (('frequency_failures','frequency_pass'),('translation_failures','translation_pass'),('oracle_failures','oracle_pass'))}
    expected.update(failed=sum(any(gate.integer(r,f)==0 for f in ('frequency_pass','translation_pass','oracle_pass')) for r in cases.values()),
        iteration_limit_cases=sum(gate.integer(r,'iteration_exhausted') for r in cases.values()),
        observer_overflow_cases=0, wheel_cases=sum(r['shape_a']=='Wheel' for r in cases.values()),
        wheel_hit_cases=sum(r['shape_a']=='Wheel' and gate.integer(r,'real_wheel_hit')==1 for r in cases.values()))
    expected['wheel_coverage'] = expected['wheel_cases'] == expected['wheel_hit_cases']
    gate.require(all(summary.get(k)==v for k,v in expected.items()), 'Native matrix aggregate mismatch')
    execution = json.loads((root/'execution.json').read_text(encoding='utf-8-sig'))
    for field,value in dict(completed_cases=4800,attempted_cases=4800,expected_frames=100200,actual_frames=100200,
            terminal_failure_cases=0,worlds_created=1,production_terminal_latches_cleared=False).items():
        gate.require(execution.get(field)==value, 'Incomplete native execution: '+field)
    generations = {gate.integer(r,'driver_generation') for r in cases.values()}
    gate.require(generations==set(range(1,execution.get('drivers_created',-1)+1)), 'Incomplete driver generation evidence')
    try:
        gate.validate_artifacts(root)
        raw = dict(state='ARTIFACT_CONTRACT_OK')
    except ValueError as exc:
        raw = dict(state='FAIL',reason=str(exc))
    return dict(state='PASS_WITH_ACCEPTED_DEBT' if faults else 'PASS',
        observed_source_tree=observed_source, frozen_source_tree=SOURCE_TREE,
        accepted_case_count=len(faults), admitted_case_limit=1880, faults=faults,
        strict_assessment=raw, evidence=evidence, raw_report_unchanged=True,
        physical_matrix_qualified=not faults, policy_sha256=POLICY_SHA256,
        diagnostics_summary=diagnostics['summary'])


def project_matrix_report(exit_code, report, assessment, directory):
    """Remove only independently checked matrix errors in an assessment copy."""
    gate.require(exit_code in (0,1), 'Matrix debt cannot admit a crash exit')
    rows = [r for r in report.get('tests',[]) if r.get('fullTestPath')==gate.IDENTITY]
    gate.require(len(rows)==1, 'Missing or duplicate matrix automation identity')
    row = rows[0]
    gate.require(row.get('state') in ('Success','Fail') and row.get('warnings')==0,
                 'Incomplete matrix automation or warning')
    gate.require(assessment['state'] in ('PASS','PASS_WITH_ACCEPTED_DEBT'), 'Missing matrix artifact assessment')
    policy = load_policy()
    cases = gate.by_case(Path(directory)/'cases.csv')
    failed = [i for i,r in sorted(cases.items()) if any(gate.integer(r,f)==0 for f in ('frequency_pass','translation_pass','oracle_pass'))]
    expected = Counter()
    for ident in failed[:20]:
        r = cases[ident]
        valid = int(all(gate.integer(r,f)==1 for f in ('complete','clock_valid','actual_delta_valid'))
                    and not any(gate.integer(r,f) for f in ('iteration_exhausted','observer_overflow','terminal_failure')))
        expected['CollisionMatrix042 case=%d %s/%s variant=%s Hz=%s offset=%s native=%d frequency=%s translation=%s oracle=%s' %
                 (ident,r['shape_a'],r['shape_b'],r['variant'],r['hz'],r['translation'],valid,r['frequency_pass'],r['translation_pass'],r['oracle_pass'])] += 1
    if failed:
        expected['all collision and invariance gates: The two values are not equal.'] += 1
    if any(r['shape_a']=='Wheel' and gate.integer(r,'real_wheel_hit')==0 for r in cases.values()):
        expected["Expected 'actual native Wheel contacts covered every Wheel case' to be true."] += 1
    actual = Counter()
    iteration_frames = set()
    for entry in row.get('entries',[]):
        event = entry.get('event',{})
        kind = event.get('type')
        gate.require(kind in ('Info','Error'), 'Unadmitted matrix event severity')
        if kind != 'Error': continue
        message = event.get('message','')
        match = re.fullmatch(r'LogTemp: \[SpeedSolverIterationLimit\] Frame=(\d+) Iterations=24 MaxIterations=24 ComponentSweeps=\d+ StaticQueries=\d+ AuthorityAttempts=\d+ ResolvedEvents=\d+ TimePassed=([\deE+.\-]+) Dt=([\deE+.\-]+) StepMs=([\deE+.\-]+)',message)
        if match:
            frame = int(match[1])
            gate.require(frame in policy['iteration_frames'] and frame not in iteration_frames,
                         'Unadmitted or duplicate iteration-limit frame')
            owner = next((r for r in cases.values() if gate.integer(r,'first_global_frame') <= frame <
                          gate.integer(r,'first_global_frame')+gate.integer(r,'frames')),None)
            gate.require(owner is not None and gate.integer(owner,'iteration_exhausted')==1,
                         'Iteration diagnostic has no corresponding exhausted case')
            for value in match.groups()[1:]: gate.number({'value':value},'value')
            dt = struct.unpack('f',struct.pack('f',1./gate.integer(owner,'hz')))[0]
            gate.require(abs(float(match[3])-dt)<1e-9 and 0<=float(match[2])<=dt+1e-9
                         and float(match[4])>=0, 'Invalid iteration diagnostic physical clock')
            iteration_frames.add(frame)
        else:
            gate.require(message in expected, 'Unadmitted matrix native error: '+message)
            actual[message] += 1
    gate.require(actual==expected, 'Missing or duplicate matrix case/aggregate errors')
    exhausted = {i for i,r in cases.items() if gate.integer(r,'iteration_exhausted')}
    diagnosed = {i for i,r in cases.items() if any(gate.integer(r,'first_global_frame')<=f<gate.integer(r,'first_global_frame')+gate.integer(r,'frames') for f in iteration_frames)}
    gate.require(exhausted==diagnosed, 'Missing iteration-limit diagnostic')
    gate.require(row.get('errors')==sum(actual.values())+len(iteration_frames), 'Matrix error counter mismatch')
    accepted = row['errors'] > 0
    gate.require((row['state']=='Fail')==accepted, 'Matrix automation state contradicts diagnostics')
    projected = copy.deepcopy(report)
    if accepted:
        target = next(r for r in projected['tests'] if r['fullTestPath']==gate.IDENTITY)
        target.update(state='Success',errors=0)
        target['entries'] = [e for e in target['entries'] if e['event']['type']=='Info']
        projected['failed'] -= 1
        projected['succeeded'] += 1
    return projected, accepted
