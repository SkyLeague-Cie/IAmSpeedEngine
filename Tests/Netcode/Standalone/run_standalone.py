"""Build the private ordinary Netcode suites against THIS repository's inputs.

No Engine/UBT/Editor or socket peers are launched. World/mechanics/gameplay
providers in these suites are synthetic. This never qualifies native restore.
"""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import os
import re
import subprocess
import time

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--vc',type=Path,default=Path('C:/Program Files/Microsoft Visual Studio/18/Community/VC/Tools/MSVC/14.38.33130'))
    parser.add_argument('--windows-kits',type=Path,default=Path('C:/Program Files (x86)/Windows Kits/10'))
    parser.add_argument('--sdk-version',default='10.0.22621.0')
    args=parser.parse_args()
    tests=Path(__file__).resolve().parent
    repo=tests.parents[2]
    source=repo/'Source'
    assert source.is_dir()
    out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy();env['VSLANG']='1033'
    env['PATH']=str(args.vc/'bin/Hostx64/x64')+os.pathsep+env['PATH']
    env['INCLUDE']=os.pathsep.join(str(p) for p in [args.vc/'include']+[args.windows_kits/'Include'/args.sdk_version/s for s in ('ucrt','shared','um')])
    env['LIB']=os.pathsep.join(str(p) for p in [args.vc/'lib/x64']+[args.windows_kits/'Lib'/args.sdk_version/s/'x64' for s in ('ucrt','um')])
    cl=args.vc/'bin/Hostx64/x64/cl.exe';link=args.vc/'bin/Hostx64/x64/link.exe'
    pins={};operations=[];results={};start=time.monotonic()
    def sha(b):return hashlib.sha256(b).hexdigest()
    def pin(p):
        p=p.resolve();b=p.read_bytes();pins[str(p)]=dict(path=str(p),bytes=len(b),sha256=sha(b))
    for root in (source,repo/'Tests/Netcode'):
        for p in sorted(root.rglob('*')):
            if p.is_file():pin(p)
    pin(cl);pin(link)
    include=[source]+sorted(p for p in (source/'IAmSpeed/Netcode').iterdir() if p.is_dir())+sorted(p for p in tests.iterdir() if p.is_dir())
    flags=['/nologo','/std:c++17','/EHsc','/W4','/WX','/Od','/MT','/showIncludes']+['/I'+str(p) for p in include]
    def execute(label,argv,expected=0):
        t=time.monotonic()
        try:
            p=subprocess.run([str(a) for a in argv],cwd=repo,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=60)
            raw=p.stdout;code=p.returncode
        except subprocess.TimeoutExpired as e:raw=e.stdout or b'';code='TIMEOUT'
        (out/(label+'.raw.txt')).write_bytes(raw)
        operations.append(dict(label=label,command=[str(a) for a in argv],actual_exit=code,expected_exit=expected,elapsed_seconds=time.monotonic()-t))
        (out/'operations.json').write_text(json.dumps(operations,indent=2)+'\n')
        print(json.dumps(dict(label=label,actual_exit=code,expected_exit=expected)),flush=True)
        if label.endswith('-run'):
            results[label]=raw.decode(errors='replace').strip();print(results[label],flush=True)
        return code==expected
    ok=execute('presentation-compile',[cl]+flags+['/c','/Fo:'+str(out/'p.obj'),source/'IAmSpeed/Input/InputPresentationScope.cpp'])
    suites=[('input',tests/'transport/TransportTests.cpp'),('state',tests/'transport/StateTransportTests.cpp'),
            ('consumed',tests/'consumed-command-v2/ConsumedTests.cpp'),('authority',tests/'authoritative-owner-v2/AuthoritativeOwnerTests.cpp'),
            ('envelope',tests/'state-authority-envelope-v2/StateEnvelopeTests.cpp'),('remote',tests/'remote-input/ExactRemoteInputProducerTests.cpp'),
            ('registry',repo/'Tests/Netcode/EmptyInputFrameTests.cpp')]
    for n,(name,cpp) in enumerate(suites):
        if not ok:break
        obj=out/(str(n)+'.obj');exe=out/(name+'.exe')
        ok=execute(name+'-compile',[cl]+flags+['/c','/Fo:'+str(obj),cpp])
        if ok:ok=execute(name+'-link',[link,'/nologo','/out:'+str(exe),obj,out/'p.obj'])
        if ok:ok=execute(name+'-run',[exe]+([out] if name=='registry' else []))
    if ok:ok=execute('registry-lifetime',[out/'registry.exe','--boundary-reservation-live-destruction'],86)
    trace=set()
    for p in out.glob('*-compile.raw.txt'):
        for line in p.read_text(errors='replace').splitlines():
            match=re.search(r'including file:\s*(.*)',line)
            if match:
                inc=Path(match.group(1).strip())
                if inc.exists():trace.add(inc.resolve())
    for p in trace:
        if str(p) not in pins:pin(p)
    unchanged=all(sha(Path(e['path']).read_bytes())==e['sha256'] for e in pins.values())
    # Every local Input implementation and protocol include must come from this
    # repository; compiler/Windows SDK headers are separately observed below.
    outside=[str(p) for p in trace if p.is_relative_to(repo.parent) and not p.is_relative_to(repo)]
    if outside or not unchanged:ok=False
    (out/'inputs.json').write_text(json.dumps(list(pins.values()),indent=2)+'\n')
    receipt=dict(schema='netcode-consolidated-repository-ordinary-tests/v1',utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        repository=str(repo),qualification_exit=0 if ok else 1,operations=operations,results=results,elapsed_seconds=time.monotonic()-start,
        inputs=str(out/'inputs.json'),inputs_unchanged=unchanged,repository_includes=sorted(str(p) for p in trace if p.is_relative_to(repo)),
        historical_artifact_includes=outside,not_run=[name for name,_ in suites if name+'-run' not in results],
        scope='Actual current rebased Input implementations and consolidated protocol headers. Synthetic world/mechanics/gameplay/ledger providers remain synthetic/default deny; no native integration/restore verdict.',
        Engine_SDK_UBT_Editor_jobs=0,socket_peer_jobs=0,full_state_v3=False,old_PASS_transferred=False,
        toolchain_headers='Compiler-discovered header hashes observed after compilation; not a frozen native toolchain admission.')
    (out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(dict(receipt=str(out/'receipt.json'),qualification_exit=receipt['qualification_exit'],repository_includes=len(receipt['repository_includes']),not_run=receipt['not_run'])))
    return receipt['qualification_exit']

if __name__=='__main__':raise SystemExit(main())
