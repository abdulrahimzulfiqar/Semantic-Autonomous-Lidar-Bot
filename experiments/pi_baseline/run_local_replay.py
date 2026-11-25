#!/usr/bin/env python3
"""Deploy and collect one bounded no-command Pi-local localization replay."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import tarfile
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['stage','output','remote-root','key']:p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--host',required=True);p.add_argument('--rate',type=float,choices=[1.,2.],default=1.)
    p.add_argument('--full-stack',action='store_true')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    ssh=['ssh','-i',str(a.key),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=8',a.host]
    archive=a.output/'sources.tar.gz'
    with tarfile.open(archive,'w:gz') as t:
        for path in a.stage.rglob('*'):
            if path.is_file():t.add(path,arcname=str(path.relative_to(a.stage)))
    subprocess.run(ssh+['mkdir '+shlex.quote(str(a.remote_root))],check=True,timeout=20)
    with archive.open('rb') as source:
        subprocess.run(ssh+['tar -xzf - -C '+shlex.quote(str(a.remote_root))],stdin=source,check=True,timeout=30)
    result=dict(passed=False,scope='Local AMCL timing diagnostic; no robot commands or full-stack performance claim')
    began=time.monotonic()
    script = 'run_role.py' if a.full_stack else 'local_replay.py'
    parameters = ['--role','pi'] if a.full_stack else ['--output',str(a.remote_root/'results'),'--rate',str(a.rate)]
    if a.full_stack:
        result['scope'] = 'Pi-local full-stack replay with stationary input and isolated command sink'
    command='source /opt/ros/humble/setup.bash; exec '+shlex.join(['python3',str(a.remote_root/script),
        '--workspace',str(a.remote_root),*parameters])
    try:
        run=subprocess.run(ssh+['bash -lc '+shlex.quote(command)],capture_output=True,text=True,timeout=370 if a.full_stack else 140)
        (a.output/'console.log').write_text(run.stdout+run.stderr);result['exit']=run.returncode
    except BaseException as error:
        result['error']=repr(error)
        pid_file = a.remote_root/'role.pid' if a.full_stack else a.remote_root/'results/owner.pid'
        stop='import os,signal;from pathlib import Path;f=Path('+repr(str(pid_file))+');'+\
            'p=int(f.read_text()) if f.exists() else 0;c=Path("/proc/"+str(p)+"/cmdline");'+\
            'os.kill(p,signal.SIGINT) if p and c.exists() and '+repr(script.encode())+' in c.read_bytes() and '+repr(str(a.remote_root).encode())+' in c.read_bytes() else None'
        subprocess.run(ssh+['python3 -c '+shlex.quote(stop)],capture_output=True,timeout=20)
        time.sleep(3)
    finally:
        try:
            with (a.output/'results.tar.gz').open('wb') as target:
                subprocess.run(ssh+['tar --exclude=latest -czf - -C '+shlex.quote(str(a.remote_root))+' results'],stdout=target,check=True,timeout=40)
            with tarfile.open(a.output/'results.tar.gz') as t:t.extractall(a.output,filter='data')
            report_name = 'role.json' if a.full_stack else 'result.json'
            result['role']=json.loads((a.output/'results'/report_name).read_text())
            result['passed']=result.get('exit')==0 and result['role']['passed']
        except Exception as error:result['collection_error']=repr(error)
        result['elapsed_wall_s']=time.monotonic()-began
        (a.output/'orchestration.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
