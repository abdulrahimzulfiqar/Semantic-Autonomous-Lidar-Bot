#!/usr/bin/env python3
"""Run ONE bounded, isolated 06B1 ROS mock session. No robot/physics launch."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
from gcu_process_audit import audit_logs


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--domain',type=int,default=186);p.add_argument('--wall-timeout',type=float,default=180.)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);w=a.workspace.resolve()
    if not 1<=a.domain<=230 or not 0<a.wall_timeout<=1200:raise ValueError('invalid isolated test budget/domain')
    # Refuse a domain already used by another process, without killing it.
    for proc in Path('/proc').glob('[0-9]*'):
        if int(proc.name)==os.getpid():continue
        try:
            values=(proc/'environ').read_bytes().split(b'\0')
            if ('ROS_DOMAIN_ID='+str(a.domain)).encode() in values:raise RuntimeError('ROS domain already in use')
        except (FileNotFoundError,PermissionError,ProcessLookupError):pass
    token=uuid.uuid4().hex;prefix='/gcu_phase6b1_'+token[:10]
    env={**os.environ,'ROS_DOMAIN_ID':str(a.domain),'ROS_LOCALHOST_ONLY':'1',
         'RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp','ROS_LOG_DIR':str(a.output.resolve()/'ros_logs'),
         'PYTHONDONTWRITEBYTECODE':'1'}
    frozen={str(f.relative_to(w)):hashlib.sha256(f.read_bytes()).hexdigest()
            for f in sorted(w.rglob('*')) if f.is_file() and 'results' not in f.relative_to(w).parts and '__pycache__' not in f.parts}
    common=['--namespace',prefix,'--mock-token',token]
    commands=[('fixture',[sys.executable,str(w/'check_gcu_mission_ros.py'),*common,'--output',str(a.output.resolve()/'report.json')]),
              ('adapter',[sys.executable,str(w/'gcu_mission_mock_adapter.py'),*common,'--assets',str(w/'assets')])]
    children=[];logs=[];result={'passed':False,'domain':a.domain,'namespace':prefix,'source_sha256':frozen,
        'scope':'06B1 ROS mock only, no Gazebo, robot controller or real NavigateToPose'}
    begin=time.monotonic()
    try:
        for name,cmd in commands:
            log=(a.output/(name+'.log')).open('w');logs.append(log)
            children.append(subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        result['checker_exit']=children[0].wait(timeout=a.wall_timeout)
        result['passed']=result['checker_exit']==0 and json.loads((a.output/'report.json').read_text())['passed']
    except Exception as error:result['error']=repr(error)
    finally:
        for proc in reversed(children):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for log in logs:log.close()
        result['child_exits']=[c.returncode for c in children]
        result['child_audit']=audit_logs(a.output)
        result['inputs_unchanged']=all((w/name).is_file() and hashlib.sha256((w/name).read_bytes()).hexdigest()==h for name,h in frozen.items())
        result['elapsed_wall_s']=time.monotonic()-begin
        result['passed']=bool(result['passed'] and result['inputs_unchanged'] and
                              result['child_exits']==[0,0] and not result['child_audit']['critical'])
        (a.output/'run.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
