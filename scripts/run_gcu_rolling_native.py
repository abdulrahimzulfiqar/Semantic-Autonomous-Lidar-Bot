#!/usr/bin/env python3
"""Bounded native plan-only rolling-window test. No controller/physics launched."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from gcu_process_audit import audit_logs


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();w=a.workspace.resolve()
    for proc in Path('/proc').glob('[0-9]*'):
        if int(proc.name)==os.getpid():continue
        try:
            if b'ROS_DOMAIN_ID=197' in (proc/'environ').read_bytes().split(b'\0'):raise RuntimeError('domain_197_in_use')
        except OSError:pass
    a.output.mkdir(parents=True,exist_ok=False);out=a.output.resolve()
    manifest=json.loads((w/'manifest.json').read_text())
    for name,want in manifest['files_sha256'].items():
        if hashlib.sha256((w/name).read_bytes()).hexdigest()!=want:raise RuntimeError('source_mismatch:'+name)
    env={**os.environ,'ROS_DOMAIN_ID':'197','ROS_LOCALHOST_ONLY':'1','RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp',
         'ROS_LOG_DIR':str(out/'ros_logs'),'PYTHONDONTWRITEBYTECODE':'1'}
    opts=['--ros-args','--params-file',str(w/'params.yaml')]
    commands=[('checker',[sys.executable,str(w/'check_gcu_rolling_native.py'),'--workspace',str(w),'--output',str(out/'report.json')]),
        ('planner',['/opt/ros/humble/lib/nav2_planner/planner_server',*opts]),
        ('local',['/opt/ros/humble/lib/nav2_planner/planner_server',*opts,'-r','__node:=local_planner','-r','__ns:=/window_local'])]
    children=[];logs=[];result={'passed':False,'scope':'Native GCU plan-only window shifts; no motion','domain':197};begin=time.monotonic()
    try:
        for name,cmd in commands:
            log=(out/(name+'.log')).open('w');logs.append(log)
            children.append(subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        result['checker_exit']=children[0].wait(timeout=180)
        result['passed']=result['checker_exit']==0 and json.loads((out/'report.json').read_text())['passed']
    except Exception as error:result['error']=repr(error)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()
        result.update(child_exits=[c.returncode for c in children],child_audit=audit_logs(out),elapsed_wall_s=time.monotonic()-begin)
        result['inputs_unchanged']=all(hashlib.sha256((w/n).read_bytes()).hexdigest()==h for n,h in manifest['files_sha256'].items())
        result['passed']=bool(result['passed'] and result['child_exits']==[0,0,0] and not result['child_audit']['critical'] and result['inputs_unchanged'])
        (out/'run.json').write_text(json.dumps(result,indent=2)+'\n')
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
