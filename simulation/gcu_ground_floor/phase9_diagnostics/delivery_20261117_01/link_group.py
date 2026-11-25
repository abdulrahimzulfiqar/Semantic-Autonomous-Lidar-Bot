#!/usr/bin/env python3
"""No-motion discovery probe with six independent DDS processes per host.

Reuses the existing namespaced echo probe in domain 217. This is a transport
diagnostic, not a navigation/sensor-rate benchmark. Every child has its own
session topics and retains its individual result and stderr.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role',choices=['pi','simulator'],required=True)
    parser.add_argument('--probe',type=Path,required=True)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--session',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    env={**os.environ,'ROS_DOMAIN_ID':'217','ROS_LOCALHOST_ONLY':'0',
        'RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp',
        'CYCLONEDDS_URI':'file://'+str(args.profile.resolve()),
        'ROS_LOG_DIR':str(args.output/'ros_logs'),'PYTHONDONTWRITEBYTECODE':'1'}
    children=[];logs=[];reports=[];started=time.monotonic()
    try:
        for index in range(6):
            log=(args.output/f'process_{index}.log').open('w');logs.append(log)
            child=subprocess.Popen([sys.executable,str(args.probe),'--role',args.role,
                '--session',args.session+'_'+str(index),'--output',str(args.output/f'process_{index}.json')],
                env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            children.append(child)
        for child in children:
            child.wait(timeout=max(1,40-(time.monotonic()-started)))
    finally:
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()
        for index in range(6):
            path=args.output/f'process_{index}.json'
            reports.append(json.loads(path.read_text()) if path.exists() else dict(passed=False,error='missing_probe_report'))
        result=dict(scope='Six independent ROS processes per host; no motion',role=args.role,
            profile_sha256=hashlib.sha256(args.profile.read_bytes()).hexdigest(),
            probe_sha256=hashlib.sha256(args.probe.read_bytes()).hexdigest(),
            source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            child_exits=[v.returncode for v in children],reports=reports,
            elapsed_wall_s=time.monotonic()-started,
            passed=len(children)==6 and all(v.returncode==0 for v in children) and all(r['passed'] for r in reports))
        (args.output/'group.json').write_text(json.dumps(result,indent=2)+'\n')
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
