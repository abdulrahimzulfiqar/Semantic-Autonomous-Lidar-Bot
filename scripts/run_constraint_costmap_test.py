#!/usr/bin/env python3
"""One bounded native plan-only filter session. Requires sourced Humble."""
import argparse,json,os,signal,subprocess,sys,time,hashlib
from pathlib import Path
from gcu_process_audit import audit_logs


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--domain',type=int,default=193);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);w=a.workspace.resolve()
    for proc in Path('/proc').glob('[0-9]*'):
        if int(proc.name)==os.getpid():continue
        try:
            if f'ROS_DOMAIN_ID={a.domain}'.encode() in (proc/'environ').read_bytes().split(b'\0'):raise RuntimeError('test domain already in use')
        except (OSError,ProcessLookupError):pass
    env={**os.environ,'ROS_DOMAIN_ID':str(a.domain),'ROS_LOCALHOST_ONLY':'1','RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp',
         'ROS_LOG_DIR':str(a.output.resolve()/'ros_logs'),'PYTHONDONTWRITEBYTECODE':'1'}
    frozen={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in w.iterdir() if f.is_file()}
    opts=['--ros-args','--params-file',str(w/'params.yaml')]
    commands=[('fixture',[sys.executable,str(w/'check_constraint_costmaps_ros.py'),'--output',str(a.output.resolve()/'report.json')]),
        ('planner',['/opt/ros/humble/lib/nav2_planner/planner_server',*opts]),
        ('local',['/opt/ros/humble/lib/nav2_costmap_2d/nav2_costmap_2d',*opts])]
    children=[];logs=[];result={'passed':False,'domain':a.domain,'source_sha256':frozen,'scope':'Synthetic plan-only native costmaps'};begin=time.monotonic()
    try:
        for name,cmd in commands:
            log=(a.output/(name+'.log')).open('w');logs.append(log)
            children.append(subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        result['checker_exit']=children[0].wait(timeout=180)
        result['passed']=result['checker_exit']==0 and json.loads((a.output/'report.json').read_text())['passed']
    except Exception as error:result['error']=repr(error)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()
        result['child_exits']=[c.returncode for c in children];result['child_audit']=audit_logs(a.output)
        result['inputs_unchanged']=all(hashlib.sha256((w/name).read_bytes()).hexdigest()==h for name,h in frozen.items())
        result['elapsed_wall_s']=time.monotonic()-begin
        result['passed']=bool(result['passed'] and result['inputs_unchanged'] and result['child_exits']==[0,0,0] and not result['child_audit']['critical'])
        (a.output/'run.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
