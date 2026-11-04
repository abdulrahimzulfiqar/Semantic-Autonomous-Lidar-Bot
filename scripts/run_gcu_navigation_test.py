#!/usr/bin/env python3
"""Fresh isolated headless autonomous trial. Does not touch real robot workspaces.

Requires --execute to start the simulated navigation controller. Positive-control
mode starts physics/contact sensing only. Results directories must be new.
"""
import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',type=Path,required=True)
    p.add_argument('--robot-workspace',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--via-click',action='store_true');p.add_argument('--execute',action='store_true');p.add_argument('--positive-control',action='store_true')
    p.add_argument('--follow-up-goal',type=float,nargs=3);p.add_argument('--goal',type=float,nargs=3,default=[1.4413560486865775,-12.960313240266368,0.])
    p.add_argument('--expected',choices=['success','rejected','cancelled'],default='success')
    p.add_argument('--sim-timeout',type=float,default=80.);p.add_argument('--wall-timeout',type=float,default=400.)
    p.add_argument('--score-map',type=Path);p.add_argument('--contact-assets',type=Path);p.add_argument('--world',type=Path);p.add_argument('--map',type=Path)
    p.add_argument('--domain',type=int,default=79);a=p.parse_args()
    if not (a.execute or a.positive_control):p.error('--execute required for simulated controller startup')
    w=a.workspace.resolve();a.output.mkdir(parents=True,exist_ok=False)
    assets=a.contact_assets or w/'assets/contact_v2'
    env=os.environ.copy()
    if not env.get('DISPLAY'):
        for proc in Path('/proc').iterdir():
            try:
                if not proc.name.isdigit() or proc.stat().st_uid!=os.getuid():continue
                e=dict(x.split('=',1) for x in (proc/'environ').read_text().split('\0') if '=' in x)
                if 'DISPLAY' in e and 'XAUTHORITY' in e:
                    env.update({k:e[k] for k in ['DISPLAY','XAUTHORITY','XDG_RUNTIME_DIR'] if k in e});break
            except (OSError,ValueError):pass
    env.update(RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',ROS_DOMAIN_ID=str(a.domain),IGN_PARTITION='gcu_navigation_'+str(os.getpid()),LIBGL_ALWAYS_SOFTWARE='1',LIBGL_DRI3_DISABLE='1')
    setup='source /opt/ros/humble/setup.bash; source '+shlex.quote(str(a.robot_workspace/'install/setup.bash'))+'; exec '
    commands=[('simulator',['ros2','launch',str(w/'gcu_contact_sim.launch.py'),'assets:='+str(assets),
        'world:='+str(a.world or assets/('positive_control.sdf' if a.positive_control else 'world.sdf'))])]
    if not a.positive_control:
        commands += [('amcl',['ros2','launch',str(w/'gcu_amcl.launch.py'),'map:='+str(a.map or w/'assets/gcu_ground_floor.yaml'),'params:='+str(w/'assets/amcl.yaml')]),
            ('navigation',['ros2','launch',str(w/'gcu_navigation.launch.py'),'params:='+str(w/'assets/nav2_autonomous.yaml'),'tree:='+str(w/'assets/navigate_strict.xml'),'through_tree:='+str(w/'assets/navigate_through_strict.xml')])]
    processes=[];logs=[];status={'domain':a.domain,'partition':env['IGN_PARTITION'],'rmw_implementation':env['RMW_IMPLEMENTATION'],'commands':commands};navstarted=False
    try:
        for name,cmd in commands:
            log=(a.output/(name+'.log')).open('w');logs.append(log)
            processes.append(subprocess.Popen(['/bin/bash','-c',setup+shlex.join(cmd)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            if name=='simulator':time.sleep(6)
            if name=='navigation':navstarted=True
            if name=='amcl':
                with (a.output/'localization_readiness.log').open('w') as log:
                    r=subprocess.run(['/bin/bash','-c',setup+shlex.join(['python3',str(w/'wait_gcu_localization.py'),'--request-updates'])],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=50)
                if r.returncode:raise RuntimeError('localization readiness failed')
        cmd=['python3',str(w/'check_gcu_navigation.py'),'--assets',str(assets),'--output',str(a.output/'report.json'),
            '--sim-timeout',str(a.sim_timeout),'--wall-timeout',str(a.wall_timeout),'--goal',*map(str,a.goal),'--expected',a.expected,
            '--positive-control' if a.positive_control else '--execute']
        if a.via_click:cmd += ['--via-click']
        if a.score_map:cmd += ['--score-map',str(a.score_map)]
        with (a.output/'check.log').open('w') as log:
            r=subprocess.run(['/bin/bash','-c',setup+shlex.join(cmd)],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=a.wall_timeout+15)
        status['checker_exit']=r.returncode
        status['passed']=r.returncode==0 and json.loads((a.output/'report.json').read_text()).get('passed',False)
        if status['passed'] and a.follow_up_goal:
            second=list(cmd);i=second.index('--goal');second[i+1:i+4]=list(map(str,a.follow_up_goal));i=second.index('--output');second[i+1]=str(a.output/'followup_report.json')
            with (a.output/'followup_check.log').open('w') as log:
                extra=subprocess.run(['/bin/bash','-c',setup+shlex.join(second)],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=a.wall_timeout+15)
            status['followup_exit']=extra.returncode
            status['passed']=extra.returncode==0 and json.loads((a.output/'followup_report.json').read_text()).get('passed',False)
    except Exception as e:status.update(passed=False,error=str(e))
    finally:
        if navstarted:
            try:
                cmd=['ros2','service','call','/gcu_navigation_lifecycle/manage_nodes','nav2_msgs/srv/ManageLifecycleNodes','{command: 4}']
                with (a.output/'shutdown.log').open('w') as log:
                    r=subprocess.run(['/bin/bash','-c',setup+shlex.join(cmd)],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=15)
                status['lifecycle_shutdown_exit']=r.returncode
            except Exception as e:status['shutdown_error']=str(e)
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try:proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        status['launch_exits']=[v.returncode for v in processes]
        for log in logs:log.close()
        (a.output/'run.json').write_text(json.dumps(status,indent=2)+'\n');print(json.dumps(status,indent=2),flush=True)
    return 0 if status.get('passed') and all(x==0 for x in status['launch_exits']) else 1


if __name__=='__main__':raise SystemExit(main())
