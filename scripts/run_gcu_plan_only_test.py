#!/usr/bin/env python3
"""Isolated stationary GCU planner/costmap check, with lifecycle shutdown.

Publishes preview goals only. Does not start Nav2 controllers or execution nodes.
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
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,required=True);parser.add_argument('--robot-workspace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--domain',type=int,default=79)
    args=parser.parse_args();p=args.workspace.resolve();args.output.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy()
    if not env.get('DISPLAY'):
        for proc in Path('/proc').iterdir():
            try:
                if not proc.name.isdigit() or proc.stat().st_uid!=os.getuid():continue
                e=dict(x.split('=',1) for x in (proc/'environ').read_text().split('\0') if '=' in x)
                if 'DISPLAY' in e and 'XAUTHORITY' in e:env.update({k:e[k] for k in ['DISPLAY','XAUTHORITY','XDG_RUNTIME_DIR'] if k in e});break
            except (OSError,ValueError):pass
    env.update(ROS_DOMAIN_ID=str(args.domain),IGN_PARTITION='gcu_plan_only_'+str(os.getpid()),LIBGL_ALWAYS_SOFTWARE='1',LIBGL_DRI3_DISABLE='1')
    setup='source /opt/ros/humble/setup.bash; source '+shlex.quote(str(args.robot_workspace/'install/setup.bash'))+'; exec '
    commands=[('simulator',['ros2','launch','lidarbot_gz','gz_launch.py','world:='+str(p/'assets/gcu_ground_floor_ogre.sdf'),
        'urdf_path:='+str(p/'robot_profile/gcu_evaluation.urdf.xacro'),'x_pose:=-0.05864395131342248','y_pose:=-12.960313240266368',
        'z_pose:=0.15','yaw_pose:=0','use_robot_localization:=false']),
        ('amcl',['ros2','launch',str(p/'gcu_amcl.launch.py'),'map:='+str(p/'assets/gcu_ground_floor.yaml'),'params:='+str(p/'assets/amcl.yaml')]),
        ('truth',['ros2','run','ros_gz_bridge','parameter_bridge','/model/lidarbot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V','--ros-args','-r','/model/lidarbot/pose:=/gcu_ground_truth']),
        ('planning',['ros2','launch',str(p/'gcu_plan_only.launch.py'),'params:='+str(p/'assets/nav2_plan_only.yaml')])]
    processes=[];logs=[];planning_started=False;status={'domain':args.domain,'partition':env['IGN_PARTITION']}
    try:
        for name,cmd in commands:
            log=(args.output/(name+'.log')).open('w');logs.append(log)
            processes.append(subprocess.Popen(['/bin/bash','-c',setup+shlex.join(cmd)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            if name=='planning':planning_started=True
            if name=='simulator':time.sleep(6)
            if name=='amcl':
                with (args.output/'localization_readiness.log').open('w') as log:
                    ready=subprocess.run(['/bin/bash','-c',setup+shlex.join(['python3',str(p/'wait_gcu_localization.py')])],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=50)
                if ready.returncode!=0:raise RuntimeError('Localization readiness gate failed before planner startup')
        time.sleep(3)
        cmd=['python3',str(p/'check_gcu_plan_only.py'),'--output',str(args.output/'report.json')]
        with (args.output/'check.log').open('w') as log:
            result=subprocess.run(['/bin/bash','-c',setup+shlex.join(cmd)],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=150)
        status['checker_exit']=result.returncode
        if (args.output/'report.json').exists():status['passed']=result.returncode==0 and json.loads((args.output/'report.json').read_text()).get('passed',False)
    except Exception as error:status.update(passed=False,error=str(error))
    finally:
        try:
            if not planning_started:raise RuntimeError('Planner not started; no lifecycle shutdown needed')
            cmd=['ros2','service','call','/gcu_planning_lifecycle/manage_nodes','nav2_msgs/srv/ManageLifecycleNodes','{command: 4}']
            with (args.output/'lifecycle_shutdown.log').open('w') as log:
                result=subprocess.run(['/bin/bash','-c',setup+shlex.join(cmd)],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=12)
            status['lifecycle_shutdown_cli_exit']=result.returncode
        except Exception as error:status['lifecycle_shutdown_error']=str(error)
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try:proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        status['launch_exits']=[proc.returncode for proc in processes]
        for log in logs:log.close()
        (args.output/'run.json').write_text(json.dumps(status,indent=2)+'\n')
        print(json.dumps(status,indent=2),flush=True)
    return 0 if status.get('passed',False) and all(x==0 for x in status['launch_exits']) else 1


if __name__=='__main__':raise SystemExit(main())
