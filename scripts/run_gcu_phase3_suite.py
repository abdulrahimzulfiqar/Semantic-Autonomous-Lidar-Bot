#!/usr/bin/env python3
"""Run Phase 03 acceptance cases sequentially in isolated Ubuntu simulations.

Use the prepared test workspace; never modifies the existing installed workspace.
Requires --execute because the watchdog and repeated-motion cases drive a model.
"""
import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--robot-workspace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--domain',type=int,default=78)
    args=parser.parse_args()
    if not args.execute:parser.error('--execute is required')
    p=args.workspace.resolve();args.output.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy()
    if not env.get('DISPLAY'):
        for proc in Path('/proc').iterdir():
            try:
                if not proc.name.isdigit() or proc.stat().st_uid!=os.getuid():continue
                e=dict(x.split('=',1) for x in (proc/'environ').read_text().split('\0') if '=' in x)
                if 'DISPLAY' in e and 'XAUTHORITY' in e:
                    env.update({k:e[k] for k in ['DISPLAY','XAUTHORITY','XDG_RUNTIME_DIR'] if k in e});break
            except (OSError,ValueError):pass
    env.update(ROS_DOMAIN_ID=str(args.domain),LIBGL_ALWAYS_SOFTWARE='1',LIBGL_DRI3_DISABLE='1')
    setup='source /opt/ros/humble/setup.bash; source '+shlex.quote(str(args.robot_workspace/'install/setup.bash'))+'; exec '
    results=[]
    cases=['recovery','watchdog','motion_1','motion_2','motion_3']
    for case in cases:
        folder=args.output/case;folder.mkdir();env['IGN_PARTITION']='gcu_phase3_final_'+case
        commands=[('simulator',['ros2','launch','lidarbot_gz','gz_launch.py','world:='+str(p/'assets/gcu_ground_floor_ogre.sdf'),
            'urdf_path:='+str(p/'robot_profile/gcu_evaluation.urdf.xacro'),'x_pose:=-0.05864395131342248','y_pose:=-12.960313240266368',
            'z_pose:=0.15','yaw_pose:=0','use_robot_localization:=false']),
            ('localization',['ros2','launch',str(p/'gcu_amcl.launch.py'),'map:='+str(p/'assets/gcu_ground_floor.yaml'),'params:='+str(p/'assets/amcl.yaml')]),
            ('truth_bridge',['ros2','run','ros_gz_bridge','parameter_bridge','/model/lidarbot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '--ros-args','-r','/model/lidarbot/pose:=/gcu_ground_truth'])]
        processes=[];status={'case':case};logs=[]
        try:
            for name,cmd in commands:
                log=(folder/(name+'.log')).open('w');logs.append(log)
                processes.append(subprocess.Popen(['/bin/bash','-c',setup+shlex.join(cmd)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            time.sleep(10)
            if case.startswith('motion'):
                cmd=['python3',str(p/'check_gcu_motion.py'),'--execute','--map',str(p/'assets/gcu_ground_floor.yaml')]
            else:
                cmd=['python3',str(p/'check_gcu_recovery_watchdog.py'),'--execute','--mode',case]
            cmd+=['--output',str(folder/'report.json')]
            with (folder/'check.log').open('w') as log:
                result=subprocess.run(['/bin/bash','-c',setup+shlex.join(cmd)],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180)
            status['checker_exit']=result.returncode
            if (folder/'report.json').exists():status['report']=json.loads((folder/'report.json').read_text())
            status['passed']=result.returncode==0 and status.get('report',{}).get('passed',False)
        except Exception as error:status.update(passed=False,error=str(error))
        finally:
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
        # Summaries omit per-sample traces; complete reports remain in case folders.
        brief={k:v for k,v in status.items() if k!='report'}
        if 'report' in status:brief['metrics']={k:v for k,v in status['report'].items() if k not in ['samples','trace']}
        results.append(brief)
        (args.output/'summary.json').write_text(json.dumps({'domain':args.domain,'cases':results,'all_passed':len(results)==len(cases) and all(x['passed'] for x in results)},indent=2)+'\n')
        print(case+': '+('PASS' if status['passed'] else 'FAIL'),flush=True)
    return 0 if all(x['passed'] for x in results) else 1


if __name__=='__main__':raise SystemExit(main())
