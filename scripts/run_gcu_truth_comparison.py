#!/usr/bin/env python3
"""Fresh simulation to compare new and native reference truth bridge payloads."""
import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
from gcu_process_audit import stop_truth_bridge


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False);w=args.workspace
    env=os.environ.copy()
    for p in Path('/proc').iterdir():
        try:
            if not p.name.isdigit() or p.stat().st_uid!=os.getuid():continue
            e=dict(x.split('=',1) for x in (p/'environ').read_text().split('\0') if '=' in x)
            if 'DISPLAY' in e and 'XAUTHORITY' in e:
                env.update({k:e[k] for k in ['DISPLAY','XAUTHORITY','XDG_RUNTIME_DIR'] if k in e});break
        except OSError:pass
    env.update(ROS_DOMAIN_ID='130',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',IGN_PARTITION='gcu_truth_comparison_'+str(os.getpid()),LIBGL_ALWAYS_SOFTWARE='1',LIBGL_DRI3_DISABLE='1')
    setup='source /opt/ros/humble/setup.bash; source /home/ubuntu/lidarbot_ws/install/setup.bash; exec '
    commands=[['ros2','launch',str(w/'gcu_phase5_sim.launch.py'),'assets:='+str(w/'assets/contact_v2'),'world:='+str(w/'assets/contact_v2/world.sdf')],
        ['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge','/model/lidarbot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V','--ros-args','-r','/model/lidarbot/pose:=/gcu_ground_truth_reference']]
    processes=[];files=[];status={'passed':False}
    try:
        for name,cmd in zip(['simulator','reference'],commands):
            log=(args.output/(name+'.log')).open('w');files.append(log)
            processes.append(subprocess.Popen(['/bin/bash','-c',setup+shlex.join(cmd)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        cmd=['python3',str(w/'check_gcu_truth_bridge.py'),'--output',str(args.output/'report.json')]
        with (args.output/'checker.log').open('w') as log:r=subprocess.run(['/bin/bash','-c',setup+shlex.join(cmd)],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=60)
        status['passed']=r.returncode==0;status['checker_exit']=r.returncode
    finally:
        if len(processes)>1:
            processes[1].send_signal(signal.SIGINT)
            try:processes[1].wait(timeout=8)
            except subprocess.TimeoutExpired:processes[1].kill();processes[1].wait()
        status['new_bridge_stop']=stop_truth_bridge(env['IGN_PARTITION'])
        if processes:
            processes[0].send_signal(signal.SIGINT)
            try:processes[0].wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(processes[0].pid,signal.SIGKILL);processes[0].wait()
        status['launch_exits']=[p.returncode for p in processes]
        status['note']='Native reference shutdown is recorded separately; equality comparison concerns pre-shutdown payloads.'
        for f in files:f.close()
        status['passed']=bool(status['passed'] and status['new_bridge_stop']['passed'] and status['launch_exits'][0]==0)
        (args.output/'run.json').write_text(json.dumps(status,indent=2)+'\n')
    return 0 if status['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
