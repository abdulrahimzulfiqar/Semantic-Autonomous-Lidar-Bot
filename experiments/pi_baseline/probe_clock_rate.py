#!/usr/bin/env python3
"""Measure Gazebo/bridge clock rate without spawning a robot or sending goals."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(a.workspace))
    task=dict(ROS_DOMAIN_ID='218',ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
        CYCLONEDDS_URI='file://'+str(a.workspace/'dds.xml'),IGN_PARTITION='gcu_phase9_clock_rate17a',
        ROS_LOG_DIR=str(a.output/'ros_logs'),LIBGL_ALWAYS_SOFTWARE='1',LIBGL_DRI3_DISABLE='1')
    os.environ.update(task);env=dict(os.environ)
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            if proc.stat().st_uid!=1000:continue
            values=dict(v.split('=',1) for v in (proc/'environ').read_text().split('\0') if '=' in v)
            if values.get('DISPLAY') and values.get('XAUTHORITY'):
                env.update({k:values[k] for k in ['DISPLAY','XAUTHORITY','XDG_RUNTIME_DIR'] if k in values})
        except (OSError,ValueError):pass
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from ament_index_python.packages import get_package_share_directory
    rclpy.init();node=rclpy.create_node('phase9_clock_rate_probe')
    received=[]
    subscription=node.create_subscription(Clock,'/clock',lambda m:
        received.append((time.monotonic(),m.clock.sec+m.clock.nanosec/1e9)),
        QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    bridge=Path(get_package_share_directory('lidarbot_gz'))/'config/lidarbot_bridge.yaml'
    logs=[];children=[];shutdown={}
    try:
        for name,cmd in [('world',['ign','gazebo','-r','-s',str(a.workspace/'assets/contact_v2/world.sdf')]),
            ('bridge',[str(a.workspace/'support_install/lib/gcu_truth_bridge/gcu_ordered_bridge'),
                '--ros-args','-p','config_file:='+str(bridge)])]:
            log=(a.output/(name+'.log')).open('w');logs.append(log)
            children.append(subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        began=time.monotonic()
        while time.monotonic()-began<25:
            rclpy.spin_once(node,timeout_sec=.01)
            if received and time.monotonic()-received[0][0]>=8:break
    finally:
        node.destroy_node();rclpy.shutdown()
        from gcu_quiescent_shutdown import stop_quiescent_simulator
        shutdown=stop_quiescent_simulator(task['IGN_PARTITION'])
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()
    wall=received[-1][0]-received[0][0] if len(received)>1 else 0
    sim=received[-1][1]-received[0][1] if len(received)>1 else 0
    result=dict(scope='Clock-only diagnostic; no robot spawned, no navigation commands',
        messages=len(received),wall_interval_s=wall,sim_interval_s=sim,
        received_per_wall_s=(len(received)-1)/wall if wall else None,
        received_per_sim_s=(len(received)-1)/sim if sim else None,
        real_time_factor=sim/wall if wall else None,
        process_exits=[c.returncode for c in children],shutdown=shutdown)
    (a.output/'rate.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
