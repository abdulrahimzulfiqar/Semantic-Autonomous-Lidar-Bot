#!/usr/bin/env python3
"""Read-only simulation recorder. Never publishes TF, goals or wheel commands.

Estimator inputs and evaluator-only truth/AMCL outputs are saved separately.
Start only AFTER the existing runner has passed its isolated-domain guard.
"""
import argparse
import bisect
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time
import numpy as np


def pose(transform):
    v=transform.translation;q=transform.rotation
    if abs(q.x)+abs(q.y)>1e-3:raise ValueError('nonplanar_transform')
    return [v.x,v.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--run-result',type=Path,required=True);p.add_argument('--initial-pose',nargs=3,type=float,required=True)
    p.add_argument('--wall-timeout',type=float,default=300);a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='195' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':p.error('isolated_simulation_domain_195_required')
    a.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import Buffer,TransformListener
    from tf2_msgs.msg import TFMessage
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseWithCovarianceStamped
    rclpy.init();node=Node('phase8_readonly_capture',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,node)
    pending=[];truth=[];measurements=[];rows=[];received=0;dropped=0;errors={};stopped=False;resources=[];last_resource=0.
    def scan(m):
        nonlocal received
        received+=1;pending.append((time.monotonic(),m))
    def ground(m):
        for t in m.transforms:
            if t.child_frame_id.strip('/').split('::')[-1]!='lidarbot':continue
            stamp=t.header.stamp.sec+t.header.stamp.nanosec/1e9
            if not truth or stamp>truth[-1][0]:truth.append((stamp,pose(t.transform)))
    def amcl(m):measurements.append(m.header.stamp.sec+m.header.stamp.nanosec/1e9)
    subscriptions=[node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data),
        node.create_subscription(TFMessage,'/gcu_ground_truth',ground,qos_profile_sensor_data),
        node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',amcl,100)]
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    begin=time.monotonic();finished=None;range_geometry=None
    try:
        while not stopped and time.monotonic()-begin<a.wall_timeout:
            rclpy.spin_once(node,timeout_sec=.01)
            while pending:
                arrived,m=pending[0];stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
                try:
                    index=bisect.bisect_left([v[0] for v in truth],stamp)
                    if index==0 or index==len(truth):raise ValueError('truth_not_bracketed')
                    left,right=truth[index-1],truth[index]
                    if right[0]-left[0]>.15:raise ValueError('truth_gap')
                    alpha=(stamp-left[0])/(right[0]-left[0]);x=(1-alpha)*left[1][0]+alpha*right[1][0];y=(1-alpha)*left[1][1]+alpha*right[1][1]
                    dy=math.atan2(math.sin(right[1][2]-left[1][2]),math.cos(right[1][2]-left[1][2]));yaw=left[1][2]+alpha*dy
                    at=Time.from_msg(m.header.stamp)
                    odo=pose(buffer.lookup_transform('odom','base_footprint',at).transform)
                    baseline=pose(buffer.lookup_transform('map','base_footprint',at).transform)
                    extrinsic=pose(buffer.lookup_transform('base_footprint',m.header.frame_id,at).transform)
                    geometry=(len(m.ranges),float(m.angle_min),float(m.angle_increment),float(m.range_min),float(m.range_max))
                    if range_geometry is not None and geometry!=range_geometry:raise RuntimeError('scan_geometry_changed')
                    range_geometry=geometry
                    rows.append((stamp,np.asarray(m.ranges,dtype=np.float32),odo,extrinsic,[x,y,yaw],baseline));pending.pop(0)
                except RuntimeError:raise
                except Exception as error:
                    if time.monotonic()-arrived<1.:break
                    dropped+=1;errors[type(error).__name__+':'+str(error).split('\n')[0][:90]]=errors.get(type(error).__name__+':'+str(error).split('\n')[0][:90],0)+1;pending.pop(0)
            if time.monotonic()-last_resource>=1.:
                # Observer-only native component samples, not total robot RAM.
                sample=[]
                for proc in Path('/proc').glob('[0-9]*'):
                    try:
                        if b'ROS_DOMAIN_ID=195' not in (proc/'environ').read_bytes().split(b'\0'):continue
                        role=Path((proc/'cmdline').read_bytes().split(b'\0')[0].decode()).name
                        if role not in ['amcl','map_server','ekf_node']:continue
                        stat=(proc/'stat').read_text().split(') ',1)[1].split();status=(proc/'status').read_text().splitlines()
                        rss=next(int(v.split()[1]) for v in status if v.startswith('VmRSS:'))
                        try:pss=next(int(v.split()[1]) for v in (proc/'smaps_rollup').read_text().splitlines() if v.startswith('Pss:'))
                        except (OSError,StopIteration):pss=None
                        sample.append(dict(pid=int(proc.name),role=role,rss_kib=rss,pss_kib=pss,cpu_s=(int(stat[11])+int(stat[12]))/os.sysconf('SC_CLK_TCK')))
                    except (OSError,ValueError,StopIteration):pass
                resources.append(dict(wall=time.monotonic(),sim=node.get_clock().now().nanoseconds/1e9,processes=sample));last_resource=time.monotonic()
            if a.run_result.exists():
                finished=finished or time.monotonic()
                if time.monotonic()-finished>1.:break
    finally:
        if not rows:raise RuntimeError('no_common_scan_observations')
        stamps=np.asarray([r[0] for r in rows]);geometry=range_geometry
        np.savez_compressed(a.output/'observations.npz',stamps=stamps,ranges=np.stack([r[1] for r in rows]),
            angles=geometry[1]+np.arange(geometry[0])*geometry[2],odom=np.array([r[2] for r in rows]),
            extrinsics=np.array([r[3] for r in rows]),range_limits=geometry[3:],initial_pose=a.initial_pose)
        np.savez_compressed(a.output/'evaluation.npz',stamps=stamps,truth=np.array([r[4] for r in rows]),
            amcl=np.array([r[5] for r in rows]),amcl_measurement_stamps=np.asarray(measurements))
        manifest=dict(scope='Read-only live simulation capture; truth/AMCL estimates excluded from candidate inputs',
            rows=len(rows),received_scans=received,dropped_scans=dropped,pending_at_exit=len(pending),drop_reasons=errors,
            max_scan_gap_sim_s=float(np.max(np.diff(stamps))) if len(stamps)>1 else None,
            initial_pose=a.initial_pose,run_result=str(a.run_result),resources=resources,
            files_sha256={n:hashlib.sha256((a.output/n).read_bytes()).hexdigest() for n in ['observations.npz','evaluation.npz']},
            recorder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        (a.output/'capture.json').write_text(json.dumps(manifest,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    return 0


if __name__=='__main__':raise SystemExit(main())
