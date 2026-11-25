#!/usr/bin/env python3
"""Bounded Pi-local AMCL timing diagnostic. Never publishes a velocity or goal."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
from replay_timing import interpolated_pose, timing_summary, validate_recording


def write(path,value):path.write_text(json.dumps(value,indent=2)+'\n')


def ros_node(name):
    import rclpy
    from rclpy.parameter import Parameter
    rclpy.init(args=[])
    return rclpy.create_node(name,parameter_overrides=[Parameter('use_sim_time',value=True)])


def feed(w,out,rate):
    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
    # NpzFile reads/decompresses on every key access. Cache once so the replay
    # scheduler does not repeatedly decompress the complete scan array.
    with np.load(w/'observations.npz',allow_pickle=False) as recording:
        data={key:recording[key] for key in recording.files}
    validate_recording(data)
    node=ros_node('phase9_local_replay_feed')
    clock=node.create_publisher(Clock,'/clock',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    scan=node.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
    tf=TransformBroadcaster(node);static=StaticTransformBroadcaster(node)
    def stamp(value):
        ns=round(value*1e9);return Time(sec=ns//10**9,nanosec=ns%10**9)
    def transform(value,frame,child,point):
        m=TransformStamped();m.header.stamp=stamp(value);m.header.frame_id=frame;m.child_frame_id=child
        m.transform.translation.x=point[0];m.transform.translation.y=point[1]
        m.transform.rotation.z=math.sin(point[2]/2);m.transform.rotation.w=math.cos(point[2]/2)
        return m
    recording_start=float(data['stamps'][0]);end=float(data['stamps'][-1])
    # AMCL future-dates map->odom by 0.2 s. A clock frozen at the first
    # observation can never compose that first transform with earlier odometry.
    # Replay a declared 1 s input warmup, then score only the remaining interval.
    first=int(np.searchsorted(data['stamps'],recording_start+1.))
    if first>=len(data['stamps'])-1:raise ValueError('recording_too_short_for_warmup')
    start=float(data['stamps'][first])
    static.sendTransform(transform(recording_start,'base_footprint','replay_laser',data['extrinsics'][0]))
    scans=[]
    def publish_scan(i,due=None):
        m=LaserScan();m.header.stamp=stamp(float(data['stamps'][i]));m.header.frame_id='replay_laser'
        m.angle_min=float(data['angles'][0]);m.angle_max=float(data['angles'][-1]);m.angle_increment=float(data['angles'][1]-data['angles'][0])
        m.range_min=float(data['range_limits'][0]);m.range_max=float(data['range_limits'][1]);m.scan_time=.1
        m.ranges=data['ranges'][i].tolist();scan.publish(m)
        if due is not None:scans.append(dict(index=i,stamp_ns=round(float(data['stamps'][i])*1e9),wall=time.monotonic(),late_wall_s=max(0.,time.monotonic()-due)))
    begin=time.monotonic();last_scan=-1.;i=0
    # Warm up at 1x, then hold that recorded state until the TF chain is ready.
    while not (out/'start_replay').exists():
        if time.monotonic()-begin>25:raise RuntimeError('local_replay_warmup_timeout')
        current=min(start,recording_start+time.monotonic()-begin)
        tf.sendTransform(transform(current,'odom','base_footprint',interpolated_pose(data['stamps'],data['odom'],current)))
        clock.publish(Clock(clock=stamp(current)))
        while i<=first and data['stamps'][i]<=current:publish_scan(i);i+=1
        if current==start and time.monotonic()-last_scan>.1:publish_scan(first);last_scan=time.monotonic()
        rclpy.spin_once(node,timeout_sec=.01)
    # A usable chain may appear during warmup; finish the declared interval.
    while current<start:
        current=min(start,recording_start+time.monotonic()-begin)
        tf.sendTransform(transform(current,'odom','base_footprint',interpolated_pose(data['stamps'],data['odom'],current)))
        clock.publish(Clock(clock=stamp(current)))
        while i<=first and data['stamps'][i]<=current:publish_scan(i);i+=1
        rclpy.spin_once(node,timeout_sec=.01)
    begin=time.monotonic();next_clock=begin;next_tf=begin;i=first;clock_publications=[]
    while True:
        now=time.monotonic();current=min(end,start+(now-begin)*rate)
        if now>=next_tf:
            tf.sendTransform(transform(current,'odom','base_footprint',interpolated_pose(data['stamps'],data['odom'],current)))
            next_tf=now+.02/rate
        if now>=next_clock:
            clock.publish(Clock(clock=stamp(current)))
            clock_publications.append(dict(stamp_ns=round(current*1e9),wall=time.monotonic()))
            next_clock=now+.01/rate
        while i<len(data['stamps']) and data['stamps'][i]<=current:
            publish_scan(i,begin+(float(data['stamps'][i])-start)/rate);i+=1
        if current>=end:break
        rclpy.spin_once(node,timeout_sec=0)
        time.sleep(.001)
    result=dict(scope='SE(2) reconstruction: original scan stamps/ranges; recorded odometry interpolated at nominal 50 simulated Hz; generated replay clock nominal 100 simulated Hz',
        rate=rate,begin_wall=begin,end_wall=time.monotonic(),duration_sim_s=end-start,
        warmup_sim_s=start-recording_start,warmup_scans_excluded=first,
        clock_publications=clock_publications,
        scan_stamps_ns=[round(float(v)*1e9) for v in data['stamps'][first:]],scan_publications=scans)
    write(out/'feed.json',result)
    # Allow the observer to drain the last original scan without advancing time.
    until=time.monotonic()+1
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
    (out/'feed_done').touch();node.destroy_node();rclpy.shutdown()


def observe(w,out):
    import rclpy
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock as ClockMessage
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from tf2_ros import Buffer,TransformListener
    node=ros_node('phase9_local_replay_observer');buffer=Buffer();listener=TransformListener(buffer,node)
    report=dict(tf=[],scans=[],clocks=[],amcl_updates=[])
    def clock(m):report['clocks'].append(dict(wall=time.monotonic(),stamp=m.clock.sec+m.clock.nanosec/1e9))
    def scan(m):report['scans'].append(dict(wall=time.monotonic(),stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec))
    def amcl(m):report['amcl_updates'].append(dict(wall=time.monotonic(),stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9))
    subscriptions=[node.create_subscription(ClockMessage,'/clock',clock,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)),
        node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data),
        node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',amcl,100)]
    def check():
        current=node.get_clock().now().nanoseconds/1e9;row=dict(wall=time.monotonic(),clock=current)
        try:
            t=buffer.lookup_transform('map','base_footprint',Time());s=t.header.stamp.sec+t.header.stamp.nanosec/1e9
            if s>current:t=buffer.lookup_transform('map','base_footprint',Time(nanoseconds=round(current*1e9)))
            row['stamp']=t.header.stamp.sec+t.header.stamp.nanosec/1e9
            if current>0:(out/'observer_ready').touch()
        except Exception as error:row['error']=type(error).__name__+':'+str(error)[:180]
        report['tf'].append(row)
    timer=node.create_timer(.04,check,clock=Clock(clock_type=ClockType.STEADY_TIME))
    begin=time.monotonic()
    try:
        while not (out/'feed_done').exists() and time.monotonic()-begin<100:rclpy.spin_once(node,timeout_sec=.01)
    finally:
        write(out/'observations.json',report);node.destroy_node();rclpy.shutdown()


def run(w,out,rate):
    from resources import Sampler
    out.mkdir(parents=True,exist_ok=False)
    (out/'owner.pid').write_text(str(os.getpid()))
    manifest=json.loads((w/'replay_manifest.json').read_text())
    for name,want in manifest['files_sha256'].items():
        if hashlib.sha256((w/name).read_bytes()).hexdigest()!=want:raise RuntimeError('replay_source_changed:'+name)
    for path in Path('/proc').glob('[0-9]*'):
        try:
            if b'ROS_DOMAIN_ID=218' in (path/'environ').read_bytes().split(b'\0'):raise RuntimeError('phase9_domain_in_use')
        except OSError:pass
    env=dict(os.environ,ROS_DOMAIN_ID='218',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
        CYCLONEDDS_URI='file://'+str(w/'dds.xml'),ROS_LOG_DIR=str(out/'ros_logs'),PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    children=[];logs=[];sampler=Sampler();report=dict(passed=False,rate=rate,scope='Local AMCL/TF timing diagnostic only; no full-stack resource or closed-loop mission result')
    def interrupted(*_):raise RuntimeError('local_replay_interrupted')
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    def launch(name,cmd):
        log=(out/(name+'.log')).open('w');logs.append(log)
        child=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append((name,child));return child
    try:
        sampler.start()
        launch('map_server',['/opt/ros/humble/lib/nav2_map_server/map_server','--ros-args','-p','use_sim_time:=true','-p','yaml_filename:='+str(w/'gcu_ground_floor.yaml')])
        launch('amcl',['/opt/ros/humble/lib/nav2_amcl/amcl','--ros-args','--params-file',str(w/'amcl.yaml'),'-p','set_initial_pose:=true'])
        launch('lifecycle',['/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager','--ros-args','-p','use_sim_time:=true','-p','autostart:=true','-p',"node_names:=['map_server','amcl']"])
        observer=launch('observer',[sys.executable,str(w/'local_replay.py'),'--role','observe','--workspace',str(w),'--output',str(out)])
        feeder=launch('feed',[sys.executable,str(w/'local_replay.py'),'--role','feed','--workspace',str(w),'--output',str(out),'--rate',str(rate)])
        begin=time.monotonic()
        while not (out/'observer_ready').exists():
            if time.monotonic()-begin>28 or any(c.poll() is not None for _,c in children):raise RuntimeError('localization_replay_not_ready')
            time.sleep(.05)
        (out/'start_replay').touch()
        feeder.wait(timeout=65/rate+5);observer.wait(timeout=5)
        report['timing']=timing_summary(json.loads((out/'feed.json').read_text()),json.loads((out/'observations.json').read_text()))
        report['passed']=report['timing']['passed'] and feeder.returncode==0 and observer.returncode==0
    except Exception as error:report['error']=repr(error)
    finally:
        for name,child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    report.setdefault('forced_cleanup',[]).append(name);os.killpg(child.pid,signal.SIGKILL);child.wait()
        sampler.stop();write(out/'resources.json',dict(samples=sampler.rows))
        report['process_exits']={name:child.returncode for name,child in children}
        report['passed']=bool(report['passed'] and not report.get('forced_cleanup') and all(child.returncode==0 for _,child in children))
        for log in logs:log.close()
        report['replay_manifest_sha256']=hashlib.sha256((w/'replay_manifest.json').read_bytes()).hexdigest()
        write(out/'result.json',report)
    print(json.dumps(report,indent=2));return 0 if report['passed'] else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--role',choices=['run','feed','observe'],default='run')
    p.add_argument('--workspace',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--rate',type=float,choices=[1.,2.],default=1.)
    a=p.parse_args()
    if a.role!='run' and (os.environ.get('ROS_DOMAIN_ID')!='218' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'):
        p.error('isolated_local_replay_environment_required')
    if a.role=='run':raise SystemExit(run(a.workspace.resolve(),a.output.resolve(),a.rate))
    if a.role=='feed':feed(a.workspace,a.output,a.rate)
    else:observe(a.workspace,a.output)
