#!/usr/bin/env python3
"""Optional reliable simulation scan hop; original message and stamp preserved."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import time


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--role',choices=['sender','receiver'],required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    profile=json.loads(Path(os.environ['EDGEOSMAG_PHASE9_PROFILE']).read_text())
    expected='simulator' if a.role=='sender' else 'navigation_pi'
    if os.environ.get('ROS_DOMAIN_ID')!='218' or not os.environ.get('IGN_PARTITION','').startswith('gcu_phase9_') or profile.get('role')!=expected or not profile.get('reliable_scan_link'):
        raise RuntimeError('explicit_phase9_scan_link_profile_required')
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import LaserScan
    rclpy.init(args=[]);node=rclpy.create_node('phase9_scan_'+a.role)
    reliable=QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE)
    source='/phase9/source_scan' if a.role=='sender' else '/phase9/remote_scan'
    target='/phase9/remote_scan' if a.role=='sender' else '/scan'
    publisher=node.create_publisher(LaserScan,target,reliable if a.role=='sender' else qos_profile_sensor_data)
    rows=[];running=True
    def receive(message):
        row=dict(wall=time.monotonic(),stamp_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
            serialized_sha256=hashlib.sha256(serialize_message(message)).hexdigest())
        publisher.publish(message);row['publication_return_wall']=time.monotonic();rows.append(row)
    subscription=node.create_subscription(LaserScan,source,receive,qos_profile_sensor_data if a.role=='sender' else reliable)
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    began=time.monotonic()
    try:
        while running and rclpy.ok() and time.monotonic()-began<360:rclpy.spin_once(node,timeout_sec=.01)
    finally:
        a.output.write_text(json.dumps(dict(role=a.role,source=source,target=target,rows=rows,
            scope='Reliable DDS simulation scan hop, unchanged serialized scan content and timestamps; no promised latency or control output'),indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
