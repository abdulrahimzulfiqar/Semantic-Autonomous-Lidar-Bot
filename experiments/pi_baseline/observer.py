#!/usr/bin/env python3
"""Simulator-side read-only truth/contact recorder; never sends robot commands."""
import argparse
import json
import math
from pathlib import Path
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scan-topic',default='/scan')
    args=parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
    from tf2_ros import Buffer,TransformListener
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan
    from ros_gz_interfaces.msg import Contacts
    from std_msgs.msg import String
    rclpy.init()
    node=Node('phase9_truth_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer()
    listener=TransformListener(buffer,node)
    report=dict(scope='Laptop-only independent truth/contact observation',truth=[],matched=[],
        commands=[],scan_receipts=[],contacts=[],contact_counts={},pilot_result=None)
    pending=[]
    subscriptions=[]
    last_truth=-1.
    def stamp(value):return value.sec+value.nanosec/1e9
    def yaw(q):return math.atan2(2*q.w*q.z,1-2*q.z*q.z)
    def truth(message):
        nonlocal last_truth
        for transform in message.transforms:
            if transform.child_frame_id.strip('/').split('::')[-1]!='lidarbot':continue
            s=stamp(transform.header.stamp)
            if s-last_truth<.019:continue
            last_truth=s
            t=transform.transform
            row=dict(sim_time=s,truth=[t.translation.x,t.translation.y,yaw(t.rotation)])
            report['truth'].append(row);pending.append(row)
    def contact(message,topic):
        report['contact_counts'][topic]=report['contact_counts'].get(topic,0)+1
        for c in message.contacts:
            names=[c.collision1.name,c.collision2.name]
            if (any('cad_walls' in n or 'obstacle' in n for n in names) or not all(names) or
                    not (any(n.startswith('ground::') for n in names) or all(n.startswith('lidarbot::') for n in names))):
                report['contacts'].append(dict(sim_time=stamp(message.header.stamp),names=names,topic=topic))
    began=time.monotonic()
    def scan(message):
        report['scan_receipts'].append(dict(stamp=stamp(message.header.stamp),
            received_ros=node.get_clock().now().nanoseconds/1e9,received_wall=time.monotonic()))
    def completed(message):report['pilot_result']=json.loads(message.data)
    subscriptions.append(node.create_subscription(TFMessage,'/gcu_ground_truth',truth,
        QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)))
    subscriptions.append(node.create_subscription(Twist,'/phase9/sim_cmd_vel',lambda m:
        report['commands'].append(dict(sim_time=node.get_clock().now().nanoseconds/1e9,
            wall=time.monotonic(),v=m.linear.x,w=m.angular.z)),100))
    subscriptions.append(node.create_subscription(LaserScan,args.scan_topic,scan,qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(String,'/phase9/pilot_result',completed,
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    collisions=json.loads((args.workspace/'assets/contact_v2/contact_manifest.json').read_text())['collisions']
    for item in collisions:
        topic=item['topic']
        subscriptions.append(node.create_subscription(Contacts,topic,
            lambda m,t=topic:contact(m,t),qos_profile_sensor_data))
    stop_at=None
    while time.monotonic()-began<330:
        rclpy.spin_once(node,timeout_sec=.01)
        for row in list(pending):
            try:
                v=buffer.lookup_transform('map','base_footprint',Time(seconds=row['sim_time'])).transform
                estimate=[v.translation.x,v.translation.y,yaw(v.rotation)]
                delta=estimate[2]-row['truth'][2]
                report['matched'].append(dict(row,estimated=estimate,
                    position_error_m=math.dist(estimate[:2],row['truth'][:2]),
                    yaw_error_deg=math.degrees(abs(math.atan2(math.sin(delta),math.cos(delta))))))
                pending.remove(row)
            except Exception:
                if node.get_clock().now().nanoseconds/1e9-row['sim_time']>2:
                    pending.remove(row)
        if report['pilot_result'] is not None and stop_at is None:
            stop_at=time.monotonic()+6
        if stop_at is not None and time.monotonic()>stop_at:break
        if (args.output.parent/'stop_observer').exists():break
    report['elapsed_wall_s']=time.monotonic()-began
    report['contact_publishers']={v['topic']:node.count_publishers(v['topic']) for v in collisions}
    report['unmatched_pending']=len(pending)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    node.destroy_node();rclpy.shutdown()
    return 0 if report['pilot_result'] is not None else 1


if __name__=='__main__':raise SystemExit(main())
