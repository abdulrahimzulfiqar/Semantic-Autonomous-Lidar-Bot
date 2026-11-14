#!/usr/bin/env python3
"""Read-only independent clock arrival trace, no publishers or motion APIs."""
import argparse
import json
import os
from pathlib import Path
import time


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--run-result',type=Path,required=True);a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='195' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('isolated_simulation_clock_probe_required')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    rclpy.init();n=Node('gcu_independent_clock_observer');last=[-float('inf')]
    with a.output.open('x') as output:
        def receive(m):
            now=time.monotonic()
            if now-last[0]<.05:return
            output.write(json.dumps(dict(wall=now,sim=m.clock.sec+m.clock.nanosec/1e9))+'\n');output.flush();last[0]=now
        subscription=n.create_subscription(Clock,'/clock',receive,qos_profile_sensor_data)
        end=time.monotonic()+850
        while time.monotonic()<end and not a.run_result.exists():rclpy.spin_once(n,timeout_sec=.1)
    n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
