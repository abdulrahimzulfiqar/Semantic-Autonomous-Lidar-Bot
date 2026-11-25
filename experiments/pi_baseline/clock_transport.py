#!/usr/bin/env python3
"""Coalesce actual simulator Clock samples for the two-host link; no prediction."""
import argparse
import json
from pathlib import Path
import signal
import time


class ClockThrottle:
    def __init__(self,rate_hz):
        if not 0<rate_hz<=1000:raise ValueError('invalid_clock_transport_rate')
        self.period=1./rate_hz;self.latest=None;self.previous=None;self.last_wall=None

    def receive(self,stamp):
        self.latest=stamp

    def take(self,wall):
        if self.latest is None or self.latest==self.previous:return None
        if self.last_wall is not None and wall-self.last_wall+1e-9<self.period:return None
        self.previous=self.latest;self.last_wall=wall
        return self.latest


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rate-hz',type=float,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    limiter=ClockThrottle(a.rate_hz)
    import rclpy
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from rosgraph_msgs.msg import Clock as ClockMessage
    rclpy.init();node=rclpy.create_node('phase9_clock_transport')
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    publisher=node.create_publisher(ClockMessage,'/clock',qos)
    state=dict(received=0,published=0);buckets=[];running=True;began=time.monotonic();last_bucket=began
    def receive(message):
        state['received']+=1;limiter.receive((message.clock.sec,message.clock.nanosec))
    subscription=node.create_subscription(ClockMessage,'/phase9/raw_clock',receive,qos)
    def publish():
        nonlocal last_bucket
        now=time.monotonic();stamp=limiter.take(now)
        if stamp is not None:
            message=ClockMessage();message.clock.sec,message.clock.nanosec=stamp
            publisher.publish(message);state['published']+=1
        if now-last_bucket>=1:
            buckets.append(dict(wall=now,**state,last_raw=limiter.latest,last_published=limiter.previous))
            last_bucket=now
    timer=node.create_timer(.5/a.rate_hz,publish,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        while running and rclpy.ok():rclpy.spin_once(node,timeout_sec=.01)
    finally:
        a.output.write_text(json.dumps(dict(scope='Simulator-side clock transport; original timestamps, no extrapolation',
            maximum_wall_rate_hz=a.rate_hz,elapsed_wall_s=time.monotonic()-began,
            counters=state,buckets=buckets,last_raw=limiter.latest,last_published=limiter.previous),indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
