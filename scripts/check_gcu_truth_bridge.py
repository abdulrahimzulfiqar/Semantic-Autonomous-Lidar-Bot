#!/usr/bin/env python3
"""Compare evaluation bridge payloads at matching simulation timestamps; no commands."""
import argparse
import json
from pathlib import Path
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from tf2_msgs.msg import TFMessage
    rclpy.init();node=rclpy.create_node('gcu_truth_bridge_comparison')
    samples={'new':{},'reference':{}};counts={'new':0,'reference':0};qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
    def record(msg,key):
        for t in msg.transforms:
            if t.child_frame_id.strip('/').split('::')[-1]!='lidarbot':continue
            counts[key]+=1
            stamp=t.header.stamp.sec*10**9+t.header.stamp.nanosec;v=t.transform
            samples[key][stamp]=[v.translation.x,v.translation.y,v.translation.z,
                                v.rotation.x,v.rotation.y,v.rotation.z,v.rotation.w]
    subscriptions=[node.create_subscription(TFMessage,topic,lambda m,k=key:record(m,k),qos)
        for key,topic in [('new','/gcu_ground_truth'),('reference','/gcu_ground_truth_reference')]]
    end=time.monotonic()+40
    while time.monotonic()<end and min(counts.values())<80:rclpy.spin_once(node,timeout_sec=.05)
    stamps=sorted(set(samples['new'])&set(samples['reference']))
    pairs=[{'sim_time_ns':s,'new':samples['new'][s],'reference':samples['reference'][s]} for s in stamps]
    maximum=max((max(abs(a-b) for a,b in zip(p['new'],p['reference'])) for p in pairs),default=None)
    span=(stamps[-1]-stamps[0])/1e9 if stamps else 0.
    passed=len(pairs)>=50 and span>=1. and maximum is not None and maximum<1e-12
    result={'passed':passed,'matched_timestamps':len(pairs),'span_sim_s':span,
            'maximum_component_difference':maximum,'scope':'model lidarbot translation/quaternion and simulation stamps; evaluation only',
            'samples':pairs,'source_message_counts':counts,'source_unique_timestamps':{k:len(v) for k,v in samples.items()},
            'source_first_samples':{k:list(v.items())[:3] for k,v in samples.items()}}
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='samples'}))
    node.destroy_node();rclpy.shutdown();return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
