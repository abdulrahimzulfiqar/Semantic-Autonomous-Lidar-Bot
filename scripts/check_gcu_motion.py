#!/usr/bin/env python3
"""Bounded SIMULATION-ONLY motion test, with ground truth used only for evaluation.

Requires explicit --execute, fresh corrected GCU spawn, AMCL and dedicated
/gcu_ground_truth TFMessage bridge. Never bridge ground truth onto /tf.
"""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
from PIL import Image
import yaml


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clearance_on_path(map_yaml, start, distance=.4, radius=.18):
    config = yaml.safe_load(Path(map_yaml).read_text())
    if abs(config['origin'][2]) > 1e-8:
        raise ValueError('Rotated map unsupported')
    image = np.flipud(np.asarray(Image.open(Path(map_yaml).parent / config['image'])))
    resolution = config['resolution']
    ys, xs = np.nonzero(image != 254)  # occupied AND unknown block this experiment
    wx = config['origin'][0] + (xs+.5)*resolution
    wy = config['origin'][1] + (ys+.5)*resolution
    minimum = math.inf
    for d in np.linspace(0, distance, max(2, int(distance/(resolution/2))+1)):
        x, y = start[0]+d*math.cos(start[2]), start[1]+d*math.sin(start[2])
        ix = int(math.floor((x-config['origin'][0])/resolution))
        iy = int(math.floor((y-config['origin'][1])/resolution))
        if not (0 <= ix < image.shape[1] and 0 <= iy < image.shape[0]) or image[iy, ix] != 254:
            return 0.0
        minimum = min(minimum, float(np.min(np.hypot(wx-x, wy-y)))-resolution/math.sqrt(2))
    return max(0., minimum)


def main():
    import rclpy
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from sensor_msgs.msg import LaserScan
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from std_srvs.srv import Empty
    from tf2_ros import Buffer, TransformListener, TransformException
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('gcu_motion_report.json'))
    args = parser.parse_args()
    if not args.execute:
        parser.error('Requires --execute; this test publishes simulated velocity commands')
    rclpy.init(args=[])
    node = rclpy.create_node('gcu_bounded_simulation_motion')
    sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    state = {'truth': None, 'scan_wall': 0., 'truth_wall': 0., 'clock_wall': 0., 'amcl_wall': 0., 'sim': None, 'truth_buffer': []}
    samples = []
    command_count = 0
    nonzero_count = 0
    def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    def truth(msg):
        for t in msg.transforms:
            if t.child_frame_id.strip('/').split('::')[-1] == 'lidarbot':
                state['truth'] = t
                state['truth_wall'] = time.monotonic()
                state['truth_buffer'].append(t)
                state['truth_buffer'] = state['truth_buffer'][-100:]
    def scan(msg):
        values = [v for v in msg.ranges if math.isfinite(v) and msg.range_min <= v <= msg.range_max]
        state['nearest'] = min(values) if values else 0.
        state['scan_wall'] = time.monotonic()
    def clock(msg):
        value = msg.clock.sec+msg.clock.nanosec/1e9
        if state['sim'] != value: state['clock_wall'] = time.monotonic()
        state['sim'] = value
    def amcl(msg): state['amcl_wall'] = time.monotonic()
    subscriptions = [node.create_subscription(TFMessage, '/gcu_ground_truth', truth, 10),
        node.create_subscription(LaserScan, '/scan', scan, sensor),
        node.create_subscription(Clock, '/clock', clock, sensor),
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl, 10)]
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    nomotion = node.create_client(Empty, '/request_nomotion_update')
    def command(v=0., w=0.):
        nonlocal command_count, nonzero_count
        msg = Twist();msg.linear.x=v;msg.angular.z=w
        publisher.publish(msg);command_count += 1
        nonzero_count += int(abs(v)+abs(w)>0)
    def position():
        t = state['truth'].transform
        return (t.translation.x, t.translation.y, yaw(t.rotation))
    last_sample = None
    def step(v=0., w=0., guard=True):
        nonlocal last_sample
        until = time.monotonic()+.05
        while time.monotonic()<until: rclpy.spin_once(node, timeout_sec=.01)
        now = time.monotonic()
        if guard:
            if any(now-state[k] > 1.5 for k in ['truth_wall','scan_wall','clock_wall']):
                raise RuntimeError('Stale ground truth, scan or advancing simulation clock')
            if state.get('nearest',0) < .5: raise RuntimeError('Obstacle within 0.5 m of laser')
            if time.monotonic()-started > 150: raise RuntimeError('Wall-clock experiment timeout')
            if math.hypot(position()[0]-start_pose[0], position()[1]-start_pose[1]) > .65:
                raise RuntimeError('Ground-truth displacement exceeds experiment envelope')
        for t in reversed(state['truth_buffer']):
            stamp=(t.header.stamp.sec,t.header.stamp.nanosec)
            if stamp==(0,0) or (last_sample is not None and stamp<=last_sample):
                continue
            try:
                estimate = buffer.lookup_transform('map','base_footprint',Time.from_msg(t.header.stamp))
                actual=(t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation))
                error=math.hypot(estimate.transform.translation.x-actual[0],estimate.transform.translation.y-actual[1])
                heading=abs(wrap(yaw(estimate.transform.rotation)-actual[2]))
                samples.append({'sim_time':stamp[0]+stamp[1]/1e9,'truth':actual,
                    'estimated':[estimate.transform.translation.x,estimate.transform.translation.y,yaw(estimate.transform.rotation)],
                    'position_error_m':error,'yaw_error_deg':math.degrees(heading)})
                last_sample=stamp
                state['match_wall']=now
                break
            except TransformException:
                continue
        if guard and now-state.get('match_wall',started)>1.5:
            raise RuntimeError('Timestamp-matched localization transform unavailable')
        command(v,w)
    def hold(seconds):
        begin=state['sim'];wall=time.monotonic()
        while state['sim']-begin < seconds:
            if time.monotonic()-wall > 30: raise RuntimeError('Stop/settle timeout')
            step()
    report={'simulation_only':True,'distance_target_m':.4,'turn_target_deg':45,
        'linear_speed_command_mps':.06,'angular_speed_command_radps':.15,
        'reference_robot_radius_m':.18,'ground_truth_fed_to_amcl':False,
        'acceptance':{'distance_tolerance_m':.05,'turn_tolerance_deg':5,
            'localization_p95_position_m':.25,'localization_p95_yaw_deg':5,'stop_drift_limit_m':.02}}
    started=time.monotonic()
    try:
        requested_update = False
        while time.monotonic()-started < 45:
            if not requested_update and time.monotonic()-started>3 and nomotion.service_is_ready():
                nomotion.call_async(Empty.Request())
                requested_update = True
            rclpy.spin_once(node, timeout_sec=.1)
            if state['truth'] is not None and state['sim'] is not None and state['amcl_wall']>0 and state['scan_wall']>0 and publisher.get_subscription_count()>0:
                if buffer.can_transform('map','base_footprint',Time()):break
        else: raise RuntimeError('Simulator/ground truth/AMCL/command path not ready')
        start_pose=position()
        if math.hypot(start_pose[0]+.0586439513,start_pose[1]+12.96031324)>.25 or abs(start_pose[2])>.1:
            raise RuntimeError('Requires fresh corrected GCU spawn near the known pose')
        minimum=clearance_on_path(args.map,start_pose)
        report['minimum_planned_path_clearance_m']=minimum
        if minimum < .5:raise RuntimeError('Planned short path lacks 0.5 m clearance')
        report['start_truth']=start_pose
        start_sim=state['sim']
        hold(2)
        begin=state['sim']
        while math.hypot(position()[0]-start_pose[0],position()[1]-start_pose[1]) < .4:
            if state['sim']-begin > 20: raise RuntimeError('Straight motion timeout')
            step(.06,0.)
        hold(2)
        straight_pose=position()
        report['straight_distance_m']=math.hypot(straight_pose[0]-start_pose[0],straight_pose[1]-start_pose[1])
        begin=state['sim'];target=math.pi/4
        while wrap(position()[2]-straight_pose[2]) < target:
            if state['sim']-begin > 15:raise RuntimeError('Rotation timeout')
            step(0.,.15)
        hold(2)
        final=position()
        report['turn_achieved_deg']=math.degrees(wrap(final[2]-straight_pose[2]))
        stop_pose=final;hold(2)
        report['stop_drift_m']=math.hypot(position()[0]-stop_pose[0],position()[1]-stop_pose[1])
        report['final_truth']=position()
        report['experiment_sim_seconds']=state['sim']-start_sim
        report['completed_motion_sequence']=True
    except Exception as error:
        report['error']=str(error)
        report['completed_motion_sequence']=False
    finally:
        # Repeated zero commands plus the controller's own timeout stop the test.
        end=time.monotonic()+1
        while time.monotonic()<end:
            command()
            rclpy.spin_once(node,timeout_sec=.001)
            time.sleep(.05)
        report['wall_seconds']=time.monotonic()-started
        report['velocity_messages_sent']=command_count
        report['nonzero_velocity_messages_sent']=nonzero_count
        report['readiness']={'ground_truth_received':state['truth'] is not None,'amcl_pose_received':state['amcl_wall']>0,'scan_received':state['scan_wall']>0,'clock_received':state['sim'] is not None}
        report['timestamp_matched_samples']=len(samples)
        if samples:
            report['p95_position_error_m']=float(np.percentile([s['position_error_m'] for s in samples],95))
            report['p95_yaw_error_deg']=float(np.percentile([s['yaw_error_deg'] for s in samples],95))
            report['max_position_error_m']=max(s['position_error_m'] for s in samples)
        report['samples']=samples
        report['passed']=bool(report['completed_motion_sequence'] and len(samples)>=30 and
            abs(report.get('straight_distance_m',0)-.4)<=.05 and abs(report.get('turn_achieved_deg',0)-45)<=5 and
            report.get('stop_drift_m',999)<=.02 and report.get('p95_position_error_m',999)<=.25 and
            report.get('p95_yaw_error_deg',999)<=5)
        report['limitations']='One short open-space trial; no contact sensor, collision-free route or recovery claim. Clearance and distance/turn limits use truth as a test supervisor; AMCL receives no truth.'
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2),flush=True)
        node.destroy_node();rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
