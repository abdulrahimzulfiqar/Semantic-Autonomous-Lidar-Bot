#!/usr/bin/env python3
"""06B1 ROS adapter, HARD-LIMITED to a token-verified mock navigator.

No live /navigate_to_pose client or velocity publisher. Real motion and costmap
readiness are intentionally unavailable until the later constraint checkpoint.
"""
import argparse
from dataclasses import replace
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import time
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', type=Path, required=True)
    parser.add_argument('--mock-token', required=True)
    parser.add_argument('--namespace', required=True)
    parser.add_argument('--ack-timeout', type=float, default=1.)
    args, rosargs = parser.parse_known_args()
    prefix = args.namespace
    if not prefix.startswith('/gcu_phase6b1_') or '/' in prefix[1:]:
        parser.error('an isolated /gcu_phase6b1_NAME namespace is required')
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as NavPath
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import Empty, String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener, TransformException
    from osmag_paper_planner import PaperPlanner
    from osmag_mission_contract import make_contract, MissionSettings, CrossingMonitor, MissionError
    from osmag_mission_state import MissionState
    from gcu_navigation_metrics import rest_window

    class Adapter(Node):
        def __init__(self):
            super().__init__('gcu_mission_mock_adapter')
            self.planner = PaperPlanner(args.assets)
            self.model = MissionState(); self.monitor = None; self.handles = {}
            self.worker = ThreadPoolExecutor(max_workers=1); self.planning = None; self.planning_cancelled = False
            self.verified = False; self.identity_pending = False
            self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
            self.client = ActionClient(self, NavigateToPose, prefix+'/navigate_to_pose')
            self.identity = self.create_client(Trigger, prefix+'/identity')
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.status_pub = self.create_publisher(String, prefix+'/mission_status', qos)
            self.event_pub = self.create_publisher(String, prefix+'/events', 100)
            self.path_pub = self.create_publisher(NavPath, prefix+'/mission_path', qos)
            self.target_pub = self.create_publisher(PoseStamped, prefix+'/segment_goal', qos)
            self.goal_sub = self.create_subscription(PoseStamped, prefix+'/mission_goal', self.goal, 10)
            self.cancel_sub = self.create_subscription(Empty, prefix+'/mission_cancel', self.cancel, 10)
            self.timer = self.create_timer(.025, self.tick)
            self.last_tf_stamp = None; self.offset = 0.; self.samples = []; self.cancel_at = None
            self.events = 0; self.status()

        def event(self, kind, **fields):
            self.events += 1
            self.event_pub.publish(String(data=json.dumps(dict(event=kind, serial=self.events,
                wall=time.monotonic(), **fields))))

        def status(self, **fields):
            data = dict(self.model.report(), mock_only=True, physical_motion_enabled=False,
                        mock_verified=self.verified, **fields)
            self.status_pub.publish(String(data=json.dumps(data)))

        def pose(self):
            tf = self.buffer.lookup_transform('map', 'base_footprint', Time())
            stamp = tf.header.stamp.sec+tf.header.stamp.nanosec/1e9
            age = self.get_clock().now().nanoseconds/1e9-stamp
            if not -.05 <= age <= .15:
                raise MissionError('robot_tf_stale')
            t = tf.transform.translation; q = tf.transform.rotation
            point = [t.x, t.y]
            heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            return point, heading, stamp, age

        def verify_identity(self, future):
            self.identity_pending = False
            try:
                response = future.result(); data = json.loads(response.message)
                self.verified = bool(response.success and data == {
                    'token': args.mock_token, 'scope': 'mock_navigator_no_actuators',
                    'action': prefix+'/navigate_to_pose'})
            except Exception:
                self.verified = False

        def effects(self, effects):
            for effect in effects:
                if effect['kind'] == 'send':
                    if not self.verified:
                        self.fault('mock_identity_unverified'); return
                    segment = effect['segment']; target = PoseStamped()
                    target.header.frame_id = 'map'; target.header.stamp = self.get_clock().now().to_msg()
                    target.pose.position.x, target.pose.position.y = map(float, segment.point)
                    target.pose.orientation.z = math.sin(segment.yaw/2)
                    target.pose.orientation.w = math.cos(segment.yaw/2)
                    self.target_pub.publish(target)
                    action = NavigateToPose.Goal(); action.pose = target
                    self.event('send', token=effect['token'], segment=segment.index,
                               confirmed=self.model.confirmed, point=segment.point)
                    self.client.send_goal_async(action).add_done_callback(
                        lambda f, token=effect['token']: self.accepted(f, token))
                else:
                    handle = self.handles.get(effect['uuid'])
                    if handle is not None:
                        self.event('cancel_request', uuid=effect['uuid'])
                        handle.cancel_goal_async().add_done_callback(self.cancel_reply)

        def accepted(self, future, token):
            try:
                handle = future.result()
                ident = bytes(handle.goal_id.uuid).hex() if handle.accepted else ''
                if handle.accepted:
                    self.handles[ident] = handle
                effects = self.model.accepted(token, ident, handle.accepted, time.monotonic())
                self.event('accepted', token=token, uuid=ident, accepted=handle.accepted)
                if handle.accepted:
                    handle.get_result_async().add_done_callback(lambda f, ident=ident: self.result(f, ident))
                self.effects(effects); self.status()
            except Exception as error:
                self.fault('action_acceptance_error:'+type(error).__name__)

        def result(self, future, ident):
            try:
                response = future.result()
                outcome = {4:'succeeded', 5:'cancelled', 6:'aborted'}[response.status]
                self.event('result', uuid=ident, outcome=outcome)
                self.effects(self.model.result(ident, outcome, time.monotonic()))
                self.handles.pop(ident, None); self.status()
            except Exception as error:
                self.fault('action_result_error:'+type(error).__name__)

        def cancel_reply(self, future):
            try:
                self.event('cancel_reply', goals_canceling=len(future.result().goals_canceling))
            except Exception as error:
                self.event('cancel_reply_error', detail=type(error).__name__)

        def cancel(self, _msg):
            if self.planning is not None:
                self.planning_cancelled = True
            if self.cancel_at is None:
                self.cancel_at = time.monotonic()
            self.effects(self.model.cancel(time.monotonic())); self.status()

        def fault(self, reason):
            if self.model.contract is None:
                self.status(rejection=reason); return
            if self.model.state not in ['cancelling', 'failed', 'cancelled', 'succeeded']:
                self.cancel_at = time.monotonic()
                self.event('fault', reason=reason)
                self.effects(self.model.cancel(time.monotonic(), reason))
            self.status()

        def goal(self, msg):
            try:
                if self.planning is not None or self.model.state not in ['idle', 'succeeded', 'failed', 'cancelled'] or self.model._unresolved():
                    raise MissionError('busy_cancel_before_new_mission')
                if not self.verified:
                    raise MissionError('mock_identity_unverified')
                if not self.client.server_is_ready():
                    raise MissionError('navigator_not_ready')
                if self.count_publishers(prefix+'/navigate_to_pose/_action/status') != 1:
                    raise MissionError('navigator_ownership_invalid')
                p = msg.pose.position; q = msg.pose.orientation
                if msg.header.frame_id != 'map':
                    raise MissionError('goal_frame_must_be_map')
                if not all(math.isfinite(v) for v in [p.x,p.y,p.z,q.x,q.y,q.z,q.w]):
                    raise MissionError('nonfinite_goal')
                if abs(p.z)>.1 or abs(q.x)+abs(q.y)>1e-6 or abs(sum(v*v for v in [q.x,q.y,q.z,q.w])-1)>.01:
                    raise MissionError('invalid_goal_pose')
                start, _, stamp, age = self.pose(); now = time.monotonic()
                settings = MissionSettings(action_ack_timeout_s=args.ack_timeout)
                yaw = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
                self.planning_cancelled = False
                self.planning = self.worker.submit(make_contract, self.planner, start, [p.x,p.y], yaw,
                    mission_id=uuid.uuid4().hex, now=now, pose_stamp=now-age, settings=settings)
                self.status(planning=True)
            except (ValueError, TransformException) as error:
                self.event('rejected', reason=str(error)); self.status(rejection=str(error))

        def planned(self):
            future = self.planning; self.planning = None
            try:
                contract = future.result()
                if self.planning_cancelled:
                    self.event('rejected', reason='planning_cancelled'); return
                latest, _, last_stamp, last_age = self.pose()
                if math.dist(contract.start, latest)>1e-5:
                    raise MissionError('robot_moved_during_planning')
                now = time.monotonic(); self.offset = now-last_age-last_stamp
                contract = replace(contract, created_at=now, pose_stamp=last_stamp+self.offset)
                self.monitor = CrossingMonitor(contract, self.planner.leaves, self.planner.passage_segments)
                self.last_tf_stamp = last_stamp; self.samples = []; self.cancel_at = None
                effects = self.model.start(contract, now)
                path = NavPath(); path.header.frame_id = 'AGmap'; path.header.stamp=self.get_clock().now().to_msg()
                for x,y in contract.path:
                    pose=PoseStamped();pose.header=path.header;pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation.w=1.
                    path.poses.append(pose)
                self.path_pub.publish(path)
                self.event('mission', contract=contract.report()); self.effects(effects); self.status()
            except (ValueError, TransformException) as error:
                self.event('rejected', reason=str(error)); self.status(rejection=str(error))

        def tick(self):
            if not self.verified and not self.identity_pending and self.identity.service_is_ready():
                self.identity_pending = True
                self.identity.call_async(Trigger.Request()).add_done_callback(self.verify_identity)
            if self.planning is not None and self.planning.done():
                self.planned()
            if self.model.contract is None:
                self.status(); return
            try:
                self.effects(self.model.check_deadlines(time.monotonic()))
                if self.model.state in ['cancelling', 'failed'] and self.cancel_at is None:
                    self.cancel_at = time.monotonic()
                point, heading, stamp, _ = self.pose()
                if stamp != self.last_tf_stamp:
                    observed = stamp+self.offset; self.last_tf_stamp=stamp
                    self.samples.append({'sim_time':observed,'truth':point+[heading]})
                    self.samples=[p for p in self.samples if observed-p['sim_time']<3.]
                    if self.model.state in ['dispatching','executing','waiting_handoff','verifying']:
                        event = self.monitor.observe(point, observed)
                        if event:self.event('crossing', **event)
                        self.effects(self.model.advance(point, self.monitor.confirmed, time.monotonic()))
                if self.model.state == 'verifying':
                    self.model.confirm_final(point+[heading], rest_window(self.samples) is not None, time.monotonic())
                elif self.model.state in ['cancelling','failed']:
                    rest = rest_window(self.samples, earliest=self.cancel_at) if self.cancel_at is not None else None
                    self.model.confirm_stopped(rest is not None, time.monotonic())
                self.status()
            except (ValueError, TransformException) as error:
                self.fault('pose_watchdog:'+str(error))

    rclpy.init(args=rosargs); node=Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.worker.shutdown(wait=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
