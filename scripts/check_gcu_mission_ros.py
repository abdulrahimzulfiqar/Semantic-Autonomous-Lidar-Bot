#!/usr/bin/env python3
"""06B1 mock NavigateToPose server + synthetic TF + ROS interface checks.

No simulator, robot description, controller or velocity publisher is created.
"""
import argparse
import json
import math
from pathlib import Path
import threading
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--namespace', required=True);p.add_argument('--mock-token',required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionServer, GoalResponse, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from nav_msgs.msg import Path as NavPath
    from std_msgs.msg import Empty, String
    from std_srvs.srv import Trigger
    from tf2_ros import TransformBroadcaster
    prefix=args.namespace; start=[-.05864395131342248,-12.960313240266368]
    class Fixture(Node):
        def __init__(self):
            super().__init__('gcu_mission_mock_fixture')
            self.group=ReentrantCallbackGroup();self.point=list(start);self.yaw=0.;self.tf_enabled=True
            self.statuses=[];self.events=[];self.server_events=[];self.commands=[];self.paths=[]
            self.mode='normal';self.latest=None;self.abort_ids=set();self.stop=False;self.correct_identity=False
            self.tf=TransformBroadcaster(self);self.tf_timer=self.create_timer(.02,self.publish_tf)
            self.identity=self.create_service(Trigger,prefix+'/identity',self.identify,callback_group=self.group)
            self.server=ActionServer(self,NavigateToPose,prefix+'/navigate_to_pose',
                execute_callback=self.execute,goal_callback=self.goal,cancel_callback=self.cancel,
                callback_group=self.group)
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subs=[self.create_subscription(String,prefix+'/mission_status',lambda m:self.statuses.append(json.loads(m.data)),qos),
                       self.create_subscription(String,prefix+'/events',lambda m:self.events.append(json.loads(m.data)),100),
                       self.create_subscription(NavPath,prefix+'/mission_path',lambda m:self.paths.append(m),qos),
                       self.create_subscription(Twist,'/cmd_vel',lambda m:self.commands.append(m),10)]
            self.pub=self.create_publisher(PoseStamped,prefix+'/mission_goal',10)
            self.cancel_pub=self.create_publisher(Empty,prefix+'/mission_cancel',10)

        def publish_tf(self):
            if not self.tf_enabled:return
            t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_footprint'
            t.header.stamp=self.get_clock().now().to_msg();t.transform.translation.x=float(self.point[0]);t.transform.translation.y=float(self.point[1])
            t.transform.rotation.z=math.sin(self.yaw/2);t.transform.rotation.w=math.cos(self.yaw/2)
            self.tf.sendTransform(t)

        def identify(self,req,response):
            response.success=True
            response.message=json.dumps({'token':args.mock_token if self.correct_identity else 'wrong',
                'scope':'mock_navigator_no_actuators','action':prefix+'/navigate_to_pose'})
            return response

        def goal(self,request):
            mode=self.mode;self.mode='normal'
            self.server_events.append({'event':'request','at':time.monotonic(),'mode':mode,
                'point':[request.pose.pose.position.x,request.pose.pose.position.y]})
            if mode=='reject':return GoalResponse.REJECT
            if self.latest is not None:
                self.abort_ids.add(self.latest)
                time.sleep(.08)  # Old result before next acceptance is intentional.
            if mode=='delay':time.sleep(.6)
            if mode=='timeout':time.sleep(1.5)
            return GoalResponse.ACCEPT

        def cancel(self,handle):
            self.server_events.append({'event':'cancel','at':time.monotonic(),'uuid':bytes(handle.goal_id.uuid).hex()})
            return CancelResponse.ACCEPT

        def execute(self,handle):
            ident=bytes(handle.goal_id.uuid).hex();self.latest=ident
            self.server_events.append({'event':'execute','at':time.monotonic(),'uuid':ident})
            target=handle.request.pose.pose.position
            # The fixture moves TF independently. An action result alone cannot move anything.
            while not self.stop:
                if handle.is_cancel_requested:
                    handle.canceled();break
                if ident in self.abort_ids:
                    handle.abort();break
                if math.dist(self.point,[target.x,target.y])<.015:
                    handle.succeed();break
                time.sleep(.01)
            else:
                handle.abort()
            self.server_events.append({'event':'terminal','at':time.monotonic(),'uuid':ident})
            if self.latest==ident:self.latest=None
            return NavigateToPose.Result()

    rclpy.init();node=Fixture();executor=MultiThreadedExecutor(num_threads=6);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    result={'passed':False,'scope':'06B1 ROS mock only; synthetic TF and fabricated navigator; no physical motion',
            'cases':[],'gazebo_started':False,'controller_started':False}
    def wait(predicate,seconds=15):
        end=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>end:raise RuntimeError('mock_check_timeout')
            time.sleep(.015)
    def goal(point,frame='map'):
        m=PoseStamped();m.header.frame_id=frame;m.pose.position.x=float(point[0]);m.pose.position.y=float(point[1]);m.pose.orientation.w=1.
        node.pub.publish(m)
    def last():return node.statuses[-1] if node.statuses else {}
    def settled():return last().get('state') in ['cancelled','succeeded','failed'] and last().get('unresolved_requests')==0 and last().get('stop_confirmed')
    def reset():
        if last().get('state') not in ['idle','cancelled','succeeded','failed'] or last().get('unresolved_requests'):
            node.cancel_pub.publish(Empty());wait(settled)
        node.point=list(start);node.yaw=0.;node.tf_enabled=True;time.sleep(.25)
    def begin(point, mode='normal'):
        reset();event_index=len(node.events);node.mode=mode;goal(point)
        wait(lambda:any(e['event']=='send' for e in node.events[event_index:]))
        return event_index
    def record(name,check):
        check();result['cases'].append({'name':name,'passed':True})
    try:
        wait(lambda:bool(node.statuses) and node.count_subscribers(prefix+'/mission_goal')==1,90)
        def identity_guard():
            i=len(node.events);n=len(node.server_events);goal([start[0]-.8,start[1]])
            wait(lambda:any(e.get('reason')=='mock_identity_unverified' for e in node.events[i:]))
            assert len(node.server_events)==n
        record('unverified_mock_identity_rejected_without_action',identity_guard)
        node.correct_identity=True;wait(lambda:last().get('mock_verified'))
        def unsupported():
            i=len(node.events);n=len(node.server_events);goal([1.,-12.],'odom')
            wait(lambda:any(e.get('reason')=='goal_frame_must_be_map' for e in node.events[i:]))
            assert len(node.server_events)==n
        record('unsupported_frame_rejected_without_action',unsupported)
        def courtyard():
            i=len(node.events);goal([0.,0.])
            wait(lambda:any(e.get('reason')=='unknown_goal' for e in node.events[i:]))
        record('courtyard_rejected',courtyard)
        def success():
            i=begin([start[0]+1.5,start[1]])
            wait(lambda:len([e for e in node.events[i:] if e['event']=='send'])>=2)
            for j in range(1,76):
                node.point=[start[0]+j*.02,start[1]];time.sleep(.035)
            wait(lambda:last().get('state')=='succeeded')
            events=node.events[i:];assert len([e for e in events if e['event']=='crossing'])==1
            assert any(e['event']=='result' and e['outcome']=='aborted' for e in events)
            assert all(e.get('confirmed',0)==0 for e in events if e['event']=='send')
            assert node.paths and node.paths[-1].header.frame_id=='AGmap'
        record('continuous_handoff_crossing_and_final_rest',success)
        def busy_and_cancel():
            i=begin([start[0]+1.5,start[1]])
            wait(lambda:last().get('state')=='executing')
            goal([-3.,-10.5]);wait(lambda:any(e.get('reason')=='busy_cancel_before_new_mission' for e in node.events[i:]))
            node.cancel_pub.publish(Empty());wait(settled)
            assert last()['state']=='cancelled'
        record('busy_goal_and_whole_mission_cancel',busy_and_cancel)
        def late():
            i=begin([start[0]-.8,start[1]],'delay')
            node.cancel_pub.publish(Empty());wait(settled)
            assert last()['state']=='cancelled'
            events=node.events[i:]
            assert any(e['event']=='accepted' and e['accepted'] for e in events)
            assert any(e['event']=='cancel_request' for e in events)
            assert len([e for e in events if e['event']=='send'])==1
        record('cancel_pending_send_cancels_late_acceptance',late)
        def rejected():
            begin([start[0]-.8,start[1]],'reject');wait(settled)
            assert last()['state']=='failed' and last()['reason']=='navigation_action_rejected'
        record('navigator_rejection_is_terminal_failure',rejected)
        def timeout():
            begin([start[0]-.8,start[1]],'timeout')
            wait(lambda:last().get('state')=='failed' and last().get('reason')=='action_ack_timeout')
            assert not last()['stop_confirmed']
            wait(settled);assert last()['state']=='failed'
        record('ack_timeout_keeps_stop_unknown_until_resolved',timeout)
        def stale():
            begin([start[0]-.8,start[1]])
            wait(lambda:last().get('state')=='executing');node.tf_enabled=False
            wait(lambda:last().get('state')=='cancelling')
            assert last()['reason'].startswith('pose_watchdog:')
            node.tf_enabled=True;wait(settled)
            assert last()['state']=='failed'
        record('stale_tf_cancels_and_requires_fresh_rest',stale)
        def absent_navigator():
            reset();node.server.destroy();node.server=None
            wait(lambda:node.count_publishers(prefix+'/navigate_to_pose/_action/status')==0)
            time.sleep(.2);i=len(node.events);goal([start[0]-.8,start[1]])
            wait(lambda:any(e.get('reason')=='navigator_not_ready' for e in node.events[i:]))
        record('absent_navigator_rejected_without_action',absent_navigator)
        result['velocity_publishers']={topic:node.count_publishers(topic) for topic in ['/cmd_vel','/cmd_vel_nav']}
        result['real_navigation_status_publishers']=node.count_publishers('/navigate_to_pose/_action/status')
        result['command_messages']=len(node.commands)
        assert not any(result['velocity_publishers'].values()) and not node.commands
        assert result['real_navigation_status_publishers']==0
        result['passed']=True
    except Exception as error:
        result['error']=repr(error)
    finally:
        node.stop=True;time.sleep(.1)
        result.update(adapter_events=node.events,server_events=node.server_events,statuses=node.statuses)
        args.output.write_text(json.dumps(result,indent=2)+'\n')
        executor.shutdown(timeout_sec=3)
        if node.server is not None:node.server.destroy()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        thread.join(timeout=3)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
