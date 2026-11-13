#!/usr/bin/env python3
"""Bounded ROS client for the actual simulation review interface; no cmd_vel."""
import argparse,json,math,os,time
from pathlib import Path
import xml.etree.ElementTree as ET


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--faults-only',action='store_true');a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='195' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('isolated_simulation_domain_required')
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Empty,String
    from rosgraph_msgs.msg import Clock
    from lifecycle_msgs.srv import ChangeState
    rclpy.init();node=Node('gcu_review_interface_client',parameter_overrides=[Parameter('use_sim_time',value=True)])
    statuses=[];events=[];paths=[];clock=[0.];checks=[];begin=time.monotonic()
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    from nav_msgs.msg import Path as NavPath
    subs=[node.create_subscription(String,'/osmag/session_status',lambda m:statuses.append(json.loads(m.data)),qos),
        node.create_subscription(String,'/osmag/passage_progress',lambda m:events.append(json.loads(m.data)),100),
        node.create_subscription(NavPath,'/osmag/mission_path',lambda m:paths.append(len(m.poses)),qos),
        node.create_subscription(Clock,'/clock',lambda m:clock.__setitem__(0,m.clock.sec+m.clock.nanosec/1e9),10)]
    goal_pub=node.create_publisher(PoseStamped,'/goal_pose',10);cancel_pub=node.create_publisher(Empty,'/osmag/mission_cancel',10)
    stop_pub=node.create_publisher(Empty,'/osmag/session_stop',10);close_pub=node.create_publisher(String,'/osmag/passage_closures',10)
    result={'passed':False,'scope':'Actual isolated simulation interface; goals/cancel/lifecycle only, no velocity publisher','checks':checks}
    def wait(predicate,seconds=90):
        end=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>end or time.monotonic()-begin>500:raise RuntimeError('review_client_timeout')
            rclpy.spin_once(node,timeout_sec=.02)
    def goal(x,y,yaw=0.,frame='map'):
        m=PoseStamped();m.header.frame_id=frame;m.header.stamp=node.get_clock().now().to_msg();m.pose.position.x=float(x);m.pose.position.y=float(y);m.pose.orientation.z=math.sin(yaw/2);m.pose.orientation.w=math.cos(yaw/2);goal_pub.publish(m)
    def state_after(mark,state,seconds=90):wait(lambda:any(v['state']==state for v in statuses[mark:]),seconds)
    def lifecycle(code):
        c=node.create_client(ChangeState,'/bt_navigator/change_state');wait(c.service_is_ready,10)
        req=ChangeState.Request();req.transition.id=code;f=c.call_async(req);wait(f.done,15);assert f.result().success;node.destroy_client(c)
    try:
        wait(lambda:any(v['state']=='ready' for v in statuses),70)
        assert not any(e['event']=='send' for e in events);checks.append('no_automatic_goal_before_click')
        if a.faults_only:
            y=-12.960313240266368;mark=len(statuses);goal(-.3,y);state_after(mark,'succeeded',90);checks.append('short_same_sector_arrival')
            lifecycle(4);mark=len(statuses);goal(1.4413560486865775,y)
            wait(lambda:any(v['state'] in ['failed','rejected'] for v in statuses[mark:]),75)
            failure=next(v for v in reversed(statuses[mark:]) if v['state'] in ['failed','rejected'])
            assert any(s in str(failure.get('reason')) for s in ['navigator','navigation_action_rejected','execution_owner','preflight_timeout']),failure
            checks.append('inactive_navigator_rejected_without_motion')
            lifecycle(3);mark=len(statuses);goal(-.3,y);state_after(mark,'succeeded',90);checks.append('recovered_after_navigator_reactivation')
            ids=[w.get('id') for w in ET.parse(a.workspace/'assets/gcu_ground_floor.osm').getroot().findall('way') if any(t.get('k')=='osmAG:type' and t.get('v')=='passage' for t in w.findall('tag'))]
            before=sum(e['event']=='send' for e in events);close_pub.publish(String(data=json.dumps({'revision':1,'disabled_passages':ids})))
            wait(lambda:any(e['event']=='closure_received' for e in events),15)
            mark=len(statuses);goal(1.4413560486865775,y);state_after(mark,'rejected',30)
            assert sum(e['event']=='send' for e in events)==before;checks.append('all_passages_closed_rejects_without_dispatch')
            result.update(passed=True,statuses=statuses);return 0
        for frame,x,y in [('odom',1.,-12.96),('map',1e6,1e6),('map',0.,0.)]:
            mark=len(statuses);goal(x,y,frame=frame);state_after(mark,'rejected',20)
            assert not any(e['event']=='send' for e in events)
        checks.append('wrong_frame_outside_and_courtyard_goals_rejected_without_dispatch')
        import yaml,numpy as np
        from PIL import Image
        cfg=yaml.safe_load((a.workspace/'assets/gcu_ground_floor.yaml').read_text());grid=np.flipud(np.asarray(Image.open(a.workspace/'assets'/cfg['image'])))
        row,col=np.argwhere(grid<20)[0];mark=len(statuses);goal(cfg['origin'][0]+(col+.5)*cfg['resolution'],cfg['origin'][1]+(row+.5)*cfg['resolution']);state_after(mark,'rejected',20)
        assert not any(e['event']=='send' for e in events);checks.append('occupied_goal_rejected_without_dispatch')
        y=-12.960313240266368;mark=len(statuses);goal(1.4413560486865775,y);state_after(mark,'running',30)
        mark=len(statuses);goal(.8,y);state_after(mark,'rejected',20);checks.append('busy_goal_rejected')
        wait(lambda:any(e['event']=='send' for e in events) and any(paths),45)
        first=next(e for e in events if e['event']=='send');wait(lambda:clock[0]>=first['sim']+11,55)
        mark=len(statuses);cancel_pub.publish(Empty());state_after(mark,'cancelled',30);checks.append('cancel_topic_stops_active_mission')
        mark=len(statuses);goal(1.4413560486865775,y);state_after(mark,'succeeded',140);checks.append('recover_after_cancel_and_arrive')
        mark=len(statuses);goal(-.5,y,math.pi);state_after(mark,'succeeded',180);checks.append('consecutive_return_with_reversed_heading')
        lifecycle(4);mark=len(statuses);goal(1.4413560486865775,y);state_after(mark,'failed',75);checks.append('inactive_navigator_does_not_start_motion')
        lifecycle(3);mark=len(statuses);goal(.5,y);state_after(mark,'succeeded',150);checks.append('recover_after_navigator_reactivation')
        ids=[w.get('id') for w in ET.parse(a.workspace/'assets/gcu_ground_floor.osm').getroot().findall('way') if any(t.get('k')=='osmAG:type' and t.get('v')=='passage' for t in w.findall('tag'))]
        assert ids;before=sum(e['event']=='send' for e in events);close_pub.publish(String(data=json.dumps({'revision':1,'disabled_passages':ids})))
        wait(lambda:any(e['event']=='closure_received' for e in events),15)
        mark=len(statuses);goal(-.5,y,math.pi);state_after(mark,'rejected',30)
        assert sum(e['event']=='send' for e in events)==before;checks.append('all_passages_closed_rejects_goal_without_dispatch')
        result.update(passed=True,preview_messages=len(paths),statuses=statuses)
    except Exception as error:result['error']=repr(error)
    finally:
        stop_pub.publish(Empty())
        until=time.monotonic()+.5
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        result['elapsed_wall_s']=time.monotonic()-begin;a.output.write_text(json.dumps(result,indent=2)+'\n');node.destroy_node();rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
