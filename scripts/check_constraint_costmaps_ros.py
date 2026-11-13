#!/usr/bin/env python3
"""Synthetic 06C1B: native global/local KeepoutFilters and ComputePathToPose.

No robot, controller, NavigateToPose or physics is launched. Saves raw grids.
"""
import argparse,json,math,time
from pathlib import Path
import numpy as np
from shapely.geometry import box
from osmag_constraint_mask import build_mask,ConsumptionGate
from gcu_plan_validation import validate_path,cell_at


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from rclpy.action import ActionClient
    from rclpy.time import Time
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.msg import CostmapFilterInfo
    from nav2_msgs.srv import ClearEntireCostmap
    from lifecycle_msgs.srv import ChangeState,GetState
    from nav2_msgs.action import ComputePathToPose
    from geometry_msgs.msg import TransformStamped,Twist
    from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster,Buffer,TransformListener
    rclpy.init();n=Node('constraint_costmap_fixture');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state={};commands=[];subs=[];receipts={'global':[],'local':[]}
    def receive(key,msg):
        state[key]=msg
        receipts[key].append({'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                              'received_at':n.get_clock().now().nanoseconds/1e9})
    for key,topic in [('global','/global_costmap/costmap'),('local','/costmap/costmap')]:
        subs.append(n.create_subscription(OccupancyGrid,topic,lambda m,k=key:receive(k,m),q))
    subs.append(n.create_subscription(Twist,'/cmd_vel',lambda m:commands.append(m),10))
    map_pub=n.create_publisher(OccupancyGrid,'/map',q);mask_pub=n.create_publisher(OccupancyGrid,'/constraint_mask',q)
    info_pub=n.create_publisher(CostmapFilterInfo,'/constraint_filter_info',q)
    dynamic=TransformBroadcaster(n);static=StaticTransformBroadcaster(n);buffer=Buffer();listener=TransformListener(buffer,n)
    tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.transform.rotation.w=1.;static.sendTransform(tf)
    def pose():
        tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='base_footprint';tf.header.stamp=n.get_clock().now().to_msg()
        tf.transform.translation.x=-1.;tf.transform.rotation.w=1.;dynamic.sendTransform(tf)
    timer=n.create_timer(.02,pose)
    grid=np.zeros((160,160),dtype=np.int16);grid[[0,-1],:]=100;grid[:,[0,-1]]=100;grid[100:110,110:115]=100
    res=.05;origin=(-4.,-4.);areas={'a':box(-2,-2,0,2),'b':box(0,-2,2,2)}
    portals={'p':[(0,-1.5),(0,1.5)]};incidence={'p':('a','b')}
    def message(data):
        msg=OccupancyGrid();msg.header.frame_id='map';msg.header.stamp=n.get_clock().now().to_msg()
        msg.info.resolution=res;msg.info.width=data.shape[1];msg.info.height=data.shape[0]
        msg.info.origin.position.x=origin[0];msg.info.origin.position.y=origin[1];msg.info.origin.orientation.w=1.
        msg.data=data.astype(int).ravel().tolist();return msg
    map_pub.publish(message(grid))
    info=CostmapFilterInfo();info.type=0;info.base=0.;info.multiplier=1.;info.filter_mask_topic='/constraint_mask'
    info_pub.publish(info)
    client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
    def wait(predicate,seconds=30):
        end=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>end:raise RuntimeError('native_costmap_check_timeout')
            rclpy.spin_once(n,timeout_sec=.02)
    def transition(name,code):
        client=n.create_client(ChangeState,'/'+name+'/change_state');wait(client.service_is_ready,5)
        req=ChangeState.Request();req.transition.id=code;future=client.call_async(req);wait(future.done,8)
        if not future.result().success:raise RuntimeError('lifecycle_transition_failed:'+name+':'+str(code))
    def refresh(name):
        client=n.create_client(ClearEntireCostmap,'/'+name+'/clear_entirely_'+name.split('/')[0])
        wait(client.service_is_ready,5);future=client.call_async(ClearEntireCostmap.Request());wait(future.done,8)
    started=[]
    result={'passed':False,'scope':'Synthetic native plan-only costmaps, no controller/physics/motion', 'cases':[]}
    def save_grids(name,snap,published):
        arrays={'mask':snap.mask,'physical':grid};metadata={'mask_published_at':published,'snapshot':snap.report(),'costmaps':{}}
        for key,msg in state.items():
            arrays[key]=np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
            metadata['costmaps'][key]={'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                'resolution':msg.info.resolution,'origin':[msg.info.origin.position.x,msg.info.origin.position.y],
                'frame':msg.header.frame_id,'last_receipt':receipts[key][-1],'message_count':len(receipts[key])}
        np.savez_compressed(a.output.parent/(name+'.npz'),**arrays)
        (a.output.parent/(name+'.json')).write_text(json.dumps(metadata,indent=2)+'\n')
    begin=time.monotonic()
    try:
        for rev,closed in [(1,False),(2,True),(3,False)]:
            snap=build_mask(grid,res,origin,areas,portals,incidence,allowed_areas=['a','b'],
                open_passages=[] if closed else ['p'],disabled_passages=['p'] if closed else [],robot_pose=(-1,0),
                mission_id='native_fixture',mask_revision=rev,closure_revision=rev-1)
            msg=message(snap.mask);published=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9;mask_pub.publish(msg)
            probes=[(-1,0),(1,0)] if closed else [(-1,0),(0,0),(1,0)]
            gate=ConsumptionGate(snap,published,probes);last_error={}
            if rev==1:
                for name in ['costmap/costmap','planner_server']:
                    transition(name,1);transition(name,3);started.append(name)
            else:
                # Humble 1.1.19 KeepoutFilter does not dirty static global bounds
                # when a new mask arrives. Reset while motion is held, then
                # verify the newly subscribed mask/static layers before planning.
                refresh('global_costmap');refresh('costmap')
            def consumed():
                now=n.get_clock().now().nanoseconds/1e9
                for key,msg in list(state.items()):
                    try:
                        x=msg.info.origin.position.x;y=msg.info.origin.position.y
                        if msg.header.frame_id=='odom':
                            t=buffer.lookup_transform('map','odom',Time()).transform
                            assert abs(t.rotation.x)+abs(t.rotation.y)+abs(t.rotation.z)<1e-9 and abs(t.rotation.w-1)<1e-9
                            x+=t.translation.x;y+=t.translation.y
                        else:assert msg.header.frame_id=='map'
                        data=np.asarray(msg.data).reshape(msg.info.height,msg.info.width)
                        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                        gate.observe(key,data,msg.info.resolution,(x,y),stamp,now)
                        last_error.pop(key,None)
                    except Exception as e:last_error[key]=str(e)
                return gate.ready(now)
            try:wait(consumed,35)
            except Exception:
                result['last_consumption_errors']=last_error
                save_grids(f'failed_revision_{rev}',snap,published)
                raise
            row={'revision':rev,'closed':closed,'mask_published_at':published,'consumed':gate.observations,'snapshot':snap.report(),'costmaps':{}}
            arrays={'mask':snap.mask,'physical':grid}
            for key,msg in state.items():
                arrays[key]=np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
                row['costmaps'][key]={'resolution':msg.info.resolution,'origin':[msg.info.origin.position.x,msg.info.origin.position.y],
                    'frame':msg.header.frame_id,'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9}
            np.savez_compressed(a.output.parent/f'revision_{rev}.npz',**arrays)
            wait(client.server_is_ready)
            req=ComputePathToPose.Goal();req.planner_id='GridBased';req.use_start=True
            req.start.header.frame_id=req.goal.header.frame_id='map';req.start.pose.position.x=-1.;req.start.pose.orientation.w=1.
            req.goal.pose.position.x=1.;req.goal.pose.orientation.w=1.
            send=client.send_goal_async(req);wait(send.done);handle=send.result();assert handle.accepted
            pending=handle.get_result_async();wait(pending.done);reply=pending.result()
            points=[[p.pose.position.x,p.pose.position.y] for p in reply.result.path.poses]
            row.update(action_status=reply.status,path=points)
            if closed:assert reply.status==6 and not points
            else:
                assert reply.status==4 and points
                row['swept_validation']=validate_path(snap.combined,res,origin,points,.20)
                assert row['swept_validation']['valid']
            row['physical_wall_global']=cell_at(arrays['global'],res,origin,1.6,1.1)
            assert row['physical_wall_global']==100
            row['passed']=True;result['cases'].append(row)
        result['velocity_publishers']={t:n.count_publishers(t) for t in ['/cmd_vel','/cmd_vel_nav']}
        result['navigation_status_publishers']=n.count_publishers('/navigate_to_pose/_action/status')
        result['command_messages']=len(commands)
        assert not any(result['velocity_publishers'].values()) and not commands and result['navigation_status_publishers']==0
        result['passed']=True
    except Exception as error:result['error']=repr(error)
    finally:
        # Stop/join update threads before SIGINT. Leave inactive objects intact
        # for normal destruction; on_cleanup previously raced plugin callbacks.
        result['lifecycle_shutdown']={'passed':False}
        try:
            states={}
            for name in reversed(started):
                transition(name,4)
                client=n.create_client(GetState,'/'+name+'/get_state');wait(client.service_is_ready,3)
                future=client.call_async(GetState.Request());wait(future.done,5)
                states[name]=future.result().current_state.label
            result['lifecycle_shutdown']={'passed':len(states)==2 and all(v=='inactive' for v in states.values()),'states':states}
        except Exception as e:result['lifecycle_shutdown']['error']=repr(e)
        result['passed']=bool(result['passed'] and result['lifecycle_shutdown']['passed'])
        result['costmap_receipts']=receipts
        result['elapsed_wall_s']=time.monotonic()-begin
        a.output.write_text(json.dumps(result,indent=2)+'\n');n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
