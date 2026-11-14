#!/usr/bin/env python3
"""Native GCU window shifts using declared synthetic poses, no motion commands."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time
import numpy as np

from osmag_rolling_raster import RollingRaster,load_osm_geometry
from osmag_window_goal import select_window_goal,route_suffix
from osmag_constraint_mask import build_mask,ConstraintError
from osmag_window_admission import WindowAdmission
from gcu_plan_validation import validate_path


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();w=a.workspace
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from rclpy.action import ActionClient
    from rcl_interfaces.srv import GetParameters
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.msg import CostmapFilterInfo
    from nav2_msgs.action import ComputePathToPose
    from nav2_msgs.srv import ClearEntireCostmap
    from lifecycle_msgs.srv import ChangeState,GetState
    from geometry_msgs.msg import TransformStamped,Twist
    from tf2_ros import TransformBroadcaster
    rclpy.init();n=Node('gcu_rolling_native_fixture');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state={};commands=[];subs=[];receipts={}
    def receive(key,msg):state[key]=msg;receipts[key]=time.monotonic()
    for key,topic in [('global','/global_costmap/costmap'),('local','/window_local/global_costmap/costmap')]:
        subs.append(n.create_subscription(OccupancyGrid,topic,lambda m,k=key:receive(k,m),q))
    subs.append(n.create_subscription(Twist,'/cmd_vel',lambda m:commands.append([m.linear.x,m.angular.z]),10))
    pub=n.create_publisher(OccupancyGrid,'/osmag/planning_window',q)
    mask_pub=n.create_publisher(OccupancyGrid,'/osmag/window_constraint',q)
    info_pub=n.create_publisher(CostmapFilterInfo,'/osmag/window_filter_info',q)
    tf=TransformBroadcaster(n);fixture=json.loads((w/'fixture.json').read_text());pose=list(fixture['start'])
    def broadcast():
        m=TransformStamped();m.header.frame_id='map';m.child_frame_id='base_footprint';m.header.stamp=n.get_clock().now().to_msg()
        m.transform.translation.x=float(pose[0]);m.transform.translation.y=float(pose[1]);m.transform.rotation.w=1.;tf.sendTransform(m)
    timer=n.create_timer(.02,broadcast)
    areas,passages,incident,source=load_osm_geometry(w/'geometry.osm')
    renderer=RollingRaster(areas,passages,resolution=fixture['resolution'],anchor=fixture['anchor'],shape=tuple(fixture['shape']),source_sha256=source)
    started=[];result=dict(passed=False,scope=fixture['scope'],cases=[]);begin=time.monotonic()
    def wait(predicate,seconds=15):
        end=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>end:raise RuntimeError('native_window_timeout')
            rclpy.spin_once(n,timeout_sec=.01)
    def transition(name,code):
        c=n.create_client(ChangeState,'/'+name+'/change_state');wait(c.service_is_ready)
        req=ChangeState.Request();req.transition.id=code;f=c.call_async(req);wait(f.done)
        if not f.result().success:raise RuntimeError('lifecycle_failed:'+name+':'+str(code))
    def clear(name):
        c=n.create_client(ClearEntireCostmap,'/'+name+'/clear_entirely_'+name.split('/')[-1]);wait(c.service_is_ready)
        f=c.call_async(ClearEntireCostmap.Request());wait(f.done)
    def message(data,snapshot):
        m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg()
        m.info.resolution=snapshot.resolution;m.info.width=data.shape[1];m.info.height=data.shape[0]
        m.info.origin.position.x=snapshot.origin[0];m.info.origin.position.y=snapshot.origin[1];m.info.origin.orientation.w=1.
        m.data=data.astype(int).ravel().tolist();return m
    source_state={}
    def refresh_source():
        if not source_state:return
        window=source_state['window'];gate=source_state['gate']
        physical=message(window.grid,window);mask=message(source_state['mask'],window)
        pub.publish(physical);mask_pub.publish(mask)
        gate.note_source_publication(n.get_clock().now().nanoseconds/1e9,window.sha256)
        source_state['publications']+=1
    refresh_timer=n.create_timer(.1,refresh_source)
    client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
    try:
        sequence=[(d,False) for d in fixture['window_distances']]+[(0.,True),(0.,False)]
        for rev,(distance,closed) in enumerate(sequence,1):
            suffix=route_suffix(fixture['route']['path'],distance);pose[:]=suffix[0]
            window=renderer.render(pose,revision=rev,stamp=n.get_clock().now().nanoseconds/1e9)
            target=select_window_goal(window,suffix,pose)
            opened=[p for p in fixture['route']['passage_ids'] if not (closed and p==fixture['closure_passage'])]
            snap=build_mask(window.grid,window.resolution,window.origin,areas,passages,incident,
                allowed_areas=fixture['route']['area_ids'],open_passages=opened,
                disabled_passages=[fixture['closure_passage']] if closed else [],robot_pose=pose,
                mission_id='rolling_native',route_revision=1,closure_revision=int(closed),mask_revision=rev)
            physical=message(window.grid,window);msg=message(snap.mask,window)
            published=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            probes=[pose] if closed else [pose,target.point]
            gate=WindowAdmission(window,snap,published,probes)
            source_state.update(window=window,gate=gate,mask=snap.mask,publications=1)
            pub.publish(physical);mask_pub.publish(msg)
            info=CostmapFilterInfo();info.type=0;info.base=0.;info.multiplier=1.;info.filter_mask_topic='/osmag/window_constraint';info_pub.publish(info)
            if rev==1:
                for name in ['window_local/local_planner','planner_server']:
                    transition(name,1);transition(name,3);started.append(name)
            else:
                clear('global_costmap');clear('window_local/global_costmap')
            errors={}
            def consumed():
                now=n.get_clock().now().nanoseconds/1e9
                for key,msg in list(state.items()):
                    try:
                        data=np.asarray(msg.data).reshape(msg.info.height,msg.info.width)
                        origin=(msg.info.origin.position.x,msg.info.origin.position.y)
                        if key=='global' and (data.shape!=window.grid.shape or any(abs(x-y)>1e-7 for x,y in zip(origin,window.origin))):
                            raise RuntimeError('old_window_geometry')
                        gate.observe(key,data,msg.info.resolution,origin,msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                                     now,frame=msg.header.frame_id)
                        errors.pop(key,None)
                    except Exception as error:errors[key]=str(error);gate.observations.pop(key,None)
                return gate.ready(now)
            try:wait(consumed,22)
            except Exception:result['last_consumption_errors']=errors;raise
            admission=None;admission_error=None
            if closed:
                try:gate.admit(target,n.get_clock().now().nanoseconds/1e9)
                except ConstraintError as error:
                    assert str(error)=='proxy_prefix_blocked_by_constraint'
                    admission_error=str(error)
                else:raise RuntimeError('closed_proxy_wrongfully_admitted')
            else:admission=gate.admit(target,n.get_clock().now().nanoseconds/1e9)
            wait(client.server_is_ready)
            req=ComputePathToPose.Goal();req.planner_id='GridBased';req.use_start=True
            req.start.header.frame_id=req.goal.header.frame_id='map'
            req.start.pose.position.x=float(pose[0]);req.start.pose.position.y=float(pose[1]);req.start.pose.orientation.w=1.
            req.goal.pose.position.x=float(target.point[0]);req.goal.pose.position.y=float(target.point[1]);req.goal.pose.orientation.w=1.
            send=client.send_goal_async(req);wait(send.done);handle=send.result();assert handle.accepted
            f=handle.get_result_async();wait(f.done);reply=f.result();points=[[v.pose.position.x,v.pose.position.y] for v in reply.result.path.poses]
            row=dict(revision=rev,closed=closed,synthetic_pose=list(pose),window=window.report(),target=asdict(target),
                     consumption=gate.observations,admission=admission,admission_rejection=admission_error,
                     source_last_publication=gate.last_source_stamp,source_publications=source_state['publications'],
                     action_status=reply.status,path=points,costmaps={})
            arrays=dict(physical=window.grid,mask=snap.mask,combined=snap.combined)
            for key,msg in state.items():
                arrays[key]=np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
                row['costmaps'][key]=dict(shape=[msg.info.height,msg.info.width],resolution=msg.info.resolution,
                    origin=[msg.info.origin.position.x,msg.info.origin.position.y],frame=msg.header.frame_id,
                    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
            np.savez_compressed(a.output.parent/f'revision_{rev:02d}.npz',**arrays)
            if closed:assert reply.status==6 and not points
            else:
                assert reply.status==4 and points
                row['swept_validation']=validate_path(snap.combined,snap.resolution,snap.origin,points,.20)
                assert row['swept_validation']['valid'],row['swept_validation']
            row['passed']=True;result['cases'].append(row)
        actual={}
        for name in ['global_costmap/global_costmap','window_local/global_costmap/global_costmap']:
            pc=n.create_client(GetParameters,'/'+name+'/get_parameters');wait(pc.service_is_ready)
            request=GetParameters.Request();request.names=['static_layer.map_topic','plugins','filters']
            f=pc.call_async(request);wait(f.done)
            actual[name]=[dict(string=v.string_value,strings=list(v.string_array_value)) for v in f.result().values]
            assert actual[name][0]['string']=='/osmag/planning_window'
        wait(lambda:not n.get_subscriptions_info_by_topic('/map'))
        result.update(actual_parameters=actual,full_map_subscribers=len(n.get_subscriptions_info_by_topic('/map')),
                      commands=commands,native_window_subscribers=len(n.get_subscriptions_info_by_topic('/osmag/planning_window')),
                      cmd_vel_publishers=n.count_publishers('/cmd_vel'),passed=True)
        assert not commands and result['cmd_vel_publishers']==0 and result['native_window_subscribers']==2
    except Exception as error:
        result.update(passed=False,error=repr(error))
        arrays={key:np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
                for key,msg in state.items()}
        if 'window' in locals():
            arrays['physical']=window.grid;result['failed_window']=window.report()
        if 'snap' in locals():arrays['mask']=snap.mask;arrays['combined']=snap.combined
        if arrays:np.savez_compressed(a.output.parent/'failure_grids.npz',**arrays)
    finally:
        source_state.clear();refresh_timer.cancel()
        try:
            states={}
            for name in reversed(started):
                transition(name,4);transition(name,2)
                c=n.create_client(GetState,'/'+name+'/get_state');wait(c.service_is_ready)
                f=c.call_async(GetState.Request());wait(f.done)
                states[name]=f.result().current_state.label
            assert all(v=='unconfigured' for v in states.values())
            result['lifecycle_cleaned']=states
        except Exception as error:result.update(passed=False,shutdown_error=repr(error))
        result['elapsed_wall_s']=time.monotonic()-begin
        a.output.write_text(json.dumps(result,indent=2)+'\n');n.destroy_node();rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
