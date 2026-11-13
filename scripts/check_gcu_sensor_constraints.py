#!/usr/bin/env python3
"""06B2A stationary real GCU sensors, stamped TF and refreshed native filters.

No navigation action is sent. A passed result is not motion authorization.
"""
import argparse,hashlib,json,math,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from gcu_costmap_observation import inspect
from gcu_plan_validation import validate_path


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.parameter import Parameter
    from rclpy.action import ActionClient
    from nav_msgs.msg import OccupancyGrid,Odometry
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist
    from nav2_msgs.msg import CostmapFilterInfo
    from nav2_msgs.srv import ClearEntireCostmap
    from nav2_msgs.action import ComputePathToPose
    from lifecycle_msgs.srv import ChangeState,GetState
    from tf2_ros import Buffer,TransformListener
    rclpy.init();n=Node('gcu_sensor_constraint_check',parameter_overrides=[Parameter('use_sim_time',value=True)])
    meta=json.loads((a.workspace/'fixture/fixture.json').read_text());arrays=np.load(a.workspace/'fixture/fixture.npz')
    snap=SimpleNamespace(**{k:v for k,v in meta['snapshot'].items() if k not in ['frame','shape','scope','blocked_cells']});snap.mask=arrays['mask']
    state={};scans=[];commands=[];odometry=[];subs=[];buffer=Buffer();listener=TransformListener(buffer,n)
    q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    def now():return n.get_clock().now().nanoseconds/1e9
    for key,topic in [('global','/global_costmap/costmap'),('local','/local_costmap/costmap')]:
        subs.append(n.create_subscription(OccupancyGrid,topic,lambda m,k=key:state.update({k:m}),q))
    subs.append(n.create_subscription(LaserScan,'/scan',lambda m:scans.append({'stamp':stamp(m),'received':now(),'count':len(m.ranges),'finite':int(np.isfinite(m.ranges).sum()),'frame':m.header.frame_id}),qos_profile_sensor_data))
    subs.append(n.create_subscription(Twist,'/cmd_vel',lambda m:commands.append([now(),m.linear.x,m.linear.y,m.angular.z]),10))
    subs.append(n.create_subscription(Odometry,'/odometry/filtered',lambda m:odometry.append([stamp(m),m.twist.twist.linear.x,m.twist.twist.angular.z]),10))
    mask_pub=n.create_publisher(OccupancyGrid,'/osmag/constraint_mask',q)
    info_pub=n.create_publisher(CostmapFilterInfo,'/osmag/constraint_filter_info',q)
    report={'passed':False,'scope':'06B2A stationary real GCU sensor/costmap integration; no navigator or mission dispatch','revisions':[]}
    started=[];begin=time.monotonic()
    def wait(predicate,seconds=30):
        end=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>end:raise RuntimeError('sensor_constraint_timeout')
            rclpy.spin_once(n,timeout_sec=.02)
    def call(kind,name,req):
        client=n.create_client(kind,name);wait(client.service_is_ready,15);future=client.call_async(req);wait(future.done,15)
        return future.result()
    def transition(name,code):
        req=ChangeState.Request();req.transition.id=code
        if not call(ChangeState,'/'+name+'/change_state',req).success:raise RuntimeError('transition_failed:'+name+':'+str(code))
    def transform(frame,stamp_msg):
        if frame=='map':return [0.,0.,0.]
        t=buffer.lookup_transform('map',frame,Time.from_msg(stamp_msg)).transform
        r=t.rotation
        if abs(r.x)+abs(r.y)>1e-5:raise RuntimeError('non_planar_costmap_transform')
        return [t.translation.x,t.translation.y,math.atan2(2*r.w*r.z,1-2*r.z*r.z)]
    def save(rev,published,observed):
        saved={'mask':snap.mask,'physical':arrays['physical']};details={}
        for key,msg in state.items():
            saved[key]=np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
            details[key]={'stamp':stamp(msg),'resolution':msg.info.resolution,'origin':[msg.info.origin.position.x,msg.info.origin.position.y],
                          'frame':msg.header.frame_id,'map_from_grid':observed.get(key,{}).get('map_from_grid')}
        np.savez_compressed(a.output.parent/('revision_'+str(rev)+'.npz'),**saved)
        return {'revision':rev,'published_at':published,'costmaps':details,'observed':observed,
                'mask_array_sha256':hashlib.sha256(snap.mask.tobytes()).hexdigest()}
    try:
        wait(lambda:now()>0 and len(scans)>=3 and odometry and buffer.can_transform('map','base_footprint',Time()),50)
        t=buffer.lookup_transform('map','base_footprint',Time()).transform.translation
        report['localized_start']=[t.x,t.y]
        report['start_footprint']=validate_path(arrays['combined'],snap.resolution,snap.origin,[[t.x,t.y]],.20)
        if not report['start_footprint']['valid']:raise RuntimeError('fresh_start_outside_constraint')
        for revision in [1,2]:
            # Revision two adds a declared off-route test exclusion inside the
            # rolling grid, proving its filter check is not vacuous.
            snap.mask=arrays['mask' if revision==1 else 'mask_after_refresh']
            combined=arrays['combined' if revision==1 else 'combined_after_refresh']
            msg=OccupancyGrid();msg.header.frame_id='map';msg.header.stamp=n.get_clock().now().to_msg()
            msg.info.resolution=snap.resolution;msg.info.width=snap.mask.shape[1];msg.info.height=snap.mask.shape[0]
            msg.info.origin.position.x=snap.origin[0];msg.info.origin.position.y=snap.origin[1];msg.info.origin.orientation.w=1.
            msg.data=snap.mask.ravel().astype(int).tolist();published=stamp(msg)
            mask_pub.publish(msg)
            info=CostmapFilterInfo();info.type=0;info.base=0.;info.multiplier=1.;info.filter_mask_topic='/osmag/constraint_mask';info_pub.publish(info)
            if revision==1:
                for name in ['planner_server','controller_server']:
                    transition(name,1);transition(name,3);started.append(name)
            else:
                # No NavigateToPose server or FollowPath client exists in this
                # checkpoint. Reset while stationary; never do this under motion.
                for name in ['global_costmap','local_costmap']:
                    call(ClearEntireCostmap,'/'+name+'/clear_entirely_'+name,ClearEntireCostmap.Request())
            refresh_finished=now();observed={};errors={}
            def ready():
                observed.clear();current=now()
                for key,m in list(state.items()):
                    try:
                        if not refresh_finished<=stamp(m)<=current or current-stamp(m)>.5:raise RuntimeError('costmap_not_fresh_after_refresh')
                        rotation=m.info.origin.orientation
                        if abs(rotation.x)+abs(rotation.y)+abs(rotation.z)>1e-8:raise RuntimeError('rotated_grid_origin')
                        data=np.asarray(m.data).reshape(m.info.height,m.info.width)
                        observed[key]=inspect(snap,data,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y),
                            transform(m.header.frame_id,m.header.stamp),[report['localized_start'],meta['goal']],full=key=='global')|{'stamp':stamp(m)}
                        errors.pop(key,None)
                    except Exception as e:errors[key]=str(e)
                recent=[s for s in scans if s['stamp']>=refresh_finished and s['received']>=refresh_finished]
                return set(observed)=={'global','local'} and len(recent)>=3 and 0<=current-recent[-1]['stamp']<=.3
            try:wait(ready,50)
            except Exception:
                report['consumption_errors']=errors;report['failed_revision']=save(revision,published,observed);raise
            row=save(revision,published,observed);recent=[s for s in scans if s['stamp']>=refresh_finished and s['received']>=refresh_finished]
            row.update(refresh_finished=refresh_finished,scan_count_after_refresh=len(recent),latest_scan_age=now()-recent[-1]['stamp'],passed=True)
            # PlannerServer also waits for its native costmap to be current.
            # This adds a native sensor-layer gate beyond observing /scan alone.
            client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');wait(client.server_is_ready,10)
            req=ComputePathToPose.Goal();req.planner_id='GridBased';req.use_start=False
            req.goal.header.frame_id='map';req.goal.header.stamp=n.get_clock().now().to_msg()
            req.goal.pose.position.x=meta['goal'][0];req.goal.pose.position.y=meta['goal'][1];req.goal.pose.orientation.w=1.
            future=client.send_goal_async(req);wait(future.done,10);handle=future.result()
            if not handle.accepted:raise RuntimeError('native_plan_rejected')
            future=handle.get_result_async();wait(future.done,15);reply=future.result()
            points=[[p.pose.position.x,p.pose.position.y] for p in reply.result.path.poses]
            row['plan']={'status':reply.status,'points':points,'swept_validation':validate_path(combined,snap.resolution,snap.origin,points,.20)}
            if reply.status!=4 or not points or not row['plan']['swept_validation']['valid']:raise RuntimeError('native_plan_not_footprint_feasible')
            if revision==2 and observed['local']['required_cells']==0:raise RuntimeError('vacuous_local_filter_check')
            report['revisions'].append(row)
        report['navigate_status_publishers']=n.count_publishers('/navigate_to_pose/_action/status')
        report['scan_publishers']=[e.node_name for e in n.get_publishers_info_by_topic('/scan')]
        if report['navigate_status_publishers'] or len(report['scan_publishers'])!=1:raise RuntimeError('unexpected_runtime_owner')
        if any(any(abs(v)>1e-10 for v in row[1:]) for row in commands):raise RuntimeError('unexpected_motion_command')
        if not odometry or max(abs(v) for x in odometry for v in x[1:])>.01:raise RuntimeError('robot_not_stationary')
        report['passed']=True
    except Exception as error:report['error']=repr(error)
    finally:
        report['lifecycle_states']={}
        try:
            for name in reversed(started):
                transition(name,4);report['lifecycle_states'][name]=call(GetState,'/'+name+'/get_state',GetState.Request()).current_state.label
            if len(started)!=2 or any(v!='inactive' for v in report['lifecycle_states'].values()):report['passed']=False
        except Exception as e:report['passed']=False;report['shutdown_error']=repr(e)
        if any(any(abs(v)>1e-10 for v in row[1:]) for row in commands):report['passed']=False
        report.update(scans=scans,commands=commands,odometry=odometry,fixture=meta,
                      elapsed_wall_s=time.monotonic()-begin)
        a.output.write_text(json.dumps(report,indent=2)+'\n');n.destroy_node();rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
