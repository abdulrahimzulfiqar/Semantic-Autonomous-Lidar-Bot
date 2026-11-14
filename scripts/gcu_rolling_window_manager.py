"""Simulation window transport/admission around the existing mission executor.

The mission contract and passage monitor stay building-wide. Only the native
metric target is replaced by a bounded proxy; an action success at a proxy does
not count as original-target arrival. Ground truth is never an input here.
"""
from dataclasses import asdict
import hashlib
import math
import threading
import time
import numpy as np

from osmag_rolling_raster import RollingRaster
from osmag_window_goal import select_window_goal
from osmag_window_route import prefix_to_target
from osmag_window_admission import WindowAdmission
from osmag_constraint_mask import build_mask
from gcu_plan_validation import validate_path


class RollingWindowManager:
    def __init__(self,node,adapter,planner,contract,grids,mask_pub,info_pub,output,wait,*,shape=(128,128),shift_m=.65,file_prefix=''):
        from rclpy.qos import QoSProfile,DurabilityPolicy
        from nav_msgs.msg import OccupancyGrid
        from nav2_msgs.action import ComputePathToPose
        from rclpy.action import ActionClient
        self.node=node;self.adapter=adapter;self.planner=planner;self.contract=contract
        self.grids=grids;self.mask_pub=mask_pub;self.info_pub=info_pub;self.output=output;self.wait=wait
        self.pub=node.create_publisher(OccupancyGrid,'/osmag/planning_window',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.file_prefix=file_prefix
        self.client=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        self.renderer=RollingRaster(planner.leaves,planner.passage_segments,
            resolution=float(np.float32(planner.resolution)),anchor=planner.origin,shape=shape,
            source_sha256=planner.fingerprints()['osm_sha256'] if 'osm_sha256' in planner.fingerprints() else hashlib.sha256((planner.assets/'gcu_ground_floor.osm').read_bytes()).hexdigest())
        self.incident={k:(v['osmAG:from'],v['osmAG:to']) for k,v in planner.passages.items()}
        self.shift_m=shift_m;self.revision=0;self.committed=None;self.pending=None
        self.last_native_plan=None;self.last_native_plan_stamp=None
        self.records=[];self.events=[];self.busy=False;self.frozen=False;self.gate_lock=threading.RLock()
        self.publish_lock=threading.RLock();self.closed=False;self.publications=[]
        self.readiness_failures=[];self.failure_arrays=[]
        self.last_tick_wall=-math.inf
        self.timer=node.create_timer(.1,self.heartbeat)

    def message(self,data,state):
        from nav_msgs.msg import OccupancyGrid
        m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=self.node.get_clock().now().to_msg()
        w=state['window'];m.info.resolution=w.resolution;m.info.width=data.shape[1];m.info.height=data.shape[0]
        m.info.origin.position.x=w.origin[0];m.info.origin.position.y=w.origin[1];m.info.origin.orientation.w=1.
        m.data=data.astype(int).ravel().tolist();return m

    def heartbeat(self):
        with self.publish_lock:
            if self.frozen or self.closed:return
            state=self.pending or self.committed
            if state is None:return
            self.pub.publish(self.message(state['window'].grid,state));self.mask_pub.publish(self.message(state['snapshot'].mask,state))
            now=self.adapter.sim()
            with self.gate_lock:state['gate'].note_source_publication(now,state['window'].sha256)
            self.publications.append(dict(sim=now,wall=time.monotonic(),revision=state['window'].revision))

    def publish_window(self,pose):
        from nav2_msgs.msg import CostmapFilterInfo
        self.revision+=1;now=self.adapter.sim();started=time.perf_counter()
        window=self.renderer.render(pose[:2],revision=self.revision,stamp=now)
        raster_s=time.perf_counter()-started;mask_started=time.perf_counter()
        snap=build_mask(window.grid,window.resolution,window.origin,self.planner.leaves,
            self.planner.passage_segments,self.incident,allowed_areas=self.contract.area_ids,
            open_passages=self.contract.passage_ids,disabled_passages=self.contract.disabled_passages,
            robot_pose=pose[:2],mission_id=self.contract.mission_id,route_revision=self.contract.route_revision,
            mask_revision=self.revision,closure_revision=self.contract.closure_revision)
        common=[]
        if self.committed is not None:
            # The old controller goal remains valid while two native topics
            # update. Validate the actual remaining native plan in BOTH windows.
            active=getattr(self,'last_native_plan',None)
            from gcu_navigation_metrics import rest_window
            stopped_proxy=(self.adapter.model.state=='waiting_proxy' and not self.adapter.handles
                           and rest_window(self.adapter.rest_samples) is not None)
            if stopped_proxy:
                common=[pose[:2]]
                for old in [self.committed['snapshot'],snap]:
                    if not validate_path(old.combined,old.resolution,old.origin,common,.20)['valid']:
                        raise RuntimeError('stopped_proxy_not_in_common_window')
            elif active:
                from shapely.geometry import LineString,Point
                from shapely.ops import substring
                line=LineString(active);part=substring(line,line.project(Point(pose[:2])),line.length)
                common=[pose[:2]]+list(part.coords)
                for old in [self.committed['snapshot'],snap]:
                    if not validate_path(old.combined,old.resolution,old.origin,common,.20)['valid']:
                        raise RuntimeError('remaining_native_plan_not_in_common_window')
            else:raise RuntimeError('missing_native_plan_for_window_shift')
        state=dict(window=window,snapshot=snap,gate=WindowAdmission(window,snap,now,[pose[:2]]),
            pose=list(pose),published_sim=now,cache={},common_native_plan=common,
            raster_s=raster_s,mask_and_overlap_s=time.perf_counter()-mask_started)
        with self.publish_lock:self.pending=state;self.heartbeat()
        info=CostmapFilterInfo();info.type=0;info.base=0.;info.multiplier=1.;info.filter_mask_topic='/osmag/constraint_mask'
        self.info_pub.publish(info)
        self.events.append(dict(event='window_published',sim=now,revision=self.revision,origin=window.origin))

    def observe(self,state):
        now=self.adapter.sim();gate=state['gate']
        with self.gate_lock:
            for key,m in list(self.grids.items()):
                stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
                if stamp>now:continue
                try:
                    gate.observe(key,np.asarray(m.data).reshape(m.info.height,m.info.width),m.info.resolution,
                        (m.info.origin.position.x,m.info.origin.position.y),stamp,now,frame=m.header.frame_id)
                    state.setdefault('last_errors',{}).pop(key,None)
                except Exception as error:state.setdefault('last_errors',{})[key]=str(error)
            return gate.ready(now)

    def ready(self):
        # While pending, keep the old acknowledged lease only within its strict
        # freshness deadline; accept the new lease after actual consumption.
        pending=self.pending;old=self.committed
        if pending:
            if self.observe(pending):return True
            with self.gate_lock:ready=bool(old and old['gate'].ready(self.adapter.sim()))
        else:ready=bool(old and self.observe(old))
        if not ready and self.adapter.enabled and len(self.readiness_failures)<8:
            now=self.adapter.sim();record=dict(sim=now,states={})
            for label,state in [('committed',old),('pending',pending)]:
                if state:record['states'][label]=dict(revision=state['window'].revision,
                    source_age=now-state['gate'].last_source_stamp,observations=dict(state['gate'].observations),
                    errors=dict(state.get('last_errors',{})))
            arrays={};record['native']={}
            for key,m in self.grids.items():
                arrays[key]=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
                record['native'][key]=dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                    origin=[m.info.origin.position.x,m.info.origin.position.y],resolution=m.info.resolution)
            self.readiness_failures.append(record);self.failure_arrays.append(arrays)
        return ready

    def prepare(self):
        from nav2_msgs.action import ComputePathToPose
        from geometry_msgs.msg import PoseStamped
        state=self.pending
        if state is None:return
        self.busy=True
        try:
            self.wait(lambda:self.observe(state),25)
            # Require a real scan after the new window publication before the
            # native preflight; heartbeat does not freshen sensor observations.
            self.wait(lambda:self.adapter.scan_stamp is not None and self.adapter.scan_stamp>=state['published_sim'],10)
            self.wait(self.client.server_is_ready,10)
            localized=[]
            def fresh_pose():
                from osmag_mission_contract import MissionError
                try:localized[:]=self.adapter.pose();return True
                except MissionError as error:
                    if str(error)!='stale_robot_tf':raise
                    return False
            # Rendering/reset service calls can outlast a TF age budget before
            # preflight starts. Acquire a fresh sample rather than using the
            # setup pose or weakening the running executor's watchdog.
            self.wait(fresh_pose,2)
            pose,_=localized;cache={};checks={};queries={}
            for segment in self.preflight_segments():
                route=prefix_to_target(self.contract.path,pose[:2],segment.point)
                goal=select_window_goal(state['window'],route,pose[:2],final_yaw=segment.yaw)
                with self.gate_lock:receipt=state['gate'].admit(goal,self.adapter.sim())
                key=tuple(goal.point)
                if key not in queries:
                    req=ComputePathToPose.Goal();req.planner_id='GridBased';req.use_start=False
                    req.goal=PoseStamped();req.goal.header.frame_id='map'
                    req.goal.pose.position.x,req.goal.pose.position.y=map(float,goal.point);req.goal.pose.orientation.w=1.
                    f=self.client.send_goal_async(req);self.wait(f.done,10);handle=f.result()
                    if not handle.accepted:raise RuntimeError('window_plan_action_rejected')
                    f=handle.get_result_async();self.wait(f.done,10);reply=f.result()
                    points=[[p.pose.position.x,p.pose.position.y] for p in reply.result.path.poses]
                    check=validate_path(state['snapshot'].combined,state['window'].resolution,state['window'].origin,points,.20)
                    if reply.status!=4 or not check['valid']:raise RuntimeError('window_native_preflight_not_feasible:'+str(check))
                    queries[key]=dict(status=reply.status,points=points,validation=check)
                cache[segment.index]=goal;checks[str(segment.index)]=dict(goal=asdict(goal),admission=receipt,**queries[key])
            state['cache']=cache
            state['committed_sim']=self.adapter.sim();window=state['window'];snap=state['snapshot']
            name=self.file_prefix+f'window_{window.revision:03d}.npz';arrays=dict(physical=window.grid,mask=snap.mask,combined=snap.combined)
            native={}
            for key,m in self.grids.items():
                arrays[key]=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
                native[key]=dict(origin=[m.info.origin.position.x,m.info.origin.position.y],resolution=m.info.resolution,
                    frame=m.header.frame_id,stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9)
            np.savez_compressed(self.output/name,**arrays)
            self.records.append(dict(window=window.report(),snapshot=snap.report(),file=name,
                published_sim=state['published_sim'],committed_sim=state['committed_sim'],pose=state['pose'],
                raster_generation_wall_s=state['raster_s'],mask_and_overlap_wall_s=state['mask_and_overlap_s'],
                preflights=checks,native=native,common_native_plan=state['common_native_plan']))
            with self.publish_lock:self.committed=state;self.pending=None
            self.events.append(dict(event='window_committed',sim=self.adapter.sim(),revision=window.revision))
        finally:self.busy=False

    def preflight_segments(self):
        # Setup precedes model.start(). A previous completed/recovered mission
        # may still have a higher index; that index cannot prune this contract.
        previous=self.adapter.model.contract
        same=(previous is not None and previous.mission_id==self.contract.mission_id
              and previous.route_revision==self.contract.route_revision)
        first=max(0,self.adapter.model.index) if same else 0
        return [segment for segment in self.contract.segments if segment.index>=first]

    def resolve(self,segment,pose,contract):
        state=self.committed
        if state is None or segment.index not in state['cache']:raise RuntimeError('window_target_not_preflighted')
        goal=state['cache'][segment.index]
        route=prefix_to_target(contract.path,pose[:2],goal.point)
        if not validate_path(state['snapshot'].combined,state['window'].resolution,state['window'].origin,route,.20)['valid']:
            raise RuntimeError('localized_proxy_connector_blocked')
        with self.gate_lock:receipt=state['gate'].admit(goal,self.adapter.sim())
        self.events.append(dict(event='goal_admitted',sim=self.adapter.sim(),segment=segment.index,goal=asdict(goal),receipt=receipt))
        return goal

    def tick(self):
        if self.busy or self.frozen or not self.adapter.enabled:return
        # Contact instrumentation can wake the evaluator thousands of times a
        # second. Window decisions need a steady cadence, not a TF lookup and
        # native-grid scan on every incoming contact message.
        now_wall=time.monotonic()
        if now_wall-self.last_tick_wall<.04:return
        self.last_tick_wall=now_wall
        if self.adapter.model.state not in ['executing','waiting_handoff','waiting_proxy']:return
        pose,_=self.adapter.pose()
        if self.committed and (math.dist(pose[:2],self.committed['pose'][:2])>=self.shift_m
                or self.adapter.model.state=='waiting_proxy'):
            # Start an asynchronous map change immediately after fresh native
            # acknowledgements, not near the end of the old lease. This keeps
            # the original .5s expiry unchanged while giving the next update a
            # full bounded overlap interval.
            if not self.observe(self.committed):return
            now=self.adapter.sim()
            with self.gate_lock:
                if not self.committed['gate'].can_replace(now):return
            # RPP publishes its actually followed/pruned path, unlike /plan,
            # which also carries unrelated ComputePath preflight responses.
            target=getattr(self.adapter,'native_target_point',None)
            from gcu_navigation_metrics import rest_window
            stopped_proxy=(self.adapter.model.state=='waiting_proxy' and not self.adapter.handles
                           and rest_window(self.adapter.rest_samples) is not None)
            if not stopped_proxy and (not self.last_native_plan or target is None
                    or math.dist(self.last_native_plan[-1],target)>.15
                    or self.last_native_plan_stamp is None
                    or not 0<=self.adapter.sim()-self.last_native_plan_stamp<=.3):return
            self.publish_window(pose);self.prepare()
        if (self.committed and self.adapter.proxy_active and
                self.adapter.proxy_window_revision!=self.committed['window'].revision):
            self.adapter.refresh_proxy()

    def freeze(self):
        with self.publish_lock:self.frozen=True;self.events.append(dict(event='source_frozen',sim=self.adapter.sim()))

    def resume(self):
        with self.publish_lock:
            if self.closed:raise RuntimeError('closed_window_source')
            self.frozen=False;self.events.append(dict(event='source_resumed',sim=self.adapter.sim()));self.heartbeat()

    def report(self):
        for index,arrays in enumerate(self.failure_arrays,1):
            np.savez_compressed(self.output/(self.file_prefix+f'window_readiness_failure_{index:02d}.npz'),**arrays)
        return dict(scope='Live raster/constraint/proxy manager; no truth input',windows=self.records,events=self.events,
                    frozen=self.frozen,window_shape=list(self.renderer.shape),readiness_failures=self.readiness_failures,
                    source_publications=list(self.publications),
                    pending_revision=self.pending['window'].revision if self.pending else None)

    def close(self):
        # Cancel first; let executor.shutdown drain queued callbacks before the
        # evaluator destroys its ROS entities. No source publishes after close.
        with self.publish_lock:
            if self.closed:return
            self.closed=True;self.timer.cancel();self.node.destroy_publisher(self.pub)
