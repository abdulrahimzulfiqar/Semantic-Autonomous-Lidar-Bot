"""Simulation-only 06B2B ROS transport for one freshly frozen mission.

Ground truth is deliberately absent. The separate evaluator owns truth scoring.
The surrounding harness owns lifecycle/preflight and enables dispatch once.
"""
from dataclasses import replace
import math,os,time,threading
from functools import wraps
from gcu_navigation_metrics import rest_window
from osmag_mission_contract import CrossingMonitor,MissionError
from osmag_mission_state import MissionState


def simulation_guard(execute,environment):
    if not execute or environment.get('ROS_DOMAIN_ID')!='195' or environment.get('ROS_LOCALHOST_ONLY')!='1' or not environment.get('IGN_PARTITION','').startswith('gcu_phase6b2b_'):
        raise MissionError('isolated_simulation_execution_required')


def serialized(method):
    @wraps(method)
    def call(self,*args,**kwargs):
        with self.state_lock:return method(self,*args,**kwargs)
    return call


def coherent_pose_time(clock,latest,maximum_age=.15):
    """Do not consume future TF or freshen genuinely old TF by relabelling it."""
    if not all(math.isfinite(v) for v in [clock,latest,maximum_age]) or maximum_age<=0:
        raise MissionError('invalid_tf_clock')
    if clock-latest>maximum_age+1e-9:raise MissionError('stale_robot_tf')
    return min(clock,latest)


def fresh_scan_at(clock,observations,maximum_age=.3):
    """Sensor age uses simulated time; tick separately guards stalled clocks.

    A slowly advancing VM can take >2 wall seconds per 0.1 s scan period.
    Real-time execution still expires after the same 0.3 seconds of ROS time.
    """
    return any(0<=clock-stamp<=maximum_age for stamp,_received in observations)


def require_completed_previous(model,handles):
    """Successful consecutive missions require observed rest and no old handles."""
    if model.contract is not None and (model.state!='succeeded' or not model.stop_confirmed or model._unresolved() or handles):
        raise MissionError('previous_mission_not_completed_and_stopped')


def create_executor(planner,execute=False,require_imu=False,imu_topic='/imu_broadcaster/imu'):
    simulation_guard(execute,os.environ)
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.action import ActionClient
    from rclpy.time import Time
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from tf2_ros import Buffer,TransformListener
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Path,Odometry
    from sensor_msgs.msg import LaserScan,Imu
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String,Empty

    class Executor(Node):
        def __init__(self):
            super().__init__('gcu_segmented_executor',parameter_overrides=[Parameter('use_sim_time',value=True)])
            self.state_lock=threading.RLock()
            self.planner=planner;self.model=MissionState();self.monitor=None;self.handles={};self.events=[]
            self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
            self.client=ActionClient(self,NavigateToPose,'/navigate_to_pose')
            self.last_tf=None;self.offset=0.;self.rest_samples=[];self.scan_stamp=None;self.scan_wall=None
            self.readiness=None;self.final_action_sim=None;self.fault_reason=None;self.enabled=False
            self.window_provider=None;self.proxy_tokens={};self.proxy_active=False;self.proxy_window_revision=None
            self.last_clock=None;self.clock_wall=time.monotonic();self.poses=[]
            self.tf_timing=[];self.scan_observations=[]
            self.imu_observations=[];self.imu_samples=[];self.require_imu=require_imu;self.imu_topic=imu_topic
            self.closure={'revision':0,'disabled_passages':[]};self.closure_result=None
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.status_pub=self.create_publisher(String,'/osmag/mission_status',qos)
            self.path_pub=self.create_publisher(Path,'/osmag/mission_path',qos)
            self.goal_pub=self.create_publisher(PoseStamped,'/osmag/segment_goal',qos)
            self.event_pub=self.create_publisher(String,'/osmag/passage_progress',100)
            self.odom_sub=self.create_subscription(Odometry,'/odometry/filtered',self.odom,100)
            self.scan_sub=self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
            if require_imu:self.imu_sub=self.create_subscription(Imu,imu_topic,self.imu,qos_profile_sensor_data)
            self.cancel_sub=self.create_subscription(Empty,'/osmag/mission_cancel',lambda _:self.cancel('user_cancel'),10)
            self.closure_sub=self.create_subscription(String,'/osmag/passage_closures',self.close_passages,10)
            self.closure_pub=self.create_publisher(String,'/osmag/closure_status',qos)
            self.timer=self.create_timer(.04,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

        def sim(self):return self.get_clock().now().nanoseconds/1e9

        def event(self,kind,**fields):
            import json
            item=dict(event=kind,wall=time.monotonic(),sim=self.sim(),**fields);self.events.append(item)
            self.event_pub.publish(String(data=json.dumps(item)))

        def pose(self):
            t=self.buffer.lookup_transform('map','base_footprint',Time());stamp=t.header.stamp.sec+t.header.stamp.nanosec/1e9
            current=self.sim();query=coherent_pose_time(current,stamp)
            self.tf_timing.append({'clock':current,'latest_stamp':stamp,'query_stamp':query,'wall':time.monotonic()})
            if query<stamp:
                # TF and /clock travel independently. Query existing buffer
                # history, never extrapolate a future observation into now.
                t=self.buffer.lookup_transform('map','base_footprint',Time(seconds=query))
            stamp=t.header.stamp.sec+t.header.stamp.nanosec/1e9
            if not -1e-9<=current-stamp<=.15+1e-9:raise MissionError('stale_robot_tf')
            v=t.transform;q=v.rotation
            return [v.translation.x,v.translation.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)],stamp

        def scan(self,m):
            self.scan_stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9;self.scan_wall=time.monotonic()
            self.scan_observations.append((self.scan_stamp,self.scan_wall));self.scan_observations=self.scan_observations[-20:]

        def imu(self,m):
            stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
            if not math.isfinite(m.angular_velocity.z):
                self.cancel('nonfinite_imu_yaw_rate');return
            self.imu_observations.append((stamp,time.monotonic()));self.imu_observations=self.imu_observations[-30:]
            self.imu_samples.append({'sim_time':stamp,'yaw_rate':m.angular_velocity.z,'frame':m.header.frame_id})
            if len(self.imu_samples)>20000:del self.imu_samples[:-20000]

        def odom(self,m):
            p=m.pose.pose.position;q=m.pose.pose.orientation;s=m.header.stamp.sec+m.header.stamp.nanosec/1e9
            if self.rest_samples and s<=self.rest_samples[-1]['sim_time']:return
            self.rest_samples.append({'sim_time':s,'truth':[p.x,p.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z)]})
            self.rest_samples=[v for v in self.rest_samples if s-v['sim_time']<3.]

        def healthy(self):
            current=self.sim()
            if not fresh_scan_at(current,self.scan_observations):raise MissionError('stale_scan')
            if self.require_imu and (self.count_publishers(self.imu_topic)!=1 or not fresh_scan_at(current,self.imu_observations,.15)):raise MissionError('stale_or_unowned_imu')
            if self.count_publishers('/navigate_to_pose/_action/status')!=1:raise MissionError('navigator_ownership_changed')
            if self.count_publishers('/phase6_disabled_goal_pose') or self.count_subscribers('/gcu/navigation_goal'):raise MissionError('competing_goal_input')
            if self.readiness is None or not self.readiness():raise MissionError('constraint_costmaps_not_ready')

        @serialized
        def begin(self,contract,readiness):
            require_completed_previous(self.model,self.handles)
            from gcu_closure import require_closure_contract
            require_closure_contract(contract,self.closure)
            if self.fault_reason is not None:raise MissionError('prior_executor_fault_requires_recovery')
            self.readiness=readiness;self.healthy();point,stamp=self.pose();now=time.monotonic()
            from gcu_plan_validation import validate_path
            if math.dist(point[:2],contract.start)>.03 or self.planner.containing_area(point[:2],'start')!=contract.area_ids[0] or not validate_path(self.planner.grid,self.planner.resolution,self.planner.origin,[point[:2],contract.start],.20)['valid']:
                raise MissionError('start_changed_during_setup')
            self.offset=now-self.sim();contract=replace(contract,created_at=now,pose_stamp=stamp+self.offset)
            self.final_action_sim=None;self.last_clock=self.sim();self.clock_wall=now
            self.monitor=CrossingMonitor(contract,self.planner.leaves,self.planner.passage_segments);self.last_tf=stamp
            msg=Path();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
            for x,y in contract.path:
                p=PoseStamped();p.header=msg.header;p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;msg.poses.append(p)
            self.path_pub.publish(msg);self.event('mission',contract=contract.report())
            self.effects(self.model.start(contract,now));self.enabled=True

        def effects(self,effects):
            for effect in effects:
                if effect['kind']=='send':
                    from gcu_closure import require_closure_contract
                    require_closure_contract(self.model.contract,self.closure)
                    self.healthy();point,_=self.pose();segment=effect['segment']
                    resolved=self.window_provider.resolve(segment,point,self.model.contract) if self.window_provider else None
                    actual_point=resolved.point if resolved else segment.point
                    actual_yaw=resolved.yaw if resolved else segment.yaw
                    self.native_target_point=actual_point
                    self.proxy_active=bool(resolved and resolved.proxy)
                    self.proxy_tokens[effect['token']]=self.proxy_active
                    self.proxy_window_revision=resolved.window_revision if resolved else None
                    target=PoseStamped();target.header.frame_id='map';target.header.stamp=self.get_clock().now().to_msg()
                    target.pose.position.x,target.pose.position.y=map(float,actual_point)
                    target.pose.orientation.z=math.sin(actual_yaw/2);target.pose.orientation.w=math.cos(actual_yaw/2)
                    self.goal_pub.publish(target);req=NavigateToPose.Goal();req.pose=target
                    self.event('send',token=effect['token'],segment=segment.index,point=actual_point,pose=point,confirmed=self.model.confirmed,
                        original_segment_point=segment.point,proxy=self.proxy_active,window_revision=self.proxy_window_revision)
                    self.client.send_goal_async(req).add_done_callback(lambda f,t=effect['token']:self.accepted(f,t))
                elif effect['uuid'] in self.handles:
                    self.event('cancel_request',uuid=effect['uuid']);self.handles[effect['uuid']].cancel_goal_async()

        @serialized
        def accepted(self,future,token):
            try:
                h=future.result();ident=bytes(h.goal_id.uuid).hex() if h.accepted else ''
                if h.accepted:self.handles[ident]=h
                self.event('accepted',token=token,uuid=ident,accepted=h.accepted)
                effects=self.model.accepted(token,ident,h.accepted,time.monotonic())
                if h.accepted:h.get_result_async().add_done_callback(lambda f,i=ident:self.result(f,i))
                self.effects(effects)
            except Exception as e:self.cancel('acceptance_error:'+str(e))

        @serialized
        def result(self,future,ident):
            try:
                reply=future.result();outcome={4:'succeeded',5:'cancelled',6:'aborted'}[reply.status]
                self.event('result',uuid=ident,outcome=outcome)
                attempt=next((v for v in self.model.attempts.values() if v.uuid==ident),None)
                proxy=bool(attempt and self.proxy_tokens.get(attempt.token,False))
                self.effects(self.model.result(ident,outcome,time.monotonic(),proxy=proxy));self.handles.pop(ident,None)
                if self.model.state=='verifying':self.final_action_sim=self.sim()
            except Exception as e:self.cancel('result_error:'+str(e))

        @serialized
        def refresh_proxy(self):
            if self.proxy_active and self.model.state in ['executing','waiting_proxy'] and self.model.pending is None:
                self.effects(self.model.repeat_current(time.monotonic()))

        @serialized
        def recover(self):
            from gcu_user_mission import require_recoverable
            require_recoverable(self.model,self.handles)
            self.pose();self.healthy()
            self.enabled=False;self.model.contract=None;self.model.state='idle';self.fault_reason=None;self.monitor=None
            self.event('recovered_after_confirmed_stop')

        @serialized
        def close_passages(self,message):
            from gcu_closure import parse_closure
            try:update=parse_closure(message.data,self.closure['revision'],self.planner.passages)
            except Exception as error:
                self.event('closure_rejected',reason=str(error));return
            self.closure=update;self.closure_result=None;self.event('closure_received',**update)
            if self.model.contract is not None and self.model.state not in ['succeeded','cancelled','failed']:
                # Invalidate execution atomically before any further timer-driven
                # dispatch. Replanning never runs while an old action can drive.
                self.cancel('passage_closed')

        @serialized
        def cancel(self,reason):
            if self.fault_reason is None:self.fault_reason=reason;self.event('fault',reason=reason)
            self.effects(self.model.cancel(time.monotonic(),self.fault_reason))

        @serialized
        def tick(self):
            if not self.enabled:return
            try:
                current=self.sim()
                if current!=self.last_clock:self.last_clock=current;self.clock_wall=time.monotonic()
                if time.monotonic()-self.clock_wall>2:raise MissionError('simulation_clock_stalled')
                self.effects(self.model.check_deadlines(time.monotonic()))
                point,stamp=self.pose()
                if self.model.state in ['dispatching','executing','waiting_handoff','waiting_proxy','verifying']:
                    self.healthy()
                    if stamp>self.last_tf:
                        event=self.monitor.observe(point[:2],stamp+self.offset);self.last_tf=stamp
                        self.poses.append({'sim_time':stamp,'pose':point})
                        if event:self.event('crossing',**event)
                        self.effects(self.model.advance(point[:2],self.monitor.confirmed,time.monotonic(),allow_handoff=not self.proxy_active))
                    if self.model.state=='verifying':
                        self.model.confirm_final(point,rest_window(self.rest_samples,earliest=self.final_action_sim) is not None,time.monotonic())
                elif self.model.state in ['cancelling','failed']:
                    self.model.confirm_stopped(rest_window(self.rest_samples) is not None,time.monotonic())
                    if self.model.stop_confirmed and self.model.reason=='passage_closed' and self.closure_result is None:
                        from gcu_closure import resolve_closure
                        import json
                        pose,stamp=self.pose();now=time.monotonic()
                        self.closure_result=resolve_closure(self.planner,self.model,self.handles,pose,self.closure,now,now-(self.sim()-stamp))
                        self.event('closure_resolution',**self.closure_result)
                        self.closure_pub.publish(String(data=json.dumps(self.closure_result)))
                import json
                self.status_pub.publish(String(data=json.dumps(self.model.report())))
            except Exception as e:self.cancel('watchdog:'+str(e))

    return Executor()
