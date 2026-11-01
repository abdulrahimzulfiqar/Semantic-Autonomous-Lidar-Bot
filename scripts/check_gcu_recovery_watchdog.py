#!/usr/bin/env python3
"""Simulation-only remaining Phase 03 checks. Requires explicit --execute.

Recovery injects a predetermined uncertain initial pose and requests stationary
sensor updates. Watchdog commands the controller directly, then sends NOTHING
until a stop is observed, so upstream mux behavior cannot fake a watchdog pass.
"""
import argparse
import json
import math
from pathlib import Path
import time


def wrap(value): return math.atan2(math.sin(value), math.cos(value))


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from tf2_msgs.msg import TFMessage
    from sensor_msgs.msg import LaserScan
    from rosgraph_msgs.msg import Clock
    from std_srvs.srv import Empty
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--mode',choices=['recovery','watchdog'],required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not args.execute:parser.error('Explicit --execute required for simulation test')
    rclpy.init(args=[])
    node=rclpy.create_node('gcu_'+args.mode+'_check')
    sensor=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
    state={'sim':None,'truth':None,'truth_wall':0.,'clock_wall':0.,'scan_wall':0.,'pose':None,'pose_count':0,'commands':[]}
    trace=[]
    def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def truth(msg):
        for t in msg.transforms:
            if t.child_frame_id.strip('/').split('::')[-1]=='lidarbot':
                state['truth']=[t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)]
                state['truth_wall']=time.monotonic()
    def clock(msg):
        t=msg.clock.sec+msg.clock.nanosec/1e9
        if t!=state['sim']:state['clock_wall']=time.monotonic()
        state['sim']=t
    def scan(msg):
        valid=[v for v in msg.ranges if math.isfinite(v) and msg.range_min<=v<=msg.range_max]
        state['nearest']=min(valid) if valid else 0.
        state['scan_wall']=time.monotonic()
    def pose(msg):
        p=msg.pose.pose
        state['pose']=[p.position.x,p.position.y,yaw(p.orientation)]
        state['pose_count']+=1
    def command(msg):
        state['commands'].append({'sim':state['sim'],'wall':time.monotonic(),'v':msg.linear.x,'w':msg.angular.z})
    subscriptions=[node.create_subscription(TFMessage,'/gcu_ground_truth',truth,10),
        node.create_subscription(Clock,'/clock',clock,sensor),node.create_subscription(LaserScan,'/scan',scan,sensor),
        node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',pose,10),
        node.create_subscription(Twist,'/diff_controller/cmd_vel_unstamped',command,10)]
    cmd=node.create_publisher(Twist,'/diff_controller/cmd_vel_unstamped',10)
    prior=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
    update=node.create_client(Empty,'/request_nomotion_update')
    started=time.monotonic();nonzero_sent=0;zeros_sent=0
    def send(v=0.):
        nonlocal nonzero_sent,zeros_sent
        m=Twist();m.linear.x=v;cmd.publish(m)
        if v:nonzero_sent+=1
        else:zeros_sent+=1
    def spin(guard=True):
        until=time.monotonic()+.05
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        if guard:
            now=time.monotonic()
            if any(now-state[k]>1.5 for k in ['truth_wall','clock_wall','scan_wall']):raise RuntimeError('Stale simulator truth, clock or scan')
            if state.get('nearest',0.)<.5:raise RuntimeError('Obstacle within test envelope')
            if math.hypot(state['truth'][0]+.0586439513,state['truth'][1]+12.96031324)>.4:raise RuntimeError('Displacement outside test envelope')
            if now-started>150:raise RuntimeError('Wall timeout')
    report={'mode':args.mode,'simulation_only':True,'passed':False}
    try:
        asked=False
        while time.monotonic()-started<45:
            spin(False)
            if not asked and time.monotonic()-started>2 and update.service_is_ready():update.call_async(Empty.Request());asked=True
            if state['truth'] is not None and state['pose'] is not None and state['sim'] is not None and state['scan_wall']:
                break
        else:raise RuntimeError('Simulator/AMCL readiness timeout')
        initial=list(state['truth'])
        if math.hypot(initial[0]+.0586439513,initial[1]+12.96031324)>.1 or abs(initial[2])>.05:raise RuntimeError('Fresh GCU spawn required')
        report['initial_truth']=initial
        if args.mode=='recovery':
            report['acceptance']={'position_m':.15,'heading_deg':3,'consecutive_pose_updates':3,'simulation_timeout_s':30,'wall_timeout_s':60}
            msg=PoseWithCovarianceStamped();msg.header.frame_id='map'
            msg.header.stamp.sec=int(state['sim']);msg.header.stamp.nanosec=int((state['sim']%1)*1e9)
            msg.pose.pose.position.x=-.05864395131342248+.20
            msg.pose.pose.position.y=-12.960313240266368-.15
            angle=math.radians(8)
            msg.pose.pose.orientation.z=math.sin(angle/2);msg.pose.pose.orientation.w=math.cos(angle/2)
            msg.pose.covariance[0]=.25**2;msg.pose.covariance[7]=.25**2;msg.pose.covariance[35]=math.radians(10)**2
            if not prior.get_subscription_count():raise RuntimeError('AMCL initial-pose subscriber missing')
            prior.publish(msg)
            report['injected_prior']={'x':msg.pose.pose.position.x,'y':msg.pose.pose.position.y,'yaw_deg':8,'position_stddev_m':.25,'yaw_stddev_deg':10}
            start_sim=state['sim'];start_wall=time.monotonic();last_request=-999;old_count=state['pose_count'];consecutive=0;requests=0
            while state['sim']-start_sim<30 and time.monotonic()-start_wall<60:
                spin()
                if state['sim']-last_request>=.5 and update.service_is_ready():
                    update.call_async(Empty.Request());last_request=state['sim'];requests+=1
                if state['pose_count']!=old_count:
                    old_count=state['pose_count'];p=state['pose'];t=state['truth']
                    e=math.hypot(p[0]-t[0],p[1]-t[1]);a=abs(math.degrees(wrap(p[2]-t[2])))
                    trace.append({'sim_elapsed':state['sim']-start_sim,'position_error_m':e,'heading_error_deg':a,'estimate':list(p)})
                    consecutive=consecutive+1 if e<=.15 and a<=3 else 0
                    if consecutive>=3:report['passed']=True;break
            report['recovery_sim_seconds']=state['sim']-start_sim
            report['recovery_wall_seconds']=time.monotonic()-start_wall
            report['stationary_update_requests']=requests
            report['final_truth_drift_m']=math.hypot(state['truth'][0]-initial[0],state['truth'][1]-initial[1])
            report['passed']=report['passed'] and report['final_truth_drift_m']<.01
            report['limitations']='Local uncertain-prior correction with repeated no-motion updates; not global/kidnapped recovery or moving recovery.'
        else:
            report['acceptance']={'command_mps':.05,'command_duration_sim_s':2,'controller_timeout_s':.5,'stop_by_sim_s':1.,'stopped_speed_mps':.005,'stable_sim_s':.3,'maximum_coast_m':.05,'post_stop_drift_m':.01}
            if not cmd.get_subscription_count():raise RuntimeError('Controller command subscriber missing')
            begin=state['sim']
            while state['sim']-begin<2:
                spin();send(.05)
            last_command_sim=state['sim'];last_command_wall=time.monotonic();release=list(state['truth'])
            # Do not send even a zero here: independently exercise controller timeout.
            history=[];quiet_since=None;stopped=None
            while state['sim']-last_command_sim<2:
                spin()
                t=state['sim'];p=list(state['truth']);history.append((t,p))
                previous=[x for x in history if .12<=t-x[0]<=.3]
                if previous:
                    t0,p0=previous[-1];speed=math.hypot(p[0]-p0[0],p[1]-p0[1])/(t-t0)
                    trace.append({'after_last_command_sim_s':t-last_command_sim,'speed_mps':speed,'truth':p})
                    if speed<.005:
                        if quiet_since is None:quiet_since=t
                        if t-quiet_since>=.3 and stopped is None:stopped=(quiet_since,list(p))
                    else:quiet_since=None
            if stopped is None:raise RuntimeError('No stable stop observed during command silence')
            external=[x for x in state['commands'] if x['wall']>last_command_wall+.15]
            report['messages_observed_after_silence_settle']=len(external)
            report['explicit_zero_messages_before_stop']=zeros_sent
            report['stop_after_last_command_sim_s']=stopped[0]-last_command_sim
            report['coast_distance_m']=math.hypot(stopped[1][0]-release[0],stopped[1][1]-release[1])
            report['post_stop_drift_m']=math.hypot(state['truth'][0]-stopped[1][0],state['truth'][1]-stopped[1][1])
            report['driven_distance_m']=math.hypot(release[0]-initial[0],release[1]-initial[1])
            report['passed']=bool(report['driven_distance_m']>.04 and report['stop_after_last_command_sim_s']<=1. and report['coast_distance_m']<=.05 and report['post_stop_drift_m']<=.01 and zeros_sent==0 and not external)
            report['limitations']='Direct simulated-controller watchdog only; mux/network/process-crash/hardware stopping are not certified.'
    except Exception as error:report['error']=str(error)
    finally:
        if args.mode=='watchdog':
            until=time.monotonic()+1
            while time.monotonic()<until:send();rclpy.spin_once(node,timeout_sec=.001);time.sleep(.05)
        report['nonzero_velocity_messages_sent']=nonzero_sent
        report['zero_velocity_messages_sent_including_cleanup']=zeros_sent
        report['wall_seconds']=time.monotonic()-started
        report['trace']=trace
        args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='trace'},indent=2),flush=True)
        node.destroy_node();rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
