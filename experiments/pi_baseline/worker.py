#!/usr/bin/env python3
"""Pi-only pilot actor: existing planner/executor/window manager; no truth input."""
import argparse
import json
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    from profile_phase9 import simulation_guard
    import os
    simulation_guard(args.execute, os.environ)
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.time import Time
    from nav_msgs.msg import OccupancyGrid, Path as NavPath
    from nav2_msgs.msg import CostmapFilterInfo
    from lifecycle_msgs.srv import ChangeState
    from std_msgs.msg import String
    from osmag_paper_planner import PaperPlanner
    from osmag_mission_contract import make_contract
    from gcu_segmented_executor import create_executor
    from gcu_rolling_window_manager import RollingWindowManager

    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(passed=False, scope='Pi navigation actor; no ground-truth subscription',
                  goal=[1., -12.960313240266368, 0.], events=[],stage='preprocessing')
    began = time.monotonic()
    preprocessing = time.monotonic()
    planner = PaperPlanner(args.workspace / 'assets', execution_padding=.02)
    report['preprocessing_wall_s'] = time.monotonic() - preprocessing
    print(f"Route preprocessing completed in {report['preprocessing_wall_s']:.2f} wall seconds", flush=True)
    profile = json.loads((args.workspace/'profile.json').read_text())
    imu_topic = '/imu_broadcaster/imu'
    if profile.get('reliable_localizer_inputs') and not profile.get('local_stack_replay'):
        from localizer_input_relay import RELAY_IMU
        imu_topic = RELAY_IMU
    rclpy.init()
    adapter = create_executor(planner, True, require_imu=True, imu_topic=imu_topic)
    node = Node('phase9_navigation_driver', parameter_overrides=[Parameter('use_sim_time', value=True)])
    trace=None
    if profile.get('separate_mission_timer') and not profile.get('dedicated_mission_timer'):
        from executor_groups import separate_mission_timer
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        report['timer_group_change']=separate_mission_timer(adapter,MutuallyExclusiveCallbackGroup)
    if profile.get('timing_trace'):
        from callback_trace import CallbackTrace,attach_adapter
        trace=CallbackTrace();attach_adapter(trace,adapter)
        clock_sub=node._time_source._clock_sub
        clock_sub.callback=trace.wrap('driver_clock',clock_sub.callback,adapter.sim,
            lambda m:dict(stamp=m.clock.sec+m.clock.nanosec/1e9))
    tf_listener = None
    if profile.get('dedicated_tf'):
        from executor_groups import DedicatedTransformListener
        service_clock = profile.get('dedicated_actor_clock', False)
        tf_listener = DedicatedTransformListener(adapter, trace, service_clock=service_clock)
        report['actor_clock_on_input_executor'] = service_clock
    mission_timer = None
    if profile.get('dedicated_mission_timer'):
        from executor_groups import MissionTimerExecutor
        mission_timer = MissionTimerExecutor(adapter)
        report['timer_group_change'] = dict(period_ns=40_000_000, executor='dedicated',
                                            callback_and_clock_preserved=True)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(adapter)
    executor.add_node(node)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status = node.create_publisher(String, '/phase9/pilot_result', qos)
    grids = {}
    subscriptions = []
    manager = None
    activated = []
    for key in ['global', 'local']:
        callback=lambda m,k=key:grids.update({k:m})
        if trace:callback=trace.wrap('grid_'+key,callback,adapter.sim,lambda m:dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9))
        subscriptions.append(node.create_subscription(OccupancyGrid, '/'+key+'_costmap/costmap',callback,qos))
    mask = node.create_publisher(OccupancyGrid, '/osmag/constraint_mask', qos)
    info = node.create_publisher(CostmapFilterInfo, '/osmag/constraint_filter_info', qos)

    def followed(message):
        if manager is None or not message.poses:
            return
        try:
            v = adapter.buffer.lookup_transform('map', message.header.frame_id,
                Time.from_msg(message.header.stamp)).transform
            yaw = math.atan2(2*v.rotation.w*v.rotation.z, 1-2*v.rotation.z*v.rotation.z)
            c, s = math.cos(yaw), math.sin(yaw)
            manager.last_native_plan = [[v.translation.x+c*p.pose.position.x-s*p.pose.position.y,
                v.translation.y+s*p.pose.position.x+c*p.pose.position.y] for p in message.poses]
            manager.last_native_plan_stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        except Exception:
            pass
    subscriptions.append(node.create_subscription(NavPath, '/received_global_plan', followed, 10))

    spin=trace.wrap('driver_spin',executor.spin_once,adapter.sim) if trace else executor.spin_once
    def tick_manager():return manager.tick()
    manager_tick=trace.wrap('window_tick',tick_manager,adapter.sim) if trace else tick_manager

    def step():
        spin(timeout_sec=.015)
        if time.monotonic()-began > 260:
            raise RuntimeError('pilot_actor_wall_timeout')
        if manager is not None:
            manager_tick()

    def wait(predicate, seconds=30):
        until = time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>until:
                raise RuntimeError('pilot_preflight_timeout:'+report['stage'])
            step()

    def transition(name, code):
        client = node.create_client(ChangeState, '/'+name+'/change_state')
        wait(client.service_is_ready, 15)
        request = ChangeState.Request()
        request.transition.id = code
        future = client.call_async(request)
        wait(future.done, 15)
        if future.result() is None or not future.result().success:
            raise RuntimeError('pilot_lifecycle_failed:'+name+':'+str(code))
        node.destroy_client(client)

    try:
        report['stage']='localized_sensors'
        wait(lambda: adapter.sim()>0 and adapter.scan_stamp is not None and
             adapter.buffer.can_transform('map','base_footprint',Time()), 55)
        point = []
        def fresh():
            try:
                point[:] = adapter.pose()
                return True
            except Exception:
                return False
        wait(fresh, 5)
        start, stamp = point
        if math.dist(start[:2],[-.5,-12.960313240266368])>.15:
            raise RuntimeError('unexpected_pilot_start')
        now = time.monotonic()
        contract = make_contract(planner,start[:2],report['goal'][:2],0.,
            mission_id='phase9_short_crossing',now=now,pose_stamp=now-(adapter.sim()-stamp))
        if contract.passage_ids != ('-93',):
            raise RuntimeError('pilot_topology_changed')
        report['contract'] = contract.report()
        manager = RollingWindowManager(node,adapter,planner,contract,grids,mask,info,
            args.output,wait,shape=(128,128),shift_m=.65,file_prefix='pilot_',
            mask_on_revision=profile.get('mask_on_revision', False))
        if trace:
            manager.timer.callback=trace.wrap('window_heartbeat',manager.timer.callback,adapter.sim)
            manager.observe=trace.wrap('window_observe',manager.observe,adapter.sim)
            manager.resolve=trace.wrap('window_resolve',manager.resolve,adapter.sim)
        manager.publish_window(start)
        adapter.window_provider = manager
        report['stage']='native_activation'
        for name in ['planner_server','controller_server','bt_navigator']:
            transition(name,1)
            transition(name,3)
            activated.append(name)
        report['stage']='native_costmap_readiness'
        wait(lambda: manager.ready() and adapter.client.server_is_ready(), 40)
        report['stage']='window_native_preflight'
        manager.prepare()
        print('Native window preflight passed', flush=True)
        owners = [v.node_name for v in node.get_publishers_info_by_topic(profile['command_topic'])]
        if owners != ['controller_server'] or node.count_publishers('/cmd_vel'):
            raise RuntimeError('pilot_velocity_ownership_changed')
        replay = profile.get('local_stack_replay', False)
        if replay:
            consumers = node.get_subscriptions_info_by_topic(profile['command_topic'])
            if [item.node_name for item in consumers] != ['phase9_replay_inputs']:
                raise RuntimeError('replay_command_sink_required')
            report['scope'] = 'Pi-local recorded input replay with native planner/controller and command sink'
        report['command_owners'] = owners
        report['dispatch_sim'] = adapter.sim()
        report['stage']='dispatch_admission'
        report['dispatch_wall'] = time.monotonic()
        report['admission_attempt_sim']=report['dispatch_sim']
        report['admission_attempt_wall']=report['dispatch_wall']
        from dispatch_admission import begin_when_fresh
        wait(lambda: manager.ready_for_transition(manager.committed), 5)
        report['dispatch_admission']=begin_when_fresh(adapter,contract,manager.ready,step)
        sent=next(e for e in adapter.events if e['event']=='send')
        report['dispatch_sim']=sent['sim'];report['dispatch_wall']=sent['wall']
        report['stage']='motion'
        print('First navigation goal sent', flush=True)
        while adapter.model.state != 'succeeded':
            step()
            if adapter.fault_reason:
                raise RuntimeError(adapter.fault_reason)
            if adapter.model.state in ['failed','cancelled','cancelling']:
                raise RuntimeError('pilot_action_failed')
            if replay and adapter.sim() - report['dispatch_sim'] >= 8.0:
                if not any(event['event'] == 'accepted' and event['accepted'] for event in adapter.events):
                    raise RuntimeError('replay_action_not_accepted')
                adapter.cancel('replay_interval_complete')
                deadline = time.monotonic() + 5
                while adapter.model._unresolved() and time.monotonic() < deadline:
                    spin(timeout_sec=.05)
                if adapter.model._unresolved() or adapter.handles:
                    raise RuntimeError('replay_cancel_not_confirmed')
                report['replay_interval_completed'] = True
                break
            if adapter.sim()-report['dispatch_sim']>90:
                raise RuntimeError('pilot_sim_timeout')
        report['passed'] = True
    except Exception as error:
        report['error'] = repr(error)
        report['failure_clock_sim']=adapter.sim()
        report['last_scan_stamp']=adapter.scan_stamp
        report['native_grids']={k:dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
            shape=[m.info.height,m.info.width],origin=[m.info.origin.position.x,m.info.origin.position.y]) for k,m in grids.items()}
        if manager is not None and manager.pending is not None:
            pending=manager.pending
            report['pending_gate']=dict(errors=pending.get('last_errors',{}),
                observations=pending['gate'].observations,source_last_stamp=pending['gate'].last_source_stamp)
            import numpy as np
            np.savez_compressed(args.output/'preflight_failure.npz',physical=pending['window'].grid,
                mask=pending['snapshot'].mask,**{k:np.asarray(m.data).reshape(m.info.height,m.info.width) for k,m in grids.items()})
        try:
            adapter.cancel('phase9_pilot_error')
            until = time.monotonic()+8
            while adapter.handles and time.monotonic()<until:
                executor.spin_once(timeout_sec=.05)
        except Exception as cancel_error:
            report['cancel_error'] = repr(cancel_error)
    finally:
        if mission_timer is not None:
            with adapter.state_lock:
                adapter.enabled = False
            mission_timer.close()
        report.update(state=adapter.model.report(),events=adapter.events,
            poses=adapter.poses,tf_timing=adapter.tf_timing,imu_samples=adapter.imu_samples,
            elapsed_wall_s=time.monotonic()-began,final_sim=adapter.sim(),
            scan_observations=list(adapter.scan_observations))
        if manager is not None:
            manager.close()
            report['rolling'] = manager.report()
        adapter.enabled = False
        for name in reversed(activated):
            try:
                transition(name,4)
            except Exception as error:
                report.setdefault('shutdown_errors',[]).append(str(error))
                report['passed'] = False
        status.publish(String(data=json.dumps(dict(passed=report['passed'],state=report['state'],
            error=report.get('error'),goal=report['goal']))))
        until = time.monotonic()+3
        while time.monotonic()<until:
            executor.spin_once(timeout_sec=.05)
        (args.output/'worker.json').write_text(json.dumps(report,indent=2)+'\n')
        executor.shutdown()
        if tf_listener is not None:
            tf_listener.close()
        adapter.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        if trace:(args.output/'callback_trace.json').write_text(json.dumps(trace.report(),indent=2)+'\n')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
