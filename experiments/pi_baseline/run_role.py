#!/usr/bin/env python3
"""Bounded per-host process owner for one Phase 09 pilot. No physical drivers."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role',choices=['pi','simulator'],required=True)
    parser.add_argument('--workspace',type=Path,required=True)
    args=parser.parse_args();w=args.workspace.resolve()
    manifest=json.loads((w/'phase9_manifest.json').read_text())
    for name,want in manifest['files_sha256'].items():
        if hashlib.sha256((w/name).read_bytes()).hexdigest()!=want:
            raise RuntimeError('phase9_source_mismatch:'+name)
    (w/'role.pid').write_text(str(os.getpid()))
    def interrupted(*_):raise RuntimeError('pilot_interrupted')
    signal.signal(signal.SIGTERM,interrupted)
    signal.signal(signal.SIGINT,interrupted)
    out=w/'results';out.mkdir(exist_ok=False)
    env=dict(os.environ)
    profile=json.loads((w/'profile.json').read_text())
    env.update(ROS_DOMAIN_ID='218',ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
        CYCLONEDDS_URI='file://'+str(w/'dds.xml'),IGN_PARTITION=profile['partition'],
        EDGEOSMAG_PHASE9_PROFILE=str(w/'profile.json'),ROS_LOG_DIR=str(out/'ros_logs'),
        PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    if profile.get('local_stack_replay'):
        if args.role != 'pi':
            raise ValueError('local_replay_requires_pi_role')
        env['ROS_LOCALHOST_ONLY'] = '1'
    for path in Path('/proc').glob('[0-9]*'):
        if int(path.name)==os.getpid():continue
        try:
            if b'ROS_DOMAIN_ID=218' in (path/'environ').read_bytes().split(b'\0'):
                raise RuntimeError('phase9_domain_in_use')
        except OSError:pass
    if args.role=='simulator':
        for proc in Path('/proc').glob('[0-9]*'):
            try:
                if proc.stat().st_uid!=1000:continue
                values=dict(v.split('=',1) for v in (proc/'environ').read_text().split('\0') if '=' in v)
                if values.get('DISPLAY') and values.get('XAUTHORITY'):
                    env.update({k:values[k] for k in ['DISPLAY','XAUTHORITY','XDG_RUNTIME_DIR'] if k in values})
            except (OSError,ValueError):pass
        if not env.get('DISPLAY'):raise RuntimeError('simulator_desktop_required')
        env.update(LIBGL_ALWAYS_SOFTWARE='1',LIBGL_DRI3_DISABLE='1')
    children=[];logs=[];sampler=None;result=dict(role=args.role,passed=False,scope='One split-host simulation pilot, not a completed benchmark')
    began=time.monotonic()
    def launch(name,command):
        log=(out/(name+'.log')).open('w');logs.append(log)
        child=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append((name,child));return child
    try:
        if args.role=='pi':
            from resources_phase9 import Sampler
            sampler=Sampler();sampler.start()
            if profile.get('local_stack_replay'):
                start_ns, end_ns = profile['stationary_clip_ns']
                launch('replay_feed', [sys.executable, str(w/'replay_stack.py'),
                    '--recording',str(w/'sensor_recording.json.gz'),'--output',str(out/'replay_feed.json'),
                    '--start-ns',str(start_ns),'--end-ns',str(end_ns)])
            if profile.get('timing_trace') and profile.get('independent_arrival_probe', True):
                probe_command = [sys.executable, str(w/'arrival_probe.py'), '--output', str(out/'arrival_probe.json')]
                if profile.get('pipeline_trace'):
                    probe_command.extend(['--pipeline', 'pi'])
                launch('arrival_probe', probe_command)
            if profile.get('reliable_scan_link'):
                launch('scan_receiver',[sys.executable,str(w/'scan_link.py'),'--role','receiver','--output',str(out/'scan_receiver.json')])
            localizer_launch = 'gcu_amcl.launch.py'
            if profile.get('reliable_localizer_inputs') and not profile.get('local_stack_replay'):
                launch('localizer_input_relay', [sys.executable, str(w/'localizer_input_relay.py'),
                                                  '--output', str(out/'localizer_input_relay.json')])
                localizer_launch = 'gcu_amcl_reliable_inputs.launch.py'
            launch('localization',['ros2','launch',str(w/localizer_launch),
                'map:='+str(w/'assets/gcu_ground_floor.yaml'),'params:='+str(w/'assets/amcl.yaml')])
            log=(out/'localization_ready.log').open('w');logs.append(log)
            ready=subprocess.run([sys.executable,str(w/'wait_gcu_localization.py'),'--request-updates'],
                env=env,stdout=log,stderr=subprocess.STDOUT,timeout=65)
            if ready.returncode:raise RuntimeError('pi_localization_not_ready')
            for package,name in [('nav2_planner','planner_server'),('nav2_controller','controller_server'),('nav2_bt_navigator','bt_navigator')]:
                launch(name,['/opt/ros/humble/lib/'+package+'/'+name,'--ros-args',
                    '--params-file',str(w/'assets/nav2_sensor_constraints.yaml'),
                    '-r','/goal_pose:=/phase6_disabled_goal_pose','-r','/cmd_vel:='+profile['command_topic']])
            actor=launch('actor',[sys.executable,str(w/'worker_phase9.py'),'--workspace',str(w),
                '--output',str(out/'actor'),'--execute'])
            actor.wait(timeout=285)
            result['passed']=actor.returncode==0 and json.loads((out/'actor/worker.json').read_text())['passed']
        else:
            if profile.get('pipeline_trace'):
                launch('pipeline_arrival', [sys.executable, str(w/'arrival_probe.py'), '--pipeline', 'simulator',
                                             '--output', str(out/'pipeline_arrival.json')])
            if profile.get('capture_inputs'):
                scan_topic = '/phase9/source_scan' if profile.get('reliable_scan_link') else '/scan'
                launch('sensor_capture',[sys.executable,str(w/'sensor_recording.py'),'capture',
                    '--recording',str(out/'sensor_recording.json.gz'),'--scan-topic',scan_topic])
            if profile.get('reliable_scan_link'):
                launch('scan_sender',[sys.executable,str(w/'scan_link.py'),'--role','sender','--output',str(out/'scan_sender.json')])
            if profile.get('clock_transport_hz'):
                launch('clock_transport',[sys.executable,str(w/'clock_transport.py'),
                    '--rate-hz',str(profile['clock_transport_hz']),'--output',str(out/'clock_transport.json')])
            observer_command=[sys.executable,str(w/'observer_phase9.py'),'--workspace',str(w),'--output',str(out/'observer.json')]
            if profile.get('reliable_scan_link'):observer_command+=['--scan-topic','/phase9/source_scan']
            observer=launch('observer',observer_command)
            launch('simulator',['ros2','launch',str(w/'gcu_phase9_sim.launch.py'),
                'assets:='+str(w/'assets/contact_v2'),'world:='+str(w/'assets/contact_v2/world.sdf')])
            observer.wait(timeout=345)
            result['passed']=observer.returncode==0
    except Exception as error:result['error']=repr(error)
    finally:
        # Finish bounded cleanup even if the host repeats a stop request.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        signal.signal(signal.SIGTERM,signal.SIG_IGN)
        if args.role=='simulator':
            try:
                from gcu_process_audit import stop_truth_bridge
                from gcu_quiescent_shutdown import stop_quiescent_simulator
                result['truth_shutdown']=stop_truth_bridge(env['IGN_PARTITION'])
                result['simulator_shutdown']=stop_quiescent_simulator(env['IGN_PARTITION'])
            except Exception as error:result['shutdown_error']=repr(error)
        for name,child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    result.setdefault('forced_cleanup',[]).append(name)
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        if sampler:
            sampler.stop();(out/'resources.json').write_text(json.dumps(dict(scope='One-second process RSS/PSS and CPU-time samples; pilot, not repeated benchmark',samples=sampler.rows),indent=2)+'\n')
        for log in logs:log.close()
        result.update(process_exits={name:c.returncode for name,c in children},elapsed_wall_s=time.monotonic()-began)
        result['passed']=bool(result['passed'] and not result.get('forced_cleanup') and
            all(c.returncode==0 for _,c in children) and not result.get('shutdown_error'))
        if args.role=='simulator':
            result['passed']=bool(result['passed'] and result.get('truth_shutdown',{}).get('passed')
                and result.get('simulator_shutdown',{}).get('passed'))
        (out/'role.json').write_text(json.dumps(result,indent=2)+'\n')
    return 0 if result['passed'] else 1


if __name__=='__main__':
    try:
        code=main()
    except Exception as error:
        # Preserve a source/profile/display startup failure even before child
        # ownership begins, so the host need not wait for a nonexistent role.
        if '--workspace' in sys.argv:
            workspace=Path(sys.argv[sys.argv.index('--workspace')+1])
            result=workspace/'results/role.json'
            if not result.exists():
                result.parent.mkdir(parents=True,exist_ok=True)
                result.write_text(json.dumps(dict(passed=False,startup_failure=repr(error),
                    process_exits={},scope='Pilot startup failure; no motion result'),indent=2)+'\n')
        import traceback
        traceback.print_exc()
        code=1
    raise SystemExit(code)
