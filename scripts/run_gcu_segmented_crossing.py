#!/usr/bin/env python3
"""Bounded 06B2B simulation-only mission crossing; explicit --execute required."""
import argparse,json,os,shlex,signal,subprocess,time
from pathlib import Path
from gcu_process_audit import audit_logs,stop_truth_bridge
from gcu_quiescent_shutdown import stop_quiescent_simulator as stop_simulator


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--domain',type=int,default=195);p.add_argument('--execute',action='store_true');p.add_argument('--fixture',type=Path);
    p.add_argument('--diagnose-stationary',action='store_true')
    p.add_argument('--cancel-near-handoff',action='store_true');p.add_argument('--close-upcoming-passage',action='store_true');p.add_argument('--drop-required-imu',action='store_true');p.add_argument('--metric-baseline',action='store_true')
    p.add_argument('--interactive',action='store_true');p.add_argument('--foxglove',action='store_true');p.add_argument('--session-wall-timeout',type=float,default=900.);a=p.parse_args()
    if a.interactive and (a.metric_baseline or a.drop_required_imu or a.close_upcoming_passage or a.cancel_near_handoff or a.diagnose_stationary or not a.fixture or not 0<a.session_wall_timeout<=1200):p.error('interactive mode requires one seed profile and a bounded session')
    if a.foxglove and not a.interactive:p.error('Foxglove is enabled only for interactive review')
    if a.metric_baseline and (a.drop_required_imu or a.close_upcoming_passage or a.cancel_near_handoff or a.diagnose_stationary or not a.fixture):p.error('baseline requires an admitted normal-motion fixture')
    if a.drop_required_imu and (a.cancel_near_handoff or a.close_upcoming_passage or a.diagnose_stationary or not a.fixture):p.error('IMU loss requires one admitted motion fixture and no other scenario')
    if a.close_upcoming_passage and (a.cancel_near_handoff or a.diagnose_stationary or not a.fixture):p.error('closure requires one admitted motion fixture and no other scenario')
    if a.cancel_near_handoff and (a.diagnose_stationary or not a.fixture):p.error('cancellation requires one admitted fixture and motion')
    if not a.execute or a.domain!=195:p.error('--execute and isolated domain 195 are required')
    w=a.workspace.resolve();a.output.mkdir(parents=True,exist_ok=False)
    fixture=None
    if a.fixture:
        from gcu_mission_fixture import load_fixture,verify_runtime_sources
        fixture,_=load_fixture(a.fixture)
        verify_runtime_sources(fixture,w)
    for name in ['assets/contact_v2/contact_manifest.json','assets/cyclonedds_local.xml','assets/gcu_ground_floor.osm','assets/nav2_sensor_constraints.yaml']:
        if not (w/name).is_file():raise RuntimeError('missing_preflight_asset:'+name)
    env=os.environ.copy()
    for proc in Path('/proc').iterdir():
        try:
            if not proc.name.isdigit():continue
            entries=(proc/'environ').read_bytes().split(b'\0')
            if ('ROS_DOMAIN_ID='+str(a.domain)).encode() in entries:raise RuntimeError('test_domain_already_in_use')
            if proc.stat().st_uid!=os.getuid():continue
            values=dict(s.split('=',1) for s in (proc/'environ').read_text().split('\0') if '=' in s)
            if 'DISPLAY' in values and 'XAUTHORITY' in values:
                env.update({k:values[k] for k in ['DISPLAY','XAUTHORITY','XDG_RUNTIME_DIR'] if k in values})
        except (OSError,ValueError):pass
    if not env.get('DISPLAY'):raise RuntimeError('logged_in_desktop_required_for_sensors')
    env.update(ROS_DOMAIN_ID=str(a.domain),RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',IGN_PARTITION='gcu_phase6b2b_'+str(os.getpid()),LIBGL_ALWAYS_SOFTWARE='1',LIBGL_DRI3_DISABLE='1',PYTHONDONTWRITEBYTECODE='1',ROS_LOCALHOST_ONLY='1',CYCLONEDDS_URI='file://'+str(w/'assets/cyclonedds_local.xml'))
    setup='source /opt/ros/humble/setup.bash; source /home/ubuntu/lidarbot_ws/install/setup.bash; exec '
    processes=[];logs=[];status={'passed':False,'domain':a.domain,'partition':env['IGN_PARTITION'],'scope':'06B2B single real-Nav2 constrained crossing, simulation only',
        'scenario':'interactive' if a.interactive else 'metric_baseline' if a.metric_baseline else 'imu_loss' if a.drop_required_imu else 'closure' if a.close_upcoming_passage else 'cancel_near_handoff' if a.cancel_near_handoff else 'nominal'}
    def launch(name,command):
        log=(a.output/(name+'.log')).open('w');logs.append(log)
        proc=subprocess.Popen(['/bin/bash','-c',setup+shlex.join(command)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(proc)
    begin=time.monotonic();checker_process=None
    try:
        launch('simulator',['ros2','launch',str(w/'gcu_phase6_sim.launch.py'),'assets:='+str(w/'assets/contact_v2'),'world:='+str(w/'assets/contact_v2/world.sdf')]);time.sleep(6)
        launch('amcl',['ros2','launch',str(w/'gcu_amcl.launch.py'),'map:='+str(w/'assets/gcu_ground_floor.yaml'),'params:='+str(w/'assets/amcl.yaml')])
        with (a.output/'localization.log').open('w') as log:
            ready=subprocess.run(['/bin/bash','-c',setup+shlex.join(['python3',str(w/'wait_gcu_localization.py'),'--request-updates'])],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=65)
        if ready.returncode:raise RuntimeError('localization_not_ready')
        for package,node in [('nav2_planner','planner_server'),('nav2_controller','controller_server'),('nav2_bt_navigator','bt_navigator')]:
            launch(node,['/opt/ros/humble/lib/'+package+'/'+node,'--ros-args','--params-file',str(w/'assets/nav2_sensor_constraints.yaml'),'-r','/goal_pose:=/phase6_disabled_goal_pose'])
        if a.foxglove:launch('foxglove',['/opt/ros/humble/lib/foxglove_bridge/foxglove_bridge','--ros-args','-p','use_sim_time:=true','-p','address:=0.0.0.0','-p','port:=8765','-p','capabilities:=[clientPublish,services,connectionGraph,assets]'])
        command=['python3',str(w/'check_gcu_segmented_crossing.py'),'--workspace',str(w),'--output',str(a.output/'report.json'),'--execute']
        if a.fixture:command+=['--fixture',str(a.fixture.resolve())]
        if a.diagnose_stationary:command+=['--diagnose-stationary']
        if a.cancel_near_handoff:command+=['--cancel-near-handoff']
        if a.close_upcoming_passage:command+=['--close-upcoming-passage']
        if a.drop_required_imu:command+=['--drop-required-imu']
        if a.metric_baseline:command+=['--metric-baseline']
        if a.interactive:command+=['--interactive','--session-wall-timeout',str(a.session_wall_timeout)]
        with (a.output/'checker.log').open('w') as log:
            if a.interactive:
                checked=subprocess.Popen(['/bin/bash','-c',setup+shlex.join(command)],env=env,stdout=log,stderr=subprocess.STDOUT)
                checker_process=checked
                try:checked.wait(timeout=a.session_wall_timeout+45)
                except KeyboardInterrupt:
                    checked.send_signal(signal.SIGINT);checked.wait(timeout=30)
            else:checked=subprocess.run(['/bin/bash','-c',setup+shlex.join(command)],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=(fixture['checker_wall_timeout_s']+20 if fixture else 440))
        status['checker_exit']=checked.returncode;status['passed']=checked.returncode==0 and json.loads((a.output/'report.json').read_text())['passed']
    except Exception as e:status['error']=repr(e)
    finally:
        if checker_process is not None and checker_process.poll() is None:
            checker_process.send_signal(signal.SIGINT)
            try:checker_process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                checker_process.kill();checker_process.wait()
            status['checker_forced_cleanup']=True;status['passed']=False
        status['truth_shutdown']=stop_truth_bridge(env['IGN_PARTITION']);status['simulator_shutdown']=stop_simulator(env['IGN_PARTITION'])
        if status['simulator_shutdown'].get('still_alive_pids'):
            (a.output/'shutdown_probe_request.json').write_text(json.dumps({'partition':env['IGN_PARTITION'],'server_pids':status['simulator_shutdown']['still_alive_pids']}))
            deadline=time.monotonic()+20
            while not (a.output/'shutdown_probe_done').exists() and time.monotonic()<deadline:time.sleep(.1)
        for proc in reversed(processes):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try:proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for log in logs:log.close()
        status['process_exits']=[p.returncode for p in processes];status['process_pids']=[p.pid for p in processes];status['audit']=audit_logs(a.output)
        status['passed']=bool(status['passed'] and all(v==0 for v in status['process_exits']) and not status['audit']['critical'] and status['truth_shutdown']['passed'] and status['simulator_shutdown']['passed'])
        status['elapsed_wall_s']=time.monotonic()-begin;(a.output/'run.json').write_text(json.dumps(status,indent=2)+'\n')
    return 0 if status['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
