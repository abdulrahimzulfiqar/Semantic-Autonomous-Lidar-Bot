#!/usr/bin/env python3
"""Bounded, stationary GCU sensor integration runner; no navigation goals."""
import argparse,json,os,shlex,signal,subprocess,time
from pathlib import Path
from gcu_process_audit import audit_logs,stop_truth_bridge,stop_simulator


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--domain',type=int,default=194);a=p.parse_args();w=a.workspace.resolve();a.output.mkdir(parents=True,exist_ok=False)
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
    env.update(ROS_DOMAIN_ID=str(a.domain),RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',IGN_PARTITION='gcu_06b2a_'+str(os.getpid()),LIBGL_ALWAYS_SOFTWARE='1',LIBGL_DRI3_DISABLE='1',PYTHONDONTWRITEBYTECODE='1')
    setup='source /opt/ros/humble/setup.bash; source /home/ubuntu/lidarbot_ws/install/setup.bash; exec '
    processes=[];logs=[];status={'passed':False,'domain':a.domain,'partition':env['IGN_PARTITION'],'scope':'Stationary GCU sensor/costmap integration only'}
    def launch(name,command):
        log=(a.output/(name+'.log')).open('w');logs.append(log)
        proc=subprocess.Popen(['/bin/bash','-c',setup+shlex.join(command)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(proc)
    begin=time.monotonic()
    try:
        launch('simulator',['ros2','launch',str(w/'gcu_phase5_sim.launch.py'),'assets:='+str(w/'assets/contact_v2'),'world:='+str(w/'assets/contact_v2/world.sdf')]);time.sleep(6)
        launch('amcl',['ros2','launch',str(w/'gcu_amcl.launch.py'),'map:='+str(w/'assets/gcu_ground_floor.yaml'),'params:='+str(w/'assets/amcl.yaml')])
        with (a.output/'localization.log').open('w') as log:
            ready=subprocess.run(['/bin/bash','-c',setup+shlex.join(['python3',str(w/'wait_gcu_localization.py'),'--request-updates'])],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=65)
        if ready.returncode:raise RuntimeError('localization_not_ready')
        for package,node in [('nav2_planner','planner_server'),('nav2_controller','controller_server')]:
            launch(node,['/opt/ros/humble/lib/'+package+'/'+node,'--ros-args','--params-file',str(w/'assets/nav2_sensor_constraints.yaml')])
        with (a.output/'checker.log').open('w') as log:
            checked=subprocess.run(['/bin/bash','-c',setup+shlex.join(['python3',str(w/'check_gcu_sensor_constraints.py'),'--workspace',str(w),'--output',str(a.output/'report.json')])],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180)
        status['checker_exit']=checked.returncode;status['passed']=checked.returncode==0 and json.loads((a.output/'report.json').read_text())['passed']
    except Exception as e:status['error']=repr(e)
    finally:
        status['truth_shutdown']=stop_truth_bridge(env['IGN_PARTITION']);status['simulator_shutdown']=stop_simulator(env['IGN_PARTITION'])
        for proc in reversed(processes):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try:proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for log in logs:log.close()
        status['process_exits']=[p.returncode for p in processes];status['audit']=audit_logs(a.output)
        status['passed']=bool(status['passed'] and all(v==0 for v in status['process_exits']) and not status['audit']['critical'] and status['truth_shutdown']['passed'] and status['simulator_shutdown']['passed'])
        status['elapsed_wall_s']=time.monotonic()-begin;(a.output/'run.json').write_text(json.dumps(status,indent=2)+'\n')
    return 0 if status['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
