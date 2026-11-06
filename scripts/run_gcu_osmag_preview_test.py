#!/usr/bin/env python3
"""Fresh simulation/localization/route-preview test with no navigation controller."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time
from gcu_process_audit import stop_truth_bridge,stop_simulator,audit_logs


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--navigation-workspace',type=Path,required=True)
    parser.add_argument('--preview-workspace',type=Path,required=True)
    parser.add_argument('--robot-workspace',type=Path,default=Path('/home/ubuntu/lidarbot_ws'))
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--domain',type=int,default=160)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    w=args.navigation_workspace;p=args.preview_workspace;env=os.environ.copy()
    for proc in Path('/proc').iterdir():
        try:
            if not proc.name.isdigit() or proc.stat().st_uid!=os.getuid():continue
            e=dict(x.split('=',1) for x in (proc/'environ').read_text().split('\0') if '=' in x)
            if 'DISPLAY' in e and 'XAUTHORITY' in e:
                env.update({k:e[k] for k in ['DISPLAY','XAUTHORITY','XDG_RUNTIME_DIR'] if k in e});break
        except OSError:pass
    env.update(ROS_DOMAIN_ID=str(args.domain),RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',IGN_PARTITION='gcu_preview_'+str(os.getpid()),LIBGL_ALWAYS_SOFTWARE='1',LIBGL_DRI3_DISABLE='1')
    setup='source /opt/ros/humble/setup.bash; source '+shlex.quote(str(args.robot_workspace/'install/setup.bash'))+'; exec '
    commands=[('simulator',['ros2','launch',str(w/'gcu_phase5_sim.launch.py'),'assets:='+str(w/'assets/contact_v2'),'world:='+str(w/'assets/contact_v2/world.sdf')]),
        ('amcl',['ros2','launch',str(w/'gcu_amcl.launch.py'),'map:='+str(w/'assets/gcu_ground_floor.yaml'),'params:='+str(w/'assets/amcl.yaml')]),
        ('preview',['ros2','launch',str(p/'gcu_osmag_preview.launch.py'),'assets:='+str(p/'assets')])]
    def preview_hashes():
        files=list(p.glob('*.py'))
        for folder in ['assets','probe_src','probe_install']:
            files += [f for f in (p/folder).rglob('*') if f.is_file()]
        return {str(f.relative_to(p)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(files)}
    frozen=preview_hashes()
    processes=[];logs=[];result={'passed':False,'domain':args.domain,'partition':env['IGN_PARTITION'],
                               'preview_source_sha256':frozen}
    try:
        for name,cmd in commands:
            log=(args.output/(name+'.log')).open('w');logs.append(log)
            processes.append(subprocess.Popen(['/bin/bash','-c',setup+shlex.join(cmd)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            if name=='simulator':time.sleep(6)
            if name=='amcl':
                with (args.output/'readiness.log').open('w') as log:
                    ready=subprocess.run(['/bin/bash','-c',setup+shlex.join(['python3',str(w/'wait_gcu_localization.py'),'--request-updates'])],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=55)
                if ready.returncode:raise RuntimeError('localization_readiness_failed')
        with (args.output/'checker.log').open('w') as log:
            command=['python3',str(p/'check_gcu_osmag_preview.py'),'--assets',str(p/'assets'),'--output',str(args.output/'report.json')]
            checked=subprocess.run(['/bin/bash','-c',setup+shlex.join(command)],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=240)
        result['checker_exit']=checked.returncode
        result['passed']=checked.returncode==0 and json.loads((args.output/'report.json').read_text())['passed']
    except Exception as error:result['error']=repr(error)
    finally:
        result['truth_bridge_stop']=stop_truth_bridge(env['IGN_PARTITION'])
        result['simulator_stop']=stop_simulator(env['IGN_PARTITION'])
        for proc in reversed(processes):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=20)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for log in logs:log.close()
        result['launch_exits']=[proc.returncode for proc in processes]
        result['child_audit']=audit_logs(args.output)
        result['inputs_unchanged']=preview_hashes()==frozen
        result['passed']=bool(result['passed'] and result['inputs_unchanged'] and result['truth_bridge_stop']['passed'] and result['simulator_stop']['passed'] and not result['child_audit']['critical'] and all(v==0 for v in result['launch_exits']))
        (args.output/'run.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
