#!/usr/bin/env python3
"""Mac-side owner for one explicitly enabled split-host pilot, not a benchmark."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import tarfile
import time

UTM='/Applications/UTM.app/Contents/MacOS/utmctl'


def apple_string(value):
    return '"'+value.replace('\\','\\\\').replace('"','\\"').replace('\n','\\n')+'"'


def guest(vm,command,timeout=50):
    script='\n'.join(['tell application "UTM"',
        'set targetVM to first virtual machine whose id is '+apple_string(vm),
        'tell (execute of targetVM at "/bin/bash" with arguments {"-lc", '+apple_string(command)+'} with output capturing)',
        'repeat '+str(int(timeout*10))+' times','set res to get result',
        'if exited of res then exit repeat','delay 0.1','end repeat',
        'if not exited of res then error "guest command timeout"',
        'if exit code of res is not 0 then error ("guest command failed: " & error text of res)',
        'return output text of res','end tell','end tell'])
    return subprocess.run(['osascript'],input=script,text=True,capture_output=True,
        check=True,timeout=timeout+10).stdout


def archive(source,destination):
    with tarfile.open(destination,'w:gz') as target:
        for path in source.rglob('*'):
            if path.is_file():target.add(path,arcname=str(path.relative_to(source)))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['stage','output','key','pi-root','sim-root']:p.add_argument('--'+name,type=Path,required=True)
    for name in ['pi-host','vm']:p.add_argument('--'+name,required=True)
    p.add_argument('--execute',action='store_true');a=p.parse_args()
    if not a.execute:p.error('--execute required')
    a.output.mkdir(parents=True,exist_ok=False)
    ssh=['ssh','-i',str(a.key),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=10',a.pi_host]
    outcome=dict(passed=False,scope='One Wi-Fi split-host simulation pilot; independent scoring separate')
    (a.output/'orchestration.json').write_text(json.dumps(outcome,indent=2)+'\n')
    began=time.monotonic()
    # The sensor renderer requires the user's logged-in display, not GDM.
    guest(a.vm,'pgrep -u 1000 -x gnome-shell >/dev/null')
    pi_archive=a.output/'pi_sources.tar.gz';vm_archive=a.output/'simulator_sources.tar.gz'
    archive(a.stage/'pi',pi_archive);archive(a.stage/'simulator',vm_archive)
    subprocess.run(ssh+['mkdir '+shlex.quote(str(a.pi_root))],check=True,timeout=20)
    with pi_archive.open('rb') as data:
        subprocess.run(ssh+['tar -xzf - -C '+shlex.quote(str(a.pi_root))],stdin=data,check=True,timeout=40)
    guest(a.vm,'mkdir '+shlex.quote(str(a.sim_root)))
    remote_archive='/tmp/'+a.sim_root.name+'.tar.gz'
    with vm_archive.open('rb') as data:
        subprocess.run([UTM,'file','push',a.vm,remote_archive],stdin=data,check=True,timeout=40)
    guest(a.vm,'tar -xzf '+shlex.quote(remote_archive)+' -C '+shlex.quote(str(a.sim_root)))
    vm_command='source /opt/ros/humble/setup.bash; source /home/ubuntu/lidarbot_ws/install/setup.bash; exec '+shlex.join([
        'python3',str(a.sim_root/'run_role.py'),'--role','simulator','--workspace',str(a.sim_root)])
    pid=guest(a.vm,'nohup /bin/bash -lc '+shlex.quote(vm_command)+' >'+shlex.quote(str(a.sim_root/'role_console.log'))+' 2>&1 < /dev/null & echo $!').strip()
    outcome['simulator_role_pid']=int(pid)
    print('Simulator role started; starting Pi localization/navigation.',flush=True)
    try:
        pi_command='source /opt/ros/humble/setup.bash; exec '+shlex.join([
            'python3',str(a.pi_root/'run_role.py'),'--role','pi','--workspace',str(a.pi_root)])
        result=subprocess.run(ssh+['bash -lc '+shlex.quote(pi_command)],capture_output=True,text=True,timeout=370)
        (a.output/'pi_console.log').write_text(result.stdout+result.stderr)
        outcome['pi_role_exit']=result.returncode
        print('Pi role exited:',result.returncode,flush=True)
    except Exception as error:
        outcome['error']=repr(error)
        stop='import os,signal;from pathlib import Path;os.kill(int(Path('+repr(str(a.pi_root/'role.pid'))+').read_text()),signal.SIGINT)'
        subprocess.run(ssh+['python3 -c '+shlex.quote(stop)],capture_output=True,timeout=20)
    finally:
        if outcome.get('pi_role_exit')!=0:
            guest(a.vm,'touch '+shlex.quote(str(a.sim_root/'results/stop_observer')))
        until=time.monotonic()+65
        while time.monotonic()<until:
            if guest(a.vm,'test -f '+shlex.quote(str(a.sim_root/'results/role.json'))+' && echo ready || true').strip():break
            time.sleep(1)
        else:
            outcome['simulator_stop_timeout']=True
            stop='import os,signal;from pathlib import Path;p='+str(int(pid))+';cmd=Path("/proc/"+str(p)+"/cmdline");'+\
                'os.kill(p,signal.SIGINT) if cmd.exists() and b"run_role.py" in cmd.read_bytes() and '+repr(str(a.sim_root).encode())+' in cmd.read_bytes() else None'
            guest(a.vm,'python3 -c '+shlex.quote(stop))
            time.sleep(3)
        try:
            with (a.output/'pi_results.tar.gz').open('wb') as data:
                subprocess.run(ssh+['tar --exclude=latest -czf - -C '+shlex.quote(str(a.pi_root))+' results'],
                    stdout=data,check=True,timeout=90)
            with tarfile.open(a.output/'pi_results.tar.gz') as target:
                target.extractall(a.output/'pi_collected',filter='data')
            import shutil
            shutil.move(str(a.output/'pi_collected/results'),str(a.output/'pi_results'))
        except Exception as error:outcome['pi_collection_error']=repr(error)
        try:
            guest(a.vm,'tar --exclude=latest -czf '+shlex.quote(str(a.sim_root/'results.tar.gz'))+
                ' -C '+shlex.quote(str(a.sim_root))+' results role_console.log')
            with (a.output/'simulator_results.tar.gz').open('wb') as data:
                subprocess.run([UTM,'file','pull',a.vm,str(a.sim_root/'results.tar.gz')],stdout=data,check=True,timeout=50)
            with tarfile.open(a.output/'simulator_results.tar.gz') as target:
                target.extractall(a.output/'simulator_results',filter='data')
        except Exception as error:outcome['simulator_collection_error']=repr(error)
        def role(path):
            return json.loads(path.read_text()) if path.exists() else dict(passed=False,error='role_report_missing')
        pi=role(a.output/'pi_results/role.json')
        vm=role(a.output/'simulator_results/results/role.json')
        outcome['pi']=pi;outcome['simulator']=vm
        outcome['passed']=pi['passed'] and vm['passed'] and not any(outcome.get(k) for k in
            ['error','simulator_stop_timeout','pi_collection_error','simulator_collection_error'])
        outcome['elapsed_wall_s']=time.monotonic()-began
        (a.output/'orchestration.json').write_text(json.dumps(outcome,indent=2)+'\n')
    print(json.dumps(outcome,indent=2),flush=True)
    return 0 if outcome['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
