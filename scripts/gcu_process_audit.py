"""Scope process shutdown to one test partition and audit launch child failures."""
import os
from pathlib import Path
import re
import signal
import subprocess
import time


def audit_logs(folder):
    critical = []
    pattern = re.compile(r'process has died.*exit code (-?\d+)|Segmentation fault|core dumped|terminate called|couldn.t find shared library')
    for path in sorted(Path(folder).glob('*.log')):
        for number, line in enumerate(path.read_text(errors='replace').splitlines(), 1):
            match = pattern.search(line)
            if match and (match.group(1) is None or int(match.group(1)) != 0):
                critical.append({'file': path.name, 'line': number, 'message': line})
    return {'critical': critical, 'scope': 'launch child errors and fatal log patterns; group exit 0 is insufficient'}


def stop_truth_bridge(partition, timeout=5.):
    """Stop only this trial's evaluation bridge while Gazebo is still alive.

    ROS launch remains its parent and records the actual exit code. This ordering
    is a mitigation to test, not a claim about the native crash's root cause.
    """
    pids = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            args = (proc/'cmdline').read_bytes().split(b'\0')
            env = (proc/'environ').read_bytes().split(b'\0')
            owned = ('IGN_PARTITION='+partition).encode() in env
            bridge = args and args[0].endswith((b'/parameter_bridge',b'/gcu_truth_bridge'))
            truth = any(b'/model/lidarbot/pose' in arg for arg in args)
            if owned and bridge and truth:
                os.kill(int(proc.name), signal.SIGINT)
                pids.append(int(proc.name))
        except (OSError, ProcessLookupError):
            continue
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        if all(not (Path('/proc')/str(pid)).exists() for pid in pids):
            return {'passed': len(pids)==1, 'pids': pids, 'order': 'truth bridge before simulator group'}
        time.sleep(.05)
    return {'passed': False, 'pids': pids, 'error': 'truth bridge did not exit before simulator shutdown'}


def stop_simulator(partition):
    """Wait for server teardown before its launcher can forward another signal.

    The RPC acknowledgement precedes renderer/thread destruction. Signalling
    that process immediately can interrupt teardown; acknowledgement alone is
    therefore not sufficient evidence of a completed stop.
    """
    servers=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            env=(proc/'environ').read_bytes().split(b'\0')
            owned=('IGN_PARTITION='+partition).encode() in env
            # Fortress's Ruby CLI rewrites argv; identify the actual loaded
            # server entrypoint rather than guessing from command-line text.
            if owned and 'libignition-gazebo6-ign.so' in (proc/'maps').read_text():
                servers.append(proc)
        except OSError:continue
    env=dict(os.environ,IGN_PARTITION=partition)
    command=['ign','service','-s','/server_control','--reqtype','ignition.msgs.ServerControl',
             '--reptype','ignition.msgs.Boolean','--timeout','5000','--req','stop: true']
    try:
        result=subprocess.run(command,env=env,capture_output=True,text=True,timeout=8)
        acknowledged=result.returncode==0 and 'data: true' in result.stdout
        deadline=time.monotonic()+20.
        alive=list(servers)
        while acknowledged and alive and time.monotonic()<deadline:
            active=[]
            for proc in alive:
                try:
                    if (proc/'stat').read_text().split(') ',1)[1].split()[0]!='Z':active.append(proc)
                except OSError:pass
            alive=active
            if alive:time.sleep(.05)
        return {'passed':acknowledged and bool(servers) and not alive,'command':command,
                'stdout':result.stdout,'stderr':result.stderr,'exit':result.returncode,
                'server_pids':[int(p.name) for p in servers],
                'still_alive_pids':[int(p.name) for p in alive]}
    except subprocess.TimeoutExpired:return {'passed':False,'error':'Gazebo server-control timeout'}
