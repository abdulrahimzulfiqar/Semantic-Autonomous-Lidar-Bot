"""Candidate sensor-shutdown mitigation, scoped to the current simulation.

Pausing changes no mission decision: call only after cancellation/deactivation.
The existing server-exit deadline and strict child audit remain authoritative.
"""
import os,subprocess,time
from gcu_process_audit import stop_simulator


def stop_quiescent_simulator(partition):
    command=['ign','service','-s','/world/gcu_ground_floor/control',
        '--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean',
        '--timeout','3000','--req','pause: true']
    try:
        pause=subprocess.run(command,env=dict(os.environ,IGN_PARTITION=partition),capture_output=True,text=True,timeout=5)
        paused=pause.returncode==0 and 'data: true' in pause.stdout
        if paused:time.sleep(.25)
        result=stop_simulator(partition)
        result['pause_before_stop']={'acknowledged':paused,'stdout':pause.stdout,'stderr':pause.stderr,'exit':pause.returncode,
            'scope':'Pause RPC acknowledgement plus 0.25 s quiescence interval; not a separate renderer-thread ACK.'}
        result['passed']=bool(result['passed'] and paused)
        return result
    except subprocess.TimeoutExpired:
        result=stop_simulator(partition);result['passed']=False;result['pause_before_stop']={'error':'pause_rpc_timeout'};return result
