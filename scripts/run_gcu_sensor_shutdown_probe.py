#!/usr/bin/env python3
"""Stationary 06B2A diagnostic: retain a hung server for a bounded backtrace.

The capture mode is run by the VM administrator because Ubuntu's ptrace policy
blocks sibling-process attachment. It verifies workspace and partition ownership
before attaching. No system ptrace setting is changed and no goal is sent.
"""
import argparse,json,os,subprocess,sys,time
from pathlib import Path


def capture(workspace,output):
    request=output/'shutdown_probe_request.json';deadline=time.monotonic()+210
    while not request.exists() and not (output/'run.json').exists() and time.monotonic()<deadline:time.sleep(.1)
    result={'captured':False}
    if request.exists():
        info=json.loads(request.read_text());result['partition']=info['partition'];result['servers']=[]
        for pid in info['server_pids']:
            proc=Path('/proc')/str(int(pid));env=(proc/'environ').read_bytes().split(b'\0')
            if ('IGN_PARTITION='+info['partition']).encode() not in env:raise RuntimeError('wrong_partition')
            if not info['partition'].startswith('gcu_06b2a_') or str(workspace).encode() not in (proc/'cmdline').read_bytes():raise RuntimeError('wrong_workspace')
            if 'libignition-gazebo6-ign.so' not in (proc/'maps').read_text():raise RuntimeError('not_gazebo_server')
            threads=[]
            for task in (proc/'task').iterdir():
                try:threads.append({'tid':int(task.name),'wchan':(task/'wchan').read_text(),'name':(task/'comm').read_text().strip()})
                except OSError:pass
            with (output/('server_'+str(pid)+'_backtrace.log')).open('w') as log:
                try:
                    r=subprocess.run(['gdb','--batch','-q','-p',str(pid),'-ex','set pagination off','-ex','thread apply all bt 16','-ex','detach'],stdout=log,stderr=subprocess.STDOUT,timeout=15)
                    result['servers'].append({'pid':pid,'gdb_exit':r.returncode,'threads':threads})
                except subprocess.TimeoutExpired:result['servers'].append({'pid':pid,'error':'gdb_timeout','threads':threads})
        result['captured']=bool(result['servers'])
    (output/'shutdown_probe_capture.json').write_text(json.dumps(result,indent=2)+'\n')
    (output/'shutdown_probe_done').touch()


def main():
    if len(sys.argv)>1 and sys.argv[1]=='capture':
        p=argparse.ArgumentParser();p.add_argument('mode');p.add_argument('--workspace',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
        a=p.parse_args();capture(a.workspace,a.output);return 0
    import run_gcu_sensor_constraints as runner
    original=runner.stop_simulator
    output=Path(sys.argv[sys.argv.index('--output')+1])
    def stop(partition):
        result=original(partition)
        if result.get('still_alive_pids'):
            (output/'shutdown_probe_request.json').write_text(json.dumps({'partition':partition,'server_pids':result['still_alive_pids']})+'\n')
            deadline=time.monotonic()+20
            while not (output/'shutdown_probe_done').exists() and time.monotonic()<deadline:time.sleep(.1)
            result['diagnostic_capture_completed']=(output/'shutdown_probe_done').exists()
        return result
    runner.stop_simulator=stop
    return runner.main()


if __name__=='__main__':raise SystemExit(main())
