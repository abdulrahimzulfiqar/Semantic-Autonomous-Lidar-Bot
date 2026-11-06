#!/usr/bin/env python3
"""Run the unchanged functional matrix with frozen Phase 05 runtime inputs.

No retry or parameter tuning. A failed case ends the suite. Re-run the entire
suite in a new output directory after addressing a failure; retain both runs.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def hashes(workspace):
    files=list(workspace.glob('*.py'))+list((workspace/'assets').glob('*.yaml'))+list((workspace/'assets').glob('*.xml'))
    files+=list((workspace/'robot_profile').glob('*'))
    for name in ['contact_v2','navigation_fixtures']:
        files += [p for p in (workspace/'assets'/name).rglob('*') if p.is_file()]
    for name in ['support_src','support_install']:
        files += [p for p in (workspace/name).rglob('*') if p.is_file()]
    files += [workspace/'assets'/n for n in ['gcu_ground_floor.pgm','gcu_ground_floor.yaml','amcl.yaml','navigation_acceptance.json']]
    return {str(p.relative_to(workspace)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(files)) if p.is_file()}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true')
    p.add_argument('--workspace',type=Path,required=True);p.add_argument('--robot-workspace',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--domain',type=int,default=79);a=p.parse_args()
    if not a.execute:p.error('--execute required for simulated autonomous trials')
    a.output.mkdir(parents=True,exist_ok=False);w=a.workspace.resolve()
    contract=json.loads((w/'assets/navigation_acceptance.json').read_text());(a.output/'contract.json').write_text(json.dumps(contract,indent=2)+'\n')
    if not (w/'assets/navigation_fixtures/fixtures.json').exists():raise RuntimeError('Prepare contact assets and navigation fixtures first')
    frozen=hashes(w);report={'passed':False,'frozen_inputs_sha256':frozen,'domain_base':a.domain,'trials':[]}
    (a.output/'frozen_inputs.json').write_text(json.dumps(frozen,indent=2)+'\n')
    common=[sys.executable,str(w/'run_gcu_navigation_test.py'),'--workspace',str(w),'--robot-workspace',str(a.robot_workspace)]
    cases=[('contact_positive',{'positive_control':True},1)]
    cases += [(c['name'],c,i) for c in contract['cases'] for i in range(1,c['repeats']+1)]
    try:
        for index,(name,c,i) in enumerate(cases):
            if hashes(w)!=frozen:raise RuntimeError('Input changed after freeze; refusing further acceptance trials')
            out=a.output/(name+'_'+str(i).zfill(2));limit=c.get('sim_timeout_s',30.)
            cmd=common+['--domain',str(a.domain+index),'--output',str(out),'--sim-timeout',str(limit),'--wall-timeout',str(max(180.,5*limit+120))]
            if c.get('positive_control'):cmd+=['--positive-control']
            else:
                cmd+=['--execute','--goal',*map(str,c['goal']),'--expected',c['expected']]
                if c.get('via_click') or (c.get('first_via_click') and i==1):cmd+=['--via-click']
            if 'fixture' in c:
                fixture=w/'assets/navigation_fixtures'/c['fixture'];cmd+=['--world',str(fixture/'world.sdf'),'--score-map',str(fixture/'map.yaml')]
                if c['fixture']=='blocked':cmd+=['--map',str(fixture/'map.yaml')]
            print('Starting '+out.name,flush=True);result=subprocess.run(cmd)
            trial={'name':out.name,'domain':a.domain+index,'process_exit':result.returncode,'report':str(out/'report.json')}
            report['trials'].append(trial)
            (a.output/'suite.json').write_text(json.dumps(report,indent=2)+'\n')
            if result.returncode:raise RuntimeError('Trial failed: '+out.name)
        report['passed']=True
    except Exception as e:report['error']=str(e)
    finally:(a.output/'suite.json').write_text(json.dumps(report,indent=2)+'\n')
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
