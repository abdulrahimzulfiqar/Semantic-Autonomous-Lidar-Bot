#!/usr/bin/env python3
"""Package measured Phase 04 evidence without discarding failed attempts.

Standard library only. Full reports/logs are losslessly compressed. Can package
an incomplete/failed suite; the summary preserves that status explicitly.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil


def summarize(source,suite,output):
    output.mkdir(parents=True,exist_ok=False);raw=output/'trials';raw.mkdir()
    result={'acceptance_passed':False,'acceptance':[],'development':[],'interrupted_suites':{},'interrupted_trials':[]}
    if (suite/'suite.json').exists():
        s=json.loads((suite/'suite.json').read_text());result['acceptance_passed']=s.get('passed',False)
        for name in ['suite.json','contract.json','frozen_inputs.json']:
            if (suite/name).exists():shutil.copy2(suite/name,output/name)
    folders=[('interrupted_trials' if p.name.startswith(('user_interrupted_','externally_interrupted_')) else 'acceptance',p) for p in sorted(suite.iterdir()) if p.is_dir()] if suite.exists() else []
    folders += [('development',p) for p in sorted(source.iterdir()) if p.is_dir() and p!=suite and not p.name.startswith('acceptance')]
    for previous in sorted(source.glob('acceptance_*')):
        if previous==suite or not previous.is_dir() or not (previous/'suite.json').exists():continue
        key='interrupted_'+previous.name;result['interrupted_suites'][key]={'status':json.loads((previous/'suite.json').read_text()),'trials':[]}
        saved=output/'interrupted_suites'/previous.name;saved.mkdir(parents=True)
        for name in ['suite.json','contract.json','frozen_inputs.json']:
            if (previous/name).exists():shutil.copy2(previous/name,saved/name)
        folders += [(key,p) for p in sorted(previous.iterdir()) if p.is_dir()]
    for kind,folder in folders:
        if not (folder/'run.json').exists():continue
        run=json.loads((folder/'run.json').read_text());row={'name':folder.name,'runner':run}
        rp=folder/'report.json'
        if rp.exists():
            report=json.loads(rp.read_text());row.update({k:v for k,v in report.items() if k not in ['truth','localization','commands','paths','contacts','display_paths','bridge_statuses','preflight']})
            if 'preflight' in report:row['preflight']={k:v for k,v in report['preflight'].items() if k!='path'}
            packed=raw/(kind+'_'+folder.name+'.json.gz')
            with gzip.GzipFile(filename=str(packed),mode='wb',mtime=0) as f:f.write(rp.read_bytes())
            row.update(full_report=str(packed.relative_to(output)),full_report_sha256=hashlib.sha256(packed.read_bytes()).hexdigest())
        else:row.update(passed=False,error=run.get('error','No evaluator report; startup/runner failure'))
        row['additional_reports']={}
        for extra in sorted(folder.glob('*_report.json')):
            payload=json.loads(extra.read_text());packed=raw/(kind+'_'+folder.name+'_'+extra.name+'.gz')
            with gzip.GzipFile(filename=str(packed),mode='wb',mtime=0) as f:f.write(extra.read_bytes())
            meta={k:v for k,v in payload.items() if k not in ['truth','localization','commands','paths','contacts','display_paths','bridge_statuses','preflight']}
            meta.update(full_report=str(packed.relative_to(output)),full_report_sha256=hashlib.sha256(packed.read_bytes()).hexdigest())
            row['additional_reports'][extra.stem]=meta
        row['passed']=bool(row.get('passed') and run.get('passed'))
        # Keep logs from all failed trials and a representative successful one.
        if not row.get('passed') or folder.name in ['corridor_01','room_entry_01','obstacle_detour_01']:
            for name in ['simulator.log','amcl.log','navigation.log','check.log','localization_readiness.log','shutdown.log']:
                p=folder/name
                if p.exists():
                    with gzip.GzipFile(filename=str(raw/(kind+'_'+folder.name+'_'+name+'.gz')),mode='wb',mtime=0) as f:f.write(p.read_bytes())
        if kind in result and isinstance(result[kind],list):result[kind].append(row)
        else:result['interrupted_suites'][kind]['trials'].append(row)
    nominal=[r for r in result['acceptance'] if r['name'].startswith(('corridor_','turn_','room_entry_'))]
    result['nominal_trials']=len(nominal);result['nominal_passes']=sum(r.get('passed',False) for r in nominal)
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',required=True,type=Path)
    p.add_argument('--suite',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--compact-history',action='store_true',help='Bundle older reports/logs after verifying their bytes')
    a=p.parse_args();summarize(a.source,a.suite,a.output)
    if a.compact_history:
        from compact_gcu_evidence import compact
        compact(a.output)
