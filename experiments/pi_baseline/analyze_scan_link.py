#!/usr/bin/env python3
"""Check scan content/coverage across the reliable hop; no wire latency claim."""
import argparse
import hashlib
import json
from pathlib import Path


def compare(sender,receiver,first_ns,last_ns):
    source={r['stamp_ns']:r['serialized_sha256'] for r in sender['rows'] if first_ns<=r['stamp_ns']<=last_ns}
    received={r['stamp_ns']:r['serialized_sha256'] for r in receiver['rows'] if first_ns<=r['stamp_ns']<=last_ns}
    missing=sorted(set(source)-set(received))
    changed=sorted(k for k in source.keys() & received.keys() if source[k]!=received[k])
    return dict(scope='Original serialized scan SHA-256 and stamp coverage in declared interval; not mission success or synchronized cross-host latency',
        first_stamp_ns=first_ns,last_stamp_ns=last_ns,source_unique_scans=len(source),received_unique_scans=len(received),
        missing_stamps_ns=missing,changed_stamps_ns=changed,passed=bool(source and not missing and not changed))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);a=p.parse_args()
    root=a.evidence;paths=dict(sender=root/'simulator_results/results/scan_sender.json',
        receiver=root/'pi_results/scan_receiver.json',worker=root/'pi_results/actor/worker.json',
        source_observer=root/'simulator_results/results/observer.json')
    data={k:json.loads(v.read_text()) for k,v in paths.items()};w=data['worker']
    result=compare(data['sender'],data['receiver'],round(w['dispatch_sim']*1e9),round(w['final_sim']*1e9))
    expected={round(r['stamp']*1e9) for r in data['source_observer']['scan_receipts']
        if w['dispatch_sim']<=r['stamp']<=w['final_sim']}
    sent={r['stamp_ns'] for r in data['sender']['rows']}
    result['source_observer_unique_scans']=len(expected)
    result['interval_kind']='navigation' if any(e['event']=='send' for e in w['events']) else 'pre_dispatch_admission'
    result['navigation_send_events']=sum(e['event']=='send' for e in w['events'])
    result['observed_source_stamps_missing_from_sender']=sorted(expected-sent)
    result['passed']=bool(result['passed'] and expected and not (expected-sent))
    result['inputs_sha256']={k:hashlib.sha256(v.read_bytes()).hexdigest() for k,v in paths.items()}
    result['analyzer_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (root/'scan_link_analysis.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)
