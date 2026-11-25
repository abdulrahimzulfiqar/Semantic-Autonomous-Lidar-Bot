#!/usr/bin/env python3
"""Run structural estimator using observations only; no evaluator/truth access."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import numpy as np
from tracker import StructuralMap,Tracker


def resources():
    result={'cpu_s':time.process_time()}
    for path,key,prefix in [('/proc/self/status','rss_kib','VmRSS:'),('/proc/self/smaps_rollup','pss_kib','Pss:')]:
        try:result[key]=next(int(v.split()[1]) for v in Path(path).read_text().splitlines() if v.startswith(prefix))
        except (OSError,StopIteration):result[key]=None
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--observations',type=Path,required=True)
    p.add_argument('--osm',type=Path,required=True);p.add_argument('--config',type=Path,default=Path(__file__).with_name('config.json'))
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():p.error('output already exists')
    data=np.load(a.observations);cfg=json.loads(a.config.read_text());begin=time.perf_counter()
    before=resources();model=StructuralMap.from_osm(a.osm);loaded=resources();load_s=time.perf_counter()-begin
    tracker=Tracker(model,data['initial_pose'],cfg);results=[];samples=[]
    tracking_start=resources()
    for i,stamp in enumerate(data['stamps']):
        results.append(tracker.update(float(stamp),data['angles'],data['ranges'][i],data['odom'][i],data['extrinsics'][i],*data['range_limits']))
        if i%25==0:samples.append(resources())
    after=resources()
    report=dict(backend='paper_inspired_structural_2d_reference_not_author_AGLoc',poses=results,
        config=cfg,resources=dict(before=before,after_map_load=loaded,tracking_start=tracking_start,after=after,samples=samples),
        map_preprocess_wall_s=load_s,segments=len(model.segments),vector_array_bytes=model.segments.nbytes+model.doors.nbytes,
        host=dict(uname=list(os.uname())),input_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in [a.observations,a.osm,a.config,Path(__file__),Path(__file__).with_name('tracker.py')]},
        scope='Tracking-only 2D adaptation; initial pose supplied; no 3D filter/global rescue/TF publication/closed-loop proof')
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(frames=len(results),accepted=sum(r['accepted'] for r in results),lost_frames=sum(r['lost'] for r in results))))


if __name__=='__main__':main()
