#!/usr/bin/env python3
"""Replay a stopped room-exit closure, revised native mask and no-route result."""
import argparse,io,json,tarfile
from pathlib import Path
import numpy as np
from gcu_cancel_metrics import score_cancellation
from gcu_costmap_observation import inspect
from gcu_plan_validation import validate_path
from osmag_constraint_mask import build_mask
from osmag_paper_planner import RouteError
from verify_gcu_cancel import score_cancel_run
from verify_gcu_segmented_crossing import verify


def score_closure(report,arrays,planner,archive,prefix):
    metrics=score_cancel_run(report,arrays,planner,lambda r:score_cancellation(r,reason='passage_closed',final_state='failed',band=(.60,.75),request_key='closure_request'))
    request=report['closure_request'];resolution=report['closure_resolution'];update=request['update']
    assert update=={'revision':1,'disabled_passages':[report['contract']['passage_ids'][0]]}
    assert resolution['status']=='no_route' and resolution['closure_revision']==1 and resolution['disabled_passages']==update['disabled_passages']
    assert report['closure_status_messages'][-1]['payload']==resolution
    received=[e for e in report['events'] if e['event']=='closure_received']
    resolved=[e for e in report['events'] if e['event']=='closure_resolution']
    assert len(received)==len(resolved)==1 and received[0]['wall']>=request['wall']
    terminal=next(e for e in report['events'] if e['event']=='result' and e['outcome']=='cancelled')
    assert resolved[0]['wall']>=terminal['wall'] and resolved[0]['sim']>=metrics['truth_rest']['start_sim']
    try:planner.plan(resolution['start'],resolution['goal'][:2],update['disabled_passages'])
    except RouteError as error:assert str(error)==resolution['reason']=='no_feasible_graph_route'
    else:raise AssertionError('Closed room still has a graph route')
    incidence={n:(v['osmAG:from'],v['osmAG:to']) for n,v in planner.passages.items()}
    stored=report['contract'];snap=build_mask(planner.grid,planner.resolution,planner.origin,planner.leaves,planner.passage_segments,incidence,
        allowed_areas=stored['area_ids'],open_passages=[],disabled_passages=update['disabled_passages'],robot_pose=report['closure_mask_pose'][:2],mission_id=stored['mission_id'],closure_revision=1,mask_revision=2)
    closed=np.load(io.BytesIO(archive.extractfile(prefix+'closure_constraint.npz').read()))
    assert all(np.array_equal(closed[k],v) for k,v in [('physical',planner.grid),('mask',snap.mask),('combined',snap.combined)])
    assert snap.report()['mask_sha256']==report['closure_snapshot']['mask_sha256']
    assert validate_path(snap.combined,planner.resolution,planner.origin,[v['truth'][:2] for v in report['truth']],.18)['valid']
    capture=next(v for v in report['native_captures'] if v['label']=='closed')
    checked={}
    for key in ['global','local']:
        grid=capture['grids'][key];assert grid['frame']=='map' and grid['stamp']>=report['closure_mask_published_sim']
        data=np.load(io.BytesIO(archive.extractfile(prefix+grid['file']).read()))['data']
        probes=[stored['start'],stored['goal'][:2]] if key=='global' else [capture['pose'][:2]]
        checked[key]=inspect(snap,data,grid['resolution'],grid['origin'],[0.,0.,0.],probes,full=key=='global')
    assert report['closed_native_plan']=={'status':6,'points':[]}
    metrics['closure_stop_delay_sim_s']=metrics.pop('cancel_stop_delay_sim_s')
    return dict(metrics,closure_revision=1,mask_revision=2,graph_no_route=True,native_no_route=True,native_mask_replay=checked)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);p.add_argument('--trial',default='01');a=p.parse_args()
    with tarfile.open(a.evidence/('attempt'+a.trial+'.tar.gz')) as t:
        result=verify(a.evidence,a.trial,'closure',mission_scorer=lambda r,arrays,planner:score_closure(r,arrays,planner,t,'results/closure_'+a.trial+'/'))
    name='historical_verification_replay.json' if result['historical_source_files'] else 'verification.json'
    (a.evidence/name).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
