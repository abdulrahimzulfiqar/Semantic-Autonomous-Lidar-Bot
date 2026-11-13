#!/usr/bin/env python3
"""Replay lidar-discovered obstacle clearance under the selected passage mask."""
import argparse,io,json,tarfile
from pathlib import Path
import numpy as np
from gcu_physical_obstacle import overlay_box
from gcu_plan_validation import validate_path
from verify_gcu_segmented_crossing import verify,score_mission


def score_obstacle(report,arrays,planner,archive,prefix):
    result=score_mission(report,arrays,planner);box=report['fixture']['physical_obstacle']
    evaluation=overlay_box(arrays['combined'],planner.resolution,planner.origin,box)
    stored=np.load(io.BytesIO(archive.extractfile(prefix+'obstacle_evaluation.npz').read()))['combined']
    assert np.array_equal(evaluation,stored)
    assert not validate_path(evaluation,planner.resolution,planner.origin,report['fixture']['route_contract']['path'],.20)['valid']
    assert validate_path(evaluation,planner.resolution,planner.origin,[v['truth'][:2] for v in report['truth']],.18)['valid']
    oracle={k:validate_path(evaluation,planner.resolution,planner.origin,v['points'],.20) for k,v in report['preflight'].items()}
    # The navigation source raster remains the original; only the evaluator
    # overlays the actual box. Its SDF is part of the frozen assets.
    import xml.etree.ElementTree as ET
    world=ET.fromstring(archive.extractfile('assets/contact_v2/world.sdf').read())
    model=world.find("world/model[@name='test_obstacle_detour']");assert model is not None
    assert np.allclose([float(x) for x in model.findtext('pose').split()[:2]],box['center'])
    assert np.allclose([float(x) for x in model.findtext('link/collision/geometry/box/size').split()],box['size'])
    return dict(result,evaluation_obstacle_clearance=True,navigation_raster_unmodified=True,unexecuted_preflight_oracle=oracle)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);p.add_argument('--trial',default='01');a=p.parse_args()
    with tarfile.open(a.evidence/('attempt'+a.trial+'.tar.gz')) as t:
        result=verify(a.evidence,a.trial,'obstacle',mission_scorer=lambda r,arrays,planner:score_obstacle(r,arrays,planner,t,'results/obstacle_'+a.trial+'/'))
    (a.evidence/'historical_verification_replay.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
