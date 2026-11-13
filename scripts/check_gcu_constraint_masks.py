#!/usr/bin/env python3
"""06C1A: bounded GCU mask/activation checks, without ROS or robot motion."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import time

import numpy as np
from gcu_plan_validation import validate_path,cell_at
from osmag_paper_planner import PaperPlanner
from osmag_constraint_mask import build_mask,ConsumptionGate,ConstraintError


def check(assets,output):
    output.mkdir(parents=True,exist_ok=False);begin=time.perf_counter();planner=PaperPlanner(assets)
    spawn=[-.05864395131342248,-12.960313240266368];goal=[1.4413560486865775,spawn[1]]
    incidence={name:(tags['osmAG:from'],tags['osmAG:to']) for name,tags in planner.passages.items()}
    source=Path(__file__).parent
    result={'passed':True,'scope':'06C1A offline raw masks and synthetic costmap observations only',
            'ros_started':False,'physical_motion_tested':False,'input_sha256':planner.fingerprints(),
            'source_sha256':{n:hashlib.sha256((source/n).read_bytes()).hexdigest() for n in [
                'check_gcu_constraint_masks.py','osmag_constraint_mask.py','gcu_plan_validation.py',
                'osmag_paper_planner.py','osmag_hierarchy.py','osmag_route_refinement.py']},'cases':[]}
    def make(name,start,goal,disabled=(),mask_revision=1,closure_revision=0):
        route=planner.plan(start,goal,disabled)
        snap=build_mask(planner.grid,planner.resolution,planner.origin,planner.leaves,planner.passage_segments,
                        incidence,allowed_areas=route['area_ids'],open_passages=route['passage_ids'],
                        disabled_passages=disabled,robot_pose=start,mission_id=name,
                        mask_revision=mask_revision,closure_revision=closure_revision)
        return route,snap
    def record(name,fn):
        try:
            data=fn();result['cases'].append({'name':name,'passed':True,**(data or {})})
        except Exception as error:
            result['cases'].append({'name':name,'passed':False,'error':repr(error)});result['passed']=False
            raise
    try:
        route,opened=make('short_open',spawn,goal)
        record('open_refined_short_route',lambda:{'snapshot':opened.report(),
            'path_validation':validate_path(opened.combined,opened.resolution,opened.origin,route['path'],opened.radius+opened.padding)})
        assert result['cases'][-1]['path_validation']['valid']
        np.savez_compressed(output/'short_open_mask.npz',mask=opened.mask,combined=opened.combined)
        def overlap():
            try:make('closed_spawn',spawn,goal,['-93'])
            except ConstraintError as e:
                assert str(e)=='robot_footprint_intersects_constraint'
                return {'reason':str(e),'expected_rejection':True}
            raise AssertionError('closed mask improperly carved around footprint')
        record('closed_interface_straddling_spawn_rejected',overlap)
        # A declared development fixture starts clear of the seam. This is not a
        # command or permission to teleport a robot during an active mission.
        clear=[-.5,spawn[1]];route2,closed=make('closed_clear_start',clear,goal,['-93'],2,1)
        def closure():
            assert cell_at(closed.mask,closed.resolution,closed.origin,-.02932198,spawn[1])==100
            assert not validate_path(closed.combined,closed.resolution,closed.origin,[clear,goal],.20)['valid']
            return {'snapshot':closed.report(),'alternate_passages':route2['passage_ids'],
                    'scope':'Initial pose and closure barrier only; full alternate execution not certified.'}
        record('clear_start_closure_blocks_direct_shortcut',closure)
        _,reopened=make('closed_clear_start',clear,goal,[],3,2)
        def reopening():
            assert cell_at(reopened.mask,reopened.resolution,reopened.origin,-.02932198,spawn[1])==0
            assert cell_at(closed.mask,closed.resolution,closed.origin,-.02932198,spawn[1])==100
            assert reopened.mask_sha256!=closed.mask_sha256
            return {'new_hash':reopened.mask_sha256,'old_hash':closed.mask_sha256}
        record('reopen_replaces_snapshot_without_mutating_old',reopening)
        def preserve():
            assert np.all(opened.combined[planner.grid==100]==100)
            assert np.all(opened.combined[planner.grid<0]<0)
            return {'physical_nonfree_cells_preserved':int(np.sum(planner.grid!=0))}
        record('walls_and_unknown_courtyard_preserved',preserve)
        gate=ConsumptionGate(opened,10.,[spawn,goal])
        def bad_ack():
            try:gate.observe('global',planner.grid,opened.resolution,opened.origin,10.1,10.2)
            except ConstraintError as e:
                assert str(e)=='mask_constraints_not_consumed'
                return {'reason':str(e)}
            raise AssertionError('unmasked occupancy counted as consumption')
        record('unmasked_costmap_is_not_acknowledgement',bad_ack)
        def both():
            gate.observe('global',opened.combined,opened.resolution,opened.origin,10.1,10.2)
            assert not gate.ready(10.2)
            gate.observe('local',opened.combined,opened.resolution,opened.origin,10.1,10.2)
            assert gate.ready(10.2) and not gate.ready(11.)
            return {'observations':gate.observations,'scope':'Synthetic arrays, not native ROS costmaps.'}
        record('both_observations_required_and_expire',both)
    except Exception as e:
        result['passed']=False;result['error']=repr(e)
    result['elapsed_wall_s']=time.perf_counter()-begin
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--assets',type=Path,default=Path('simulation/gcu_ground_floor'))
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();r=check(a.assets,a.output)
    print(json.dumps({k:v for k,v in r.items() if k not in ['cases','source_sha256','input_sha256']},indent=2))
    for row in r['cases']:print(row['name'],row['passed'],row.get('error',''))
    raise SystemExit(0 if r['passed'] else 1)
