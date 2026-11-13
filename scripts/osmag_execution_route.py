"""Pure execution-mask route reconstruction within an already selected area sequence.

Our paper-based integration policy, not author runtime. No alternate passage
sequence or Euclidean fallback is introduced when the padded corridor is blocked.
"""
import copy
import cv2
import numpy as np
from osmag_constraint_mask import build_mask
from osmag_route_refinement import RouteCorridor,refine_route,path_length
from gcu_plan_validation import validate_path


def constrain_route(planner,route,padding):
    areas=route['area_ids'];passages=route['passage_ids'];effective=planner.radius+padding
    incidence={n:(v['osmAG:from'],v['osmAG:to']) for n,v in planner.passages.items()}
    snapshot=build_mask(planner.grid,planner.resolution,planner.origin,planner.leaves,planner.passage_segments,incidence,
        allowed_areas=areas,open_passages=passages,disabled_passages=route['disabled_passages'],robot_pose=route['start'],
        mission_id='execution_route',radius=planner.radius,padding=padding)
    # Reuse the tested leaf raster/connectors against the actual constraint
    # geometry, without modifying the original planner's graph or caches.
    local=copy.copy(planner);local.grid=snapshot.combined;local.radius=effective;local.rasters={}
    free=np.pad((local.grid==0).astype(np.uint8),1)
    distance=cv2.distanceTransform(free,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)[1:-1,1:-1]*local.resolution
    local.safe=distance>effective+local.resolution/np.sqrt(2)+local.resolution/8
    reference=[list(route['start'])]
    for i,ident in enumerate(passages):
        crossing=local.center_crossing(ident,areas[i],areas[i+1])
        if crossing is None:raise ValueError('constraint_crossing_unavailable:'+ident)
        before,after=crossing;piece=local.grid_path(areas[i],reference[-1],before)
        if piece is None:raise ValueError('constraint_leaf_connection_unavailable:'+areas[i])
        reference.extend(piece[1:]);reference.append(list(after))
    piece=local.grid_path(areas[-1],reference[-1],route['goal'])
    if piece is None:raise ValueError('constraint_goal_connection_unavailable')
    reference.extend(piece[1:])
    physical=lambda points:validate_path(snapshot.combined,local.resolution,local.origin,points,effective)
    if not physical(reference)['valid']:raise ValueError('constraint_reference_padding_failed')
    corridor=RouteCorridor(local.leaves,local.passage_segments,areas,passages,route['disabled_passages'])
    path,refinement=refine_route(reference,corridor,physical)
    check=physical(path)
    if not check['valid'] or not corridor.validate(path)['valid']:raise ValueError('constraint_refinement_failed')
    result=dict(route,path=path,refinement=refinement,refined_length_m=path_length(path),validation=check)
    result['execution_refinement']={'method':'selected_sequence_padded_mask_leaf_astar_then_visibility',
        'padding_m':padding,'effective_radius_m':effective,'reference_path':reference,'validation':check,
        'scope':'Same selected passage sequence; no continuous global optimality or author-runtime equivalence claim'}
    return result
