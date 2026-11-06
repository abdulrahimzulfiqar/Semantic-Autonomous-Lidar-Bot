#!/usr/bin/env python3
"""Offline acceptance of the labelled paper-based GCU route preview, without ROS.

Uses SciPy's independent graph shortest paths for the cross-sector cost check.
Reports query/preprocessing separately and retains actual rejected outcomes.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import sys
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from osmag_paper_planner import PaperPlanner,RouteError,length
from shapely.geometry import LineString


def reference_cost(planner,start,goal,disabled):
    first=planner.containing_area(start,'start');last=planner.containing_area(goal,'goal')
    if first==last:return None
    names=[n for n in planner.points if n.split('|')[-1] not in disabled]+['@start','@goal']
    index={n:i for i,n in enumerate(names)};matrix=np.zeros((len(names),len(names)))
    for a in names[:-2]:
        for b,cost in planner.graph[a].items():
            if b in index:matrix[index[a],index[b]]=cost
    for virtual,area,point in [('@start',first,start),('@goal',last,goal)]:
        for n in planner.residents[area]:
            if n not in index:continue
            path=planner.grid_path(area,point,planner.points[n])
            if path is not None:matrix[index[virtual],index[n]]=matrix[index[n],index[virtual]]=length(path)
    return float(dijkstra(csr_matrix(matrix),directed=False,indices=index['@start'])[index['@goal']])


def check(assets,output):
    output.mkdir(parents=True,exist_ok=False);planner=PaperPlanner(assets)
    coordinate=planner.coordinate_report();assert coordinate['passed']
    start=[-.05864395131342248,-12.960313240266368]
    cases=[('same_sector',[-3.,-10.5],[], 'ready'),
           ('adjacent_sector',[1.4413560486865775,-12.960313240266368],[], 'ready'),
           ('adjacent_diagonal',[5.,-8.],[], 'ready'),
           ('adjacent_down',[.5,-16.],[], 'ready'),
           ('room_route',[-10.,-7.5],[], 'ready'),
           ('courtyard',[0.,0.],[], 'unknown_goal'),
           ('occupied',[-1.4074548315221413,-16.244374513818027],[], 'occupied_goal'),
           ('outside',[200.,200.],[], 'outside_goal'),
           ('all_passages_closed',[-10.,-7.5],list(planner.passages), 'no_feasible_graph_route'),
           ('selected_passage_closed',[1.4413560486865775,-12.960313240266368],['-93'],'reference_connectivity')]
    result={'passed':True,'backend':planner.backend,'environment':platform.platform(),
            'preprocess_s':planner.preprocess_s,'coordinate_audit':coordinate,
            'input_sha256':planner.fingerprints(),
            'source_sha256':{name:hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
                             for name in ['check_osmag_routes.py','osmag_paper_planner.py','osmag_hierarchy.py',
                                          'osmag_route_refinement.py','audit_gcu_interfaces.py','gcu_plan_validation.py']},'cases':[]}
    (output/'contract.json').write_text(json.dumps({'start':start,'cases':cases,'radius_m':planner.radius},indent=2)+'\n')
    for name,goal,disabled,expected in cases:
        row={'name':name,'start':start,'goal':goal,'disabled_passages':disabled,'expected':expected}
        reference=None
        if expected=='reference_connectivity':
            reference=reference_cost(planner,start,goal,set(disabled))
            expected='ready' if np.isfinite(reference) else 'no_feasible_graph_route'
            row['expected_from_independent_graph']=expected
        try:
            route=planner.plan(start,goal,disabled);row.update(state='ready',route=route)
            if reference is None:reference=reference_cost(planner,start,goal,set(disabled))
            if reference is not None:
                row['independent_reference_cost_m']=reference
                assert abs(route['graph_cost_m']-reference)<1e-7
            assert expected=='ready';assert not set(route['passage_ids'])&set(disabled)
            assert abs(length(route['graph_path'])-route['graph_cost_m'])<1e-7
            assert abs(length(route['path'])-route['refined_length_m'])<1e-7
            assert route['refined_length_m']<=route['graph_cost_m']+1e-7
            assert route['refinement']['topology']['passage_ids']==route['passage_ids']
            if name in ['same_sector','adjacent_sector','adjacent_diagonal','adjacent_down']:
                assert abs(route['refined_length_m']-length([start,goal]))<1e-7
            for ident in disabled:
                barrier=LineString(planner.passage_segments[ident])
                assert all(not LineString([a,b]).intersects(barrier) for a,b in zip(route['path'],route['path'][1:]))
            row['passed']=True
        except RouteError as error:row.update(state='rejected',reason=str(error),passed=str(error)==expected)
        except Exception as error:row.update(state='failed',error=repr(error),passed=False)
        result['cases'].append(row);result['passed']=result['passed'] and row['passed']
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    (output/'passage_audit.json').write_text(json.dumps({'input_sha256':planner.fingerprints(),
        'interfaces':planner.interface_report['interfaces'],'local_edge_failures':planner.local_edge_failures,
        'redundant_interface_pairs_excluded':planner.redundant_interface_pairs,
        'base_states':len(planner.points),'base_edges':sum(len(e) for e in planner.graph.values())//2,
        'maximum_hierarchy_depth':max(len(planner.hierarchy.ancestors(a)) for a in planner.leaves),
        'scope':'sampled straight crossings plus conservative leaf-raster connections; no motion certificate'},indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--assets',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();result=check(args.assets,args.output)
    print(json.dumps({'passed':result['passed'],'cases':[(c['name'],c['passed']) for c in result['cases']]},indent=2))
    sys.exit(0 if result['passed'] else 1)
