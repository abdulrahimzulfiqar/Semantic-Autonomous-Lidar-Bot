#!/usr/bin/env python3
"""Audit fixed-passage route overhead offline, without changing the planner.

Compare the published model with independent flat-graph costs and swept-circle
checks of direct segments. A physically clear line may still violate a disabled
passage, so diagnostic shortcuts must not be used as an execution fallback.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

from shapely.geometry import LineString
from shapely.ops import unary_union
from check_osmag_routes import reference_cost
from gcu_plan_validation import validate_path
from osmag_paper_planner import PaperPlanner, RouteError, length


def audit(assets, output):
    output.mkdir(parents=True, exist_ok=False)
    planner = PaperPlanner(assets)
    start = [-.05864395131342248, -12.960313240266368]
    cases = [
        ('adjacent_short', [1.4413560486865775, -12.960313240266368], []),
        ('adjacent_diagonal', [5., -8.], []),
        ('adjacent_down', [.5, -16.], []),
        ('same_sector', [-3., -10.5], []),
        ('courtyard', [0., 0.], []),
        ('closed_virtual_interface', [1.4413560486865775, -12.960313240266368], ['-93']),
    ]
    rows = []
    for name, goal, disabled in cases:
        direct = LineString([start, goal])
        disabled_crossings = [ident for ident in disabled
                              if direct.intersects(LineString(planner.passage_segments[ident]))]
        row = {'name': name, 'start': start, 'goal': goal, 'disabled_passages': disabled,
               'straight_length_m': math.dist(start, goal),
               'direct_swept': validate_path(planner.grid, planner.resolution,
                                             planner.origin, [start, goal], planner.radius),
               'direct_crosses_disabled_passages': disabled_crossings}
        try:
            route = planner.plan(start, goal, disabled)
            cost = reference_cost(planner, start, goal, set(disabled))
            assert cost is None or abs(cost-route['graph_cost_m']) < 1e-7
            row.update(route=route, independent_flat_graph_cost_m=cost,
                       ratio_to_straight=route['graph_cost_m']/row['straight_length_m'])
            row['direct_within_selected_area_union'] = unary_union(
                [planner.leaves[area] for area in route['area_ids']]).covers(direct)
            row['crossings'] = []
            for ident in route['passage_ids']:
                segment = planner.passage_segments[ident]
                row['crossings'].append({
                    'id': ident, 'segment': segment,
                    'segment_length_m': math.dist(*segment),
                    'anchors': {node: point for node, point in planner.points.items()
                                if node.endswith('|'+ident)},
                    'direct_intersection': direct.intersection(LineString(segment)).wkt})
            if route['passage_ids'] == ['-93']:
                midpoint = [sum(values)/2 for values in zip(*planner.passage_segments['-93'])]
                center_path = [start, midpoint, goal]
                row['center_only_diagnostic'] = {
                    'path': center_path, 'length_m': length(center_path),
                    'validation': validate_path(planner.grid, planner.resolution,
                                                planner.origin, center_path, planner.radius),
                    'scope': 'Fixed-center geometric restriction without offset anchors; not an author runtime run.'}
        except RouteError as error:
            row['rejected'] = str(error)
        rows.append(row)
    duplicates = []
    for area, residents in planner.residents.items():
        for index, first in enumerate(residents):
            for second in residents[index+1:]:
                a = planner.passages[first.split('|')[-1]]
                b = planner.passages[second.split('|')[-1]]
                if {a['osmAG:from'], a['osmAG:to']} == {b['osmAG:from'], b['osmAG:to']}:
                    duplicates.append({'area': area, 'states': [first, second],
                                       'edge_present': second in planner.graph[first]})
    source = Path(__file__).parent
    result = {'scope': 'Read-only offline route-quality audit; no planner/runtime changes',
              'backend': planner.backend, 'input_sha256': planner.fingerprints(),
              'source_sha256': {name: hashlib.sha256((source/name).read_bytes()).hexdigest()
                                for name in ['audit_gcu_route_quality.py', 'osmag_paper_planner.py',
                                             'osmag_hierarchy.py', 'check_osmag_routes.py',
                                             'gcu_plan_validation.py']},
              'same_adjacent_area_interface_pairs_not_filtered': duplicates, 'cases': rows}
    assert rows[0]['direct_swept']['valid'] and rows[0]['route']['graph_cost_m'] > rows[0]['straight_length_m']
    assert rows[4]['rejected'] == 'unknown_goal' and not rows[4]['direct_swept']['valid']
    assert rows[5]['direct_swept']['valid'] and rows[5]['direct_crosses_disabled_passages'] == ['-93']
    (output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def plot(result, assets, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    from PIL import Image
    import yaml
    cfg = yaml.safe_load((assets/'gcu_ground_floor.yaml').read_text())
    image = np.asarray(Image.open(assets/cfg['image']))
    x, y = cfg['origin'][:2]
    r = cfg['resolution']
    extent = [x, x+r*image.shape[1], y, y+r*image.shape[0]]
    fig, axes = plt.subplots(1, 3, figsize=(13, 6), constrained_layout=True)
    limits = [(-.7, 1.9, -14.8, -12.4), (-1., 6., -15., -7.), (-.7, 1.3, -16.5, -12.3)]
    for axis, row, bounds in zip(axes, result['cases'][:3], limits):
        axis.imshow(image, cmap='gray', vmin=0, vmax=254, extent=extent)
        crossing = row['crossings'][0]
        axis.plot(*np.asarray(crossing['segment']).T, color='#39b854', label='virtual interface -93')
        axis.plot(*np.asarray(row['route'].get('graph_path', row['route']['path'])).T,
                  color='#8057dd', linewidth=2.5, label='fixed-anchor graph reference')
        if 'refined_length_m' in row['route']:
            axis.plot(*np.asarray(row['route']['path']).T, color='#007e89', linewidth=2,
                      label='validated refined preview')
        axis.plot(*np.asarray([row['start'], row['goal']]).T, '--', color='#202020', label='clear direct segment')
        center = row['center_only_diagnostic']['path'][1]
        axis.plot(*center, 'o', color='#db8400', label='interface midpoint')
        axis.plot(*row['start'], 'ko')
        axis.plot(*row['goal'], 'rx')
        axis.set(xlim=bounds[:2], ylim=bounds[2:], xlabel='map x (m)', ylabel='map y (m)',
                 title=f"{row['route']['graph_cost_m']:.2f} m graph route / {row['straight_length_m']:.2f} m direct")
        axis.set_aspect('equal')
    axes[1].legend(loc='lower right', fontsize=8)
    fig.suptitle('GCU offline diagnosis: fixed crossing points create open-space detours\n'
                 'Direct segments passed the 0.18 m swept-footprint check; runtime unchanged')
    fig.savefig(output/'detour_explanation.png', dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', type=Path, default=Path('simulation/gcu_ground_floor'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.assets, args.output)
    plot(result, args.assets, args.output)
    for row in result['cases']:
        print(row['name'], row.get('route', {}).get('graph_cost_m'),
              'direct:', row['straight_length_m'], 'clear:', row['direct_swept']['valid'])
