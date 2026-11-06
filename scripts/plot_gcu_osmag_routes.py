#!/usr/bin/env python3
"""Render recorded route examples, including the fixed-anchor path overhead."""
import argparse
import gzip
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    results = json.loads((args.evidence/'preview_acceptance_final/results.json').read_text())
    assert results['passed']
    cases = {row['name']: row for row in results['cases']}
    cfg = yaml.safe_load((args.assets/'gcu_ground_floor.yaml').read_text())
    raster = np.asarray(Image.open(args.assets/cfg['image']))
    x, y = cfg['origin'][:2]
    r = cfg['resolution']
    extent = [x, x+r*raster.shape[1], y, y+r*raster.shape[0]]
    phase4 = args.assets/'phase4_diagnostics/autonomous_acceptance'
    prior = json.loads((phase4/'results.json').read_text())
    corridor = next(row for row in prior['acceptance'] if row['name']=='corridor_01')
    raw = json.loads(gzip.decompress((phase4/corridor['full_report']).read_bytes()))
    metric = np.asarray(raw['preflight']['path'])
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), constrained_layout=True)
    axes[0].imshow(raster, cmap='gray', vmin=0, vmax=254, extent=extent)
    for name, color, label in [('same_sector', '#2787db', 'same sector'),
                              ('adjacent_sector', '#a33fe0', 'adjacent sector'),
                              ('room_route', '#008d54', 'room route')]:
        route = np.asarray(cases[name]['route']['path'])
        axes[0].plot(*route.T, color=color, label=label)
        axes[0].plot(*route[-1], 'x', color=color)
    axes[0].plot(0., 0., 'rx', label='courtyard: rejected')
    axes[0].set_title('Paper-based osmAG previews (no motion)')
    axes[0].legend(fontsize=9)
    anchor = np.asarray(cases['adjacent_sector']['route']['path'])
    anchor_length = cases['adjacent_sector']['route']['graph_cost_m']
    metric_length = sum(math.dist(a, b) for a, b in zip(metric, metric[1:]))
    axes[1].plot(*anchor.T, color='#a33fe0', label=f'osmAG anchors: {anchor_length:.2f} m')
    axes[1].plot(*metric.T, '--', color='#d88700', label=f'Phase 04 Nav2 plan: {metric_length:.2f} m')
    axes[1].set_title('Short-route overhead from fixed anchors')
    axes[1].legend(fontsize=9)
    for axis in axes:
        axis.set(xlabel='Map x (m)', ylabel='Map y (m)')
        axis.set_aspect('equal', adjustable='datalim')
    fig.suptitle('GCU Phase 05: feasibility examples; no author-runtime or speedup claim')
    fig.savefig(args.evidence/'route_previews.png', dpi=150)
    comparison = {'paper_based_route_length_m': anchor_length,
                  'recorded_phase4_nav2_plan_length_m': metric_length,
                  'ratio': anchor_length/metric_length,
                  'scope': 'Different planning representations and fixed portal anchors; not a same-run performance comparison.'}
    (args.evidence/'short_route_comparison.json').write_text(json.dumps(comparison, indent=2)+'\n')


if __name__ == '__main__':
    main()
