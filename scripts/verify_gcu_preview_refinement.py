#!/usr/bin/env python3
"""Replay the bounded v2 preview evidence; no ROS or simulation is started."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import tarfile

from shapely.geometry import LineString
from gcu_plan_validation import validate_path
from osmag_paper_planner import PaperPlanner, length
from osmag_route_refinement import RouteCorridor
from check_osmag_routes import reference_cost


def digest(payload):
    return hashlib.sha256(payload).hexdigest()


def verify(evidence, assets):
    root = Path(__file__).resolve().parents[1]
    assets = assets.resolve()
    manifest = json.loads((evidence/'manifest.json').read_text())
    archive_path = evidence/manifest['archive']
    assert digest(archive_path.read_bytes()) == manifest['sha256']
    with tarfile.open(archive_path) as archive:
        assert set(archive.getnames()) == set(manifest['members'])
        for name, expected in manifest['members'].items():
            assert digest(archive.extractfile(name).read()) == expected, name
        def read(name):
            return archive.extractfile(name).read()
        report = json.loads(read('results/acceptance_01/report.json'))
        run = json.loads(read('results/acceptance_01/run.json'))
        assert report == json.loads((evidence/'live_report.json').read_text())
        assert run == json.loads((evidence/'live_run.json').read_text())
        matched = []
        for name, expected in run['preview_source_sha256'].items():
            assert digest(read('runtime/'+name)) == expected, name
            if name.startswith('assets/'):
                path = assets/name.removeprefix('assets/')
            elif name.startswith('probe_src/'):
                path = root/'simulation/gcu_ground_floor/support/gcu_tf_probe'/name.removeprefix('probe_src/')
            elif '/' not in name:
                path = root/'scripts'/name
            else:
                continue  # Compiled probe installation is verified from the archive.
            assert path.is_file() and digest(path.read_bytes()) == expected, name
            matched.append(str(path.relative_to(root)))
    assert run['passed'] and run['checker_exit'] == 0 and run['inputs_unchanged']
    assert run['launch_exits'] == [0, 0, 0] and not run['child_audit']['critical']
    assert run['truth_bridge_stop']['passed'] and run['simulator_stop']['passed']
    assert not run['simulator_stop']['still_alive_pids']
    assert report['passed'] and len(report['cases']) == 15
    assert report['velocity_publishers'] == 0 and not report['command_messages']
    assert not report['motion_commands_sent_by_evaluator']
    assert all(v == 0 for v in report['navigation_action_publishers'].values())
    assert report['maximum_truth_drift_m'] < .01 and report['truth_samples'] >= 50
    assert report['duplicate_tf_fixture_detected']
    assert report['tf_edge_publishers'] == {'AGmap->map': 1, 'map->odom': 1, 'odom->base_footprint': 1}
    cleanup = json.loads((evidence/'process_cleanup.json').read_text())
    assert cleanup['passed'] and not cleanup['matching_processes']
    offline = json.loads((evidence/'results.json').read_text())
    assert offline['passed'] and len(offline['cases']) == 10
    planner = PaperPlanner(assets)
    assert planner.fingerprints() == offline['input_sha256']
    for name, expected in offline['source_sha256'].items():
        assert digest((root/'scripts'/name).read_bytes()) == expected, name
    local = json.loads((evidence/'local_validation.json').read_text())
    log = (evidence/local['log']).read_bytes()
    assert digest(log) == local['log_sha256'] and b'Ran 47 tests' in gzip.decompress(log)
    assert local['unit_tests_passed'] == 47
    for name, expected in local['source_sha256'].items():
        assert digest((root/name).read_bytes()) == expected, name
    verified = []
    for origin, rows in [('offline', offline['cases']), ('live', report['cases'])]:
        for row in rows:
            assert row['passed'], row['name']
            if origin == 'offline':
                if row['state'] != 'ready':
                    continue
                status = row['route']; path = status['path']; graph_path = status['graph_path']
            else:
                status = row['status']; path = row['path']['points']; graph_path = row['graph_path']['points']
                if status['state'] != 'ready':
                    assert not path and not graph_path
                    continue
                assert row['path']['frame'] == row['graph_path']['frame'] == 'AGmap'
            assert status['planner_version'] == 'phase5_preview_v2'
            assert abs(length(path)-status['refined_length_m']) < 1e-7
            assert abs(length(graph_path)-status['graph_cost_m']) < 1e-7
            assert length(path) <= length(graph_path)+1e-7
            assert math.dist(path[0], status['start']) < 1e-7
            assert math.dist(path[-1], status['goal']) < 1e-7
            direct_cases = (['same_sector', 'adjacent_sector', 'adjacent_diagonal', 'adjacent_down']
                            if origin == 'offline' else ['short_goal', 'diagonal_goal', 'downward_goal'])
            if row['name'] in direct_cases:
                assert abs(length(path)-math.dist(status['start'], status['goal'])) < 1e-7
            for geometry in [path, graph_path]:
                assert validate_path(planner.grid, planner.resolution, planner.origin, geometry, planner.radius)['valid']
            corridor = RouteCorridor(planner.leaves, planner.passage_segments, status['area_ids'],
                                     status['passage_ids'], status['disabled_passages'])
            topology = corridor.validate(path)
            assert topology['valid'] and topology['passage_ids'] == status['passage_ids']
            for ident in status['disabled_passages']:
                closed = LineString(planner.passage_segments[ident])
                assert all(not LineString([a, b]).intersects(closed) for a, b in zip(path, path[1:]))
            reference = reference_cost(planner, status['start'], status['goal'], set(status['disabled_passages']))
            assert reference is None or abs(reference-status['graph_cost_m']) < 1e-7
            verified.append(origin+'/'+row['name'])
    result = {'passed': True, 'unit_tests': 47, 'offline_cases': 10, 'live_cases': 15,
              'ready_routes_recomputed': verified, 'runtime_files_verified': len(manifest['members']),
              'runtime_repository_sources_matched': matched, 'maximum_truth_drift_m': report['maximum_truth_drift_m'],
              'post_test_process_check_passed': cleanup['passed'],
              'verifier_sha256': digest(Path(__file__).read_bytes()),
              'scope': 'Motion-free refined preview only; metric execution evidence remains frozen v1. '
                       'No author-runtime equivalence or general continuous-space optimum claim.'}
    (evidence/'verification.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def plot(evidence, assets):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    from PIL import Image
    import yaml
    rows = {r['name']: r for r in json.loads((evidence/'results.json').read_text())['cases']}
    cfg = yaml.safe_load((assets/'gcu_ground_floor.yaml').read_text())
    image = np.asarray(Image.open(assets/cfg['image']))
    x, y = cfg['origin'][:2]; r = cfg['resolution']
    extent = [x, x+r*image.shape[1], y, y+r*image.shape[0]]
    fig, axes = plt.subplots(1, 3, figsize=(13, 5.2), constrained_layout=True)
    names = ['adjacent_sector', 'adjacent_diagonal', 'adjacent_down']
    limits = [(-.7, 1.9, -14.8, -12.4), (-1., 6., -15., -7.), (-.7, 1.3, -16.5, -12.3)]
    for axis, name, bounds in zip(axes, names, limits):
        row = rows[name]; route = row['route']
        axis.imshow(image, cmap='gray', vmin=0, vmax=254, extent=extent)
        axis.plot(*np.asarray(route['graph_path']).T, color='#8057dd', linewidth=2.5, label='Graph reference')
        axis.plot(*np.asarray(route['path']).T, color='#007e89', linewidth=2.5, label='Refined preview')
        axis.plot(*row['start'], 'ko', label='Start'); axis.plot(*row['goal'], 'rx', label='Goal')
        axis.set(xlim=bounds[:2], ylim=bounds[2:], xlabel='map x (m)', ylabel='map y (m)',
                 title=f"{route['graph_cost_m']:.2f} m → {route['refined_length_m']:.2f} m")
        axis.set_aspect('equal')
    axes[1].legend(loc='lower right', fontsize=8)
    fig.suptitle('GCU route refinement: same passage sequence, shorter geometry\n'
                 'All three refined segments pass the 0.18 m swept-footprint check')
    fig.savefig(evidence/'before_after.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--assets', type=Path, default=Path('simulation/gcu_ground_floor'))
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()
    result = verify(args.evidence, args.assets)
    if args.plot:
        plot(args.evidence, args.assets)
    print(json.dumps(result, indent=2))
