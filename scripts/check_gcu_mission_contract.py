#!/usr/bin/env python3
"""Bounded Phase 06A GCU geometry and mock-event replay; NEVER starts ROS/UTM.

Resampled reference points are synthetic observations, not measured trajectories.
Mock acknowledgements prove state-contract behavior, not real Nav2 preemption.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
import time

from osmag_paper_planner import PaperPlanner
from osmag_mission_contract import make_contract, CrossingMonitor
from osmag_mission_state import MissionState


def samples(path, step=.01):
    for a, b in zip(path, path[1:]):
        count = max(1, math.ceil(math.dist(a, b)/step))
        for i in range(1, count+1):
            yield tuple(x+(y-x)*i/count for x, y in zip(a, b))


def replay(contract, planner):
    monitor = CrossingMonitor(contract, planner.leaves, planner.passage_segments)
    model = MissionState(); effects = model.start(contract, 0.)
    token = effects[0]['token']; uuid = 'mock-0'
    model.accepted(token, uuid, True, .001)
    dispatches = [{'segment': 0, 'at': 0., 'confirmed_crossings': 0}]
    stamp = 0.; count = 0
    for count, point in enumerate(samples(contract.path), 1):
        stamp = count*.1
        monitor.observe(point, stamp)
        effects = model.advance(point, monitor.confirmed, stamp)
        for effect in effects:
            assert effect['kind'] == 'send'
            old_uuid = uuid
            uuid = 'mock-'+str(effect['segment'].index)
            # Exercise the race: old result arrives BEFORE next acceptance ACK.
            model.result(old_uuid, 'aborted', stamp+.001)
            model.accepted(effect['token'], uuid, True, stamp+.002)
            dispatches.append({'segment': effect['segment'].index, 'at': stamp,
                               'confirmed_crossings': monitor.confirmed})
    assert monitor.confirmed == len(contract.passage_ids), 'unconfirmed_route_transition'
    assert model.index == len(contract.segments)-1, 'terminal_goal_not_dispatched'
    model.result(uuid, 'succeeded', stamp+.01)
    assert model.state == 'verifying'
    # Explicit test fixture input, not an independent physical rest observation.
    assert model.confirm_final(contract.goal, True, stamp+.02)
    return {'passed': True, 'synthetic_observations': count, 'crossings': monitor.events,
            'dispatches': dispatches, 'state': model.report(),
            'scope': 'Reference-path geometry replay and fabricated action results/rest; no physical success claim.'}


def check(assets, output):
    output.mkdir(parents=True, exist_ok=False)
    begin = time.perf_counter(); planner = PaperPlanner(assets)
    start = [-.05864395131342248, -12.960313240266368]
    cases = [('same_sector', [-3., -10.5], [], 'refined_crossing'),
             ('adjacent_sector', [1.4413560486865775, start[1]], [], 'refined_crossing'),
             ('diagonal', [5., -8.], [], 'refined_crossing'),
             ('room_route', [-10., -7.5], [], 'refined_crossing'),
             ('closed_virtual_interface', [1.4413560486865775, start[1]], ['-93'], 'refined_crossing'),
             ('paper_center_comparison', [1.4413560486865775, start[1]], [], 'paper_center')]
    result = {'passed': True, 'scope': 'Phase 06A offline geometry/mock checkpoint only',
              'environment': platform.platform(), 'input_sha256': planner.fingerprints(),
              'parent_checkpoint': '253ce3c', 'ros_started': False, 'utm_started': False,
              'robot_motion_tested': False, 'cases': []}
    source = Path(__file__).parent
    result['source_sha256'] = {name: hashlib.sha256((source/name).read_bytes()).hexdigest() for name in [
        'check_gcu_mission_contract.py', 'osmag_mission_contract.py', 'osmag_mission_state.py',
        'osmag_paper_planner.py', 'osmag_hierarchy.py', 'osmag_route_refinement.py', 'gcu_plan_validation.py']}
    for name, goal, disabled, policy in cases:
        row = {'name': name, 'policy': policy, 'disabled_passages': disabled, 'passed': False}
        try:
            contract = make_contract(planner, start, goal, 0., mission_id=name, now=0., pose_stamp=0.,
                                     disabled_passages=disabled, policy=policy)
            row['contract'] = contract.report(); row['replay'] = replay(contract, planner)
            if name == 'adjacent_sector':
                assert abs(contract.segments[0].point[1]-start[1]) < 1e-7
            if disabled:
                assert not set(contract.passage_ids) & set(disabled)
            row['passed'] = True
        except Exception as error:
            row['error'] = repr(error)
        result['cases'].append(row)
        if not row['passed']:
            result['passed'] = False
            break  # Diagnose the first failure; never start an open-ended sweep.
    for name, goal, reason in [('courtyard', [0., 0.], 'unknown_goal'),
                               ('occupied', [-1.4074548315221413, -16.244374513818027], 'occupied_goal')]:
        if not result['passed']:
            break
        row = {'name': name, 'passed': False}
        try:
            make_contract(planner, start, goal, 0., mission_id=name, now=0., pose_stamp=0.)
        except ValueError as error:
            row.update(reason=str(error), passed=str(error) == reason)
        result['cases'].append(row); result['passed'] &= row['passed']
    result['elapsed_wall_s'] = time.perf_counter()-begin
    (output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', type=Path, default=Path('simulation/gcu_ground_floor'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); result = check(args.assets, args.output)
    print(json.dumps({k: v for k, v in result.items() if k not in ['cases', 'input_sha256', 'source_sha256']}, indent=2))
    for row in result['cases']:
        print(row['name'], row['passed'], row.get('error', ''))
    sys.exit(0 if result['passed'] else 1)
