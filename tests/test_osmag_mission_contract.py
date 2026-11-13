"""Synthetic geometry/callback fixtures; these are not simulated robot trials."""
import copy
import math
from pathlib import Path
import sys
import unittest

import numpy as np
from shapely.geometry import box, LineString, Point

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from osmag_mission_contract import make_contract, CrossingMonitor, MissionError, MissionSettings
from osmag_route_refinement import RouteCorridor
from gcu_plan_validation import validate_path


class FixturePlanner:
    def __init__(self, three=False, door=False):
        self.leaves = {'a': box(-2, -2, 0, 2), 'b': box(0, -2, 2, 2)}
        self.passage_segments = {'p': [(0, -.35 if door else -2), (0, .35 if door else 2)]}
        self.passages = {'p': {'osmAG:from': 'a', 'osmAG:to': 'b'}}
        if three:
            self.leaves['c'] = box(2, -2, 4, 2)
            self.passage_segments['q'] = [(2, -2), (2, 2)]
            self.passages['q'] = {'osmAG:from': 'b', 'osmAG:to': 'c'}
        self.grid = np.zeros((240, 240), dtype=int)
        self.resolution = .05; self.origin = [-4, -4]; self.radius = .18
        self.path = None; self.calls = 0

    def plan(self, start, goal, disabled=()):
        self.calls += 1
        area = lambda point: next(n for n, p in self.leaves.items() if p.contains(Point(point)))
        first, last = area(start), area(goal)
        if first == last:
            areas = [first]; passages = []
        elif last == 'c':
            areas = ['a', 'b', 'c']; passages = ['p', 'q']
        else:
            areas = ['a', 'b']; passages = ['p']
        path = self.path or [list(start), list(goal)]
        topology = RouteCorridor(self.leaves, self.passage_segments, areas, passages, disabled).validate(path)
        if not topology['valid']:
            raise MissionError('fixture_route_invalid')
        return {'start': list(start), 'goal': list(goal), 'path': path, 'area_ids': areas,
                'passage_ids': passages, 'disabled_passages': list(disabled), 'map_sha256': 'fixture-map',
                'refinement': {'topology': topology}}

    def grid_path(self, area, start, goal):
        return [list(start), list(goal)] if self.leaves[area].covers(LineString([start, goal])) else None


def contract(planner=None, start=(-.12, 1.), goal=(1., 1.), **kwargs):
    return make_contract(planner or FixturePlanner(), start, goal, kwargs.pop('yaw', .7),
                         mission_id='fixture', now=0., pose_stamp=0., **kwargs)


class ContractTests(unittest.TestCase):
    def test_metric_comparator_keeps_constraints_but_has_only_final_goal(self):
        from gcu_metric_baseline import metric_contract
        original=contract();metric=metric_contract(original)
        self.assertEqual(metric.passage_ids,original.passage_ids);self.assertEqual(metric.area_ids,original.area_ids)
        self.assertEqual(metric.path,original.path);self.assertEqual(len(metric.segments),1)
        self.assertEqual(metric.segments[0].point,original.goal[:2]);self.assertEqual(metric.segments[0].index,0)
        self.assertNotEqual(metric.route_sha256,original.route_sha256);self.assertEqual(len(original.segments),2)

    def test_wide_interface_uses_actual_refined_crossing_without_midpoint_detour(self):
        p = FixturePlanner(); c = contract(p)
        self.assertLess(math.dist(c.segments[0].point, (0., 1.)), 1e-9)
        self.assertEqual(c.segments[0].yaw, 0.)
        self.assertEqual(c.segments[-1].yaw, .7)
        self.assertEqual(p.calls, 1)

    def test_paper_center_is_labelled_and_reconnects_locally(self):
        c = contract(policy='paper_center')
        self.assertEqual(c.segments[0].point, (0., 0.))
        self.assertAlmostEqual(c.segments[0].yaw, math.pi/4)
        self.assertNotEqual(c.route_sha256, contract().route_sha256)

    def test_same_sector_has_only_terminal_goal(self):
        c = contract(start=(-1., 0.), goal=(-.5, .5))
        self.assertEqual(c.passage_ids, ()); self.assertEqual(len(c.segments), 1)
        self.assertIsNone(c.segments[0].passage_id)

    def test_narrow_door_keeps_named_boundary_crossing(self):
        p = FixturePlanner(door=True); c = contract(p, start=(-1., 0.), goal=(1., 0.))
        self.assertEqual(c.segments[0].point, (0., 0.))
        self.assertEqual(c.segments[0].destination_area, 'b')

    def test_three_areas_keep_ordered_crossings(self):
        c = contract(FixturePlanner(three=True), goal=(3., 1.))
        self.assertEqual(c.passage_ids, ('p', 'q'))
        self.assertEqual([s.index for s in c.segments], [0, 1, 2])

    def test_bad_clock_and_policy_are_rejected_before_query(self):
        p = FixturePlanner()
        for now, stamp in [(1., 0.), (0., 1.), (math.nan, 0.)]:
            with self.subTest(now=now), self.assertRaises(MissionError):
                make_contract(p, (-1, 0), (1, 0), 0., mission_id='x', now=now, pose_stamp=stamp)
        with self.assertRaisesRegex(MissionError, 'unsupported_segment_policy'):
            contract(p, policy='unchecked_straight')
        self.assertEqual(p.calls, 0)

    def test_map_settings_and_route_revisions_are_frozen(self):
        c = contract(route_revision=2, closure_revision=3)
        self.assertEqual(c.route_revision, 2); self.assertEqual(c.closure_revision, 3)
        self.assertEqual(len(c.settings_sha256), 64)
        with self.assertRaises(AttributeError):
            c.policy = 'changed'

    def test_invalid_confirmation_parameters_are_rejected(self):
        for settings in [{'confirmation_samples': 1}, {'maximum_pose_step_m': 0.},
                         {'crossing_hysteresis_m': math.nan}]:
            with self.subTest(settings=settings), self.assertRaises(MissionError):
                MissionSettings(**settings)

    def test_closed_physically_clear_portal_is_rejected(self):
        p = FixturePlanner()
        self.assertTrue(validate_path(p.grid, p.resolution, p.origin, [[-1, 0], [1, 0]], p.radius)['valid'])
        with self.assertRaisesRegex(ValueError, 'disabled_selected_passage'):
            contract(p, start=(-1, 0), goal=(1, 0), disabled_passages=['p'])

    def test_crossing_provenance_mismatch_is_rejected(self):
        p = FixturePlanner(); original = p.plan
        def corrupt(*args):
            route = copy.deepcopy(original(*args))
            route['refinement']['topology']['crossings'][0]['point'] = [0., 0.]
            return route
        p.plan = corrupt
        with self.assertRaisesRegex(MissionError, 'crossing_provenance_mismatch'):
            contract(p)

    def test_invalid_incident_area_or_physical_obstacle_is_rejected(self):
        p = FixturePlanner(); p.passages['p']['osmAG:to'] = 'c'
        with self.assertRaisesRegex(MissionError, 'passage_incident_area_mismatch'):
            contract(p)
        p = FixturePlanner(); p.grid[98:102, 78:82] = 100
        with self.assertRaisesRegex(MissionError, 'footprint_blocked'):
            contract(p)

    def test_final_goal_too_close_to_transition_cannot_claim_hysteresis_confirmation(self):
        with self.assertRaisesRegex(MissionError, 'final_goal_inside_crossing_hysteresis'):
            contract(goal=(.03, 1.))


class CrossingTests(unittest.TestCase):
    def setUp(self):
        self.p = FixturePlanner(); self.c = contract(self.p)
        self.m = CrossingMonitor(self.c, self.p.leaves, self.p.passage_segments)

    def test_crossing_requires_three_deep_fresh_observations(self):
        for point, t in [((-.02, 1), .05), ((.01, 1), .10), ((.07, 1), .15), ((.08, 1), .20)]:
            self.assertIsNone(self.m.observe(point, t))
        event = self.m.observe((.09, 1), .25)
        self.assertEqual(event['passage_id'], 'p'); self.assertEqual(self.m.confirmed, 1)

    def test_boundary_jitter_resets_confirmation_without_counting_twice(self):
        points = [(-.01, 1), (.01, 1), (-.01, 1), (.02, 1), (.07, 1), (.08, 1), (.09, 1)]
        for i, point in enumerate(points):
            self.m.observe(point, .05*(i+1))
        self.assertEqual(len(self.m.events), 1)
        self.m.observe((.10, 1), .4); self.assertEqual(len(self.m.events), 1)

    def test_pose_gap_duplicate_stamp_and_jump_do_not_advance(self):
        for point, stamp, reason in [((.1, 1), .3, 'gap'), ((.1, 1), 0., 'stamp'), ((1., 1), .1, 'jump')]:
            with self.subTest(reason=reason), self.assertRaisesRegex(MissionError, reason):
                self.m.observe(point, stamp)
            self.assertEqual(self.m.confirmed, 0)

    def test_wrong_portal_cannot_be_confirmed_even_with_same_adjacent_areas(self):
        p = FixturePlanner(door=True); c = contract(p, start=(-.12, 0), goal=(1, 0))
        m = CrossingMonitor(c, p.leaves, p.passage_segments)
        for i in range(1, 8):
            m.observe((-.12, i*.1), i*.1)
        with self.assertRaisesRegex(MissionError, 'unexpected_area_transition'):
            m.observe((.1, .7), .8)

    def test_confirmed_transition_cannot_reverse_to_previous_area(self):
        for i, point in enumerate([(.07, 1), (.08, 1), (.09, 1)]):
            self.m.observe(point, .1*(i+1))
        with self.assertRaisesRegex(MissionError, 'unexpected_area_transition'):
            self.m.observe((-.1, 1), .4)

    def test_direct_shortcut_to_later_area_cannot_skip_ordered_passages(self):
        p = FixturePlanner(three=True)
        p.leaves = {'a': box(-2, 0, 0, 2), 'b': box(-2, 2, 2, 4), 'c': box(0, 0, 2, 2)}
        p.passage_segments = {'p': [(-2, 2), (0, 2)], 'q': [(0, 2), (2, 2)]}
        p.path = [[-.12, 1], [-.12, 3], [1, 3], [1, 1]]
        c = contract(p, goal=(1, 1))
        m = CrossingMonitor(c, p.leaves, p.passage_segments)
        with self.assertRaisesRegex(MissionError, 'unexpected_area_transition'):
            m.observe((.1, 1), .1)
        self.assertEqual(m.confirmed, 0)


if __name__ == '__main__':
    unittest.main()
