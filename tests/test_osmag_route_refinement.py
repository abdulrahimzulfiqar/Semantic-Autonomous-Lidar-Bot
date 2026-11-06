import math
from pathlib import Path
import sys
import unittest
import numpy as np
from shapely.geometry import box, LineString
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from osmag_route_refinement import RouteCorridor, refine_route, path_length
from gcu_plan_validation import validate_path


class CorridorRefinementTests(unittest.TestCase):
    def test_wide_open_portal_does_not_force_midpoint(self):
        corridor=RouteCorridor({'a':box(-2,-3,0,3),'b':box(0,-3,2,3)},
                               {'p':[(0,-3),(0,3)]},['a','b'],['p'])
        original=[[-1,2],[-.3,0],[.3,0],[1,2]]
        path,report=refine_route(original,corridor,lambda _: {'valid':True})
        self.assertEqual(path,[original[0],original[-1]])
        self.assertEqual(report['topology']['passage_ids'],['p'])
        self.assertAlmostEqual(report['topology']['crossings'][0]['point'][1],2.)

    def test_real_narrow_door_keeps_crossing_inside_door(self):
        corridor=RouteCorridor({'a':box(-2,-3,0,3),'b':box(0,-3,2,3)},
                               {'p':[(0,-.3),(0,.3)]},['a','b'],['p'])
        original=[[-1,2],[-.3,0],[.3,0],[1,2]]
        path,report=refine_route(original,corridor,lambda _: {'valid':True})
        self.assertGreater(path_length(path),2.)
        self.assertLessEqual(abs(report['topology']['crossings'][0]['point'][1]),.3)

    def test_closed_interface_cannot_be_shortcut_even_when_physically_clear(self):
        areas={'a':box(-2,-3,0,3),'b':box(0,-3,2,3)}
        segments={'open':[(0,1),(0,2)],'closed':[(0,-1),(0,1)]}
        corridor=RouteCorridor(areas,segments,['a','b'],['open'],['closed'])
        original=[[-1,0],[-.3,1.5],[.3,1.5],[1,0]]
        path,report=refine_route(original,corridor,lambda _: {'valid':True})
        self.assertTrue(report['topology']['valid'])
        self.assertTrue(all(not LineString([a,b]).intersects(LineString(segments['closed']))
                            for a,b in zip(path,path[1:])))

    def test_swept_footprint_checker_prevents_shortcut_through_obstacle(self):
        grid=np.zeros((100,100),dtype=int);grid[45:55,45:55]=100
        physical=lambda points:validate_path(grid,.1,[0,0],points,.18)
        corridor=RouteCorridor({'a':box(0,0,10,10)}, {}, ['a'], [])
        original=[[2,5],[2,8],[8,8],[8,5]]
        self.assertFalse(physical([original[0],original[-1]])['valid'])
        path,report=refine_route(original,corridor,physical)
        self.assertTrue(physical(path)['valid']);self.assertGreater(path_length(path),6.)

    def test_portal_vertex_and_collinear_points_preserve_order(self):
        corridor=RouteCorridor({'a':box(-2,-1,0,1),'b':box(0,-1,2,1),'c':box(2,-1,4,1)},
                               {'p':[(0,-1),(0,1)],'q':[(2,-1),(2,1)]},['a','b','c'],['p','q'])
        path,report=refine_route([[-1,0],[0,0],[1,0],[2,0],[3,0]],corridor,lambda _: {'valid':True})
        self.assertEqual(path,[[-1,0],[3,0]])
        self.assertEqual(report['topology']['passage_ids'],['p','q'])

    def test_shortcut_does_not_skip_intermediate_sector(self):
        corridor=RouteCorridor({'a':box(-2,0,0,2),'b':box(-2,2,2,4),'c':box(0,0,2,2)},
                               {'p':[(-2,2),(0,2)],'q':[(0,2),(2,2)]},['a','b','c'],['p','q'])
        original=[[-1,1],[-1,3],[1,3],[1,1]]
        path,report=refine_route(original,corridor,lambda _: {'valid':True})
        self.assertEqual(report['topology']['passage_ids'],['p','q'])
        self.assertGreater(path_length(path),2.)

    def test_budget_exhaustion_retains_valid_route(self):
        corridor=RouteCorridor({'a':box(-2,-3,0,3),'b':box(0,-3,2,3)},
                               {'p':[(0,-.3),(0,.3)]},['a','b'],['p'])
        original=[[-1,2],[-.3,0],[.3,0],[1,2]]
        path,report=refine_route(original,corridor,lambda _: {'valid':True},maximum_attempts=0)
        self.assertTrue(corridor.validate(path)['valid'])
        self.assertTrue(report['budget_exhausted'])


if __name__=='__main__':unittest.main()
