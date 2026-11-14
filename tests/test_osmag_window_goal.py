from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from shapely.geometry import box
from osmag_rolling_raster import RollingRaster
from osmag_window_goal import select_window_goal,route_suffix


class WindowGoalTests(unittest.TestCase):
    def setUp(self):
        self.raster=RollingRaster({'a':box(-20,-20,20,20)},{},resolution=.05,shape=(100,100))
        self.window=self.raster.render((0.,0.),revision=1,stamp=1.)

    def test_distant_goal_becomes_padded_proxy(self):
        goal=select_window_goal(self.window,[(0.,0.),(10.,0.)],(0.,0.))
        self.assertTrue(goal.proxy)
        self.assertAlmostEqual(goal.point[0],2.05)
        self.assertEqual(goal.window_sha256,self.window.sha256)

    def test_in_window_goal_keeps_final_heading(self):
        g=select_window_goal(self.window,[(0.,0.),(1.,1.)],(0.,0.),final_yaw=math.pi)
        self.assertFalse(g.proxy);self.assertEqual(g.point,(1.,1.));self.assertEqual(g.yaw,math.pi)

    def test_exit_then_reentry_does_not_skip_first_exit(self):
        g=select_window_goal(self.window,[(0.,0.),(8.,0.),(1.,1.)],(0.,0.))
        self.assertTrue(g.proxy);self.assertAlmostEqual(g.point[0],2.05)
        self.assertEqual(len(g.prefix),2)

    def test_unknown_or_barrier_cannot_be_bypassed(self):
        r=RollingRaster({'a':box(-2,-2,-.4,2),'b':box(.4,-2,2,2)},{},resolution=.05,shape=(100,100))
        with self.assertRaisesRegex(ValueError,'footprint'):
            select_window_goal(r.render((-1.,0.),revision=1,stamp=1.),[(-1.,0.),(1.,0.)],(-1.,0.))

    def test_unsafe_margin_and_offroute_pose_rejected(self):
        with self.assertRaises(ValueError):select_window_goal(self.window,[(0.,0.),(1.,0.)],(0.,0.),edge_margin=.1)
        with self.assertRaises(ValueError):select_window_goal(self.window,[(0.,0.),(1.,0.)],(.2,0.))
        with self.assertRaises(ValueError):select_window_goal(self.window,[(2.4,0.),(4.,0.)],(2.4,0.))

    def test_shift_moves_proxy_forward_without_changing_world_route(self):
        route=[(0.,0.),(10.,0.)]
        a=select_window_goal(self.window,route,(0.,0.))
        suffix=route_suffix(route,1.)
        b=select_window_goal(self.raster.render((1.,0.),revision=2,stamp=2.),suffix,suffix[0])
        self.assertGreater(b.point[0],a.point[0]);self.assertEqual(b.window_revision,2)

    def test_suffix_interpolates_and_handles_duplicate_points(self):
        self.assertEqual(route_suffix([(0,0),(0,0),(2,0),(2,2)],3),[[2.,1.],[2,2]])
        self.assertEqual(route_suffix([(0,0),(2,0)],5),[[2,0]])

    def test_nonfinite_inputs_rejected(self):
        with self.assertRaises(ValueError):route_suffix([(0,0),(2,0)],float('nan'))
        with self.assertRaises(ValueError):select_window_goal(self.window,[(0,0),(float('inf'),0)],(0,0))


if __name__=='__main__':unittest.main()
