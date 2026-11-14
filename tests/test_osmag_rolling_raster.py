from pathlib import Path
import sys
import unittest
import tempfile

import numpy as np
from shapely.geometry import Polygon, box

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from osmag_rolling_raster import RollingRaster
from gcu_plan_validation import cell_at, validate_path
from osmag_constraint_mask import build_mask


class RollingRasterTests(unittest.TestCase):
    def make(self, **kw):
        options = dict(resolution=.05, anchor=(-4., -4.), shape=(100, 100))
        options.update(kw)
        return RollingRaster({'a': box(-3, -3, 0, 3), 'b': box(0, -3, 3, 3)},
                             {'p': [(0., -1.), (0., 1.)]}, **options)

    def test_named_passage_open_but_remaining_partition_closed(self):
        s = self.make().render((0., 0.), revision=1, stamp=1.)
        self.assertEqual(cell_at(s.grid, s.resolution, s.origin, 0., 0.), 0)
        self.assertEqual(cell_at(s.grid, s.resolution, s.origin, 0., 1.5), 100)
        self.assertTrue(validate_path(s.grid, s.resolution, s.origin,
                                      [(-.5, 0.), (.5, 0.)], .20)['valid'])

    def test_hole_stays_nonfree(self):
        p = Polygon([(-3,-3),(3,-3),(3,3),(-3,3)],
                    holes=[[(-.5,-.5),(.5,-.5),(.5,.5),(-.5,.5)]])
        s = RollingRaster({'a':p}, {}, resolution=.05).render((0.,0.), revision=1, stamp=0.)
        self.assertNotEqual(cell_at(s.grid,s.resolution,s.origin,0.,0.),0)
        self.assertFalse(validate_path(s.grid,s.resolution,s.origin,[(-1.,0.),(1.,0.)],.2)['valid'])

    def test_shifted_windows_keep_identical_overlap_except_explicit_edge(self):
        r = self.make()
        a = r.render((0.,0.), revision=1, stamp=1.)
        b = r.render((.5,0.), revision=2, stamp=2.)
        np.testing.assert_array_equal(a.grid[1:-1,11:-1],b.grid[1:-1,1:-11])
        self.assertEqual(a.grid.nbytes,b.grid.nbytes)
        self.assertEqual(a.grid.shape,b.grid.shape)

    def test_outside_geometry_unknown_with_blocked_window_border(self):
        s = self.make().render((100.,100.), revision=1, stamp=0.)
        self.assertTrue(np.all(s.grid[1:-1,1:-1] == -1))
        self.assertTrue(np.all(s.grid[0,:] == 100))
        self.assertFalse(validate_path(s.grid,s.resolution,s.origin,[(100.,100.)],.2)['valid'])

    def test_small_pose_change_does_not_shift_grid_until_cell_boundary(self):
        r = self.make()
        a = r.render((.001,.001), revision=1, stamp=1.)
        b = r.render((.002,.002), revision=2, stamp=2.)
        self.assertEqual(a.origin,b.origin)
        np.testing.assert_array_equal(a.grid,b.grid)

    def test_revision_and_stamp_are_hashed_and_arrays_readonly(self):
        r = self.make()
        a = r.render((0.,0.), revision=1, stamp=1.)
        b = r.render((0.,0.), revision=2, stamp=2.)
        self.assertNotEqual(a.sha256,b.sha256)
        self.assertEqual(a.source_sha256,b.source_sha256)
        with self.assertRaises(ValueError):a.grid[4,4] = 0

    def test_existing_constraint_builder_closes_a_passage_on_same_window(self):
        s = self.make().render((0.,0.), revision=1, stamp=1.)
        m = build_mask(s.grid,s.resolution,s.origin,
                       {'a':box(-3,-3,0,3),'b':box(0,-3,3,3)},
                       {'p':[(0.,-1.),(0.,1.)]}, {'p':('a','b')},
                       allowed_areas=['a','b'],open_passages=[],disabled_passages=['p'],
                       robot_pose=(-1.,0.),mission_id='rolling_fixture')
        self.assertEqual(cell_at(m.combined,s.resolution,s.origin,0.,0.),100)
        self.assertFalse(validate_path(m.combined,s.resolution,s.origin,[(-1.,0.),(1.,0.)],.2)['valid'])

    def test_invalid_grid_parameters_rejected(self):
        for options in [dict(resolution=0),dict(resolution=float('nan')),
                        dict(shape=(0,100)),dict(shape=(100.,100)),dict(anchor=(float('inf'),0))]:
            with self.subTest(options=options),self.assertRaises(ValueError):self.make(**options)

    def test_invalid_pose_or_identity_rejected(self):
        for options in [dict(center=(float('nan'),0),revision=1,stamp=1),
                        dict(center=(0,0),revision=0,stamp=1),
                        dict(center=(0,0),revision=1,stamp=-1)]:
            with self.subTest(options=options),self.assertRaises(ValueError):self.make().render(**options)

    def test_invalid_source_geometry_rejected(self):
        with self.assertRaises(ValueError):
            RollingRaster({'a':Polygon([(0,0),(1,1),(1,0),(0,1)])},{},resolution=.1)
        with self.assertRaises(ValueError):
            RollingRaster({'a':box(-1,-1,1,1)},{'p':[(0,0),(0,0)]},resolution=.1)

    def test_gcu_windows_preserve_physical_occupancy_and_admitted_routes(self):
        from check_gcu_rolling_raster import audit
        assets = Path(__file__).resolve().parents[1]/'simulation/gcu_ground_floor'
        with tempfile.TemporaryDirectory() as folder:
            r = audit(assets,Path(folder)/'audit')
            self.assertTrue(r['offline_geometry_gate_passed'])
            self.assertFalse(r['native_motion_admitted'])
            self.assertEqual(r['full_floor_evaluation']['unique_false_free_cells'],0)
            self.assertTrue(r['full_floor_evaluation']['all_reference_cells_covered'])
            self.assertEqual(len(r['route_clearance_checks']),4)


if __name__ == '__main__':unittest.main()
