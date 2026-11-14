from dataclasses import replace
from pathlib import Path
import sys
import unittest
import numpy as np
from shapely.geometry import box
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from osmag_rolling_raster import RollingRaster
from osmag_window_goal import select_window_goal
from osmag_window_admission import WindowAdmission
from osmag_constraint_mask import build_mask,ConstraintError


class WindowAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.areas={'a':box(-3,-3,3,3)}
        self.window=RollingRaster(self.areas,{},resolution=.05,shape=(100,100)).render((0.,0.),revision=1,stamp=1.)
        self.snapshot=build_mask(self.window.grid,.05,self.window.origin,self.areas,{}, {},allowed_areas=['a'],
            open_passages=[],disabled_passages=[],robot_pose=(0.,0.),mission_id='test')
        self.goal=select_window_goal(self.window,[(0.,0.),(10.,0.)],(0.,0.))
        self.gate=WindowAdmission(self.window,self.snapshot,1.,[(0.,0.)])

    def consume(self):
        for source in ['global','local']:
            self.gate.observe(source,self.snapshot.combined,.05,self.window.origin,1.1,1.2)

    def test_both_costmaps_required_before_admission(self):
        with self.assertRaises(ConstraintError):self.gate.admit(self.goal,1.2)
        self.consume();self.assertEqual(self.gate.admit(self.goal,1.2)['window_revision'],1)

    def test_old_proxy_and_stale_consumption_rejected(self):
        self.consume()
        with self.assertRaisesRegex(ConstraintError,'obsolete'):self.gate.admit(replace(self.goal,window_revision=2),1.2)
        with self.assertRaisesRegex(ConstraintError,'freshly'):self.gate.admit(self.goal,2.)

    def test_full_floor_or_old_origin_global_map_cannot_ack_window(self):
        self.consume()
        with self.assertRaises(ConstraintError):
            self.gate.observe('global',np.zeros((200,200)),.05,self.window.origin,1.1,1.2)
        self.assertFalse(self.gate.ready(1.2))
        with self.assertRaises(ConstraintError):
            self.gate.observe('global',self.snapshot.combined,.05,(0.,0.),1.1,1.2)

    def test_mismatched_physical_content_or_revision_rejected(self):
        for snapshot in [replace(self.snapshot,mask_revision=2),replace(self.snapshot,physical_sha256='wrong')]:
            with self.subTest(snapshot=snapshot.mask_revision),self.assertRaises(ConstraintError):
                WindowAdmission(self.window,snapshot,1.,[(0.,0.)])

    def test_goal_crossing_newly_forbidden_geometry_rejected(self):
        # Different valid proxy on the same window can still have an unsafe prefix.
        self.consume()
        bad=replace(self.goal,prefix=((0.,0.),(2.49,0.)),point=(2.49,0.))
        with self.assertRaisesRegex(ConstraintError,'blocked'):self.gate.admit(bad,1.2)

    def test_fresh_native_grids_cannot_hide_stale_source_publication(self):
        for source in ['global','local']:
            self.gate.observe(source,self.snapshot.combined,.05,self.window.origin,1.9,2.)
        with self.assertRaisesRegex(ConstraintError,'freshly'):self.gate.admit(self.goal,2.)
        with self.assertRaises(ConstraintError):self.gate.note_source_publication(1.9,'wrong')
        self.gate.note_source_publication(1.9,self.window.sha256)
        self.assertEqual(self.gate.admit(self.goal,2.)['window_revision'],1)

    def test_update_waits_for_new_native_ack_without_extending_old_lease(self):
        self.consume();self.gate.note_source_publication(1.25,self.window.sha256)
        self.assertTrue(self.gate.ready(1.3));self.assertFalse(self.gate.can_replace(1.3))
        for source in ['global','local']:
            self.gate.observe(source,self.snapshot.combined,.05,self.window.origin,1.25,1.3)
        self.assertTrue(self.gate.can_replace(1.3))
        self.assertFalse(self.gate.ready(2.))


if __name__=='__main__':unittest.main()
