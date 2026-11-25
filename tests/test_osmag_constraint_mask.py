from pathlib import Path
import sys
import unittest
import tempfile

import numpy as np
from shapely.geometry import box, Polygon
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from osmag_constraint_mask import build_mask, ConsumptionGate, ConstraintError
from gcu_plan_validation import cell_at, validate_path


class MaskFixture:
    def setUp(self):
        self.grid=np.zeros((160,160),dtype=np.int16);self.res=.05;self.origin=(-4.,-4.)
        self.areas={'a':box(-2,-2,0,2),'b':box(0,-2,2,2)}
        self.portals={'p':[(0,-1.5),(0,1.5)]};self.incident={'p':('a','b')}

    def make(self,**kw):
        options=dict(allowed_areas=['a','b'],open_passages=['p'],disabled_passages=[],
                     robot_pose=(-1.,0.),mission_id='fixture')
        options.update(kw)
        return build_mask(self.grid,self.res,self.origin,self.areas,self.portals,self.incident,**options)

    def value(self,snap,x,y):return cell_at(snap.mask,self.res,self.origin,x,y)


class MaskTests(MaskFixture,unittest.TestCase):
    def test_open_crossing_stays_free_while_nonpassage_boundary_is_closed(self):
        s=self.make()
        self.assertEqual(self.value(s,0.,0.),0)
        self.assertEqual(self.value(s,0.,1.8),100)
        self.assertTrue(validate_path(s.combined,self.res,self.origin,[(-1,0),(1,0)],.20)['valid'])

    def test_closed_interface_blocks_clear_raster_shortcut(self):
        s=self.make(open_passages=[],disabled_passages=['p'])
        self.assertEqual(self.value(s,0.,0.),100)
        self.assertFalse(validate_path(s.combined,self.res,self.origin,[(-1,0),(1,0)],.20)['valid'])
        self.assertTrue(validate_path(self.grid,self.res,self.origin,[(-1,0),(1,0)],.20)['valid'])

    def test_unselected_interface_stays_closed_between_selected_areas(self):
        self.portals={'p':[(0,-1.5),(0,-.5)],'q':[(0,.5),(0,1.5)]}
        self.incident['q']=('a','b')
        s=self.make()
        self.assertEqual(self.value(s,0.,-1.),0);self.assertEqual(self.value(s,0.,1.),100)

    def test_unexported_shared_boundary_cannot_create_offroute_shortcut(self):
        self.areas={'a':box(-2,0,0,2),'b':box(-2,2,2,4),'c':box(0,0,2,2)}
        self.portals={'p':[(-1.8,2),(-.2,2)],'q':[(.2,2),(1.8,2)]}
        self.incident={'p':('a','b'),'q':('b','c')}
        s=self.make(allowed_areas=['a','b','c'],open_passages=['p','q'],robot_pose=(-1.,1.))
        self.assertEqual(self.value(s,0.,1.),100)
        self.assertFalse(validate_path(s.combined,self.res,self.origin,[(-1,1),(1,1)],.2)['valid'])

    def test_physical_wall_and_unknown_never_become_free(self):
        self.grid[80,90]=100;self.grid[81,90]=-1
        original=self.grid.copy();s=self.make()
        self.assertEqual(s.mask[80,90],0)
        self.assertEqual(s.combined[80,90],100);self.assertEqual(s.combined[81,90],-1)
        np.testing.assert_array_equal(self.grid,original)

    def test_polygon_hole_smaller_than_cell_is_still_excluded_conservatively(self):
        self.areas={'a':Polygon([(-2,-2),(2,-2),(2,2),(-2,2)],holes=[[(.01,.01),(.02,.01),(.02,.02),(.01,.02)]])}
        self.portals={};self.incident={}
        s=self.make(allowed_areas=['a'],open_passages=[])
        self.assertEqual(self.value(s,.015,.015),100)

    def test_initial_footprint_conflict_is_rejected_without_carving_exception(self):
        with self.assertRaisesRegex(ConstraintError,'footprint_intersects_constraint'):
            self.make(open_passages=[],disabled_passages=['p'],robot_pose=(-.05,0))

    def test_reopen_builds_new_snapshot_without_retaining_or_mutating_old_mask(self):
        closed=self.make(open_passages=[],disabled_passages=['p'],closure_revision=1,mask_revision=1)
        opened=self.make(closure_revision=2,mask_revision=2)
        self.assertNotEqual(closed.mask_sha256,opened.mask_sha256)
        self.assertEqual(self.value(closed,0,0),100);self.assertEqual(self.value(opened,0,0),0)
        with self.assertRaises(ValueError):closed.mask[0,0]=0

    def test_invalid_open_closed_or_incident_identity_is_rejected(self):
        for options in [dict(disabled_passages=['p']),dict(open_passages=['missing']),
                        dict(allowed_areas=['a']),dict(mask_revision=0)]:
            with self.subTest(options=options),self.assertRaises(ConstraintError):self.make(**options)


class GateTests(MaskFixture,unittest.TestCase):
    def setUp(self):
        MaskFixture.setUp(self);self.snap=self.make();self.gate=ConsumptionGate(self.snap,10.,[(-1,0),(1,0)])

    def observe(self,source,**kw):
        values=dict(grid=self.snap.combined,resolution=self.res,origin=self.origin,stamp=10.1,now=10.2)
        values.update(kw);return self.gate.observe(source,**values)

    def test_both_fresh_costmaps_required_and_old_observations_expire(self):
        self.observe('global');self.assertFalse(self.gate.ready(10.2))
        self.observe('local');self.assertTrue(self.gate.ready(10.2));self.assertFalse(self.gate.ready(11.))

    def test_missing_mask_cells_and_stale_clearing_are_not_acknowledgements(self):
        self.observe('global')
        with self.assertRaisesRegex(ConstraintError,'not_consumed'):self.observe('global',grid=self.grid)
        data=self.snap.combined.copy();data[80,100]=100
        with self.assertRaisesRegex(ConstraintError,'free_probe_blocked'):self.observe('global',grid=data)

    def test_stamp_frame_resolution_and_obsolete_revision_guards(self):
        self.observe('global');self.observe('local')
        for kwargs,reason in [(dict(stamp=9.9),'not_fresh'),(dict(frame='odom'),'transform_required'),
                              (dict(resolution=.1),'resolution'),(dict(mask_sha256='old'),'obsolete')]:
            with self.subTest(kwargs=kwargs),self.assertRaisesRegex(ConstraintError,reason):self.observe('global',**kwargs)
            self.assertFalse(self.gate.ready(10.2))

    def test_global_partial_map_cannot_satisfy_full_mask_readiness(self):
        with self.assertRaisesRegex(ConstraintError,'does_not_cover'):
            self.observe('global',grid=self.snap.combined[40:120,40:120],origin=(-2,-2))

    def test_local_window_can_validate_its_overlap_with_a_free_probe(self):
        self.observe('global')
        self.observe('local',grid=self.snap.combined[40:120,40:120],origin=(-2,-2))
        self.assertTrue(self.gate.ready(10.2))
        with self.assertRaisesRegex(ConstraintError, 'does_not_overlap'):
            self.observe('local', grid=self.snap.combined[40:120,40:120], origin=(20,20))
        self.assertFalse(self.gate.ready(10.2))


class ConfigTests(unittest.TestCase):
    def test_humble_native_params_do_not_contain_yaml_aliases(self):
        from prepare_constraint_costmap_test import prepare
        import yaml
        with tempfile.TemporaryDirectory() as folder:
            target=Path(folder)/'workspace';prepare(target)
            text=(target/'params.yaml').read_text()
            self.assertNotIn('&id',text);self.assertNotIn('*id',text)
            config=yaml.safe_load(text)
            for name in ['global_costmap','costmap']:
                self.assertEqual(config[name][name]['ros__parameters']['filters'],['mission_keepout'])


if __name__=='__main__':unittest.main()
