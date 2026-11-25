from pathlib import Path
import json
import math
import tempfile
import shutil
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from experiments.localization_comparison.structural_2d.tracker import StructuralMap,Tracker,compose,relative,paper_weights


class StructuralLocalizationTests(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads((ROOT/'experiments/localization_comparison/structural_2d/config.json').read_text())
        self.room=StructuralMap([[[0,0],[8,0]],[[8,0],[8,6]],[[8,6],[0,6]],[[0,6],[0,0]]])
        self.angles=np.linspace(-math.pi,math.pi,360,endpoint=False)

    def scan(self,model,base,extrinsic=(0.,0.,0.)):
        sensor=compose(base,extrinsic);directions=np.column_stack([np.cos(self.angles+sensor[2]),np.sin(self.angles+sensor[2])])
        return model.intersect(model.segments,sensor[:2],directions)[0]

    def test_equation_eight_boundaries_and_asymmetry(self):
        values=paper_weights(np.array([-2.,-1.,-.5,0.,.5,3.,np.nan]))
        np.testing.assert_allclose(values,[0,0,1/1.75,1,1/2.5,0,0])

    def test_se2_relative_inverse_with_heading_wrap(self):
        a=[1.,2.,3.12];b=[3.,4.,-3.12]
        np.testing.assert_allclose(compose(a,relative(a,b)),b,atol=1e-12)

    def test_ray_intersections_and_parallel_segments(self):
        distance,_=self.room.intersect(self.room.segments,[2.,3.],np.array([[1.,0.],[0.,1.],[-1.,0.],[0.,-1.]]))
        np.testing.assert_allclose(distance,[6,3,2,3])

    def test_local_pose_correction_and_sensor_extrinsic(self):
        truth=[2.5,2.,.22];extrinsic=[.15,-.08,.1]
        tracker=Tracker(self.room,[2.65,1.9,.27],self.cfg)
        out=tracker.update(1.,self.angles,self.scan(self.room,truth,extrinsic),[0.,0.,0.],extrinsic,.15,12.)
        self.assertTrue(out['accepted']);np.testing.assert_allclose(out['pose'],truth,atol=.015)

    def test_far_foreground_clutter_is_not_a_wall(self):
        truth=[3.,3.,.1];ranges=self.scan(self.room,truth);ranges[:140]=.5
        tracker=Tracker(self.room,[3.1,2.9,.13],self.cfg)
        out=tracker.update(1.,self.angles,ranges,[0.,0.,0.],[0.,0.,0.],.15,12.)
        self.assertTrue(out['accepted']);np.testing.assert_allclose(out['pose'],truth,atol=.03)
        self.assertLess(out['matches'],len(ranges))

    def test_unobserved_corridor_axis_uses_odometry_not_invented_correction(self):
        model=StructuralMap([[[-100,-2],[100,-2]],[[100,2],[-100,2]]])
        tracker=Tracker(model,[.35,.1,.02],self.cfg)
        out=tracker.update(1.,self.angles,self.scan(model,[0.,0.,0.]),[0.,0.,0.],[0.,0.,0.],.15,12.)
        self.assertAlmostEqual(out['pose'][0],.35,places=5);self.assertEqual(out['reason'],'weak_geometry')
        out=tracker.update(1.1,self.angles,self.scan(model,[.5,0.,0.]),[.5,0.,0.],[0.,0.,0.],.15,12.)
        self.assertAlmostEqual(out['pose'][0],.85,places=5);self.assertLess(abs(out['pose'][1]),.01)

    def test_unknown_maximum_ranges_are_not_fabricated_wall_returns(self):
        tracker=Tracker(self.room,[2.,2.,0.],self.cfg)
        for i in range(5):out=tracker.update(1.+i*.1,self.angles,np.full(360,np.inf),[i*.1,0.,0.],[0.,0.,0.],.15,12.)
        self.assertFalse(out['accepted']);self.assertTrue(out['lost']);self.assertAlmostEqual(out['pose'][0],2.4)

    def test_scan_gap_is_explicit_and_odometry_prediction_is_retained(self):
        tracker=Tracker(self.room,[2.,2.,0.],self.cfg);ranges=self.scan(self.room,[2.,2.,0.])
        tracker.update(1.,self.angles,ranges,[0.,0.,0.],[0.,0.,0.],.15,12.)
        out=tracker.update(2.,self.angles,ranges,[.2,0.,0.],[0.,0.,0.],.15,12.)
        self.assertFalse(out['accepted']);self.assertEqual(out['reason'],'scan_gap');self.assertAlmostEqual(out['pose'][0],2.2)

    def test_repeated_stamp_and_nonfinite_odometry_are_rejected(self):
        tracker=Tracker(self.room,[2.,2.,0.],self.cfg);ranges=self.scan(self.room,[2.,2.,0.])
        tracker.update(1.,self.angles,ranges,[0.,0.,0.],[0.,0.,0.],.15,12.)
        with self.assertRaises(ValueError):tracker.update(1.,self.angles,ranges,[0.,0.,0.],[0.,0.,0.],.15,12.)
        with self.assertRaises(ValueError):tracker.update(1.1,self.angles,ranges,[np.nan,0.,0.],[0.,0.,0.],.15,12.)

    def test_closed_door_return_and_open_passage_have_distinct_correspondences(self):
        model=StructuralMap(self.room.segments,[[[4,0],[4,6]]])
        for measured,want in [(2.,4.),(6.,8.)]:
            result=model.correspond([2.,3.,0.],np.array([[measured,0.]]),np.array([0.]),np.array([measured]),[0.,0.,0.],.1)
            self.assertAlmostEqual(result[3][0,0,0],want)

    def test_osm_only_map_can_load_without_occupancy_files(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'map.osm';shutil.copyfile(ROOT/'simulation/gcu_ground_floor/gcu_ground_floor.osm',path)
            model=StructuralMap.from_osm(path)
            self.assertGreater(len(model.segments),100);self.assertEqual(list(Path(folder).iterdir()),[path])




class PairedScoringTests(unittest.TestCase):
    def test_fixed_position_bias_is_not_aligned_away(self):
        from experiments.localization_comparison.common.score import metrics
        truth=np.array([[0.,0.,0.],[1.,0.,0.]])
        result=metrics(truth+np.array([1.,0.,0.]),truth,[1.,2.])
        self.assertAlmostEqual(result['position_rmse_m'],1.)

    def test_heading_error_wraps_at_pi(self):
        from experiments.localization_comparison.common.score import metrics
        result=metrics([[0.,0.,-math.pi+.01],[0.,0.,-math.pi+.01]],[[0.,0.,math.pi-.01],[0.,0.,math.pi-.01]],[1.,2.])
        self.assertAlmostEqual(result['heading_rmse_deg'],math.degrees(.02))

    def test_failed_frames_remain_in_error_metrics(self):
        from experiments.localization_comparison.common.score import metrics
        result=metrics([[0.,0.,0.],[10.,0.,0.]],[[0.,0.,0.],[0.,0.,0.]],[1.,2.])
        self.assertAlmostEqual(result['fraction_over_025m_or_5deg'],.5)
        self.assertGreater(result['position_rmse_m'],7.)


if __name__=='__main__':unittest.main()
