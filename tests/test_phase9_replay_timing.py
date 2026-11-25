import math
import unittest
import numpy as np
from experiments.pi_baseline.replay_timing import interpolated_pose, timing_summary, validate_recording,clock_path_summary


class ReplayTimingTests(unittest.TestCase):
    def test_interpolation_crosses_angle_wrap_without_extrapolating(self):
        value=interpolated_pose(np.array([1.,2.]),np.array([[0.,0.,math.radians(179)],[2.,4.,math.radians(-179)]]),1.5)
        self.assertAlmostEqual(value[0],1.);self.assertAlmostEqual(value[1],2.)
        self.assertAlmostEqual(abs(value[2]),math.pi)
        for at in [.99,2.01,float('nan')]:
            with self.assertRaisesRegex(ValueError,'replay_extrapolation'):
                interpolated_pose(np.array([1.,2.]),np.zeros((2,3)),at)

    def test_recording_cannot_silently_accept_time_reversal_or_moving_extrinsic(self):
        data=dict(stamps=np.array([1.,1.1]),odom=np.zeros((2,3)),extrinsics=np.zeros((2,3)),
            ranges=np.ones((2,3)),angles=np.array([0.,.1,.2]),initial_pose=np.zeros(3))
        validate_recording(data)
        with self.assertRaises(ValueError):validate_recording({**data,'stamps':np.array([1.,1.])})
        with self.assertRaises(ValueError):validate_recording({**data,'extrinsics':np.array([[0.,0.,0.],[.1,0.,0.]])})

    def fixture(self):
        feed=dict(begin_wall=10.,end_wall=10.2,rate=1.,duration_sim_s=.2,scan_stamps_ns=[1000000000,1100000000],
            scan_publications=[dict(late_wall_s=.001),dict(late_wall_s=.002)])
        obs=dict(tf=[dict(wall=10.+i*.04,clock=1.+i*.04,stamp=1.+i*.04-.01) for i in range(5)],
            scans=[dict(wall=10.01,stamp_ns=1000000000),dict(wall=10.11,stamp_ns=1100000000)],
            clocks=[dict(wall=10.+i*.01,stamp=1.+i*.01) for i in range(20)])
        return feed,obs

    def test_missing_scan_tf_query_and_pacing_failures_are_not_ignored(self):
        feed,obs=self.fixture();self.assertTrue(timing_summary(feed,obs)['passed'])
        for changed in [{**obs,'scans':obs['scans'][:1]},
            {**obs,'tf':obs['tf']+[dict(wall=10.15,clock=1.15,error='missing')]},
            {**obs,'tf':[dict(wall=10.,clock=1.,stamp=.8)]}]:
            with self.subTest(changed=changed):self.assertFalse(timing_summary(feed,changed)['passed'])
        late={**feed,'scan_publications':[dict(late_wall_s=.101)]}
        self.assertFalse(timing_summary(late,obs)['passed'])
        self.assertFalse(timing_summary({**feed,'end_wall':10.4},obs)['passed'])

    def test_gap_threshold_rejects_actual_observation_gap(self):
        feed,obs=self.fixture()
        obs['tf']=[dict(wall=10.,clock=1.,stamp=1.),dict(wall=10.16,clock=1.16,stamp=1.157)]
        result=timing_summary(feed,obs)
        self.assertFalse(result['passed']);self.assertAlmostEqual(result['max_pose_observation_gap_sim_s'],.157)

    def test_frozen_clock_cannot_pass_using_fresh_historical_tf(self):
        feed,obs=self.fixture()
        obs['clocks']=[dict(wall=10.+i*.01,stamp=.5) for i in range(20)]
        result=timing_summary(feed,obs)
        self.assertFalse(result['passed']);self.assertFalse(result['clock_covers_recording'])

    def test_clock_trace_distinguishes_continuing_source_from_source_pause(self):
        feed,obs=self.fixture()
        feed['clock_publications']=[dict(wall=10.+i*.01,stamp_ns=round((1.+i*.01)*1e9)) for i in range(20)]
        obs['clocks']=[obs['clocks'][0],obs['clocks'][-1]]
        trace=clock_path_summary(feed,obs)
        self.assertEqual(trace['source_publications_returned_during_receipt_gap'],18)
        feed['clock_publications']=[feed['clock_publications'][0],feed['clock_publications'][-1]]
        trace=clock_path_summary(feed,obs)
        self.assertEqual(trace['source_publications_returned_during_receipt_gap'],0)
