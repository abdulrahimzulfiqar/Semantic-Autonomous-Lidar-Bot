import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from gcu_navigation_metrics import rest_window,coverage


def trace(poses):return [{'sim_time':i*.05,'truth':p} for i,p in enumerate(poses)]


class NavigationMetricsTest(unittest.TestCase):
    def test_rotation_is_not_rest(self):
        self.assertIsNone(rest_window(trace([(0,0,i*.02) for i in range(60)])))

    def test_one_slow_sample_does_not_establish_cancelled_stop(self):
        poses=[(min(i,20)*.005 if i<23 else .1+(i-23)*.005,0,0) for i in range(60)]
        self.assertIsNone(rest_window(trace(poses),earliest=1.))

    def test_measured_stop_requires_continuous_window(self):
        result=rest_window(trace([(min(i,20)*.005,0,0) for i in range(65)]),earliest=.5)
        self.assertIsNotNone(result)
        self.assertGreaterEqual(result['start_sim'],1.)
        self.assertGreaterEqual(result['confirmed_sim']-result['start_sim'],1.)

    def test_missing_observations_cannot_certify_rest(self):
        samples=[{'sim_time':t,'truth':[0,0,0]} for t in [0,.1,.2,1.5,1.6,1.7]]
        self.assertIsNone(rest_window(samples))
        self.assertGreater(coverage(samples,samples)['max_truth_gap_sim_s'],1.)


if __name__=='__main__':unittest.main()
