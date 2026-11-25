import unittest
from experiments.pi_baseline.analyze_callbacks import correlate


class CallbackAnalysisTests(unittest.TestCase):
    def test_independent_progress_is_preserved_during_actor_gap(self):
        trace=dict(rows=dict(actor_clock=[dict(begin_wall=10.,end_wall=10.001,stamp=1.),
            dict(begin_wall=11.,end_wall=11.001,stamp=1.2)],
            executor_timer=[dict(begin_wall=10.1,end_wall=10.9)]))
        probe=dict(rows=dict(clock=[dict(wall=10.+i*.1,stamp=1.+i*.02) for i in range(11)]))
        result=correlate(trace,probe,10.,11.1)
        gap=result['largest_actor_clock_callback_gap']
        self.assertAlmostEqual(gap['duration_wall_s'],1.)
        self.assertEqual(gap['independent_clock_callbacks_during_gap'],9)
        self.assertAlmostEqual(result['durations']['executor_timer']['max_wall_s'],.8)

    def test_no_independent_progress_and_negative_offset_are_not_hidden(self):
        trace=dict(rows=dict(actor_clock=[dict(begin_wall=10.,end_wall=10.001,stamp=1.),
            dict(begin_wall=11.,end_wall=11.001,stamp=1.2)]))
        probe=dict(rows=dict(clock=[dict(wall=10.01,stamp=1.),dict(wall=11.01,stamp=1.2)]))
        result=correlate(trace,probe,10.02,11.1)
        self.assertIsNone(result['largest_actor_clock_callback_gap'])
        self.assertAlmostEqual(result['matching_clock_offsets']['minimum_wall_s'],-.01)
