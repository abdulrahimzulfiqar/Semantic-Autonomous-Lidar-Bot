import unittest
from experiments.pi_baseline.analyze_pilot_resources import process_costs


def row(at,cpu,pss,pid=1):
    return dict(wall_monotonic=at,processes=[dict(pid=pid,argv=['worker'],cpu_s=cpu,
        memory_kib=dict(Pss=pss,Rss=pss+5))])


class ResourceIntervalTests(unittest.TestCase):
    def test_cpu_uses_actual_in_interval_span_and_can_exceed_one_core(self):
        result=process_costs([row(0.,0.,999),row(1.,2.,10),row(3.,5.,20),row(4.,6.,999)],1.,3.)
        self.assertEqual(result['cpu_percent_one_core_total'],150.)
        self.assertEqual(result['sampled_concurrent_domain_pss_max_kib'],20)
        self.assertEqual(result['sample_span_wall_s'],2.)

    def test_process_churn_or_one_sample_cannot_produce_complete_cpu_total(self):
        result=process_costs([row(1.,2.,10),row(2.,.1,20,pid=2)],1.,2.)
        self.assertFalse(result['stable_process_set']);self.assertIsNone(result['cpu_percent_one_core_total'])
        result=process_costs([row(1.,2.,10)],1.,2.)
        self.assertIsNone(result['cpu_percent_one_core_total'])
        self.assertIsNone(result['processes'][0]['cpu_percent_one_core'])
        result=process_costs([dict(wall_monotonic=1.,processes=[]),dict(wall_monotonic=2.,processes=[])],1.,2.)
        self.assertIsNone(result['cpu_percent_one_core_total'])
