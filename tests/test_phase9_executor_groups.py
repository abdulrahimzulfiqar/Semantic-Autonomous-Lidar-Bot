from types import SimpleNamespace
import unittest
from experiments.pi_baseline.executor_groups import separate_mission_timer


class ExecutorGroupTests(unittest.TestCase):
    def test_clock_callback_and_period_are_preserved_in_new_group(self):
        callback=lambda:None;clock=object();group=object()
        old=SimpleNamespace(timer_period_ns=40000000,callback=callback,clock=clock)
        seen=[];new=object()
        def create(period,handler,**options):seen.append((period,handler,options));return new
        adapter=SimpleNamespace(timer=old,destroy_timer=lambda timer:seen.append(timer),create_timer=create)
        result=separate_mission_timer(adapter,lambda:group)
        self.assertIs(seen[0],old);self.assertIs(adapter.timer,new)
        self.assertEqual(seen[1][0],.04);self.assertIs(seen[1][1],callback)
        self.assertIs(seen[1][2]['clock'],clock);self.assertIs(seen[1][2]['callback_group'],group)
        self.assertTrue(result['callback_and_clock_preserved'])

    def test_unexpected_timer_is_rejected_before_destroy(self):
        adapter=SimpleNamespace(timer=SimpleNamespace(timer_period_ns=20000000,callback=lambda:None))
        with self.assertRaisesRegex(RuntimeError,'unexpected_mission_timer'):
            separate_mission_timer(adapter,lambda:object())
