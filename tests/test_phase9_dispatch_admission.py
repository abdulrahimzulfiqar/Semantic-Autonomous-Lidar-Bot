from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from osmag_mission_contract import MissionError
from experiments.pi_baseline.dispatch_admission import begin_when_fresh


class DispatchAdmissionTests(unittest.TestCase):
    def adapter(self):
        return SimpleNamespace(model=SimpleNamespace(state='idle',contract=None,attempts={}),handles={})

    def test_idle_stale_tf_can_wait_without_issuing_duplicate_action(self):
        adapter=self.adapter();calls=[];steps=[]
        def begin(*_):
            calls.append(1)
            if len(calls)==1:raise MissionError('stale_robot_tf')
            adapter.model.state='dispatching';adapter.model.attempts['one']='pending'
        adapter.begin=begin
        result=begin_when_fresh(adapter,object(),lambda:True,lambda:steps.append(1))
        self.assertEqual(result['attempts'],2);self.assertEqual(steps,[1])
        self.assertEqual(len(adapter.model.attempts),1)

    def test_idle_stale_input_requires_new_observation_before_dispatch(self):
        for reason in ['stale_scan', 'constraint_costmaps_not_ready']:
            with self.subTest(reason=reason):
                adapter = self.adapter()
                fresh = False
                def update():
                    nonlocal fresh
                    fresh = True
                def begin(*_):
                    if not fresh:
                        raise MissionError(reason)
                    adapter.model.state = 'dispatching'
                adapter.begin = begin
                result = begin_when_fresh(adapter, object(), lambda: True, update)
                self.assertEqual(result['attempts'], 2)

    def test_state_change_or_other_failure_cannot_be_retried(self):
        for field,value in [('state','dispatching'),('contract',object()),('attempts',{'one':'pending'})]:
            adapter=self.adapter();setattr(adapter.model,field,value)
            def fail(*_):raise MissionError('stale_robot_tf')
            adapter.begin=fail
            with self.assertRaisesRegex(MissionError,'stale_robot_tf'):
                begin_when_fresh(adapter,object(),lambda:True,lambda:self.fail('must not retry'))
        adapter=self.adapter()
        def fail_other(*_):raise MissionError('navigator_ownership_changed')
        adapter.begin=fail_other
        with self.assertRaisesRegex(MissionError,'navigator_ownership_changed'):
            begin_when_fresh(adapter,object(),lambda:True,lambda:self.fail('must not retry'))

    def test_no_fresh_tf_still_times_out(self):
        adapter=self.adapter();tick=[0.]
        def fail(*_):raise MissionError('stale_robot_tf')
        def step():tick[0]+=1.
        adapter.begin=fail
        with self.assertRaisesRegex(MissionError,'stale_robot_tf'):
            begin_when_fresh(adapter,object(),lambda:True,step,timeout_s=2.,wall=lambda:tick[0])
        self.assertEqual(tick[0],2.)
