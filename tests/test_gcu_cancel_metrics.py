"""Cancellation cannot pass from an ACK, initial rest, or later resumed motion."""
import copy,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from gcu_cancel_metrics import score_cancellation


def evidence():
    truth=[{'sim_time':i*.05,'truth':[min(i*.005,.51),0.,0.]} for i in range(151)]
    return {'cancel_request':{'wall':100.,'sim':5.,'pose':[.5,0.,0.],'segment_distance_m':.42},
        'contract':{'segments':[{'point':[.92,0.]}]},'truth':truth,'localization':copy.deepcopy(truth),
        'commands':[{'sim_time':i*.05,'v':.1 if i<=100 else 0.,'w':0.} for i in range(151)],
        'state':{'state':'cancelled','stop_confirmed':True,'unresolved_requests':0},
        'events':[{'event':'send','wall':90.,'token':'1:0:0'},
            {'event':'accepted','wall':91.,'accepted':True,'token':'1:0:0','uuid':'a'},
            {'event':'fault','wall':100.01,'reason':'user_cancel'},
            {'event':'cancel_request','wall':100.01,'uuid':'a'},
            {'event':'result','wall':100.1,'uuid':'a','outcome':'cancelled'}]}


class CancelMetricsTest(unittest.TestCase):
    def test_moving_cancel_with_native_terminal_and_independent_rest(self):
        self.assertLessEqual(score_cancellation(evidence())['cancel_stop_delay_sim_s'],1.)

    def test_native_ack_does_not_replace_physical_stop(self):
        r=evidence()
        for row in r['truth']:row['truth'][0]=row['sim_time']*.1
        with self.assertRaisesRegex(ValueError,'cancel_stop'):score_cancellation(r)

    def test_late_segment_send_is_rejected(self):
        r=evidence();r['events'].append({'event':'send','wall':100.02,'token':'1:1:1'})
        with self.assertRaisesRegex(ValueError,'segment_advanced'):score_cancellation(r)

    def test_initial_stationary_robot_cannot_count_as_interrupted_motion(self):
        r=evidence()
        for row in r['truth']:row['truth'][0]=.5
        with self.assertRaisesRegex(ValueError,'meaningful_motion'):score_cancellation(r)

    def test_resumed_motion_after_verified_rest_is_rejected(self):
        r=evidence()
        for row in r['truth']:
            if row['sim_time']>=7.:row['truth'][0]+=.1*(row['sim_time']-7.)
        with self.assertRaisesRegex(ValueError,'motion_resumed'):score_cancellation(r)

    def test_cancelled_state_without_native_terminal_is_rejected(self):
        r=evidence();r['events']=r['events'][:-1]
        with self.assertRaisesRegex(ValueError,'native_cancel_not_confirmed'):score_cancellation(r)
