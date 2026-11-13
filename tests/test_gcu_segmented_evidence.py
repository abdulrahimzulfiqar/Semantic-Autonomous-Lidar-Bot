import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from verify_gcu_segmented_crossing import handoff_rest_report
from analyze_gcu_native_capture import connectivity
import numpy as np


class MultiPassageEvidenceTest(unittest.TestCase):
    def test_native_connectivity_does_not_treat_inscribed_or_unknown_as_free(self):
        data=np.zeros((9,9),dtype=np.uint8);data[:,4]=253
        self.assertFalse(connectivity(data,1.,(0.,0.),(2.5,4.5),(6.5,4.5))['connected'])
        data[4,4]=0
        self.assertTrue(connectivity(data,1.,(0.,0.),(2.5,4.5),(6.5,4.5))['connected'])
        data[4,6]=255
        self.assertFalse(connectivity(data,1.,(0.,0.),(2.5,4.5),(6.5,4.5))['connected'])
    def test_later_handoff_stop_is_not_hidden_by_clean_first_handoff(self):
        samples=[]
        for i in range(121):
            t=i*.05;x=.1*(min(t,2.8)+max(0.,t-4.4))
            samples.append({'sim_time':t,'truth':[x,0.,0.]})
        checks=handoff_rest_report(samples,[.5,3.])
        self.assertEqual(checks[0]['longest_rest_sim_s'],0.)
        self.assertGreater(checks[1]['longest_rest_sim_s'],1.)
