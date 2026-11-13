import json,sys,tempfile,unittest,hashlib
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from gcu_mission_fixture import load_fixture,verify_runtime_sources,validate_current_contract
from types import SimpleNamespace
import numpy as np


class FixtureAdmissionTest(unittest.TestCase):
    def test_consecutive_fixture_preserves_map_frame_and_checks_next_deadline(self):
        first=self.fixture();later=self.fixture();later.update(name='return',spawn=first['goal'],goal=first['spawn'])
        first['next_mission']=later;self.read(first)
        later['checker_wall_timeout_s']=9999
        with self.assertRaisesRegex(ValueError,'next_deadlines'):self.read(first)
        later['checker_wall_timeout_s']=420;later['input_sha256']={'other':'map'}
        with self.assertRaisesRegex(ValueError,'map_or_frame'):self.read(first)
    def test_relocalized_route_is_rechecked_against_padded_mask(self):
        grid=np.zeros((20,20),dtype=np.int16);grid[:,10]=100
        snapshot=SimpleNamespace(combined=grid,resolution=.1,origin=(0.,0.),radius=.18,padding=.02)
        safe=SimpleNamespace(path=[(.5,.5),(.5,1.5)])
        self.assertTrue(validate_current_contract(safe,snapshot)['valid'])
        # Both starts have a free center cell, but the shifted route's padded
        # footprint intersects the frozen logical barrier.
        shifted=SimpleNamespace(path=[(.85,.5),(.85,1.5)])
        with self.assertRaisesRegex(ValueError,'current_contract_padding'):
            validate_current_contract(shifted,snapshot)

    def test_stale_executor_copy_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'executor.py';p.write_text('accepted')
            fixture={'execution_source_sha256':{'executor.py':hashlib.sha256(p.read_bytes()).hexdigest()}}
            verify_runtime_sources(fixture,directory);p.write_text('old staged copy')
            with self.assertRaisesRegex(ValueError,'execution_source_changed'):verify_runtime_sources(fixture,directory)

    def fixture(self):
        return dict(name='door',spawn=[1,2,3.14159],goal=[0,2,3.14159],expected_passages=['p'],expected_areas=['a','b'],mask_validation={'valid':True},simulation_timeout_s=80,checker_wall_timeout_s=420,input_sha256={'map':'abc'})

    def read(self,data):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'fixture.json';p.write_text(json.dumps(data));return load_fixture(p)

    def test_valid_signed_headings_and_content_digest(self):
        value,digest=self.read(self.fixture());self.assertGreater(value['goal'][2],3);self.assertEqual(len(digest),64)

    def test_failed_padding_check_and_changed_digest_are_not_admitted(self):
        data=self.fixture();first=self.read(data)[1];data['goal'][0]=-.1;self.assertNotEqual(self.read(data)[1],first)
        data['mask_validation']['valid']=False
        with self.assertRaisesRegex(ValueError,'padding'):self.read(data)

    def test_nonfinite_pose_wrong_topology_and_unbounded_deadline_rejected(self):
        for field,value in [('goal',[float('nan'),0,0]),('expected_areas',['a']),('checker_wall_timeout_s',9999),('local_costmap_frame','base_link'),('amcl_sigma_hit',float('nan'))]:
            data=self.fixture();data[field]=value
            with self.assertRaises(ValueError):self.read(data)
