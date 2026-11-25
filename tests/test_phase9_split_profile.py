import ast
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from experiments.pi_baseline.simulation_profile import simulation_guard
from experiments.pi_baseline.prepare import derive_executor,prepare


class SplitProfileTests(unittest.TestCase):
    def test_local_stack_replay_requires_loopback_and_logging_sink(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            dds = root/'dds.xml'
            dds.write_text('<CycloneDDS/>')
            path = root/'profile.json'
            profile = dict(scope='gcu_phase9_local_stack_replay', local_stack_replay=True,
                command_topic='/phase9/replay_commands', role='navigation_pi', domain=218,
                partition='gcu_phase9_test', dds_sha256=hashlib.sha256(dds.read_bytes()).hexdigest())
            path.write_text(json.dumps(profile))
            env = dict(ROS_DOMAIN_ID='218', ROS_LOCALHOST_ONLY='1', IGN_PARTITION='gcu_phase9_test',
                RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', CYCLONEDDS_URI='file://'+str(dds),
                EDGEOSMAG_PHASE9_PROFILE=str(path))
            simulation_guard(True, env)
            with self.assertRaisesRegex(ValueError,'wrong_network_scope'):
                simulation_guard(True, {**env,'ROS_LOCALHOST_ONLY':'0'})
            profile['command_topic'] = '/cmd_vel'
            path.write_text(json.dumps(profile))
            with self.assertRaisesRegex(ValueError,'profile_mismatch'):
                simulation_guard(True, env)

    def test_explicit_lan_profile_and_pinned_dds_are_required(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);dds=root/'dds.xml';dds.write_text('<CycloneDDS/>')
            profile=dict(scope='gcu_phase9_processor_in_loop',command_topic='/phase9/sim_cmd_vel',
                role='navigation_pi',domain=218,partition='gcu_phase9_test',
                pi_ip='192.168.1.2',simulator_ip='192.168.1.3',
                dds_sha256=hashlib.sha256(dds.read_bytes()).hexdigest())
            path=root/'profile.json';path.write_text(json.dumps(profile))
            env=dict(ROS_DOMAIN_ID='218',ROS_LOCALHOST_ONLY='0',IGN_PARTITION='gcu_phase9_test',
                RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',CYCLONEDDS_URI='file://'+str(dds),
                EDGEOSMAG_PHASE9_PROFILE=str(path))
            simulation_guard(True,env)
            for key in env:
                with self.subTest(key=key),self.assertRaises((ValueError,OSError)):
                    simulation_guard(True,{k:v for k,v in env.items() if k!=key})
            with self.assertRaises(ValueError):simulation_guard(False,env)
            dds.write_text('changed')
            with self.assertRaisesRegex(ValueError,'dds_profile_changed'):simulation_guard(True,env)

    def test_generated_executor_only_changes_the_guard(self):
        source=(Path(__file__).resolve().parents[1]/'scripts/gcu_segmented_executor.py').read_text()
        derived=derive_executor(source)
        self.assertIn('from profile_phase9 import simulation_guard',derived)
        original=ast.parse(source)
        self.assertEqual(len([n for n in original.body if isinstance(n,ast.FunctionDef) and n.name=='simulation_guard']),1)
        with self.assertRaisesRegex(ValueError,'unexpected_executor_guard'):derive_executor('x=1')

    def test_invalid_time_or_clock_rate_cannot_create_a_stage(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);destination=root/'stage'
            for factor,rate in [(0.,100.),(-1.,100.),(float('nan'),100.),(1.1,100.),(1.,-1.),(1.,float('nan'))]:
                with self.subTest(factor=factor,rate=rate),self.assertRaises(ValueError):
                    prepare(root/'missing',destination,root/'pi',root/'sim','192.168.1.2','192.168.1.3','test',rate,factor)
                self.assertFalse(destination.exists())


if __name__=='__main__':unittest.main()
