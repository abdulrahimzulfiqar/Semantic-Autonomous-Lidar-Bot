"""Prove disallowed action namespaces fail before ROS can even be imported."""
from pathlib import Path
import subprocess
import sys
import unittest


class MockScopeTests(unittest.TestCase):
    def test_live_and_nested_namespaces_are_rejected_before_ros_import(self):
        script=Path(__file__).resolve().parents[1]/'scripts/gcu_mission_mock_adapter.py'
        for namespace in ['/navigate_to_pose','/osmag','/gcu_phase6b1_test/navigate_to_pose']:
            with self.subTest(namespace=namespace):
                result=subprocess.run([sys.executable,str(script),'--assets','/unused',
                    '--mock-token','fixture','--namespace',namespace],capture_output=True,text=True)
                self.assertEqual(result.returncode,2)
                self.assertIn('isolated /gcu_phase6b1_NAME namespace',result.stderr)
                self.assertNotIn('ModuleNotFoundError',result.stderr)


if __name__=='__main__':unittest.main()
