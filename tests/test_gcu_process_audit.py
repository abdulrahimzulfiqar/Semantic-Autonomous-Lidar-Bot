import sys
from pathlib import Path
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from gcu_process_audit import audit_logs


class ChildAuditTests(unittest.TestCase):
    def audit(self, text):
        with tempfile.TemporaryDirectory() as folder:
            (Path(folder)/'simulator.log').write_text(text)
            return audit_logs(folder)

    def test_child_segfault_is_failure_even_when_launcher_succeeds(self):
        result=self.audit('[ERROR] [bridge]: process has died [pid 1, exit code -11]\nlaunch exited 0')
        self.assertEqual(len(result['critical']),1)

    def test_normal_sigint_messages_are_not_crashes(self):
        self.assertFalse(self.audit("sending signal 'SIGINT'\nprocess has finished cleanly")['critical'])

    def test_nonzero_child_exit_and_fatal_exception_are_retained(self):
        result=self.audit('process has died [pid 2, exit code 1]\nterminate called after exception')
        self.assertEqual(len(result['critical']),2)


if __name__=='__main__':unittest.main()
