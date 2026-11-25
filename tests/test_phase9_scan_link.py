import unittest
from experiments.pi_baseline.analyze_scan_link import compare


class ScanLinkTests(unittest.TestCase):
    def test_loss_and_changed_content_fail_without_relabelling_stamps(self):
        sender=dict(rows=[dict(stamp_ns=1,serialized_sha256='a'),dict(stamp_ns=2,serialized_sha256='b')])
        self.assertTrue(compare(sender,sender,1,2)['passed'])
        missing=dict(rows=sender['rows'][:1]);self.assertFalse(compare(sender,missing,1,2)['passed'])
        changed=dict(rows=[sender['rows'][0],dict(stamp_ns=2,serialized_sha256='c')])
        result=compare(sender,changed,1,2);self.assertFalse(result['passed']);self.assertEqual(result['changed_stamps_ns'],[2])

    def test_empty_interval_cannot_pass_and_startup_outside_interval_is_explicit(self):
        sender=dict(rows=[dict(stamp_ns=1,serialized_sha256='a'),dict(stamp_ns=2,serialized_sha256='b')])
        receiver=dict(rows=sender['rows'][1:])
        self.assertFalse(compare(sender,receiver,3,4)['passed'])
        result=compare(sender,receiver,2,2)
        self.assertTrue(result['passed']);self.assertEqual(result['source_unique_scans'],1)
