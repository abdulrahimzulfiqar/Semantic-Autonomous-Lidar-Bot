import struct
import unittest

from experiments.pi_baseline.analyze_pipeline import compare_hosts, compare_pi
from experiments.pi_baseline.arrival_probe import raw_header_stamp
from experiments.pi_baseline.sensor_link_probe import with_stamp


class PipelineTraceTests(unittest.TestCase):
    def test_reads_header_stamp_in_both_cdr_byte_orders(self):
        for order, encoding in [('<', b'\x00\x01'), ('>', b'\x00\x00')]:
            payload = encoding+b'\x00\x00'+struct.pack(order+'iI', 27, 123)
            self.assertEqual(raw_header_stamp(payload), 27_000_000_123)
        with self.assertRaisesRegex(ValueError, 'unsupported_stamp_encoding'):
            raw_header_stamp(b'\x00\x07'+b'\x00'*20)

    def test_stamp_matches_do_not_compare_wall_times_across_hosts(self):
        source = [dict(stamp_ns=stamp, wall=1_200+stamp/100) for stamp in [10, 20, 30]]
        pi = [dict(stamp_ns=stamp, wall=100+stamp/100) for stamp in [10, 30, 30]]
        output = compare_hosts(source, pi, fault_wall=100.2)
        self.assertEqual(output['missing_on_pi_ns'], [20])
        self.assertEqual(output['matched_after_pi_fault_ns'], [30])
        self.assertEqual(output['matched_stamps'], 2)

    def test_sensor_load_replay_preserves_payload_outside_stamp(self):
        for order, encoding in [('<', b'\x00\x01'), ('>', b'\x00\x00')]:
            payload = encoding+b'\x00\x00'+struct.pack(order+'iI', 27, 123)+bytes(range(32))
            moved = with_stamp(payload, 56_000_000_789)
            self.assertEqual(raw_header_stamp(moved), 56_000_000_789)
            self.assertEqual(moved[:4], payload[:4])
            self.assertEqual(moved[12:], payload[12:])

    def test_local_producer_and_actor_use_separate_time_bases(self):
        probe = [dict(stamp_ns=10, wall=20., unix_ns=1_000_012, dds_source_ns=1_000_000),
                 dict(stamp_ns=20, wall=21., unix_ns=1_100_000, dds_source_ns=0)]
        actor = [dict(stamp_ns=10, wall=20.25), dict(stamp_ns=20, wall=21.)]
        output = compare_pi(probe, actor, fault_wall=20.1)
        self.assertEqual(output['native_source_to_probe_samples'], 1)
        self.assertEqual(output['native_source_to_probe_max_ns'], 12)
        self.assertEqual(output['latest_probe_stamp_before_fault_ns'], 10)
        self.assertIsNone(output['latest_actor_stamp_before_fault_ns'])
        self.assertEqual(output['probe_to_actor_max_s'], .25)
