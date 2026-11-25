import unittest
from experiments.pi_baseline.clock_transport import ClockThrottle


class ClockTransportTests(unittest.TestCase):
    def test_only_real_samples_are_forwarded_and_bursts_are_coalesced(self):
        throttle=ClockThrottle(100.)
        self.assertIsNone(throttle.take(0.))
        throttle.receive((1,0));self.assertEqual(throttle.take(0.),(1,0))
        throttle.receive((1,1000000));self.assertIsNone(throttle.take(.005))
        throttle.receive((1,7000000));self.assertEqual(throttle.take(.01),(1,7000000))

    def test_paused_or_missing_source_never_advances_time(self):
        throttle=ClockThrottle(100.)
        throttle.receive((5,0));self.assertEqual(throttle.take(0.),(5,0))
        self.assertIsNone(throttle.take(10.))
        throttle.receive((5,0));self.assertIsNone(throttle.take(20.))

    def test_real_backward_jump_is_preserved(self):
        throttle=ClockThrottle(100.)
        throttle.receive((10,0));throttle.take(0.)
        throttle.receive((0,0));self.assertEqual(throttle.take(.02),(0,0))

    def test_invalid_rate_rejected(self):
        for rate in [0.,-1.,1001.,float('nan')]:
            with self.subTest(rate=rate),self.assertRaises(ValueError):ClockThrottle(rate)


if __name__=='__main__':unittest.main()
