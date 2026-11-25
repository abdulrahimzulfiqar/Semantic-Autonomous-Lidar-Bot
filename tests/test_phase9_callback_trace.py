import threading
import unittest
from experiments.pi_baseline.callback_trace import CallbackTrace


class CallbackTraceTests(unittest.TestCase):
    def test_return_arguments_and_exception_identity_are_preserved(self):
        trace=CallbackTrace()
        wrapped=trace.wrap('sum',lambda a,b=0:a+b,clock=lambda:12.)
        self.assertEqual(wrapped(3,b=4),7)
        failure=ValueError('original')
        def fail():raise failure
        with self.assertRaises(ValueError) as caught:trace.wrap('fail',fail)()
        self.assertIs(caught.exception,failure)
        rows=trace.report()['rows']
        self.assertEqual(rows['sum'][0]['begin_ros'],12.)
        self.assertEqual(rows['fail'][0]['callback_error'],'ValueError')

    def test_metadata_error_does_not_prevent_original_callback(self):
        trace=CallbackTrace();calls=[]
        def bad_metadata(*_):raise RuntimeError('trace only')
        result=trace.wrap('callback',lambda value:calls.append(value),fields=bad_metadata)(42)
        self.assertIsNone(result);self.assertEqual(calls,[42])
        self.assertIn('trace_metadata_error',trace.report()['rows']['callback'][0])

    def test_concurrent_recording_is_bounded_and_counts_dropped_history(self):
        trace=CallbackTrace(limit=5)
        def run():
            for i in range(25):trace.record('clock',dict(stamp=i))
        threads=[threading.Thread(target=run) for _ in range(4)]
        for thread in threads:thread.start()
        for thread in threads:thread.join()
        report=trace.report();self.assertEqual(report['total_calls']['clock'],100)
        self.assertEqual(len(report['rows']['clock']),5)
