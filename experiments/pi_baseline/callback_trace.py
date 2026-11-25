"""Optional bounded callback timing; delegates calls without changing decisions."""
from collections import defaultdict,deque
from functools import wraps
import threading
import time


class CallbackTrace:
    def __init__(self,limit=20000,wall=time.monotonic):
        if limit<1:raise ValueError('invalid_trace_limit')
        self.limit=limit;self.wall=wall;self.rows={};self.counts=defaultdict(int);self.lock=threading.Lock()

    def record(self,name,row):
        with self.lock:
            self.rows.setdefault(name,deque(maxlen=self.limit)).append(row);self.counts[name]+=1

    def wrap(self,name,callback,clock=None,fields=None):
        @wraps(callback)
        def measured(*args,**kwargs):
            row=dict(begin_wall=self.wall(),thread=threading.get_ident())
            try:
                if clock is not None:row['begin_ros']=clock()
                if fields is not None:row.update(fields(*args,**kwargs))
            except Exception as error:row['trace_metadata_error']=repr(error)
            try:return callback(*args,**kwargs)
            except BaseException as error:
                row['callback_error']=type(error).__name__;raise
            finally:
                row['end_wall']=self.wall()
                try:
                    if clock is not None:row['end_ros']=clock()
                except Exception:pass
                self.record(name,row)
        return measured

    def report(self):
        with self.lock:
            return dict(scope='Diagnostic call entry/return timestamps; includes lock waits and instrumentation overhead, not middleware arrival or isolated CPU time',
                maximum_rows_per_kind=self.limit,total_calls=dict(self.counts),
                rows={k:list(v) for k,v in self.rows.items()})


def attach_adapter(trace,adapter):
    # Verified against installed Humble Timer/Subscription.callback attributes.
    clock_subscription=adapter._time_source._clock_sub
    entities=[('executor_timer',adapter.timer,None),('actor_clock',clock_subscription,
        lambda m:dict(stamp=m.clock.sec+m.clock.nanosec/1e9)),
        ('actor_scan',adapter.scan_sub,lambda m:dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9)),
        ('actor_imu',adapter.imu_sub,lambda m:dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9))]
    if any(entity is None or not callable(entity.callback) for _,entity,_ in entities):
        raise RuntimeError('unsupported_humble_callback_trace')
    for name,entity,fields in entities:
        entity.callback=trace.wrap(name,entity.callback,adapter.sim,fields)
    for name in ['effects', 'accepted', 'result', 'healthy', 'pose']:
        setattr(adapter,name,trace.wrap('executor_'+name,getattr(adapter,name),adapter.sim))
    for name in ['count_publishers', 'count_subscribers']:
        setattr(adapter,name,trace.wrap('executor_'+name,getattr(adapter,name),adapter.sim,
            lambda topic_name: {'topic': topic_name}))
