"""Explicit diagnostic separation of control timer from node data callbacks."""
import threading


def separate_mission_timer(adapter,group_factory):
    old=adapter.timer
    if old.timer_period_ns!=40000000 or not callable(old.callback):
        raise RuntimeError('unexpected_mission_timer')
    # Preserve the same callback (including its state lock), steady clock and
    # period. The separate group permits clock/sensor callbacks to progress
    # while control work occupies the timer group.
    callback=old.callback;clock=old.clock;group=group_factory()
    adapter.destroy_timer(old)
    adapter.timer=adapter.create_timer(.04,callback,callback_group=group,clock=clock)
    return dict(period_ns=40000000,callback_group='separate_mutually_exclusive',
        callback_and_clock_preserved=True)


class MissionTimerExecutor:
    """Schedule the existing mission callback independently of driver work."""

    def __init__(self, adapter):
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        old = adapter.timer
        if old.timer_period_ns != 40_000_000 or not callable(old.callback):
            raise RuntimeError('unexpected_mission_timer')
        self.node = Node('phase9_mission_timer')
        self.node.create_timer(.04, old.callback, clock=old.clock)
        adapter.destroy_timer(old)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, name='phase9-mission', daemon=True)
        self.thread.start()

    def close(self):
        self.executor.shutdown()
        self.thread.join()
        self.node.destroy_node()


class DedicatedTransformListener:
    """Service motion inputs on a dedicated ROS executor."""

    def __init__(self, adapter, trace=None, *, service_clock=False):
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu, LaserScan
        from tf2_ros import TransformListener

        adapter.listener.unregister()
        self.node = Node('phase9_tf_listener', parameter_overrides=[
            Parameter('use_sim_time', value=service_clock)])
        if service_clock:
            # Both TimeSources are configured once at startup. Move the
            # actor's ROSClock to the source served by the input executor.
            previous_source = adapter._time_source
            previous_source.detach_node()
            adapter.remove_on_set_parameters_callback(previous_source._on_parameter_event)
            input_source = self.node._time_source
            input_source.attach_clock(adapter.get_clock())
            if trace is not None:
                subscription = input_source._clock_sub
                subscription.callback = trace.wrap(
                    'actor_clock', subscription.callback, adapter.sim,
                    lambda message: {'stamp': message.clock.sec + message.clock.nanosec / 1e9})
        self.listener = TransformListener(adapter.buffer, self.node)
        if trace is not None:
            subscription = self.listener.tf_sub
            subscription.callback = trace.wrap(
                'tf_ingest', subscription.callback,
                fields=lambda message: {'stamps': [
                    [item.header.frame_id, item.child_frame_id,
                     item.header.stamp.sec + item.header.stamp.nanosec / 1e9]
                    for item in message.transforms
                    if item.child_frame_id in ('odom', 'base_footprint')
                ]})
        # Sensor freshness should reflect delivery of sensor data, not queueing
        # behind Python planning/window work. Preserve the existing callbacks,
        # their source stamps, topic names and QoS.
        for attribute, kind, topic in (
                ('scan_sub', LaserScan, '/scan'),
                ('imu_sub', Imu, adapter.imu_topic)):
            old = getattr(adapter, attribute)
            if old is not None:
                callback = old.callback
                adapter.destroy_subscription(old)
                setattr(adapter, attribute,
                        self.node.create_subscription(kind, topic, callback, qos_profile_sensor_data))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, name='phase9-tf', daemon=True)
        self.thread.start()

    def close(self):
        self.executor.shutdown()
        self.thread.join()
        self.listener.unregister()
        self.node.destroy_node()
