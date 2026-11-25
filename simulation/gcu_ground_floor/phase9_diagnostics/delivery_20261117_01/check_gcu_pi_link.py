#!/usr/bin/env python3
"""Bounded, no-motion, bidirectional ROS link probe for Phase 09.

Only namespaced String ping/echo messages are published. No TF, sensor, map,
action, goal or velocity topics are used. RTT uses one host's monotonic clock;
it is not one-way latency or a navigation performance measurement.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import statistics
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--role', choices=['pi', 'simulator'], required=True)
    p.add_argument('--session', required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if not re.fullmatch('[a-zA-Z0-9_]{1,32}', a.session):
        p.error('invalid session')
    if (os.environ.get('ROS_DOMAIN_ID') != '217' or
            os.environ.get('ROS_LOCALHOST_ONLY') != '0' or
            os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp'):
        p.error('requires the isolated Phase 09 link profile (domain 217)')
    a.output.parent.mkdir(parents=True, exist_ok=True)
    if a.output.exists():
        p.error('output already exists; use a new result path')
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('phase9_link_' + a.role)
    peer = 'simulator' if a.role == 'pi' else 'pi'
    prefix = '/edgeosmag/phase9_link/' + a.session + '/'
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    ping_pub = node.create_publisher(String, prefix + a.role + '/ping', qos)
    echo_pub = node.create_publisher(String, prefix + peer + '/echo', qos)
    sent, received, peer_received = {}, {}, set()

    def on_ping(message):
        try:
            item = json.loads(message.data)
            if (item['sender'] == peer and item['session'] == a.session and
                    isinstance(item['seq'], int) and 0 <= item['seq'] < 60 and
                    item['payload'] == 'x' * 2048):
                peer_received.add(item['seq'])
                echo_pub.publish(message)
        except (ValueError, KeyError, TypeError):
            pass

    def on_echo(message):
        try:
            item = json.loads(message.data)
            seq = item['seq']
            if (item['sender'] == a.role and item['session'] == a.session and
                    seq in sent and seq not in received and
                    item['sent_ns'] == sent[seq] and item['payload'] == 'x' * 2048):
                received[seq] = (time.monotonic_ns() - sent[seq]) / 1e6
        except (ValueError, KeyError, TypeError):
            pass

    subscriptions = [node.create_subscription(String, prefix + peer + '/ping', on_ping, qos),
                     node.create_subscription(String, prefix + a.role + '/echo', on_echo, qos)]
    began = time.monotonic()
    ready_at = None
    failure = None
    try:
        while time.monotonic() - began < 15:
            rclpy.spin_once(node, timeout_sec=.02)
            if ping_pub.get_subscription_count() and echo_pub.get_subscription_count():
                ready_at = time.monotonic()
                break
        if ready_at is None:
            raise RuntimeError('peer_discovery_timeout')
        while len(sent) < 60:
            now = time.monotonic()
            if now >= ready_at + len(sent) / 10:
                seq = len(sent)
                sent[seq] = time.monotonic_ns()
                ping_pub.publish(String(data=json.dumps(dict(sender=a.role,
                    session=a.session, seq=seq, sent_ns=sent[seq], payload='x' * 2048))))
            rclpy.spin_once(node, timeout_sec=.01)
            if time.monotonic() - began > 25:
                raise RuntimeError('probe_wall_timeout')
        # Keep echoing after our own stream finishes so the other host can finish.
        until = time.monotonic() + 5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)
    except Exception as error:
        failure = repr(error)
    finally:
        durations = sorted(received.values())
        report = dict(scope='Bidirectional best-effort String connectivity probe; no motion; not a navigation benchmark',
            role=a.role, session=a.session, domain=217, requested_messages=60,
            requested_rate_hz=10, payload_characters=2048, sent=len(sent),
            echoed=len(received), peer_messages_received=len(peer_received),
            missing_echo_sequences=sorted(set(sent) - set(received)),
            rtt_ms_by_sequence=received, error=failure,
            source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            discovery_wall_s=None if ready_at is None else ready_at - began,
            elapsed_wall_s=time.monotonic() - began,
            rtt_p50_ms=statistics.median(durations) if durations else None,
            rtt_p95_ms=durations[max(0, int(.95 * len(durations)) - 1)] if durations else None,
            passed=failure is None and len(sent) == len(received) == len(peer_received) == 60)
        a.output.write_text(json.dumps(report, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
