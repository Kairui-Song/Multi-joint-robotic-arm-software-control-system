#!/usr/bin/env python3
"""Measure actual control periods and ROS feedback transport separately; no motion."""
import argparse
import csv
import json
import math
import statistics
import sys
import time
from pathlib import Path


def summary(values):
    if not values:
        return {'samples': 0}
    ordered = sorted(values)
    def percentile(p):
        return ordered[min(len(ordered) - 1, math.ceil(p * len(ordered)) - 1)]
    return {'samples': len(values), 'min': ordered[0], 'mean': statistics.fmean(values),
            'p50': percentile(.5), 'p95': percentile(.95), 'p99': percentile(.99), 'max': ordered[-1]}


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.utilities import remove_ros_args
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', default='latency_report')
    parser.add_argument('--max-p99-jitter-us', type=float, default=500)
    args = parser.parse_args(remove_ros_args()[1:])
    if not math.isfinite(args.seconds) or args.seconds <= 0 or not math.isfinite(args.max_p99_jitter_us) or args.max_p99_jitter_us < 0:
        parser.error('Invalid duration or jitter threshold')
    rclpy.init(); node = Node('arm_latency_probe')
    rows, arrivals, ages, faults, drops = [], [], [], [], []
    previous = [None]
    last_seen = {'timing': None, 'states': None, 'diagnostics': None}
    def timing(msg):
        last_seen['timing'] = time.monotonic()
        if len(msg.data) % 5:
            faults.append('Malformed timing packet'); return
        rows.extend(tuple(msg.data[i:i + 5]) for i in range(0, len(msg.data), 5))
    def states(msg):
        last_seen['states'] = time.monotonic()
        now = time.monotonic_ns()
        if previous[0] is not None:
            arrivals.append((now - previous[0]) / 1000)
        previous[0] = now
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        ages.append((node.get_clock().now().nanoseconds - stamp) / 1000)
    def diagnostics(msg):
        for status in msg.status:
            if status.name == 'arm_control/hardware':
                last_seen['diagnostics'] = time.monotonic()
                if status.level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE):
                    faults.append(status.message)
                values = dict((v.key, v.value) for v in status.values)
                drops.append(int(values.get('timing_dropped', '0')))
    node.create_subscription(Float64MultiArray, 'arm_hardware/timing', timing, 10)
    node.create_subscription(JointState, 'joint_states', states, qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray, 'diagnostics', diagnostics, 10)
    try:
        # DDS discovery is not part of the measurement window. Require actual,
        # recent messages from all three streams before starting the full test.
        startup_deadline = time.monotonic() + 10.0
        ready = False
        while time.monotonic() < startup_deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if all(stamp is not None and now - stamp < 1.0 for stamp in last_seen.values()):
                ready = True
                break
        if ready:
            rows.clear(); arrivals.clear(); ages.clear(); drops.clear()
            previous[0] = None
        else:
            faults.append('Timed out waiting for fresh timing, joint_states and hardware diagnostics')
        deadline = time.monotonic() + (args.seconds if ready else 0.0)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.1, max(0., deadline - time.monotonic())))
    finally:
        node.destroy_node(); rclpy.shutdown()
    jitter = [abs(row[1] - row[4]) for row in rows]
    finished = time.monotonic()
    stale_streams = [key for key, stamp in last_seen.items() if stamp is None or finished - stamp > 1.0]
    covered_s = sum(row[1] for row in rows) * 1e-6
    gaps = sum(max(0, int(b[0] - a[0] - 1)) for a, b in zip(rows, rows[1:]))
    resets = sum(b[0] <= a[0] for a, b in zip(rows, rows[1:]))
    report = {'units': 'microseconds', 'duration_s': args.seconds,
              'control_period': summary([r[1] for r in rows]), 'control_abs_jitter': summary(jitter),
              'hardware_read': summary([r[2] for r in rows]), 'hardware_write': summary([r[3] for r in rows]),
              'joint_state_arrival_interval': summary(arrivals), 'joint_state_stamp_age': summary(ages),
              'missing_control_samples': gaps, 'sequence_resets': resets,
              'stale_streams': stale_streams, 'control_sample_coverage_s': covered_s,
              'timing_ring_drops_during_test': max(drops) - min(drops) if drops else None,
              'faults': sorted(set(faults)),
              'scope': 'Host control cycle and DDS observation; not physical command-to-motion or wire latency'}
    passed = (len(rows) >= 10 and len(ages) >= 2 and bool(drops) and not faults and not gaps and not resets
              and not stale_streams and covered_s >= 0.8 * args.seconds
              and max(drops) == min(drops) and min(ages) >= 0
              and report['control_abs_jitter']['p99'] <= args.max_p99_jitter_us)
    report['passed'] = passed
    output = Path(args.output); output.parent.mkdir(parents=True, exist_ok=True)
    output.with_suffix('.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    with output.with_suffix('.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['cycle', 'period_us', 'read_us', 'write_us', 'expected_period_us']); writer.writerows(rows)
    print(json.dumps(report, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
