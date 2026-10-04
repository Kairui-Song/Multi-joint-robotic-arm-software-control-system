#!/usr/bin/env python3
"""Observe an already active Linglong MOCK: 30 min trajectories + 5 min idle."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from datetime import datetime

import rclpy
from control_msgs.msg import DynamicJointState
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from linglong_control_tools.health import HealthMonitor
from linglong_control_tools.interfaces import DYNAMIC_STATES


def main():
    out = Path.home() / 'linglong_1025_2/ros2_ws/verification' / (
        'fifo35min_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True)
    report = {'passed': False, 'motion_required_s': 1800, 'idle_required_s': 300,
              'rounds_passed': 0, 'domain': os.getenv('ROS_DOMAIN_ID', ''),
              'scope': 'Linglong MOCK actions and observed health; not hard-real-time certification'}
    def save():
        (out / 'result.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    save()
    print('Results:', out, flush=True)
    # Record actual scheduling, not just the caller's ulimit.
    ids = subprocess.run(['pgrep', '-f', '^/opt/ros/jazzy/lib/controller_manager/ros2_control_node'],
                         capture_output=True, text=True).stdout.split()
    threads = ''.join(subprocess.run(['ps', '-L', '-p', pid, '-o', 'pid,tid,cls,rtprio,comm'],
                                    capture_output=True, text=True).stdout for pid in ids)
    (out / 'threads_before.txt').write_text(threads, encoding='utf-8')
    if len(ids) != 1 or not any('FF' in line.split() and '50' in line.split() for line in threads.splitlines()):
        report['failure'] = 'Require one control manager and an FF/50 thread; inspect threads_before.txt'
        save(); print(report['failure']); return 1
    rclpy.init()
    node = rclpy.create_node('linglong_timed_validation')
    monitor = HealthMonitor()
    phase = 'startup'
    fatal = []
    armed = False
    process = None
    child_log = None
    trace = (out / 'telemetry.jsonl').open('w', encoding='utf-8', buffering=1)
    def emit(kind, data):
        trace.write(json.dumps({'kind': kind, 'phase': phase, 'wall': time.time(),
                                'monotonic': time.monotonic(), 'data': data}) + '\n')
    def states(msg):
        now = time.monotonic()
        monitor.receive(msg.joint_names,
                        [(v.interface_names, v.values) for v in msg.interface_values], now)
        level, message = monitor.status(now)
        emit('health', {'level': level, 'message': message,
                       'health': monitor.snapshot['control_health'] if monitor.snapshot else None})
        if armed and monitor.snapshot:
            h = monitor.snapshot['control_health']
            if level >= 2 or h['mock'] != 1 or h['active'] != 1 or h['feedback_age_seconds'] != 0:
                fatal.append(message)
    def diagnostic(msg):
        emit('diagnostics', [{'name': s.name,
              'level': int.from_bytes(s.level, 'little') if isinstance(s.level, bytes) else int(s.level),
              'message': s.message, 'values': {v.key: v.value for v in s.values}}
              for s in msg.status])
    node.create_subscription(DynamicJointState, DYNAMIC_STATES, states,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(DiagnosticArray, '/diagnostics', diagnostic, 100)
    def check():
        level, message = monitor.status(time.monotonic())
        if fatal or level >= 2 or monitor.snapshot is None:
            raise RuntimeError(fatal[0] if fatal else message)
        h = monitor.snapshot['control_health']
        if h['active'] != 1 or h['mock'] != 1 or h['feedback_age_seconds'] != 0:
            raise RuntimeError('Fresh active MOCK state required')
    try:
        deadline = time.monotonic() + 15
        while monitor.snapshot is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        check(); armed = True
        phase = 'motion'
        started = time.monotonic()
        report['motion_started_monotonic'] = started
        while time.monotonic() - started < 1800:
            check()
            number = report['rounds_passed'] + 1
            path = out / f'motion_{number:05d}.log'
            child_log = path.open('w', encoding='utf-8')
            process = subprocess.Popen(['ros2', 'run', 'linglong_control', 'trajectory_demo'],
                stdout=child_log, stderr=subprocess.STDOUT, start_new_session=True)
            end = time.monotonic() + 45
            while process.poll() is None:
                rclpy.spin_once(node, timeout_sec=.05)
                check()
                if time.monotonic() > end:
                    raise TimeoutError(f'Round {number}: action client exceeded 45 seconds')
            child_log.close(); child_log = None
            if process.returncode != 0:
                raise RuntimeError(f'Round {number}: client exit={process.returncode}; see {path.name}')
            payloads = [json.loads(line) for line in path.read_text().splitlines() if line.startswith('{')]
            if not any(p.get('scenario') == 'trajectory' and p.get('passed') is True for p in payloads):
                raise RuntimeError(f'Round {number}: success evidence missing')
            report['rounds_passed'] = number
            report['motion_elapsed_s'] = time.monotonic() - started
            save()
            print(f'PASS {number}; motion {report["motion_elapsed_s"]:.1f}/1800 s', flush=True)
        phase = 'idle'
        idle_start = time.monotonic()
        print('Motion phase complete; observing 300 seconds without new goals', flush=True)
        while time.monotonic() - idle_start < 300:
            rclpy.spin_once(node, timeout_sec=.05)
            check()
            report['idle_elapsed_s'] = time.monotonic() - idle_start
        report['passed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['failure'] = str(exc) or 'Interrupted'
        report['failed_phase'] = phase
        print('STOP:', report['failure'], flush=True)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
            report['interrupted_client'] = True
            report['motion_completion_unknown'] = True
        if child_log:
            child_log.close()
        phase = 'post_test'
        try:
            end = time.monotonic() + 2
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
            if report['passed']:
                check()
        except Exception as exc:
            report['passed'] = False
            report['post_test_failure'] = str(exc)
        report['fatal_observations'] = fatal
        save(); trace.close()
        node.destroy_node(); rclpy.shutdown()
    print(json.dumps(report, indent=2))
    print('Results:', out)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
