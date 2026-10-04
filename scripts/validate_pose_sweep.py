#!/usr/bin/env python3
"""Simulation-only pose sweep. Run with Jazzy sourced and controllers active."""
import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml


def stats(values):
    a = np.asarray(values, dtype=float)
    if not a.size:
        return {"samples": 0}
    return {"samples": int(a.size), "max": float(a.max()),
            "rms": float(np.sqrt(np.mean(a * a))),
            "p95": float(np.percentile(a, 95)), "p99": float(np.percentile(a, 99))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', required=True, type=Path)
    parser.add_argument('--output', type=Path, default=Path.home() / 'arm_validation')
    parser.add_argument('--seed', type=int, default=20261001)
    args = parser.parse_args()
    sys.path.insert(0, str(args.package / 'scripts'))
    from kinematics import Chain, error
    from send_trajectory import execute_trajectory, fresh_positions
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import Float64MultiArray
    from control_msgs.msg import JointTrajectoryControllerState

    config_path = args.package / 'config' / 'robot_mit.yaml'
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    if config['hardware']['backend'] != 'can_mock' or len(config['joints']) != 7:
        raise RuntimeError('Requires the seven-axis can_mock configuration')
    chain = Chain(config)
    names = [j['name'] for j in chain.joints]
    rng = np.random.default_rng(args.seed)
    # 14 single-axis targets plus 16 mixed targets: 30 distinct reachable poses.
    targets = []
    for i in range(7):
        for sign in (-1, 1):
            q = np.zeros(7); q[i] = sign * .18
            targets.append(q.tolist())
    targets += rng.uniform(-.25, .25, (16, 7)).tolist()
    out = args.output / ('pose_sweep_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True)
    report = {'seed': args.seed, 'planned': 30, 'results': [], 'all_passed': False,
              'scope': 'Simulation model only; sampled controller tracking, not physical accuracy',
              'thresholds': {'final_joint_rad': .02, 'sampled_tracking_rad': .2,
                             'final_model_position_m': .005, 'final_model_orientation_rad': .02},
              'targets_fk_joints': targets, 'config': config}
    def save():
        (out / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    save()
    print('Results:', out, flush=True)
    rclpy.init()
    node = rclpy.create_node('arm_pose_sweep')
    case = [0]
    health = [None, 0.0]
    faults, tracking, periods, sequences, drops = [], [], [], [], []
    received = {'tracking': 0.0, 'timing': 0.0}
    files = [open(out / name, 'w', newline='', encoding='utf-8') for name in
             ('tracking.csv', 'timing.csv', 'diagnostics.csv')]
    tw, cw, dw = [csv.writer(f) for f in files]
    tw.writerow(['case', 'stamp_ns', *['reference_' + n for n in names],
                 *['feedback_' + n for n in names], *['error_' + n for n in names]])
    cw.writerow(['receipt_case', 'cycle', 'period_us', 'read_us', 'write_us', 'expected_period_us'])
    dw.writerow(['case', 'level', 'backend', 'message', 'values'])

    def diagnostic(msg):
        for s in msg.status:
            if s.name != 'arm_control/hardware':
                continue
            health[:] = [s, time.monotonic()]
            values = {v.key: v.value for v in s.values}
            dw.writerow([case[0], s.level, s.hardware_id, s.message, json.dumps(values)])
            drops.append(int(values.get('timing_dropped', '0')))
            level = int.from_bytes(s.level, 'little') if isinstance(s.level, (bytes, bytearray)) else int(s.level)
            if level != 0 or s.hardware_id != 'can_mock':
                faults.append(f'{s.hardware_id}: level={s.level} {s.message}')

    def timing(msg):
        received['timing'] = time.monotonic()
        if len(msg.data) % 5:
            faults.append('Malformed timing data'); return
        for i in range(0, len(msg.data), 5):
            row = list(msg.data[i:i + 5])
            if not all(math.isfinite(v) for v in row):
                faults.append('Nonfinite timing data'); continue
            cw.writerow([case[0], *row])
            sequences.append(row[0]); periods.append(row)

    def controller(msg):
        ref = getattr(msg, 'reference', getattr(msg, 'desired', None))
        actual = getattr(msg, 'feedback', getattr(msg, 'actual', None))
        if ref is None or actual is None or len(msg.joint_names) != 7 or set(msg.joint_names) != set(names):
            faults.append('Unexpected controller state schema or joints'); return
        if len(ref.positions) != 7 or len(actual.positions) != 7:
            faults.append('Missing controller position feedback'); return
        order = [msg.joint_names.index(n) for n in names]
        r = np.array(ref.positions)[order]; a = np.array(actual.positions)[order]
        if not np.isfinite(r).all() or not np.isfinite(a).all():
            faults.append('Nonfinite controller feedback'); return
        e = r - a
        tracking.append(e.tolist()); received['tracking'] = time.monotonic()
        tw.writerow([case[0], msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec, *r, *a, *e])

    node.create_subscription(DiagnosticArray, '/diagnostics', diagnostic, 100)
    node.create_subscription(Float64MultiArray, '/arm_hardware/timing', timing, 100)
    node.create_subscription(JointTrajectoryControllerState,
                             '/joint_trajectory_controller/controller_state', controller, 100)
    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
    def check():
        if faults:
            raise RuntimeError(faults[0])
        now = time.monotonic()
        if health[0] is None or now - health[1] > 1:
            raise RuntimeError('Fresh active can_mock diagnostics required')
        if any(now - received[k] > 1 for k in received):
            raise RuntimeError('Controller tracking or timing stream stale')
    try:
        started = time.monotonic()
        while time.monotonic() - started < 10:
            spin(.1)
            if health[0] and all(received.values()):
                break
        check()
        report['startup_wait_s'] = time.monotonic() - started
        execute_trajectory(node, chain.joints, [0.] * 7, 12.)
        spin(.3); check()
        # Remove discovery and initial positioning from measured test data.
        tracking.clear(); periods.clear(); sequences.clear()
        drops[:] = drops[-1:]
        for index, q in enumerate(targets, 1):
            case[0] = index
            record = {'case': index, 'fk_input_rad': q, 'passed': False}
            report['results'].append(record)
            print(f'[{index}/30] pose', flush=True)
            save()
            check()
            seed = fresh_positions(node, chain.joints)
            target = chain.fk(q)
            solved = chain.ik(target, seed)
            residual = error(target, chain.fk(solved))
            spin(.2); check()
            start_tracking = len(tracking)
            measured = execute_trajectory(node, chain.joints, solved.tolist(), 12.)
            spin(.3); check()
            observed = np.asarray(tracking[start_tracking:])
            if len(observed) < 20:
                raise RuntimeError('Insufficient controller tracking samples')
            final = error(target, chain.fk(measured))
            peak = float(np.max(np.abs(observed)))
            record.update(solved_rad=solved.tolist(), measured_rad=measured,
                          solver_position_m=float(np.linalg.norm(residual[:3])),
                          solver_orientation_rad=float(np.linalg.norm(residual[3:])),
                          final_model_position_m=float(np.linalg.norm(final[:3])),
                          final_model_orientation_rad=float(np.linalg.norm(final[3:])),
                          final_joint_max_rad=float(np.max(np.abs(np.asarray(measured) - solved))),
                          tracking_samples=len(observed), sampled_tracking_max_rad=peak,
                          tracking_rms_rad_per_joint=np.sqrt(np.mean(observed**2, axis=0)).tolist())
            if peak > .2 or np.linalg.norm(final[:3]) > .005 or np.linalg.norm(final[3:]) > .02:
                raise RuntimeError('Tracking or final model pose threshold exceeded')
            record['passed'] = True
            save()
            for f in files:
                f.flush()
            print('PASS', flush=True)
        report['all_passed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['failure'] = str(exc) or 'Interrupted'
        print('STOP:', report['failure'], flush=True)
        # Action failure can arrive before the next 100 ms hardware diagnostic.
        # Observe only: do not recover or submit another motion.
        if not isinstance(exc, KeyboardInterrupt):
            try:
                spin(2.0)
            except Exception as capture_error:
                report['post_failure_capture_error'] = str(capture_error)
    finally:
        report['passed'] = sum(r['passed'] for r in report['results'])
        report['diagnostic_faults'] = faults
        report['timing'] = {
            'period_us': stats([r[1] for r in periods]),
            'absolute_jitter_us': stats([abs(r[1] - r[4]) for r in periods]),
            'missing_cycles': int(sum(max(0, b-a-1) for a,b in zip(sequences, sequences[1:]))),
            'sequence_resets': sum(b <= a for a,b in zip(sequences, sequences[1:])),
            'ring_drops': max(drops)-min(drops) if drops else None,
            'note': 'Whole sweep including inter-motion intervals; receipt_case is not exact cycle ownership. No hard-real-time pass claim.'}
        report['timing']['sample_integrity_passed'] = bool(periods) and not (
            report['timing']['missing_cycles'] or report['timing']['sequence_resets']
            or report['timing']['ring_drops'])
        save()
        for f in files:
            f.close()
        node.destroy_node(); rclpy.shutdown()
    print(f"Motion results: {report['passed']}/30; directory: {out}")
    return 0 if report['all_passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
