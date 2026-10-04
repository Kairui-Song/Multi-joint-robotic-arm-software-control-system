#!/usr/bin/env python3
"""FK -> IK -> Bezier -> FollowJointTrajectory -> configured CAN arm."""
import argparse
import json
import sys
import yaml
import numpy as np
from kinematics import Chain, transform, error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--xyz', type=float, nargs=3, help='Target xyz in base_link, metres')
    group.add_argument('--fk-joints', type=float, nargs='+', help='Use FK of these joints as an IK target (round-trip test)')
    parser.add_argument('--rpy', type=float, nargs=3, default=[0, 0, 0], help='Fixed-axis RPY, radians')
    parser.add_argument('--seed', type=float, nargs='+', help='IK seed for offline calculation; live execution uses encoder state')
    parser.add_argument('--duration', type=float, default=5.)
    parser.add_argument('--execute', action='store_true', help='Submit the solved motion to ROS2')
    # Keep offline FK/IK usable without ROS; strip ROS arguments only when supplied.
    cli_args = sys.argv[1:]
    if '--ros-args' in cli_args:
        from rclpy.utilities import remove_ros_args
        cli_args = remove_ros_args()[1:]
    args = parser.parse_args(cli_args)
    node = None
    try:
        with open(args.config, encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        chain = Chain(config)
        if args.execute:
            import rclpy
            from rclpy.node import Node
            from send_trajectory import fresh_positions, execute_trajectory
            rclpy.init(); node = Node('arm_pose_goal')
            # The running hardware's backend is checked, not just a local YAML flag.
            from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
            import time
            live = []
            def diagnostic(msg):
                for status in msg.status:
                    if status.name == 'arm_control/hardware' and status.level == DiagnosticStatus.OK:
                        live[:] = [status.hardware_id]
            sub = node.create_subscription(DiagnosticArray, 'diagnostics', diagnostic, 10)
            deadline = time.monotonic() + 3
            while not live and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            node.destroy_subscription(sub)
            if not live:
                raise ValueError('Active hardware diagnostics unavailable')
            if live[0] not in ('sim', 'can_mock', 'rm_mock') and not chain.calibrated:
                raise ValueError('Real arm motion requires calibrated kinematics; sample geometry is not valid')
            seed = fresh_positions(node, config['joints'])
        else:
            seed = args.seed if args.seed is not None else np.clip(np.zeros(len(chain.joints)), chain.lower, chain.upper)
        target = chain.fk(args.fk_joints) if args.fk_joints is not None else transform(args.xyz, args.rpy)
        solved = chain.ik(target, seed)
        residual = error(target, chain.fk(solved))
        print(json.dumps({'joint_names': [j['name'] for j in chain.joints], 'joint_target_rad': solved.tolist(),
                          'target_transform': target.tolist(), 'position_error_m': float(np.linalg.norm(residual[:3])),
                          'orientation_error_rad': float(np.linalg.norm(residual[3:])),
                          'model_calibrated': chain.calibrated, 'execute': args.execute}, indent=2))
        if args.execute:
            execute_trajectory(node, config['joints'], solved.tolist(), args.duration)
    except Exception as exc:
        print(str(exc), file=sys.stderr); return 1
    finally:
        if node is not None:
            node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
