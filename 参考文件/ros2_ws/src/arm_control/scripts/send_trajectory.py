#!/usr/bin/env python3
"""Joint goal -> synchronized cubic Bezier -> FollowJointTrajectory action."""
import argparse
import math
import sys
import time


def bezier(start, goal, u, duration):
    s = u * u * (3 - 2 * u)
    ds = 6 * u * (1 - u) / duration
    dds = (6 - 12 * u) / duration ** 2
    return ([a + (b - a) * s for a, b in zip(start, goal)],
            [(b - a) * ds for a, b in zip(start, goal)],
            [(b - a) * dds for a, b in zip(start, goal)])


def fresh_positions(node, joints, *, require_stopped=False):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    latest = []
    def receive(msg):
        age = (node.get_clock().now().nanoseconds - (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)) / 1e9
        if 0 <= age < .5 and len(msg.name) == len(msg.position) and len(set(msg.name)) == len(msg.name):
            values = dict(zip(msg.name, msg.position))
            if all(j['name'] in values for j in joints):
                velocities = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
                latest[:] = [[values[j['name']] for j in joints], time.monotonic(), velocities]
    sub = node.create_subscription(JointState, 'joint_states', receive, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 10
        while not latest and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not latest or time.monotonic() - latest[1] > .5:
            raise RuntimeError('Fresh feedback for every configured joint is required')
        for j, q in zip(joints, latest[0]):
            if not math.isfinite(q) or not j['lower'] <= q <= j['upper']:
                raise RuntimeError('Invalid encoder position: ' + j['name'])
            if require_stopped:
                velocity = latest[2].get(j['name'], float('nan'))
                # This curve starts at zero velocity; moving-goal replacement is unsupported.
                tolerance = min(.02, .1 * j['max_velocity'])
                if not math.isfinite(velocity) or abs(velocity) > tolerance:
                    raise RuntimeError('Fresh stationary velocity feedback is required before starting: ' + j['name'])
        return latest[0]
    finally:
        node.destroy_subscription(sub)


def validate_motion(joints, start, target, duration):
    if not math.isfinite(duration) or duration < .02 or duration > 2000:
        raise ValueError('Duration must be 0.02..2000 seconds')
    if len(start) != len(joints) or len(target) != len(joints):
        raise ValueError('Provide exactly one target per joint')
    if not joints or len({j['name'] for j in joints}) != len(joints):
        raise ValueError('Joint names must be nonempty and unique')
    for j, a, b in zip(joints, start, target):
        for key in ('lower', 'upper', 'max_velocity'):
            if not math.isfinite(j[key]):
                raise ValueError('Non-finite joint configuration: ' + j['name'])
        if j['lower'] >= j['upper'] or j['max_velocity'] <= 0:
            raise ValueError('Invalid joint limits: ' + j['name'])
        if 'max_acceleration' in j and (not math.isfinite(j['max_acceleration']) or j['max_acceleration'] <= 0):
            raise ValueError('Invalid acceleration limit: ' + j['name'])
        if not math.isfinite(a) or not math.isfinite(b) or not j['lower'] <= a <= j['upper'] or not j['lower'] <= b <= j['upper']:
            raise ValueError('Joint target/start outside limits: ' + j['name'])
        if 1.5 * abs(b - a) / duration > j['max_velocity']:
            raise ValueError('Increase duration: peak velocity exceeds limit for ' + j['name'])
        if 'max_acceleration' in j and 6 * abs(b - a) / duration ** 2 > j['max_acceleration']:
            raise ValueError('Increase duration: peak acceleration exceeds limit for ' + j['name'])


def trajectory_samples(joints, start, target, duration):
    """ROS-independent trajectory contract: integer timestamps and analytic derivatives."""
    validate_motion(joints, start, target, duration)
    duration = round(duration * 1e9) / 1e9
    validate_motion(joints, start, target, duration)
    samples = max(2, math.ceil(duration * 50))
    for k in range(samples + 1):
        ns = round(duration * k / samples * 1e9)
        # Evaluate at the encoded timestamp, not its unrounded precursor.
        yield ns, bezier(start, target, ns / (duration * 1e9), duration)


def execute_trajectory(node, joints, target, duration, *, waypoints=None, durations=None):
    import rclpy
    from rclpy.action import ActionClient
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from action_msgs.msg import GoalStatus
    client = ActionClient(node, FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory')
    handle, completed = None, False
    try:
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('Trajectory action server unavailable')
        start = fresh_positions(node, joints, require_stopped=True)
        if waypoints is None:
            validate_motion(joints, start, target, duration)
            samples = trajectory_samples(joints, start, target, duration)
        else:
            from trajectory_path import plan_path, path_samples
            segments = plan_path(joints, start, waypoints, durations)
            samples = path_samples(segments)
            target = list(waypoints[-1])
            duration = sum(ns for ns, _ in segments) / 1e9
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = [j['name'] for j in joints]
        for ns, values in samples:
            point = JointTrajectoryPoint()
            point.positions, point.velocities, point.accelerations = values
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 10**9)
            goal.trajectory.points.append(point)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done():
            raise RuntimeError('Goal acceptance timed out; acceptance state is unknown, inspect controller')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('Goal rejected')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=duration + 10)
        if not result.done():
            raise RuntimeError('Trajectory completion timed out')
        completed = True
        response = result.result()
        if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'Trajectory failed (status={response.status}, code={response.result.error_code}): '
                               + response.result.error_string)
        measured = fresh_positions(node, joints)
        if any(abs(q - wanted) > .02 for q, wanted in zip(measured, target)):
            raise RuntimeError('Action succeeded but final measured position differs from target')
        print('Trajectory completed; final encoder positions agree with the joint targets')
        return measured
    finally:
        try:
            if handle is not None and handle.accepted and not completed:
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel, timeout_sec=3)
                if not cancel.done() or not cancel.result().goals_canceling:
                    node.get_logger().error('Cancellation not confirmed; inspect controller and hardware state')
        finally:
            client.destroy()


def main():
    import yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--positions', type=float, nargs='+')
    mode.add_argument('--path', help='JSON file with joint_names, waypoints and durations')
    parser.add_argument('--duration', type=float, default=5.)
    args = parser.parse_args(remove_ros_args()[1:])
    rclpy.init(); node = Node('arm_bezier_trajectory')
    try:
        with open(args.config, encoding='utf-8') as stream:
            joints = yaml.safe_load(stream)['joints']
        if args.path:
            import json
            from trajectory_path import load_path
            with open(args.path, encoding='utf-8') as stream:
                waypoints, durations = load_path(json.load(stream), joints)
            execute_trajectory(node, joints, None, None, waypoints=waypoints, durations=durations)
        else:
            execute_trajectory(node, joints, args.positions, args.duration)
    except Exception as exc:
        print(str(exc), file=sys.stderr); return 1
    finally:
        node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
