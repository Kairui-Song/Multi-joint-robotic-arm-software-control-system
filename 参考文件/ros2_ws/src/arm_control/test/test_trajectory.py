"""Offline contract tests; no ROS runtime or motors required."""
import importlib.util
from pathlib import Path
import unittest
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

spec = importlib.util.spec_from_file_location(
    'trajectory', Path(__file__).resolve().parents[1] / 'scripts/send_trajectory.py')
trajectory = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trajectory)


class TrajectoryContractTest(unittest.TestCase):
    def setUp(self):
        self.joints = [dict(name='joint1', lower=-2., upper=2.,
                           max_velocity=2., max_acceleration=4.)]

    def test_encoded_time_and_boundary_conditions(self):
        duration = 1.234567891
        rows = list(trajectory.trajectory_samples(self.joints, [0.], [1.], duration))
        self.assertEqual(rows[0][0], 0)
        self.assertEqual(rows[-1][0], round(duration * 1e9))
        self.assertTrue(all(a[0] < b[0] for a, b in zip(rows, rows[1:])))
        self.assertAlmostEqual(rows[-1][1][0][0], 1.)
        self.assertAlmostEqual(rows[-1][1][1][0], 0.)
        for stamp, (q, v, a) in rows:
            t = stamp / 1e9
            self.assertAlmostEqual(q[0], 3*t*t/duration**2 - 2*t**3/duration**3)
            self.assertAlmostEqual(v[0], 6*t/duration**2 - 6*t*t/duration**3)
            self.assertAlmostEqual(a[0], 6/duration**2 - 12*t/duration**3)

    def test_invalid_limits_cannot_disable_validation(self):
        for key in ('max_velocity', 'max_acceleration', 'lower', 'upper'):
            for value in (float('nan'), float('inf')):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    trajectory.validate_motion([dict(self.joints[0], **{key: value})], [0.], [1.], 2.)
        for key in ('max_velocity', 'max_acceleration'):
            with self.assertRaises(ValueError):
                trajectory.validate_motion([dict(self.joints[0], **{key: 0})], [0.], [1.], 2.)

    def test_peak_constraints_and_dimensions(self):
        for start, target, duration in (([0.], [1.], .1), ([0.], [3.], 3.),
                                        ([], [1.], 3.), ([0.], [float('nan')], 3.)):
            with self.assertRaises(ValueError):
                list(trajectory.trajectory_samples(self.joints, start, target, duration))
        with self.assertRaises(ValueError):
            trajectory.validate_motion(self.joints * 2, [0., 0.], [1., 1.], 2.)

    def test_stationary_feedback_required_before_motion(self):
        for velocities in ([0.], [.01], [.2], [-.2], [], [float('nan')], [float('inf')]):
            with self.subTest(velocities=velocities):
                node = Mock()
                node.get_clock.return_value.now.return_value.nanoseconds = 1_000_000_000
                msg = NS(header=NS(stamp=NS(sec=1, nanosec=0)), name=['joint1'],
                         position=[.1], velocity=velocities)
                node.create_subscription.side_effect = lambda kind, topic, cb, qos: cb(msg)
                modules = {'rclpy': NS(), 'rclpy.qos': NS(qos_profile_sensor_data=object()),
                           'sensor_msgs.msg': NS(JointState=object())}
                with patch.dict(sys.modules, modules):
                    if velocities in ([0.], [.01]):
                        self.assertEqual(trajectory.fresh_positions(node, self.joints, require_stopped=True), [.1])
                    else:
                        with self.assertRaisesRegex(RuntimeError, 'stationary velocity'):
                            trajectory.fresh_positions(node, self.joints, require_stopped=True)
                node.destroy_subscription.assert_called_once()

    def run_action(self, *, accepted=True, status=4, done=True, cancel_error=False, moving=False, path=False, invalid_path=False):
        response = NS(status=status, result=NS(error_code=0, error_string='test result'))
        result = Mock(); result.done.return_value = done; result.result.return_value = response
        handle = Mock(accepted=accepted); handle.get_result_async.return_value = result
        cancel = Mock(); cancel.done.return_value = True
        cancel.result.return_value = NS(goals_canceling=[NS()])
        handle.cancel_goal_async.return_value = cancel
        if cancel_error:
            handle.cancel_goal_async.side_effect = RuntimeError('cancel transport failed')
        future = Mock(); future.done.return_value = True; future.result.return_value = handle
        client = Mock(); client.wait_for_server.return_value = True; client.send_goal_async.return_value = future
        modules = {
            'rclpy': NS(spin_until_future_complete=Mock()),
            'rclpy.action': NS(ActionClient=Mock(return_value=client)),
            'control_msgs.action': NS(FollowJointTrajectory=NS(
                Goal=lambda: NS(trajectory=NS(joint_names=[], points=[])), Result=NS(SUCCESSFUL=0))),
            'trajectory_msgs.msg': NS(JointTrajectoryPoint=lambda: NS(time_from_start=NS(sec=0, nanosec=0))),
            'action_msgs.msg': NS(GoalStatus=NS(STATUS_SUCCEEDED=4)),
        }
        self.client, self.handle = client, handle
        feedback = RuntimeError('stationary velocity required') if moving else [[0.], [1.]]
        with patch.dict(sys.modules, modules), patch.object(trajectory, 'fresh_positions', side_effect=feedback) as positions:
            try:
                options = dict(waypoints=[[.5], [3. if invalid_path else 1.]], durations=[3., 3.]) if path else {}
                return trajectory.execute_trajectory(Mock(), self.joints, [1.], 2., **options)
            finally:
                self.assertTrue(positions.call_args_list[0].kwargs['require_stopped'])

    def test_moving_start_does_not_send_goal(self):
        with self.assertRaisesRegex(RuntimeError, 'stationary velocity'):
            self.run_action(moving=True)
        self.client.send_goal_async.assert_not_called()
        self.client.destroy.assert_called_once()

    def test_path_uses_one_goal_with_continuous_join(self):
        self.assertEqual(self.run_action(path=True), [1.])
        self.client.send_goal_async.assert_called_once()
        points = self.client.send_goal_async.call_args.args[0].trajectory.points
        times = [p.time_from_start.sec * 10**9 + p.time_from_start.nanosec for p in points]
        self.assertEqual(times[-1], 6 * 10**9)
        self.assertTrue(all(a < b for a, b in zip(times, times[1:])))
        self.assertEqual(times.count(3 * 10**9), 1)
        self.assertGreater(points[times.index(3 * 10**9)].velocities[0], 0.)

    def test_invalid_later_waypoint_prevents_entire_submission(self):
        with self.assertRaises(ValueError):
            self.run_action(path=True, invalid_path=True)
        self.client.send_goal_async.assert_not_called()
        self.client.destroy.assert_called_once()

    def test_action_success_checks_feedback_and_cleans_up(self):
        self.assertEqual(self.run_action(), [1.])
        self.handle.cancel_goal_async.assert_not_called()
        self.client.destroy.assert_called_once()

    def test_rejection_and_preemption_are_not_success(self):
        for options in ({'accepted': False}, {'status': 5}):
            with self.subTest(options=options), self.assertRaises(RuntimeError):
                self.run_action(**options)
            self.client.destroy.assert_called_once()

    def test_timeout_cancels_accepted_goal(self):
        with self.assertRaisesRegex(RuntimeError, 'completion timed out'):
            self.run_action(done=False)
        self.handle.cancel_goal_async.assert_called_once()
        self.client.destroy.assert_called_once()

    def test_cancel_transport_failure_still_releases_client(self):
        with self.assertRaisesRegex(RuntimeError, 'cancel transport failed'):
            self.run_action(done=False, cancel_error=True)
        self.client.destroy.assert_called_once()


if __name__ == '__main__':
    unittest.main()
