"""Continuous path regressions; requires only Python's standard library."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from trajectory_path import evaluate, plan_path, path_samples, load_path


class PathTest(unittest.TestCase):
    def setUp(self):
        self.joints = [dict(name='joint1', lower=-2., upper=2.,
                           max_velocity=3., max_acceleration=10.)]

    def test_nonunit_durations_and_unique_timestamps(self):
        plan = plan_path(self.joints, [0.], [[.1], [.2]], [.7, 1.234567891])
        rows = list(path_samples(plan))
        self.assertEqual(rows[0][0], 0)
        self.assertEqual(rows[-1][0], 1934567891)
        self.assertTrue(all(a[0] < b[0] for a, b in zip(rows, rows[1:])))
        self.assertEqual(sum(t == 700000000 for t, _ in rows), 1)
        self.assertAlmostEqual(rows[-1][1][0][0], .2)
        self.assertEqual(rows[0][1][1], [0.])
        self.assertEqual(rows[-1][1][1], [0.])

    def test_c2_join_and_nonzero_through_velocity(self):
        plan = plan_path(self.joints, [0.], [[.1], [.3]], [1., 2.])
        left, right = plan[0][1][0], plan[1][1][0]
        for derivative in range(3):
            self.assertAlmostEqual(evaluate(left[derivative], 1), evaluate(right[derivative], 0))
        self.assertAlmostEqual(evaluate(left[1], 1), .1)
        self.assertAlmostEqual(evaluate(left[2], 1), 0.)

    def test_reversal_and_dwell_stop(self):
        for final in (0., .1):
            plan = plan_path(self.joints, [0.], [[.1], [final]], [1., 1.])
            self.assertAlmostEqual(evaluate(plan[0][1][0][1], 1), 0.)
            self.assertAlmostEqual(evaluate(plan[1][1][0][1], 0), 0.)

    def test_derivatives_match_finite_differences(self):
        plan = plan_path(self.joints, [0.], [[.1], [.3]], [.8, 1.6])
        for ns, axes in plan:
            p, v, a = axes[0]
            for u in (.1, .4, .8):
                h = 1e-5
                duration = ns / 1e9
                self.assertAlmostEqual((evaluate(p, u+h)-evaluate(p, u-h))/(2*h*duration),
                                       evaluate(v, u), places=7)
                self.assertAlmostEqual((evaluate(v, u+h)-evaluate(v, u-h))/(2*h*duration),
                                       evaluate(a, u), places=7)

    def test_reject_invalid_and_excessive_motion(self):
        cases = [([], []), ([[.1]], []), ([[.1, .2]], [1.]),
                 ([[float('nan')]], [1.]), ([[3.]], [1.]),
                 ([[.1]], [0.]), ([[.1]], [float('nan')]),
                 ([[1.]], [.02]), ([[.1], [.2]], [1500., 1500.])]
        for points, times in cases:
            with self.subTest(points=points, times=times), self.assertRaises(ValueError):
                plan_path(self.joints, [0.], points, times)
        with self.assertRaisesRegex(ValueError, 'acceleration'):
            plan_path([dict(self.joints[0], max_acceleration=.01)], [0.], [[.1]], [1.])

    def test_joint_order_and_seven_axis_mapping(self):
        joints = [dict(self.joints[0], name=f'joint{i+1}') for i in range(7)]
        data = dict(joint_names=[j['name'] for j in joints],
                    waypoints=[[.1*(-1)**i for i in range(7)]], durations=[2.])
        points, times = load_path(data, joints)
        rows = list(path_samples(plan_path(joints, [0.]*7, points, times)))
        self.assertEqual(rows[-1][1][0], points[-1])
        data['joint_names'].reverse()
        with self.assertRaises(ValueError):
            load_path(data, joints)


if __name__ == '__main__':
    unittest.main()
