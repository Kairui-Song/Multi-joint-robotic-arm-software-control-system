import copy
from pathlib import Path
import sys
import unittest
import numpy as np
import yaml
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from kinematics import Chain, error, rotation_log, rotation
from send_trajectory import validate_motion


class KinematicsTest(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT / 'config/robot_can.yaml').read_text(encoding='utf-8'))
        self.chain = Chain(self.config)

    def test_known_fk_zero(self):
        np.testing.assert_allclose(self.chain.fk([0.] * 6)[:3, 3], [.8, 0., .2], atol=1e-12)

    def test_fk_ik_roundtrip(self):
        for q in ([.2, -.4, .6, .3, -.2, .1], [-.3, .5, -.7, -.2, .3, -.4], [.1, .1, .1, .1, .1, .1]):
            target = self.chain.fk(q)
            solved = self.chain.ik(target, [0.] * 6)
            e = error(target, self.chain.fk(solved))
            self.assertLess(np.linalg.norm(e[:3]), 1e-4)
            self.assertLess(np.linalg.norm(e[3:]), 1e-3)
            validate_motion(self.config['joints'], [0.] * 6, solved, 10.)

    def test_jacobian_finite_difference(self):
        q = np.array([.2, -.4, .6, .3, -.2, .1]); t, j = self.chain.fk(q, jacobian=True)
        for i in range(6):
            perturbed = q.copy(); perturbed[i] += 1e-7
            difference = error(self.chain.fk(perturbed), t) / 1e-7
            np.testing.assert_allclose(j[:, i], difference, atol=1e-6)

    def test_unreachable_and_invalid(self):
        t = self.chain.fk([0.] * 6); t[0, 3] = 10
        with self.assertRaises(ValueError): self.chain.ik(t, [0.] * 6, attempts=2, iterations=40)
        t[0, 0] = 2
        with self.assertRaises(ValueError): self.chain.ik(t, [0.] * 6)
        with self.assertRaises(ValueError): self.chain.fk([10.] * 6)
        with self.assertRaises(ValueError): self.chain.fk([float('nan')] * 6)
        config = copy.deepcopy(self.config); config['joints'][0]['axis'] = [0, 0, 0]
        with self.assertRaises(ValueError): Chain(config)

    def test_half_turn_orientation(self):
        r = rotation([0., 0., 1.], np.pi)
        self.assertAlmostEqual(np.linalg.norm(rotation_log(r)), np.pi)

    def test_motion_rejects_unsafe_targets(self):
        for target, duration in [([3.] * 6, 10), ([1.] * 6, .1), ([float('nan')] * 6, 10), ([0.] * 5, 10)]:
            with self.assertRaises(ValueError): validate_motion(self.config['joints'], [0.] * 6, target, duration)


if __name__ == '__main__': unittest.main()
