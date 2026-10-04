"""Revolute serial-chain FK and damped least-squares IK, in metres and radians."""
import math
import numpy as np


def vector(value, size, name):
    v = np.asarray(value, dtype=float)
    if v.shape != (size,) or not np.isfinite(v).all():
        raise ValueError(f'{name} must contain {size} finite numbers')
    return v


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def rotation(axis, angle):
    k = skew(axis)
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def transform(xyz, rpy):
    xyz = vector(xyz, 3, 'xyz'); r, p, y = vector(rpy, 3, 'rpy')
    t = np.eye(4)
    t[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    t[:3, 3] = xyz
    return t


def rotation_log(r):
    cos_angle = np.clip((np.trace(r) - 1) / 2, -1., 1.)
    angle = math.acos(cos_angle)
    vee = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    if angle < 1e-7:
        return vee / 2
    if math.pi - angle < 1e-5:
        # Eigenvector avoids division by sin(pi) for a half-turn.
        values, vectors = np.linalg.eigh((r + r.T) / 2)
        axis = vectors[:, np.argmax(values)]
        if np.dot(axis, vee) < 0:
            axis = -axis
        return angle * axis
    return angle * vee / (2 * math.sin(angle))


def error(target, actual):
    return np.r_[target[:3, 3] - actual[:3, 3], rotation_log(target[:3, :3] @ actual[:3, :3].T)]


class Chain:
    def __init__(self, config):
        if 'kinematics' not in config:
            raise ValueError('Missing kinematics configuration; placeholder control URDF is not a robot model')
        self.joints = config['joints']; n = len(self.joints)
        if not n or len({j['name'] for j in self.joints}) != n:
            raise ValueError('Empty or duplicate joint names')
        self.origins, self.axes = [], []
        self.lower = vector([j['lower'] for j in self.joints], n, 'lower limits')
        self.upper = vector([j['upper'] for j in self.joints], n, 'upper limits')
        if np.any(self.lower >= self.upper):
            raise ValueError('Invalid joint limits')
        for j in self.joints:
            self.origins.append(transform(j['origin_xyz'], j['origin_rpy']))
            axis = vector(j['axis'], 3, 'axis'); length = np.linalg.norm(axis)
            if length < 1e-10:
                raise ValueError('Joint axis has zero length')
            self.axes.append(axis / length)
        k = config['kinematics']
        self.tool = transform(k.get('tool_xyz', [0, 0, 0]), k.get('tool_rpy', [0, 0, 0]))
        self.calibrated = k.get('calibrated', False) is True

    def check_q(self, q):
        q = vector(q, len(self.joints), 'joint positions')
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError('Joint positions outside limits')
        return q

    def fk(self, q, jacobian=False):
        q = self.check_q(q)
        t = np.eye(4); origins, axes = [], []
        for origin, axis, value in zip(self.origins, self.axes, q):
            t = t @ origin
            origins.append(t[:3, 3].copy()); axes.append(t[:3, :3] @ axis)
            turn = np.eye(4); turn[:3, :3] = rotation(axis, value)
            t = t @ turn
        t = t @ self.tool
        if not jacobian:
            return t
        jac = np.column_stack([np.r_[np.cross(a, t[:3, 3] - p), a] for p, a in zip(origins, axes)])
        return t, jac

    def ik(self, target, seed, *, attempts=8, iterations=250, position_tolerance=1e-4, orientation_tolerance=1e-3):
        target = np.asarray(target, dtype=float)
        if target.shape != (4, 4) or not np.isfinite(target).all() or not np.allclose(target[3], [0, 0, 0, 1]):
            raise ValueError('Invalid target transform')
        r = target[:3, :3]
        if not np.allclose(r.T @ r, np.eye(3), atol=1e-7) or not np.isclose(np.linalg.det(r), 1., atol=1e-7):
            raise ValueError('Target rotation must be in SO(3)')
        seed = self.check_q(seed)
        rng = np.random.default_rng(0)
        weights = np.array([1., 1., 1., .3, .3, .3])
        for attempt in range(attempts):
            q = seed.copy() if attempt == 0 else rng.uniform(self.lower, self.upper)
            for _ in range(iterations):
                actual, jac = self.fk(q, jacobian=True); e = error(target, actual)
                if np.linalg.norm(e[:3]) <= position_tolerance and np.linalg.norm(e[3:]) <= orientation_tolerance:
                    return q
                j = weights[:, None] * jac; ew = weights * e
                step = j.T @ np.linalg.solve(j @ j.T + 0.01 ** 2 * np.eye(6), ew)
                maximum = np.max(np.abs(step))
                if maximum > .2:
                    step *= .2 / maximum
                accepted = False
                for factor in (1., .5, .25, .125, .0625):
                    candidate = np.clip(q + factor * step, self.lower, self.upper)
                    if np.linalg.norm(weights * error(target, self.fk(candidate))) < np.linalg.norm(ew):
                        q = candidate; accepted = True; break
                if not accepted:
                    break
        raise ValueError('IK did not converge within limits; target may be unreachable or a different seed/model is needed')
