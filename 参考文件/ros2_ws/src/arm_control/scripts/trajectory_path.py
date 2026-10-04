"""ROS-independent C2 quintic Bezier paths with conservative continuous bounds."""
import math


def evaluate(control, u):
    values = list(control)
    while len(values) > 1:
        values = [(1 - u) * a + u * b for a, b in zip(values, values[1:])]
    return values[0]


def plan_path(joints, start, waypoints, durations):
    """Return integer segment durations and per-axis quintic control polygons.

    Interior velocity follows the smaller adjacent secant for monotone motion;
    reversals stop. Acceleration is zero at every knot. Convex-hull bounds on
    position and derivative polygons cover the entire curve, not only samples.
    """
    from send_trajectory import validate_motion
    if not waypoints or len(waypoints) != len(durations):
        raise ValueError('Provide one duration per waypoint')
    if len(waypoints) > 1000:
        raise ValueError('At most 1000 waypoints are supported')
    points = [list(start)] + [list(p) for p in waypoints]
    times = []
    for a, b, duration in zip(points, points[1:], durations):
        # Validate endpoints without imposing the single-segment cubic speed bound.
        validate_motion(joints, a, a, 1.)
        validate_motion(joints, b, b, 1.)
        if not math.isfinite(duration) or not .02 <= duration <= 2000:
            raise ValueError('Each duration must be 0.02..2000 seconds')
        times.append(round(duration * 1e9))
    if sum(times) > 2000 * 10**9:
        raise ValueError('Total path duration must not exceed 2000 seconds')
    velocity = [[0.] * len(joints) for _ in points]
    for k in range(1, len(points) - 1):
        for j in range(len(joints)):
            left = (points[k][j] - points[k-1][j]) / (times[k-1] / 1e9)
            right = (points[k+1][j] - points[k][j]) / (times[k] / 1e9)
            if left * right > 0:
                velocity[k][j] = math.copysign(min(abs(left), abs(right)), left)
    segments = []
    for k, ns in enumerate(times):
        duration = ns / 1e9
        polygons = []
        for j, joint in enumerate(joints):
            a, b = points[k][j], points[k+1][j]
            va, vb = velocity[k][j], velocity[k+1][j]
            p = [a, a + va*duration/5, a + 2*va*duration/5,
                 b - 2*vb*duration/5, b - vb*duration/5, b]
            v = [5*(y-x)/duration for x, y in zip(p, p[1:])]
            acc = [4*(y-x)/duration for x, y in zip(v, v[1:])]
            if any(not math.isfinite(x) for x in p + v + acc):
                raise ValueError('Non-finite path coefficients: ' + joint['name'])
            if min(p) < joint['lower'] or max(p) > joint['upper']:
                raise ValueError('Path position bound exceeds limits: ' + joint['name'])
            if max(map(abs, v)) > joint['max_velocity']:
                raise ValueError('Increase duration: path velocity bound exceeds limit for ' + joint['name'])
            if 'max_acceleration' in joint and max(map(abs, acc)) > joint['max_acceleration']:
                raise ValueError('Increase duration: path acceleration bound exceeds limit for ' + joint['name'])
            polygons.append((p, v, acc))
        segments.append((ns, polygons))
    return segments


def path_samples(segments):
    offset = 0
    for index, (ns, polygons) in enumerate(segments):
        count = max(2, math.ceil(ns / 1e9 * 50))
        for k in range(0 if index == 0 else 1, count + 1):
            local = round(ns * k / count)
            u = local / ns
            yield offset + local, tuple(
                [evaluate(axis[derivative], u) for axis in polygons]
                for derivative in range(3))
        offset += ns


def load_path(data, joints):
    """JSON contract: exact configured joint order, waypoints and durations."""
    if not isinstance(data, dict) or data.get('joint_names') != [j['name'] for j in joints]:
        raise ValueError('Path joint_names must match configured names and order exactly')
    return data['waypoints'], data['durations']
