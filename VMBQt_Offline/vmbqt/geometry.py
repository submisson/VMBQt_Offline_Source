"""Metric 3D polylines, with planar offsets and length-based correspondence."""
import math
import numpy as np


def length(points, planar=False):
    p = np.asarray(points, dtype=float)
    if len(p) < 2:
        return 0.
    if planar:
        p = p[:, :2]
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def sample(points, count):
    p = np.asarray(points, dtype=float)
    if len(p) < 2:
        raise ValueError("折线至少需要两个点")
    distances = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    keep = np.r_[True, np.diff(distances) > 1e-9]
    p, distances = p[keep], distances[keep]
    if len(p) < 2:
        raise ValueError("折线长度为零")
    t = np.linspace(0, distances[-1], max(2, int(count)))
    return np.column_stack([np.interp(t, distances, p[:, axis]) for axis in range(3)])


def offset(points, distance):
    p = np.asarray(points, dtype=float)
    if len(p) < 2:
        raise ValueError("偏移折线至少需要两个点")
    directions = np.diff(p[:, :2], axis=0)
    norms = np.linalg.norm(directions, axis=1)
    if np.any(norms < 1e-8):
        raise ValueError("折线含重叠 XY 点，无法计算偏移")
    directions /= norms[:, None]
    normals = np.column_stack([-directions[:, 1], directions[:, 0]])
    vectors = np.empty((len(p), 2)); vectors[0] = normals[0]; vectors[-1] = normals[-1]
    for i in range(1, len(p) - 1):
        normal = normals[i - 1] + normals[i]
        norm = np.linalg.norm(normal)
        if norm < 1e-5:
            raise ValueError("折线包含折返点，请拆分后再偏移")
        normal /= norm
        divisor = np.dot(normal, normals[i])
        if divisor < .25:
            raise ValueError("转角过尖，偏移可能自相交，请先增加转弯半径")
        vectors[i] = normal / divisor
    out = p.copy(); out[:, :2] += float(distance) * vectors
    return out


def centerline(left, right, spacing=1.):
    n = max(2, int(math.ceil(max(length(left), length(right)) / max(.1, spacing))) + 1)
    return (sample(left, n) + sample(right, n)) / 2


def nearest_segment(points, point):
    p = np.asarray(points, dtype=float); q = np.asarray(point, dtype=float)
    best = None
    for i in range(len(p) - 1):
        a, b = p[i], p[i + 1]
        denominator = np.dot((b - a)[:2], (b - a)[:2])
        t = float(np.clip(np.dot((q - a)[:2], (b - a)[:2]) / max(denominator, 1e-20), 0, 1))
        foot = a + t * (b - a)
        distance = float(np.linalg.norm((q - foot)[:2]))
        if best is None or distance < best[0]:
            best = distance, i, t, foot
    if best is None:
        raise ValueError("折线至少需要两个点")
    return best


def signed_area(points):
    p = np.asarray(points, dtype=float)
    if len(p) < 3:
        return 0.
    return .5 * float(np.sum(p[:, 0] * np.roll(p[:, 1], -1) -
                             p[:, 1] * np.roll(p[:, 0], -1)))


def contains(points, xy):
    p = np.asarray(points)
    if len(p) < 3:
        return False
    x, y = xy[:2]; inside = False
    j = len(p) - 1
    for i in range(len(p)):
        xi, yi = p[i, :2]; xj, yj = p[j, :2]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def self_intersects(points, closed=False):
    p = np.asarray(points, dtype=float)[:, :2]
    if closed and len(p) and np.linalg.norm(p[0] - p[-1]) > 1e-9:
        p = np.vstack([p, p[0]])
    def cross(a, b, c):
        ab, ac = b - a, c - a
        return ab[0] * ac[1] - ab[1] * ac[0]
    for i in range(len(p) - 1):
        for j in range(i + 2, len(p) - 1):
            if closed and i == 0 and j == len(p) - 2:
                continue
            a, b, c, d = p[i], p[i + 1], p[j], p[j + 1]
            if cross(a, b, c) * cross(a, b, d) < -1e-12 and \
                    cross(c, d, a) * cross(c, d, b) < -1e-12:
                return True
    return False


def smooth(points, iterations=2):
    p = np.asarray(points, dtype=float)
    for _ in range(max(1, min(5, iterations))):
        q = np.empty((2 * len(p), 3))
        q[0] = p[0]; q[-1] = p[-1]
        q[1:-1:2] = .75 * p[:-1] + .25 * p[1:]
        q[2:-1:2] = .25 * p[:-1] + .75 * p[1:]
        p = q
    return p


def bezier_connector(a, tangent_a, b, tangent_b, spacing=1.):
    a, b = np.array(a), np.array(b)
    distance = np.linalg.norm((b - a)[:2])
    if distance < .05:
        raise ValueError("两车道端点过近，请使用连接端点操作")
    ta, tb = np.array(tangent_a, dtype=float), np.array(tangent_b, dtype=float)
    ta /= max(np.linalg.norm(ta[:2]), 1e-12)
    tb /= max(np.linalg.norm(tb[:2]), 1e-12)
    c, d = a + ta * distance / 3, b - tb * distance / 3
    t = np.linspace(0, 1, max(4, int(distance / max(.2, spacing)) + 1))[:, None]
    return (1-t)**3 * a + 3*(1-t)**2*t*c + 3*(1-t)*t**2*d + t**3*b
