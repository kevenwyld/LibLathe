"""Geometry helpers shared by the segment group and the operations."""
import math


def sample_points(seg, step):
    """Points (x, z) along a segment (line or arc) at most step apart, ends included."""
    sx, sz, ex, ez = seg.start.X, seg.start.Z, seg.end.X, seg.end.Z
    if seg.bulge == 0:
        length = math.hypot(ex - sx, ez - sz)
        n = max(1, int(math.ceil(length / step)))
        return [(sx + (ex - sx) * i / n, sz + (ez - sz) * i / n) for i in range(n + 1)]
    c = seg.get_centre_point()
    rad = seg.get_radius()
    a0 = math.atan2(sx - c.X, sz - c.Z)
    included = seg.get_angle()
    sweep = included
    px, pz = c.X + rad * math.sin(a0 - included), c.Z + rad * math.cos(a0 - included)
    qx, qz = c.X + rad * math.sin(a0 + included), c.Z + rad * math.cos(a0 + included)
    if math.hypot(px - ex, pz - ez) < math.hypot(qx - ex, qz - ez):
        sweep = -included
    n = max(1, int(math.ceil(abs(sweep) * rad / step)))
    return [(c.X + rad * math.sin(a0 + sweep * i / n), c.Z + rad * math.cos(a0 + sweep * i / n))
            for i in range(n + 1)]


def simplify_polyline(points, tolerance):
    """Douglas-Peucker on (x, z) points; keeps the end points."""
    if len(points) < 3:
        return points
    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        a, b = stack.pop()
        ax, az = points[a]
        bx, bz = points[b]
        dx, dz = bx - ax, bz - az
        length = math.hypot(dx, dz)
        worst, index = 0.0, None
        for i in range(a + 1, b):
            px, pz = points[i]
            d = math.hypot(px - ax, pz - az) if length == 0 else abs(dx * (az - pz) - dz * (ax - px)) / length
            if d > worst:
                worst, index = d, i
        if index is not None and worst > tolerance:
            keep[index] = True
            stack.append((a, index))
            stack.append((index, b))
    return [p for p, k in zip(points, keep) if k]


def dilate_rect_profile(values, sx, sz, step):
    """Radii on a Z grid of `step` grown by sz along Z, then sx radially (numpy array in, new array out).
    Negative values shrink it, never past the axis; -inf marks Z with no part."""
    import numpy as np

    out = values.copy()
    pick = np.maximum if sz >= 0 else np.minimum
    for k in range(1, int(math.ceil(abs(sz) / step - 1e-9)) + 1):
        out[k:] = pick(out[k:], values[:-k])
        out[:-k] = pick(out[:-k], values[k:])
    out = out + sx
    if sx < 0 or sz < 0:
        finite = np.isfinite(out)
        out[finite] = np.maximum(out[finite], 0.0)
    return out
