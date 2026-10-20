import math

import numpy as np

import liblathe.op.base
from liblathe.base.command import Command


class GrooveOP(liblathe.op.base.BaseOP):
    """
    Grooving with a grooving (or parting) insert: a blade of width W whose
    cutting edge is a flat bottom with a corner radius R at each end
    (R = W / 2 is a full radius insert), with straight flanks above it.

    The path is programmed to the tool's reference point: the cutting edge's
    point closest to the axis (X) and its chuck-side flank (Z). So the blade
    spans Z0 .. Z0 + W and its edge rises from X0 at the bottom flat to
    X0 + R at the flanks.

    The tool can cut in both directions, so the path is found by "drop
    cutting": at each Z0, the reference X0 is the lowest the blade can go
    without its cutting edge or flanks crossing the part (plus the stock to
    leave). Roughing plunges straight in at a spacing of step_over along Z;
    then finish passes follow the X0 profile from the front to the back.

    The cutting depth is measured from the outer limit (x_outer(): the
    stock, or the max diameter if one is set), which is also where plunges
    start. Measuring from the stock is the safe default: the holder cannot
    hit uncut stock. When the surface has already been turned smaller, the
    max diameter says so.

    Internally X is negative (the part profile is on the -X side); the
    output X is a positive radius.
    """

    # the tool is described by the parameters below, not a liblathe Tool
    needs_tool = False

    # grid spacing along Z for the drop cutter
    resolution = 0.005

    # tool geometry
    groove_width = 0
    corner_radius = 0
    cutting_depth = 0
    turning_depth = 0
    # material the plunges leave for the finish passes
    finish_allowance = 0.2

    def generate_path(self):
        self.warnings = []
        self.plunges = []
        self.finish_paths = []

        w = float(self.groove_width)
        if w <= 0:
            raise ValueError("Grooving tool width must be greater than zero")
        r = max(0.0, min(float(self.corner_radius), w / 2))
        dz = self.resolution

        # Z range of the reference point (the chuck-side flank), as for the
        # turning ops' tip: from the window end (so no part of the blade goes
        # past it toward the chuck) to the window start, or the stock front.
        # Toward the tailstock the blade may extend up to its width past the
        # window start, which lets it reach the top of a wall at the start;
        # the drop cutter keeps it clear of the part there too.
        top = self.z_start() if self.starts_in_stock() else self.stock.z_max
        z_lo = self.z_end()
        z_hi = top

        r_outer = -self.x_outer()            # stock radius (or max diameter / 2)
        # lowest the reference may go: the minimum diameter if one is set,
        # otherwise past the axis by the corner radius (puts the round end's
        # centre on the axis, which a ball or point needs), and never deeper
        # than the tool's cutting depth below the stock surface
        r_axis = -self.x_inner() if self.min_dia > 0 else -r
        r_min = max(r_axis, r_outer - self.cutting_depth) if self.cutting_depth > 0 else r_axis

        # grid of reference positions, plus one blade width of profile beyond it
        n_ref = int(math.floor((z_hi - z_lo) / dz + 1e-9)) + 1
        z_ref = z_hi - np.arange(n_ref) * dz
        if z_ref[-1] > z_lo + 1e-9:
            z_ref = np.append(z_ref, z_lo)
        m = int(math.ceil(w / dz))
        u = np.linspace(0.0, w, m + 1)
        edge = _edge_height(u, w, r)

        profile = self._profile_radius(z_lo, top + w, dz)

        def drop(allowance):
            # profile dilated by the allowance (a disc around each point)
            prof = _dilate(profile, allowance, dz)
            x0 = np.full(len(z_ref), -np.inf)
            for k in range(m + 1):
                x0 = np.maximum(x0, prof.at(z_ref + u[k]) - edge[k])
            # one cell either side, so straight moves between grid points
            # cannot dip into a steep wall
            guard = np.maximum(x0, np.maximum(np.roll(x0, 1), np.roll(x0, -1)))
            guard[0] = max(x0[0], x0[1]) if len(x0) > 1 else x0[0]
            guard[-1] = max(x0[-1], x0[-2]) if len(x0) > 1 else x0[-1]
            return guard

        final = drop(self.stock_to_leave)
        deep = final < r_min - 1e-9
        if deep.any():
            self.warnings.append(
                "The part is deeper than the tool can reach: material is left between Z %.3f and "
                "Z %.3f. The cutting depth is measured from the max diameter (default: the stock); "
                "if the surface has already been turned smaller, set Max Diameter to that diameter. "
                "A Min Diameter also limits the depth." % (z_ref[deep].min(), z_ref[deep].max()))
        final = np.maximum(final, r_min)

        passes = max(0, int(self.finish_passes))
        allowance = self.finish_allowance if passes else 0.0
        if passes and self.turning_depth > 0 and allowance > self.turning_depth + 1e-9:
            self.warnings.append(
                "The finish allowance (%.3f) is more than the tool's turning depth (%.3f)"
                % (allowance, self.turning_depth))
        rough = np.maximum(drop(self.stock_to_leave + allowance), r_min) if allowance > 0 else final

        # roughing plunges, front to back, at most step_over apart (and never
        # further apart than 80% of the blade width so neighbours overlap)
        spacing = min(self.step_over if self.step_over > 0 else w, 0.8 * w)
        count = int(math.ceil((z_hi - z_lo) / spacing - 1e-9)) + 1 if z_hi > z_lo else 1
        for j in range(count):
            z0 = z_hi - min(j * spacing, z_hi - z_lo)
            floor = float(np.interp(z0, z_ref[::-1], rough[::-1]))
            if floor < r_outer - 1e-6:
                self.plunges.append((z0, floor))

        # finish passes: the allowance in equal steps down to the final profile
        for k in range(passes):
            offset = allowance * (passes - 1 - k) / passes
            prof = final if offset <= 0 else np.maximum(drop(self.stock_to_leave + offset), r_min)
            pts = _simplify(list(zip(prof.tolist(), z_ref.tolist())), 0.0005)
            self.finish_paths.append(pts)

    def _profile_radius(self, z_lo, z_hi, dz):
        """Largest part radius at each Z on a grid (None where there is no part)."""
        pad = max(self.stock_to_leave + self.finish_allowance, 0) + float(self.groove_width) + 2 * dz
        start = z_lo - pad
        n = int(math.ceil((z_hi + pad - start) / dz)) + 1
        values = np.full(n, -np.inf)
        step = dz / 2.5
        for seg in self.part_segment_group.get_segments():
            for x, z in _sample_segment(seg, step):
                i = int(round((z - start) / dz))
                if 0 <= i < n:
                    values[i] = max(values[i], -x)
        return _Profile(start, dz, values)

    def generate_gcode(self):
        """Plunges then finish passes, as commands with a positive radius X."""
        cmds = [Command('G18', {})]
        r_retract = -self.x_outer() + self.clearance
        for z0, floor in self.plunges:
            cmds.append(Command('G0', {'X': r_retract, 'Z': z0}))
            cmds.append(Command('G1', {'X': floor, 'Z': z0, 'F': self.vfeed}))
            cmds.append(Command('G0', {'X': r_retract, 'Z': z0}))
        for pts in self.finish_paths:
            if not pts:
                continue
            x, z = pts[0]
            cmds.append(Command('G0', {'X': r_retract, 'Z': z}))
            cmds.append(Command('G1', {'X': x, 'Z': z, 'F': self.vfeed}))
            for x, z in pts[1:]:
                cmds.append(Command('G1', {'X': x, 'Z': z, 'F': self.hfeed}))
            cmds.append(Command('G0', {'X': r_retract, 'Z': pts[-1][1]}))
        return cmds


class _Profile:
    """Radius values on a regular Z grid; -inf where there is no part."""

    def __init__(self, start, dz, values):
        self.start, self.dz, self.values = start, dz, values

    def at(self, z):
        i = np.rint((np.asarray(z) - self.start) / self.dz).astype(int)
        out = np.full(i.shape, -np.inf)
        ok = (i >= 0) & (i < len(self.values))
        out[ok] = self.values[i[ok]]
        return out


def _dilate(profile, radius, dz):
    """Grow the profile by a disc of the given radius (stock to leave)."""
    if radius <= 0:
        return profile
    vals = profile.values
    out = vals.copy()
    k_max = int(math.floor(radius / dz))
    for k in range(-k_max, k_max + 1):
        lift = math.sqrt(max(radius * radius - (k * dz) ** 2, 0.0))
        shifted = np.full(len(vals), -np.inf)
        if k >= 0:
            shifted[k:] = vals[:len(vals) - k]
        else:
            shifted[:k] = vals[-k:]
        out = np.maximum(out, shifted + lift)
    return _Profile(profile.start, dz, out)


def _edge_height(u, w, r):
    """Height of the cutting edge above its lowest point at distance u from the chuck-side flank."""
    h = np.zeros_like(u)
    if r <= 0:
        return h
    left = u < r
    right = u > w - r
    h[left] = r - np.sqrt(np.maximum(r * r - (r - u[left]) ** 2, 0.0))
    h[right] = r - np.sqrt(np.maximum(r * r - (u[right] - (w - r)) ** 2, 0.0))
    return h


def _sample_segment(seg, step):
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


def _simplify(points, tolerance):
    """Douglas-Peucker on a polyline of (x, z) points; keeps the end points."""
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
            if length == 0:
                d = math.hypot(px - ax, pz - az)
            else:
                d = abs(dx * (az - pz) - dz * (ax - px)) / length
            if d > worst:
                worst, index = d, i
        if index is not None and worst > tolerance:
            keep[index] = True
            stack.append((a, index))
            stack.append((index, b))
    return [p for p, k in zip(points, keep) if k]
