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

    # tool geometry (cutting_depth is the insert's grooving depth, La max)
    groove_width = 0
    corner_radius = 0
    cutting_depth = 0
    turning_depth = 0
    # material the plunges leave for the finish passes
    finish_allowance = 0.2
    # spacing of the finish passes; 0 splits the allowance equally
    finish_step_over = 0.0

    def generate_path(self):
        self.warnings = []
        self.plunges = []
        self.finish_paths = []

        w = float(self.groove_width)
        if w <= 0:
            raise ValueError("Grooving tool width must be greater than zero")
        if float(self.corner_radius) > w / 2 + 1e-9:
            self.warnings.append(
                "The corner radius (%.3f) is more than half the width (%.3f); using half the width, "
                "i.e. a full radius" % (float(self.corner_radius), w))
        r = max(0.0, min(float(self.corner_radius), w / 2))
        dz = self.resolution

        # Z range of the reference point (the chuck-side flank): the whole
        # blade stays inside the window, so it never touches the part outside
        # the Z range the user chose. Toward the tailstock the limit is the
        # window start, or the stock front when the window starts in front
        # of the stock.
        top = self.z_start() if self.starts_in_stock() else self.stock.z_max
        z_lo = self.z_end()
        z_hi = top - w
        if z_hi < z_lo - 1e-9:
            raise ValueError("The Z range (%.3f) is narrower than the grooving tool (%.3f)" % (top - z_lo, w))

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

        sx, sz = self.stock_xz()

        def drop(allowance):
            # profile grown by the stock to leave plus the allowance: a
            # rectangle, sx radially and sz along Z, so X stock lands on
            # diameters and Z stock on walls and faces
            prof = _dilate_rect(profile, sx + allowance, sz + allowance, dz)
            x0 = np.full(len(z_ref), -np.inf)
            for k in range(m + 1):
                x0 = np.maximum(x0, prof.at(z_ref + u[k]) - edge[k])
            # one cell either side, so straight moves between grid points
            # cannot dip into a steep wall
            guard = np.maximum(x0, np.maximum(np.roll(x0, 1), np.roll(x0, -1)))
            guard[0] = max(x0[0], x0[1]) if len(x0) > 1 else x0[0]
            guard[-1] = max(x0[-1], x0[-2]) if len(x0) > 1 else x0[-1]
            return guard

        final = drop(0.0)
        deep = final < r_min - 1e-9
        if deep.any():
            self.warnings.append(
                "The part is deeper than the tool can reach: material is left between Z %.3f and "
                "Z %.3f. The grooving depth is measured from the max diameter (default: the stock); "
                "if the surface has already been turned smaller, set Max Diameter to that diameter. "
                "A Min Diameter also limits the depth." % (z_ref[deep].min(), z_ref[deep].max()))
        final = np.maximum(final, r_min)

        passes = max(0, int(self.finish_passes))
        allowance = self.finish_allowance if passes else 0.0
        if passes and self.turning_depth > 0 and allowance > self.turning_depth + 1e-9:
            self.warnings.append(
                "The finish allowance (%.3f) is more than the tool's turning depth (%.3f)"
                % (allowance, self.turning_depth))
        rough = np.maximum(drop(allowance), r_min) if allowance > 0 else final

        # roughing plunges, front to back, across each stretch where the blade
        # can actually go down (leaving the allowance): from one end of the
        # stretch to the other, at most `spacing` apart (and never further
        # than 80% of the blade width, so neighbours overlap). Positions
        # fixed from the ends of the Z range miss the stretches next to
        # walls, where the allowance blocks the blade at the range ends.
        spacing = min(self.step_over if self.step_over > 0 else w, 0.8 * w)
        # where a plunge actually reaches the profile (to within the finish
        # allowance), not just dips below the surface: a blade blocked by a
        # wall still dips a little as its corner rolls over the edge
        open_ = (rough < r_outer - 1e-3) & (rough <= final + allowance + 0.05)
        z_asc, rough_asc = z_ref[::-1], rough[::-1]
        for a, b in _runs(open_):
            za, zb = z_ref[a], z_ref[b]
            n = int(math.ceil((za - zb) / spacing - 1e-9)) if za > zb else 0
            for j in range(n + 1):
                z0 = za - (za - zb) * j / n if n else za
                self.plunges.append((z0, float(np.interp(z0, z_asc, rough_asc))))

        # finish passes: the outermost first, down to the final profile;
        # finish_step_over apart when set, otherwise the allowance in equal steps
        offsets = []
        for k in range(passes):
            n_from_final = passes - 1 - k
            if self.finish_step_over and self.finish_step_over > 0:
                offsets.append(min(n_from_final * self.finish_step_over, allowance))
            else:
                offsets.append(allowance * n_from_final / passes)
        targets = [final if off <= 0 else np.maximum(drop(off), r_min) for off in offsets]

        # Respect the insert's turning depth. Track the material the plunges
        # leave, then follow each finish pass in order: wherever it would
        # cut sideways deeper than the turning depth (e.g. where no plunge
        # could reach), put side-turning passes in front of it that step down
        # by at most the turning depth each.
        geo = _Blade(z_lo, z_hi, w, dz, u, edge, r_outer)
        for z0, floor in self.plunges:
            geo.cut_position(floor, z0)
        t_depth = float(self.turning_depth or 0.0)
        steps_added = 0
        for target in targets:
            if t_depth > 0 and geo.side_cut(_halves(target, z_ref)) > t_depth + 1e-6:
                touch = geo.touch(z_ref)
                for m in range(1, 1000):
                    level = np.maximum(target, touch - m * t_depth)
                    if np.all(level <= target + 1e-9):
                        break
                    self._add_finish(level, z_ref)
                    geo.follow(_halves(level, z_ref))
                    steps_added += 1
            self._add_finish(target, z_ref)
            geo.follow(_halves(target, z_ref))
        if steps_added:
            self.warnings.append(
                "Added %d side-turning passes, at most the turning depth (%.3f) deep, where the plunges "
                "could not rough the groove" % (steps_added, t_depth))

        # spring pass: the last finish pass (both of its halves) once more
        if self.spring_pass and self.finish_paths:
            self.finish_paths.extend(self.finish_paths[-2:])

    def _add_finish(self, prof, z_ref):
        """
        Add a finish pass as two paths that meet at its deepest point: from
        the front down the first wall and along the bottom, then from the
        back down the other wall and along the bottom the other way. Each
        wall is finished moving straight down it, and the bottom is crossed
        moving away from the walls, so the blade never moves sideways into
        the stock left on a wall.
        """
        for xs, zs in _halves(prof, z_ref):
            self.finish_paths.append(_simplify(list(zip(xs.tolist(), zs.tolist())), 0.0005))

    def _profile_radius(self, z_lo, z_hi, dz):
        """Largest part radius at each Z on a grid (None where there is no part)."""
        pad = max(max(self.stock_xz()) + self.finish_allowance, 0) + float(self.groove_width) + 2 * dz
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


def _runs(mask):
    """(first, last) index pairs of the runs of True in a boolean array."""
    runs, start = [], None
    for i, v in enumerate(mask):
        if v and start is None:
            start = i
        elif not v and start is not None:
            runs.append((start, i - 1)); start = None
    if start is not None:
        runs.append((start, len(mask) - 1))
    return runs


def _halves(prof, z_ref):
    """
    A pass split at its deepest point (the middle of the deepest stretch)
    into two paths in cutting order: front to that point, and back to it.
    """
    deepest = prof.min()
    low = np.where(prof <= deepest + 1e-3)[0]
    split = int(low[len(low) // 2])
    front = (prof[:split + 1], z_ref[:split + 1])
    back = (prof[split:][::-1], z_ref[split:][::-1])
    return [front, back]


class _Blade:
    """
    The material left in the Z range as the blade cuts, on a Z grid: a
    radius per Z, starting at the outer limit. Used to check how deep each
    pass cuts sideways.
    """

    def __init__(self, z_lo, z_hi, w, dz, u, edge, r_outer):
        self.z0, self.dz = z_lo, dz
        self.n = int(math.ceil((z_hi + w - z_lo) / dz)) + 2
        self.material = np.full(self.n, float(r_outer))
        self.offsets = np.rint(np.asarray(u) / dz).astype(int)
        self.edge = np.asarray(edge)

    def _index(self, z0):
        return np.rint((np.asarray(z0) - self.z0) / self.dz).astype(int)

    def cut_position(self, x0, z0):
        """The blade at one position removes everything above its edge."""
        idx = int(self._index(z0)) + self.offsets
        ok = (idx >= 0) & (idx < self.n)
        self.material[idx[ok]] = np.minimum(self.material[idx[ok]], x0 + self.edge[ok])

    def touch(self, z_ref):
        """Reference X at which the blade meets the material, at each position."""
        base = self._index(z_ref)
        out = np.full(len(z_ref), -np.inf)
        for k, off in enumerate(self.offsets):
            idx = base + off
            ok = (idx >= 0) & (idx < self.n)
            out[ok] = np.maximum(out[ok], self.material[idx[ok]] - self.edge[k])
        return out

    def side_cut(self, paths):
        """
        Deepest radial cut while the blade moves sideways along the paths,
        following them in order so that material removed earlier (e.g. by
        going down a wall) is gone when the blade gets further. Moving
        straight down is plunging and does not count.
        """
        saved = self.material.copy()
        worst = 0.0
        for prof, z_ref in paths:
            prev = None
            for x0, z0 in zip(prof, z_ref):
                if prev is not None and abs(x0 - prev) <= 2 * self.dz:
                    idx = int(self._index(z0)) + self.offsets
                    ok = (idx >= 0) & (idx < self.n)
                    if ok.any():
                        worst = max(worst, float(np.max(self.material[idx[ok]] - self.edge[ok])) - x0)
                self.cut_position(x0, z0)
                prev = x0
        self.material = saved
        return worst

    def follow(self, paths):
        """Remove the material along the paths."""
        for prof, z_ref in paths:
            for x0, z0 in zip(prof, z_ref):
                self.cut_position(x0, z0)


def _dilate_rect(profile, sx, sz, dz):
    """Grow the profile by a rectangle: sz either way along Z, then sx radially."""
    vals = profile.values
    out = vals.copy()
    for k in range(1, int(math.ceil(max(sz, 0.0) / dz - 1e-9)) + 1):
        out[k:] = np.maximum(out[k:], vals[:-k])
        out[:-k] = np.maximum(out[:-k], vals[k:])
    return _Profile(profile.start, dz, out + max(sx, 0.0))


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
