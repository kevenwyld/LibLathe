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
    # finish each pass as two halves meeting at the deepest point (down each
    # wall toward the middle) instead of one pass in, across and out
    finish_two_sided = False

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

        def guard(x0):
            # one cell either side, so straight moves between grid points
            # cannot dip into a steep wall
            out = np.maximum(x0, np.maximum(np.roll(x0, 1), np.roll(x0, -1)))
            out[0] = max(x0[0], x0[1]) if len(x0) > 1 else x0[0]
            out[-1] = max(x0[-1], x0[-2]) if len(x0) > 1 else x0[-1]
            return out

        def drop(allowance, guarded=True):
            # profile grown by the stock to leave plus the allowance: a
            # rectangle, sx radially and sz along Z, so X stock lands on
            # diameters and Z stock on walls and faces
            prof = _dilate_rect(profile, sx + allowance, sz + allowance, dz)
            x0 = np.full(len(z_ref), -np.inf)
            # the two flank lines (u = 0 and u = W) are the blade's boundary:
            # part there is touching the flank (e.g. a wall the blade goes
            # down flush with), not inside the blade
            for k in range(1, m):
                x0 = np.maximum(x0, prof.at(z_ref + u[k]) - edge[k])
            return guard(x0) if guarded else x0

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
        # unguarded here: the guard is applied within each pass's own range
        # below, so positions outside it do not lift its ends
        final_raw = np.maximum(drop(0.0, guarded=False), r_min)
        targets = [final_raw if off <= 0 else np.maximum(drop(off, guarded=False), r_min) for off in offsets]
        # Each pass works only within its own Z range: the range shrunk by
        # the Z stock it leaves. Otherwise, at a range end next to a wall, the
        # blade overlaps the stock the pass leaves on that wall and rolls its
        # corner over the edge of that stock (cutting a rounded edge into it)
        # before it can go down. Shrunk, it goes straight down at its own
        # distance from the wall.
        windows = []
        for off in offsets:
            keep = (z_ref <= z_hi - (sz + off) + 1e-9) & (z_ref >= z_lo + (sz + off) - 1e-9)
            if not keep.any():
                keep = np.zeros(len(z_ref), bool); keep[len(z_ref) // 2] = True
            windows.append(keep)

        # Respect the insert's turning depth. Track the material the plunges
        # leave; wherever a finish pass would take a cut more than the
        # turning depth deep over more than the turning depth of length
        # (where no plunge could reach), put side-turning passes in front of
        # it that step down by at most 90% of the turning depth each, re-measured
        # from the material left, over just the stretches that need it. A thin layer,
        # such as the finish stock on a wall or the bottom, is fine: that is
        # a light cut whichever side of the blade takes it.
        geo = _Blade(z_lo, z_hi, w, dz, u, edge, r_outer)
        for z0, floor in self.plunges:
            geo.cut_position(floor, z0)
        t_depth = float(self.turning_depth or 0.0)
        steps_added = 0
        for target, keep in zip(targets, windows):
            target, zs = guard(target[keep]), z_ref[keep]
            # step down from the material actually left, only over the
            # stretches that need it, until the pass is within the limit
            for _ in range(1000):
                if not (t_depth > 0 and geo.overload(target, zs, t_depth)):
                    break
                level = np.maximum(target, geo.touch(zs) - 0.9 * t_depth)
                need = level > target + 1e-6
                if not need.any():
                    break
                for a, b in _runs(need):
                    self.finish_paths.append(_simplify(list(zip(level[a:b + 1].tolist(), zs[a:b + 1].tolist())), 0.0005))
                    steps_added += 1
                geo.follow(level, zs)
            self._add_finish(target, zs)
            geo.follow(target, zs)
        if steps_added:
            self.warnings.append(
                "Added %d side-turning passes, each at most the turning depth (%.3f) deep, where the plunges "
                "left more than the insert can take sideways" % (steps_added, t_depth))

        # spring pass: the last finish pass (both halves, if two-sided) once more
        if self.spring_pass and self.finish_paths:
            self.finish_paths.extend(self.finish_paths[-(2 if self.finish_two_sided else 1):])

    def _add_finish(self, prof, z_ref):
        """
        Add a finish pass: by default one path in, across and out (down the
        front wall, along the bottom, up the back wall), which gives the
        best finish; with finish_two_sided, two paths meeting at the deepest
        point (down each wall toward the middle).
        """
        parts = _halves(prof, z_ref) if self.finish_two_sided else [(prof, z_ref)]
        for xs, zs in parts:
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
        # passes go straight in and straight out: a pass may end down beside a
        # wall, and the move out is still cutting (the wall's stock), so feed
        # out until clear of the diameter, then rapid
        r_clear = min(-self.x_outer() + 0.5, r_retract)
        for pts in self.finish_paths:
            if not pts:
                continue
            x, z = pts[0]
            cmds.append(Command('G0', {'X': r_retract, 'Z': z}))
            cmds.append(Command('G1', {'X': x, 'Z': z, 'F': self.vfeed}))
            for x, z in pts[1:]:
                cmds.append(Command('G1', {'X': x, 'Z': z, 'F': self.hfeed}))
            x, z = pts[-1]
            if x < r_clear:
                cmds.append(Command('G1', {'X': r_clear, 'Z': z, 'F': self.vfeed}))
            cmds.append(Command('G0', {'X': r_retract, 'Z': z}))
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

    def removed(self, prof, z_ref):
        """Depth of material a path would remove, at each Z."""
        before = self.material.copy()
        self.follow(prof, z_ref)
        depth = before - self.material
        self.material = before
        return depth

    def overload(self, prof, z_ref, limit):
        """
        True when a path would remove a cut more than `limit` deep over more
        than `limit` of length along Z: more than the insert takes sideways.
        A thin layer (a wall's finish stock, the bottom's allowance) is not.
        """
        deep = self.removed(prof, z_ref) > limit + 1e-6
        longest = max((b - a + 1 for a, b in _runs(deep)), default=0)
        return longest * self.dz > limit + 1e-6

    def follow(self, prof, z_ref):
        """Remove the material along a path."""
        for x0, z0 in zip(prof, z_ref):
            self.cut_position(x0, z0)


def _dilate_rect(profile, sx, sz, dz):
    """
    Grow the profile by a rectangle: sz either way along Z, then sx
    radially. Negative values shrink it (a negative stock to leave, cutting
    under the model's size): along Z the smallest neighbour rather than the
    largest, and radii stop at the axis.
    """
    vals = profile.values
    out = vals.copy()
    pick = np.maximum if sz >= 0 else np.minimum
    for k in range(1, int(math.ceil(abs(sz) / dz - 1e-9)) + 1):
        out[k:] = pick(out[k:], vals[:-k])
        out[:-k] = pick(out[:-k], vals[k:])
    out = out + sx
    if sx < 0 or sz < 0:
        finite = np.isfinite(out)
        out[finite] = np.maximum(out[finite], 0.0)
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
