import math

import numpy as np

import liblathe.op.base
from liblathe.base.command import Command
from liblathe.base.geometry import dilate_rect_profile, sample_points, simplify_polyline


class GrooveOP(liblathe.op.base.BaseOP):
    """
    Grooving with a blade of width W and corner radius R, programmed to the edge's lowest point (X) and
    its chuck-side flank (Z). Drop-cutting finds the lowest X0 per Z0 clear of the part; roughing plunges,
    then finish passes follow that profile. Cutting depth is measured from x_outer(), where plunges start.
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
    # roughing: "plunge" (side by side, full depth) or "side_turn" (levels:
    # a plunge of one step down, then a traverse along Z at that depth,
    # alternating direction, less tool pressure in wide grooves)
    strategy = "plunge"
    # side_turn: depth of each level; 0: 90% of the turning depth (half the
    # blade width without one)
    step_down = 0.0
    # pecking on plunges: peck_depth > 0 feeds in that far at a time, backing
    # off peck_retract between pecks (or out of the cut, with peck_full_retract)
    peck_depth = 0.0
    peck_retract = 0.2
    peck_full_retract = False

    def generate_path(self):
        self.warnings = []
        self.plunges = []
        self.rough_paths = []
        self.finish_paths = []

        w = float(self.groove_width)
        if w <= 0:
            raise ValueError("The grooving insert's width must be above zero")
        if float(self.corner_radius) > w / 2 + 1e-9:
            self.warnings.append(
                "Corner radius %.3f is more than half the width (%.3f): a full radius is used" % (float(self.corner_radius), w))
        r = max(0.0, min(float(self.corner_radius), w / 2))
        dz = self.resolution

        # Z range of the reference point: the whole blade stays inside the window
        top = self.z_start() if self.starts_in_stock() else self.stock.z_max
        z_lo = self.z_end()
        z_hi = top - w
        if z_hi < z_lo - 1e-9:
            self.refuse("groove-narrower-than-blade",
                        "The Z range (%.3f) is narrower than the insert (%.3f)" % (top - z_lo, w))
            # waived: one plunge, the blade's width from the back of the range
            z_hi = z_lo

        r_outer = -self.x_outer()            # stock radius (or max diameter / 2)
        # lowest the reference may go: the min diameter, else the corner radius past the axis,
        # and never deeper than the cutting depth below the stock
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
            prof = _Profile(profile.start, dz, dilate_rect_profile(profile.values, sx + allowance, sz + allowance, dz))
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
                "Material left between Z %.3f and Z %.3f: deeper than the insert's grooving depth from Max "
                "Diameter. If the surface was turned smaller already, set Max Diameter to it"
                % (z_ref[deep].min(), z_ref[deep].max()))
        final = np.maximum(final, r_min)

        passes = max(0, int(self.finish_passes))
        allowance = self.finish_allowance if passes else 0.0
        if passes and self.turning_depth > 0 and allowance > self.turning_depth + 1e-9:
            self.warnings.append(
                "Finish allowance %.3f is more than the insert's turning depth %.3f" % (allowance, self.turning_depth))
        rough = np.maximum(drop(allowance), r_min) if allowance > 0 else final

        # plunges across each stretch the blade can go down, at most `spacing` apart
        # and never more than 80% of the width, so neighbours overlap
        spacing = min(self.step_over if self.step_over > 0 else w, 0.8 * w)
        # where a plunge actually reaches the profile (to within the finish
        # allowance), not just dips below the surface: a blade blocked by a
        # wall still dips a little as its corner rolls over the edge
        open_ = (rough < r_outer - 1e-3) & (rough <= final + allowance + 0.05)
        z_asc, rough_asc = z_ref[::-1], rough[::-1]
        geo = _Blade(z_lo, z_hi, w, dz, u, edge, r_outer)
        t_depth = float(self.turning_depth or 0.0)
        if self.strategy == "side_turn":
            self._side_turn(rough, z_ref, r_outer, w, t_depth, geo)
            # what the traverses could not reach (a groove the blade
            # practically fills): plunged as usual, and said so
            open_ = open_ & (geo.touch(z_ref) > rough + allowance + 0.05)
            # the corner pockets the traverses leave by the walls (within a
            # corner radius) are the finish's: not plunged, not reported
            reach = float(self.corner_radius or 0.0) + 2 * dz
            for a, b in _runs(open_):
                if z_ref[a] - z_ref[b] <= reach + 1e-9:
                    open_[a:b + 1] = False
            for a, b in _runs(open_):
                self.warnings.append(
                    "Side-turn: Z %.3f to %.3f plunged instead, the groove is barely wider than the insert (%.3f)"
                    % (z_ref[a] + w, z_ref[b], w))
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
        # each pass works within the range shrunk by the Z stock it leaves, so by a wall
        # it goes straight down instead of rolling its corner over that stock
        windows = []
        for off in offsets:
            keep = (z_ref <= z_hi - (sz + off) + 1e-9) & (z_ref >= z_lo + (sz + off) - 1e-9)
            if not keep.any():
                keep = np.zeros(len(z_ref), bool); keep[len(z_ref) // 2] = True
            windows.append(keep)

        # where a finish pass would cut deeper than the turning depth over more than its length,
        # side-turning passes stepping down by at most 90% of it go first (see _Blade.overload)
        for z0, floor in self.plunges:
            geo.cut_position(floor, z0)
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
                    self.finish_paths.append(simplify_polyline(list(zip(level[a:b + 1].tolist(), zs[a:b + 1].tolist())), 0.0005))
                    steps_added += 1
                geo.follow(level, zs)
            self._add_finish(target, zs)
            geo.follow(target, zs)
        if steps_added:
            self.warnings.append(
                "%d side-turning passes added where the plunges left more than the turning depth (%.3f)"
                % (steps_added, t_depth))

        # spring pass: the last finish pass (both halves, if two-sided) once more
        if self.spring_pass and self.finish_paths:
            self.finish_paths.extend(self.finish_paths[-(2 if self.finish_two_sided else 1):])

    def _side_turn(self, rough, z_ref, r_outer, w, t_depth, geo):
        """
        Roughing in levels of step_down: a traverse along Z at each level over every stretch with
        material, alternating direction. Stretches under SIDE_TURN_MIN blade widths are left to plunges.
        """
        step = float(self.step_down) if self.step_down and self.step_down > 0 else (
            0.9 * t_depth if t_depth > 0 else 0.5 * w)
        if t_depth > 0 and step > t_depth + 1e-9:
            self.warnings.append("Step down %.3f is more than the insert's turning depth %.3f" % (step, t_depth))
        floor = float(np.min(rough))
        corner = float(self.corner_radius or 0.0)
        dz = abs(z_ref[0] - z_ref[1]) if len(z_ref) > 1 else 0.0
        level, forward, k = r_outer, True, 0
        while level > floor + 1e-6 and k < 10000:
            k += 1
            level = max(level - step, floor)
            path = np.maximum(rough, level)
            cuts = path < geo.touch(z_ref) - 1e-6
            runs = [(a, b) for a, b in _runs(cuts) if z_ref[a] - z_ref[b] >= SIDE_TURN_MIN * w - 1e-9]
            if not forward:
                runs = runs[::-1]
            for a, b in runs:
                # within a corner radius of a wall the floor rises along the corner: the traverse
                # starts and ends where it is down at its level (the finish does the walls)
                reach = corner + 2 * dz
                a0, b0 = a, b
                while a < b and path[a] > level + 1e-6 and z_ref[a0] - z_ref[a] <= reach:
                    a += 1
                while b > a and path[b] > level + 1e-6 and z_ref[b] - z_ref[b0] <= reach:
                    b -= 1
                if z_ref[a] - z_ref[b] < SIDE_TURN_MIN * w - 1e-9:
                    continue
                xs, zs = path[a:b + 1], z_ref[a:b + 1]
                pts = list(zip(xs.tolist(), zs.tolist()))
                self.rough_paths.append(simplify_polyline(pts if forward else pts[::-1], 0.0005))
                geo.follow(xs, zs)
            forward = not forward

    def _plunge(self, cmds, z0, floor, r_retract, r_top):
        """A plunge at z0 from the retract radius to floor: in one feed move,
        or pecking from the material's top (r_top) by peck_depth."""
        cmds.append(Command('G0', {'X': r_retract, 'Z': z0}))
        peck = float(self.peck_depth or 0.0)
        if peck > 0:
            x = min(r_top, r_retract)
            while x - peck > floor + 1e-6:
                x -= peck
                cmds.append(Command('G1', {'X': x, 'Z': z0, 'F': self.vfeed}))
                back = r_retract if self.peck_full_retract else min(x + float(self.peck_retract), r_retract)
                cmds.append(Command('G0', {'X': back, 'Z': z0}))
                if self.peck_full_retract:
                    # back in at rapid to just short of where it reached
                    cmds.append(Command('G0', {'X': min(x + float(self.peck_retract), r_retract), 'Z': z0}))
        cmds.append(Command('G1', {'X': floor, 'Z': z0, 'F': self.vfeed}))
        cmds.append(Command('G0', {'X': r_retract, 'Z': z0}))

    def _add_finish(self, prof, z_ref):
        """A finish pass: in, across and out; or with finish_two_sided two halves meeting at the deepest point."""
        parts = _halves(prof, z_ref) if self.finish_two_sided else [(prof, z_ref)]
        for xs, zs in parts:
            self.finish_paths.append(simplify_polyline(list(zip(xs.tolist(), zs.tolist())), 0.0005))

    def _profile_radius(self, z_lo, z_hi, dz):
        """Largest part radius at each Z on a grid (None where there is no part)."""
        pad = max(max(self.stock_xz()) + self.finish_allowance, 0) + float(self.groove_width) + 2 * dz
        start = z_lo - pad
        n = int(math.ceil((z_hi + pad - start) / dz)) + 1
        values = np.full(n, -np.inf)
        step = dz / 2.5
        for seg in self.part_segment_group.get_segments():
            for x, z in sample_points(seg, step):
                i = int(round((z - start) / dz))
                if 0 <= i < n:
                    values[i] = max(values[i], -x)
        return _Profile(start, dz, values)

    def generate_gcode(self):
        """Plunges then finish passes, as commands with a positive radius X."""
        cmds = [Command('G18', {})]
        r_retract = -self.x_outer() + self.radial_clearance()
        r_top = -self.x_outer()
        for z0, floor in self.plunges:
            self._plunge(cmds, z0, floor, r_retract, r_top)
        # passes go straight in and straight out: a pass may end down beside a
        # wall, and the move out is still cutting (the wall's stock), so feed
        # out until clear of the diameter, then rapid
        r_clear = min(-self.x_outer() + 0.5, r_retract)
        # side-turning levels: a level that starts where the last one ended
        # (alternating direction) steps straight down instead of retracting
        last, last_span = None, None
        for i, pts in enumerate(self.rough_paths):
            if not pts:
                continue
            x, z = pts[0]
            span = (min(p[1] for p in pts), max(p[1] for p in pts))
            deeper = last is not None and x <= last[0] + 1e-9
            if deeper and abs(last[1] - z) < 1e-6:
                cmds.append(Command('G1', {'X': x, 'Z': z, 'F': self.vfeed}))
            elif deeper and last_span[0] - 1e-6 <= z <= last_span[1] + 1e-6:
                # starts within what the last level cut: along it (air) to
                # here, then down
                cmds.append(Command('G1', {'X': last[0], 'Z': z, 'F': self.hfeed}))
                cmds.append(Command('G1', {'X': x, 'Z': z, 'F': self.vfeed}))
            elif deeper and span[0] - 1e-6 <= last[1] <= span[1] + 1e-6:
                # a wider level: down where the last ended, out along this
                # level (its own material) to where it starts
                cmds.append(Command('G1', {'X': x, 'Z': last[1], 'F': self.vfeed}))
                cmds.append(Command('G1', {'X': x, 'Z': z, 'F': self.hfeed}))
            else:
                if last is not None:
                    if last[0] < r_clear:
                        cmds.append(Command('G1', {'X': r_clear, 'Z': last[1], 'F': self.vfeed}))
                    cmds.append(Command('G0', {'X': r_retract, 'Z': last[1]}))
                cmds.append(Command('G0', {'X': r_retract, 'Z': z}))
                cmds.append(Command('G1', {'X': x, 'Z': z, 'F': self.vfeed}))
            for x, z in pts[1:]:
                cmds.append(Command('G1', {'X': x, 'Z': z, 'F': self.hfeed}))
            last, last_span = pts[-1], span
        if last is not None:
            if last[0] < r_clear:
                cmds.append(Command('G1', {'X': r_clear, 'Z': last[1], 'F': self.vfeed}))
            cmds.append(Command('G0', {'X': r_retract, 'Z': last[1]}))
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


# Side-turn traverses a stretch at least this many blade widths long (the
# groove at least 1.1 blade widths wide); shorter ones are plunged
SIDE_TURN_MIN = 0.1


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
