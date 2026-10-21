import math

import liblathe.op.base
from liblathe.base.command import Command

# Recommended number of infeed passes by pitch (mm) for external 60 degree
# threads (typical insert makers' tables); interpolated in between.
_PASSES_BY_PITCH = ((0.5, 4), (0.75, 4), (1.0, 5), (1.25, 6), (1.5, 6), (1.75, 8), (2.0, 8),
                    (2.5, 10), (3.0, 12), (3.5, 12), (4.0, 14), (5.0, 16), (6.0, 18))


def recommended_passes(pitch):
    """Number of infeed passes to the full depth for a pitch, from the table above."""
    table = _PASSES_BY_PITCH
    if pitch <= table[0][0]:
        return table[0][1]
    for (p1, n1), (p2, n2) in zip(table[:-1], table[1:]):
        if pitch <= p2:
            return int(round(n1 + (n2 - n1) * (pitch - p1) / (p2 - p1)))
    return table[-1][1]


class ThreadOP(liblathe.op.base.BaseOP):
    """
    Single-point threading along Z, from the window start (front) toward the
    window end (back, toward the chuck).

    The tool is programmed to the threading insert's tip (the point of its
    profile): X at the tip, Z at the tip. The thread starts from
    `thread_radius`, the surface it is cut into (the major radius of an
    external thread, the bore's radius for an internal one), and goes
    `thread_depth` deep (radially).

    `passes` sets how many infeed passes reach the full depth (0: the
    recommended number for the pitch), unless first_depth is given.
    Pass n is first_depth * n ** (1 / degression) deep: degression 1.5 (the
    default) is a gentle schedule with a light first pass, 2 constant area
    (the first pass deepest), 1 constant depth; never less than min_depth deeper than the pass
    before, the last at the full depth, followed by spring_passes at the full
    depth. Each pass is shifted along the cut (toward the chuck) by its depth
    times tan(infeed_angle), so the leading flank does the cutting (29.5 is
    the modified flank infeed, 30 the flank, 0 radial). Every pass starts
    lead_in in front of the thread, so Z is at speed when it reaches it.

    Output: explicit passes (rapid to the start, rapid to depth, G33 along Z
    at the pitch, rapid out, back to the start), or, with `canned`, one
    LinuxCNC G76 cycle with the same values.
    """

    needs_tool = False

    pitch = 1.0
    thread_depth = 0.0
    thread_radius = 0.0
    internal = False
    passes = 0               # infeed passes to full depth; 0: recommended for the pitch
    first_depth = 0.0        # 0: from the number of passes
    min_depth = 0.05
    degression = 1.5
    infeed_angle = 29.5
    spring_passes = 1
    lead_in = 0.0            # 0: three pitches
    thread_length = 0.0      # 0: the whole window, from its start
    # run-out: the last run_out of the thread (along Z) tapers out to the
    # crest instead of stopping dead at full depth (G76's E, with L2); 0:
    # stop dead (into a relief groove)
    run_out = 0.0
    canned = False
    # the insert's profile, to keep it clear of the part at both ends
    tool_angle = 60.0
    tool_width = 0.0

    def generate_path(self):
        self.warnings = []
        if self.pitch <= 0:
            raise ValueError("The thread pitch must be greater than zero")
        if self.thread_depth <= 0:
            raise ValueError("The thread depth must be greater than zero")
        if self.thread_radius <= 0:
            raise ValueError("The thread diameter must be greater than zero")
        if self.internal and self.thread_depth >= self.thread_radius * 2:
            raise ValueError("The thread is deeper than the bore is wide")

        depth = float(self.thread_depth)
        self.z_front = self.z_start()
        self.z_back = self.z_end()
        if self.thread_length and self.thread_length > 0:
            self.z_back = max(self.z_back, self.z_front - self.thread_length)
        self.run_out = max(0.0, float(self.run_out or 0.0))
        if self.run_out >= self.z_front - self.z_back - 1e-9:
            raise ValueError("The run-out (%.3f) is as long as the thread (%.3f) or longer"
                             % (self.run_out, self.z_front - self.z_back))
        # where the full-depth thread ends: the run-out tapers out from here
        self.z_full = self.z_back + self.run_out
        lead = self.lead_in if self.lead_in > 0 else 3 * self.pitch
        self.z_drive = self.z_front + lead
        sign = 1.0 if self.internal else -1.0       # deeper is outward for internal threads

        n_passes = int(self.passes) if self.passes and self.passes > 0 else recommended_passes(self.pitch)
        first = self.first_depth if self.first_depth > 0 else depth / n_passes ** (1.0 / self.degression)
        self.first = min(first, depth)
        depths = []
        n = 1
        while True:
            d = min(self.first * n ** (1.0 / self.degression), depth)
            if depths and d - depths[-1] < self.min_depth:
                d = min(depths[-1] + self.min_depth, depth)
            depths.append(d)
            if d >= depth - 1e-9:
                break
            n += 1
            if n > 1000:
                raise ValueError("Too many threading passes: check the first and minimum pass depths")
        depths += [depth] * max(0, int(self.spring_passes))
        self.depths = depths

        shift = math.tan(math.radians(self.infeed_angle))
        self.passes = []
        for d in depths:
            self.passes.append((self.thread_radius + sign * d, self.z_drive - d * shift))

        # where the tool waits between passes: clear of the surface it cuts
        if self.internal:
            self.r_clear = max(self.thread_radius - min(self.clearance, 1.0), self.thread_radius * 0.5)
        else:
            self.r_clear = self.thread_radius + min(self.clearance, 1.0)

        if not self.internal:
            self._check_clearance(depth)

    def _check_clearance(self, depth):
        """
        Keep the insert clear of the part outside the thread (external
        threads). Only the part is checked: where it is held (the chuck, a
        collet, a fixture) is up to the operator, not assumed from the stock. In front of the thread every pass runs in from the lead-in
        at depth, so nothing there may stand above the deepest pass's tip.
        Behind the thread's end the insert's chuck-side flank stops: its
        profile (tool_angle) rises from the tip up to its half width
        (tool_width), then it is straight, so the part there must stay under
        that flank. Without a width only the tip is checked.
        """
        tip = self.thread_radius - depth                # the deepest pass's tip
        half = self.tool_width / 2 if self.tool_width > 0 else 0.0
        slope = math.tan(math.radians(self.tool_angle / 2)) if self.tool_angle > 0 else 0.0
        front, back = self.z_front, self.z_back
        latest_end = None        # the highest Z the thread must end at to clear what is behind it
        proud_front = None       # the part above the root in front of the thread
        for seg in self.part_segment_group.get_segments():
            for x, z in _sample(seg, 0.05):
                r = -x
                if r <= tip + 1e-3:
                    continue
                if front + 1e-6 < z <= self.z_drive + half:
                    proud_front = z if proud_front is None else max(proud_front, z)
                    continue
                # behind the end, the insert's flank cutting into the thread's
                # own diameter is just how the thread ends; only part standing
                # proud of that diameter (a shoulder) can be hit
                # (a micron's tolerance: the outline comes from a section of the
                # solid, and its corners are not exactly on the thread's radius)
                if half > 0 and back - half - 1e-6 <= z <= back + 1e-6 and r > self.thread_radius + 1e-3:
                    # the flank is (r - tip) * slope behind the tip at height r
                    end = z + min((r - tip) * slope, half)
                    if end > back + 1e-6 and (latest_end is None or end > latest_end):
                        latest_end = end
        if proud_front is not None:
            raise ValueError(
                "In front of the thread the part stands above the thread's root (radius %.3f) from Z %.3f to "
                "Z %.3f. Every pass starts in front of the thread at its depth and would cut into it, whatever "
                "the lead-in: a single-point thread needs the part in front of it below the root (a chamfer or "
                "a relief groove)" % (tip, front, proud_front))
        if latest_end is not None:
            # round the suggestion the safe way (up), so using it clears
            latest_end = math.ceil(latest_end * 1000 - 1e-6) / 1000
            raise ValueError(
                "The insert would hit the part behind the end of the thread (at Z %.3f): end the thread at "
                "Z %.3f or above (a thread length of at most %.3f), or cut a relief groove there"
                % (back, latest_end, front - latest_end))

    def _max_radius_between(self, z1, z2):
        """Largest part radius between two Z values (None where there is no part)."""
        worst = None
        lo, hi = min(z1, z2), max(z1, z2)
        for seg in self.part_segment_group.get_segments():
            for p in (seg.start, seg.end):
                if lo + 1e-6 < p.Z <= hi:
                    worst = -p.X if worst is None else max(worst, -p.X)
            # a segment crossing the range at its ends
            for z in (lo, hi):
                if min(seg.start.Z, seg.end.Z) < z < max(seg.start.Z, seg.end.Z) and seg.bulge == 0:
                    t = (z - seg.start.Z) / (seg.end.Z - seg.start.Z)
                    r = -(seg.start.X + t * (seg.end.X - seg.start.X))
                    if z > lo + 1e-6:
                        worst = r if worst is None else max(worst, r)
        return worst

    def generate_gcode(self):
        cmds = [Command('G18', {})]
        if self.canned:
            # LinuxCNC G76: the drive line is the current X; I is the thread
            # peak's offset from it (negative for external threads)
            peak_offset = self.thread_radius - self.r_clear
            cmds.append(Command('G0', {'X': self.r_clear, 'Z': self.z_drive}))
            params = {'P': self.pitch, 'Z': self.z_back, 'I': peak_offset,
                      'J': self.first, 'K': self.thread_depth, 'R': self.degression,
                      'Q': self.infeed_angle, 'H': max(0, int(self.spring_passes))}
            if self.run_out > 0:
                # a taper at the exit (L2) over the run-out's length (E)
                params.update({'E': self.run_out, 'L': 2})
            cmds.append(Command('G76', params))
            return cmds
        for x, z in self.passes:
            cmds.append(Command('G0', {'X': self.r_clear, 'Z': z}))
            cmds.append(Command('G0', {'X': x, 'Z': z}))
            # FreeCAD's path display skips G33 (and keeps the tool where the
            # pass started), so it would draw the retract below from the pass's
            # start as a long tapered line. A G1 to where each G33 ended is a
            # zero-length move on the machine, and in the display it draws
            # the thread cut itself.
            # (the move goes nowhere, so any feed will do; one is needed, as a
            # control refuses G1 without a feed rate, and threading has none)
            feed = self.hfeed if self.hfeed and self.hfeed > 0 else 1.0
            cmds.append(Command('G33', {'X': x, 'Z': self.z_full, 'K': self.pitch}))
            cmds.append(Command('G1', {'X': x, 'Z': self.z_full, 'F': feed}))
            x_end = x
            if self.run_out > 0:
                # the run-out: out to the crest by the thread's end, still
                # synchronised with the spindle
                x_end = self.thread_radius
                cmds.append(Command('G33', {'X': x_end, 'Z': self.z_back, 'K': self.pitch}))
                cmds.append(Command('G1', {'X': x_end, 'Z': self.z_back, 'F': feed}))
            cmds.append(Command('G0', {'X': self.r_clear, 'Z': self.z_back}))
        cmds.append(Command('G0', {'X': self.r_clear, 'Z': self.z_drive}))
        return cmds


def _sample(seg, step):
    """Points (x, z) along a segment, ends included."""
    from liblathe.base.segmentgroup import _sample_points
    return _sample_points(seg, step)
