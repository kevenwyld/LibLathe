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
    Single-point threading along Z toward the chuck, programmed to the insert's sharp point, from
    thread_radius to thread_depth deep. Pass n is first_depth * n ** (1 / degression) deep, shifted
    along Z by depth * tan(infeed_angle). Output: G33 passes, or one G76 cycle with `canned`.
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
            raise ValueError("Pitch must be above zero")
        if self.thread_depth <= 0:
            raise ValueError("Thread depth must be above zero")
        if self.thread_radius <= 0:
            raise ValueError("Thread diameter must be above zero")
        if self.internal and self.thread_depth >= self.thread_radius * 2:
            raise ValueError("Thread depth is more than the bore's radius")

        depth = float(self.thread_depth)
        self.z_front = self.z_start()
        self.z_back = self.z_end()
        if self.thread_length and self.thread_length > 0:
            self.z_back = max(self.z_back, self.z_front - self.thread_length)
        self.run_out = max(0.0, float(self.run_out or 0.0))
        if self.run_out >= self.z_front - self.z_back - 1e-9:
            raise ValueError("Run-out %.3f is not shorter than the thread (%.3f)"
                             % (self.run_out, self.z_front - self.z_back))
        # where the full-depth thread ends: the run-out tapers out from here
        self.z_full = self.z_back + self.run_out
        lead = self.lead_in if self.lead_in > 0 else 3 * self.pitch
        if not self.internal:
            lead = self._fit_lead_in(lead, depth)
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
                raise ValueError("Over 1000 passes: raise First pass depth or Min pass depth")
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

    def _insert_profile(self):
        """The insert's half width and its flanks' run per unit of height
        (tan of half its angle): its profile rises from the tip at that
        slope up to its half width, then is straight. 0 when not given."""
        half = self.tool_width / 2 if self.tool_width > 0 else 0.0
        slope = math.tan(math.radians(self.tool_angle / 2)) if self.tool_angle > 0 else 0.0
        return half, slope

    def _front_room(self, depth):
        """
        (longest lead-in the part in front leaves room for, Z of the part limiting it), or (None, None).
        Part h above the deepest tip is clear of an insert more than min(h * slope, half width) behind it.
        """
        tip = self.thread_radius - depth
        half, slope = self._insert_profile()
        room = where = None
        for seg in self.part_segment_group.get_segments():
            for x, z in _sample(seg, 0.05):
                h = -x - tip
                if h <= 1e-3 or z <= self.z_front + 1e-6:
                    continue
                reach = min(h * slope, half) if slope > 0 else half
                fits = z - reach - self.z_front
                if room is None or fits < room:
                    room, where = fits, z
        return room, where

    def _fit_lead_in(self, lead, depth):
        """The lead-in, shortened to the room in front of the thread if it
        was worked out (three pitches), refused if it was set longer, and the
        thread refused if there is no room at all (external threads)."""
        room, where = self._front_room(depth)
        if room is None or lead <= room + 1e-9:
            return lead
        tip = self.thread_radius - depth
        if room <= 1e-3:
            self.refuse(
                "thread-front-clearance",
                "The part at Z %.3f stands above the thread's root (radius %.3f), so every pass would cut into it. "
                "Chamfer, neck or groove the part in front of the thread to below the root" % (where, tip))
            return lead                              # waived: the lead-in as set, or three pitches
        if self.lead_in > 0:
            self.refuse(
                "thread-lead-in-too-long",
                "Lead-in %.3f does not fit: the part at Z %.3f stands above the thread's root (radius %.3f). "
                "At most %.3f fits" % (lead, where, tip, math.floor(room * 1000) / 1000))
            return lead                              # waived: the lead-in as set
        room = math.floor(room * 1000) / 1000
        self.warnings.append(
            "Lead-in shortened to %.3f from %.3f: the part at Z %.3f stands above the thread's root. Z has less "
            "room to reach speed, so check the start of the thread" % (room, lead, where))
        return room

    def _check_clearance(self, depth):
        """
        Refuse when the part behind the thread's end stands into the insert's chuck-side flank
        (external threads). Only the part is checked, not where it is held.
        """
        tip = self.thread_radius - depth                # the deepest pass's tip
        half, slope = self._insert_profile()
        front, back = self.z_front, self.z_back
        latest_end = None        # the highest Z the thread must end at to clear what is behind it
        for seg in self.part_segment_group.get_segments():
            for x, z in _sample(seg, 0.05):
                r = -x
                if r <= tip + 1e-3 or z > front + 1e-6:
                    continue
                # only part standing proud of the thread's diameter (a shoulder) can be hit;
                # a micron's tolerance, as the section's corners are not exactly on the radius
                if half > 0 and back - half - 1e-6 <= z <= back + 1e-6 and r > self.thread_radius + 1e-3:
                    # the flank is (r - tip) * slope behind the tip at height r
                    end = z + min((r - tip) * slope, half)
                    if end > back + 1e-6 and (latest_end is None or end > latest_end):
                        latest_end = end
        if latest_end is not None:
            # round the suggestion the safe way (up), so using it clears
            latest_end = math.ceil(latest_end * 1000 - 1e-6) / 1000
            self.refuse(
                "thread-hits-behind-end",
                "The insert would hit the part behind the thread's end at Z %.3f. End the thread at Z %.3f or "
                "above (length at most %.3f), or cut a relief groove" % (back, latest_end, front - latest_end))

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
        # every pass is the deepest one moved back along the infeed line, end and run-out
        # included (as G76's zoff), so it lies inside the finished groove to its end
        full = float(self.thread_depth)
        shift = math.tan(math.radians(self.infeed_angle))
        ends = []
        for d in self.depths:
            back = (full - d) * shift            # toward the tailstock
            z_full = self.z_full + back
            # the run-out at the deepest pass's slope, out to the crest
            z_end = z_full - self.run_out * d / full if self.run_out > 0 else z_full
            ends.append((z_full, z_end))
        for (x, z), (z_full, z_end) in zip(self.passes, ends):
            cmds.append(Command('G0', {'X': self.r_clear, 'Z': z}))
            cmds.append(Command('G0', {'X': x, 'Z': z}))
            # FreeCAD's path display skips G33, so a zero-length G1 to each G33's end draws the cut;
            # it needs some feed rate, as a control refuses G1 without one
            feed = self.hfeed if self.hfeed and self.hfeed > 0 else 1.0
            cmds.append(Command('G33', {'X': x, 'Z': z_full, 'K': self.pitch}))
            cmds.append(Command('G1', {'X': x, 'Z': z_full, 'F': feed}))
            if self.run_out > 0:
                # run-out to the crest, still synchronised: G33's K is per revolution along the move,
                # so K is scaled by the move's length over its length along Z (as G76's taper)
                dz, dx = abs(z_full - z_end), abs(self.thread_radius - x)
                k = self.pitch * math.hypot(dz, dx) / dz if dz > 1e-9 else self.pitch
                cmds.append(Command('G33', {'X': self.thread_radius, 'Z': z_end, 'K': k}))
                cmds.append(Command('G1', {'X': self.thread_radius, 'Z': z_end, 'F': feed}))
            cmds.append(Command('G0', {'X': self.r_clear, 'Z': z_end}))
        cmds.append(Command('G0', {'X': self.r_clear, 'Z': self.z_drive}))
        return cmds


def _sample(seg, step):
    """Points (x, z) along a segment, ends included."""
    from liblathe.base.segmentgroup import _sample_points
    return _sample_points(seg, step)
