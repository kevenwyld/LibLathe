from liblathe.base.command import Command
from liblathe.base.point import Point
from liblathe.base.segment import Segment
from liblathe.base.segmentgroup import SegmentGroup
from liblathe.base.refusal import Refusal


# grid for the rectangle dilation when it cuts under the model's size
FINE = 0.001


class BaseOP:
    """Base class for all turning operations."""

    # the checks (see liblathe.base.refusal) whose refusals are waived: the
    # operation warns and carries on instead
    ignored_checks = ()

    def refuse(self, check, message):
        """Refuse the path (raise a Refusal naming the check), unless the check
        is waived: then record the reason as a warning and return, and the
        caller carries on."""
        if check not in (self.ignored_checks or ()):
            raise Refusal(check, message)
        if not hasattr(self, "warnings") or self.warnings is None:
            self.warnings = []
        self.warnings.append("Generated anyway (check %s waived): %s" % (check, message))

    def defeatured(self):
        """The part's outline the operation can follow (see
        SegmentGroup.defeature); one that would cut into the part is refused,
        unless that check is waived."""
        try:
            return self.part_segment_group.defeature(self.stock.z_min, self.tool, self.allow_grooving)
        except Refusal as refusal:
            self.refuse(refusal.check, str(refusal))
            return self.part_segment_group.defeature(self.stock.z_min, self.tool, self.allow_grooving, check=False)

    # apply tool nose radius compensation to contour paths
    nose_comp = True

    # the operation needs a turning insert (a liblathe Tool); operations
    # that describe their tool with their own parameters set this False
    needs_tool = True

    def __init__(self):

        self.stock = None
        self.tool = None
        self.part_segment_group = SegmentGroup()

        self.tool_paths = []

        self.min_dia = 0
        self.extra_dia = 0
        self.start_offset = 0
        self.end_offset = 0
        self.allow_grooving = False
        self.step_over = 1.5
        self.finish_passes = 1
        self.stock_to_leave = 0
        # separate stock to leave in X (radial, per side) and Z (axial); when
        # unset (None) both are stock_to_leave
        self.stock_to_leave_x = None
        self.stock_to_leave_z = None
        # repeat the last finish path once more ("spring pass")
        self.spring_pass = False
        # spacing of the finish passes; 0 uses the operation's own default
        self.finish_step_over = 0.0
        # Profile: also finish the front face (normally left to Face)
        self.face_front = False
        self.hfeed = 100
        self.vfeed = 50
        self.clearance = 3
        # radial clearance for the safe position and radial approaches (None:
        # the clearance); along Z the clearance is used
        self.clearance_x = None
        # radial lift-off at the end of each pass, beyond the outermost radius
        # reached (None: step_over * finish_passes, as before)
        self.retract = None
        # how far in front of the window's start the passes begin (along Z);
        # 0 enters radially. None: worked out (see lead_in_length). (Not
        # lead_in: the thread op has its own, along its thread.)
        self.pass_lead_in = None
        # clip the outer side too (boring, in the mirrored space: beyond the
        # stock's top is the hole, the axis and the material past a blind
        # bore's bottom, not air)
        self.clip_outer = False
        # edge breaking: every sharp outside corner of the part chamfered by
        # this much (0: off), for the operations that do it (breaks_edges)
        self.break_edges = 0.0
        # Face: the plane to face down to (None: the part's front)
        self.face_z = None
        # the corners broken, as Points (for the caller to report)
        self.edges_broken = []

        self.leadin_angle = 270
        self.leadout_angle = 315
        # TODO: implement invert_x to pass to segment_group().to_commands()

    # The working window: Z from z_end() (chuck side) to z_start(), X from x_outer() to x_inner().
    # X is negative internally (the part profile is on the -X side).

    def stock_xz(self):
        """Stock to leave as (X, Z): radial on diameters, axial on faces."""
        sx = self.stock_to_leave if self.stock_to_leave_x is None else self.stock_to_leave_x
        sz = self.stock_to_leave if self.stock_to_leave_z is None else self.stock_to_leave_z
        return float(sx), float(sz)

    def target_outline(self):
        """The surface the op may cut to: the part, shrunk by any negative stock to leave. Used by the checks."""
        sx, sz = self.stock_xz()
        nx, nz = min(sx, 0.0), min(sz, 0.0)
        if nx == 0 and nz == 0:
            return self.part_segment_group
        if abs(nx - nz) < 1e-9:
            return self.part_segment_group.offset_path(nx)
        return self.part_segment_group.dilate_rect(nx, nz, resolution=FINE)

    def offset_contour(self, segmentgroup, extra=0.0):
        """
        The contour offset by the stock to leave plus `extra`: an exact offset when X and Z stock are
        equal, otherwise a rectangle dilation (X on diameters, Z on faces) as a fine polyline.
        """
        sx, sz = self.stock_xz()
        sx, sz = sx + extra, sz + extra
        if abs(sx - sz) < 1e-9:
            return segmentgroup.offset_path(sx)
        # the grid errs outward by a step or two, which is fine for stock left
        # on; cutting under size (a negative stock) is a finished dimension,
        # so a finer grid there
        if sx < 0 or sz < 0:
            return segmentgroup.dilate_rect(sx, sz, resolution=FINE)
        return segmentgroup.dilate_rect(sx, sz)

    def z_start(self):
        """Z where the operation starts: the stock front plus start_offset."""
        return self.stock.z_max + self.start_offset

    def z_end(self):
        """Z where the operation must stop: the stock end minus end_offset."""
        return self.stock.z_min - self.end_offset

    def x_outer(self):
        """Largest radius (most negative X): the stock radius plus extra_dia / 2."""
        return self.stock.x_min - self.extra_dia * 0.5

    def x_inner(self):
        """Smallest radius (least negative X): min_dia / 2."""
        return 0 - self.min_dia * 0.5

    def starts_in_stock(self):
        """True when z_start() is behind the front of the stock, i.e. there is
        uncut material in front of the window."""
        return self.z_start() < self.stock.z_max - 1e-6

    def part_in_front(self):
        """True when some of the part lies in front of the window's start
        (e.g. a flange ahead of a selected diameter)."""
        zs = self.z_start()
        return any(max(seg.start.Z, seg.end.Z) > zs + 1e-6 for seg in self.part_segment_group.get_segments())

    def lead_in_length(self):
        """
        How far in front of the window the passes begin: as set; else a clearance, through any stock in
        front of the window, or 0 (radial entry) when part of the model is in front.
        """
        if self.pass_lead_in is not None:
            lead = max(0.0, float(self.pass_lead_in))
        elif not self.starts_in_stock():
            lead = float(self.clearance)
        elif self.part_in_front():
            lead = 0.0
        else:
            lead = self.stock.z_max - self.z_start() + float(self.clearance)
        self.lead_in_used = lead
        return lead

    def radial_entry(self):
        """The passes are entered radially (no lead-in along Z)."""
        return self.lead_in_length() <= 1e-9

    def set_params(self, params):
        """Set operations parameters"""

        for param in params:
            if hasattr(self, param):
                setattr(self, param, params[param])
            else:
                raise Warning("Unknown operation parameter '%s'" % param)

    def get_params(self):
        """Return operations parameters"""
        return {'min_dia': self.min_dia, 'extra_dia': self.extra_dia, 'start_offset': self.start_offset,
                'end_offset': self.end_offset, 'allow_grooving': self.allow_grooving, 'step_over': self.step_over,
                'finish_passes': self.finish_passes, 'stock_to_leave': self.stock_to_leave, 'hfeed': self.hfeed, 
                'vfeed': self.vfeed, 'clearance': self.clearance}

    # operations whose finishing follows the part's outline, so its corners
    # can be broken (Rough's cleanup pass, Profile, Face)
    breaks_edges = False

    def apply_edge_break(self):
        """With break_edges set, the part's outline with its sharp outside
        corners chamfered: the operation then cuts (and checks against) that."""
        self.edges_broken = []
        size = float(self.break_edges or 0.0)
        if not self.breaks_edges or size <= 0 or not self.part_segment_group.count():
            return
        from liblathe.base import corners
        z_min = self.part_segment_group.boundbox().z_min
        self.part_segment_group, self.edges_broken = corners.break_edges(self.part_segment_group, size, z_min=z_min)

    def get_gcode(self):
        """Base function for all turning operations"""

        if self.needs_tool and self.tool is None:
            raise Warning("No tool set for the operation")
        self.apply_edge_break()
        # an unset (zero) vertical feed would put F0 on the moves into the
        # cut; use the horizontal feed for them instead
        if not self.vfeed or self.vfeed <= 0:
            self.vfeed = self.hfeed

        # a window with no length along Z has nothing to cut; don't produce
        # a degenerate path for it
        if self.z_start() - self.z_end() <= 1e-4:
            return []
        self.generate_path()
        path = self.generate_gcode()
        if path:
            # keep the plane selection (G18) first
            head = []
            if path[0].get_movement() == 'G18':
                head, path = path[:1], path[1:]
            path = head + self.safe_start(path) + path + self.safe_end()
            self.check_tool_outline(path)
        return path

    # operations with a turning insert check its outline along the path
    checks_tool_outline = False

    def check_tool_outline(self, path):
        """Warn where the insert's outline (outside its cutting corner) would
        enter the part along the path; see liblathe.base.collision."""
        if not self.checks_tool_outline or self.tool is None or not getattr(self.tool, "tip_angle", None) \
                or not getattr(self.tool, "edge_length", None):
            return
        from liblathe.base import collision
        if not hasattr(self, "warnings"):
            self.warnings = []
        for z_from, z_to, depth in collision.check(path, self.tool, self.target_outline()):
            span = "at Z %.3f" % z_from if abs(z_from - z_to) < 1e-6 else "between Z %.3f and Z %.3f" % (z_from, z_to)
            self.warnings.append(
                "The insert's edge or body (not its nose) would hit the part %s, up to %.2f deep. Use a smaller "
                "tip angle or another rotation, or keep the operation away from there" % (span, depth))

    def radial_clearance(self):
        """The radial clearance: clearance_x if set, else the clearance."""
        return self.clearance if self.clearance_x is None else float(self.clearance_x)

    def safe_x(self):
        """Output X (a radius, positive) outside the stock and any max diameter."""
        return 0 - (self.x_outer() - self.radial_clearance())

    def safe_z(self):
        """Z a clearance in front of the stock, or of the window if it starts further out."""
        return max(self.stock.z_max, self.z_start()) + self.clearance

    def safe_start(self, path):
        """
        Rapids to the op's start without crossing the stock: out to the safe X, then along Z to the safe Z
        or, for a first cut inside the stock, to its Z.
        """
        first = next((c for c in path if c.get_movement() == 'G0' and 'Z' in c.get_params()), None)
        z = self.safe_z()
        if first is not None and first.get_params()['Z'] < self.stock.z_max:
            z = first.get_params()['Z']
        return [Command('G0', {'X': self.safe_x()}), Command('G0', {'X': self.safe_x(), 'Z': z})]

    def safe_end(self):
        """Rapids to the safe position: out in X at the current Z, then to the safe Z in front of the stock."""
        return [Command('G0', {'X': self.safe_x()}), Command('G0', {'X': self.safe_x(), 'Z': self.safe_z()})]

    def generate_path(self):
        """Main processing function for each op"""

        pass

    def generate_gcode(self):
        """Generate Gcode for the op segments"""

        return ""

    def add_part_edges(self, part_edges):
        """Add edges to define the part geometry part_edges = array of LibLathe segments"""

        for segment in part_edges:
            self.part_segment_group.add_segment(segment)

        self.part_segment_group.validate()
        # self.part_segment_group.create_freecad_shape('part_segment_group')

    def add_stock(self, stock_bb):
        """Define bounding box for the stock material stock_bb = LibLathe BoundBox"""
        self.stock = stock_bb

    def add_tool(self, tool):
        """Set the tool for the operation"""
        self.tool = tool

    def contour_path(self, segmentgroup, error="Calculated contour path intersects part", drop_front_face=False):
        """
        A contour as a tool path: checked against the part, nose compensated, clipped to the window,
        with a lead-in. None when nothing of it is inside the window.
        """
        # use a small offset to reduce false positives
        if segmentgroup.intersects_group(self.target_outline().offset_path(-0.001)):
            self.refuse("path-intersects-part", error)
        # tool nose radius compensation (imaginary tip programming)
        segmentgroup = self.nose_compensate(segmentgroup)
        if drop_front_face:
            segmentgroup = _without_front_face(segmentgroup)
        # keep the whole nose inside the working window
        segmentgroup = segmentgroup.clip_to_box(*self.clip_box())
        # a pass is done once it has left the stock: not round the part's
        # outer corner in air before retracting
        if not self.clip_outer:
            segmentgroup = _without_air_tail(segmentgroup, self.stock.x_min)
        if not segmentgroup.count():
            return None
        self.add_leadin(segmentgroup)
        return segmentgroup

    def clip_box(self):
        """
        The box the tip path is clipped to, (x_min, x_max, z_min, z_max): the whole nose stays inside
        z_end(), x_inner() and x_outer(); the tip may pass z_start() by a nose diameter and the axis by a
        nose radius. Sides in air (stock front, stock diameter) stay open.
        """
        inf = float("inf")
        d = 2 * self.nose_radius()
        x_min = self.x_outer() + d if (self.clip_outer or self.x_outer() > self.stock.x_min + 1e-6) else -inf
        x_max = self.x_inner() if self.min_dia > 0 else self.nose_radius()
        z_min = self.z_end()
        z_max = self.z_start() if self.starts_in_stock() else inf
        return x_min, x_max, z_min, z_max

    def nose_radius(self):
        r = getattr(self.tool, "nose_radius", None) or 0
        return r if self.nose_comp else 0

    def nose_compensate(self, segmentgroup):
        """
        Profile path to imaginary-tip path: offset outward by the nose radius, then shift by one nose
        radius toward the axis (+X) and the chuck (-Z). Diameters and faces are unchanged.
        """
        r = getattr(self.tool, "nose_radius", None) or 0
        if not self.nose_comp or r <= 0:
            return segmentgroup

        segs_in = segmentgroup.get_segments()
        z_floor = min(min(seg.start.Z, seg.end.Z) for seg in segs_in)

        centre_path = segmentgroup.offset_path(r)
        compensated = SegmentGroup()
        previous = None
        for seg in centre_path.get_segments():
            start = Point(seg.start.X + r, seg.start.Y, seg.start.Z - r)
            end = Point(seg.end.X + r, seg.end.Y, seg.end.Z - r)

            # close tiny gaps left by the arc offset so the path stays continuous
            if previous is not None and previous.end.distance_to(start) < 1e-4:
                start = previous.end

            # the shift toward the chuck must not run a straight diameter past
            # the end of the original path (e.g. into stock beyond the part)
            if abs(start.X - end.X) < 1e-9 and seg.bulge == 0:
                if end.Z < z_floor:
                    end = Point(end.X, end.Y, z_floor)
                if start.Z < z_floor:
                    start = Point(start.X, start.Y, z_floor)

            new_seg = Segment(start, end)
            new_seg.bulge = seg.bulge  # rigid translation keeps the arc's bulge
            compensated.add_segment(new_seg)
            previous = new_seg
        return compensated

    def add_leadin(self, segmentgroup):
        # get the first segment of the segment group
        segment = segmentgroup.get_segments()[0]
        if self.radial_entry():
            # the window starts inside the stock with the part in front (or
            # the lead-in is set to 0): approach radially from outside the
            # stock instead of along Z through that material
            x_pos = self.x_outer() - self.radial_clearance()
            start_point = Point(x_pos, segment.start.Y, segment.start.Z)
        elif self.pass_lead_in is None and not self.starts_in_stock():
            # the default: a clearance in front of the contour's start
            z_pos = segment.start.Z + self.clearance
            self.lead_in_used = self.clearance
            start_point = Point(segment.start.X, segment.start.Y, z_pos)
        else:
            # along Z, from the lead-in (as set, or through the stock in
            # front of the window) in front of the window's start
            z_pos = self.z_start() + self.lead_in_length()
            # create the new start point
            start_point = Point(segment.start.X, segment.start.Y, z_pos)
        # use the segments start as the leadin end point
        end_point = segment.start
        # create the leadin segment
        leadin = Segment(start_point, end_point)
        # add the leadin segment at the start of segment group
        segmentgroup.insert_segment(leadin, 0)


def _without_air_tail(segmentgroup, x_air, tol=0.05):
    """
    The path without trailing segments wholly outside the stock radius by more than tol (a bar's size
    tolerance). The segment that leaves the stock is kept, so the pass ends clear of it.
    """
    from liblathe.base.segmentgroup import _sample_points
    segs = segmentgroup.get_segments()
    n = len(segs)
    while n > 1 and all(x < x_air - tol for x, _ in _sample_points(segs[n - 1], 0.05)):
        n -= 1
    if n == len(segs):
        return segmentgroup
    out = SegmentGroup()
    for seg in segs[:n]:
        out.add_segment(seg)
    return out


def _without_front_face(segmentgroup):
    """The path without the flat front-face segments it starts with; a front chamfer or radius is kept."""
    segs = segmentgroup.get_segments()
    i = 0
    while (i < len(segs) and segs[i].bulge == 0 and abs(segs[i].start.Z - segs[i].end.Z) < 1e-6
           and segs[i].start.X > -1e-6):
        i += 1
    if i == 0 or i >= len(segs):
        return segmentgroup
    out = SegmentGroup()
    for seg in segs[i:]:
        out.add_segment(seg)
    return out
