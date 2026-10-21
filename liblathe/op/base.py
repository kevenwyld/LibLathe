from liblathe.base.command import Command
from liblathe.base.point import Point
from liblathe.base.segment import Segment
from liblathe.base.segmentgroup import SegmentGroup


# grid for the rectangle dilation when it cuts under the model's size
FINE = 0.001


class BaseOP:
    """Base class for all turning operations."""

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

        self.leadin_angle = 270
        self.leadout_angle = 315
        # TODO: implement invert_x to pass to segment_group().to_commands()

    # The working window. Operations only move the tool inside it:
    #   Z from z_end() (toward the chuck) to z_start() (toward the tailstock)
    #   X from x_outer() (largest radius) to x_inner() (smallest radius)
    # X is negative internally (the part profile is on the -X side).

    def stock_xz(self):
        """Stock to leave as (X, Z): radial on diameters, axial on faces."""
        sx = self.stock_to_leave if self.stock_to_leave_x is None else self.stock_to_leave_x
        sz = self.stock_to_leave if self.stock_to_leave_z is None else self.stock_to_leave_z
        return float(sx), float(sz)

    def target_outline(self):
        """
        The surface the operation may cut to: the part, or with a negative
        stock to leave (cutting under the model's size, e.g. a thread's major
        diameter turned to its tolerance) the part shrunk by it. The checks
        that a path stays clear of the part use this.
        """
        sx, sz = self.stock_xz()
        nx, nz = min(sx, 0.0), min(sz, 0.0)
        if nx == 0 and nz == 0:
            return self.part_segment_group
        if abs(nx - nz) < 1e-9:
            return self.part_segment_group.offset_path(nx)
        return self.part_segment_group.dilate_rect(nx, nz, resolution=FINE)

    def offset_contour(self, segmentgroup, extra=0.0):
        """
        Offset a contour (e.g. the defeatured part) by the stock to leave,
        plus `extra` in both directions. Equal X and Z stock is the exact
        offset (a disc: lines and arcs). Different X and Z stock is the
        rectangle dilation (X stock on diameters, Z stock on faces), as a
        fine polyline: a contour with stock left on it is never a final
        surface.
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

    def set_params(self, params):
        """Set operations parameters"""

        for param in params:
            if hasattr(self, param):
                setattr(self, param, params[param])
            else:
                raise Warning("Attempting to set undefined parameter '%s'" % param)

    def get_params(self):
        """Return operations parameters"""
        return {'min_dia': self.min_dia, 'extra_dia': self.extra_dia, 'start_offset': self.start_offset,
                'end_offset': self.end_offset, 'allow_grooving': self.allow_grooving, 'step_over': self.step_over,
                'finish_passes': self.finish_passes, 'stock_to_leave': self.stock_to_leave, 'hfeed': self.hfeed, 
                'vfeed': self.vfeed, 'clearance': self.clearance}

    def get_gcode(self):
        """Base function for all turning operations"""

        if self.needs_tool and self.tool is None:
            raise Warning("Tool is unset")
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
                "The insert would hit the part with an edge or its body (not the cutting nose) %s, up to %.2f deep: "
                "the part there is steeper or tighter than this insert can follow. Use an insert with a smaller "
                "tip angle or a different rotation, or limit the operation there" % (span, depth))

    def safe_x(self):
        """Output X (a radius, positive) outside the stock and any max diameter."""
        return 0 - (self.x_outer() - self.clearance)

    def safe_z(self):
        """Z a clearance in front of the stock, or of the window if it starts further out."""
        return max(self.stock.z_max, self.z_start()) + self.clearance

    def safe_start(self, path):
        """
        Moves from wherever the tool is to the start of the operation without
        crossing the stock: straight out in X to the safe diameter (radially,
        at the current Z), then along Z outside the stock. If the first cut
        starts in front of the stock, stop at the safe Z in front of it;
        otherwise go straight to the first cut's Z, outside the stock, so the
        operation's own first move only travels in X.
        """
        first = next((c for c in path if c.get_movement() == 'G0' and 'Z' in c.get_params()), None)
        z = self.safe_z()
        if first is not None and first.get_params()['Z'] < self.stock.z_max:
            z = first.get_params()['Z']
        return [Command('G0', {'X': self.safe_x()}), Command('G0', {'X': self.safe_x(), 'Z': z})]

    def safe_end(self):
        """
        Leave the operation at the safe position: straight out in X at the
        current Z, then to the safe Z in front of the stock, ready for the
        next operation or a tool change.
        """
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
        Turn a contour (a path where the cutting edge touches the part, e.g. an
        offset of the defeatured part) into a finished tool path: check it
        does not cut into the part, apply nose radius compensation, clip it to
        the working window and add a lead in. Returns None when nothing of the
        contour is inside the window.
        """
        # use a small offset to reduce false positives
        if segmentgroup.intersects_group(self.target_outline().offset_path(-0.001)):
            raise ValueError(error)
        # tool nose radius compensation (imaginary tip programming)
        segmentgroup = self.nose_compensate(segmentgroup)
        if drop_front_face:
            segmentgroup = _without_front_face(segmentgroup)
        # keep the whole nose inside the working window
        segmentgroup = segmentgroup.clip_to_box(*self.clip_box())
        if not segmentgroup.count():
            return None
        self.add_leadin(segmentgroup)
        return segmentgroup

    def clip_box(self):
        """
        The box the tip path is clipped to: (x_min, x_max, z_min, z_max).

        The path is the path of the imaginary tip; the nose circle spans one
        nose diameter from the tip toward the tailstock (+Z) and away from
        the axis. So clipping the tip keeps the whole nose behind z_end()
        (the chuck side) and above x_inner() (the minimum diameter), and the
        outer limit is shrunk by the nose diameter so the nose stays inside
        x_outer(). At z_start() the nose may reach up to one nose diameter
        past the limit: that keeps the edges of selected faces reachable, and
        the path itself never gouges the part.

        Only real limits clip. Beyond the stock front and outside the stock
        diameter the path is in air, so those sides stay open unless the
        window is set inside the stock; likewise the axis side stays open
        unless a minimum diameter is set (a ball or point needs the tip to
        reach past the axis).
        """
        inf = float("inf")
        d = 2 * self.nose_radius()
        x_min = self.x_outer() + d if self.x_outer() > self.stock.x_min + 1e-6 else -inf
        x_max = self.x_inner() if self.min_dia > 0 else inf
        z_min = self.z_end()
        z_max = self.z_start() if self.starts_in_stock() else inf
        return x_min, x_max, z_min, z_max

    def nose_radius(self):
        r = getattr(self.tool, "nose_radius", None) or 0
        return r if self.nose_comp else 0

    def nose_compensate(self, segmentgroup):
        """
        Convert a profile path (where the cutting edge must touch the part) into
        the path of the tool's imaginary tip, so the round nose of the insert
        lands on dimension on tapers and arcs, not just on diameters and faces.

        nose centre path   = profile offset outward by the nose radius
        imaginary tip path = nose centre shifted by one nose radius toward the
                             axis (+X internally, the part is on -X) and toward
                             the chuck (-Z)

        This matches the usual touch-off: X set on a turned diameter and Z on a
        faced end, both measured to the imaginary tip. Straight diameters and
        faces are unchanged; tapers and arcs are corrected.
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
        if self.starts_in_stock():
            # the window starts inside the stock, with uncut material in
            # front: approach radially from outside the stock instead of
            # along Z through that material
            x_pos = self.x_outer() - self.clearance
            start_point = Point(x_pos, segment.start.Y, segment.start.Z)
        else:
            # calculate the new z_pos with lead in
            z_pos = segment.start.Z + self.clearance
            # create the new start point
            start_point = Point(segment.start.X, segment.start.Y, z_pos)
        # use the segments start as the leadin end point
        end_point = segment.start
        # create the leadin segment
        leadin = Segment(start_point, end_point)
        # add the leadin segment at the start of segment group
        segmentgroup.insert_segment(leadin, 0)


def _without_front_face(segmentgroup):
    """
    The path without its front face: the flat segments (along X, at one Z)
    it starts with on or past the axis (offset_path() always starts a
    contour on the axis, and nose compensation moves that start past it).
    The path then starts at the front corner; a front chamfer or radius is
    not flat and is kept.
    """
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
