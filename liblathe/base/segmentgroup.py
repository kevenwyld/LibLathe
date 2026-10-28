import math

from liblathe.base.boundbox import BoundBox
from liblathe.base.command import Command
from liblathe.base.geometry import dilate_rect_profile, sample_points, simplify_polyline
from liblathe.base.point import Point
from liblathe.base.segment import Segment



# Tolerance (degrees) for angle comparisons in defeature(). Geometry from
# FreeCAD carries floating-point noise (e.g. 269.99998 instead of 270), which
# otherwise flips tangent/limit cases and silently drops profile features.
ANGLE_TOL = 1e-3

class SegmentGroup:
    """Container Group for segments"""

    def __init__(self):
        self.segments = []

    def add_segment(self, segment):
        """Add segment to group"""

        self.segments.append(segment)

    def insert_segment(self, segment, position):
        """Insert segment into group at position"""
        self.segments.insert(position, segment)

    def get_segments(self):
        """Return segments of group as a list"""

        return self.segments
    
    def extend(self, segmentgroup):
        """Add segment group to this segmentgroup"""

        self.segments.extend(segmentgroup.get_segments())

    def copy(self):
        """A new group of copies of the segments (the Points are shared, as segments replace rather than edit them)."""
        out = SegmentGroup()
        out.segments = [Segment(seg.start, seg.end, seg.bulge) for seg in self.segments]
        return out

    def count(self):
        """Return the number of segments in the segmentgroup"""

        return len(self.segments)

    def boundbox(self):
        """Return the boundbox for the segmentgroup"""
        
        xvalues = []
        yvalues = []
        zvalues = []

        # collect all points from each segment by direction
        for segment in self.get_segments():
            bb = segment.get_boundbox()
            xvalues.extend([bb.x_min, bb.x_max])
            yvalues.extend([bb.y_min, bb.y_max])
            zvalues.extend([bb.z_min, bb.z_max])

        x_min = min(xvalues)
        x_max = max(xvalues)
        y_min = min(yvalues)
        y_max = max(yvalues)
        z_min = min(zvalues)
        z_max = max(zvalues)

        pt1 = Point(x_min, y_min, z_min)
        pt2 = Point(x_max, y_max, z_max)

        segmentgroupBoundBox = BoundBox(pt1, pt2)

        return segmentgroupBoundBox
    
    def intersects_group(self, segment_group):
        """check if the segment_group intersects self"""
        for segment in segment_group.get_segments():
            for seg in self.segments:
                intersect, point = segment.intersect(seg)
                if intersect:
                    return True

    def z_at_x(self, x):
        """get the z value at the first intersection at the given x position"""
        boundbox = self.boundbox()
        offset = boundbox.z_length()
        start_pt = Point(x, 0, boundbox.z_max + offset)
        end_pt = Point(x, 0, boundbox.z_min - offset)

        line_segment = Segment(start_pt, end_pt)
        for segment in self.get_segments():
            intersect, pts = segment.intersect(line_segment)
            if intersect:
                return pts[0].Z
        return None

    def previous_segment_connected(self, segment):
        """returns bool if segment is connect to the previous segment"""

        currentIdx = self.segments.index(segment)
        previousIdx = currentIdx - 1

        if not currentIdx == 0:
            currentStartPt = segment.start
            previousEndPt = self.segments[previousIdx].end

            if currentStartPt.is_same(previousEndPt):
                return True

        return False

    def get_min_retract_x(self, segment, part_segment_group):
        """ returns the minimum x retract based on the current segments and the part_segments """

        part_segments = part_segment_group.get_segments()
        currentIdx = self.segments.index(segment)
        x_values = []

        # get the x_max from the current pass segments
        for idx, seg in enumerate(self.segments):
            x_values.extend([seg.get_boundbox().x_min, seg.get_boundbox().x_max])
            if idx == currentIdx:
                break

        # get the x_max from the part segments up to the z position of the current segment
        seg_z_max = segment.get_boundbox().z_max
        for part_seg in part_segments:

            part_seg_z_max = part_seg.get_boundbox().z_max
            x_values.extend([part_seg.get_boundbox().x_min, part_seg.get_boundbox().x_max])

            if part_seg_z_max < seg_z_max:
                break

        min_retract_x = max(x_values, key=abs)
        return min_retract_x

    def to_commands(self, part_segment_group, stock, step_over, finish_passes, hSpeed, vSpeed, invert_x=True,
                    retract=None):
        """converts segmentgroup to gcode commands. retract: the radial lift-off
        beyond the outermost radius reached (default step_over * finish_passes)"""

        def get_pos(pnt):
            x = pnt.X
            y = pnt.Y
            z = pnt.Z

            if invert_x:
                x = 0 - x

            return Point(x, y, z)

        def get_arc_type(bulge):
            if bulge > 0:
                arcType = 'G2' if invert_x else 'G3'
            else:
                arcType = 'G3' if invert_x else 'G2'

            return arcType

        # drop zero-length segments (they would retract and plunge) and arcs whose
        # ends nearly coincide (a controller reads those as a full circle)
        kept = []
        for s in self.segments:
            if s.get_length() <= 1e-6 or (s.bulge != 0 and s.start.distance_to(s.end) <= 1e-3):
                continue
            # close the hair-line gap a dropped piece leaves, so the path
            # stays continuous (no retract and re-approach there)
            if kept and kept[-1].end.distance_to(s.start) < 1e-3 and not kept[-1].end.is_same(s.start):
                s = Segment(kept[-1].end, s.end, s.bulge)
            kept.append(s)
        # a local group: self.segments is left as it was
        path = SegmentGroup()
        path.segments = kept
        segments = kept

        cmds = []
        # TODO: Move the G18 to a PATH Class? it doesn't need to be added to every segment group
        cmd = Command('G18')  # xz plane
        cmds.append(cmd)

        for seg in segments:
            min_x_retract = path.get_min_retract_x(seg, part_segment_group)
            x_retract = min_x_retract - (step_over * finish_passes if retract is None else float(retract))
            z_retract = segments[0].start.Z

            if invert_x:
                x_retract = 0 - x_retract

            # rapid to the start of the segmentgroup
            if segments.index(seg) == 0:
                pt = get_pos(seg.start)
                params = {'X': pt.X, 'Z': pt.Z, 'F': hSpeed}
                rapid = Command('G0', params)
                cmds.append(rapid)

            # handle unconnected segments (lines and arcs): retract, move
            # across at the retract diameter, then feed in to the start
            if not path.previous_segment_connected(seg) and segments.index(seg) != 0:
                pt = get_pos(seg.start)
                # rapid to the x_min
                params = {'X': x_retract, 'F': hSpeed}
                rapid = Command('G0', params)
                cmds.append(rapid)
                # rapid at x_min to the start of the segment
                params = {'X': x_retract, 'Z': pt.Z, 'F': hSpeed}
                rapid = Command('G0', params)
                cmds.append(rapid)
                # feed (not rapid) to the start of the cutting move: the
                # start lies on or near the part, possibly in material
                params = {'X': pt.X, 'Z': pt.Z, 'F': vSpeed}
                cmd = Command('G1', params)
                cmds.append(cmd)

            # handle line segments
            if seg.bulge == 0:
                # perform the cutting
                pt = get_pos(seg.end)
                params = {'X': pt.X, 'Z': pt.Z, 'F': hSpeed}
                cmd = Command('G1', params)
                cmds.append(cmd)
            # handle arc segments
            if seg.bulge != 0:
                pt1 = get_pos(seg.start)
                pt2 = get_pos(seg.end)
                # set the arc direction
                arcType = get_arc_type(seg.bulge)

                # set the arc parameters
                cen = get_pos(seg.get_centre_point()).sub(pt1)
                params = {'X': pt2.X, 'Z': pt2.Z, 'I': cen.X, 'K': cen.Z, 'F': hSpeed}
                cmd = Command(arcType, params)
                cmds.append(cmd)

            # handle the lead out at the end of the segmentgroup
            if segments.index(seg) == len(segments) - 1:
                pt = get_pos(seg.end)
                #TODO: Remove the F parameter from rapid moves
                params = {'X': x_retract, 'Z': pt.Z, 'F': hSpeed}
                rapid = Command('G0', params)
                cmds.append(rapid)

                params = {'X': x_retract, 'Z': z_retract, 'F': hSpeed}
                rapid = Command('G0', params)
                cmds.append(rapid)

        return cmds

    def dilate_rect(self, sx, sz, resolution=0.005, tolerance=0.0005):
        """
        The contour (one radius per Z) grown by sx radially and sz along Z, as a polyline from the axis.
        Negative values shrink it, never past the axis. Computed on a Z grid, erring outward by one step.
        """
        import numpy as np

        segs = self.get_segments()
        if not segs:
            return SegmentGroup()
        pts = []
        for seg in segs:
            pts.extend(sample_points(seg, resolution / 2.5))
        zs = [z for x, z in pts]
        z_top_contour, z_end = max(zs), segs[-1].end.Z
        start = min(zs) - sz - 2 * resolution
        n = int(math.ceil((z_top_contour + sz + 2 * resolution - start) / resolution)) + 1
        prof = np.full(n, -np.inf)
        for x, z in pts:
            i = int(round((z - start) / resolution))
            if 0 <= i < n and -x > prof[i]:
                prof[i] = -x
        grown = dilate_rect_profile(prof, sx, sz, resolution)
        guard = grown.copy()
        guard[1:] = np.maximum(guard[1:], grown[:-1])
        guard[:-1] = np.maximum(guard[:-1], grown[1:])

        valid = np.where(np.isfinite(guard))[0]
        i_top = valid.max()
        i_end = max(int(math.floor((z_end - start) / resolution)), valid.min())
        line = [(0.0, start + i_top * resolution)]
        for i in range(i_top, i_end - 1, -1):
            if np.isfinite(guard[i]):
                line.append((-float(guard[i]), start + i * resolution))
        line = simplify_polyline(line, tolerance)
        out = SegmentGroup()
        for (x1, z1), (x2, z2) in zip(line[:-1], line[1:]):
            out.add_segment(Segment(Point(x1, 0, z1), Point(x2, 0, z2)))
        return out

    def clip_to_box(self, x_min, x_max, z_min, z_max, tol=1e-6):
        """
        New group with only the pieces inside the box, split at its edges (arcs keep centre and direction).
        May hold several disconnected chains (see split_connected()).
        """
        out = SegmentGroup()
        for seg in self.get_segments():
            for piece in _clip_segment(seg, x_min, x_max, z_min, z_max, tol):
                out.add_segment(piece)
        return out

    def split_connected(self):
        """Split the group into a list of SegmentGroups of connected segments."""
        groups = []
        current = None
        for seg in self.get_segments():
            if current is None or not current.get_segments()[-1].end.is_same(seg.start):
                current = SegmentGroup()
                groups.append(current)
            current.add_segment(seg)
        return groups

    def offset_path(self, step_over):
        """Create an offset segmentgroup by the distance of step_over"""
        # TODO Sort Edges to ensure they're in order.

        if step_over == 0:
            return self.copy()

        segs = self.get_segments()
        segmentgroup = SegmentGroup()

        for seg in segs:
            segment = seg.offset(step_over)

            if segment:
                segmentgroup.add_segment(segment)

        # TODO: create arcs at the intersections between segments, radius needs to be matched to the selected tool

        segmentgroup.join_segments()
        segmentgroup.validate()
        return segmentgroup

    def join_segments(self):
        """join segments of the segmentgroup"""

        segments = self.get_segments()
        segmentgroupOut = SegmentGroup()

        for i in range(len(segments)):

            pt1 = segments[i].start
            pt2 = segments[i].end

            if i != 0:
                seg1 = segments[i - 1]
                intersect, pt = seg1.intersect(segments[i], extend=True)
                if intersect:
                    if type(pt) is list:
                        pt = pt1.nearest(pt)
                    pt1 = pt

            if i != len(segments) - 1:
                seg2 = segments[i + 1]
                intersect2, pt = seg2.intersect(segments[i], extend=True)
                if intersect2:
                    if type(pt) is list:
                        pt = pt2.nearest(pt)
                    pt2 = pt

            if pt1 and pt2:
                if segments[i].bulge != 0:
                    #rad = segments[i].get_centre_point().distance_to(pt1)
                    nseg = Segment(pt1, pt2)
                    ang = segments[i].angle_from_points(nseg.start, nseg.end)
                    nseg.set_bulge(ang)
                    # nseg.derive_bulge(segments[i], rad)
                    segmentgroupOut.add_segment(nseg)
                else:
                    segmentgroupOut.add_segment(Segment(pt1, pt2))
            else:
                # No Intersections found. Return the segment in its current state
                segmentgroupOut.add_segment(segments[i])

        self.segments = segmentgroupOut.get_segments()
        self.clean_offset_path()

    def merge_segments(self):
        """merge adjacent line segments"""
        segments = self.segments
        i = 0
        while i + 1 < len(segments):
            a, b = segments[i], segments[i + 1]
            if a.get_rotation() == b.get_rotation() and a.bulge == 0 and b.bulge == 0:
                # a new segment, not a and b edited: they are shared with the part outline
                segments[i] = Segment(a.start, b.end)
                segments.pop(i + 1)
                # the merged segment may now line up with the one before it
                i = max(i - 1, 0)
            else:
                i += 1

    def clean_offset_path(self, index=0):
        """
        remove any self intersecting features from the path.
        index is the first segment to evaluate
        """
        segments = self.segments
        while index + 1 < len(segments):
            for i in range(len(segments) - 1, index, -1):
                if segments[index].end.is_same(segments[i].start):
                    continue

                intersect, pt = segments[index].intersect(segments[i])
                if intersect:
                    if type(pt) is list:
                        pt = pt[0]

                    angle = segments[index].angle_from_points(segments[index].start, pt)
                    segments[index].end = pt
                    if segments[index].bulge:
                        segments[index].set_bulge(angle)

                    angle = segments[i].angle_from_points(pt, segments[i].end)
                    segments[i].start = pt
                    if segments[i].bulge != 0:
                        segments[i].set_bulge(angle)
                    if i != index + 1:
                        del segments[index + 1:i]

                    break
            index += 1

    def defeature(self, stock_z_min, tool, allow_grooving=False, check=True):
        """Defeature the segment group. Remove features that cannot be turned. e.g. undercuts / grooves"""
        segments = self.get_segments()
        segs_out = SegmentGroup()
        index = 0

        while index < len(segments):
            seg = segments[index]
            next_index = False
            pt1 = seg.start
            pt2 = seg.end
            pt = None

            # TODO: Tidy this mess

            if seg.bulge > 0:
                segAng = round(math.degrees(seg.get_angle()), 5)
                # get angle tangent to the start point
                startPtAng = round(pt1.angle_to(seg.get_centre_point()) - 90, 5)
                if startPtAng >= tool.get_tool_cutting_angle() - ANGLE_TOL:
                    if startPtAng + segAng <= 270 + ANGLE_TOL:
                        segs_out.add_segment(seg)
                else:
                    ang = tool.get_tool_cutting_angle()
                    # calculate the length required to project the point to the centreline
                    length = abs(pt1.X / math.cos(math.radians(ang - 90)))
                    proj_pt = pt1.project(ang, length)
                    projseg = Segment(pt1, proj_pt)
                    intersect, pts = projseg.intersect(segments[index])
                    if intersect and allow_grooving:
                        # add the intersecting line to the segment_group
                        new_seg = Segment(pt1, pts[0])
                        segs_out.add_segment(new_seg)
                        # add the remainder of the arc to the segment_group
                        remaining_seg = Segment(pts[0], pt2)
                        angle = seg.angle_from_points(remaining_seg.start, remaining_seg.end)
                        #remaining_seg.derive_bulge(seg)
                        remaining_seg.set_bulge(angle)
                        segs_out.add_segment(remaining_seg)
                    else:
                        if seg.start.angle_to(seg.end) <= 180 + ANGLE_TOL:
                            seg = Segment(pt1, pt2)
                            segs_out.add_segment(seg)
                        else:
                            pt = seg.start
                            next_index, pt = self.find_next_good_edge(index, stock_z_min, tool, allow_grooving, pt)

            if seg.bulge < 0:
                # Limit the arc movement to the X extents or the tangent at the max tool angle if allow_grooving
                angle_limit = 270 if allow_grooving is False else tool.get_tool_cutting_angle() + 90
                if seg.get_centre_point().angle_to(pt2) >= angle_limit - ANGLE_TOL:
                    segs_out.add_segment(seg)
                else:
                    rad = seg.get_radius()
                    if not allow_grooving:
                        # define a point vertically down on the x axis.
                        x = seg.get_centre_point().X - rad
                        y = seg.get_centre_point().Y
                        z = seg.get_centre_point().Z
                        pt = Point(x, y, z)
                    else:
                        # project a point from the centre of the arc along the angle limit to the radius
                        pt = seg.get_centre_point().project(angle_limit, rad)

                    nseg = Segment(pt1, pt)
                    #nseg.derive_bulge(seg, rad)
                    angle = seg.angle_from_points(nseg.start, nseg.end)
                    nseg.set_bulge(angle)
                    segs_out.add_segment(nseg)

                    pt1 = pt
                    next_index, pt = self.find_next_good_edge(index, stock_z_min, tool, allow_grooving, pt)

            elif seg.bulge == 0:
                if pt1.angle_to(pt2) < tool.get_tool_cutting_angle() - ANGLE_TOL:
                    next_index, pt = self.find_next_good_edge(index, stock_z_min, tool, allow_grooving)
                else:
                    segs_out.add_segment(seg)
            
            if next_index is False and pt is not None:
                seg = Segment(pt1, pt)
                segs_out.add_segment(seg)
                break
            if next_index is not False and next_index != index:
                seg = Segment(pt1, pt)
                segs_out.add_segment(seg)

                next_pt1 = pt
                next_pt2 = segments[next_index].end
                seg = Segment(next_pt1, next_pt2)
                
                #seg.derive_bulge(segments[next_index])
                if segments[next_index].bulge != 0:
                    n_angle = segments[next_index].angle_from_points(seg.start, seg.end)
                    seg.set_bulge(n_angle)
                segs_out.add_segment(seg)

                next_index += 1
                index = next_index
                continue

            index += 1
        segs_out.merge_segments()
        # create an inset version of the part_segmentgroup
        # check if the defeatured segmentgroup intersects the part
        # TODO: This is a bit hacky. Is there a better way?
        if check and self.offset_path(-0.0001).intersects_group(segs_out):
            # segs_out.create_freecad_shape("defeatured_part_segment_group")
            from liblathe.base.refusal import Refusal
            raise Refusal("part-defeaturing-failed", "The outline this insert can follow cuts into the part")

        return segs_out

    def find_next_good_edge(self, current_index, stock_z_min, tool, allow_grooving, pt=None):
        segments = self.get_segments()
        index = current_index
        if pt is None:
            pt1 = segments[index].start
        else:
            pt1 = pt

        index += 1
        while index < len(segments):

            if allow_grooving:
                # create a new point at the max angle from pt1
                ang = tool.get_tool_cutting_angle()
                # calculate the length required to project the point to the centreline
                length = abs(pt1.X / math.cos(math.radians(ang - 90)))
                pt2 = pt1.project(ang, length)
            else:
                pt2 = Point(pt1.X, pt1.Y, stock_z_min)

            # create a new projected segment
            seg = Segment(pt1, pt2)

            # loop through the remaining segments and see if the projected segments
            idx = index
            while idx < len(segments):
                intersect, pts = seg.intersect(segments[idx])
                if intersect:
                    return idx, pts[0]
                idx += 1
            index += 1

        stock_pt = Point(pt1.X, pt1.Y, stock_z_min)
        seg = Segment(pt1, stock_pt)
        index = current_index
        index += 1

        while index < len(segments):
            intersect, pts = seg.intersect(segments[index])
            if intersect:
                return index, pts[0]

            index += 1
        # No solution :(
        return False, stock_pt

    def validate(self):
        """validate the segment group"""
        # check first segment starts at X0
        # check if the last segment ends at x0

        count = self.count()

        if count == 0:
            raise ValueError("The part outline has no segments")

        # check the first segment starts at x = 0
        start_segment = self.segments[0]

        if start_segment.start.X != 0:
            new_start_point = Point(0, 0, start_segment.start.Z)
            # check if the points are the same within rounding errors
            if new_start_point.is_same(start_segment.start):
                start_segment.start = new_start_point
            else:
                new_start_seg = Segment(new_start_point, start_segment.start)
                self.insert_segment(new_start_seg, 0)
                if self.count() != count + 1:
                    raise ValueError("Could not close the part outline to the axis")

        # check the last segment end at x = 0
        count = self.count()
        end_segment = self.segments[-1]

        if end_segment.end.X != 0:
            new_end_point = Point(0, 0, end_segment.end.Z)
            # check if the points are the same within rounding errors
            if new_end_point.is_same(end_segment.end):
                end_segment.end = new_end_point

    def create_freecad_shape(self, name):
        """ create a FreeCAD shape for debugging"""
        import FreeCAD
        import Part

        if self.count == 0:
            raise ValueError("The segment group is empty")

        part_edges = []
        for segment in self.segments:
            start_point = FreeCAD.Vector(segment.start.X, segment.start.Y, segment.start.Z)
            end_point = FreeCAD.Vector(segment.end.X, segment.end.Y, segment.end.Z)

            if segment.bulge == 0:
                edge = Part.makeLine(start_point, end_point)
            else:
                center = segment.get_centre_point()
                axis = FreeCAD.Vector(0.0, 1.0, 0.0)
                start_angle = center.angle_to(segment.start) - 90
                end_angle = center.angle_to(segment.end) - 90
                if segment.bulge > 0:
                    edge = Part.makeCircle(segment.get_radius(),
                                           FreeCAD.Vector(center.X, center.Y, center.Z),
                                           axis, start_angle, end_angle)
                else:
                    edge = Part.makeCircle(segment.get_radius(),
                                           FreeCAD.Vector(center.X, center.Y, center.Z),
                                           axis, end_angle, start_angle)

            part_edges.append(edge)

        path_profile = Part.makeCompound(part_edges)
        Part.show(path_profile, name)


def _inside(x, z, x_min, x_max, z_min, z_max, tol):
    return x_min - tol <= x <= x_max + tol and z_min - tol <= z <= z_max + tol


def _clip_segment(seg, x_min, x_max, z_min, z_max, tol):
    """Return the pieces of seg inside the box, in order, as new Segments."""
    sx, sz, ex, ez = seg.start.X, seg.start.Z, seg.end.X, seg.end.Z

    # zero-length segments carry no geometry; drop them
    if seg.start.distance_to(seg.end) <= 1e-9:
        return []

    if seg.bulge == 0:
        # Liang-Barsky on the parameter t in [0, 1]
        t0, t1 = 0.0, 1.0
        dx, dz = ex - sx, ez - sz
        for p, q in ((-dx, sx - x_min), (dx, x_max - sx), (-dz, sz - z_min), (dz, z_max - sz)):
            if abs(p) < 1e-15:
                if q < -tol:
                    return []
                continue
            r = q / p
            if p < 0:
                t0 = max(t0, r)
            else:
                t1 = min(t1, r)
        if t1 - t0 <= 1e-12:
            return []
        if t0 <= 0 and t1 >= 1:
            return [Segment(seg.start, seg.end)]
        start = seg.start if t0 <= 0 else Point(sx + dx * t0, seg.start.Y, sz + dz * t0)
        end = seg.end if t1 >= 1 else Point(sx + dx * t1, seg.start.Y, sz + dz * t1)
        if start.distance_to(end) <= 1e-9:
            return []
        return [Segment(start, end)]

    # arc: parametrise by angle a measured as atan2(dX, dZ) about the centre
    c = seg.get_centre_point()
    rad = seg.get_radius()
    a0 = math.atan2(sx - c.X, sz - c.Z)
    included = seg.get_angle()

    def pt_at(a):
        return (c.X + rad * math.sin(a), c.Z + rad * math.cos(a))

    # choose the sweep direction that lands on the segment end
    sweep = included
    px, pz = pt_at(a0 - included)
    qx, qz = pt_at(a0 + included)
    if math.hypot(px - ex, pz - ez) < math.hypot(qx - ex, qz - ez):
        sweep = -included

    # parameters (0..1) where the arc crosses the box edges
    ts = {0.0, 1.0}
    crossings = []
    for xb in (x_min, x_max):
        v = (xb - c.X) / rad
        if -1 <= v <= 1:
            s_ = math.asin(v)
            crossings += [s_, math.pi - s_]
    for zb in (z_min, z_max):
        v = (zb - c.Z) / rad
        if -1 <= v <= 1:
            c_ = math.acos(v)
            crossings += [c_, -c_]
    two_pi = 2 * math.pi
    for a in crossings:
        if sweep > 0:
            delta = (a - a0) % two_pi
        else:
            delta = -((a0 - a) % two_pi)
        t = delta / sweep
        # ignore crossings within tol (as arc length) of the ends
        t_tol = tol / (abs(sweep) * rad)
        if t_tol < t < 1 - t_tol:
            ts.add(t)

    ts = sorted(ts)
    pieces = []
    for t_a, t_b in zip(ts[:-1], ts[1:]):
        mx, mz = pt_at(a0 + sweep * (t_a + t_b) / 2)
        if not _inside(mx, mz, x_min, x_max, z_min, z_max, tol):
            continue
        if t_a <= 0:
            start = seg.start
        else:
            x, z = pt_at(a0 + sweep * t_a)
            start = Point(x, seg.start.Y, z)
        if t_b >= 1:
            end = seg.end
        else:
            x, z = pt_at(a0 + sweep * t_b)
            end = Point(x, seg.start.Y, z)
        if start.distance_to(end) <= 1e-9:
            continue
        bulge = math.copysign(math.tan(abs(sweep) * (t_b - t_a) / 4), seg.bulge)
        pieces.append(Segment(start, end, bulge))

    return pieces

