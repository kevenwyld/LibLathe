import math
from collections import namedtuple

import liblathe.op.base
from liblathe.base.point import Point
from liblathe.base.segment import Segment
from liblathe.base.segmentgroup import SegmentGroup

# create a namedtuple to hold intersection data
Intersection = namedtuple('Intersection', 'point, seg')


class RoughOP(liblathe.op.base.BaseOP):

    checks_tool_outline = True

    # finish roughing with a contour pass at the stock to leave
    cleanup_pass = True

    def generate_path(self):
        """Generate the path for the Rough operation"""
        roughing_segment_group = self.part_segment_group.defeature(self.stock.z_min, self.tool, self.allow_grooving)
        # self.part_segment_group.create_freecad_shape('roughing_segment_group')
        self.clearing_paths = []
        # passes run from the window start to the window end. When the window
        # starts in front of the stock they start a clearance away, in air;
        # when it starts inside the stock they start on the window edge and
        # are entered radially (see below)
        # radial entry with the part in front of the window (or lead-in set
        # to 0); otherwise the passes start the lead-in in front, along Z
        starts_in_stock = self.radial_entry()
        z_start = self.z_start()
        z_max = z_start + self.lead_in_length()
        z_min = self.z_end()
        x_inner = self.x_inner()
        # create roughing boundary offset by the stock to leave value
        # include a minimal offset to ensure the roughing passes don't intersect the part
        roughing_boundary = self.offset_contour(roughing_segment_group, extra=0.01)
        # The passes start and stop with the imaginary tip on this boundary,
        # so it must be the tip's locus where the nose just touches the part,
        # not the part itself: where the boundary falls away behind a pass
        # (the pass starts inside a groove, behind its front wall), the nose
        # reaches back into the wall otherwise. Diameters and faces in front
        # of the tip are unchanged.
        roughing_boundary = self.nose_compensate(roughing_boundary)
        # with the tip programmed, the check that a pass stays clear of the
        # part is made on the nose: its centre path against the part grown by
        # the nose radius (a hair less, so touching is not a hit). The
        # imaginary tip itself may lie inside the outline where the nose
        # touches a slope or an arc.
        r_nose = getattr(self.tool, "nose_radius", 0) or 0 if self.nose_comp else 0
        target = self.target_outline()
        check_group = target.offset_path(r_nose - 0.005) if r_nose > 0.005 else target
        # define the x limit for roughing
        x_min = -abs(math.ceil(self.stock.x_length() + self.extra_dia * 0.5))

        # start from a small offset to ensure the roughing passes intersect with the roughing_boundary
        # TODO: This is a bit hacky, is there a better way?
        x_pos = -1e-6
        # work from 0 to x_min creating roughing passes
        while x_pos > x_min:
            # skip passes below the minimum diameter
            if x_pos > x_inner + 1e-9:
                x_pos -= self.step_over
                continue

            pt1 = Point(x_pos, 0, z_max)
            pt2 = Point(x_pos, 0, z_min)
            path_line = Segment(pt1, pt2)
            intersections = []
            for seg in roughing_boundary.get_segments():
                intersect, point = seg.intersect(path_line)
                if intersect:
                    for p in point:
                        # a crossing exactly at a vertex is reported by both
                        # segments; count it once
                        if any(abs(p.Z - i.point.Z) < 1e-7 for i in intersections):
                            continue
                        intersections.append(Intersection(p, seg))

            # The pass alternates between air and material at each crossing.
            # Whether it starts in material is not assumed: a window that
            # starts inside the stock (e.g. limited to a groove) can put the
            # pass start under the part's outline.
            inside = self.inside_boundary(roughing_boundary, x_pos, z_max, x_min)

            points = [Intersection(pt1, None)]
            points.extend(sorted(intersections, key=lambda p: p.point.Z, reverse=True))
            points.append(Intersection(pt2, None))

            # build list of segments
            segmentgroup = SegmentGroup()
            for i in range(len(points) - 1):
                in_material = inside != (i % 2 == 1)
                if in_material:
                    continue
                primary_segment = Segment(points[i].point, points[i + 1].point)
                if primary_segment.get_length() < 1e-6:
                    continue
                # entirely in front of the stock: nothing to cut
                if min(points[i].point.Z, points[i + 1].point.Z) >= self.stock.z_max - 1e-6:
                    continue
                if points[i].seg:
                    # starts on the boundary: skip slivers shorter than a step
                    # over, otherwise lead in radially from the level above
                    if primary_segment.get_length() < self.step_over:
                        continue
                    endPt = points[i].point
                    startPt = endPt.project(self.leadin_angle, self.step_over)
                    segmentgroup.add_segment(Segment(startPt, endPt))

                # add the primary segment to the segment group
                segmentgroup.add_segment(primary_segment)

                if points[i + 1].seg:
                    # add a lead out
                    startPt = points[i + 1].point
                    endPt = startPt.project(self.leadout_angle, self.step_over)
                    segmentgroup.add_segment(Segment(startPt, endPt))

            # entering inside the stock: feed in radially along the window
            # edge from the previous (already cut) pass level
            segs = segmentgroup.get_segments()
            if starts_in_stock and segs and segs[0].start.is_same(pt1):
                entry = Segment(Point(x_pos - self.step_over, 0, z_start), pt1)
                segmentgroup.insert_segment(entry, 0)

            x_pos -= self.step_over

            if segmentgroup.count():
                if self._nose_path(segmentgroup).intersects_group(check_group):
                    raise ValueError("Calculated roughing path intersects part")

                self.tool_paths.append(segmentgroup)

        if self.cleanup_pass:
            self.add_cleanup_pass(roughing_segment_group)

    def _nose_path(self, segmentgroup):
        """The path of the nose centre for a path of the imaginary tip."""
        r = getattr(self.tool, "nose_radius", 0) or 0
        if not self.nose_comp or r <= 0.005:
            return segmentgroup
        out = SegmentGroup()
        for seg in segmentgroup.get_segments():
            out.add_segment(Segment(Point(seg.start.X - r, 0, seg.start.Z + r), Point(seg.end.X - r, 0, seg.end.Z + r), seg.bulge))
        return out

    @staticmethod
    def inside_boundary(boundary, x, z, x_far):
        """
        True when the point (x, z) is on the part side of the roughing
        boundary (between the boundary and the axis). Casts a ray from the
        point straight out (toward x_far) and counts boundary crossings: the
        boundary runs from the axis at the front of the part to its back, so
        an odd count means the point is under it. The ray is nudged a hair
        into the pass so a pass that starts level with a boundary vertex is
        not counted twice.
        """
        z -= 1e-5
        ray = Segment(Point(x, 0, z), Point(x_far - 10, 0, z))
        hits = []
        for seg in boundary.get_segments():
            intersect, points = seg.intersect(ray)
            if intersect:
                for p in points:
                    if not any(abs(p.X - h) < 1e-7 for h in hits):
                        hits.append(p.X)
        return len(hits) % 2 == 1

    def add_cleanup_pass(self, roughing_segment_group):
        """
        Add a contour pass along the part at the stock to leave, after the
        roughing passes. The roughing passes leave a staircase where the part
        is tapered or curved (up to one step over high), and leave sections
        alone entirely when there is less than one step over of material on
        them; the cleanup pass takes all of that off, with nose radius
        compensation, so the finishing pass meets an even stock to leave.
        """
        contour = self.offset_contour(roughing_segment_group)
        cleanup = self.contour_path(contour, "Calculated roughing cleanup path intersects part")
        if cleanup is not None:
            # tool_paths are emitted in reverse order: put the cleanup last
            self.tool_paths.insert(0, cleanup)

    def generate_gcode(self):
        """Generate Gcode for the op segments"""

        path = []

        for segmentgroup in reversed(self.tool_paths):
            rough = segmentgroup.to_commands(self.part_segment_group, self.stock, self.step_over, self.finish_passes, self.hfeed, self.vfeed,
                                         retract=self.retract)
            path.extend(rough)

        return path
