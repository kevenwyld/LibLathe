import math

import liblathe.op.base
from liblathe.base.point import Point
from liblathe.base.segment import Segment
from liblathe.base.segmentgroup import SegmentGroup


class FaceOP(liblathe.op.base.BaseOP):

    checks_tool_outline = True
    breaks_edges = True

    def generate_path(self):
        """Generate the path for the profile operation"""
        part_boundbox = self.part_segment_group.boundbox()
        x_min = self.stock.x_min - self.extra_dia * 0.5 - self.radial_clearance()
        # end with the nose's centre on the inner limit (the centreline, or the
        # minimum diameter, e.g. a bore): ending the imaginary tip there leaves
        # the nose's round uncut at the inner edge (the nub at the centre)
        r_nose = getattr(self.tool, "nose_radius", 0) or 0 if self.nose_comp else 0
        x_max = 0 - self.min_dia * 0.5 + r_nose
        # face down to face_z (a selected face, or Back set by hand), or the
        # part's front
        face = part_boundbox.z_max if self.face_z is None else float(self.face_z)
        z_min = face + self.stock_xz()[1]
        z_max = self.stock.z_max + self.start_offset
        self.check_faced_part(z_min, z_max)

        self.clearing_paths = []

        # TODO: Move the final pass to finishing passes for a slower pass
        # work backwards from z_min to z_max adding a segmentgroup for each stepover
        z_pos = z_min
        while z_pos < z_max:
            segmentgroup = SegmentGroup()
            pt1 = Point(x_min, 0, z_pos)
            pt2 = Point(x_max, 0, z_pos)
            seg = Segment(pt1, pt2)
            segmentgroup.add_segment(seg)

            pt3 = pt2.project(self.leadout_angle, self.step_over)
            leadout = Segment(pt2, pt3)
            segmentgroup.add_segment(leadout)
            z_pos += self.step_over
            self.clearing_paths.append(segmentgroup)

        self.add_front_chamfer(part_boundbox, face)

    def part_radius_above(self, z_lo, z_hi):
        """The part's largest radius between z_lo and z_hi (its outline's
        segments there, lines exactly, arcs by their end points and middle)."""
        best = 0.0
        for seg in self.part_segment_group.get_segments():
            pts = [seg.start, seg.end]
            if seg.bulge:
                pts.append(seg.start.lerp(seg.end, 0.5))
            lo, hi = min(seg.start.Z, seg.end.Z), max(seg.start.Z, seg.end.Z)
            if hi < z_lo - 1e-9 or lo > z_hi + 1e-9:
                continue
            for p in pts:
                if z_lo - 1e-9 <= p.Z <= z_hi + 1e-9:
                    best = max(best, -p.X)
            # a line crossing the band's ends: its radius there
            if not seg.bulge and hi - lo > 1e-12:
                for zz in (z_lo, z_hi):
                    if lo <= zz <= hi:
                        t = (zz - seg.start.Z) / (seg.end.Z - seg.start.Z)
                        best = max(best, -(seg.start.X + t * (seg.end.X - seg.start.X)))
        return best

    def check_faced_part(self, z_min, z_max):
        """Facing passes go in to the minimum diameter: refuse when the part
        reaches further out than that in front of the plane faced to (e.g. a
        shoulder faced without Min Diameter at its smaller diameter)."""
        r_in = self.min_dia * 0.5
        r_part = self.part_radius_above(z_min + 1e-4, z_max)
        if r_part > r_in + 1e-4:
            self.refuse("face-cuts-part",
                        "Facing to Z %.3f would cut into the part (radius %.3f in front of it): set Min Diameter "
                        "to at least %.3f" % (z_min, r_part, 2 * r_part))

    def add_front_chamfer(self, part_boundbox, front=None):
        """With edge breaking, a short contour pass round the chamfered front corner, after the facing passes."""
        front = part_boundbox.z_max if front is None else front
        corner = [p for p in self.edges_broken if abs(p.Z - front) < 1e-4]
        if not corner:
            return
        p, size = corner[0], float(self.break_edges)
        inf = float("inf")
        piece = self.part_segment_group.clip_to_box(-inf, p.X + size + 1.0, p.Z - size - 1.0, inf)
        if not piece.count():
            return
        contour = self.offset_contour(piece)
        path = self.contour_path(contour, "Calculated chamfer path intersects part")
        if path is not None:
            # clearing paths are emitted in reverse order: the chamfer last
            self.clearing_paths.insert(0, path)

    def generate_gcode(self):
        """Generate Gcode for the op segments"""

        path = []
        for segmentgroup in reversed(self.clearing_paths):
            rough = segmentgroup.to_commands(self.part_segment_group, self.stock, self.step_over, self.finish_passes, self.hfeed, self.vfeed,
                                         retract=self.retract)
            path.extend(rough)

        return path
