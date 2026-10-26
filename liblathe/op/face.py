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
        z_min = part_boundbox.z_max + self.stock_xz()[1]
        z_max = self.stock.z_max + self.start_offset

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

        self.add_front_chamfer(part_boundbox)

    def add_front_chamfer(self, part_boundbox):
        """
        With edge breaking, the front corner (the face meeting the diameter)
        is chamfered: the facing passes go straight across the face, so a
        short contour pass follows the broken outline round the corner (a
        millimetre of the face, the chamfer, a millimetre of the diameter),
        last.
        """
        front = part_boundbox.z_max
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
