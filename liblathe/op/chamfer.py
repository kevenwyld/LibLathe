"""
Chamfers or fillets at chosen corners, each as (radius, z, kind, size): the outline is changed there and
a short nose-compensated contour pass runs round each corner, as Profile's finishing pass does.
"""

import liblathe.op.base
from liblathe.base import corners as corner_tools


class ChamferOP(liblathe.op.base.BaseOP):

    checks_tool_outline = True

    def __init__(self):
        super().__init__()
        # [(radius, z, kind, size)]: the corners to chamfer or fillet
        self.corners = []

    def generate_path(self):
        self.tool_paths = []
        self.corners_done = []
        found = corner_tools.find_corners(self.part_segment_group)
        picks, wanted = {}, {}
        for radius, z, kind, size in self.corners:
            match = [c for c in found if abs(-c["point"].X - radius) < 1e-3 and abs(c["point"].Z - z) < 1e-3]
            if not match:
                self.warnings.append("No sharp corner between straight edges at radius %.3f, Z %.3f" % (radius, z))
                continue
            c = match[0]
            if not c["outside"]:
                self.warnings.append("Radius %.3f, Z %.3f is an inside corner: left alone" % (radius, z))
                continue
            picks[c["index"]] = (kind, size)
            wanted[c["index"]] = (c, size)
        self.part_segment_group, done = corner_tools.replace_corners(self.part_segment_group, picks)
        for index in wanted:
            if index not in done:
                c, size = wanted[index]
                self.warnings.append("Edges at radius %.3f, Z %.3f too short for a %.3f %s: left alone"
                                     % (-c["point"].X, c["point"].Z, size, picks[index][0]))
        # each corner as Profile would finish it with a Z range and a minimum
        # diameter round it: the whole (changed) outline, its working window
        # limited to the corner and a millimetre of the edges either side
        saved = (self.start_offset, self.end_offset, self.min_dia)
        try:
            for index in done:
                c, size = wanted[index]
                p = c["point"]
                reach = size * 2 + 1.0
                self.start_offset = (p.Z + reach) - self.stock.z_max
                self.end_offset = self.stock.z_min - (p.Z - reach)
                self.min_dia = max(saved[2], 2 * max(0.0, -p.X - reach))
                path = self.contour_path(self.offset_contour(self.part_segment_group),
                                         "Calculated chamfer path intersects part")
                if path is not None:
                    self.tool_paths.append(path)
                    self.corners_done.append((-p.X, p.Z))
        finally:
            self.start_offset, self.end_offset, self.min_dia = saved

    def generate_gcode(self):
        path = []
        for segmentgroup in self.tool_paths:
            path.extend(segmentgroup.to_commands(self.part_segment_group, self.stock, self.step_over,
                                                 self.finish_passes, self.hfeed, self.feed_in(), retract=self.retract))
        return path
