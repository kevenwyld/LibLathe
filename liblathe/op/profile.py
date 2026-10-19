import liblathe.op.base
from liblathe.base.point import Point
from liblathe.base.segment import Segment
from liblathe.base.segmentgroup import SegmentGroup


class ProfileOP(liblathe.op.base.BaseOP):

    def generate_path(self):
        """Generate the path for the profile operation"""
        # get the defeature part profile
        profile_segment_group = self.part_segment_group.defeature(self.stock.z_min, self.tool, self.allow_grooving)
        # define the base segment group using the stock to leave as a datum
        base_segment_group = profile_segment_group.offset_path(self.stock_to_leave)
        f_pass = 0
        while f_pass < self.finish_passes:
            # create a new segment group
            segmentgroup = SegmentGroup()
            # generate the offset profile path
            segmentgroup.extend(base_segment_group.offset_path(self.step_over * f_pass))
            # nose compensation, window clipping and lead in
            segmentgroup = self.contour_path(segmentgroup, "Calculated profile path intersects part")
            if segmentgroup is None:
                f_pass += 1
                continue
            # add the segment group to the tool paths
            self.tool_paths.append(segmentgroup)
            # increment f_pass to the next pass
            f_pass += 1

    def generate_gcode(self):
        """Generate Gcode for the op segments"""

        path = []

        for segmentgroup in reversed(self.tool_paths):
            finish = segmentgroup.to_commands(self.part_segment_group, self.stock, self.step_over, self.finish_passes, self.hfeed, self.vfeed)
            path.extend(finish)

        return path
