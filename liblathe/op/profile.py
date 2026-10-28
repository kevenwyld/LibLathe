import liblathe.op.base
from liblathe.base.segmentgroup import SegmentGroup


class ProfileOP(liblathe.op.base.BaseOP):

    checks_tool_outline = True
    breaks_edges = True

    def generate_path(self):
        """Generate the path for the profile operation"""
        # get the defeature part profile
        profile_segment_group = self.defeatured()

        # define the base segment group using the stock to leave as a datum
        base_segment_group = self.offset_contour(profile_segment_group)
        f_pass = 0
        while f_pass < self.finish_passes:
            # create a new segment group
            segmentgroup = SegmentGroup()
            # generate the offset profile path
            spacing = self.finish_step_over if self.finish_step_over and self.finish_step_over > 0 else self.step_over
            segmentgroup.extend(base_segment_group.offset_path(spacing * f_pass))
            # nose compensation, window clipping and lead in
            # facing is the Face operation's job: unless asked to, the profile
            # leaves out the front face and starts at the front corner
            segmentgroup = self.contour_path(segmentgroup, "Calculated profile path intersects part",
                                             drop_front_face=not self.face_front)
            if segmentgroup is None:
                f_pass += 1
                continue
            # add the segment group to the tool paths
            self.tool_paths.append(segmentgroup)
            # increment f_pass to the next pass
            f_pass += 1

        # spring pass: the last (innermost) path once more. tool_paths are
        # emitted in reverse, so the innermost is tool_paths[0]
        if self.spring_pass and self.tool_paths:
            self.tool_paths.insert(0, self.tool_paths[0])

    def generate_gcode(self):
        """Generate Gcode for the op segments"""

        path = []

        for segmentgroup in reversed(self.tool_paths):
            finish = segmentgroup.to_commands(self.part_segment_group, self.stock, self.step_over, self.finish_passes, self.hfeed, self.feed_in(),
                                         retract=self.retract)
            path.extend(finish)

        return path
