import liblathe.op.base
from liblathe.base.point import Point
from liblathe.base.segment import Segment
from liblathe.base.segmentgroup import SegmentGroup


class ProfileOP(liblathe.op.base.BaseOP):

    # apply tool nose radius compensation to the profile path
    nose_comp = True

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
            # check if the generated pass intersects the part. use a small offset to reduce false positives
            if segmentgroup.intersects_group(self.part_segment_group.offset_path(-0.001)):
                raise ValueError("Calculated profile path intersects part")
            # tool nose radius compensation (imaginary tip programming)
            segmentgroup = self.nose_compensate(segmentgroup)
            # keep the whole nose inside the working window
            segmentgroup = segmentgroup.clip_to_box(*self.clip_box())
            if not segmentgroup.count():
                f_pass += 1
                continue
            # add lead in to the profile path
            self.add_leadin(segmentgroup)
            # add the segment group to the tool paths
            self.tool_paths.append(segmentgroup)
            # increment f_pass to the next pass
            f_pass += 1

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

    def generate_gcode(self):
        """Generate Gcode for the op segments"""

        path = []

        for segmentgroup in reversed(self.tool_paths):
            finish = segmentgroup.to_commands(self.part_segment_group, self.stock, self.step_over, self.finish_passes, self.hfeed, self.vfeed)
            path.extend(finish)

        return path
