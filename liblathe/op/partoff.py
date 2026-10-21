import liblathe.op.base
from liblathe.base.command import Command


class PartoffOP(liblathe.op.base.BaseOP):
    """
    Part off with a parting or grooving insert: one plunge from outside the
    stock past the centreline.

    The insert is described like GrooveOP's (groove_width, corner_radius,
    cutting_depth) and programmed to the same reference point: X at the
    cutting edge's point closest to the axis, Z at its chuck-side flank. The
    blade's part-side flank lands on the window end (z_end(), the part's
    back face), so the part keeps that face and the cut is made in the
    material on the chuck side: the reference Z is z_end() - width.

    The plunge goes `overtravel` past the centreline so no pip is left, or to
    the minimum diameter if one is set. If the insert's cutting depth cannot
    reach that far from the outer limit, it stops at its reach and says so.
    """

    needs_tool = False

    groove_width = 0
    corner_radius = 0
    cutting_depth = 0
    overtravel = 0.2

    def generate_path(self):
        self.warnings = []
        w = float(self.groove_width)
        if w <= 0:
            raise ValueError("Parting tool width must be greater than zero")
        self.z_ref = self.z_end() - w
        if self.z_ref < self.stock.z_min - 1e-6:
            raise ValueError(
                "Parting off at Z %.3f needs the blade's width (%.3f) of stock behind the part's back face, "
                "but the stock ends at Z %.3f: the blade would cut past the end of the stock (toward the chuck)"
                % (self.z_end(), w, self.stock.z_min))
        r_outer = -self.x_outer()
        self.r_start = r_outer + self.clearance
        self.r_end = -self.x_inner() if self.min_dia > 0 else -float(self.overtravel)
        if self.cutting_depth > 0 and r_outer - self.cutting_depth > self.r_end + 1e-9:
            reach = r_outer - self.cutting_depth
            self.warnings.append(
                "The insert's grooving depth (%.3f) cannot reach the centreline from the max diameter "
                "(radius %.3f): the plunge stops at radius %.3f and does not part off"
                % (self.cutting_depth, r_outer, reach))
            self.r_end = reach

    def generate_gcode(self):
        return [Command('G18', {}),
                Command('G0', {'X': self.r_start, 'Z': self.z_ref}),
                Command('G1', {'X': self.r_end, 'Z': self.z_ref, 'F': self.vfeed}),
                Command('G0', {'X': self.r_start, 'Z': self.z_ref})]
