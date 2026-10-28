import liblathe.op.base
from liblathe.base.command import Command


class PartoffOP(liblathe.op.base.BaseOP):
    """
    One parting plunge with the blade's part-side flank on z_end(), programmed like GrooveOP (X at the
    edge's lowest point, Z at the chuck-side flank), `overtravel` past the centreline or min diameter.
    """

    needs_tool = False

    groove_width = 0
    corner_radius = 0
    cutting_depth = 0
    overtravel = 0.2
    # pecking: peck_depth > 0 feeds in that far at a time, backing off
    # peck_retract between pecks (or out of the cut, with peck_full_retract)
    peck_depth = 0.0
    peck_retract = 0.2
    peck_full_retract = False

    def generate_path(self):
        self.warnings = []
        w = float(self.groove_width)
        if w <= 0:
            raise ValueError("Parting tool width must be greater than zero")
        self.z_ref = self.z_end() - w
        r_outer = -self.x_outer()
        self.r_start = r_outer + self.radial_clearance()
        # past the inner limit by the overtravel: the centreline, or the
        # minimum diameter (the bore of a tube), so the part comes free
        self.r_end = (-self.x_inner() if self.min_dia > 0 else 0.0) - float(self.overtravel)
        if self.cutting_depth > 0 and r_outer - self.cutting_depth > self.r_end + 1e-9:
            reach = r_outer - self.cutting_depth
            self.warnings.append(
                "The insert's grooving depth (%.3f) cannot reach the centreline from the max diameter "
                "(radius %.3f): the plunge stops at radius %.3f and does not part off"
                % (self.cutting_depth, r_outer, reach))
            self.r_end = reach

    def generate_gcode(self):
        cmds = [Command('G18', {}), Command('G0', {'X': self.r_start, 'Z': self.z_ref})]
        peck = float(self.peck_depth or 0.0)
        if peck > 0:
            # from the stock's surface in, peck_depth at a time
            x = min(-self.x_outer(), self.r_start)
            while x - peck > self.r_end + 1e-6:
                x -= peck
                cmds.append(Command('G1', {'X': x, 'Z': self.z_ref, 'F': self.vfeed}))
                back = self.r_start if self.peck_full_retract else min(x + float(self.peck_retract), self.r_start)
                cmds.append(Command('G0', {'X': back, 'Z': self.z_ref}))
                if self.peck_full_retract:
                    cmds.append(Command('G0', {'X': min(x + float(self.peck_retract), self.r_start), 'Z': self.z_ref}))
        cmds.append(Command('G1', {'X': self.r_end, 'Z': self.z_ref, 'F': self.vfeed}))
        cmds.append(Command('G0', {'X': self.r_start, 'Z': self.z_ref}))
        return cmds
