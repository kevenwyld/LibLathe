from liblathe.base.segmentgroup import SegmentGroup


class BaseOP:
    """Base class for all turning operations."""

    def __init__(self):

        self.stock = None
        self.tool = None
        self.part_segment_group = SegmentGroup()

        self.tool_paths = []

        self.min_dia = 0
        self.extra_dia = 0
        self.start_offset = 0
        self.end_offset = 0
        self.allow_grooving = False
        self.step_over = 1.5
        self.finish_passes = 1
        self.stock_to_leave = 0
        self.hfeed = 100
        self.vfeed = 50
        self.clearance = 3

        self.leadin_angle = 270
        self.leadout_angle = 315
        # TODO: implement invert_x to pass to segment_group().to_commands()

    # The working window. Operations only move the tool inside it:
    #   Z from z_end() (toward the chuck) to z_start() (toward the tailstock)
    #   X from x_outer() (largest radius) to x_inner() (smallest radius)
    # X is negative internally (the part profile is on the -X side).

    def z_start(self):
        """Z where the operation starts: the stock front plus start_offset."""
        return self.stock.z_max + self.start_offset

    def z_end(self):
        """Z where the operation must stop: the stock end minus end_offset."""
        return self.stock.z_min - self.end_offset

    def x_outer(self):
        """Largest radius (most negative X): the stock radius plus extra_dia / 2."""
        return self.stock.x_min - self.extra_dia * 0.5

    def x_inner(self):
        """Smallest radius (least negative X): min_dia / 2."""
        return 0 - self.min_dia * 0.5

    def starts_in_stock(self):
        """True when z_start() is behind the front of the stock, i.e. there is
        uncut material in front of the window."""
        return self.z_start() < self.stock.z_max - 1e-6

    def set_params(self, params):
        """Set operations parameters"""

        for param in params:
            if hasattr(self, param):
                setattr(self, param, params[param])
            else:
                raise Warning("Attempting to set undefined parameter '%s'" % param)

    def get_params(self):
        """Return operations parameters"""
        return {'min_dia': self.min_dia, 'extra_dia': self.extra_dia, 'start_offset': self.start_offset,
                'end_offset': self.end_offset, 'allow_grooving': self.allow_grooving, 'step_over': self.step_over,
                'finish_passes': self.finish_passes, 'stock_to_leave': self.stock_to_leave, 'hfeed': self.hfeed, 
                'vfeed': self.vfeed, 'clearance': self.clearance}

    def get_gcode(self):
        """Base function for all turning operations"""

        if self.tool is None:
            raise Warning("Tool is unset")

        self.generate_path()
        path = self.generate_gcode()
        return path

    def generate_path(self):
        """Main processing function for each op"""

        pass

    def generate_gcode(self):
        """Generate Gcode for the op segments"""

        return ""

    def add_part_edges(self, part_edges):
        """Add edges to define the part geometry part_edges = array of LibLathe segments"""

        for segment in part_edges:
            self.part_segment_group.add_segment(segment)

        self.part_segment_group.validate()
        # self.part_segment_group.create_freecad_shape('part_segment_group')

    def add_stock(self, stock_bb):
        """Define bounding box for the stock material stock_bb = LibLathe BoundBox"""
        self.stock = stock_bb

    def add_tool(self, tool):
        """Set the tool for the operation"""
        self.tool = tool
