"""
Helpers for the regression tests: build an operation from a test part (a
JSON outline and stock) and parameters, and write its G-code as text.

A part file holds "segs": [[x1, z1, x2, z2, bulge], ...] (X negative, the
outline from the front on the axis to the back), "stock": [xmin, xmax,
zmin, zmax] and "tool": the turning insert's TipAngle, EdgeLength,
TipRadius and Rotation.
"""
import json
import os
import sys

# run from the repository root (unittest discover -s tests) or from tests/
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from liblathe.base.point import Point
from liblathe.base.segment import Segment
from liblathe.base.boundbox import BoundBox
from liblathe.base.tool import Tool
import liblathe.op.rough
import liblathe.op.profile
import liblathe.op.face
import liblathe.op.partoff
import liblathe.op.groove
import liblathe.op.thread
import liblathe.op.chamfer

HERE = os.path.dirname(os.path.abspath(__file__))
OPS = {"rough": liblathe.op.rough.RoughOP, "profile": liblathe.op.profile.ProfileOP,
       "face": liblathe.op.face.FaceOP, "partoff": liblathe.op.partoff.PartoffOP,
       "groove": liblathe.op.groove.GrooveOP, "thread": liblathe.op.thread.ThreadOP,
       "chamfer": liblathe.op.chamfer.ChamferOP}
# the ops that take a turning insert (the others carry their tool in params)
TURNING = ("rough", "profile", "face", "chamfer")
DEFAULTS = {'step_over': 1.0, 'finish_passes': 2, 'stock_to_leave': 0, 'clearance': 3, 'min_dia': 0,
            'extra_dia': 0, 'hfeed': 200, 'vfeed': 100, 'start_offset': 0, 'end_offset': 0,
            'allow_grooving': False}


def load_part(name):
    with open(os.path.join(HERE, "data", name + ".json")) as fp:
        return json.load(fp)


def part_segments(part):
    segs = []
    for x1, z1, x2, z2, b in part["segs"]:
        s = Segment(Point(x1, 0, z1), Point(x2, 0, z2))
        s.bulge = b
        segs.append(s)
    return segs


def make_op(opname, partname, params=None):
    """The operation, set up on the part; params override the defaults. For
    the thread op, "S" and "F" give the thread's Z range (front, back)."""
    part = load_part(partname)
    xmin, xmax, zmin, zmax = part["stock"]
    p = dict(DEFAULTS)
    if opname == "rough":
        p["stock_to_leave"] = 0.25
    p.update(params or {})
    if opname == "thread":
        front, back = p.pop("S", zmax), p.pop("F", zmin)
        p["start_offset"], p["end_offset"] = front - zmax, zmin - back
    op = OPS[opname]()
    op.set_params(p)
    op.add_stock(BoundBox(Point(xmin, 0, zmin), Point(0, 0, zmax)))
    op.add_part_edges(part_segments(part))
    if opname in TURNING:
        t, tool = part["tool"], Tool()
        tool.set_tip_angle(t["TipAngle"])
        tool.set_edge_length(t["EdgeLength"])
        tool.set_nose_radius(t["TipRadius"])
        tool.set_rotation(t["Rotation"])
        op.add_tool(tool)
    return op


def gcode_text(commands):
    lines = []
    for c in commands:
        words = " ".join("%s%.4f" % (k, v) for k, v in sorted(c.get_params().items()))
        lines.append((c.get_movement() + " " + words).strip())
    return "".join(line + "\n" for line in lines)   # an empty path: an empty file


def moves(commands):
    """(movement, x, z) for every move, X and Z carried from the move before."""
    out, x, z = [], None, None
    for c in commands:
        p = c.get_params()
        x, z = p.get("X", x), p.get("Z", z)
        if c.get_movement() in ("G0", "G1", "G2", "G3", "G33"):
            out.append((c.get_movement(), x, z))
    return out
