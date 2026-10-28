"""
Find the sharp corners between straight edges of a part outline and replace them with chamfers or fillets.
Outside corners (turning with the loop) remove material when broken; inside corners and arcs are left alone.
"""

import math

from liblathe.base.point import Point
from liblathe.base.segment import Segment
from liblathe.base.segmentgroup import SegmentGroup

# a corner turns at least this much (smaller: a smooth transition, or noise)
SHARP = math.radians(10.0)
TOL = 1e-6


def _direction(seg):
    dx, dz = seg.end.X - seg.start.X, seg.end.Z - seg.start.Z
    n = math.hypot(dx, dz)
    return (dx / n, dz / n) if n > TOL else (0.0, 0.0)


def _edges(group):
    """The outline's segments, consecutive collinear straight ones joined: a
    list of [start, end, bulge] (Points)."""
    edges = []
    for seg in group.get_segments():
        if seg.start.distance_to(seg.end) < TOL:
            continue
        if edges and seg.bulge == 0 and edges[-1][2] == 0:
            prev = Segment(edges[-1][0], edges[-1][1])
            a, b = _direction(prev), _direction(seg)
            if abs(a[0] * b[1] - a[1] * b[0]) < 1e-9 and a[0] * b[0] + a[1] * b[1] > 0:
                edges[-1][1] = seg.end
                continue
        edges.append([seg.start, seg.end, seg.bulge])
    return edges


def _orientation(edges):
    """+1 for an anticlockwise loop (X across, Z up), -1 for clockwise."""
    area = 0.0
    for start, end, _ in edges:
        area += start.X * end.Z - end.X * start.Z
    return 1.0 if area >= 0 else -1.0


def find_corners(group):
    """Sharp corners between straight edges as dicts: index, point, in, out, turn, outside, len_in, len_out."""
    edges = _edges(group)
    if len(edges) < 2:
        return []
    orient = _orientation(edges)
    found = []
    for i in range(len(edges) - 1):
        e1, e2 = edges[i], edges[i + 1]
        if e1[2] != 0 or e2[2] != 0 or not e1[1].is_same(e2[0]):
            continue
        a, b = _direction(Segment(e1[0], e1[1])), _direction(Segment(e2[0], e2[1]))
        cross = a[0] * b[1] - a[1] * b[0]
        turn = math.atan2(abs(cross), a[0] * b[0] + a[1] * b[1])
        if turn < SHARP:
            continue
        found.append({
            "index": i, "point": e1[1], "in": a, "out": b, "turn": turn,
            "outside": cross * orient > 0,
            "len_in": e1[0].distance_to(e1[1]), "len_out": e2[0].distance_to(e2[1]),
        })
    return found


def replace_corners(group, picks):
    """
    New outline with corners replaced; picks: {index: ("chamfer", size) | ("fillet", radius)}.
    Corners whose edges are too short are left. Returns (group, indices changed).
    """
    edges = _edges(group)
    corners = {c["index"]: c for c in find_corners(group)}
    orient = _orientation(edges)
    starts = [e[0] for e in edges]
    ends = [e[1] for e in edges]
    joins = {}
    done = []
    for index, (kind, size) in picks.items():
        c = corners.get(index)
        if c is None or size <= 0:
            continue
        if kind == "fillet":
            back = size * math.tan(c["turn"] / 2)
        else:
            back = size
        if back >= c["len_in"] - TOL or back >= c["len_out"] - TOL:
            continue
        p, a, b = c["point"], c["in"], c["out"]
        p_in = Point(p.X - back * a[0], 0.0, p.Z - back * a[1])
        p_out = Point(p.X + back * b[0], 0.0, p.Z + back * b[1])
        ends[index] = p_in
        starts[index + 1] = p_out
        bulge = 0.0
        if kind == "fillet":
            # the arc turns the way the corner does; in liblathe's bulge sign
            # (its arcs' centre side) that is the opposite sign of the turn's
            # cross product in X, Z
            turning = 1.0 if (a[0] * b[1] - a[1] * b[0]) > 0 else -1.0
            bulge = -turning * math.tan(c["turn"] / 4)
        joins[index] = Segment(p_in, p_out, bulge)
        done.append(index)
    out = SegmentGroup()
    for i, e in enumerate(edges):
        out.add_segment(Segment(starts[i], ends[i], e[2]))
        if i in joins:
            out.add_segment(joins[i])
    return out, sorted(done)


def break_edges(group, size, z_min=None):
    """
    Every sharp outside corner chamfered by size: not corners on the axis, nor
    at the part's back end (z_min: left to parting off), nor those whose edges
    are too short. Returns (group, the corners' points changed).
    """
    picks = {}
    points = {}
    for c in find_corners(group):
        p = c["point"]
        if not c["outside"] or abs(p.X) < 1e-4:
            continue
        if z_min is not None and p.Z <= z_min + 1e-4:
            continue
        picks[c["index"]] = ("chamfer", size)
        points[c["index"]] = p
    new, done = replace_corners(group, picks)
    return new, [points[i] for i in done]
