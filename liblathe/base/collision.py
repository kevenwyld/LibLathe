"""
Tool outline collision check for turning inserts.

The operations place the insert's imaginary tip on the path and check the
nose against the part. This checks the rest of the insert: its outline,
swept along the path, against the part (the surface the operation may cut
to). It catches what the nose check cannot: the insert's back or leading
edge, or its body, running into the part (a wall steeper than the insert
can follow, a pass ending in a groove).

The insert, in the tool's frame (u radially outward, w along Z toward the
tailstock, the imaginary tip at the origin): a rhombus of side EdgeLength
with the cutting corner's included angle TipAngle, its bisector at Rotation
from +w toward +u, and a nose of TipRadius whose centre is at (r, r).
"""
import math

import numpy as np

from liblathe.base.segmentgroup import _sample_points

GRID = 0.02          # raster cell (mm)
TOLERANCE = 0.01     # the part is shrunk by this: touching is not a hit


def insert_outline(tool, step=0.1):
    """Points (u, w) on the insert's outline, outside its cutting corner
    (the nose and the first stretch of each edge next to it, which cut)."""
    r = float(tool.nose_radius or 0)
    eps = math.radians(float(tool.tip_angle))
    beta = math.radians(float(tool.tool_rotation))
    L = float(tool.edge_length)
    if L <= 0 or eps <= 0:
        return []
    b = (math.sin(beta), math.cos(beta))
    c = (r, r)
    corner = (c[0] - r / math.sin(eps / 2) * b[0], c[1] - r / math.sin(eps / 2) * b[1])
    d1 = (math.sin(beta - eps / 2), math.cos(beta - eps / 2))
    d2 = (math.sin(beta + eps / 2), math.cos(beta + eps / 2))
    verts = [corner,
             (corner[0] + L * d1[0], corner[1] + L * d1[1]),
             (corner[0] + L * (d1[0] + d2[0]), corner[1] + L * (d1[1] + d2[1])),
             (corner[0] + L * d2[0], corner[1] + L * d2[1])]
    # the cutting corner: where the nose meets each edge, and a little more
    skip = r / math.tan(eps / 2) + 3 * TOLERANCE + 0.2 * L
    pts = []
    for (a, bb) in zip(verts, verts[1:] + verts[:1]):
        n = max(1, int(math.ceil(math.hypot(bb[0] - a[0], bb[1] - a[1]) / step)))
        for i in range(n):
            p = (a[0] + (bb[0] - a[0]) * i / n, a[1] + (bb[1] - a[1]) * i / n)
            if math.hypot(p[0] - corner[0], p[1] - corner[1]) > skip:
                pts.append(p)
    return pts


def rasterize(segmentgroup, z_lo, z_hi, r_hi):
    """
    The region inside an outline (a profile from the front on the axis to
    the back, X negative) as a grid: cell [i, j] is material at
    z = z_lo + i * GRID, radius j * GRID. Scanlines along r: at each z the
    outline's crossings, filled in pairs from the axis outward.
    """
    pts = []
    for seg in segmentgroup.get_segments():
        s = _sample_points(seg, GRID / 2)
        pts.extend(s if not pts else s[1:])
    if len(pts) < 2:
        return None
    pts.append((0.0, pts[-1][1]))
    pts.append((0.0, pts[0][1]))           # close along the axis
    rs = np.array([-x for x, z in pts])
    zs = np.array([z for x, z in pts])
    nz = int(math.ceil((z_hi - z_lo) / GRID)) + 1
    nr = int(math.ceil(r_hi / GRID)) + 1
    grid = np.zeros((nz, nr), dtype=bool)
    r0, r1, z0, z1 = rs[:-1], rs[1:], zs[:-1], zs[1:]
    for i in range(nz):
        z = z_lo + i * GRID
        cross = ((z0 <= z) & (z1 > z)) | ((z1 <= z) & (z0 > z))
        if not cross.any():
            continue
        t = (z - z0[cross]) / (z1[cross] - z0[cross])
        rc = np.sort(r0[cross] + t * (r1[cross] - r0[cross]))
        for a, b in zip(rc[0::2], rc[1::2]):
            j0, j1 = max(0, int(math.ceil(a / GRID))), min(nr - 1, int(math.floor(b / GRID)))
            if j1 >= j0:
                grid[i, j0:j1 + 1] = True
    return grid


def path_points(commands, step=0.05):
    """The tool tip's positions (radius, z) along the moves, sampled."""
    out, x, z = [], None, None
    for c in commands:
        name, p = c.get_movement(), c.get_params()
        nx, nz = p.get("X", x), p.get("Z", z)
        known = None not in (x, z, nx, nz)      # (the first moves give X before Z)
        if name in ("G0", "G1") and known:
            n = max(1, int(math.ceil(math.hypot(nx - x, nz - z) / step)))
            out.extend((x + (nx - x) * i / n, z + (nz - z) * i / n) for i in range(n + 1))
        elif name in ("G2", "G3") and known:
            cx, cz = x + p.get("I", 0.0), z + p.get("K", 0.0)
            rad = math.hypot(x - cx, z - cz)
            a0, a1 = math.atan2(x - cx, z - cz), math.atan2(nx - cx, nz - cz)
            sweep = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi     # the shorter way round
            n = max(1, int(math.ceil(abs(sweep) * rad / step)))
            out.extend((cx + rad * math.sin(a0 + sweep * i / n), cz + rad * math.cos(a0 + sweep * i / n))
                       for i in range(n + 1))
        x, z = nx, nz
    return out


def check(commands, tool, target):
    """
    Runs of the path along which the insert's outline enters the target:
    a list of (z_from, z_to, depth) with the tip's Z range and how far the
    insert reaches into the part (mm, to the grid).
    """
    outline = insert_outline(tool)
    tips = path_points(commands)
    if not outline or not tips or target is None or not target.count():
        return []
    shrunk = target.offset_path(-TOLERANCE)
    bb_z = [z for seg in target.get_segments() for z in (seg.start.Z, seg.end.Z)]
    bb_r = [-x for seg in target.get_segments() for x in (seg.start.X, seg.end.X)]
    z_lo, z_hi, r_hi = min(bb_z) - 1.0, max(bb_z) + 1.0, max(bb_r) + 1.0
    grid = rasterize(shrunk, z_lo, z_hi, r_hi)
    if grid is None:
        return []
    U = np.array([u for u, w in outline])
    W = np.array([w for u, w in outline])
    hits = []
    for tr, tz in tips:
        pr, pz = tr + U, tz + W
        i = np.round((pz - z_lo) / GRID).astype(int)
        j = np.round(pr / GRID).astype(int)
        ok = (i >= 0) & (i < grid.shape[0]) & (j >= 0) & (j < grid.shape[1])
        inside = np.zeros_like(ok)
        inside[ok] = grid[i[ok], j[ok]]
        if inside.any():
            # how deep: the material above each point inside, in its column
            depth = 0.0
            for ii, jj in zip(i[inside], j[inside]):
                col = grid[ii, jj:]
                free = np.argmin(col) if not col.all() else len(col)
                depth = max(depth, free * GRID)
            hits.append((tz, depth))
    runs = []
    for z, d in hits:
        if runs and abs(z - runs[-1][1]) <= 0.5:
            runs[-1] = (runs[-1][0], z, max(runs[-1][2], d))
        else:
            runs.append((z, z, d))
    # one report per place: runs from different passes over the same Z merge
    spans = sorted((min(a, b), max(a, b), d) for a, b, d in runs)
    merged = []
    for lo, hi, d in spans:
        if merged and lo <= merged[-1][1] + 0.5:
            merged[-1] = (merged[-1][0], max(merged[-1][1], hi), max(merged[-1][2], d))
        else:
            merged.append((lo, hi, d))
    return [(hi, lo, d) for lo, hi, d in sorted(merged, key=lambda m: -m[1])]
