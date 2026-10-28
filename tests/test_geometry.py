import math
import os
import sys
import unittest

import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from liblathe.base.geometry import dilate_rect_profile, sample_points, simplify_polyline
from liblathe.base.point import Point
from liblathe.base.segment import Segment


class test_geometry(unittest.TestCase):

    def test_sample_points_line_and_arc(self):
        pts = sample_points(Segment(Point(0, 0, 0), Point(-3, 0, -4)), 1.0)
        self.assertEqual((pts[0], pts[-1], len(pts)), ((0, 0), (-3, -4), 6))
        arc = Segment(Point(0, 0, 0), Point(-2, 0, -2))
        arc.set_bulge(math.pi / 2)
        pts = sample_points(arc, 0.1)
        self.assertAlmostEqual(pts[-1][0], -2)
        self.assertAlmostEqual(pts[-1][1], -2)
        c, r = arc.get_centre_point(), arc.get_radius()
        self.assertTrue(all(abs(math.hypot(x - c.X, z - c.Z) - r) < 1e-9 for x, z in pts))

    def test_simplify_polyline(self):
        line = [(0.0, float(z)) for z in range(10)] + [(-1.0, 9.0)]
        self.assertEqual(simplify_polyline(line, 1e-6), [(0.0, 0.0), (0.0, 9.0), (-1.0, 9.0)])

    def test_dilate_rect_profile(self):
        values = np.full(41, -np.inf)
        values[20] = 5.0
        grown = dilate_rect_profile(values, 0.5, 0.07, 0.005)
        self.assertEqual(int(np.isfinite(grown).sum()), 2 * 14 + 1)
        self.assertEqual(float(grown.max()), 5.5)
        self.assertEqual(float(values[19]), -np.inf)
        shrunk = dilate_rect_profile(np.full(5, 1.0), -2.0, 0.0, 0.005)
        self.assertEqual(shrunk.tolist(), [0.0] * 5)


if __name__ == '__main__':
    unittest.main()
