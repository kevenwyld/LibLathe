"""
Behaviour tests: results the operations must keep, each checked against
its expected value (see the commits that fixed them).
"""
import math
import unittest

import numpy as np

import regression_helpers as R
import liblathe.op.groove as G

BLADE = {"groove_width": 3, "corner_radius": 0.3, "cutting_depth": 30}
M12 = {"pitch": 1.75, "thread_depth": 1.0735, "thread_radius": 6.0, "tool_width": 6}


def feed_radii(commands):
    """Radii (positive) of the feed moves' end points."""
    return [x for mv, x, z in R.moves(commands) if mv in ("G1", "G2", "G3") and x is not None]


class test_Turning(unittest.TestCase):

    def test_profile_negative_stock_turns_under_size(self):
        """Stock to leave X -0.075 turns a 12 mm bar to 11.85: last pass at radius 5.925."""
        cmds = R.make_op("profile", "bar12", {"stock_to_leave_x": -0.075, "stock_to_leave_z": 0}).get_gcode()
        self.assertAlmostEqual(min(r for r in feed_radii(cmds) if r > 1), 5.925, places=3)

    def test_profile_negative_stock_moves_the_front_face(self):
        """Z stock -0.05 with the front face finished puts it within the 0.001 grid of 0.05 deeper."""
        cmds = R.make_op("profile", "bar12", {"stock_to_leave_x": -0.075, "stock_to_leave_z": -0.05,
                                              "face_front": True}).get_gcode()
        face_z = [z for mv, x, z in R.moves(cmds) if mv == "G1" and x is not None and x < 0]
        self.assertTrue(face_z)
        self.assertAlmostEqual(face_z[-1], -0.05, delta=0.003)

    def test_rough_negative_stock(self):
        """Rough with negative X stock stops at the under-size radius."""
        cmds = R.make_op("rough", "bar12", {"stock_to_leave": 0, "stock_to_leave_x": -0.075,
                                            "stock_to_leave_z": 0}).get_gcode()
        self.assertAlmostEqual(max(feed_radii(cmds)), 5.925, places=3)

    def test_face_finishes_to_the_bore(self):
        """Face with a 10 mm min diameter ends a nose radius past the bore (radius 5 - 0.4)."""
        cmds = R.make_op("face", "groove", {"min_dia": 10}).get_gcode()
        self.assertAlmostEqual(min(feed_radii(cmds)), 4.6, places=3)

    def test_face_finishes_the_centre(self):
        """Face on solid stock ends with the nose centre on the axis: no nub."""
        cmds = R.make_op("face", "groove", {}).get_gcode()
        self.assertAlmostEqual(min(feed_radii(cmds)), -0.4, places=3)


class test_ToolOutline(unittest.TestCase):

    def _warnings(self, op, part, tool=None):
        o = R.make_op(op, part, {})
        if tool:
            o.tool.set_tip_angle(tool[0])
            o.tool.set_rotation(tool[1])
        o.get_gcode()
        return [w for w in getattr(o, "warnings", []) if "would hit the part" in w]

    def test_no_false_alarms(self):
        """The test parts with their inserts: nothing but the nose touches the part."""
        for part in ("ball", "groove", "bar12", "m12bar", "dome", "shoulder"):
            for op in ("rough", "profile", "face"):
                with self.subTest(part=part, op=op):
                    self.assertEqual(self._warnings(op, part), [])

    def test_square_shoulder_with_a_swept_back_leading_edge(self):
        """An 80 deg insert at rotation 95 (leading edge at 135 deg) cannot reach into a
        square shoulder's corner: its leading edge would be inside the shoulder."""
        w = self._warnings("profile", "shoulder", (80, 95))
        self.assertEqual(len(w), 1)
        self.assertIn("Z -20.000", w[0])

    def test_square_shoulder_with_a_90_deg_leading_edge(self):
        self.assertEqual(self._warnings("profile", "shoulder", (55, 62.5)), [])


class test_Clearances(unittest.TestCase):

    def _rapid_radii(self, params):
        cmds = R.make_op("rough", "bar12", dict(params, stock_to_leave=0)).get_gcode()
        return sorted({round(x, 3) for mv, x, z in R.moves(cmds) if mv == "G0" and x is not None and x > 0})

    def test_defaults(self):
        """Lift-off: step_over * finish_passes (2); safe position: the clearance (3)."""
        self.assertEqual(self._rapid_radii({}), [8.0, 9.0])

    def test_set_retract_and_radial_clearance(self):
        """Lift-off 0.3 and radial clearance 0.5 off the 12 mm bar."""
        self.assertEqual(self._rapid_radii({"retract": 0.3, "clearance_x": 0.5}), [6.3, 6.5])


class test_LeadIn(unittest.TestCase):
    """The grooved part's front is 1 behind the stock's (Z 0); the window is
    moved with start_offset (from the stock's front)."""

    def _start(self, op, params):
        o = R.make_op(op, "groove", dict(params, stock_to_leave=0))
        cmds = o.get_gcode()
        feeds = [(x, z) for mv, x, z in R.moves(cmds) if mv == "G1"]
        return o, feeds[0]

    def test_only_stock_in_front(self):
        """The range starts at the part's front: the passes lead in from a clearance in
        front of the stock, through the stock in front of the part."""
        for op in ("rough", "profile"):
            with self.subTest(op=op):
                o, (x, z) = self._start(op, {"start_offset": -1.0})
                self.assertAlmostEqual(o.lead_in_used, 1.0 + 3.0)
                self.assertFalse(o.radial_entry())

    def test_part_in_front(self):
        """The range starts inside the part: entered radially."""
        o, _ = self._start("rough", {"start_offset": -10.0})
        self.assertTrue(o.radial_entry())
        self.assertEqual(o.lead_in_used, 0.0)

    def test_set(self):
        o, _ = self._start("rough", {"start_offset": -1.0, "pass_lead_in": 1.5})
        self.assertAlmostEqual(o.lead_in_used, 1.5)


class test_Partoff(unittest.TestCase):

    def test_past_the_centre(self):
        cmds = R.make_op("partoff", "groove", dict(BLADE, end_offset=-60)).get_gcode()
        self.assertAlmostEqual(min(feed_radii(cmds)), -0.2, places=3)

    def test_past_the_bore(self):
        """A tube (min diameter 10) is parted 0.2 past its bore, so it comes free."""
        cmds = R.make_op("partoff", "groove", dict(BLADE, end_offset=-60, min_dia=10)).get_gcode()
        self.assertAlmostEqual(min(feed_radii(cmds)), 4.8, places=3)

    def test_stock_end_is_not_the_chuck(self):
        """Parting at the end of the stock is allowed: the fixture is not assumed."""
        cmds = R.make_op("partoff", "groove", dict(BLADE)).get_gcode()
        self.assertTrue(feed_radii(cmds))


class test_Groove(unittest.TestCase):

    def _clearance(self, params, sx=0.0, sz=0.0):
        """Smallest blade-to-target clearance over all feed positions (mm)."""
        p = dict(BLADE, cutting_depth=10, **params)
        op = R.make_op("groove", "groove", p)
        cmds = op.get_gcode()
        W, Rr = p["groove_width"], p["corner_radius"]

        def edge(u):
            h = np.zeros_like(u)
            left, right = u < Rr, u > W - Rr
            h[left] = Rr - np.sqrt(np.maximum(Rr * Rr - (Rr - u[left]) ** 2, 0))
            h[right] = Rr - np.sqrt(np.maximum(Rr * Rr - (u[right] - (W - Rr)) ** 2, 0))
            return h
        pts = []
        for s in R.part_segments(R.load_part("groove")):
            pts += G._sample_segment(s, 0.01)
        pr = np.array([-x for x, z in pts]); pz = np.array([z for x, z in pts])
        worst, pos = math.inf, None
        for mv, x, z in R.moves(cmds):
            if mv == "G1" and pos is not None:
                n = max(2, int(math.hypot(x - pos[0], z - pos[1]) / 0.02))
                for i in range(n + 1):
                    x0, z0 = pos[0] + (x - pos[0]) * i / n, pos[1] + (z - pos[1]) * i / n
                    m = (pz >= z0 - sz + 0.001) & (pz <= z0 + W + sz - 0.001)
                    if m.any():
                        worst = min(worst, np.min(x0 + edge(pz[m] - z0) - (pr[m] + sx)))
            pos = (x, z)
        return worst

    def test_no_gouge(self):
        self.assertGreater(self._clearance({}), -1e-4)

    def test_no_gouge_with_stock(self):
        self.assertGreater(self._clearance({"stock_to_leave_x": 0.2, "stock_to_leave_z": 0.1}), -1e-4)

    def test_negative_x_stock_cuts_exactly_that_deep(self):
        """X stock -0.05: the blade goes 0.05 into the model, no further."""
        self.assertGreater(self._clearance({"stock_to_leave_x": -0.05, "stock_to_leave_z": 0}, sx=-0.05), -1e-4)


class test_Thread(unittest.TestCase):

    def _thread(self, **extra):
        return R.make_op("thread", "m12bar", dict(M12, S=56.196, F=0.0, **extra))

    def test_root_and_schedule(self):
        """The last pass reaches the depth; passes get shallower; spring pass repeats the last."""
        op = self._thread()
        cmds = op.get_gcode()
        g33 = [c.get_params() for c in cmds if c.get_movement() == "G33"]
        self.assertAlmostEqual(g33[-1]["X"], 6.0 - 1.0735, places=4)
        self.assertAlmostEqual(g33[-2]["X"], g33[-1]["X"], places=6)   # the spring pass
        cuts = [b - a for a, b in zip([0.0] + op.depths[:-1], op.depths)]
        self.assertTrue(all(c2 <= c1 + 1e-9 for c1, c2 in zip(cuts[:-2], cuts[1:-1])))

    def test_g33_then_g1_to_its_end(self):
        """Each G33 carries X and is followed by a zero-length G1 with a feed (for the path display)."""
        cmds = self._thread().get_gcode()
        for i, c in enumerate(cmds):
            if c.get_movement() == "G33":
                p, n = c.get_params(), cmds[i + 1]
                self.assertIn("X", p)
                self.assertEqual(n.get_movement(), "G1")
                self.assertAlmostEqual(n.get_params()["X"], p["X"])
                self.assertAlmostEqual(n.get_params()["Z"], p["Z"])
                self.assertGreater(n.get_params()["F"], 0)

    def test_g76_keeps_q(self):
        cmds = self._thread(canned=True).get_gcode()
        g76 = [c.get_params() for c in cmds if c.get_movement() == "G76"]
        self.assertEqual(len(g76), 1)
        self.assertAlmostEqual(g76[0]["Q"], 29.5)

    def test_shoulder_refused(self):
        with self.assertRaisesRegex(ValueError, "would hit the part behind"):
            R.make_op("thread", "shoulder", dict(M12, S=0.0, F=-20.0)).get_gcode()

    def test_shoulder_suggests_where_to_end(self):
        """The refusal says where the thread must end to clear the insert's flank."""
        with self.assertRaisesRegex(ValueError, r"end the thread at Z -18\.\d+ or above"):
            R.make_op("thread", "shoulder", dict(M12, S=0.0, F=-20.0)).get_gcode()

    def test_feature_in_front_refused(self):
        """A dome of the thread's own radius in front stands above the root: refused, whatever the lead-in."""
        with self.assertRaisesRegex(ValueError, "stands above the thread's root"):
            R.make_op("thread", "dome", {"pitch": 2.5, "thread_depth": 1.5336, "thread_radius": 10.5355,
                                         "tool_width": 6, "S": 65.0, "F": 40.0}).get_gcode()

    def test_runs_to_the_stock_end(self):
        """Threading to the end of the stock is allowed: the fixture is not assumed."""
        cmds = self._thread().get_gcode()
        self.assertAlmostEqual(min(c.get_params()["Z"] for c in cmds if c.get_movement() == "G33"), 0.0, places=4)


if __name__ == '__main__':
    unittest.main()
