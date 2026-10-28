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

    def test_nothing_cut_outside_the_range(self):
        """Approached along Z through the stock in front, but nothing in front of the
        part (Z > -1) is cut below the part's front radius (12.821, to the tip), and the
        front face (on the range's edge) is not faced: the range is the part from its front."""
        for op in ("rough", "profile"):
            with self.subTest(op=op):
                cmds = R.make_op(op, "groove", {"start_offset": -1.0, "stock_to_leave": 0}).get_gcode()
                pts, prev = [], None
                for mv, x, z in R.moves(cmds):
                    if mv == "G1" and prev is not None:
                        pts += [(prev[0] + (x - prev[0]) * t, prev[1] + (z - prev[1]) * t) for t in (0, 0.5, 1)]
                    prev = (x, z)
                in_front = [r for r, z in pts if z > -1 + 1e-3]
                self.assertTrue(in_front)
                # the imaginary tip, not the nose: up to a nose radius (0.4) under the
                # part where it goes round the front corner (it went to the axis before)
                self.assertGreater(min(in_front), 12.821 - 0.4)
                self.assertEqual([r for r, z in pts if abs(z + 1) < 1e-3 and r < 12.7], [])

    def test_part_in_front(self):
        """The range starts inside the part: entered radially."""
        o, _ = self._start("rough", {"start_offset": -10.0})
        self.assertTrue(o.radial_entry())
        self.assertEqual(o.lead_in_used, 0.0)

    def test_set(self):
        o, _ = self._start("rough", {"start_offset": -1.0, "pass_lead_in": 1.5})
        self.assertAlmostEqual(o.lead_in_used, 1.5)


class test_GrooveStrategies(unittest.TestCase):

    BLADE = {"groove_width": 3, "corner_radius": 0.3, "cutting_depth": 10, "turning_depth": 1.5}

    def test_side_turn_levels(self):
        """Side-turn: traverses one step down (0.9 x the turning depth) apart, chained."""
        op = R.make_op("groove", "groove", dict(self.BLADE, strategy="side_turn"))
        cmds = op.get_gcode()
        levels = sorted({round(p[0][0], 6) for p in op.rough_paths if p}, reverse=True)
        steps = [a - b for a, b in zip([-op.x_outer()] + levels[:-1], levels)]
        self.assertTrue(op.rough_paths)
        self.assertTrue(all(st <= 0.9 * 1.5 + 1e-6 for st in steps))
        g0_plunge = sum(1 for c in R.make_op("groove", "groove", dict(self.BLADE)).get_gcode() if c.get_movement() == "G0")
        self.assertLess(sum(1 for c in cmds if c.get_movement() == "G0"), g0_plunge / 4)

    def test_side_turn_no_gouge(self):
        self.assertGreater(test_Groove._clearance(test_Groove(), {"strategy": "side_turn", "turning_depth": 1.5}), -1e-4)

    def _narrow(self, width):
        """A 6 mm wide window (Z -2 to -8) on the shoulder part, a blade of the given width."""
        op = R.make_op("groove", "shoulder", {"groove_width": width, "corner_radius": 0.3, "cutting_depth": 10,
                                              "turning_depth": 1.0, "strategy": "side_turn", "start_offset": -2.0,
                                              "end_offset": -22.0, "stock_to_leave": 0})
        op.get_gcode()
        return op

    def test_side_turn_narrow_groove(self):
        """A groove about 1.5 blade widths wide is side-turned, not quietly plunged."""
        op = self._narrow(4.05)
        self.assertTrue(op.rough_paths)
        self.assertEqual(op.plunges, [])
        self.assertEqual(op.warnings, [])

    def test_side_turn_traverses_stay_at_their_level(self):
        """A flat-floored groove: each traverse runs at its level end to end, with no
        climb along the blade's corner radius at its ends (a wiggle grazing the top)."""
        op = self._narrow(4.05)
        for pts in op.rough_paths:
            xs = [x for x, z in pts]
            self.assertAlmostEqual(max(xs), min(xs), places=6)

    def test_side_turn_falls_back_with_a_warning(self):
        """A groove the blade practically fills is plunged, and the op says so."""
        op = self._narrow(5.6)
        self.assertTrue(op.plunges)
        self.assertTrue(any(w.startswith("Side-turn:") and "plunged" in w for w in op.warnings))

    def test_pecking(self):
        """Pecks of peck_depth from the stock's top, backing off peck_retract, to the floor."""
        op = R.make_op("groove", "groove", dict(self.BLADE, peck_depth=2.0))
        cmds = op.get_gcode()
        z0, floor = op.plunges[0]
        xs = [(c.get_movement(), c.get_params()["X"]) for c in cmds if abs(c.get_params().get("Z", 99) - z0) < 1e-9]
        feeds = [x for m, x in xs if m == "G1"]
        top = -op.x_outer()
        self.assertAlmostEqual(feeds[0], top - 2.0)
        self.assertAlmostEqual(feeds[-1], floor)
        self.assertTrue(all(abs((a - b) - 2.0) < 1e-6 for a, b in zip(feeds[:-2], feeds[1:-1])))

    def test_partoff_pecking(self):
        cmds = R.make_op("partoff", "groove", dict(BLADE, end_offset=-60, peck_depth=3.0, peck_full_retract=True)).get_gcode()
        feeds = [c.get_params()["X"] for c in cmds if c.get_movement() == "G1"]
        self.assertGreater(len(feeds), 4)
        self.assertAlmostEqual(feeds[-1], -0.2)


class test_ThreadRunOut(unittest.TestCase):
    """A run-out: each pass tapers out to the crest over the last run_out of
    the thread, instead of stopping dead at full depth."""

    def _op(self, **extra):
        return R.make_op("thread", "m12bar", dict(M12, S=56.196, F=0.0, thread_length=20.0, **extra))

    def _pass_ends(self, op):
        """Per pass (depth, X at depth, start Z, end of the full-depth stretch, end of the run-out)."""
        cmds = op.get_gcode()
        out, i = [], 0
        for d, (x, z0) in zip(op.depths, op.passes):
            while cmds[i].get_movement() != "G33":
                i += 1
            straight = cmds[i].get_params()
            i += 2                                   # the G33 and its display G1
            end = straight
            if op.run_out > 0:
                while cmds[i].get_movement() != "G33":
                    i += 1
                end = cmds[i].get_params()
                i += 2
            out.append((d, x, z0, straight["Z"], end["Z"], end["X"], straight["K"], end["K"]))
        return out

    def test_run_out_keeps_the_pitch_along_z(self):
        """G33's K is the distance along the move per revolution (LinuxCNC): the sloping run-out's K is
        the pitch times its length over its length along Z, so Z still advances one pitch per turn."""
        op = self._op(run_out=1.75)
        for d, x, z0, z_full, z_end, x_end, k_straight, k_out in self._pass_ends(op):
            self.assertAlmostEqual(k_straight, 1.75)
            dz, dx = z_full - z_end, x_end - x
            self.assertAlmostEqual(k_out * dz / math.hypot(dz, dx), 1.75)      # the pitch along Z
            self.assertAlmostEqual(k_out, 1.75 * math.hypot(1.75, op.thread_depth) / 1.75)

    def test_g33_passes(self):
        """In pairs: at depth, then out to the crest at the deepest pass's slope; the deepest pass
        ends at the thread's end, each shallower one that pass moved back along the infeed line."""
        op = self._op(run_out=1.75)
        full, shift = op.thread_depth, math.tan(math.radians(op.infeed_angle))
        rows = self._pass_ends(op)
        self.assertEqual(len(rows), len(op.passes))
        for d, x, z0, z_full, z_end, x_end, *_ in rows:
            self.assertAlmostEqual(z_full, op.z_back + 1.75 + (full - d) * shift)
            self.assertAlmostEqual(x_end, op.thread_radius)
            self.assertAlmostEqual((z_full - z_end) / d, 1.75 / full)        # one slope for every pass
        deepest = max(rows)
        self.assertAlmostEqual(deepest[4], op.z_back)

    def test_passes_nest(self):
        """Every pass is the deepest pass translated along the infeed line (as LinuxCNC's G76 offsets a
        whole pass by zoff): out by the depth it is short and toward the tailstock by that times
        tan(infeed), at its start, the end of its full depth and its run-out alike. Passes that all
        ended at one Z left grooves of their own in the run-out."""
        op = self._op(run_out=1.75)
        full, shift = op.thread_depth, math.tan(math.radians(op.infeed_angle))
        rows = self._pass_ends(op)
        D = max(rows)
        for d, x, z0, z_full, z_end, x_end, *_ in rows:
            back = (full - d) * shift
            self.assertAlmostEqual(x - D[1], full - d)                          # out (external: larger X)
            self.assertAlmostEqual(z0 - D[2], back)
            self.assertAlmostEqual(z_full - D[3], back)
            # the run-out ends where the deepest pass's run-out, moved, reaches this pass's depth
            self.assertAlmostEqual(z_end, D[3] + back - 1.75 * d / full)

    def test_no_run_out(self):
        """Stopping dead: each pass ends at the thread's end moved back along the infeed line, the deepest at it."""
        op = self._op()
        full, shift = op.thread_depth, math.tan(math.radians(op.infeed_angle))
        rows = self._pass_ends(op)
        for d, x, z0, z_full, z_end, x_end, *_ in rows:
            self.assertAlmostEqual(z_full, op.z_back + (full - d) * shift)
        self.assertAlmostEqual(min(r[3] for r in rows), op.z_back)

    def test_g76_taper_words(self):
        params = [c.get_params() for c in self._op(run_out=1.75, canned=True).get_gcode() if c.get_movement() == "G76"][0]
        self.assertAlmostEqual(params["E"], 1.75)
        self.assertEqual(params["L"], 2)
        self.assertNotIn("E", [c.get_params() for c in self._op(canned=True).get_gcode() if c.get_movement() == "G76"][0])

    def test_too_long(self):
        with self.assertRaises(ValueError):
            self._op(run_out=25.0).get_gcode()


class test_EdgeBreak(unittest.TestCase):
    """Break edges: every sharp outside corner chamfered (the shoulder part: the
    front corner at radius 6, Z 0, and the shoulder's outer edge at radius 8,
    Z -20; its inside corner at radius 6, Z -20 and the back end are left)."""

    def _diagonals(self, op):
        out, prev = [], None
        for mv, x, z in R.moves(op.get_gcode()):
            if mv == "G1" and prev is not None and abs(x - prev[0]) > 1e-3 and abs(z - prev[1]) > 1e-3:
                out.append((prev, (x, z)))
            prev = (x, z)
        return out

    def test_corners_found(self):
        op = R.make_op("profile", "shoulder", {"stock_to_leave": 0, "break_edges": 0.5})
        op.get_gcode()
        self.assertEqual(sorted((round(p.X, 3), round(p.Z, 3)) for p in op.edges_broken), [(-8.0, -20.0), (-6.0, 0.0)])

    def test_profile_rough_face(self):
        """45 degree moves (the chamfers) in Profile's finish, Rough's cleanup and Face's
        corner pass; none without breaking."""
        for opname, extra in (("profile", {}), ("rough", {"cleanup_pass": True}), ("face", {})):
            with self.subTest(op=opname):
                base = dict({"stock_to_leave": 0}, **extra)
                plain = [d for d in self._diagonals(R.make_op(opname, "shoulder", base))
                         if abs(abs(d[1][0] - d[0][0]) - abs(d[1][1] - d[0][1])) < 0.02 and abs(d[0][1]) < 0.01]
                broken = [d for d in self._diagonals(R.make_op(opname, "shoulder", dict(base, break_edges=0.5)))
                          if abs(abs(d[1][0] - d[0][0]) - abs(d[1][1] - d[0][1])) < 0.02 and abs(d[0][1]) < 0.01]
                self.assertEqual(plain, [])
                self.assertTrue(broken)


class test_FaceToPlane(unittest.TestCase):
    """Face down to a chosen plane (face_z): the shoulder part's shoulder is at Z -20,
    radius 6 in front of it."""

    def test_shoulder_with_min_diameter(self):
        op = R.make_op("face", "shoulder", {"stock_to_leave": 0, "face_z": -20.0, "min_dia": 12.0})
        feeds = [(x, z) for mv, x, z in R.moves(op.get_gcode()) if mv == "G1"]
        self.assertAlmostEqual(min(z for x, z in feeds), -20.0)
        # the nose's centre on the minimum diameter: the tip a nose radius inside it
        self.assertGreater(min(x for x, z in feeds), 6.0 - 0.4 - 1e-6)

    def test_refused_into_the_part(self):
        for params in ({"face_z": -20.0}, {"face_z": -1.0}):
            with self.subTest(**params):
                with self.assertRaises(ValueError):
                    R.make_op("face", "shoulder", dict({"stock_to_leave": 0}, **params)).get_gcode()


class test_Chamfer(unittest.TestCase):

    def _op(self, corners):
        op = R.make_op("chamfer", "shoulder", {"stock_to_leave": 0, "corners": corners})
        return op, op.get_gcode()

    def test_chamfer_and_fillet(self):
        op, cmds = self._op([(6.0, 0.0, "chamfer", 0.5), (8.0, -20.0, "fillet", 1.0)])
        self.assertEqual(sorted(op.corners_done), [(6.0, 0.0), (8.0, -20.0)])
        self.assertEqual(op.warnings, [])
        arcs = [c for c in cmds if c.get_movement() in ("G2", "G3")]
        self.assertEqual(len(arcs), 1)     # the fillet (with the nose: radius 1.4)
        self.assertAlmostEqual(abs(arcs[0].get_params()["K"]), 1.4, places=3)

    def test_refused(self):
        op, _ = self._op([(6.0, -20.0, "chamfer", 0.5), (7.0, -10.0, "chamfer", 0.5)])
        self.assertEqual(op.corners_done, [])
        self.assertTrue(any("inside corner" in w for w in op.warnings))
        self.assertTrue(any("No sharp corner" in w for w in op.warnings))


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
        with self.assertRaisesRegex(ValueError, r"End the thread at Z -18\.\d+ or above"):
            R.make_op("thread", "shoulder", dict(M12, S=0.0, F=-20.0)).get_gcode()

    def test_feature_in_front_refused(self):
        """A dome of the thread's own radius in front stands above the root: refused, whatever the lead-in."""
        with self.assertRaisesRegex(ValueError, "stands above the thread's root"):
            R.make_op("thread", "dome", {"pitch": 2.5, "thread_depth": 1.5336, "thread_radius": 10.5355,
                                         "tool_width": 6, "S": 65.0, "F": 40.0}).get_gcode()

    def test_behind_a_short_neck_lead_in_shortened(self):
        """A thread behind a 3 mm neck: the lead-in is shortened to fit in the neck (the insert's
        flank, not its full width, ahead of the tip), with a note, and every pass starts in the neck."""
        o = R.make_op("thread", "neck", dict(M12, S=-11.0, F=-25.0))
        cmds = o.get_gcode()
        lead = o.z_drive - o.z_front
        # the head at Z -8 stands 1.0735 above the root: the flank (60 degrees) is clear of it
        # 1.0735 * tan(30) = 0.62 behind it, so the pass starts 3 - 0.62 = 2.38 in front of the thread
        self.assertAlmostEqual(lead, 3.0 - 1.0735 * math.tan(math.radians(30)), delta=0.06)
        self.assertTrue(any("Lead-in shortened" in w for w in o.warnings))
        starts = [c.get_params()["Z"] for c in cmds if c.get_movement() == "G0" and "Z" in c.get_params()]
        self.assertLess(max(z for z in starts if z < -1.0), -8.0 + 1e-6)

    def test_behind_a_short_neck_set_lead_in_too_long(self):
        with self.assertRaisesRegex(ValueError, r"At most 2\.\d+ fits"):
            R.make_op("thread", "neck", dict(M12, S=-11.0, F=-25.0, lead_in=4.0)).get_gcode()

    def test_behind_a_long_neck_lead_in_unchanged(self):
        o = R.make_op("thread", "neck_long", dict(M12, S=-11.0, F=-25.0))
        o.get_gcode()
        self.assertAlmostEqual(o.z_drive - o.z_front, 3 * 1.75)
        self.assertFalse(any("Lead-in" in w for w in o.warnings))

    def test_runs_to_the_stock_end(self):
        """Threading to the end of the stock is allowed: the fixture is not assumed."""
        cmds = self._thread().get_gcode()
        self.assertAlmostEqual(min(c.get_params()["Z"] for c in cmds if c.get_movement() == "G33"), 0.0, places=4)


class test_WaivedChecks(unittest.TestCase):
    """A refusal names its check (liblathe.base.refusal); with the check waived the operation makes the
    path it refused, and says so in its warnings."""

    THREAD_DOME = {"pitch": 2.5, "thread_depth": 1.5336, "thread_radius": 10.5355, "tool_width": 6, "S": 65.0, "F": 40.0}

    def _refused_then_waived(self, opname, part, params, check, moves=("G0", "G1", "G33")):
        from liblathe.base.refusal import Refusal, CHECKS
        self.assertIn(check, CHECKS)
        with self.assertRaises(Refusal) as refused:
            R.make_op(opname, part, dict(params)).get_gcode()
        self.assertEqual(refused.exception.check, check)
        op = R.make_op(opname, part, dict(params, ignored_checks=(check,)))
        cmds = op.get_gcode()
        self.assertTrue(any(c.get_movement() in moves for c in cmds))
        self.assertTrue(any(("check %s waived" % check) in w for w in op.warnings))
        return op, cmds

    def test_thread_front_clearance(self):
        """The dome refuses in front; waived, the next check (a shoulder behind) refuses in turn; both
        waived, the thread is made with three pitches of lead-in."""
        from liblathe.base.refusal import Refusal
        with self.assertRaises(Refusal) as refused:
            R.make_op("thread", "dome", dict(self.THREAD_DOME)).get_gcode()
        self.assertEqual(refused.exception.check, "thread-front-clearance")
        with self.assertRaises(Refusal) as refused:
            R.make_op("thread", "dome", dict(self.THREAD_DOME, ignored_checks=("thread-front-clearance",))).get_gcode()
        self.assertEqual(refused.exception.check, "thread-hits-behind-end")
        op = R.make_op("thread", "dome", dict(self.THREAD_DOME, ignored_checks=("thread-front-clearance", "thread-hits-behind-end")))
        cmds = op.get_gcode()
        self.assertTrue(any(c.get_movement() == "G33" for c in cmds))
        self.assertAlmostEqual(op.z_drive - op.z_front, 3 * 2.5)
        self.assertEqual(sum("waived" in w for w in op.warnings), 2)

    def test_thread_lead_in_too_long(self):
        op, _ = self._refused_then_waived("thread", "neck", dict(M12, S=-11.0, F=-25.0, lead_in=4.0),
                                          "thread-lead-in-too-long")
        self.assertAlmostEqual(op.z_drive - op.z_front, 4.0)          # the lead-in as set

    def test_thread_hits_behind_end(self):
        op, _ = self._refused_then_waived("thread", "shoulder", dict(M12, S=0.0, F=-20.0), "thread-hits-behind-end")
        self.assertAlmostEqual(op.z_back, -20.0)                      # ends where set

    def test_face_cuts_part(self):
        self._refused_then_waived("face", "shoulder", {"stock_to_leave": 0, "face_z": -20.0}, "face-cuts-part",
                                  moves=("G0", "G1"))

    def test_other_checks_still_refuse(self):
        """Waiving one check leaves the others: the shoulder thread still refuses with another one waived."""
        with self.assertRaisesRegex(ValueError, "would hit the part behind"):
            R.make_op("thread", "shoulder", dict(M12, S=0.0, F=-20.0, ignored_checks=("thread-front-clearance",))).get_gcode()

    def test_hard_errors_not_waivable(self):
        """No sensible path: a plain error, whatever is waived."""
        from liblathe.base.refusal import Refusal, CHECKS
        with self.assertRaises(ValueError) as err:
            R.make_op("thread", "dome", dict(self.THREAD_DOME, pitch=0, ignored_checks=tuple(CHECKS))).get_gcode()
        self.assertNotIsInstance(err.exception, Refusal)


if __name__ == '__main__':
    unittest.main()
