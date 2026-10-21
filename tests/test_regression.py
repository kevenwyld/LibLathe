"""
Regression tests: every operation on a set of test parts, compared with the
G-code it made when last checked (tests/golden). A change to an operation
that changes a toolpath fails here until it is looked at; if the new output
is intended, run with LIBLATHE_UPDATE_GOLDEN=1 to rewrite the golden files,
and commit them with the change.
"""
import os
import unittest

import regression_helpers as R

BLADE = {"groove_width": 3, "corner_radius": 0.3, "cutting_depth": 30}
M12 = {"pitch": 1.75, "thread_depth": 1.0735, "thread_radius": 6.0, "tool_width": 6}

CASES = {
    # the turning ops on each part
    **{f"{op}_{part}": (op, part, {}) for op in ("rough", "profile", "face") for part in ("ball", "groove", "bar12")},
    # stock to leave: unequal X / Z (rectangle dilation), and negative (under size)
    "profile_bar12_stock_x0.2_z0.1": ("profile", "bar12", {"stock_to_leave_x": 0.2, "stock_to_leave_z": 0.1}),
    "profile_bar12_stock_neg_x": ("profile", "bar12", {"stock_to_leave_x": -0.075, "stock_to_leave_z": 0}),
    "rough_bar12_stock_neg_xz": ("rough", "bar12", {"stock_to_leave": 0, "stock_to_leave_x": -0.075, "stock_to_leave_z": -0.05}),
    # facing and parting a tube (min diameter = its bore)
    "face_groove_min_dia_10": ("face", "groove", {"min_dia": 10}),
    "partoff_groove": ("partoff", "groove", dict(BLADE, end_offset=-60)),
    "partoff_groove_min_dia_10": ("partoff", "groove", dict(BLADE, end_offset=-60, min_dia=10)),
    # grooving
    "groove_groove": ("groove", "groove", dict(BLADE, cutting_depth=10)),
    "groove_groove_stock_x0.2_z0.1": ("groove", "groove", dict(BLADE, cutting_depth=10, stock_to_leave_x=0.2, stock_to_leave_z=0.1)),
    "groove_groove_stock_neg": ("groove", "groove", dict(BLADE, cutting_depth=10, stock_to_leave_x=-0.05, stock_to_leave_z=-0.05)),
    # threading: explicit G33 passes and a G76 cycle
    "thread_m12bar_g33": ("thread", "m12bar", dict(M12, S=56.196, F=0.0)),
    "thread_m12bar_g76": ("thread", "m12bar", dict(M12, S=56.196, F=0.0, canned=True)),
}


class test_Regression(unittest.TestCase):
    """Each case's G-code against its golden file."""

    def test_cases(self):
        update = os.environ.get("LIBLATHE_UPDATE_GOLDEN")
        for name, (opname, part, params) in sorted(CASES.items()):
            with self.subTest(case=name):
                text = R.gcode_text(R.make_op(opname, part, params).get_gcode())
                path = os.path.join(R.HERE, "golden", name + ".gcode")
                if update or not os.path.exists(path):
                    with open(path, "w") as fp:
                        fp.write(text)
                    continue
                with open(path) as fp:
                    expected = fp.read()
                self.assertEqual(text, expected, "%s: output changed (LIBLATHE_UPDATE_GOLDEN=1 to accept)" % name)


if __name__ == '__main__':
    unittest.main()
