import os
import sys
import unittest

thisFolder = os.path.dirname(os.path.abspath(__file__))
parentFolder = os.path.dirname(thisFolder)
sys.path.append(parentFolder)
from liblathe.base.tool import Tool
from liblathe.base.tool import ToolOri


class test_tool(unittest.TestCase):
    """Test for tool.py"""
    def setUp(self):
        self.tool = Tool()
        self.tool.set_tool_from_string('DCMT070204R')

    def test_create_tool(self):
        shape = "D"
        length = self.tool.edge_length
        nose_radius = self.tool.nose_radius
        direction = self.tool.direction
        orientation = self.tool.orientation

        self.assertEqual(shape, "D")
        self.assertEqual(length, 6.35)
        self.assertEqual(nose_radius, 0.4)
        self.assertEqual(direction, "R")
        self.assertEqual(orientation, ToolOri.X)

        with self.assertRaises(ValueError):
            Tool('xyz')

    def test_get_tool_cutting_angle(self):
        cuttingAngle = self.tool.get_tool_cutting_angle()
        self.assertEqual(cuttingAngle, 209.5)

    def test_getShapeAngle(self):
        shapeAngle = self.tool.get_tip_angle_from_shape("D")
        self.assertEqual(shapeAngle, 55)

    def test_get_edge_length(self):
        edgeLength = self.tool.get_edge_length("D", "07")
        self.assertEqual(edgeLength, 6.35)

        with self.assertRaises(ValueError):
            self.tool.get_edge_length("A", "A")

        with self.assertRaises(ValueError):
            self.tool.get_edge_length("D", "A")

    def test_get_nose_radius(self):
        noseRadius = self.tool.get_nose_radius("04")
        self.assertEqual(noseRadius, 0.4)

        with self.assertRaises(ValueError):
            self.tool.get_nose_radius("A")

    def test_get_cutting_direction(self):
        cuttingDirection = self.tool.get_cutting_direction()
        self.assertEqual(cuttingDirection, "R")

    def test_get_rotation(self):
        rotation = self.tool.get_rotation()
        self.assertEqual(rotation, 0)

    def test_get_max_doc(self):
        max_doc = self.tool.get_max_doc()
        self.assertEqual(max_doc, 1.5875)

        with self.assertRaises(ValueError):
            tool = Tool()
            max_doc = tool.get_max_doc()

    def test_set_tip_angle(self):
        self.tool.set_tip_angle(22.5)
        self.assertEqual(self.tool.tip_angle, 22.5)

        with self.assertRaises(ValueError):
            self.tool.set_tip_angle("A")

        with self.assertRaises(ValueError):
            self.tool.set_tip_angle(-1)

    def test_set_edge_length(self):
        self.tool.set_edge_length(5)
        self.assertEqual(self.tool.edge_length, 5)

        with self.assertRaises(ValueError):
            self.tool.set_edge_length("A")

        with self.assertRaises(ValueError):
            self.tool.set_edge_length(-1)

    def test_set_nose_radius(self):
        self.tool.set_nose_radius(0.5)
        self.assertEqual(self.tool.nose_radius, 0.5)

        with self.assertRaises(ValueError):
            self.tool.set_nose_radius("A")

        with self.assertRaises(ValueError):
            self.tool.set_nose_radius(-1)

    def test_set_direction(self):
        self.tool.set_direction("N")
        self.assertEqual(self.tool.direction, "N")

        with self.assertRaises(ValueError):
            self.tool.set_direction("A")

    def test_set_rotation(self):
        self.tool.set_rotation(45)
        self.assertEqual(self.tool.tool_rotation, 45)

        self.tool.set_rotation(22.5)
        self.assertEqual(self.tool.tool_rotation, 22.5)

        with self.assertRaises(ValueError):
            self.tool.set_rotation(361)

        with self.assertRaises(ValueError):
            self.tool.set_rotation(-1)

        with self.assertRaises(ValueError):
            self.tool.set_rotation("A")

    def test_set_orientation(self):
        self.tool.set_orientation(ToolOri.X)
        self.assertEqual(self.tool.orientation, ToolOri.X)

        with self.assertRaises(ValueError):
            self.tool.set_orientation("X")


    def test_tip_angle_boundaries(self):
        for angle in (0.5, 90, 180):
            self.tool.set_tip_angle(angle)
            self.assertEqual(self.tool.tip_angle, angle)
        for angle in (0, 180.01, float("nan"), None, True):
            with self.assertRaises(ValueError) as ctx:
                self.tool.set_tip_angle(angle)
            self.assertIn(repr(angle), str(ctx.exception))

    def test_rotation_boundaries(self):
        for rotation in (0, 0.0, 359.9):
            self.tool.set_rotation(rotation)
            self.assertEqual(self.tool.tool_rotation, rotation)
        for rotation in (360, -0.1):
            with self.assertRaises(ValueError) as ctx:
                self.tool.set_rotation(rotation)
            self.assertIn(repr(rotation), str(ctx.exception))

    def test_nose_radius_boundaries(self):
        self.tool.set_nose_radius(0)
        self.assertEqual(self.tool.nose_radius, 0)
        with self.assertRaises(ValueError) as ctx:
            self.tool.set_nose_radius(-0.1)
        self.assertIn("-0.1", str(ctx.exception))

    def test_square_and_sharp_inserts_from_string(self):
        tool = Tool('SNMG120400R')
        tool.set_tip_angle(tool.tip_angle)
        tool.set_nose_radius(tool.nose_radius)
        tool.set_rotation(0)
        self.assertEqual((tool.tip_angle, tool.nose_radius, tool.tool_rotation), (90, 0, 0))

    def test_errors_name_the_value(self):
        for setter, value in ((self.tool.set_edge_length, -2), (self.tool.set_direction, "Q"),
                              (self.tool.set_orientation, "X")):
            with self.assertRaises(ValueError) as ctx:
                setter(value)
            self.assertIn(repr(value), str(ctx.exception))


if __name__ == '__main__':
    unittest.main()
