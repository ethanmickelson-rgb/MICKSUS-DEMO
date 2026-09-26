"""Display/export axis conventions (chassis CAD frame vs tool frame)."""

import unittest

import numpy as np

from suspension_tool.axes import CONVENTIONS
from suspension_tool.geometry import example_baja_front
from suspension_tool.onshape_sync import build_variables
from suspension_tool.project import hardpoints_csv


class TestConventions(unittest.TestCase):
    def test_all_orthonormal_right_handed_round_trip(self):
        p = np.array([123.4, -56.7, 89.0])
        for conv in CONVENTIONS.values():
            m = conv.mat
            np.testing.assert_allclose(m @ m.T, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(m), 1.0, places=12)
            np.testing.assert_allclose(conv.from_display(conv.to_display(p)),
                                       p, atol=1e-12)

    def test_chassis_plus_y_mapping(self):
        c = CONVENTIONS["chassis+y"]
        # forward (tool +x) -> chassis +y ; left (tool +y) -> chassis -x
        np.testing.assert_allclose(c.to_display([1, 0, 0]), [0, 1, 0])
        np.testing.assert_allclose(c.to_display([0, 1, 0]), [-1, 0, 0])
        np.testing.assert_allclose(c.to_display([0, 0, 1]), [0, 0, 1])

    def test_chassis_minus_y_mapping(self):
        c = CONVENTIONS["chassis-y"]
        # forward -> chassis -y ; left -> chassis +x
        np.testing.assert_allclose(c.to_display([1, 0, 0]), [0, -1, 0])
        np.testing.assert_allclose(c.to_display([0, 1, 0]), [1, 0, 0])
        np.testing.assert_allclose(c.to_display([0, 0, 1]), [0, 0, 1])

    def test_csv_remaps_points(self):
        hp = example_baja_front()
        conv = CONVENTIONS["chassis+y"]
        text = hardpoints_csv(hp, conv=conv)
        row = next(l for l in text.splitlines()
                   if l.startswith("wheel_center,"))
        _, x, y, z = row.split(",")
        p = conv.to_display(hp.wheel_center)
        self.assertAlmostEqual(float(x), p[0], places=3)
        self.assertAlmostEqual(float(y), p[1], places=3)
        self.assertAlmostEqual(float(z), p[2], places=3)
        self.assertIn("Y fwd", text)   # header documents the frame

    def test_onshape_variables_remap(self):
        hp = example_baja_front()
        conv = CONVENTIONS["chassis+y"]
        wb = 1549.4
        flat = build_variables({"front": hp, "rear": hp}, wb, conv=conv)
        by_name = {v["name"]: float(v["expression"].split()[0]) for v in flat}
        # front wheel centre: chassis x = -tool y, chassis y = tool x
        self.assertAlmostEqual(by_name["front_wheel_center_x"],
                               -hp.wheel_center[1], places=3)
        self.assertAlmostEqual(by_name["front_wheel_center_y"],
                               hp.wheel_center[0], places=3)
        # rear shifts rearward BEFORE remapping: chassis y = tool x - wb
        self.assertAlmostEqual(by_name["rear_wheel_center_y"],
                               hp.wheel_center[0] - wb, places=3)
        self.assertAlmostEqual(by_name["rear_wheel_center_z"],
                               hp.wheel_center[2], places=3)

    def test_default_tool_frame_unchanged(self):
        hp = example_baja_front()
        text = hardpoints_csv(hp)     # no conv: legacy tool frame
        row = next(l for l in text.splitlines()
                   if l.startswith("wheel_center,"))
        _, x, y, z = row.split(",")
        self.assertAlmostEqual(float(x), hp.wheel_center[0], places=3)
        self.assertAlmostEqual(float(y), hp.wheel_center[1], places=3)


if __name__ == "__main__":
    unittest.main()
