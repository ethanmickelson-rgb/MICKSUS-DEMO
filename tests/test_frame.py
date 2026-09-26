"""Phase 6 verification: frame-mesh backdrop loading and transform."""

import os
import unittest

import numpy as np

from suspension_tool.frame import FrameTransform, guess_scale, load_frame_mesh

MESH = os.path.join(os.path.dirname(__file__), "..", "examples",
                   "demo_frame.stl")


class TestFrameTransform(unittest.TestCase):
    def test_matrix_by_hand(self):
        # scale 1000 (metres -> mm), rotate +90 about Z, then offset.
        t = FrameTransform(scale=1000.0, rot_z_deg=90.0,
                           dx=10.0, dy=20.0, dz=30.0)
        m = t.matrix()
        p = m @ np.array([1.0, 0.0, 0.0, 1.0])   # +X point in mesh frame
        # x=1 scales to 1000, rotates +90 deg about Z onto +Y, then offsets
        np.testing.assert_allclose(p[:3], [10.0, 1020.0, 30.0], atol=1e-9)

    def test_dict_round_trip(self):
        t = FrameTransform(scale=25.4, rot_z_deg=270.0, dx=-5.0, dy=1.0, dz=2.0)
        t2 = FrameTransform.from_dict(t.to_dict())
        self.assertEqual(t, t2)
        # unknown keys from future versions are ignored
        d = t.to_dict()
        d["future"] = 1
        self.assertEqual(FrameTransform.from_dict(d), t)

    def test_guess_scale(self):
        self.assertEqual(guess_scale([0.9, 2.1, 1.3]), 1000.0)   # metres
        self.assertEqual(guess_scale([35.0, 83.0, 50.0]), 25.4)  # inches
        self.assertEqual(guess_scale([900.0, 2100.0, 1300.0]), 1.0)  # mm


class TestFrameLoad(unittest.TestCase):
    def test_loads_a_chassis_mesh_and_detects_metres(self):
        """Loads the shipped demo frame and infers its units.

        This used to assert >1000 points, which was really a property of
        one team's chassis export rather than of the loader. What the
        loader must actually do is return a non-empty surface whose
        extents let `guess_scale` recognise a mesh drawn in metres.
        """
        mesh = load_frame_mesh(MESH)
        self.assertGreater(mesh.n_points, 0)
        self.assertGreater(mesh.n_cells, 0)
        span = np.ptp(mesh.points, axis=0)
        self.assertEqual(guess_scale(span), 1000.0,
                         f"metres not detected from span {span}")
        # a chassis, not a stray part: metres-scale and roughly car-sized
        self.assertGreater(max(span) * 1000.0, 500.0)


class TestProjectFramePassthrough(unittest.TestCase):
    def test_frame_survives_project_round_trip(self):
        from suspension_tool.project import (ProjectState, dict_to_project,
                                             project_to_dict)
        state = ProjectState(front=None, rear=None, unit_key="mm",
                             frame={"path": "C:/frames/mk4.3mf",
                                    "scale": 1000.0, "rot_z_deg": 90.0,
                                    "dx": 1.0, "dy": 2.0, "dz": 3.0})
        restored = dict_to_project(project_to_dict(state))
        self.assertEqual(restored.frame["path"], "C:/frames/mk4.3mf")
        self.assertEqual(restored.frame["rot_z_deg"], 90.0)


if __name__ == "__main__":
    unittest.main()
