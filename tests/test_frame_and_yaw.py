"""Three UI/workflow changes (v1.40).

  1. The chassis backdrop mesh now rides INSIDE the project file. Storing
     a path breaks the moment the design moves to another machine, which
     is the one thing a save file exists to survive.
  2. The backdrop is drawn at 60% opacity instead of 35%. VTK draws
     transparent surfaces without depth sorting, so several imported STL
     parts stacked into a muddle; 60% kills most of that while leaving
     the linkages readable behind it.
  3. Sketch-plane yaw is a TWEAK now, not only a seed input. Getting the
     sign wrong (typing +20 for tubes that run -20) used to mean
     re-seeding; it is now a spin box that re-squares the corner.
"""

import hashlib
import os
import tempfile
import unittest

import numpy as np

from suspension_tool.frame import (embed_mesh_file, embedded_size_mb,
                                   extract_mesh_file)
from suspension_tool.geometry import example_baja_front, sketch_planarity
from suspension_tool.seed import SetupVariables
from suspension_tool.tweaks import (has_sketch_plane, set_sketch_yaw,
                                    sketch_yaw_deg)

_MESH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "examples", "demo_frame.stl")


class TestMeshEmbedding(unittest.TestCase):

    def test_round_trip_is_byte_identical(self):
        """The SOURCE bytes are stored, not a re-export -- round-tripping
        through a writer of ours would quietly change the geometry."""
        blob = embed_mesh_file(_MESH)
        with tempfile.TemporaryDirectory() as d:
            out = extract_mesh_file(blob, d)
            with open(out, "rb") as a, open(_MESH, "rb") as b:
                self.assertEqual(hashlib.sha256(a.read()).digest(),
                                 hashlib.sha256(b.read()).digest())

    def test_the_original_filename_survives(self):
        blob = embed_mesh_file(_MESH)
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(os.path.basename(extract_mesh_file(blob, d)),
                             os.path.basename(_MESH))

    def test_the_embedded_mesh_still_loads(self):
        from suspension_tool.frame import load_frame_mesh
        blob = embed_mesh_file(_MESH)
        with tempfile.TemporaryDirectory() as d:
            mesh = load_frame_mesh(extract_mesh_file(blob, d))
        self.assertGreater(mesh.n_points, 100)

    def test_size_is_reported_so_the_cost_is_visible(self):
        """The reported size must track the blob it describes.

        This asserted > 1.0 MB, which only held because the fixture was a
        2.4 MB chassis export -- an absolute threshold standing in for a
        property of one file. What matters is that the number is real:
        it scales with the payload and is zero for nothing.
        """
        blob = embed_mesh_file(_MESH)
        mb = embedded_size_mb(blob)
        self.assertGreater(mb, 0.0)
        self.assertEqual(embedded_size_mb(None), 0.0)
        raw_mb = os.path.getsize(_MESH) / (1024.0 * 1024.0)
        self.assertLessEqual(mb, raw_mb * 1.05,
                             "stored size cannot exceed the source")
        self.assertGreater(mb, raw_mb * 0.01,
                           "a plausible fraction of the source, not a stub")

    def test_an_ascii_mesh_compresses_hard(self):
        """3MF is a zip already so it barely shrinks, but STL is text and
        should compress several-fold -- worth knowing before someone
        imports a 50 MB STL."""
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "box.stl")
            with open(p, "w") as fh:
                fh.write("solid box\n")
                for _ in range(2000):
                    fh.write("  facet normal 0 0 1\n    outer loop\n"
                             "      vertex 0 0 0\n      vertex 1 0 0\n"
                             "      vertex 0 1 0\n    endloop\n  endfacet\n")
                fh.write("endsolid box\n")
            blob = embed_mesh_file(p)
            self.assertLess(embedded_size_mb(blob) * 1024 * 1024,
                            blob["bytes"] / 4.0)


class TestSketchYawTweak(unittest.TestCase):

    def _sv(self):
        return SetupVariables(
            track_width=1500.0, wheelbase=1550.0, ride_height=350.0,
            tire_radius=292.0, tire_width=178.0, shock_min_length=373.0,
            shock_max_length=576.0, motion_ratio_goal=0.54)

    def test_a_double_wishbone_lands_on_the_requested_yaw(self):
        hp = example_baja_front()
        for want in (20.1, -20.1, 0.0, 7.5):
            got = sketch_yaw_deg(set_sketch_yaw(hp, want))
            self.assertAlmostEqual(got, want, places=6)

    def test_the_sign_flip_is_the_reported_use_case(self):
        """Typed +20.1 for tubes that run -20.1: one tweak, not a reseed."""
        hp = set_sketch_yaw(example_baja_front(), 20.1)
        fixed = set_sketch_yaw(hp, -20.1)
        self.assertAlmostEqual(sketch_yaw_deg(fixed), -20.1, places=6)

    def test_squaring_leaves_the_axes_parallel(self):
        out = set_sketch_yaw(example_baja_front(), -20.1)
        self.assertLess(sketch_planarity(out)["axis_misalign_deg"], 1e-6)

    def test_kickup_elevation_is_preserved(self):
        """This is why yaw is applied BEFORE the kickup row: squaring to a
        new plane keeps the elevation, so the kickup spin still describes
        the same corner afterwards."""
        hp = example_baja_front()
        before = sketch_planarity(hp)["sketch_kickup_deg"]
        after = sketch_planarity(set_sketch_yaw(hp, -20.1))["sketch_kickup_deg"]
        self.assertAlmostEqual(before, after, places=6)

    def test_it_is_reversible(self):
        hp = example_baja_front()
        start = sketch_yaw_deg(hp)
        there_and_back = set_sketch_yaw(set_sketch_yaw(hp, -25.0), start)
        self.assertAlmostEqual(sketch_yaw_deg(there_and_back), start, places=6)

    def test_every_carrier_type_re_squares(self):
        from suspension_tool.chub_front import seed_chub_front
        from suspension_tool.harm_rear import seed_harm_rear
        from suspension_tool.loaded_halfshaft import seed_loaded_halfshaft
        from suspension_tool.tweaks import (chub_params, harm_params,
                                            loaded_hs_params, set_chub,
                                            set_harm, set_loaded_hs)
        sv = self._sv()
        cases = [
            (seed_chub_front(sv), chub_params, set_chub,
             ("d", "h", "ang", "caster", "kpi")),
            (seed_loaded_halfshaft(sv), loaded_hs_params, set_loaded_hs,
             ("d", "h", "ang", "plan_deg", "elev_deg")),
            (seed_harm_rear(sv), harm_params, set_harm,
             ("d", "h", "ang", "plan_deg", "elev_deg")),
        ]
        for hp, params, setter, req in cases:
            p = params(hp)
            self.assertIn("sketch_yaw", p, type(hp).__name__)
            out = setter(hp, *[p[k] for k in req], sketch_yaw=-15.0)
            self.assertAlmostEqual(sketch_yaw_deg(out), -15.0, places=6,
                                   msg=type(hp).__name__)

    def test_types_without_a_sketch_are_left_alone(self):
        """A trailing arm has no 2D design sketch, so the row must not
        appear and the setter must be a no-op rather than a crash."""
        from suspension_tool.trailing_arm import seed_trailing_arm
        from suspension_tool.tweaks import trailing_arm_params
        hp = seed_trailing_arm(self._sv())
        self.assertFalse(has_sketch_plane(hp))
        self.assertNotIn("sketch_yaw", trailing_arm_params(hp))
        self.assertIs(set_sketch_yaw(hp, -20.0), hp)


class TestFrameOpacity(unittest.TestCase):

    def test_the_default_moved_off_the_see_through_value(self):
        from suspension_tool.gui.viewport import FRAME_OPACITY
        self.assertAlmostEqual(FRAME_OPACITY, 0.60, places=6)
        self.assertGreater(FRAME_OPACITY, 0.35, "was the old value")


class TestPanelsCarryTheRows(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def test_the_yaw_row_is_on_the_four_sketch_types_only(self):
        from suspension_tool.gui.panels import TweaksPanel
        rows = {"dw": TweaksPanel.DW_ROWS, "ch": TweaksPanel.CH_ROWS,
                "lh": TweaksPanel.LH_ROWS, "ha": TweaksPanel.HA_ROWS}
        for key, rs in rows.items():
            self.assertIn("sketch_yaw", [r[0] for r in rs], key)
        for key, rs in (("ta", TweaksPanel.TA_ROWS),
                        ("ml", TweaksPanel.ML_ROWS)):
            self.assertNotIn("sketch_yaw", [r[0] for r in rs], key)

    def test_the_frame_panel_has_an_opacity_control(self):
        from suspension_tool.gui.panels import FramePanel
        from suspension_tool.units import UNITS
        p = FramePanel(UNITS["mm"])
        self.assertAlmostEqual(p.opacity_spin.value(), 60.0, places=6)
        self.assertEqual(p.opacity_spin.minimum(), 0.0)
        self.assertEqual(p.opacity_spin.maximum(), 100.0)


if __name__ == "__main__":
    unittest.main()
