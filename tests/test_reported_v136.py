"""Three issues reported against v1.35, from a 1/10 RC design (v1.36).

  1. The steering rack limit did not survive a save/reload. It lived only
     on the window and was never written to the project file -- the one UI
     setting a saved design did not carry.
  2. "Bump steer does not change when I move the toe link" with the link
     mounted on the LOWER ARM. Not a bug: the inner ball joint is bolted to
     the same arm as the lower ball joint, so it RIDES that arm and its
     distance to the LBJ is fixed through travel. Moving it barely moves
     bump steer. The tool simply never said so.
  3. "The roll axis is really sloped, the front roll centre must be wrong."
     The front roll centre is correct (verified against an independent
     construction). The roll axis is only ~3 deg. What was wrong was the
     hand-entered CG: full-size values left on a 1/10 car, putting the CG
     592 mm BEHIND the rear axle of a 260 mm-wheelbase car.
"""

import dataclasses
import os
import unittest
from unittest import mock

import numpy as np

from suspension_tool import metrics
from suspension_tool.project import load_project
from suspension_tool.solver import DoubleWishboneSolver
from suspension_tool.suspension_types import solver_for

_E71 = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "examples", "rc_buggy_1_10.MICK")


class TestArmMountedToeLink(unittest.TestCase):
    """The physics, so the guidance the UI now gives is grounded."""

    def setUp(self):
        self.hp = load_project(_E71).rear.hardpoints
        self.assertTrue(self.hp.tierod_on_lca)

    def _bump_steer(self, hp):
        s = DoubleWishboneSolver(hp)
        return (metrics.toe_deg(s.solve(4.0))
                - metrics.toe_deg(s.solve(-4.0))) / 8.0

    def test_the_inner_is_rigid_to_the_arm(self):
        """This is WHY the knob does nothing: the inner ball joint and the
        lower ball joint are both bolted to the lower arm, so the distance
        between them cannot change through travel."""
        s = DoubleWishboneSolver(self.hp)
        d = [float(np.linalg.norm(st.tierod_inner_cur - st.lbj))
             for st in (s.solve(-8.0), s.solve(0.0), s.solve(8.0))]
        self.assertAlmostEqual(d[0], d[1], places=9)
        self.assertAlmostEqual(d[1], d[2], places=9)

    def test_moving_the_inner_barely_moves_bump_steer(self):
        base = self._bump_steer(self.hp)
        worst = 0.0
        for d in ([0, 0, 30], [0, 0, -30], [0, 40, 0], [40, 0, 0]):
            hp2 = dataclasses.replace(
                self.hp, tierod_inner=self.hp.tierod_inner + np.array(d, float))
            worst = max(worst, abs(self._bump_steer(hp2) - base))
        self.assertLess(worst, 0.05,
                        "inner mount should be nearly inert when arm-mounted")

    def test_the_outer_end_is_the_real_knob(self):
        """~10x the authority of the inner, and it crosses zero -- which is
        what makes it useful for actually dialling bump steer out."""
        base = self._bump_steer(self.hp)
        vals = []
        for d in ([0, 0, 10], [0, 0, -10]):
            hp2 = dataclasses.replace(
                self.hp, tierod_outer=self.hp.tierod_outer + np.array(d, float))
            vals.append(self._bump_steer(hp2))
        self.assertGreater(max(vals) - min(vals), 0.4)
        self.assertLess(min(vals), base)
        self.assertGreater(max(vals), base)

    def test_a_chassis_mounted_inner_is_hugely_sensitive(self):
        """The contrast that makes the arm-mounted case look broken."""
        free = dataclasses.replace(self.hp, tierod_on_lca=False)
        vals = []
        for d in ([0, 0, 20], [0, 0, -20]):
            hp2 = dataclasses.replace(
                free, tierod_inner=free.tierod_inner + np.array(d, float))
            vals.append(self._bump_steer(hp2))
        self.assertGreater(max(vals) - min(vals), 1.0)


class TestFrontRollCentreIsCorrect(unittest.TestCase):
    """The reported suspicion, checked rather than assumed."""

    def test_rc_matches_an_independent_construction(self):
        st = load_project(_E71)
        hp = st.front.hardpoints
        s = solver_for(hp)(hp)
        state = s.solve(0.0)
        ic = metrics.front_view_ic(s, state)
        cp = state.contact_patch
        # roll centre = where the contact-patch -> IC line crosses the
        # centreline, measured above this axle's own ground plane
        t = (0.0 - cp[1]) / (ic[0] - cp[1])
        expected = cp[2] + t * (ic[1] - cp[2]) - cp[2]
        got = metrics.corner_metrics(s, state)["roll_center_height_mm"]
        self.assertAlmostEqual(got, expected, places=6)

    def test_the_roll_axis_is_not_steeply_sloped(self):
        st = load_project(_E71)
        rc = {}
        for ax in ("front", "rear"):
            hp = getattr(st, ax).hardpoints
            s = solver_for(hp)(hp)
            rc[ax] = metrics.corner_metrics(s, s.solve(0.0))[
                "roll_center_height_mm"]
        wb = float((st.vehicle or {}).get("wheelbase", 260.0))
        ang = np.degrees(np.arctan2(rc["rear"] - rc["front"], wb))
        self.assertLess(abs(ang), 8.0, f"roll axis at {ang:.1f} deg")


class TestCgPlausibility(unittest.TestCase):
    """The real cause of the 'sloped roll axis' report: a full-size CG left
    on a 1/10 car. Judged against the car's own geometry so one check
    serves both scales."""

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _panel(self, cg_height, cg_behind_front):
        from suspension_tool.gui.panels import VehiclePanel
        from suspension_tool.units import UNITS
        from suspension_tool.vehicle import VehicleParams
        p = VehiclePanel(UNITS["mm"])
        p.load_params(VehicleParams(wheelbase=260.0, cg_height=cg_height,
                                    cg_behind_front=cg_behind_front))
        return p

    def test_the_reported_file_is_flagged(self):
        p = self._panel(558.8, 852.17)
        msgs = p.check_cg_plausible(wheelbase_mm=260.0, track_mm=220.4,
                                    tire_radius_mm=37.5)
        self.assertGreaterEqual(len(msgs), 2)
        self.assertTrue(any("BEHIND the rear axle" in m for m in msgs))
        self.assertTrue(any("tire radius" in m for m in msgs))

    def test_a_sane_rc_car_is_not_flagged(self):
        p = self._panel(70.0, 130.0)
        self.assertEqual(
            p.check_cg_plausible(wheelbase_mm=260.0, track_mm=220.4,
                                 tire_radius_mm=37.5), [])

    def test_a_sane_baja_car_is_not_flagged(self):
        """The same check, at 6x the scale, must stay quiet."""
        p = self._panel(560.0, 780.0)
        self.assertEqual(
            p.check_cg_plausible(wheelbase_mm=1550.0, track_mm=1600.0,
                                 tire_radius_mm=292.0), [])

    def test_a_cg_ahead_of_the_front_axle_is_flagged(self):
        p = self._panel(70.0, -40.0)
        msgs = p.check_cg_plausible(wheelbase_mm=260.0, track_mm=220.4,
                                    tire_radius_mm=37.5)
        self.assertTrue(any("AHEAD of the front axle" in m for m in msgs))


class TestSteerLimitPersists(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def test_round_trip(self):
        import tempfile
        from PySide6.QtWidgets import QWidget

        class _Stub(QWidget):
            def __init__(self, *a, **k):
                super().__init__()
                self.scene = mock.MagicMock()

            def __getattr__(self, n):
                return mock.MagicMock()

        import suspension_tool.gui.main_window as mw
        with mock.patch.object(mw, "CornerViewport", _Stub):
            win = mw.MainWindow()
            self.app.processEvents()
            win.setup_form._emit()
            self.app.processEvents()
            win.steer_limit = 12.5
            win._apply_steer_unit()
            with tempfile.TemporaryDirectory() as d:
                f = os.path.join(d, "rack.MICK")
                win._write(f, quiet=True)
                win.steer_limit = 38.0          # clobber before reloading
                win._load_project_file(f)
                self.app.processEvents()
            self.assertAlmostEqual(win.steer_limit, 12.5, places=9)

    def test_an_older_file_without_the_key_keeps_the_default(self):
        st = load_project(_E71)
        self.assertIsNone(st.steer_limit_mm)


if __name__ == "__main__":
    unittest.main()
