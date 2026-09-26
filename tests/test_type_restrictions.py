"""Suspension-type restrictions, seed dispatch, and the corner-move tweaks
(v1.32).

Three things pinned here:

  * "Generate seed hardpoints" must honour the TYPE DROPDOWN. It used to
    always run the double-wishbone seeder, so picking C-hub and pressing
    Generate silently threw the C-hub away.
  * The rear-only types (H-arm, loaded halfshaft) must never reach the
    FRONT axle. Both hold toe rigidly by design and have no steering
    input, so a car built with one on the front physically cannot steer --
    and the solver would happily pose it and plot a flat toe curve as if
    that were fine.
  * corner_x / corner_z move a whole corner without reseeding.

The MainWindow tests stub the VTK viewport: the headless sandbox aborts on
its first GL paint, and none of this is 3D -- it is dropdown wiring and
seed dispatch.
"""

import os
import unittest
from unittest import mock

import numpy as np

from suspension_tool import metrics, tweaks
from suspension_tool.chub_front import CHubFrontSolver, seed_chub_front
from suspension_tool.harm_rear import HArmRearSolver, seed_harm_rear
from suspension_tool.loaded_halfshaft import (LoadedHalfshaftSolver,
                                              seed_loaded_halfshaft)
from suspension_tool.multilink import MultilinkSolver, from_double_wishbone
from suspension_tool.seed import IN, SetupVariables, generate_seed
from suspension_tool.solver import DoubleWishboneSolver
from suspension_tool.suspension_types import (REAR_ONLY_TYPES,
                                              SUSPENSION_TYPES,
                                              is_allowed_on, steers,
                                              types_for_axle)
from suspension_tool.trailing_arm import TrailingArmSolver, seed_trailing_arm


def _sv(**kw) -> SetupVariables:
    base = dict(track_width=63 * IN, wheelbase=61 * IN, ride_height=14 * IN,
                tire_radius=11.5 * IN, tire_width=7 * IN,
                shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
                motion_ratio_goal=1.0 / 1.85,
                shock_length_at_ride=20.38 * IN,
                static_camber_deg=-0.87, static_toe_deg=0.2)
    base.update(kw)
    return SetupVariables(**base)


class TestAxleRestrictionRegistry(unittest.TestCase):

    def test_rear_only_types_are_the_non_steering_ones(self):
        self.assertEqual(
            REAR_ONLY_TYPES,
            {"Loaded halfshaft (rear)", "H-arm (rear)", "Trailing arm"})
        for name in REAR_ONLY_TYPES:
            self.assertFalse(steers(name))
            self.assertFalse(is_allowed_on(name, "front"))
            self.assertTrue(is_allowed_on(name, "rear"))

    def test_front_list_is_the_rear_list_minus_rear_only(self):
        front, rear = types_for_axle("front"), types_for_axle("rear")
        self.assertEqual(set(rear), set(SUSPENSION_TYPES))
        self.assertEqual(set(front), set(SUSPENSION_TYPES) - REAR_ONLY_TYPES)
        # a steered axle must still have real choice, not just one option
        self.assertGreaterEqual(len(front), 3)

    def test_double_wishbone_is_allowed_everywhere(self):
        for axle in ("front", "rear"):
            self.assertTrue(is_allowed_on("Double wishbone", axle))


class TestRearOnlyMeansNoSteering(unittest.TestCase):
    """The restriction is a claim about PHYSICS, so test the physics: every
    rear-only type must be unable to steer, and every front-legal type must
    be able to. Keeps the list honest if a type is ever added."""

    def _toe_response(self, hp, solver_cls) -> float:
        s = solver_cls(hp)
        return abs(metrics.toe_deg(s.solve(0.0, 20.0))
                   - metrics.toe_deg(s.solve(0.0, 0.0)))

    def test_rear_only_types_do_not_respond_to_the_rack(self):
        sv = _sv()
        for hp, solver_cls in ((seed_trailing_arm(sv), TrailingArmSolver),
                               (seed_harm_rear(sv), HArmRearSolver),
                               (seed_loaded_halfshaft(sv),
                                LoadedHalfshaftSolver)):
            with self.subTest(suspension=type(hp).__name__):
                self.assertLess(self._toe_response(hp, solver_cls), 0.01)

    def test_front_legal_types_do_respond_to_the_rack(self):
        sv = _sv()
        rep = generate_seed(sv)
        dw = rep[0] if isinstance(rep, tuple) else rep.hp
        for hp, solver_cls in ((dw, DoubleWishboneSolver),
                               (from_double_wishbone(dw), MultilinkSolver),
                               (seed_chub_front(sv), CHubFrontSolver)):
            with self.subTest(suspension=type(hp).__name__):
                self.assertGreater(self._toe_response(hp, solver_cls), 1.0)


class TestCornerMoveTweaks(unittest.TestCase):
    """Move a corner fore/aft and up/down without reseeding. Both are
    RIGID: they translate every point, so no curve may change."""

    def _corners(self):
        sv = _sv()
        rep = generate_seed(sv)
        dw = rep[0] if isinstance(rep, tuple) else rep.hp
        return [
            ("double wishbone", dw, DoubleWishboneSolver),
            ("trailing arm", seed_trailing_arm(sv), TrailingArmSolver),
            ("multilink", from_double_wishbone(dw), MultilinkSolver),
            ("c-hub", seed_chub_front(sv), CHubFrontSolver),
            ("loaded halfshaft", seed_loaded_halfshaft(sv),
             LoadedHalfshaftSolver),
            ("h-arm", seed_harm_rear(sv), HArmRearSolver),
        ]

    def test_corner_z_moves_exactly_and_changes_no_kinematics(self):
        for label, hp, solver_cls in self._corners():
            with self.subTest(suspension=label):
                s = DoubleWishboneSolver if False else solver_cls
                before = np.array([metrics.camber_deg(s(hp).solve(float(t)))
                                   for t in (-30.0, 0.0, 30.0)])
                z0 = tweaks.corner_z(hp)
                hp2 = tweaks.set_corner_z(hp, z0 + 17.0)
                self.assertAlmostEqual(tweaks.corner_z(hp2), z0 + 17.0,
                                       places=9)
                after = np.array([metrics.camber_deg(s(hp2).solve(float(t)))
                                  for t in (-30.0, 0.0, 30.0)])
                np.testing.assert_allclose(after, before, atol=1e-9)

    def test_corner_x_and_z_are_independent(self):
        for label, hp, _ in self._corners():
            with self.subTest(suspension=label):
                x0, z0 = tweaks.corner_x(hp), tweaks.corner_z(hp)
                hp2 = tweaks.set_corner_z(tweaks.set_corner_x(hp, x0 + 30.0),
                                          z0 - 12.0)
                self.assertAlmostEqual(tweaks.corner_x(hp2), x0 + 30.0,
                                       places=9)
                self.assertAlmostEqual(tweaks.corner_z(hp2), z0 - 12.0,
                                       places=9)

    def test_corner_z_refuses_when_a_point_is_locked(self):
        """A rigid move drags locked points too, so it must refuse rather
        than silently overriding the pin."""
        hp = seed_chub_front(_sv())
        with self.assertRaises(tweaks.LockedPointError):
            tweaks.set_corner_z(hp, tweaks.corner_z(hp) + 5.0,
                                locked=frozenset({"wheel_center"}))

    def test_every_type_offers_both_rows(self):
        from suspension_tool.gui.panels import TweaksPanel
        for key in ("dw", "ta", "ml", "ch", "lh", "ha"):
            rows = {r for r, _, k in getattr(TweaksPanel,
                                             f"{key.upper()}_ROWS")
                    if k != "head"}
            self.assertIn("corner_x", rows, key)
            self.assertIn("corner_z", rows, key)


class _StubViewport:
    """Stand-in for CornerViewport: the sandbox's offscreen GL aborts on
    its first paint and nothing tested here is 3D."""

    def __new__(cls, *a, **k):
        from PySide6.QtWidgets import QWidget
        w = QWidget()
        w.scene = mock.MagicMock()
        # any other attribute the window pokes at is a no-op mock
        w.__class__ = type("StubViewport", (QWidget,),
                           {"__getattr__": lambda s, n: mock.MagicMock()})
        w.scene = mock.MagicMock()
        return w


class TestMainWindowSeedDispatch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _window(self):
        import suspension_tool.gui.main_window as mw
        patcher = mock.patch.object(mw, "CornerViewport", _StubViewport)
        patcher.start()
        self.addCleanup(patcher.stop)
        win = mw.MainWindow()
        self.app.processEvents()
        return win

    def test_generate_honours_the_type_dropdown(self):
        """The reported bug: pick a type, press Generate, get a DW."""
        win = self._window()
        # front-legal types only: the rear-only ones are covered by
        # test_rear_only_type_forced_onto_the_front_is_refused
        for want in ("C-hub front", "Multilink", "Double wishbone"):
            with self.subTest(type=want):
                win.type_combo.setCurrentText(want)
                self.app.processEvents()
                win.setup_form._emit()          # the Generate button
                self.app.processEvents()
                self.assertEqual(win._type_name_of(win.hp), want)

    def test_front_dropdown_hides_rear_only_types(self):
        win = self._window()
        win.axle_combo.setCurrentIndex(0)
        self.app.processEvents()
        items = {win.type_combo.itemText(i)
                 for i in range(win.type_combo.count())}
        self.assertEqual(items & REAR_ONLY_TYPES, set())
        win.axle_combo.setCurrentIndex(1)
        self.app.processEvents()
        items = {win.type_combo.itemText(i)
                 for i in range(win.type_combo.count())}
        self.assertTrue(REAR_ONLY_TYPES <= items)

    def test_rear_only_type_forced_onto_the_front_is_refused(self):
        """Hiding it in the dropdown is not enough -- a loaded project or a
        script can still ask for it, so the handler must refuse too."""
        win = self._window()
        win.axle_combo.setCurrentIndex(0)
        self.app.processEvents()
        win.setup_form._emit()
        self.app.processEvents()
        before = win._type_name_of(win.hp)
        for name in REAR_ONLY_TYPES:
            win._on_type_changed(name)
            self.app.processEvents()
            self.assertEqual(win._type_name_of(win.hp), before,
                             f"{name} was accepted on the front axle")

    def test_rear_only_type_still_works_on_the_rear(self):
        win = self._window()
        win.axle_combo.setCurrentIndex(1)
        self.app.processEvents()
        for name in sorted(REAR_ONLY_TYPES):
            win.type_combo.setCurrentText(name)
            self.app.processEvents()
            self.assertEqual(win._type_name_of(win.hp), name)


if __name__ == "__main__":
    unittest.main()
