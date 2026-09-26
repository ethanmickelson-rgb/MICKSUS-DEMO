"""Deferred-commit numeric inputs, and the CAD-style CG marker (v1.38).

Reported: typing into any non-hardpoint field re-solved the linkage on
every keystroke, so clicking into a box showing `45`, forgetting to clear
it and typing `120` walked the model through 45 -> 451 -> 4512 -> 45123
and threw a link off to infinity. The hardpoint table already required
Enter; everything else did not.

The rule now is: nothing fires until Enter, Tab, focus loss or the
stepper, and the click that focuses a box selects its contents so typing
REPLACES rather than appends. The seed and optimizer panels keep their
apply-on-button behaviour, which means their spins have to be forced to
interpret pending text before the button reads them -- pinned below,
because it is the one way this change could silently lose an edit.
"""

import os
import unittest

import numpy as np
from unittest import mock

from PySide6.QtCore import Qt


class _QtCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])


class TestCommitSpin(_QtCase):

    def spin(self, value=45.0):
        from suspension_tool.gui.widgets import CommitSpin
        s = CommitSpin()
        s.setRange(-1e6, 1e6)
        s.setDecimals(2)
        s.setValue(value)
        return s

    def test_typing_does_not_fire_valuechanged(self):
        """The whole point: 45 -> 45123 must not walk the model through
        451 and 4512 on the way."""
        s = self.spin()
        seen = []
        s.valueChanged.connect(seen.append)
        for text in ("451", "4512", "45123"):
            s.lineEdit().setText(text)
        self.assertEqual(seen, [])
        self.assertEqual(s.value(), 45.0, "value must stay committed")

    def test_commit_applies_the_typed_text_once(self):
        s = self.spin()
        seen = []
        s.valueChanged.connect(seen.append)
        s.lineEdit().setText("45123")
        s.commit()
        self.assertEqual(s.value(), 45123.0)
        self.assertEqual(seen, [45123.0], "exactly one update, not four")

    def test_committing_twice_does_not_fire_twice(self):
        s = self.spin()
        s.lineEdit().setText("120")
        s.commit()
        seen = []
        s.valueChanged.connect(seen.append)
        s.commit()
        self.assertEqual(seen, [])

    def test_escape_abandons_the_edit(self):
        s = self.spin()
        s.lineEdit().setText("99999")
        s.keyPressEvent(_key(Qt.Key.Key_Escape))
        s.commit()
        self.assertEqual(s.value(), 45.0)

    def test_escape_restores_the_text_with_prefix_and_suffix(self):
        s = self.spin()
        s.setSuffix(" mm")
        s.lineEdit().setText("99999")
        s.keyPressEvent(_key(Qt.Key.Key_Escape))
        self.assertEqual(s.text(), "45.00 mm")

    def test_the_stepper_still_acts_immediately(self):
        """Deferring the KEYBOARD must not defer the arrows -- nudging a
        value is the one place live feedback is wanted."""
        s = self.spin()
        seen = []
        s.valueChanged.connect(seen.append)
        s.setSingleStep(1.0)
        s.stepUp()
        self.assertEqual(seen, [46.0])

    def test_setvalue_from_code_still_fires(self):
        """Panels push values programmatically all the time."""
        s = self.spin()
        seen = []
        s.valueChanged.connect(seen.append)
        s.setValue(3.5)
        self.assertEqual(seen, [3.5])

    def test_mouse_focus_selects_all_so_typing_replaces(self):
        s = self.spin()
        s.focusInEvent(_focus(Qt.FocusReason.MouseFocusReason))
        s.mousePressEvent(_click())
        self.assertTrue(s.lineEdit().hasSelectedText())

    def test_a_second_click_leaves_the_cursor_alone(self):
        """Click once to replace the number, click again to edit a digit."""
        s = self.spin()
        s.focusInEvent(_focus(Qt.FocusReason.MouseFocusReason))
        s.mousePressEvent(_click())
        s.lineEdit().deselect()
        s.mousePressEvent(_click())
        self.assertFalse(s.lineEdit().hasSelectedText())

    def test_the_int_twin_behaves_the_same(self):
        from suspension_tool.gui.widgets import CommitIntSpin
        s = CommitIntSpin()
        s.setRange(0, 10000)
        s.setValue(12)
        seen = []
        s.valueChanged.connect(seen.append)
        s.lineEdit().setText("1234")
        self.assertEqual(seen, [])
        s.commit()
        self.assertEqual(s.value(), 1234)


class TestCommitSpins(_QtCase):
    """The button-press safety net."""

    def test_it_commits_every_pending_child(self):
        from PySide6.QtWidgets import QWidget, QVBoxLayout
        from suspension_tool.gui.widgets import CommitSpin, commit_spins
        box = QWidget()
        lay = QVBoxLayout(box)
        spins = []
        for _ in range(3):
            s = CommitSpin()
            s.setRange(0, 1e6)
            lay.addWidget(s)
            spins.append(s)
        for s in spins:
            s.lineEdit().setText("777")
        self.assertTrue(all(s.value() == 0.0 for s in spins))
        commit_spins(box)
        self.assertTrue(all(s.value() == 777.0 for s in spins))

    def test_it_ignores_plain_qt_spins(self):
        from PySide6.QtWidgets import QWidget, QVBoxLayout, QDoubleSpinBox
        from suspension_tool.gui.widgets import commit_spins
        box = QWidget()
        QVBoxLayout(box).addWidget(QDoubleSpinBox())
        commit_spins(box)          # must not raise


class TestPanelsDeferCommit(_QtCase):
    """The panels the report was actually about."""

    def _tweaks(self):
        from suspension_tool.gui.panels import TweaksPanel
        from suspension_tool.units import UNITS
        return TweaksPanel(UNITS["mm"])

    def test_the_tweaks_panel_does_not_recompute_mid_word(self):
        from suspension_tool.gui.widgets import CommitSpin
        p = self._tweaks()
        spins = p.findChildren(CommitSpin)
        self.assertTrue(spins, "tweaks panel should use CommitSpin")
        self.assertTrue(all(not s.keyboardTracking() for s in spins))

    def test_every_live_panel_uses_commit_spins(self):
        """A panel that recomputes on `changed` must not live-update from
        the keyboard -- that is the whole report."""
        from PySide6.QtWidgets import QDoubleSpinBox
        from suspension_tool.gui.panels import (FramePanel, HalfshaftPanel,
                                                TweaksPanel, VehiclePanel)
        from suspension_tool.gui.dynamics_panel import DynamicsPanel
        from suspension_tool.units import UNITS
        u = UNITS["mm"]
        for cls in (TweaksPanel, VehiclePanel, FramePanel, HalfshaftPanel,
                    DynamicsPanel):
            panel = cls(u)
            spins = panel.findChildren(QDoubleSpinBox)
            self.assertTrue(spins, cls.__name__)
            for s in spins:
                self.assertFalse(s.keyboardTracking(),
                                 f"{cls.__name__}: a spin still live-updates")

    def test_the_seed_form_still_applies_only_on_its_button(self):
        """Explicitly unchanged behaviour -- and the pending edit must be
        picked up, not read stale, when the button fires."""
        from suspension_tool.gui.panels import SetupForm
        from suspension_tool.units import UNITS
        f = SetupForm(UNITS["mm"])
        seen = []
        f.generate_requested.connect(seen.append)
        spin = f._fields["track_width"]      # a length row, shown in mm
        spin.lineEdit().setText("900")
        self.assertEqual(seen, [], "typing must not run the seed")
        f._emit()
        self.assertEqual(len(seen), 1)
        self.assertAlmostEqual(seen[0].track_width, 900.0, places=6,
                               msg="the button read a stale value")

    def test_the_optimizer_panel_picks_up_a_pending_edit_too(self):
        from suspension_tool.gui.panels import OptimizePanel
        from suspension_tool.gui.widgets import CommitSpin
        from suspension_tool.units import UNITS
        p = OptimizePanel(UNITS["mm"])
        seen = []
        p.optimize_requested.connect(seen.append)
        spins = p.findChildren(CommitSpin)
        self.assertTrue(spins)
        self.assertEqual(seen, [])
        p._emit()
        self.assertEqual(len(seen), 1)


class TestCgMarker(unittest.TestCase):
    """The CAD centre-of-mass symbol."""

    def meshes(self, r=22.0):
        from suspension_tool.gui.viewport import _cg_octant_meshes
        return _cg_octant_meshes(r)

    def test_the_sphere_splits_into_two_even_octant_groups(self):
        light, dark = self.meshes()
        self.assertGreater(light.n_cells, 0)
        self.assertGreater(dark.n_cells, 0)
        total = light.n_cells + dark.n_cells
        self.assertLess(abs(light.n_cells - dark.n_cells) / total, 0.02)

    def test_each_group_holds_one_parity_only(self):
        """Opposite octants share a colour -- that is what makes it the
        chequer and not four random patches."""
        for mesh, want in zip(self.meshes(), (0, 1)):
            c = mesh.cell_centers().points
            par = ((c[:, 0] >= 0).astype(int) + (c[:, 1] >= 0).astype(int)
                   + (c[:, 2] >= 0).astype(int)) % 2
            self.assertEqual(set(np.unique(par)), {want})

    def test_the_radius_is_honoured(self):
        for r in (4.4, 22.0):
            for mesh in self.meshes(r):
                self.assertAlmostEqual(
                    float(np.abs(mesh.points).max()), r, delta=0.05 * r)

    def test_both_groups_together_cover_the_whole_sphere(self):
        import pyvista as pv
        light, dark = self.meshes()
        full = pv.Sphere(radius=22.0, center=(0, 0, 0),
                         theta_resolution=48, phi_resolution=48)
        self.assertEqual(light.n_cells + dark.n_cells, full.n_cells)


class TestCgMarkerScalesWithTheCar(_QtCase):
    """A 44 mm ball on a 260 mm-wheelbase RC car buries the geometry it is
    supposed to annotate. The radius comes from the car, not from the
    ground plane's 700 mm half-track floor."""

    def _scene_radius(self, wheel_y, tire_r, wheelbase):
        # the sizing rule, mirrored from viewport.rebuild
        car = max(2.0 * (abs(wheel_y) + tire_r), wheelbase)
        return float(np.clip(0.015 * car, 2.0, 40.0))

    def test_a_baja_car_keeps_roughly_the_old_marker(self):
        r = self._scene_radius(760.0, 292.0, 1550.0)
        self.assertGreater(r, 18.0)
        self.assertLess(r, 40.0)

    def test_an_rc_car_gets_a_much_smaller_one(self):
        r = self._scene_radius(110.2, 37.5, 260.0)
        self.assertLess(r, 8.0)
        self.assertGreater(r, 2.0)

    def test_it_never_collapses_to_nothing(self):
        self.assertGreaterEqual(self._scene_radius(1.0, 1.0, 1.0), 2.0)


def _key(k):
    from PySide6.QtGui import QKeyEvent
    from PySide6.QtCore import QEvent
    return QKeyEvent(QEvent.Type.KeyPress, k, Qt.KeyboardModifier.NoModifier)


def _focus(reason):
    from PySide6.QtGui import QFocusEvent
    from PySide6.QtCore import QEvent
    return QFocusEvent(QEvent.Type.FocusIn, reason)


def _click():
    from PySide6.QtGui import QMouseEvent
    from PySide6.QtCore import QEvent, QPointF
    return QMouseEvent(QEvent.Type.MouseButtonPress, QPointF(5.0, 5.0),
                       Qt.MouseButton.LeftButton, Qt.MouseButton.LeftButton,
                       Qt.KeyboardModifier.NoModifier)


if __name__ == "__main__":
    unittest.main()
