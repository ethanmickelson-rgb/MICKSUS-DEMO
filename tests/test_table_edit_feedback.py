"""The hardpoint table must never fail silently (v1.34.1).

Reported symptom: typing a coordinate did not move the point, and locking a
row did nothing. Three separate ways the table could swallow an action with
no feedback at all are closed here.

  1. refresh() latching. It set _refreshing = True at the top and cleared it
     at the bottom with no try/finally. _refreshing gates _on_cell, so ONE
     exception partway through left the flag set forever: from then on every
     keystroke and every lock was ignored, with no error and no way back but
     a restart. That single mechanism reproduces BOTH reported symptoms at
     once, which is why it is closed first.
  2. The wheel centre's fore/aft station is DERIVED on a double wishbone --
     the knuckle-consistency convention keeps the spin axis in the
     ball-joint plane, so a typed value is projected straight back out. It
     used to vanish without a word.
  3. Locking a halfshaft CV row returned silently, because those rows are
     derived rather than hardpoints.

The MainWindow tests stub the VTK viewport: none of this is 3D, and the
sandbox's offscreen GL aborts on its first paint.
"""

import os
import unittest
from unittest import mock

import numpy as np

from suspension_tool.geometry import DoubleWishbonePoints


class _Stub:
    def __new__(cls, *a, **k):
        from PySide6.QtWidgets import QWidget
        w = QWidget()
        w.__class__ = type("StubViewport", (QWidget,),
                           {"__getattr__": lambda s, n: mock.MagicMock()})
        w.scene = mock.MagicMock()
        return w


class _WindowCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    _win = None

    def _window(self, suspension="Double wishbone"):
        """One MainWindow for the whole class, re-seeded per test.

        Constructing the window costs ~25 s here; seeding costs a fraction
        of that. Building one per test pushed this file alone to nearly
        four minutes, which is not a reasonable share of the suite."""
        import suspension_tool.gui.main_window as mw
        cls = type(self)
        if cls._win is None:
            patcher = mock.patch.object(mw, "CornerViewport", _Stub)
            patcher.start()
            type(self).addClassCleanup(patcher.stop)
            cls._win = mw.MainWindow()
            self.app.processEvents()
        win = cls._win
        win.axle_combo.setCurrentIndex(0)
        self.app.processEvents()
        win.hardpoint_table.set_locked(())        # clean slate per test
        win.type_combo.setCurrentText(suspension)
        self.app.processEvents()
        win.setup_form._fields["has_halfshaft"].setChecked(False)
        win.setup_form._emit()
        self.app.processEvents()
        win.statusBar().clearMessage()
        return win


class TestRefreshCannotBrickTheTable(_WindowCase):

    def test_an_exception_in_refresh_reopens_the_edit_gate(self):
        from suspension_tool.gui import panels
        win = self._window()
        ht = win.hardpoint_table
        orig = panels.HardpointTable._refresh_inner
        fired = [False]

        def exploding(self_, hp):
            if not fired[0]:
                fired[0] = True
                self_._refreshing = True          # latch, then fail
                raise RuntimeError("simulated failure mid-refresh")
            return orig(self_, hp)

        with mock.patch.object(panels.HardpointTable, "_refresh_inner",
                               exploding):
            with self.assertRaises(RuntimeError):
                ht.refresh(win.hp)
        self.assertFalse(ht._refreshing,
                         "_refreshing latched: the table is now dead")

    def test_edits_still_work_after_a_failed_refresh(self):
        from suspension_tool.gui import panels
        win = self._window()
        ht = win.hardpoint_table
        try:
            with mock.patch.object(
                    panels.HardpointTable, "_refresh_inner",
                    side_effect=RuntimeError("boom")):
                ht.refresh(win.hp)
        except RuntimeError:
            pass
        ht.refresh(win.hp)
        self.app.processEvents()
        attr = ht._fields_for(win.hp)[0][0]
        before = np.array(getattr(win.hp, attr), float).copy()
        cell = ht.table.item(0, 2)
        cell.setText(f"{float(cell.text()) + 0.5}")
        self.app.processEvents()
        moved = float(np.linalg.norm(
            np.array(getattr(win.hp, attr), float) - before))
        self.assertGreater(moved, 1.0, "table stopped accepting edits")


class TestSwallowedActionsExplainThemselves(_WindowCase):

    def test_derived_wheel_centre_edit_reports_why(self):
        win = self._window()
        ht = win.hardpoint_table
        rows = [a for a, _ in ht._fields_for(win.hp)]
        row = rows.index("wheel_center")
        cell = ht.table.item(row, 1)          # fore/aft column
        cell.setText(f"{float(cell.text()) + 0.5}")
        self.app.processEvents()
        msg = win.statusBar().currentMessage()
        self.assertIn("DERIVED", msg)
        self.assertIn("wheel", msg.lower())

    def test_a_normal_edit_reports_nothing(self):
        """The explanation must be specific, not a warning on every edit."""
        win = self._window()
        ht = win.hardpoint_table
        win.statusBar().clearMessage()
        cell = ht.table.item(0, 2)            # UCA inner front, Z
        cell.setText(f"{float(cell.text()) + 0.5}")
        self.app.processEvents()
        self.assertNotIn("DERIVED", win.statusBar().currentMessage())

    def test_locking_a_derived_cv_row_reports_why(self):
        win = self._window()
        win.setup_form._fields["has_halfshaft"].setChecked(True)
        win.setup_form._emit()
        self.app.processEvents()
        ht = win.hardpoint_table
        n_points = len(ht._fields_for(win.hp))
        self.assertGreater(ht.table.rowCount(), n_points,
                           "no CV rows to test against")
        ht._toggle_lock(ht.table.rowCount() - 1)
        self.app.processEvents()
        self.assertIn("nothing to lock", win.statusBar().currentMessage())

    def test_locking_a_real_hardpoint_still_locks(self):
        win = self._window()
        ht = win.hardpoint_table
        attr = ht._fields_for(win.hp)[0][0]
        ht._toggle_lock(0)
        self.app.processEvents()
        self.assertIn(attr, ht._locked)
        ht._toggle_lock(0)
        self.app.processEvents()
        self.assertNotIn(attr, ht._locked)


class TestPerpendicularSketchColumn(_WindowCase):
    """v1.34 gave the single-arm carrier types a real 2D sketch (bushing
    axis parallel to carrier pin), so the ⊥ column can be filled for them
    instead of reading a dash."""

    def test_chub_has_a_sketch_normal(self):
        win = self._window("C-hub front")
        text = win.hardpoint_table.table.item(0, 3).text()
        self.assertNotIn(text, ("—", ""))
        float(text)                      # a real number, not a placeholder

    def test_double_wishbone_still_has_one(self):
        win = self._window()
        float(win.hardpoint_table.table.item(0, 3).text())

    def test_multilink_has_none(self):
        """A 5-link upright has no bushing axis, so the column stays a dash
        rather than inventing a plane."""
        win = self._window("Multilink")
        self.assertEqual(win.hardpoint_table.table.item(0, 3).text(), "—")


if __name__ == "__main__":
    unittest.main()
