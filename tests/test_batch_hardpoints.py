"""Batch hardpoint entry (v1.39).

Reported: "sometimes trying to input x y z for everything would have one
of the 3 dimensions lock and would let me enter a value but wouldn't
actually input it."

MEASURED cause, not the one first guessed at. The initial theory was that
single-cell edits walk the linkage through unbuildable intermediate
states and get refused. That was checked and is FALSE: over 864 two-axis
destinations on the example front geometry, every single-axis
intermediate also solved. Nothing was rejected.

What is actually happening is the knuckle-consistency convention, which
makes some coordinates DERIVED. Typing +10 mm into each of the 33
coordinates and asking what survives, in the TOOL frame:

    wheel_center X  ->  0.00 mm kept   (the axle station is derived)
    tierod_outer Y  ->  0.13 mm kept   (the steering-arm plane)

As the table actually presents it -- default Chassis (+Y fwd) display
convention, so the columns are NOT the tool axes -- the wheel centre
swallows its "Y fwd" column and the tie-rod outer swallows ALL THREE,
since the steering-arm plane pins that point in 3D.

Only the wheel-centre case was ever reported, and it was hard-coded by
name. The tie-rod outer swallowed what you typed in silence, which is
indistinguishable from a dead cell -- one specific dimension of one
specific point, exactly as described.

The display/tool frame distinction is itself a trap worth a test: an
earlier cut of this check compared tool-frame axes against display column
indices and reported the wrong axis (and missed the right one).

So two things are pinned here. The derived-coordinate warning, now
general over every point and axis but restricted to cells the user
actually TYPED (other points shifting is the convention working, and
reporting that too would bury the case that matters). And batch mode,
which was the feature requested: hold every cell until Apply, and if the
apply is refused, KEEP what was typed rather than reverting a whole table
because the last cell was wrong.
"""

import os
import unittest

import numpy as np

from suspension_tool.geometry import example_baja_front
from suspension_tool.units import UNITS


class _TableCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def table(self, batch=False, unit="mm"):
        from suspension_tool.gui.panels import HardpointTable
        t = HardpointTable(UNITS[unit])
        t.refresh(example_baja_front())
        t.batch_check.setChecked(batch)
        return t

    def row_of(self, t, attr):
        for row, (a, _) in enumerate(t._fields_for(t._hp)):
            if a == attr:
                return row
        raise AssertionError(attr)

    def type_in(self, t, row, col, text):
        t.table.item(row, col).setText(text)


class TestLiveModeIsUnchanged(_TableCase):

    def test_a_cell_edit_still_emits_immediately(self):
        t = self.table(batch=False)
        seen = []
        t.edited.connect(seen.append)
        self.type_in(t, self.row_of(t, "uca_outer"), 2, "300.0")
        self.assertEqual(len(seen), 1)

    def test_the_batch_buttons_are_hidden_in_live_mode(self):
        t = self.table(batch=False)
        self.assertFalse(t.apply_btn.isVisible())
        self.assertFalse(t.revert_btn.isVisible())


class TestBatchDefersEverything(_TableCase):

    def test_typing_emits_nothing(self):
        t = self.table(batch=True)
        seen = []
        t.edited.connect(seen.append)
        row = self.row_of(t, "uca_outer")
        for col, text in ((0, "-20.0"), (1, "290.0"), (2, "300.0")):
            self.type_in(t, row, col, text)
        self.assertEqual(seen, [], "batch mode must not solve while typing")
        self.assertEqual(len(t._pending), 3)

    def test_apply_emits_once_with_every_cell(self):
        t = self.table(batch=True)
        seen = []
        t.edited.connect(seen.append)
        r1, r2 = self.row_of(t, "uca_outer"), self.row_of(t, "lca_outer")
        self.type_in(t, r1, 2, "300.0")
        self.type_in(t, r2, 2, "120.0")
        t.apply_batch()
        self.assertEqual(len(seen), 1, "one solve, not one per cell")
        self.assertAlmostEqual(float(seen[0].uca_outer[2]), 300.0, places=6)
        self.assertAlmostEqual(float(seen[0].lca_outer[2]), 120.0, places=6)

    def test_apply_clears_the_pending_set(self):
        t = self.table(batch=True)
        self.type_in(t, self.row_of(t, "uca_outer"), 2, "300.0")
        self.assertTrue(t._pending)
        t.apply_batch()
        self.assertFalse(t._pending)

    def test_the_button_counts_what_is_waiting(self):
        t = self.table(batch=True)
        self.assertFalse(t.apply_btn.isEnabled())
        row = self.row_of(t, "uca_outer")
        self.type_in(t, row, 0, "-20.0")
        self.type_in(t, row, 1, "290.0")
        self.assertTrue(t.apply_btn.isEnabled())
        self.assertIn("2", t.apply_btn.text())

    def test_revert_throws_the_edits_away(self):
        t = self.table(batch=True)
        before = float(t._hp.uca_outer[2])
        seen = []
        t.edited.connect(seen.append)
        self.type_in(t, self.row_of(t, "uca_outer"), 2, "300.0")
        t.revert_batch()
        self.assertEqual(seen, [])
        self.assertFalse(t._pending)
        self.assertAlmostEqual(float(t._hp.uca_outer[2]), before, places=9)

    def test_unticking_the_box_applies_what_is_pending(self):
        """Going back to live means the cells on screen become the truth,
        so they must be adopted rather than silently dropped."""
        t = self.table(batch=True)
        seen = []
        t.edited.connect(seen.append)
        self.type_in(t, self.row_of(t, "uca_outer"), 2, "300.0")
        t.batch_check.setChecked(False)
        self.assertEqual(len(seen), 1)
        self.assertAlmostEqual(float(seen[0].uca_outer[2]), 300.0, places=6)

    def test_a_scalar_edit_also_waits(self):
        t = self.table(batch=True)
        seen = []
        t.edited.connect(seen.append)
        t._scalars["tire_radius"].setValue(280.0)
        self.assertEqual(seen, [])
        self.assertTrue(t.apply_btn.isEnabled())
        t.apply_batch()
        self.assertEqual(len(seen), 1)
        self.assertAlmostEqual(float(seen[0].tire_radius), 280.0, places=6)


class TestPendingSurvives(_TableCase):
    """A refresh from anywhere else must not eat unapplied typing."""

    def test_an_unrelated_refresh_keeps_the_typed_text(self):
        t = self.table(batch=True)
        row = self.row_of(t, "uca_outer")
        self.type_in(t, row, 2, "300.0")
        t.refresh(t._hp)                      # e.g. the travel slider moved
        self.assertEqual(t.table.item(row, 2).text(), "300.0")
        self.assertEqual(len(t._pending), 1)

    def test_the_typed_cell_is_marked(self):
        t = self.table(batch=True)
        row = self.row_of(t, "uca_outer")
        self.type_in(t, row, 2, "300.0")
        self.assertTrue(t.table.item(row, 2).font().bold())

    def test_a_unit_change_drops_pending_rather_than_misreading_it(self):
        """300 typed as mm is not 300 as inches, and there is no honest
        way to guess which was meant."""
        t = self.table(batch=True)
        self.type_in(t, self.row_of(t, "uca_outer"), 2, "300.0")
        t.set_units(UNITS["in"])
        self.assertFalse(t._pending)
        self.assertIn("unit", t.edit_status.text().lower())


class TestRejectionKeepsTheWork(_TableCase):
    """The actual report: a refused edit used to take the value you had
    just typed with it."""

    def test_batch_keeps_every_cell_when_the_solver_refuses(self):
        t = self.table(batch=True)
        row = self.row_of(t, "uca_outer")
        self.type_in(t, row, 0, "-20.0")
        self.type_in(t, row, 2, "300.0")
        t.apply_batch()
        t.edit_rejected("links cannot close")
        self.assertEqual(t.table.item(row, 0).text(), "-20.0")
        self.assertEqual(t.table.item(row, 2).text(), "300.0")
        self.assertEqual(len(t._pending), 2, "still pending, still fixable")
        self.assertTrue(t.apply_btn.isEnabled())

    def test_the_reason_is_shown_in_the_panel(self):
        t = self.table(batch=True)
        self.type_in(t, self.row_of(t, "uca_outer"), 2, "300.0")
        t.apply_batch()
        t.edit_rejected("links cannot close")
        self.assertIn("links cannot close", t.edit_status.text())

    def test_live_mode_still_reverts_but_now_explains(self):
        t = self.table(batch=False)
        row = self.row_of(t, "uca_outer")
        before = t.table.item(row, 2).text()
        self.type_in(t, row, 2, "300.0")
        t.edit_rejected("links cannot close")
        self.assertEqual(t.table.item(row, 2).text(), before)
        self.assertIn("links cannot close", t.edit_status.text())

    def test_a_second_apply_after_fixing_goes_through(self):
        t = self.table(batch=True)
        row = self.row_of(t, "uca_outer")
        self.type_in(t, row, 2, "300.0")
        t.apply_batch()
        t.edit_rejected("links cannot close")
        seen = []
        t.edited.connect(seen.append)
        self.type_in(t, row, 2, "260.0")
        t.apply_batch()
        self.assertEqual(len(seen), 1)
        self.assertAlmostEqual(float(seen[0].uca_outer[2]), 260.0, places=6)


class TestAgainstTheRealWindow(_TableCase):
    """End to end, because the revert lives in `apply_hardpoints`, not in
    the table."""

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        from unittest import mock
        from PySide6.QtWidgets import QWidget

        class _Stub(QWidget):
            def __init__(self, *a, **k):
                super().__init__()
                self.scene = mock.MagicMock()

            def __getattr__(self, n):
                return mock.MagicMock()

        import suspension_tool.gui.main_window as mw
        with mock.patch.object(mw, "CornerViewport", _Stub):
            cls.win = mw.MainWindow()
        cls.app.processEvents()
        cls.win.setup_form._emit()
        cls.app.processEvents()

    def test_a_two_axis_move_survives_its_own_intermediate_state(self):
        """The reported workflow: shift one point in two axes at once. In
        batch mode the half-done single-axis geometry is never solved."""
        win = self.win
        t = win.hardpoint_table
        t.batch_check.setChecked(True)
        row = self.row_of(t, "uca_outer")
        base = np.array(t._hp.uca_outer, float)
        u = t._unit
        shown = t._to_view(base)
        for col, delta in ((0, 12.0), (2, 9.0)):
            t.table.item(row, col).setText(
                f"{u.from_mm(shown[col] + delta):.4f}")
        self.app.processEvents()
        t.apply_batch()
        self.app.processEvents()
        got = np.array(win.active.hp.uca_outer, float)
        self.assertAlmostEqual(float(np.linalg.norm(got - base)),
                               float(np.hypot(12.0, 9.0)), delta=0.05)
        t.batch_check.setChecked(False)


if __name__ == "__main__":
    unittest.main()


class TestDerivedCoordinatesAreReported(_TableCase):
    """The measured cause of "it let me enter a value but didn't input it".

    The knuckle-consistency convention makes some coordinates DERIVED. On
    the example front geometry, typing +10 mm into each of the 33
    coordinates and asking what actually survives:

        wheel_center X  ->  0.00 mm kept   (the axle station is derived)
        tierod_outer Y  ->  0.13 mm kept   (the steering-arm plane)

    Only the wheel-centre case was ever reported, and it was hard-coded by
    name. The tie-rod outer discarded 99% of what you typed in silence,
    which is indistinguishable from a broken cell.
    """

    def test_the_tierod_outer_lateral_is_all_but_derived(self):
        """The measurement this whole check exists for."""
        import dataclasses
        from suspension_tool.geometry import enforce_knuckle_planes
        base = enforce_knuckle_planes(example_baja_front())
        p0 = np.array(base.tierod_outer, float)
        want = p0 + np.array([0.0, 10.0, 0.0])
        got = np.array(enforce_knuckle_planes(
            dataclasses.replace(base, tierod_outer=want)).tierod_outer, float)
        self.assertLess(abs(got[1] - p0[1]), 1.0,
                        "if this ever holds, the warning can go")

    def test_every_tierod_outer_column_reports(self):
        """The steering-arm plane pins this point in 3D, so ALL THREE
        display columns swallow what you type. It was the silent one."""
        for col in range(3):
            t = self.table(batch=False)
            shown = t._to_view(t._hp.tierod_outer)
            t.table.item(self.row_of(t, "tierod_outer"), col).setText(
                f"{shown[col] + 10.0:.3f}")
            self.assertIn("DERIVED", t.edit_status.text(), f"column {col}")
            self.assertIn("Tie rod outer", t.edit_status.text())

    def test_the_wheel_centre_fore_aft_still_reports(self):
        """Column 1 is fore/aft in the default Chassis (+Y fwd) display
        convention -- the table's columns are NOT the tool axes, which is
        exactly the trap this check has to get right."""
        t = self.table(batch=False)
        self.assertEqual(t._conv.labels[1], "Y fwd")
        t.table.item(self.row_of(t, "wheel_center"), 1).setText(
            f"{t._to_view(t._hp.wheel_center)[1] + 10.0:.3f}")
        self.assertIn("DERIVED", t.edit_status.text())

    def test_the_wheel_centre_lateral_is_free(self):
        """Only the STATION is derived; the other two take normally."""
        t = self.table(batch=False)
        t.table.item(self.row_of(t, "wheel_center"), 0).setText(
            f"{t._to_view(t._hp.wheel_center)[0] + 10.0:.3f}")
        self.assertEqual(t.edit_status.text(), "")

    def test_a_coordinate_that_does_land_says_nothing(self):
        t = self.table(batch=False)
        t.table.item(self.row_of(t, "uca_outer"), 2).setText(
            f"{t._to_view(t._hp.uca_outer)[2] + 5.0:.3f}")
        self.assertEqual(t.edit_status.text(), "")

    def test_batch_reports_it_on_apply(self):
        t = self.table(batch=True)
        t.table.item(self.row_of(t, "tierod_outer"), 0).setText(
            f"{t._to_view(t._hp.tierod_outer)[0] + 10.0:.3f}")
        self.assertEqual(t.edit_status.text(), "")   # quiet while typing
        t.apply_batch()
        self.assertIn("DERIVED", t.edit_status.text())

    def test_the_note_signal_fires_too(self):
        t = self.table(batch=False)
        seen = []
        t.note.connect(seen.append)
        t.table.item(self.row_of(t, "wheel_center"), 1).setText(
            f"{t._to_view(t._hp.wheel_center)[1] + 10.0:.3f}")
        self.assertTrue(seen)

    def test_non_wishbone_types_are_left_alone(self):
        """The convention is a double-wishbone rule; nothing else has it."""
        from suspension_tool.multilink import from_double_wishbone
        t = self.table(batch=False)
        hp = from_double_wishbone(example_baja_front())
        self.assertEqual(t._projection_note(hp, [("link1_outer", 1)]), "")

    def test_a_consequential_move_is_not_reported_as_a_failure(self):
        """Moving an inner pickup legitimately re-derives the knuckle --
        the tie-rod outer and wheel centre DO shift by millimetres. That
        is the convention working, not a value that failed to land, and
        reporting it would bury the case that matters."""
        t = self.table(batch=False)
        t.table.item(self.row_of(t, "uca_inner_front"), 2).setText(
            f"{t._to_view(t._hp.uca_inner_front)[2] + 5.0:.3f}")
        self.assertEqual(t.edit_status.text(), "")
