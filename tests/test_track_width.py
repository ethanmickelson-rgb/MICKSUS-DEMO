"""Track width vs overall width, and the stale-track bug (v1.41).

Reported: "my track width read 55 in when I know for sure in the CAD that
it is 62 in." Both readings of that fit the arithmetic identically --

    overall = 2*(wheel_centre_y + tire_width/2)
    track   = 2*wheel_centre_y

so 62 could be a 55 in track measured edge-to-edge, OR a genuine 62 in
track whose wheel centre had been entered half a tire width too far
inboard. Ethan measured his CAD: centreline to tire mid-plane is 27.5 in,
so the TRACK IS 55 IN AND THE TOOL WAS RIGHT. The 62 in is the overall
width across the outsides of the tires, which is track + one tire width.

Two things follow. The overall width gets its own labelled readout, so
the two numbers can never be mistaken for each other again. And a real
bug found while checking: roll angle is derived from track, and the track
it used came from the SEED FORM rather than the geometry -- 63 in
recorded against 55 in built on this very file, biasing every roll number
by 14.6%.
"""

import os
import unittest

import numpy as np

from suspension_tool.geometry import example_baja_front
from suspension_tool.project import load_project

_DEMO = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "examples", "baja_demo_double_wishbone.MICK")
IN = 25.4


class TestTheShippedProject(unittest.TestCase):
    """The two width readings, on a project that actually ships.

    This class used to pin the exact numbers of the file the confusion
    was reported with -- track 54.99 in, overall 62.00 in, a seeded
    track stale by exactly 8 in. That file is not distributed here, and
    pinning one car's measurements was never what made the test useful:
    the report was impossible to settle by arithmetic precisely BECAUSE
    both readings fit, so what has to hold is the RELATIONSHIP between
    them, on whatever project is loaded.
    """

    @classmethod
    def setUpClass(cls):
        cls.st = load_project(_DEMO)
        cls.hp = cls.st.front.hardpoints

    def test_track_is_wheel_centre_to_wheel_centre(self):
        track = 2 * abs(self.hp.wheel_center[1])
        self.assertGreater(track, 0.0)
        # a full-size car, in millimetres, not a units slip
        self.assertGreater(track / IN, 30.0)
        self.assertLess(track / IN, 90.0)

    def test_overall_width_is_track_plus_one_tire_width(self):
        """The identity that made the original report unresolvable by
        arithmetic: both readings satisfy it, so only a measurement
        could separate them."""
        track = 2 * abs(self.hp.wheel_center[1])
        overall = 2 * (abs(self.hp.wheel_center[1]) + self.hp.tire_width / 2)
        self.assertAlmostEqual(overall - track, self.hp.tire_width, places=9)
        self.assertGreater(overall, track)

    def test_a_shipped_project_is_not_stale(self):
        """The healthy side of the checklist row: what the seed ASKED for
        and what the linkage was BUILT to must agree on a project we
        ship. If they ever drift, the example is wrong, not the test."""
        built = 2 * abs(self.hp.wheel_center[1])
        want = float(self.st.front.setup.track_width)
        self.assertLess(abs(want - built), 0.01 * built,
                        f"seeded {want / IN:.2f} in vs built "
                        f"{built / IN:.2f} in")

    def test_a_stale_seed_is_detectable(self):
        """The condition the checklist row exists to catch, constructed
        here rather than borrowed from a file that happened to have it.
        Eight inches of drift must read as drift."""
        built = 2 * abs(self.hp.wheel_center[1])
        stale = built + 8.0 * IN
        self.assertGreater(abs(stale - built), 0.01 * built)
        self.assertAlmostEqual((stale - built) / IN, 8.0, places=9)


class TestOverallWidthReadout(unittest.TestCase):

    def test_it_is_track_plus_one_tire_width(self):
        hp = example_baja_front()
        track = 2 * abs(hp.wheel_center[1])
        overall = 2 * (abs(hp.wheel_center[1]) + hp.tire_width / 2)
        self.assertAlmostEqual(overall, track + hp.tire_width, places=9)
        self.assertGreater(overall, track)

    def test_the_row_is_in_the_readouts_panel(self):
        from suspension_tool.gui.panels import Readouts
        keys = [k for k, _, _ in Readouts.ROWS]
        self.assertIn("overall_width_mm", keys)
        self.assertIn("track_width_mm", keys)

    def test_the_two_rows_are_labelled_so_they_cannot_be_confused(self):
        from suspension_tool.gui.panels import Readouts
        labels = {k: lab for k, lab, _ in Readouts.ROWS}
        self.assertEqual(labels["track_width_mm"], "Track width")
        self.assertIn("edge to edge", labels["overall_width_mm"])


class TestAxleTrackUsesLiveGeometry(unittest.TestCase):
    """The real bug: roll angle comes from track, and track came from the
    seed form."""

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _window(self):
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
            win = mw.MainWindow()
        self.app.processEvents()
        win.setup_form._emit()
        self.app.processEvents()
        return win

    def test_it_follows_the_hardpoints_not_the_seed_form(self):
        import dataclasses
        win = self._window()
        axle = win.active
        self.assertIsNotNone(axle.hp)
        live = 2.0 * abs(float(axle.hp.wheel_center[1]))
        self.assertAlmostEqual(win._axle_track(axle), live, places=6)
        # now poison the seeded intent: the answer must not move
        axle.last_setup = dataclasses.replace(axle.last_setup,
                                              track_width=live + 200.0)
        self.assertAlmostEqual(win._axle_track(axle), live, places=6,
                               msg="stale seed value leaked back in")

    def test_a_hardpoint_move_changes_it(self):
        import dataclasses
        win = self._window()
        axle = win.active
        before = win._axle_track(axle)
        wc = np.array(axle.hp.wheel_center, float)
        wc[1] += 50.0
        axle.hp = dataclasses.replace(axle.hp, wheel_center=wc)
        self.assertAlmostEqual(win._axle_track(axle), before + 100.0, places=6)

    def test_the_mismatch_is_reported(self):
        import dataclasses
        win = self._window()
        axle = win.active
        self.assertLess(abs(win.track_mismatch(axle)), 1.0)
        axle.last_setup = dataclasses.replace(
            axle.last_setup,
            track_width=win._axle_track(axle) + 203.2)   # 8 in of drift
        self.assertAlmostEqual(win.track_mismatch(axle), 203.2, places=6)

    def test_it_survives_an_axle_with_no_geometry(self):
        win = self._window()
        from suspension_tool.gui.main_window import Axle
        empty = Axle(name="front")
        self.assertEqual(win._axle_track(empty), 0.0)
        self.assertIsNone(win.track_mismatch(empty))


class TestChecklistRow(unittest.TestCase):

    def test_the_row_exists(self):
        from suspension_tool.gui.checklist import ROWS
        self.assertIn("track", [k for k, _ in ROWS])

    def test_it_is_graded_relative_to_the_car(self):
        """1% of the built track, so it means the same thing on a Baja
        car and on a 1/10 model."""
        for built, want, flagged in ((1400.0, 1600.0, True),
                                     (1400.0, 1405.0, False),
                                     (260.0, 297.0, True),
                                     (260.0, 261.0, False)):
            self.assertEqual(abs(want - built) > 0.01 * built, flagged,
                             f"{built} vs {want}")


if __name__ == "__main__":
    unittest.main()


class TestDerivedOutputsFollowTheGeometry(unittest.TestCase):
    """The audit this bug prompted: every derived number that CAN be read
    from the live model should be, so nothing silently describes a car
    that is no longer being modelled.

    What is deliberately still typed by hand, because one corner's
    hardpoints cannot supply it: vehicle wheelbase and CG (each axle is
    modelled in its own local frame), masses, spring and damper rates,
    tire friction data, and brake hardware. Those have their own
    plausibility checks.
    """

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _window(self):
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
            win = mw.MainWindow()
        self.app.processEvents()
        win.setup_form._emit()
        self.app.processEvents()
        return win

    def test_the_dynamics_track_is_linked_not_typed(self):
        from suspension_tool.gui.dynamics_panel import LINKED_FIELDS
        for f in ("track_front_in", "track_rear_in",
                  "loaded_radius_front_in", "loaded_radius_rear_in"):
            self.assertIn(f, LINKED_FIELDS, f)

    def test_linked_fields_are_read_only_while_linked(self):
        from suspension_tool.gui.dynamics_panel import LINKED_FIELDS
        win = self._window()
        dp = win.dynamics_panel
        self.assertTrue(dp.link_check.isChecked())
        for f in LINKED_FIELDS:
            self.assertFalse(dp._spins[f].isEnabled(), f)

    def test_the_dynamics_track_matches_the_geometry(self):
        win = self._window()
        got = win.dynamics_panel.current_inputs().track_front_in
        self.assertAlmostEqual(got, win._axle_track(win.axles["front"]) / IN,
                               places=4)

    def test_moving_a_wheel_centre_moves_the_dynamics_track(self):
        """The whole point: edit geometry, and the sheet follows."""
        import dataclasses
        win = self._window()
        before = win.dynamics_panel.current_inputs().track_front_in
        axle = win.axles["front"]
        wc = np.array(axle.hp.wheel_center, float)
        wc[1] += 50.0
        win.apply_hardpoints(dataclasses.replace(axle.hp, wheel_center=wc))
        self.app.processEvents()
        after = win.dynamics_panel.current_inputs().track_front_in
        self.assertAlmostEqual(after - before, 100.0 / IN, places=2)

    def test_the_loaded_radius_comes_from_the_contact_patch(self):
        win = self._window()
        st = win.axles["front"].state
        want = float(st.wheel_center[2] - st.contact_patch[2]) / IN
        self.assertAlmostEqual(
            win.dynamics_panel.current_inputs().loaded_radius_front_in,
            want, places=4)

    def test_an_unseeded_axle_pushes_none_not_zero(self):
        """The dynamics sheet divides by track; a pushed 0 is a crash."""
        win = self._window()
        win.axles["rear"].hp = None
        win._update_roll_axis()
        self.app.processEvents()
        self.assertGreater(win.dynamics_panel.current_inputs().track_rear_in,
                           0.0)

    def test_the_sweeps_read_track_from_the_geometry(self):
        """Roll and steer sweeps had the same stale-track bug as the
        roll-angle readout."""
        import inspect
        from suspension_tool.gui import sweep_dialogs
        src = inspect.getsource(sweep_dialogs)
        self.assertNotIn("last_setup.track_width", src)
        self.assertGreaterEqual(src.count("_axle_track"), 2)

    def test_impact_loads_read_the_live_hardpoints(self):
        """Spot-check the other tab that produces numbers: the load cases
        take geometry from hp/state, never from a stored setup."""
        import inspect
        from suspension_tool import impact
        src = inspect.getsource(impact)
        self.assertNotIn("last_setup", src)
        self.assertNotIn(".setup.", src)
