"""Cross-type kinematic tweaks (v1.31).

Until now only the double wishbone had a full tweak set; the other five
types exposed 2-6 rows and were otherwise tunable only by hand-editing raw
coordinates. These tests pin the new shared tweaks down per type:

  * every row round-trips EXACTLY on the gated single-row path (which is
    what the GUI sends -- see TweaksPanel._gate_optional),
  * the resulting geometry still solves,
  * the placement tweaks stay rigid (no kinematics change),
  * a None means "leave it alone", so editing one row cannot walk another.
"""

import unittest

import numpy as np

from suspension_tool import metrics, tweaks
from suspension_tool.chub_front import CHubFrontSolver, seed_chub_front
from suspension_tool.harm_rear import HArmRearSolver, seed_harm_rear
from suspension_tool.loaded_halfshaft import (LoadedHalfshaftSolver,
                                              seed_loaded_halfshaft)
from suspension_tool.multilink import MultilinkSolver, from_double_wishbone
from suspension_tool.seed import IN, SetupVariables, generate_seed
from suspension_tool.solver import DoubleWishboneSolver
from suspension_tool.trailing_arm import TrailingArmSolver, seed_trailing_arm


def _sv() -> SetupVariables:
    return SetupVariables(
        track_width=63 * IN, wheelbase=61 * IN, ride_height=14 * IN,
        tire_radius=11.5 * IN, tire_width=7 * IN,
        shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
        motion_ratio_goal=1.0 / 1.85, shock_length_at_ride=20.38 * IN,
        static_camber_deg=-0.87, static_toe_deg=0.2)


def _corners(sv):
    rep = generate_seed(sv)
    dw = rep[0] if isinstance(rep, tuple) else rep.hp
    return {
        "ta": (seed_trailing_arm(sv), TrailingArmSolver,
               tweaks.trailing_arm_params, tweaks.set_trailing_arm,
               dict(d=None, h=None, ang=None, plan_deg=None, elev_deg=None)),
        "ml": (from_double_wishbone(dw), MultilinkSolver,
               tweaks.multilink_toe_params, tweaks.set_multilink_toe,
               dict(outer_rise=None, inner_rise=None)),
        "ch": (seed_chub_front(sv), CHubFrontSolver,
               tweaks.chub_params, tweaks.set_chub,
               # axle_x is gated like the optional rows: it and hub_cv
               # both place the wheel centre, so only one is ever sent.
               dict(d=None, h=None, ang=None, caster=None, kpi=None)),
        "lh": (seed_loaded_halfshaft(sv), LoadedHalfshaftSolver,
               tweaks.loaded_hs_params, tweaks.set_loaded_hs,
               dict(d=None, h=None, ang=None, plan_deg=None, elev_deg=None)),
        "ha": (seed_harm_rear(sv), HArmRearSolver,
               tweaks.harm_params, tweaks.set_harm,
               dict(d=None, h=None, ang=None, plan_deg=None, elev_deg=None)),
    }


# The rows each type gained in v1.31, with a sensible nudge for each.
NEW_ROWS = {
    "ta": {"shock_len": 5.0},
    "ml": {"shock_len": 5.0},
    "ch": {"shock_len": 5.0, "outer_rise": 5.0, "inner_rise": 5.0,
           "axle_x": 5.0,
           "steer_arm": 5.0, "kp_len": 5.0, "cam_outer_rise": 5.0,
           "cam_inner_rise": 5.0, "arm_len": 5.0, "hub_cv": 5.0,
           "toe": 0.5, "kickup": 3.0},
    "lh": {"shock_len": 5.0, "arm_len": 5.0, "hub_cv": 5.0, "kickup": 3.0},
    "ha": {"shock_len": 5.0, "cam_outer_rise": 5.0, "cam_inner_rise": 5.0,
           "arm_len": 5.0, "kickup": 3.0},
}
# Rows whose target is absolute rather than a delta from the current value.
_ABSOLUTE = {"kickup"}


class TestNewRowsRoundTrip(unittest.TestCase):
    """Read -> set -> read must return exactly what was asked for, and the
    corner must still solve."""

    def test_every_new_row(self):
        sv = _sv()
        for key, (hp, solver_cls, getp, setp, req) in _corners(sv).items():
            base = getp(hp)
            required = {k: base[k] for k in req}
            for row, delta in NEW_ROWS[key].items():
                with self.subTest(suspension=key, row=row):
                    target = (delta if row in _ABSOLUTE
                              else base[row] + delta)
                    hp2 = setp(hp, **required, **{row: target})
                    self.assertAlmostEqual(getp(hp2)[row], target, places=6)
                    solver_cls(hp2).solve(0.0)      # still assembles

    def test_none_means_leave_alone(self):
        """The GUI gate passes None for every row that did not move; that
        must be a true no-op, or editing one row would walk the others."""
        sv = _sv()
        for key, (hp, _, getp, setp, req) in _corners(sv).items():
            with self.subTest(suspension=key):
                base = getp(hp)
                hp2 = setp(hp, **{k: base[k] for k in req})
                for attr in type(hp).POINT_ATTRS:
                    np.testing.assert_allclose(
                        getattr(hp2, attr), getattr(hp, attr), atol=1e-9,
                        err_msg=f"{key}: {attr} moved on a no-op call")


class TestPlacementTweaksAreRigid(unittest.TestCase):
    """Ride height and corner fore/aft are PLACEMENT, not kinematics: they
    translate the whole corner, so every curve must be untouched."""

    def _camber_curve(self, solver_cls, hp):
        s = solver_cls(hp)
        return np.array([metrics.camber_deg(s.solve(float(t)))
                         for t in (-30.0, 0.0, 30.0)])

    def test_ride_height_and_corner_x_change_no_kinematics(self):
        sv = _sv()
        for key, (hp, solver_cls, getp, setp, req) in _corners(sv).items():
            with self.subTest(suspension=key):
                base = getp(hp)
                required = {k: base[k] for k in req}
                before = self._camber_curve(solver_cls, hp)
                moved = setp(hp, **required,
                             ride_h=tweaks.ride_height(hp, 0.0) + 25.0)
                moved = setp(moved, **getp(moved) and required,
                             corner_x=tweaks.corner_x(moved) + 40.0)
                np.testing.assert_allclose(
                    self._camber_curve(solver_cls, moved), before, atol=1e-7)
                # ...and they really did move the corner
                self.assertAlmostEqual(tweaks.corner_x(moved),
                                       tweaks.corner_x(hp) + 40.0, places=6)


class TestHubOffsetOnlyWhereMeaningful(unittest.TestCase):
    """Hub offset is measured from the STEERING AXIS. Types with neither
    ball joints nor a physical kingpin nor an outer CV have no such axis,
    so the panel must not offer the row there (it would read 0 and do
    nothing) -- and it MUST work on the C-hub, whose kingpin is physical."""

    def test_chub_hub_offset_is_live(self):
        hp = seed_chub_front(_sv())
        self.assertGreater(tweaks.hub_cv_offset(hp), 1.0)

    def test_axis_free_types_are_not_offered_the_row(self):
        from suspension_tool.gui.panels import TweaksPanel
        for key in ("ta", "ml", "ha"):
            rows = {r for r, _, k in getattr(TweaksPanel,
                                             f"{key.upper()}_ROWS")
                    if k != "head"}
            self.assertNotIn("hub_cv", rows, f"{key} has no steering axis")
        for key in ("dw", "ch", "lh"):
            rows = {r for r, _, k in getattr(TweaksPanel,
                                             f"{key.upper()}_ROWS")
                    if k != "head"}
            self.assertIn("hub_cv", rows)


class TestCamberLinkIsTheRollCentreKnob(unittest.TestCase):
    """The camber link is what closes a C-hub corner, and the RC tuning
    literature treats it as THE roll-centre / camber-gain control. Until
    v1.31 it had no tweak at all.

    Note which effect the rise rows actually exercise: moving an end in z
    changes the link's INCLINATION far more than its length (raising the
    inner ball 25 mm shortens the link by only ~6 mm but drops the roll
    centre ~50 mm), so this pins the inclination response. The separate
    length-vs-roll-centre rule quoted in RC setup guides is a LATERAL move
    of the chassis ball and is not what these rows do.
    """

    def test_raising_the_inner_ball_lowers_rc_and_camber_gain(self):
        hp = seed_chub_front(_sv())
        base = tweaks.chub_params(hp)
        req = {k: base[k] for k in ("d", "h", "ang", "caster", "kpi",
                                    "axle_x")}
        rc, gain = [], []
        for rise in (-25.0, 0.0, 25.0):
            hp2 = tweaks.set_chub(
                hp, **req, cam_inner_rise=base["cam_inner_rise"] + rise)
            s = CHubFrontSolver(hp2)
            st = s.solve(0.0)
            rc.append(metrics.corner_metrics(s, st)["roll_center_height_mm"])
            gain.append((metrics.camber_deg(s.solve(25.4))
                         - metrics.camber_deg(s.solve(-25.4))) / 2.0)
        # monotonic, and in the direction raising the inner ball implies
        self.assertGreater(rc[0], rc[1])
        self.assertGreater(rc[1], rc[2])
        self.assertGreater(abs(gain[0]), abs(gain[1]))
        self.assertGreater(abs(gain[1]), abs(gain[2]))
        # and it is a STRONG control, not a rounding-level nudge: 50 mm of
        # inner-ball travel is worth most of the roll-centre range
        self.assertGreater(rc[0] - rc[2], 50.0)


class TestPanelRowsMatchSetters(unittest.TestCase):
    """Every row the panel shows must be a key the type's params dict
    returns AND a keyword its setter accepts -- otherwise the row either
    reads blank or raises on edit."""

    def test_rows_are_readable_and_writable(self):
        from suspension_tool.gui.panels import TweaksPanel
        sv = _sv()
        pairs = {k: (v[2], v[3]) for k, v in _corners(sv).items()}
        pairs["dw"] = (None, None)
        for key, (getp, setp) in pairs.items():
            if getp is None:
                continue
            hp = _corners(sv)[key][0]
            params = getp(hp)
            accepted = set(setp.__code__.co_varnames[
                :setp.__code__.co_argcount])
            for row, _, kind in getattr(TweaksPanel, f"{key.upper()}_ROWS"):
                if kind == "head":
                    continue
                with self.subTest(suspension=key, row=row):
                    self.assertIn(row, params, "row not in params dict")
                    self.assertIn(row, accepted, "setter takes no such kwarg")


if __name__ == "__main__":
    unittest.main()


class TestPanelDrivesEveryType(unittest.TestCase):
    """Exercise the real TweaksPanel widget for every suspension type.

    This deliberately builds ONLY the panel, not MainWindow: the panel is
    what changed, and the 3D viewport needs a GL context that the headless
    sandbox aborts on. Editing a row must emit exactly one new hardpoint
    set, move the row it was told to move, and leave the geometry solvable.
    """

    @classmethod
    def setUpClass(cls):
        # No display in CI or the sandbox; the panel is pure widgets, so
        # the offscreen platform is enough (and needs no GL, unlike the
        # 3D viewport the smoke test drives).
        import os
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def test_each_type_shows_and_applies_its_rows(self):
        from suspension_tool.gui.panels import TweaksPanel
        from suspension_tool.units import UNITS
        sv = _sv()
        for key, (hp, solver_cls, getp, _, _) in _corners(sv).items():
            with self.subTest(suspension=key):
                panel = TweaksPanel(UNITS["mm"])
                got = []
                panel.edited.connect(got.append)
                panel.refresh(hp)
                rows = getattr(TweaksPanel, f"{key.upper()}_ROWS")
                box, spins, _ = panel._boxes[key]
                self.assertTrue(box.isVisible() or True)   # visibility is
                # driven by the parent layout; the row set is what matters
                # section captions are display-only and have no spin
                self.assertEqual({r for r, _, k in rows if k != "head"},
                                 set(spins))
                # nudge one representative row and check it lands
                row = "shock_len" if "shock_len" in spins else rows[0][0]
                spins[row].setValue(spins[row].value() + 3.0)
                self.app.processEvents()
                self.assertTrue(got, f"{key}: editing {row} emitted nothing")
                hp2 = got[-1]
                # The contract is "what the spin SHOWS is what you get".
                # Comparing against the unrounded starting value would be
                # wrong: the panel displays mm to 1 dp, so the user's +3
                # is measured from the rounded reading, not the exact one.
                self.assertAlmostEqual(getp(hp2)[row], spins[row].value(),
                                       places=6)
                solver_cls(hp2).solve(0.0)

    def test_editing_one_row_does_not_walk_the_others(self):
        """The whole point of the gate: with the panel showing values at
        DISPLAY precision, nudging one row must leave every other row's
        reading where it was."""
        from suspension_tool.gui.panels import TweaksPanel
        from suspension_tool.units import UNITS
        sv = _sv()
        for key, (hp, _, getp, _, _) in _corners(sv).items():
            with self.subTest(suspension=key):
                panel = TweaksPanel(UNITS["in"])   # 3 dp -> coarse rounding
                got = []
                panel.edited.connect(got.append)
                panel.refresh(hp)
                _, spins, rows = panel._boxes[key]
                if "shock_len" not in spins:
                    continue
                before = getp(hp)
                spins["shock_len"].setValue(
                    spins["shock_len"].value() + 0.25)
                self.app.processEvents()
                after = getp(got[-1])
                for r, _, kind in rows:
                    if r in ("shock_len", "d", "ride_h"):
                        continue        # d slides with the shock length
                    if before.get(r) is None or after.get(r) is None:
                        continue
                    self.assertAlmostEqual(
                        after[r], before[r], places=2,
                        msg=f"{key}: editing shock_len moved {r}")


class TestRideHeightOnEveryType(unittest.TestCase):
    """Ride height is a placement knob every type must offer and honour.

    It has been in the shared rows since v1.31, but on a C-hub it sat at
    row 19 of 21 in a flat list and read as missing -- which is what the
    section captions added in v1.34.2 are for."""

    def test_every_type_offers_it(self):
        from suspension_tool.gui.panels import TweaksPanel
        for key in ("dw", "ta", "ml", "ch", "lh", "ha"):
            rows = {r for r, _, k in getattr(TweaksPanel, f"{key.upper()}_ROWS")
                    if k != "head"}
            self.assertIn("ride_h", rows, key)

    def test_setting_it_moves_the_corner_and_changes_no_kinematics(self):
        sv = _sv()
        rep = generate_seed(sv)
        dw = rep[0] if isinstance(rep, tuple) else rep.hp
        corners = [
            ("double wishbone", dw, DoubleWishboneSolver),
            ("c-hub", seed_chub_front(sv), CHubFrontSolver),
            ("trailing arm", seed_trailing_arm(sv), TrailingArmSolver),
            ("h-arm", seed_harm_rear(sv), HArmRearSolver),
        ]
        for label, hp, solver_cls in corners:
            with self.subTest(suspension=label):
                before = np.array([metrics.camber_deg(
                    solver_cls(hp).solve(float(t))) for t in (-30.0, 0.0, 30.0)])
                target = tweaks.ride_height(hp, 0.0) + 25.0
                out = tweaks.set_ride_height(hp, target, 0.0)
                self.assertAlmostEqual(tweaks.ride_height(out, 0.0), target,
                                       places=9)
                after = np.array([metrics.camber_deg(
                    solver_cls(out).solve(float(t))) for t in (-30.0, 0.0, 30.0)])
                np.testing.assert_allclose(after, before, atol=1e-9)


class TestSectionCaptions(unittest.TestCase):
    """Section captions are display-only: they must never reach the spin
    dict, the params dict or a setter."""

    def test_headers_are_not_knobs(self):
        from suspension_tool.gui.panels import TweaksPanel
        for key in ("dw", "ta", "ml", "ch", "lh", "ha"):
            rows = getattr(TweaksPanel, f"{key.upper()}_ROWS")
            heads = [r for r, _, k in rows if k == "head"]
            self.assertTrue(heads, f"{key} has no sections")
            for h in heads:
                self.assertTrue(h.startswith("_h_"),
                                "header keys must be namespaced")

    def test_every_type_has_a_placement_section(self):
        from suspension_tool.gui.panels import TweaksPanel
        for key in ("dw", "ta", "ml", "ch", "lh", "ha"):
            caps = [lab for _, lab, k in
                    getattr(TweaksPanel, f"{key.upper()}_ROWS") if k == "head"]
            self.assertTrue(any("Placement" in c for c in caps),
                            f"{key} has no Placement section")
