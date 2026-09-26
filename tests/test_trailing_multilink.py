"""Verification for the trailing-arm and multilink solvers.

Trailing arm: hand-checkable because the motion is one rigid rotation —
a purely lateral pivot axis must give constant camber/toe, a circular
wheel path, a ground-level roll centre, and an exact side-view IC on the
pivot axis. Skewing the axis must create camber/toe change.

Multilink: a double wishbone IS a five-link (two rods per A-arm meeting
at the ball joint + the tie rod), so the multilink solver fed those ten
points must reproduce the double-wishbone solver's camber/toe/wheel-path
curves and its steering response. That cross-validates both solvers.
"""

import unittest

import numpy as np

from suspension_tool.metrics import (camber_deg, corner_metrics,
                                     front_view_ic, roll_center_height_mm,
                                     side_view_ic, toe_deg)
from suspension_tool.multilink import MultilinkPoints, MultilinkSolver
from suspension_tool.seed import IN, SetupVariables, generate_seed
from suspension_tool.solver import DoubleWishboneSolver
from suspension_tool.trailing_arm import TrailingArmPoints, TrailingArmSolver


def _pt(x, y, z):
    return np.array([x, y, z], dtype=float)


def pure_trailing_arm() -> TrailingArmPoints:
    """Pivot axis exactly lateral (along Y): the textbook trailing arm.

    Zero static camber/toe on purpose: the textbook 'no camber/toe change'
    property holds exactly only when the wheel spin axis is PARALLEL to
    the pivot axis. With static alignment dialed in, a real trailing arm
    picks up tiny camber/toe change through travel — the solver models
    that faithfully, so the hand-checks here use the aligned case."""
    return TrailingArmPoints(
        pivot_inner=_pt(350.0, 150.0, 320.0),
        pivot_outer=_pt(350.0, 450.0, 320.0),
        wheel_center=_pt(0.0, 700.0, 292.0),
        shock_inner=_pt(150.0, 300.0, 650.0),
        shock_outer=_pt(120.0, 400.0, 330.0),
        tire_radius=292.0,
    )


def semi_trailing_arm() -> TrailingArmPoints:
    """Axis skewed in plan (classic semi-trailing): camber/toe change."""
    hp = pure_trailing_arm()
    hp.pivot_outer = _pt(250.0, 450.0, 340.0)   # sweep the axis back + up
    return hp


class TestTrailingArm(unittest.TestCase):
    def test_pure_arm_constant_camber_toe(self):
        # Spin axis parallel to the pivot axis -> rotation leaves the
        # spindle invariant -> camber and toe exactly constant.
        s = TrailingArmSolver(pure_trailing_arm())
        for t in np.linspace(-80, 120, 9):
            st = s.solve(t)
            self.assertAlmostEqual(st.travel, t, places=9)
            self.assertAlmostEqual(camber_deg(st), 0.0, places=9)
            self.assertAlmostEqual(toe_deg(st), 0.0, places=9)

    def test_misaligned_spindle_couples_slightly(self):
        # With static camber/toe dialled in, the spindle is NOT parallel to
        # the lateral pivot axis, so a small real camber/toe drift appears
        # through travel — the physics the aligned case hides.
        import dataclasses
        hp = dataclasses.replace(pure_trailing_arm(),
                                 static_camber_deg=-1.0, static_toe_deg=0.2)
        s = TrailingArmSolver(hp)
        # camber is a front-view projection: with toe dialled in it
        # differs from the applied rotation by O(toe^2*camber) ~ 1e-5
        self.assertAlmostEqual(camber_deg(s.solve(0.0)), -1.0, places=4)
        drift = abs(camber_deg(s.solve(100.0)) - camber_deg(s.solve(0.0)))
        self.assertGreater(drift, 0.001)
        self.assertLess(drift, 0.5)

    def test_pure_arm_wheel_path_is_hand_circle(self):
        # In side view the wheel centre rides a circle about the pivot:
        # radius = distance from the axis, centred at (pivot_x, pivot_z).
        hp = pure_trailing_arm()
        s = TrailingArmSolver(hp)
        r0 = np.hypot(hp.wheel_center[0] - 350.0, hp.wheel_center[2] - 320.0)
        for t in (-60.0, 40.0, 110.0):
            wc = s.solve(t).wheel_center
            self.assertAlmostEqual(
                np.hypot(wc[0] - 350.0, wc[2] - 320.0), r0, places=8)
            self.assertAlmostEqual(wc[1], hp.wheel_center[1], places=8)

    def test_pure_arm_roll_center_on_ground_and_ics(self):
        s = TrailingArmSolver(pure_trailing_arm())
        st = s.solve(0.0)
        # lateral axis: no front-view IC (wheel goes straight up in front
        # view) -> roll centre at contact-patch height (the ground)
        self.assertIsNone(front_view_ic(s, st))
        self.assertAlmostEqual(roll_center_height_mm(s, st),
                               st.contact_patch[2], places=6)
        # side-view IC is EXACTLY the pivot (x, z) — pure rotation about it
        svic = side_view_ic(s, st)
        self.assertAlmostEqual(svic[0], 350.0, places=9)
        self.assertAlmostEqual(svic[1], 320.0, places=9)

    def test_semi_trailing_gains_camber_and_toe(self):
        s = TrailingArmSolver(semi_trailing_arm())
        c0, t0 = camber_deg(s.solve(0.0)), toe_deg(s.solve(0.0))
        c1, t1 = camber_deg(s.solve(80.0)), toe_deg(s.solve(80.0))
        self.assertGreater(abs(c1 - c0), 0.5)
        self.assertGreater(abs(t1 - t0), 0.5)
        # and the front-view IC now exists (skewed axis pierces the plane)
        self.assertIsNotNone(front_view_ic(s, s.solve(0.0)))

    def test_kingpin_metrics_nan_and_shock_rigid(self):
        s = TrailingArmSolver(pure_trailing_arm())
        st = s.solve(30.0)
        m = corner_metrics(s, st)
        for key in ("caster_deg", "kpi_deg", "scrub_radius_mm",
                    "caster_trail_mm"):
            self.assertTrue(np.isnan(m[key]), key)
        # the shock outer is rigid with the arm: constant distance to wc
        d0 = np.linalg.norm(s.solve(0.0).shock_outer
                            - s.solve(0.0).wheel_center)
        d1 = np.linalg.norm(st.shock_outer - st.wheel_center)
        self.assertAlmostEqual(d0, d1, places=8)

    def test_out_of_reach_raises(self):
        s = TrailingArmSolver(pure_trailing_arm())
        with self.assertRaises(ValueError):
            s.solve(5000.0)


def dw_as_five_link(hp) -> MultilinkPoints:
    """Express a double-wishbone corner as five rods (the equivalence the
    multilink test leans on)."""
    return MultilinkPoints(
        link1_inner=hp.uca_inner_front, link1_outer=hp.uca_outer,
        link2_inner=hp.uca_inner_rear, link2_outer=hp.uca_outer,
        link3_inner=hp.lca_inner_front, link3_outer=hp.lca_outer,
        link4_inner=hp.lca_inner_rear, link4_outer=hp.lca_outer,
        link5_inner=hp.tierod_inner, link5_outer=hp.tierod_outer,
        wheel_center=hp.wheel_center,
        shock_inner=hp.shock_inner,
        # the DW shock rides the LCA, not the upright — park the multilink
        # shock ON the upright at the LBJ so lengths stay comparable-ish;
        # shock behaviour is not part of the equivalence check.
        shock_outer=hp.lca_outer,
        tire_radius=hp.tire_radius,
        static_camber_deg=hp.static_camber_deg,
        static_toe_deg=hp.static_toe_deg,
    )


class TestMultilinkAgainstDoubleWishbone(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sv = SetupVariables(
            track_width=63.0 * IN, wheelbase=61.0 * IN, ride_height=14.0 * IN,
            tire_radius=11.5 * IN, tire_width=7.0 * IN,
            shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
            shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85,
            static_camber_deg=-0.87, kickup_deg=10.0)
        cls.hp, _ = generate_seed(sv)
        cls.dw = DoubleWishboneSolver(cls.hp)
        cls.ml = MultilinkSolver(dw_as_five_link(cls.hp))

    def test_matches_dw_through_travel(self):
        for t in np.linspace(-90.0, 150.0, 9):
            a = self.dw.solve(t)
            b = self.ml.solve(t)
            np.testing.assert_allclose(b.wheel_center, a.wheel_center,
                                       atol=1e-4)
            self.assertAlmostEqual(camber_deg(b), camber_deg(a), places=5)
            self.assertAlmostEqual(toe_deg(b), toe_deg(a), places=5)

    def test_matches_dw_with_steer(self):
        for s in (-20.0, 12.0):
            a = self.dw.solve(30.0, steer=s)
            b = self.ml.solve(30.0, steer=s)
            self.assertAlmostEqual(toe_deg(b), toe_deg(a), places=5)
            self.assertAlmostEqual(camber_deg(b), camber_deg(a), places=5)

    def test_numeric_roll_center_matches_dw_construction(self):
        # The generic numeric IC path (used for multilink) must agree with
        # the double wishbone's geometric construction on the same corner.
        rc_dw = roll_center_height_mm(self.dw, self.dw.solve(0.0))
        rc_ml = roll_center_height_mm(self.ml, self.ml.solve(0.0))
        # the numeric method finite-differences at h=1mm and projects a
        # not-perfectly-rigid front-view field, so allow <2% on a ~370mm
        # roll-centre height (~7mm)
        self.assertAlmostEqual(rc_ml, rc_dw, delta=7.0)

    def test_walk_travels_warm_start(self):
        travels = np.linspace(-80.0, 140.0, 12)
        states = self.ml.walk_travels(travels)
        for t, st in zip(travels, states):
            self.assertAlmostEqual(st.travel, t, places=6)

    def test_link_lengths_rigid(self):
        st = self.ml.solve(100.0)
        for k in range(1, 5):
            inner = getattr(self.ml.hp, f"link{k}_inner")
            self.assertAlmostEqual(
                np.linalg.norm(st.outer_points[k] - inner),
                self.ml._lengths[k], places=7)


if __name__ == "__main__":
    unittest.main()
