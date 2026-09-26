"""Halfshaft CV-angle / plunge checks, hand-verified.

v1.12 model: the OUTER CV sits ON the TIRE'S CENTRE (spin) axis — the
point of the wheel-spin axis nearest the kingpin (LBJ->UBJ). This keeps
the CV output line (CV -> tire centre) COINCIDENT with the tire
centreline; a CV cocked at an angle to the wheel is buildable but out of
scope. It still slides with the hub/wheel via tweaks.set_hub_along_kingpin
(a shift along the kingpin leaves the on-axis point's kingpin parameter
unchanged, so the CV moves rigidly with the wheel). Kingpin-less types
fall back to the wheel centre.

(v1.8-1.11 put the CV on the KINGPIN axis instead, which left the CV
output line at a few-degree angle to the tire centreline — the defect
this model fixes.)
"""

import unittest

import numpy as np

from suspension_tool.geometry import example_baja_front, static_spindle
from suspension_tool.halfshaft import (HalfshaftConfig, default_inner_for,
                                       halfshaft_curves, halfshaft_state,
                                       halfshaft_warnings, inner_is_stale,
                                       outer_cv_of_hp, outer_cv_of_state,
                                       static_shaft_length)
from suspension_tool.solver import DoubleWishboneSolver
from suspension_tool.tweaks import hub_along_kingpin, set_hub_along_kingpin


class TestOuterCv(unittest.TestCase):
    def setUp(self):
        self.hp = example_baja_front()
        self.solver = DoubleWishboneSolver(self.hp)

    def test_outer_is_on_tire_axis_nearest_kingpin(self):
        o = outer_cv_of_hp(self.hp)
        wc = self.hp.wheel_center
        s = static_spindle(self.hp)
        # ON the tire's centre (spin) axis: the CV output line is
        # coincident with the tire centreline (zero angle) — the physical
        # requirement (a CV cocked to the wheel is out of scope)
        self.assertLess(np.linalg.norm(np.cross(o - wc, s)), 1e-9)
        # ...and as close to the kingpin as the spin axis ever gets:
        # distance(o, kingpin line) == min line-line distance
        a, b = self.hp.lca_outer, self.hp.uca_outer
        d = (b - a) / np.linalg.norm(b - a)
        d_o = np.linalg.norm(np.cross(o - a, d))
        n = np.cross(d, s)
        gap = abs(np.dot(wc - a, n / np.linalg.norm(n)))
        self.assertAlmostEqual(d_o, gap, places=9)

    def test_cv_output_line_coincides_with_tire_centreline(self):
        # THE fix: CV -> tire-centre stays coincident with the tire
        # centreline (spin axis) at ride AND through steer + travel, since
        # both the CV point and the spindle swing rigidly with the knuckle.
        for travel, steer in [(0.0, 0.0), (40.0, 0.0), (0.0, 20.0),
                              (-50.0, -15.0)]:
            st = self.solver.solve(travel, steer)
            v = st.wheel_center - outer_cv_of_state(st)
            sin_line = (np.linalg.norm(np.cross(v, st.spindle))
                        / (np.linalg.norm(v) * np.linalg.norm(st.spindle)))
            self.assertLess(sin_line, 1e-9,
                            f"CV off tire axis at travel={travel} steer={steer}")

    def test_state_outer_matches_static_at_ride(self):
        st = self.solver.solve(0.0)
        np.testing.assert_allclose(outer_cv_of_state(st),
                                   outer_cv_of_hp(self.hp), atol=1e-6)

    def test_outer_rides_with_upright_through_travel(self):
        # the outer CV is upright-fixed: its distances to LBJ/UBJ are
        # invariant through travel
        o0 = outer_cv_of_hp(self.hp)
        d_l0 = np.linalg.norm(o0 - self.hp.lca_outer)
        d_u0 = np.linalg.norm(o0 - self.hp.uca_outer)
        for t in (-60.0, 80.0, 150.0):
            st = self.solver.solve(t)
            o = outer_cv_of_state(st)
            self.assertAlmostEqual(np.linalg.norm(o - st.lbj), d_l0, places=6)
            self.assertAlmostEqual(np.linalg.norm(o - st.ubj), d_u0, places=6)

    def test_kingpinless_type_falls_back_to_wheel_center(self):
        class Fake:
            wheel_center = np.array([1.0, 2.0, 3.0])
        np.testing.assert_allclose(outer_cv_of_hp(Fake()), [1, 2, 3])

    def test_hub_tweak_slides_outer_and_wheel_together(self):
        hp2 = set_hub_along_kingpin(self.hp, hub_along_kingpin(self.hp) + 30.0)
        a, b = self.hp.lca_outer, self.hp.uca_outer
        d = (b - a) / np.linalg.norm(b - a)
        np.testing.assert_allclose(hp2.wheel_center,
                                   self.hp.wheel_center + 30.0 * d,
                                   atol=1e-9)
        np.testing.assert_allclose(outer_cv_of_hp(hp2),
                                   outer_cv_of_hp(self.hp) + 30.0 * d,
                                   atol=1e-9)
        # tire height changed (the user-visible effect)
        self.assertGreater(abs(hp2.wheel_center[2]
                               - self.hp.wheel_center[2]), 5.0)


class TestHalfshaftMath(unittest.TestCase):
    def setUp(self):
        self.hp = example_baja_front()
        self.solver = DoubleWishboneSolver(self.hp)
        self.outer0 = outer_cv_of_hp(self.hp)

    def test_static_level_shaft_hand_case(self):
        # Inner joint level with and straight inboard of the OUTER CV:
        # the shaft is pure lateral -> inner CV angle ~0, plunge 0.
        cfg = HalfshaftConfig(enabled=True,
                              inner=self.outer0 - np.array([0., 500., 0.]))
        self.assertAlmostEqual(static_shaft_length(cfg, self.hp), 500.0,
                               places=9)
        out = halfshaft_state(cfg, self.solver.solve(0.0), 500.0)
        self.assertAlmostEqual(out["cv_inner_deg"], 0.0, places=6)
        self.assertAlmostEqual(out["plunge_mm"], 0.0, places=6)

    def test_dropped_inner_gives_hand_computed_angle(self):
        cfg = HalfshaftConfig(enabled=True,
                              inner=self.outer0
                              - np.array([0.0, 500.0, 292.0]))
        out = halfshaft_state(cfg, self.solver.solve(0.0),
                              static_shaft_length(cfg, self.hp))
        self.assertAlmostEqual(out["cv_inner_deg"],
                               np.degrees(np.arctan2(292.0, 500.0)),
                               places=6)

    def test_curves_and_warnings(self):
        cfg = HalfshaftConfig(enabled=True,
                              inner=self.outer0 - np.array([0., 480., 0.]))
        states = self.solver.walk_travels(np.linspace(-70.0, 70.0, 15))
        curves = halfshaft_curves(cfg, self.hp, states)
        np.testing.assert_allclose(
            curves["cv_max_deg"],
            np.maximum(curves["cv_inner_deg"], curves["cv_outer_deg"]))
        worst_inner = float(np.max(curves["cv_inner_deg"]))
        worst_outer = float(np.max(curves["cv_outer_deg"]))
        self.assertGreater(max(worst_inner, worst_outer), 6.0)
        # trip both angle limits (plunge budget generous so it stays clear)
        cfg.max_cv_inner_deg = worst_inner - 0.5
        cfg.max_cv_outer_deg = worst_outer - 0.5
        cfg.plunge_total_mm = 1e6
        self.assertEqual(len(halfshaft_warnings(cfg, curves)), 2)
        cfg.max_cv_inner_deg = worst_inner + 1.0
        cfg.max_cv_outer_deg = worst_outer + 1.0
        self.assertEqual(halfshaft_warnings(cfg, curves), [])

    def test_plunge_split_directions_and_fixed_joint(self):
        """Plunge-in (compression) and pull-out (extension) are checked
        against their own allowances from the total stroke and the split
        %; a fixed (Rzeppa) joint can't plunge, so any length change warns
        regardless of the budget."""
        # synthetic sweep: shaft compresses to -20 and extends to +8 mm
        curves = {"cv_inner_deg": np.array([5.0, 5.0]),
                  "cv_outer_deg": np.array([5.0, 5.0]),
                  "plunge_mm": np.array([-20.0, 8.0])}
        cfg = HalfshaftConfig(enabled=True, inner_joint="plunging_cv",
                              max_cv_inner_deg=90.0, max_cv_outer_deg=90.0,
                              plunge_total_mm=40.0, plunge_in_pct=75.0)
        # allow: in = 30, out = 10  ->  -20<30 and 8<10  ->  clear
        self.assertEqual(halfshaft_warnings(cfg, curves), [])
        # tighten total so BOTH directions trip (in=15<20, out=5<8)
        cfg.plunge_total_mm = 20.0
        warns = halfshaft_warnings(cfg, curves)
        self.assertTrue(any("plunge-in" in w for w in warns))
        self.assertTrue(any("pull-out" in w for w in warns))
        # bias the split so only pull-out trips (in=36>20 ok, out=4<8)
        cfg.plunge_total_mm, cfg.plunge_in_pct = 40.0, 90.0
        warns = halfshaft_warnings(cfg, curves)
        self.assertTrue(any("pull-out" in w for w in warns))
        self.assertFalse(any("plunge-in" in w for w in warns))
        # a fixed Rzeppa joint: any real length change warns, budget ignored
        cfg.inner_joint = "rzeppa"
        self.assertFalse(cfg.plunges)
        warns = halfshaft_warnings(cfg, curves)
        self.assertEqual(len(warns), 1)
        self.assertIn("FIXED", warns[0])

    def test_separate_inner_outer_limits(self):
        """The two joints are checked against their OWN limits — a slack
        outer limit must not hide an inner-joint violation, and vice-versa;
        the warning names which joint."""
        cfg = HalfshaftConfig(enabled=True,
                              inner=self.outer0 - np.array([0., 480., 0.]))
        states = self.solver.walk_travels(np.linspace(-70.0, 70.0, 15))
        curves = halfshaft_curves(cfg, self.hp, states)
        wi = float(np.max(curves["cv_inner_deg"]))
        wo = float(np.max(curves["cv_outer_deg"]))
        # inner over its limit, outer well under its (generous) limit
        cfg.max_cv_inner_deg = wi - 0.5
        cfg.max_cv_outer_deg = wo + 20.0
        cfg.plunge_total_mm = 1e6
        warns = halfshaft_warnings(cfg, curves)
        self.assertEqual(len(warns), 1)
        self.assertIn("inner", warns[0])
        # now only the outer trips
        cfg.max_cv_inner_deg = wi + 20.0
        cfg.max_cv_outer_deg = wo - 0.5
        warns = halfshaft_warnings(cfg, curves)
        self.assertEqual(len(warns), 1)
        self.assertIn("outer", warns[0])

    def test_default_inner_and_stale_detection(self):
        inner = default_inner_for(self.hp, bump=268.0, droop=107.0)
        # at the axle station, level at mid-travel
        self.assertAlmostEqual(inner[0], self.hp.wheel_center[0])
        self.assertAlmostEqual(inner[2],
                               self.hp.wheel_center[2] + (268 - 107) / 2)
        good = HalfshaftConfig(enabled=True, inner=inner)
        self.assertFalse(inner_is_stale(good, self.hp))
        # the pre-offset class default a station away IS stale
        import dataclasses
        shifted = dataclasses.replace(
            self.hp, wheel_center=self.hp.wheel_center
            + np.array([990.6, 0.0, 0.0]))
        stale = HalfshaftConfig()          # inner (0, 150, 0)
        self.assertTrue(inner_is_stale(stale, shifted))

    def test_round_trip_dict(self):
        cfg = HalfshaftConfig(enabled=True,
                              inner=np.array([1.0, 2.0, 3.0]),
                              inner_joint="double_cardan",
                              max_cv_inner_deg=22.0, max_cv_outer_deg=46.0,
                              plunge_total_mm=60.0, plunge_in_pct=70.0)
        back = HalfshaftConfig.from_dict(cfg.to_dict())
        self.assertTrue(back.enabled)
        np.testing.assert_allclose(back.inner, [1.0, 2.0, 3.0])
        self.assertEqual(back.inner_joint, "double_cardan")
        self.assertEqual(back.max_cv_inner_deg, 22.0)
        self.assertEqual(back.max_cv_outer_deg, 46.0)
        self.assertEqual(back.plunge_total_mm, 60.0)
        self.assertEqual(back.plunge_in_pct, 70.0)
        self.assertEqual(HalfshaftConfig.from_dict(None).enabled, False)

    def test_legacy_migrations(self):
        """An old .MICK with a single max_cv_deg migrates onto BOTH joint
        limits, and a symmetric max_plunge_mm migrates to total = 2x with a
        50/50 split — behaviour-preserving, so old files load unchanged."""
        legacy = {"enabled": True, "inner": [0.0, 150.0, 0.0],
                  "max_cv_deg": 33.0, "max_plunge_mm": 25.0}
        cfg = HalfshaftConfig.from_dict(legacy)
        self.assertEqual(cfg.max_cv_inner_deg, 33.0)
        self.assertEqual(cfg.max_cv_outer_deg, 33.0)
        self.assertEqual(cfg.plunge_total_mm, 50.0)   # 2 x 25
        self.assertEqual(cfg.plunge_in_pct, 50.0)
        self.assertEqual(cfg.inner_joint, "plunging_cv")
        # the ±25 symmetric budget is reproduced: in = out = 25 mm
        self.assertEqual(cfg.plunge_allow(), (25.0, 25.0))


if __name__ == "__main__":
    unittest.main()
