"""Verification for v1.1: multilink virtual kingpin, per-type tweaks +
optimization, and the bump-steer map."""

import dataclasses
import unittest

import numpy as np

from suspension_tool.metrics import camber_deg, corner_metrics, toe_deg
from suspension_tool.multilink import (MultilinkSolver, from_double_wishbone,
                                       seed_multilink)
from suspension_tool.optimize import Goal, optimize
from suspension_tool.seed import IN, SetupVariables, generate_seed
from suspension_tool.solver import DoubleWishboneSolver
from suspension_tool.sweeps import bump_steer_map
from suspension_tool.trailing_arm import TrailingArmSolver, seed_trailing_arm
from suspension_tool.tweaks import (multilink_toe_params, set_multilink_toe,
                                    set_trailing_arm, trailing_arm_params)


def mk4_setup() -> SetupVariables:
    return SetupVariables(
        track_width=63.0 * IN, wheelbase=61.0 * IN, ride_height=14.0 * IN,
        tire_radius=11.5 * IN, tire_width=7.0 * IN,
        shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
        shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85,
        static_camber_deg=-0.87, kickup_deg=10.0)


class TestVirtualKingpin(unittest.TestCase):
    """The virtual-ball-joint axis of a multilink must reproduce the
    physical kingpin of the equivalent double wishbone EXACTLY (the link
    pairs intersect at the real ball joints)."""

    def test_matches_dw_kingpin_metrics(self):
        hp, _ = generate_seed(mk4_setup())
        dw = DoubleWishboneSolver(hp)
        ml = MultilinkSolver(from_double_wishbone(hp))
        for t in (0.0, 100.0, -80.0):
            ma = corner_metrics(dw, dw.solve(t))
            mb = corner_metrics(ml, ml.solve(t))
            for key in ("caster_deg", "kpi_deg", "scrub_radius_mm",
                        "caster_trail_mm"):
                self.assertAlmostEqual(mb[key], ma[key], places=6, msg=key)

    def test_general_multilink_kingpin_finite(self):
        # a real (non-DW) multilink: perturb link ends so the pairs no
        # longer intersect; the virtual axis must still exist and give
        # finite kingpin metrics
        hp = from_double_wishbone(generate_seed(mk4_setup())[0])
        hp = dataclasses.replace(
            hp,
            link1_outer=hp.link1_outer + np.array([15.0, 0.0, 10.0]),
            link3_outer=hp.link3_outer + np.array([-12.0, 0.0, 8.0]))
        ml = MultilinkSolver(hp)
        m = corner_metrics(ml, ml.solve(0.0))
        for key in ("caster_deg", "kpi_deg", "scrub_radius_mm"):
            self.assertTrue(np.isfinite(m[key]), key)


class TestPerTypeTweaks(unittest.TestCase):
    def test_trailing_arm_round_trip_and_skew(self):
        hp = seed_trailing_arm(mk4_setup())
        p = trailing_arm_params(hp)
        hp2 = set_trailing_arm(hp, **p)
        np.testing.assert_allclose(hp2.pivot_outer, hp.pivot_outer, atol=1e-9)
        np.testing.assert_allclose(hp2.shock_outer, hp.shock_outer, atol=1e-6)
        # the seed's lateral axis has no camber gain; skewing the plan
        # angle (semi-trailing) must create it
        s0 = TrailingArmSolver(hp)
        gain0 = camber_deg(s0.solve(50.0)) - camber_deg(s0.solve(-50.0))
        hp3 = set_trailing_arm(hp, p["d"], p["h"], p["ang"],
                               plan_deg=20.0, elev_deg=0.0)
        q = trailing_arm_params(hp3)
        self.assertAlmostEqual(q["plan_deg"], 20.0, places=6)
        s3 = TrailingArmSolver(hp3)
        gain3 = camber_deg(s3.solve(50.0)) - camber_deg(s3.solve(-50.0))
        self.assertGreater(abs(gain3), abs(gain0) + 0.5)

    def test_multilink_toe_round_trip_and_bump_steer(self):
        hp = seed_multilink(mk4_setup())
        p = multilink_toe_params(hp)
        hp2 = set_multilink_toe(hp, **p)
        np.testing.assert_allclose(hp2.link5_inner, hp.link5_inner, atol=1e-9)

        def bs(h):
            s = MultilinkSolver(h)
            return (toe_deg(s.solve(1.0)) - toe_deg(s.solve(-1.0))) / 2.0
        self.assertLess(abs(bs(hp)), 0.001)          # DW-equivalent seed
        hp3 = set_multilink_toe(hp, p["outer_rise"] + 20.0, p["inner_rise"])
        self.assertGreater(abs(bs(hp3)), 0.005)      # de-tuned as expected


class TestPerTypeOptimizer(unittest.TestCase):
    def test_trailing_arm_camber_gain(self):
        hp = seed_trailing_arm(mk4_setup())
        result = optimize(hp, [Goal("camber_gain", target=-0.03)],
                          ["pivot_outer"], np.linspace(-60, 60, 7),
                          box=120.0, max_nfev=80)
        self.assertTrue(result.success, result.message)
        s = TrailingArmSolver(result.hp_after)
        gain = (camber_deg(s.solve(25.0)) - camber_deg(s.solve(-25.0))) / 50.0
        self.assertAlmostEqual(gain, -0.03, delta=0.012)

    def test_multilink_bump_steer_repair(self):
        hp = seed_multilink(mk4_setup())
        p = multilink_toe_params(hp)
        bad = set_multilink_toe(hp, p["outer_rise"] + 15.0, p["inner_rise"])
        result = optimize(bad, [Goal("bump_steer")], ["link5_inner"],
                          np.linspace(-70, 70, 7), box=40.0, max_nfev=60)
        self.assertTrue(result.success, result.message)
        s = MultilinkSolver(result.hp_after)
        bs = (toe_deg(s.solve(1.0)) - toe_deg(s.solve(-1.0))) / 2.0
        self.assertLess(abs(bs), 0.003)

    def test_undefined_goal_errors_clearly(self):
        hp = seed_trailing_arm(mk4_setup())
        with self.assertRaises(ValueError) as ctx:
            optimize(hp, [Goal("caster", target=4.0)], ["pivot_outer"],
                     np.linspace(-40, 40, 5))
        self.assertIn("undefined", str(ctx.exception))


class TestBumpSteerMap(unittest.TestCase):
    def test_map_shape_and_zero_rack_column(self):
        hp, _ = generate_seed(mk4_setup())
        solver = DoubleWishboneSolver(hp)
        travels = np.linspace(-80.0, 80.0, 9)
        racks = np.linspace(-30.0, 30.0, 7)
        res = bump_steer_map(solver, travels, racks)
        self.assertEqual(res["toe_deg"].shape, (9, 7))
        self.assertTrue(np.all(np.isfinite(res["toe_deg"])))
        # the zero-rack column must equal the plain toe-vs-travel curve
        j0 = 3
        self.assertAlmostEqual(res["rack_mm"][j0], 0.0, places=9)
        for i, t in enumerate(travels):
            self.assertAlmostEqual(res["toe_deg"][i, j0],
                                   toe_deg(solver.solve(t)), places=6)
        # steering changes the toe-vs-travel curve shape (that is the
        # entire reason the map exists)
        self.assertGreater(
            np.max(np.abs(res["toe_deg"][:, -1] - res["toe_deg"][:, j0])), 0.5)


if __name__ == "__main__":
    unittest.main()
