"""Phase 4 verification: the goal-driven hardpoint optimizer.

Strategy: create a known defect (or a reachable target), let the optimizer
fix it with a limited free set, and confirm the result — plus confirm the
hard invariants survive optimization (bushing axes stay normal to the
sketch plane, packaging bounds respected, geometry always articulates).
"""

import unittest

import numpy as np

from suspension_tool.metrics import camber_deg, toe_deg
from suspension_tool.optimize import FREE_GROUPS, Goal, optimize, _apply, _pack
from suspension_tool.seed import (IN, SetupVariables, generate_seed,
                                  sketch_plane_normal)
from suspension_tool.solver import DoubleWishboneSolver


def mk4_setup() -> SetupVariables:
    return SetupVariables(
        track_width=63.0 * IN, wheelbase=61.0 * IN, ride_height=14.0 * IN,
        tire_radius=11.5 * IN, tire_width=7.0 * IN,
        shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
        shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85,
        static_camber_deg=-0.87, kickup_deg=10.0)


def bump_steer_of(hp, h=1.0):
    from suspension_tool.metrics import toe_deg
    s = DoubleWishboneSolver(hp)
    return (toe_deg(s.solve(h)) - toe_deg(s.solve(-h))) / (2.0 * h)


class TestWalkTravels(unittest.TestCase):
    def test_walk_matches_global_solve(self):
        hp, rep = generate_seed(mk4_setup())
        travels = np.linspace(-rep.droop_travel, rep.bump_travel, 15)
        walk = DoubleWishboneSolver(hp).walk_travels(travels)
        ref = DoubleWishboneSolver(hp)
        for t, st in zip(travels, walk):
            np.testing.assert_allclose(st.wheel_center, ref.solve(t).wheel_center,
                                       atol=1e-6)


class TestOptimizer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hp, cls.report = generate_seed(mk4_setup())

    def test_restores_zero_bump_steer(self):
        # Break the tie rod on purpose (raise the inner end 15 mm), then let
        # the optimizer fix it with ONLY the tie-rod inner free.
        import dataclasses
        bad = dataclasses.replace(
            self.hp, tierod_inner=self.hp.tierod_inner + np.array([0, 0, 15.0]))
        self.assertGreater(abs(bump_steer_of(bad)), 0.01)  # defect exists

        result = optimize(bad, [Goal("bump_steer", weight=1.0)],
                          ["tierod_inner"], np.linspace(-80, 80, 9),
                          box=40.0, max_nfev=80)
        self.assertTrue(result.success, result.message)
        self.assertLess(abs(bump_steer_of(result.hp_after)), 0.002)

    def test_hits_camber_gain_target(self):
        # Ask for a noticeably different camber gain and check it is
        # approached with the upper arm free.
        target = -0.055  # deg/mm; seed is ~-0.037 near static
        result = optimize(self.hp, [Goal("camber_gain", target=target)],
                          ["uca_inner", "uca_outer"],
                          np.linspace(-60, 60, 9), box=60.0, max_nfev=120)
        self.assertTrue(result.success, result.message)
        s = DoubleWishboneSolver(result.hp_after)
        h = 25.0
        gain = (camber_deg(s.solve(h)) - camber_deg(s.solve(-h))) / (2 * h)
        self.assertAlmostEqual(gain, target, delta=abs(target) * 0.2)

    def test_passive_rear_steer_toe_slope(self):
        # Ask for toe that changes -0.005 deg/mm through travel (toe-OUT
        # in bump, IN in droop) with only the tie-rod inner free.
        target = -0.005
        result = optimize(self.hp, [Goal("toe_slope", target=target)],
                          ["tierod_inner"], np.linspace(-60, 60, 9),
                          box=50.0, max_nfev=120)
        self.assertTrue(result.success, result.message)
        s = DoubleWishboneSolver(result.hp_after)
        travels = np.linspace(-60, 60, 9)
        toes = [toe_deg(st) for st in s.walk_travels(travels)]
        slope = float(np.polyfit(travels, toes, 1)[0])
        self.assertAlmostEqual(slope, target, delta=abs(target) * 0.2)

    def test_invariants_survive_optimization(self):
        # Whatever the optimizer does, the bushing axes must stay normal to
        # the sketch plane (parallel, correct spread) and packaging bounds
        # must hold — the parameterisation guarantees it; verify anyway.
        box = 50.0
        result = optimize(self.hp,
                          [Goal("camber_gain", target=-0.05),
                           Goal("bump_steer", weight=0.5)],
                          ["uca_inner", "uca_outer", "tierod_inner"],
                          np.linspace(-60, 60, 7), box=box, max_nfev=60)
        hp2 = result.hp_after
        n = sketch_plane_normal(10.0)
        for f, r in [(hp2.uca_inner_front, hp2.uca_inner_rear),
                     (hp2.lca_inner_front, hp2.lca_inner_rear)]:
            axis = (f - r) / np.linalg.norm(f - r)
            self.assertAlmostEqual(abs(float(np.dot(axis, n))), 1.0, places=9)
        # spread preserved exactly
        self.assertAlmostEqual(
            np.linalg.norm(hp2.uca_inner_front - hp2.uca_inner_rear),
            np.linalg.norm(self.hp.uca_inner_front - self.hp.uca_inner_rear),
            places=9)
        # moved coordinates stayed within the search box
        before, after = _pack(self.hp, ["uca_inner", "uca_outer", "tierod_inner"]), \
            _pack(hp2, ["uca_inner", "uca_outer", "tierod_inner"])
        self.assertTrue(np.all(np.abs(after - before) <= box + 1e-6))
        # and the result still articulates the evaluated range
        DoubleWishboneSolver(hp2).walk_travels(np.linspace(-60, 60, 7))

    def test_impossible_target_returns_safely(self):
        # A roll-centre target the tie rod cannot influence: must not crash,
        # must hand back solvable geometry, and must not make things worse.
        result = optimize(self.hp, [Goal("rc_height", target=1000.0)],
                          ["tierod_outer"], np.linspace(-50, 50, 5),
                          box=30.0, max_nfev=30)
        DoubleWishboneSolver(result.hp_after).walk_travels(
            np.linspace(-50, 50, 5))
        self.assertLessEqual(result.cost_after, result.cost_before + 1e-9)

    def test_pack_apply_round_trip(self):
        free = list(FREE_GROUPS)
        vals = _pack(self.hp, free)
        hp2 = _apply(self.hp, free, vals)
        for attr in ("uca_inner_front", "uca_inner_rear", "lca_inner_front",
                     "lca_inner_rear", "uca_outer", "lca_outer",
                     "tierod_inner", "tierod_outer", "shock_inner",
                     "shock_outer"):
            np.testing.assert_allclose(getattr(hp2, attr), getattr(self.hp, attr),
                                       atol=1e-9)

    def test_input_validation(self):
        with self.assertRaises(ValueError):
            optimize(self.hp, [], ["uca_inner"], [0.0, 10.0])
        with self.assertRaises(ValueError):
            optimize(self.hp, [Goal("bump_steer")], [], [0.0, 10.0])
        with self.assertRaises(ValueError):
            optimize(self.hp, [Goal("bump_steer")], ["nope"], [0.0, 10.0])


if __name__ == "__main__":
    unittest.main()
