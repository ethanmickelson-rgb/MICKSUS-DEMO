"""Phase 5 verification: the suspension-type interface and rack steer."""

import unittest

import numpy as np

from suspension_tool.metrics import camber_deg, toe_deg
from suspension_tool.seed import IN, SetupVariables, generate_seed
from suspension_tool.solver import DoubleWishboneSolver
from suspension_tool.suspension_types import (SUSPENSION_POINT_TYPES,
                                              SUSPENSION_TYPES, CornerSolver,
                                              MultilinkSolver,
                                              TrailingArmSolver)


def mk4_setup() -> SetupVariables:
    return SetupVariables(
        track_width=63.0 * IN, wheelbase=61.0 * IN, ride_height=14.0 * IN,
        tire_radius=11.5 * IN, tire_width=7.0 * IN,
        shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
        shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85,
        static_camber_deg=-0.87, kickup_deg=10.0)


class TestInterface(unittest.TestCase):
    def test_registry_and_contract(self):
        self.assertEqual(list(SUSPENSION_TYPES),
                         ["Double wishbone", "Trailing arm", "Multilink",
                          "C-hub front", "Loaded halfshaft (rear)",
                          "H-arm (rear)"])
        self.assertIs(SUSPENSION_TYPES["Double wishbone"], DoubleWishboneSolver)
        hp, _ = generate_seed(mk4_setup())
        solver = SUSPENSION_TYPES["Double wishbone"](hp)
        self.assertIsInstance(solver, CornerSolver)
        # the interface methods exist and work through the registry
        st = solver.solve(10.0, steer=0.0)
        self.assertAlmostEqual(st.travel, 10.0, places=6)
        self.assertEqual(len(solver.walk_travels([-10.0, 0.0, 10.0])), 3)

    def test_all_types_registered_and_real(self):
        # Since v0.9 every registered type is a real solver satisfying the
        # CornerSolver contract (no more stubs).
        self.assertIs(SUSPENSION_TYPES["Trailing arm"], TrailingArmSolver)
        self.assertIs(SUSPENSION_TYPES["Multilink"], MultilinkSolver)
        for name, cls in SUSPENSION_TYPES.items():
            self.assertTrue(issubclass(cls, CornerSolver), name)
        # each non-DW type declares its own hardpoint dataclass
        self.assertIsNotNone(SUSPENSION_POINT_TYPES["Trailing arm"])
        self.assertIsNotNone(SUSPENSION_POINT_TYPES["Multilink"])
        self.assertIsNotNone(SUSPENSION_POINT_TYPES["C-hub front"])
        self.assertIsNotNone(SUSPENSION_POINT_TYPES["Loaded halfshaft (rear)"])
        self.assertIsNotNone(SUSPENSION_POINT_TYPES["H-arm (rear)"])


class TestRackSteer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hp, cls.report = generate_seed(mk4_setup())
        cls.solver = DoubleWishboneSolver(cls.hp)

    def test_zero_steer_matches_original(self):
        for t in (-50.0, 0.0, 120.0):
            a = self.solver.solve(t)
            b = self.solver.solve(t, steer=0.0)
            np.testing.assert_allclose(a.tro, b.tro, atol=1e-9)
            np.testing.assert_allclose(a.wheel_center, b.wheel_center, atol=1e-9)

    def test_steer_steers_the_wheel(self):
        toe0 = toe_deg(self.solver.solve(0.0))
        left = toe_deg(self.solver.solve(0.0, steer=+15.0))
        right = toe_deg(self.solver.solve(0.0, steer=-15.0))
        # opposite rack directions steer opposite ways, by a real amount
        self.assertGreater(abs(left - toe0), 1.0)
        self.assertLess((left - toe0) * (right - toe0), 0.0)

    def test_steer_holds_wheel_height_and_link_lengths(self):
        st = self.solver.solve(60.0, steer=20.0)
        self.assertAlmostEqual(st.travel, 60.0, places=6)
        self.assertEqual(st.steer, 20.0)
        # the tie rod is rigid: length from the DISPLACED rack point
        rack = self.hp.tierod_inner + np.array([0.0, 20.0, 0.0])
        self.assertAlmostEqual(
            np.linalg.norm(st.tro - rack),
            np.linalg.norm(self.hp.tierod_outer - self.hp.tierod_inner),
            places=7)

    def test_caster_induced_camber_with_steer(self):
        # With ~4 deg caster, steering must change camber a little — the
        # classic caster-camber coupling; sanity-check it is nonzero.
        c0 = camber_deg(self.solver.solve(0.0))
        c1 = camber_deg(self.solver.solve(0.0, steer=25.0))
        self.assertGreater(abs(c1 - c0), 0.05)

    def test_walk_travels_with_steer(self):
        travels = np.linspace(-60.0, 120.0, 7)
        states = self.solver.walk_travels(travels, steer=10.0)
        for t, st in zip(travels, states):
            self.assertAlmostEqual(st.travel, t, places=5)
            self.assertEqual(st.steer, 10.0)


if __name__ == "__main__":
    unittest.main()
