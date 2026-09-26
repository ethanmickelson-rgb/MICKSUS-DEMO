"""Verification for Phase 5.3: per-type seeds, type-aware project files,
anti-dive/anti-squat percentages, and the report CSV."""

import unittest

import numpy as np

from suspension_tool.multilink import MultilinkSolver, seed_multilink
from suspension_tool.project import (AxleDesign, ProjectState,
                                     dict_to_project, hardpoints_csv,
                                     project_to_dict, report_csv)
from suspension_tool.seed import IN, SetupVariables, generate_seed
from suspension_tool.solver import DoubleWishboneSolver
from suspension_tool.trailing_arm import (TrailingArmPoints,
                                          TrailingArmSolver,
                                          seed_trailing_arm)
from suspension_tool.vehicle import VehicleParams, anti_geometry


def mk4_setup() -> SetupVariables:
    return SetupVariables(
        track_width=63.0 * IN, wheelbase=61.0 * IN, ride_height=14.0 * IN,
        tire_radius=11.5 * IN, tire_width=7.0 * IN,
        shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
        shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85,
        static_camber_deg=-0.87, kickup_deg=10.0)


class TestPerTypeSeeds(unittest.TestCase):
    def test_trailing_arm_seed(self):
        hp = seed_trailing_arm(mk4_setup())
        s = TrailingArmSolver(hp)
        # hits the MR goal and articulates the shock budget
        mr = (s.solve(-1.0).shock_length
              - s.solve(1.0).shock_length) / 2.0      # shock/wheel, < 1
        self.assertAlmostEqual(mr, 1.0 / 1.85, delta=0.02)
        for t in np.linspace(-100.0, 250.0, 8):
            s.solve(t)
        # wheel at half track, tire on the ground
        self.assertAlmostEqual(hp.wheel_center[1], 63.0 * IN / 2, places=6)

    def test_multilink_seed_matches_dw(self):
        sv = mk4_setup()
        hp_ml = seed_multilink(sv)
        hp_dw, _ = generate_seed(sv)
        ml, dw = MultilinkSolver(hp_ml), DoubleWishboneSolver(hp_dw)
        from suspension_tool.metrics import camber_deg
        for t in (-60.0, 80.0):
            self.assertAlmostEqual(camber_deg(ml.solve(t)),
                                   camber_deg(dw.solve(t)), places=5)


class TestTypedProjects(unittest.TestCase):
    def test_mixed_type_project_round_trip(self):
        front, _ = generate_seed(mk4_setup())
        rear = seed_trailing_arm(mk4_setup())
        state = ProjectState(
            front=AxleDesign(front, mk4_setup(), 100.0, 250.0),
            rear=AxleDesign(rear, None, 100.0, 250.0),
            unit_key="in")
        restored = dict_to_project(project_to_dict(state))
        self.assertIsInstance(restored.rear.hardpoints, TrailingArmPoints)
        np.testing.assert_allclose(restored.rear.hardpoints.pivot_inner,
                                   rear.pivot_inner, atol=1e-9)
        np.testing.assert_allclose(restored.front.hardpoints.uca_outer,
                                   front.uca_outer, atol=1e-9)

    def test_typed_csv(self):
        rear = seed_trailing_arm(mk4_setup())
        text = hardpoints_csv(rear)
        self.assertIn("pivot_inner,", text)
        self.assertIn("suspension_type,trailing_arm", text)


class TestAntiGeometry(unittest.TestCase):
    def test_trailing_arm_anti_squat_by_hand(self):
        # Rear trailing arm with a LATERAL pivot at (x=+350, z=320) local
        # and the wheel centre at z=292: the side-view IC is exactly the
        # pivot, so tan(theta) = (320-292)/350 and
        #   %anti-squat = 100 * tan(theta) * L / h.
        rear_hp = TrailingArmPoints(
            pivot_inner=np.array([350.0, 150.0, 320.0]),
            pivot_outer=np.array([350.0, 450.0, 320.0]),
            wheel_center=np.array([0.0, 700.0, 292.0]),
            shock_inner=np.array([150.0, 300.0, 650.0]),
            shock_outer=np.array([120.0, 400.0, 330.0]),
            tire_radius=292.0)
        rear = TrailingArmSolver(rear_hp)
        front_hp, _ = generate_seed(mk4_setup())
        front = DoubleWishboneSolver(front_hp)
        params = VehicleParams(wheelbase=1549.4, cg_height=558.8)
        anti = anti_geometry((front, front.solve(0.0)),
                             (rear, rear.solve(0.0)), params)
        expected = 100.0 * ((320.0 - 292.0) / 350.0) * 1549.4 / 558.8
        self.assertAlmostEqual(anti["anti_squat_rear_pct"], expected,
                               delta=0.01)
        # the front DW (parallel arm axes, kicked wheel path) gives a
        # finite anti-dive via the numeric fallback
        self.assertTrue(np.isfinite(anti["anti_dive_front_pct"]))


class TestReportCsv(unittest.TestCase):
    def test_report_structure(self):
        from suspension_tool.metrics import sweep_metrics
        hp, rep = generate_seed(mk4_setup())
        sweep = sweep_metrics(DoubleWishboneSolver(hp),
                              np.linspace(-50, 50, 5))
        text = report_csv({"front": sweep, "rear": None},
                          {"wheelbase_in": "61.0"}, 25.4, "in")
        self.assertIn("static design table", text)
        self.assertIn("camber_deg,", text)
        self.assertIn("scrub_radius_in", text)      # unit-converted header
        self.assertIn("# --- front sweep ---", text)
        self.assertNotIn("rear sweep", text)
        # 5 sweep rows follow the sweep header
        tail = text.split("# --- front sweep ---")[1].strip().splitlines()
        self.assertEqual(len(tail), 6)   # header + 5 rows


if __name__ == "__main__":
    unittest.main()
