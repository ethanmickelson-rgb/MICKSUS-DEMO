"""v1.16: fixed shock hardware spec + the travel limits it imposes."""

import unittest

import numpy as np

from suspension_tool.seed import IN, SetupVariables, generate_seed
from suspension_tool.shock import ShockSpec, shock_travel_limits
from suspension_tool.solver import DoubleWishboneSolver


def mk4_setup() -> SetupVariables:
    return SetupVariables(
        track_width=63.0 * IN, wheelbase=61.0 * IN, ride_height=14.0 * IN,
        tire_radius=11.5 * IN, tire_width=7.0 * IN,
        shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
        shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85,
        static_camber_deg=-0.87, kickup_deg=10.0)


class TestShockSpec(unittest.TestCase):
    def test_stroke_and_round_trip(self):
        s = ShockSpec(372.9, 576.1)
        self.assertAlmostEqual(s.stroke, 203.2, places=9)
        back = ShockSpec.from_dict(s.to_dict())
        self.assertAlmostEqual(back.min_length, 372.9, places=9)
        self.assertAlmostEqual(back.max_length, 576.1, places=9)
        self.assertIsNone(ShockSpec.from_dict(None))
        self.assertIsNone(ShockSpec.from_dict({}))


class TestShockTravelLimits(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sv = mk4_setup()
        cls.spec = ShockSpec(sv.shock_min_length, sv.shock_max_length)
        hp, cls.report = generate_seed(sv)
        cls.solver = DoubleWishboneSolver(hp)

    def test_limits_land_on_hardware_lengths(self):
        lim = shock_travel_limits(self.solver, self.spec)
        self.assertGreater(lim["bump"], 10.0)
        self.assertGreater(lim["droop"], 10.0)
        if lim["bump_stop"] == "shock":
            l_b = self.solver.solve(lim["bump"]).shock_length
            self.assertAlmostEqual(l_b, self.spec.min_length, delta=1.0)
        if lim["droop_stop"] == "shock":
            l_d = self.solver.solve(-lim["droop"]).shock_length
            self.assertAlmostEqual(l_d, self.spec.max_length, delta=1.0)

    def test_within_limits_shock_stays_inside_hardware(self):
        lim = shock_travel_limits(self.solver, self.spec)
        for t in np.linspace(-lim["droop"] + 0.5, lim["bump"] - 0.5, 21):
            length = self.solver.solve(float(t)).shock_length
            self.assertGreaterEqual(length, self.spec.min_length - 0.5)
            self.assertLessEqual(length, self.spec.max_length + 0.5)

    def test_tiny_stroke_clamps_tight(self):
        at_ride = self.solver.solve(0.0).shock_length
        tight = ShockSpec(at_ride - 10.0, at_ride + 10.0)
        lim = shock_travel_limits(self.solver, tight)
        self.assertLess(lim["bump"], 40.0)
        self.assertLess(lim["droop"], 40.0)
        self.assertEqual(lim["bump_stop"], "shock")
        self.assertEqual(lim["droop_stop"], "shock")


if __name__ == "__main__":
    unittest.main()
