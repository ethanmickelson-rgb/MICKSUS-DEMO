"""Verification for the roll / pitch / steering sweeps."""

import unittest

import numpy as np

from suspension_tool.seed import IN, SetupVariables, generate_seed
from suspension_tool.solver import DoubleWishboneSolver
from suspension_tool.sweeps import pitch_sweep, roll_sweep, steer_sweep


def mk4_setup() -> SetupVariables:
    return SetupVariables(
        track_width=63.0 * IN, wheelbase=61.0 * IN, ride_height=14.0 * IN,
        tire_radius=11.5 * IN, tire_width=7.0 * IN,
        shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
        shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85,
        static_camber_deg=-0.87, kickup_deg=10.0)


class TestRollSweep(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hp, _ = generate_seed(mk4_setup())
        cls.solver = DoubleWishboneSolver(cls.hp)
        cls.track = 63.0 * IN
        cls.res = roll_sweep(cls.solver, cls.track, 80.0, n=17)

    def test_zero_roll_matches_static(self):
        k = 8   # centre point of 17
        self.assertAlmostEqual(self.res["roll_deg"][k], 0.0, places=9)
        self.assertAlmostEqual(self.res["camber_left_deg"][k], -0.87, places=3)
        # symmetric car at zero roll: left/right identical, RC on centreline
        self.assertAlmostEqual(self.res["camber_left_deg"][k],
                               self.res["camber_right_deg"][k], places=6)
        self.assertAlmostEqual(self.res["rc_y_mm"][k], 0.0, places=3)

    def test_roll_angle_hand_formula(self):
        # phi = atan(2t / track)
        t = self.res["travel_left_mm"][-1]
        self.assertAlmostEqual(self.res["roll_deg"][-1],
                               np.degrees(np.arctan2(2 * t, self.track)),
                               places=9)

    def test_left_right_symmetry(self):
        # rolling the other way must swap the two wheels' curves
        np.testing.assert_allclose(self.res["camber_left_deg"],
                                   self.res["camber_right_deg"][::-1],
                                   atol=1e-6)
        np.testing.assert_allclose(self.res["rc_y_mm"],
                                   -self.res["rc_y_mm"][::-1], atol=1e-3)

    def test_outside_wheel_loses_negative_camber_in_roll(self):
        # positive roll = leaning onto the LEFT wheels (left = outside).
        # Ground camber of the loaded outside wheel should move POSITIVE
        # (toward upright/positive) relative to static — the classic
        # camber loss the roll sweep exists to expose.
        k0, k1 = 8, -1
        self.assertGreater(self.res["camber_left_deg"][k1],
                           self.res["camber_left_deg"][k0])

    def test_rc_migrates_laterally_in_roll(self):
        self.assertGreater(abs(self.res["rc_y_mm"][-1]), 5.0)


class TestPitchSweep(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hp, _ = generate_seed(mk4_setup())
        f, r = DoubleWishboneSolver(cls.hp), DoubleWishboneSolver(cls.hp)
        cls.res = pitch_sweep(f, r, wheelbase=61.0 * IN,
                              cg_behind_front=0.55 * 61.0 * IN,
                              max_pitch_deg=3.0, n=13)

    def test_travel_coupling_hand_formula(self):
        th = np.radians(self.res["pitch_deg"][-1])
        d_f = 0.55 * 61.0 * IN
        d_r = 61.0 * IN - d_f
        self.assertAlmostEqual(self.res["travel_front_mm"][-1],
                               d_f * np.tan(th), places=9)
        self.assertAlmostEqual(self.res["travel_rear_mm"][-1],
                               -d_r * np.tan(th), places=9)

    def test_ground_caster_decreases_nose_down(self):
        # nose-down pitch tips the kingpin forward relative to the ground:
        # ground caster at max nose-down < caster at zero pitch, by roughly
        # the pitch angle itself.
        k0, k1 = 6, -1
        drop = (self.res["caster_front_ground_deg"][k0]
                - self.res["caster_front_ground_deg"][k1])
        self.assertGreater(drop, 0.5 * self.res["pitch_deg"][-1])

    def test_zero_pitch_is_reference(self):
        self.assertAlmostEqual(self.res["wheelbase_change_mm"][6], 0.0,
                               places=9)


class TestSteerSweep(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hp, _ = generate_seed(mk4_setup())
        cls.solver = DoubleWishboneSolver(cls.hp)
        cls.res = steer_sweep(cls.solver, rack_limit=35.0, travel=0.0,
                              track_width=63.0 * IN, wheelbase=61.0 * IN,
                              n=15)

    def test_antisymmetry(self):
        # steering the other way swaps the wheels
        np.testing.assert_allclose(self.res["steer_left_deg"],
                                   -self.res["steer_right_deg"][::-1],
                                   atol=1e-9)

    def test_steer_direction_and_magnitude(self):
        # positive rack (toward the left corner) on a rear-steer linkage
        # must produce a consistent, substantial steer on both wheels
        d_l, d_r = self.res["steer_left_deg"][-1], self.res["steer_right_deg"][-1]
        self.assertGreater(abs(d_l), 5.0)
        self.assertGreater(d_l * d_r, 0.0)   # same direction

    def test_ackermann_is_finite_and_sane_at_full_lock(self):
        a = self.res["ackermann_pct"][-1]
        self.assertTrue(np.isfinite(a))
        self.assertGreater(a, -200.0)
        self.assertLess(a, 300.0)

    def test_turn_diameter_shrinks_with_steer(self):
        d = self.res["turn_diameter_m"]
        self.assertTrue(np.isnan(d[7]))          # straight ahead: undefined
        self.assertGreater(d[-2], d[-1] * 0.5)   # finite, ordered-ish
        self.assertLess(d[-1], 30.0)             # a Baja car, not a truck
        self.assertGreater(d[-1], 2.0)

    def test_caster_camber_coupling_with_steer(self):
        # with ~4 deg caster the outside wheel gains camber with steer
        dc = abs(self.res["camber_left_deg"][-1]
                 - self.res["camber_left_deg"][7])
        self.assertGreater(dc, 0.1)


if __name__ == "__main__":
    unittest.main()
