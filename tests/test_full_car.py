"""Tests for the 7-DOF full-car ride model (v1.26.0).

The valuable ones are the physics invariants that a wrong assembly would
break: the wheelbase-filter nulls, the ARB roll stiffness against the number
the dynamics sheet computes independently, and the reduction to the
quarter-car wheel-hop frequencies.
"""

import unittest

import numpy as np

from suspension_tool import dynamics
from suspension_tool.full_car import (FullCar, from_dynamics,
                                      full_car_report,
                                      wheelbase_filter_speeds)

MPH = 17.6


def _symmetric(**kw):
    """A perfectly front/rear- and left/right-symmetric car. Symmetry is
    what makes the wheelbase-filter nulls EXACT, so it isolates the effect."""
    base = dict(sprung_lb=450.0, a_in=31.5, b_in=31.5,
                track_front_in=52.0, track_rear_in=52.0,
                pitch_radius_gyr_in=31.5, roll_radius_gyr_in=18.0,
                unsprung_front_lb=30.0, unsprung_rear_lb=30.0,
                wheel_rate_front_lbin=61.25, wheel_rate_rear_lbin=61.25,
                tire_rate_front_lbin=250.0, tire_rate_rear_lbin=250.0,
                damp_front_lbsin=8.0, damp_rear_lbsin=8.0)
    base.update(kw)
    return FullCar(**base)


class TestWheelbaseFilter(unittest.TestCase):
    """The signature result of a full-car model: the axles are in phase at
    f = n*V/L (so pitch is not excited) and opposed at f = (2n+1)*V/2L (so
    heave is not excited). Exact for a symmetric car."""

    V = 20.0

    def test_pitch_nulls_when_axles_are_in_phase(self):
        car = _symmetric()
        for n in (1, 2, 3):
            f = n * self.V * MPH / car.wheelbase_in
            r = car.response(np.array([f]), self.V)
            self.assertLess(r["pitch_deg_per_in"][0], 1e-9,
                            f"pitch should vanish at f = {n}V/L")

    def test_heave_nulls_when_axles_are_opposed(self):
        car = _symmetric()
        for n in (0, 1, 2):
            f = (2 * n + 1) * self.V * MPH / (2.0 * car.wheelbase_in)
            r = car.response(np.array([f]), self.V)
            self.assertLess(r["heave"][0], 1e-9,
                            f"heave should vanish at f = {2*n+1}V/2L")

    def test_comb_moves_with_speed(self):
        car = _symmetric()
        # the first pitch null is at V/L, so doubling the speed doubles it
        f_slow = self.V * MPH / car.wheelbase_in
        r = car.response(np.array([f_slow]), 2.0 * self.V)
        self.assertGreater(r["pitch_deg_per_in"][0], 1e-6,
                           "at twice the speed that frequency is no longer "
                           "an in-phase point")

    def test_helper_matches_the_nulls(self):
        car = _symmetric()
        f = 3.0
        sp = wheelbase_filter_speeds(car.wheelbase_in, f)
        # at the reported speed, f must be an in-phase (pitch-null) point
        v = sp["pitch_null_mph"][0]
        r = car.response(np.array([f]), v)
        self.assertLess(r["pitch_deg_per_in"][0], 1e-9)
        v2 = sp["bounce_null_mph"][0]
        r2 = car.response(np.array([f]), v2)
        self.assertLess(r2["heave"][0], 1e-9)


class TestSymmetryAndRoll(unittest.TestCase):
    def test_symmetric_road_never_excites_roll(self):
        """Both wheels of an axle see the same input, so a left/right
        symmetric car cannot roll. If this breaks, the sign of a y-term is
        wrong somewhere."""
        car = _symmetric()
        r = car.response(np.logspace(-1, 1.5, 60), 25.0)
        self.assertLess(float(np.max(r["roll_deg_per_in"])), 1e-9)

    def test_arb_roll_stiffness_matches_the_dynamics_sheet(self):
        """CROSS-CHECK. K[1,1] of the assembled matrix is the total roll
        stiffness; `dynamics` computes the bar contributions a completely
        different way."""
        v = dynamics.DynamicsInputs()
        r = dynamics.compute(v)
        car = from_dynamics(v, r)
        _m, _c, k = car.matrices()
        got = k[1, 1] / 12.0 / (180.0 / np.pi)      # lb-in/rad -> lb-ft/deg
        springs = sum(
            kw * t ** 2 / 2.0 / 12.0 / (180.0 / np.pi)
            for kw, t in ((r["wheel_rate_front_lbin"], v.track_front_in),
                          (r["wheel_rate_rear_lbin"], v.track_rear_in)))
        expect = (r["arb_roll_rate_front_lbftdeg"]
                  + r["arb_roll_rate_rear_lbftdeg"] + springs)
        self.assertLess(abs(got - expect) / expect, 1e-9)

    def test_arb_raises_the_roll_mode_only(self):
        soft = _symmetric()
        stiff = _symmetric(arb_wheel_rate_front_lbin=400.0,
                           arb_wheel_rate_rear_lbin=400.0)
        self.assertGreater(stiff.body_modes()["roll"],
                           soft.body_modes()["roll"])
        # bars act differentially, so heave is untouched
        self.assertAlmostEqual(stiff.body_modes()["heave"],
                               soft.body_modes()["heave"], places=9)


class TestModes(unittest.TestCase):
    def test_seven_modes_labelled(self):
        car = _symmetric()
        modes = car.modes()
        self.assertEqual(len(modes), 7)
        labels = [m["label"] for m in modes]
        self.assertIn("heave/bounce", labels)
        self.assertIn("pitch", labels)
        self.assertIn("roll", labels)
        self.assertEqual(sum("wheel hop" in x for x in labels), 4)

    def test_modes_are_sorted(self):
        f = [m["freq_hz"] for m in _symmetric().modes()]
        self.assertEqual(f, sorted(f))

    def test_body_modes_match_the_dynamics_sheet(self):
        """CROSS-CHECK: the coupled eigenproblem must land near the
        decoupled closed forms the dynamics sheet reports."""
        v = dynamics.DynamicsInputs()
        r = dynamics.compute(v)
        body = from_dynamics(v, r).body_modes()
        self.assertLess(abs(body["heave"] - r["bounce_freq_hz"])
                        / r["bounce_freq_hz"], 0.05)
        self.assertLess(abs(body["pitch"] - r["pitch_freq_hz"])
                        / r["pitch_freq_hz"], 0.05)

    def test_wheel_hop_matches_the_quarter_car(self):
        """The full car must reproduce the quarter car's hop frequencies —
        they are the same physics with more bookkeeping."""
        from suspension_tool.ride_freq import from_dynamics as qc_from
        v = dynamics.DynamicsInputs()
        r = dynamics.compute(v)
        car = from_dynamics(v, r)
        hops = sorted(m["freq_hz"] for m in car.modes()
                      if "wheel hop" in m["label"])
        for axle in ("front", "rear"):
            qc_hop = qc_from(v, results=r, axle=axle).wheel_hop_hz
            self.assertTrue(
                any(abs(h - qc_hop) / qc_hop < 0.02 for h in hops),
                f"no full-car mode near the quarter-car {axle} hop "
                f"{qc_hop:.2f} Hz (got {hops})")

    def test_heavier_sprung_mass_lowers_the_body_modes(self):
        light = _symmetric(sprung_lb=300.0).body_modes()["heave"]
        heavy = _symmetric(sprung_lb=700.0).body_modes()["heave"]
        self.assertGreater(light, heavy)


class TestResponse(unittest.TestCase):
    def test_dc_limits(self):
        car = _symmetric()
        r = car.response(np.array([1e-3]), 20.0)
        # at DC the whole car simply rides the road
        self.assertAlmostEqual(float(r["heave"][0]), 1.0, places=4)
        for c in ("FL", "RL"):
            self.assertLess(float(r[f"tire_load_{c}_lb_per_in"][0]), 1e-2)
            self.assertLess(float(r[f"travel_{c}"][0]), 1e-3)

    def test_pitch_vanishes_in_the_dc_limit(self):
        # Pitch does not hit exactly zero at any finite frequency (the rear
        # axle always lags a little), so assert the LIMIT: it keeps shrinking
        # as the frequency falls.
        # It is first order in the axle lag omega*tau, so a 10x lower
        # frequency gives 10x less pitch — assert that scaling law rather
        # than an arbitrary absolute floor.
        car = _symmetric()
        vals = [float(car.response(np.array([f]), 20.0)["pitch_deg_per_in"][0])
                for f in (1e-2, 1e-3, 1e-4)]
        self.assertTrue(vals[0] > vals[1] > vals[2], vals)
        for a, b in zip(vals, vals[1:]):
            self.assertAlmostEqual(a / b, 10.0, delta=0.1)

    def test_all_channels_finite_and_positive(self):
        car = from_dynamics(dynamics.DynamicsInputs())
        r = car.response(np.logspace(-1, 2, 200), 25.0)
        for key, val in r.items():
            if key == "freq_hz":
                continue
            self.assertTrue(np.all(np.isfinite(val)), key)
            self.assertTrue(np.all(val >= 0.0), key)

    def test_axle_accel_differs_from_cg_when_pitching(self):
        car = from_dynamics(dynamics.DynamicsInputs())
        r = car.response(np.logspace(-1, 1, 80), 20.0)
        self.assertFalse(np.allclose(r["accel_front_axle_g"],
                                     r["heave_accel_g"]),
                         "pitch must make the axle stations differ from CG")

    def test_speed_changes_the_response(self):
        car = from_dynamics(dynamics.DynamicsInputs())
        f = np.logspace(-1, 1, 100)
        a = car.response(f, 10.0)["pitch_deg_per_in"]
        b = car.response(f, 40.0)["pitch_deg_per_in"]
        self.assertFalse(np.allclose(a, b))


class TestReport(unittest.TestCase):
    def test_report_is_complete(self):
        rep = full_car_report(dynamics.DynamicsInputs(), speed_mph=25.0)
        self.assertEqual(len(rep.modes), 7)
        self.assertEqual(rep.speed_mph, 25.0)
        self.assertTrue(rep.notes)
        self.assertIn("heave", rep.response)
        self.assertIsNotNone(rep.mode("pitch"))

    def test_inertia_estimate_is_disclosed(self):
        rep = full_car_report(dynamics.DynamicsInputs())
        self.assertIn("estimate", " ".join(rep.notes).lower())

    def test_roll_gyration_input_is_used(self):
        v = dynamics.DynamicsInputs()
        v.roll_radius_gyr_in = 12.0
        a = from_dynamics(v).body_modes()["roll"]
        v.roll_radius_gyr_in = 30.0
        b = from_dynamics(v).body_modes()["roll"]
        self.assertGreater(a, b)      # more inertia -> lower frequency


if __name__ == "__main__":
    unittest.main()
