"""Milliken dynamics module vs hand-keyed spreadsheet arithmetic.

Every expected number below was worked outindependently on a calculator from the
formulas in reference/suspension_design_tool_v2.xlsx with its 'Baja SAE'
preset (the DynamicsInputs defaults), so a port bug can't hide behind
the same code computing both sides.
"""

import unittest

import numpy as np

from suspension_tool.dynamics import DynamicsInputs, compute


class TestBajaPreset(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.out = compute(DynamicsInputs())

    # ---- Vehicle sheet ------------------------------------------------
    def test_mass_and_cg(self):
        o = self.out
        self.assertAlmostEqual(o["total_weight_lb"], 580.0)
        self.assertAlmostEqual(o["front_weight_frac"], 260.0 / 580.0)
        # b = WF*L/W = 260*63/580 = 28.2414; a = 63 - b
        self.assertAlmostEqual(o["cg_to_rear_in"], 28.2414, places=3)
        self.assertAlmostEqual(o["cg_to_front_in"], 34.7586, places=3)
        # Ws = 580 - 2*30 - 2*35 = 450
        self.assertAlmostEqual(o["sprung_weight_lb"], 450.0)
        # hs = (580*20 - 60*11.5 - 70*11.5)/450 = 10105/450 = 22.4556
        self.assertAlmostEqual(o["sprung_cg_height_in"], 22.4556, places=3)
        # as = (260-60)/450 = 0.44444
        self.assertAlmostEqual(o["front_sprung_frac"], 0.44444, places=4)
        # zRA = 3 + (5-3)*(1-0.44444) = 4.1111; H = 22.4556 - 4.1111
        self.assertAlmostEqual(o["roll_axis_at_cg_in"], 4.1111, places=3)
        self.assertAlmostEqual(o["roll_moment_arm_in"], 18.3444, places=3)

    # ---- Setup sheet: rates and frequencies ----------------------------
    def test_wheel_and_ride_rates(self):
        o = self.out
        # KwF = 125*0.7^2 = 61.25 ; KwR = 150*0.75^2 = 84.375
        self.assertAlmostEqual(o["wheel_rate_front_lbin"], 61.25)
        self.assertAlmostEqual(o["wheel_rate_rear_lbin"], 84.375)
        # KRF = 61.25*250/311.25 = 49.1968 ; KRR = 84.375*250/334.375
        self.assertAlmostEqual(o["ride_rate_front_lbin"], 49.1968, places=3)
        self.assertAlmostEqual(o["ride_rate_rear_lbin"], 63.0841, places=3)

    def test_ride_frequencies_and_olley(self):
        o = self.out
        # fF = (1/2pi)*sqrt(49.1968*386.4/100) = 2.1946 Hz
        self.assertAlmostEqual(o["ride_freq_front_hz"], 2.1946, places=3)
        # fR = (1/2pi)*sqrt(63.0841*386.4/125) = 2.2229 Hz
        self.assertAlmostEqual(o["ride_freq_rear_hz"], 2.2229, places=3)
        self.assertAlmostEqual(o["olley_ratio"], 2.2229 / 2.1946, places=3)

    def test_dampers(self):
        o = self.out
        # front: C_wheel = 15*0.49 = 7.35 ; Ccr = 2*sqrt(61.25*100/386.4)
        #       = 2*sqrt(15.8514) = 7.9628 ; zeta = 0.9231
        self.assertAlmostEqual(o["damp_wheel_bump_front_lbsin"], 7.35)
        self.assertAlmostEqual(o["damp_critical_front_lbsin"], 7.9628,
                               places=3)
        self.assertAlmostEqual(o["zeta_bump_front"], 0.9231, places=3)
        # rebound front: 20*0.49/7.9628 = 1.2308
        self.assertAlmostEqual(o["zeta_rebound_front"], 1.2308, places=3)

    def test_roll_rates(self):
        o = self.out
        # front ARB: 150*(0.6*(52/12)/0.5)^2 = 150*5.2^2 = 4056 lb-ft/deg
        self.assertAlmostEqual(o["arb_roll_rate_front_lbftdeg"], 4056.0,
                               places=1)
        # rear ARB: 60*(0.65*(50/12)/0.4)^2 = 60*6.7708^2 = 2750.6
        self.assertAlmostEqual(o["arb_roll_rate_rear_lbftdeg"], 2750.65,
                               delta=0.1)
        # front total: A=4056*180/pi=232391.7 ; S=12*61.25*4.3333^2/2=6900.8
        # T=12*250*4.3333^2/2=28166.7 ; (A+S)*T/(A+S+T)/57.2958 = 439.83
        self.assertAlmostEqual(o["roll_rate_front_lbftdeg"], 439.83,
                               delta=0.15)
        # rear total: A=2750.65*180/pi=157600.4 ; S=12*84.375*4.1667^2/2=8789.1
        # T=12*250*4.1667^2/2=26041.7 ; (A+S)*T/(A+S+T)/57.2958 = 393.00
        self.assertAlmostEqual(o["roll_rate_rear_lbftdeg"], 393.00,
                               delta=0.15)

    def test_roll_gradient(self):
        o = self.out
        # -(450*(18.3444/12))/(439.83+393.00) = -687.92/832.83 = -0.8260
        self.assertAlmostEqual(o["roll_gradient_deg_g"], -0.8260, delta=5e-4)

    # ---- Loads sheet ----------------------------------------------------
    def test_lateral_weight_transfer(self):
        # v1.22.1: full Milliken 18.4 — SPRUNG weight (450) in BOTH sprung
        # terms + a separate unsprung-at-wheel-centre reaction. (The old
        # port used total weight 580 and dropped the unsprung term,
        # overstating transfer ~12%.)
        o = self.out
        kf, kr = o["roll_rate_front_lbftdeg"], o["roll_rate_rear_lbftdeg"]
        Ws = 450.0
        expect_f = (1.0 / (52.0 / 12.0)) * (
            Ws * (18.3444 / 12.0) * kf / (kf + kr)
            + Ws * (28.2414 / 63.0) * (3.0 / 12.0)
            + 60.0 * (11.5 / 12.0))
        expect_r = (1.0 / (50.0 / 12.0)) * (
            Ws * (18.3444 / 12.0) * kr / (kf + kr)
            + Ws * (34.7586 / 63.0) * (5.0 / 12.0)
            + 70.0 * (11.5 / 12.0))
        self.assertAlmostEqual(o["lat_transfer_front_lb"], expect_f, delta=0.05)
        self.assertAlmostEqual(o["lat_transfer_rear_lb"], expect_r, delta=0.05)

    def test_lateral_transfer_moment_invariant(self):
        # PHYSICAL PROOF the equation is right: all lateral inertia acts at
        # the mass CG, so summing each axle's transfer × its track must
        # equal Ay × the total ground-plane moment (Ws·hs + Σ Wu·z_wc).
        # (The old total-weight form failed this by ~12%.)
        o = self.out
        lhs = (o["lat_transfer_front_lb"] * (52.0 / 12.0)
               + o["lat_transfer_rear_lb"] * (50.0 / 12.0))
        rhs = 1.0 * (450.0 * (o["sprung_cg_height_in"] / 12.0)
                     + 60.0 * (11.5 / 12.0) + 70.0 * (11.5 / 12.0))
        self.assertAlmostEqual(lhs, rhs, delta=0.5)   # <0.05% (b/L vs a_s)

    def test_banking_sign(self):
        # SAE sign: positive bank = into the turn = LESS tyre demand.
        self.assertAlmostEqual(self.out["ay_banked_g"], 1.0, places=6)  # flat
        banked = compute(DynamicsInputs(bank_deg=15.0))["ay_banked_g"]
        self.assertLess(banked, 1.0)
        self.assertAlmostEqual(banked, np.cos(np.radians(15))
                               - np.sin(np.radians(15)), places=6)

    def test_longitudinal_transfer_and_corner_loads(self):
        o = self.out
        # dWx = 0.7*580*20/63 = 128.889
        self.assertAlmostEqual(o["long_transfer_lb"], 128.889, places=2)
        # corner loads reassemble from their pieces (no aero at CLA=0)
        self.assertAlmostEqual(
            o["load_front_out_lb"],
            130.0 + o["lat_transfer_front_lb"] + 128.889 / 2.0, places=2)
        self.assertAlmostEqual(
            o["load_rear_in_lb"],
            160.0 - o["lat_transfer_rear_lb"] - 128.889 / 2.0, places=2)

    def test_anti_geometry_svsa(self):
        # v1.22.1: squat is drawn from the WHEEL CENTRE (inboard drive),
        # dive/lift from the CONTACT PATCH (outboard brakes) — for the same
        # IC those lines differ by the loaded radius, so the quoted SVSA
        # height is converted per reference instead of reused for both.
        o = self.out
        # h/L = 20/63 ; AS_rear = (3/25)/(20/63)*100 = 37.8  (wheel centre)
        self.assertAlmostEqual(o["anti_squat_rear_pct"], 37.8, places=1)
        # AD_front = 0.6*((2+11.5)/30)/(20/63)*100 = 85.05  (contact patch)
        self.assertAlmostEqual(o["anti_dive_front_pct"], 85.05, places=1)
        # AL_rear = 0.4*((3+11.5)/25)/(20/63)*100 = 73.08
        self.assertAlmostEqual(o["anti_lift_rear_pct"], 73.08, places=1)
        self.assertIn("SVSA approximation", o["anti_source"])
        self.assertIn("wheel centre", o["anti_source"])

    def test_svsa_reference_roundtrip(self):
        # quoting the SAME instant centre from the contact patch (height
        # bigger by one loaded radius) must reproduce the same anti values
        o_cp = compute(DynamicsInputs(
            svsa_ref="contact_patch",
            svsa_height_front_in=2.0 + 11.5, svsa_height_rear_in=3.0 + 11.5))
        for key in ("anti_squat_rear_pct", "anti_dive_front_pct",
                    "anti_lift_rear_pct"):
            self.assertAlmostEqual(o_cp[key], self.out[key], places=6, msg=key)

    def test_kinematic_anti_overrides(self):
        o = compute(DynamicsInputs(), kinematic_anti={
            "anti_squat_rear_pct": 42.0, "anti_dive_front_pct": 11.0,
            "anti_lift_rear_pct": 9.0})
        self.assertEqual(o["anti_squat_rear_pct"], 42.0)
        self.assertEqual(o["anti_source"], "exact kinematic model")

    def test_brake_chain(self):
        o = self.out
        # F_MC = 150*4.846*0.6 = 436.14 ; P = 436.14/(pi*0.375^2) = 987.15
        self.assertAlmostEqual(o["brake_pressure_front_psi"], 987.15,
                               delta=0.2)
        # clamp = 987.15*1.06 = 1046.4 ; T = 2*(2*0.4*1046.4*2.975) = 4980.8
        self.assertAlmostEqual(o["brake_torque_front_lbin"], 4980.8,
                               delta=2.0)
        # F_ground = 4980.8/11.5 = 433.11 ; rear same but bias 0.4:
        # rear force = 433.11*(0.4/0.6) = 288.74 ; decel = 721.85/580
        self.assertAlmostEqual(o["brake_force_front_lb"], 433.11, delta=0.2)
        self.assertAlmostEqual(o["brake_decel_g"], 721.85 / 580.0,
                               delta=0.002)

    # ---- Dynamics sheet ---------------------------------------------------
    def test_bounce_and_pitch(self):
        # v1.22.1: sprung modes use the RIDE rate (tire in series), like the
        # ride frequencies — NOT the bare wheel rate. The bounce frequency
        # is a mass-weighted blend of the two corner oscillators, so it MUST
        # lie between the two ride frequencies (the wheel-rate version put it
        # above both, which is impossible).
        o = self.out
        ms = 450.0 / 386.4
        krf, krr = o["ride_rate_front_lbin"], o["ride_rate_rear_lbin"]
        self.assertAlmostEqual(
            o["bounce_freq_hz"],
            np.sqrt((2 * krf + 2 * krr) / ms) / (2 * np.pi), places=6)
        lo = min(o["ride_freq_front_hz"], o["ride_freq_rear_hz"])
        hi = max(o["ride_freq_front_hz"], o["ride_freq_rear_hz"])
        self.assertGreaterEqual(o["bounce_freq_hz"], lo - 1e-6)
        self.assertLessEqual(o["bounce_freq_hz"], hi + 1e-6)
        # pitch mode also on ride rates; k_y = sqrt(a*b) (Olley DI=1)
        self.assertAlmostEqual(o["pitch_radius_gyr_in"], 31.331, places=2)
        self.assertLess(abs(o["pitch_freq_hz"] - 2.2054), 0.01)

    def test_understeer(self):
        o = self.out
        # K_us = 260/150 - 320/170 = 1.7333 - 1.8824 = -0.1490
        self.assertAlmostEqual(o["understeer_grad_deg_g"], -0.1490,
                               places=3)
        self.assertEqual(o["understeer_note"], "Mild oversteer")

    def test_roundtrip_dict(self):
        d = DynamicsInputs().to_dict()
        d["ay_g"] = 1.4
        v2 = DynamicsInputs.from_dict({**d, "not_a_field": 1.0})
        self.assertEqual(v2.ay_g, 1.4)


class TestSpringStacks(unittest.TestCase):
    def test_series_and_parallel(self):
        v = DynamicsInputs(spring1_front_lbin=100.0,
                           spring2_front_lbin=50.0, stack_front="Series")
        self.assertAlmostEqual(compute(v)["spring_eff_front_lbin"],
                               100.0 * 50.0 / 150.0)
        v.stack_front = "Parallel"
        self.assertAlmostEqual(compute(v)["spring_eff_front_lbin"], 150.0)
        v.stack_front = "Single"
        self.assertAlmostEqual(compute(v)["spring_eff_front_lbin"], 100.0)


if __name__ == "__main__":
    unittest.main()
