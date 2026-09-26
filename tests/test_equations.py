"""Tests for the equation reference (v1.27.0).

The point of these is DRIFT PROTECTION. An equation sheet that quietly stops
matching the code is worse than no sheet at all, because it is trusted. So
where a formula is cheap to evaluate, the test evaluates it by hand and
compares against what the software actually computes.
"""

import os
import unittest

import numpy as np

from suspension_tool import dynamics
from suspension_tool.equations import (SECTIONS, SYMBOLS, Equation,
                                       all_equations, to_html)

G = 386.4


class TestStructure(unittest.TestCase):
    def test_every_equation_is_populated(self):
        eqs = all_equations()
        self.assertGreater(len(eqs), 60)
        for eq in eqs:
            self.assertTrue(eq.name.strip(), "unnamed equation")
            self.assertTrue(eq.expr.strip(), f"{eq.name} has no formula")
            self.assertTrue(eq.module.strip(), f"{eq.name} names no module")

    def test_named_modules_exist(self):
        """A formula pointing at a module that no longer exists is a broken
        cross-reference."""
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        for eq in all_equations():
            for mod in eq.module.split(","):
                mod = mod.strip()
                if not mod:
                    continue
                self.assertTrue(
                    os.path.exists(os.path.join(root, "suspension_tool", mod)),
                    f"{eq.name} references missing module {mod}")

    def test_names_are_unique(self):
        names = [f"{s.title}|{e.name}" for s in SECTIONS for e in s.equations]
        self.assertEqual(len(names), len(set(names)), "duplicate equation")

    def test_sections_have_content(self):
        for s in SECTIONS:
            self.assertTrue(s.equations, f"empty section {s.title}")

    def test_symbol_table(self):
        self.assertGreater(len(SYMBOLS), 20)
        for sym, mean, unit in SYMBOLS:
            self.assertTrue(sym.strip() and mean.strip())


class TestHtml(unittest.TestCase):
    def test_html_renders_everything(self):
        from suspension_tool.equations import _esc
        html = to_html()
        for s in SECTIONS:
            # titles are HTML-escaped on the way in ('&' -> '&amp;')
            self.assertIn(_esc(s.title.split("·")[-1].strip()[:20]), html)
        self.assertIn("386.4", html)
        self.assertGreater(len(html), 10000)

    def test_html_escapes_comparison_operators(self):
        """Formulas contain < and >; unescaped they would eat the page."""
        html = to_html()
        self.assertIn("&lt;", html)
        self.assertNotIn("<code style='background:#F2F5FB'>camber <", html)


class TestFormulasMatchTheCode(unittest.TestCase):
    """Evaluate the documented formula by hand; compare to the software."""

    def setUp(self):
        self.v = dynamics.DynamicsInputs()
        self.r = dynamics.compute(self.v)

    def test_cg_longitudinal(self):
        # b = W_F * L / W;  a = L - b
        v, r = self.v, self.r
        w = v.weight_empty_lb + v.driver_lb
        b = v.front_axle_lb * v.wheelbase_in / w
        self.assertAlmostEqual(r["cg_to_rear_in"], b, places=9)
        self.assertAlmostEqual(r["cg_to_front_in"], v.wheelbase_in - b,
                               places=9)

    def test_sprung_weight_and_cg_height(self):
        v, r = self.v, self.r
        w = v.weight_empty_lb + v.driver_lb
        ws = w - 2 * v.unsprung_front_lb - 2 * v.unsprung_rear_lb
        self.assertAlmostEqual(r["sprung_weight_lb"], ws, places=9)
        hs = (w * v.cg_height_in
              - 2 * v.unsprung_front_lb * v.loaded_radius_front_in
              - 2 * v.unsprung_rear_lb * v.loaded_radius_rear_in) / ws
        self.assertAlmostEqual(r["sprung_cg_height_in"], hs, places=9)

    def test_wheel_and_ride_rate(self):
        # K_w = K_s * MR^2 ; K_r = K_w K_t / (K_w + K_t)
        v, r = self.v, self.r
        kw = v.spring1_front_lbin * v.mr_spring_front ** 2
        self.assertAlmostEqual(r["wheel_rate_front_lbin"], kw, places=9)
        kt = v.tire_rate_front_lbin
        self.assertAlmostEqual(r["ride_rate_front_lbin"], kw * kt / (kw + kt),
                               places=9)

    def test_roll_moment_arm(self):
        # H = h_s - z_RA ,  z_RA interpolated by the sprung fraction
        r = self.r
        v = self.v
        z_ra = (v.rc_front_in
                + (v.rc_rear_in - v.rc_front_in) * (1 - r["front_sprung_frac"]))
        self.assertAlmostEqual(r["roll_axis_at_cg_in"], z_ra, places=9)
        self.assertAlmostEqual(r["roll_moment_arm_in"],
                               r["sprung_cg_height_in"] - z_ra, places=9)

    def test_longitudinal_transfer(self):
        # dW_x = A_x * W * h / L
        v, r = self.v, self.r
        w = v.weight_empty_lb + v.driver_lb
        self.assertAlmostEqual(
            r["long_transfer_lb"],
            v.ax_g * w * v.cg_height_in / v.wheelbase_in, places=9)

    def test_lateral_transfer_three_terms(self):
        """The documented three-reaction Milliken 18.4 form."""
        v, r = self.v, self.r
        ws = r["sprung_weight_lb"]
        h = r["roll_moment_arm_in"]
        kf, kr = r["roll_rate_front_lbftdeg"], r["roll_rate_rear_lbftdeg"]
        b, ll = r["cg_to_rear_in"], v.wheelbase_in
        dwf = (v.ay_g / (v.track_front_in / 12.0)) * (
            ws * (h / 12.0) * kf / (kf + kr)
            + ws * (b / ll) * (v.rc_front_in / 12.0)
            + 2 * v.unsprung_front_lb * (v.loaded_radius_front_in / 12.0))
        self.assertAlmostEqual(r["lat_transfer_front_lb"], dwf, places=8)

    def test_roll_gradient(self):
        v, r = self.v, self.r
        expect = -(r["sprung_weight_lb"] * (r["roll_moment_arm_in"] / 12.0)
                   / (r["roll_rate_front_lbftdeg"]
                      + r["roll_rate_rear_lbftdeg"]))
        self.assertAlmostEqual(r["roll_gradient_deg_g"], expect, places=9)

    def test_banking_sign(self):
        v, r = self.v, self.r
        expect = (v.ay_g * np.cos(np.radians(v.bank_deg))
                  - np.sin(np.radians(v.bank_deg)))
        self.assertAlmostEqual(r["ay_banked_g"], expect, places=9)

    def test_understeer_gradient(self):
        v, r = self.v, self.r
        expect = (v.front_axle_lb / v.cornering_stiff_front_lbdeg
                  - v.rear_axle_lb / v.cornering_stiff_rear_lbdeg)
        self.assertAlmostEqual(r["understeer_grad_deg_g"], expect, places=9)

    def test_damping_ratio_and_critical_damping(self):
        # c_crit = 2 sqrt(K_w m_s_corner) ; zeta = c*MR_d^2 / c_crit
        v, r = self.v, self.r
        ms = r["corner_sprung_front_lb"] / G
        c_crit = 2 * np.sqrt(r["wheel_rate_front_lbin"] * ms)
        self.assertAlmostEqual(r["damp_critical_front_lbsin"], c_crit,
                               places=8)
        zeta = v.damp_bump_front_lbsin * v.mr_damper_front ** 2 / c_crit
        self.assertAlmostEqual(r["zeta_bump_front"], zeta, places=8)

    def test_pitch_radius_of_gyration(self):
        r = self.r
        self.assertAlmostEqual(
            r["pitch_radius_gyr_in"],
            np.sqrt(r["cg_to_front_in"] * r["cg_to_rear_in"]), places=9)

    def test_brake_chain(self):
        v, r = self.v, self.r
        f_mc = v.pedal_force_lb * v.pedal_ratio * v.brake_bias_front
        press = f_mc / (np.pi * (v.mc_bore_front_in / 2.0) ** 2)
        clamp = press * v.caliper_area_front_in2
        torque = 2.0 * (2.0 * v.pad_mu_front * clamp * v.rotor_radius_front_in)
        self.assertAlmostEqual(r["brake_pressure_front_psi"], press, places=7)
        self.assertAlmostEqual(r["brake_torque_front_lbin"], torque, places=6)
        self.assertAlmostEqual(r["brake_force_front_lb"],
                               torque / v.loaded_radius_front_in, places=6)


class TestKinematicFormulas(unittest.TestCase):
    def test_impulse_and_peak_force(self):
        from suspension_tool.impact import ImpactCase
        c = ImpactCase(speed_mph=15.0, final_speed_mph=0.0,
                       contact_time_s=0.15, vehicle_weight_lb=580.0,
                       corner_share=0.5, pulse="half_sine",
                       safety_factor=1.0)
        m = 580.0 / G
        j = m * (15.0 * 17.6)
        self.assertAlmostEqual(c.impulse_lb_s(), j, places=9)
        self.assertAlmostEqual(c.average_force_lb(), j / 0.15, places=9)
        self.assertAlmostEqual(c.peak_force_lb(),
                               (j / 0.15) * (np.pi / 2) * 0.5, places=8)

    def test_drop_height(self):
        from suspension_tool.impact import drop_height_to_mph
        self.assertAlmostEqual(drop_height_to_mph(36.0),
                               np.sqrt(2 * G * 36.0) / 17.6, places=9)

    def test_quarter_car_hop_formula(self):
        from suspension_tool.ride_freq import QuarterCar
        qc = QuarterCar("t", 100.0, 30.0, 61.25, 250.0, 7.0, 7.0)
        closed = np.sqrt((61.25 + 250.0) / (30.0 / G)) / (2 * np.pi)
        self.assertLess(abs(qc.wheel_hop_hz - closed) / closed, 0.02)

    def test_terrain_excitation_formula(self):
        from suspension_tool.ride_freq import excitation_speed_mph
        # V = f * lambda
        self.assertAlmostEqual(excitation_speed_mph(2.0, 10.0),
                               2.0 * 10.0 * 12.0 / 17.6, places=9)

    def test_froude_frequency_scaling(self):
        """frequency ~ 1/sqrt(lambda): 1:10 gives 3.16x."""
        self.assertAlmostEqual(1.0 / np.sqrt(0.1), 3.1623, places=3)

    def test_arb_wheel_rate_conversion(self):
        """k_bar = K_phi / track^2, with K_phi in lb-in/rad."""
        from suspension_tool.full_car import from_dynamics
        v = dynamics.DynamicsInputs()
        r = dynamics.compute(v)
        car = from_dynamics(v, r)
        expect = (r["arb_roll_rate_front_lbftdeg"] * 12.0 * 180.0 / np.pi
                  / v.track_front_in ** 2)
        self.assertAlmostEqual(car.arb_wheel_rate_front_lbin, expect,
                               places=7)


class TestSheetShipped(unittest.TestCase):
    def test_pdf_exists_and_is_a_pdf(self):
        from suspension_tool.gui.main_window import MainWindow
        path = MainWindow.equation_sheet_path()
        self.assertTrue(os.path.exists(path), f"missing sheet: {path}")
        with open(path, "rb") as fh:
            self.assertEqual(fh.read(5), b"%PDF-")
        self.assertGreater(os.path.getsize(path), 10000)

    def test_builder_script_present(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.assertTrue(os.path.exists(
            os.path.join(root, "tools", "build_equation_sheet.py")),
            "the PDF must stay regenerable from equations.py")


if __name__ == "__main__":
    unittest.main()
