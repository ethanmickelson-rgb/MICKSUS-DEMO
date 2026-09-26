"""Milliken Moment Method yaw-moment diagrams, and the measured tire data
they run on (v1.35).

Both came from an MMM implementation shared on the Baja SAE Discord by an
Auburn student. Two things are pinned here:

  * the EXPONENTIAL TIRE MODEL and its derived quantities. Peak mu and
    cornering stiffness both fall out of two coefficients, and the
    relations (mu = c*Fz^m, C_alpha = Fz*mu^2) are exact, so they can be
    checked analytically rather than by eyeball.
  * the MMM itself, including that it reproduces the qualitative behaviour
    the author documented -- a car that "turns into a pig" at low speed.

The Auburn vehicle file is a REAL Baja car, so it doubles as an
independent validation case for our own dynamics defaults, which have
never had one.
"""

import unittest

import numpy as np

from suspension_tool.tire import (DUNLOP_KT821_CLAY, DUNLOP_KT821_GRAVEL,
                                  DESIGN_FZ_LB, SURFACE_PEAK_MU,
                                  ExponentialTire, surface_tire,
                                  tire_data_spread)
from suspension_tool.yaw_moment import (AUBURN_EXAMPLE, YawMomentVehicle,
                                        solve_point, yaw_moment_diagram)


class TestExponentialTireModel(unittest.TestCase):

    def test_peak_mu_is_the_power_law(self):
        t = DUNLOP_KT821_CLAY
        for fz in (50.0, 150.0, 400.0):
            self.assertAlmostEqual(float(t.peak_mu(fz)),
                                   t.coeff_c * fz ** t.coeff_m, places=12)

    def test_mu_falls_with_load(self):
        """Load sensitivity is the whole point of the power law: a tire
        that kept its mu as load rose would make load transfer free."""
        t = DUNLOP_KT821_CLAY
        mus = [float(t.peak_mu(fz)) for fz in (100.0, 200.0, 400.0)]
        self.assertTrue(mus[0] > mus[1] > mus[2])

    def test_cornering_stiffness_matches_the_derivative(self):
        """C_alpha = Fz*mu^2 is claimed as EXACT, so check it against a
        numerical derivative rather than trusting the algebra."""
        for t in (DUNLOP_KT821_CLAY, DUNLOP_KT821_GRAVEL):
            for fz in (100.0, 250.0):
                # Fy is ODD in slip, so the central difference reduces to
                # Fy(h)/h and its error is FIRST order: relative error is
                # mu*h/2, not the h^2 a central difference usually gives.
                # h = 1e-5 puts that at ~3e-6; compare relative, since
                # C_alpha is order 30 lb/deg.
                h = 1e-5
                num = (float(t.lateral_force_lb(fz, h))
                       - float(t.lateral_force_lb(fz, -h))) / (2 * h)
                ana = float(t.cornering_stiffness_lb_per_deg(fz))
                self.assertLess(abs(num - ana) / ana, 1e-4,
                                f"{num} vs {ana}")

    def test_force_saturates_at_mu_fz(self):
        t = DUNLOP_KT821_CLAY
        fz = 150.0
        far = float(t.lateral_force_lb(fz, 90.0))
        self.assertAlmostEqual(far, fz * float(t.peak_mu(fz)), places=4)

    def test_force_is_odd_in_slip_angle(self):
        t = DUNLOP_KT821_CLAY
        for a in (0.5, 4.0, 12.0):
            self.assertAlmostEqual(float(t.lateral_force_lb(150.0, a)),
                                   -float(t.lateral_force_lb(150.0, -a)),
                                   places=9)

    def test_zero_load_makes_zero_force(self):
        """A lifted wheel must contribute nothing, not a NaN."""
        self.assertEqual(float(DUNLOP_KT821_CLAY.lateral_force_lb(0.0, 5.0)),
                         0.0)

    def test_slip_at_frac_of_peak_inverts_exactly(self):
        t = DUNLOP_KT821_CLAY
        for fz in (100.0, 250.0):
            a = t.slip_at_frac_of_peak(fz, 0.95)
            got = float(t.lateral_force_lb(fz, a))
            self.assertAlmostEqual(got / (fz * float(t.peak_mu(fz))), 0.95,
                                   places=9)

    def test_gravel_grips_less_than_hardpack(self):
        for fz in (100.0, 250.0):
            self.assertLess(float(DUNLOP_KT821_GRAVEL.peak_mu(fz)),
                            float(DUNLOP_KT821_CLAY.peak_mu(fz)))


class TestSurfaceTable(unittest.TestCase):

    def test_measured_surfaces_return_the_measured_curves(self):
        self.assertIs(surface_tire("dry_hardpack"), DUNLOP_KT821_CLAY)
        self.assertIs(surface_tire("loose_gravel"), DUNLOP_KT821_GRAVEL)

    def test_interpolated_surfaces_keep_the_measured_exponent(self):
        for name in ("dry_pavement", "damp_hardpack", "sand", "mud"):
            self.assertAlmostEqual(surface_tire(name).coeff_m,
                                   DUNLOP_KT821_CLAY.coeff_m, places=12)

    def test_surface_peak_mu_is_derived_and_ordered(self):
        mu = SURFACE_PEAK_MU
        self.assertGreater(mu["dry_pavement"], mu["dry_hardpack"])
        self.assertGreater(mu["dry_hardpack"], mu["loose_gravel"])
        self.assertGreater(mu["loose_gravel"], mu["mud"])
        # and it agrees with the model it is derived from
        self.assertAlmostEqual(
            mu["dry_hardpack"],
            float(DUNLOP_KT821_CLAY.peak_mu(DESIGN_FZ_LB)), places=12)

    def test_the_spread_between_measured_and_estimated_is_reported(self):
        """We recalibrated toward measured data whose author suspects it
        under-reports, from an estimate that ran optimistic. The gap is
        large and the tool must be able to say so rather than pretend to
        a precision it does not have."""
        s = tire_data_spread()
        self.assertGreater(s["high_lb"], s["low_lb"])
        self.assertGreater(s["spread_frac"], 0.2)


class TestVehicleGeometry(unittest.TestCase):

    def test_cg_position_follows_the_weight_split(self):
        v = AUBURN_EXAMPLE
        # moments about the front axle must balance
        self.assertAlmostEqual(v.weight_lb * v.a_ft,
                               (v.weight_lb - v.weight_front_lb)
                               * v.wheelbase_ft, places=9)

    def test_load_transfer_is_linear_and_signed(self):
        v = AUBURN_EXAMPLE
        f1, r1 = v.lateral_load_transfer_lb(0.5)
        f2, r2 = v.lateral_load_transfer_lb(1.0)
        self.assertAlmostEqual(f2, 2 * f1, places=6)
        self.assertAlmostEqual(r2, 2 * r1, places=6)
        self.assertGreater(f1, 0.0)

    def test_the_stiffer_narrower_axle_transfers_more(self):
        """Auburn's rear is stiffer (77 vs 64 ft-lb/deg) AND narrower
        (44 vs 51 in), so it must take the bigger share."""
        f, r = AUBURN_EXAMPLE.lateral_load_transfer_lb(0.5)
        self.assertGreater(r, f)

    def test_wheel_loads_never_go_negative(self):
        for ay in (0.0, 0.5, 1.0, 2.0):
            for fz in AUBURN_EXAMPLE.wheel_loads_lb(ay):
                self.assertGreaterEqual(fz, 0.0)

    def test_total_load_is_conserved_until_a_wheel_lifts(self):
        v = AUBURN_EXAMPLE
        self.assertAlmostEqual(sum(v.wheel_loads_lb(0.3)), v.weight_lb,
                               places=6)


class TestYawMomentDiagram(unittest.TestCase):

    def test_the_origin_is_trimmed_and_still(self):
        pt = solve_point(AUBURN_EXAMPLE, 0.0, 0.0, 25.0)
        self.assertAlmostEqual(pt["ay_g"], 0.0, places=6)
        self.assertAlmostEqual(pt["cn"], 0.0, places=6)

    def test_limit_ay_cannot_exceed_the_tire_mu(self):
        """The envelope is tire-limited: no arrangement of slip angles can
        produce more lateral g than the tires' peak friction."""
        d = yaw_moment_diagram(AUBURN_EXAMPLE, 25.0)
        mu_max = float(DUNLOP_KT821_CLAY.peak_mu(
            AUBURN_EXAMPLE.weight_lb / 4.0))
        self.assertLessEqual(d.limit_ay_g, mu_max * 1.05)
        self.assertGreater(d.limit_ay_g, 0.2)

    def test_trim_ay_does_not_exceed_the_limit(self):
        d = yaw_moment_diagram(AUBURN_EXAMPLE, 25.0)
        self.assertLessEqual(d.trim_ay_g(), d.limit_ay_g + 1e-9)

    def test_diagram_is_antisymmetric(self):
        """Left and right turns are mirror images on a symmetric car."""
        betas = np.linspace(-6.0, 6.0, 7)
        steers = np.linspace(-12.0, 12.0, 7)
        d = yaw_moment_diagram(AUBURN_EXAMPLE, 25.0, betas, steers)
        np.testing.assert_allclose(d.ay_g, -d.ay_g[::-1, ::-1], atol=1e-6)
        np.testing.assert_allclose(d.cn, -d.cn[::-1, ::-1], atol=1e-6)

    def test_low_speed_is_more_stable_than_high_speed(self):
        """The author's own headline observation: the car 'turns into a
        pig' at low speed. In MMM terms the stability index should climb
        (toward unstable) as speed rises."""
        si = [yaw_moment_diagram(AUBURN_EXAMPLE, v).stability_index
              for v in (10.0, 20.0, 40.0)]
        self.assertTrue(si[0] < si[1] < si[2], si)
        self.assertLess(si[0], 0.0, "should be stable at low speed")

    def test_constant_radius_mode_runs_and_differs(self):
        free = yaw_moment_diagram(AUBURN_EXAMPLE, 25.0)
        fixed = yaw_moment_diagram(AUBURN_EXAMPLE, 25.0, radius_ft=60.0)
        self.assertFalse(np.allclose(free.cn, fixed.cn))

    def test_a_grippier_front_pushes_the_balance_toward_oversteer(self):
        """Sanity of the sign convention: more front grip must move the
        limit balance in the loose direction, not the tight one."""
        import dataclasses
        base = AUBURN_EXAMPLE
        grippy = dataclasses.replace(
            base.tire_front,
            coeff_c=base.tire_front.coeff_c * 1.35,
            label="front +35% grip")
        loose = dataclasses.replace(base, tire_front=grippy)
        b0 = yaw_moment_diagram(base, 25.0).limit_balance
        b1 = yaw_moment_diagram(loose, 25.0).limit_balance
        self.assertGreater(b1, b0)

    def test_summary_reports_every_headline_number(self):
        s = yaw_moment_diagram(AUBURN_EXAMPLE, 25.0).summary()
        for key in ("limit_ay_g", "trim_ay_g", "stability_index",
                    "control_moment_gain", "lateral_accel_gain",
                    "limit_balance", "stable", "balance"):
            self.assertIn(key, s)
        self.assertTrue(np.isfinite(s["stability_index"]))


class TestAuburnCarAgainstOurDynamics(unittest.TestCase):
    """The Auburn file is a real Baja car with its own published roll
    rates and roll-centre heights, so our load-transfer maths can be
    checked against a vehicle nobody on this team parameterised."""

    def test_our_roll_gradient_is_in_a_sane_band(self):
        v = AUBURN_EXAMPLE
        k_total = (v.roll_rate_front_ftlb_deg + v.roll_rate_rear_ftlb_deg)
        # roll gradient = W*H1 / K_total, deg per g
        grad = v.weight_lb * v.roll_moment_arm_ft / k_total
        self.assertGreater(grad, 1.0)
        self.assertLess(grad, 12.0, f"{grad:.2f} deg/g is implausible")

    def test_load_transfer_distribution_matches_roll_stiffness(self):
        """The stiffer axle must take the larger share of the sprung
        couple -- the single most important consequence of roll-rate
        split, and the reason an ARB changes balance at all."""
        v = AUBURN_EXAMPLE
        f, r = v.lateral_load_transfer_lb(1.0)
        share_rear = r / (f + r)
        self.assertGreater(share_rear, 0.5)
        self.assertLess(share_rear, 0.75)


if __name__ == "__main__":
    unittest.main()
