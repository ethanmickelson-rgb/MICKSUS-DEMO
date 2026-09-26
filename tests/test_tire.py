"""Sanity checks for the tire model seam (tire.py).

These pin the MODEL SHAPE (linear near zero, saturating, load-scaling) and
that every SUN-F estimate is provenance-tagged — NOT any real grip number,
which is still a documented estimate until the team runs a tire test."""

import unittest

import numpy as np

from suspension_tool.tire import (SUNF_23x7, SUNF_23x7_PLACEHOLDER,
                                  SURFACE_PEAK_MU, Confidence, TireEstimate,
                                  TireModel, estimate_cornering_stiffness_n_per_deg,
                                  pacejka_lateral_coeffs)


class TestTireModel(unittest.TestCase):
    def setUp(self):
        self.t = TireModel(cornering_stiffness_n_per_deg=120.0,
                           peak_friction=1.0, rated_load_n=1000.0)

    def test_linear_near_zero_matches_stiffness(self):
        # dFy/dalpha at ~0 slip is the cornering stiffness
        f = self.t.lateral_force(0.01)
        self.assertAlmostEqual(f / 0.01, 120.0, delta=1.0)

    def test_saturates_at_mu_fz(self):
        f = self.t.lateral_force(40.0)      # large slip
        self.assertLessEqual(f, 1000.0)
        self.assertGreater(f, 0.95 * 1000.0)

    def test_force_scales_with_load(self):
        light = self.t.lateral_force(20.0, fz_n=500.0)
        heavy = self.t.lateral_force(20.0, fz_n=1500.0)
        self.assertGreater(heavy, light)

    def test_odd_symmetry(self):
        self.assertAlmostEqual(self.t.lateral_force(5.0),
                               -self.t.lateral_force(-5.0), places=9)

    def test_array_input(self):
        out = self.t.lateral_force(np.array([0.0, 5.0, 10.0]))
        self.assertEqual(out.shape, (3,))
        self.assertTrue(np.all(np.diff(out) > 0))

    def test_peak_slip_positive(self):
        self.assertGreater(self.t.peak_slip_deg(), 0.0)

    def test_lb_per_deg_conversion(self):
        self.assertAlmostEqual(self.t.cornering_stiffness_lb_per_deg(),
                               120.0 / 4.4482216153, places=6)

    def test_estimate_is_fraction_of_load(self):
        self.assertAlmostEqual(
            estimate_cornering_stiffness_n_per_deg(1000.0, factor=0.11),
            110.0, places=6)

    def test_model_is_labelled_estimated_not_measured(self):
        # the force model must never masquerade as measured data
        self.assertIn("ESTIMATED", SUNF_23x7_PLACEHOLDER.label.upper())
        self.assertIs(SUNF_23x7_PLACEHOLDER, SUNF_23x7.model)


class TestSunfEstimate(unittest.TestCase):
    def test_every_param_is_provenance_tagged(self):
        # no bare numbers — each must carry a Confidence and a basis
        self.assertTrue(SUNF_23x7.params)
        for p in SUNF_23x7.params:
            self.assertIsInstance(p.confidence, Confidence)
            self.assertTrue(p.basis and len(p.basis) > 10, p.name)
            self.assertTrue(p.units)

    def test_physical_dims_match_size_code(self):
        self.assertEqual(SUNF_23x7.param("overall_diameter").value, 23.0)
        self.assertEqual(SUNF_23x7.param("section_width").value, 7.0)
        self.assertEqual(SUNF_23x7.param("rim_diameter").value, 10.0)
        # size-code values are the only MEASURED ones
        self.assertEqual(SUNF_23x7.param("overall_diameter").confidence,
                         Confidence.MEASURED)

    def test_peak_slip_matches_the_measured_tire(self):
        """MEASURED DATA OVERTURNED AN ASSUMPTION HERE (v1.35).

        This used to assert a peak beyond 8 deg, on the standard belief
        that knobbies peak far later than slicks because the lugs squirm.
        The measured Dunlop KT821 says otherwise: it reaches 95% of peak at
        6.7 deg at the design load. The reason is that our estimate had the
        cornering stiffness nearly HALF what it really is -- a tire that
        builds force twice as fast, against a lower peak mu, necessarily
        peaks earlier. Both errors pointed the same way and hid each other.

        Still later than a slick's 5-7 deg, but not the 10-15 deg assumed.
        """
        peak = SUNF_23x7.model.peak_slip_deg()
        self.assertGreater(peak, 3.0)
        self.assertLess(peak, 8.0)
        from suspension_tool.tire import DUNLOP_KT821_CLAY, DESIGN_FZ_LB
        self.assertAlmostEqual(
            DUNLOP_KT821_CLAY.slip_at_frac_of_peak(DESIGN_FZ_LB, 0.95),
            6.7, delta=0.3)

    def test_axle_cornering_stiffness_reasonable_for_knobby(self):
        # per-axle stiffness for two ~150 lb tires, much lower than a slick
        ca = SUNF_23x7.cornering_stiffness_axle_lb_per_deg()
        self.assertGreater(ca, 10.0)
        self.assertLess(ca, 80.0)

    def test_surface_mu_ordering(self):
        m = SURFACE_PEAK_MU
        self.assertGreater(m["dry_pavement"], m["dry_hardpack"])
        self.assertGreater(m["dry_hardpack"], m["loose_gravel"])
        self.assertGreater(m["loose_gravel"], m["mud"])
        self.assertLess(m["mud"], 0.6)     # low-grip surface

    def test_pacejka_coeffs_reconstruct_peak_and_slope(self):
        c = pacejka_lateral_coeffs()
        # D is the peak force = mu*Fz
        self.assertAlmostEqual(
            c["D"], SUNF_23x7.model.peak_friction * c["fz_n"], places=3)
        # B*C*D recovers the cornering stiffness (N/rad)
        bcd = c["B"] * c["C"] * c["D"]
        expect = (SUNF_23x7.model.cornering_stiffness_n_per_deg
                  * 180.0 / np.pi)
        self.assertAlmostEqual(bcd, expect, delta=1.0)


if __name__ == "__main__":
    unittest.main()
