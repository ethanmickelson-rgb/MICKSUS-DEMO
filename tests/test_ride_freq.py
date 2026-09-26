"""Tests for the quarter-car frequency response (v1.25.0).

The important ones are the CROSS-CHECKS: the 2-DOF model must reproduce, from
a completely different formulation, numbers the tool already computes another
way. That makes this module a check on the dynamics layer as much as the
other way round.
"""

import unittest

import numpy as np

from suspension_tool import dynamics
from suspension_tool.ride_freq import (DEFAULT_TERRAIN, QuarterCar,
                                       default_frequencies,
                                       excitation_speed_mph, from_dynamics,
                                       ride_report, terrain_table)

G = 386.4


def _qc(**kw):
    base = dict(name="test", sprung_lb=100.0, unsprung_lb=30.0,
                wheel_rate_lbin=61.25, tire_rate_lbin=250.0,
                damp_bump_lbsin=7.35, damp_rebound_lbsin=9.8)
    base.update(kw)
    return QuarterCar(**base)


class TestModalFrequencies(unittest.TestCase):
    def test_body_mode_matches_the_dynamics_ride_frequency(self):
        """CROSS-CHECK. `dynamics` derives the ride frequency in closed form
        from the RIDE rate (spring and tire in series); this module solves a
        2-DOF eigenproblem. Different maths, same car — they must agree to
        about a percent (the small gap is the real modal coupling, which the
        closed form neglects)."""
        v = dynamics.DynamicsInputs()
        r = dynamics.compute(v)
        for axle in ("front", "rear"):
            qc = from_dynamics(v, results=r, axle=axle)
            body, _hop = qc.modal_frequencies_hz()
            ref = r[f"ride_freq_{axle}_hz"]
            self.assertLess(abs(body - ref) / ref, 0.02,
                            f"{axle}: 2-DOF body mode {body:.3f} Hz vs "
                            f"dynamics ride freq {ref:.3f} Hz")

    def test_wheel_hop_matches_the_standard_closed_form(self):
        """f_hop = sqrt((k_s + k_t)/m_u)/2pi — the textbook decoupled form."""
        qc = _qc()
        _body, hop = qc.modal_frequencies_hz()
        closed = np.sqrt((qc.wheel_rate_lbin + qc.tire_rate_lbin)
                         / qc.m_u) / (2 * np.pi)
        self.assertLess(abs(hop - closed) / closed, 0.02)

    def test_hop_is_well_above_the_body_mode(self):
        body, hop = _qc().modal_frequencies_hz()
        self.assertGreater(hop, 3.0 * body)

    def test_modes_are_independent_of_damping(self):
        # taken from the undamped eigenproblem on purpose
        a = _qc(damp_bump_lbsin=1.0, damp_rebound_lbsin=1.0)
        b = _qc(damp_bump_lbsin=50.0, damp_rebound_lbsin=90.0)
        np.testing.assert_allclose(a.modal_frequencies_hz(),
                                   b.modal_frequencies_hz(), rtol=1e-12)

    def test_softer_spring_lowers_the_body_mode(self):
        stiff = _qc(wheel_rate_lbin=120.0).modal_frequencies_hz()[0]
        soft = _qc(wheel_rate_lbin=40.0).modal_frequencies_hz()[0]
        self.assertGreater(stiff, soft)

    def test_heavier_unsprung_lowers_wheel_hop(self):
        light = _qc(unsprung_lb=20.0).modal_frequencies_hz()[1]
        heavy = _qc(unsprung_lb=60.0).modal_frequencies_hz()[1]
        self.assertGreater(light, heavy)


class TestTransferFunctions(unittest.TestCase):
    def test_asymptotes(self):
        qc = _qc()
        lo = qc.response(np.array([1e-3]))
        hi = qc.response(np.array([500.0]))
        # DC: the body rides the road exactly, nothing deflects
        self.assertAlmostEqual(float(lo["body_travel"][0]), 1.0, places=5)
        self.assertAlmostEqual(float(lo["tire_deflection"][0]), 0.0, places=5)
        self.assertAlmostEqual(float(lo["suspension_travel"][0]), 0.0,
                               places=5)
        self.assertAlmostEqual(float(lo["body_accel"][0]), 0.0, places=5)
        # High frequency: the body cannot follow at all...
        self.assertLess(float(hi["body_travel"][0]), 1e-2)
        # ...and the wheel cannot either, so the tire absorbs the whole input
        self.assertAlmostEqual(float(hi["tire_deflection"][0]), 1.0, places=1)

    def test_response_is_positive_and_finite(self):
        qc = _qc()
        r = qc.response(default_frequencies())
        for key in ("body_travel", "body_accel", "tire_deflection",
                    "suspension_travel"):
            self.assertTrue(np.all(np.isfinite(r[key])), key)
            self.assertTrue(np.all(r[key] >= 0.0), key)

    def test_more_damping_cuts_the_body_resonance_peak(self):
        f = default_frequencies()
        light = _qc().response(f, damping_lbsin=2.0)["body_travel"].max()
        heavy = _qc().response(f, damping_lbsin=12.0)["body_travel"].max()
        self.assertGreater(light, heavy)

    def test_band_brackets_the_two_damping_curves(self):
        qc = _qc()
        f = default_frequencies()
        band = qc.response_band(f)
        for key in ("body_accel", "tire_deflection"):
            self.assertTrue(np.all(band[f"{key}_min"]
                                   <= band[f"{key}_max"] + 1e-12))
            # the band edges ARE the bump and rebound responses
            both = np.vstack([band[f"{key}_bump"], band[f"{key}_rebound"]])
            np.testing.assert_allclose(band[f"{key}_min"], both.min(axis=0))
            np.testing.assert_allclose(band[f"{key}_max"], both.max(axis=0))

    def test_scalar_frequency_accepted(self):
        r = _qc().response(2.0)
        self.assertEqual(r["body_travel"].shape, (1,))


class TestDampingRatios(unittest.TestCase):
    def test_critical_damping_definition(self):
        qc = _qc()
        self.assertAlmostEqual(
            qc.critical_damping_lbsin,
            2.0 * np.sqrt(qc.wheel_rate_lbin * qc.sprung_lb / G), places=9)

    def test_zeta_one_is_critical(self):
        qc = _qc()
        c = qc.critical_damping_lbsin
        qc2 = _qc(damp_bump_lbsin=c, damp_rebound_lbsin=c)
        self.assertAlmostEqual(qc2.zeta_bump, 1.0, places=9)
        self.assertAlmostEqual(qc2.zeta_rebound, 1.0, places=9)

    def test_damper_rate_is_taken_to_the_wheel_by_mr_squared(self):
        v = dynamics.DynamicsInputs()
        qc = from_dynamics(v, axle="front")
        self.assertAlmostEqual(
            qc.damp_bump_lbsin,
            v.damp_bump_front_lbsin * v.mr_damper_front ** 2, places=9)


class TestTerrain(unittest.TestCase):
    def test_excitation_speed_formula(self):
        # 2 Hz over bumps 10 ft apart = 20 ft/s = 13.64 mph
        self.assertAlmostEqual(excitation_speed_mph(2.0, 10.0),
                               2.0 * 10.0 * 12.0 / 17.6, places=9)

    def test_speed_scales_with_frequency_and_spacing(self):
        a = excitation_speed_mph(2.0, 10.0)
        self.assertAlmostEqual(excitation_speed_mph(4.0, 10.0), 2 * a)
        self.assertAlmostEqual(excitation_speed_mph(2.0, 20.0), 2 * a)

    def test_table_covers_every_feature_and_hop_is_faster(self):
        qc = _qc()
        rows = terrain_table(qc)
        self.assertEqual(len(rows), len(DEFAULT_TERRAIN))
        for row in rows:
            # wheel hop is the higher frequency, so it needs a higher speed
            self.assertGreater(row["wheel_hop_mph"], row["body_mph"])

    def test_custom_terrain(self):
        rows = terrain_table(_qc(), {"my bumps": 5.0})
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["spacing_ft"], 5.0)


class TestRideReport(unittest.TestCase):
    def test_report_is_complete(self):
        v = dynamics.DynamicsInputs()
        rep = ride_report(v, "front")
        self.assertGreater(rep.wheel_hop_hz, rep.body_hz)
        self.assertEqual(len(rep.freq_hz), len(rep.band["freq_hz"]))
        for key in ("body_accel", "tire_deflection"):
            self.assertIn(key, rep.peaks)
            self.assertGreater(rep.peaks[key]["peak"], 0.0)
        self.assertTrue(rep.notes)
        self.assertTrue(rep.terrain)

    def test_overdamped_setup_is_flagged(self):
        v = dynamics.DynamicsInputs()
        v.damp_bump_front_lbsin = 200.0
        v.damp_rebound_front_lbsin = 300.0
        rep = ride_report(v, "front")
        joined = " ".join(rep.notes).lower()
        self.assertIn("damping ratio", joined)

    def test_sensible_setup_has_no_damping_warning(self):
        v = dynamics.DynamicsInputs()
        qc = from_dynamics(v, axle="front")
        ccr = qc.critical_damping_lbsin
        mr2 = v.mr_damper_front ** 2
        v.damp_bump_front_lbsin = 0.35 * ccr / mr2
        v.damp_rebound_front_lbsin = 0.60 * ccr / mr2
        rep = ride_report(v, "front")
        joined = " ".join(rep.notes[2:]).lower()
        self.assertNotIn("very high", joined)
        self.assertNotIn("over critical", joined)

    def test_bad_axle_rejected(self):
        with self.assertRaises(ValueError):
            from_dynamics(dynamics.DynamicsInputs(), axle="middle")


class TestPublishedYAxisTargets(unittest.TestCase):
    """The guide publishes specific peak MAGNITUDES. If the model drifts,
    the printed guide silently becomes wrong — so pin the numbers here."""

    def _corner(self, zeta):
        ks, kt, ms, mu = 61.58, 251.1, 100.0, 30.0
        c = zeta * 2 * np.sqrt(ks * ms / G)
        return QuarterCar("g", ms, mu, ks, kt, c, c)

    def test_zeta_to_peak_table(self):
        # (zeta, body motion, tire deflection, suspension travel)
        published = [(0.25, 3.3, 2.2, 2.5), (0.35, 2.5, 1.8, 1.9),
                     (0.45, 2.1, 1.6, 1.6), (0.70, 1.8, 1.4, 1.3)]
        f = default_frequencies(0.2, 60.0, 4000)
        for zeta, body, tire, travel in published:
            r = self._corner(zeta).response(f)
            for got, want, name in (
                    (r["body_travel"].max(), body, "body motion"),
                    (r["tire_deflection"].max(), tire, "tire deflection"),
                    (r["suspension_travel"].max(), travel, "susp travel")):
                self.assertAlmostEqual(
                    got, want, delta=0.06,
                    msg=f"zeta {zeta}: {name} {got:.2f} vs published {want}")

    def test_hard_asymptotes_the_guide_promises(self):
        r = self._corner(0.35)
        lo = r.response(np.array([1e-3]))
        hi = r.response(np.array([2000.0]))
        # "body motion starts at exactly 1.0; tire deflection ends at 1.0"
        self.assertAlmostEqual(float(lo["body_travel"][0]), 1.0, places=6)
        self.assertAlmostEqual(float(hi["tire_deflection"][0]), 1.0, places=3)

    def test_over_damping_makes_tire_deflection_worse_again(self):
        """The non-monotonic result the guide calls the fingerprint of
        over-damping: past ~0.7 the tire-deflection peak turns back up."""
        f = default_frequencies(0.2, 60.0, 4000)
        peak = {z: self._corner(z).response(f)["tire_deflection"].max()
                for z in (0.55, 0.70, 0.90)}
        self.assertLess(peak[0.70], peak[0.55])     # still improving
        self.assertGreater(peak[0.90], peak[0.70])  # now getting worse

    def test_tire_liftoff_amplitude(self):
        """Guide: 0.29 in of road at the hop resonance fully unloads a
        130 lb Baja corner."""
        f = default_frequencies(0.2, 60.0, 4000)
        qc = self._corner(0.35)
        peak = qc.response(f)["tire_deflection"].max()
        amp = 130.0 / (peak * qc.tire_rate_lbin)
        self.assertAlmostEqual(amp, 0.29, delta=0.02)


class TestBodeGuideShipped(unittest.TestCase):
    """The two-page PDF guide is linked from the in-app help, so it has to
    actually be present in the install — a broken link there is invisible
    until someone clicks it."""

    def test_guide_pdf_exists_and_is_a_pdf(self):
        import os
        from suspension_tool.gui.main_window import MainWindow
        path = MainWindow.bode_guide_path()
        self.assertTrue(os.path.exists(path), f"missing guide: {path}")
        with open(path, "rb") as fh:
            self.assertEqual(fh.read(5), b"%PDF-")
        self.assertGreater(os.path.getsize(path), 4000)

    def test_help_html_carries_the_link_token(self):
        from suspension_tool.gui.help_text import HELP_HTML
        self.assertIn("__BODE_GUIDE_URL__", HELP_HTML,
                      "help must contain the token show_help() substitutes")

    def test_show_help_substitutes_a_real_file_url(self):
        import os
        from PySide6.QtCore import QUrl
        from suspension_tool.gui.help_text import HELP_HTML
        from suspension_tool.gui.main_window import MainWindow
        url = QUrl.fromLocalFile(MainWindow.bode_guide_path()).toString()
        html = HELP_HTML.replace("__BODE_GUIDE_URL__", url)
        self.assertNotIn("__BODE_GUIDE_URL__", html)
        self.assertTrue(url.startswith("file://"))
        self.assertTrue(os.path.exists(MainWindow.bode_guide_path()))


if __name__ == "__main__":
    unittest.main()
