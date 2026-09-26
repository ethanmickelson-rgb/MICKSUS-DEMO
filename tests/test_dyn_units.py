"""Metric/imperial display in the dynamics panel (v1.37).

The panel used to be imperial no matter what the toolbar said, which is
awkward for a 1/10 RC car whose masses are grams and whose springs are
catalogued in N/mm. It now follows the display-unit toggle: inch keeps
Milliken's imperial set, mm shows SI with MASS in kg.

The invariant that matters most is that this is a DISPLAY layer. Every
number that leaves the panel -- to `compute()`, to the sweep dialogs, to
a saved file -- is imperial at either setting, and flipping the toggle
must not move a single value. Both are pinned here.
"""

import os
import unittest

from suspension_tool import dyn_units as du
from suspension_tool.dynamics import DynamicsInputs, compute
from suspension_tool.units import UNITS


class TestConversionFactors(unittest.TestCase):
    """All of these are exact by definition, so check them exactly."""

    def test_the_defined_constants(self):
        self.assertEqual(du.LB_TO_KG, 0.45359237)
        self.assertEqual(du.IN_TO_MM, 25.4)
        self.assertEqual(du.FT_TO_MM, 304.8)
        self.assertEqual(du.MPH_TO_KMH, 1.609344)
        self.assertEqual(du.IN2_TO_MM2, 645.16)

    def test_a_pound_force_is_a_pound_mass_times_g(self):
        """The spreadsheet's 'weight in lb' is shown as kg for mass and N
        for force; the two conversions must be consistent through g."""
        g = 9.80665
        self.assertAlmostEqual(du.LB_TO_KG * g, du.LBF_TO_N, places=9)

    def test_rate_conversion_is_force_over_length(self):
        self.assertAlmostEqual(du.LBIN_TO_NMM,
                               du.LBF_TO_N / du.IN_TO_MM, places=15)

    def test_round_trip_is_exact_enough(self):
        for key in du.DIMS:
            d = du.dim(key)
            for v in (0.0, 1.0, 123.456, -7.25):
                back = d.to_imperial(d.to_display(v, True), True)
                self.assertAlmostEqual(back, v, places=9, msg=key)

    def test_imperial_is_a_no_op(self):
        for key in du.DIMS:
            d = du.dim(key)
            self.assertEqual(d.to_display(3.5, False), 3.5)
            self.assertEqual(d.to_imperial(3.5, False), 3.5)

    def test_unknown_keys_are_dimensionless_not_an_error(self):
        d = du.dim("nonsense")
        self.assertEqual(d.unit(True), "")
        self.assertEqual(d.to_display(2.0, True), 2.0)


class TestFormatHelpers(unittest.TestCase):

    def test_shift_fmt_moves_the_decimals(self):
        self.assertEqual(du.shift_fmt("{:.1f}", 2), "{:.3f}")
        self.assertEqual(du.shift_fmt("{:+.2f}", -1), "{:+.1f}")
        self.assertEqual(du.shift_fmt("{:.2f}", 0), "{:.2f}")

    def test_shift_fmt_never_goes_below_zero(self):
        self.assertEqual(du.shift_fmt("{:.1f}", -5), "{:.0f}")

    def test_shift_fmt_passes_through_what_it_cannot_parse(self):
        self.assertEqual(du.shift_fmt("{}", 2), "{}")
        self.assertEqual(du.shift_fmt("{:.1%}", 1), "{:.2%}")

    def test_nice_step_rounds_to_one_two_or_five(self):
        self.assertEqual(du.nice_step(5 * du.LB_TO_KG), 2.0)   # 2.268 kg
        self.assertEqual(du.nice_step(0.5 * 25.4), 10.0)       # 12.7 mm
        self.assertEqual(du.nice_step(0.25 * 25.4), 5.0)       # 6.35 mm

    def test_nice_step_survives_nonsense(self):
        for bad in (0.0, -1.0, float("nan"), float("inf")):
            self.assertGreater(du.nice_step(bad), 0.0)

    def test_widen_rounds_outward_so_nothing_clips(self):
        """Qt rounds a spin box's range to its decimals. If the low end
        rounded UP, a value that is legal in imperial would be clipped."""
        lo, hi = du.widen(0.2224, 8896.4, 2)
        self.assertLessEqual(lo, 0.2224)
        self.assertGreaterEqual(hi, 8896.4)

    def test_widen_leaves_exact_bounds_alone(self):
        self.assertEqual(du.widen(0.0, 3.0, 2), (0.0, 3.0))


class _PanelCase(unittest.TestCase):
    """Shared Qt setup. DynamicsPanel is plain widgets -- no GL, so it can
    be built directly without the MainWindow retry dance."""

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def panel(self, unit_key="in"):
        from suspension_tool.gui.dynamics_panel import DynamicsPanel
        return DynamicsPanel(UNITS[unit_key])


class TestDisplayLayerNeverMovesAValue(_PanelCase):
    """The headline invariant."""

    def test_toggling_units_leaves_every_input_identical(self):
        p = self.panel("in")
        before = p.current_inputs().to_dict()
        p.set_units(UNITS["mm"])
        metric = p.current_inputs().to_dict()
        p.set_units(UNITS["in"])
        after = p.current_inputs().to_dict()
        for k, v in before.items():
            if isinstance(v, float):
                self.assertEqual(metric[k], v, f"{k} moved switching to mm")
                self.assertEqual(after[k], v, f"{k} moved on the round trip")

    def test_ten_round_trips_do_not_drift(self):
        """Values live in the panel's imperial store, not in the spin
        boxes, so repeated switching cannot grind them through the
        display precision."""
        p = self.panel("in")
        start = p.current_inputs().weight_empty_lb
        for _ in range(10):
            p.set_units(UNITS["mm"])
            p.set_units(UNITS["in"])
        self.assertEqual(p.current_inputs().weight_empty_lb, start)

    def test_what_leaves_the_panel_is_imperial_in_metric_mode(self):
        p = self.panel("mm")
        v = p.current_inputs()
        d = DynamicsInputs()
        self.assertAlmostEqual(v.weight_empty_lb, d.weight_empty_lb, places=9)
        self.assertAlmostEqual(v.wheelbase_in, d.wheelbase_in, places=9)


class TestMetricRendering(_PanelCase):

    def test_mass_rows_say_kg_and_call_it_mass(self):
        p = self.panel("in")
        self.assertEqual(p._labels["weight_empty_lb"].text(),
                         "Empty weight (lb)")
        p.set_units(UNITS["mm"])
        self.assertEqual(p._labels["weight_empty_lb"].text(),
                         "Empty mass (kg)")

    def test_the_si_suffixes(self):
        p = self.panel("mm")
        cases = {
            "wheelbase_in": "(mm)",
            "spring1_front_lbin": "(N/mm)",
            "damp_bump_front_lbsin": "(N·s/mm)",
            "arb_rate_front_lbftdeg": "(N·m/deg)",
            "cornering_stiff_front_lbdeg": "(N/deg)",
            "speed_mph": "(km/h)",
            "pedal_force_lb": "(N)",
            "cla_ft2": "(m²)",
            "caliper_area_front_in2": "(mm²)",
            "arb_arm_front_ft": "(mm)",
        }
        for attr, suffix in cases.items():
            self.assertIn(suffix, p._labels[attr].text(), attr)

    def test_the_spin_shows_the_converted_number(self):
        p = self.panel("mm")
        # 63 in wheelbase -> 1600.2 mm
        self.assertAlmostEqual(p._spins["wheelbase_in"].value(), 1600.2,
                               places=1)
        # 400 lb -> 181.44 kg
        self.assertAlmostEqual(p._spins["weight_empty_lb"].value(), 181.437,
                               places=2)

    def test_dimensionless_rows_are_untouched(self):
        p = self.panel("mm")
        self.assertEqual(p._labels["mr_damper_front"].text(),
                         "Front damper MR")
        self.assertAlmostEqual(p._spins["mr_damper_front"].value(), 0.7,
                               places=6)

    def test_results_carry_si_units(self):
        p = self.panel("in")
        self.assertTrue(p._outs["load_front_out_lb"][0].text().endswith("lb"))
        p.set_units(UNITS["mm"])
        for key, suffix in (("load_front_out_lb", "N"),
                            ("wheel_rate_front_lbin", "N/mm"),
                            ("roll_rate_front_lbftdeg", "N·m/deg"),
                            ("roll_moment_arm_in", "mm"),
                            ("sprung_weight_lb", "kg")):
            self.assertTrue(p._outs[key][0].text().endswith(suffix),
                            f"{key}: {p._outs[key][0].text()!r}")

    def test_result_labels_follow_the_noun_swap(self):
        p = self.panel("mm")
        form = p._out_box.layout()
        row = form.labelForField(p._outs["sprung_weight_lb"][0])
        self.assertEqual(row.text(), "Sprung mass")

    def test_unitless_results_gain_no_stray_suffix(self):
        p = self.panel("mm")
        self.assertEqual(p._outs["olley_ratio"][0].text().count(" "), 0)

    def test_a_result_matches_the_hand_conversion(self):
        p = self.panel("in")
        lb = float(p._outs["load_front_out_lb"][0].text().split()[0])
        p.set_units(UNITS["mm"])
        n = float(p._outs["load_front_out_lb"][0].text().split()[0])
        self.assertAlmostEqual(n, lb * du.LBF_TO_N, delta=0.2)


class TestEditingInMetric(_PanelCase):

    def test_typing_kg_stores_pounds(self):
        p = self.panel("mm")
        p._spins["weight_empty_lb"].setValue(3.10)      # a 1/10 buggy
        self.assertAlmostEqual(p.current_inputs().weight_empty_lb,
                               3.10 / du.LB_TO_KG, places=6)

    def test_typing_n_per_mm_stores_lb_per_in(self):
        p = self.panel("mm")
        p._spins["spring1_front_lbin"].setValue(2.25)
        self.assertAlmostEqual(p.current_inputs().spring1_front_lbin,
                               2.25 / du.LBIN_TO_NMM, places=6)

    def test_editing_one_field_does_not_re_round_the_others(self):
        """The spin boxes are a rendering, not the store. Only the field
        that changed is read back."""
        p = self.panel("mm")
        p._imperial["cg_height_in"] = 2.7559055118110236    # exactly 70 mm
        p._render_values(["cg_height_in"])
        p._spins["ay_g"].setValue(0.8)
        self.assertEqual(p.current_inputs().cg_height_in,
                         2.7559055118110236)

    def test_an_rc_scale_mass_survives_the_metric_precision(self):
        """0.07 kg of unsprung mass per corner has to make it through the
        spin box -- rounding it to zero destroys every ride number."""
        p = self.panel("mm")
        p._spins["unsprung_front_lb"].setValue(0.07)
        got = p.current_inputs().unsprung_front_lb * du.LB_TO_KG
        self.assertAlmostEqual(got, 0.07, places=4)

    def test_editing_emits_changed(self):
        p = self.panel("mm")
        seen = []
        p.changed.connect(lambda: seen.append(1))
        p._spins["ay_g"].setValue(0.55)
        self.assertTrue(seen)


class TestRangesCoverBothScales(_PanelCase):

    def test_every_imperial_bound_survives_conversion(self):
        """A value that is legal in inch mode must stay legal in mm mode,
        including at the very ends of the range."""
        p = self.panel("mm")
        for attr, (_lab, dkey, lo, hi, dec, _st) in p._spec.items():
            d = du.dim(dkey)
            s = p._spins[attr]
            self.assertLessEqual(s.minimum(), d.to_display(lo, True) + 1e-9,
                                 f"{attr} minimum clips")
            self.assertGreaterEqual(s.maximum(),
                                    d.to_display(hi, True) - 1e-9,
                                    f"{attr} maximum clips")

    def test_metric_precision_is_never_coarser_than_imperial(self):
        p_in, p_mm = self.panel("in"), self.panel("mm")
        for attr, (_lab, dkey, _lo, _hi, dec, _st) in p_in._spec.items():
            d = du.dim(dkey)
            imp = 10.0 ** -dec                      # imperial quantum
            met = 10.0 ** -p_mm._spins[attr].decimals() / d.factor
            self.assertLessEqual(met, imp * 1.0000001,
                                 f"{attr} loses resolution in metric")

    def test_load_and_save_are_unaffected_by_the_display(self):
        d = {"weight_empty_lb": 6.83, "wheelbase_in": 10.236,
             "unsprung_front_lb": 0.154}
        for key in ("in", "mm"):
            p = self.panel(key)
            p.load_inputs(d)
            got = p.current_inputs().to_dict()
            for k, v in d.items():
                self.assertAlmostEqual(got[k], v, places=6, msg=f"{key}/{k}")

    def test_the_kinematic_link_still_speaks_imperial(self):
        p = self.panel("mm")
        p.set_kinematic({"cg_height_in": 2.76, "wheelbase_in": 10.24})
        self.assertAlmostEqual(p.current_inputs().cg_height_in, 2.76,
                               places=6)
        self.assertAlmostEqual(p._spins["cg_height_in"].value(), 70.1,
                               places=1)


class TestScaleRelativeWarnings(unittest.TestCase):
    """Two dynamics notes carried absolute pound thresholds, so on a 7 lb
    RC car one never fired and the other always did."""

    def _rc_car(self, **kw):
        v = DynamicsInputs(
            weight_empty_lb=6.83, driver_lb=0.0,
            front_axle_lb=3.35, rear_axle_lb=3.48,
            unsprung_front_lb=0.154, unsprung_rear_lb=0.154,
            wheelbase_in=10.24, track_front_in=8.68, track_rear_in=8.68,
            cg_height_in=2.76, loaded_radius_front_in=1.48,
            loaded_radius_rear_in=1.48, ay_g=0.5, ax_g=0.0)
        for k, val in kw.items():
            setattr(v, k, val)
        return v

    def _notes(self, v):
        return " ".join(compute(v).get("notes", []))

    def test_a_healthy_rc_car_is_not_told_it_is_about_to_lift_a_wheel(self):
        """Every corner of this car carries about 1.7 lb, so the old
        50 lb threshold fired at every Ay including zero."""
        self.assertNotIn("lift-off", self._notes(self._rc_car(ay_g=0.5)))

    def test_an_rc_car_that_really_is_near_lift_off_is_told(self):
        """6% of the car's weight on the lightest wheel — the same
        condition that used to be worth 50 lb on a Baja car."""
        self.assertIn("lightest wheel", self._notes(self._rc_car(ay_g=0.9)))

    def test_an_rc_car_that_lifts_a_wheel_still_gets_the_hard_warning(self):
        self.assertIn("WHEEL LIFT-OFF", self._notes(self._rc_car(ay_g=1.6)))

    def test_a_baja_car_near_lift_off_still_warns(self):
        """The threshold moved from absolute to relative, so the case it
        was originally tuned for must be unchanged."""
        self.assertIn("lightest wheel",
                      self._notes(DynamicsInputs(ay_g=0.9, ax_g=0.0)))

    def test_the_axle_sum_check_catches_an_rc_scale_error(self):
        """0.5 lb of slop is 7% of this car -- the old absolute tolerance
        would have let it through."""
        v = self._rc_car(rear_axle_lb=3.9)
        self.assertIn("don't sum", self._notes(v))

    def test_a_consistent_car_passes(self):
        self.assertNotIn("don't sum", self._notes(self._rc_car()))

    def test_the_warnings_carry_no_units(self):
        """They are shown under a panel that may be displaying kg and N."""
        for v in (self._rc_car(rear_axle_lb=3.9), self._rc_car(ay_g=0.9)):
            for note in compute(v).get("notes", []):
                self.assertNotIn(" lb", note)


if __name__ == "__main__":
    unittest.main()
