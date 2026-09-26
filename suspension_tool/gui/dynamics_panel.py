"""Dynamics panel — the Milliken & Milliken spreadsheet, live in the tool.

Two modes (the toggle at the top):
  * Kinematics mode (default): only the handful of numbers a geometry
    designer needs while moving hardpoints — roll gradient, weight
    transfer, corner loads, ride frequencies — driven by the live
    kinematic model. No spring/damper/brake bookkeeping on screen.
  * Full dynamics: every input and output of the spreadsheet (springs,
    motion ratios, dampers, ARBs, tires, brakes, aero, understeer).

"Link kinematics" (default on) pipes the live model in: roll-centre
heights, CG height, wheelbase, motion ratios, and the EXACT kinematic
anti-percentages (replacing the spreadsheet's SVSA approximation).

UNITS follow the toolbar's display-unit toggle. Inch shows the
spreadsheet's imperial set (lb, in, lb/in, lb-ft/deg, mph); mm shows SI
(kg, mm, N, N/mm, N·m/deg, km/h) — see `dyn_units.py`. The MATH never
changes: `current_inputs()` always returns imperial, so `compute()`, the
sweep dialogs and every saved file are identical either way.

The panel keeps its own imperial copy of every field and treats that as
the source of truth. Spin boxes are a rendering of it, never the store —
otherwise switching units would quietly re-round every value through the
display precision each time.
"""

from functools import partial

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox,
                               QFormLayout, QGroupBox, QLabel, QVBoxLayout)

from ..dyn_units import dim as _dim
from ..dyn_units import nice_step, shift_fmt, widen
from ..dynamics import DynamicsInputs, compute
from .widgets import CommitSpin


def _spin(lo, hi, dec, step) -> QDoubleSpinBox:
    s = CommitSpin()
    s.setDecimals(dec)
    s.setRange(lo, hi)
    s.setSingleStep(step)
    return s


# (attr, label, dim, lo, hi, decimals, step) — grouped into sections.
# Bounds and precision are authored in IMPERIAL and converted for display.
# {u} is the unit suffix, {q} the quantity noun (weight/mass).
#
# SCALE: ranges and precision must cover a 1/10 RC buggy (~3.5 lb all up,
# ~0.15 lb unsprung per corner, ~9.8 in track) as well as a full-size Baja
# car, so weights carry 2 decimals — at 0 decimals an RC car's unsprung
# mass rounds to zero and every dynamics result becomes garbage.
SECTIONS = [
    ("Mass & CG", "simple", [
        ("weight_empty_lb", "Empty {q} ({u})", "mass", 0, 10000, 2, 5),
        ("driver_lb", "Driver / payload ({u})", "mass", 0, 500, 2, 5),
        ("front_axle_lb", "Front axle {q} w/ driver ({u})", "mass",
         0, 5000, 2, 5),
        ("rear_axle_lb", "Rear axle {q} w/ driver ({u})", "mass",
         0, 5000, 2, 5),
        ("unsprung_front_lb", "Unsprung front, per side ({u})", "mass",
         0, 500, 3, 1),
        ("unsprung_rear_lb", "Unsprung rear, per side ({u})", "mass",
         0, 500, 3, 1),
        ("cg_height_in", "CG height ({u}) 🔗", "len", 0, 60, 2, 0.5),
        ("wheelbase_in", "Wheelbase ({u}) 🔗", "len", 1, 200, 2, 0.5),
        ("track_front_in", "Front track ({u}) 🔗", "len", 1, 100, 2, 0.5),
        ("track_rear_in", "Rear track ({u}) 🔗", "len", 1, 100, 2, 0.5),
        ("loaded_radius_front_in", "Loaded tire radius F ({u}) 🔗", "len",
         1, 30, 2, 0.25),
        ("loaded_radius_rear_in", "Loaded tire radius R ({u}) 🔗", "len",
         1, 30, 2, 0.25),
        ("roll_radius_gyr_in", "Roll radius of gyration ({u}, EST)", "len",
         1, 100, 2, 0.5),
        ("rc_front_in", "Front RC height ({u}) 🔗", "len", -20, 30, 2, 0.25),
        ("rc_rear_in", "Rear RC height ({u}) 🔗", "len", -20, 30, 2, 0.25),
    ]),
    ("Operating condition", "simple", [
        ("ay_g", "Lateral accel Ay (g)", "", 0, 3, 2, 0.1),
        ("ax_g", "Longitudinal accel Ax (g, + brake)", "", -3, 3, 2, 0.1),
        ("bank_deg", "Road bank (deg)", "", -45, 45, 1, 1),
    ]),
    ("Springs & motion ratios", "full", [
        ("spring1_front_lbin", "Front spring K1 ({u})", "rate",
         0, 3000, 1, 5),
        ("spring2_front_lbin", "Front spring K2, 0=single ({u})", "rate",
         0, 3000, 1, 5),
        ("spring1_rear_lbin", "Rear spring K1 ({u})", "rate", 0, 3000, 1, 5),
        ("spring2_rear_lbin", "Rear spring K2, 0=single ({u})", "rate",
         0, 3000, 1, 5),
        ("mr_spring_front", "Front MR spring/wheel 🔗", "", 0.1, 3, 3, 0.01),
        ("mr_spring_rear", "Rear MR spring/wheel 🔗", "", 0.1, 3, 3, 0.01),
        ("mr_damper_front", "Front damper MR", "", 0.1, 3, 3, 0.01),
        ("mr_damper_rear", "Rear damper MR", "", 0.1, 3, 3, 0.01),
    ]),
    ("Tires", "full", [
        ("tire_rate_front_lbin", "Tire vertical rate F ({u})", "rate",
         0.5, 5000, 1, 10),
        ("tire_rate_rear_lbin", "Tire vertical rate R ({u})", "rate",
         0.5, 5000, 1, 10),
        ("cornering_stiff_front_lbdeg", "Cornering stiffness F ({u})",
         "cstiff", 0.05, 2000, 2, 5),
        ("cornering_stiff_rear_lbdeg", "Cornering stiffness R ({u})",
         "cstiff", 0.05, 2000, 2, 5),
    ]),
    ("Dampers", "full", [
        ("damp_bump_front_lbsin", "Bump coef F ({u})", "damp", 0, 200, 3, 1),
        ("damp_bump_rear_lbsin", "Bump coef R ({u})", "damp", 0, 200, 3, 1),
        ("damp_rebound_front_lbsin", "Rebound coef F ({u})", "damp",
         0, 200, 3, 1),
        ("damp_rebound_rear_lbsin", "Rebound coef R ({u})", "damp",
         0, 200, 3, 1),
    ]),
    ("Anti-roll bars", "full", [
        ("arb_rate_front_lbftdeg", "Front ARB rate, 0=none ({u})",
         "rollrate", 0, 2000, 1, 5),
        ("arb_rate_rear_lbftdeg", "Rear ARB rate, 0=none ({u})",
         "rollrate", 0, 2000, 1, 5),
        ("arb_arm_front_ft", "Front ARB lever arm ({u})", "len_ft",
         0.01, 3, 3, 0.05),
        ("arb_arm_rear_ft", "Rear ARB lever arm ({u})", "len_ft",
         0.01, 3, 3, 0.05),
        ("arb_ir_front", "Front ARB install ratio", "", 0.05, 2, 3, 0.05),
        ("arb_ir_rear", "Rear ARB install ratio", "", 0.05, 2, 3, 0.05),
    ]),
    ("Aero", "full", [
        ("speed_mph", "Speed ({u})", "speed", 0, 200, 0, 5),
        ("cla_ft2", "CL·A, 0=no aero ({u})", "area_l", 0, 20, 2, 0.1),
        ("aero_front_frac", "Front aero balance (0-1)", "", 0, 1, 2, 0.05),
    ]),
    ("Brakes", "full", [
        ("brake_bias_front", "Brake bias, front fraction", "", 0, 1, 2, 0.05),
        ("pedal_force_lb", "Pedal force ({u})", "force", 0, 500, 0, 10),
        ("pedal_ratio", "Pedal ratio", "", 1, 10, 3, 0.1),
        ("mc_bore_front_in", "MC bore F ({u})", "len", 0.1, 2, 3, 0.0625),
        ("mc_bore_rear_in", "MC bore R ({u})", "len", 0.1, 2, 3, 0.0625),
        ("caliper_area_front_in2", "Caliper piston area F ({u})", "area_s",
         0.1, 20, 2, 0.1),
        ("caliper_area_rear_in2", "Caliper piston area R ({u})", "area_s",
         0.1, 20, 2, 0.1),
        ("rotor_radius_front_in", "Effective rotor radius F ({u})", "len",
         0.5, 10, 3, 0.125),
        ("rotor_radius_rear_in", "Effective rotor radius R ({u})", "len",
         0.5, 10, 3, 0.125),
        ("pad_mu_front", "Pad friction μ F", "", 0.05, 1, 2, 0.02),
        ("pad_mu_rear", "Pad friction μ R", "", 0.05, 1, 2, 0.02),
    ]),
    ("Anti-geometry (SVSA, used only when kinematics unlinked)", "full", [
        ("svsa_height_front_in", "Front SVSA height ({u})", "len",
         0, 30, 2, 0.25),
        ("svsa_length_front_in", "Front SVSA length ({u})", "len",
         1, 200, 1, 1),
        ("svsa_height_rear_in", "Rear SVSA height ({u})", "len",
         0, 30, 2, 0.25),
        ("svsa_length_rear_in", "Rear SVSA length ({u})", "len",
         1, 200, 1, 1),
    ]),
]

# Fields overwritten by the live kinematic model while linked.
LINKED_FIELDS = ("cg_height_in", "wheelbase_in", "rc_front_in",
                 "rc_rear_in", "mr_spring_front", "mr_spring_rear",
                 "track_front_in", "track_rear_in",
                 "loaded_radius_front_in", "loaded_radius_rear_in")

# (key, label, format, dim) output rows. The format carries no unit — the
# dimension supplies both the suffix and the metric decimal shift.
# Corner loads run to 1 decimal because an RC car's are single-digit pounds.
OUT_SIMPLE = [
    ("roll_gradient_deg_g", "Roll gradient", "{:+.2f}", "deg_g"),
    ("body_roll_at_ay_deg", "Body roll @ Ay", "{:+.2f}", "deg"),
    ("roll_moment_arm_in", "Roll moment arm H", "{:.2f}", "len"),
    ("ride_freq_front_hz", "Ride frequency front", "{:.2f}", "hz"),
    ("ride_freq_rear_hz", "Ride frequency rear", "{:.2f}", "hz"),
    ("olley_ratio", "Olley ratio fR/fF", "{:.3f}", ""),
    ("lat_transfer_front_lb", "Lateral transfer front", "{:.1f}", "force"),
    ("lat_transfer_rear_lb", "Lateral transfer rear", "{:.1f}", "force"),
    ("tlltd_front", "Lateral LT distribution (front)", "{:.1%}", ""),
    ("load_front_out_lb", "Front outside wheel", "{:.1f}", "force"),
    ("load_front_in_lb", "Front inside wheel", "{:.1f}", "force"),
    ("load_rear_out_lb", "Rear outside wheel", "{:.1f}", "force"),
    ("load_rear_in_lb", "Rear inside wheel", "{:.1f}", "force"),
    ("anti_squat_rear_pct", "Anti-squat rear", "{:+.1f}", "pct"),
    ("anti_dive_front_pct", "Anti-dive front", "{:+.1f}", "pct"),
    ("understeer_grad_deg_g", "Understeer gradient", "{:+.2f}", "deg_g"),
]
OUT_FULL = [
    ("sprung_weight_lb", "Sprung {q}", "{:.1f}", "mass"),
    ("sprung_cg_height_in", "Sprung CG height", "{:.2f}", "len"),
    ("wheel_rate_front_lbin", "Wheel rate front", "{:.1f}", "rate"),
    ("wheel_rate_rear_lbin", "Wheel rate rear", "{:.1f}", "rate"),
    ("ride_rate_front_lbin", "Ride rate front", "{:.1f}", "rate"),
    ("ride_rate_rear_lbin", "Ride rate rear", "{:.1f}", "rate"),
    ("zeta_bump_front", "Damping ratio ζ bump F", "{:.2f}", ""),
    ("zeta_bump_rear", "Damping ratio ζ bump R", "{:.2f}", ""),
    ("zeta_rebound_front", "Damping ratio ζ rebound F", "{:.2f}", ""),
    ("zeta_rebound_rear", "Damping ratio ζ rebound R", "{:.2f}", ""),
    ("roll_rate_front_lbftdeg", "Roll rate front", "{:.1f}", "rollrate"),
    ("roll_rate_rear_lbftdeg", "Roll rate rear", "{:.1f}", "rollrate"),
    ("roll_rate_front_frac", "Roll-rate distribution (front)", "{:.1%}", ""),
    ("long_transfer_lb", "Longitudinal transfer", "{:.1f}", "force"),
    ("bounce_freq_hz", "Bounce frequency", "{:.2f}", "hz"),
    ("pitch_freq_hz", "Pitch frequency", "{:.2f}", "hz"),
    ("brake_decel_g", "Brake chain deceleration", "{:.2f}", "g"),
    ("anti_lift_rear_pct", "Anti-lift rear (braking)", "{:+.1f}", "pct"),
]


class DynamicsPanel(QGroupBox):
    """All inputs + all outputs, recomputed live (pure math, instant)."""

    changed = Signal()   # inputs edited (project is dirty)

    def __init__(self, unit=None, parent=None):
        super().__init__("Dynamics (Milliken / RCVD)", parent)
        self._metric = getattr(unit, "key", "in") == "mm"
        self.inputs = DynamicsInputs()
        self._kin: dict = {}          # live kinematic link data
        self._loading = False
        lay = QVBoxLayout(self)

        self.full_check = QCheckBox("Full dynamics mode (all inputs)")
        self.full_check.setToolTip(
            "Off: just the key numbers for geometry work. On: the whole "
            "spreadsheet — springs, dampers, ARBs, tires, brakes, aero.")
        self.full_check.toggled.connect(self._apply_mode)
        lay.addWidget(self.full_check)
        self.link_check = QCheckBox("Link kinematics (RC, CG, MR, anti-%)")
        self.link_check.setChecked(True)
        self.link_check.setToolTip(
            "Pull roll-centre heights, CG height, wheelbase, motion "
            "ratios and the exact anti-percentages from the live model "
            "(fields marked 🔗). Untick to type your own.")
        self.link_check.toggled.connect(self._on_link_toggled)
        lay.addWidget(self.link_check)

        # attr -> (label template, dim key, lo, hi, decimals, step), all
        # imperial. The panel converts for display and back on edit.
        self._spec: dict[str, tuple] = {}
        self._imperial: dict[str, float] = {}
        self._spins: dict[str, QDoubleSpinBox] = {}
        self._labels: dict[str, QLabel] = {}
        self._sections: list[tuple[QGroupBox, str]] = []
        for title, mode, fields in SECTIONS:
            box = QGroupBox(title)
            form = QFormLayout(box)
            for attr, label, dkey, lo, hi, dec, step in fields:
                self._spec[attr] = (label, dkey, lo, hi, dec, step)
                self._imperial[attr] = float(getattr(self.inputs, attr))
                s = _spin(lo, hi, dec, step)
                s.valueChanged.connect(partial(self._on_edit, attr))
                self._spins[attr] = s
                lbl = QLabel(label)
                self._labels[attr] = lbl
                form.addRow(lbl, s)
            if title.startswith("Springs"):
                for end in ("front", "rear"):
                    c = QComboBox()
                    c.addItems(["Series", "Parallel", "Single"])
                    c.currentTextChanged.connect(self._on_stack_changed)
                    setattr(self, f"stack_{end}_combo", c)
                    form.addRow(QLabel(f"{end.title()} stack mode"), c)
            lay.addWidget(box)
            self._sections.append((box, mode))

        self._out_box = QGroupBox("Results")
        out_form = QFormLayout(self._out_box)
        self._outs: dict[str, tuple[QLabel, str, str, str, str]] = {}
        for key, label, fmt, dkey in OUT_SIMPLE + OUT_FULL:
            val_lbl = QLabel("-")
            mode = ("simple" if (key, label, fmt, dkey) in OUT_SIMPLE
                    else "full")
            self._outs[key] = (val_lbl, label, fmt, dkey, mode)
            out_form.addRow(QLabel(label), val_lbl)
        lay.addWidget(self._out_box)
        self.notes_label = QLabel("")
        self.notes_label.setWordWrap(True)
        lay.addWidget(self.notes_label)

        self._apply_units()
        self._apply_mode()
        self._apply_link_state()
        self.recompute()

    # ------------------------------------------------------------------
    # Units
    # ------------------------------------------------------------------
    def set_units(self, unit) -> None:
        """Follow the toolbar toggle: mm → SI, in → the imperial set."""
        metric = getattr(unit, "key", "in") == "mm"
        if metric == self._metric:
            return
        self._metric = metric
        self._apply_units()
        self.recompute()

    def _apply_units(self) -> None:
        """Re-label, re-range and re-render every row for the current
        system. The stored imperial values are untouched, so flipping the
        toggle can never move a number."""
        m = self._metric
        for attr, (label, dkey, lo, hi, dec, step) in self._spec.items():
            d = _dim(dkey)
            dec_d = d.decimals(dec, m)
            lo_d, hi_d = d.to_display(lo, m), d.to_display(hi, m)
            if m:
                lo_d, hi_d = widen(lo_d, hi_d, dec_d)
            s = self._spins[attr]
            s.blockSignals(True)
            s.setDecimals(dec_d)
            s.setRange(lo_d, hi_d)
            s.setSingleStep(nice_step(d.to_display(step, m)) if m else step)
            s.blockSignals(False)
            self._labels[attr].setText(
                label.format(u=d.unit(m), q=d.noun(m)))
        self._render_values()
        form = self._out_box.layout()
        for val_lbl, label, _fmt, dkey, _mode in self._outs.values():
            row = form.labelForField(val_lbl)
            if row is not None:
                row.setText(label.format(q=_dim(dkey).noun(m)))

    def _render_values(self, attrs=None) -> None:
        """Push the stored imperial values onto the spin boxes."""
        for attr in (attrs if attrs is not None else self._spec):
            d = _dim(self._spec[attr][1])
            s = self._spins[attr]
            s.blockSignals(True)
            s.setValue(d.to_display(self._imperial[attr], self._metric))
            s.blockSignals(False)

    # ------------------------------------------------------------------
    def _apply_mode(self, *_):
        full = self.full_check.isChecked()
        for box, mode in self._sections:
            box.setVisible(full or mode == "simple")
        form = self._out_box.layout()
        for val_lbl, _label, _fmt, _dkey, mode in self._outs.values():
            row = form.labelForField(val_lbl)
            show = full or mode == "simple"
            val_lbl.setVisible(show)
            if row is not None:
                row.setVisible(show)

    def _on_link_toggled(self, *_):
        self._apply_link_state()
        self.recompute()
        self.changed.emit()

    def _apply_link_state(self):
        linked = self.link_check.isChecked()
        for attr in LINKED_FIELDS:
            self._spins[attr].setEnabled(not linked)
        if linked:
            self._push_kin_values()

    def _push_kin_values(self):
        """Copy the live kinematic values into their (disabled) fields.
        The link always speaks imperial, whatever the display shows."""
        changed = [a for a in LINKED_FIELDS
                   if self._kin.get(a) is not None]
        for attr in changed:
            self._imperial[attr] = float(self._kin[attr])
        self._render_values(changed)

    def _on_edit(self, attr, value):
        """One field edited. Only that field is read back, so the others
        keep full imperial precision instead of being re-rounded through
        the display every time anything moves."""
        if self._loading:
            return
        d = _dim(self._spec[attr][1])
        self._imperial[attr] = d.to_imperial(float(value), self._metric)
        self.recompute()
        self.changed.emit()

    def _on_stack_changed(self, *_):
        if not self._loading:
            self.recompute()
            self.changed.emit()

    # ------------------------------------------------------------------
    def set_kinematic(self, data: dict) -> None:
        """Live link from the model. Keys: any of LINKED_FIELDS (imperial)
        plus 'anti' ({anti_squat_rear_pct, ...}) or None."""
        self._kin = data or {}
        if self.link_check.isChecked():
            self._push_kin_values()
            self.recompute()

    def current_inputs(self) -> DynamicsInputs:
        """Always imperial — the contract every other module relies on."""
        v = DynamicsInputs()
        for attr, val in self._imperial.items():
            setattr(v, attr, val)
        v.stack_front = self.stack_front_combo.currentText()
        v.stack_rear = self.stack_rear_combo.currentText()
        return v

    def load_inputs(self, d: dict | None, linked: bool = True) -> None:
        """Restore from a saved project (imperial, at either unit setting)."""
        self._loading = True
        self.inputs = DynamicsInputs.from_dict(d or {})
        for attr in self._spec:
            self._imperial[attr] = float(getattr(self.inputs, attr))
        self._render_values()
        self.stack_front_combo.setCurrentText(self.inputs.stack_front)
        self.stack_rear_combo.setCurrentText(self.inputs.stack_rear)
        self.link_check.setChecked(linked)
        self._loading = False
        self._apply_link_state()
        self.recompute()

    # ------------------------------------------------------------------
    def recompute(self) -> None:
        v = self.current_inputs()
        anti = (self._kin.get("anti")
                if self.link_check.isChecked() else None)
        out = compute(v, kinematic_anti=anti)
        self._last = out
        m = self._metric
        for key, (val_lbl, _label, fmt, dkey, _mode) in self._outs.items():
            val = out.get(key)
            d = _dim(dkey)
            try:
                if val is None:
                    val_lbl.setText("-")
                else:
                    txt = shift_fmt(fmt, d.dec_shift if m else 0).format(
                        d.to_display(float(val), m))
                    val_lbl.setText(f"{txt} {d.unit(m)}".rstrip())
            except (ValueError, TypeError):
                val_lbl.setText("-")
        notes = list(out.get("notes", []))
        notes.append(f"Anti-%: {out.get('anti_source', '-')}")
        notes.append(out.get("balance_note", ""))
        notes.append(out.get("understeer_note", ""))
        notes.append(out.get("tuning_note", ""))
        self.notes_label.setText("\n".join(n for n in notes if n))
