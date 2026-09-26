"""Left-panel widgets: setup variables (seed generator input), the
editable hardpoint table, and the live metric readouts.

Everything is stored internally in mm and degrees; these widgets convert
to/from the project's current display unit (see suspension_tool.units).
Field "kind" drives the conversion: "len" lengths convert, "ang" angles
and "ratio" dimensionless values never do.
"""

import dataclasses

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QPushButton, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from ..geometry import DoubleWishbonePoints
from ..optimize import GOAL_SPECS
from ..seed import IN, SetupVariables
from ..tweaks import (set_shock_mount, set_tierod_rises, shock_mount_params,
                      tierod_rises)
from ..units import UNITS, Unit
from .widgets import CommitSpin, commit_spins

# (attribute, label) for every editable 3D hardpoint, in display order.
POINT_FIELDS = [
    ("uca_inner_front", "UCA inner front"),
    ("uca_inner_rear", "UCA inner rear"),
    ("uca_outer", "UCA outer (UBJ)"),
    ("lca_inner_front", "LCA inner front"),
    ("lca_inner_rear", "LCA inner rear"),
    ("lca_outer", "LCA outer (LBJ)"),
    ("tierod_inner", "Tie rod inner"),
    ("tierod_outer", "Tie rod outer"),
    ("wheel_center", "Wheel centre"),
    ("shock_inner", "Shock inner"),
    ("shock_outer", "Shock outer"),
]
# Hardpoint-table sizing. COL_MIN_W fits a signed 4-digit value ("-1234.56")
# AND the widest column caption, so no heading elides at the dock's normal
# width; ROW_LABEL_MAX_W stops the point-name header from eating the panel
# (it self-sized to ~111 px of a 358 px dock, starving all four columns).
COL_MIN_W = 78
ROW_LABEL_MAX_W = 88

# (attribute, base label, kind, lo, hi) — lo/hi are in mm for "len".
# Lower bounds are deliberately permissive: the tool is used at 1/10 RC
# scale as well as full-size Baja, so every length range must reach down
# to model-car dimensions (see SetupForm.SPECS).
SCALAR_FIELDS = [
    ("tire_radius", "Tire radius", "len", 5.0, 600.0),
    ("tire_width", "Tire width", "len", 3.0, 400.0),
    ("static_camber_deg", "Static camber", "ang", -10.0, 10.0),
    ("static_toe_deg", "Static toe", "ang", -5.0, 5.0),
]


def _spin(lo, hi, decimals, step) -> QDoubleSpinBox:
    # CommitSpin: waits for Enter/focus-out, selects on click. See widgets.py.
    s = CommitSpin()
    s.setRange(lo, hi)
    s.setDecimals(decimals)
    s.setSingleStep(step)
    return s


class SetupForm(QGroupBox):
    """Setup variables + 'Generate seed'. Defaults are round numbers for a
    plausible full-size Baja car, not any particular vehicle."""

    generate_requested = Signal(object)  # SetupVariables
    measure_requested = Signal()         # fill the form FROM the design

    # (name, label, kind, lo, hi, default) — lengths in mm.
    # SCALE: these ranges must span 1/10 RC buggy AND full-size Baja, so the
    # lower bounds are set from model-car dimensions (a 1/10 buggy runs
    # ~250 mm track, ~285 mm wheelbase, ~30 mm tire radius, ~65 mm shocks),
    # not from Baja values. Defaults stay full-size, and are deliberately
    # ROUND numbers for a plausible Baja car rather than any particular
    # vehicle's spec -- these are the first values a new user sees, and a
    # real car's measurements sitting here read as a recommendation. They
    # match main_window.DEFAULT_SETUP; change both together.
    SPECS = [
        ("track_width", "Track width", "len", 50, 2500, 60.0 * IN),
        ("wheelbase", "Wheelbase", "len", 50, 3000, 60.0 * IN),
        ("ride_height", "Ride height", "len", 2, 600, 12.0 * IN),
        ("tire_radius", "Tire radius", "len", 5, 500, 11.5 * IN),
        ("tire_width", "Tire width", "len", 3, 400, 7.0 * IN),
        ("shock_min_length", "Shock min length", "len", 10, 800, 14.0 * IN),
        ("shock_max_length", "Shock max length", "len", 12, 900, 22.0 * IN),
        ("shock_length_at_ride", "Shock length @ ride", "len", 10, 900, 19.0 * IN),
        ("shock_on_uca", "Shock on upper arm", "bool", 0, 0, False),
        ("tierod_on_lca", "Steering link on LOWER arm (fixed-toe rear)",
         "bool", 0, 0, False),
        ("tierod_on_uca", "Steering link on UPPER arm (fixed-toe rear)",
         "bool", 0, 0, False),
        ("has_halfshaft", "Driven axle: model halfshaft", "bool", 0, 0, False),
        ("motion_ratio_goal", "Motion ratio goal (shock/wheel)", "ratio", 0.15, 1.0, 0.55),
        ("hub_offset", "Hub offset (WC → BJ plane, seed)", "len", 1, 200, 60.0),
        ("desired_scrub_radius", "Desired scrub radius", "len", -120, 120, 30.0),
        ("desired_caster_deg", "Desired caster (design frame)", "ang", -5, 15, 4.0),
        ("steering_arm_x", "Steering arm fore/aft (- = rack behind axle)",
         "len", -400, 400, -80.0),
        ("steering_arm_plane_deg", "Steering-arm plane angle (to tire plane)",
         "ang", 45, 135, 90.0),
        ("independent_caster", "Independent caster (allow kingpin twist)",
         "bool", 0, 0, False),
        # Max 30 deg: RC buggies run far more kickup than a Baja car (the
        # B7 class is up in the 25-30 deg range), and the seed maths has no
        # clamp of its own — this spin range was the only limit.
        ("kickup_deg", "Chassis kickup", "ang", -20, 30, 8.0),
        ("sketch_yaw_deg", "Sketch plane yaw (angled frame tubes)",
         "ang", -45, 45, 0.0),
        ("inboard_sep_frac", "Inboard axis sep (frac of kingpin)", "ratio", 0.1, 0.8, 0.4),
        ("inboard_sep", "Inboard axis gap (overrides frac)", "len0", 0, 400, 0.0),
        ("kingpin_length", "Kingpin length (LBJ-UBJ span)", "len0", 0, 500, 0.0),
        ("static_camber_deg", "Static camber", "ang", -10, 10, -1.0),
        ("static_toe_deg", "Static toe", "ang", -5, 5, 0.0),
        # Rigid translation of every seeded hardpoint so the coordinates
        # land on the team's Onshape frame origin (kinematics unchanged);
        # the live "Ride height" tweak drives the z stance afterwards.
        ("x_offset", "Initial seed placement X (+fwd)", "len", -3000, 3000, 39.0 * IN),
        ("z_offset", "Initial seed placement Z (+up)", "len", -3000, 3000, -14.0 * IN),
        ("frame_tube_offset", "Frame-tube offset (ride-height datum)",
         "len", -200, 200, 0.625 * IN),
    ]

    def __init__(self, unit: Unit, parent=None):
        super().__init__("Setup variables (seed generator)", parent)
        self._unit = unit
        form = QFormLayout(self)
        self._fields = {}     # name -> spinbox
        self._kinds = {}      # name -> kind
        self._labels = {}     # name -> QLabel (to retitle with the unit)
        self._base = {}       # name -> base label text
        self._values_mm = {}  # name -> value (mm for len, native otherwise)
        for name, label, kind, lo, hi, default in self.SPECS:
            self._kinds[name] = kind
            self._base[name] = label
            self._values_mm[name] = default
            if kind == "ang":
                spin = _spin(lo, hi, 2, 0.1)
            elif kind == "ratio":
                spin = _spin(lo, hi, 3, 0.05)
            elif kind == "bool":
                spin = QCheckBox()
                spin.setChecked(bool(default))
            else:                       # "len" / "len0": range set per unit
                spin = _spin(0, 0, 0, 0)
            self._fields[name] = spin
            lbl = QLabel(label)
            self._labels[name] = lbl
            form.addRow(lbl, spin)
        # Lower / upper arm mounts are mutually exclusive (chassis = both off)
        self._fields["tierod_on_lca"].toggled.connect(
            lambda on: on and self._fields["tierod_on_uca"].setChecked(False))
        self._fields["tierod_on_uca"].toggled.connect(
            lambda on: on and self._fields["tierod_on_lca"].setChecked(False))
        self._apply_unit(unit)
        btn = QPushButton("Generate seed hardpoints")
        btn.clicked.connect(self._emit)
        form.addRow(btn)
        # Reseed WITHOUT losing the design: measure every setup value off
        # the current (or a loaded candidate's) geometry, then Generate
        # reproduces it — so a seed-only option can be flipped in place.
        measure_btn = QPushButton("Measure setup from current design")
        measure_btn.setToolTip(
            "Fill this form from the ACTIVE axle's live geometry (track, "
            "scrub, caster, kingpin, arm gap, motion ratio, placement…). "
            "Then change the one seed option you need and Generate — the "
            "reseed lands on the measured design instead of the defaults. "
            "Load a candidate first to measure that instead.")
        measure_btn.clicked.connect(self.measure_requested.emit)
        form.addRow(measure_btn)

    def _apply_unit(self, unit: Unit) -> None:
        """Show every field in `unit` without changing the stored mm values."""
        for name, label, kind, lo, hi, _ in self.SPECS:
            spin = self._fields[name]
            spin.blockSignals(True)
            if kind in ("len", "len0"):
                spin.setRange(unit.from_mm(lo), unit.from_mm(hi))
                spin.setDecimals(unit.decimals)
                spin.setSingleStep(unit.step)
                spin.setSuffix(f" {unit.label}")
                if kind == "len0":      # minimum (0) displays as "auto"
                    spin.setSpecialValueText("auto")
                spin.setValue(unit.from_mm(self._values_mm[name] or 0.0))
                self._labels[name].setText(self._base[name])
            elif kind == "bool":
                spin.setChecked(bool(self._values_mm[name]))
            else:
                suffix = " deg" if kind == "ang" else ""
                spin.setSuffix(suffix)
                spin.setValue(self._values_mm[name])
            spin.blockSignals(False)

    def set_units(self, unit: Unit) -> None:
        # Capture what's on screen in the OLD unit before switching.
        self._capture()
        self._unit = unit
        self._apply_unit(unit)

    def load_setup(self, sv: SetupVariables) -> None:
        """Populate the form from a SetupVariables (e.g. an opened project)."""
        _missing = object()
        for name, kind in self._kinds.items():
            v = getattr(sv, name, _missing)
            if v is _missing:
                continue
            if v is None:               # optional field on "auto"
                v = 0.0 if kind == "len0" else self._values_mm[name]
            self._values_mm[name] = v
        self._apply_unit(self._unit)

    def _capture(self) -> None:
        for name, kind in self._kinds.items():
            f = self._fields[name]
            if kind == "bool":
                self._values_mm[name] = f.isChecked()
            elif kind in ("len", "len0"):
                self._values_mm[name] = self._unit.to_mm(f.value())
            else:
                self._values_mm[name] = f.value()

    def current_setup(self) -> SetupVariables:
        """The SetupVariables currently shown in the form."""
        self._capture()
        kw = dict(self._values_mm)
        for name, kind in self._kinds.items():
            if kind == "len0" and kw[name] <= 0.0:
                kw[name] = None         # "auto": let the seed's rule decide
        return SetupVariables(**kw)

    def set_sketch_yaw(self, deg: float) -> None:
        """Adopt a yaw chosen elsewhere (the tweaks panel re-squaring),
        without firing the seed."""
        spin = self._fields.get("sketch_yaw_deg")
        if spin is None:
            return
        self._values_mm["sketch_yaw_deg"] = float(deg)
        spin.blockSignals(True)
        spin.setValue(float(deg))
        spin.blockSignals(False)

    def _emit(self) -> None:
        # The spins defer their commit until focus moves, and a button with
        # a NoFocus policy never takes focus — so force any half-typed field
        # to interpret itself before the seed reads it.
        commit_spins(self)
        self.generate_requested.emit(self.current_setup())


class HardpointTable(QGroupBox):
    """Editable x/y/z table of every hardpoint, plus the scalar fields.
    This is the manual-tuning loop: any edit re-solves everything."""

    edited = Signal(object)  # DoubleWishbonePoints
    halfshaft_inner_edited = Signal(object)   # new inner CV point (mm)
    halfshaft_outer_edited = Signal(object)   # requested outer CV point
    locks_changed = Signal(object)   # new frozenset of locked point attrs
    note = Signal(str)               # why an edit or a lock did nothing

    def __init__(self, unit: Unit, parent=None):
        super().__init__("Hardpoints (edit to tune)", parent)
        from ..axes import CONVENTIONS, DEFAULT_CONVENTION
        self._hp = None
        self._unit = unit
        self._conv = CONVENTIONS[DEFAULT_CONVENTION]
        self._hs_provider = lambda: None   # main window injects this
        self._design_yaw_provider = None   # main window injects this
        # The knuckle convention's steering-arm plane angle. Must match the
        # one `apply_hardpoints` uses, or the derived-coordinate warning
        # below would fire on geometry that is actually fine.
        self._arm_plane_provider = None
        self._refreshing = False
        self._locked = set()               # pinned point attrs (persisted)
        # Fore-aft (tool +X) shift into the SHARED VEHICLE FRAME so the
        # coordinates read/entered here match the CSV/Onshape export and
        # the 3D scene: 0 for the front axle, -wheelbase for the rear, so
        # both axles reference ONE origin (the front/firewall datum). Set
        # by the main window per active axle. Storage stays axle-local.
        self._veh_dx = 0.0
        lay = QVBoxLayout(self)

        self.table = QTableWidget(len(POINT_FIELDS), 4)
        self.table.setAlternatingRowColors(True)
        self.table.setVerticalHeaderLabels([label for _, label in POINT_FIELDS])
        self.table.cellChanged.connect(self._on_cell)
        # Double-click a row's name to pin/unpin that hardpoint: a locked
        # point won't drag and scrub/caster tweaks refuse to move it, so
        # fixed halfshaft mounts and chassis points you like stay put.
        self.table.verticalHeader().sectionDoubleClicked.connect(
            self._toggle_lock)
        self.table.verticalHeader().setToolTip(
            "Double-click a point's name to lock/unlock it (🔒 = pinned: "
            "won't drag, tweaks won't move it)")
        self.table.setMinimumHeight(360)
        # ALL FOUR columns (X, Y, Z and ⊥ sketch) must stay readable at the
        # dock's normal width. Three things gang up to squeeze them:
        #   * the row-name header is greedy (it sized itself to ~111 px,
        #     nearly a third of the panel), so cap it — the full name is
        #     still one hover away, and the tooltip carries it;
        #   * Stretch alone divides whatever is left FOUR ways, which fell
        #     to ~55 px per column — narrower than the header captions, so
        #     every heading elided to "…". A minimum section size keeps a
        #     column wide enough for a signed 4-digit value plus its
        #     caption, and Stretch still shares any SURPLUS width evenly;
        #   * with the h-scrollbar forced off, anything that overflowed was
        #     simply unreachable (the "can't see all four columns" bug).
        #     AsNeeded means a too-narrow dock scrolls instead of hiding.
        from PySide6.QtWidgets import QHeaderView
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.Stretch)
        hh.setMinimumSectionSize(COL_MIN_W)
        self.table.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        vh = self.table.verticalHeader()
        vh.setDefaultSectionSize(26)
        vh.setMaximumWidth(ROW_LABEL_MAX_W)
        vh.setTextElideMode(Qt.ElideRight)
        # Ask for enough width that the dock opens with all four columns
        # already legible instead of needing a drag.
        self.table.setMinimumWidth(ROW_LABEL_MAX_W + 4 * COL_MIN_W + 8)

        # BATCH EDIT. Live editing re-solves on every cell, and a set of
        # coordinates that is only half typed can easily describe geometry
        # that does not close -- at which point the whole edit is rejected
        # and the table snaps back, taking the value you just typed with
        # it. Moving a point 40 mm in X and 40 mm in Z is enough to do it:
        # the X-only intermediate is not a real design. Batch mode holds
        # every cell until Apply, so the linkage is only ever asked about
        # complete, intentional geometry.
        bar = QHBoxLayout()
        self.batch_check = QCheckBox("Batch edit")
        self.batch_check.setToolTip(
            "Hold every cell until Apply, so a half-entered set of "
            "coordinates is never solved. Unticking applies whatever is "
            "still pending.")
        self.batch_check.toggled.connect(self._on_batch_toggled)
        bar.addWidget(self.batch_check)
        self.apply_btn = QPushButton("Apply")
        self.apply_btn.setToolTip("Solve the whole table at once")
        self.apply_btn.clicked.connect(self.apply_batch)
        bar.addWidget(self.apply_btn)
        self.revert_btn = QPushButton("Revert")
        self.revert_btn.setToolTip("Throw away every pending cell")
        self.revert_btn.clicked.connect(self.revert_batch)
        bar.addWidget(self.revert_btn)
        bar.addStretch(1)
        lay.addLayout(bar)
        self._batch = False
        self._pending: dict[tuple[int, int], str] = {}
        self._last_batch: dict[tuple[int, int], str] = {}
        self._set_batch_state()

        lay.addWidget(self.table)
        # Why an edit was refused. The status bar carries this too, but it
        # is at the far corner of a big window and it times out -- which is
        # most of the reason a rejected edit reads as "the table ignored me".
        self.edit_status = QLabel("")
        self.edit_status.setWordWrap(True)
        lay.addWidget(self.edit_status)

        form = QFormLayout()
        self._scalars = {}    # name -> spinbox
        self._scalar_kind = {}
        self._scalar_label = {}
        for name, label, kind, lo, hi in SCALAR_FIELDS:
            self._scalar_kind[name] = kind
            if kind == "ang":
                s = _spin(lo, hi, 2, 0.1)
                s.setSuffix(" deg")
            else:
                s = _spin(unit.from_mm(lo), unit.from_mm(hi), unit.decimals, unit.step)
            s.valueChanged.connect(self._on_scalar)
            self._scalars[name] = s
            lbl = QLabel(label)
            self._scalar_label[name] = (lbl, label)
            form.addRow(lbl, s)
        lay.addLayout(form)
        # Manufacturability readout: can the arms still be built from ONE
        # 2D sketch with the bushing tubes normal to it? Goes red past
        # tolerance; the optimizer also pulls this back to zero.
        self.sketch_fit = QLabel("")
        lay.addWidget(self.sketch_fit)
        self._apply_unit_headers()

    def _update_sketch_fit(self, hp) -> None:
        if not isinstance(hp, DoubleWishbonePoints):
            self.sketch_fit.setText("")
            return
        from ..geometry import sketch_planarity
        p = sketch_planarity(hp)
        ang, kick = p["axis_misalign_deg"], p["sketch_kickup_deg"]
        yaw = p["axis_yaw_deg"]
        kp_off = p["kingpin_off_plane_deg"]
        want = 0.0
        if self._design_yaw_provider is not None:
            want = float(self._design_yaw_provider() or 0.0)
        dev = abs(yaw - want)
        ok = ang <= 0.1 and dev <= 0.25
        color = "#2a9d2a" if ok else "#d03030"
        # kingpin twist is a NOTE (planarizing forces caster~=kickup — a
        # design trade-off), not a pass/fail, so a fresh seed still reads
        # OK. Show the reconciliation so the number is checkable against
        # the caster/kickup the designer typed: off-plane ~= caster −
        # kickup ONLY on an un-yawed sketch — with sketch yaw the
        # kingpin's KPI lean also tilts it relative to the plane, so the
        # measured 3D angle legitimately differs from that subtraction.
        kp_note = ""
        if kp_off > 0.5:
            from ..tweaks import caster_angle_deg
            cast = caster_angle_deg(hp)
            why = (f"caster {cast:.1f} vs kickup {kick:.1f} deg"
                   if abs(yaw) <= 0.5 else
                   f"caster {cast:.1f} deg, kickup {kick:.1f} deg at "
                   f"{yaw:.0f} deg yaw — KPI tilts it too, so it is NOT "
                   "simply caster−kickup")
            kp_note = (f"; kingpin {kp_off:.1f} deg off sketch ({why}; "
                       "Re-square planarizes)")
        if ok:
            twist = f" twisted {yaw:.1f} deg" if abs(want) > 0.25 else ""
            msg = (f"Sketch fit OK — one 2D sketch at {kick:.1f} deg "
                   f"kickup{twist}{kp_note}")
        elif ang > 0.1:
            msg = (f"Bushing axes {ang:.2f} deg apart — arms no longer "
                   "share one 2D sketch (Re-square fixes this)")
        else:
            msg = (f"Sketch plane at {yaw:.2f} deg vs the {want:.1f} deg "
                   "design yaw — knuckle twists in side view (Re-square "
                   "fixes this)")
        self.sketch_fit.setText(f"<span style='color:{color}'>{msg}</span>")

    def set_design_yaw_provider(self, fn) -> None:
        """Callable returning the active axle's design sketch yaw (deg)."""
        self._design_yaw_provider = fn

    def set_arm_plane_provider(self, fn) -> None:
        """Callable returning the active axle's steering-arm plane (deg)."""
        self._arm_plane_provider = fn

    def set_vehicle_dx(self, dx: float) -> None:
        """Fore-aft (tool +X) offset of the ACTIVE axle into the shared
        vehicle frame (0 front, -wheelbase rear). Redraws so the table
        reads in that frame."""
        if abs(dx - self._veh_dx) < 1e-9:
            return
        self._veh_dx = float(dx)
        if self._hp is not None:
            self.refresh(self._hp)

    def _veh_off(self) -> np.ndarray:
        return np.array([self._veh_dx, 0.0, 0.0])

    def _to_view(self, pt) -> np.ndarray:
        """Axle-local stored point -> shared-vehicle + display frame."""
        return self._conv.to_display(np.asarray(pt, float) + self._veh_off())

    def _from_view(self, q) -> np.ndarray:
        """Shared-vehicle display value -> axle-local stored point."""
        return self._conv.from_display(np.asarray(q, float)) - self._veh_off()

    def _apply_unit_headers(self) -> None:
        u = self._unit.label
        # Column 4 is the point's offset ALONG the sketch normal (the
        # bushing-axis direction): editing it slides the point
        # perpendicular to the 2D sketch — packaging moves that mostly
        # preserve the in-sketch kinematics.
        # "⊥ sk" not "⊥ sketch": the long caption was the one heading too
        # wide for its column, so it always elided — the tooltip below
        # carries the full meaning.
        self.table.setHorizontalHeaderLabels(
            [f"{lbl} ({u})" for lbl in self._conv.labels]
            + [f"⊥ sk ({u})"])
        tips = [f"{lbl} coordinate ({u})" for lbl in self._conv.labels] + [
            f"Offset ALONG the 2D sketch normal ({u}) — editing it slides "
            "the point perpendicular to the sketch plane, a packaging move "
            "that mostly preserves the in-sketch kinematics. Double "
            "wishbone only."]
        for col, tip in enumerate(tips):
            item = self.table.horizontalHeaderItem(col)
            if item is not None:
                item.setToolTip(tip)

    def _sketch_normal_of(self, hp):
        """Measured sketch-plane normal (unit, fore-oriented) for the
        ⊥-sketch column. None for types that have no 2D sketch."""
        if not isinstance(hp, DoubleWishbonePoints):
            # Single-arm carrier types (C-hub, loaded halfshaft, H-arm) DO
            # have one: their chassis bushing axis and carrier pin are both
            # normal to it (v1.34). The column used to read "—" for them.
            if hasattr(hp, "arm_inner_front"):
                a = np.asarray(hp.arm_inner_front, float) - np.asarray(
                    hp.arm_inner_rear, float)
                n = float(np.linalg.norm(a))
                if n < 1e-9:
                    return None
                a = a / n
                return a if a[0] >= 0.0 else -a
            return None
        au = hp.uca_inner_front - hp.uca_inner_rear
        al = hp.lca_inner_front - hp.lca_inner_rear
        au = au / np.linalg.norm(au)
        al = al / np.linalg.norm(al)
        if np.dot(au, al) < 0.0:
            al = -al
        n = au + al
        n = n / np.linalg.norm(n)
        return -n if n[0] < 0.0 else n

    def set_convention(self, conv) -> None:
        """Switch the DISPLAY axis convention (tool vs chassis frame).
        Stored geometry never changes — only how it reads on screen."""
        self._conv = conv
        self._apply_unit_headers()
        if self._hp is not None:
            self.refresh(self._hp)

    def set_units(self, unit: Unit) -> None:
        # Pending text is in the OLD unit and there is no honest way to
        # reinterpret it, so drop it rather than silently applying inches
        # as millimetres.
        if self._pending:
            self._pending.clear()
            self.edit_status.setText(
                "Pending cells were cleared — the display unit changed.")
            self._set_batch_state()
        self._unit = unit
        for name, label, kind, lo, hi in SCALAR_FIELDS:
            if kind == "len":
                s = self._scalars[name]
                s.blockSignals(True)
                s.setRange(unit.from_mm(lo), unit.from_mm(hi))
                s.setDecimals(unit.decimals)
                s.setSingleStep(unit.step)
                s.setSuffix(f" {unit.label}")
                s.blockSignals(False)
        self._apply_unit_headers()
        if self._hp is not None:
            self.refresh(self._hp)

    def _fields_for(self, hp) -> list:
        """(attr, label) rows for any suspension type: the curated list
        for the double wishbone, prettified POINT_ATTRS otherwise."""
        if isinstance(hp, DoubleWishbonePoints):
            return POINT_FIELDS
        labels = getattr(type(hp), "LABELS", {})
        return [(a, labels.get(a, a.replace("_", " ")))
                for a in type(hp).POINT_ATTRS]

    def set_halfshaft_provider(self, fn) -> None:
        """fn() -> the active axle's HalfshaftConfig (or None). Lets the
        table append the CV joint rows without knowing about axles."""
        self._hs_provider = fn

    def _hs_rows(self, hp) -> list:
        """[(label, point_mm)] for the halfshaft, when enabled: the inner
        CV (chassis) and the outer CV (on the kingpin axis). A type whose
        shaft is STRUCTURAL (loaded halfshaft) already lists hs_inner /
        hs_outer as regular hardpoints — no extra rows."""
        if hasattr(hp, "hs_inner"):
            return []
        cfg = self._hs_provider()
        if cfg is None or not getattr(cfg, "enabled", False):
            return []
        from ..halfshaft import outer_cv_of_hp
        return [("Halfshaft inner CV", np.asarray(cfg.inner, float)),
                ("Halfshaft outer CV", outer_cv_of_hp(hp))]

    def refresh(self, hp) -> None:
        """Show `hp` (stored in mm) in the current unit, no edit signals.
        None clears the table (blank workspace, v1.9)."""
        try:
            self._refresh_inner(hp)
        finally:
            # _refreshing gates _on_cell. If a refresh ever raises partway
            # it used to stay True FOREVER, and from then on the table
            # silently ignored every keystroke and every lock -- a dead
            # panel with no error and no way back but a restart. Whatever
            # goes wrong, the gate reopens.
            self._refreshing = False

    def _refresh_inner(self, hp) -> None:
        self._refreshing = True
        self._hp = hp
        if hp is None:
            self.table.setRowCount(0)
            self.sketch_fit.setText("")
            self._refreshing = False
            return
        fields = self._fields_for(hp)
        hs_rows = self._hs_rows(hp)
        total = len(fields) + len(hs_rows)
        if self.table.rowCount() != total:
            self.table.setRowCount(total)
        row_labels = ([(f"🔒 {label}" if attr in self._locked else label)
                       for attr, label in fields]
                      + [lbl for lbl, _ in hs_rows])
        self.table.setVerticalHeaderLabels(row_labels)
        # The name column is width-capped so the four coordinate columns
        # stay legible, so a long name can elide — keep the full text (and
        # the lock hint) reachable on hover.
        for row, label in enumerate(row_labels):
            item = self.table.verticalHeaderItem(row)
            if item is not None:
                item.setToolTip(
                    f"{label} — double-click to lock/unlock"
                    if row < len(fields) else label)
        normal = self._sketch_normal_of(hp)

        def _norm_item(pt, locked=False):
            if normal is None:
                item = QTableWidgetItem("—")
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            else:
                item = QTableWidgetItem(
                    self._unit.fmt(float(np.dot(pt, normal))))
                if locked:
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            return item

        for row, (attr, _) in enumerate(fields):
            pt = getattr(hp, attr)
            p = self._to_view(pt)
            locked = attr in self._locked
            for col in range(3):
                item = QTableWidgetItem(self._unit.fmt(p[col]))
                item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                if locked:      # pinned: not editable in the table
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.table.setItem(row, col, item)
            self.table.setItem(row, 3, _norm_item(pt, locked))
        for k, (_, pt) in enumerate(hs_rows):
            p = self._to_view(pt)
            for col in range(3):
                item = QTableWidgetItem(self._unit.fmt(p[col]))
                item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.table.setItem(len(fields) + k, col, item)
            self.table.setItem(len(fields) + k, 3, _norm_item(pt))
        for name, kind in self._scalar_kind.items():
            s = self._scalars[name]
            s.blockSignals(True)
            v = getattr(hp, name)
            s.setValue(self._unit.from_mm(v) if kind == "len" else v)
            s.blockSignals(False)
        self._update_sketch_fit(hp)
        # Anything can trigger a refresh — the travel slider, an axle
        # switch, a tweak. In batch mode that must not silently erase cells
        # the user has typed but not applied yet.
        if self._batch and self._pending:
            self._paint_pending()
        self._refreshing = False

    # ------------------------------------------------------------------
    # Derived coordinates
    # ------------------------------------------------------------------
    # A double wishbone carries the knuckle-consistency convention: the tire
    # spin axis stays in the ball-joint/wheel-centre plane and the steering
    # arm stays at its set angle to it. That makes some coordinates DERIVED
    # -- you can type into them, and the convention puts them straight back.
    #
    # Measured on the example front geometry, +10 mm typed into each of the
    # 33 coordinates:
    #     wheel_center X   ->  0.00 mm kept   (the axle station is derived)
    #     tierod_outer Y   ->  0.13 mm kept   (the steering-arm plane)
    # plus millimetre-scale trims on tierod_outer Z and wheel_center Z, and
    # edits to OTHER points quietly dragging tierod_outer/wheel_center by up
    # to ~6 mm. Only the wheel-centre case used to be reported, by name, in
    # a special case -- so the tie-rod outer looked exactly like a dead cell.
    # This says it for every point and every axis, by comparing what was
    # typed against what the convention actually kept.
    PROJECTION_TOL_MM = 0.05

    def _projection_note(self, edited, touched) -> str:
        """Which of the coordinates the user JUST TYPED were moved back.

        Only the typed ones. Other points shifting is the convention doing
        its job — moving an inner pickup legitimately re-derives the
        knuckle — and reporting that as well would bury the one case that
        actually looks like a broken cell.

        `touched` is an iterable of (attr, axis index) pairs.
        """
        pairs = [(a, i) for a, i in touched if 0 <= i < 3]
        if not pairs or not isinstance(edited, DoubleWishbonePoints):
            return ""
        from ..geometry import enforce_knuckle_planes
        plane = 90.0
        if self._arm_plane_provider is not None:
            plane = float(self._arm_plane_provider() or 90.0)
        try:
            kept = enforce_knuckle_planes(edited, plane)
        except (ValueError, ZeroDivisionError):
            return ""
        labels = dict(self._fields_for(edited))
        heads = self._conv.labels if hasattr(self._conv, "labels") else "XYZ"
        moved = []
        for attr, col in pairs:
            if not hasattr(edited, attr):
                continue
            # The table's columns are in the DISPLAY convention and the
            # convention is not the tool frame, so the drift has to be
            # rotated before it can be indexed by column. Differencing two
            # `_to_view` calls does that exactly, translation and all.
            drift = (self._to_view(getattr(kept, attr))
                     - self._to_view(getattr(edited, attr)))
            d = abs(float(drift[col]))
            if d > self.PROJECTION_TOL_MM:
                moved.append((labels.get(attr, attr), heads[col],
                              self._unit.from_mm(d)))
        if not moved:
            return ""
        bits = ", ".join(
            f"{name} {ax} by {d:.{self._unit.decimals}f} {self._unit.label}"
            for name, ax, d in moved[:4])
        more = f" (+{len(moved) - 4} more)" if len(moved) > 4 else ""
        return (f"That value did not land: the knuckle convention moved "
                f"{bits}{more}. Those coordinates are DERIVED — the spin "
                f"axis stays in the ball-joint plane and the steering arm "
                f"at its set angle. Move the ball joints, or use the corner "
                f"tweaks, to shift them.")

    def _report_projection(self, edited, touched) -> None:
        note = self._projection_note(edited, touched)
        self.edit_status.setText(note)
        if note:
            self.note.emit(note)

    def _touched_pairs(self):
        """(attr, axis) for every pending cell in the XYZ columns."""
        fields = self._fields_for(self._hp)
        out = []
        for (row, col) in self._pending:
            if 0 <= col < 3 and 0 <= row < len(fields):
                out.append((fields[row][0], col))
        return out

    # ------------------------------------------------------------------
    # Batch edit
    # ------------------------------------------------------------------
    def _set_batch_state(self) -> None:
        n = len(self._pending)
        self.apply_btn.setText(f"Apply {n} edits" if n else "Apply")
        self.apply_btn.setEnabled(self._batch and n > 0)
        self.revert_btn.setEnabled(self._batch and n > 0)
        self.apply_btn.setVisible(self._batch)
        self.revert_btn.setVisible(self._batch)

    def _on_batch_toggled(self, on: bool) -> None:
        self._batch = bool(on)
        if not self._batch and self._pending:
            self.apply_batch()      # leaving batch mode means "go live now"
        self._set_batch_state()

    def _mark_pending(self, row: int, col: int) -> None:
        item = self.table.item(row, col)
        if item is None:
            return
        self._pending[(row, col)] = item.text()
        self._paint_pending()
        self._set_batch_state()
        self.edit_status.setText("")

    def _paint_pending(self, rejected: bool = False) -> None:
        """Put typed-but-unapplied text back on screen and colour it.

        Foreground rather than a background wash on purpose: the tool ships
        a light and a dark theme, and one tint cannot serve both.
        """
        from PySide6.QtGui import QColor
        was, self._refreshing = self._refreshing, True
        try:
            for (row, col), text in self._pending.items():
                item = self.table.item(row, col)
                if item is None:
                    continue
                if item.text() != text:
                    item.setText(text)
                font = item.font()
                font.setBold(True)
                item.setFont(font)
                item.setForeground(QColor("#c02020" if rejected
                                          else "#b06000"))
        finally:
            self._refreshing = was

    def revert_batch(self) -> None:
        """Discard every pending cell and redraw from the live geometry."""
        self._pending.clear()
        self._last_batch.clear()
        self.edit_status.setText("")
        self._set_batch_state()
        if self._hp is not None:
            self.refresh(self._hp)

    def apply_batch(self) -> None:
        """Solve the whole table in one go.

        Order matters: the XYZ columns and the scalars build the new point
        set together, then a perpendicular-to-sketch cell slides its own
        row's result (so typing both on one row means the normal wins), and
        the halfshaft CV rows go last because they travel on their own
        signals.
        """
        if self._hp is None or not self._pending:
            return
        self.edit_status.setText("")
        fields = self._fields_for(self._hp)
        n_fields = len(fields)
        edited = self._current_edit()
        for (row, col) in sorted(self._pending):
            if col != 3 or row >= n_fields:
                continue
            attr = fields[row][0]
            pt = self._norm_edit_point(row, getattr(edited, attr))
            if pt is not None:
                edited = dataclasses.replace(edited, **{attr: pt})
        hs_rows = self._hs_rows(self._hp)
        hs_emits = []
        for k, (_, base) in enumerate(hs_rows):
            row = n_fields + k
            cols = [c for (r, c) in self._pending if r == row]
            if not cols:
                continue
            pt = (self._norm_edit_point(row, base) if cols == [3]
                  else self._read_row_point(row, base))
            if pt is not None:
                hs_emits.append((k, pt))
        # Clear BEFORE emitting: a successful apply refreshes the table, and
        # stale pending text must not be painted back over the new values.
        touched = self._touched_pairs()
        self._last_batch = dict(self._pending)
        self._pending.clear()
        self._set_batch_state()
        self._report_projection(edited, touched)
        self.edited.emit(edited)
        for k, pt in hs_emits:
            (self.halfshaft_inner_edited if k == 0
             else self.halfshaft_outer_edited).emit(pt)

    def edit_rejected(self, reason: str) -> None:
        """The geometry would not close, so the edit was refused.

        In live mode this reverts, as it always has. In batch mode it keeps
        every cell exactly as typed and marks them red — throwing away a
        whole table of coordinates because the last one was wrong is the
        behaviour batch mode exists to avoid.
        """
        if self._batch and self._last_batch:
            self._pending = dict(self._last_batch)
            self._paint_pending(rejected=True)
            self._set_batch_state()
        elif self._hp is not None:
            self.refresh(self._hp)
        self.edit_status.setText(
            f"<span style='color:#c02020'>Edit refused — the linkage does "
            f"not close at ride height ({reason}). "
            f"{'Fix a cell and Apply again.' if self._batch else ''}</span>")

    def _current_edit(self):
        """Build a new hardpoint set (mm) from the widgets' current values."""
        kw = {}
        for row, (attr, _) in enumerate(self._fields_for(self._hp)):
            shown = self._to_view(getattr(self._hp, attr))
            vals = []
            for col in range(3):
                try:
                    vals.append(self._unit.to_mm(float(self.table.item(row, col).text())))
                except (TypeError, ValueError):
                    vals.append(shown[col])   # unparsable cell: keep old
            kw[attr] = self._from_view(np.array(vals))
        for name, kind in self._scalar_kind.items():
            v = self._scalars[name].value()
            kw[name] = self._unit.to_mm(v) if kind == "len" else v
        return dataclasses.replace(self._hp, **kw)

    def _read_row_point(self, row: int, fallback) -> np.ndarray:
        shown = self._to_view(fallback)
        vals = []
        for col in range(3):
            try:
                vals.append(self._unit.to_mm(
                    float(self.table.item(row, col).text())))
            except (TypeError, ValueError):
                vals.append(shown[col])
        return self._from_view(np.array(vals))

    def _norm_edit_point(self, row: int, base_pt) -> np.ndarray | None:
        """New point for a ⊥-sketch (column 4) edit: slide `base_pt`
        along the sketch normal to the typed offset. None = unparsable
        cell or no normal (non-DW type)."""
        normal = self._sketch_normal_of(self._hp)
        if normal is None:
            return None
        try:
            target = self._unit.to_mm(float(self.table.item(row, 3).text()))
        except (TypeError, ValueError):
            return None
        cur = float(np.dot(base_pt, normal))
        return np.asarray(base_pt, float) + (target - cur) * normal

    def _on_cell(self, row=-1, col=0, *_):
        # cellChanged emits (row, column)
        if self._refreshing or self._hp is None:
            return
        if self._batch:
            self._mark_pending(row, col)
            return
        n_fields = len(self._fields_for(self._hp))
        if row >= n_fields:                      # a halfshaft CV row
            hs_rows = self._hs_rows(self._hp)
            k = row - n_fields
            if k < len(hs_rows):
                pt = (self._norm_edit_point(row, hs_rows[k][1])
                      if col == 3 else
                      self._read_row_point(row, hs_rows[k][1]))
                if pt is None:
                    self.refresh(self._hp)
                    return
                if k == 0:
                    self.halfshaft_inner_edited.emit(pt)
                else:
                    self.halfshaft_outer_edited.emit(pt)
            return
        if col == 3:                             # ⊥-sketch edit: slide the
            attr = self._fields_for(self._hp)[row][0]   # point along the
            pt = self._norm_edit_point(row, getattr(self._hp, attr))
            if pt is None:                       # sketch normal only
                self.refresh(self._hp)
                return
            self.edited.emit(dataclasses.replace(self._hp, **{attr: pt}))
            return
        edited = self._current_edit()
        self._report_projection(
            edited, [(self._fields_for(self._hp)[row][0], col)])
        self.edited.emit(edited)

    def _on_scalar(self, *_):
        if self._refreshing or self._hp is None:
            return
        if self._batch:
            # Scalars ride along in `_current_edit`, so a batch apply picks
            # them up; they just need to COUNT so Apply lights up.
            self._pending.setdefault((-1, -1), "")
            self._set_batch_state()
            self.edit_status.setText("")
            return
        self.edited.emit(self._current_edit())

    def set_locked(self, locked) -> None:
        """Set the pinned point attrs (from the active axle) and redraw."""
        self._locked = set(locked or ())
        if self._hp is not None:
            self.refresh(self._hp)

    def _toggle_lock(self, row: int) -> None:
        """Pin/unpin the hardpoint on `row` (double-clicked name)."""
        if self._hp is None:
            return
        fields = self._fields_for(self._hp)
        if row >= len(fields):
            # The halfshaft CV rows are DERIVED (the outer rides the
            # kingpin, the inner is the config's own point), so there is
            # nothing to pin. Say so -- returning silently here reads
            # exactly like a broken lock.
            self.note.emit(
                "That row is a derived halfshaft CV position, not a "
                "hardpoint — there is nothing to lock. Double-click a "
                "point name above instead.")
            return
        attr = fields[row][0]
        self._locked.symmetric_difference_update({attr})
        self.refresh(self._hp)
        self.locks_changed.emit(frozenset(self._locked))


class TweaksPanel(QGroupBox):
    """Relative-dimension tuning for EVERY suspension type — adjust the
    kinematic dimensions designers think in, per type:

      double wishbone: shock mount along/off/about its arm + tie-rod rises
      trailing arm:    shock mount on the arm line + pivot-axis skew angles
                       (the semi-trailing knobs for camber/toe gain)
      multilink:       toe-link end rises (the bump-steer knobs)

    Reads the current geometry whenever it changes elsewhere; any edit
    re-solves everything with the same reject-and-revert safety as the
    hardpoint table."""

    edited = Signal(object)  # a new hardpoint set
    # Re-squaring to a new sketch yaw changes the corner's DESIGN
    # INTENT, not just its geometry, so the recorded seed value has
    # to follow or the checklist keeps grading against the old one.
    sketch_yaw_changed = Signal(float)
    blocked = Signal(str)    # a tweak hit a locked point (message for the bar)

    DW_ROWS = [
        ("_h_shock", "Shock mount", "head"),
        ("d", "Shock mount along arm (from inboard)", "len"),
        ("h", "Shock mount height off arm line", "len"),
        ("ang", "Shock mount angle (0 = up, + = fwd)", "ang"),
        ("shock_len", "Shock length @ ride (installed)", "len"),
        ("_h_steer", "Steering (bump steer, Ackermann, toe)", "head"),
        ("outer_rise", "Tie rod outer rise above LBJ", "len"),
        ("inner_rise", "Tie rod inner rise above LCA axis", "len"),
        ("hub_cv", "Hub offset (outer CV -> tire centre)", "len"),
        ("hub_t", "Hub/outer-CV along kingpin (+ = up)", "len"),
        ("steer_arm", "Steering arm (kingpin -> tie rod, sketch-normal)",
         "len"),
        ("toe_link", "Static toe (threads the tie rod)", "ang"),
        ("_h_geo", "Kingpin & arm geometry", "head"),
        ("scrub", "Scrub radius (leans the kingpin)", "len"),
        ("caster", "Caster angle (UBJ fore/aft)", "ang"),
        ("kickup", "Sketch kickup (rotates bushing axes)", "ang"),
        ("sketch_yaw", "Sketch plane yaw (angled frame tubes)", "ang"),
        ("kp_len", "Kingpin length (UBJ slides)", "len"),
        ("axis_sep", "Inboard UC-LC axis gap (direct)", "len"),
        ("uca_len", "UCA length on sketch (BJ slides)", "len"),
        ("lca_len", "LCA length on sketch (BJ slides)", "len"),
        ("_h_place", "Placement (moves the whole corner — no kinematics change)",
         "head"),
        ("ride_h", "Ride height (ground -> frame datum)", "len"),
        ("corner_x", "Move corner fore/aft (wheel-centre X, + fwd)", "len"),
        ("corner_z", "Move corner up/down (wheel-centre Z, + up)", "len"),
    ]
    # Rows shared by every type (the placement + installed-length knobs).
    # Kept as a suffix so each type's own rows read first.
    COMMON_ROWS = [
        ("_h_place", "Placement (moves the whole corner — no kinematics change)",
         "head"),
        ("ride_h", "Ride height (ground -> frame datum)", "len"),
        ("corner_x", "Move corner fore/aft (wheel-centre X, + fwd)", "len"),
        ("corner_z", "Move corner up/down (wheel-centre Z, + up)", "len"),
    ]
    # Hub offset needs a STEERING AXIS to measure from — ball joints, a
    # physical kingpin, or an outer CV. Trailing-arm, multilink and H-arm
    # uprights have none, so the row would read a degenerate 0 and do
    # nothing; it is offered only where it means something.
    HUB_ROW = [("hub_cv", "Hub offset (steering axis -> tire centre)", "len")]
    TA_ROWS = [
        ("_h_shock", "Shock mount", "head"),
        ("d", "Shock mount along arm (from pivots)", "len"),
        ("h", "Shock mount height off arm line", "len"),
        ("ang", "Shock mount angle (0 = up, + = fwd)", "ang"),
        ("shock_len", "Shock length @ ride (installed)", "len"),
        ("_h_pivot", "Pivot axis skew", "head"),
        ("plan_deg", "Pivot plan skew (+ = semi-trailing)", "ang"),
        ("elev_deg", "Pivot elevation skew (+ = outer up)", "ang"),
    ] + COMMON_ROWS
    ML_ROWS = [
        ("_h_toe l", "Toe link (bump steer)", "head"),
        ("outer_rise", "Toe link outer rise vs wheel centre", "len"),
        ("inner_rise", "Toe link inner rise vs link inners", "len"),
        ("shock_len", "Shock length @ ride (installed)", "len"),
    ] + COMMON_ROWS
    CH_ROWS = [
        ("_h_shock", "Shock mount", "head"),
        ("d", "Shock mount along arm (from bushings)", "len"),
        ("h", "Shock mount height off arm line", "len"),
        ("ang", "Shock mount angle (0 = up, + = fwd)", "ang"),
        ("shock_len", "Shock length @ ride (installed)", "len"),
        ("_h_block", "C-hub block & kingpin", "head"),
        ("caster", "Kingpin caster (C-hub block angle)", "ang"),
        ("kpi", "Kingpin inclination (KPI insert)", "ang"),
        ("kp_len", "Kingpin length (upper end slides)", "len"),
        ("axle_x", "Wheel centre fore of kingpin axis", "len"),
        ("_h_steer", "Steering (bump steer, Ackermann, toe)", "head"),
        ("outer_rise", "Tie rod outer rise above kingpin base", "len"),
        ("inner_rise", "Tie rod inner rise above arm axis", "len"),
        ("steer_arm", "Steering arm (kingpin axis -> tie rod)", "len"),
        ("toe", "Static toe", "ang"),
        ("_h_arm", "Arm, camber link & sketch", "head"),
        ("cam_outer_rise", "Camber link outer rise above kingpin top", "len"),
        ("cam_inner_rise", "Camber link inner rise above arm axis", "len"),
        ("arm_len", "Lower arm length (bushings -> hinge pin)", "len"),
        ("kickup", "Arm bushing-axis kickup", "ang"),
        ("sketch_yaw", "Sketch plane yaw (angled frame tubes)", "ang"),
        ("pill", "Caster pill at the C-block (adds to kickup)", "ang"),
    ] + HUB_ROW + COMMON_ROWS
    LH_ROWS = [
        ("_h_shock", "Shock mount", "head"),
        ("d", "Shock mount along arm (from bushings)", "len"),
        ("h", "Shock mount height off arm line", "len"),
        ("ang", "Shock mount angle (0 = up, + = fwd)", "ang"),
        ("shock_len", "Shock length @ ride (installed)", "len"),
        ("plan_deg", "Knuckle pin plan skew (+ front out)", "ang"),
        ("elev_deg", "Knuckle pin elevation (+ front up)", "ang"),
        ("_h_arm", "Arm & sketch", "head"),
        ("arm_len", "Upper arm length (bushings -> knuckle pin)", "len"),
        ("kickup", "Arm bushing-axis kickup", "ang"),
        ("sketch_yaw", "Sketch plane yaw (angled frame tubes)", "ang"),
    ] + HUB_ROW + COMMON_ROWS
    HA_ROWS = [
        ("_h_shock", "Shock mount", "head"),
        ("d", "Shock mount along H-arm (from bushings)", "len"),
        ("h", "Shock mount height off arm line", "len"),
        ("ang", "Shock mount angle (0 = up, + = fwd)", "ang"),
        ("shock_len", "Shock length @ ride (installed)", "len"),
        ("plan_deg", "Grab-line plan skew (+ front out)", "ang"),
        ("elev_deg", "Grab-line elevation (+ front up)", "ang"),
        ("_h_arm", "Arm, camber link & sketch", "head"),
        ("cam_outer_rise", "Camber link outer rise above grab line", "len"),
        ("cam_inner_rise", "Camber link inner rise above arm axis", "len"),
        ("arm_len", "H-arm length (bushings -> grab line)", "len"),
        ("kickup", "Arm bushing-axis kickup", "ang"),
        ("sketch_yaw", "Sketch plane yaw (angled frame tubes)", "ang"),
    ] + COMMON_ROWS

    # Rows each type's setter takes POSITIONALLY and always applies. Every
    # other row is optional and gets gated: it is only re-applied when its
    # spin genuinely moved, so a display-rounded readback can never walk
    # the geometry while a different row is being edited.
    REQUIRED_ROWS = {
        "ta": {"d", "h", "ang", "plan_deg", "elev_deg"},
        "ml": {"outer_rise", "inner_rise"},
        # axle_x is deliberately NOT required: it and hub_cv both place the
        # wheel centre, so gating it means whichever one the user actually
        # edited is the one that lands exactly.
        "ch": {"d", "h", "ang", "caster", "kpi"},
        "lh": {"d", "h", "ang", "plan_deg", "elev_deg"},
        "ha": {"d", "h", "ang", "plan_deg", "elev_deg"},
    }

    def __init__(self, unit: Unit, parent=None):
        super().__init__("Kinematic tweaks (relative dimensions)", parent)
        self._unit = unit
        self._hp = None
        self._refreshing = False
        # setup context for the tweaks that need more than the hardpoints:
        # ride-height datum + the shock's stroke limits (for the split).
        self._frame_tube_offset = 0.0
        self._shock_min = None
        self._shock_max = None
        self._locked = frozenset()   # hardpoints the user has pinned
        self._independent_caster = False
        lay = QVBoxLayout(self)
        self._boxes = {}     # type key -> (QWidget, {row_key: spin})
        for key, rows in (("dw", self.DW_ROWS), ("ta", self.TA_ROWS),
                          ("ml", self.ML_ROWS), ("ch", self.CH_ROWS),
                          ("lh", self.LH_ROWS), ("ha", self.HA_ROWS)):
            box = QWidget()
            form = QFormLayout(box)
            form.setContentsMargins(0, 0, 0, 0)
            spins = {}
            for row_key, label, kind in rows:
                if kind == "head":
                    # Section caption. These panels run to 21 rows now, and
                    # a flat list that long hides its own contents -- the
                    # ride-height row sat at 19 of 21 and read as missing.
                    cap = QLabel(label)
                    cap.setStyleSheet(
                        "color: palette(mid); font-weight: bold; "
                        "margin-top: 6px;")
                    form.addRow(cap)
                    continue
                if kind == "ang":
                    s = _spin(-90.0, 90.0, 2, 1.0)
                    s.setSuffix(" deg")
                else:
                    s = _spin(-1e6, 1e6, unit.decimals, unit.step)
                s.valueChanged.connect(self._on_change)
                spins[row_key] = s
                form.addRow(QLabel(label), s)
            lay.addWidget(box)
            box.setVisible(False)
            self._boxes[key] = (box, spins, rows)
        # A muted caption for the two hint lines (which arm the shock
        # mounts to; the shock stroke split) so they read as help text,
        # not a stray sentence floating under the spin boxes.
        self.arm_label = QLabel("")
        self.arm_label.setWordWrap(True)
        self.arm_label.setStyleSheet("color: gray; font-style: italic;")
        lay.addWidget(self.arm_label)
        self.split_label = QLabel("")   # shock-length bump/droop split
        self.split_label.setWordWrap(True)
        self.split_label.setStyleSheet("color: gray; font-style: italic;")
        lay.addWidget(self.split_label)
        self._apply_unit()

    def set_context(self, setup) -> None:
        """Feed the panel the active axle's setup so the ride-height and
        shock-length tweaks have their datum + stroke limits. Safe to call
        with None (keeps the last known context)."""
        if setup is None:
            return
        self._frame_tube_offset = float(getattr(setup, "frame_tube_offset",
                                                 0.0) or 0.0)
        self._shock_min = getattr(setup, "shock_min_length", None)
        self._shock_max = getattr(setup, "shock_max_length", None)
        self._independent_caster = bool(getattr(setup, "independent_caster",
                                                False))
        if self._hp is not None:
            self.refresh(self._hp)

    def set_locked(self, locked) -> None:
        """The active axle's pinned hardpoints, so scrub/caster tweaks
        refuse to move a locked UBJ instead of silently overriding it."""
        self._locked = frozenset(locked or ())

    @staticmethod
    def _type_key(hp) -> str:
        from ..chub_front import CHubFrontPoints
        from ..harm_rear import HArmRearPoints
        from ..loaded_halfshaft import LoadedHalfshaftPoints
        from ..multilink import MultilinkPoints
        from ..trailing_arm import TrailingArmPoints
        if isinstance(hp, TrailingArmPoints):
            return "ta"
        if isinstance(hp, MultilinkPoints):
            return "ml"
        if isinstance(hp, CHubFrontPoints):
            return "ch"
        if isinstance(hp, LoadedHalfshaftPoints):
            return "lh"
        if isinstance(hp, HArmRearPoints):
            return "ha"
        return "dw"

    def _params_of(self, hp) -> dict:
        from ..tweaks import (chub_params, harm_params, loaded_hs_params,
                              multilink_toe_params, shock_mount_params,
                              tierod_rises, trailing_arm_params)
        key = self._type_key(hp)
        per_type = {"ta": trailing_arm_params, "ml": multilink_toe_params,
                    "ch": chub_params, "lh": loaded_hs_params,
                    "ha": harm_params}
        if key in per_type:
            d = dict(per_type[key](hp))
            # ride height needs the frame-tube datum, which only the panel
            # knows, so the tweak modules hand back None for it.
            if d.get("ride_h") is None:
                from ..tweaks import ride_height
                d["ride_h"] = ride_height(hp, self._frame_tube_offset)
            return d
        from ..tweaks import (arm_length_sketch, caster_angle_deg, corner_x,
                              corner_z,
                              hub_along_kingpin, hub_cv_offset,
                              inboard_axis_sep, kingpin_length, ride_height,
                              scrub_radius, shock_length_at_ride,
                              steer_arm_length)
        from ..tweaks import sketch_kickup
        from ..tweaks import sketch_yaw_deg as _sketch_yaw_deg
        return {**shock_mount_params(hp), **tierod_rises(hp),
                "corner_x": corner_x(hp),
                "corner_z": corner_z(hp),
                "kickup": sketch_kickup(hp),
                "sketch_yaw": _sketch_yaw_deg(hp) or 0.0,
                "hub_cv": hub_cv_offset(hp),
                "hub_t": hub_along_kingpin(hp),
                "steer_arm": steer_arm_length(hp),
                "scrub": scrub_radius(hp),
                "caster": caster_angle_deg(hp),
                "kp_len": kingpin_length(hp),
                "axis_sep": inboard_axis_sep(hp),
                "uca_len": arm_length_sketch(hp, True),
                "lca_len": arm_length_sketch(hp, False),
                "toe_link": float(hp.static_toe_deg),
                "shock_len": shock_length_at_ride(hp),
                "ride_h": ride_height(hp, self._frame_tube_offset)}

    def _apply_unit(self) -> None:
        u = self._unit
        for _, (box, spins, rows) in self._boxes.items():
            for row_key, _, kind in rows:
                if kind == "len" and row_key in spins:
                    s = spins[row_key]
                    s.blockSignals(True)
                    s.setDecimals(u.decimals)
                    s.setSingleStep(u.step)
                    s.setSuffix(f" {u.label}")
                    s.blockSignals(False)

    def set_units(self, unit: Unit) -> None:
        self._unit = unit
        self._apply_unit()
        if self._hp is not None:
            self.refresh(self._hp)

    def refresh(self, hp) -> None:
        """Show the dimensions of `hp` without emitting edit signals."""
        self._refreshing = True
        self._hp = hp
        active = self._type_key(hp)
        vals = self._params_of(hp)
        for key, (box, spins, rows) in self._boxes.items():
            box.setVisible(key == active)
            if key != active:
                continue
            for row_key, _, kind in rows:
                if kind == "head":
                    continue
                s = spins[row_key]
                s.blockSignals(True)
                v = vals[row_key]
                s.setValue(v if kind == "ang" else self._unit.from_mm(v))
                s.blockSignals(False)
        arm_toe = (getattr(hp, "tierod_on_lca", False)
                   or getattr(hp, "tierod_on_uca", False))
        dw_note = ("Shock-mount rows act on the UPPER arm."
                   if getattr(hp, "shock_on_uca", False)
                   else "Shock-mount rows act on the LOWER arm.")
        if arm_toe:
            # The toe-link inner is bolted to the same arm as the ball
            # joint, so it RIDES that arm: its distance to the ball joint
            # is fixed through travel and moving it barely changes bump
            # steer (measured: ~10% over 60 mm, against ~500x that for a
            # chassis mount). The knob that works here is the OUTER end.
            which = "UPPER" if getattr(hp, "tierod_on_uca", False) else "LOWER"
            dw_note += (f"  Toe link is mounted on the {which} ARM, so the "
                        "'Tie rod inner rise' row is nearly INERT — the "
                        "inner rides the arm. Tune bump steer with the "
                        "OUTER rise (or the steering arm / upper arm).")
        notes = {"dw": dw_note,
                 "ta": "Pivot_inner bushing stays fixed when skewing.",
                 "ml": "Link 5 is the toe link.",
                 "ch": "Kingpin re-aims about its midpoint (block swap).",
                 "lh": "Pin skew couples knuckle swing into toe.",
                 "ha": "Grab-line skew couples upright swing into toe."}
        self.arm_label.setText(notes[active])
        self.split_label.setText(self._split_text(hp) if active == "dw"
                                 else "")
        self._refreshing = False

    def _split_text(self, hp) -> str:
        """How the installed shock length splits its stroke into bump /
        droop travel — the readout that guides the shock-length tweak."""
        if self._shock_min is None or self._shock_max is None:
            return ""
        from ..tweaks import shock_travel_split
        s = shock_travel_split(hp, self._shock_min, self._shock_max)
        if not np.isfinite(s["bump_frac"]):
            return ""
        u = self._unit
        return (f"Stroke split: bump {100 * s['bump_frac']:.0f}% / "
                f"droop {100 * s['droop_frac']:.0f}%  "
                f"(installed {u.from_mm(s['length']):.{u.decimals}f} "
                f"{u.label}, stroke {u.from_mm(s['stroke']):.{u.decimals}f} "
                f"{u.label})")

    def _gate_optional(self, vals, rows, key) -> dict:
        """Replace every OPTIONAL row that has not actually moved with None,
        which the tweak setters read as "leave it alone".

        Without this, editing one row would re-apply every other row at its
        DISPLAY precision and walk the geometry by the rounding error each
        time — the same trap the double wishbone's hand-rolled path already
        guards with an epsilon. The gate must exceed half a display ULP.
        """
        u = self._unit
        eps_len = max(0.02, 0.75 * u.to_mm(10 ** -u.decimals))
        eps_ang = 0.02
        required = self.REQUIRED_ROWS.get(key, set())
        current = self._params_of(self._hp)
        out = {}
        for rk, _, kind in rows:
            if kind == "head":
                continue
            v = vals[rk]
            if rk in required:
                out[rk] = v
                continue
            was = current.get(rk)
            eps = eps_ang if kind == "ang" else eps_len
            out[rk] = None if (was is not None and abs(v - was) <= eps) else v
        return out

    def _on_change(self, *_):
        if self._refreshing or self._hp is None:
            return
        from ..tweaks import (set_chub, set_harm, set_loaded_hs,
                              set_multilink_toe, set_shock_mount,
                              set_tierod_rises, set_trailing_arm)
        u = self._unit
        key = self._type_key(self._hp)
        _, spins, rows = self._boxes[key]
        vals = {rk: (spins[rk].value() if kind == "ang"
                     else u.to_mm(spins[rk].value()))
                for rk, _, kind in rows if kind != "head"}
        if key in ("ta", "ml", "ch", "lh", "ha"):
            setter = {"ta": set_trailing_arm, "ml": set_multilink_toe,
                      "ch": set_chub, "lh": set_loaded_hs,
                      "ha": set_harm}[key]
            vals = self._gate_optional(vals, rows, key)
            if vals.get("sketch_yaw") is not None:
                self.sketch_yaw_changed.emit(float(vals["sketch_yaw"]))
            hp = setter(self._hp, **vals,
                        frame_tube_offset=self._frame_tube_offset,
                        locked=self._locked)
        else:
            from ..tweaks import (hub_along_kingpin, set_hub_along_kingpin,
                                  set_steer_arm_length, steer_arm_length)
            # Re-apply a length tweak only if its spin genuinely moved.
            # The gate must exceed the display ROUNDING (half a display ULP)
            # so a readback rounded by the spin never re-fires a setter when
            # a DIFFERENT tweak is edited — otherwise switching to mm units
            # (0.1 mm display) could nudge the whole corner by ~0.05 mm.
            eps = max(0.02, 0.75 * u.to_mm(10 ** -u.decimals))
            hp = set_shock_mount(self._hp, vals["d"], vals["h"], vals["ang"])
            hp = set_tierod_rises(hp, vals["outer_rise"], vals["inner_rise"])
            # only re-apply hub/steer-arm positions if THAT spin moved —
            # their display-rounded readbacks must not drift the geometry
            # when some other tweak is being edited
            from ..tweaks import (hub_cv_offset, set_hub_cv_offset,
                                  set_shock_length_at_ride,
                                  shock_length_at_ride)
            if abs(vals["shock_len"]
                   - shock_length_at_ride(self._hp)) > eps:
                hp = set_shock_length_at_ride(hp, vals["shock_len"])
            if abs(vals["hub_cv"] - hub_cv_offset(self._hp)) > eps:
                hp = set_hub_cv_offset(hp, vals["hub_cv"])
            if abs(vals["hub_t"] - hub_along_kingpin(self._hp)) > eps:
                hp = set_hub_along_kingpin(hp, vals["hub_t"])
            if abs(vals["steer_arm"]
                   - steer_arm_length(self._hp)) > eps:
                hp = set_steer_arm_length(hp, vals["steer_arm"])
            from ..tweaks import (arm_length_sketch, inboard_axis_sep,
                                  kingpin_length, set_arm_length_sketch,
                                  set_inboard_axis_sep,
                                  set_kingpin_length)
            if abs(vals["kp_len"] - kingpin_length(self._hp)) > eps:
                hp = set_kingpin_length(hp, vals["kp_len"])
            if abs(vals["axis_sep"]
                   - inboard_axis_sep(self._hp)) > eps:
                hp = set_inboard_axis_sep(hp, vals["axis_sep"])
            if abs(vals["uca_len"]
                   - arm_length_sketch(self._hp, True)) > eps:
                hp = set_arm_length_sketch(hp, True, vals["uca_len"])
            if abs(vals["lca_len"]
                   - arm_length_sketch(self._hp, False)) > eps:
                hp = set_arm_length_sketch(hp, False, vals["lca_len"])
            if abs(vals["toe_link"]
                   - float(self._hp.static_toe_deg)) > 0.005:
                from ..tweaks import set_toe_by_tierod
                hp = set_toe_by_tierod(hp, vals["toe_link"])
            from ..tweaks import (LockedPointError, caster_angle_deg,
                                  scrub_radius, set_caster_angle_deg,
                                  set_scrub_radius, set_sketch_kickup,
                                  sketch_kickup)
            from ..tweaks import set_sketch_yaw as _set_sketch_yaw
            from ..tweaks import sketch_yaw_deg as _sketch_yaw_deg
            from ..tweaks import (corner_x as _corner_x,
                                  corner_z as _corner_z,
                                  ride_height as _ride_height, set_corner_x,
                                  set_corner_z, set_ride_height)
            try:
                if abs(vals["scrub"] - scrub_radius(self._hp)) > eps:
                    hp = set_scrub_radius(hp, vals["scrub"], self._locked)
                if abs(vals["caster"] - caster_angle_deg(self._hp)) > 0.01:
                    hp = set_caster_angle_deg(hp, vals["caster"], self._locked)
                if abs(vals["sketch_yaw"]
                       - (_sketch_yaw_deg(self._hp) or 0.0)) > 0.01:
                    # Before kickup: squaring to a new yaw PRESERVES the
                    # kickup elevation, so doing it first leaves the
                    # kickup row still describing the same corner.
                    hp = _set_sketch_yaw(
                        hp, vals["sketch_yaw"],
                        planarize_kingpin_too=not self._independent_caster)
                    self.sketch_yaw_changed.emit(float(vals["sketch_yaw"]))
                if abs(vals["kickup"] - sketch_kickup(self._hp)) > 0.01:
                    hp = set_sketch_kickup(
                        hp, vals["kickup"],
                        planarize=not self._independent_caster,
                        locked=self._locked)
                if abs(vals["ride_h"]
                       - _ride_height(self._hp,
                                      self._frame_tube_offset)) > eps:
                    hp = set_ride_height(hp, vals["ride_h"],
                                         self._frame_tube_offset,
                                         locked=self._locked)
                # corner_z after ride_h: both move the corner in z, and an
                # explicit "move it up 10 mm" should win over the derived
                # ride-height target if somehow both changed.
                if abs(vals["corner_z"] - _corner_z(self._hp)) > eps:
                    hp = set_corner_z(hp, vals["corner_z"],
                                      locked=self._locked)
                if abs(vals["corner_x"] - _corner_x(self._hp)) > eps:
                    hp = set_corner_x(hp, vals["corner_x"],
                                      locked=self._locked)
            except LockedPointError as e:
                self.refresh(self._hp)          # revert the spin box
                self.blocked.emit(str(e))
                return
        self.edited.emit(hp)


class OptimizePanel(QGroupBox):
    """Phase 4: pick goals (+ targets and weights), pick which hardpoints
    are free, and run the optimizer. Emits a plain payload dict; the main
    window owns the actual optimization run."""

    optimize_requested = Signal(object)  # payload dict (internal mm units)
    resquare_requested = Signal()        # fix one-sketch only, no goals

    # (key, default_enabled, default_target_internal, default_weight)
    GOAL_ROWS = [
        ("camber_gain", True, -0.04, 1.0),    # deg per mm of travel
        ("bump_steer", True, 0.0, 1.0),
        ("toe_slope", False, -0.005, 1.0),    # deg/mm; - = out in bump
        ("rc_height", False, 300.0, 1.0),
        ("rc_migration", False, 0.0, 1.0),
        ("motion_ratio", False, 0.54, 1.0),
        ("caster", False, 4.0, 1.0),
        ("scrub_radius", False, 30.0, 1.0),
        ("kpi", False, 7.0, 1.0),
        ("caster_trail", False, 15.0, 1.0),
    ]
    FREE_ROWS = [
        ("uca_inner", "UCA inner axis", True),
        ("uca_outer", "UCA outer (UBJ)", True),
        ("lca_inner", "LCA inner axis", False),
        ("lca_outer", "LCA outer (LBJ)", False),
        ("tierod_inner", "Tie rod inner", True),
        ("tierod_outer", "Tie rod outer", False),
        ("shock_inner", "Shock inner", False),
        ("shock_outer", "Shock outer", False),
    ]

    def __init__(self, unit: Unit, parent=None):
        super().__init__("Optimization goals (Phase 4)", parent)
        self._unit = unit
        lay = QVBoxLayout(self)

        # Goal rows: [enable] label [target] [weight]
        self._enable = {}
        self._target = {}   # spinboxes; values in DISPLAY units
        self._weight = {}
        self._targets_internal = {}  # source of truth, internal units
        grid = QGridLayout()
        grid.addWidget(QLabel("<i>goal</i>"), 0, 0)
        grid.addWidget(QLabel("<i>target</i>"), 0, 1)
        grid.addWidget(QLabel("<i>weight</i>"), 0, 2)
        for row, (key, on, target, weight) in enumerate(self.GOAL_ROWS, start=1):
            spec = GOAL_SPECS[key]
            cb = QCheckBox(spec["label"])
            cb.setChecked(on)
            self._enable[key] = cb
            grid.addWidget(cb, row, 0)
            self._targets_internal[key] = target
            if spec["unit_kind"] is not None:
                spin = _spin(-1e6, 1e6, 3, 0.1)
                self._target[key] = spin
                grid.addWidget(spin, row, 1)
            w = _spin(0.0, 100.0, 2, 0.5)
            w.setValue(weight)
            self._weight[key] = w
            grid.addWidget(w, row, 2)
        lay.addLayout(grid)

        lay.addWidget(QLabel("<b>Free hardpoints</b> (everything else stays fixed):"))
        self._free = {}
        self._free_grid = QGridLayout()
        self._rebuild_free([key for key, _, _ in self.FREE_ROWS],
                           {key for key, _, on in self.FREE_ROWS if on})
        lay.addLayout(self._free_grid)

        form = QFormLayout()
        self._box = _spin(0.0, 0.0, 0, 0.0)  # configured in _apply_unit
        self._box_mm = 75.0
        self._box_label = QLabel("Search range ±")
        form.addRow(self._box_label, self._box)
        lay.addLayout(form)

        btn = QPushButton("Run optimization")
        btn.clicked.connect(self._emit)
        lay.addWidget(btn)
        resq = QPushButton("Re-square arms (fix one-sketch only)")
        resq.setToolTip(
            "Double wishbone only: rotate the UCA/LCA bushing axes back "
            "onto one shared 2D sketch plane (midpoints and spreads kept) "
            "WITHOUT running any optimization goals. Undo-able.")
        resq.clicked.connect(self.resquare_requested)
        lay.addWidget(resq)
        self._apply_unit()

    # -- unit conversion helpers -------------------------------------
    def _to_display(self, key: str, v: float) -> float:
        kind = GOAL_SPECS[key]["unit_kind"]
        if kind == "len":
            return self._unit.from_mm(v)
        if kind == "rate":              # deg/mm -> deg per display unit
            return v * self._unit.mm
        return v

    def _to_internal(self, key: str, v: float) -> float:
        kind = GOAL_SPECS[key]["unit_kind"]
        if kind == "len":
            return self._unit.to_mm(v)
        if kind == "rate":
            return v / self._unit.mm
        return v

    def _rebuild_free(self, names, checked) -> None:
        """Recreate the free-hardpoint checkboxes (the available groups
        differ per suspension type)."""
        while self._free_grid.count():
            item = self._free_grid.takeAt(0)
            if item.widget() is not None:
                item.widget().deleteLater()
        self._free = {}
        for i, key in enumerate(names):
            cb = QCheckBox(key.replace("_", " "))
            cb.setChecked(key in checked)
            self._free[key] = cb
            self._free_grid.addWidget(cb, i // 2, i % 2)

    def set_free_groups_for(self, hp) -> None:
        """Point the free list at the active axle's suspension type."""
        from ..optimize import free_groups_for
        names = list(free_groups_for(hp))
        defaults = {
            "uca_inner", "uca_outer", "tierod_inner",   # double wishbone
            "pivot_outer",                              # trailing arm
            "link5_inner",                              # multilink toe link
        }
        current = {k for k, cb in self._free.items() if cb.isChecked()}
        keep = current & set(names)
        self._rebuild_free(names, keep if keep else defaults & set(names))

    def _apply_unit(self) -> None:
        u = self._unit
        for key, spin in self._target.items():
            spec = GOAL_SPECS[key]
            spin.blockSignals(True)
            if spec["unit_kind"] == "len":
                spin.setDecimals(u.decimals)
                spin.setSingleStep(u.step)
                spin.setSuffix(f" {u.label}")
            elif spec["unit_kind"] == "rate":
                spin.setDecimals(3)
                spin.setSingleStep(0.05)
                spin.setSuffix(f" deg/{u.label}")
            elif spec["unit_kind"] == "ang":
                spin.setSuffix(" deg")
            spin.setValue(self._to_display(key, self._targets_internal[key]))
            spin.blockSignals(False)
        self._box.blockSignals(True)
        self._box.setRange(u.from_mm(0.5), u.from_mm(300.0))
        self._box.setDecimals(u.decimals)
        self._box.setSingleStep(u.step)
        self._box.setSuffix(f" {u.label}")
        self._box.setValue(u.from_mm(self._box_mm))
        self._box.blockSignals(False)

    def _capture(self) -> None:
        for key, spin in self._target.items():
            self._targets_internal[key] = self._to_internal(key, spin.value())
        self._box_mm = self._unit.to_mm(self._box.value())

    def set_units(self, unit: Unit) -> None:
        self._capture()
        self._unit = unit
        self._apply_unit()

    # -- payload / persistence ----------------------------------------
    def payload(self) -> dict:
        self._capture()
        goals = [{"key": key,
                  "target": self._targets_internal.get(key, 0.0),
                  "weight": self._weight[key].value()}
                 for key, _, _, _ in self.GOAL_ROWS if self._enable[key].isChecked()]
        # LIVE checkboxes, not the static DW FREE_ROWS (rebuilt per type)
        free = [key for key, cb in self._free.items() if cb.isChecked()]
        return {"goals": goals, "free": free, "box_mm": self._box_mm}

    def _emit(self) -> None:
        commit_spins(self)      # same reason as SetupForm._emit
        self.optimize_requested.emit(self.payload())

    def to_dict(self) -> dict:
        """Full panel state (internal units) for saving in a project file."""
        self._capture()
        return {
            "goals": [{"key": key,
                       "enabled": self._enable[key].isChecked(),
                       "target": self._targets_internal.get(key, 0.0),
                       "weight": self._weight[key].value()}
                      for key, _, _, _ in self.GOAL_ROWS],
            # iterate the LIVE checkboxes, not the static DW FREE_ROWS —
            # the free groups are rebuilt per suspension type, so a C-hub
            # or loaded-halfshaft axle has no uca_inner/etc. key here
            "free": [key for key, cb in self._free.items() if cb.isChecked()],
            "box_mm": self._box_mm,
        }

    def load_dict(self, data: dict) -> None:
        for g in data.get("goals", []):
            key = g.get("key")
            if key in self._enable:
                self._enable[key].setChecked(bool(g.get("enabled", False)))
                self._targets_internal[key] = float(g.get("target", 0.0))
                self._weight[key].setValue(float(g.get("weight", 1.0)))
        free = set(data.get("free", []))
        for key, cb in self._free.items():
            cb.setChecked(key in free)
        self._box_mm = float(data.get("box_mm", 75.0))
        self._apply_unit()


class VehiclePanel(QGroupBox):
    """Vehicle-level inputs (wheelbase, overall CG) and the derived roll-
    axis numbers they unlock once BOTH axles have geometry."""

    changed = Signal()
    ride_height_requested = Signal(float)   # vehicle ride height (mm)

    # lo/hi in mm — lower bounds reach 1/10 RC scale (see SetupForm.SPECS).
    FIELDS = [
        ("wheelbase", "Wheelbase", 50.0, 4000.0),
        ("cg_height", "CG height above ground", 5.0, 2000.0),
        ("cg_behind_front", "CG distance behind front axle", 0.0, 4000.0),
    ]
    # dimensionless input, handled separately from the length FIELDS
    RATIO_FIELDS = [("brake_front_frac", "Front brake fraction", 0.0, 1.0)]
    DERIVED = [
        ("cg_pos", "CG position (x, y, z)"),
        ("stance", "F/R stance (ground-plane offset)"),
        ("front_rc", "Front RC height (current)"),
        ("rear_rc", "Rear RC height (current)"),
        ("roll_axis_angle_deg", "Roll axis angle (+ = up toward rear)"),
        ("roll_axis_height_at_cg_mm", "Roll axis height @ CG"),
        ("roll_moment_arm_mm", "Roll moment arm (CG to axis)"),
        ("weight_split", "Static weight split F/R"),
        ("anti_dive_front_pct", "Anti-dive front (braking)"),
        ("anti_lift_rear_pct", "Anti-lift rear (braking)"),
        ("anti_squat_rear_pct", "Anti-squat rear (accel)"),
    ]

    def __init__(self, unit: Unit, parent=None):
        super().__init__("Vehicle (CG + roll axis)", parent)
        from ..vehicle import VehicleParams
        self._unit = unit
        self._params = VehicleParams()
        form = QFormLayout(self)
        self._spins = {}
        for name, label, lo, hi in self.FIELDS:
            s = _spin(0, 0, 0, 0)
            s.valueChanged.connect(self._on_change)
            self._spins[name] = (s, lo, hi)
            form.addRow(QLabel(label), s)
        self._ratio_spins = {}
        for name, label, lo, hi in self.RATIO_FIELDS:
            s = _spin(lo, hi, 2, 0.05)
            s.setValue(getattr(self._params, name))
            s.valueChanged.connect(self._on_change)
            self._ratio_spins[name] = s
            form.addRow(QLabel(label), s)
        # Vehicle ride height: ONE knob that rigid-translates BOTH axles
        # to the same measured ride height (ground -> frame datum, the
        # same convention as the per-axle tweak) — set the whole car's
        # stance, or zero a red F/R stance row, without reseeding.
        rh_row = QWidget()
        rh_lay = QHBoxLayout(rh_row)
        rh_lay.setContentsMargins(0, 0, 0, 0)
        self.rh_spin = _spin(0, 0, 0, 0)     # range/format set per unit
        rh_lay.addWidget(self.rh_spin, stretch=1)
        rh_btn = QPushButton("Set")
        rh_btn.setToolTip(
            "Rigidly translate BOTH axles in z until each reads this ride "
            "height (no kinematics change — the same move as the per-axle "
            "ride-height tweak, applied to the whole car)")
        rh_btn.clicked.connect(
            lambda: self.ride_height_requested.emit(
                self._unit.to_mm(self.rh_spin.value())))
        rh_lay.addWidget(rh_btn)
        form.addRow(QLabel("Vehicle ride height (both axles)"), rh_row)
        self._out = {}
        for key, label in self.DERIVED:
            lbl = QLabel("-")
            self._out[key] = lbl
            form.addRow(QLabel(label), lbl)
        # CG plausibility. These three numbers are typed by hand and are
        # the easiest thing in the tool to leave at another car's values --
        # a full-size CG height on a 1/10 model puts the roll axis and the
        # roll moment arm into nonsense while every other readout still
        # looks fine, so nothing else catches it.
        self.cg_warning = QLabel("")
        self.cg_warning.setWordWrap(True)
        self.cg_warning.setStyleSheet("color: #c46210; font-weight: bold;")
        form.addRow(self.cg_warning)
        self._apply_unit()

    def check_cg_plausible(self, wheelbase_mm: float, track_mm: float,
                           tire_radius_mm: float) -> list:
        """Warnings about a CG that cannot belong to this car.

        Checked against the GEOMETRY rather than against absolute limits,
        so it works at Baja and 1/10 RC scale alike."""
        out = []
        p = self._params
        wb = float(wheelbase_mm or 0.0)
        if wb > 1.0:
            frac = float(p.cg_behind_front) / wb
            if not (0.0 <= frac <= 1.0):
                where = "BEHIND the rear axle" if frac > 1 else "AHEAD of the front axle"
                out.append(
                    f"CG is {where}: {p.cg_behind_front:.0f} mm behind the "
                    f"front on a {wb:.0f} mm wheelbase ({frac:.2f}x). That "
                    f"implies a static front weight fraction of {1 - frac:+.2f}, "
                    "which is impossible — every load-transfer and "
                    "anti-geometry number downstream is meaningless.")
        if tire_radius_mm and float(p.cg_height) > 8.0 * float(tire_radius_mm):
            out.append(
                f"CG height {p.cg_height:.0f} mm is {p.cg_height / tire_radius_mm:.0f}x "
                f"the tire radius (a real car is 2-6x). Left over from a "
                "different-scale design?")
        if track_mm and float(p.cg_height) > 1.2 * float(track_mm):
            out.append(
                f"CG height {p.cg_height:.0f} mm exceeds the {track_mm:.0f} mm "
                "track — the car would roll over before it slid.")
        return out

    def show_cg_warnings(self, msgs) -> None:
        self.cg_warning.setText("⚠ " + "  ⚠ ".join(msgs) if msgs else "")

    def update_ride_height_display(self, mm: float | None) -> None:
        """Track the FRONT axle's measured ride height in the spin (the
        canonical reference plane) — skipped while the user is typing."""
        if mm is None or self.rh_spin.hasFocus():
            return
        self.rh_spin.blockSignals(True)
        self.rh_spin.setValue(self._unit.from_mm(mm))
        self.rh_spin.blockSignals(False)

    def _apply_unit(self) -> None:
        u = self._unit
        for name, (s, lo, hi) in self._spins.items():
            s.blockSignals(True)
            s.setRange(u.from_mm(lo), u.from_mm(hi))
            s.setDecimals(u.decimals)
            s.setSingleStep(u.step)
            s.setSuffix(f" {u.label}")
            s.setValue(u.from_mm(getattr(self._params, name)))
            s.blockSignals(False)
        s = self.rh_spin
        s.blockSignals(True)
        s.setRange(u.from_mm(0.0), u.from_mm(1000.0))
        s.setDecimals(u.decimals)
        s.setSingleStep(u.step)
        s.setSuffix(f" {u.label}")
        s.blockSignals(False)

    def set_units(self, unit: Unit) -> None:
        rh_mm = self._unit.to_mm(self.rh_spin.value())
        self._unit = unit
        self._apply_unit()
        self.rh_spin.blockSignals(True)
        self.rh_spin.setValue(unit.from_mm(rh_mm))
        self.rh_spin.blockSignals(False)

    def params(self):
        return self._params

    def load_params(self, params) -> None:
        self._params = params
        for name, s in self._ratio_spins.items():
            s.blockSignals(True)
            s.setValue(getattr(params, name))
            s.blockSignals(False)
        self._apply_unit()

    def _on_change(self, *_):
        for name, (s, _, _) in self._spins.items():
            setattr(self._params, name, self._unit.to_mm(s.value()))
        for name, s in self._ratio_spins.items():
            setattr(self._params, name, s.value())
        self.changed.emit()

    def show_derived(self, rc_front, rc_rear, metrics: dict | None,
                     anti: dict | None = None, cg=None,
                     stance=None) -> None:
        """Update the derived rows; None values display as '-' (e.g. when
        only one axle has geometry yet). cg: absolute (x, y, z) mm of the
        CG in model coordinates (matches the 3D marker and Onshape).
        stance: front ground z minus rear ground z (mm) — the two axles
        should sit on ONE ground, so anything beyond a couple of mm is
        flagged red (+ = front tires lower / rear floating)."""
        u = self._unit
        if stance is None:
            self._out["stance"].setText("-")
        else:
            s_txt = f"{u.from_mm(stance):+.{u.decimals}f} {u.label}"
            if abs(stance) <= 2.0:
                self._out["stance"].setText(
                    f"<span style='color:#2a9d2a'>matched "
                    f"({s_txt})</span>")
            else:
                self._out["stance"].setText(
                    f"<span style='color:#d03030'>{s_txt} — axles on "
                    "different ground planes; equalize ride heights"
                    "</span>")
        def L(v):   # length in display units
            return "-" if v is None else f"{u.from_mm(v):+.{u.decimals}f} {u.label}"
        self._out["cg_pos"].setText(
            "-" if cg is None else
            "(" + ", ".join(f"{u.from_mm(c):.{u.decimals}f}" for c in cg)
            + f") {u.label}")
        self._out["front_rc"].setText(L(rc_front))
        self._out["rear_rc"].setText(L(rc_rear))
        for key in ("anti_dive_front_pct", "anti_lift_rear_pct",
                    "anti_squat_rear_pct"):
            v = None if anti is None else anti.get(key)
            self._out[key].setText(
                "-" if v is None or not np.isfinite(v) else f"{v:+.1f} %")
        if metrics is None:
            for key in ("roll_axis_angle_deg", "roll_axis_height_at_cg_mm",
                        "roll_moment_arm_mm", "weight_split"):
                self._out[key].setText("-")
            return
        self._out["roll_axis_angle_deg"].setText(
            f"{metrics['roll_axis_angle_deg']:+.2f} deg")
        self._out["roll_axis_height_at_cg_mm"].setText(
            L(metrics["roll_axis_height_at_cg_mm"]))
        self._out["roll_moment_arm_mm"].setText(
            L(metrics["roll_moment_arm_mm"]))
        self._out["weight_split"].setText(
            f"{metrics['front_weight_frac']*100:.0f} / "
            f"{metrics['rear_weight_frac']*100:.0f} %")


class FramePanel(QGroupBox):
    """Chassis backdrop controls (Phase 6): load an STL/3MF, then line it
    up with the hardpoints using scale / Z-rotation / XYZ offset."""

    load_requested = Signal()
    changed = Signal()          # transform or visibility changed

    def __init__(self, unit: Unit, parent=None):
        super().__init__("Frame backdrop", parent)
        from ..frame import FrameTransform
        self._unit = unit
        self.transform = FrameTransform()
        self.path: str | None = None
        form = QFormLayout(self)
        btn = QPushButton("Load frame mesh (STL / 3MF)…")
        btn.clicked.connect(self.load_requested.emit)
        form.addRow(btn)
        self.file_label = QLabel("no frame loaded")
        form.addRow(self.file_label)
        self.show_check = QCheckBox("Show frame")
        self.show_check.setChecked(True)
        self.show_check.toggled.connect(lambda _: self.changed.emit())
        form.addRow(self.show_check)
        # Overlapping STL shells stack badly at high transparency, so this
        # is dialled per project rather than fixed. 100 = solid.
        from .viewport import FRAME_OPACITY
        self.opacity_spin = _spin(0.0, 100.0, 0, 5.0)
        self.opacity_spin.setSuffix(" %")
        self.opacity_spin.setValue(FRAME_OPACITY * 100.0)
        self.opacity_spin.setToolTip(
            "How solid the chassis looks. Lower shows the linkages "
            "through it; higher reads mount faces cleanly and avoids the "
            "muddle you get when several imported parts overlap.")
        self.opacity_spin.valueChanged.connect(lambda _: self.changed.emit())
        form.addRow(QLabel("Opacity"), self.opacity_spin)
        self.scale_spin = _spin(0.0001, 100000.0, 4, 1.0)
        self.scale_spin.setSuffix(" x")
        form.addRow(QLabel("Scale (mesh units → mm)"), self.scale_spin)
        self.rot_combo = QComboBox()
        for deg in (0, 90, 180, 270):
            self.rot_combo.addItem(f"{deg}°", float(deg))
        form.addRow(QLabel("Rotate about Z"), self.rot_combo)
        self._offsets = {}
        for key, label in (("dx", "Offset X (fwd)"), ("dy", "Offset Y (left)"),
                           ("dz", "Offset Z (up)")):
            s = _spin(-1e6, 1e6, unit.decimals, unit.step)
            self._offsets[key] = s
            form.addRow(QLabel(label), s)
        self.scale_spin.valueChanged.connect(self._on_change)
        self.rot_combo.currentIndexChanged.connect(self._on_change)
        for s in self._offsets.values():
            s.valueChanged.connect(self._on_change)
        # The frame backdrop marks WHERE THE REAL CHASSIS IS. If the
        # suspension doesn't meet it, move the suspension (drag points /
        # seed offsets) — moving the frame here only changes the picture,
        # and exported coordinates would no longer land on the real car.
        self.align_warning = QLabel("")
        self.align_warning.setWordWrap(True)
        form.addRow(self.align_warning)
        self._refreshing = False
        self._apply_unit()

    def _update_align_warning(self) -> None:
        t = self.transform
        canonical = (t.rot_z_deg == 270.0 and t.dx == 0.0
                     and t.dy == 0.0 and t.dz == 0.0)
        if canonical or self.path is None:
            self.align_warning.setText("")
        else:
            self.align_warning.setText(
                "<span style='color:#d07030'>⚠ Frame moved off the chassis "
                "alignment. The backdrop is only a picture — if the "
                "suspension doesn't meet the frame, move the SUSPENSION "
                "(drag points / seed offsets), or exported coordinates "
                "won't land on the real chassis.</span>")

    def _apply_unit(self) -> None:
        u = self._unit
        for s in self._offsets.values():
            s.blockSignals(True)
            s.setDecimals(u.decimals)
            s.setSingleStep(u.step)
            s.setSuffix(f" {u.label}")
            s.blockSignals(False)
        self.refresh()

    def set_units(self, unit: Unit) -> None:
        self._unit = unit
        self._apply_unit()

    def refresh(self) -> None:
        """Show self.transform in the widgets without emitting signals."""
        self._refreshing = True
        t, u = self.transform, self._unit
        self.scale_spin.blockSignals(True)
        self.scale_spin.setValue(t.scale)
        self.scale_spin.blockSignals(False)
        idx = {0.0: 0, 90.0: 1, 180.0: 2, 270.0: 3}.get(t.rot_z_deg % 360, 0)
        self.rot_combo.blockSignals(True)
        self.rot_combo.setCurrentIndex(idx)
        self.rot_combo.blockSignals(False)
        for key, s in self._offsets.items():
            s.blockSignals(True)
            s.setValue(u.from_mm(getattr(t, key)))
            s.blockSignals(False)
        self.file_label.setText(
            self.path.split("/")[-1].split("\\")[-1] if self.path
            else "no frame loaded")
        self._update_align_warning()
        self._refreshing = False

    def _on_change(self, *_):
        if self._refreshing:
            return
        t, u = self.transform, self._unit
        t.scale = self.scale_spin.value()
        t.rot_z_deg = self.rot_combo.currentData()
        for key, s in self._offsets.items():
            setattr(t, key, u.to_mm(s.value()))
        self._update_align_warning()
        self.changed.emit()


class Readouts(QGroupBox):
    """Live numbers at the currently displayed travel position."""

    # (metric key, base label, kind). "rate" = degrees per length unit.
    ROWS = [
        ("travel_mm", "Travel", "len"),
        ("ride_height_mm", "Ride height (frame-ground)", "len"),
        ("wheelbase_mm", "Wheelbase", "len"),
        ("track_width_mm", "Track width", "len"),
        ("overall_width_mm", "Overall width (tire edge to edge)", "len"),
        ("camber_deg", "Camber", "ang"),
        ("toe_deg", "Toe", "ang"),
        ("caster_deg", "Caster", "ang"),
        ("kpi_deg", "KPI", "ang"),
        ("scrub_radius_mm", "Scrub radius", "len"),
        ("caster_trail_mm", "Caster trail", "len"),
        ("roll_center_height_mm", "Roll centre h", "len"),
        ("shock_length_mm", "Shock length", "len"),
        ("shock_split_txt", "Stroke split @ ride (bump/droop)", "text"),
        ("shock_use_txt", "Stroke used over travel range", "text"),
        ("motion_ratio", "Motion ratio (shock/wheel)", "ratio"),
        ("bump_steer_deg_per_mm", "Bump steer", "rate"),
        ("half_track_change_mm", "Half-track change", "len"),
        ("wheel_recession_mm", "Wheel recession (rear +)", "len"),
    ]

    def __init__(self, unit: Unit, parent=None):
        super().__init__("Readouts @ current travel", parent)
        self._unit = unit
        self._last = {}
        form = QFormLayout(self)
        self._labels = {}     # key -> value QLabel
        self._names = {}      # key -> name QLabel
        for key, label, _ in self.ROWS:
            name_lbl = QLabel(label)
            val_lbl = QLabel("-")
            self._labels[key] = val_lbl
            self._names[key] = name_lbl
            form.addRow(name_lbl, val_lbl)
        self._retitle()

    def _retitle(self) -> None:
        u = self._unit.label
        for key, label, kind in self.ROWS:
            suffix = {"len": f" ({u})", "ang": " (deg)",
                      "rate": f" (deg/{u})", "ratio": "", "text": ""}[kind]
            self._names[key].setText(label + suffix)

    def set_units(self, unit: Unit) -> None:
        self._unit = unit
        self._retitle()
        if self._last:
            self.update_values(self._last)

    def update_values(self, values: dict) -> None:
        self._last = values
        for key, _, kind in self.ROWS:
            v = values.get(key)
            if v is None:
                self._labels[key].setText("-")
                continue
            if kind == "text":                 # pre-formatted string rows
                self._labels[key].setText(str(v))
                continue
            if kind == "len":
                shown = self._unit.from_mm(v)
            elif kind == "rate":           # deg/mm -> deg per length unit
                shown = v * self._unit.mm
            else:
                shown = v
            try:
                ok = np.isfinite(shown)
            except TypeError:
                ok = False
            self._labels[key].setText(f"{shown:+.3f}" if ok else "-")


class HalfshaftPanel(QGroupBox):
    """Per-axle halfshaft (CV axle) checks: tick the axle that has one,
    place the inner CV joint (gearbox output flange), set the joint's
    rated articulation and plunge, and the tool reports CV angles and
    plunge through the whole travel sweep — warnings, not hard stops
    (fix the geometry or the mount, or accept it knowingly)."""

    changed = Signal()

    def __init__(self, unit: Unit, parent=None):
        super().__init__("Halfshafts (CV joints)", parent)
        from ..axes import CONVENTIONS, DEFAULT_CONVENTION
        from ..halfshaft import HalfshaftConfig
        self._unit = unit
        self._conv = CONVENTIONS[DEFAULT_CONVENTION]
        self._loading = False
        self._cfgs = {"front": HalfshaftConfig(),
                      "rear": HalfshaftConfig(enabled=False)}
        self._shock_specs = {"front": None, "rear": None}  # ShockSpec|None
        # fore-aft shift into the shared vehicle frame per axle (0 front,
        # -wheelbase rear) so the inner-CV coords match the hardpoint table
        # and Onshape — one origin for the whole car.
        self._veh_dx = {"front": 0.0, "rear": 0.0}
        lay = QVBoxLayout(self)
        self._w = {}
        for key in ("front", "rear"):
            box = QGroupBox(f"{key.title()} halfshaft")
            form = QFormLayout(box)
            en = QCheckBox("Model this axle's halfshaft")
            en.toggled.connect(self._on_edit)
            spins = {}
            for axis in range(3):
                s = _spin(0, 0, 0, 0)
                s.valueChanged.connect(self._on_edit)
                spins[axis] = s
                form.addRow(QLabel(""), s)   # captions set by _apply_units
            form.insertRow(0, QLabel(""), en)
            # Inner (diff-side) joint TYPE: sets the default inner angle and
            # whether the joint plunges (a fixed Rzeppa can't).
            from ..halfshaft import JOINT_TYPES
            joint = QComboBox()
            for jk, jm in JOINT_TYPES.items():
                joint.addItem(jm["label"], jk)
            joint.currentIndexChanged.connect(
                lambda _i, kk=key: self._on_joint_type(kk))
            # Separate angle limits for the two joints — the inner joint
            # (type above) tolerates less than the outer fixed joint
            # (Rzeppa), which also swallows steer on a driven front.
            cv_in = _spin(5.0, 60.0, 1, 1.0)
            cv_in.setSuffix(" deg")
            cv_in.valueChanged.connect(self._on_edit)
            cv_out = _spin(5.0, 60.0, 1, 1.0)
            cv_out.setSuffix(" deg")
            cv_out.valueChanged.connect(self._on_edit)
            # Plunge budget: total axial stroke + % of it available as
            # plunge-IN (compression) from ride height; the rest is pull-OUT.
            pl_total = _spin(0, 0, 0, 0)
            pl_total.valueChanged.connect(self._on_edit)
            pl_pct = _spin(0.0, 100.0, 0, 5.0)
            pl_pct.setSuffix(" %")
            pl_pct.valueChanged.connect(self._on_edit)
            form.addRow(QLabel("Inner joint type"), joint)
            form.addRow(QLabel("Max CV angle — inner"), cv_in)
            form.addRow(QLabel("Max CV angle — outer (fixed)"), cv_out)
            form.addRow(QLabel("Plunge total (stroke)"), pl_total)
            form.addRow(QLabel("Plunge-in % (rest pull-out)"), pl_pct)
            # Shock HARDWARE limits: fixed datasheet numbers that live
            # with the axle (like the CV specs above) instead of the seed
            # form, so they can never get lost — the travel range is
            # clamped so no pose can over-stroke the shock.
            sh_min = _spin(0, 0, 0, 0)
            sh_min.valueChanged.connect(self._on_edit)
            sh_max = _spin(0, 0, 0, 0)
            sh_max.valueChanged.connect(self._on_edit)
            form.addRow(QLabel("Shock min length (hardware)"), sh_min)
            form.addRow(QLabel("Shock max length (hardware)"), sh_max)
            status = QLabel("")
            status.setWordWrap(True)
            form.addRow(status)
            lay.addWidget(box)
            self._w[key] = {"enable": en, "spins": spins, "joint": joint,
                            "cv_in": cv_in, "cv_out": cv_out,
                            "pl_total": pl_total, "pl_pct": pl_pct,
                            "sh_min": sh_min, "sh_max": sh_max,
                            "status": status, "form": form}
        self._apply_units()
        self._push_cfgs()

    # ------------------------------------------------------------------
    def _apply_units(self) -> None:
        u = self._unit
        for key, w in self._w.items():
            for axis, s in w["spins"].items():
                s.blockSignals(True)
                s.setRange(u.from_mm(-5000), u.from_mm(5000))
                s.setDecimals(u.decimals)
                s.setSingleStep(u.step)
                s.setSuffix(f" {u.label}")
                s.blockSignals(False)
                lbl = w["form"].labelForField(s)
                if lbl is not None:
                    lbl.setText(f"Inner CV {self._conv.labels[axis]}")
            pl = w["pl_total"]
            pl.blockSignals(True)
            pl.setRange(u.from_mm(0.0), u.from_mm(300.0))
            pl.setDecimals(u.decimals)
            pl.setSingleStep(u.step)
            pl.setSuffix(f" {u.label}")
            pl.blockSignals(False)
            for sk in ("sh_min", "sh_max"):
                s = w[sk]
                s.blockSignals(True)
                s.setRange(u.from_mm(0.0), u.from_mm(2000.0))
                s.setDecimals(u.decimals)
                s.setSingleStep(u.step)
                s.setSuffix(f" {u.label}")
                s.blockSignals(False)
        self._push_cfgs()

    def set_units(self, unit: Unit) -> None:
        self._capture()
        self._unit = unit
        self._apply_units()

    def set_convention(self, conv) -> None:
        self._capture()
        self._conv = conv
        self._apply_units()

    def set_vehicle_dx(self, front_dx: float, rear_dx: float) -> None:
        """Per-axle fore-aft shift into the shared vehicle frame (0 front,
        -wheelbase rear) so inner-CV coords match the hardpoint table."""
        self._capture()
        self._veh_dx = {"front": float(front_dx), "rear": float(rear_dx)}
        self._push_cfgs()

    # ------------------------------------------------------------------
    def _push_cfgs(self) -> None:
        """Write self._cfgs into the widgets (no signals)."""
        self._loading = True
        u = self._unit
        for key, w in self._w.items():
            cfg = self._cfgs[key]
            w["enable"].setChecked(cfg.enabled)
            off = np.array([self._veh_dx.get(key, 0.0), 0.0, 0.0])
            disp = self._conv.to_display(np.asarray(cfg.inner, float) + off)
            for axis, s in w["spins"].items():
                s.blockSignals(True)
                s.setValue(u.from_mm(disp[axis]))
                s.blockSignals(False)
            jw = w["joint"]
            jw.blockSignals(True)
            ji = jw.findData(cfg.inner_joint)
            jw.setCurrentIndex(ji if ji >= 0 else 0)
            jw.blockSignals(False)
            for wk, cv in (("cv_in", cfg.max_cv_inner_deg),
                           ("cv_out", cfg.max_cv_outer_deg)):
                w[wk].blockSignals(True)
                w[wk].setValue(cv)
                w[wk].blockSignals(False)
            for wk, v in (("pl_total", u.from_mm(cfg.plunge_total_mm)),
                          ("pl_pct", cfg.plunge_in_pct)):
                w[wk].blockSignals(True)
                w[wk].setValue(v)
                # a fixed joint (Rzeppa) can't plunge — grey the budget out
                w[wk].setEnabled(cfg.plunges)
                w[wk].blockSignals(False)
            spec = self._shock_specs.get(key)
            for sk, v in (("sh_min", getattr(spec, "min_length", 0.0)),
                          ("sh_max", getattr(spec, "max_length", 0.0))):
                w[sk].blockSignals(True)
                w[sk].setValue(u.from_mm(float(v or 0.0)))
                w[sk].blockSignals(False)
        self._loading = False

    def _capture(self) -> None:
        """Read the widgets into self._cfgs."""
        if self._loading:
            return
        u = self._unit
        for key, w in self._w.items():
            cfg = self._cfgs[key]
            cfg.enabled = w["enable"].isChecked()
            disp = np.array([u.to_mm(w["spins"][a].value())
                             for a in range(3)])
            off = np.array([self._veh_dx.get(key, 0.0), 0.0, 0.0])
            cfg.inner = self._conv.from_display(disp) - off
            cfg.inner_joint = w["joint"].currentData()
            cfg.max_cv_inner_deg = w["cv_in"].value()
            cfg.max_cv_outer_deg = w["cv_out"].value()
            cfg.plunge_total_mm = u.to_mm(w["pl_total"].value())
            cfg.plunge_in_pct = w["pl_pct"].value()
            mn = u.to_mm(w["sh_min"].value())
            mx = u.to_mm(w["sh_max"].value())
            if mx > mn > 0.0:
                from ..shock import ShockSpec
                self._shock_specs[key] = ShockSpec(mn, mx)
            else:
                self._shock_specs[key] = None

    def shock_specs(self) -> dict:
        """{axle: ShockSpec|None} as currently entered."""
        self._capture()
        return dict(self._shock_specs)

    def load_shock_specs(self, front, rear) -> None:
        self._shock_specs = {"front": front, "rear": rear}
        self._push_cfgs()

    def _on_edit(self, *_):
        if not self._loading:
            self._capture()
            self.changed.emit()

    def _on_joint_type(self, key: str) -> None:
        """Type combo changed: preset the inner angle to the type's default
        and enable/disable the plunge budget (a fixed joint can't plunge)."""
        if self._loading:
            return
        from ..halfshaft import JOINT_TYPES
        w = self._w[key]
        meta = JOINT_TYPES.get(w["joint"].currentData(), {})
        w["cv_in"].blockSignals(True)
        w["cv_in"].setValue(float(meta.get("angle", w["cv_in"].value())))
        w["cv_in"].blockSignals(False)
        for wk in ("pl_total", "pl_pct"):
            w[wk].setEnabled(bool(meta.get("plunges", True)))
        self._on_edit()

    # ------------------------------------------------------------------
    def configs(self) -> dict:
        self._capture()
        return self._cfgs

    def load(self, front_cfg, rear_cfg) -> None:
        self._cfgs = {"front": front_cfg, "rear": rear_cfg}
        self._push_cfgs()

    def show_status(self, axle_key: str, warnings: list | None) -> None:
        """None = disabled; [] = all clear; else the violations."""
        lbl = self._w[axle_key]["status"]
        if warnings is None:
            lbl.setText("")
        elif not warnings:
            lbl.setText("<span style='color:#2a9d2a'>Within CV limits "
                        "through the whole sweep</span>")
        else:
            lbl.setText("<span style='color:#d03030'>⚠ "
                        + "; ".join(warnings) + "</span>")
