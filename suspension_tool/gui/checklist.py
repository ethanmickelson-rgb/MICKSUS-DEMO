"""Design Checklist — the workflow's traffic lights.

Grades the design LIVE on every edit, per axle where that makes sense:

  articulation   the swept range reached what was requested (edits that
                 shrink it auto-clamp instead of being rejected; this row
                 is where the shortfall shows)
  one 2D sketch  double wishbone only: UCA/LCA bushing axes parallel
  bump steer     |static d(toe)/d(travel)| under the threshold
  RC height      static roll-centre height inside the target band
  CV joints      halfshaft (when modeled) within its rated angle/plunge
  turn radius    front only: outside-front-wheel turning radius at ride
                 height and full rack travel vs the desired goal, plus an
                 embedded Ackermann-percentage graph over the rack sweep

Thresholds are editable right in the panel (including the rack's travel
and the turning-radius goal). Everything green = the design is ready to
Optimize / export; the banner says so.

The intended flow:  Seed -> drag/edit mounts -> checklist green ->
Optimize -> Candidates snapshot -> export/sync.
"""

import numpy as np
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QGridLayout, QGroupBox,
                               QLabel, QVBoxLayout)

from ..units import Unit
from .widgets import CommitSpin

OK = "<span style='color:#2a9d2a'>✓</span>"
BAD = "<span style='color:#d03030'>✗</span>"
NA = "<span style='color:#888888'>—</span>"


def _spin(lo, hi, dec, step):
    s = CommitSpin()
    s.setRange(lo, hi)
    s.setDecimals(dec)
    s.setSingleStep(step)
    return s


ROWS = [
    ("articulation", "Articulates requested travel"),
    ("track", "Built track matches the seeded intent"),
    ("sketch", "One 2D sketch (arm bushing axes)"),
    ("bump_steer", "Static bump steer"),
    ("rc", "Roll centre height in band"),
    ("cv", "Halfshaft CV within limits"),
    ("turn", "Turning radius @ ride (full lock)"),
]


class ChecklistPanel(QGroupBox):
    changed = Signal()   # thresholds edited -> window re-evaluates

    def __init__(self, unit: Unit, parent=None):
        super().__init__("Design checklist", parent)
        self._unit = unit
        lay = QVBoxLayout(self)
        self.banner = QLabel("")
        self.banner.setWordWrap(True)
        lay.addWidget(self.banner)

        grid = QGridLayout()
        grid.addWidget(QLabel("<b>Check</b>"), 0, 0)
        grid.addWidget(QLabel("<b>Front</b>"), 0, 1)
        grid.addWidget(QLabel("<b>Rear</b>"), 0, 2)
        self._cells = {}
        for r, (key, label) in enumerate(ROWS, start=1):
            grid.addWidget(QLabel(label), r, 0)
            for c, axle in ((1, "front"), (2, "rear")):
                cell = QLabel(NA)
                cell.setToolTip("")
                self._cells[(key, axle)] = cell
                grid.addWidget(cell, r, c)
        lay.addLayout(grid)

        thr = QGroupBox("Thresholds")
        tg = QGridLayout(thr)
        self.bs_spin = _spin(0.001, 0.2, 3, 0.005)   # deg/mm
        self.bs_spin.setValue(0.010)
        self.bs_spin.setSuffix(" deg/mm")
        self.rc_lo = _spin(0, 0, 0, 0)               # unit-applied below
        self.rc_hi = _spin(0, 0, 0, 0)
        self._rc_lo_mm, self._rc_hi_mm = 0.0, 12.0 * 25.4
        # Steering: the rack's usable travel from centre (defines "full
        # lock") and the turning-radius goal. Both length-unit aware.
        self.rack_spin = _spin(0, 0, 0, 0)
        self.turn_goal = _spin(0, 0, 0, 0)
        self._rack_mm = 40.0            # ± from centre; team value TBD
        self._turn_goal_mm = 3500.0     # 3.5 m — set your course number
        for w in (self.bs_spin, self.rc_lo, self.rc_hi,
                  self.rack_spin, self.turn_goal):
            w.valueChanged.connect(self._on_thresh)
        tg.addWidget(QLabel("Max |bump steer|"), 0, 0)
        tg.addWidget(self.bs_spin, 0, 1)
        tg.addWidget(QLabel("RC height band"), 1, 0)
        tg.addWidget(self.rc_lo, 1, 1)
        tg.addWidget(self.rc_hi, 1, 2)
        tg.addWidget(QLabel("Rack travel (± from centre)"), 2, 0)
        tg.addWidget(self.rack_spin, 2, 1)
        tg.addWidget(QLabel("Turning radius goal"), 3, 0)
        tg.addWidget(self.turn_goal, 3, 1)
        lay.addWidget(thr)

        # Ackermann-percentage graph over the rack sweep at ride height
        # (100 = ideal, 0 = parallel steer, negative = anti-Ackermann).
        from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
        from matplotlib.figure import Figure
        self._fig = Figure(figsize=(3.4, 1.9), tight_layout=True)
        self._ack_canvas = FigureCanvasQTAgg(self._fig)
        self._ack_canvas.setMinimumHeight(150)
        lay.addWidget(self._ack_canvas)
        self._steer = None           # last steer_sweep result (or None)
        self._apply_unit()

    # ------------------------------------------------------------------
    def _apply_unit(self):
        u = self._unit
        for spin, v, hi in ((self.rc_lo, self._rc_lo_mm, 1000),
                            (self.rc_hi, self._rc_hi_mm, 1000),
                            (self.rack_spin, self._rack_mm, 200),
                            (self.turn_goal, self._turn_goal_mm, 20000)):
            spin.blockSignals(True)
            spin.setRange(u.from_mm(-500), u.from_mm(hi))
            spin.setDecimals(u.decimals)
            spin.setSingleStep(u.step)
            spin.setSuffix(f" {u.label}")
            spin.setValue(u.from_mm(v))
            spin.blockSignals(False)

    def set_units(self, unit: Unit):
        self._capture_thresholds()
        self._unit = unit
        self._apply_unit()

    def _capture_thresholds(self):
        self._rc_lo_mm = self._unit.to_mm(self.rc_lo.value())
        self._rc_hi_mm = self._unit.to_mm(self.rc_hi.value())
        self._rack_mm = self._unit.to_mm(self.rack_spin.value())
        self._turn_goal_mm = self._unit.to_mm(self.turn_goal.value())

    def _on_thresh(self, *_):
        self._capture_thresholds()
        self.changed.emit()

    @property
    def rack_travel_mm(self) -> float:
        return self._rack_mm

    def to_dict(self) -> dict:
        """Thresholds (internal mm) for saving inside the project."""
        return {"bump_steer_deg_per_mm": self.bs_spin.value(),
                "rc_lo_mm": self._rc_lo_mm, "rc_hi_mm": self._rc_hi_mm,
                "rack_mm": self._rack_mm,
                "turn_goal_mm": self._turn_goal_mm}

    def load_dict(self, data: dict) -> None:
        if not data:
            return
        self.bs_spin.blockSignals(True)
        self.bs_spin.setValue(float(data.get("bump_steer_deg_per_mm",
                                             self.bs_spin.value())))
        self.bs_spin.blockSignals(False)
        self._rc_lo_mm = float(data.get("rc_lo_mm", self._rc_lo_mm))
        self._rc_hi_mm = float(data.get("rc_hi_mm", self._rc_hi_mm))
        self._rack_mm = float(data.get("rack_mm", self._rack_mm))
        self._turn_goal_mm = float(data.get("turn_goal_mm",
                                            self._turn_goal_mm))
        self._apply_unit()

    # ------------------------------------------------------------------
    def _set(self, key, axle, ok, tip: str = ""):
        cell = self._cells[(key, axle)]
        cell.setText(NA if ok is None else (OK if ok else BAD))
        cell.setToolTip(tip)
        return ok

    def set_steering(self, steer: dict | None) -> None:
        """Latest front steer_sweep result (rack -> angles/Ackermann/turn
        diameter) or None when the front can't steer. Redraws the graph;
        evaluate() grades the radius row from the same data."""
        self._steer = steer
        self._draw_ackermann()

    def _turn_radius_m(self) -> float:
        """Outside-front-wheel turning radius at full lock (m, nan when
        the sweep is missing or the car barely steers)."""
        if not self._steer or "turn_diameter_m" not in self._steer:
            return float("nan")
        d = self._steer["turn_diameter_m"]
        return float(d[-1]) / 2.0 if len(d) else float("nan")

    def _draw_ackermann(self) -> None:
        self._fig.clear()
        ax = self._fig.add_subplot(111)
        ax.set_title("Ackermann % over rack sweep (ride height)",
                     fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.3)
        u = self._unit
        if self._steer is not None and "rack_mm" in self._steer:
            x = np.array([u.from_mm(v) for v in self._steer["rack_mm"]])
            y = np.asarray(self._steer["ackermann_pct"], float)
            if np.any(np.isfinite(y)):
                ax.plot(x, y, lw=1.4)
                ax.axhline(100.0, ls="--", lw=0.8, alpha=0.6)
                ax.axhline(0.0, ls=":", lw=0.8, alpha=0.6)
            else:
                ax.text(0.5, 0.5, "steers too little to grade",
                        ha="center", va="center", fontsize=8,
                        transform=ax.transAxes)
            ax.set_xlabel(f"rack ({u.label})", fontsize=7)
            ax.set_ylabel("%", fontsize=7)
        else:
            ax.text(0.5, 0.5, "no steerable front axle", ha="center",
                    va="center", fontsize=8, transform=ax.transAxes)
        try:
            self._ack_canvas.draw_idle()
        except Exception:
            pass    # never let a paint problem break the checklist

    def evaluate(self, axles: dict) -> None:
        """axles: name -> the window's Axle records (hp, sweep, ranges,
        rc_h, hs_warns). Cheap — reads what's already computed."""
        from ..geometry import DoubleWishbonePoints, sketch_planarity
        all_ok, any_geo = True, False
        for name, ax in axles.items():
            if ax.hp is None or ax.sweep is None:
                for key, _ in ROWS:
                    self._set(key, name, None)
                continue
            any_geo = True
            short_b = ax.req_bump - ax.bump
            short_d = ax.req_droop - ax.droop
            ok = self._set(
                "articulation", name, short_b < 1.0 and short_d < 1.0,
                f"reaches +{ax.bump:.0f}/-{ax.droop:.0f} of requested "
                f"+{ax.req_bump:.0f}/-{ax.req_droop:.0f} mm")
            all_ok &= bool(ok)
            # The seed form's track_width is what was ASKED for. Hardpoint
            # edits move the real track away from it, and roll angle is
            # derived from the real one -- so a silent divergence biases
            # every roll number. Graded against the car's own size (1%)
            # so it means the same thing on a Baja car and a 1/10 model.
            built = (2.0 * abs(float(ax.hp.wheel_center[1]))
                     if ax.hp is not None else None)
            want = (float(ax.last_setup.track_width)
                    if ax.last_setup is not None else None)
            if built is None or want is None or built <= 0.0:
                self._set("track", name, None, "nothing to compare")
            else:
                gap = want - built
                ok = abs(gap) <= 0.01 * built
                self._set("track", name, ok,
                          f"built {built:.0f} mm vs seeded {want:.0f} mm"
                          + ("" if ok else
                             f" ({gap:+.0f} mm) — the geometry wins; "
                             "re-fill the setup form from it"))
                all_ok &= bool(ok)
            if isinstance(ax.hp, DoubleWishbonePoints):
                p = sketch_planarity(ax.hp)
                mis, yaw = p["axis_misalign_deg"], p["axis_yaw_deg"]
                twist = p["kingpin_off_plane_deg"]
                want = float(getattr(ax.last_setup, "sketch_yaw_deg",
                                     0.0) or 0.0)
                dev = abs(yaw - want)
                # Grade the kingpin twist ONLY when this axle follows the
                # kickup (planar kingpin). In independent-caster mode the
                # kingpin legitimately sits off the sketch — just note it.
                indep = bool(getattr(ax.last_setup, "independent_caster",
                                     False))
                twist_bad = (not indep) and twist > 0.5
                ok = self._set(
                    "sketch", name,
                    mis <= 0.1 and dev <= 0.25 and not twist_bad,
                    f"axes {mis:.3f} deg apart, plane at {yaw:.2f} deg "
                    f"vs {want:.1f} deg design yaw, kingpin {twist:.2f} deg "
                    "off sketch" + ("" if indep
                                    else " (Re-square planarizes it)"))
                all_ok &= bool(ok)
            else:
                # The toe-holding layouts keep their chassis bushing axis
                # LEVEL on purpose: they have no toe link, so tilting it
                # would put ~0.19 deg of bump toe per degree of kickup into
                # a corner that cannot correct it. The price is ~0%
                # anti-dive / anti-squat, which reads like a broken number
                # unless we say so here.
                from ..geometry import carrier_planarity
                from ..harm_rear import HArmRearPoints
                from ..loaded_halfshaft import LoadedHalfshaftPoints
                from ..trailing_arm import TrailingArmPoints
                # The toe-holding layouts keep their bushing axis LEVEL on
                # purpose: no toe link, so tilting it would put ~0.19 deg of
                # bump toe per degree of kickup into a corner that cannot
                # correct it. ~0% anti-dive/anti-squat is the price, and it
                # reads like a broken number unless we say so.
                trade = ("  |  this layout holds toe with a level bushing "
                         "axis, trading anti-dive/anti-squat (~0%) for toe "
                         "control"
                         if isinstance(ax.hp, (TrailingArmPoints,
                                               HArmRearPoints,
                                               LoadedHalfshaftPoints))
                         else "")
                if hasattr(ax.hp, "arm_inner_front"):
                    # Single-arm carrier (C-hub, loaded halfshaft, H-arm):
                    # the bushing axis and the carrier pin are both meant to
                    # be normal to the same 2D sketch, so grade that the way
                    # a double wishbone's two arm axes are graded.
                    cp = carrier_planarity(ax.hp)
                    want = float(getattr(ax.last_setup, "sketch_yaw_deg",
                                         0.0) or 0.0)
                    dev = abs(cp["axis_yaw_deg"] - want)
                    ok = self._set(
                        "sketch", name,
                        cp["axis_misalign_deg"] <= 0.1 and dev <= 0.25,
                        f"arm axis and carrier pin "
                        f"{cp['axis_misalign_deg']:.3f} deg apart, plane at "
                        f"{cp['axis_yaw_deg']:.2f} deg vs {want:.1f} deg "
                        f"design yaw, kickup {cp['sketch_kickup_deg']:.1f} "
                        f"deg (Re-square fixes it)" + trade)
                    all_ok &= bool(ok)
                else:
                    self._set("sketch", name, None,
                              "no A-arms on this type" + trade)
            k0 = int(np.argmin(np.abs(ax.sweep["travel_mm"])))
            bs = abs(float(ax.sweep["bump_steer_deg_per_mm"][k0]))
            ok = self._set("bump_steer", name, bs <= self.bs_spin.value(),
                           f"{bs:.4f} deg/mm at ride")
            all_ok &= bool(ok)
            rc = float(ax.sweep["roll_center_height_mm"][k0])
            if np.isfinite(rc):
                ok = self._set(
                    "rc", name,
                    self._rc_lo_mm <= rc <= self._rc_hi_mm,
                    f"{self._unit.fmt(rc)} {self._unit.label} above ground")
                all_ok &= bool(ok)
            else:
                self._set("rc", name, None, "undefined at ride")
            if ax.hs_warns is None:
                self._set("cv", name, None, "no halfshaft modeled")
            else:
                ok = self._set("cv", name, not ax.hs_warns,
                               "; ".join(ax.hs_warns) or "within limits")
                all_ok &= bool(ok)
            if name != "front":
                self._set("turn", name, None, "front axle steers")
            else:
                r = self._turn_radius_m()
                if np.isfinite(r):
                    goal = self._turn_goal_mm / 1000.0
                    ok = self._set(
                        "turn", name, r <= goal,
                        f"{r:.2f} m ({r * 3.28084:.1f} ft) to the outside "
                        f"front wheel at ±{self._rack_mm:.0f} mm rack — "
                        f"goal {goal:.2f} m")
                    all_ok &= bool(ok)
                else:
                    self._set("turn", name, None,
                              "no steer sweep (rack travel 0 or "
                              "unsteerable type)")
        if not any_geo:
            self.banner.setText("")
        elif all_ok:
            self.banner.setText(
                "<b><span style='color:#2a9d2a'>All checks green — ready "
                "to Optimize / snapshot / export.</span></b>")
        else:
            self.banner.setText(
                "<b><span style='color:#d07030'>Open items — hover a ✗ "
                "for the number behind it.</span></b>")
