"""Sweep dialogs (roll / pitch / steering): parameter spinboxes on top, an
embedded matplotlib figure below, a Run button in between. Each dialog is
non-modal so it can sit next to the main window while you tweak geometry
— hit Run again to re-sweep the current design.

Everything displays in the project's current unit; angles stay degrees.
"""

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PySide6.QtWidgets import (QDialog, QHBoxLayout, QLabel,
                               QPushButton, QVBoxLayout)

from ..sweeps import bump_steer_map, pitch_sweep, roll_sweep, steer_sweep
from .widgets import CommitSpin


class _SweepDialog(QDialog):
    """Shared shell: parameter row + Run + a 2x2 figure."""

    TITLE = "Sweep"

    def __init__(self, window, parent=None):
        super().__init__(parent)
        self.window = window          # the MainWindow (live geometry access)
        self.setWindowTitle(self.TITLE)
        self.resize(900, 640)
        lay = QVBoxLayout(self)
        self.param_row = QHBoxLayout()
        lay.addLayout(self.param_row)
        run = QPushButton("Run sweep")
        run.clicked.connect(self.run)
        self.param_row.addWidget(run)
        self.param_row.addStretch()
        self.canvas = FigureCanvasQTAgg(Figure(figsize=(9, 6),
                                               tight_layout=True))
        lay.addWidget(self.canvas)
        self.status = QLabel("")
        lay.addWidget(self.status)

    def _spin(self, label, value, lo, hi, decimals=1, suffix=""):
        self.param_row.insertWidget(self.param_row.count() - 2,
                                    QLabel(label))
        s = CommitSpin()
        s.setRange(lo, hi)
        s.setDecimals(decimals)
        s.setValue(value)
        s.setSuffix(suffix)
        self.param_row.insertWidget(self.param_row.count() - 2, s)
        return s

    def _axes(self, n):
        fig = self.canvas.figure
        fig.clf()
        axes = fig.subplots(2, int(np.ceil(n / 2)), squeeze=False).ravel()
        for ax in axes[n:]:
            ax.set_visible(False)
        return axes

    def _plot(self, ax, x, ys, xlabel, title):
        for label, y in ys:
            ax.plot(x, y, lw=1.5, label=label)
        if len(ys) > 1:
            ax.legend(fontsize=7)
        ax.set_xlabel(xlabel, fontsize=8)
        ax.set_title(title, fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
        ax.axvline(0.0, color="gray", lw=0.6, ls=":")

    def run(self):  # pragma: no cover - overridden
        raise NotImplementedError


class RollSweepDialog(_SweepDialog):
    TITLE = "Roll sweep (opposite wheel travel)"

    def __init__(self, window, parent=None):
        super().__init__(window, parent)
        u = window.unit
        self.max_travel = self._spin("± wheel travel", u.from_mm(75.0), 1,
                                     u.from_mm(300.0), u.decimals,
                                     f" {u.label}")
        self.run()

    def run(self):
        w = self.window
        axle = w.active
        if axle.solver is None:
            return
        u = w.unit
        # Track from the GEOMETRY, never the seed form: the seeded value
        # is what was asked for, and any hardpoint edit moves the real
        # track away from it. Roll angle is derived from this.
        track = w._axle_track(axle)
        try:
            res = roll_sweep(axle.solver, track,
                             u.to_mm(self.max_travel.value()))
        except ValueError as e:
            self.status.setText(f"sweep failed: {e}")
            return
        axes = self._axes(4)
        x = res["roll_deg"]
        xl = "roll angle (deg, + = onto left wheels)"
        self._plot(axes[0], x, [("left", res["camber_left_deg"]),
                                ("right", res["camber_right_deg"])],
                   xl, "Ground camber (deg)")
        self._plot(axes[1], x, [("left", res["toe_left_deg"]),
                                ("right", res["toe_right_deg"])],
                   xl, "Ground toe (deg) — roll steer")
        self._plot(axes[2], x, [("RC lateral", u.from_mm(res["rc_y_mm"]))],
                   xl, f"Roll-centre lateral migration ({u.label})")
        self._plot(axes[3], x, [("RC height", u.from_mm(res["rc_z_mm"]))],
                   xl, f"Roll-centre height ({u.label})")
        self.canvas.draw_idle()
        self.status.setText(
            f"{w.active_key} axle, track {u.fmt(track)} {u.label}")


class PitchSweepDialog(_SweepDialog):
    TITLE = "Pitch sweep (front + rear coupled)"

    def __init__(self, window, parent=None):
        super().__init__(window, parent)
        self.max_pitch = self._spin("± pitch", 3.0, 0.5, 15.0, 1, " deg")
        self.run()

    def run(self):
        w = self.window
        f, r = w.axles["front"], w.axles["rear"]
        if f.solver is None or r.solver is None:
            self.status.setText(
                "pitch needs BOTH axles designed — switch to the rear once "
                "to auto-seed it")
            return
        params = w.vehicle_panel.params()
        u = w.unit
        try:
            res = pitch_sweep(f.solver, r.solver, params.wheelbase,
                              params.cg_behind_front,
                              self.max_pitch.value())
        except ValueError as e:
            self.status.setText(f"sweep failed: {e}")
            return
        axes = self._axes(4)
        x = res["pitch_deg"]
        xl = "pitch angle (deg, + = nose down)"
        self._plot(axes[0], x, [("front (ground)",
                                 res["caster_front_ground_deg"])],
                   xl, "Front caster vs ground (deg)")
        self._plot(axes[1], x, [("front", res["camber_front_deg"]),
                                ("rear", res["camber_rear_deg"])],
                   xl, "Camber (deg)")
        self._plot(axes[2], x,
                   [("Δ wheelbase", u.from_mm(res["wheelbase_change_mm"]))],
                   xl, f"Wheelbase change ({u.label})")
        self._plot(axes[3], x,
                   [("front", u.from_mm(res["travel_front_mm"])),
                    ("rear", u.from_mm(res["travel_rear_mm"]))],
                   xl, f"Wheel travel used ({u.label})")
        self.canvas.draw_idle()
        self.status.setText(
            f"wheelbase {u.fmt(params.wheelbase)} {u.label}, CG "
            f"{u.fmt(params.cg_behind_front)} {u.label} behind front")


class SteerSweepDialog(_SweepDialog):
    TITLE = "Steering sweep (Ackermann)"

    def __init__(self, window, parent=None):
        super().__init__(window, parent)
        u = window.unit
        self.rack = self._spin("± rack", u.from_mm(window.steer_limit), 1,
                               u.from_mm(150.0), u.decimals, f" {u.label}")
        self.travel = self._spin("at travel", u.from_mm(0.0),
                                 -u.from_mm(300.0), u.from_mm(300.0),
                                 u.decimals, f" {u.label}")
        self.run()

    def run(self):
        w = self.window
        front = w.axles["front"]
        if front.solver is None:
            return
        u = w.unit
        params = w.vehicle_panel.params()
        track = w._axle_track(front)      # geometry, not the seed form
        try:
            res = steer_sweep(front.solver, u.to_mm(self.rack.value()),
                              u.to_mm(self.travel.value()), track,
                              params.wheelbase)
        except ValueError as e:
            self.status.setText(f"sweep failed: {e}")
            return
        axes = self._axes(4)
        x = u.from_mm(res["rack_mm"])
        xl = f"rack displacement ({u.label}, + = toward left corner)"
        self._plot(axes[0], x, [("left", res["steer_left_deg"]),
                                ("right", res["steer_right_deg"])],
                   xl, "Wheel steer angle (deg, + = left)")
        self._plot(axes[1], x, [("% Ackermann", res["ackermann_pct"])],
                   xl, "Percent Ackermann")
        self._plot(axes[2], x, [("left", res["camber_left_deg"]),
                                ("right", res["camber_right_deg"])],
                   xl, "Camber with steer (deg)")
        self._plot(axes[3], x, [("diameter", res["turn_diameter_m"])],
                   xl, "Est. outside turn diameter (m)")
        self.canvas.draw_idle()
        self.status.setText(
            f"at travel {self.travel.value():+.1f} {u.label}; low-speed "
            "geometry estimate, kingpin track approximated by wheel track")


class BumpSteerMapDialog(_SweepDialog):
    TITLE = "Bump-steer map (travel × rack)"

    def __init__(self, window, parent=None):
        super().__init__(window, parent)
        u = window.unit
        self.rack = self._spin("± rack", u.from_mm(window.steer_limit), 1,
                               u.from_mm(150.0), u.decimals, f" {u.label}")
        self.run()

    def run(self):
        w = self.window
        front = w.axles["front"]
        if front.solver is None:
            return
        u = w.unit
        travels = np.linspace(-front.droop, front.bump, 21)
        racks = np.linspace(-u.to_mm(self.rack.value()),
                            u.to_mm(self.rack.value()), 13)
        try:
            res = bump_steer_map(front.solver, travels, racks)
        except ValueError as e:
            self.status.setText(f"map failed: {e}")
            return
        fig = self.canvas.figure
        fig.clf()
        ax_map, ax_lines = fig.subplots(1, 2)
        t_disp = u.from_mm(res["travel_mm"])
        r_disp = u.from_mm(res["rack_mm"])
        # left: the toe map itself
        pc = ax_map.pcolormesh(r_disp, t_disp, res["toe_deg"],
                               shading="auto", cmap="RdBu_r")
        fig.colorbar(pc, ax=ax_map, label="toe (deg, + = toe-in)")
        ax_map.set_xlabel(f"rack ({u.label})", fontsize=8)
        ax_map.set_ylabel(f"wheel travel ({u.label})", fontsize=8)
        ax_map.set_title("Left-wheel toe over travel × rack", fontsize=9)
        # right: toe-vs-travel at a handful of rack positions
        for j in np.linspace(0, len(r_disp) - 1, 5).astype(int):
            ax_lines.plot(t_disp, res["toe_deg"][:, j], lw=1.4,
                          label=f"rack {r_disp[j]:+.2f} {u.label}")
        ax_lines.legend(fontsize=7)
        ax_lines.grid(True, alpha=0.3)
        ax_lines.axvline(0.0, color="gray", lw=0.6, ls=":")
        ax_lines.set_xlabel(f"wheel travel ({u.label})", fontsize=8)
        ax_lines.set_title("Toe vs travel at fixed rack", fontsize=9)
        self.canvas.draw_idle()
        self.status.setText(
            f"{len(res['travel_mm'])} travels × {len(res['rack_mm'])} rack "
            "positions, front axle")


class RideBodeDialog(_SweepDialog):
    """Quarter-car frequency response — the Bode view of ride and grip.

    Four channels, front and rear, each drawn as a BAND between the bump and
    rebound damping curves (one linear coefficient cannot represent an
    asymmetric damper, so we show the range the real car lives in). The two
    resonances are marked, and the terrain table underneath converts them
    into the road speeds at which evenly-spaced bumps drive them — which is
    what makes a resonance actionable rather than trivia.
    """

    TITLE = "Ride / grip frequency response (quarter-car Bode)"

    CHANNELS = [
        ("body_accel", "Body acceleration — harshness", "g per in of road"),
        ("tire_deflection", "Tire deflection — GRIP", "in per in of road"),
        ("body_travel", "Body motion — isolation", "in per in of road"),
        ("suspension_travel", "Suspension travel used", "in per in of road"),
    ]

    def __init__(self, window, parent=None):
        super().__init__(window, parent)
        from PySide6.QtWidgets import QComboBox, QLineEdit
        self.model = QComboBox()
        self.model.addItem("Quarter car (one corner, 2 DOF)", "quarter")
        self.model.addItem("Full car (7 DOF, wheelbase filtering)", "full")
        self.model.currentIndexChanged.connect(self.run)
        self.param_row.insertWidget(self.param_row.count() - 2,
                                    QLabel("model"))
        self.param_row.insertWidget(self.param_row.count() - 2, self.model)
        self.speed = self._spin("speed", 20.0, 1.0, 80.0, 1, " mph")
        self.speed.setToolTip(
            "Full car only: the rear axle meets each bump L/V after the "
            "front, so the bounce/pitch split depends on SPEED.")
        self.speed.valueChanged.connect(self.run)
        self.spacings = QLineEdit("2, 6, 12, 25")
        self.spacings.setToolTip(
            "Bump-to-bump spacings in FEET, comma separated. Each becomes a "
            "row in the terrain table: the speed at which that spacing "
            "drives the body and wheel-hop resonances.")
        self.spacings.setMaximumWidth(160)
        self.spacings.returnPressed.connect(self.run)
        self.param_row.insertWidget(self.param_row.count() - 2,
                                    QLabel("terrain spacing (ft)"))
        self.param_row.insertWidget(self.param_row.count() - 2, self.spacings)
        self.run()

    def _terrain(self):
        out = {}
        for tok in self.spacings.text().split(","):
            tok = tok.strip()
            if not tok:
                continue
            try:
                v = float(tok)
            except ValueError:
                continue
            if v > 0:
                out[f"{v:g} ft"] = v
        return out or None

    def run(self):
        w = self.window
        try:
            dyn = w.dynamics_panel.current_inputs()
        except Exception as e:                      # panel not ready
            self.status.setText(f"dynamics unavailable: {e}")
            return
        if self.model.currentData() == "full":
            self._run_full(dyn)
        else:
            self._run_quarter(dyn)

    # ------------------------------------------------------------------
    def _run_full(self, dyn):
        """Whole-vehicle 7-DOF response at one road speed."""
        from ..full_car import full_car_report, wheelbase_filter_speeds
        try:
            rep = full_car_report(dyn, speed_mph=self.speed.value())
        except (KeyError, ValueError, np.linalg.LinAlgError) as e:
            self.status.setText(f"full-car model failed: {e}")
            return
        f = rep.freq_hz
        res = rep.response
        axes = self._axes(4)
        # 1: body acceleration at the CG and at both axle stations
        for lab, key, col in (("CG", "heave_accel_g", "tab:green"),
                              ("front axle", "accel_front_axle_g",
                               "tab:blue"),
                              ("rear axle", "accel_rear_axle_g", "tab:red")):
            axes[0].plot(f, res[key], lw=1.4, color=col, label=lab)
        axes[0].set_title("Body acceleration — harshness", fontsize=9)
        axes[0].set_ylabel("g per in of road", fontsize=7)
        # 2: pitch and heave — the wheelbase comb lives here
        axes[1].plot(f, res["pitch_deg_per_in"], lw=1.4, color="tab:purple",
                     label="pitch")
        axes[1].set_ylabel("deg per in of road", fontsize=7)
        axes[1].set_title("Pitch — nulls where the axles are in phase",
                          fontsize=9)
        axes[2].plot(f, res["heave"], lw=1.4, color="tab:green",
                     label="heave")
        axes[2].set_ylabel("in per in of road", fontsize=7)
        axes[2].set_title("Heave — nulls where the axles are opposed",
                          fontsize=9)
        # mark the comb: pitch nulls at n*V/L, heave nulls at (2n+1)*V/2L
        v_in_s = max(self.speed.value(), 1e-6) * 17.6
        base = v_in_s / rep.car.wheelbase_in
        for n in range(1, 6):
            if n * base < f[-1]:
                axes[1].axvline(n * base, color="gray", lw=0.7, ls=":")
        for n in range(0, 6):
            if (2 * n + 1) * base / 2.0 < f[-1]:
                axes[2].axvline((2 * n + 1) * base / 2.0, color="gray",
                                lw=0.7, ls=":")
        # 3: dynamic tire load — the grip channel
        for corner, col in (("FL", "tab:blue"), ("RL", "tab:red")):
            axes[3].plot(f, res[f"tire_load_{corner}_lb_per_in"], lw=1.4,
                         color=col, label=f"{corner} tire load")
        axes[3].set_title("Dynamic tire load — GRIP", fontsize=9)
        axes[3].set_ylabel("lb per in of road", fontsize=7)
        for ax in axes:
            ax.set_xscale("log")
            ax.set_xlim(f[0], f[-1])
            ax.set_xlabel("frequency (Hz)", fontsize=8)
            ax.grid(True, which="both", alpha=0.3)
            ax.tick_params(labelsize=8)
            ax.legend(fontsize=6)
        # mark the body modes on every panel
        for m in rep.modes:
            if m["label"] in ("heave/bounce", "pitch", "roll"):
                for ax in axes:
                    ax.axvline(m["freq_hz"], color="k", lw=0.6, ls="--",
                               alpha=0.4)
        self.canvas.draw_idle()
        modes = "  ".join(
            f"{m['label']} {m['freq_hz']:.2f}" for m in rep.modes
            if m["participation"] > 0.25)
        comb = wheelbase_filter_speeds(rep.car.wheelbase_in,
                                       rep.car.body_modes().get("pitch", 0.0))
        pit = ", ".join(f"{s:.0f}" for s in comb["pitch_null_mph"][:3])
        self.status.setText(
            f"7-DOF full car at {rep.speed_mph:.0f} mph — modes (Hz): {modes}"
            f"\nDotted = wheelbase comb at this speed (pitch nulls every "
            f"{base:.2f} Hz = V/L). "
            f"Speeds that put the PITCH MODE on a bounce-only point: {pit} mph."
            f"\n⚠ Inertias are estimates (pitch from Olley DI=1, roll from "
            f"the Dynamics panel input) — see Help.")

    # ------------------------------------------------------------------
    def _run_quarter(self, dyn):
        from ..ride_freq import ride_report
        w = self.window
        terrain = self._terrain()
        try:
            reports = {a: ride_report(dyn, a, terrain=terrain)
                       for a in ("front", "rear")}
        except (KeyError, ValueError) as e:
            self.status.setText(f"could not build the quarter car: {e}")
            return
        axes = self._axes(4)
        colors = {"front": "tab:blue", "rear": "tab:red"}
        for ax, (key, title, ylab) in zip(axes, self.CHANNELS):
            for axle, rep in reports.items():
                f = rep.freq_hz
                b = rep.band
                ax.fill_between(f, b[f"{key}_min"], b[f"{key}_max"],
                                color=colors[axle], alpha=0.22,
                                label=f"{axle} (bump→rebound)")
                ax.plot(f, 0.5 * (b[f"{key}_min"] + b[f"{key}_max"]),
                        lw=1.4, color=colors[axle])
                ax.axvline(rep.body_hz, color=colors[axle], lw=0.8, ls=":")
                ax.axvline(rep.wheel_hop_hz, color=colors[axle], lw=0.8,
                           ls="--")
            ax.set_xscale("log")
            ax.set_xlim(f[0], f[-1])
            ax.set_xlabel("frequency (Hz)", fontsize=8)
            ax.set_ylabel(ylab, fontsize=7)
            ax.set_title(title, fontsize=9)
            ax.grid(True, which="both", alpha=0.3)
            ax.tick_params(labelsize=8)
            ax.legend(fontsize=6)
        # isolation reference: below 1.0 the body moves less than the road
        axes[2].axhline(1.0, color="gray", lw=0.8, ls="-.")
        self.canvas.draw_idle()
        f_rep = reports["front"]
        r_rep = reports["rear"]
        rows = []
        for t in f_rep.terrain:
            rows.append(f"{t['feature']}: body {t['body_mph']:.0f} / hop "
                        f"{t['wheel_hop_mph']:.0f} mph")
        warn = [n for n in f_rep.notes[2:] + r_rep.notes[2:]]
        self.status.setText(
            f"dotted = body mode (front {f_rep.body_hz:.2f} Hz, rear "
            f"{r_rep.body_hz:.2f} Hz)   dashed = WHEEL HOP (front "
            f"{f_rep.wheel_hop_hz:.2f} Hz, rear {r_rep.wheel_hop_hz:.2f} Hz)"
            f"   ζ bump {f_rep.quarter_car.zeta_bump:.2f}/"
            f"{r_rep.quarter_car.zeta_bump:.2f}, rebound "
            f"{f_rep.quarter_car.zeta_rebound:.2f}/"
            f"{r_rep.quarter_car.zeta_rebound:.2f}\n"
            "Speeds that excite each mode — " + ";  ".join(rows)
            + (("\n⚠ " + warn[0]) if warn else ""))


class YawMomentDialog(_SweepDialog):
    """Milliken Moment Method yaw-moment diagram — the manoeuvring envelope.

    Everything else in this tool describes one operating point. This is the
    whole envelope: sweep body slip (beta) against steer (delta) and plot
    yaw moment against lateral acceleration.

    How to read it. WIDTH is the lateral g available. HEIGHT is spare yaw
    moment — how hard the car can still rotate. The Cn = 0 line is TRIM, a
    steady turn; where the envelope crosses it is the cornering the car
    will actually hold. The sign of Cn at maximum Ay is the LIMIT BALANCE:
    below the axis the car pushes, above it the car rotates.

    Drop the speed and watch the constant-beta lines splay out — that is
    the car becoming a pig at low speed, and it is a real effect, not a
    plotting artifact: at low speed a given lateral acceleration demands a
    much higher yaw rate, which saturates the rear on yaw-rate slip alone.
    """

    TITLE = "Yaw-moment diagram (Milliken Moment Method)"

    def __init__(self, window, parent=None):
        super().__init__(window, parent)
        from PySide6.QtWidgets import QComboBox
        self.speed = self._spin("speed", 25.0, 3.0, 80.0, 1, " mph")
        self.speed.valueChanged.connect(self.run)
        self.mode = QComboBox()
        self.mode.addItem("Constant speed (general track)", "speed")
        self.mode.addItem("Constant radius (one corner)", "radius")
        self.mode.currentIndexChanged.connect(self.run)
        self.param_row.insertWidget(self.param_row.count() - 2, QLabel("mode"))
        self.param_row.insertWidget(self.param_row.count() - 2, self.mode)
        self.radius = self._spin("radius", 60.0, 10.0, 500.0, 0, " ft")
        self.radius.valueChanged.connect(self.run)
        self.surface = QComboBox()
        from ..tire import SURFACE_PEAK_MU
        for key in SURFACE_PEAK_MU:
            self.surface.addItem(f"{key} (μ {SURFACE_PEAK_MU[key]:.2f})", key)
        self.surface.setCurrentIndex(
            list(SURFACE_PEAK_MU).index("dry_hardpack"))
        self.surface.currentIndexChanged.connect(self.run)
        self.param_row.insertWidget(self.param_row.count() - 2,
                                    QLabel("surface"))
        self.param_row.insertWidget(self.param_row.count() - 2, self.surface)

    def _vehicle(self):
        """Build the MMM car from the Dynamics panel, so the diagram runs
        on the design that is open."""
        import numpy as np
        from ..tire import surface_tire
        from ..yaw_moment import YawMomentVehicle
        w = self.window
        from ..dynamics import compute as dyn_compute
        inp = w.dynamics_panel.current_inputs()
        try:
            res = dyn_compute(inp)
        except Exception:
            res = {}
        tire = surface_tire(self.surface.currentData())
        wf = float(getattr(inp, "front_axle_lb", 300.0))
        wr = float(getattr(inp, "rear_axle_lb", 300.0))
        # dynamics.compute names them *_lbftdeg (lb-ft per degree)
        kf = float(res.get("roll_rate_front_lbftdeg", 0.0) or 0.0)
        kr = float(res.get("roll_rate_rear_lbftdeg", 0.0) or 0.0)
        if kf <= 0.0 or kr <= 0.0:      # dynamics has not produced them
            kf, kr = 64.0, 77.0
        return YawMomentVehicle(
            weight_fl_lb=wf / 2.0, weight_fr_lb=wf / 2.0,
            weight_rl_lb=wr / 2.0, weight_rr_lb=wr / 2.0,
            track_front_in=float(getattr(inp, "track_front_in", 51.0)),
            track_rear_in=float(getattr(inp, "track_rear_in", 51.0)),
            wheelbase_in=float(getattr(inp, "wheelbase_in", 59.0)),
            cg_height_in=float(getattr(inp, "cg_height_in", 21.0)),
            roll_centre_front_in=float(getattr(inp, "rc_front_in", 6.0)),
            roll_centre_rear_in=float(getattr(inp, "rc_rear_in", 6.0)),
            roll_rate_front_ftlb_deg=kf, roll_rate_rear_ftlb_deg=kr,
            tire_front=tire, tire_rear=tire, name="current design")

    def run(self) -> None:
        import numpy as np
        from ..yaw_moment import yaw_moment_diagram
        self.radius.setEnabled(self.mode.currentData() == "radius")
        veh = self._vehicle()
        betas = np.linspace(-10.0, 10.0, 21)
        steers = np.linspace(-20.0, 20.0, 21)
        d = yaw_moment_diagram(
            veh, self.speed.value(), betas, steers,
            radius_ft=(self.radius.value()
                       if self.mode.currentData() == "radius" else None))
        fig = self.canvas.figure
        fig.clear()
        ax = fig.add_subplot(111)
        # constant-beta lines (red) and constant-steer lines (blue): the
        # classic MMM grid. Where they splay out, the car is misbehaving.
        for i in range(len(betas)):
            ax.plot(d.ay_g[i, :], d.cn[i, :], color="tab:red", lw=0.9,
                    alpha=0.75)
        for j in range(len(steers)):
            ax.plot(d.ay_g[:, j], d.cn[:, j], color="tab:blue", lw=0.9,
                    alpha=0.75)
        ax.axhline(0.0, color="black", lw=1.0)
        ax.axvline(0.0, color="black", lw=0.8, ls=":")
        s = d.summary()
        ax.set_xlabel("lateral acceleration $A_y$ (g)", fontsize=9)
        ax.set_ylabel(r"yaw moment coefficient $C_n = N/(W\cdot L)$",
                      fontsize=9)
        ax.set_title(
            f"{d.vehicle_name} — {d.speed_mph:.0f} mph"
            + (f", R = {d.radius_ft:.0f} ft" if d.radius_ft else "")
            + f"   [red = constant β, blue = constant steer]", fontsize=9)
        ax.grid(True, alpha=0.3)
        self.canvas.draw_idle()
        verdict = ("STABLE" if s["stable"]
                   else "UNSTABLE — the driver is doing the stabilising")
        self.status.setText(
            f"limit {s['limit_ay_g']:.2f} g · trim {s['trim_ay_g']:.2f} g · "
            f"stability index {s['stability_index']:+.3f} Cn/g ({verdict}) · "
            f"control moment gain {s['control_moment_gain']:+.2f} Cn/rad · "
            f"{s['balance']}")
