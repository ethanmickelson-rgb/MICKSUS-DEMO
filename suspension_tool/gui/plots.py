"""Metric-vs-travel plots (matplotlib embedded in Qt).

Eight panels — camber, toe, caster, roll-centre height, motion ratio,
bump steer, half-track change, wheel recession — each against wheel
travel, with a marker line at the travel shown in the 3D view. The user
can toggle which panels are visible (the grid re-flows) and switch dark
mode. Length axes follow the project display unit. A baseline sweep, when
set, draws dashed "before" curves for optimization comparisons.
"""

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from ..units import Unit

# (metric key, base title, kind). kind: "ang" deg, "len" length, "ratio",
# "rate" degrees per length unit.
PANELS = [
    ("camber_deg", "Camber", "ang"),
    ("toe_deg", "Toe", "ang"),
    ("caster_deg", "Caster", "ang"),
    ("roll_center_height_mm", "Roll centre h", "len"),
    ("motion_ratio", "Motion ratio (shock/wheel)", "ratio"),
    ("bump_steer_deg_per_mm", "Bump steer", "rate"),
    ("half_track_change_mm", "Half-track change", "len"),
    ("wheel_recession_mm", "Wheel recession (rear +)", "len"),
    ("anti_squat_pct", "Anti-squat (drive) %", "ratio"),
    ("anti_dive_pct", "Anti-dive/lift (brake) %", "ratio"),
    ("cv_max_deg", "CV angle (worst joint)", "ang"),
    ("plunge_mm", "Halfshaft plunge", "len"),
]

LIGHT = {"fig": "white", "ax": "white", "text": "black", "grid": 0.3}
DARK = {"fig": "#232629", "ax": "#2b2f33", "text": "#dddddd", "grid": 0.25}


class MetricPlots(FigureCanvasQTAgg):
    def __init__(self, unit: Unit, parent=None):
        self._fig = Figure(figsize=(9, 4), tight_layout=True)
        super().__init__(self._fig)
        self.setParent(parent)
        self._unit = unit
        self._visible = [key for key, _, _ in PANELS]
        self._dark = False
        self._markers = []
        self._marker_x = 0.0
        self._sweep = None
        self._snapshots = []   # (label, travels_mm, metrics) design overlays
        # hover readout: axes -> (label, kind, x_disp, y_disp) for the live
        # curve, so moving the mouse shows the exact value at that travel.
        self._axmap = {}
        self._hover_annot = None
        self.mpl_connect("motion_notify_event", self._on_hover)
        self.mpl_connect("figure_leave_event", lambda _e: self._clear_hover())

    # -- configuration --------------------------------------------------
    def set_units(self, unit: Unit) -> None:
        self._unit = unit
        self._redraw()

    def set_visible_panels(self, keys) -> None:
        """Show only these metric panels; the grid re-flows to fit."""
        self._visible = [k for k, _, _ in PANELS if k in set(keys)]
        self._redraw()

    def set_dark(self, dark: bool) -> None:
        self._dark = dark
        self._redraw()

    def set_sweep(self, travels_mm, metrics, baseline=None) -> None:
        """New curves (after an edit). baseline, if given, is a
        (travels_mm, metrics) pair drawn dashed as the "before" overlay."""
        self._sweep = (travels_mm, metrics, baseline)
        self._redraw()

    def add_snapshot(self, label: str, travels_mm, metrics) -> None:
        """Design-comparison overlay: keep a named copy of a sweep and draw
        it behind the live curves (max 4, oldest dropped)."""
        self._snapshots.append((label, np.array(travels_mm),
                                {k: np.array(v) for k, v in metrics.items()
                                 if k not in ("states", "wheel_center")}))
        self._snapshots = self._snapshots[-4:]
        self._redraw()

    def clear_snapshots(self) -> None:
        self._snapshots = []
        self._redraw()

    def set_marker(self, travel_mm: float) -> None:
        """Move the red current-travel line (cheap, called while sliding)."""
        self._marker_x = travel_mm
        x = self._unit.from_mm(travel_mm)
        for line in self._markers:
            line.set_xdata([x, x])
        self.draw_idle()

    # -- drawing ---------------------------------------------------------
    def _display(self, kind, y):
        if kind == "len":
            return self._unit.from_mm(np.asarray(y))
        if kind == "rate":
            return np.asarray(y) * self._unit.mm
        return y

    def _title(self, label, kind):
        u = self._unit.label
        return {"len": f"{label} ({u})", "rate": f"{label} (deg/{u})",
                "ang": f"{label} (deg)"}.get(kind, label)

    def _redraw(self) -> None:
        colors = DARK if self._dark else LIGHT
        self._fig.clf()
        self._fig.patch.set_facecolor(colors["fig"])
        self._markers = []
        self._axmap = {}
        self._hover_annot = None   # fig.clf() dropped the old artist
        panels = [p for p in PANELS if p[0] in self._visible]
        if not panels or self._sweep is None:
            self.draw_idle()
            return
        n = len(panels)
        rows = 1 if n <= 3 else 2
        cols = int(np.ceil(n / rows))
        axes = self._fig.subplots(rows, cols, squeeze=False).ravel()
        travels_mm, metrics, baseline = self._sweep
        travels = self._unit.from_mm(travels_mm)
        for ax, (key, label, kind) in zip(axes, panels):
            ax.set_facecolor(colors["ax"])
            for si, (slabel, st, sm) in enumerate(self._snapshots):
                if key in sm:
                    ax.plot(self._unit.from_mm(st),
                            self._display(kind, sm[key]), lw=1.0, ls="-.",
                            alpha=0.8, color=f"C{si + 2}", label=slabel)
            if baseline is not None:
                bt, bm = baseline
                ax.plot(self._unit.from_mm(bt), self._display(kind, bm[key]),
                        lw=1.2, ls="--", color="#999999", label="before")
            if key not in metrics:
                ax.set_visible(False)
                self._markers.append(ax.axvline(0.0))
                continue
            ydisp = self._display(kind, metrics[key])
            ax.plot(travels, ydisp, lw=1.5,
                    label=("after" if baseline is not None
                           else "current" if self._snapshots else None))
            self._axmap[ax] = (label, kind, np.asarray(travels),
                               np.asarray(ydisp))
            if baseline is not None or self._snapshots:
                leg = ax.legend(fontsize=6, loc="best")
                for t in leg.get_texts():
                    t.set_color(colors["text"])
            ax.axvline(0.0, color="gray", lw=0.6, ls=":")
            ax.set_title(self._title(label, kind), fontsize=9,
                         color=colors["text"])
            ax.tick_params(labelsize=8, colors=colors["text"])
            for spine in ax.spines.values():
                spine.set_color(colors["text"])
            ax.grid(True, alpha=colors["grid"])
            self._markers.append(
                ax.axvline(self._unit.from_mm(self._marker_x), color="red", lw=1.0))
        for ax in axes[len(panels):]:
            ax.set_visible(False)
        for ax in axes[max(0, len(panels) - cols):len(panels)]:
            ax.set_xlabel(f"wheel travel ({self._unit.label})", fontsize=8,
                          color=colors["text"])
        self.draw_idle()

    # -- hover readout ---------------------------------------------------
    def _clear_hover(self) -> None:
        if self._hover_annot is not None:
            try:
                self._hover_annot.remove()
            except (ValueError, NotImplementedError):
                pass
            self._hover_annot = None
            self.draw_idle()

    def _hover_text(self, label, kind, x_travel, y) -> str:
        u = self._unit
        if kind == "ang":
            val = f"{y:+.2f} deg"
        elif kind == "rate":
            val = f"{y:+.4f} deg/{u.label}"
        elif kind == "len":
            val = f"{y:+.2f} {u.label}"
        else:
            val = f"{y:+.3f}"
        return f"{label} @ {x_travel:+.1f} {u.label}: {val}"

    def _on_hover(self, event) -> None:
        info = self._axmap.get(event.inaxes)
        if info is None or event.xdata is None:
            self._clear_hover()
            return
        label, kind, xs, ys = info
        if xs.size == 0:
            return
        y = float(np.interp(event.xdata, xs, ys))
        colors = DARK if self._dark else LIGHT
        text = self._hover_text(label, kind, event.xdata, y)
        if self._hover_annot is None:
            self._hover_annot = event.inaxes.annotate(
                text, xy=(0.02, 0.98), xycoords="axes fraction",
                va="top", ha="left", fontsize=8, color=colors["text"],
                bbox=dict(boxstyle="round,pad=0.3", fc=colors["ax"],
                          ec=colors["text"], alpha=0.9))
        else:
            # reuse the artist, re-parenting to the hovered axes if needed
            if self._hover_annot.axes is not event.inaxes:
                try:
                    self._hover_annot.remove()
                except (ValueError, NotImplementedError):
                    pass
                self._hover_annot = event.inaxes.annotate(
                    text, xy=(0.02, 0.98), xycoords="axes fraction",
                    va="top", ha="left", fontsize=8, color=colors["text"],
                    bbox=dict(boxstyle="round,pad=0.3", fc=colors["ax"],
                              ec=colors["text"], alpha=0.9))
            else:
                self._hover_annot.set_text(text)
        self.draw_idle()
