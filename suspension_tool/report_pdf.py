"""Design-notes PDF: the readouts and curves that travel INTO the
Onshape document alongside the geometry, so a teammate opening the CAD
sees what the kinematic model achieved without launching MICKSUS.

Rendered with matplotlib's PdfPages (already a dependency) into an
in-memory bytes buffer for blob upload. GUI-free and testable offline.
"""

import io

import numpy as np


def _fmt(v, unit=""):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return "--"
    return f"{v:.2f}{unit}"


def design_notes_pdf(axles: dict, wheelbase_mm: float,
                     version: str = "") -> bytes:
    """Build the PDF. `axles` maps 'front'/'rear' -> a dict with:
        hp        : the hardpoints (for the coordinate table)
        static    : corner_metrics dict at ride height (may be None)
        sweep     : the metric-curve dict (may be None)
        goals     : optional {label: (target, achieved)} rows
        travel    : optional (droop, bump) mm
        halfshaft : optional {'cv_max': deg, 'plunge': mm}
    Returns PDF file bytes."""
    from matplotlib.backends.backend_pdf import PdfPages
    from matplotlib.figure import Figure

    buf = io.BytesIO()
    with PdfPages(buf) as pdf:
        _cover_page(pdf, Figure, axles, wheelbase_mm, version)
        for name, data in axles.items():
            if data is None or data.get("hp") is None:
                continue
            _axle_page(pdf, Figure, name, data)
    return buf.getvalue()


def _cover_page(pdf, Figure, axles, wheelbase_mm, version):
    fig = Figure(figsize=(8.5, 11))
    fig.suptitle("MICKSUS — Suspension Design Notes",
                 fontsize=18, fontweight="bold", y=0.96)
    ax = fig.add_axes([0.08, 0.08, 0.84, 0.82])
    ax.axis("off")
    lines = [f"Baja SAE suspension kinematic model" ,
             f"MICKSUS v{version}" if version else "",
             f"Wheelbase: {_fmt(wheelbase_mm, ' mm')}",
             ""]
    for name in ("front", "rear"):
        d = axles.get(name)
        if d is None or d.get("hp") is None:
            lines.append(f"{name.title()} axle: (not defined)")
            continue
        st = d.get("static") or {}
        tv = d.get("travel")
        lines.append(f"{name.title()} axle")
        lines.append(f"    camber {_fmt(st.get('camber_deg'), ' deg')}   "
                     f"caster {_fmt(st.get('caster_deg'), ' deg')}   "
                     f"KPI {_fmt(st.get('kpi_deg'), ' deg')}")
        lines.append(f"    scrub {_fmt(st.get('scrub_radius_mm'), ' mm')}   "
                     f"RC height "
                     f"{_fmt(st.get('roll_center_height_mm'), ' mm')}")
        if tv:
            lines.append(f"    travel  +{_fmt(tv[1])} / -{_fmt(tv[0])} mm")
        hs = d.get("halfshaft")
        if hs:
            lines.append(f"    halfshaft: CV max {_fmt(hs.get('cv_max'), ' deg')}"
                         f", plunge {_fmt(hs.get('plunge'), ' mm')}")
        lines.append("")
    ax.text(0.0, 1.0, "\n".join(l for l in lines if l is not None),
            va="top", ha="left", fontsize=11, family="monospace",
            transform=ax.transAxes)
    ax.text(0.0, 0.02,
            "Coordinates are in the document's chassis frame. Generated "
            "from the kinematic model; do not treat it as as-built.",
            va="bottom", ha="left", fontsize=8, style="italic",
            transform=ax.transAxes)
    pdf.savefig(fig)


def _axle_page(pdf, Figure, name, data):
    fig = Figure(figsize=(8.5, 11))
    fig.suptitle(f"{name.title()} axle", fontsize=15, fontweight="bold")
    hp = data["hp"]
    # hardpoint coordinate table
    ax_t = fig.add_axes([0.06, 0.60, 0.88, 0.30])
    ax_t.axis("off")
    rows = []
    for attr in type(hp).POINT_ATTRS:
        p = getattr(hp, attr)
        rows.append([attr, f"{p[0]:.1f}", f"{p[1]:.1f}", f"{p[2]:.1f}"])
    table = ax_t.table(cellText=rows,
                       colLabels=["hardpoint", "X (mm)", "Y (mm)", "Z (mm)"],
                       loc="upper center", cellLoc="left")
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.scale(1.0, 1.15)

    # goals vs achieved, if provided
    goals = data.get("goals")
    if goals:
        ax_g = fig.add_axes([0.06, 0.44, 0.88, 0.12])
        ax_g.axis("off")
        grows = [[k, _fmt(t), _fmt(a)] for k, (t, a) in goals.items()]
        gt = ax_g.table(cellText=grows,
                        colLabels=["goal", "target", "achieved"],
                        loc="upper center", cellLoc="left")
        gt.auto_set_font_size(False)
        gt.set_fontsize(8)

    # a couple of key curves
    sweep = data.get("sweep")
    if sweep is not None and "travel_mm" in sweep:
        t = sweep["travel_mm"]
        for i, (key, title) in enumerate(
                (("camber_deg", "Camber (deg)"),
                 ("toe_deg", "Toe (deg)"))):
            if key not in sweep:
                continue
            ax = fig.add_axes([0.10 + 0.46 * i, 0.10, 0.36, 0.26])
            ax.plot(t, sweep[key], lw=1.4)
            ax.axvline(0, color="r", lw=0.8, alpha=0.5)
            ax.set_title(title, fontsize=9)
            ax.set_xlabel("wheel travel (mm)", fontsize=8)
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=7)
    pdf.savefig(fig)
