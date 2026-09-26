"""Save/load a whole suspension project as one JSON file (.MICK).

A project bundles everything needed to reopen a design exactly: the setup
variables, the current hardpoints, the chosen display unit, the travel
range, and a slot for optimization goals (filled in Phase 4). The on-disk
format is plain JSON regardless of the .MICK extension, so it stays human
-readable and diff-able. Lengths are stored in MILLIMETRES, always — the
display unit is recorded separately and only affects presentation.

These functions are GUI-free so they can be unit-tested directly.
"""

import dataclasses
import json

import numpy as np

from .chub_front import CHubFrontPoints
from .geometry import DoubleWishbonePoints
from .harm_rear import HArmRearPoints
from .loaded_halfshaft import LoadedHalfshaftPoints
from .multilink import MultilinkPoints
from .seed import SetupVariables
from .trailing_arm import TrailingArmPoints

FORMAT_VERSION = 3
FILE_EXTENSION = ".MICK"

# v3 flipped the MOTION-RATIO convention from wheel/shock (> 1)
# to the standard shock/wheel (< 1, e.g. 0.54), so the value drops straight
# into Kw = Ks*MR^2. Files written before v3 hold the old form and are
# inverted on load. A stamp is written into every new file so the check is
# explicit rather than a guess; for older files with no stamp we fall back
# to "a goal above 1 must be the old convention", which is safe because a
# shock/wheel ratio is physically < 1 for any inboard-mounted shock.
MR_CONVENTION = "shock_per_wheel"
_LEGACY_MR_CONVENTION = "wheel_per_shock"

# File-format keys for each suspension type's hardpoint class. Files
# without a "suspension_type" key (v1 / early v2) are double wishbones.
_TYPE_KEYS = {
    "double_wishbone": DoubleWishbonePoints,
    "trailing_arm": TrailingArmPoints,
    "multilink": MultilinkPoints,
    "chub_front": CHubFrontPoints,
    "loaded_halfshaft": LoadedHalfshaftPoints,
    "harm_rear": HArmRearPoints,
}


def type_key_of(hp) -> str:
    for key, cls in _TYPE_KEYS.items():
        if isinstance(hp, cls):
            return key
    raise ValueError(f"unknown hardpoint type {type(hp).__name__}")


@dataclasses.dataclass
class AxleDesign:
    """One axle's design: hardpoints, the setup that seeded it, travel."""

    hardpoints: DoubleWishbonePoints
    setup: SetupVariables | None
    droop_travel: float          # mm
    bump_travel: float           # mm
    halfshaft: dict | None = None  # HalfshaftConfig.to_dict() (v2.2+)
    locked: list | None = None   # pinned hardpoint attr names (v2 flexibility)
    shock: dict | None = None    # ShockSpec.to_dict(): fixed hardware
                                 # min/max shock lengths (v1.16)


@dataclasses.dataclass
class ProjectState:
    """Everything a saved project holds (format v2: front + rear axles
    plus vehicle-level parameters)."""

    front: AxleDesign | None
    rear: AxleDesign | None
    unit_key: str
    goals: dict | None = None    # optimization goals (Phase 4)
    vehicle: dict | None = None  # VehicleParams as a plain dict
    frame: dict | None = None    # chassis backdrop: path + FrameTransform
    dynamics: dict | None = None # DynamicsInputs as a dict (+ "_linked")
    coords: str | None = None    # display axis convention key (axes.py)
    steer_limit_mm: float | None = None  # rack travel limit the steer
                                 # slider is clamped to. Lives on the
                                 # window, so it was the one UI setting
                                 # that silently reset on every reload.
    candidates: list | None = None  # named whole-vehicle snapshots
    onshape: dict | None = None  # push manifest (tab/feature ids) so a
                                 # re-push updates the SAME Onshape tabs
    migration_notes: list = dataclasses.field(default_factory=list)
    # Human-readable record of anything changed while loading an older file
    # (currently: the v3 motion-ratio convention flip). The GUI surfaces
    # these so a silent reinterpretation of saved data never happens.


def _hp_to_dict(hp) -> dict:
    """Serialize ANY suspension type's hardpoints: the point list comes
    from the class's POINT_ATTRS, every other dataclass field is a plain
    scalar/bool that JSON round-trips natively."""
    points = type(hp).POINT_ATTRS
    out = {"suspension_type": type_key_of(hp)}
    for a in points:
        out[a] = [float(x) for x in getattr(hp, a)]
    for f in dataclasses.fields(hp):
        if f.name not in points:
            v = getattr(hp, f.name)
            out[f.name] = bool(v) if isinstance(v, bool) else float(v)
    return out


def _hp_from_dict(hpd: dict):
    cls = _TYPE_KEYS[hpd.get("suspension_type", "double_wishbone")]
    points = cls.POINT_ATTRS
    kw = {a: np.array(hpd[a], dtype=float) for a in points}
    for f in dataclasses.fields(cls):
        if f.name in points:
            continue
        if f.name in hpd:
            kw[f.name] = hpd[f.name]
        # absent scalar (older file): the dataclass default applies —
        # except tire_radius which has no default and always exists
    return cls(**kw)


def _axle_to_dict(axle: AxleDesign | None) -> dict | None:
    if axle is None:
        return None
    return {
        "hardpoints_mm": _hp_to_dict(axle.hardpoints),
        "setup": (dataclasses.asdict(axle.setup)
                  if axle.setup is not None else None),
        "travel_mm": {"droop": axle.droop_travel, "bump": axle.bump_travel},
        "halfshaft": axle.halfshaft,
        "locked": list(axle.locked) if axle.locked else None,
        "shock": axle.shock,
    }


def _setup_from_dict(data: dict | None, legacy_mr: bool = False,
                     notes: list | None = None,
                     label: str = "") -> SetupVariables | None:
    if data is None:
        return None
    # Keep only fields this build knows, so older/newer files still load.
    known = {f.name for f in dataclasses.fields(SetupVariables)}
    kept = {k: v for k, v in data.items() if k in known}
    goal = kept.get("motion_ratio_goal")
    if legacy_mr and isinstance(goal, (int, float)) and goal > 1.0:
        kept["motion_ratio_goal"] = 1.0 / float(goal)
        if notes is not None:
            notes.append(
                f"{label or 'axle'}: motion-ratio goal {goal:.3g} was saved "
                f"in the old wheel/shock convention — converted to "
                f"{kept['motion_ratio_goal']:.3g} (shock/wheel).")
    return SetupVariables(**kept)


def _axle_from_dict(data: dict | None, legacy_mr: bool = False,
                    notes: list | None = None,
                    label: str = "") -> AxleDesign | None:
    if data is None:
        return None
    travel = data.get("travel_mm", {})
    return AxleDesign(
        hardpoints=_hp_from_dict(data["hardpoints_mm"]),
        setup=_setup_from_dict(data.get("setup"), legacy_mr, notes, label),
        droop_travel=float(travel.get("droop", 75.0)),
        bump_travel=float(travel.get("bump", 75.0)),
        halfshaft=data.get("halfshaft"),
        locked=data.get("locked"),
        shock=data.get("shock"),
    )


def project_to_dict(state: ProjectState) -> dict:
    return {
        "format": "baja-suspension-kinematics",
        "version": FORMAT_VERSION,
        "unit": state.unit_key,
        "front": _axle_to_dict(state.front),
        "rear": _axle_to_dict(state.rear),
        "vehicle": state.vehicle,
        "goals": state.goals,
        "frame": state.frame,
        "dynamics": state.dynamics,
        "coords": state.coords,
        "steer_limit_mm": state.steer_limit_mm,
        "candidates": state.candidates,
        "onshape": state.onshape,
        "mr_convention": MR_CONVENTION,
    }


def _migrate_goal_targets(goals: dict | None, legacy_mr: bool,
                          notes: list) -> dict | None:
    """The optimizer's motion-ratio TARGET is stored in the same convention
    as the metric, so it needs the same inversion."""
    if not goals or not legacy_mr:
        return goals
    rows = goals.get("goals")
    if not isinstance(rows, list):
        return goals
    out = dict(goals)
    new_rows = []
    for row in rows:
        if (isinstance(row, dict) and row.get("key") == "motion_ratio"
                and isinstance(row.get("target"), (int, float))
                and row["target"] > 1.0):
            row = dict(row)
            old = float(row["target"])
            row["target"] = 1.0 / old
            notes.append(f"optimizer motion-ratio target {old:.3g} -> "
                         f"{row['target']:.3g} (shock/wheel).")
        new_rows.append(row)
    out["goals"] = new_rows
    return out


def dict_to_project(data: dict) -> ProjectState:
    version = data.get("version")
    # Anything without the v3 stamp predates the motion-ratio flip.
    legacy_mr = data.get("mr_convention") != MR_CONVENTION
    notes: list = []
    if version == 1:
        # v1 files held a single corner: load it as the FRONT axle.
        travel = data.get("travel_mm", {})
        front = AxleDesign(
            hardpoints=_hp_from_dict(data["hardpoints_mm"]),
            setup=_setup_from_dict(data.get("setup"), legacy_mr, notes,
                                   "front"),
            droop_travel=float(travel.get("droop", 75.0)),
            bump_travel=float(travel.get("bump", 75.0)),
        )
        return ProjectState(front=front, rear=None,
                            unit_key=data.get("unit", "mm"),
                            goals=_migrate_goal_targets(data.get("goals"),
                                                        legacy_mr, notes),
                            vehicle=None, migration_notes=notes)
    if version not in (2, FORMAT_VERSION):
        raise ValueError(
            f"unsupported project version {version!r} "
            f"(this build reads versions 1, 2 and {FORMAT_VERSION})")
    return ProjectState(
        front=_axle_from_dict(data.get("front"), legacy_mr, notes, "front"),
        rear=_axle_from_dict(data.get("rear"), legacy_mr, notes, "rear"),
        unit_key=data.get("unit", "mm"),
        goals=_migrate_goal_targets(data.get("goals"), legacy_mr, notes),
        vehicle=data.get("vehicle"),
        frame=data.get("frame"),
        dynamics=data.get("dynamics"),
        coords=data.get("coords"),
        steer_limit_mm=data.get("steer_limit_mm"),
        candidates=data.get("candidates"),
        onshape=data.get("onshape"),
        migration_notes=notes,
    )


def _conv_or_identity(conv):
    if conv is None:
        from .axes import CONVENTIONS
        return CONVENTIONS["tool"]
    return conv


def hardpoints_csv(hp: DoubleWishbonePoints, unit_mm: float = 1.0,
                   unit_label: str = "mm", conv=None) -> str:
    """Hardpoints as CSV text for CAD handoff (Onshape FeatureScript, Excel,
    SolidWorks equations...). One row per point: name,x,y,z in the chosen
    unit. Comment lines (#) carry the conventions and the scalars so the
    file is self-describing; parsers just skip them.

    conv: an axes.AxisConvention mapping tool coordinates to the CAD's
    display frame (None = tool frame, +X forward)."""
    conv = _conv_or_identity(conv)
    lines = [
        f"# Baja Suspension Tool hardpoints ({unit_label})",
        f"# coords: {conv.labels[0]}, {conv.labels[1]}, {conv.labels[2]}"
        f"  ({conv.name}); LEFT corner",
        f"# tire_radius,{hp.tire_radius / unit_mm:.4f}",
        f"# tire_width,{hp.tire_width / unit_mm:.4f}",
        f"# static_camber_deg,{hp.static_camber_deg:.4f}",
        f"# static_toe_deg,{hp.static_toe_deg:.4f}",
        f"# suspension_type,{type_key_of(hp)}",
        f"# shock_on_uca,{int(getattr(hp, 'shock_on_uca', False))}",
        "name,x,y,z",
    ]
    for attr in type(hp).POINT_ATTRS:
        p = conv.to_display(getattr(hp, attr))
        lines.append(f"{attr},{p[0] / unit_mm:.4f},{p[1] / unit_mm:.4f},"
                     f"{p[2] / unit_mm:.4f}")
    return "\n".join(lines) + "\n"


def export_hardpoints_csv(path: str, hp: DoubleWishbonePoints,
                          unit_mm: float = 1.0, unit_label: str = "mm",
                          conv=None) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(hardpoints_csv(hp, unit_mm, unit_label, conv=conv))


def vehicle_csv(axles: dict, wheelbase: float,
                unit_mm: float = 1.0, unit_label: str = "mm",
                conv=None) -> str:
    """Whole-vehicle CSV: every configured axle's points with a front_/rear_
    name prefix, all in ONE vehicle frame — origin at the FRONT axle, so
    rear points are shifted rearward by the wheelbase (in tool coordinates,
    then remapped through `conv` like everything else)."""
    conv = _conv_or_identity(conv)
    lines = [
        f"# Baja Suspension Tool vehicle hardpoints ({unit_label})",
        f"# coords: {conv.labels[0]}, {conv.labels[1]}, {conv.labels[2]}"
        f"  ({conv.name}); LEFT side",
        f"# wheelbase,{wheelbase / unit_mm:.4f}",
        "name,x,y,z",
    ]
    for axle_name, hp in axles.items():
        if hp is None:
            continue
        dx = 0.0 if axle_name == "front" else -wheelbase
        for attr in type(hp).POINT_ATTRS:
            p = getattr(hp, attr) + np.array([dx, 0.0, 0.0])
            p = conv.to_display(p)
            lines.append(
                f"{axle_name}_{attr},{p[0] / unit_mm:.4f},"
                f"{p[1] / unit_mm:.4f},{p[2] / unit_mm:.4f}")
    return "\n".join(lines) + "\n"


def save_project(path: str, state: ProjectState) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(project_to_dict(state), f, indent=2)


def load_project(path: str) -> ProjectState:
    with open(path, "r", encoding="utf-8") as f:
        return dict_to_project(json.load(f))


# ----------------------------------------------------------------------
# Report export (static design table + full curves, Excel-friendly CSV)
# ----------------------------------------------------------------------
_REPORT_SKIP = {"states", "wheel_center", "contact_patch_y_mm",
                "wheel_center_x_mm"}


def _report_value(key: str, v: float, unit_mm: float) -> str:
    if key.endswith("_mm"):
        return f"{v / unit_mm:.4f}"
    if key.endswith("_deg_per_mm"):
        return f"{v * unit_mm:.5f}"
    return f"{v:.4f}"


def _report_header(key: str, unit_label: str) -> str:
    if key.endswith("_mm"):
        return f"{key[:-3]}_{unit_label}"
    if key.endswith("_deg_per_mm"):
        return f"{key[:-11]}_deg_per_{unit_label}"
    return key


def report_csv(axle_sweeps: dict, vehicle_rows: dict,
               unit_mm: float = 1.0, unit_label: str = "mm") -> str:
    """One CSV with three sections: the static design table (all channels
    at zero travel, front vs rear), the vehicle-level numbers, and the
    full metric curves per axle. '#' lines separate sections so the file
    opens cleanly in Excel and stays parseable."""
    lines = [f"# Baja Suspension Tool report ({unit_label} / deg)"]
    axles = {k: s for k, s in axle_sweeps.items() if s is not None}
    keys = []
    for sweep in axles.values():
        for k in sweep:
            if k not in _REPORT_SKIP and k not in keys and k != "travel_mm":
                keys.append(k)

    lines.append("# --- static design table (values at zero travel) ---")
    lines.append("metric," + ",".join(axles))
    for key in keys:
        row = [_report_header(key, unit_label)]
        for sweep in axles.values():
            # keys are the UNION across axles (e.g. only one axle may
            # carry halfshaft channels) — blanks where an axle lacks one
            if key not in sweep:
                row.append("")
                continue
            k0 = int(np.argmin(np.abs(sweep["travel_mm"])))
            row.append(_report_value(key, float(sweep[key][k0]), unit_mm))
        lines.append(",".join(row))

    lines.append("# --- vehicle ---")
    for name, value in vehicle_rows.items():
        lines.append(f"{name},{value}")

    for axle_name, sweep in axles.items():
        lines.append(f"# --- {axle_name} sweep ---")
        lines.append(f"travel_{unit_label}," +
                     ",".join(_report_header(k, unit_label) for k in keys))
        for i in range(len(sweep["travel_mm"])):
            row = [f"{sweep['travel_mm'][i] / unit_mm:.4f}"]
            row += [(_report_value(k, float(sweep[k][i]), unit_mm)
                     if k in sweep else "") for k in keys]
            lines.append(",".join(row))
    return "\n".join(lines) + "\n"
