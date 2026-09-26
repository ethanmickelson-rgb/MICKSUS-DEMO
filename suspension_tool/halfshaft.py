"""Halfshaft (CV axle) articulation checks.

A halfshaft runs from a chassis-fixed inner CV joint (gearbox/diff output
flange) to the outer CV at the wheel — modeled at the knuckle centre =
the wheel centre, which every suspension type's solved state carries, so
this works for double wishbone, trailing arm and multilink alike.

Per travel position we report the three numbers the physical joints
limit:

  cv_inner_deg  angle between the shaft and the gearbox OUTPUT AXIS
                (assumed pure lateral — a transverse gearbox, the Baja
                norm). The inner joint is usually the plunging one.
  cv_outer_deg  angle between the shaft and the wheel SPINDLE axis —
                what the outer (fixed) joint articulates.
  plunge_mm     shaft length change from static. Physically absorbed by
                the plunging inner joint's axial travel.

Limits are user inputs (read the joint datasheet); exceeding them is a
WARNING, not a hard stop — same philosophy as the sketch-planarity
guard: show it, let the designer (or a later optimizer pass) fix it.

Pure functions + a small config dataclass, GUI-free, unit-tested.
"""

from dataclasses import dataclass, field

import numpy as np

LATERAL = np.array([0.0, 1.0, 0.0])   # gearbox output axis (left corner)


# Inner (diff-side) joint types the user can pick. Each preselects a
# default working-angle limit and whether the joint can PLUNGE (absorb the
# shaft-length change through travel):
#   plunging_cv    tripod/AAR/VL ball-plunge — plunges INSIDE the joint,
#                  moderate angle (~22-33 deg working; Baja pushes the top)
#   rzeppa         fixed CV — high angle (~45-50 deg) but does NOT plunge;
#                  the shaft length must stay ~constant or a slip section is
#                  needed elsewhere
#   double_cardan  two U-joints back-to-back with an EXTENDING (slip-spline)
#                  shaft — high angle, plunge taken by the shaft, not the joint
JOINT_TYPES = {
    "plunging_cv":   {"label": "Plunging CV (tripod/AAR/VL)",
                      "angle": 25.0, "plunges": True},
    "rzeppa":        {"label": "Rzeppa (fixed CV)",
                      "angle": 47.0, "plunges": False},
    "double_cardan": {"label": "Double-cardan U-joint (slip shaft)",
                      "angle": 45.0, "plunges": True},
}
DEFAULT_JOINT = "plunging_cv"


@dataclass
class HalfshaftConfig:
    enabled: bool = False
    inner: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 150.0, 0.0]))  # mm, chassis
    # Inner-joint TYPE (see JOINT_TYPES): sets the default inner angle limit
    # and whether the joint plunges.
    inner_joint: str = DEFAULT_JOINT
    # The two joints carry SEPARATE angle limits (a shared cap is wrong for a
    # driven front, where the OUTER joint also swallows steering): INNER is
    # the diff-side joint (type above); OUTER is the fixed wheel joint
    # (Rzeppa/UF, ~45-50 deg).
    max_cv_inner_deg: float = 25.0   # inner joint max working angle
    max_cv_outer_deg: float = 45.0   # outer (fixed) joint max angle
    # Plunge budget, defined from the RIDE-HEIGHT shaft length as a total
    # axial stroke plus the % of it available for plunge-IN (compression);
    # the remainder is pull-OUT (extension). Asymmetric because ride height
    # rarely sits centred in the joint's stroke on a bump-biased car — same
    # idea as the shock's bump/droop split. Ignored when the joint is fixed
    # (rzeppa): a fixed joint cannot plunge, so any length change is flagged.
    plunge_total_mm: float = 50.0    # total axial stroke
    plunge_in_pct: float = 50.0      # % of the total available as plunge-in

    @property
    def plunges(self) -> bool:
        return JOINT_TYPES.get(self.inner_joint, JOINT_TYPES[DEFAULT_JOINT])[
            "plunges"]

    def plunge_allow(self) -> tuple:
        """(plunge_in_allow, pull_out_allow) in mm from ride height."""
        f = max(0.0, min(1.0, self.plunge_in_pct / 100.0))
        return self.plunge_total_mm * f, self.plunge_total_mm * (1.0 - f)

    def to_dict(self) -> dict:
        return {"enabled": bool(self.enabled),
                "inner": [float(v) for v in self.inner],
                "inner_joint": str(self.inner_joint),
                "max_cv_inner_deg": float(self.max_cv_inner_deg),
                "max_cv_outer_deg": float(self.max_cv_outer_deg),
                "plunge_total_mm": float(self.plunge_total_mm),
                "plunge_in_pct": float(self.plunge_in_pct)}

    @classmethod
    def from_dict(cls, d: dict | None) -> "HalfshaftConfig":
        if not d:
            return cls()
        # Back-compat: an older file has a single "max_cv_deg" (both joints
        # shared it) — migrate onto BOTH. Older files also had a symmetric
        # "max_plunge_mm" (± about ride): migrate to a total stroke of twice
        # that with a 50/50 split, so the load is behaviour-preserving.
        old = d.get("max_cv_deg")
        inner = d.get("max_cv_inner_deg", old if old is not None else 25.0)
        outer = d.get("max_cv_outer_deg", old if old is not None else 45.0)
        old_pl = d.get("max_plunge_mm")
        total = d.get("plunge_total_mm",
                      2.0 * old_pl if old_pl is not None else 50.0)
        pct = d.get("plunge_in_pct", 50.0)
        jt = d.get("inner_joint", DEFAULT_JOINT)
        if jt not in JOINT_TYPES:
            jt = DEFAULT_JOINT
        return cls(enabled=bool(d.get("enabled", False)),
                   inner=np.array(d.get("inner", [0.0, 150.0, 0.0]),
                                  dtype=float),
                   inner_joint=jt,
                   max_cv_inner_deg=float(inner),
                   max_cv_outer_deg=float(outer),
                   plunge_total_mm=float(total),
                   plunge_in_pct=float(pct))


def _angle_deg(u, v) -> float:
    u = np.asarray(u, float)
    v = np.asarray(v, float)
    c = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
    return float(np.degrees(np.arccos(np.clip(abs(c), -1.0, 1.0))))
    # abs(): an axis has no sign — 170 deg between vectors is a 10 deg
    # joint articulation.


def _on_axis_point(lbj, ubj, wheel_center, spindle=None) -> np.ndarray:
    """Where the outer CV physically sits: ON the TIRE'S CENTRE AXIS — the
    wheel spin axis through the wheel centre — at the point nearest the
    kingpin (LBJ->UBJ) axis.

    Putting it on the spin axis keeps the CV output line (CV -> tire
    centre) COINCIDENT with the tire centreline: a CV cocked at an angle
    to the wheel is buildable but deliberately out of scope, so the shaft
    must meet the wheel along its spin axis. "Nearest the kingpin" keeps
    the joint as close to the steering axis as the skew geometry allows
    (low steer-induced plunge). The steering link and this CV link then
    both swing rigidly with the knuckle about the kingpin, staying
    coincident with the tire centreline through steer and travel.

    Without a spindle direction (or with the lines near-parallel) fall
    back to the kingpin point nearest the wheel centre."""
    a = np.asarray(lbj, float)
    d = np.asarray(ubj, float) - a
    d = d / np.linalg.norm(d)
    w = np.asarray(wheel_center, float)
    if spindle is not None:
        s = np.asarray(spindle, float)
        s = s / np.linalg.norm(s)
        n = np.cross(d, s)
        n2 = float(np.dot(n, n))
        if n2 > 1e-12:
            # point on the SPIN axis (w + u*s) closest to the kingpin line
            u = float(np.dot(np.cross(w - a, d), n)) / n2
            return w + u * s
    return a + np.dot(w - a, d) * d


def outer_cv_of_state(state) -> np.ndarray:
    """The outer CV joint for a SOLVED pose: the pose's OWN hs_outer when
    the suspension type carries one (loaded halfshaft — the shaft is a
    structural link and its outer end IS the CV); otherwise on the kingpin
    axis at the tire's centre axis when the pose carries ball joints
    (double wishbone, or a multilink after its virtual kingpin is
    cached); the wheel centre itself for kingpin-less types (trailing
    arm)."""
    hs = getattr(state, "hs_outer", None)
    if hs is not None:
        return np.asarray(hs, float)
    lbj = getattr(state, "lbj", None)
    ubj = getattr(state, "ubj", None)
    if lbj is None or ubj is None:
        return np.asarray(state.wheel_center, float)
    return _on_axis_point(lbj, ubj, state.wheel_center,
                          getattr(state, "spindle", None))


def outer_cv_of_hp(hp) -> np.ndarray:
    """Static outer CV position from a hardpoint set (same rule, using
    the static camber/toe spindle). A type with its own hs_outer
    hardpoint (loaded halfshaft) IS the CV — return it directly."""
    hs = getattr(hp, "hs_outer", None)
    if hs is not None:
        return np.asarray(hs, float)
    lbj = getattr(hp, "lca_outer", None)
    ubj = getattr(hp, "uca_outer", None)
    if lbj is None or ubj is None:
        # C-hub: the steering axis is a PHYSICAL kingpin, so its two ends
        # play the role the ball joints do on a double wishbone. Without
        # this the hub offset reads a degenerate 0 on every C-hub corner.
        lbj = getattr(hp, "kingpin_lower", None)
        ubj = getattr(hp, "kingpin_upper", None)
    if lbj is None or ubj is None:
        return np.asarray(hp.wheel_center, float)
    from .geometry import static_spindle
    return _on_axis_point(lbj, ubj, hp.wheel_center, static_spindle(hp))


def halfshaft_state(cfg: HalfshaftConfig, state,
                    static_length: float) -> dict:
    """CV angles + plunge for ONE solved corner state."""
    shaft = outer_cv_of_state(state) - cfg.inner
    return {
        "cv_inner_deg": _angle_deg(shaft, LATERAL),
        "cv_outer_deg": _angle_deg(shaft, state.spindle),
        "plunge_mm": float(np.linalg.norm(shaft)) - static_length,
    }


def static_shaft_length(cfg: HalfshaftConfig, hp) -> float:
    return float(np.linalg.norm(outer_cv_of_hp(hp) - cfg.inner))


def default_inner_for(hp, bump: float, droop: float) -> np.ndarray:
    """Smart inner-CV placement from the current geometry: at the axle
    station, just off the centreline (gearbox flange), and HIGH enough
    that the shaft runs level at MID-travel — the placement that
    minimizes plunge over a bump-biased range."""
    wc = np.asarray(hp.wheel_center, float)
    return np.array([wc[0], 120.0, wc[2] + (bump - droop) / 2.0])


def inner_is_stale(cfg: HalfshaftConfig, hp) -> bool:
    """True when the inner joint is obviously not at this axle (e.g. the
    pre-offset class default while the design sits at x ~ +990 mm) — the
    'tangled mess' failure mode: a metre-long shaft slashing across the
    scene. Trigger: inner more than 500 mm fore/aft of the axle station
    or an absurd shaft length."""
    wc = np.asarray(hp.wheel_center, float)
    if abs(float(cfg.inner[0]) - wc[0]) > 500.0:
        return True
    return static_shaft_length(cfg, hp) > 2.5 * abs(wc[1])


def halfshaft_curves(cfg: HalfshaftConfig, hp, states) -> dict:
    """Arrays over a sweep's states: cv_inner_deg, cv_outer_deg,
    cv_max_deg (worst of the two, plotted as the absolute-worst joint
    angle) and plunge_mm. The two joints are checked against their OWN
    limits in halfshaft_warnings, not this shared maximum."""
    l0 = static_shaft_length(cfg, hp)
    rows = [halfshaft_state(cfg, st, l0) for st in states]
    out = {k: np.array([r[k] for r in rows]) for k in rows[0]}
    out["cv_max_deg"] = np.maximum(out["cv_inner_deg"],
                                   out["cv_outer_deg"])
    return out


def halfshaft_warnings(cfg: HalfshaftConfig, curves: dict) -> list[str]:
    """Human-readable limit violations over a sweep ([] = all clear). The
    inner (plunging) and outer (fixed) joints are checked against their
    OWN angle limits — a shared cap is wrong because the fixed outer joint
    tolerates far more articulation than the plunging inner (and, on a
    driven front, the outer also carries the steer angle)."""
    out = []
    worst_inner = float(np.nanmax(curves["cv_inner_deg"]))
    if worst_inner > cfg.max_cv_inner_deg:
        out.append(f"inner CV angle peaks at {worst_inner:.1f} deg "
                   f"(limit {cfg.max_cv_inner_deg:.0f} deg)")
    worst_outer = float(np.nanmax(curves["cv_outer_deg"]))
    if worst_outer > cfg.max_cv_outer_deg:
        out.append(f"outer CV angle peaks at {worst_outer:.1f} deg "
                   f"(limit {cfg.max_cv_outer_deg:.0f} deg)")
    # Plunge: plunge_mm = shaft length − static, so + is pull-OUT
    # (extension) and − is plunge-IN (compression). Check each direction
    # against its own allowance from the ride-height position.
    lo = float(np.nanmin(curves["plunge_mm"]))   # most compression (≤0)
    hi = float(np.nanmax(curves["plunge_mm"]))   # most extension (≥0)
    if not cfg.plunges:
        # A fixed joint (Rzeppa) cannot absorb any length change.
        swing = max(abs(lo), abs(hi))
        if swing > 1.0:
            out.append(
                f"{JOINT_TYPES[cfg.inner_joint]['label']} is a FIXED joint "
                f"but the shaft length changes {lo:+.1f}..{hi:+.1f} mm — "
                f"needs a plunging joint or a slip-spline shaft")
    else:
        plunge_in_allow, pull_out_allow = cfg.plunge_allow()
        if -lo > plunge_in_allow:
            out.append(f"plunge-in reaches {-lo:.1f} mm "
                       f"(limit {plunge_in_allow:.0f} mm)")
        if hi > pull_out_allow:
            out.append(f"pull-out reaches {hi:.1f} mm "
                       f"(limit {pull_out_allow:.0f} mm)")
    return out
