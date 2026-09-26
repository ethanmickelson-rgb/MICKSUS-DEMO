"""Phase 4: synthesis — refine hardpoints toward kinematic goals.

The user picks GOALS (each with a target and a weight), picks which
hardpoint groups are FREE to move, and this module runs a bounded
least-squares refinement (scipy.optimize.least_squares) starting from the
current geometry — normally a Phase 2 seed, per the project brief: refine
a limited free set, never all hardpoints at once.

How geometry is parameterised (this encodes the team's hard constraints):

  * The inboard bushing PAIRS ("uca_inner", "lca_inner") move only via
    their axis MIDPOINT (y, z). The front/rear half-spread vector is kept
    verbatim, so the axis stays exactly normal to the 2D sketch plane and
    both axes stay parallel no matter what the optimizer does.
  * Ball joints and tie-rod ends move as plain points; the shock points
    move in the sketch plane (y, z).
  * Every free coordinate is bounded to a box around its start value,
    clipped to the packaging envelope (inside the wheel, above ground).

Candidates that cannot articulate the requested travel range return a
large flat residual, so the optimizer steps away from them ("reject
geometry that fails to solve"). If the refinement somehow ends worse than
it started, the original geometry is returned unchanged.

Residual scaling: goals mix units (deg, mm, ratios), so each goal kind has
a fixed scale chosen so "1 unit of residual" is a comparable amount of
badness (1 deg ~ 10 mm ~ 0.05 of motion ratio). User weights multiply on
top of that.
"""

import dataclasses
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from .chub_front import CHubFrontPoints
from .geometry import DoubleWishbonePoints
from .harm_rear import HArmRearPoints
from .loaded_halfshaft import LoadedHalfshaftPoints
from .metrics import curves_from_states
from .multilink import MultilinkPoints
from .solver import DoubleWishboneSolver
from .trailing_arm import TrailingArmPoints

# Parameter-space scale (mm per optimizer unit): keeps all parameters O(1)
# so scipy's default finite-difference steps behave.
PARAM_SCALE = 25.0
# Residual value used for candidates whose linkage cannot articulate.
REJECT_RESIDUAL = 25.0

# ----------------------------------------------------------------------
# Goals
# ----------------------------------------------------------------------
# kind: how the residual is built; unit_kind: how the GUI converts display.
#   "curve_slope"  target = slope of (metric - metric@static) vs travel
#   "curve_zero"   drive (metric - metric@static) to zero over travel
#   "static"       metric at zero travel hits target
GOAL_SPECS = {
    "camber_gain":   {"kind": "curve_slope", "metric": "camber_deg", "scale": 1.0,
                      "label": "Camber gain", "unit_kind": "rate"},
    "bump_steer":    {"kind": "curve_zero",  "metric": "toe_deg",    "scale": 2.0,
                      "label": "Bump steer -> 0", "unit_kind": None},
    # Passive rear steer: drive toe to CHANGE linearly through travel at
    # a chosen rate instead of staying flat — e.g. a rear with a fixed
    # toe link that toes OUT in bump and IN in droop (target < 0 in the
    # tool's toe-in-positive convention).
    "toe_slope":     {"kind": "curve_slope", "metric": "toe_deg", "scale": 2.0,
                      "label": "Toe vs travel (passive steer)",
                      "unit_kind": "rate"},
    "rc_height":     {"kind": "static", "metric": "roll_center_height_mm",
                      "scale": 0.1, "label": "Roll centre height", "unit_kind": "len"},
    "rc_migration":  {"kind": "curve_zero", "metric": "roll_center_height_mm",
                      "scale": 0.05, "label": "RC migration -> 0", "unit_kind": None},
    # MR is now shock/wheel (below 1, was wheel/shock above 1), so an equivalent
    # design error shows up as a ~3.42x smaller residual — the scale rises by
    # the same factor to keep this goal weighted as it was before v1.28.
    "motion_ratio":  {"kind": "static", "metric": "motion_ratio", "scale": 68.0,
                      "label": "Motion ratio", "unit_kind": "ratio"},
    "caster":        {"kind": "static", "metric": "caster_deg", "scale": 1.0,
                      "label": "Caster", "unit_kind": "ang"},
    "scrub_radius":  {"kind": "static", "metric": "scrub_radius_mm", "scale": 0.1,
                      "label": "Scrub radius", "unit_kind": "len"},
    "kpi":           {"kind": "static", "metric": "kpi_deg", "scale": 1.0,
                      "label": "KPI", "unit_kind": "ang"},
    "caster_trail":  {"kind": "static", "metric": "caster_trail_mm", "scale": 0.1,
                      "label": "Caster trail", "unit_kind": "len"},
}


@dataclass
class Goal:
    key: str            # one of GOAL_SPECS
    target: float = 0.0  # ignored for "curve_zero" goals
    weight: float = 1.0


# ----------------------------------------------------------------------
# Free hardpoint groups and their parameterisation
# ----------------------------------------------------------------------
# name -> (kind, attrs). kind "axis_mid": (y, z) of the bushing-axis
# midpoint; "point": (x, y, z); "point_yz": (y, z), x fixed (sketch plane).
FREE_GROUPS = {
    "uca_inner":    ("axis_mid", ("uca_inner_front", "uca_inner_rear")),
    "lca_inner":    ("axis_mid", ("lca_inner_front", "lca_inner_rear")),
    "uca_outer":    ("point", ("uca_outer",)),
    "lca_outer":    ("point", ("lca_outer",)),
    "tierod_inner": ("point", ("tierod_inner",)),
    "tierod_outer": ("point", ("tierod_outer",)),
    "shock_inner":  ("point_yz", ("shock_inner",)),
    "shock_outer":  ("point_yz", ("shock_outer",)),
}

TRAILING_ARM_GROUPS = {
    "pivot_inner":  ("point", ("pivot_inner",)),
    "pivot_outer":  ("point", ("pivot_outer",)),
    "shock_inner":  ("point_yz", ("shock_inner",)),
    "shock_outer":  ("point", ("shock_outer",)),
}

MULTILINK_GROUPS = {
    **{f"link{k}_{end}": ("point", (f"link{k}_{end}",))
       for k in range(1, 6) for end in ("inner", "outer")},
    "shock_inner":  ("point_yz", ("shock_inner",)),
    "shock_outer":  ("point", ("shock_outer",)),
}

CHUB_GROUPS = {
    "arm_inner":    ("axis_mid", ("arm_inner_front", "arm_inner_rear")),
    "camber_inner": ("point", ("camber_inner",)),
    "camber_outer": ("point", ("camber_outer",)),
    "tierod_inner": ("point", ("tierod_inner",)),
    "tierod_outer": ("point", ("tierod_outer",)),
    "shock_inner":  ("point_yz", ("shock_inner",)),
    "shock_outer":  ("point", ("shock_outer",)),
}

LOADED_HS_GROUPS = {
    "arm_inner":    ("axis_mid", ("arm_inner_front", "arm_inner_rear")),
    "hs_inner":     ("point", ("hs_inner",)),
    "shock_inner":  ("point_yz", ("shock_inner",)),
    "shock_outer":  ("point", ("shock_outer",)),
}

HARM_GROUPS = {
    "arm_inner":    ("axis_mid", ("arm_inner_front", "arm_inner_rear")),
    "camber_inner": ("point", ("camber_inner",)),
    "camber_outer": ("point", ("camber_outer",)),
    "shock_inner":  ("point_yz", ("shock_inner",)),
    "shock_outer":  ("point", ("shock_outer",)),
}


def free_groups_for(hp) -> dict:
    """The free-hardpoint groups available for this suspension type."""
    if isinstance(hp, TrailingArmPoints):
        return TRAILING_ARM_GROUPS
    if isinstance(hp, MultilinkPoints):
        return MULTILINK_GROUPS
    if isinstance(hp, CHubFrontPoints):
        return CHUB_GROUPS
    if isinstance(hp, LoadedHalfshaftPoints):
        return LOADED_HS_GROUPS
    if isinstance(hp, HArmRearPoints):
        return HARM_GROUPS
    return FREE_GROUPS


@dataclass
class OptimizeResult:
    hp_before: DoubleWishbonePoints
    hp_after: DoubleWishbonePoints
    cost_before: float
    cost_after: float
    success: bool
    message: str
    n_evals: int


def _pack(hp: DoubleWishbonePoints, free: list[str]) -> np.ndarray:
    """Current values (mm) of every free parameter, in a fixed order."""
    vals = []
    groups = free_groups_for(hp)
    for name in free:
        kind, attrs = groups[name]
        if kind == "axis_mid":
            mid = (getattr(hp, attrs[0]) + getattr(hp, attrs[1])) / 2.0
            vals += [mid[1], mid[2]]
        elif kind == "point":
            vals += list(getattr(hp, attrs[0]))
        else:  # point_yz
            p = getattr(hp, attrs[0])
            vals += [p[1], p[2]]
    return np.array(vals, dtype=float)


def _apply(hp: DoubleWishbonePoints, free: list[str], vals) -> DoubleWishbonePoints:
    """Rebuild a hardpoint set with the free parameters set to `vals` (mm)."""
    kw = {}
    i = 0
    groups = free_groups_for(hp)
    for name in free:
        kind, attrs = groups[name]
        if kind == "axis_mid":
            f0, r0 = getattr(hp, attrs[0]), getattr(hp, attrs[1])
            half = (f0 - r0) / 2.0          # kept verbatim: normality + spread
            mid = (f0 + r0) / 2.0
            new_mid = np.array([mid[0], vals[i], vals[i + 1]])
            kw[attrs[0]] = new_mid + half
            kw[attrs[1]] = new_mid - half
            i += 2
        elif kind == "point":
            kw[attrs[0]] = np.array(vals[i:i + 3], dtype=float)
            i += 3
        else:  # point_yz
            p = getattr(hp, attrs[0])
            kw[attrs[0]] = np.array([p[0], vals[i], vals[i + 1]])
            i += 2
    return dataclasses.replace(hp, **kw)


def _bounds(hp: DoubleWishbonePoints, free: list[str], box: float):
    """Per-parameter (lo, hi) in mm: a ±box around the start value, clipped
    to the packaging envelope (above ground, inboard of the wheel)."""
    # Packaging clearances scale with the car: the fractions of tire radius
    # below reproduce the old fixed 10 mm / 5 mm at Baja size and stay
    # proportionate at 1/10 RC scale, where 10 mm is a tenth of the track.
    clear_y = 0.034 * hp.tire_radius
    clear_z = 0.017 * hp.tire_radius
    y_max = hp.wheel_center[1] - clear_y   # stay inboard of the wheel face
    # Ground = the tire contact plane, NOT z = 0 (the Onshape-aligned
    # coordinate offset puts the ground at negative z).
    z_min = float(hp.wheel_center[2]) - hp.tire_radius + clear_z
    lo, hi = [], []
    groups = free_groups_for(hp)
    for name in free:
        kind, attrs = groups[name]
        if kind == "axis_mid" or kind == "point_yz":
            p = ((getattr(hp, attrs[0]) + getattr(hp, attrs[1])) / 2.0
                 if kind == "axis_mid" else getattr(hp, attrs[0]))
            per_axis = [("y", p[1]), ("z", p[2])]
        else:
            p = getattr(hp, attrs[0])
            per_axis = [("x", p[0]), ("y", p[1]), ("z", p[2])]
        for axis, v in per_axis:
            a, b = v - box, v + box
            if axis == "y":
                a, b = max(a, clear_z), min(b, y_max)
            elif axis == "z":
                a = max(a, z_min)
            if b - a < 1e-6:            # fully clipped: pin the parameter
                a, b = v - 1e-6, v + 1e-6
            lo.append(a)
            hi.append(b)
    return np.array(lo), np.array(hi)


def _residuals(curves: dict, travels: np.ndarray, goals: list[Goal]) -> np.ndarray:
    """Weighted, scaled residual vector for one solved candidate."""
    res = []
    n = len(travels)
    static_idx = int(np.argmin(np.abs(travels)))
    for g in goals:
        spec = GOAL_SPECS[g.key]
        y = curves[spec["metric"]]
        w = g.weight * spec["scale"]
        if spec["kind"] == "static":
            res.append(w * (y[static_idx] - g.target))
        elif spec["kind"] == "curve_slope":
            # deviation of the curve from the target straight line through
            # the static point, normalised so goal size ~ curve size
            dev = (y - y[static_idx]) - g.target * (travels - travels[static_idx])
            res.extend(w * dev / np.sqrt(n))
        else:  # curve_zero
            dev = y - y[static_idx]
            res.extend(w * dev / np.sqrt(n))
    return np.array(res, dtype=float)


def _n_residuals(travels, goals) -> int:
    n = len(travels)
    return sum(1 if GOAL_SPECS[g.key]["kind"] == "static" else n for g in goals)


def optimize(hp: DoubleWishbonePoints, goals: list[Goal], free: list[str],
             travels, box: float = 75.0, max_nfev: int = 300,
             sketch_yaw_deg: float = 0.0) -> OptimizeResult:
    """Refine `hp` toward the goals by moving only the `free` groups.

    travels: the wheel-travel positions the curve goals are evaluated at
    (the candidate must articulate all of them). box: search range (mm)
    around each free coordinate. sketch_yaw_deg: the axle's DESIGN sketch
    twist — the entry re-square targets this angle.
    """
    if not goals:
        raise ValueError("no goals selected")
    if not free:
        raise ValueError("no free hardpoints selected")
    groups = free_groups_for(hp)
    for name in free:
        if name not in groups:
            raise ValueError(
                f"unknown free group {name!r} for {type(hp).__name__}")

    # Manufacturability repair (double wishbone): if hand edits have let
    # the two bushing axes drift out of parallel, no single 2D sketch can
    # host both arms — re-square them onto their mean direction before
    # refining. The axis_mid parameterisation below then PRESERVES that
    # parallelism for every candidate, so the result stays buildable.
    # Automatic on purpose (not a user option, per the team's workflow).
    if isinstance(hp, DoubleWishbonePoints):
        from .geometry import sketch_planarity, square_bushing_axes
        p = sketch_planarity(hp)
        if (p["axis_misalign_deg"] > 1e-9
                or abs(p["axis_yaw_deg"] - sketch_yaw_deg) > 1e-9):
            hp = square_bushing_axes(hp, yaw_deg=sketch_yaw_deg)

    travels = np.asarray(travels, dtype=float)
    x0_mm = _pack(hp, free)
    lo_mm, hi_mm = _bounds(hp, free, box)
    x0_mm = np.clip(x0_mm, lo_mm, hi_mm)
    n_res = _n_residuals(travels, goals)

    from .suspension_types import solver_for
    solver_cls = solver_for(hp)

    # Pre-flight: a goal whose metric is undefined for this type (e.g.
    # caster on a trailing arm, which has no kingpin) must error clearly
    # instead of feeding NaNs to the optimizer.
    probe = solver_cls(hp)
    curves0 = curves_from_states(probe, probe.walk_travels(travels), travels)
    for g in goals:
        metric = GOAL_SPECS[g.key]["metric"]
        if not np.any(np.isfinite(np.atleast_1d(curves0[metric]))):
            raise ValueError(
                f"goal {g.key!r} is undefined for {type(hp).__name__} "
                "(no kingpin) — deselect it")

    def residuals_mm(vals_mm) -> np.ndarray:
        cand = _apply(hp, free, vals_mm)
        try:
            solver = solver_cls(cand)
            states = solver.walk_travels(travels)
        except ValueError:
            return np.full(n_res, REJECT_RESIDUAL)
        res = _residuals(curves_from_states(solver, states, travels),
                         travels, goals)
        return np.nan_to_num(res, nan=REJECT_RESIDUAL)

    # Work in normalised parameter space so scipy's finite-difference steps
    # are sensible for every coordinate (values range from ~0 to ~800 mm).
    def residuals_norm(x):
        return residuals_mm(x0_mm + PARAM_SCALE * x)

    result = least_squares(
        residuals_norm,
        np.zeros_like(x0_mm),
        bounds=((lo_mm - x0_mm) / PARAM_SCALE, (hi_mm - x0_mm) / PARAM_SCALE),
        method="trf",
        max_nfev=max_nfev,
    )

    x_after = x0_mm + PARAM_SCALE * result.x
    hp_after = _apply(hp, free, x_after)
    cost_before = 0.5 * float(np.sum(residuals_mm(x0_mm) ** 2))
    cost_after = 0.5 * float(np.sum(residuals_mm(x_after) ** 2))

    # Never hand back something worse (or broken) than the start point.
    try:
        solver_cls(hp_after).walk_travels(travels)
        valid = True
    except ValueError:
        valid = False
    if not valid or cost_after >= cost_before:
        return OptimizeResult(hp, hp, cost_before, cost_before, False,
                              "no improvement found — geometry unchanged",
                              result.nfev)
    return OptimizeResult(hp, hp_after, cost_before, cost_after, True,
                          f"cost {cost_before:.4g} -> {cost_after:.4g}",
                          result.nfev)
