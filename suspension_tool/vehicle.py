"""Vehicle-level parameters and the roll-axis calculations they enable.

The corner solvers know nothing about the whole car; this module holds
the few scalars that tie the two axles together (wheelbase, CG location)
and the basic dynamics-adjacent numbers designers want next to the
kinematics:

  * roll axis: the line joining the front and rear roll centres. Vehicle
    coordinates put the FRONT axle at x = 0 and the rear axle at
    x = -wheelbase (behind it), so the axis runs from (0, 0, rc_front)
    to (-wheelbase, 0, rc_rear).
  * roll moment arm: vertical distance from the CG down to the roll axis
    at the CG's fore-aft station — the lever the sprung mass rolls with.
  * static front/rear weight split from the CG's position.

Pure functions, mm and degrees, GUI-free and unit-tested.
"""

from dataclasses import dataclass

import numpy as np

from .seed import IN


@dataclass
class VehicleParams:
    wheelbase: float = 60.0 * IN         # mm
    cg_height: float = 22.0 * IN         # mm above ground
    cg_behind_front: float = 0.55 * 60.0 * IN  # mm behind the front axle
    brake_front_frac: float = 0.6        # share of braking on the front axle


def roll_axis_points(rc_front_h: float, rc_rear_h: float,
                     wheelbase: float) -> tuple[np.ndarray, np.ndarray]:
    """Endpoints of the roll axis in vehicle coordinates (front axle at
    x = 0, rear at x = -wheelbase, both on the centreline y = 0)."""
    return (np.array([0.0, 0.0, rc_front_h]),
            np.array([-wheelbase, 0.0, rc_rear_h]))


def roll_axis_metrics(rc_front_h: float, rc_rear_h: float,
                      params: VehicleParams) -> dict:
    """The numbers the roll axis gives you once the CG is known."""
    wb = params.wheelbase
    frac = np.clip(params.cg_behind_front / wb, 0.0, 1.0)
    # linear interpolation of axis height at the CG station
    axis_h_at_cg = rc_front_h + (rc_rear_h - rc_front_h) * frac
    return {
        # positive angle = axis climbs toward the rear
        "roll_axis_angle_deg": float(np.degrees(
            np.arctan2(rc_rear_h - rc_front_h, wb))),
        "roll_axis_height_at_cg_mm": float(axis_h_at_cg),
        # the lever arm the sprung mass rolls about — THE handling number
        "roll_moment_arm_mm": float(params.cg_height - axis_h_at_cg),
        "front_weight_frac": float(1.0 - frac),
        "rear_weight_frac": float(frac),
    }


# ----------------------------------------------------------------------
# Anti-dive / anti-lift / anti-squat (pure side-view geometry)
# ----------------------------------------------------------------------
# The percentages compare the suspension's geometric support angle against
# the angle that would react ALL of the longitudinal load transfer:
#
#   %anti-dive (front, braking) = 100 * fb      * tan(phi_f) * L / h
#   %anti-lift (rear,  braking) = 100 * (1-fb)  * tan(phi_r) * L / h
#   %anti-squat (rear,  accel)  = 100 *           tan(theta) * L / h
#
# where the support line runs from the CONTACT PATCH to the side-view IC
# for outboard brakes, and from the WHEEL CENTRE for inboard drive (the
# Baja rear: gearbox + CV axles), fb = front brake fraction, L wheelbase,
# h CG height. No forces are resolved — these are ratios of angles, which
# is why a pure-kinematics tool can report them.


def _support_direction(solver, state, use_wheel_center: bool):
    """(dx, dz) from the reference point toward the side-view IC — exact
    when the IC is finite. When the IC is at infinity (parallel side-view
    arm traces — which is the tool's DEFAULT seed state and where real
    actually sit), use the arm-plane LINE DIRECTION, matching what
    metrics.roll_center_height_mm does for its front-view infinity case.
    The old fallback finite-differenced the wheel/patch path and took its
    perpendicular; for a spatial (kicked-up) linkage that direction
    differs from the arm-plane line by ~0.7 deg, so anti-squat/dive JUMPED
    ~2-4 percentage points the instant the arms went from exactly parallel
    to slightly skewed. The arm-plane direction is the limit the finite
    branch converges to, so the metric is now continuous through the
    designed state. None if indeterminate."""
    from .metrics import _side_view_arm_line, side_view_ic
    ref = state.wheel_center if use_wheel_center else state.contact_patch
    ic = side_view_ic(solver, state)
    if ic is not None:
        return ic[0] - ref[0], ic[1] - ref[2]
    # IC at infinity: for a double wishbone, use the (upper) arm plane's
    # side-view slice direction — the same line the finite side_view_ic
    # intersects, so the two branches agree in the limit.
    hp = getattr(solver, "hp", None)
    if hp is not None and hasattr(hp, "uca_inner_front"):
        a1, b1, _ = _side_view_arm_line(hp.uca_inner_front,
                                        hp.uca_inner_rear, state.ubj)
        if np.hypot(a1, b1) > 1e-12:
            return -b1, a1        # line dir = perpendicular to the normal
    # non-DW (multilink / trailing arm) with no arm planes: the numeric
    # path-perpendicular is the best available construction.
    steer = getattr(state, "steer", 0.0)
    try:
        lo = solver.solve(state.travel - 1.0, steer)
        hi = solver.solve(state.travel + 1.0, steer)
    except (ValueError, TypeError):
        return None
    attr = "wheel_center" if use_wheel_center else "contact_patch"
    v = getattr(hi, attr) - getattr(lo, attr)
    if np.hypot(v[0], v[2]) < 1e-12:
        return None
    return -v[2], v[0]        # perpendicular of the (v_x, v_z) velocity


def _support_slope(solver, state, use_wheel_center: bool,
                   toward_rear: bool) -> float:
    """tan of the support angle, measured with the line pointing toward
    the rear (front suspension) or toward the front (rear suspension);
    positive slope = anti behaviour."""
    d = _support_direction(solver, state, use_wheel_center)
    if d is None:
        return float("nan")
    dx, dz = d
    want = -1.0 if toward_rear else 1.0
    if dx * want < 0.0:       # the line points both ways; face it correctly
        dx, dz = -dx, -dz
    if abs(dx) < 1e-9:
        return float("nan")
    return dz / abs(dx)


def anti_geometry(front, rear, params: VehicleParams) -> dict:
    """front / rear: (solver, state) pairs at the travel of interest.
    Outboard brakes both ends; inboard (chassis-mounted) drive at the rear
    — the standard Baja layout."""
    L, h, fb = params.wheelbase, params.cg_height, params.brake_front_frac
    out = {}
    f_solver, f_state = front
    r_solver, r_state = rear
    tan_f = _support_slope(f_solver, f_state, False, toward_rear=True)
    tan_r = _support_slope(r_solver, r_state, False, toward_rear=False)
    tan_s = _support_slope(r_solver, r_state, True, toward_rear=False)
    out["anti_dive_front_pct"] = 100.0 * fb * tan_f * L / h
    out["anti_lift_rear_pct"] = 100.0 * (1.0 - fb) * tan_r * L / h
    out["anti_squat_rear_pct"] = 100.0 * tan_s * L / h
    return out


def anti_percent_curves(solver, states, params: VehicleParams,
                        axle_name: str):
    """Per-travel anti percentages for ONE axle, for plotting:

      anti_squat_pct — the wheel-centre support line (drive reaction for
                       an inboard-drive axle; the number Baja rears tune)
      anti_dive_pct  — the contact-patch support line scaled by this
                       axle's brake share (fb front, 1-fb rear: the
                       braking anti-dive / anti-lift percentage)

    Returns (anti_squat, anti_dive) numpy arrays matching `states`."""
    L, h, fb = params.wheelbase, params.cg_height, params.brake_front_frac
    frac = fb if axle_name == "front" else (1.0 - fb)
    toward_rear = axle_name == "front"
    squat, dive = [], []
    for st in states:
        squat.append(100.0 * _support_slope(solver, st, True,
                                            toward_rear) * L / h)
        dive.append(100.0 * frac * _support_slope(solver, st, False,
                                                  toward_rear) * L / h)
    return np.array(squat), np.array(dive)
