"""Vehicle motion sweeps beyond pure heave (the OptimumKinematics/Adams
"motion" set, kinematic subset): ROLL (opposite wheel travel), PITCH
(front/rear coupled travel about the CG station), and STEERING (rack
sweep with Ackermann outputs).

All of it reuses the corner solvers — a sweep is just a schedule of
(travel, steer) inputs plus the frame bookkeeping to express results
relative to the GROUND instead of the chassis:

  * Roll: left wheel at +t, right wheel at -t. The right corner is the
    mirror of the left, so its states come from solving the LEFT model at
    -t and mirroring. The chassis is rolled relative to the ground by
    phi = atan(2t / track); ground-frame camber/toe are measured after
    rotating the spindles by that roll. Positive roll = body leans onto
    the LEFT wheels (a right-hand turn).
  * Pitch: nose-down angle theta rotates the chassis about the CG
    station, so the front wheels see bump  d_f*tan(theta) and the rears
    see droop (wheelbase - d_f)*tan(theta). Ground caster subtracts the
    pitch from the chassis-frame kingpin.
  * Steer: rack from -limit to +limit at a fixed travel; wheel steer
    angles for both sides (right side = mirrored left at -s), percent
    Ackermann against the ideal cot(do) - cot(di) = -track/wheelbase
    relation, and a low-speed outside-front turn-diameter estimate.

Pure functions returning dicts of numpy arrays; GUI dialogs plot them.
"""

import numpy as np

from .metrics import front_view_ic

MIRROR = np.array([1.0, -1.0, 1.0])


def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _camber_of(spindle, side: int) -> float:
    """Front-view camber (deg) of a GLOBAL-frame spindle. side=+1 left
    (outboard = +y), side=-1 right (outboard = -y); negative camber = top
    of tire toward the centreline for both."""
    return -np.degrees(np.arctan2(spindle[2], side * spindle[1]))


def _toe_of(spindle, side: int) -> float:
    """Top-view toe (deg, + = toe-in) of a GLOBAL-frame spindle."""
    s = np.asarray(spindle, float) * (MIRROR if side < 0 else 1.0)
    f = np.cross(s, [0.0, 0.0, 1.0])
    f = f / np.linalg.norm(f)
    return -np.degrees(np.arctan2(f[1], f[0]))


def _steer_of(spindle, side: int) -> float:
    """Wheel steer angle (deg, + = steering LEFT) in the global frame."""
    toe = _toe_of(spindle, side)
    return -toe if side > 0 else toe


def roll_sweep(solver, track_width: float, max_travel: float,
               n: int = 21) -> dict:
    """Opposite wheel travel: left at +t, right at -t, t in ±max_travel."""
    travels = np.linspace(-max_travel, max_travel, n)
    out = {k: [] for k in ("roll_deg", "travel_left_mm",
                           "camber_left_deg", "camber_right_deg",
                           "toe_left_deg", "toe_right_deg",
                           "rc_y_mm", "rc_z_mm")}
    for t in travels:
        left = solver.solve(t)
        right_local = solver.solve(-t)          # right corner, own frame
        phi = np.arctan2(2.0 * t, track_width)  # + = lean onto left wheels
        # Ground frame = chassis frame un-rolled: rotate by -phi about +x
        # (this puts the line between the two contact patches level).
        r = _rot_x(-phi)
        s_left = r @ left.spindle
        s_right = r @ (MIRROR * right_local.spindle)
        out["roll_deg"].append(np.degrees(phi))
        out["travel_left_mm"].append(t)
        out["camber_left_deg"].append(_camber_of(s_left, +1))
        out["camber_right_deg"].append(_camber_of(s_right, -1))
        out["toe_left_deg"].append(_toe_of(s_left, +1))
        out["toe_right_deg"].append(_toe_of(s_right, -1))
        rc = _asymmetric_roll_center(solver, left, right_local)
        out["rc_y_mm"].append(rc[0])
        out["rc_z_mm"].append(rc[1])
    res = {k: np.array(v) for k, v in out.items()}
    # Report RC height above the STATIC ground plane, not absolute z (the
    # Onshape-aligned coordinate offset puts the ground below z = 0).
    try:
        res["rc_z_mm"] = res["rc_z_mm"] - solver.solve(0.0).contact_patch[2]
    except ValueError:
        pass
    return res


def _asymmetric_roll_center(solver, left, right_local):
    """Roll centre with the two corners at DIFFERENT travels: intersect the
    left corner's contact-patch->IC line with the mirrored right one (in
    the chassis frame's front view). (nan, nan) if either is at infinity."""
    ic_l = front_view_ic(solver, left)
    ic_r = front_view_ic(solver, right_local)
    if ic_l is None or ic_r is None:
        return float("nan"), float("nan")
    p1 = np.array([left.contact_patch[1], left.contact_patch[2]])
    d1 = np.array(ic_l) - p1
    p2 = np.array([-right_local.contact_patch[1], right_local.contact_patch[2]])
    d2 = np.array([-ic_r[0], ic_r[1]]) - p2
    m = np.column_stack([d1, -d2])
    det = np.linalg.det(m)
    if abs(det) < 1e-9 * np.linalg.norm(d1) * np.linalg.norm(d2):
        return float("nan"), float("nan")
    s = np.linalg.solve(m, p2 - p1)
    rc = p1 + s[0] * d1
    return float(rc[0]), float(rc[1])


# public name for the GUI's live roll visualization
asymmetric_roll_center = _asymmetric_roll_center


def pitch_sweep(front_solver, rear_solver, wheelbase: float,
                cg_behind_front: float, max_pitch_deg: float,
                n: int = 21) -> dict:
    """Chassis pitch about the CG station; positive = nose down (braking).
    Front and rear travels are coupled: t_f = d_f*tan(theta) bump,
    t_r = -(wheelbase - d_f)*tan(theta)."""
    d_f = float(np.clip(cg_behind_front, 0.0, wheelbase))
    d_r = wheelbase - d_f
    pitches = np.radians(np.linspace(-max_pitch_deg, max_pitch_deg, n))
    out = {k: [] for k in ("pitch_deg", "travel_front_mm", "travel_rear_mm",
                           "caster_front_ground_deg", "camber_front_deg",
                           "camber_rear_deg", "wheelbase_change_mm")}
    for th in pitches:
        t_f, t_r = d_f * np.tan(th), -d_r * np.tan(th)
        f = front_solver.solve(t_f)
        r = rear_solver.solve(t_r)
        out["pitch_deg"].append(np.degrees(th))
        out["travel_front_mm"].append(t_f)
        out["travel_rear_mm"].append(t_r)
        # ground caster: express the chassis-frame kingpin in the ground
        # frame. Nose-down = the chassis rotated by +theta about +y (that
        # rotation sends x-forward downward), so chassis vectors pick up
        # exactly that rotation when viewed from the ground.
        k = _rot_y(th) @ (f.ubj - f.lbj)
        out["caster_front_ground_deg"].append(
            np.degrees(np.arctan2(-k[0], k[2])))
        out["camber_front_deg"].append(_camber_of(f.spindle, +1))
        out["camber_rear_deg"].append(_camber_of(r.spindle, +1))
        # wheelbase = front wc x  -  (rear wc x - wheelbase), both local
        out["wheelbase_change_mm"].append(
            f.wheel_center[0] - r.wheel_center[0])
    result = {k: np.array(v) for k, v in out.items()}
    result["wheelbase_change_mm"] -= result["wheelbase_change_mm"][n // 2]
    return result


def steer_sweep(front_solver, rack_limit: float, travel: float,
                track_width: float, wheelbase: float, n: int = 21) -> dict:
    """Rack sweep at a fixed wheel travel. Right wheel = the mirrored left
    corner fed the opposite rack displacement."""
    racks = np.linspace(-rack_limit, rack_limit, n)
    out = {k: [] for k in ("rack_mm", "steer_left_deg", "steer_right_deg",
                           "ackermann_pct", "camber_left_deg",
                           "camber_right_deg", "turn_diameter_m")}
    for s in racks:
        left = front_solver.solve(travel, s)
        right_local = front_solver.solve(travel, -s)
        d_l = _steer_of(left.spindle, +1)
        d_r = _steer_of(MIRROR * right_local.spindle, -1)
        out["rack_mm"].append(s)
        out["steer_left_deg"].append(d_l)
        out["steer_right_deg"].append(d_r)
        out["camber_left_deg"].append(_camber_of(left.spindle, +1))
        out["camber_right_deg"].append(
            _camber_of(MIRROR * right_local.spindle, -1))
        out["ackermann_pct"].append(
            _ackermann_pct(d_l, d_r, track_width, wheelbase))
        mean = np.radians((d_l + d_r) / 2.0)
        if abs(mean) > np.radians(0.5):
            r_rear = wheelbase / np.tan(abs(mean))
            out["turn_diameter_m"].append(
                2.0 * np.hypot(r_rear + track_width / 2.0, wheelbase) / 1000.0)
        else:
            out["turn_diameter_m"].append(float("nan"))
    return {k: np.array(v) for k, v in out.items()}


def _ackermann_pct(d_left, d_right, track, wheelbase) -> float:
    """Percent Ackermann: 100 = perfect (inner wheel exactly on the ideal
    cot relation), 0 = parallel steer, negative = anti-Ackermann. Only
    meaningful once the outer wheel is steering a real amount."""
    steering_left = (d_left + d_right) > 0.0
    d_i, d_o = (d_left, d_right) if steering_left else (-d_right, -d_left)
    if d_o < 1.0 or d_i <= 0.0:            # too straight to judge
        return float("nan")
    o = np.radians(d_o)
    ideal_i = np.degrees(np.arctan(1.0 / (1.0 / np.tan(o) - track / wheelbase)))
    if abs(ideal_i - d_o) < 1e-9:
        return float("nan")
    return 100.0 * (d_i - d_o) / (ideal_i - d_o)


def bump_steer_map(solver, travels, racks) -> dict:
    """Toe of the LEFT wheel over the full travel x rack grid — the map
    that shows whether your bump steer stays acceptable AT STEER, not just
    on-centre (with steer the tie rod works at an angle, so the toe-vs-
    travel curve changes shape as the rack moves). Also returns camber
    over the same grid. Grid arrays are indexed [i_travel, j_rack]."""
    travels = np.asarray(travels, dtype=float)
    racks = np.asarray(racks, dtype=float)
    toe = np.full((len(travels), len(racks)), np.nan)
    camber = np.full_like(toe, np.nan)
    for j, s in enumerate(racks):
        try:
            states = solver.walk_travels(travels, s)
        except (ValueError, TypeError):
            continue
        for i, st in enumerate(states):
            toe[i, j] = _toe_of(st.spindle, +1)
            camber[i, j] = _camber_of(st.spindle, +1)
    return {"travel_mm": travels, "rack_mm": racks,
            "toe_deg": toe, "camber_deg": camber}
