"""Relative-dimension tweaks: adjust kinematic geometry the way a designer
does by hand — "move the shock mount 10 mm out along the arm", "raise the
steering arm 5 mm" — instead of typing raw hardpoint coordinates.

Each tweak has a read side (derive the current dimension from the
hardpoints) and a write side (produce a new hardpoint set with that
dimension changed and everything else untouched). GUI-free and unit-tested.

Shock-mount parameterisation (the arm is the one the shock mounts to):

    inner_mid ●————————————● outer BJ        u  = along-arm direction
                  |d|——————→                 d  = distance along the arm
                       ● shock outer         h  = offset off the arm line
                      /|                     ang= rotation of that offset
                     h |                          about the arm line
                    ang↺                          (0 = straight up,
                                                   + tilts toward +X/front)

`h` and `ang` matter because the physical tab is often NOT on the 2D
sketch plane (clearance), and the mount is sometimes twisted because the
bump/droop split isn't 50/50.

Tie-rod heights are expressed as rises relative to their natural
references: the outer end relative to the LBJ (steering-arm height on the
knuckle) and the inner end relative to the LCA inboard axis midpoint —
the dimensions you nudge to hone bump steer.
"""

import dataclasses

import numpy as np

from .geometry import DoubleWishbonePoints

Z_UP = np.array([0.0, 0.0, 1.0])


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)


def _mount_arm_frame(hp: DoubleWishbonePoints):
    """(origin, u, p_up, p_fore) of the shock's mounting arm: origin at the
    inboard-axis midpoint, u along the arm toward the ball joint, p_up the
    perpendicular closest to vertical, p_fore completing the right-handed
    frame (points roughly toward +X / the front of the car)."""
    if hp.shock_on_uca:
        inner = (hp.uca_inner_front + hp.uca_inner_rear) / 2.0
        outer = hp.uca_outer
    else:
        inner = (hp.lca_inner_front + hp.lca_inner_rear) / 2.0
        outer = hp.lca_outer
    u = _unit(outer - inner)
    p_up = _unit(Z_UP - np.dot(Z_UP, u) * u)
    p_fore = np.cross(u, p_up)
    if p_fore[0] < 0.0:            # keep "+angle tilts toward the front"
        p_fore = -p_fore
    return inner, u, p_up, p_fore


def shock_mount_params(hp: DoubleWishbonePoints) -> dict:
    """Read the current (d, h, ang) of the shock's arm mount. Lengths in
    mm, ang in degrees; ang is 0 when the tab points straight up."""
    origin, u, p_up, p_fore = _mount_arm_frame(hp)
    rel = hp.shock_outer - origin
    d = float(np.dot(rel, u))
    perp = rel - d * u
    h = float(np.linalg.norm(perp))
    ang = 0.0 if h < 1e-9 else float(
        np.degrees(np.arctan2(np.dot(perp, p_fore), np.dot(perp, p_up))))
    return {"d": d, "h": h, "ang": ang}


def set_shock_mount(hp: DoubleWishbonePoints, d: float, h: float,
                    ang: float) -> DoubleWishbonePoints:
    """New hardpoints with the shock's arm mount at (d, h, ang). The
    chassis-side shock_inner stays put — moving the tab really does change
    the installed shock length, which the readouts will show."""
    origin, u, p_up, p_fore = _mount_arm_frame(hp)
    a = np.radians(ang)
    outer = origin + d * u + h * (np.cos(a) * p_up + np.sin(a) * p_fore)
    return dataclasses.replace(hp, shock_outer=outer)


def tierod_rises(hp: DoubleWishbonePoints) -> dict:
    """Current tie-rod heights relative to their kingpin-side references:
    outer above the LBJ, inner above the LCA inboard-axis midpoint (mm)."""
    lca_mid = (hp.lca_inner_front + hp.lca_inner_rear) / 2.0
    return {
        "outer_rise": float(hp.tierod_outer[2] - hp.lca_outer[2]),
        "inner_rise": float(hp.tierod_inner[2] - lca_mid[2]),
    }


def set_tierod_rises(hp: DoubleWishbonePoints, outer_rise: float,
                     inner_rise: float) -> DoubleWishbonePoints:
    """New hardpoints with the tie-rod ends at the requested rises. Only
    the z coordinates move — the classic bump-steer tuning motion."""
    lca_mid = (hp.lca_inner_front + hp.lca_inner_rear) / 2.0
    tro = hp.tierod_outer.copy()
    tri = hp.tierod_inner.copy()
    tro[2] = hp.lca_outer[2] + outer_rise
    tri[2] = lca_mid[2] + inner_rise
    return dataclasses.replace(hp, tierod_outer=tro, tierod_inner=tri)


# ----------------------------------------------------------------------
# Trailing-arm tweaks: shock on the arm line + the pivot-axis skew angles
# (the semi-trailing knobs that dial camber/toe gain)
# ----------------------------------------------------------------------
def _ta_arm_frame(hp):
    """Arm line for the shock decomposition: pivot-axis midpoint toward
    the wheel centre, with the same up/fore perpendicular frame."""
    inner = (hp.pivot_inner + hp.pivot_outer) / 2.0
    u = _unit(hp.wheel_center - inner)
    p_up = _unit(Z_UP - np.dot(Z_UP, u) * u)
    p_fore = np.cross(u, p_up)
    if p_fore[0] < 0.0:
        p_fore = -p_fore
    return inner, u, p_up, p_fore


def trailing_arm_params(hp) -> dict:
    """Current relative dimensions of a trailing-arm corner: the shock
    mount (d, h, ang) on the pivot-mid -> wheel-centre line, plus the
    pivot-axis skew: plan_deg (+ = outboard bushing swept REARWARD, the
    classic semi-trailing angle) and elev_deg (+ = outboard bushing
    higher). Both zero = pure trailing arm."""
    origin, u, p_up, p_fore = _ta_arm_frame(hp)
    rel = hp.shock_outer - origin
    d = float(np.dot(rel, u))
    perp = rel - d * u
    h = float(np.linalg.norm(perp))
    ang = 0.0 if h < 1e-9 else float(
        np.degrees(np.arctan2(np.dot(perp, p_fore), np.dot(perp, p_up))))
    a = hp.pivot_outer - hp.pivot_inner
    plan = float(np.degrees(np.arctan2(-a[0], a[1])))
    elev = float(np.degrees(np.arctan2(a[2], np.hypot(a[0], a[1]))))
    return {"d": d, "h": h, "ang": ang, "plan_deg": plan, "elev_deg": elev,
            "shock_len": installed_shock_length(hp),
            "ride_h": None, "corner_x": corner_x(hp),
            "corner_z": corner_z(hp)}


def set_trailing_arm(hp, d: float, h: float, ang: float, plan_deg: float,
                     elev_deg: float, shock_len: float = None,
                     hub_cv: float = None, ride_h: float = None,
                     corner_x: float = None, corner_z: float = None,
                     frame_tube_offset: float = 0.0, locked=frozenset()):
    """New trailing-arm hardpoints with the requested relative dimensions.
    The pivot_inner bushing and the axis LENGTH stay fixed; skewing swings
    pivot_outer around it. The shock inner (chassis) stays put."""
    length = float(np.linalg.norm(hp.pivot_outer - hp.pivot_inner))
    plan, elev = np.radians(plan_deg), np.radians(elev_deg)
    axis = np.array([-np.sin(plan) * np.cos(elev),
                     np.cos(plan) * np.cos(elev),
                     np.sin(elev)])
    hp2 = dataclasses.replace(hp, pivot_outer=hp.pivot_inner + length * axis)
    origin, u, p_up, p_fore = _ta_arm_frame(hp2)
    a = np.radians(ang)
    outer = origin + d * u + h * (np.cos(a) * p_up + np.sin(a) * p_fore)
    hp2 = dataclasses.replace(hp2, shock_outer=outer)
    return _apply_common(hp2, shock_len=shock_len, hub_cv=hub_cv,
                         ride_h=ride_h, corner_x=corner_x, corner_z=corner_z,
                         frame_tube_offset=frame_tube_offset, locked=locked)


# ----------------------------------------------------------------------
# Multilink tweaks: the toe-link rises (the bump-steer knobs)
# ----------------------------------------------------------------------
def multilink_toe_params(hp) -> dict:
    """Toe-link end heights relative to their natural references: the
    outer end above the WHEEL CENTRE, the inner end above the mean height
    of the four locating links' chassis points (the 'rack height' knob)."""
    inner_ref = np.mean([getattr(hp, f"link{k}_inner")[2]
                         for k in range(1, 5)])
    return {
        "outer_rise": float(hp.link5_outer[2] - hp.wheel_center[2]),
        "inner_rise": float(hp.link5_inner[2] - inner_ref),
        "shock_len": installed_shock_length(hp),
        "ride_h": None, "corner_x": corner_x(hp),
        "corner_z": corner_z(hp),
    }


def set_multilink_toe(hp, outer_rise: float, inner_rise: float,
                      shock_len: float = None, hub_cv: float = None,
                      ride_h: float = None, corner_x: float = None,
                      corner_z: float = None,
                      frame_tube_offset: float = 0.0, locked=frozenset()):
    """Move the toe-link end HEIGHTS — the classic multilink bump-steer
    tuning motion — plus any of the cross-type placement tweaks."""
    inner_ref = np.mean([getattr(hp, f"link{k}_inner")[2]
                         for k in range(1, 5)])
    outer = hp.link5_outer.copy()
    inner = hp.link5_inner.copy()
    outer[2] = hp.wheel_center[2] + outer_rise
    inner[2] = inner_ref + inner_rise
    hp2 = dataclasses.replace(hp, link5_outer=outer, link5_inner=inner)
    return _apply_common(hp2, shock_len=shock_len, hub_cv=hub_cv,
                         ride_h=ride_h, corner_x=corner_x, corner_z=corner_z,
                         frame_tube_offset=frame_tube_offset, locked=locked)


# ----------------------------------------------------------------------
# Hinge-carrier tweaks (C-hub front & loaded-halfshaft rear)
# ----------------------------------------------------------------------
def _carrier_pin(hp):
    """The carrier's rotation-axis endpoints for a hinge-carrier type:
    hinge_front/hinge_rear (C-hub, loaded halfshaft) or the H-arm's two
    outboard grab points (outer_front/outer_rear)."""
    if hasattr(hp, "hinge_front"):
        return hp.hinge_front, hp.hinge_rear
    return hp.outer_front, hp.outer_rear


def _carrier_arm_frame(hp):
    """Arm line for the shock decomposition on the hinge-carrier types:
    chassis-bushing midpoint toward the carrier-pin midpoint (the arm all
    these types mount their shock on), same up/fore frame as the other
    tweaks."""
    inner = (hp.arm_inner_front + hp.arm_inner_rear) / 2.0
    pf, pr = _carrier_pin(hp)
    pin = (pf + pr) / 2.0
    u = _unit(pin - inner)
    p_up = _unit(Z_UP - np.dot(Z_UP, u) * u)
    p_fore = np.cross(u, p_up)
    if p_fore[0] < 0.0:
        p_fore = -p_fore
    return inner, u, p_up, p_fore


def _carrier_shock_dha(hp) -> dict:
    origin, u, p_up, p_fore = _carrier_arm_frame(hp)
    rel = hp.shock_outer - origin
    d = float(np.dot(rel, u))
    perp = rel - d * u
    h = float(np.linalg.norm(perp))
    ang = 0.0 if h < 1e-9 else float(
        np.degrees(np.arctan2(np.dot(perp, p_fore), np.dot(perp, p_up))))
    return {"d": d, "h": h, "ang": ang}


def _carrier_set_shock(hp, d: float, h: float, ang: float):
    origin, u, p_up, p_fore = _carrier_arm_frame(hp)
    a = np.radians(ang)
    outer = origin + d * u + h * (np.cos(a) * p_up + np.sin(a) * p_fore)
    return dataclasses.replace(hp, shock_outer=outer)


def chub_params(hp) -> dict:
    """Current relative dimensions of a C-hub front corner: the shock
    mount (d, h, ang) on the arm line, the PHYSICAL kingpin's caster and
    KPI (the RC world's caster-block / KPI-insert angles — same sign
    conventions as the metrics), and axle_x = how far the wheel centre
    sits AHEAD of the kingpin axis at wheel-centre height (the steering-
    block axle offset, the mechanical-trail knob)."""
    k = hp.kingpin_upper - hp.kingpin_lower
    caster = float(np.degrees(np.arctan2(-k[0], k[2])))
    kpi = float(np.degrees(np.arctan2(-k[1], k[2])))
    x_axis = (hp.kingpin_lower[0]
              + (hp.wheel_center[2] - hp.kingpin_lower[2]) * k[0] / k[2])
    return {**_carrier_shock_dha(hp),
            "caster": caster, "kpi": kpi,
            "axle_x": float(hp.wheel_center[0] - x_axis),
            "shock_len": installed_shock_length(hp),
            **chub_tierod_rises(hp),
            "toe": chub_static_toe(hp),
            "steer_arm": chub_steer_arm(hp),
            "kp_len": chub_kingpin_length(hp),
            **camber_link_rises(hp),
            "arm_len": carrier_arm_length(hp),
            "sketch_yaw": sketch_yaw_deg(hp) or 0.0,
            "kickup": arm_axis_kickup(hp),
            "pill": chub_pill_caster_deg(hp),
            "hub_cv": hub_cv_offset(hp),
            "ride_h": None, "corner_x": corner_x(hp),
            "corner_z": corner_z(hp)}


def set_chub(hp, d: float, h: float, ang: float, caster: float, kpi: float,
             axle_x: float = None, shock_len: float = None,
             outer_rise: float = None, inner_rise: float = None,
             toe: float = None, steer_arm: float = None,
             kp_len: float = None,
             cam_outer_rise: float = None, cam_inner_rise: float = None,
             arm_len: float = None, kickup: float = None,
             pill: float = None,
             hub_cv: float = None, ride_h: float = None,
             corner_x: float = None, corner_z: float = None,
             frame_tube_offset: float = 0.0, locked=frozenset(), sketch_yaw: float | None = None):
    """New C-hub hardpoints with the requested dimensions. The kingpin
    re-aims about its own MIDPOINT (length kept) — exactly like swapping
    the C-hub / steering-block inserts for different caster / KPI parts;
    axle_x slides the wheel centre fore/aft only (trail tuning)."""
    hp = _pre_square(hp, sketch_yaw)
    hp2 = _carrier_set_shock(hp, d, h, ang)
    # The hub slide and axle_x BOTH place the wheel centre (the spindle is
    # not purely lateral once there is toe), so they cannot both be exact.
    # Do the hub slide FIRST and let axle_x land last: axle_x is the
    # mechanical-trail number a designer aims at, so it wins the tie.
    if hub_cv is not None:
        hp2 = set_hub_cv_offset(hp2, hub_cv)
        hub_cv = None
    mid = (hp.kingpin_upper + hp.kingpin_lower) / 2.0
    length = float(np.linalg.norm(hp.kingpin_upper - hp.kingpin_lower))
    dvec = np.array([-np.tan(np.radians(caster)),
                     -np.tan(np.radians(kpi)), 1.0])
    dvec = dvec / np.linalg.norm(dvec)
    ku = mid + 0.5 * length * dvec
    kl = mid - 0.5 * length * dvec
    hp2 = dataclasses.replace(hp2, kingpin_upper=ku, kingpin_lower=kl)
    if axle_x is not None:
        x_axis = kl[0] + (hp2.wheel_center[2] - kl[2]) * (ku[0] - kl[0]) / (
            ku[2] - kl[2])
        wc = hp2.wheel_center.copy()
        wc[0] = x_axis + axle_x
        hp2 = dataclasses.replace(hp2, wheel_center=wc)
    return _apply_common(hp2, shock_len=shock_len, hub_cv=hub_cv,
                         ride_h=ride_h, corner_x=corner_x, corner_z=corner_z,
                         frame_tube_offset=frame_tube_offset, locked=locked,
                         arm_len=arm_len, kickup=kickup,
                         cam_outer_rise=cam_outer_rise,
                         cam_inner_rise=cam_inner_rise,
                         chub_rises=(outer_rise, inner_rise),
                         chub_toe=toe, chub_steer_arm=steer_arm,
                         chub_kp_len=kp_len, chub_pill=pill)


def loaded_hs_params(hp) -> dict:
    """Current relative dimensions of a loaded-halfshaft corner: the
    shock mount (d, h, ang) on the upper-arm line, plus the knuckle
    bushing-pin skew: plan_deg (+ = front end of the pin swung OUTBOARD)
    and elev_deg (+ = front end raised). Skewing the pin couples the
    knuckle's swing into TOE — a GENTLE passive-steer knob (the knuckle
    only rotates a little relative to the arm; a straight fore-aft pin
    keeps toe fixed through travel). For stronger passive steer skew the
    ARM bushing axis itself (edit the arm_inner points)."""
    a = hp.hinge_front - hp.hinge_rear      # points forward at zero skew
    plan = float(np.degrees(np.arctan2(a[1], a[0])))
    elev = float(np.degrees(np.arctan2(a[2], np.hypot(a[0], a[1]))))
    return {**_carrier_shock_dha(hp), "plan_deg": plan, "elev_deg": elev,
            "shock_len": installed_shock_length(hp),
            "arm_len": carrier_arm_length(hp),
            "sketch_yaw": sketch_yaw_deg(hp) or 0.0,
            "kickup": arm_axis_kickup(hp),
            "hub_cv": hub_cv_offset(hp),
            "ride_h": None, "corner_x": corner_x(hp),
            "corner_z": corner_z(hp)}


def set_loaded_hs(hp, d: float, h: float, ang: float, plan_deg: float,
                  elev_deg: float, shock_len: float = None,
                  arm_len: float = None, kickup: float = None,
                  hub_cv: float = None, ride_h: float = None,
                  corner_x: float = None, corner_z: float = None,
                  frame_tube_offset: float = 0.0, locked=frozenset(), sketch_yaw: float | None = None):
    """New loaded-halfshaft hardpoints with the requested dimensions. The
    bushing pin re-aims about its own MIDPOINT (length kept); the shock
    mount re-seats on the arm line."""
    hp = _pre_square(hp, sketch_yaw)
    hp2 = _carrier_set_shock(hp, d, h, ang)
    mid = (hp.hinge_front + hp.hinge_rear) / 2.0
    length = float(np.linalg.norm(hp.hinge_front - hp.hinge_rear))
    plan, elev = np.radians(plan_deg), np.radians(elev_deg)
    axis = np.array([np.cos(plan) * np.cos(elev),
                     np.sin(plan) * np.cos(elev),
                     np.sin(elev)])
    hp2 = dataclasses.replace(hp2,
                              hinge_front=mid + 0.5 * length * axis,
                              hinge_rear=mid - 0.5 * length * axis)
    return _apply_common(hp2, shock_len=shock_len, hub_cv=hub_cv,
                         ride_h=ride_h, corner_x=corner_x, corner_z=corner_z,
                         frame_tube_offset=frame_tube_offset, locked=locked,
                         arm_len=arm_len, kickup=kickup)


def harm_params(hp) -> dict:
    """Current relative dimensions of an H-arm rear corner: the shock
    mount (d, h, ang) on the lower H-arm line, plus the outboard GRAB-LINE
    skew: plan_deg (+ = front grab point swung OUTBOARD) and elev_deg
    (+ = front grab point raised). Skewing the grab line couples the
    upright's swing into TOE — the H-arm's gentle passive-steer knob
    (a straight fore-aft grab line keeps toe fixed through travel)."""
    a = hp.outer_front - hp.outer_rear      # points forward at zero skew
    plan = float(np.degrees(np.arctan2(a[1], a[0])))
    elev = float(np.degrees(np.arctan2(a[2], np.hypot(a[0], a[1]))))
    return {**_carrier_shock_dha(hp), "plan_deg": plan, "elev_deg": elev,
            "shock_len": installed_shock_length(hp),
            **camber_link_rises(hp),
            "arm_len": carrier_arm_length(hp),
            "sketch_yaw": sketch_yaw_deg(hp) or 0.0,
            "kickup": arm_axis_kickup(hp),
            "ride_h": None, "corner_x": corner_x(hp),
            "corner_z": corner_z(hp)}


def set_harm(hp, d: float, h: float, ang: float, plan_deg: float,
             elev_deg: float, shock_len: float = None,
             cam_outer_rise: float = None, cam_inner_rise: float = None,
             arm_len: float = None, kickup: float = None,
             hub_cv: float = None, ride_h: float = None,
             corner_x: float = None, corner_z: float = None,
             frame_tube_offset: float = 0.0, locked=frozenset(), sketch_yaw: float | None = None):
    """New H-arm hardpoints with the requested dimensions. The outboard
    grab line re-aims about its own MIDPOINT (length kept); the shock
    mount re-seats on the H-arm line."""
    hp = _pre_square(hp, sketch_yaw)
    hp2 = _carrier_set_shock(hp, d, h, ang)
    mid = (hp.outer_front + hp.outer_rear) / 2.0
    length = float(np.linalg.norm(hp.outer_front - hp.outer_rear))
    plan, elev = np.radians(plan_deg), np.radians(elev_deg)
    axis = np.array([np.cos(plan) * np.cos(elev),
                     np.sin(plan) * np.cos(elev),
                     np.sin(elev)])
    hp2 = dataclasses.replace(hp2,
                              outer_front=mid + 0.5 * length * axis,
                              outer_rear=mid - 0.5 * length * axis)
    return _apply_common(hp2, shock_len=shock_len, hub_cv=hub_cv,
                         ride_h=ride_h, corner_x=corner_x, corner_z=corner_z,
                         frame_tube_offset=frame_tube_offset, locked=locked,
                         arm_len=arm_len, kickup=kickup,
                         cam_outer_rise=cam_outer_rise,
                         cam_inner_rise=cam_inner_rise)


# ----------------------------------------------------------------------
# Hub / outer-CV position along the kingpin (double wishbone)
# ----------------------------------------------------------------------
# The hub (and the halfshaft's outer CV joint inside it) sits at some
# height ALONG the kingpin axis. Sliding it re-mounts the wheel higher or
# lower on the upright: for the same hardpoints the tire moves relative
# to the chassis — a real ride-stance knob, and it moves the outer CV
# with it. Measured as the signed distance from the LBJ->UBJ midpoint to
# the wheel centre's projection onto the axis (+ = toward the UBJ).


def hub_along_kingpin(hp: DoubleWishbonePoints) -> float:
    a, b = hp.lca_outer, hp.uca_outer
    d = b - a
    d = d / np.linalg.norm(d)
    mid = (a + b) / 2.0
    return float(np.dot(hp.wheel_center - mid, d))


def set_hub_along_kingpin(hp: DoubleWishbonePoints,
                          t: float) -> DoubleWishbonePoints:
    """Slide the wheel centre (hub + outer CV) along the kingpin axis to
    signed position `t` from the ball-joint midpoint. Nothing else moves:
    the knuckle stays, so camber/caster/KPI are unchanged — the wheel
    (and tire) translate along the axis."""
    a, b = hp.lca_outer, hp.uca_outer
    d = b - a
    d = d / np.linalg.norm(d)
    delta = (t - hub_along_kingpin(hp)) * d
    return dataclasses.replace(hp, wheel_center=hp.wheel_center + delta)


# ----------------------------------------------------------------------
# Steering-arm length (double wishbone): kingpin axis -> tie-rod outer,
# measured along the sketch normal — the lever the rack pushes on. A
# longer arm lowers steering effort but also lowers the max road-wheel
# angle for the same rack travel.
# ----------------------------------------------------------------------
def _sketch_normal(hp: DoubleWishbonePoints) -> np.ndarray:
    """The (mean) bushing-axis direction = the design sketch's normal."""
    au = _unit(hp.uca_inner_front - hp.uca_inner_rear)
    al = _unit(hp.lca_inner_front - hp.lca_inner_rear)
    if np.dot(au, al) < 0.0:
        al = -al
    return _unit(au + al)


def steer_arm_length(hp: DoubleWishbonePoints) -> float:
    """Signed distance from the kingpin axis to tierod_outer measured
    along the sketch normal (+ = the arm sweeps toward the front for a
    normal front-steer layout, matching the seed's rearward tie rod being
    negative or positive per the design)."""
    n = _sketch_normal(hp)
    a, b = hp.lca_outer, hp.uca_outer
    d = _unit(b - a)
    p = a + np.dot(hp.tierod_outer - a, d) * d
    return float(np.dot(hp.tierod_outer - p, n))


def set_steer_arm_length(hp: DoubleWishbonePoints,
                         length: float) -> DoubleWishbonePoints:
    """Slide tierod_outer along the sketch normal until the steering arm
    reads `length`. The tie-rod inner (rack end) stays put and the link's
    working length is re-derived from the two points, so static toe — a
    stored alignment parameter — is untouched: only steering effort /
    ratio / max angle change.

    The reading's kingpin foot point follows tierod_outer, so a move of
    D along the normal changes the reading by D*(1 - (n.d)^2) — divide
    by that factor (affine, exact) instead of iterating."""
    n = _sketch_normal(hp)
    a, b = hp.lca_outer, hp.uca_outer
    d = _unit(b - a)
    gain = 1.0 - float(np.dot(n, d)) ** 2
    if gain < 1e-6:      # kingpin parallel to sketch normal: not a car
        return hp
    delta = (length - steer_arm_length(hp)) / gain * n
    return dataclasses.replace(hp, tierod_outer=hp.tierod_outer + delta)


# ----------------------------------------------------------------------
# v1.9 sketch-dimension tweaks (double wishbone): the numbers a designer
# dimensions ON the 2D sketch — kingpin length, the direct gap between
# the two inboard pivot axes, and each arm's in-sketch length. All
# "move + flag": the tweak applies exactly; if travel shrinks, the
# window auto-clamps and the checklist's articulation row goes red.
# ----------------------------------------------------------------------
def kingpin_length(hp: DoubleWishbonePoints) -> float:
    """Overall kingpin length: LBJ to UBJ."""
    return float(np.linalg.norm(hp.uca_outer - hp.lca_outer))


def set_kingpin_length(hp: DoubleWishbonePoints,
                       length: float) -> DoubleWishbonePoints:
    """Slide the UBJ along the kingpin axis to the requested length. The
    LBJ, lower arm and wheel centre stay put — the classic upper-arm
    camber-gain adjustment."""
    d = _unit(hp.uca_outer - hp.lca_outer)
    return dataclasses.replace(hp, uca_outer=hp.lca_outer + length * d)


def inboard_axis_sep(hp: DoubleWishbonePoints) -> float:
    """Direct (closest) distance between the UCA and LCA inboard pivot
    AXES — the camber-curve driver (keep well below the kingpin span or
    the front-view IC crosses over)."""
    mu = (hp.uca_inner_front + hp.uca_inner_rear) / 2.0
    ml = (hp.lca_inner_front + hp.lca_inner_rear) / 2.0
    du = _unit(hp.uca_inner_front - hp.uca_inner_rear)
    dl = _unit(hp.lca_inner_front - hp.lca_inner_rear)
    n = np.cross(du, dl)
    if np.linalg.norm(n) < 1e-6:      # parallel (the squared case)
        rel = mu - ml
        return float(np.linalg.norm(rel - np.dot(rel, dl) * dl))
    return float(abs(np.dot(mu - ml, _unit(n))))


def _axis_sep_direction(hp) -> np.ndarray:
    """Unit direction from the LCA axis toward the UCA axis along their
    common perpendicular (the direction that changes the separation)."""
    mu = (hp.uca_inner_front + hp.uca_inner_rear) / 2.0
    ml = (hp.lca_inner_front + hp.lca_inner_rear) / 2.0
    du = _unit(hp.uca_inner_front - hp.uca_inner_rear)
    dl = _unit(hp.lca_inner_front - hp.lca_inner_rear)
    n = np.cross(du, dl)
    if np.linalg.norm(n) < 1e-6:
        rel = mu - ml
        perp = rel - np.dot(rel, dl) * dl
        return _unit(perp)
    n = _unit(n)
    return n if np.dot(mu - ml, n) >= 0.0 else -n


def set_inboard_axis_sep(hp: DoubleWishbonePoints,
                         sep: float) -> DoubleWishbonePoints:
    """Translate the UPPER inboard pair (both bushings together, axis
    direction and spread kept) along the axes' common perpendicular to
    the requested separation. Lower arm and ball joints stay put —
    consistent with the kingpin-length convention (upper side moves)."""
    delta = (sep - inboard_axis_sep(hp)) * _axis_sep_direction(hp)
    return dataclasses.replace(
        hp,
        uca_inner_front=hp.uca_inner_front + delta,
        uca_inner_rear=hp.uca_inner_rear + delta)


def arm_length_sketch(hp: DoubleWishbonePoints, upper: bool) -> float:
    """The arm's length as dimensioned ON the 2D sketch: inboard-axis
    midpoint to ball joint, projected into the sketch plane (the BJ's
    sketch-normal caster offset doesn't count)."""
    if upper:
        mid = (hp.uca_inner_front + hp.uca_inner_rear) / 2.0
        bj = hp.uca_outer
    else:
        mid = (hp.lca_inner_front + hp.lca_inner_rear) / 2.0
        bj = hp.lca_outer
    n = _sketch_normal(hp)
    rel = bj - mid
    return float(np.linalg.norm(rel - np.dot(rel, n) * n))


def set_arm_length_sketch(hp: DoubleWishbonePoints, upper: bool,
                          length: float) -> DoubleWishbonePoints:
    """Slide the ball joint toward/away from the inboard midpoint in the
    sketch plane; its sketch-normal (caster) offset is preserved. The
    in-sketch camber dial: pulling an arm shorter/longer bends the
    camber curve without touching the frame side."""
    if upper:
        mid = (hp.uca_inner_front + hp.uca_inner_rear) / 2.0
        bj = hp.uca_outer
    else:
        mid = (hp.lca_inner_front + hp.lca_inner_rear) / 2.0
        bj = hp.lca_outer
    n = _sketch_normal(hp)
    rel = bj - mid
    off = np.dot(rel, n) * n            # keep the caster offset
    inplane = rel - off
    r = np.linalg.norm(inplane)
    if r < 1e-9:
        return hp                       # degenerate: BJ on the axis
    new_bj = mid + off + (length / r) * inplane
    key = "uca_outer" if upper else "lca_outer"
    return dataclasses.replace(hp, **{key: new_bj})


# ----------------------------------------------------------------------
# Post-seed KINGPIN targets (double wishbone): adjust scrub radius and
# caster IN PLACE, without regenerating the seed. Each was previously a
# one-shot seed input baked into the hardpoints and then unrecoverable; a
# designer who had hand-placed the arms had to start over to re-target
# them. These invert that: the scrub lever leans the kingpin sideways
# (moves the UBJ in Y), the caster lever tilts it fore/aft (moves the UBJ
# in X). The LBJ, lower arm and wheel centre stay put, so TRACK and the
# stored static camber/toe are preserved; only the kingpin angle (and the
# quantity being targeted) change — exactly the coupling on the real car.
# ----------------------------------------------------------------------
class LockedPointError(ValueError):
    """A tweak needed to move a hardpoint the user has pinned/locked.

    Carries the offending point name so the GUI can say which lock to
    release. Raised instead of silently moving a locked point."""

    def __init__(self, point: str, message: str):
        super().__init__(message)
        self.point = point


def _static_contact_patch(hp: DoubleWishbonePoints) -> np.ndarray:
    """The static (ride-height) contact patch, built exactly like the
    solver's: one tire radius down the wheel plane from the wheel centre,
    so camber tips it sideways like a real cambered wheel."""
    from .geometry import static_spindle
    s = static_spindle(hp)
    fwd = _unit(np.cross(s, Z_UP))
    down = _unit(np.cross(s, fwd))
    return hp.wheel_center + hp.tire_radius * down


def scrub_radius(hp: DoubleWishbonePoints) -> float:
    """Static scrub radius (mm): contact patch outboard (+) of where the
    kingpin axis pierces the ground. Matches metrics.scrub_radius_mm at
    ride height."""
    cp = _static_contact_patch(hp)
    k = hp.uca_outer - hp.lca_outer
    t = (cp[2] - hp.lca_outer[2]) / k[2]
    ground_y = hp.lca_outer[1] + t * k[1]
    return float(cp[1] - ground_y)


def _secant(measure, x0: float, step: float, target: float,
            iters: int = 60) -> float:
    """Return x with measure(x) ≈ target (1D, smooth), warm from x0."""
    f0 = measure(x0) - target
    x1 = x0 + step
    f1 = measure(x1) - target
    for _ in range(iters):
        if abs(f1) < 1e-9 or f1 == f0:
            break
        x0, x1, f0 = x1, x1 - f1 * (x1 - x0) / (f1 - f0), f1
        f1 = measure(x1) - target
    return x1


def set_scrub_radius(hp: DoubleWishbonePoints, target: float,
                     locked=frozenset()) -> DoubleWishbonePoints:
    """Lean the kingpin sideways (move the UBJ in Y) until the scrub radius
    reads `target` mm. LBJ, lower arm and wheel centre stay put, so track
    and the stored static camber/toe are unchanged — KPI moves with scrub,
    as it must at fixed track. Raises LockedPointError if the UBJ is
    pinned."""
    if "uca_outer" in locked:
        raise LockedPointError(
            "uca_outer",
            "scrub radius is changed by leaning the kingpin (moving the "
            "UBJ sideways), but the UBJ is locked — unlock it to retarget "
            "scrub")

    def measure(uy):
        u = hp.uca_outer.copy()
        u[1] = uy
        return scrub_radius(dataclasses.replace(hp, uca_outer=u))

    uy = _secant(measure, float(hp.uca_outer[1]), 5.0, float(target))
    u = hp.uca_outer.copy()
    u[1] = uy
    return dataclasses.replace(hp, uca_outer=u)


def caster_angle_deg(hp: DoubleWishbonePoints) -> float:
    """Static caster (deg): top of the kingpin axis leaning rearward is
    positive. Matches metrics.caster_deg at ride height."""
    k = hp.uca_outer - hp.lca_outer
    return float(np.degrees(np.arctan2(-k[0], k[2])))


def set_caster_angle_deg(hp: DoubleWishbonePoints, target: float,
                         locked=frozenset()) -> DoubleWishbonePoints:
    """Tilt the kingpin fore/aft (move the UBJ in X) until caster reads
    `target` deg — the upper-ball-joint caster adjustment. Scrub, track and
    static camber/toe are untouched (X and Y levers are independent).

    NOTE: in the default one-sketch workflow, hitting Re-square pulls the
    kingpin back into the sketch plane and caster reverts to following the
    kickup. Enable independent-caster mode to keep a hand-set caster across
    a re-square. Raises LockedPointError if the UBJ is pinned."""
    if "uca_outer" in locked:
        raise LockedPointError(
            "uca_outer",
            "caster is changed by tilting the kingpin fore/aft (moving the "
            "UBJ), but the UBJ is locked — unlock it to retarget caster")

    def measure(ux):
        u = hp.uca_outer.copy()
        u[0] = ux
        return caster_angle_deg(dataclasses.replace(hp, uca_outer=u))

    ux = _secant(measure, float(hp.uca_outer[0]), 5.0, float(target))
    u = hp.uca_outer.copy()
    u[0] = ux
    return dataclasses.replace(hp, uca_outer=u)


# ----------------------------------------------------------------------
# Static toe via tie-rod length (double wishbone): the mechanic's
# adjustment. Threading the tie rod longer/shorter rotates the whole
# knuckle about the kingpin — so the wheel, hub and the tie rod's
# knuckle-side point all swing together, and the link's working length
# changes implicitly. This keeps geometry and the stored static toe
# CONSISTENT (typing static_toe alone only re-aims the spindle).
# ----------------------------------------------------------------------
def _rot_about_axis(p, origin, axis, ang):
    """Rodrigues rotation of point p about the (origin, axis) line."""
    v = np.asarray(p, float) - origin
    k = _unit(np.asarray(axis, float))
    c, s = np.cos(ang), np.sin(ang)
    return origin + v * c + np.cross(k, v) * s + k * np.dot(k, v) * (1 - c)


def _static_spindle_of(camber_deg, toe_deg):
    # Exact construction (matches geometry.static_spindle): camber and toe
    # both read back exactly for any input; identical to the old sequential
    # form when toe = 0.
    g, t = np.radians(camber_deg), np.radians(toe_deg)
    s = np.array([np.tan(t), 1.0, -np.tan(g)])
    return s / np.linalg.norm(s)


def set_toe_by_tierod(hp: DoubleWishbonePoints,
                      toe_deg: float) -> DoubleWishbonePoints:
    """Rotate the knuckle about the kingpin until the wheel reads
    `toe_deg` at ride height, moving tierod_outer and wheel_center with
    it and updating the stored static toe AND camber (a leaned kingpin
    couples them, exactly like on the real car). The tie-rod length
    re-derives from the moved point — that length change IS the
    adjustment a mechanic would make."""
    a = np.asarray(hp.lca_outer, float)
    d = _unit(np.asarray(hp.uca_outer, float) - a)
    s0 = _static_spindle_of(hp.static_camber_deg, hp.static_toe_deg)

    def rot(v, b):
        return (v * np.cos(b) + np.cross(d, v) * np.sin(b)
                + d * np.dot(d, v) * (1 - np.cos(b)))

    # metrics.py conventions: toe-in positive (heading toward -Y),
    # negative camber = top leans inboard
    def toe_of(s):
        f = _unit(np.cross(s, np.array([0.0, 0.0, 1.0])))
        return float(-np.degrees(np.arctan2(f[1], f[0])))

    def camber_of(s):
        return float(-np.degrees(np.arctan2(s[2], s[1])))

    # solve the knuckle rotation about the kingpin (1D, smooth): secant
    target = float(toe_deg)
    b0, f0 = 0.0, toe_of(s0) - target
    b1 = np.radians(target - toe_of(s0)) or 1e-4
    f1 = toe_of(rot(s0, b1)) - target
    for _ in range(50):
        if abs(f1) < 1e-12 or f1 == f0:
            break
        b0, b1, f0 = b1, b1 - f1 * (b1 - b0) / (f1 - f0), f1
        f1 = toe_of(rot(s0, b1)) - target
    beta = b1
    s_new = rot(s0, beta)
    return dataclasses.replace(
        hp,
        tierod_outer=_rot_about_axis(hp.tierod_outer, a, d, beta),
        wheel_center=_rot_about_axis(hp.wheel_center, a, d, beta),
        static_toe_deg=toe_of(s_new),
        static_camber_deg=camber_of(s_new),
    )


# ----------------------------------------------------------------------
# Hub offset = outer-CV -> tire-centre distance (double wishbone). This is
# the hub/knuckle "stickout": how far the wheel centre stands off from the
# outer CV joint (which sits on the kingpin axis at the spindle line). It's
# the number hub, bearing and CV-cup packaging is built around, and moving
# it slides the tire in/out along its spin axis — scrub radius and
# half-track change, camber / caster / KPI do not.
# ----------------------------------------------------------------------
def hub_cv_offset(hp: DoubleWishbonePoints) -> float:
    """Distance from the outer CV joint to the tire centre (wheel centre)."""
    from .halfshaft import outer_cv_of_hp
    return float(np.linalg.norm(hp.wheel_center - outer_cv_of_hp(hp)))


def set_hub_cv_offset(hp: DoubleWishbonePoints,
                      dist: float) -> DoubleWishbonePoints:
    """Slide the wheel centre along its spin (spindle) axis until the outer
    CV -> tire-centre distance reads `dist`. The knuckle and kingpin stay
    put, so camber/caster/KPI are unchanged; the outer CV (derived from the
    wheel centre and kingpin) and the tire move with it. Iterated because
    the CV foot depends on the wheel centre it is measured from."""
    from .geometry import static_spindle
    from .halfshaft import outer_cv_of_hp
    s = static_spindle(hp)
    s = s / np.linalg.norm(s)
    if s[1] < 0.0:                    # point outboard (left corner: +Y)
        s = -s
    # The wheel centre can get no closer to the CV than the spin axis's
    # perpendicular offset from the kingpin; clamp so an unreachable target
    # doesn't walk the iteration off.
    v0 = hp.wheel_center - outer_cv_of_hp(hp)
    min_reach = float(np.linalg.norm(v0 - np.dot(v0, s) * s))
    dist = max(float(dist), min_reach, 1e-6)
    hp2 = hp
    for _ in range(8):
        d_now = hub_cv_offset(hp2)
        step = dist - d_now
        if abs(step) < 1e-9:
            break
        hp2 = dataclasses.replace(hp2, wheel_center=hp2.wheel_center + step * s)
    return hp2


# ----------------------------------------------------------------------
# Installed shock length at ride (double wishbone): eye-to-eye length of
# the shock in the static pose. It sets how the shock's stroke is split
# between bump and droop (a shorter installed length reserves more droop),
# so it's the knob to hit a target travel split. Set by sliding the
# arm-side mount ALONG the arm, keeping its off-arm height and angle.
# ----------------------------------------------------------------------
def shock_length_at_ride(hp: DoubleWishbonePoints) -> float:
    """Installed (eye-to-eye) shock length in the current static pose."""
    return float(np.linalg.norm(hp.shock_outer - hp.shock_inner))


def set_shock_length_at_ride(hp: DoubleWishbonePoints,
                             length: float) -> DoubleWishbonePoints:
    """Slide the shock's arm mount along the arm (keeping h and ang) until
    the installed length matches `length`. The chassis end stays put."""
    p = shock_mount_params(hp)
    origin, u, p_up, p_fore = _mount_arm_frame(hp)
    a = np.radians(p["ang"])
    off = p["h"] * (np.cos(a) * p_up + np.sin(a) * p_fore)
    inner = hp.shock_inner

    def excess(d):
        return float(np.linalg.norm(origin + d * u + off - inner)) - length

    d0 = p["d"]
    f0 = excess(d0)
    d1 = d0 + 10.0
    f1 = excess(d1)
    for _ in range(60):
        if abs(f1) < 1e-9 or f1 == f0:
            break
        d0, d1, f0 = d1, d1 - f1 * (d1 - d0) / (f1 - f0), f1
        f1 = excess(d1)
    return set_shock_mount(hp, d1, p["h"], p["ang"])


def shock_travel_split(hp: DoubleWishbonePoints, shock_min: float,
                       shock_max: float) -> dict:
    """How the installed length splits the shock stroke: fraction of the
    stroke still available in bump (compression) and droop (extension).
    Returns {'bump_frac', 'droop_frac', 'length', 'stroke'} (fracs sum to
    1; clamped to [0,1] so an out-of-range length reads 0/100)."""
    length = shock_length_at_ride(hp)
    stroke = float(shock_max) - float(shock_min)
    if stroke <= 1e-9:
        return {"bump_frac": float("nan"), "droop_frac": float("nan"),
                "length": length, "stroke": stroke}
    bump = float(np.clip((length - float(shock_min)) / stroke, 0.0, 1.0))
    return {"bump_frac": bump, "droop_frac": 1.0 - bump,
            "length": length, "stroke": stroke}


# ----------------------------------------------------------------------
# Sketch kickup (double wishbone): change the chassis kickup angle IN
# PLACE — no reseed. The four inboard bushings rotate about their own
# midpoints onto the new sketch normal (midpoints, spreads and the sketch
# yaw are all kept), so the frame tubes stay where they are and only the
# axis angle changes; the kingpin is then re-planarized so caster follows
# the new kickup, exactly as a seed would. This is the knob for "how does
# kickup move my roll centre" studies — the reseed route moves half the
# chassis points through placement heuristics and buries the answer.
# ----------------------------------------------------------------------
def sketch_kickup(hp: DoubleWishbonePoints) -> float:
    """Measured sketch kickup (deg): elevation of the mean bushing axis."""
    from .geometry import sketch_planarity
    return float(sketch_planarity(hp)["sketch_kickup_deg"])


def set_sketch_kickup(hp: DoubleWishbonePoints, kickup_deg: float,
                      planarize: bool = True,
                      locked=frozenset()) -> DoubleWishbonePoints:
    """Rotate the bushing axes to `kickup_deg` about their own midpoints
    (yaw and spreads preserved), then re-planarize the kingpin (caster
    follows kickup, as in the default one-sketch workflow; pass
    planarize=False in independent-caster mode). Raises LockedPointError
    if any inboard bushing is pinned."""
    from .geometry import planarize_kingpin, sketch_planarity
    from .seed import sketch_plane_normal
    pairs = (("uca_inner_front", "uca_inner_rear"),
             ("lca_inner_front", "lca_inner_rear"))
    for pair in pairs:
        for attr in pair:
            if attr in locked:
                raise LockedPointError(
                    attr, "changing kickup rotates the inboard bushing "
                          f"axes, but {attr} is locked — unlock the "
                          "bushings to retune kickup")
    yaw = float(sketch_planarity(hp)["axis_yaw_deg"])
    n = sketch_plane_normal(float(kickup_deg), yaw)
    kw = {}
    for front, rear in pairs:
        f, r = getattr(hp, front), getattr(hp, rear)
        mid = (f + r) / 2.0
        spread = 0.5 * float(np.linalg.norm(f - r))
        sign = 1.0 if np.dot(f - r, n) >= 0.0 else -1.0
        kw[front] = mid + sign * spread * n
        kw[rear] = mid - sign * spread * n
    hp2 = dataclasses.replace(hp, **kw)
    if planarize and "uca_outer" not in locked:
        hp2 = planarize_kingpin(hp2)
    return hp2


# ----------------------------------------------------------------------
# Corner fore/aft position (any type): a PLACEMENT knob like ride height,
# but along the chassis-forward (X) axis. Rigidly slides the WHOLE corner
# — 2D sketch plane, arms, knuckle, shock — forward/backward on the frame
# without touching any kinematics, so the initial seed placement can be
# re-tuned without reseeding. Measured at the wheel centre's X station.
# ----------------------------------------------------------------------
def corner_x(hp) -> float:
    """The corner's fore/aft station: wheel-centre X (mm, + = forward)."""
    return float(hp.wheel_center[0])


def _refuse_rigid_move_if_locked(hp, locked, what: str) -> None:
    """A rigid whole-corner translation moves EVERY point — locked ones
    included — so with any pin set it must refuse, not override."""
    for attr in type(hp).POINT_ATTRS:
        if attr in locked:
            raise LockedPointError(
                attr, f"{what} rigidly moves the whole corner, but "
                      f"{attr} is locked — unlock it (or move the free "
                      "points individually) first")


def set_corner_x(hp, x: float, locked=frozenset()):
    """Rigidly translate the whole corner in X so the wheel centre lands
    at station `x`. No kinematics change — the same pure placement move
    as the seed's x_offset, but live. Refuses if any point is locked."""
    dx = float(x) - corner_x(hp)
    if abs(dx) < 1e-12:
        return hp
    _refuse_rigid_move_if_locked(hp, locked, "the corner fore/aft tweak")
    shift = np.array([dx, 0.0, 0.0])
    return dataclasses.replace(
        hp, **{attr: getattr(hp, attr) + shift
               for attr in type(hp).POINT_ATTRS})


def corner_z(hp) -> float:
    """The corner's vertical station: wheel-centre Z (mm, + = up)."""
    return float(hp.wheel_center[2])


def set_corner_z(hp, z: float, locked=frozenset()):
    """Rigidly translate the whole corner in Z so the wheel centre lands at
    height `z`. Changes NO kinematics — the direct vertical twin of
    set_corner_x, for nudging a corner onto the frame without reseeding.

    This is deliberately separate from the ride-height tweak: ride height
    is a TARGET measured from the ground up to the frame datum (so it moves
    the corner the OTHER way and depends on the tire radius and the
    frame-tube offset), which is the wrong mental model when all you want
    is "shift this corner up 10 mm". Refuses if any point is locked."""
    dz = float(z) - corner_z(hp)
    if abs(dz) < 1e-12:
        return hp
    _refuse_rigid_move_if_locked(hp, locked, "the corner up/down tweak")
    shift = np.array([0.0, 0.0, dz])
    return dataclasses.replace(
        hp, **{attr: getattr(hp, attr) + shift
               for attr in type(hp).POINT_ATTRS})


# ----------------------------------------------------------------------
# Ride height (any type): a PLACEMENT knob, not a kinematic one. Ride
# height is the ground-to-frame-datum distance; the datum sits
# `frame_tube_offset` above the model origin (the frame-tube thickness in
# the CAD model). Setting it rigidly translates the whole corner in z —
# which changes NO kinematics, exactly like the seed's z-offset — so the
# design lands at the right stance on the CAD frame. tire radius sets the
# ground; camber's tiny contact-patch shift is ignored for this datum.
# ----------------------------------------------------------------------
def ride_height(hp, frame_tube_offset: float = 0.0) -> float:
    """Static ride height: ground (tire contact) up to the frame datum,
    with the datum `frame_tube_offset` above the model origin."""
    cp_z = float(hp.wheel_center[2]) - float(hp.tire_radius)
    return -cp_z - float(frame_tube_offset)


def set_ride_height(hp, target: float, frame_tube_offset: float = 0.0,
                    locked=frozenset()):
    """Rigidly translate the corner in z so the ride height reads `target`.
    No kinematics change — it only places the design vertically on the CAD
    frame, driven by an intuitive ride-height number and the frame-tube
    datum instead of a raw z-offset. Refuses if any point is locked."""
    dz = ride_height(hp, frame_tube_offset) - float(target)
    if abs(dz) < 1e-12:
        return hp
    _refuse_rigid_move_if_locked(hp, locked, "the ride-height tweak")
    shift = np.array([0.0, 0.0, dz])
    return dataclasses.replace(
        hp, **{attr: getattr(hp, attr) + shift
               for attr in type(hp).POINT_ATTRS})


# ======================================================================
# Cross-type tweaks (v1.31)
# ----------------------------------------------------------------------
# Most of what a designer wants to adjust is not double-wishbone-specific
# — an installed shock length, a hub offset, a bushing-axis kickup and an
# arm length mean the same thing on every type. Until now only the double
# wishbone exposed them, so the other five types were tunable almost
# exclusively by hand-editing raw coordinates. These helpers dispatch on
# whatever structure the hardpoint set actually has, so one implementation
# serves every type that can support the knob.
# ======================================================================

def _arm_frame_of(hp):
    """(origin, u, p_up, p_fore) for the arm this type mounts its shock
    on, or None when the type has no single arm (multilink). `u` runs
    outboard along the arm; p_up/p_fore span the plane normal to it."""
    if isinstance(hp, DoubleWishbonePoints):
        return _mount_arm_frame(hp)
    if hasattr(hp, "pivot_inner"):                  # trailing arm
        return _ta_arm_frame(hp)
    if hasattr(hp, "arm_inner_front"):              # C-hub / loaded HS / H-arm
        return _carrier_arm_frame(hp)
    return None                                     # multilink: no arm


def installed_shock_length(hp) -> float:
    """Eye-to-eye shock length in the static pose (every type)."""
    return float(np.linalg.norm(hp.shock_outer - hp.shock_inner))


def set_installed_shock_length(hp, length: float):
    """Re-seat the shock's OUTER mount so the installed length reads
    `length`. On a type with an arm the mount slides ALONG that arm,
    keeping its off-arm height and angle — the same motion the double
    wishbone has always used, so the motion ratio moves with it. On a
    multilink (no single arm) the mount slides along the shock's own axis
    instead, which changes the length without disturbing the linkage."""
    target = float(length)
    if target <= 0.0:
        raise ValueError("installed shock length must be positive")
    frame = _arm_frame_of(hp)
    if frame is None:                               # multilink
        v = hp.shock_outer - hp.shock_inner
        n = float(np.linalg.norm(v))
        if n < 1e-9:
            raise ValueError("shock has zero length; cannot re-seat it")
        return dataclasses.replace(
            hp, shock_outer=hp.shock_inner + (target / n) * v)
    origin, u, p_up, p_fore = frame
    rel = hp.shock_outer - origin
    d0 = float(np.dot(rel, u))
    perp = rel - d0 * u
    h = float(np.linalg.norm(perp))
    a = 0.0 if h < 1e-9 else float(
        np.arctan2(np.dot(perp, p_fore), np.dot(perp, p_up)))
    off = h * (np.cos(a) * p_up + np.sin(a) * p_fore)

    def length_at(d):
        return float(np.linalg.norm(origin + d * u + off - hp.shock_inner))

    d1 = _secant(length_at, d0, 10.0, target)
    return dataclasses.replace(hp, shock_outer=origin + d1 * u + off)


# -- inboard bushing-axis kickup, for the single-arm carrier types ------
# The double wishbone's `set_sketch_kickup` rotates BOTH bushing axes.
# C-hub, loaded-halfshaft and H-arm corners have exactly one chassis
# bushing axis (arm_inner_front/rear) and had no control over it at all,
# even though its elevation is what sets anti-dive / anti-squat and, on an
# RC buggy, is the single most-tuned front-end number.

def _has_arm_axis(hp) -> bool:
    return hasattr(hp, "arm_inner_front") and hasattr(hp, "arm_inner_rear")


def arm_axis_kickup(hp) -> float:
    """Elevation of the chassis bushing axis (deg, + = front end higher),
    i.e. the chassis kickup this corner is built with."""
    a = hp.arm_inner_front - hp.arm_inner_rear
    return float(np.degrees(np.arctan2(a[2], np.hypot(a[0], a[1]))))


def set_arm_axis_kickup(hp, kickup_deg: float):
    """Re-aim the chassis bushing axis to `kickup_deg` of elevation about
    its own midpoint, keeping its length and its plan (yaw) angle.

    The CARRIER PIN is rotated by the same amount, exactly as the double
    wishbone's set_sketch_kickup tilts BOTH bushing axes together. Before
    v1.34 only the arm moved, so the pin was left behind and the corner
    came out of square by the full kickup angle (25 deg of kickup meant
    25 deg of misalignment) -- which is what made a C-hub go strange as
    soon as the kickup was touched. Nothing the carrier holds moves, so
    the kingpin's angle relative to its pin -- the caster pill -- is
    unchanged, just as the DW leaves its knuckle on the sketch."""
    a = hp.arm_inner_front - hp.arm_inner_rear
    length = float(np.linalg.norm(a))
    if length < 1e-9:
        raise ValueError("bushing axis has zero length")
    plan = float(np.arctan2(a[1], a[0]))
    elev = np.radians(float(kickup_deg))
    axis = np.array([np.cos(plan) * np.cos(elev),
                     np.sin(plan) * np.cos(elev),
                     np.sin(elev)])
    mid = (hp.arm_inner_front + hp.arm_inner_rear) / 2.0
    moved = {"arm_inner_front": mid + 0.5 * length * axis,
             "arm_inner_rear": mid - 0.5 * length * axis}
    pf, pr = _carrier_pin(hp)
    pin_len = float(np.linalg.norm(pf - pr))
    if pin_len > 1e-9:
        pin_plan = float(np.arctan2((pf - pr)[1], (pf - pr)[0]))
        pin_axis = np.array([np.cos(pin_plan) * np.cos(elev),
                             np.sin(pin_plan) * np.cos(elev),
                             np.sin(elev)])
        pin_mid = 0.5 * (pf + pr)
        fa = "hinge_front" if hasattr(hp, "hinge_front") else "outer_front"
        ra = "hinge_rear" if hasattr(hp, "hinge_rear") else "outer_rear"
        moved[fa] = pin_mid + 0.5 * pin_len * pin_axis
        moved[ra] = pin_mid - 0.5 * pin_len * pin_axis
    return dataclasses.replace(hp, **moved)


# -- the caster pill (C-hub) -------------------------------------------
# On the real car the caster block is driven by an eccentric PILL where the
# lower arm's pin passes through it: turning the pill moves the pin's seat
# inside the block, which rotates the block -- and the kingpin fixed in it
# -- relative to the arm. So caster has TWO contributions, matching the RC
# convention that total caster = kick-up + caster-block angle:
#   * the chassis kickup, carried by the arm bushing axis, and
#   * the pill, a rotation of the carrier about its pin's lateral axis.
# The pill is therefore reported as caster ABOVE the kickup, which is the
# number stamped on the part.

def chub_pill_caster_deg(hp) -> float:
    """Caster contributed by the pill: the kingpin's caster minus the
    chassis kickup the arm bushing axis carries."""
    from .metrics import caster_deg
    from .suspension_types import solver_for
    st = solver_for(hp)(hp).solve(0.0)
    return float(caster_deg(st)) - arm_axis_kickup(hp)


def set_chub_pill_caster(hp, deg: float):
    """Rotate the carrier (kingpin, camber-link ball, tie-rod ball, wheel
    centre) about its pin's LATERAL axis until the pill reads `deg`.

    The arm, the pin and every chassis point stay exactly where they are —
    this is the block turning on its pin, which is what the pill does.
    Iterated because caster is only locally linear in the rotation."""
    pf, pr = _carrier_pin(hp)
    pin_mid = 0.5 * (pf + pr)
    pin = _unit(pf - pr)
    lateral = np.cross(Z_UP, pin)          # perpendicular to pin and to up
    n = float(np.linalg.norm(lateral))
    if n < 1e-9:
        raise ValueError("carrier pin is vertical; no lateral axis")
    lateral = lateral / n
    carried = [a for a in ("kingpin_upper", "kingpin_lower", "camber_outer",
                           "tierod_outer", "wheel_center", "hs_outer")
               if hasattr(hp, a)]
    def turned(h, ang):
        return dataclasses.replace(h, **{
            a: rotate_about(pin_mid, lateral, getattr(h, a), ang)
            for a in carried})

    # Which way does a positive rotation move caster? It depends on the
    # pin's sense, so PROBE it rather than assume: guessing wrong makes the
    # iteration double its error every pass instead of converging.
    probe = np.radians(0.5)
    d_dtheta = ((chub_pill_caster_deg(turned(hp, probe))
                 - chub_pill_caster_deg(hp)) / probe)
    if abs(d_dtheta) < 1e-6:
        raise ValueError("carrier rotation does not move caster here")
    hp2 = hp
    for _ in range(12):
        err = float(deg) - chub_pill_caster_deg(hp2)
        if abs(err) < 1e-9:
            break
        # d_dtheta is degrees of pill per RADIAN of rotation, so the step
        # is err/d_dtheta radians -- converting err to radians first would
        # scale every step by 1/57 and stall short of the target.
        hp2 = turned(hp2, err / d_dtheta)
    return hp2


def rotate_about(origin, axis, p, ang):
    """Rodrigues rotation of point `p` about the line (origin, unit axis)."""
    r = np.asarray(p, float) - np.asarray(origin, float)
    c, s = np.cos(ang), np.sin(ang)
    return (np.asarray(origin, float) + r * c + np.cross(axis, r) * s
            + np.asarray(axis, float) * np.dot(axis, r) * (1.0 - c))


# -- arm length, for the single-arm carrier types -----------------------
def carrier_arm_length(hp) -> float:
    """Distance from the chassis bushing-axis midpoint to the carrier
    pin's midpoint — the effective control-arm length."""
    inner = (hp.arm_inner_front + hp.arm_inner_rear) / 2.0
    pf, pr = _carrier_pin(hp)
    return float(np.linalg.norm((pf + pr) / 2.0 - inner))


def set_carrier_arm_length(hp, length: float):
    """Slide the carrier pin (both ends together) along the arm direction
    until the arm measures `length`. The pin keeps its own orientation, so
    the skew tweaks stay where they were; everything mounted on the
    carrier rides along, so camber curve and track change together — the
    swing-arm-length knob."""
    origin, u, _, _ = _carrier_arm_frame(hp)
    pf, pr = _carrier_pin(hp)
    pin_mid = (pf + pr) / 2.0
    shift = (float(length) - float(np.linalg.norm(pin_mid - origin))) * u
    if hasattr(hp, "hinge_front"):
        moved = {"hinge_front": pf + shift, "hinge_rear": pr + shift}
    else:
        moved = {"outer_front": pf + shift, "outer_rear": pr + shift}
    # Everything the carrier carries has to travel with the pin, or the
    # "arm length" change would silently redesign the upright instead.
    for attr in ("kingpin_upper", "kingpin_lower", "camber_outer",
                 "tierod_outer", "wheel_center", "hs_outer"):
        if hasattr(hp, attr):
            moved[attr] = getattr(hp, attr) + shift
    return dataclasses.replace(hp, **moved)


# -- camber-link rises, for the types closed by a camber link -----------
# C-hub and H-arm corners are closed by a single chassis->carrier link.
# Per the RC tuning literature this link is THE roll-centre and
# camber-gain control: shortening it raises the roll centre and increases
# camber gain, lengthening it does the reverse. It had no tweak at all.

def _camber_link_ref(hp):
    """(outer reference point, inner reference point) the two link ends
    are measured above."""
    if hasattr(hp, "kingpin_upper"):        # C-hub: measure off the kingpin
        outer_ref = hp.kingpin_upper
    else:                                   # H-arm: measure off the carrier
        pf, pr = _carrier_pin(hp)
        outer_ref = (pf + pr) / 2.0
    inner_ref = (hp.arm_inner_front + hp.arm_inner_rear) / 2.0
    return outer_ref, inner_ref


def camber_link_rises(hp) -> dict:
    """Heights of the camber link's two ends above their references (mm):
    the outer ball above the kingpin top (C-hub) or the carrier pin
    (H-arm), the inner ball above the chassis bushing-axis midpoint."""
    outer_ref, inner_ref = _camber_link_ref(hp)
    return {
        "cam_outer_rise": float(hp.camber_outer[2] - outer_ref[2]),
        "cam_inner_rise": float(hp.camber_inner[2] - inner_ref[2]),
    }


def set_camber_link_rises(hp, cam_outer_rise: float, cam_inner_rise: float):
    """Move the camber link's ends to the requested heights. Only z moves
    — the direct roll-centre / camber-gain motion."""
    outer_ref, inner_ref = _camber_link_ref(hp)
    co = hp.camber_outer.copy()
    ci = hp.camber_inner.copy()
    co[2] = outer_ref[2] + float(cam_outer_rise)
    ci[2] = inner_ref[2] + float(cam_inner_rise)
    return dataclasses.replace(hp, camber_outer=co, camber_inner=ci)


# -- C-hub steering: the bump-steer and Ackermann knobs -----------------
def chub_tierod_rises(hp) -> dict:
    """Tie-rod end heights on a C-hub corner: the outer ball above the
    kingpin's lower end, the inner (rack) ball above the chassis
    bushing-axis midpoint. These two are the bump-steer tuning motion —
    the same pair the double wishbone has, referenced to the C-hub's own
    kingpin instead of a lower ball joint."""
    inner_ref = (hp.arm_inner_front + hp.arm_inner_rear) / 2.0
    return {
        "outer_rise": float(hp.tierod_outer[2] - hp.kingpin_lower[2]),
        "inner_rise": float(hp.tierod_inner[2] - inner_ref[2]),
    }


def set_chub_tierod_rises(hp, outer_rise: float, inner_rise: float):
    """New C-hub hardpoints with the tie-rod ends at the requested rises.
    Only z moves."""
    inner_ref = (hp.arm_inner_front + hp.arm_inner_rear) / 2.0
    tro = hp.tierod_outer.copy()
    tri = hp.tierod_inner.copy()
    tro[2] = hp.kingpin_lower[2] + float(outer_rise)
    tri[2] = inner_ref[2] + float(inner_rise)
    return dataclasses.replace(hp, tierod_outer=tro, tierod_inner=tri)


def chub_steer_arm(hp) -> float:
    """Steering-arm length on a C-hub: perpendicular distance from the
    kingpin AXIS to the tie-rod outer ball. It sets the steering ratio and
    the Ackermann rate."""
    axis = _unit(hp.kingpin_upper - hp.kingpin_lower)
    rel = hp.tierod_outer - hp.kingpin_lower
    return float(np.linalg.norm(rel - np.dot(rel, axis) * axis))


def set_chub_steer_arm(hp, length: float):
    """Slide the tie-rod outer ball to `length` from the kingpin axis,
    along its current perpendicular direction (so the arm's fore/aft
    sweep, hence the Ackermann sense, is preserved)."""
    axis = _unit(hp.kingpin_upper - hp.kingpin_lower)
    rel = hp.tierod_outer - hp.kingpin_lower
    perp = rel - np.dot(rel, axis) * axis
    n = float(np.linalg.norm(perp))
    if n < 1e-9:
        raise ValueError("tie-rod ball sits on the kingpin axis; nudge it "
                         "off the axis before setting a steering-arm length")
    return dataclasses.replace(
        hp, tierod_outer=hp.tierod_outer + (float(length) - n) * (perp / n))


def chub_kingpin_length(hp) -> float:
    """Kingpin length on a C-hub (lower end to upper end)."""
    return float(np.linalg.norm(hp.kingpin_upper - hp.kingpin_lower))


def set_chub_kingpin_length(hp, length: float):
    """Slide the kingpin's UPPER end along the axis to set its length,
    keeping the lower end and the axis direction — so caster, KPI, scrub
    and trail are all unchanged. A longer kingpin just spreads the
    steering-block bearings further apart."""
    axis = _unit(hp.kingpin_upper - hp.kingpin_lower)
    return dataclasses.replace(
        hp, kingpin_upper=hp.kingpin_lower + float(length) * axis)


def chub_static_toe(hp) -> float:
    """Static toe of a C-hub corner (deg). On this type the tie rod does
    NOT set static toe: the solver measures the tie-rod length in the
    static pose, so the steering closure is always zero there and the toe
    comes from the stored static_toe_deg. The tie rod sets how toe CHANGES
    with travel (bump steer) — that is what the rise rows do."""
    return float(getattr(hp, "static_toe_deg", 0.0))


def set_chub_static_toe(hp, toe_deg: float):
    """Set the C-hub corner's static toe."""
    return dataclasses.replace(hp, static_toe_deg=float(toe_deg))


def _pre_square(hp, sketch_yaw):
    """Re-square to a requested sketch yaw, before anything skews off it.

    Two guards, both learned the hard way:

    * ORDER. `plan_deg` / `elev_deg` deliberately skew the carrier pin OFF
      the sketch plane, and squaring puts it back. Squaring has to happen
      FIRST so the skew is applied on top of the new plane, not erased by
      it. That is why this is not in `_apply_common`, which runs last.
    * NO-OP WHEN UNCHANGED. Callers routinely hand back the whole params
      dict with one value edited, so the yaw usually arrives equal to what
      it already is. Squaring is not free even then — it would also
      straighten any misalignment the caller never asked to touch.
    """
    if sketch_yaw is None:
        return hp
    cur = sketch_yaw_deg(hp)
    if cur is None or abs(cur - float(sketch_yaw)) <= 1e-6:
        return hp
    return set_sketch_yaw(hp, float(sketch_yaw))


def _apply_common(hp, *, shock_len=None, hub_cv=None, ride_h=None,
                  corner_x=None, corner_z=None,
                  frame_tube_offset=0.0, locked=frozenset(),
                  arm_len=None, kickup=None,
                  cam_outer_rise=None, cam_inner_rise=None,
                  chub_rises=None, chub_toe=None, chub_steer_arm=None,
                  chub_kp_len=None, chub_pill=None):
    """Apply whichever cross-type tweaks were actually requested.

    A value of None means "leave it alone". That is not laziness — the GUI
    reads every row back at DISPLAY precision, so blindly re-applying an
    unchanged row would walk the geometry by the rounding error every time
    a different row is edited. The caller passes None for every row whose
    spin has not moved beyond its display quantum, exactly as the double
    wishbone's hand-rolled path has always done.

    Order matters: shape the linkage first, then the rigid placement moves
    (ride height, corner fore/aft) last, so their targets are measured on
    the final geometry."""
    # -- linkage shape --------------------------------------------------
    if arm_len is not None:
        hp = set_carrier_arm_length(hp, arm_len)
    if kickup is not None:
        hp = set_arm_axis_kickup(hp, kickup)
    if cam_outer_rise is not None or cam_inner_rise is not None:
        cur = camber_link_rises(hp)
        hp = set_camber_link_rises(
            hp,
            cur["cam_outer_rise"] if cam_outer_rise is None else cam_outer_rise,
            cur["cam_inner_rise"] if cam_inner_rise is None else cam_inner_rise)
    if chub_kp_len is not None:
        hp = set_chub_kingpin_length(hp, chub_kp_len)
    # The pill turns the block on its pin, so it must land AFTER the arm
    # and pin have been aimed -- it is measured relative to them.
    if chub_pill is not None:
        hp = set_chub_pill_caster(hp, chub_pill)
    # Steering-arm length and the tie-rod OUTER rise both position the same
    # ball, so order decides which one is exact. Bump steer is the more
    # critical target, so the rise is applied last and wins the z.
    if chub_steer_arm is not None:
        hp = set_chub_steer_arm(hp, chub_steer_arm)
    if chub_rises is not None and any(v is not None for v in chub_rises):
        cur = chub_tierod_rises(hp)
        outer, inner = chub_rises
        hp = set_chub_tierod_rises(
            hp,
            cur["outer_rise"] if outer is None else outer,
            cur["inner_rise"] if inner is None else inner)
    if chub_toe is not None:
        hp = set_chub_static_toe(hp, chub_toe)
    # -- hub, then shock length (the mount slides, so do it after the arm)
    if hub_cv is not None:
        hp = set_hub_cv_offset(hp, hub_cv)
    if shock_len is not None:
        hp = set_installed_shock_length(hp, shock_len)
    # -- rigid placement, last -------------------------------------------
    # ride height and corner_z are the same axis expressed two ways; apply
    # ride height first so an explicit corner_z wins if both are sent.
    if ride_h is not None:
        hp = set_ride_height(hp, ride_h, frame_tube_offset, locked)
    if corner_z is not None:
        hp = set_corner_z(hp, corner_z, locked)
    if corner_x is not None:
        hp = set_corner_x(hp, corner_x, locked)
    return hp


# ----------------------------------------------------------------------
# Sketch plane yaw
#
# The 2D design sketch a corner is drawn on has a plan-view angle: zero
# for a classic front-parallel sketch, non-zero for a frame whose mount
# tubes run at an angle (Ethan's rear measures +20.1 deg). It is entered
# in the seed form, and until now the only way to change it afterwards
# was Re-square, which reads the seed form rather than the corner. That
# makes a sign error awkward to undo: you typed +20 when the tubes run
# -20, and the geometry is already built the wrong way round.
#
# This is the same minimal repair Re-square performs -- rotate each
# bushing pair's half-spread onto the common direction at the new yaw,
# keeping midpoints, spread lengths and the kickup ELEVATION -- exposed
# as a tweak so it can be dialled like any other relative dimension.
# Types without a 2D sketch (trailing arm, multilink) return None.
# ----------------------------------------------------------------------

def sketch_yaw_deg(hp) -> float | None:
    """Plan-view angle of this corner's design sketch, or None."""
    from .geometry import (DoubleWishbonePoints, carrier_planarity,
                           sketch_planarity)
    try:
        if isinstance(hp, DoubleWishbonePoints):
            return float(sketch_planarity(hp)["axis_yaw_deg"])
        return float(carrier_planarity(hp)["axis_yaw_deg"])
    except (ValueError, AttributeError, KeyError, ZeroDivisionError):
        return None


def has_sketch_plane(hp) -> bool:
    return sketch_yaw_deg(hp) is not None


def set_sketch_yaw(hp, yaw_deg: float, *, planarize_kingpin_too: bool = True):
    """Rotate this corner's axes onto a new sketch-plane yaw.

    On a double wishbone the kingpin is pulled back into the resulting
    plane afterwards, because a one-sketch corner draws the kingpin on
    that same sketch — skip it with `planarize_kingpin_too=False` when
    the axle is in independent-caster mode, exactly as Re-square does.
    """
    from .geometry import (DoubleWishbonePoints, planarize_kingpin,
                           square_bushing_axes, square_carrier_axes)
    if not has_sketch_plane(hp):
        return hp
    if isinstance(hp, DoubleWishbonePoints):
        out = square_bushing_axes(hp, yaw_deg=float(yaw_deg))
        return planarize_kingpin(out) if planarize_kingpin_too else out
    # single-arm carriers keep their caster pill through the rotation
    return square_carrier_axes(hp, yaw_deg=float(yaw_deg), keep_pill=True)
