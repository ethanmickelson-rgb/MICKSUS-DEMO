"""Hardpoint definitions for a double-wishbone corner.

All points are 3D numpy arrays in vehicle coordinates (see package
docstring): +X forward, +Y left, +Z up, origin at ground level on the
vehicle centreline at this axle's X-station. Units: mm. Left corner (y > 0).
"""

from dataclasses import dataclass, field

import numpy as np


def _pt(x: float, y: float, z: float) -> np.ndarray:
    """Convenience: build a 3D point as a float numpy array."""
    return np.array([x, y, z], dtype=float)


@dataclass
class DoubleWishbonePoints:
    """Full hardpoint set for one double-wishbone corner.

    "Inner" points are on the chassis (they never move).
    "Outer" points live on the moving suspension (control arms / upright).
    """

    # Upper control arm: two chassis pivots define its rotation axis,
    # the outer point is the upper ball joint (UBJ) on the upright.
    uca_inner_front: np.ndarray
    uca_inner_rear: np.ndarray
    uca_outer: np.ndarray

    # Lower control arm: same layout, outer point is the lower ball joint (LBJ).
    lca_inner_front: np.ndarray
    lca_inner_rear: np.ndarray
    lca_outer: np.ndarray

    # Tie rod: inner end on the chassis/rack (fixed for now, steer comes
    # later), outer end on the upright's steering arm.
    tierod_inner: np.ndarray
    tierod_outer: np.ndarray

    # Wheel centre (centre of the wheel/hub) — rigidly part of the upright.
    wheel_center: np.ndarray

    # Shock: inner end on the chassis, outer end mounted on a control arm
    # (LOWER by default — typical Baja; set shock_on_uca for upper-arm
    # mounting). The outer end rotates with that arm, not with the upright.
    shock_inner: np.ndarray
    shock_outer: np.ndarray

    # Tire outer radius (mm). Used to place the contact patch.
    tire_radius: float

    # Tire width (mm). Display only (3D wheel size); no kinematic effect.
    tire_width: float = 150.0

    # Static wheel orientation. Hardpoints alone don't define the spindle
    # (wheel rotation axis) direction, so the user states the alignment at
    # ride height. Conventions: camber negative = top of tire leans toward
    # the vehicle centreline; toe positive = toe-in (front of tire toward
    # the centreline).
    static_camber_deg: float = 0.0
    static_toe_deg: float = 0.0

    # Which arm carries the shock's outer end: False = lower (default),
    # True = upper. The solver rotates the outer mount with this arm.
    shock_on_uca: bool = False

    # Where the tie rod's INNER end mounts. Default = chassis (front rack /
    # fixed rear toe link). Setting one of these moves the INNER BALL JOINT
    # onto a control arm: its centre then rides that arm's rotation through
    # travel (the tie rod still pivots freely at both ball joints). This
    # does NOT fix toe — an arm-mounted link generally gives large bump
    # steer, minimized by dropping the OUTBOARD link low on the kingpin
    # (lower-arm mount) or high (upper-arm mount) so the tie-rod IC stays
    # near the arm's IC. At most one is True.
    tierod_on_lca: bool = False   # inner rides the LOWER arm
    tierod_on_uca: bool = False   # inner rides the UPPER arm

    POINT_ATTRS = ["uca_inner_front", "uca_inner_rear", "uca_outer",
                   "lca_inner_front", "lca_inner_rear", "lca_outer",
                   "tierod_inner", "tierod_outer", "wheel_center",
                   "shock_inner", "shock_outer"]


def static_spindle(hp) -> np.ndarray:
    """Static wheel-spin axis (left corner, unit) from the user's static
    camber/toe — the same construction the solver uses: start pure
    outboard (+Y), tilt for camber about X, steer for toe about Z."""
    gamma = np.radians(getattr(hp, "static_camber_deg", 0.0))
    tau = np.radians(getattr(hp, "static_toe_deg", 0.0))
    # Build the spindle so BOTH the front-view camber and the top-view toe
    # read back EXACTLY (metrics.camber_deg / toe_deg) for every input.
    # camber_deg = -atan2(s_z, s_y) = gamma  => s_z = -tan(gamma)*s_y;
    # toe_deg   =  atan2(s_x, s_y) = tau     => s_x =  tan(tau)*s_y.
    # (The old Rz(-tau)Rx(-gamma) form only reproduced toe exactly and
    # coupled a gamma*tau^2/2 error into camber; the two agree when toe=0.)
    s = np.array([np.tan(tau), 1.0, -np.tan(gamma)])
    return s / np.linalg.norm(s)


def sketch_planarity(hp: DoubleWishbonePoints) -> dict:
    """Can this corner still be designed on ONE 2D sketch with both arms'
    bushing tubes normal to it — the team's CAD workflow?

    A shared sketch plane exists if and only if the UCA and LCA bushing
    axes are PARALLEL (the plane's normal is that common axis direction).
    Everything else is free: the ball joints may sit fore/aft of the
    sketch (that offset IS the designed caster) because each arm's ball
    joint sweeps a circle parallel to the sketch plane regardless.

    Returns:
      axis_misalign_deg — angle between the two bushing axes. ~0 = one
          sketch works; anything else means the two arms demand two
          DIFFERENT sketch planes and the corner can't be built with all
          four inboard connections as sketch-normal bushings.
      sketch_kickup_deg — the kickup (elevation) angle the (mean) sketch
          plane implies, measured yaw-invariantly: atan2 of the axis's
          vertical component vs its horizontal magnitude.
      axis_yaw_deg — SIGNED plan-view twist of the (mean) bushing axis
          out of the side-view (X-Z) plane (+ = the tubes' front ends
          tip toward the car centreline on the left corner). The corner
          is square when this matches the axle's DESIGN sketch yaw —
          zero for a classic front-parallel sketch, non-zero for frames
          whose suspension-mount tubes run at an angle (v1.9 twisted
          sketch plane). Deviation from the design angle twists the
          arms/knuckle visibly in side view.
      kingpin_off_plane_deg — how far the kingpin (LBJ->UBJ) tilts OUT of
          the 2D sketch plane. For a true one-sketch corner the kingpin
          is drawn ON the sketch, so this must be ~0; any value is a
          knuckle twist you can see in side view. Repaired by
          planarize_kingpin (which re-square runs).
    """
    au = hp.uca_inner_front - hp.uca_inner_rear
    al = hp.lca_inner_front - hp.lca_inner_rear
    au = au / np.linalg.norm(au)
    al = al / np.linalg.norm(al)
    if np.dot(au, al) < 0.0:
        al = -al
    # atan2(|cross|, dot) stays accurate for tiny angles (arccos loses
    # ~7 digits near parallel).
    ang = float(np.degrees(np.arctan2(np.linalg.norm(np.cross(au, al)),
                                      np.dot(au, al))))
    n = au + al
    n = n / np.linalg.norm(n)
    if n[0] < 0.0:        # orient fore-aft positive so the yaw sign is
        n = -n            # well-defined
    kick = float(np.degrees(np.arctan2(n[2], np.hypot(n[0], n[1]))))
    yaw = float(np.degrees(np.arctan2(n[1], n[0])))
    kp = hp.uca_outer - hp.lca_outer
    kp = kp / np.linalg.norm(kp)
    off = float(np.degrees(np.arcsin(np.clip(abs(np.dot(kp, n)), 0.0, 1.0))))
    return {"axis_misalign_deg": ang, "sketch_kickup_deg": kick,
            "axis_yaw_deg": yaw, "kingpin_off_plane_deg": off}


def planarize_kingpin(hp: DoubleWishbonePoints) -> DoubleWishbonePoints:
    """Bring the kingpin (LBJ->UBJ) INTO the 2D sketch plane so a true
    one-sketch corner can be drawn — the fix for the knuckle twist you
    otherwise see in side view. The lower ball joint and kingpin length
    are kept; the UPPER ball joint swings until the kingpin is
    perpendicular to the bushing axis (the sketch normal). Camber/toe
    (the stored spindle) are untouched; KPI/scrub/caster follow the new
    kingpin. Run this AFTER square_bushing_axes, so the sketch normal is
    already at the design yaw."""
    import dataclasses
    au = hp.uca_inner_front - hp.uca_inner_rear
    al = hp.lca_inner_front - hp.lca_inner_rear
    au = au / np.linalg.norm(au)
    al = al / np.linalg.norm(al)
    if np.dot(au, al) < 0.0:
        al = -al
    n = au + al
    n = n / np.linalg.norm(n)               # sketch normal = bushing axis
    a = hp.lca_outer                        # keep the lower ball joint
    kp = hp.uca_outer - a
    length = np.linalg.norm(kp)
    planar = kp - np.dot(kp, n) * n         # project onto the sketch plane
    pl = np.linalg.norm(planar)
    if pl < 1e-9:                           # kingpin ~parallel to n: bail
        return hp
    return dataclasses.replace(hp, uca_outer=a + length * planar / pl)


def enforce_knuckle_planes(hp: DoubleWishbonePoints,
                           steering_arm_plane_deg: float = 90.0
                           ) -> DoubleWishbonePoints:
    """Ethan's knuckle-consistency convention: the tire's spin (centre)
    axis must lie IN the plane of the two ball joints and the wheel centre
    (so the CV output line is coincident with the tire centreline, not
    cocked to it), and the steering-arm plane (ball joints + tie-rod
    outer) sits at a set angle to that tire plane — both measured about
    the kingpin. Enforced automatically so the knuckle is always buildable.

    Constraint 1 slides the WHEEL CENTRE the minimum distance onto the
    tire plane. Because the move is (almost) pure fore/aft, track, scrub,
    camber and toe are preserved. Constraint 2 swings TIEROD_OUTER about
    the kingpin to the target steering-arm-plane angle, keeping its
    kingpin-height and its steering-arm length; the tie-rod inner stays
    put and the link length re-derives, so static toe is unchanged (only
    the steering ratio / Ackermann move — same rule as the steering-arm
    tweak)."""
    import dataclasses
    lbj = np.asarray(hp.lca_outer, float)
    ubj = np.asarray(hp.uca_outer, float)
    king = ubj - lbj
    kn = np.linalg.norm(king)
    if kn < 1e-9:
        return hp
    king = king / kn
    s = static_spindle(hp)
    s = s / np.linalg.norm(s)

    # --- Constraint 1: wheel centre onto the tire plane ---------------
    # The spin axis lies in plane(UBJ, LBJ, WC) iff WC has no component
    # along m = spin_axis x kingpin; m is independent of WC, so this is a
    # single exact projection.
    wc = np.asarray(hp.wheel_center, float)
    m = np.cross(s, king)
    mn = np.linalg.norm(m)
    if mn > 1e-9:
        m = m / mn
        # Reach the plane by moving the wheel centre FORE/AFT only (x), so
        # track (y) and ride height (z) are preserved exactly — the spindle
        # is near-lateral, so m is near-x and this is a small nudge. If the
        # plane is nearly fore/aft (ill-conditioned), take the minimal move.
        if abs(m[0]) > 0.2:
            wc = wc + np.array([-np.dot(wc - lbj, m) / m[0], 0.0, 0.0])
        else:
            wc = wc - np.dot(wc - lbj, m) * m

    # --- Constraint 2: steering-arm plane at theta from the tire plane -
    tro = np.asarray(hp.tierod_outer, float)
    rel = (wc - lbj) - np.dot(wc - lbj, king) * king   # WC dir, perp to KP
    rn = np.linalg.norm(rel)
    foot = lbj + np.dot(tro - lbj, king) * king         # arm root on the KP
    perp = tro - foot
    r = np.linalg.norm(perp)
    if rn < 1e-9 or r < 1e-9:
        return dataclasses.replace(hp, wheel_center=wc)
    e1 = rel / rn                       # in-plane, toward the wheel centre
    e2 = np.cross(king, e1)             # in-plane, perpendicular to e1
    theta = np.radians(steering_arm_plane_deg)
    side = 1.0 if np.dot(perp, e2) >= 0.0 else -1.0     # keep the arm's side
    new_dir = np.cos(theta) * e1 + side * np.sin(theta) * e2
    tro_new = foot + r * new_dir
    return dataclasses.replace(hp, wheel_center=wc, tierod_outer=tro_new)


def square_bushing_axes(hp: DoubleWishbonePoints,
                        yaw_deg: float = 0.0) -> DoubleWishbonePoints:
    """Repair a corner whose bushing axes have drifted apart or twisted:
    rotate each pair's half-spread onto a common direction at the DESIGN
    sketch yaw (`yaw_deg`, 0 = classic front-parallel sketch), keeping
    every pair's midpoint and spread length and the kickup elevation the
    current geometry implies. This is the smallest edit that restores a
    single buildable sketch plane at the intended twist. Ball joints are
    never touched (whatever caster the designer dragged in stays). The
    optimizer applies this automatically on entry so hand-edited geometry
    comes out manufacturable."""
    import dataclasses
    au = hp.uca_inner_front - hp.uca_inner_rear
    al = hp.lca_inner_front - hp.lca_inner_rear
    nu, nl = np.linalg.norm(au), np.linalg.norm(al)
    au, al = au / nu, al / nl
    if np.dot(au, al) < 0.0:
        al = -al
    n = au + al
    n = n / np.linalg.norm(n)
    if n[0] < 0.0:
        n = -n
    # Rebuild the axis at the design yaw, keeping the measured kickup
    # elevation: n = (cos e cos y, cos e sin y, sin e).
    e = np.arctan2(n[2], np.hypot(n[0], n[1]))
    y = np.radians(yaw_deg)
    n = np.array([np.cos(e) * np.cos(y), np.cos(e) * np.sin(y),
                  np.sin(e)])
    umid = (hp.uca_inner_front + hp.uca_inner_rear) / 2.0
    lmid = (hp.lca_inner_front + hp.lca_inner_rear) / 2.0
    return dataclasses.replace(
        hp,
        uca_inner_front=umid + 0.5 * nu * n,
        uca_inner_rear=umid - 0.5 * nu * n,
        lca_inner_front=lmid + 0.5 * nl * n,
        lca_inner_rear=lmid - 0.5 * nl * n,
    )


def _carrier_pin_attrs(hp):
    """(front_attr, rear_attr) of the carrier pin on a single-arm type:
    the hinge pin (C-hub, loaded halfshaft) or the H-arm's grab line."""
    if hasattr(hp, "hinge_front"):
        return "hinge_front", "hinge_rear"
    if hasattr(hp, "outer_front"):
        return "outer_front", "outer_rear"
    return None, None


def carrier_planarity(hp) -> dict:
    """Squareness of a single-arm carrier corner (C-hub, loaded halfshaft,
    H-arm) — the analogue of sketch_planarity for a double wishbone.

    These types have ONE chassis bushing axis and one carrier pin, and both
    are meant to be normal to the same 2D design sketch, i.e. parallel. The
    numbers:

      axis_misalign_deg — angle between the bushing axis and the carrier
                          pin (0 = square)
      axis_yaw_deg      — the bushing axis's twist about vertical
      sketch_kickup_deg — its elevation (the chassis kickup)
    """
    fa, ra = _carrier_pin_attrs(hp)
    if fa is None:
        raise ValueError("not a single-arm carrier type")
    a = np.asarray(hp.arm_inner_front, float) - np.asarray(hp.arm_inner_rear,
                                                           float)
    p = np.asarray(getattr(hp, fa), float) - np.asarray(getattr(hp, ra),
                                                        float)
    a = a / np.linalg.norm(a)
    p = p / np.linalg.norm(p)
    if np.dot(a, p) < 0.0:
        p = -p
    # arccos(dot) loses precision exactly where it matters here: at a
    # near-zero angle dot is 1 - O(eps) and the result is O(sqrt(eps)), so a
    # perfectly square corner reads ~1e-6 deg instead of 0. The half-angle
    # form 2*atan2(|a-p|, |a+p|) stays accurate all the way down.
    return {
        "axis_misalign_deg": float(np.degrees(2.0 * np.arctan2(
            np.linalg.norm(a - p), np.linalg.norm(a + p)))),
        "axis_yaw_deg": float(np.degrees(np.arctan2(a[1], a[0]))),
        "sketch_kickup_deg": float(np.degrees(
            np.arctan2(a[2], np.hypot(a[0], a[1])))),
    }


def square_carrier_axes(hp, yaw_deg: float = 0.0,
                        keep_pill: bool = True):
    """Re-square a single-arm carrier corner onto ONE 2D sketch.

    The chassis bushing axis is rebuilt at the design `yaw_deg`, keeping the
    kickup elevation the geometry already implies, and the carrier pin is
    made PARALLEL to it — the square state. Everything the carrier holds
    (kingpin, camber-link ball, tie-rod ball, wheel centre, outer CV) is
    carried through the SAME rotation the pin undergoes, so the carrier's
    pose relative to its pin is untouched: a caster pill dialled in at the
    block survives the repair.

    Midpoints and lengths are preserved, so this is the smallest edit that
    restores a buildable sketch. The double wishbone's equivalent is
    square_bushing_axes.
    """
    import dataclasses
    fa, ra = _carrier_pin_attrs(hp)
    if fa is None:
        raise ValueError("not a single-arm carrier type")
    arm_f = np.asarray(hp.arm_inner_front, float)
    arm_r = np.asarray(hp.arm_inner_rear, float)
    pin_f = np.asarray(getattr(hp, fa), float)
    pin_r = np.asarray(getattr(hp, ra), float)

    a = arm_f - arm_r
    la = float(np.linalg.norm(a))
    p = pin_f - pin_r
    lp = float(np.linalg.norm(p))
    a_hat = a / la
    # Target direction: the design yaw at the CURRENT kickup elevation.
    e = np.arctan2(a_hat[2], np.hypot(a_hat[0], a_hat[1]))
    y = np.radians(float(yaw_deg))
    n = np.array([np.cos(e) * np.cos(y), np.cos(e) * np.sin(y), np.sin(e)])

    p_hat = p / lp
    if np.dot(p_hat, n) < 0.0:
        p_hat = -p_hat
    # The rotation that takes the pin onto n; the carrier rides along.
    rot = _rotation_between(p_hat, n)
    pin_mid = 0.5 * (pin_f + pin_r)
    arm_mid = 0.5 * (arm_f + arm_r)

    moved = {
        "arm_inner_front": arm_mid + 0.5 * la * n,
        "arm_inner_rear": arm_mid - 0.5 * la * n,
        fa: pin_mid + 0.5 * lp * n,
        ra: pin_mid - 0.5 * lp * n,
    }
    for attr in ("kingpin_upper", "kingpin_lower", "camber_outer",
                 "tierod_outer", "wheel_center", "hs_outer"):
        if hasattr(hp, attr):
            v = np.asarray(getattr(hp, attr), float) - pin_mid
            moved[attr] = pin_mid + rot @ v
    out = dataclasses.replace(hp, **moved)
    if not keep_pill or not hasattr(hp, "kingpin_upper"):
        return out
    # Carrying the block through the pin's rotation is close but not exact:
    # that rotation has a fore/aft component, so caster drifts ~1 deg on a
    # badly skewed corner. Re-assert the pill so a dialled-in caster setting
    # really does survive the repair.
    from .tweaks import chub_pill_caster_deg, set_chub_pill_caster
    try:
        return set_chub_pill_caster(out, chub_pill_caster_deg(hp))
    except (ValueError, ZeroDivisionError):
        return out


def _rotation_between(u, v) -> np.ndarray:
    """Smallest rotation matrix taking unit vector u onto unit vector v."""
    u = np.asarray(u, float)
    v = np.asarray(v, float)
    c = float(np.clip(np.dot(u, v), -1.0, 1.0))
    if c > 1.0 - 1e-15:
        return np.eye(3)
    if c < -1.0 + 1e-12:                    # antiparallel: any perpendicular
        axis = np.cross(u, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-9:
            axis = np.cross(u, np.array([0.0, 1.0, 0.0]))
        axis = axis / np.linalg.norm(axis)
        ang = np.pi
    else:
        axis = np.cross(u, v)
        ang = np.arctan2(float(np.linalg.norm(axis)), c)
        axis = axis / np.linalg.norm(axis)
    k = np.array([[0.0, -axis[2], axis[1]],
                  [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    return (np.eye(3) + np.sin(ang) * k + (1.0 - np.cos(ang)) * (k @ k))


def example_baja_front() -> DoubleWishbonePoints:
    """A plausible Baja SAE front-left corner, for demos and tests.

    Roughly: 1300 mm track, 23 in (584 mm OD) tire, unequal-length
    non-parallel arms giving some negative camber gain in bump.
    """
    return DoubleWishbonePoints(
        uca_inner_front=_pt(140.0, 260.0, 380.0),
        uca_inner_rear=_pt(-140.0, 260.0, 380.0),
        uca_outer=_pt(-15.0, 575.0, 400.0),
        lca_inner_front=_pt(160.0, 230.0, 210.0),
        lca_inner_rear=_pt(-160.0, 230.0, 210.0),
        lca_outer=_pt(0.0, 600.0, 180.0),
        # Tie-rod inner height chosen so static bump steer is ~zero
        # (a designed-in property, like a real car would want).
        tierod_inner=_pt(-140.0, 270.0, 269.0),
        tierod_outer=_pt(-110.0, 585.0, 260.0),
        wheel_center=_pt(0.0, 650.0, 292.0),
        shock_inner=_pt(0.0, 280.0, 560.0),
        shock_outer=_pt(0.0, 470.0, 215.0),
        tire_radius=292.0,
    )
