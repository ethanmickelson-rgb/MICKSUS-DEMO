"""Kinematic metrics computed from solved corner states.

All angle conventions are for the LEFT corner (+Y side), in degrees:

  camber   negative = top of tire leans toward the vehicle centreline.
  toe      positive = toe-in (front of tire toward the centreline).
  caster   positive = top of the kingpin axis leans rearward.
  KPI      positive = top of the kingpin axis leans inboard.
  scrub radius   positive = contact patch outboard of where the kingpin
                 axis pierces the ground.
  caster trail   positive = contact patch behind where the kingpin axis
                 pierces the ground.
  roll centre height   from the classic front-view construction (below).
  motion ratio   SHOCK travel / WHEEL travel — the standard convention, so
                 the value is BELOW 1 (a Baja corner reads like "0.54") and
                 drops straight into Kw = Ks * MR^2 with no inversion.
                 (Before v1.28 this was the reciprocal, wheel/shock > 1.
                 The number was right but reading it into a textbook
                 formula without inverting gives a 3.4x error, so the
                 convention was flipped to match the literature.) Computed
                 by numeric differentiation of shock length along the sweep.
  bump steer     d(toe)/d(travel) in deg/mm, numeric derivative of the toe
                 curve. (The toe curve itself is also returned.)
"""

import numpy as np

from .solver import CornerState, DoubleWishboneSolver, Z_UP, _unit


# ----------------------------------------------------------------------
# Wheel orientation angles from the spindle axis
# ----------------------------------------------------------------------
def camber_deg(state: CornerState) -> float:
    # Front view: the spindle (outboard) tilting UP means the top of the
    # wheel leans INBOARD -> negative camber, hence the minus sign.
    s = state.spindle
    return -np.degrees(np.arctan2(s[2], s[1]))


def toe_deg(state: CornerState) -> float:
    # Top view: wheel heading = spindle × ẑ (points forward for the left
    # corner). Heading rotated toward -Y (centreline) = toe-in = positive.
    s = state.spindle
    f = _unit(np.cross(s, Z_UP))
    return -np.degrees(np.arctan2(f[1], f[0]))


# ----------------------------------------------------------------------
# Kingpin (steering axis) metrics — the axis runs LBJ -> UBJ
# ----------------------------------------------------------------------
def _kingpin(state):
    """UBJ->LBJ axis, or None for types without a steering knuckle
    (trailing arm, multilink) whose states carry no ball joints."""
    ubj = getattr(state, "ubj", None)
    lbj = getattr(state, "lbj", None)
    if ubj is None or lbj is None:
        return None
    return ubj - lbj


def caster_deg(state) -> float:
    k = _kingpin(state)  # k[2] > 0 always (UBJ above LBJ)
    if k is None:
        return float("nan")
    return np.degrees(np.arctan2(-k[0], k[2]))  # top rearward = positive


def kpi_deg(state) -> float:
    k = _kingpin(state)
    if k is None:
        return float("nan")
    return np.degrees(np.arctan2(-k[1], k[2]))  # top inboard = positive (left)


def _kingpin_ground_point(state: CornerState) -> np.ndarray:
    """Where the LBJ->UBJ line, extended, pierces the GROUND plane — the
    horizontal plane through the tire contact patch. (Not z = 0: the
    Onshape-aligned coordinate offset puts the ground at negative z.)"""
    k = state.ubj - state.lbj
    t = (state.contact_patch[2] - state.lbj[2]) / k[2]
    return state.lbj + t * k


def scrub_radius_mm(state) -> float:
    if _kingpin(state) is None:
        return float("nan")
    kp = _kingpin_ground_point(state)
    return state.contact_patch[1] - kp[1]  # outboard (+Y, left corner) = positive


def caster_trail_mm(state) -> float:
    if _kingpin(state) is None:
        return float("nan")
    kp = _kingpin_ground_point(state)
    return kp[0] - state.contact_patch[0]  # patch behind axis = positive


# ----------------------------------------------------------------------
# Roll centre — classic front-view geometric construction
# ----------------------------------------------------------------------
# The roll centre lives in the FRONT VIEW: the transverse vertical plane
# (Y-Z), looking down the length of the car — the SAE J670 / ISO 8855
# definition ("the point in the transverse vertical plane through the
# wheel centres ..."). NOT the kickup-tilted 2D manufacturing sketch
# plane. Each control arm's front-view instant-centre line is found, the
# two lines meet at the front-view IC, and the roll centre is where the
# line from the tire contact patch through the IC crosses the centreline.
#
# Constructing one arm's front-view line correctly (this matters once the
# arms are inclined fore/aft — kickup or anti-geometry): an A-arm's ball
# joint rotates about the inboard pivot axis, so its velocity is EXACTLY
# parallel to the arm-plane normal  n = (inner_r-inner_f) x (outer-inner_f)
# (v = omega * n, since d x (BJ-axis_pt) = d x (BJ-inner_f)). The
# front-view IC line is therefore the line through the ball joint's OWN
# (y,z) projection with normal (n_y, n_z):
#     n_y*y + n_z*z = n_y*BJ_y + n_z*BJ_z
# i.e. the arm plane sliced at the BALL JOINT's own x-station. Slicing
# both arms at the shared wheel-centre plane instead offsets each line by
# n_x*(BJ_x - x_wc); that is zero only for transverse (zero-kickup) arms
# and mislocates the IC by tens of mm at large kickup / anti (verified
# against a rigid-body finite-difference IC — they now agree exactly).


def _front_view_arm_line(inner_f, inner_r, outer):
    """Front-view (Y-Z) line of a control arm, as a 2D line a*y + b*z = c:
    through the ball joint's (y,z) projection, normal (n_y, n_z). See the
    section comment for why it slices at the ball joint's own station."""
    n = np.cross(inner_r - inner_f, outer - inner_f)  # arm plane normal
    return n[1], n[2], n[1] * outer[1] + n[2] * outer[2]


def _numeric_view_ic(solver, state, axes: tuple, h: float = 1.0):
    """Numeric instant centre in a projection plane, valid for ANY linkage
    (this is what Adams does): finite-difference the velocities of two
    upright points through travel, then intersect the perpendiculars to
    those velocities — the IC is where both perpendiculars meet. `axes`
    picks the projection: (1, 2) = front view (y, z), (0, 2) = side view
    (x, z). Returns None when the motion is a pure translation in that
    view (IC at infinity)."""
    steer = getattr(state, "steer", 0.0)
    try:
        lo = solver.solve(state.travel - h, steer)
        hi = solver.solve(state.travel + h, steer)
    except (ValueError, TypeError):
        return None
    i, j = axes
    lines = []
    for attr in ("wheel_center", "contact_patch"):
        p = np.array([getattr(state, attr)[i], getattr(state, attr)[j]])
        v = np.array([getattr(hi, attr)[i] - getattr(lo, attr)[i],
                      getattr(hi, attr)[j] - getattr(lo, attr)[j]])
        if np.linalg.norm(v) < 1e-12:
            return None
        lines.append((p, np.array([-v[1], v[0]])))   # perpendicular to v
    (p1, n1), (p2, n2) = lines
    m = np.column_stack([n1, -n2])
    if abs(np.linalg.det(m)) < 1e-9 * np.linalg.norm(n1) * np.linalg.norm(n2):
        return None                                   # parallel: IC at infinity
    s = np.linalg.solve(m, p2 - p1)
    ic = p1 + s[0] * n1
    return float(ic[0]), float(ic[1])


def front_view_ic(solver, state):
    """Front-view instant centre (y, z), or None if it is at infinity.

    Dispatch: a solver may provide its own exact construction (the
    trailing arm's IC is where its pivot axis pierces the front-view
    plane); the double wishbone uses the classic arm-plane construction
    below; anything else (multilink) falls back to the numeric method."""
    if hasattr(solver, "exact_front_view_ic"):
        return solver.exact_front_view_ic(state)
    if not hasattr(solver.hp, "uca_inner_front"):
        return _numeric_view_ic(solver, state, (1, 2))
    hp = solver.hp
    a1, b1, c1 = _front_view_arm_line(hp.uca_inner_front, hp.uca_inner_rear, state.ubj)
    a2, b2, c2 = _front_view_arm_line(hp.lca_inner_front, hp.lca_inner_rear, state.lbj)
    m = np.array([[a1, b1], [a2, b2]])
    det = np.linalg.det(m)
    # Normalise the parallel test by the line magnitudes so it's unit-safe.
    if abs(det) < 1e-9 * np.linalg.norm(m[0]) * np.linalg.norm(m[1]):
        return None
    y, z = np.linalg.solve(m, np.array([c1, c2]))
    return float(y), float(z)


def _side_view_arm_line(inner_f, inner_r, outer):
    """Side-view (X-Z) line of a control arm, as a 2D line a*x + b*z = c:
    through the ball joint's (x,z) projection, normal (n_x, n_z) — the arm
    plane sliced at the BALL JOINT's own y-station (same reasoning as the
    front-view line; slicing at the shared wheel-centre y mislocates the
    side-view IC once the arms are skewed for anti-geometry)."""
    n = np.cross(inner_r - inner_f, outer - inner_f)
    return n[0], n[2], n[0] * outer[0] + n[2] * outer[2]


def side_view_ic(solver, state):
    """Side-view instant centre (x, z) — the same construction as the
    front-view IC but sliced in the vertical-longitudinal plane through
    the wheel centre. It governs wheel recession and the anti-dive /
    anti-squat geometry.

    Returns None when the two arm-plane lines are parallel — which is the
    DESIGNED state for this tool's seeds: both bushing axes are normal to
    the 2D sketch plane, so their side-view traces are parallel and the
    IC sits at infinity (straight-line wheel path in side view). It
    becomes finite when the user skews the arm axes for anti geometry.

    Same dispatch as front_view_ic: exact per-solver construction if the
    solver provides one, arm planes for double wishbone, numeric fallback
    otherwise."""
    if hasattr(solver, "exact_side_view_ic"):
        return solver.exact_side_view_ic(state)
    if not hasattr(solver.hp, "uca_inner_front"):
        return _numeric_view_ic(solver, state, (0, 2))
    hp = solver.hp
    a1, b1, c1 = _side_view_arm_line(hp.uca_inner_front, hp.uca_inner_rear,
                                     state.ubj)
    a2, b2, c2 = _side_view_arm_line(hp.lca_inner_front, hp.lca_inner_rear,
                                     state.lbj)
    m = np.array([[a1, b1], [a2, b2]])
    det = np.linalg.det(m)
    if abs(det) < 1e-9 * np.linalg.norm(m[0]) * np.linalg.norm(m[1]):
        return None
    x, z = np.linalg.solve(m, np.array([c1, c2]))
    return float(x), float(z)


def roll_center_height_mm(solver, state) -> float:
    """Roll-centre height ABOVE THE GROUND PLANE (the plane through the
    tire contact patch — not z = 0, which the Onshape-aligned coordinate
    offset puts below the car). Absolute-z callers (the 3D marker) add
    state.contact_patch[2] back."""
    cp = state.contact_patch
    ic = front_view_ic(solver, state)
    if ic is None:
        # IC at infinity: the contact-patch->IC "line" runs along the
        # direction toward that infinite IC — for a double wishbone the
        # arms' common front-view direction, generically the perpendicular
        # of the patch's front-view velocity. Walk it to the centreline.
        if hasattr(solver.hp, "uca_inner_front"):
            a1, b1, _ = _front_view_arm_line(
                solver.hp.uca_inner_front, solver.hp.uca_inner_rear,
                state.ubj)
            dy, dz = -b1, a1
        else:
            steer = getattr(state, "steer", 0.0)
            try:
                lo = solver.solve(state.travel - 1.0, steer)
                hi = solver.solve(state.travel + 1.0, steer)
            except (ValueError, TypeError):
                return float("nan")
            v = hi.contact_patch - lo.contact_patch
            dy, dz = -v[2], v[1]     # perpendicular to (v_y, v_z)
        if abs(dy) < 1e-12:
            return float("nan")  # motion horizontal in front view: undefined
        return (0.0 - cp[1]) * (dz / dy)
    y_ic, z_ic = ic
    if abs(y_ic - cp[1]) < 1e-9:
        return float("nan")  # IC directly above/below the patch: undefined
    slope = (z_ic - cp[2]) / (y_ic - cp[1])
    return (0.0 - cp[1]) * slope


# ----------------------------------------------------------------------
# Single-position summary and full sweep
# ----------------------------------------------------------------------
def corner_metrics(solver, state) -> dict:
    """All position-level metrics at one travel position."""
    # Types without a physical kingpin can still have a VIRTUAL steering
    # axis (multilink): compute it once and cache two points of it on the
    # state, after which every kingpin metric works unchanged.
    if (getattr(state, "ubj", None) is None
            and hasattr(solver, "virtual_kingpin")
            and not getattr(state, "_vk_done", False)):
        state._vk_done = True
        vk = solver.virtual_kingpin(state)
        if vk is not None:
            p, d = vk
            state.lbj = p - 100.0 * d
            state.ubj = p + 100.0 * d
    return {
        "travel_mm": state.travel,
        "camber_deg": camber_deg(state),
        "toe_deg": toe_deg(state),
        "caster_deg": caster_deg(state),
        "kpi_deg": kpi_deg(state),
        "scrub_radius_mm": scrub_radius_mm(state),
        "caster_trail_mm": caster_trail_mm(state),
        "roll_center_height_mm": roll_center_height_mm(solver, state),
        "shock_length_mm": state.shock_length,
        # raw positions used for the change-vs-static channels below
        "contact_patch_y_mm": float(state.contact_patch[1]),
        "wheel_center_x_mm": float(state.wheel_center[0]),
    }


def curves_from_states(solver: DoubleWishboneSolver, states, travels) -> dict:
    """Metric curves (numpy arrays) for already-solved states, plus the
    derivative-based curves (motion ratio, bump steer) via np.gradient."""
    travels = np.asarray(travels, dtype=float)
    rows = [corner_metrics(solver, s) for s in states]
    out = {key: np.array([r[key] for r in rows]) for key in rows[0]}
    # Motion ratio = SHOCK travel / WHEEL travel — the standard convention,
    # so this is BELOW 1 for a shock mounted inboard on the arm and feeds
    # Kw = Ks*MR^2 directly. Shock length DECREASES in bump, hence the minus
    # sign that makes it positive.
    # edge_order=2 keeps the two ENDPOINT samples second-order accurate
    # (numpy's default drops to first-order one-sided there), so the
    # motion-ratio / bump-steer curves are trustworthy at full bump/droop,
    # not just mid-travel.
    out["motion_ratio"] = -np.gradient(out["shock_length_mm"],
                                       travels, edge_order=2)
    out["bump_steer_deg_per_mm"] = np.gradient(out["toe_deg"], travels,
                                               edge_order=2)
    # Change-vs-static channels (industry-standard "half-track change" and
    # "wheel recession"): how far the contact patch scrubs sideways and the
    # wheel centre walks fore/aft through travel. Big travel = big scrub, so
    # these matter on a Baja car (tire drag, CV plunge). The datum is TRUE
    # static (travel = 0), solved exactly — NOT the nearest grid sample,
    # which on the app's asymmetric -droop..+bump sweeps sits a few mm off
    # zero and shifted the whole curve by ~1 mm.
    try:
        m0 = corner_metrics(solver, solver.solve(0.0))
        cp_y0 = m0["contact_patch_y_mm"]
        wc_x0 = m0["wheel_center_x_mm"]
    except (ValueError, TypeError):
        cp_y0 = float(np.interp(0.0, travels, out["contact_patch_y_mm"]))
        wc_x0 = float(np.interp(0.0, travels, out["wheel_center_x_mm"]))
    out["half_track_change_mm"] = out["contact_patch_y_mm"] - cp_y0
    # positive = wheel moves REARWARD (favourable recession absorbs bumps)
    out["wheel_recession_mm"] = wc_x0 - out["wheel_center_x_mm"]
    return out


def sweep_metrics(solver: DoubleWishboneSolver, travels) -> dict:
    """Solve the corner at each travel value and return metric curves as
    numpy arrays (see curves_from_states)."""
    travels = np.asarray(travels, dtype=float)
    states = solver.walk_travels(travels)
    out = curves_from_states(solver, states, travels)
    out["wheel_center"] = np.vstack([s.wheel_center for s in states])
    out["states"] = states
    return out
