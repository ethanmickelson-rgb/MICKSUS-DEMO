"""Phase 2: parametric seed generator.

Turns high-level SETUP VARIABLES (track width, ride height, tire size,
shock lengths, motion-ratio goal, ...) into a sensible, *valid* starting
set of double-wishbone hardpoints. This is HEURISTIC placement — rules of
thumb that produce a geometry the Phase 1 solver can articulate through
the full shock travel — not optimization. Phase 4 refines it.

Two small 1D secant tunes are the only "solving" done here, because they
directly place hardware the setup variables specify:
  * the shock's arm-mount position is slid along the LCA until the static
    motion ratio matches the goal (the brief: "shock length/travel +
    motion-ratio goal place the shock mounts"), and
  * the tie-rod inner height is nudged until static bump steer is ~zero
    (any sane seed should start there).

Units: mm and degrees throughout, same coordinate convention as Phase 1
(+X forward, +Y left, +Z up, origin on the ground at the centreline,
left corner modelled). MOTION RATIO = SHOCK travel / WHEEL travel — the
standard convention, so it reads BELOW 1 (~0.54 for a Baja corner) and
drops straight into Kw = Ks * MR^2. Flipped from the old wheel/shock form
in v1.28; see `project.py` for the automatic migration of saved files.
"""

from dataclasses import dataclass

import numpy as np

from .geometry import DoubleWishbonePoints, _pt
from .metrics import caster_deg, toe_deg
from .solver import DoubleWishboneSolver

IN = 25.4  # handy inch -> mm factor for callers who think in inches


def sketch_plane_normal(kickup_deg: float,
                        yaw_deg: float = 0.0) -> np.ndarray:
    """Unit normal of the 2D design sketch plane, in vehicle coordinates.

    The team's front-suspension sketch is a projection taken at the chassis
    kickup angle, so the plane it lives on is tilted by that angle about the
    lateral (Y) axis. Its normal — the direction the inboard bushing axes
    must point so each arm's front->rear line is NORMAL to the sketch — is
    therefore the fore-aft axis tilted up toward the front of the car:
    (cos k, 0, sin k). Spreading the four bushings along this normal (and
    leaving the ball joints, knuckle and shock on the sketch) is what makes
    the suspension follow the 2D path without inflating knuckle caster.

    yaw_deg TWISTS the sketch plane about the vertical axis (+ = the
    bushing tubes' front ends tip toward the car centreline on the left
    corner) — for frames whose suspension-mount tubes run at an angle,
    e.g. a rear whose rails converge. The knuckle/wheel still point
    straight ahead; the corner simply pivots in the twisted plane (so the
    wheelbase breathes a little through travel — expected and fine).
    """
    k = np.radians(kickup_deg)
    y = np.radians(yaw_deg)
    return np.array([np.cos(k) * np.cos(y), np.cos(k) * np.sin(y),
                     np.sin(k)])


@dataclass
class SetupVariables:
    """High-level inputs the user actually knows at the start of a design.

    Required values first; everything with a default is a packaging knob
    the user MAY set, otherwise a rule of thumb fills it in.
    """

    track_width: float          # mm, measured at ride height (this is also
                                # treated as the max-track packaging limit)
    wheelbase: float            # mm — not used for a single corner yet,
                                # carried for later phases
    ride_height: float          # mm, frame-bottom ground clearance
    tire_radius: float          # mm
    tire_width: float           # mm
    shock_min_length: float     # mm, eye-to-eye fully compressed
    shock_max_length: float     # mm, eye-to-eye fully extended
    motion_ratio_goal: float    # SHOCK travel / wheel travel (standard
                                # convention, < 1) e.g. 0.54. Feeds
                                # Kw = Ks*MR^2 with no inversion.

    shock_length_at_ride: float | None = None  # default: min + 0.7 * stroke
                                # (~70% of travel reserved for bump)
    hub_offset: float = 60.0    # wheel centre to ball-joint plane, lateral
    desired_scrub_radius: float = 30.0  # sets the kingpin lean (KPI)
    desired_caster_deg: float = 4.0
    independent_caster: bool = False  # False (default): caster FOLLOWS the
                                # kickup — the kingpin is kept planar in the
                                # 2D sketch (a kingpin lying in a
                                # kickup-tilted sketch has caster ~= kickup),
                                # so desired_caster_deg is informational.
                                # True: caster is its own knob (ball joints
                                # sit fore/aft of the sketch); the kingpin
                                # is then NOT perfectly planar and re-square
                                # leaves it alone.
    static_camber_deg: float = 0.0
    static_toe_deg: float = 0.0
    frame_half_width: float | None = None      # lower-rail half width;
                                               # default 0.07 * track
    steering_arm_x: float = -80.0  # tie-rod arm offset; negative = rack
                                   # behind the axle (rear steer)
    steering_arm_plane_deg: float = 90.0  # angle (about the kingpin) of the
                                # steering-arm plane (ball joints + tie-rod
                                # outer) to the TIRE plane (ball joints +
                                # wheel centre). The knuckle-consistency
                                # convention keeps the tire spin axis IN the
                                # tire plane and the steering arm at this
                                # angle to it; 90 deg is the usual choice.
    sketch_yaw_deg: float = 0.0  # twist of the whole 2D sketch plane about
                                # vertical (+ = tube front ends toward the
                                # centreline, left corner). For frames whose
                                # mount tubes run at an angle (e.g. an
                                # angled rear). Squareness is judged
                                # against THIS angle, not zero.
    kickup_deg: float = 10.0    # chassis kickup baked into the frame: the
                                # 2D design sketch is taken at this angle, so
                                # the four inboard bushing axes are tilted
                                # this much about the lateral (Y) axis (front
                                # end UP) to stay NORMAL to the sketch plane.
                                # Only the bushings tilt; the knuckle, ball
                                # joints and shock stay on the sketch path.
    inboard_sep_frac: float = 0.4  # vertical gap between the UPPER and LOWER
                                # inboard pivot axes, as a fraction of the
                                # kingpin's vertical extent. Bigger = more
                                # frame-mounting room, but it MUST stay below
                                # 1.0 (clamped to 0.8) or the camber curve
                                # reverses direction. ~0.5 is a good default.
    inboard_sep: float | None = None  # OR give the gap directly (mm, on the
                                # 2D sketch plane). Overrides inboard_sep_frac
                                # when set; same 0.8-of-kingpin clamp applies.
    kingpin_length: float | None = None  # outboard kingpin length: vertical
                                # span between the LBJ and UBJ, centred on the
                                # wheel centre (mm). Default 0.9 * tire radius.
                                # Ties directly into camber gain + roll centre.
    shock_on_uca: bool = False  # mount the shock's outer end on the UPPER
                                # control arm instead of the lower.
    tierod_on_lca: bool = False  # mount the toe link's INNER ball joint on
                                # the LOWER control arm (its centre rides the
                                # arm through travel). The seed drops the
                                # OUTBOARD link low on the kingpin (near the
                                # LBJ) to keep the tie-rod IC near the arm's
                                # IC — an arm-mounted link does NOT fix toe,
                                # so this just minimizes the (still real)
                                # bump steer. Meant for a fixed-toe rear DW.
    tierod_on_uca: bool = False  # same, but the inner rides the UPPER arm;
                                # the seed then puts the outboard link HIGH
                                # on the kingpin (near the UBJ).
    has_halfshaft: bool = False  # this axle is DRIVEN: model a CV axle.
                                # The GUI auto-places the inner joint at
                                # seed time (level at mid-travel) and the
                                # Halfshafts panel tunes it afterwards.
    x_offset: float = 39.0 * IN  # rigid translation applied to EVERY seed
    z_offset: float = -14.0 * IN # hardpoint (y untouched) so the coordinates
                                # land directly on the team's Onshape frame
                                # origin. With the -14 in drop the ground
                                # plane sits at z = -ride_height instead of
                                # z = 0; all ground-referenced metrics use
                                # the tire contact plane, so nothing else
                                # cares. Both values are user-editable. These
                                # are the INITIAL SEED PLACEMENT — a rigid
                                # shift only, no kinematic effect (the live
                                # "Ride height" tweak drives the z stance).
    frame_tube_offset: float = 0.625 * IN  # the ride-height datum sits this
                                # far above the model origin: it accounts for
                                # the frame tube's thickness in the CAD model
                                # (team default 5/8 in). Drives the live
                                # ride-height tweak; adjust it for a
                                # different vehicle's frame reference.


@dataclass
class SeedReport:
    """What the generator achieved, so the UI can show it next to the goals."""

    bump_travel: float        # mm of wheel travel available in bump
    droop_travel: float       # mm of wheel travel available in droop
    motion_ratio_static: float
    bump_steer_static: float  # deg/mm at ride height
    caster_static: float      # deg at ride height (the design caster; the
                              # kickup tilts the bushing axes, not the
                              # knuckle, so it does NOT inflate this)
    kickup_deg: float         # the kickup angle actually applied


def _measured_mr(hp: DoubleWishbonePoints, h: float = 1.0) -> float:
    """Static motion ratio (SHOCK travel / wheel travel, < 1) by central
    finite difference. Shock length shrinks in bump, so the numerator is
    written droop-minus-bump to come out positive."""
    s = DoubleWishboneSolver(hp)
    return (s.solve(-h).shock_length - s.solve(h).shock_length) / (2.0 * h)


def _measured_bump_steer(hp: DoubleWishbonePoints, h: float = 1.0) -> float:
    """Static d(toe)/d(travel) in deg/mm by central finite difference."""
    s = DoubleWishboneSolver(hp)
    return (toe_deg(s.solve(h)) - toe_deg(s.solve(-h))) / (2.0 * h)


def _reachable_travel(solver: DoubleWishboneSolver, want: float, sign: float) -> float:
    """Largest travel magnitude (mm) the linkage can solve in one direction,
    up to `want`. Steps outward until a solve fails, then backs off a hair so
    the returned endpoint stays safely inside the solvable range (the sweep
    and slider must be able to solve the reported extremes)."""
    # ~120 probes over the budget, but never coarser than 1/60th of it —
    # a flat 1 mm floor was most of an RC car's travel in a single step.
    step = max(want / 120.0, min(1.0, want / 60.0))
    reached = 0.0
    t = step
    while t <= want + 1e-9:
        try:
            solver.solve(sign * t)
        except ValueError:
            break
        reached = t
        t += step
    if reached >= want - 1e-6:
        return want                 # full budget is reachable
    return max(reached - step, 0.0)  # clamp just inside the true limit


def apply_seed_offset(hp, sv) -> None:
    """Rigidly place a freshly-seeded corner on the CAD frame. Changes no
    kinematics — it only moves the coordinate readout onto the Onshape
    frame origin.

    BOTH offsets name a datum on the CAR (v1.33):
      * x_offset is the AXLE STATION — the wheel centre lands on it.
      * z_offset is the GROUND PLANE — the tire contact patch lands on it
        (the seed builds the wheel centre one tire radius up, so a plain
        z shift already does this).

    x_offset used to be a plain translation, which anchored the INBOARD
    BUSHING AXES instead: the seed pins those at x = 0 and the kickup
    rotation then carries the wheel centre fore/aft by ~2.4 mm per degree,
    so the axle landed up to ~38 mm off the station that was asked for.
    That mattered because the rear axle is exported shifted by exactly the
    setup's wheelbase, so two axles seeded at different kickup no longer
    sat the entered wheelbase apart — and the wheelbase feeds every
    anti-geometry percentage. It also disagreed with the MEASURE path,
    which has always read x_offset back as the wheel-centre x."""
    off = np.array([float(sv.x_offset) - float(hp.wheel_center[0]),
                    0.0, float(sv.z_offset)])
    if not np.any(off):
        return
    for attr in hp.POINT_ATTRS:
        setattr(hp, attr, getattr(hp, attr) + off)


def _secant(f, x0, x1, tol, max_iter=10):
    """Tiny secant root find for the two placement tunes."""
    f0, f1 = f(x0), f(x1)
    for _ in range(max_iter):
        if abs(f1) < tol or abs(f1 - f0) < 1e-12:
            break
        x0, x1, f0 = x1, x1 - f1 * (x1 - x0) / (f1 - f0), f1
        f1 = f(x1)
    return x1


def setup_from_hardpoints(hp: DoubleWishbonePoints,
                          base: SetupVariables) -> SetupVariables:
    """MEASURE a SetupVariables from an existing double-wishbone design —
    the reverse of generate_seed. Every measurable setup value is read
    from the geometry (track, scrub, caster, kingpin length, axis gap,
    motion ratio, arm placement, sketch kickup/yaw, seed offsets...);
    values geometry can't express (shock hardware min/max, travel modes,
    the frame-tube datum) carry over from `base`.

    Purpose: reseed WITHOUT losing the design — load a candidate or the
    live geometry into the Setup form, flip the one seed-only option you
    actually wanted (shock arm, toe-link mount...), and Generate lands a
    seed that reproduces the measured design instead of resetting to
    defaults.

    Caveat (same as any seed): in the default one-sketch mode the seed
    planarizes the kingpin, so caster comes out following the measured
    KICKUP; a measured caster that differs from the kickup only survives
    a reseed with independent_caster enabled."""
    import dataclasses as _dc

    from .geometry import sketch_planarity
    from .tweaks import (caster_angle_deg, shock_length_at_ride,
                         steer_arm_length)
    p = sketch_planarity(hp)
    wc = hp.wheel_center
    lca_mid = (hp.lca_inner_front + hp.lca_inner_rear) / 2.0
    uca_mid = (hp.uca_inner_front + hp.uca_inner_rear) / 2.0
    ground_z = float(wc[2]) - float(hp.tire_radius)
    # Scrub per the SEED's own construction (kingpin ground pierce vs the
    # wheel-centre Y, flat patch) — NOT the metrics scrub, whose cambered
    # contact patch shifts a few mm; feeding that back would drift the
    # reseed by exactly that shift.
    lbj, ubj = hp.lca_outer, hp.uca_outer
    pierce_y = float(lbj[1] + (ground_z - lbj[2])
                     * (ubj[1] - lbj[1]) / (ubj[2] - lbj[2]))
    return _dc.replace(
        base,
        track_width=2.0 * abs(float(wc[1])),
        # invert the seed's z_lca = ride_height + 0.07 * tire_radius rule
        # (measured from the ground plane through the contact patch)
        ride_height=float(lca_mid[2]) - ground_z - 0.07 * float(hp.tire_radius),
        tire_radius=float(hp.tire_radius),
        tire_width=float(hp.tire_width),
        static_camber_deg=float(hp.static_camber_deg),
        static_toe_deg=float(hp.static_toe_deg),
        shock_on_uca=bool(hp.shock_on_uca),
        tierod_on_lca=bool(getattr(hp, "tierod_on_lca", False)),
        tierod_on_uca=bool(getattr(hp, "tierod_on_uca", False)),
        shock_length_at_ride=shock_length_at_ride(hp),
        motion_ratio_goal=float(_measured_mr(hp)),
        hub_offset=float(wc[1] - hp.lca_outer[1]),
        desired_scrub_radius=float(wc[1]) - pierce_y,
        desired_caster_deg=caster_angle_deg(hp),
        steering_arm_x=steer_arm_length(hp),
        kickup_deg=float(p["sketch_kickup_deg"]),
        sketch_yaw_deg=float(p["axis_yaw_deg"]),
        # the seed's inboard_sep and kingpin_length are VERTICAL spans
        inboard_sep=float(uca_mid[2] - lca_mid[2]),
        kingpin_length=float(hp.uca_outer[2] - hp.lca_outer[2]),
        frame_half_width=float(lca_mid[1]),
        # placement: the design frame puts the wheel centre at x = 0 with
        # the ground at z = 0, so the current station IS the offset
        x_offset=float(wc[0]),
        z_offset=float(wc[2] - hp.tire_radius),
    )


def apply_locked_points(hp_new, hp_ref, locked,
                        independent_caster: bool = False,
                        steering_arm_plane_deg: float = 90.0):
    """Make a fresh seed HONOR the previous design's locked hardpoints:
    the reseed then builds AROUND the points you pinned (welded tabs,
    fixed halfshaft-driven mounts) instead of relocating them through the
    placement heuristics.

    Two steps: (1) rigidly slide the whole new seed in X/Z by the mean
    locked-point delta — a pure placement move that keeps the seed's
    kinematics and kills the bulk of any height/fore-aft jump; (2) paste
    each locked point EXACTLY, then re-square the kingpin (unless the UBJ
    itself is locked or independent-caster is on) and re-enforce the
    knuckle convention so the pasted residuals stay consistent. Returns
    hp_new untouched when there is nothing applicable to honor."""
    import dataclasses as _dc

    if hp_ref is None or type(hp_new) is not type(hp_ref):
        return hp_new
    attrs = [a for a in (locked or ()) if a in type(hp_new).POINT_ATTRS]
    if not attrs:
        return hp_new
    deltas = np.array([np.asarray(getattr(hp_ref, a), float)
                       - np.asarray(getattr(hp_new, a), float)
                       for a in attrs])
    t = deltas.mean(axis=0)
    t[1] = 0.0            # y is the arms' business: track must not shift
    hp2 = _dc.replace(hp_new, **{a: getattr(hp_new, a) + t
                                 for a in type(hp_new).POINT_ATTRS})
    hp2 = _dc.replace(hp2, **{a: np.asarray(getattr(hp_ref, a), float).copy()
                              for a in attrs})
    if isinstance(hp2, DoubleWishbonePoints):
        from .geometry import enforce_knuckle_planes, planarize_kingpin
        if "uca_outer" not in attrs and not independent_caster:
            hp2 = planarize_kingpin(hp2)
        hp2 = enforce_knuckle_planes(hp2, steering_arm_plane_deg)
    return hp2


def generate_seed(sv: SetupVariables) -> tuple[DoubleWishbonePoints, SeedReport]:
    """Build a starting hardpoint set from the setup variables."""
    # ------------------------------------------------------------------
    # Wheel and travel budget
    # ------------------------------------------------------------------
    stroke = sv.shock_max_length - sv.shock_min_length
    l_ride = (sv.shock_length_at_ride
              if sv.shock_length_at_ride is not None
              else sv.shock_min_length + 0.7 * stroke)
    if not sv.shock_min_length < l_ride < sv.shock_max_length:
        raise ValueError("shock length at ride must be between min and max")
    bump = (l_ride - sv.shock_min_length) / sv.motion_ratio_goal
    droop = (sv.shock_max_length - l_ride) / sv.motion_ratio_goal

    # Wheel centre: half track out, one tire radius up (tire on the ground).
    wc = _pt(0.0, sv.track_width / 2.0, sv.tire_radius)

    # ------------------------------------------------------------------
    # Kingpin: ball joints from hub offset + desired scrub/caster
    # ------------------------------------------------------------------
    # Ball joints split the kingpin length symmetrically about the wheel
    # centre. Default span (0.9 * tire radius) tucks both joints inside the
    # wheel; the user can set kingpin_length directly since it ties into
    # camber gain and roll centre.
    kp_span = (sv.kingpin_length if sv.kingpin_length is not None
               else 0.9 * sv.tire_radius)
    z_lbj = wc[2] - kp_span / 2.0
    z_ubj = wc[2] + kp_span / 2.0
    y_lbj = wc[1] - sv.hub_offset
    # The kingpin must pierce the ground `scrub` inboard of the contact
    # patch. The line through that ground point and the LBJ fixes the KPI,
    # and extending it up to z_ubj places the UBJ laterally.
    y_ground = wc[1] - sv.desired_scrub_radius
    kingpin_slope = (y_lbj - y_ground) / z_lbj      # dy per dz, < 0 = lean in
    y_ubj = y_ground + kingpin_slope * z_ubj
    # Caster: split the fore-aft offset between the two joints.
    half_caster = 0.5 * np.tan(np.radians(sv.desired_caster_deg)) * (z_ubj - z_lbj)
    lbj = _pt(+half_caster, y_lbj, z_lbj)
    ubj = _pt(-half_caster, y_ubj, z_ubj)

    # ------------------------------------------------------------------
    # Control arms: chassis pivots on the frame rails
    # ------------------------------------------------------------------
    fhw = sv.frame_half_width if sv.frame_half_width is not None else 0.07 * sv.track_width
    fhw_upper = 1.6 * fhw          # upper rail is typically wider than lower
    z_lca = sv.ride_height + 0.07 * sv.tire_radius  # tab just above the rail
    # Vertical gap between the upper and lower inboard axes. It needs to be
    # generous enough to bolt to separate frame tubes, yet stay well under
    # the kingpin's vertical extent — once the inboard axes are spaced as far
    # apart as the ball joints, the front-view instant centre crosses over
    # and the camber curve reverses. Clamp to keep that margin. The gap can
    # be given directly (inboard_sep, mm) or as a fraction of the kingpin.
    gap = (sv.inboard_sep if sv.inboard_sep is not None
           else sv.inboard_sep_frac * (z_ubj - z_lbj))
    gap = float(np.clip(gap, 0.1 * (z_ubj - z_lbj), 0.8 * (z_ubj - z_lbj)))
    z_uca = z_lca + gap
    # Pivot pairs straddle the axle fore/aft, ~quarter of the arm span each
    # way. CRUCIAL (per the team's CAD): each arm's front->rear bushing line
    # must be NORMAL to the 2D sketch plane. That sketch is the front view
    # taken at the chassis kickup angle, so its normal is tilted by the
    # kickup (front end up). Spreading the bushings along that normal — and
    # NOT rotating anything else — keeps the ball joints, knuckle and shock
    # on the 2D sketch path, so the suspension follows the sketch and no
    # spurious caster is baked into the knuckle.
    n = sketch_plane_normal(sv.kickup_deg, sv.sketch_yaw_deg)
    lca_spread = 0.25 * (y_lbj - fhw)
    uca_spread = 0.22 * (y_ubj - fhw_upper)
    lca_mid_in = _pt(0.0, fhw, z_lca)         # lower bushing-axis midpoint
    uca_mid_in = _pt(0.0, fhw_upper, z_uca)   # upper bushing-axis midpoint
    lca_if = lca_mid_in + lca_spread * n
    lca_ir = lca_mid_in - lca_spread * n
    uca_if = uca_mid_in + uca_spread * n
    uca_ir = uca_mid_in - uca_spread * n

    # ------------------------------------------------------------------
    # Tie rod: steering/toe arm off the kingpin, inner end at the chassis
    # (default) or on a control arm.
    # ------------------------------------------------------------------
    # Outboard link height ON the kingpin. Chassis-mounted links sit ~1/4
    # up; an ARM-mounted inner needs the outboard end near the SAME arm's
    # ball joint so the tie-rod IC stays close to the arm's IC and bump
    # steer is minimized (verified: dropping it to the LBJ cuts lower-arm
    # bump steer ~8x). So low for a lower-arm mount, high for an upper.
    tro_frac = (0.08 if sv.tierod_on_lca
                else 0.92 if sv.tierod_on_uca else 0.25)
    z_tro = z_lbj + tro_frac * (z_ubj - z_lbj)
    # Put the arm pivot ON the kingpin line at that height, then offset it
    # fore/aft to make the actual steering arm.
    y_tro = y_ground + kingpin_slope * z_tro
    x_kp = lbj[0] + (z_tro - z_lbj) / (z_ubj - z_lbj) * (ubj[0] - lbj[0])
    tro = _pt(x_kp + sv.steering_arm_x, y_tro, z_tro)
    tri_y = 0.8 * fhw

    def tierod_inner_at(z: float) -> np.ndarray:
        return _pt(tro[0], tri_y, z)   # rack end in the same fore-aft plane

    # ------------------------------------------------------------------
    # Shock: outer end slides along its mounting arm until the motion
    # ratio is hit (lower arm by default; upper if shock_on_uca)
    # ------------------------------------------------------------------
    lbj_fv = _pt(0.0, y_lbj, z_lbj)
    ubj_fv = _pt(0.0, y_ubj, z_ubj)
    mount_inner = uca_mid_in if sv.shock_on_uca else lca_mid_in
    mount_outer = ubj_fv if sv.shock_on_uca else lbj_fv
    shock_dir = _pt(0.0, -np.cos(np.radians(68.0)), np.sin(np.radians(68.0)))
    # ~68 deg from horizontal, leaning inboard — typical coilover stance.

    def build(p: float, tri_par: float) -> DoubleWishbonePoints:
        """Assemble the full hardpoint set for a given shock-mount fraction
        `p` along the mounting arm and tie-rod inner parameter `tri_par`:
        the inner's HEIGHT (mm) when chassis-mounted, or its FRACTION along
        the lower arm (axis midpoint -> LBJ) when tierod_on_lca. Either
        way it is the bump-steer tuning knob below.

        Only the four inboard bushings carry the kickup (they were spread
        along the sketch-plane normal above); everything else sits on the
        2D front-view sketch. So the tire is already on the ground and the
        knuckle follows the sketch — no whole-corner rotation or re-seat.
        """
        shock_outer = mount_inner + p * (mount_outer - mount_inner)
        shock_inner = shock_outer + l_ride * shock_dir
        # Arm-mounted toe link: the inner BALL JOINT sits ON the chosen arm
        # at fraction tri_par along it (axis midpoint -> ball joint) and its
        # centre rides that arm's rotation. Chassis-mounted (default): the
        # rack/fixed end at height tri_par.
        if sv.tierod_on_uca:
            tri = uca_mid_in + tri_par * (ubj - uca_mid_in)
        elif sv.tierod_on_lca:
            tri = lca_mid_in + tri_par * (lbj - lca_mid_in)
        else:
            tri = tierod_inner_at(tri_par)
        hp = DoubleWishbonePoints(
            uca_inner_front=uca_if, uca_inner_rear=uca_ir, uca_outer=ubj,
            lca_inner_front=lca_if, lca_inner_rear=lca_ir, lca_outer=lbj,
            tierod_inner=tri, tierod_outer=tro,
            wheel_center=wc,
            shock_inner=shock_inner, shock_outer=shock_outer,
            tire_radius=sv.tire_radius,
            tire_width=sv.tire_width,
            static_camber_deg=sv.static_camber_deg,
            static_toe_deg=sv.static_toe_deg,
            shock_on_uca=sv.shock_on_uca,
            tierod_on_lca=sv.tierod_on_lca,
            tierod_on_uca=sv.tierod_on_uca,
        )
        # Default one-sketch build: the kingpin lies IN the 2D sketch (no
        # knuckle twist; caster then follows the kickup). Doing it HERE,
        # inside build(), means the MR and bump-steer tuning below see the
        # planarized geometry. Independent-caster mode keeps the kingpin
        # off-plane so the designer's caster is honoured.
        if not sv.independent_caster:
            from .geometry import planarize_kingpin
            hp = planarize_kingpin(hp)
        # Knuckle-consistency convention (always on): put the tire spin
        # axis in the (ball joints + wheel centre) plane and the steering
        # arm at the set angle to it. Done inside build() so the MR and
        # bump-steer tuning below see the final wheel-centre / tie-rod.
        from .geometry import enforce_knuckle_planes
        hp = enforce_knuckle_planes(hp, sv.steering_arm_plane_deg)
        return hp

    # First guess: a mount at fraction p moves ~p of the wheel travel, and
    # the inclined shock sees that motion foreshortened by sin(68 deg).
    p0 = sv.motion_ratio_goal / np.sin(np.radians(68.0))
    arm_mount = sv.tierod_on_lca or sv.tierod_on_uca
    # Tie-rod inner start: an arm fraction (arm mount) or a chassis height.
    tri_par0 = 0.5 if arm_mount else z_tro + (z_lca - z_lbj)

    p = _secant(lambda p: _measured_mr(build(p, tri_par0)) - sv.motion_ratio_goal,
                p0, p0 + 0.05, tol=1e-3)
    p = float(np.clip(p, 0.2, 0.95))
    # Bump-steer tuning slides the CHASSIS-mounted inner's height. An
    # ARM-mounted inner is a different animal: bolting the toe-link inner
    # ball joint to the same arm as the lower/upper ball joint forms a
    # near-rigid BJ-inner-steering-arm triangle, so the steering arm
    # effectively follows that arm and the toe DOES change through travel
    # (large bump steer) — an arm-mounted link does NOT fix toe. That bump
    # steer is set by the OUTBOARD link height on the kingpin (seeded low
    # for a lower mount, high for an upper, to keep the ICs close) and is
    # nearly INSENSITIVE to where along the arm the inner picks up, so the
    # inner-height secant below is skipped; the seed reports the real bump
    # steer and the designer tunes it with the steering-arm (outer) tweak.
    tri_par = (tri_par0 if arm_mount
               else _secant(lambda v: _measured_bump_steer(build(p, v)),
                            tri_par0, tri_par0 + 10.0, tol=1e-5))

    hp = build(p, tri_par)   # already planarized inside build() by default

    # ------------------------------------------------------------------
    # Usable travel = the shock-stroke budget, CLAMPED to what the linkage
    # can actually articulate. The budget (MR_static * stroke) is optimistic
    # because the motion ratio drops in bump, so the geometry often tops out
    # a little short; report the achievable travel rather than rejecting an
    # otherwise good seed. Only a genuinely broken seed (can't reach even a
    # third of the budget) is rejected.
    # ------------------------------------------------------------------
    solver = DoubleWishboneSolver(hp)
    bump = _reachable_travel(solver, bump, +1.0)
    droop = _reachable_travel(solver, droop, -1.0)
    if bump < 0.3 * ((l_ride - sv.shock_min_length) / sv.motion_ratio_goal) or \
       droop < 0.3 * ((sv.shock_max_length - l_ride) / sv.motion_ratio_goal):
        raise ValueError(
            f"seed geometry can barely articulate (+{bump:.0f}/-{droop:.0f} mm); "
            f"check the setup variables")

    report = SeedReport(
        bump_travel=bump,
        droop_travel=droop,
        motion_ratio_static=_measured_mr(hp),
        bump_steer_static=_measured_bump_steer(hp),
        caster_static=caster_deg(solver.solve(0.0)),
        kickup_deg=sv.kickup_deg,
    )
    apply_seed_offset(hp, sv)   # rigid shift onto the Onshape frame origin
    return hp, report
