"""C-hub front suspension (the 1/10-scale RC buggy front end, scaled up).

Team Associated B7-style corner: the LOWER control arm swings on its
chassis bushing axis and carries a HINGE PIN at its outer end. The C-hub
(caster block) rotates on that pin, and a single CAMBER LINK — a
turnbuckle with ball ends running from a chassis point (usually near the
shock tower) to the top of the C-hub — closes the loop and sets the
camber curve. The steering block pivots on a PHYSICAL KINGPIN fixed in
the C-hub, so caster / KPI / trail are properties of the C-hub and
steering-block hardware (the RC world's interchangeable caster blocks
and KPI inserts), not of a ball-joint-to-ball-joint line.

Kinematic chain (see hinge_carrier.py for the shared closed-form core):

    chassis --arm axis--> lower arm --hinge pin--> C-hub --kingpin--> block
                                                    |
    chassis -------------- camber link -------------+
    chassis/rack --------- tie rod ---------------------------- block

Pose parameters, all zero at static: `a` arm angle, `b` C-hub angle
about the posed hinge pin (closed by the camber link), `c` steer angle
about the posed kingpin (closed by the tie rod; the rack end moves
laterally with `steer` exactly like the double wishbone's). Each closure
is the A*cos + B*sin = C identity — fully closed form; only the travel ->
arm-angle inversion is a scalar root find.

The solved state carries the posed kingpin ends as ubj/lbj, so every
kingpin metric (caster, KPI, scrub, trail) works unchanged; the front-
and side-view instant centres come from metrics.py's NUMERIC path (the
Adams-style finite-difference construction).

Why the numeric path (measured, v1.30): it is NOT the extra hinge that
breaks the graphical construction. Straight-ahead, intersecting the
lower-arm line with the camber-link line predicts the true instant centre
to 0.01 mm at ride and within 4.7 mm over +/-45 mm of travel — which is
why seed_chub_front is entitled to place the camber link with exactly
that construction. What breaks it is STEER: once the block rotates on the
kingpin the tie rod governs the wheel's motion, the carrier's IC stops
being the wheel's IC, and the construction is out by ~170-220 mm at 30 mm
of rack. So the wheel's IC must be measured, not drawn.

Two invariants worth knowing when reading the curves, both verified
numerically: camber + KPI is CONSTANT (4.99 deg on the seeded corner, to
within 0.006 deg) because the kingpin is a physical pin inside the
carrier and the wheel is rigidly attached to the block — it is the
included angle of the hardware, not a pose-dependent quantity. And caster
is nearly constant through travel (0.041 deg over +/-60 mm) because the
C-hub rotates about a roughly fore-aft pin, which leans the kingpin
sideways (KPI, hence camber) but barely fore-aft. This matches how RC
racers treat the part: caster is set by swapping the caster block.
"""

from dataclasses import dataclass

import numpy as np

from .hinge_carrier import closing_angle, rotate_about, rotate_dir, solve_travel, _unit

Z_UP = np.array([0.0, 0.0, 1.0])


@dataclass
class CHubFrontPoints:
    """Hardpoints for one C-hub front corner (tool frame, mm, LEFT side,
    static pose — link lengths are measured from these positions)."""

    arm_inner_front: np.ndarray   # lower-arm chassis bushing, forward
    arm_inner_rear: np.ndarray    # lower-arm chassis bushing, rearward
    hinge_front: np.ndarray       # C-hub hinge pin, forward end (ON the arm)
    hinge_rear: np.ndarray        # C-hub hinge pin, rearward end (ON the arm)
    camber_inner: np.ndarray      # camber link, chassis ball (near the tower)
    camber_outer: np.ndarray      # camber link, ball ON the C-hub (top)
    kingpin_upper: np.ndarray     # steering pivot, upper end (IN the C-hub)
    kingpin_lower: np.ndarray     # steering pivot, lower end (IN the C-hub)
    tierod_inner: np.ndarray      # tie rod, chassis/rack end
    tierod_outer: np.ndarray      # tie rod, ball ON the steering block
    wheel_center: np.ndarray      # in the steering block
    shock_inner: np.ndarray       # chassis end
    shock_outer: np.ndarray       # ON THE LOWER ARM (B7 style)
    tire_radius: float
    tire_width: float = 150.0
    static_camber_deg: float = 0.0
    static_toe_deg: float = 0.0

    POINT_ATTRS = ["arm_inner_front", "arm_inner_rear",
                   "hinge_front", "hinge_rear",
                   "camber_inner", "camber_outer",
                   "kingpin_upper", "kingpin_lower",
                   "tierod_inner", "tierod_outer",
                   "wheel_center", "shock_inner", "shock_outer"]

    LABELS = {
        "arm_inner_front": "Lower arm bushing — front",
        "arm_inner_rear": "Lower arm bushing — rear",
        "hinge_front": "C-hub hinge pin — front",
        "hinge_rear": "C-hub hinge pin — rear",
        "camber_inner": "Camber link — chassis",
        "camber_outer": "Camber link — C-hub",
        "kingpin_upper": "Kingpin — upper (in C-hub)",
        "kingpin_lower": "Kingpin — lower (in C-hub)",
        "tierod_inner": "Tie rod — chassis/rack",
        "tierod_outer": "Tie rod — steering block",
        "wheel_center": "Wheel centre",
        "shock_inner": "Shock — chassis",
        "shock_outer": "Shock — on lower arm",
    }


@dataclass
class CHubFrontState:
    """Solved pose. ubj/lbj are the POSED KINGPIN ends, so the kingpin
    metrics read the physical steering axis; tro is the posed tie-rod
    ball. Duck-type compatible with CornerState."""

    travel: float
    steer: float
    arm_angle: float        # a, rad
    chub_angle: float       # b, rad (about the hinge pin)
    steer_angle: float      # c, rad (about the kingpin)
    wheel_center: np.ndarray
    spindle: np.ndarray
    contact_patch: np.ndarray
    shock_outer: np.ndarray
    shock_length: float
    ubj: np.ndarray         # posed kingpin_upper
    lbj: np.ndarray         # posed kingpin_lower
    tro: np.ndarray         # posed tierod_outer
    hinge_front: np.ndarray
    hinge_rear: np.ndarray
    camber_outer: np.ndarray


class CHubFrontSolver:
    """Forward solver: same public contract as DoubleWishboneSolver."""

    def __init__(self, hp: CHubFrontPoints):
        self.hp = hp
        self._arm_origin = hp.arm_inner_front
        self._arm_axis = _unit(hp.arm_inner_rear - hp.arm_inner_front)
        # Closing-link lengths, measured AT STATIC (that is what makes the
        # static pose a = b = c = 0 an exact assembly).
        self._camber_len = float(np.linalg.norm(hp.camber_outer
                                                - hp.camber_inner))
        self._tierod_len = float(np.linalg.norm(hp.tierod_outer
                                                - hp.tierod_inner))
        # Exact static spindle (v1.17 form): camber AND toe read back the
        # stored values exactly for any pair.
        gamma = np.radians(hp.static_camber_deg)
        tau = np.radians(hp.static_toe_deg)
        s = np.array([np.tan(tau), 1.0, -np.tan(gamma)])
        self._spindle0 = s / np.linalg.norm(s)
        self._wc_z0 = float(hp.wheel_center[2])
        self._last_a = 0.0            # travel-inversion warm start

    # -- the pose chain -------------------------------------------------
    def _arm_pt(self, p, a):
        return rotate_about(self._arm_origin, self._arm_axis, p, a)

    def _pose(self, a: float):
        """Arm + C-hub pose at arm angle `a`: posed hinge pin, the C-hub
        angle b from the camber-link closure, and a carrier-point mapper."""
        hf = self._arm_pt(self.hp.hinge_front, a)
        hr = self._arm_pt(self.hp.hinge_rear, a)
        axis = _unit(hr - hf)
        b = closing_angle(hf, axis, self._arm_pt(self.hp.camber_outer, a),
                          self.hp.camber_inner, self._camber_len)

        def chub_pt(p0):
            return rotate_about(hf, axis, self._arm_pt(p0, a), b)
        return hf, hr, axis, b, chub_pt

    def solve_at_arm_angle(self, a: float, steer: float = 0.0) -> CHubFrontState:
        hp = self.hp
        hf, hr, axis, b, chub_pt = self._pose(a)
        ku, kl = chub_pt(hp.kingpin_upper), chub_pt(hp.kingpin_lower)
        kp_axis = _unit(ku - kl)
        # Steering closure: rotate the block about the posed kingpin until
        # the tie rod reaches the rack end (displaced laterally by steer).
        rack = hp.tierod_inner + np.array([0.0, steer, 0.0])
        c = closing_angle(kl, kp_axis, chub_pt(hp.tierod_outer), rack,
                          self._tierod_len)

        def block_pt(p0):
            return rotate_about(kl, kp_axis, chub_pt(p0), c)
        wc = block_pt(hp.wheel_center)
        # Directions compose through the same three rotations as points.
        spindle = rotate_dir(
            kp_axis,
            rotate_dir(axis, rotate_dir(self._arm_axis, self._spindle0, a), b),
            c)
        forward = _unit(np.cross(spindle, Z_UP))
        down = _unit(np.cross(spindle, forward))
        shock_outer = self._arm_pt(hp.shock_outer, a)
        return CHubFrontState(
            travel=wc[2] - self._wc_z0,
            steer=steer,
            arm_angle=a, chub_angle=b, steer_angle=c,
            wheel_center=wc,
            spindle=spindle,
            contact_patch=wc + hp.tire_radius * down,
            shock_outer=shock_outer,
            shock_length=float(np.linalg.norm(shock_outer - hp.shock_inner)),
            ubj=ku, lbj=kl, tro=block_pt(hp.tierod_outer),
            hinge_front=hf, hinge_rear=hr,
            camber_outer=chub_pt(hp.camber_outer),
        )

    # -- public contract ------------------------------------------------
    def solve(self, travel: float, steer: float = 0.0) -> CHubFrontState:
        def wc_z(a):
            return self.solve_at_arm_angle(a, steer).wheel_center[2]
        a = solve_travel(wc_z, self._wc_z0 + travel, last_a=self._last_a)
        self._last_a = a
        return self.solve_at_arm_angle(a, steer)

    def walk_travels(self, travels, steer: float = 0.0):
        """Warm-started walk outward from static (the continuation pattern
        every solver in this tool uses to stay on the physical branch)."""
        travels = np.asarray(travels, dtype=float)
        states = [None] * len(travels)
        self._last_a = 0.0
        for idx in np.argsort(np.abs(travels), kind="stable"):
            states[idx] = self.solve(travels[idx], steer)
        return states


def seed_chub_front(sv) -> CHubFrontPoints:
    """Heuristic starting geometry for a C-hub front corner from the same
    SetupVariables the double wishbone uses.

    Placement logic (Baja scale, pre-offset frame: ground z = 0, wheel at
    y = track/2):
      * lower arm bushings on the frame rail at the sketch spacing;
      * hinge pin fore-aft under the kingpin, straddling it;
      * kingpin centred near the wheel centre, inset by hub_offset, tilted
        by desired_caster (fore-aft) and leaned so its ground pierce hits
        desired_scrub_radius (closed form — no secant needed);
      * camber link from a chassis point above the upper rail to the top
        of the C-hub (roughly parallel to the arm: gentle camber gain);
      * tie rod behind the kingpin at steering_arm_x, its chassis end at
        the rack; the INNER end height is secant-tuned for low bump steer;
      * shock on the lower arm, its mount fraction secant-tuned to the
        motion-ratio goal (same approach as every other seed).

    Raises ValueError if the corner cannot articulate the shock-stroke
    travel budget."""
    fhw = (sv.frame_half_width if sv.frame_half_width is not None
           else 0.07 * sv.track_width)
    r = sv.tire_radius
    wc = np.array([0.0, sv.track_width / 2.0, r])
    y_kp = wc[1] - sv.hub_offset          # kingpin plane, inboard of wheel

    # Kingpin: centre at wheel-centre height, direction from caster + the
    # lean that lands the ground pierce at the scrub target.
    kp_len = sv.kingpin_length if sv.kingpin_length is not None else 0.38 * r
    patch_y = wc[1]
    pierce_y = patch_y - sv.desired_scrub_radius
    # The axis leaves kp_mid (at wheel-centre height) and reaches the
    # ground pierce point wc_z below: tan(KPI) = inboard shift per unit
    # height, positive = top leans inboard (the metrics convention).
    tan_kpi = (pierce_y - y_kp) / wc[2]
    d = np.array([-np.tan(np.radians(sv.desired_caster_deg)), -tan_kpi, 1.0])
    d = d / np.linalg.norm(d)
    kp_mid = np.array([0.0, y_kp, wc[2]])
    kingpin_upper = kp_mid + 0.5 * kp_len * d
    kingpin_lower = kp_mid - 0.5 * kp_len * d

    # Hinge pin: fore-aft, straddling the kingpin just below its lower end
    # (the C-hub wraps around the pin, B7 style).
    pin_half = 0.16 * r
    pin_c = kp_mid - 0.62 * kp_len * d    # a little below kingpin_lower
    hinge_front = pin_c + np.array([pin_half, 0.0, 0.0])
    hinge_rear = pin_c - np.array([pin_half, 0.0, 0.0])

    # Lower arm bushings on the rail, spread like the DW sketch spacing.
    spread = 0.55 * (wc[1] - fhw)
    z_rail = 0.55 * r
    arm_inner_front = np.array([+0.5 * spread, fhw, z_rail])
    arm_inner_rear = np.array([-0.5 * spread, fhw, z_rail])

    # Camber link: from the top of the C-hub to a chassis ball, AIMED so
    # the link's front-view line converges with the lower arm's line at a
    # sensible inboard instant centre (a swing-arm length of ~1.5 tracks)
    # — the placement that gives a low-but-positive roll centre and mild
    # negative camber gain instead of an accidental below-ground IC.
    camber_outer = kingpin_upper + np.array([0.0, 0.02 * r, 0.14 * r])
    arm_slope = (pin_c[2] - z_rail) / (pin_c[1] - fhw)
    ic_y = -0.55 * wc[1]                      # past the centreline, inboard
    ic = np.array([ic_y, z_rail + (ic_y - fhw) * arm_slope])
    y_ci = fhw + 0.06 * sv.track_width        # chassis ball lateral spot
    t = (y_ci - camber_outer[1]) / (ic[0] - camber_outer[1])
    camber_inner = np.array([0.0, y_ci,
                             camber_outer[2] + t * (ic[1] - camber_outer[2])])

    # Tie rod: steering arm behind (or ahead of) the kingpin per
    # steering_arm_x; rack end near the centreline at the same height.
    tierod_outer = np.array([sv.steering_arm_x, y_kp,
                             kingpin_lower[2] + 0.25 * kp_len])
    tierod_inner0 = np.array([sv.steering_arm_x, fhw, tierod_outer[2]])

    stroke = sv.shock_max_length - sv.shock_min_length
    l_ride = (sv.shock_length_at_ride
              if sv.shock_length_at_ride is not None
              else sv.shock_min_length + 0.7 * stroke)
    bump = (l_ride - sv.shock_min_length) / sv.motion_ratio_goal
    droop = (sv.shock_max_length - l_ride) / sv.motion_ratio_goal

    arm_mid = (arm_inner_front + arm_inner_rear) / 2.0
    shock_dir = np.array([0.0, -np.cos(np.radians(68.0)),
                          np.sin(np.radians(68.0))])

    def build(p: float, tri_z: float) -> CHubFrontPoints:
        outer = arm_mid + p * (pin_c - arm_mid)
        tri = tierod_inner0.copy()
        tri[2] = tri_z
        return CHubFrontPoints(
            arm_inner_front=arm_inner_front, arm_inner_rear=arm_inner_rear,
            hinge_front=hinge_front, hinge_rear=hinge_rear,
            camber_inner=camber_inner, camber_outer=camber_outer,
            kingpin_upper=kingpin_upper, kingpin_lower=kingpin_lower,
            tierod_inner=tri, tierod_outer=tierod_outer,
            wheel_center=wc,
            shock_inner=outer + l_ride * shock_dir, shock_outer=outer,
            tire_radius=r, tire_width=sv.tire_width,
            static_camber_deg=sv.static_camber_deg,
            static_toe_deg=sv.static_toe_deg)

    def mr(p: float) -> float:
        s = CHubFrontSolver(build(p, tierod_inner0[2]))
        # MR = SHOCK travel / WHEEL travel (< 1). Shock shortens in
        # bump, so droop-minus-bump keeps it positive.
        return (s.solve(-1.0).shock_length
                - s.solve(1.0).shock_length) / 2.0

    # secant on the shock-mount fraction toward the motion-ratio goal
    p0, p1 = sv.motion_ratio_goal, sv.motion_ratio_goal + 0.05
    f0, f1 = mr(p0) - sv.motion_ratio_goal, mr(p1) - sv.motion_ratio_goal
    for _ in range(10):
        if abs(f1) < 1e-3 or abs(f1 - f0) < 1e-12:
            break
        p0, p1, f0 = p1, p1 - f1 * (p1 - p0) / (f1 - f0), f1
        p1 = float(np.clip(p1, 0.2, 0.98))
        f1 = mr(p1) - sv.motion_ratio_goal

    # secant on the tie-rod INNER height for low bump steer at ride (the
    # classic tuning motion; the outer end stays on the steering arm)
    from .metrics import toe_deg

    def bump_steer(tri_z: float) -> float:
        s = CHubFrontSolver(build(p1, tri_z))
        h = min(10.0, 0.4 * bump, 0.4 * droop)
        return (toe_deg(s.solve(h)) - toe_deg(s.solve(-h))) / (2.0 * h)

    z0, z1 = tierod_inner0[2], tierod_inner0[2] + 5.0
    try:
        g0, g1 = bump_steer(z0), bump_steer(z1)
        for _ in range(10):
            if abs(g1) < 2e-4 or abs(g1 - g0) < 1e-12:
                break
            z0, z1, g0 = z1, z1 - g1 * (z1 - z0) / (g1 - g0), g1
            # Search band scales with the car (0.41*r reproduces the old
            # 120 mm at Baja size, and stays sane at 1/10 RC scale where a
            # fixed 120 mm is nearly half the wheelbase).
            z_lim = 0.41 * r
            z1 = float(np.clip(z1, tierod_inner0[2] - z_lim,
                               tierod_inner0[2] + z_lim))
            g1 = bump_steer(z1)
    except ValueError:
        z1 = tierod_inner0[2]        # keep the geometric default

    hp = build(p1, z1)
    solver = CHubFrontSolver(hp)
    solver.solve(bump)               # articulation check: raises if short
    solver.solve(-droop)
    from .seed import apply_seed_offset
    apply_seed_offset(hp, sv)        # rigid shift onto the Onshape origin
    return hp
