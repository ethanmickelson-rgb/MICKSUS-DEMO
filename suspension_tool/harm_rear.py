"""H-arm rear suspension (a simple, effective driven rear end).

The lower member is a rigid H-arm: two inboard bushings on a fore-aft
axis (like a trailing/semi-trailing lower arm) and TWO outboard points
that grab the upright. Because it grabs the upright at two points, the
lower arm alone sets toe AND locates the bottom of the upright — no
separate toe link is needed, which is what makes the layout simple and
robust (the wide fore-aft base resists toe deflection under load). A
single upper CAMBER LINK (a turnbuckle, ball ends, chassis -> top of the
upright) then sets camber and closes the last freedom.

Kinematic chain (see hinge_carrier.py for the shared closed-form core):

    chassis --arm axis--> lower H-arm --outboard grab line--> upright
                                                              |
    chassis ------------------ camber link -------------------+

The upright rotates about the line joining the two outboard grab points
(that line is the "hinge pin" of the shared core); the camber link
closes it. Count the freedoms: arm 1, grab-line pin +1 (the upright's
camber freedom), camber-link length -1 = one degree of freedom, the
wheel travel.

This is a DRIVEN axle: a CV halfshaft is modelled as the usual non-
structural driveline check (angle + plunge warnings), inner joint at the
diff flange, outer joint on the tire centreline. There is no steering
knuckle, so solve() ignores the steer argument and the kingpin metrics
(caster/KPI/scrub/trail) read NaN by design — like the trailing arm.

ANTI-GEOMETRY vs TOE (measured, v1.32.1). The chassis bushing axis is built
LEVEL here — SetupVariables.kickup_deg deliberately does NOT reach it, unlike
the double wishbone, whose tilted 2D sketch plane carries the kickup into all
four bushing axes. That is a design choice, not an oversight: this type has
NO toe link, so nothing can correct bump toe, and tilting the arm axis buys
anti-geometry at a measured cost of roughly 0.19 deg of toe swing per degree
of kickup (0 deg -> 0.0003 deg of toe over +/-60 mm; 25 deg -> 4.87 deg).
Holding toe is the whole point of the layout, so the seed keeps the axis
level and the corner reports ~0% anti-squat / anti-lift.
A user who wants that trade can make it explicitly: the "Arm bushing-axis
kickup" row in the tweaks panel (v1.31) tilts the axis live, and the toe
curve shows the cost immediately.
"""

from dataclasses import dataclass

import numpy as np

from .hinge_carrier import closing_angle, rotate_about, rotate_dir, solve_travel, _unit

Z_UP = np.array([0.0, 0.0, 1.0])


@dataclass
class HArmRearPoints:
    """Hardpoints for one H-arm rear corner (tool frame, mm, LEFT side,
    static pose — the camber-link length is measured from these)."""

    arm_inner_front: np.ndarray   # lower H-arm chassis bushing, forward
    arm_inner_rear: np.ndarray    # lower H-arm chassis bushing, rearward
    outer_front: np.ndarray       # H-arm outboard grab point, forward (ON the arm)
    outer_rear: np.ndarray        # H-arm outboard grab point, rearward (ON the arm)
    camber_inner: np.ndarray      # upper camber link, chassis ball
    camber_outer: np.ndarray      # upper camber link, ball ON the upright (top)
    wheel_center: np.ndarray      # in the upright
    shock_inner: np.ndarray       # chassis end
    shock_outer: np.ndarray       # ON THE LOWER H-ARM
    tire_radius: float
    tire_width: float = 150.0
    static_camber_deg: float = 0.0
    static_toe_deg: float = 0.0

    POINT_ATTRS = ["arm_inner_front", "arm_inner_rear",
                   "outer_front", "outer_rear",
                   "camber_inner", "camber_outer",
                   "wheel_center", "shock_inner", "shock_outer"]

    LABELS = {
        "arm_inner_front": "H-arm bushing — front",
        "arm_inner_rear": "H-arm bushing — rear",
        "outer_front": "H-arm outboard — front",
        "outer_rear": "H-arm outboard — rear",
        "camber_inner": "Camber link — chassis",
        "camber_outer": "Camber link — upright",
        "wheel_center": "Wheel centre",
        "shock_inner": "Shock — chassis",
        "shock_outer": "Shock — on H-arm",
    }


@dataclass
class HArmRearState:
    """Solved pose. No steering knuckle: ubj/lbj/tro stay None so kingpin
    metrics NaN out (the trailing-arm convention)."""

    travel: float
    steer: float
    arm_angle: float          # a, rad
    upright_angle: float      # b, rad (about the outboard grab line)
    wheel_center: np.ndarray
    spindle: np.ndarray
    contact_patch: np.ndarray
    shock_outer: np.ndarray
    shock_length: float
    camber_outer: np.ndarray  # posed upper camber-link ball
    outer_front: np.ndarray
    outer_rear: np.ndarray
    ubj = None
    lbj = None
    tro = None


class HArmRearSolver:
    """Forward solver: same public contract as DoubleWishboneSolver."""

    def __init__(self, hp: HArmRearPoints):
        self.hp = hp
        self._arm_origin = hp.arm_inner_front
        self._arm_axis = _unit(hp.arm_inner_rear - hp.arm_inner_front)
        # The camber-link LENGTH is the closure constraint, at static.
        self._camber_len = float(np.linalg.norm(hp.camber_outer
                                                - hp.camber_inner))
        gamma = np.radians(hp.static_camber_deg)
        tau = np.radians(hp.static_toe_deg)
        s = np.array([np.tan(tau), 1.0, -np.tan(gamma)])
        self._spindle0 = s / np.linalg.norm(s)
        self._wc_z0 = float(hp.wheel_center[2])
        self._last_a = 0.0

    def _arm_pt(self, p, a):
        return rotate_about(self._arm_origin, self._arm_axis, p, a)

    def solve_at_arm_angle(self, a: float, steer: float = 0.0) -> HArmRearState:
        hp = self.hp
        of = self._arm_pt(hp.outer_front, a)
        orr = self._arm_pt(hp.outer_rear, a)
        axis = _unit(orr - of)                 # the upright's rotation axis
        # Camber closure: swing the upright about the grab line until the
        # camber link spans exactly its fixed length to the chassis ball.
        b = closing_angle(of, axis, self._arm_pt(hp.camber_outer, a),
                          hp.camber_inner, self._camber_len)

        def upright_pt(p0):
            return rotate_about(of, axis, self._arm_pt(p0, a), b)
        wc = upright_pt(hp.wheel_center)
        spindle = rotate_dir(
            axis, rotate_dir(self._arm_axis, self._spindle0, a), b)
        forward = _unit(np.cross(spindle, Z_UP))
        down = _unit(np.cross(spindle, forward))
        shock_outer = self._arm_pt(hp.shock_outer, a)
        return HArmRearState(
            travel=wc[2] - self._wc_z0,
            steer=steer,                        # carried, ignored (rear axle)
            arm_angle=a, upright_angle=b,
            wheel_center=wc,
            spindle=spindle,
            contact_patch=wc + hp.tire_radius * down,
            shock_outer=shock_outer,
            shock_length=float(np.linalg.norm(shock_outer - hp.shock_inner)),
            camber_outer=upright_pt(hp.camber_outer),
            outer_front=of, outer_rear=orr,
        )

    # -- public contract ------------------------------------------------
    def solve(self, travel: float, steer: float = 0.0) -> HArmRearState:
        def wc_z(a):
            return self.solve_at_arm_angle(a).wheel_center[2]
        a = solve_travel(wc_z, self._wc_z0 + travel, last_a=self._last_a)
        self._last_a = a
        return self.solve_at_arm_angle(a, steer)

    def walk_travels(self, travels, steer: float = 0.0):
        travels = np.asarray(travels, dtype=float)
        states = [None] * len(travels)
        self._last_a = 0.0
        for idx in np.argsort(np.abs(travels), kind="stable"):
            states[idx] = self.solve(travels[idx], steer)
        return states


def seed_harm_rear(sv) -> HArmRearPoints:
    """Heuristic starting geometry for an H-arm rear corner from the same
    SetupVariables as every other seed.

    Placement logic (pre-offset frame: ground z = 0, wheel at y = track/2):
      * lower H-arm bushings on the frame rail on a fore-aft axis, spread
        fore/aft for lateral stiffness;
      * two outboard grab points on the upright, hub_offset inboard of the
        wheel centre and BELOW it, spread fore/aft (the base that holds
        toe) — the line between them is the camber rotation axis;
      * upper camber link from the top of the upright to a chassis ball,
        AIMED at a sensible inboard instant centre (low-but-positive roll
        centre, mild negative camber gain) — the same aiming the C-hub
        seed uses, so it isn't accidentally below ground;
      * shock on the lower H-arm, mount fraction secant-tuned to the
        motion-ratio goal.

    Raises ValueError if the corner cannot articulate the travel budget."""
    fhw = (sv.frame_half_width if sv.frame_half_width is not None
           else 0.07 * sv.track_width)
    r = sv.tire_radius
    wc = np.array([0.0, sv.track_width / 2.0, r])
    y_grab = wc[1] - sv.hub_offset

    # Outboard grab points: below the wheel centre, spread fore/aft.
    grab_half = 0.20 * r
    z_grab = wc[2] - 0.30 * r
    outer_front = np.array([+grab_half, y_grab, z_grab])
    outer_rear = np.array([-grab_half, y_grab, z_grab])

    # Lower H-arm inboard bushings on the rail, fore-aft axis, level with
    # the grab points (a roughly horizontal lower arm).
    in_half = 0.22 * r
    z_rail = z_grab
    arm_inner_front = np.array([+in_half, fhw, z_rail])
    arm_inner_rear = np.array([-in_half, fhw, z_rail])

    # Upper camber link: top of the upright to a chassis ball, aimed so the
    # front-view camber-link line converges with the lower arm's line at an
    # inboard IC (swing-arm length ~1.5 tracks) — a low positive RC.
    camber_outer = np.array([0.0, y_grab + 0.03 * r, wc[2] + 0.30 * r])
    lower_slope = (z_grab - z_rail) / (y_grab - fhw)   # 0 here, kept general
    ic_y = -0.55 * wc[1]
    ic_z = z_rail + (ic_y - fhw) * lower_slope
    y_ci = fhw + 0.06 * sv.track_width
    t = (y_ci - camber_outer[1]) / (ic_y - camber_outer[1])
    camber_inner = np.array([0.0, y_ci,
                             camber_outer[2] + t * (ic_z - camber_outer[2])])

    stroke = sv.shock_max_length - sv.shock_min_length
    l_ride = (sv.shock_length_at_ride
              if sv.shock_length_at_ride is not None
              else sv.shock_min_length + 0.7 * stroke)
    bump = (l_ride - sv.shock_min_length) / sv.motion_ratio_goal
    droop = (sv.shock_max_length - l_ride) / sv.motion_ratio_goal

    arm_mid = (arm_inner_front + arm_inner_rear) / 2.0
    grab_mid = (outer_front + outer_rear) / 2.0
    shock_dir = np.array([0.0, -np.cos(np.radians(70.0)),
                          np.sin(np.radians(70.0))])

    def build(p: float) -> HArmRearPoints:
        outer = arm_mid + p * (grab_mid - arm_mid)
        return HArmRearPoints(
            arm_inner_front=arm_inner_front, arm_inner_rear=arm_inner_rear,
            outer_front=outer_front, outer_rear=outer_rear,
            camber_inner=camber_inner, camber_outer=camber_outer,
            wheel_center=wc,
            shock_inner=outer + l_ride * shock_dir, shock_outer=outer,
            tire_radius=r, tire_width=sv.tire_width,
            static_camber_deg=sv.static_camber_deg,
            static_toe_deg=sv.static_toe_deg)

    def mr(p: float) -> float:
        s2 = HArmRearSolver(build(p))
        # MR = SHOCK travel / WHEEL travel (< 1). Shock shortens in
        # bump, so droop-minus-bump keeps it positive.
        return (s2.solve(-1.0).shock_length
                - s2.solve(1.0).shock_length) / 2.0

    p0, p1 = sv.motion_ratio_goal, sv.motion_ratio_goal + 0.05
    f0, f1 = mr(p0) - sv.motion_ratio_goal, mr(p1) - sv.motion_ratio_goal
    for _ in range(10):
        if abs(f1) < 1e-3 or abs(f1 - f0) < 1e-12:
            break
        p0, p1, f0 = p1, p1 - f1 * (p1 - p0) / (f1 - f0), f1
        p1 = float(np.clip(p1, 0.2, 0.98))
        f1 = mr(p1) - sv.motion_ratio_goal

    hp = build(p1)
    solver = HArmRearSolver(hp)
    solver.solve(bump)               # articulation check: raises if short
    solver.solve(-droop)
    from .seed import apply_seed_offset
    apply_seed_offset(hp, sv)        # rigid shift onto the Onshape origin
    return hp
