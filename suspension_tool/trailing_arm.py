"""Trailing-arm / semi-trailing-arm kinematics (a common Baja rear end).

The whole corner is ONE rigid body — arm + upright + wheel — rotating
about a chassis pivot axis defined by two bushing points. The axis
orientation is free, which covers the whole family:

  * pivot axis purely lateral (along Y): PURE trailing arm — the wheel
    travels straight up/down in the front view, so camber and toe never
    change and the roll centre sits on the ground.
  * axis skewed in plan/elevation: SEMI-trailing — the rotation now has
    components about X and Z, so the wheel gains camber and toe through
    travel (the classic semi-trailing behaviour).

Because the motion is a single rotation, everything is closed form:
the wheel-centre height vs arm angle reduces to A*cos + B*sin = C (the
same identity the double-wishbone solver uses for its upper arm), and the
instant centres are EXACT — the front-/side-view IC is simply where the
pivot axis pierces that view's plane.

There is no steering knuckle: solve() accepts the interface's steer
argument but a rear trailing arm ignores it, and kingpin metrics
(caster/KPI/scrub/trail) read NaN by design.
"""

from dataclasses import dataclass

import numpy as np

Z_UP = np.array([0.0, 0.0, 1.0])


def _unit(v):
    return v / np.linalg.norm(v)


@dataclass
class TrailingArmPoints:
    """Hardpoints for one trailing-arm corner (vehicle coords, mm, left
    side, origin at this axle like every other corner in the tool)."""

    pivot_inner: np.ndarray   # chassis bushing, inboard end of the axis
    pivot_outer: np.ndarray   # chassis bushing, outboard end
    wheel_center: np.ndarray
    shock_inner: np.ndarray   # chassis end
    shock_outer: np.ndarray   # ON THE ARM (rotates with it)
    tire_radius: float
    tire_width: float = 150.0
    static_camber_deg: float = 0.0
    static_toe_deg: float = 0.0

    POINT_ATTRS = ["pivot_inner", "pivot_outer", "wheel_center",
                   "shock_inner", "shock_outer"]

    LABELS = {
        "pivot_inner": "Pivot bushing — inboard",
        "pivot_outer": "Pivot bushing — outboard",
        "wheel_center": "Wheel centre",
        "shock_inner": "Shock — chassis",
        "shock_outer": "Shock — on arm",
    }


@dataclass
class TrailingArmState:
    """Solved pose. Duck-type compatible with CornerState where it makes
    sense; ubj/lbj/tro stay None (no knuckle) so kingpin metrics NaN out."""

    travel: float
    steer: float
    arm_angle: float          # rotation from static about the pivot axis (rad)
    wheel_center: np.ndarray
    spindle: np.ndarray
    contact_patch: np.ndarray
    shock_outer: np.ndarray
    shock_length: float
    ubj = None
    lbj = None
    tro = None


class TrailingArmSolver:
    """Forward solver: same public contract as DoubleWishboneSolver."""

    def __init__(self, hp: TrailingArmPoints):
        self.hp = hp
        self._origin = hp.pivot_inner
        self._axis = _unit(hp.pivot_outer - hp.pivot_inner)
        # Static spindle from the stated alignment (same construction and
        # sign conventions as the double wishbone: see solver.py).
        # EXACT form (v1.17, as the double wishbone and C-hub use): build
        # the spindle so BOTH camber and toe read back exactly for any
        # pair. The old Rz(-tau)Rx(-gamma) composition below reproduced toe
        # exactly but coupled a gamma*tau^2/2 error into camber -- 5.3e-06
        # deg here against the DW's 1e-15.
        gamma = np.radians(hp.static_camber_deg)
        tau = np.radians(hp.static_toe_deg)
        s = np.array([np.tan(tau), 1.0, -np.tan(gamma)])
        self._spindle0 = s / np.linalg.norm(s)
        # Decompose the wheel centre's circle about the axis (identical
        # trick to the double wishbone's UBJ): wc(a) = c + u cos a + v sin a
        rel = hp.wheel_center - self._origin
        c = self._origin + self._axis * np.dot(self._axis, rel)
        self._wc_center = c
        self._wc_u = hp.wheel_center - c
        self._wc_v = np.cross(self._axis, self._wc_u)
        self._wc_z0 = hp.wheel_center[2]

    # -- rigid-body helpers ---------------------------------------------
    def _rot(self, p, a):
        """Rodrigues rotation of point p about the pivot axis by angle a."""
        r = p - self._origin
        ax = self._axis
        return (self._origin + r * np.cos(a) + np.cross(ax, r) * np.sin(a)
                + ax * np.dot(ax, r) * (1.0 - np.cos(a)))

    def _rot_dir(self, d, a):
        """Rotate a direction vector (no origin) about the axis."""
        ax = self._axis
        return (d * np.cos(a) + np.cross(ax, d) * np.sin(a)
                + ax * np.dot(ax, d) * (1.0 - np.cos(a)))

    def solve_at_angle(self, a: float, steer: float = 0.0) -> TrailingArmState:
        hp = self.hp
        wc = self._wc_center + self._wc_u * np.cos(a) + self._wc_v * np.sin(a)
        spindle = self._rot_dir(self._spindle0, a)
        shock_outer = self._rot(hp.shock_outer, a)
        forward = _unit(np.cross(spindle, Z_UP))
        down = _unit(np.cross(spindle, forward))
        return TrailingArmState(
            travel=wc[2] - self._wc_z0,
            steer=steer,
            arm_angle=a,
            wheel_center=wc,
            spindle=spindle,
            contact_patch=wc + hp.tire_radius * down,
            shock_outer=shock_outer,
            shock_length=float(np.linalg.norm(shock_outer - hp.shock_inner)),
        )

    # -- public contract ---------------------------------------------------
    def solve(self, travel: float, steer: float = 0.0) -> TrailingArmState:
        """Closed form: wc_z(a) = c_z + u_z cos a + v_z sin a = target is
        A cos a + B sin a = C; take the root nearest static (a = 0)."""
        target = self._wc_z0 + travel
        A, B = self._wc_u[2], self._wc_v[2]
        C = target - self._wc_center[2]
        R = np.hypot(A, B)
        if R < 1e-12 or abs(C) > R:
            raise ValueError(
                f"travel {travel:+.1f} mm is outside the trailing arm's reach")
        phi = np.arctan2(B, A)
        delta = np.arccos(np.clip(C / R, -1.0, 1.0))
        a = min(phi + delta, phi - delta, key=abs)
        return self.solve_at_angle(a, steer)

    def walk_travels(self, travels, steer: float = 0.0):
        return [self.solve(t, steer) for t in travels]

    # -- exact instant centres (used by metrics' dispatch) ----------------
    def exact_front_view_ic(self, state):
        """The rigid body rotates about the pivot AXIS, so the front-view
        IC is exactly where that axis pierces the plane x = x_wheel.
        A purely lateral axis never pierces it: IC at infinity (pure
        trailing arm -> roll centre on the ground)."""
        return self._axis_pierce(state, axis_index=0,
                                 plane_at=state.wheel_center[0], keep=(1, 2))

    def exact_side_view_ic(self, state):
        """Side view: the axis pierced through the plane y = y_wheel gives
        the side-view IC — this is what sets wheel recession and the
        anti-squat geometry of a trailing arm."""
        return self._axis_pierce(state, axis_index=1,
                                 plane_at=state.wheel_center[1], keep=(0, 2))

    def _axis_pierce(self, state, axis_index, plane_at, keep):
        d = self._axis[axis_index]
        if abs(d) < 1e-9:
            return None                       # axis parallel to the plane
        t = (plane_at - self._origin[axis_index]) / d
        p = self._origin + t * self._axis
        return float(p[keep[0]]), float(p[keep[1]])


def seed_trailing_arm(sv) -> TrailingArmPoints:
    """Heuristic starting geometry for a (semi-)trailing-arm corner from
    the same SetupVariables the double wishbone uses. The pivot axis is
    laid LATERAL (pure trailing) ahead of the axle at roughly frame-rail
    height — skew the pivots afterwards for semi-trailing camber/toe
    behaviour. The shock mount is slid along the arm with a small secant
    tune until the static motion ratio hits the goal (same approach as
    the double-wishbone seed). Steering-related setup fields are ignored
    (no knuckle). Raises ValueError if the arm cannot articulate the
    shock-stroke travel budget."""
    fhw = (sv.frame_half_width if sv.frame_half_width is not None
           else 0.07 * sv.track_width)
    wc = np.array([0.0, sv.track_width / 2.0, sv.tire_radius])
    # arm length ~ a third of the wheelbase, pivots at rail height ahead
    arm_x = 0.30 * sv.wheelbase
    pivot_z = sv.ride_height + 0.10 * sv.tire_radius
    pivot_inner = np.array([arm_x, 0.8 * fhw, pivot_z])
    pivot_outer = np.array([arm_x, 2.2 * fhw, pivot_z])

    stroke = sv.shock_max_length - sv.shock_min_length
    l_ride = (sv.shock_length_at_ride
              if sv.shock_length_at_ride is not None
              else sv.shock_min_length + 0.7 * stroke)
    bump = (l_ride - sv.shock_min_length) / sv.motion_ratio_goal
    droop = (sv.shock_max_length - l_ride) / sv.motion_ratio_goal

    mid = (pivot_inner + pivot_outer) / 2.0
    shock_dir = np.array([0.0, -np.cos(np.radians(70.0)),
                          np.sin(np.radians(70.0))])

    def build(p):
        outer = mid + p * (wc - mid)
        return TrailingArmPoints(
            pivot_inner=pivot_inner, pivot_outer=pivot_outer,
            wheel_center=wc,
            shock_inner=outer + l_ride * shock_dir, shock_outer=outer,
            tire_radius=sv.tire_radius, tire_width=sv.tire_width,
            static_camber_deg=sv.static_camber_deg,
            static_toe_deg=sv.static_toe_deg)

    def mr(p):
        s = TrailingArmSolver(build(p))
        # MR = SHOCK travel / WHEEL travel (< 1). Shock shortens in
        # bump, so droop-minus-bump keeps it positive.
        return (s.solve(-1.0).shock_length
                - s.solve(1.0).shock_length) / 2.0

    # secant on the mount fraction toward the motion-ratio goal
    p0, p1 = sv.motion_ratio_goal, sv.motion_ratio_goal + 0.05
    f0, f1 = mr(p0) - sv.motion_ratio_goal, mr(p1) - sv.motion_ratio_goal
    for _ in range(10):
        if abs(f1) < 1e-3 or abs(f1 - f0) < 1e-12:
            break
        p0, p1, f0 = p1, p1 - f1 * (p1 - p0) / (f1 - f0), f1
        p1 = float(np.clip(p1, 0.2, 0.98))
        f1 = mr(p1) - sv.motion_ratio_goal
    hp = build(p1)
    solver = TrailingArmSolver(hp)
    solver.solve(bump)
    solver.solve(-droop)
    from .seed import apply_seed_offset
    apply_seed_offset(hp, sv)   # rigid shift onto the Onshape frame origin
    return hp
