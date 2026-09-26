"""Multilink (5-link) kinematics.

Five rigid links, each a fixed-length rod from a chassis point to an
upright point, fully locate the upright: the upright has 6 degrees of
freedom, the five rod lengths remove five, and the requested wheel-centre
height removes the sixth. Unlike the double wishbone there is no closed
form — the upright pose is found numerically (scipy least_squares on the
6 pose parameters against the 6 constraints), warm-started from the
previous solution so sweeps stay fast.

Link 5 is the TOE LINK by convention: its chassis end rides the steering
rack, so `steer` displaces it laterally exactly like the double
wishbone's tie rod (pass 0 for a rear axle with a fixed-length toe link).

The upright pose is parameterised as a rotation vector (Rodrigues) plus a
translation of the wheel centre. Kingpin metrics (caster/KPI/scrub/trail)
read NaN — a multilink has only a *virtual* steering axis, out of scope
for now — while camber/toe/roll-centre come from the generic paths in
metrics.py (the roll centre via the numeric instant-centre method, which
is exactly how Adams handles linkages the 2D construction can't).

A double wishbone IS a 5-link (each A-arm = two rods meeting at the ball
joint, plus the tie rod), which the tests exploit: the multilink solver
fed those ten points must reproduce the double-wishbone solver's results.
"""

from dataclasses import dataclass

import dataclasses

import numpy as np
from scipy.optimize import least_squares

Z_UP = np.array([0.0, 0.0, 1.0])


def _unit(v):
    return v / np.linalg.norm(v)


def _closest_point_between_lines(l1, l2):
    """Midpoint of the common perpendicular of two 3D lines, each given
    as (point, direction) — the exact intersection when they meet. None
    when near-parallel (no meaningful virtual joint)."""
    p1, d1 = l1
    p2, d2 = l2
    d1 = d1 / np.linalg.norm(d1)
    d2 = d2 / np.linalg.norm(d2)
    cross = np.cross(d1, d2)
    denom = float(np.dot(cross, cross))
    if denom < 1e-12:
        return None
    r = p2 - p1
    t1 = float(np.dot(np.cross(r, d2), cross)) / denom
    t2 = float(np.dot(np.cross(r, d1), cross)) / denom
    return ((p1 + t1 * d1) + (p2 + t2 * d2)) / 2.0


def _rodrigues(rvec: np.ndarray) -> np.ndarray:
    """Rotation matrix from a rotation vector (angle * unit axis)."""
    angle = np.linalg.norm(rvec)
    if angle < 1e-12:
        return np.eye(3)
    ax = rvec / angle
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


@dataclass
class MultilinkPoints:
    """Hardpoints for one 5-link corner. linkN_inner on the chassis,
    linkN_outer on the upright. Link 5 = toe link (steer input)."""

    link1_inner: np.ndarray
    link1_outer: np.ndarray
    link2_inner: np.ndarray
    link2_outer: np.ndarray
    link3_inner: np.ndarray
    link3_outer: np.ndarray
    link4_inner: np.ndarray
    link4_outer: np.ndarray
    link5_inner: np.ndarray   # toe link, chassis/rack end
    link5_outer: np.ndarray
    wheel_center: np.ndarray
    shock_inner: np.ndarray   # chassis end
    shock_outer: np.ndarray   # ON THE UPRIGHT (rides with the pose)
    tire_radius: float
    tire_width: float = 150.0
    static_camber_deg: float = 0.0
    static_toe_deg: float = 0.0

    POINT_ATTRS = ["link1_inner", "link1_outer", "link2_inner", "link2_outer",
                   "link3_inner", "link3_outer", "link4_inner", "link4_outer",
                   "link5_inner", "link5_outer", "wheel_center",
                   "shock_inner", "shock_outer"]

    # Human labels for the GUI table (the numbering convention in words).
    LABELS = {
        "link1_inner": "Upper link A — chassis",
        "link1_outer": "Upper link A — upright",
        "link2_inner": "Upper link B — chassis",
        "link2_outer": "Upper link B — upright",
        "link3_inner": "Lower link A — chassis",
        "link3_outer": "Lower link A — upright",
        "link4_inner": "Lower link B — chassis",
        "link4_outer": "Lower link B — upright",
        "link5_inner": "Toe link — chassis/rack",
        "link5_outer": "Toe link — upright",
        "wheel_center": "Wheel centre",
        "shock_inner": "Shock — chassis",
        "shock_outer": "Shock — upright",
    }


@dataclass
class MultilinkState:
    travel: float
    steer: float
    pose: np.ndarray          # the 6 solved pose parameters (rvec, dxyz)
    wheel_center: np.ndarray
    spindle: np.ndarray
    contact_patch: np.ndarray
    shock_outer: np.ndarray
    shock_length: float
    outer_points: dict        # solved upright-side link points, for drawing
    ubj = None
    lbj = None
    tro = None


class MultilinkSolver:
    """Forward solver: same public contract as DoubleWishboneSolver."""

    def __init__(self, hp: MultilinkPoints):
        self.hp = hp
        self._wc0 = hp.wheel_center
        self._outers0 = {k: getattr(hp, f"link{k}_outer") for k in range(1, 6)}
        self._inners0 = {k: getattr(hp, f"link{k}_inner") for k in range(1, 6)}
        self._lengths = {k: np.linalg.norm(self._outers0[k] - self._inners0[k])
                         for k in range(1, 6)}
        gamma = np.radians(hp.static_camber_deg)
        tau = np.radians(hp.static_toe_deg)
        s = np.array([0.0, np.cos(gamma), -np.sin(gamma)])
        rz = np.array([[np.cos(tau), np.sin(tau), 0.0],
                       [-np.sin(tau), np.cos(tau), 0.0],
                       [0.0, 0.0, 1.0]])
        self._spindle0 = rz @ s
        self._wc_z0 = hp.wheel_center[2]
        self._last_pose = np.zeros(6)   # warm start (static is all zeros)

    # ------------------------------------------------------------------
    def _upright_point(self, p0, rot, disp):
        """Static upright point -> posed point (rotate about the wheel
        centre, then translate the wheel centre by disp)."""
        return rot @ (p0 - self._wc0) + self._wc0 + disp

    def _residuals(self, pose, target_z, steer):
        rot = _rodrigues(pose[:3])
        disp = pose[3:]
        res = np.empty(6)
        for i, k in enumerate(range(1, 6)):
            inner = self._inners0[k]
            if k == 5:
                inner = inner + np.array([0.0, steer, 0.0])   # rack input
            outer = self._upright_point(self._outers0[k], rot, disp)
            res[i] = np.linalg.norm(outer - inner) - self._lengths[k]
        res[5] = (self._wc0[2] + disp[2]) - target_z
        return res

    def solve(self, travel: float, steer: float = 0.0) -> MultilinkState:
        target_z = self._wc_z0 + travel
        result = least_squares(
            self._residuals, self._last_pose, args=(target_z, steer),
            method="lm", xtol=1e-12, ftol=1e-12)
        if np.max(np.abs(result.fun)) > 1e-6:
            # retry cold before giving up (warm start may be on a bad branch)
            result = least_squares(
                self._residuals, np.zeros(6), args=(target_z, steer),
                method="lm", xtol=1e-12, ftol=1e-12)
            if np.max(np.abs(result.fun)) > 1e-6:
                raise ValueError(
                    f"multilink cannot articulate to travel {travel:+.1f} mm")
        pose = result.x
        self._last_pose = pose.copy()
        rot = _rodrigues(pose[:3])
        disp = pose[3:]
        wc = self._wc0 + disp
        spindle = rot @ self._spindle0
        forward = _unit(np.cross(spindle, Z_UP))
        down = _unit(np.cross(spindle, forward))
        shock_outer = self._upright_point(self.hp.shock_outer, rot, disp)
        return MultilinkState(
            travel=wc[2] - self._wc_z0,
            steer=steer,
            pose=pose,
            wheel_center=wc,
            spindle=spindle,
            contact_patch=wc + self.hp.tire_radius * down,
            shock_outer=shock_outer,
            shock_length=float(np.linalg.norm(shock_outer - self.hp.shock_inner)),
            outer_points={k: self._upright_point(self._outers0[k], rot, disp)
                          for k in range(1, 6)},
        )

    def virtual_kingpin(self, state):
        """The multilink's VIRTUAL steering axis, by the standard
        virtual-ball-joint construction: extend the UPPER link pair
        (links 1 & 2) as infinite lines and take their closest point —
        the virtual upper ball joint; same for the LOWER pair (3 & 4).
        The axis joins the two. For a double wishbone expressed as five
        links each pair intersects exactly AT the ball joint, so this
        reproduces the physical UBJ-LBJ axis — the tests' validation.

        Convention (also used by the seed): links 1-2 are the upper pair,
        3-4 the lower pair, 5 the toe link. Returns (point, unit direction
        with d_z > 0), or None if a pair is near-parallel."""
        def line(k):
            a = self._inners0[k]
            return a, state.outer_points[k] - a
        upper = _closest_point_between_lines(line(1), line(2))
        lower = _closest_point_between_lines(line(3), line(4))
        if upper is None or lower is None:
            return None
        d = upper - lower
        n = np.linalg.norm(d)
        if n < 1e-9:
            return None
        d = d / n
        if d[2] < 0.0:
            d = -d
        return lower, d

    def walk_travels(self, travels, steer: float = 0.0):
        """The warm start makes ordered walks cheap; solve() already reuses
        the previous pose, so just order the work outward from static."""
        travels = np.asarray(travels, dtype=float)
        states = [None] * len(travels)
        self._last_pose = np.zeros(6)
        for idx in np.argsort(np.abs(travels), kind="stable"):
            states[idx] = self.solve(travels[idx], steer)
        return states


def from_double_wishbone(hp) -> MultilinkPoints:
    """A double wishbone expressed as five rods: two per A-arm meeting at
    the ball joint, plus the tie rod as the toe link. Used both as the
    multilink SEED (edit link ends freely from there — that is the whole
    point of a multilink) and as the solver cross-validation case. The
    shock is re-homed onto the upright at the lower ball joint."""
    return MultilinkPoints(
        link1_inner=hp.uca_inner_front, link1_outer=hp.uca_outer,
        link2_inner=hp.uca_inner_rear, link2_outer=hp.uca_outer,
        link3_inner=hp.lca_inner_front, link3_outer=hp.lca_outer,
        link4_inner=hp.lca_inner_rear, link4_outer=hp.lca_outer,
        link5_inner=hp.tierod_inner, link5_outer=hp.tierod_outer,
        wheel_center=hp.wheel_center,
        shock_inner=hp.shock_inner,
        shock_outer=hp.lca_outer,
        tire_radius=hp.tire_radius, tire_width=hp.tire_width,
        static_camber_deg=hp.static_camber_deg,
        static_toe_deg=hp.static_toe_deg)


def seed_multilink(sv) -> MultilinkPoints:
    """Multilink seed = the 5-link equivalent of the double-wishbone seed
    for the same setup variables (a genuinely good starting point: split
    the A-arms and start moving individual rod ends).

    The shock needs its own placement here. from_double_wishbone re-homes
    it onto the upright at the lower ball joint, because a multilink has no
    control arm to mount on -- but the double wishbone's shock was tuned
    ALONG that arm, so simply inheriting it ignored two of the setup
    variables outright (v1.33 fix: installed length read 653 mm against a
    requested 517.65, and MR 0.600 against a 0.541 goal).

    With the outer eye pinned to the upright the only freedom left is the
    CHASSIS end, and it happens to control both wanted quantities cleanly:
    the motion ratio is the shock axis dotted with the outer eye's velocity,
    so the axis DIRECTION sets MR, and the distance along it sets the
    installed length. One secant on the axis inclination, then a placement.
    """
    import numpy as np
    from .seed import generate_seed
    dw, _ = generate_seed(sv)
    hp = from_double_wishbone(dw)

    stroke = sv.shock_max_length - sv.shock_min_length
    l_ride = (sv.shock_length_at_ride if sv.shock_length_at_ride is not None
              else sv.shock_min_length + 0.7 * stroke)

    # Outer-eye velocity per mm of wheel travel (the upright's motion).
    h = 2.0
    s0 = MultilinkSolver(hp)
    v = (s0.solve(h).shock_outer - s0.solve(-h).shock_outer) / (2.0 * h)

    def axis_of(theta_deg):
        """Unit vector from the outer eye toward the chassis: up and
        inboard by `theta` from vertical (the left corner is at +Y)."""
        t = np.radians(theta_deg)
        return np.array([0.0, -np.sin(t), np.cos(t)])

    def mr_of(theta_deg):
        """Shock travel per mm of wheel travel. With the chassis eye fixed,
        d(length)/d(travel) = -axis . v, and the tool's MR is the negative
        of that (positive while the shock shortens in bump) -- so MR is
        just the projection of the outer eye's velocity on the shock axis.
        """
        return float(np.dot(axis_of(theta_deg), v))

    # MR is NOT monotone in theta: it peaks where the axis lines up with the
    # velocity (near vertical) and falls off either side, so a bracketed
    # bisection is the wrong tool. The range is one degree-wide scan, which
    # is exact enough and cannot land on the wrong branch.
    grid = np.linspace(-89.0, 89.0, 3561)
    theta = float(grid[np.argmin(np.abs(
        np.array([mr_of(t) for t in grid]) - sv.motion_ratio_goal))])
    return dataclasses.replace(
        hp, shock_inner=hp.shock_outer + l_ride * axis_of(theta))
