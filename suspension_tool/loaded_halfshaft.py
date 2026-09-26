"""Loaded-halfshaft rear suspension.

The halfshaft is not just a driveline component here — it is a
STRUCTURAL LATERAL LINK. The corner is:

  * an UPPER control arm on a chassis bushing axis, carrying the shock;
  * the knuckle mounted to that arm's outer end through a BUSHING (a
    revolute pin — one rotational freedom, not a ball joint);
  * the HALFSHAFT, a fixed-length link from the diff output flange
    (chassis) to the outer CV in the hub, closing the loop and locating
    the bottom of the knuckle laterally.

Because the knuckle-to-arm joint is a pin (not a ball), no toe link is
needed: the pin axis constrains toe, and the shaft length constrains the
knuckle's swing about the pin — camber. Count the freedoms: arm 1, pin
+1, shaft length -1 = one degree of freedom, the wheel travel.

Same closed-form chain as the C-hub front (see hinge_carrier.py) minus
the steering block. A loaded shaft CANNOT plunge — its length is the
closure constraint — so the halfshaft readout's plunge channel reads
exactly zero for this type (a built-in sanity check) while the CV angles
remain the real numbers to watch.

No steering knuckle: solve() accepts the interface's steer argument and
ignores it; kingpin metrics read NaN by design (like the trailing arm).

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
class LoadedHalfshaftPoints:
    """Hardpoints for one loaded-halfshaft rear corner (tool frame, mm,
    LEFT side, static pose — the shaft length is measured from these)."""

    arm_inner_front: np.ndarray   # upper-arm chassis bushing, forward
    arm_inner_rear: np.ndarray    # upper-arm chassis bushing, rearward
    hinge_front: np.ndarray       # knuckle bushing pin, forward end (ON the arm)
    hinge_rear: np.ndarray        # knuckle bushing pin, rearward end (ON the arm)
    hs_inner: np.ndarray          # halfshaft inner CV — diff output flange (chassis)
    hs_outer: np.ndarray          # halfshaft outer CV — in the hub (ON the knuckle)
    wheel_center: np.ndarray      # in the knuckle
    shock_inner: np.ndarray       # chassis end
    shock_outer: np.ndarray       # ON THE UPPER ARM
    tire_radius: float
    tire_width: float = 150.0
    static_camber_deg: float = 0.0
    static_toe_deg: float = 0.0

    POINT_ATTRS = ["arm_inner_front", "arm_inner_rear",
                   "hinge_front", "hinge_rear",
                   "hs_inner", "hs_outer",
                   "wheel_center", "shock_inner", "shock_outer"]

    LABELS = {
        "arm_inner_front": "Upper arm bushing — front",
        "arm_inner_rear": "Upper arm bushing — rear",
        "hinge_front": "Knuckle bushing pin — front",
        "hinge_rear": "Knuckle bushing pin — rear",
        "hs_inner": "Halfshaft inner CV (diff, LOADED)",
        "hs_outer": "Halfshaft outer CV (hub, LOADED)",
        "wheel_center": "Wheel centre",
        "shock_inner": "Shock — chassis",
        "shock_outer": "Shock — on upper arm",
    }


@dataclass
class LoadedHalfshaftState:
    """Solved pose. No steering knuckle: ubj/lbj/tro stay None so kingpin
    metrics NaN out (the trailing-arm convention). hs_outer is the POSED
    outer CV — halfshaft.py prefers it over the wheel centre, so the CV
    readouts follow the structural shaft exactly."""

    travel: float
    steer: float
    arm_angle: float          # a, rad
    knuckle_angle: float      # b, rad (about the bushing pin)
    wheel_center: np.ndarray
    spindle: np.ndarray
    contact_patch: np.ndarray
    shock_outer: np.ndarray
    shock_length: float
    hs_outer: np.ndarray      # posed outer CV (the closing-link end)
    hinge_front: np.ndarray
    hinge_rear: np.ndarray
    ubj = None
    lbj = None
    tro = None


class LoadedHalfshaftSolver:
    """Forward solver: same public contract as DoubleWishboneSolver."""

    def __init__(self, hp: LoadedHalfshaftPoints):
        self.hp = hp
        self._arm_origin = hp.arm_inner_front
        self._arm_axis = _unit(hp.arm_inner_rear - hp.arm_inner_front)
        # The SHAFT LENGTH is the closure constraint, measured at static.
        self._shaft_len = float(np.linalg.norm(hp.hs_outer - hp.hs_inner))
        gamma = np.radians(hp.static_camber_deg)
        tau = np.radians(hp.static_toe_deg)
        s = np.array([np.tan(tau), 1.0, -np.tan(gamma)])
        self._spindle0 = s / np.linalg.norm(s)
        self._wc_z0 = float(hp.wheel_center[2])
        self._last_a = 0.0

    def _arm_pt(self, p, a):
        return rotate_about(self._arm_origin, self._arm_axis, p, a)

    def solve_at_arm_angle(self, a: float,
                           steer: float = 0.0) -> LoadedHalfshaftState:
        hp = self.hp
        hf = self._arm_pt(hp.hinge_front, a)
        hr = self._arm_pt(hp.hinge_rear, a)
        axis = _unit(hr - hf)
        # Knuckle closure: swing about the bushing pin until the shaft
        # spans exactly its fixed length to the diff flange.
        b = closing_angle(hf, axis, self._arm_pt(hp.hs_outer, a),
                          hp.hs_inner, self._shaft_len)

        def knuckle_pt(p0):
            return rotate_about(hf, axis, self._arm_pt(p0, a), b)
        wc = knuckle_pt(hp.wheel_center)
        spindle = rotate_dir(
            axis, rotate_dir(self._arm_axis, self._spindle0, a), b)
        forward = _unit(np.cross(spindle, Z_UP))
        down = _unit(np.cross(spindle, forward))
        shock_outer = self._arm_pt(hp.shock_outer, a)
        return LoadedHalfshaftState(
            travel=wc[2] - self._wc_z0,
            steer=steer,                      # carried, ignored (rear axle)
            arm_angle=a, knuckle_angle=b,
            wheel_center=wc,
            spindle=spindle,
            contact_patch=wc + hp.tire_radius * down,
            shock_outer=shock_outer,
            shock_length=float(np.linalg.norm(shock_outer - hp.shock_inner)),
            hs_outer=knuckle_pt(hp.hs_outer),
            hinge_front=hf, hinge_rear=hr,
        )

    # -- public contract ------------------------------------------------
    def solve(self, travel: float, steer: float = 0.0) -> LoadedHalfshaftState:
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


def seed_loaded_halfshaft(sv) -> LoadedHalfshaftPoints:
    """Heuristic starting geometry for a loaded-halfshaft rear corner from
    the same SetupVariables as every other seed.

    Placement logic (pre-offset frame: ground z = 0, wheel at y = track/2):
      * outer CV on the tire's spin axis, hub_offset inboard of the wheel
        centre (the CV output line coincident with the tire centreline —
        the tool's standing halfshaft convention);
      * diff flange near the centreline at a height that runs the shaft
        LEVEL AT MID-TRAVEL (the plunge lesson applied to CV angles: a
        loaded shaft cannot plunge, so level-at-mid minimises its CV-angle
        swing over a bump-biased range);
      * knuckle bushing pin fore-aft, above and slightly inboard of the
        wheel centre (the knuckle spans pin -> hub);
      * upper arm from the frame rail out to the pin, shock on the arm,
        mount fraction secant-tuned to the motion-ratio goal.

    Raises ValueError if the corner cannot articulate the travel budget."""
    fhw = (sv.frame_half_width if sv.frame_half_width is not None
           else 0.07 * sv.track_width)
    r = sv.tire_radius
    wc = np.array([0.0, sv.track_width / 2.0, r])

    stroke = sv.shock_max_length - sv.shock_min_length
    l_ride = (sv.shock_length_at_ride
              if sv.shock_length_at_ride is not None
              else sv.shock_min_length + 0.7 * stroke)
    bump = (l_ride - sv.shock_min_length) / sv.motion_ratio_goal
    droop = (sv.shock_max_length - l_ride) / sv.motion_ratio_goal

    # Static spindle (exact form) to place the outer CV ON the spin axis.
    gamma = np.radians(sv.static_camber_deg)
    tau = np.radians(sv.static_toe_deg)
    s = np.array([np.tan(tau), 1.0, -np.tan(gamma)])
    s = s / np.linalg.norm(s)
    hs_outer = wc - sv.hub_offset * s
    # Diff flange: near the centreline, height set so the shaft runs level
    # at MID-travel of the (usually bump-biased) range.
    # Lateral station = the frame rail (fhw), which is where a diff output
    # actually sits and scales with the car. A fixed 120 mm was fine at Baja
    # size but put the flange out at the wheel on a 1/10 RC buggy.
    hs_inner = np.array([0.0, fhw, hs_outer[2] + (bump - droop) / 2.0])

    # Knuckle bushing pin: fore-aft, above the hub.
    pin_half = 0.16 * r
    pin_c = np.array([0.0, wc[1] - sv.hub_offset, wc[2] + 0.40 * r])
    hinge_front = pin_c + np.array([pin_half, 0.0, 0.0])
    hinge_rear = pin_c - np.array([pin_half, 0.0, 0.0])

    # Upper arm on the rail, level with the pin.
    spread = 0.55 * (pin_c[1] - fhw)
    arm_inner_front = np.array([+0.5 * spread, fhw, pin_c[2]])
    arm_inner_rear = np.array([-0.5 * spread, fhw, pin_c[2]])

    arm_mid = (arm_inner_front + arm_inner_rear) / 2.0
    shock_dir = np.array([0.0, -np.cos(np.radians(70.0)),
                          np.sin(np.radians(70.0))])

    def build(p: float) -> LoadedHalfshaftPoints:
        outer = arm_mid + p * (pin_c - arm_mid)
        return LoadedHalfshaftPoints(
            arm_inner_front=arm_inner_front, arm_inner_rear=arm_inner_rear,
            hinge_front=hinge_front, hinge_rear=hinge_rear,
            hs_inner=hs_inner, hs_outer=hs_outer,
            wheel_center=wc,
            shock_inner=outer + l_ride * shock_dir, shock_outer=outer,
            tire_radius=r, tire_width=sv.tire_width,
            static_camber_deg=sv.static_camber_deg,
            static_toe_deg=sv.static_toe_deg)

    def mr(p: float) -> float:
        s2 = LoadedHalfshaftSolver(build(p))
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
    solver = LoadedHalfshaftSolver(hp)
    solver.solve(bump)               # articulation check: raises if short
    solver.solve(-droop)
    from .seed import apply_seed_offset
    apply_seed_offset(hp, sv)        # rigid shift onto the Onshape origin
    return hp
