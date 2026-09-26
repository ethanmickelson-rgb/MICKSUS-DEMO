"""Steering effort: what the rack has to push, and what the driver feels.

WHY THIS EXISTS: sizing the bolted joint that pins onto the rack, and
checking the rack itself. Those are structural questions, so the number
that matters is the AXIAL FORCE IN THE RACK, not a comfort figure.

THE CHAIN, and where each link comes from:

  1. A force at the tire's contact patch.
  2. Its moment about the KINGPIN AXIS. Computed exactly, in 3D:
         M = k . [ (P - A) x F ]
     with k the unit kingpin axis, A a point on it, P the point the force
     acts through and F the contact-patch force. Pneumatic trail moves P
     rearward rather than adding a separate scalar moment -- see
     `kingpin_moment` for why that matters.
     This is deliberately NOT the textbook `Fy * trail + Fz * scrub`
     decomposition. That form assumes the kingpin is near vertical and
     the force horizontal, and it quietly drops the cross terms. The
     cross product needs no such assumption and the tool already knows
     every point in it. The classical form is reproduced by it as a
     limiting case, which the tests check.
  3. The tie-rod force that balances it. The moment arm is again exact:
         r_eff = k . [ (T - A) x u ]
     with T the outer tie-rod ball and u the tie rod's unit direction.
     This is NOT the same as the scalar "steering arm length", which is
     a perpendicular distance and ignores that the tie rod is generally
     not perpendicular to the kingpin. On a real front corner the two differ
     by 16% (47.7 mm against 57.1 mm), all of it in the unconservative
     direction, so the distinction is worth the extra algebra.
  4. The rack force: the tie-rod force resolved onto the rack axis.
  5. Steering-wheel torque, from rack travel per turn.

LOAD CASE: cornering at the grip limit. Lateral force at each front
contact patch from the load-sensitive tire model, using the wheel loads
the dynamics sheet computes (so lateral load transfer is already in
them). Both front wheels contribute to the rack.

ACCURACY: checked against virtual work through the solver -- perturb the
rack, measure how far the knuckle turns about its kingpin, and confirm
F_rack * d(rack) == M_kingpin * d(theta). That route shares no code with
the formulas above. They agree to 1.2e-4, and the residual is not
numerical: it converges to a constant as the difference step shrinks.
It is STEERING JACK. The arm treats the knuckle as rotating about a
fixed kingpin, but steering about an inclined kingpin tries to lift the
corner, so the control arm rotates slightly to hold wheel-centre height
and the ball joints translate a little. Irrelevant for sizing a bolt.

Two honest limitations, both of which make this an UNDER-estimate:

  * PNEUMATIC TRAIL is zero by default. At the limit the tire is
    saturated and its pneumatic trail has collapsed, which is the
    standard assumption -- but the aligning moment PEAKS BELOW the
    limit, where trail is still large and lateral force is already most
    of the way up. `pneumatic_trail_mm` exists so that peak can be
    covered; feed it a few percent of the tire radius to see it.
  * PARKING EFFORT is not modelled here. Turning the wheel stationary
    twists the tire in place against friction over the whole contact
    patch and, on a manual rack, usually governs. It needs tire
    pressure, which nothing in the tool holds yet.
"""

from dataclasses import dataclass

import numpy as np

IN = 25.4
LBF_PER_N = 0.22480894387096
N_PER_LBF = 4.4482216152605


def kingpin_axis(state) -> np.ndarray:
    """Unit vector along the kingpin, lower ball joint to upper."""
    a = np.asarray(state.lbj, float)
    b = np.asarray(state.ubj, float)
    d = b - a
    n = float(np.linalg.norm(d))
    if n < 1e-9:
        raise ValueError("kingpin has zero length")
    return d / n


def wheel_forward(state) -> np.ndarray:
    """Unit vector along the wheel's heading, in the ground plane."""
    s = np.asarray(state.spindle, float)
    f = np.cross(s, np.array([0.0, 0.0, 1.0]))
    n = float(np.linalg.norm(f))
    if n < 1e-9:                       # spindle vertical: degenerate
        return np.array([1.0, 0.0, 0.0])
    return f / n


def kingpin_moment(state, force, pneumatic_trail: float = 0.0) -> float:
    """Moment about the kingpin axis from a force at the contact patch.

    `force` is (Fx, Fy, Fz) in the tool frame, in any consistent force
    unit; the result carries that unit times millimetres.

    PNEUMATIC TRAIL is handled GEOMETRICALLY -- the force is applied
    that far BEHIND the contact-patch centre, along the wheel's heading
    -- rather than as a separate scalar Mz added on. Same physics, but
    the sign cannot be got wrong: pneumatic trail simply extends the
    mechanical trail, and the total is what the lateral force acts
    through. Adding a scalar moment instead means reasoning about its
    sense relative to a cross product, which is how it WAS wrong first
    time round: the term subtracted, and a trail made the rack lighter.
    """
    k = kingpin_axis(state)
    point = np.asarray(state.contact_patch, float)
    if pneumatic_trail:
        point = point - float(pneumatic_trail) * wheel_forward(state)
    arm = point - np.asarray(state.lbj, float)
    return float(np.dot(k, np.cross(arm, np.asarray(force, float))))


def tierod_effective_arm(state, hp) -> float:
    """Signed moment arm (mm) of the tie rod about the kingpin axis.

    A unit force along the tie rod makes this much moment about the
    kingpin. Sign follows the tie rod's stored direction; callers take
    the magnitude for a force check.
    """
    k = kingpin_axis(state)
    outer = np.asarray(state.tro, float)
    inner = np.asarray(getattr(state, "tierod_inner_cur", hp.tierod_inner),
                       float)
    d = outer - inner
    n = float(np.linalg.norm(d))
    if n < 1e-9:
        raise ValueError("tie rod has zero length")
    u = d / n
    arm = outer - np.asarray(state.lbj, float)
    return float(np.dot(k, np.cross(arm, u)))


def rack_axis_fraction(state, hp) -> float:
    """How much of a tie-rod force lands on the rack axis.

    The rack slides laterally, so only the tie rod's lateral component
    pushes it; the rest is reacted by the rack housing.
    """
    outer = np.asarray(state.tro, float)
    inner = np.asarray(getattr(state, "tierod_inner_cur", hp.tierod_inner),
                       float)
    d = outer - inner
    n = float(np.linalg.norm(d))
    if n < 1e-9:
        return 0.0
    return abs(float(d[1] / n))


@dataclass
class CornerEffort:
    """One front wheel's contribution."""

    name: str
    fz_lb: float
    mu: float
    fy_lb: float
    kingpin_torque_lbin: float
    effective_arm_mm: float
    tierod_force_lb: float
    rack_share_lb: float


@dataclass
class SteeringEffort:
    corners: list
    rack_force_lb: float
    steering_wheel_torque_lbin: float | None
    hand_force_lb: float | None
    notes: list

    @property
    def rack_force_n(self) -> float:
        return self.rack_force_lb * N_PER_LBF

    def summary(self) -> dict:
        return {
            "rack_force_lb": self.rack_force_lb,
            "rack_force_n": self.rack_force_n,
            "steering_wheel_torque_lbin": self.steering_wheel_torque_lbin,
            "hand_force_lb": self.hand_force_lb,
            "kingpin_torque_lbin": max(
                (c.kingpin_torque_lbin for c in self.corners), default=0.0),
            "tierod_force_lb": max(
                (c.tierod_force_lb for c in self.corners), default=0.0),
        }


def cornering_effort(hp, state, wheel_loads_lb, tire,
                     pneumatic_trail_mm: float = 0.0,
                     rack_travel_per_turn_mm: float | None = None,
                     wheel_diameter_mm: float | None = None,
                     ) -> SteeringEffort:
    """Rack force with both front tires at their lateral grip limit.

    `wheel_loads_lb` is (outside, inside) vertical load in pounds, taken
    from the dynamics sheet so that lateral load transfer is already
    included. `tire` supplies `peak_mu(fz_lb)`; grip is load-sensitive,
    so the two wheels do NOT simply scale with load.
    """
    corners = []
    notes = []
    arm_mm = tierod_effective_arm(state, hp)
    frac = rack_axis_fraction(state, hp)
    if abs(arm_mm) < 1e-6:
        raise ValueError("tie rod passes through the kingpin axis — "
                         "no steering arm, so no effort to report")
    for name, fz in zip(("outside", "inside"), wheel_loads_lb):
        fz = max(float(fz), 0.0)
        mu = float(tire.peak_mu(fz)) if fz > 0.0 else 0.0
        fy = mu * fz
        # Lateral force acts at the contact patch, toward the turn centre.
        # Its moment about the kingpin is what the tie rod must hold.
        force = np.array([0.0, -fy, 0.0])
        m_lbmm = kingpin_moment(state, force,
                                pneumatic_trail=pneumatic_trail_mm)
        m_lbin = m_lbmm / IN
        f_tr = abs(m_lbmm / arm_mm)
        corners.append(CornerEffort(
            name=name, fz_lb=fz, mu=mu, fy_lb=fy,
            kingpin_torque_lbin=abs(m_lbin), effective_arm_mm=abs(arm_mm),
            tierod_force_lb=f_tr, rack_share_lb=f_tr * frac))
    rack = sum(c.rack_share_lb for c in corners)
    if pneumatic_trail_mm <= 0.0:
        notes.append(
            "Pneumatic trail taken as zero: at the grip limit the tire is "
            "saturated and its trail has collapsed. The aligning moment "
            "PEAKS BELOW the limit, so this is an under-estimate of the "
            "worst cornering case — set a trail to see that peak.")
    notes.append(
        "Parking effort is not included and usually governs a manual "
        "rack. Size the joint on the larger of the two.")
    sw_torque = hand = None
    if rack_travel_per_turn_mm and rack_travel_per_turn_mm > 0.0:
        # Virtual work: the wheel turns once (2*pi) while the rack moves
        # one travel-per-turn, so T = F * travel / (2*pi).
        sw_torque = rack * (rack_travel_per_turn_mm / IN) / (2.0 * np.pi)
        if wheel_diameter_mm and wheel_diameter_mm > 0.0:
            hand = sw_torque / (0.5 * wheel_diameter_mm / IN)
    return SteeringEffort(corners=corners, rack_force_lb=rack,
                          steering_wheel_torque_lbin=sw_torque,
                          hand_force_lb=hand, notes=notes)


def classical_kingpin_moment(fy_lb: float, trail_mm: float,
                             fz_lb: float = 0.0,
                             scrub_mm: float = 0.0) -> float:
    """The textbook 2D decomposition, in pound-millimetres.

    Kept so the exact 3D form can be checked against the thing every
    reference book prints: lateral force through the mechanical trail,
    plus vertical load through the scrub radius. Valid when the kingpin
    is near vertical; the tests measure how far apart the two get.
    """
    return fy_lb * trail_mm + fz_lb * scrub_mm
