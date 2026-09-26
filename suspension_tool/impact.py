"""Impact load cases -> forces at every suspension mounting point (FEA).

WHY: the kinematic model already knows exactly where every hardpoint is and
which member connects to which. That is most of what you need to turn an
impact scenario into the boundary conditions for an ANSYS / Onshape
Simulation stress run. This module closes the gap:

    impact scenario  ->  contact-patch load  ->  load path through the links
                     ->  a force vector at every chassis pickup + the axial
                         force in every member  ->  CSV for FEA

STEP 1 — IMPULSE (`ImpactCase.wheel_load`)
Momentum: an impact that changes the vehicle's speed by dv in a contact
time dt needs an average force  F_avg = m*dv/dt  (impulse J = m*dv = F*dt).
Real impacts are not rectangular pulses, so the PEAK force is higher than
the average; `pulse` scales it:
    rectangular  1.00   (idealised, non-physical - a lower bound)
    half_sine    1.571  (= pi/2; the usual elastic-contact assumption)
    triangular   2.00   (sharp rise and decay - conservative)
`peak_factor` is applied to the average to get the design force.

Sign/axis convention is the tool's internal frame (+X forward, +Y left,
+Z up), forces are what the GROUND applies TO THE TIRE:
  * head-on / kerb strike     -> -X on the wheel (decelerating the car)
  * vertical bump or landing  -> +Z up into the tire
  * lateral slide into a rock -> +/-Y

STEP 2 — LOAD PATH (`corner_loads`) — EVERY suspension type
Static equilibrium of the corner treating each control arm / carrier as a
RIGID body and each link (tie rod, camber link, shock, halfshaft) as a
two-force member. Three formulations, picked by type:

  * DOUBLE WISHBONE — two arms feed ONE upright, so it is a fork, not a
    chain. Unknowns: the two ball-joint force vectors (3+3), the tie-rod
    tension and the shock force (1+1) = 8. Equations: the upright's 6
    (3 force + 3 moment) plus ONE moment equation per arm about its own
    chassis bushing axis (an A-arm is free to rotate about that axis, so
    the net moment about it must vanish) = 8. Square and determinate; each
    arm's two bushing reactions then follow from that arm's equilibrium.

  * MULTILINK — the upright is held by five links plus the shock, all
    two-force members to the chassis: 6 unknowns, 6 equations, square.

  * SERIAL CHAINS (trailing arm, H-arm, C-hub, loaded halfshaft) — a chain
    of rigid bodies solved OUTBOARD to INBOARD, each by `_solve_body`, with
    each body's joint reactions passed to the next body inboard:
        trailing arm      [arm+upright]                      (shock)
        H-arm             upright -> H-arm                   (camber, shock)
        loaded halfshaft  knuckle -> upper arm               (SHAFT, shock)
        C-hub             steering block -> C-hub -> arm  (tie rod, camber,
                                                            shock)
    The closing link of each body (camber link, tie rod, halfshaft, shock)
    is what reacts the moment its joint pair cannot, which is exactly what
    makes each body solvable.

STEP 3 — LOAD MATRIX + ENVELOPE (`standard_load_matrix`, `envelope`)
A single impulse along one axis is not the worst case a member sees. The
Lotus / standard-FSAE method loads the patch with a COMBINED force from the
traction circle (Fx, Fy, Fz together) over a whole matrix of cases, each
evaluated THROUGH the travel range, and keeps the WORST per member/pickup.
`friction_circle_loads` sweeps |F_horiz| = mu*Fz around the circle;
`standard_load_matrix` auto-derives mu (tire, by surface) and the per-corner
vertical (dynamics, incl. transfer) so the set matches the car; `envelope`
re-poses the double wishbone at each travel and returns the design envelope
(peak tension AND compression per member, with the governing case + pose).
An opt-in rocker (`corner_loads_rocker`) routes the pushrod axial through a
bellcrank to the damper force and pivot reaction.

Two spherical joints on a LINE cannot resolve force along that line (the
axial pair is self-equilibrating), so wherever a body pins to the next one
through two joints — an A-arm's bushings, an H-arm's grab points, a hinge
pin, a kingpin — that axial split is statically indeterminate. The solver
returns the minimum-norm split and reports the TOTAL axial in the note: if
axial load sizes your bracket, check it against the whole force, not half.

Mount loads are tagged `is_chassis`. True = a frame pickup, and those are
the rows to apply in a chassis FEA (they sum exactly to the wheel load).
False = an INTERNAL joint (ball joint, hinge pin, kingpin) — use it to size
the joint and the upright, but it cancels inside the corner.

LIMITS — read before trusting a stress result:
  * Quasi-static: the peak impact force is applied as a static load. There
    is no structural dynamics, no wave propagation, no bushing compliance.
    That is the standard hand-calculation approach for sizing links, and it
    is conservative when the pulse is long compared with the member's
    natural period - which is why `pulse` matters.
  * Rigid links, frictionless spherical joints, massless members.
  * The tire is a point contact at the contact patch.
"""

from dataclasses import dataclass, field

import numpy as np

G_IN_S2 = 386.4          # gravity, in/s^2 (imperial, matches dynamics.py)
MPH_TO_IN_S = 17.6       # 1 mph = 17.6 in/s
IN_TO_MM = 25.4
LB_TO_N = 4.4482216153

# Peak / average force for a contact pulse of a given shape.
PULSE_SHAPES = {
    "rectangular": 1.0,
    "half_sine": np.pi / 2.0,
    "triangular": 2.0,
}
DEFAULT_PULSE = "half_sine"


@dataclass
class ImpactCase:
    """One impact scenario, in the imperial units the dynamics layer uses.

    `speed_mph` -> `final_speed_mph` over `contact_time_s`. The mass that
    decelerates is `vehicle_weight_lb` (whole car for a head-on hit); the
    share reaching THIS corner is `corner_share` (e.g. 0.5 for a symmetric
    two-wheel front strike, 1.0 for a single wheel hitting a rock).

    `direction` picks which axis the impulse acts along:
      "head_on"  -X (fore-aft, decelerating)
      "vertical" +Z (bump / landing)
      "lateral"  +Y (slide into a kerb, left corner)
    """

    name: str = "head-on impact"
    direction: str = "head_on"          # head_on | vertical | lateral
    speed_mph: float = 15.0
    final_speed_mph: float = 0.0
    contact_time_s: float = 0.15
    vehicle_weight_lb: float = 580.0
    corner_share: float = 0.5
    pulse: str = DEFAULT_PULSE
    safety_factor: float = 1.0
    # carried through so a vertical case can add the static corner load
    static_corner_load_lb: float = 0.0
    include_static: bool = True

    # ---------------------------------------------------------------
    @property
    def delta_v_in_s(self) -> float:
        return (self.speed_mph - self.final_speed_mph) * MPH_TO_IN_S

    @property
    def peak_factor(self) -> float:
        return PULSE_SHAPES.get(self.pulse, PULSE_SHAPES[DEFAULT_PULSE])

    def impulse_lb_s(self) -> float:
        """J = m*dv, in lb-s (mass = W/g)."""
        m = self.vehicle_weight_lb / G_IN_S2
        return m * self.delta_v_in_s

    def average_force_lb(self) -> float:
        """F_avg = J / dt, the whole-vehicle average contact force."""
        if self.contact_time_s <= 0.0:
            raise ValueError("contact_time_s must be > 0 "
                             "(an instantaneous impact implies infinite force)")
        return self.impulse_lb_s() / self.contact_time_s

    def peak_force_lb(self) -> float:
        """Design force at ONE corner: average x pulse shape x share x SF."""
        return (self.average_force_lb() * self.peak_factor
                * self.corner_share * self.safety_factor)

    def equivalent_g(self) -> float:
        """The deceleration this case represents, in g (sanity check)."""
        return self.average_force_lb() * self.peak_factor / self.vehicle_weight_lb

    def wheel_load(self) -> np.ndarray:
        """Force the ground applies to the tire at the contact patch (lb),
        in the tool frame (+X fwd, +Y left, +Z up)."""
        f = self.peak_force_lb()
        d = self.direction.lower()
        if d == "head_on":
            load = np.array([-f, 0.0, 0.0])     # pushes the car backwards
        elif d == "vertical":
            load = np.array([0.0, 0.0, f])      # bump pushes the tire up
        elif d == "lateral":
            load = np.array([0.0, f, 0.0])      # into the left-side tire
        else:
            raise ValueError(f"unknown impact direction {self.direction!r} "
                             "(head_on | vertical | lateral)")
        if self.include_static and self.static_corner_load_lb:
            load = load + np.array([0.0, 0.0, self.static_corner_load_lb])
        return load

    def summary(self) -> dict:
        return {
            "name": self.name,
            "direction": self.direction,
            "delta_v_mph": self.speed_mph - self.final_speed_mph,
            "contact_time_s": self.contact_time_s,
            "impulse_lb_s": self.impulse_lb_s(),
            "average_force_lb": self.average_force_lb(),
            "pulse": self.pulse,
            "peak_factor": self.peak_factor,
            "corner_share": self.corner_share,
            "safety_factor": self.safety_factor,
            "corner_force_lb": self.peak_force_lb(),
            "equivalent_g": self.equivalent_g(),
        }


# ----------------------------------------------------------------------
# Load path: contact-patch force -> member forces -> mount reactions
# ----------------------------------------------------------------------
def _unit(v):
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("degenerate (zero-length) member")
    return np.asarray(v, float) / n


@dataclass
class MountLoad:
    """Force the suspension applies TO one chassis pickup (lb) — i.e. the
    load you apply to the FRAME in an FEA model. (The equal-and-opposite
    force acts on the arm/link itself.) Summing these over a corner
    reproduces the wheel load exactly, which is the built-in check."""
    name: str
    point: np.ndarray            # mm, tool frame
    force: np.ndarray            # lb, tool frame
    note: str = ""
    is_chassis: bool = True      # False = an INTERNAL joint (ball joint,
                                 # hinge pin, kingpin): size the joint and
                                 # the upright with it, but do NOT apply it
                                 # to a frame FEA — it cancels internally.

    @property
    def magnitude(self) -> float:
        return float(np.linalg.norm(self.force))


@dataclass
class MemberLoad:
    """Axial force in a two-force member (lb, + = TENSION)."""
    name: str
    axial_lb: float
    inboard: np.ndarray = field(default_factory=lambda: np.zeros(3))
    outboard: np.ndarray = field(default_factory=lambda: np.zeros(3))

    @property
    def state(self) -> str:
        return "tension" if self.axial_lb >= 0 else "compression"

    @property
    def length_mm(self) -> float:
        return float(np.linalg.norm(self.outboard - self.inboard))


@dataclass
class BodyLoadSet:
    """The COMPLETE, self-equilibrated set of forces acting on ONE part —
    what you apply to that part in a component-level FEA (ANSYS / Onshape).

    Because the set is self-equilibrated (ΣF = 0 and ΣM = 0 to machine
    precision), an FEA solver can take it directly with only a soft restraint
    to remove rigid-body modes: the loads themselves already balance. This is
    what lets you see the BENDING a mid-span shock mount induces in a control
    arm — bending is an internal stress resultant, so it never appears in the
    joint reactions, only in the stressed part.

    `loads` is [(label, point_mm, force_lb)]; `couples` are pure moments
    (lb*mm), e.g. the tire's aligning torque on the upright."""
    name: str
    loads: list
    couples: list = field(default_factory=list)

    def force_residual(self) -> float:
        return float(np.linalg.norm(
            sum((np.asarray(f, float) for _, _, f in self.loads),
                np.zeros(3))))

    def moment_residual(self, about=None) -> float:
        ref = (np.zeros(3) if about is None else np.asarray(about, float))
        m = sum((np.cross(np.asarray(p, float) - ref, np.asarray(f, float))
                 for _, p, f in self.loads), np.zeros(3))
        m = m + sum((np.asarray(c, float) for c in self.couples), np.zeros(3))
        return float(np.linalg.norm(m))


@dataclass
class CornerLoadResult:
    mounts: list
    members: list
    wheel_load_lb: np.ndarray
    residual: float              # equilibrium residual (should be ~0)
    notes: list = field(default_factory=list)
    bodies: dict = field(default_factory=dict)   # name -> BodyLoadSet

    def mount(self, name: str) -> MountLoad:
        for m in self.mounts:
            if m.name == name:
                return m
        raise KeyError(name)

    def worst_mount(self) -> MountLoad:
        return max(self.mounts, key=lambda m: m.magnitude)

    def worst_member(self) -> MemberLoad:
        return max(self.members, key=lambda m: abs(m.axial_lb))


def _arm_axis_moment_row(inner_f, inner_r, point, direction_basis):
    """Row of the moment-about-the-arm-axis equation for a unit force at
    `point` expressed in `direction_basis` (list of unit vectors)."""
    axis = _unit(np.asarray(inner_r, float) - np.asarray(inner_f, float))
    r = np.asarray(point, float) - np.asarray(inner_f, float)
    return [float(np.dot(axis, np.cross(r, d))) for d in direction_basis]


def _solve_body(joints, links, external, ref, ext_moments=None):
    """Equilibrium of ONE rigid body.

    joints   : [(name, point)] spherical joints to the next body inboard —
               3 unknown force components each, acting ON this body.
    links    : [(name, inboard_pt, outboard_pt)] two-force members; the
               unknown is the axial TENSION T, and the force ON THIS BODY
               is -T*n with n = unit(outboard - inboard).
    external : [(point, force)] already-known loads on this body.
    ref      : moment reference point.
    ext_moments : optional [3-vec] pure couples applied to this body (e.g.
               the tire's aligning / overturning torque at the contact
               patch, which has no force-offset representation). lb*mm.

    Returns (joint_forces, link_tensions). Six equations; the unknown count
    may exceed six when two joints sit on a line (their shared axial pair
    is self-equilibrating and therefore indeterminate) — least squares then
    returns the minimum-norm split, which is the standard convention and is
    flagged in the caller's notes.
    """
    n_j, n_l = len(joints), len(links)
    A = np.zeros((6, 3 * n_j + n_l))
    b = np.zeros(6)
    basis = list(np.eye(3))
    for jj, (_, p) in enumerate(joints):
        r = np.asarray(p, float) - ref
        for k in range(3):
            A[0:3, 3 * jj + k] = basis[k]
            A[3:6, 3 * jj + k] = np.cross(r, basis[k])
    for li, (_, pin, pout) in enumerate(links):
        n = _unit(np.asarray(pout, float) - np.asarray(pin, float))
        col = 3 * n_j + li
        A[0:3, col] = -n
        A[3:6, col] = np.cross(np.asarray(pout, float) - ref, -n)
    for p, f in external:
        b[0:3] -= np.asarray(f, float)
        b[3:6] -= np.cross(np.asarray(p, float) - ref, np.asarray(f, float))
    for m in (ext_moments or []):
        b[3:6] -= np.asarray(m, float)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    residual = float(np.linalg.norm(A @ sol - b))
    jf = [sol[3 * i:3 * i + 3] for i in range(n_j)]
    lt = [float(sol[3 * n_j + i]) for i in range(n_l)]
    return jf, lt, residual


# Per-type body chain, OUTBOARD first. Each entry describes one rigid body:
#   joints : attribute names of the points where it pins to the next body
#            inboard (or to the chassis, for the last body)
#   links  : (name, inboard attr, outboard attr) two-force members, all of
#            which anchor to the chassis in these layouts
#   ground : True when `joints` land on the chassis (-> chassis mounts)
_CHAINS = {
    "TrailingArmPoints": [
        # one rigid body: arm + upright + wheel, swinging on its pivot axis
        dict(name="arm", joints=["pivot_inner", "pivot_outer"],
             links=[("shock", "shock_inner", "shock_outer")], ground=True),
    ],
    "HArmRearPoints": [
        dict(name="upright", joints=["outer_front", "outer_rear"],
             links=[("camber_link", "camber_inner", "camber_outer")],
             ground=False),
        dict(name="harm", joints=["arm_inner_front", "arm_inner_rear"],
             links=[("shock", "shock_inner", "shock_outer")], ground=True),
    ],
    "LoadedHalfshaftPoints": [
        # the halfshaft is STRUCTURAL here — it is the closing link
        dict(name="knuckle", joints=["hinge_front", "hinge_rear"],
             links=[("halfshaft", "hs_inner", "hs_outer")], ground=False),
        dict(name="upper_arm", joints=["arm_inner_front", "arm_inner_rear"],
             links=[("shock", "shock_inner", "shock_outer")], ground=True),
    ],
    "CHubFrontPoints": [
        dict(name="steering_block", joints=["kingpin_upper", "kingpin_lower"],
             links=[("tie_rod", "tierod_inner", "tierod_outer")],
             ground=False),
        dict(name="c_hub", joints=["hinge_front", "hinge_rear"],
             links=[("camber_link", "camber_inner", "camber_outer")],
             ground=False),
        dict(name="lower_arm", joints=["arm_inner_front", "arm_inner_rear"],
             links=[("shock", "shock_inner", "shock_outer")], ground=True),
    ],
}


def _chain_loads(hp, w, cp, patch_moment=None) -> CornerLoadResult:
    """Load path for the serial-chain types (trailing arm, H-arm, C-hub,
    loaded halfshaft): solve each body outboard-to-inboard, passing the
    joint reactions down the chain. A contact-patch couple `patch_moment`
    (lb*mm) is applied to the OUTBOARD body only — the inboard bodies see
    it through the joint reactions carried down the chain."""
    chain = _CHAINS[type(hp).__name__]
    mounts, members, notes = [], [], []
    carried = [(cp, w)]                      # loads on the current body
    worst_res = 0.0
    for bi, body in enumerate(chain):
        joints = [(a, np.asarray(getattr(hp, a), float))
                  for a in body["joints"]]
        links = [(nm, np.asarray(getattr(hp, i), float),
                  np.asarray(getattr(hp, o), float))
                 for nm, i, o in body["links"]]
        ref = joints[0][1]
        em = ([patch_moment] if (bi == 0 and patch_moment is not None)
              else None)
        jf, lt, res = _solve_body(joints, links, carried, ref, ext_moments=em)
        worst_res = max(worst_res, res)
        for (nm, pin, pout), t in zip(links, lt):
            members.append(MemberLoad(nm, t, pin, pout))
            # a link in tension pulls the chassis toward the wheel
            mounts.append(MountLoad(f"{nm}_inner", pin,
                                    t * _unit(pout - pin),
                                    "two-force member (chassis end)"))
        if body["ground"]:
            axis_note = ""
            if len(joints) == 2:
                axis = _unit(joints[1][1] - joints[0][1])
                axial = float(np.dot(jf[0] + jf[1], axis))
                axis_note = ("axial share between the two bushings is "
                             "statically indeterminate; total axial along "
                             f"the arm axis = {axial:+.1f} lb (min-norm "
                             "split shown)")
            for (nm, p), f in zip(joints, jf):
                mounts.append(MountLoad(nm, p, -f, axis_note))
        else:
            for (nm, p), f in zip(joints, jf):
                mounts.append(MountLoad(
                    f"{body['name']}:{nm}", p, -f,
                    "INTERNAL joint — size the joint/upright with this, do "
                    "not apply it to a frame FEA", is_chassis=False))
            # pass the reactions to the next body inboard
            carried = [(p, -f) for (_, p), f in zip(joints, jf)]
    notes.append("Quasi-static: the peak impact force is applied as a "
                 "static load (no structural dynamics or bushing "
                 "compliance). Rigid links, spherical joints.")
    return CornerLoadResult(mounts=mounts, members=members, wheel_load_lb=w,
                            residual=worst_res, notes=notes)


def _multilink_loads(hp, w, cp, patch_moment=None) -> CornerLoadResult:
    """Multilink: the upright is held by five links plus the shock, all
    two-force members anchored to the chassis — six unknowns, six
    equations, square and determinate."""
    links = [(f"link{k}", np.asarray(getattr(hp, f"link{k}_inner"), float),
              np.asarray(getattr(hp, f"link{k}_outer"), float))
             for k in range(1, 6)]
    links.append(("shock", np.asarray(hp.shock_inner, float),
                  np.asarray(hp.shock_outer, float)))
    em = [patch_moment] if patch_moment is not None else None
    _, lt, res = _solve_body([], links, [(cp, w)],
                             np.asarray(hp.wheel_center, float),
                             ext_moments=em)
    mounts, members = [], []
    for (nm, pin, pout), t in zip(links, lt):
        members.append(MemberLoad(nm, t, pin, pout))
        mounts.append(MountLoad(f"{nm}_inner", pin, t * _unit(pout - pin),
                                "two-force member (chassis end)"))
    return CornerLoadResult(
        mounts=mounts, members=members, wheel_load_lb=w, residual=res,
        notes=["Quasi-static: the peak impact force is applied as a static "
               "load (no structural dynamics or bushing compliance). Rigid "
               "links, spherical joints."])


def corner_loads(hp, wheel_load_lb, contact_patch=None,
                 shock_force_known=None, patch_moment=None) -> CornerLoadResult:
    """Static load path for ANY of the tool's suspension types.

    Dispatches to the right formulation:
      * double wishbone — two parallel arms feeding one upright (below)
      * multilink       — five links + shock, square 6x6
      * trailing arm / H-arm / C-hub / loaded halfshaft — a serial chain of
        rigid bodies solved outboard-to-inboard (`_chain_loads`)

    `patch_moment` (optional 3-vec, lb*mm) is a pure couple applied at the
    contact patch — the tire's aligning torque Mz and overturning moment Mx,
    which a point force cannot represent. (The loaded-radius moment arm of a
    longitudinal/lateral force IS already captured, because the force acts
    at the contact patch, offset below the wheel centre.)
    """
    from .geometry import DoubleWishbonePoints
    from .multilink import MultilinkPoints
    if not isinstance(hp, DoubleWishbonePoints):
        w = np.asarray(wheel_load_lb, float)
        cp = (np.asarray(contact_patch, float) if contact_patch is not None
              else np.array([hp.wheel_center[0], hp.wheel_center[1],
                             hp.wheel_center[2] - hp.tire_radius]))
        pm = None if patch_moment is None else np.asarray(patch_moment, float)
        if isinstance(hp, MultilinkPoints):
            return _multilink_loads(hp, w, cp, pm)
        key = type(hp).__name__
        if key in _CHAINS:
            return _chain_loads(hp, w, cp, pm)
        raise NotImplementedError(
            f"no load-path formulation for {key}")
    return _dw_corner_loads(hp, wheel_load_lb, contact_patch,
                            shock_force_known, patch_moment)


def _dw_corner_loads(hp, wheel_load_lb, contact_patch=None,
                     shock_force_known=None,
                     patch_moment=None) -> CornerLoadResult:
    """Static load path for a DOUBLE WISHBONE corner.

    hp             : DoubleWishbonePoints (mm, tool frame)
    wheel_load_lb  : (3,) force the ground applies at the contact patch, lb
    contact_patch  : where it acts (mm). Defaults to straight below the
                     wheel centre by the tire radius.
    shock_force_known : optionally pin the shock axial force (lb, +tension)
                     instead of solving for it.

    Returns the force at every chassis pickup and the axial force in every
    two-force member. See the module docstring for the formulation.
    """
    w = np.asarray(wheel_load_lb, float)
    if contact_patch is None:
        contact_patch = np.array([hp.wheel_center[0], hp.wheel_center[1],
                                  hp.wheel_center[2] - hp.tire_radius])
    cp = np.asarray(contact_patch, float)

    ubj = np.asarray(hp.uca_outer, float)
    lbj = np.asarray(hp.lca_outer, float)
    tro = np.asarray(hp.tierod_outer, float)
    tri = np.asarray(hp.tierod_inner, float)
    n_tr = _unit(tro - tri)                     # tie rod acts along its axis

    shock_on_uca = bool(getattr(hp, "shock_on_uca", False))
    n_sh = _unit(np.asarray(hp.shock_outer, float)
                 - np.asarray(hp.shock_inner, float))
    sh_pt = np.asarray(hp.shock_outer, float)

    # Where the tie rod's INNER end lands. Normally the chassis/rack, but it
    # may bolt to a control arm (geometry.tierod_on_lca / tierod_on_uca — a
    # supported Setup option the kinematic solver honours). When it does,
    # the tie rod is NOT a chassis pickup: its inboard reaction +T*n_tr is a
    # load applied to that ARM, so it must enter that arm's moment equation
    # and its bushing split, exactly like the shock's.
    tierod_on_lca = bool(getattr(hp, "tierod_on_lca", False))
    tierod_on_uca = bool(getattr(hp, "tierod_on_uca", False))

    # Unknowns: F_ubj(3), F_lbj(3), T_tierod, T_shock   (forces ON the
    # upright from the arms/links; T_* are axial tensions).
    ex, ey, ez = np.eye(3)
    basis = [ex, ey, ez]
    n_unk = 8
    A = np.zeros((8, n_unk))
    b = np.zeros(8)

    # SIGN CONVENTION: T is the member's axial TENSION (+ = pulled). A
    # member in tension pulls each end TOWARD the other, so with
    # n = unit(outboard - inboard) the force it applies to the OUTBOARD
    # body is -T*n and to the INBOARD (chassis) body is +T*n.
    #
    # --- upright: sum of forces = 0 -------------------------------------
    for i in range(3):
        A[i, i] = 1.0            # F_ubj  (force ON the upright from the UCA)
        A[i, 3 + i] = 1.0        # F_lbj
        A[i, 6] = -n_tr[i]       # tie rod pulls the upright inboard
        b[i] = -w[i]
    # the shock does NOT act on the upright — it mounts on an arm, so it
    # enters through that arm's moment equation below.
    # --- upright: sum of moments about the wheel centre = 0 -------------
    wc = np.asarray(hp.wheel_center, float)
    r_ubj, r_lbj, r_tro = ubj - wc, lbj - wc, tro - wc
    r_cp = cp - wc
    for j in range(3):           # moment rows 3..5
        row = 3 + j
        for k in range(3):
            A[row, k] = np.cross(r_ubj, basis[k])[j]
            A[row, 3 + k] = np.cross(r_lbj, basis[k])[j]
        A[row, 6] = np.cross(r_tro, -n_tr)[j]
        b[row] = -np.cross(r_cp, w)[j]
        if patch_moment is not None:
            b[row] -= float(patch_moment[j])   # tire aligning/overturning couple

    # --- each arm: moment about its OWN bushing axis = 0 ----------------
    # The arm carries -F_bj (the reaction of what it applies to the
    # upright) plus, on whichever arm they mount to, the shock force -T*n_sh
    # and (if the toe link is arm-mounted) the tie-rod reaction +T*n_tr.
    # Two spherical bushings on a line contribute NO moment about that line,
    # so this single scalar equation per arm is exact and complete.
    def _axis_moment(inner_f, inner_r, point, direction):
        """Moment about the arm's bushing axis, per unit axial tension, of a
        unit force `direction` applied at `point`."""
        axis = _unit(np.asarray(inner_r, float) - np.asarray(inner_f, float))
        return float(np.dot(axis, np.cross(
            np.asarray(point, float) - np.asarray(inner_f, float),
            direction)))

    uca_row = _arm_axis_moment_row(hp.uca_inner_front, hp.uca_inner_rear,
                                   ubj, basis)
    for k in range(3):
        A[6, k] = -uca_row[k]
    if shock_on_uca:
        A[6, 7] = _axis_moment(hp.uca_inner_front, hp.uca_inner_rear,
                               sh_pt, -n_sh)
    if tierod_on_uca:
        A[6, 6] = _axis_moment(hp.uca_inner_front, hp.uca_inner_rear,
                               tri, n_tr)
    lca_row = _arm_axis_moment_row(hp.lca_inner_front, hp.lca_inner_rear,
                                   lbj, basis)
    for k in range(3):
        A[7, 3 + k] = -lca_row[k]
    if not shock_on_uca:
        A[7, 7] = _axis_moment(hp.lca_inner_front, hp.lca_inner_rear,
                               sh_pt, -n_sh)
    if tierod_on_lca:
        A[7, 6] = _axis_moment(hp.lca_inner_front, hp.lca_inner_rear,
                               tri, n_tr)

    if shock_force_known is not None:
        # pin the shock and drop its column into the RHS
        b = b - A[:, 7] * float(shock_force_known)
        A = A[:, :7]
        sol7, *_ = np.linalg.lstsq(A, b, rcond=None)
        sol = np.append(sol7, float(shock_force_known))
        residual = float(np.linalg.norm(A @ sol7 - b))
    else:
        sol, res, rank, _sv = np.linalg.lstsq(A, b, rcond=None)
        residual = float(np.linalg.norm(A @ sol - b))

    f_ubj, f_lbj = sol[0:3], sol[3:6]
    t_tr, t_sh = float(sol[6]), float(sol[7])

    notes = []
    if residual > 1e-6 * max(1.0, float(np.linalg.norm(w))):
        notes.append(f"equilibrium residual {residual:.3g} lb — the corner "
                     "may be near-singular (check for collinear members)")

    members = [
        MemberLoad("tie_rod", t_tr, tri, tro),
        MemberLoad("shock", t_sh, np.asarray(hp.shock_inner, float), sh_pt),
    ]

    # --- arm bushing reactions ------------------------------------------
    mounts = []
    bodies = {}

    def _arm_bushings(prefix, inner_f, inner_r, applied):
        """Split the arm's applied loads into its two bushing reactions.
        `applied` is [(label, point, force)] — every load the arm carries
        BESIDES the bushings (ball joint, shock, arm-mounted tie rod). Two
        spherical bushings on a line cannot resolve load ALONG that line, so
        the axial share is the minimum-norm split — noted in the output."""
        p1 = np.asarray(inner_f, float)
        p2 = np.asarray(inner_r, float)
        loads = [(np.asarray(p, float), np.asarray(f, float))
                 for _, p, f in applied]
        # unknowns R1(3), R2(3); equations: sum F = 0, sum M about p1 = 0
        M = np.zeros((6, 6))
        rhs = np.zeros(6)
        for i in range(3):
            M[i, i] = 1.0
            M[i, 3 + i] = 1.0
            rhs[i] = -sum(f[i] for _, f in loads)
        for j in range(3):
            for k in range(3):
                M[3 + j, 3 + k] = np.cross(p2 - p1, basis[k])[j]
            rhs[3 + j] = -sum(np.cross(p - p1, f)[j] for p, f in loads)
        r, *_ = np.linalg.lstsq(M, rhs, rcond=None)   # min-norm on the axis
        axis = _unit(p2 - p1)
        axial = float(np.dot(r[0:3] + r[3:6], axis))
        note = ("axial share between the two bushings is statically "
                f"indeterminate; total axial along the arm axis = "
                f"{axial:+.1f} lb (min-norm split shown)")
        # r is the force ON THE ARM from the chassis; a chassis FEA wants
        # the equal-and-opposite force the suspension applies TO the frame.
        mounts.append(MountLoad(f"{prefix}_inner_front", p1, -r[0:3], note))
        mounts.append(MountLoad(f"{prefix}_inner_rear", p2, -r[3:6], note))
        # The arm's OWN load set (what you apply to the part in an FEA):
        # the bushing reactions ON the arm plus everything else it carries.
        bodies[prefix] = BodyLoadSet(
            prefix,
            [(f"{prefix}_inner_front", p1, np.asarray(r[0:3], float)),
             (f"{prefix}_inner_rear", p2, np.asarray(r[3:6], float))]
            + [(lab, np.asarray(p, float), np.asarray(f, float))
               for lab, p, f in applied])

    # The arm carries the REACTION of the ball-joint force it applies to
    # the upright (-f_ubj / -f_lbj), plus the shock where it mounts. The
    # shock's force ON THE ARM is -T*n_sh (tension pulls the arm toward the
    # chassis end) — the same sign the arm moment equation above used. An
    # arm-mounted tie rod adds +T*n_tr at its inner ball joint.
    f_sh_on_arm = -t_sh * n_sh
    f_tr_on_arm = t_tr * n_tr
    uca_applied = [("uca_outer_ball_joint", ubj, -f_ubj)]
    lca_applied = [("lca_outer_ball_joint", lbj, -f_lbj)]
    (uca_applied if shock_on_uca else lca_applied).append(
        ("shock_outer", sh_pt, f_sh_on_arm))
    if tierod_on_uca:
        uca_applied.append(("tierod_inner", tri, f_tr_on_arm))
    elif tierod_on_lca:
        lca_applied.append(("tierod_inner", tri, f_tr_on_arm))
    _arm_bushings("uca", hp.uca_inner_front, hp.uca_inner_rear, uca_applied)
    _arm_bushings("lca", hp.lca_inner_front, hp.lca_inner_rear, lca_applied)

    # Ball joints and the shock's arm end are INTERNAL to the corner: they
    # size the joint / upright / arm, but must never be applied to a frame
    # FEA (they cancel inside the corner). Forces quoted ON THE UPRIGHT.
    mounts.append(MountLoad(
        "uca_outer_ball_joint", ubj, f_ubj,
        "INTERNAL joint — force on the UPRIGHT from the upper arm "
        "(equal and opposite on the arm); do not apply to a frame FEA",
        is_chassis=False))
    mounts.append(MountLoad(
        "lca_outer_ball_joint", lbj, f_lbj,
        "INTERNAL joint — force on the UPRIGHT from the lower arm "
        "(equal and opposite on the arm); do not apply to a frame FEA",
        is_chassis=False))
    mounts.append(MountLoad(
        "shock_outer", sh_pt, -f_sh_on_arm,
        "INTERNAL joint — shock's arm end; force on the SHOCK from the arm "
        "(equal and opposite on the arm). This is the load that BENDS the "
        "arm; do not apply it to a frame FEA", is_chassis=False))

    # Link inboard ends. A member in tension pulls its anchor TOWARD the
    # wheel, i.e. along +n (inboard -> outboard). The tie rod's inner end is
    # a chassis pickup ONLY when it is not bolted to an arm.
    tierod_on_arm = tierod_on_lca or tierod_on_uca
    mounts.append(MountLoad(
        "tierod_inner", tri, t_tr * n_tr,
        ("INTERNAL joint — the toe link's inner ball joint rides the "
         + ("UPPER" if tierod_on_uca else "LOWER") + " arm, so this is a "
         "load on that ARM, not on the frame")
        if tierod_on_arm else "two-force member (rack/chassis end)",
        is_chassis=not tierod_on_arm))
    mounts.append(MountLoad("shock_inner", np.asarray(hp.shock_inner, float),
                            t_sh * n_sh, "two-force member (chassis end)"))

    # The upright's own load set, for a component FEA of the knuckle.
    upright_loads = [("contact_patch", cp, w),
                     ("uca_outer_ball_joint", ubj, np.asarray(f_ubj, float)),
                     ("lca_outer_ball_joint", lbj, np.asarray(f_lbj, float)),
                     ("tierod_outer", tro, -t_tr * n_tr)]
    bodies["upright"] = BodyLoadSet(
        "upright", upright_loads,
        couples=([np.asarray(patch_moment, float)]
                 if patch_moment is not None else []))

    if tierod_on_arm:
        notes.append(
            "Toe link's inner ball joint is mounted on the "
            + ("UPPER" if tierod_on_uca else "LOWER")
            + " arm: its reaction is carried by that arm (included in the "
              "arm's equilibrium and bushing split), NOT by the frame.")
    notes.append("Quasi-static: the peak impact force is applied as a "
                 "static load (no structural dynamics or bushing "
                 "compliance). Rigid links, spherical joints.")
    return CornerLoadResult(mounts=mounts, members=members, bodies=bodies,
                            wheel_load_lb=w, residual=residual, notes=notes)


# ======================================================================
# Combined load cases — the friction-circle / load-matrix method
# ======================================================================
# A single impulse along one axis is not the worst case a member sees. The
# established practice (Lotus Suspension Analysis loads, and every FSAE
# "suspension forces" writeup) is to load the contact patch with a COMBINED
# force from the traction/friction circle — Fx, Fy AND Fz together — and to
# take the worst result per member over a whole MATRIX of cases (max bump,
# max braking, max cornering, combined, kerb) evaluated THROUGH the travel
# range. Different (Fx,Fy) directions load different links, so the envelope,
# not any single case, is what sizes the hardware.


@dataclass
class WheelLoad:
    """One quasi-static contact-patch load, stated directly in force (the
    currency a friction-circle analysis works in — cf. ImpactCase, which
    DERIVES a force from an impulse). `force` is lb in the tool frame;
    `moment` is the tire's aligning/overturning couple (lb*mm) at the patch;
    `fz_lb` records the vertical load the case assumed, for reporting."""
    name: str
    force: np.ndarray
    moment: np.ndarray = field(default_factory=lambda: np.zeros(3))
    fz_lb: float = 0.0
    note: str = ""


def friction_circle_loads(fz_lb, mu, n_dir=12, pneumatic_trail_mm=0.0,
                          name="mu") -> list:
    """Horizontal forces of magnitude mu*Fz swept around the traction circle
    at a fixed vertical load Fz. theta = 0 is pure braking (-X), 90 deg pure
    lateral (+Y), 180 deg pure drive (+X), 270 deg (-Y) — so the set spans
    braking, cornering, drive and every combination between. Each carries
    the self-aligning couple Mz = -Fy * pneumatic_trail (the lateral force
    acts a trail behind the patch centre)."""
    h = mu * fz_lb
    out = []
    for k in range(n_dir):
        th = 2.0 * np.pi * k / n_dir
        fx = -h * np.cos(th)          # theta=0 -> braking (-X)
        fy = h * np.sin(th)           # theta=90 -> +Y (left)
        mz = -fy * pneumatic_trail_mm
        out.append(WheelLoad(
            f"{name}@{int(round(np.degrees(th)))}deg",
            np.array([fx, fy, fz_lb]), np.array([0.0, 0.0, mz]), fz_lb,
            "friction-circle combined load (|F_horiz| = mu*Fz)"))
    return out


def corner_conditions(corner="front_out", tire=None, dyn=None,
                      surface=None) -> dict:
    """Auto-derive the per-corner load inputs from the models already in the
    tool: the vertical load Fz from `dynamics.compute` (static + lateral +
    longitudinal transfer + aero, for the requested corner), the peak
    friction mu from the tire's surface table, and the pneumatic trail from
    the tire. `corner` is one of front_out / front_in / rear_out / rear_in."""
    from . import dynamics
    from .tire import SUNF_23x7
    tire = tire or SUNF_23x7
    dyn = dyn or dynamics.DynamicsInputs()
    res = dynamics.compute(dyn)
    fz = float(res[f"load_{corner}_lb"])
    axle_static = (dyn.front_axle_lb if corner.startswith("front")
                   else dyn.rear_axle_lb)
    surface = surface or tire.design_surface
    return {
        "corner": corner,
        "fz_lb": fz,                          # dynamic (transfer-loaded)
        "static_corner_lb": axle_static / 2.0,
        "mu": float(tire.surface_mu[surface]),
        "pneumatic_trail_mm": float(tire.param("pneumatic_trail").value
                                    * IN_TO_MM),
        "surface": surface,
        "ay_g": dyn.ay_g, "ax_g": dyn.ax_g,
    }


def standard_load_matrix(corner="front_out", tire=None, dyn=None,
                         surface=None, n_dir=12, bump_g=3.0,
                         include_impulse=True,
                         vehicle_weight_lb=580.0) -> tuple:
    """The Lotus-style load-case matrix for one corner, auto-derived from
    the tire + dynamics models. Returns (loads, conditions):

      * static                     — the transfer-loaded vertical alone
      * mu@0..330deg               — the friction circle (braking .. drive ..
                                     cornering .. every combination) at Fz
      * bump_{bump_g}g             — a pure vertical hit, static corner x g
      * kerb_lateral, landing      — the two impulse cases (if include_impulse)

    The envelope over this whole set (and over travel) is what sizes the
    hardware — see `envelope`."""
    cond = corner_conditions(corner, tire, dyn, surface)
    fz, mu, trail = cond["fz_lb"], cond["mu"], cond["pneumatic_trail_mm"]
    static = cond["static_corner_lb"]
    loads = [WheelLoad("static", np.array([0.0, 0.0, fz]), fz_lb=fz,
                       note="transfer-loaded vertical, no horizontal force")]
    loads += friction_circle_loads(fz, mu, n_dir, trail)
    loads.append(WheelLoad(
        f"bump_{bump_g:g}g", np.array([0.0, 0.0, static * bump_g]),
        fz_lb=static * bump_g,
        note=f"pure vertical bump: static corner {static:.0f} lb x {bump_g:g}g"))
    if include_impulse:
        share = 0.5
        kerb = ImpactCase(name="kerb", direction="lateral", speed_mph=10.0,
                          contact_time_s=0.10, corner_share=share,
                          vehicle_weight_lb=vehicle_weight_lb)
        land = ImpactCase(name="landing", direction="vertical",
                          speed_mph=drop_height_to_mph(36.0),
                          contact_time_s=0.25, corner_share=0.25,
                          vehicle_weight_lb=vehicle_weight_lb,
                          static_corner_load_lb=static)
        loads.append(WheelLoad("kerb_lateral", kerb.wheel_load(),
                               fz_lb=fz, note="lateral kerb-strike impulse"))
        loads.append(WheelLoad("landing_vertical", land.wheel_load(),
                               fz_lb=static, note="vertical landing impulse"))
    return loads, cond


# ----------------------------------------------------------------------
# Internal stress resultants — the BENDING a mid-span shock mount induces
# ----------------------------------------------------------------------
# Joint reactions are blind to bending: for a rigid body, equilibrium fixes
# the external reactions no matter how the load is distributed inside, and
# an internal bending moment is self-equilibrating by definition. So the
# load path above is complete and correct — but it is NOT enough to size a
# control arm, because a shock hanging off the middle of an arm bends it,
# and an arm sized as a pure two-force truss would miss that entirely.
#
# These functions cut the arm and report what is transmitted across the cut.
# IDEALISATION (stated because it cannot be derived from hardpoints alone):
# the arm is treated as two straight legs, each running from one chassis
# bushing to the ball joint. Each leg carries its own bushing reaction; any
# other load (the shock, an arm-mounted toe link) is assigned to the leg it
# sits nearest, at the station where it projects onto that leg. A real arm
# has tube layout, gussets and a cross-member that redistribute this — treat
# the result as the correct ORDER and DISTRIBUTION of bending, and use the
# per-body load set with a real FEA mesh when you need the true stress.


@dataclass
class SectionLoad:
    """Internal stress resultants transmitted across one cut through a
    member, resolved in the member's own axes. Forces lb, moments lb*mm."""
    station_mm: float          # distance from the inboard (bushing) end
    point: np.ndarray
    axial_lb: float            # + = tension
    shear_lb: float            # magnitude perpendicular to the member axis
    bending_lb_mm: float       # magnitude of the moment perpendicular to it
    torsion_lb_mm: float       # moment component ALONG the member axis
    force: np.ndarray          # full internal force vector
    moment: np.ndarray         # full internal moment vector

    @property
    def bending_lb_in(self) -> float:
        return self.bending_lb_mm / IN_TO_MM


@dataclass
class BeamResult:
    """Internal loads along one leg of an arm, inboard (bushing) -> outboard
    (ball joint)."""
    name: str
    root: np.ndarray
    tip: np.ndarray
    sections: list
    carried: list = field(default_factory=list)   # labels assigned to this leg
    notes: list = field(default_factory=list)

    @property
    def length_mm(self) -> float:
        return float(np.linalg.norm(self.tip - self.root))

    def max_bending(self) -> SectionLoad:
        return max(self.sections, key=lambda s: s.bending_lb_mm)

    def max_torsion(self) -> SectionLoad:
        return max(self.sections, key=lambda s: abs(s.torsion_lb_mm))

    def max_shear(self) -> SectionLoad:
        return max(self.sections, key=lambda s: s.shear_lb)


def _project_station(p, a, b):
    """(clamped parameter t in [0,1] of p's projection onto segment a->b,
    perpendicular distance from p to the segment)."""
    d = np.asarray(b, float) - np.asarray(a, float)
    ll = float(np.dot(d, d))
    if ll < 1e-12:
        return 0.0, float(np.linalg.norm(np.asarray(p, float) - a))
    t = float(np.dot(np.asarray(p, float) - a, d) / ll)
    tc = min(max(t, 0.0), 1.0)
    foot = np.asarray(a, float) + tc * d
    return tc, float(np.linalg.norm(np.asarray(p, float) - foot))


def beam_internal_loads(root_pt, tip_pt, applied, n_stations: int = 41,
                        name: str = "leg") -> BeamResult:
    """Internal resultants along a straight leg from `root_pt` (bushing) to
    `tip_pt` (ball joint).

    `applied` is [(label, point, force)] — every load this leg carries,
    INCLUDING its own bushing reaction at the root. At each cut the
    transmitted resultants are minus the sum of everything at or inboard of
    the cut, which is what the outboard portion must supply. (Loads exactly
    at the cut are counted, so a section is reported just OUTBOARD of each
    load — that is why the bending is zero at a spherical bushing root and
    steps at the shock mount.)"""
    a = np.asarray(root_pt, float)
    b = np.asarray(tip_pt, float)
    axis = _unit(b - a)
    length = float(np.linalg.norm(b - a))
    # station of each applied load along the leg
    placed = []
    for lab, p, f in applied:
        t, _ = _project_station(np.asarray(p, float), a, b)
        placed.append((t * length, np.asarray(p, float), np.asarray(f, float),
                       lab))
    sections = []
    for s in np.linspace(0.0, length, n_stations):
        cut = a + axis * s
        fi = np.zeros(3)
        mi = np.zeros(3)
        for st, p, f, _lab in placed:
            if st <= s + 1e-9:                # at or inboard of the cut
                fi = fi + f
                mi = mi + np.cross(p - cut, f)
        f_int, m_int = -fi, -mi
        axial = float(np.dot(f_int, axis))
        shear = float(np.linalg.norm(f_int - axial * axis))
        torsion = float(np.dot(m_int, axis))
        bending = float(np.linalg.norm(m_int - torsion * axis))
        sections.append(SectionLoad(
            station_mm=float(s), point=cut, axial_lb=axial, shear_lb=shear,
            bending_lb_mm=bending, torsion_lb_mm=torsion,
            force=f_int, moment=m_int))
    return BeamResult(name=name, root=a, tip=b, sections=sections,
                      carried=[lab for _, _, _, lab in placed])


def control_arm_beams(body: BodyLoadSet, n_stations: int = 41) -> list:
    """Split a control arm's load set into its two legs (each bushing -> the
    ball joint) and return the internal loads along each.

    Works off the labels the solver emits (`*_inner_front`, `*_inner_rear`,
    `*_ball_joint`); every other load (shock, arm-mounted toe link) is
    assigned to the leg it lies nearest. Raises ValueError for a body that
    is not a two-bushing arm."""
    root_f = root_r = tip = None
    others = []
    for lab, p, f in body.loads:
        if lab.endswith("_inner_front"):
            root_f = (lab, np.asarray(p, float), np.asarray(f, float))
        elif lab.endswith("_inner_rear"):
            root_r = (lab, np.asarray(p, float), np.asarray(f, float))
        elif lab.endswith("_ball_joint"):
            tip = (lab, np.asarray(p, float), np.asarray(f, float))
        else:
            others.append((lab, np.asarray(p, float), np.asarray(f, float)))
    if root_f is None or root_r is None or tip is None:
        raise ValueError(
            f"body {body.name!r} is not a two-bushing arm with a ball joint "
            "(needs *_inner_front, *_inner_rear and *_ball_joint loads)")
    legs = [("front", root_f), ("rear", root_r)]
    # assign each remaining load to the leg it sits nearest
    assigned = {"front": [], "rear": []}
    for lab, p, f in others:
        d = {}
        for key, (_l, rp, _rf) in legs:
            _t, dist = _project_station(p, rp, tip[1])
            d[key] = dist
        assigned[min(d, key=d.get)].append((lab, p, f))
    out = []
    for key, (rlab, rp, rf) in legs:
        # Cutting one leg of a V-shaped arm separates it into the inboard
        # stub and everything else, so the resultants at the cut follow from
        # the stub's own loads alone: this leg's bushing reaction plus
        # whatever is mounted on it. The ball-joint force is NOT a load on
        # the stub — it acts at the apex, outboard of every cut.
        carried = [(rlab, rp, rf)] + assigned[key]
        beam = beam_internal_loads(
            rp, tip[1], carried,
            n_stations=n_stations, name=f"{body.name}_{key}_leg")
        beam.notes.append(
            "Idealisation: straight leg from this bushing to the ball joint "
            "(a V-arm, so a cut separates the leg); other loads assigned to "
            "the nearest leg. The moment reported AT the apex is the weld "
            "moment between the two legs — the ball joint itself passes only "
            "force to the upright.")
        beam.notes.append(
            "INDETERMINACY: the split of load between the two bushings is "
            "statically indeterminate (two spherical joints on a line), and "
            "these leg diagrams inherit that choice (min-norm). Bending "
            "CAUSED BY A MID-SPAN MOUNT is robust to it; the baseline "
            "bending in an unloaded leg is not. Use the body load set with a "
            "real mesh when the number has to be exact.")
        out.append(beam)
    return out


def shock_bending_summary(result: CornerLoadResult) -> dict:
    """The assumption-light answer to 'how much does the shock bend the arm?'

    The shock's force applied at its arm mount, moment-armed about the arm's
    ball joint and about its bushing axis. Unlike the full beam diagrams this
    needs NO idealisation of tube layout or bushing split — it is just the
    moment of a known force about a known line — so it is the number to quote
    when you want to know whether an arm must be treated as a bending member
    rather than a truss."""
    out = {}
    for arm in ("lca", "uca"):
        body = result.bodies.get(arm)
        if body is None:
            continue
        pts = {lab: (np.asarray(p, float), np.asarray(f, float))
               for lab, p, f in body.loads}
        sh = next((v for k, v in pts.items() if k == "shock_outer"), None)
        bj = next((v for k, v in pts.items() if k.endswith("_ball_joint")),
                  None)
        if sh is None or bj is None:
            continue
        p1 = pts[f"{arm}_inner_front"][0]
        p2 = pts[f"{arm}_inner_rear"][0]
        axis = _unit(p2 - p1)
        m_bj = np.cross(sh[0] - bj[0], sh[1])
        m_axis = float(np.dot(axis, np.cross(sh[0] - p1, sh[1])))
        out[arm] = {
            "shock_force_lb": float(np.linalg.norm(sh[1])),
            "lever_from_ball_joint_mm": float(np.linalg.norm(sh[0] - bj[0])),
            "bending_about_ball_joint_lb_mm": float(np.linalg.norm(m_bj)),
            "bending_about_ball_joint_lb_in": float(
                np.linalg.norm(m_bj) / IN_TO_MM),
            "moment_about_bushing_axis_lb_mm": m_axis,
        }
    return out


# ----------------------------------------------------------------------
# Pushrod / bellcrank (rocker) load path — OPT-IN
# ----------------------------------------------------------------------
# A direct coil-over is a single two-force member; a pushrod/pullrod car
# routes the wheel load through a rocker (bellcrank), so the pushrod and the
# damper carry DIFFERENT forces and the rocker pivot takes a bearing load
# none of them shows. The base DoubleWishbonePoints carries no rocker
# hardpoints, so this is provided separately and is opt-in.
@dataclass
class RockerGeometry:
    """A pushrod + bellcrank actuation. The rocker turns about `pivot_axis`
    through `pivot_pt` (one DOF); the pushrod drives it and the damper
    resists it. All points mm, tool frame. For physical consistency the
    corner's shock member should run along the PUSHROD line, i.e. set the
    geometry's shock_inner = pushrod_rocker_pt and shock_outer =
    pushrod_arm_pt."""
    pivot_pt: np.ndarray
    pivot_axis: np.ndarray
    pushrod_rocker_pt: np.ndarray     # pushrod's rocker end
    pushrod_arm_pt: np.ndarray        # pushrod's arm/upright end (outboard)
    damper_rocker_pt: np.ndarray      # damper's rocker end
    damper_chassis_pt: np.ndarray     # damper's chassis end


@dataclass
class RockerResult:
    pushrod: MemberLoad
    damper: MemberLoad
    mounts: list                      # pivot bracket + damper chassis end
    residual: float


def rocker_loads(rk: RockerGeometry, pushrod_axial_lb: float) -> RockerResult:
    """Resolve a rocker: given the pushrod axial (tension +, from the corner
    solve), find the damper force and the pivot bearing reaction by the
    rocker's single-DOF equilibrium (moment about its pivot axis = 0) plus
    force balance. Returns the pushrod and damper member loads and the two
    chassis mounts (pivot bracket, damper chassis end)."""
    axis = _unit(np.asarray(rk.pivot_axis, float))
    pr_r = np.asarray(rk.pushrod_rocker_pt, float)
    dr_r = np.asarray(rk.damper_rocker_pt, float)
    piv = np.asarray(rk.pivot_pt, float)
    # pushrod force ON the rocker (tension pulls the rocker end toward the arm)
    u_pr = _unit(np.asarray(rk.pushrod_arm_pt, float) - pr_r)
    f_pr = pushrod_axial_lb * u_pr
    # damper force ON the rocker, unit along rocker->chassis; unknown mag D
    u_d = _unit(np.asarray(rk.damper_chassis_pt, float) - dr_r)
    # moment about the pivot axis: axis . [(pr-piv)x f_pr + (dr-piv)x D*u_d] = 0
    a = float(np.dot(axis, np.cross(pr_r - piv, f_pr)))
    bcoef = float(np.dot(axis, np.cross(dr_r - piv, u_d)))
    if abs(bcoef) < 1e-9:
        raise ValueError("rocker is singular: the damper line makes no "
                         "moment about the pivot axis")
    d_mag = -a / bcoef                 # damper axial (tension +)
    f_d = d_mag * u_d
    reaction = -(f_pr + f_d)           # pivot force ON the rocker
    residual = abs(float(np.dot(axis, np.cross(pr_r - piv, f_pr)
                                + np.cross(dr_r - piv, f_d))))
    mounts = [
        MountLoad("rocker_pivot", piv, -reaction,       # force ON the bracket
                  "rocker pivot bearing reaction"),
        MountLoad("damper_chassis", np.asarray(rk.damper_chassis_pt, float),
                  -f_d, "damper chassis end (two-force member)"),
    ]
    return RockerResult(
        pushrod=MemberLoad("pushrod", pushrod_axial_lb, pr_r,
                           np.asarray(rk.pushrod_arm_pt, float)),
        damper=MemberLoad("damper", d_mag, dr_r,
                          np.asarray(rk.damper_chassis_pt, float)),
        mounts=mounts, residual=residual)


def corner_loads_rocker(hp, wheel_load_lb, rk: RockerGeometry,
                        contact_patch=None, patch_moment=None
                        ) -> CornerLoadResult:
    """Double-wishbone corner load path with a rocker: solve the corner
    (the 'shock' member IS the pushrod), then route that pushrod axial
    through the rocker to get the damper force and pivot reaction. The
    single lumped 'shock_inner' chassis mount is replaced by the pivot
    bracket + damper chassis mounts, and the damper is added as a member."""
    base = _dw_corner_loads(hp, wheel_load_lb, contact_patch,
                            patch_moment=patch_moment)
    pushrod_axial = [m for m in base.members if m.name == "shock"][0].axial_lb
    rr = rocker_loads(rk, pushrod_axial)
    mounts = [m for m in base.mounts if m.name != "shock_inner"] + rr.mounts
    members = [m for m in base.members if m.name != "shock"] + [
        MemberLoad("pushrod", rr.pushrod.axial_lb, rr.pushrod.inboard,
                   rr.pushrod.outboard), rr.damper]
    notes = list(base.notes) + [
        "Rocker actuation: the 'shock' line is the PUSHROD; the damper force "
        "and pivot reaction come from the rocker's own equilibrium.",
        f"rocker moment residual {rr.residual:.2e} lb-mm."]
    return CornerLoadResult(mounts=mounts, members=members,
                            wheel_load_lb=base.wheel_load_lb,
                            residual=max(base.residual, rr.residual),
                            notes=notes)


# ----------------------------------------------------------------------
# Through-travel posing + the load envelope
# ----------------------------------------------------------------------
def _posed_dw(hp, state):
    """A DoubleWishbonePoints posed at a solved travel: the OUTER (moving)
    points move to their solved positions, the chassis points stay put. This
    lets the static load-path solver run at any point in the travel range —
    the worst link angle (hence worst axial load) is usually at full
    bump/droop, not at ride height."""
    import dataclasses
    arm_link = (getattr(hp, "tierod_on_lca", False)
                or getattr(hp, "tierod_on_uca", False))
    return dataclasses.replace(
        hp,
        uca_outer=np.asarray(state.ubj, float),
        lca_outer=np.asarray(state.lbj, float),
        tierod_outer=np.asarray(state.tro, float),
        wheel_center=np.asarray(state.wheel_center, float),
        shock_outer=np.asarray(state.shock_outer, float),
        tierod_inner=(np.asarray(state.tierod_inner_cur, float)
                      if arm_link else hp.tierod_inner),
    )


def _poses(hp, solver=None, travels=None):
    """Yield (pose_label, posed_hp, contact_patch) over the travel range.
    Falls back to the single static pose when no solver/travels are given or
    the type has no through-travel posing yet (only the double wishbone does
    — the other types are evaluated at their static geometry)."""
    from .geometry import DoubleWishbonePoints
    if (solver is None or travels is None
            or not isinstance(hp, DoubleWishbonePoints)):
        cp = np.array([hp.wheel_center[0], hp.wheel_center[1],
                       hp.wheel_center[2] - hp.tire_radius])
        yield "static", hp, cp
        return
    for t in np.asarray(travels, float):
        try:
            st = solver.solve(float(t))
        except ValueError:
            continue                       # travel outside the solvable range
        yield (f"t={t:+.0f}", _posed_dw(hp, st),
               np.asarray(st.contact_patch, float))


@dataclass
class EnvelopeMount:
    """Worst force a single pickup sees over the whole load matrix x travel
    sweep, tagged with which case and pose produced it."""
    name: str
    point: np.ndarray
    force: np.ndarray
    is_chassis: bool
    governing_case: str
    governing_pose: str

    @property
    def magnitude(self) -> float:
        return float(np.linalg.norm(self.force))


@dataclass
class EnvelopeMember:
    """Worst axial a single member sees. Tension and compression are tracked
    separately — a link is sized by its peak tension (yield) AND its peak
    compression (buckling), which usually come from different cases."""
    name: str
    inboard: np.ndarray
    outboard: np.ndarray
    worst_axial_lb: float          # largest magnitude, signed
    max_tension_lb: float
    max_compression_lb: float      # <= 0
    governing_case: str
    governing_pose: str

    @property
    def state(self) -> str:
        return "tension" if self.worst_axial_lb >= 0 else "compression"

    @property
    def length_mm(self) -> float:
        return float(np.linalg.norm(self.outboard - self.inboard))


@dataclass
class EnvelopeResult:
    mounts: list
    members: list
    conditions: dict
    case_names: list
    n_poses: int
    notes: list = field(default_factory=list)

    def worst_mount(self):
        return max(self.mounts, key=lambda m: m.magnitude)

    def worst_member(self):
        return max(self.members, key=lambda m: abs(m.worst_axial_lb))


def envelope(hp, loads, solver=None, travels=None, conditions=None
             ) -> EnvelopeResult:
    """Run the whole load matrix (`loads`, a list of WheelLoad) through the
    load-path solver at every travel pose and keep, per pickup and per
    member, the WORST result — the design envelope. This is the number that
    sizes the hardware, not any single case.

    Pass `solver` + `travels` to sweep through the travel range (double
    wishbone); omit them to evaluate at static geometry."""
    best_mount, best_member = {}, {}
    n_poses = 0
    for pose_label, posed, cp in _poses(hp, solver, travels):
        n_poses += 1
        for wl in loads:
            pm = wl.moment if np.any(wl.moment) else None
            r = corner_loads(posed, wl.force, contact_patch=cp,
                             patch_moment=pm)
            for m in r.mounts:
                cur = best_mount.get(m.name)
                if cur is None or m.magnitude > cur.magnitude:
                    best_mount[m.name] = EnvelopeMount(
                        m.name, np.asarray(m.point, float),
                        np.asarray(m.force, float), m.is_chassis,
                        wl.name, pose_label)
            for mem in r.members:
                e = best_member.get(mem.name)
                if e is None:
                    best_member[mem.name] = EnvelopeMember(
                        mem.name, np.asarray(mem.inboard, float),
                        np.asarray(mem.outboard, float), mem.axial_lb,
                        max(mem.axial_lb, 0.0), min(mem.axial_lb, 0.0),
                        wl.name, pose_label)
                else:
                    e.max_tension_lb = max(e.max_tension_lb, mem.axial_lb)
                    e.max_compression_lb = min(e.max_compression_lb,
                                               mem.axial_lb)
                    if abs(mem.axial_lb) > abs(e.worst_axial_lb):
                        e.worst_axial_lb = mem.axial_lb
                        e.governing_case = wl.name
                        e.governing_pose = pose_label
    notes = [
        "ENVELOPE: worst force per pickup / worst axial per member over the "
        "whole load matrix" + (" x travel sweep" if n_poses > 1 else
                               " (static geometry)") + ".",
        "Each member lists peak tension AND peak compression — size for "
        "both (yield in tension, buckling in compression).",
        "Quasi-static, rigid links, spherical joints — see the module docs.",
    ]
    return EnvelopeResult(
        mounts=list(best_mount.values()),
        members=list(best_member.values()),
        conditions=conditions or {}, case_names=[wl.name for wl in loads],
        n_poses=n_poses, notes=notes)


# ----------------------------------------------------------------------
# FEA export
# ----------------------------------------------------------------------
def loads_csv(case: ImpactCase, result: CornerLoadResult,
              unit_mm: float = 1.0, unit_label: str = "mm",
              force_in_newtons: bool = False, conv=None) -> str:
    """CSV of the mount loads + member forces, ready for ANSYS / Onshape.

    One row per pickup: name, x, y, z, Fx, Fy, Fz, |F|. Comment (#) lines
    carry the scenario so the file is self-describing. `conv` remaps the
    coordinates into the CAD display frame exactly like the hardpoint
    export, so the points line up with the model you apply them to."""
    from .project import _conv_or_identity
    conv = _conv_or_identity(conv)
    fu = LB_TO_N if force_in_newtons else 1.0
    flabel = "N" if force_in_newtons else "lb"
    s = case.summary()
    lines = [
        f"# MICKSUS impact load case: {s['name']}",
        f"# direction,{s['direction']}",
        f"# delta_v_mph,{s['delta_v_mph']:.4f}",
        f"# contact_time_s,{s['contact_time_s']:.4f}",
        f"# impulse_lb_s,{s['impulse_lb_s']:.4f}",
        f"# average_force_lb,{s['average_force_lb']:.4f}",
        f"# pulse_shape,{s['pulse']} (peak/avg {s['peak_factor']:.3f})",
        f"# corner_share,{s['corner_share']:.4f}",
        f"# safety_factor,{s['safety_factor']:.4f}",
        f"# corner_force_lb,{s['corner_force_lb']:.4f}",
        f"# equivalent_g,{s['equivalent_g']:.4f}",
        f"# wheel_load_{flabel},"
        + ",".join(f"{v * fu:.4f}" for v in result.wheel_load_lb),
        f"# coords,{conv.labels[0]},{conv.labels[1]},{conv.labels[2]}"
        f" ({conv.name}); lengths in {unit_label}; forces in {flabel}",
        "# NOTE quasi-static peak load — see the module docs for limits",
        "# apply_to: 'chassis' rows are the frame FEA load set; 'internal'"
        " rows are ball-joint / hinge / kingpin loads for sizing the joint"
        " and upright (they cancel inside the corner)",
        f"name,x_{unit_label},y_{unit_label},z_{unit_label},"
        f"Fx_{flabel},Fy_{flabel},Fz_{flabel},magnitude_{flabel},apply_to",
    ]
    for m in result.mounts:
        p = conv.to_display(np.asarray(m.point, float))
        f = conv.to_display(np.asarray(m.force, float)) * fu
        lines.append(
            f"{m.name},{p[0] / unit_mm:.4f},{p[1] / unit_mm:.4f},"
            f"{p[2] / unit_mm:.4f},{f[0]:.4f},{f[1]:.4f},{f[2]:.4f},"
            f"{m.magnitude * fu:.4f},"
            f"{'chassis' if m.is_chassis else 'internal'}")
    lines.append("# --- member axial forces (+ tension / - compression) ---")
    lines.append(f"member,axial_{flabel},state,length_{unit_label}")
    for mem in result.members:
        lines.append(f"{mem.name},{mem.axial_lb * fu:.4f},{mem.state},"
                     f"{mem.length_mm / unit_mm:.4f}")
    return "\n".join(lines) + "\n"


def export_loads_csv(path: str, case: ImpactCase, result: CornerLoadResult,
                     **kw) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(loads_csv(case, result, **kw))


def envelope_csv(result: EnvelopeResult, unit_mm: float = 1.0,
                 unit_label: str = "mm", force_in_newtons: bool = False,
                 conv=None) -> str:
    """CSV of the load ENVELOPE (worst per pickup / member over the whole
    load matrix x travel sweep). Each row also names the governing case and
    pose, so you can trace where a peak came from. Ready for ANSYS / Onshape:
    apply the chassis rows to the frame; the member rows size the links."""
    from .project import _conv_or_identity
    conv = _conv_or_identity(conv)
    fu = LB_TO_N if force_in_newtons else 1.0
    flabel = "N" if force_in_newtons else "lb"
    c = result.conditions
    lines = [
        "# MICKSUS load-case ENVELOPE (Lotus-style load matrix)",
        f"# corner,{c.get('corner', '?')}",
        f"# surface,{c.get('surface', '?')} (mu {c.get('mu', 0):.2f})",
        f"# design_Fz_lb,{c.get('fz_lb', 0):.1f} "
        f"(transfer-loaded; static corner {c.get('static_corner_lb', 0):.1f})",
        f"# operating,ay {c.get('ay_g', 0):.2f} g / ax {c.get('ax_g', 0):.2f} g",
        f"# cases,{len(result.case_names)}: " + " ".join(result.case_names),
        f"# travel_poses,{result.n_poses}",
        f"# coords,{conv.labels[0]},{conv.labels[1]},{conv.labels[2]}"
        f" ({conv.name}); lengths in {unit_label}; forces in {flabel}",
        "# apply_to: 'chassis' rows are the frame FEA load set; 'internal'"
        " rows size the joint/upright",
    ]
    for note in result.notes:
        lines.append(f"# {note}")
    lines.append(
        f"name,x_{unit_label},y_{unit_label},z_{unit_label},"
        f"Fx_{flabel},Fy_{flabel},Fz_{flabel},magnitude_{flabel},apply_to,"
        f"governing_case,governing_pose")
    for m in sorted(result.mounts, key=lambda x: -x.magnitude):
        p = conv.to_display(np.asarray(m.point, float))
        f = conv.to_display(np.asarray(m.force, float)) * fu
        lines.append(
            f"{m.name},{p[0] / unit_mm:.4f},{p[1] / unit_mm:.4f},"
            f"{p[2] / unit_mm:.4f},{f[0]:.4f},{f[1]:.4f},{f[2]:.4f},"
            f"{m.magnitude * fu:.4f},"
            f"{'chassis' if m.is_chassis else 'internal'},"
            f"{m.governing_case},{m.governing_pose}")
    lines.append("# --- member axial envelope (+ tension / - compression) ---")
    lines.append(
        f"member,worst_axial_{flabel},max_tension_{flabel},"
        f"max_compression_{flabel},state,length_{unit_label},"
        f"governing_case,governing_pose")
    for mem in sorted(result.members, key=lambda x: -abs(x.worst_axial_lb)):
        lines.append(
            f"{mem.name},{mem.worst_axial_lb * fu:.4f},"
            f"{mem.max_tension_lb * fu:.4f},{mem.max_compression_lb * fu:.4f},"
            f"{mem.state},{mem.length_mm / unit_mm:.4f},"
            f"{mem.governing_case},{mem.governing_pose}")
    return "\n".join(lines) + "\n"


def export_envelope_csv(path: str, result: EnvelopeResult, **kw) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(envelope_csv(result, **kw))


def body_loads_csv(result: CornerLoadResult, unit_mm: float = 1.0,
                   unit_label: str = "mm", force_in_newtons: bool = False,
                   conv=None, include_beams: bool = True) -> str:
    """CSV of the PER-PART load sets — what you apply to an individual
    component (control arm, upright) in ANSYS / Onshape Simulation.

    Each body's rows are SELF-EQUILIBRATED (ΣF = 0, ΣM = 0), so the solver
    needs only a weak restraint to kill rigid-body modes. This is the export
    that lets FEA show you the BENDING a mid-span shock mount puts into a
    control arm — bending never appears in the joint reactions, because it
    is an internal stress resultant."""
    from .project import _conv_or_identity
    conv = _conv_or_identity(conv)
    fu = LB_TO_N if force_in_newtons else 1.0
    flabel = "N" if force_in_newtons else "lb"
    mlabel = f"{flabel}*{unit_label}"
    lines = [
        "# MICKSUS per-body load sets (component FEA)",
        "# Each body's rows are self-equilibrated: apply them all to that "
        "part and add only a weak restraint to remove rigid-body motion.",
        f"# coords,{conv.labels[0]},{conv.labels[1]},{conv.labels[2]}"
        f" ({conv.name}); lengths in {unit_label}; forces in {flabel}",
        "# NOTE bending is an INTERNAL stress resultant — it does not appear "
        "in any joint reaction, only in the stressed part. That is why a "
        "mid-span shock mount needs this export, not the chassis one.",
        f"body,load,x_{unit_label},y_{unit_label},z_{unit_label},"
        f"Fx_{flabel},Fy_{flabel},Fz_{flabel},magnitude_{flabel}",
    ]
    for name, body in sorted(result.bodies.items()):
        for lab, p, f in body.loads:
            pd = conv.to_display(np.asarray(p, float))
            fd = conv.to_display(np.asarray(f, float)) * fu
            lines.append(
                f"{name},{lab},{pd[0] / unit_mm:.4f},{pd[1] / unit_mm:.4f},"
                f"{pd[2] / unit_mm:.4f},{fd[0]:.4f},{fd[1]:.4f},{fd[2]:.4f},"
                f"{float(np.linalg.norm(f)) * fu:.4f}")
        for c in body.couples:
            cd = conv.to_display(np.asarray(c, float)) * fu
            lines.append(f"# {name},couple (pure moment {mlabel}),,,,"
                         f"{cd[0]:.4f},{cd[1]:.4f},{cd[2]:.4f},")
        lines.append(f"# {name} check: |sum F| = {body.force_residual():.3e} "
                     f"{flabel}, |sum M| = "
                     f"{body.moment_residual(body.loads[0][1]):.3e} {mlabel}")
    if include_beams:
        lines.append("# --- arm internal loads (bending / torsion / shear) ---")
        lines.append(
            f"leg,station_{unit_label},axial_{flabel},shear_{flabel},"
            f"bending_{mlabel},torsion_{mlabel}")
        for name, body in sorted(result.bodies.items()):
            try:
                beams = control_arm_beams(body)
            except ValueError:
                continue                       # not a two-bushing arm
            for beam in beams:
                for note in beam.notes:
                    lines.append(f"# {beam.name}: {note}")
                for s in beam.sections:
                    lines.append(
                        f"{beam.name},{s.station_mm / unit_mm:.4f},"
                        f"{s.axial_lb * fu:.4f},{s.shear_lb * fu:.4f},"
                        f"{s.bending_lb_mm * fu:.4f},"
                        f"{s.torsion_lb_mm * fu:.4f}")
    return "\n".join(lines) + "\n"


def export_body_loads_csv(path: str, result: CornerLoadResult, **kw) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(body_loads_csv(result, **kw))


# Ready-made scenarios a Baja team actually checks.
STANDARD_CASES = {
    "head_on_rock": ImpactCase(
        name="head-on rock strike", direction="head_on",
        speed_mph=15.0, final_speed_mph=0.0, contact_time_s=0.15,
        corner_share=0.5, pulse="half_sine"),
    "curb_strike_lateral": ImpactCase(
        name="lateral kerb strike", direction="lateral",
        speed_mph=10.0, final_speed_mph=0.0, contact_time_s=0.10,
        corner_share=0.5, pulse="half_sine"),
    "landing_3ft": ImpactCase(
        name="landing from 3 ft", direction="vertical",
        # v = sqrt(2*g*h): 3 ft = 36 in -> 167 in/s -> 9.5 mph
        speed_mph=float(np.sqrt(2 * G_IN_S2 * 36.0) / MPH_TO_IN_S),
        final_speed_mph=0.0, contact_time_s=0.25,
        corner_share=0.25, pulse="half_sine"),
    "square_edge_bump": ImpactCase(
        name="square-edge bump at speed", direction="vertical",
        speed_mph=4.0, final_speed_mph=0.0, contact_time_s=0.05,
        corner_share=0.25, pulse="triangular"),
}


def drop_height_to_mph(height_in: float) -> float:
    """Impact speed from a free-fall drop height (in) -> mph."""
    return float(np.sqrt(2.0 * G_IN_S2 * max(0.0, height_in)) / MPH_TO_IN_S)
