"""Every equation the tool uses, in one place — the hand-check reference.

WHY THIS EXISTS: the software reports a lot of numbers, and the only way to
trust them is to be able to reproduce one by hand. This module is the single
source of truth for that: the in-app Help > Equations window and the
printable PDF are both rendered from the data below, so they cannot drift
apart, and neither can drift from this file without someone editing it.

Each entry carries the formula, what every symbol means, and which module
implements it, so you can go from a number on screen to the line of code.

CONVENTION NOTE: the solver works internally in millimetres with
+X forward, +Y left, +Z up; the dynamics layer works in INCHES, POUNDS and
SECONDS (so mass = W/g with g = 386.4 in/s^2). Mixing the two is the single
most common hand-calculation error — every equation below states its units.
"""

from dataclasses import dataclass, field

G_IN_S2 = 386.4          # gravity, in/s^2 — the imperial-unit gotcha
MM_PER_IN = 25.4


@dataclass(frozen=True)
class Equation:
    """One formula, with its symbol definitions and where it lives."""
    name: str
    expr: str                       # the formula itself (plain text)
    where: str = ""                 # symbol definitions, ';'-separated
    note: str = ""                  # why / caveats / sign conventions
    module: str = ""                # implementing module


@dataclass(frozen=True)
class Section:
    title: str
    blurb: str = ""
    equations: list = field(default_factory=list)


# ======================================================================
SYMBOLS = [
    ("W", "total vehicle weight incl. driver", "lb"),
    ("W_F, W_R", "front / rear AXLE weight (with driver)", "lb"),
    ("W_u", "unsprung weight (per side unless noted)", "lb"),
    ("W_s", "sprung weight = W - total unsprung", "lb"),
    ("L", "wheelbase", "in"),
    ("t", "track width (t_f front, t_r rear)", "in"),
    ("a, b", "sprung CG to FRONT / REAR axle (a + b = L)", "in"),
    ("h", "CG height above ground", "in"),
    ("h_s", "SPRUNG-mass CG height above ground", "in"),
    ("H", "roll moment arm = h_s - roll-axis height at the CG", "in"),
    ("z_RC", "roll-centre height above ground", "in"),
    ("r_l", "loaded tire radius", "in"),
    ("K_s", "spring rate (at the spring)", "lb/in"),
    ("K_w", "wheel rate = K_s x MR^2", "lb/in"),
    ("K_t", "tire vertical rate", "lb/in"),
    ("K_r", "ride rate = K_w in series with K_t", "lb/in"),
    ("K_phi", "axle roll rate", "lb-ft/deg"),
    ("MR", "motion ratio = SPRING (shock) travel / wheel travel; < 1", "-"),
    ("c", "damper coefficient (c_w = at the wheel)", "lb-s/in"),
    ("zeta", "damping ratio c / c_crit", "-"),
    ("A_y, A_x", "lateral / longitudinal acceleration (+A_x = braking)", "g"),
    ("m_s, m_u", "sprung / unsprung MASS = weight / 386.4", "lb-s^2/in"),
    ("g", "gravity — 386.4 in/s^2 in this tool's imperial units", "in/s^2"),
    ("f", "frequency", "Hz"),
    ("omega", "angular frequency = 2*pi*f", "rad/s"),
    ("s", "Laplace variable; s = j*omega for frequency response", "-"),
    ("mu", "peak friction coefficient", "-"),
    ("C_alpha", "cornering stiffness (per axle unless noted)", "lb/deg"),
    ("T", "axial TENSION in a two-force member (+ = pulled)", "lb"),
    ("n_hat", "unit vector along a member, inboard -> outboard", "-"),
]


SECTIONS = [

    # ------------------------------------------------------------------
    Section("1 · Conventions", """Internal geometry is millimetres in a
right-handed frame: +X forward, +Y left, +Z up, origin at ground level on
the centreline. The tool models the LEFT corner and mirrors it. Display and
CAD export default to the CHASSIS convention (+Y forward). The dynamics
layer is imperial: inches, pounds, seconds.""", [
        Equation("Mass from weight",
                 "m = W / g,   g = 386.4 in/s^2",
                 "W lb; m lb-s^2/in",
                 "Using 32.2 instead of 386.4 is the classic factor-of-12 "
                 "error. 386.4 = 32.2 ft/s^2 x 12 in/ft.",
                 "dynamics.py, impact.py, ride_freq.py"),
        Equation("Angle sign conventions",
                 "camber < 0 = top of tire leans toward the centreline; "
                 "toe > 0 = toe-IN; caster > 0 = kingpin top rearward; "
                 "KPI > 0 = kingpin top inboard",
                 "", "Matches SAE J670.", "metrics.py"),
    ]),

    # ------------------------------------------------------------------
    Section("2 · Kinematic solver core", """The double-wishbone corner has
one degree of freedom once the tie rod is fixed. It is solved in closed
form — no iteration except a 1-D root find for the requested wheel travel.""",
            [
        Equation("Arm rotation (Rodrigues)",
                 "p' = o + r*cos(t) + (a_hat x r)*sin(t) "
                 "+ a_hat*(a_hat . r)*(1 - cos(t)),   r = p - o",
                 "o point on the pivot axis; a_hat unit axis; t rotation angle",
                 "Rotates a ball joint about its arm's chassis pivot axis.",
                 "solver.py"),
        Equation("Upper ball joint: circle ∩ sphere",
                 "A*cos(b) + B*sin(b) = C,   b = phi ± delta",
                 "A = 2*w.u; B = 2*w.v; C = l^2 - w.w - u.u; "
                 "w = c - LBJ; u radius vector; v = axis x u; "
                 "l = kingpin length; phi = atan2(B, A); "
                 "delta = arccos(C / sqrt(A^2 + B^2))",
                 "The UBJ lies on the circle swept by the upper arm AND on "
                 "the sphere of fixed kingpin length about the LBJ. The two "
                 "roots are the two ASSEMBLY BRANCHES; the branch is fixed "
                 "once from the static pose and never allowed to switch.",
                 "solver.py"),
        Equation("Tie-rod outer: trilateration",
                 "x = (r1^2 - r2^2 + d^2) / (2d);  "
                 "y = (r1^2 - r3^2 + i^2 + j^2)/(2j) - (i/j)*x;  "
                 "z = ± sqrt(r1^2 - x^2 - y^2)",
                 "r1,r2,r3 distances from LBJ, UBJ, tie-rod inner; "
                 "d = |UBJ - LBJ|; i, j local-frame coordinates of the third "
                 "centre",
                 "Intersection of three spheres. The ± sign is the mirror "
                 "solution, fixed from the static geometry.",
                 "solver.py"),
        Equation("Upright pose (Kabsch)",
                 "H = A^T B;   U S V^T = svd(H);   "
                 "R = V * diag(1, 1, det(V U^T)) * U^T",
                 "A static upright points (centred); B current points "
                 "(centred)",
                 "Exact here, not a fit: three rigid points fully define the "
                 "pose. Carries the wheel centre and spindle axis along.",
                 "solver.py"),
        Equation("Contact patch",
                 "CP = WC + R_tire * d_hat,   "
                 "d_hat = unit(s_hat x unit(s_hat x z_hat))",
                 "WC wheel centre; s_hat spindle axis; R_tire tire radius",
                 "Goes one tire radius 'down' WITHIN the wheel plane, so "
                 "camber tips the patch sideways as on a real wheel.",
                 "solver.py"),
    ]),

    # ------------------------------------------------------------------
    Section("3 · Wheel orientation angles", """All read off the solved "
spindle axis s_hat and the kingpin vector k = UBJ - LBJ.""", [
        Equation("Camber", "gamma = -atan2(s_z, s_y)", "s_hat spindle unit "
                 "vector (points outboard, +Y on the left corner)",
                 "Negative = top of the tire toward the centreline.",
                 "metrics.py"),
        Equation("Toe", "tau = -atan2(f_y, f_x),   f_hat = unit(s_hat x z_hat)",
                 "f_hat wheel heading in top view",
                 "Positive = toe-in.", "metrics.py"),
        Equation("Caster", "caster = atan2(-k_x, k_z)",
                 "k = UBJ - LBJ", "Positive = kingpin top rearward.",
                 "metrics.py"),
        Equation("Kingpin inclination", "KPI = atan2(-k_y, k_z)", "k as above",
                 "Positive = kingpin top inboard (left corner).",
                 "metrics.py"),
        Equation("Kingpin ground intercept",
                 "P = LBJ + [(CP_z - LBJ_z)/k_z] * k",
                 "P where the steering axis pierces the ground plane through "
                 "the contact patch",
                 "Uses the patch's own z, not z = 0 — the CAD-aligned origin "
                 "puts ground below zero.",
                 "metrics.py"),
        Equation("Scrub radius", "r_scrub = CP_y - P_y", "",
                 "Positive = contact patch outboard of the steering axis.",
                 "metrics.py"),
        Equation("Mechanical (caster) trail", "trail = P_x - CP_x", "",
                 "Positive = patch behind the steering axis (stabilising).",
                 "metrics.py"),
    ]),

    # ------------------------------------------------------------------
    Section("4 · Instant centre & roll centre", """Built in the FRONT VIEW —
the transverse vertical plane (Y-Z), per SAE J670 — NOT the tilted 2-D
manufacturing sketch plane. Each arm is sliced at its OWN ball-joint
station.""", [
        Equation("Arm-plane normal",
                 "n = (inner_r - inner_f) x (outer - inner_f)",
                 "inner_f, inner_r the two chassis bushings; outer the ball "
                 "joint",
                 "The ball joint's velocity is exactly parallel to n, which "
                 "is what makes the construction exact for a spatial arm.",
                 "metrics.py"),
        Equation("Front-view arm line",
                 "n_y * y + n_z * z = n_y * BJ_y + n_z * BJ_z", "",
                 "Slicing BOTH arms at the shared wheel-centre plane instead "
                 "offsets each line by n_x*(BJ_x - x_wc) — zero only for "
                 "flat arms, tens of mm out at large kickup.",
                 "metrics.py"),
        Equation("Side-view arm line",
                 "n_x * x + n_z * z = n_x * BJ_x + n_z * BJ_z", "",
                 "Same construction in the X-Z plane; governs wheel "
                 "recession and anti-geometry.",
                 "metrics.py"),
        Equation("Instant centre",
                 "IC = solve( [a1 b1; a2 b2] [y; z] = [c1; c2] )",
                 "the two arm lines a*y + b*z = c",
                 "Parallel lines (det = 0) means the IC is at infinity — the "
                 "DESIGNED state for this tool's seeds.",
                 "metrics.py"),
        Equation("Roll-centre height",
                 "z_RC = (0 - CP_y) * (z_IC - CP_z) / (y_IC - CP_y)",
                 "CP contact patch; IC front-view instant centre",
                 "Height above the GROUND PLANE. The line runs contact patch "
                 "-> IC, extended to the centreline y = 0.",
                 "metrics.py"),
    ]),

    # ------------------------------------------------------------------
    Section("5 · Motion ratio, rates and anti-geometry", "", [
        Equation("Motion ratio",
                 "MR = d(shock length) / d(wheel travel)",
                 "MR < 1 for a shock mounted inboard on the arm",
                 "STANDARD convention (shock/wheel), so it feeds Kw = Ks*MR^2 "
                 "with NO inversion. Computed by numerical gradient over the "
                 "travel sweep, so it VARIES through travel. Before v1.28 "
                 "this tool reported the reciprocal (wheel/shock, > 1); "
                 "reading that into a textbook formula without inverting "
                 "gives a 3.4x error, which is why it was changed.",
                 "metrics.py"),
        Equation("Wheel rate", "K_w = K_s * MR^2",
                 "MR = shock travel / wheel travel, BELOW 1",
                 "Squared — the most common hand-calc slip. Damping goes the "
                 "same way: c_w = c * MR_damper^2. Since MR < 1 the wheel "
                 "rate is always LOWER than the spring rate; if your answer "
                 "comes out higher, the ratio is upside down.",
                 "dynamics.py"),
        Equation("Ride rate", "K_r = K_w * K_t / (K_w + K_t)", "",
                 "Spring and tire in SERIES. Sprung-mass modes use K_r, "
                 "never the bare wheel rate.",
                 "dynamics.py"),
        Equation("Spring stacks",
                 "Series: K = K1*K2/(K1+K2);   Parallel: K = K1 + K2", "",
                 "", "dynamics.py"),
        Equation("Anti-dive (front, braking)",
                 "%anti = 100 * f_b * tan(phi_f) * L / h",
                 "f_b front brake fraction; phi_f support angle from the "
                 "CONTACT PATCH to the side-view IC",
                 "Outboard brakes -> contact-patch line.", "vehicle.py"),
        Equation("Anti-lift (rear, braking)",
                 "%anti = 100 * (1 - f_b) * tan(phi_r) * L / h", "",
                 "", "vehicle.py"),
        Equation("Anti-squat (rear, drive)",
                 "%anti = 100 * tan(theta) * L / h",
                 "theta support angle from the WHEEL CENTRE to the side-view "
                 "IC",
                 "Inboard (chassis-mounted) drive -> wheel-centre line. It "
                 "differs from the dive/lift line by the loaded radius over "
                 "the SVSA length, so one slope cannot serve both.",
                 "vehicle.py"),
    ]),

    # ------------------------------------------------------------------
    Section("6 · Mass, CG and the roll axis", "", [
        Equation("CG longitudinal position",
                 "b = W_F * L / W;   a = L - b",
                 "b = CG to REAR axle; a = CG to FRONT axle", "",
                 "dynamics.py"),
        Equation("Sprung weight",
                 "W_s = W - 2*W_uf - 2*W_ur",
                 "W_uf, W_ur unsprung PER SIDE", "", "dynamics.py"),
        Equation("Sprung CG height",
                 "h_s = (W*h - 2*W_uf*r_lf - 2*W_ur*r_lr) / W_s", "",
                 "The unsprung mass sits at wheel-centre height, so removing "
                 "it raises the remaining CG.",
                 "dynamics.py"),
        Equation("Front sprung fraction",
                 "a_s = (W_F - 2*W_uf) / W_s", "", "", "dynamics.py"),
        Equation("Roll-axis height at the sprung CG",
                 "z_RA = z_RCf + (z_RCr - z_RCf) * (1 - a_s)", "",
                 "Linear interpolation along the roll axis.",
                 "dynamics.py"),
        Equation("Roll moment arm", "H = h_s - z_RA", "",
                 "THE handling lever. Shrink it and body roll falls.",
                 "dynamics.py"),
    ]),

    # ------------------------------------------------------------------
    Section("7 · Roll rates and roll gradient",
            "Milliken RCVD ch. 16.3. Note the unit juggling: track and arm "
            "lengths in FEET inside these.", [
        Equation("Anti-roll-bar contribution",
                 "K_ARB = K_bar * (IR * t / L_arm)^2",
                 "K_bar bar torsional rate lb-ft/deg; IR install ratio; "
                 "t track ft; L_arm bar lever arm ft", "", "dynamics.py"),
        Equation("Axle roll rate (springs + bar, in series with tires)",
                 "K_spring = 12 * K_w * t_ft^2 / 2   [lb-ft/rad];   "
                 "K_axle = series(K_ARB + K_spring, K_tire)", "",
                 "The tire is a series spring in roll too — ignoring it "
                 "overstates roll stiffness.",
                 "dynamics.py"),
        Equation("Roll gradient",
                 "d(phi)/d(A_y) = -W_s * (H/12) / (K_phi_f + K_phi_r)",
                 "H in inches, /12 to feet; result deg/g",
                 "Negative = the body leans OUT of the turn (normal).",
                 "dynamics.py"),
    ]),

    # ------------------------------------------------------------------
    Section("8 · Load transfer", """Milliken 18.4. The lateral transfer has
THREE reactions per axle, not one: the sprung mass reacts elastically
through the roll stiffness AND geometrically through the links at the roll
centre, and the unsprung mass reacts directly at the wheel centre. Both
sprung terms carry W_s — using total W and dropping the unsprung term
overstates the transfer by roughly 12%.""", [
        Equation("Lateral transfer, front axle",
                 "dW_f = (A_y / t_f) * [ W_s*H*K_phi_f/(K_phi_f + K_phi_r) "
                 "+ W_s*(b/L)*z_RCf + W_uf_axle*r_lf ]",
                 "all lengths in FEET; W_uf_axle = 2 x per-side unsprung; "
                 "t_f front track ft",
                 "Rear is the mirror with a/L and z_RCr.", "dynamics.py"),
        Equation("Longitudinal transfer", "dW_x = A_x * W * h / L", "",
                 "Positive A_x = braking = load onto the front.",
                 "dynamics.py"),
        Equation("Banked-road lateral demand",
                 "A_y_eff = A_y*cos(beta) - sin(beta)",
                 "beta bank angle, POSITIVE = banked INTO the turn",
                 "Positive bank HELPS, hence the minus. SAE sign.",
                 "dynamics.py"),
        Equation("Aerodynamic download",
                 "F_z = 0.5 * rho * V^2 * C_L*A;   "
                 "ride drop = F_z_axle / (2*K_r)", "", "", "dynamics.py"),
        Equation("Dynamic corner loads",
                 "F_out = W_axle/2 + dW_lat ± dW_x/2 + F_aero/2;   "
                 "F_in  = W_axle/2 - dW_lat ± dW_x/2 + F_aero/2",
                 "+dW_x/2 front, -dW_x/2 rear under braking",
                 "A negative result means that wheel has LIFTED.",
                 "dynamics.py"),
        Equation("Lateral load transfer distribution",
                 "TLLTD = dW_f / (dW_f + dW_r)", "",
                 "Above the static front weight fraction = understeer trend.",
                 "dynamics.py"),
    ]),

    # ------------------------------------------------------------------
    Section("9 · Ride frequencies and damping", "", [
        Equation("Corner ride frequency",
                 "f_ride = (1/2pi) * sqrt( K_r * g / W_sprung_corner )", "",
                 "Uses the RIDE rate (spring + tire in series).",
                 "dynamics.py"),
        Equation("Bounce frequency",
                 "f_bounce = (1/2pi) * sqrt( (2*K_rf + 2*K_rr) / m_s )", "",
                 "Decoupled approximation — the full-car model solves the "
                 "coupled eigenproblem instead.",
                 "dynamics.py"),
        Equation("Pitch radius of gyration (Olley)",
                 "k_y = sqrt(a*b)", "",
                 "Dynamic index = 1, the flat-ride assumption. An ESTIMATE "
                 "until you measure it.", "dynamics.py"),
        Equation("Pitch frequency",
                 "f_pitch = (1/2pi) * sqrt( (2*K_rf*a^2 + 2*K_rr*b^2) "
                 "/ (m_s * a * b) )", "", "", "dynamics.py"),
        Equation("Critical damping and damping ratio",
                 "c_crit = 2*sqrt(K_w * m_s_corner);   "
                 "zeta = c_wheel / c_crit,   c_wheel = c_damper * MR_d^2",
                 "", "Racing practice is zeta ~0.3-0.5 in bump.",
                 "dynamics.py"),
        Equation("Olley ratio", "olley = f_rear / f_front", "",
                 "Target 1.1-1.2 — rear stiffer so the car settles flat "
                 "rather than pitching.",
                 "dynamics.py"),
    ]),

    # ------------------------------------------------------------------
    Section("10 · Steering and brakes", "", [
        Equation("Understeer gradient",
                 "K_us = W_F / C_alpha_F - W_R / C_alpha_R", "",
                 "Positive = understeer. deg/g.", "dynamics.py"),
        Equation("Ackermann percentage",
                 "%Ack = 100 * (d_i - d_o) / (d_ideal - d_o);   "
                 "cot(d_o) - cot(d_i) = t / L",
                 "d_i, d_o inner / outer wheel steer angles",
                 "100 = perfect Ackermann, 0 = parallel steer, negative = "
                 "anti-Ackermann.", "sweeps.py"),
        Equation("Low-speed turn diameter",
                 "D = 2 * sqrt( (R_rear + t/2)^2 + L^2 ),   "
                 "R_rear = L / tan(mean steer)", "", "", "sweeps.py"),
        Equation("Brake chain",
                 "F_mc = F_pedal * pedal_ratio * bias;   "
                 "P = F_mc / (pi*(bore/2)^2);   clamp = P * A_caliper;   "
                 "T = 2 * (2 * mu_pad * clamp * r_eff);   "
                 "F_brake = T / r_l;   decel = sum(F_brake) / W",
                 "r_eff effective rotor radius to the pad centre",
                 "The 2 x (2 x ...) is two wheels per axle, two pads per "
                 "caliper.", "dynamics.py"),
    ]),

    # ------------------------------------------------------------------
    Section("11 · Tire model",
            "All force-and-moment values for the SunF 23x7-10 are ESTIMATES "
            "— no measured data exists. Treat outputs as qualitative.", [
        Equation("Saturating lateral force",
                 "F_y = mu*F_z * tanh( C_alpha / (mu*F_z) * alpha )",
                 "alpha slip angle deg; C_alpha per-tire cornering stiffness",
                 "Matches the cornering stiffness at small slip and "
                 "saturates at mu*F_z. Stand-in for a fitted Pacejka curve.",
                 "tire.py"),
        Equation("Cornering stiffness from load",
                 "C_alpha = CC * F_z",
                 "CC normalised cornering coefficient, ~0.11 /deg for a "
                 "knobby (street radials 0.15-0.25)", "", "tire.py"),
        Equation("Peak-slip marker",
                 "alpha_peak = (mu*F_z / C_alpha) * artanh(0.98)", "",
                 "Where the model reaches 98% of saturation.", "tire.py"),
        Equation("Pacejka seed coefficients",
                 "D = mu*F_z;   C ~ 1.40;   B = C_alpha_rad / (C*D);   "
                 "E ~ 0.90", "",
                 "Built from the estimated peak and initial slope — NOT a "
                 "fit to data.", "tire.py"),
        Equation("Tire vertical rate vs pressure",
                 "K_t ~ 65 + 22 * psi   [lb/in]", "",
                 "Informed estimate for a small bias tire; verify on your "
                 "own tire.", "tire.py"),
    ]),

    # ------------------------------------------------------------------
    Section("12 · Halfshaft / CV joints", "", [
        Equation("Joint working angle",
                 "angle = arccos( |u_hat . v_hat| )",
                 "u_hat shaft direction; v_hat the joint's reference axis "
                 "(gearbox output for the inner, spindle for the outer)",
                 "Absolute value: the joint does not care about sign.",
                 "halfshaft.py"),
        Equation("Plunge budget",
                 "plunge_in = total * pct/100;   "
                 "pull_out = total * (1 - pct/100)", "",
                 "Measured from the RIDE-HEIGHT shaft length, and "
                 "asymmetric — ride height rarely sits centred in the "
                 "joint's stroke.",
                 "halfshaft.py"),
    ]),

    # ------------------------------------------------------------------
    Section("13 · Impact loads (FEA export)", """Impulse-momentum for the
wheel load, then rigid-body statics for the load path. Quasi-static: the
peak force is applied as a static load.""", [
        Equation("Impulse and average force",
                 "J = m * dv,   m = W/g;   F_avg = J / dt", "",
                 "dt is the CONTACT TIME. Halving it doubles every mount "
                 "load, which is why measuring it matters.",
                 "impact.py"),
        Equation("Design peak force",
                 "F_peak = F_avg * pulse * share * SF",
                 "pulse: rectangular 1.00, half-sine pi/2 = 1.571, "
                 "triangular 2.00; share = fraction of the car this corner "
                 "takes; SF safety factor", "", "impact.py"),
        Equation("Drop height to impact speed",
                 "v = sqrt(2*g*height)", "", "", "impact.py"),
        Equation("Friction-circle combined load",
                 "|F_horizontal| = mu * F_z,   swept in direction;   "
                 "M_z = -F_y * pneumatic trail", "",
                 "The worst member load is a COMBINED case, never one axis "
                 "alone.", "impact.py"),
        Equation("Rigid-body equilibrium (per body)",
                 "sum(F) = 0;   sum(M) = 0", "",
                 "Six equations per body, solved by least squares.",
                 "impact.py"),
        Equation("Two-force member",
                 "force on the OUTBOARD body = -T * n_hat;   "
                 "force on the INBOARD (chassis) = +T * n_hat",
                 "T tension (+ = pulled); n_hat = unit(outboard - inboard)",
                 "Getting this sign wrong breaks equilibrium silently.",
                 "impact.py"),
        Equation("Control-arm reduced equation",
                 "a_hat . sum( (r_i - p1) x F_i ) = 0",
                 "a_hat the arm's bushing axis; p1 one bushing; F_i every "
                 "load on the arm (ball joint, shock, arm-mounted toe link)",
                 "Two spherical bushings ON A LINE produce no moment about "
                 "that line, so dotting the moment equation with the axis "
                 "annihilates both bushing unknowns and leaves exactly one "
                 "scalar constraint. This is why the shock's position along "
                 "the arm changes the answer.",
                 "impact.py"),
        Equation("Indeterminate bushing split",
                 "min ||R||  subject to equilibrium  (min-norm least "
                 "squares)", "",
                 "Two spherical joints on a line cannot resolve force ALONG "
                 "that line. The TOTAL axial is reported; size a bracket "
                 "against all of it, not half.", "impact.py"),
        Equation("Beam internal loads",
                 "F_internal(s) = -sum( F_i : station_i <= s );   "
                 "M_internal(s) = -sum( (p_i - cut) x F_i )", "",
                 "Bending is an INTERNAL stress resultant — it never appears "
                 "in a joint reaction, which is why it needs its own output.",
                 "impact.py"),
        Equation("Rocker (bellcrank)",
                 "a_hat . [ (r_pr) x F_pushrod + D * (r_d) x u_hat_d ] = 0",
                 "a_hat pivot axis; r measured from the pivot; D damper "
                 "axial force",
                 "One equation for the damper force; force balance then "
                 "gives the pivot bearing reaction.", "impact.py"),
        Equation("Virtual-work check",
                 "F_shock = F_wheel / MR",
                 "MR = shock/wheel < 1, so the shock force EXCEEDS the wheel "
                 "force",
                 "INDEPENDENT of the statics — it uses only the kinematic "
                 "sweep. The single best check that a load path is right.",
                 "impact.py"),
    ]),

    # ------------------------------------------------------------------
    Section("14 · Quarter-car frequency response", """The Bode plot. Sprung
mass on the wheel-rate spring and damper, unsprung mass on the tire
spring.""", [
        Equation("Equations of motion",
                 "m_s*z_s'' = -K_w*(z_s - z_u) - c*(z_s' - z_u');   "
                 "m_u*z_u'' = +K_w*(z_s - z_u) + c*(z_s' - z_u') "
                 "- K_t*(z_u - z_r)",
                 "z_s body; z_u wheel; z_r road; ' = d/dt", "",
                 "ride_freq.py"),
        Equation("Frequency-response system",
                 "[ m_s*s^2 + c*s + K_w        -(c*s + K_w)          ] [Z_s]"
                 "   [ 0     ]\n"
                 "[ -(c*s + K_w)     m_u*s^2 + c*s + K_w + K_t ] [Z_u] = "
                 "[ K_t*Z_r ]",
                 "s = j*omega",
                 "Invert this 2x2 and every plotted curve is a ratio to Z_r.",
                 "ride_freq.py"),
        Equation("Body and wheel-hop frequencies",
                 "f_body ~ (1/2pi)*sqrt(K_ride/m_s);   "
                 "f_hop ~ (1/2pi)*sqrt((K_w + K_t)/m_u)", "",
                 "Approximations. The tool takes the exact values from the "
                 "undamped eigenproblem, which is damping-independent — "
                 "peak-picking a damped curve is unreliable near zeta = 1.",
                 "ride_freq.py"),
        Equation("Isolation crossover",
                 "transmissibility = 1 exactly at f = sqrt(2) * f_body", "",
                 "Below it you amplify the road; above it you isolate — and "
                 "above it MORE damping makes isolation WORSE.",
                 "ride_freq.py"),
        Equation("Terrain excitation",
                 "f = V / lambda", "V road speed; lambda bump spacing",
                 "Converts a resonance into the speed that excites it.",
                 "ride_freq.py"),
        Equation("Dynamic tire load / lift-off",
                 "dF_z = (tire deflection ratio) * K_t * bump;   "
                 "tire lifts when dF_z >= static corner load", "",
                 "How a dimensionless curve becomes a decision.",
                 "ride_freq.py"),
    ]),

    # ------------------------------------------------------------------
    Section("15 · Full-car (7-DOF) ride model", """Heave, roll and pitch of
the sprung mass plus four unsprung masses.""", [
        Equation("Sprung displacement at a corner",
                 "z_i = z_s + phi*y_i - theta*x_i",
                 "x_i, y_i corner position from the sprung CG (+x forward, "
                 "+y left); phi roll; theta pitch",
                 "From the rotation vector: dz = phi*y - theta*x. So "
                 "+theta = NOSE DOWN and +phi = LEFT SIDE UP.",
                 "full_car.py"),
        Equation("Assembled system",
                 "M*q'' + C*q' + K*q = F_road,   "
                 "q = [z_s, phi, theta, z_u1..z_u4]", "",
                 "A corner spring K adds K*b*b^T to the sprung block with "
                 "b = [1, y_i, -x_i], -K*b to the coupling, and K to the "
                 "unsprung diagonal.", "full_car.py"),
        Equation("Anti-roll bar stiffness",
                 "K adds k_bar * g * g^T,   g = b_left - b_right;   "
                 "k_bar = K_phi_ARB / track^2",
                 "K_phi in lb-in/rad = lb-ft/deg x 12 x 180/pi",
                 "Note track^2, NOT track^2/2 — the /2 in the spring term "
                 "comes from each wheel moving only half the track.",
                 "full_car.py"),
        Equation("Wheelbase filtering",
                 "Z_rear = Z_front * exp(-j*omega*tau),   tau = L / V", "",
                 "The rear axle meets each bump L/V after the front, so the "
                 "response depends on SPEED.", "full_car.py"),
        Equation("Comb nulls",
                 "pitch null at f = n*V/L   (axles in phase);   "
                 "heave null at f = (2n+1)*V/(2L)   (axles opposed)", "",
                 "Exact for a symmetric car; deep minima on a real one.",
                 "full_car.py"),
        Equation("Modal frequencies and labelling",
                 "solve det(K - omega^2 M) = 0;   "
                 "label by KE share = m_i * v_i^2 / sum", "",
                 "Seven bare frequencies are useless without knowing which "
                 "is bounce, pitch, roll or hop.", "full_car.py"),
        Equation("Froude scaling (model cars)",
                 "length ~ lambda;   time ~ sqrt(lambda);   "
                 "frequency ~ 1/sqrt(lambda);   velocity ~ sqrt(lambda)",
                 "lambda scale factor (0.1 for 1:10)",
                 "Gravity does not scale. At 1:10 frequencies are 3.16x "
                 "higher and speeds 3.16x lower for dynamic similarity.",
                 "full_car.py"),
    ]),
]


# ======================================================================
def all_equations():
    """Flat list of every Equation, in document order."""
    return [eq for sec in SECTIONS for eq in sec.equations]


def _esc(text: str) -> str:
    return (str(text).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;"))


def to_html() -> str:
    """The in-app Help > Equations page."""
    out = [
        "<h1>Equation reference</h1>",
        "<p><i>Every formula the tool uses, so you can reproduce any number "
        "by hand. Each entry names the module that implements it.</i></p>",
        "<p><b>Units warning.</b> Geometry is millimetres (+X forward, "
        "+Y left, +Z up, left corner). The dynamics layer is inches, pounds "
        "and seconds, so <b>mass = W / 386.4</b>. Using 32.2 instead of "
        "386.4 is the classic factor-of-12 error.</p>",
        "<h2>Symbols</h2><table width='100%' cellspacing='0' "
        "cellpadding='4'>",
        "<tr bgcolor='#D9E2F3'><th align='left'>Symbol</th>"
        "<th align='left'>Meaning</th><th align='left'>Units</th></tr>",
    ]
    for i, (sym, mean, unit) in enumerate(SYMBOLS):
        bg = "#F2F5FB" if i % 2 else "#FFFFFF"
        out.append(f"<tr bgcolor='{bg}'><td><b>{_esc(sym)}</b></td>"
                   f"<td>{_esc(mean)}</td><td>{_esc(unit)}</td></tr>")
    out.append("</table>")

    for sec in SECTIONS:
        out.append(f"<h2>{_esc(sec.title)}</h2>")
        if sec.blurb:
            out.append(f"<p><i>{_esc(' '.join(sec.blurb.split()))}</i></p>")
        for eq in sec.equations:
            out.append(f"<p><b>{_esc(eq.name)}</b><br>")
            out.append("<code style='background:#F2F5FB'>"
                       + _esc(eq.expr).replace("\n", "<br>") + "</code>")
            if eq.where:
                out.append(f"<br><small><b>where</b> {_esc(eq.where)}</small>")
            if eq.note:
                out.append(f"<br><small><i>{_esc(eq.note)}</i></small>")
            if eq.module:
                out.append(f"<br><small><font color='#707070'>"
                           f"{_esc(eq.module)}</font></small>")
            out.append("</p>")
    return "\n".join(out)
