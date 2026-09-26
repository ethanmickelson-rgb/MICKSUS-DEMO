"""Full-car (7-DOF) ride model — the whole-vehicle Bode plot.

The quarter car in `ride_freq.py` answers "how does ONE corner respond".
Three things it structurally cannot answer, and this module can:

  1. BOUNCE AND PITCH ARE COUPLED. A car does not heave and pitch
     independently — a bump at the front does both. The dynamics sheet
     reports `bounce_freq_hz` and `pitch_freq_hz` from the classic DECOUPLED
     formulas; solving the coupled eigenproblem gives the true modes (and
     they differ whenever the car is not at Olley's flat-ride condition).
  2. WHEELBASE FILTERING — the big one. The rear wheels hit the same bump the
     front wheels did, delayed by tau = L/V. So the excitation the CAR sees
     depends on SPEED, and the response is a comb filter:
         at f = n*V/L        the axles are IN phase  -> pure bounce, and
                             pitch has a NULL
         at f = (2n+1)*V/2L  the axles are OPPOSED   -> pure pitch, and
                             bounce has a NULL
     This is why a given set of whoops is brutal at one speed and calm at
     another, and no quarter-car model can show it.
  3. ROLL, AND THEREFORE THE ANTI-ROLL BARS. A quarter car has no roll DOF,
     so an ARB is invisible to it. Here the bars enter the roll stiffness
     and the roll mode is reported.

MODEL — 7 DOF:
    sprung mass:  z_s (heave), phi (roll), theta (pitch)
    unsprung:     z_u at each of the four corners
Each corner has its wheel-rate spring and damper to the sprung mass, and a
tire spring to the road. Anti-roll bars couple the two wheels of an axle.

SIGN CONVENTIONS (stated because they are easy to get backwards):
    +z up. Corner positions are measured from the SPRUNG-mass CG, with
    x = +a forward (front axle), x = -b rearward, y = +t/2 on the LEFT.
    A small rotation vector (phi, theta, 0) moves a point at (x, y) by
        dz = phi*y - theta*x
    so +theta is NOSE DOWN (it lowers the front) and +phi raises the LEFT
    side. These follow from the right-hand rule in the project's +X fwd,
    +Y left, +Z up frame; they are NOT the same as the chassis-vs-ground
    roll sign used in `sweeps.py`, which is a different measurement.

ROAD INPUT: both wheels of an axle see the same profile (the terrain that
matters for bounce/pitch — whoops, jumps, washboard), and the rear axle sees
it delayed by tau = L/V, i.e. multiplied by exp(-j*omega*tau). With a
left/right-symmetric car this never excites roll, which is correct and is
asserted in the tests; the roll MODE is still reported from the eigenproblem
because that is where you see what the bars do.

LIMITS: linear and small-perturbation, exactly as for the quarter car — no
bump stops, no progressive springs, no tire lift-off, no load transfer. The
INERTIAS ARE ESTIMATES: pitch from Olley's dynamic-index-1 (k_y^2 = a*b) and
roll from the `roll_radius_gyr_in` input, which defaults to an estimate.
Both are flagged in the report's notes. Treat the whole thing as a
design-target and comparison instrument.
"""

from dataclasses import dataclass, field

import numpy as np

G_IN_S2 = 386.4
MPH_TO_IN_S = 17.6

# Order of the generalized coordinates.
DOF_NAMES = ["heave", "roll", "pitch", "u_FL", "u_FR", "u_RL", "u_RR"]
CORNERS = ["FL", "FR", "RL", "RR"]


@dataclass
class FullCar:
    """Whole-vehicle ride model. Imperial: lb, in, s. All suspension rates
    are AT THE WHEEL (motion ratio already applied)."""

    sprung_lb: float
    a_in: float                  # sprung CG -> front axle (forward, +x)
    b_in: float                  # sprung CG -> rear axle
    track_front_in: float
    track_rear_in: float
    pitch_radius_gyr_in: float   # k_y  (I_y = m_s k_y^2)
    roll_radius_gyr_in: float    # k_x  (I_x = m_s k_x^2)

    unsprung_front_lb: float     # per side
    unsprung_rear_lb: float
    wheel_rate_front_lbin: float
    wheel_rate_rear_lbin: float
    tire_rate_front_lbin: float
    tire_rate_rear_lbin: float
    damp_front_lbsin: float      # at the wheel
    damp_rear_lbsin: float
    arb_wheel_rate_front_lbin: float = 0.0   # bar, as an equivalent wheel rate
    arb_wheel_rate_rear_lbin: float = 0.0

    # ---------------------------------------------------------------
    @property
    def wheelbase_in(self) -> float:
        return self.a_in + self.b_in

    @property
    def m_s(self) -> float:
        return self.sprung_lb / G_IN_S2

    @property
    def i_pitch(self) -> float:
        return self.m_s * self.pitch_radius_gyr_in ** 2

    @property
    def i_roll(self) -> float:
        return self.m_s * self.roll_radius_gyr_in ** 2

    def _corner_geometry(self):
        """(x, y) of each corner relative to the sprung CG, in FL/FR/RL/RR
        order. +x forward, +y left."""
        tf, tr = self.track_front_in / 2.0, self.track_rear_in / 2.0
        return [(self.a_in, tf), (self.a_in, -tf),
                (-self.b_in, tr), (-self.b_in, -tr)]

    def _corner_props(self):
        """(k_spring, c_damper, k_tire, m_unsprung) per corner."""
        f = (self.wheel_rate_front_lbin, self.damp_front_lbsin,
             self.tire_rate_front_lbin, self.unsprung_front_lb / G_IN_S2)
        r = (self.wheel_rate_rear_lbin, self.damp_rear_lbsin,
             self.tire_rate_rear_lbin, self.unsprung_rear_lb / G_IN_S2)
        return [f, f, r, r]

    # ---------------------------------------------------------------
    def matrices(self):
        """Assemble (M, C, K) for q = [z_s, phi, theta, u_FL, u_FR, u_RL,
        u_RR].

        The sprung-side displacement at corner i is z_i = b_i . p with
        b_i = [1, y_i, -x_i], so a corner spring k contributes k*b_i b_i^T to
        the sprung block, -k*b_i to the coupling, and k to the unsprung
        diagonal. Anti-roll bars act on the DIFFERENCE across an axle, which
        adds k_bar*g g^T with g = b_left - b_right."""
        n = 7
        M = np.zeros((n, n))
        C = np.zeros((n, n))
        K = np.zeros((n, n))
        M[0, 0] = self.m_s
        M[1, 1] = self.i_roll
        M[2, 2] = self.i_pitch
        geom = self._corner_geometry()
        props = self._corner_props()
        for i, ((x, y), (k, c, kt, mu)) in enumerate(zip(geom, props)):
            b = np.array([1.0, y, -x])
            j = 3 + i
            M[j, j] = mu
            # sprung block
            K[0:3, 0:3] += k * np.outer(b, b)
            C[0:3, 0:3] += c * np.outer(b, b)
            # coupling
            K[0:3, j] -= k * b
            K[j, 0:3] -= k * b
            C[0:3, j] -= c * b
            C[j, 0:3] -= c * b
            # unsprung diagonal: suspension + tire
            K[j, j] += k + kt
            C[j, j] += c
        # anti-roll bars: differential stiffness across each axle
        for (li, ri), k_bar in (((0, 1), self.arb_wheel_rate_front_lbin),
                                ((2, 3), self.arb_wheel_rate_rear_lbin)):
            if k_bar <= 0.0:
                continue
            (xl, yl), (xr, yr) = geom[li], geom[ri]
            g = np.array([1.0, yl, -xl]) - np.array([1.0, yr, -xr])
            e = np.zeros(4)
            e[li], e[ri] = 1.0, -1.0
            K[0:3, 0:3] += k_bar * np.outer(g, g)
            K[0:3, 3:7] -= k_bar * np.outer(g, e)
            K[3:7, 0:3] -= k_bar * np.outer(e, g)
            K[3:7, 3:7] += k_bar * np.outer(e, e)
        return M, C, K

    # ---------------------------------------------------------------
    def modes(self) -> list:
        """The seven undamped modes, each LABELLED from its eigenvector.

        Labels matter: seven bare frequencies are not much use unless you
        know which is bounce, which is pitch, and which are wheel hop. The
        label is the DOF carrying the largest share of the mode's kinetic
        energy, which is the standard way to name a mode shape."""
        M, _C, K = self.matrices()
        w2, vecs = np.linalg.eig(np.linalg.solve(M, K))
        order = np.argsort(np.real(w2))
        out = []
        for idx in order:
            f = float(np.sqrt(max(np.real(w2[idx]), 0.0)) / (2.0 * np.pi))
            v = np.real(vecs[:, idx])
            # kinetic-energy share per DOF: m_i * v_i^2 (M is diagonal here)
            energy = np.diag(M) * v ** 2
            share = energy / energy.sum() if energy.sum() > 0 else energy
            dom = int(np.argmax(share))
            if dom >= 3:
                axle = "front" if dom in (3, 4) else "rear"
                label = f"wheel hop ({axle})"
            else:
                label = ("heave/bounce", "roll", "pitch")[dom]
            out.append({
                "freq_hz": f, "label": label,
                "dominant_dof": DOF_NAMES[dom],
                "participation": float(share[dom]),
                "shape": v / (np.max(np.abs(v)) or 1.0),
            })
        return out

    def body_modes(self) -> dict:
        """Just the three sprung-mass modes, keyed by name — the ones that
        compare directly against the dynamics sheet."""
        out = {}
        for m in self.modes():
            key = m["label"].split("/")[0]
            if key in ("heave", "roll", "pitch") and key not in out:
                out[key] = m["freq_hz"]
        return out

    # ---------------------------------------------------------------
    def response(self, freq_hz, speed_mph: float) -> dict:
        """Forced response to a road profile traversed at `speed_mph`.

        Both wheels of an axle see the same input; the rear axle sees it
        delayed by tau = L/V. Everything is per INCH of road amplitude."""
        f = np.atleast_1d(np.asarray(freq_hz, float))
        M, C, K = self.matrices()
        v_in_s = max(float(speed_mph), 1e-6) * MPH_TO_IN_S
        tau = self.wheelbase_in / v_in_s
        props = self._corner_props()
        geom = self._corner_geometry()
        n_f = len(f)
        q = np.zeros((n_f, 7), dtype=complex)
        for idx, fi in enumerate(f):
            w = 2.0 * np.pi * fi
            s = 1j * w
            lag = np.exp(-1j * w * tau)          # rear axle sees it later
            rhs = np.zeros(7, dtype=complex)
            for i, (_k, _c, kt, _mu) in enumerate(props):
                rhs[3 + i] = kt * (1.0 if i < 2 else lag)
            q[idx] = np.linalg.solve(M * s ** 2 + C * s + K, rhs)
        z_s, phi, theta = q[:, 0], q[:, 1], q[:, 2]
        out = {
            "freq_hz": f,
            "heave": np.abs(z_s),
            "pitch_deg_per_in": np.degrees(np.abs(theta)),
            "roll_deg_per_in": np.degrees(np.abs(phi)),
            "heave_accel_g": np.abs(z_s * (1j * 2 * np.pi * f) ** 2) / G_IN_S2,
        }
        # body acceleration at the axle stations — shows what pitch adds to
        # what the driver actually feels, with no extra inputs needed
        w = 2.0 * np.pi * f
        for name, x in (("front_axle", self.a_in), ("rear_axle", -self.b_in)):
            z = z_s - theta * x
            out[f"accel_{name}_g"] = np.abs(z * (1j * w) ** 2) / G_IN_S2
        # per-corner tire load variation and suspension travel
        for i, corner in enumerate(CORNERS):
            x, y = geom[i]
            kt = props[i][2]
            z_road = np.ones(n_f, dtype=complex) if i < 2 else np.exp(
                -1j * w * tau)
            z_u = q[:, 3 + i]
            z_body = z_s + phi * y - theta * x
            out[f"tire_load_{corner}_lb_per_in"] = np.abs(kt * (z_u - z_road))
            out[f"travel_{corner}"] = np.abs(z_body - z_u)
        return out


# ----------------------------------------------------------------------
def from_dynamics(v, results=None) -> FullCar:
    """Build the full car from the dynamics inputs + outputs. Every value
    already exists there except the roll radius of gyration, which is an
    explicit (estimated) input."""
    from . import dynamics
    r = dynamics.compute(v) if results is None else results
    L = float(v.wheelbase_in)
    # sprung CG: front_sprung_frac is the share carried by the front axle,
    # and the CG sits that fraction of the wheelbase AHEAD of the rear.
    b = float(r["front_sprung_frac"]) * L      # sprung CG -> rear axle
    a = L - b                                   # sprung CG -> front axle
    # `compute()` returns None for the pitch radius when the axle weights
    # put the CG outside the wheelbase — an inconsistency it already warns
    # about. Fall back on the magnitudes so the ride model still reports
    # instead of crashing or filling the plots with NaN.
    ky = r["pitch_radius_gyr_in"]
    if ky is None or not (float(ky) > 0.0):
        ky = max(float(np.sqrt(abs(a * b))), 1e-6)
    mr_df, mr_dr = v.mr_damper_front, v.mr_damper_rear
    # ARB roll rate (lb-ft/deg) -> an equivalent differential wheel rate:
    # a bar giving K_phi resists roll with K_phi = k_bar * track^2.
    def bar_rate(k_lbftdeg, track_in):
        if k_lbftdeg <= 0.0 or track_in <= 0.0:
            return 0.0
        k_lbin_rad = k_lbftdeg * 12.0 * 180.0 / np.pi
        return k_lbin_rad / track_in ** 2
    return FullCar(
        sprung_lb=float(r["sprung_weight_lb"]),
        a_in=a, b_in=b,
        track_front_in=float(v.track_front_in),
        track_rear_in=float(v.track_rear_in),
        pitch_radius_gyr_in=float(ky),
        roll_radius_gyr_in=float(getattr(v, "roll_radius_gyr_in", 18.0)),
        unsprung_front_lb=float(v.unsprung_front_lb),
        unsprung_rear_lb=float(v.unsprung_rear_lb),
        wheel_rate_front_lbin=float(r["wheel_rate_front_lbin"]),
        wheel_rate_rear_lbin=float(r["wheel_rate_rear_lbin"]),
        tire_rate_front_lbin=float(v.tire_rate_front_lbin),
        tire_rate_rear_lbin=float(v.tire_rate_rear_lbin),
        damp_front_lbsin=0.5 * (v.damp_bump_front_lbsin
                                + v.damp_rebound_front_lbsin) * mr_df ** 2,
        damp_rear_lbsin=0.5 * (v.damp_bump_rear_lbsin
                               + v.damp_rebound_rear_lbsin) * mr_dr ** 2,
        arb_wheel_rate_front_lbin=bar_rate(
            float(r.get("arb_roll_rate_front_lbftdeg", 0.0)),
            float(v.track_front_in)),
        arb_wheel_rate_rear_lbin=bar_rate(
            float(r.get("arb_roll_rate_rear_lbftdeg", 0.0)),
            float(v.track_rear_in)),
    )


def wheelbase_filter_speeds(wheelbase_in: float, freq_hz: float) -> dict:
    """The speeds at which a given frequency lands on a bounce-only or a
    pitch-only point of the wheelbase comb.

    Pitch is not excited when the axles are IN phase (f = n*V/L), bounce is
    not excited when they are OPPOSED (f = (2n+1)*V/2L). Inverted for V."""
    if freq_hz <= 0.0:
        return {"pitch_null_mph": [], "bounce_null_mph": []}
    L = wheelbase_in
    pitch_null = [freq_hz * L / n / MPH_TO_IN_S for n in (1, 2, 3)]
    bounce_null = [2.0 * freq_hz * L / (2 * n + 1) / MPH_TO_IN_S
                   for n in (0, 1, 2)]
    return {"pitch_null_mph": pitch_null, "bounce_null_mph": bounce_null}


@dataclass
class FullCarReport:
    car: FullCar
    freq_hz: np.ndarray
    response: dict
    modes: list
    speed_mph: float
    notes: list = field(default_factory=list)

    def mode(self, label: str):
        for m in self.modes:
            if m["label"].startswith(label):
                return m
        return None


def full_car_report(v, speed_mph: float = 20.0, freq_hz=None,
                    results=None) -> FullCarReport:
    """The whole-vehicle frequency-domain picture at one road speed."""
    from .ride_freq import default_frequencies
    car = from_dynamics(v, results=results)
    f = default_frequencies() if freq_hz is None else np.asarray(freq_hz,
                                                                 float)
    modes = car.modes()
    notes = [
        "Linear 7-DOF ride model — no bump stops, progressive springs or "
        "tire lift-off. A design-target and comparison tool.",
        "INERTIAS ARE ESTIMATES: pitch from Olley's dynamic index 1 "
        f"(k_y = {car.pitch_radius_gyr_in:.1f} in), roll from the "
        f"Dynamics-panel input (k_x = {car.roll_radius_gyr_in:.1f} in). "
        "Measure them if a decision turns on the pitch/roll split.",
        f"Road input: both wheels of an axle together, rear axle delayed by "
        f"L/V = {car.wheelbase_in / (max(speed_mph, 1e-6) * MPH_TO_IN_S) * 1000:.0f} ms "
        f"at {speed_mph:.0f} mph. Symmetric input never excites roll — the "
        "roll mode is still reported below because that is where the "
        "anti-roll bars show up.",
    ]
    body = car.body_modes()
    if "heave" in body and "pitch" in body:
        ratio = body["pitch"] / body["heave"] if body["heave"] > 0 else 0.0
        if ratio < 1.0:
            notes.append(
                f"Pitch mode ({body['pitch']:.2f} Hz) is BELOW the bounce "
                f"mode ({body['heave']:.2f} Hz) — the car will tend to pitch "
                "rather than heave over bumps (Olley's flat-ride guidance "
                "wants the rear softer so pitch settles first).")
    return FullCarReport(car=car, freq_hz=f,
                         response=car.response(f, speed_mph), modes=modes,
                         speed_mph=float(speed_mph), notes=notes)
