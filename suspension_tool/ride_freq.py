"""Quarter-car frequency response (the Bode plot) for ride and grip.

WHY a frequency-domain view, when the dynamics sheet already reports a ride
frequency and a damping ratio? Because those are SCALARS, and the two things
that actually decide how a Baja car behaves on rough ground are shapes:

  * there are TWO resonances, not one. The body mode (~2 Hz) is the one the
    ride-frequency number describes. The WHEEL-HOP mode (~10 Hz), where the
    unsprung mass bounces on the tire spring, is invisible to every scalar
    the tool reported before this module — and it is the one that makes a
    tire skip and stop putting power down.
  * ride and grip pull in OPPOSITE directions as you add damping, and how
    much you lose on each side is a curve, not a number.

MODEL — the standard 2-DOF quarter car (Gillespie ch.5, Milliken ch.17, and
every quarter-car paper in the off-road literature):

      z_s  ── sprung corner mass m_s
       │
      [k_s]  [c]          suspension AT THE WHEEL (spring rate and damping
       │                  already divided by the motion ratio squared)
      z_u  ── unsprung mass m_u
       │
      [k_t]               tire as a spring (no tire damping — it is ~1% and
       │                  conventionally dropped)
      z_r  ── road

    m_s z_s" = -k_s (z_s - z_u) - c (z_s' - z_u')
    m_u z_u" =  k_s (z_s - z_u) + c (z_s' - z_u') - k_t (z_u - z_r)

In the Laplace domain that is a 2x2 system; inverting it gives the four
transfer functions this module reports (see `TRANSFER_FUNCTIONS`).

WHAT IT IS GOOD FOR — and what it is not.
This is a LINEAR, small-perturbation model. A Baja car is emphatically not:
it runs huge travel, progressive springs, bump stops, and dampers whose
rebound force is often ~3x bump (Gillespie). So:
  * treat it as a DESIGN-TARGET and COMPARISON instrument — resonance
    placement, damping selection, "which of these two setups is calmer" —
    not as an absolute prediction of anything;
  * because one linear damping value cannot represent an asymmetric damper,
    `response_band` returns the bump and rebound curves as a BAND, which is
    the honest picture: the real car lives somewhere inside it;
  * the model has no tire lift-off. Once the tire leaves the ground (which
    it does on genuinely rough ground, exactly where |tire deflection| goes
    large) the linear transfer function stops meaning anything. The tire-
    deflection channel tells you WHERE that is about to happen, which is
    most of its value.

SELF-CHECK: the body mode computed here from the undamped eigenproblem must
agree with `dynamics.compute`'s `ride_freq_*_hz`, which is derived a
completely different way (closed form on the ride rate). They agree to <1%;
`tests/test_ride_freq.py` enforces it, so this module doubles as a
cross-check on the dynamics layer.
"""

from dataclasses import dataclass, field

import numpy as np

G_IN_S2 = 386.4          # gravity, in/s^2 (matches dynamics.py / impact.py)
MPH_TO_IN_S = 17.6       # 1 mph = 17.6 in/s

# The four channels, with what each one is FOR.
TRANSFER_FUNCTIONS = {
    "body_travel": "body motion per inch of road — ride isolation "
                   "(<1 = the body moves less than the road)",
    "body_accel": "body acceleration in g per inch of road — harshness, "
                  "the channel that fatigues the driver",
    "tire_deflection": "dynamic tire deflection per inch of road — the GRIP "
                       "channel; big values mean the tire is unloading and "
                       "about to skip",
    "suspension_travel": "suspension travel used per inch of road — how fast "
                         "you run out of rattle space",
}

# Representative Baja terrain, as bump-to-bump spacing in FEET. Editable in
# the GUI — these are starting points, not measurements.
DEFAULT_TERRAIN = {
    "washboard": 2.0,
    "small whoops": 6.0,
    "whoops": 12.0,
    "big rollers": 25.0,
}


@dataclass
class QuarterCar:
    """One corner, reduced to the 2-DOF quarter car. All rates are AT THE
    WHEEL (motion ratio already applied), imperial: lb, in, s."""

    name: str
    sprung_lb: float             # sprung mass carried by this corner
    unsprung_lb: float           # wheel + upright + ~half the arms
    wheel_rate_lbin: float       # spring rate at the wheel (k_s)
    tire_rate_lbin: float        # tire vertical rate (k_t)
    damp_bump_lbsin: float       # damping at the wheel, bump
    damp_rebound_lbsin: float    # damping at the wheel, rebound

    # ---------------------------------------------------------------
    @property
    def m_s(self) -> float:
        """Sprung mass in lb*s^2/in."""
        return self.sprung_lb / G_IN_S2

    @property
    def m_u(self) -> float:
        return self.unsprung_lb / G_IN_S2

    @property
    def critical_damping_lbsin(self) -> float:
        """c_crit = 2*sqrt(k_s * m_s) — quoted on the SPRUNG mode, which is
        the convention every damping-ratio target in the literature uses."""
        return 2.0 * np.sqrt(self.wheel_rate_lbin * self.m_s)

    @property
    def zeta_bump(self) -> float:
        return self.damp_bump_lbsin / self.critical_damping_lbsin

    @property
    def zeta_rebound(self) -> float:
        return self.damp_rebound_lbsin / self.critical_damping_lbsin

    # ---------------------------------------------------------------
    def modal_frequencies_hz(self) -> tuple:
        """(body mode, wheel-hop mode) in Hz, from the UNDAMPED eigenproblem.

        Taken this way on purpose: the modal frequencies are a property of
        the masses and stiffnesses alone, so they are exact and completely
        independent of the damping. Reading them off the peaks of a damped
        response instead is unreliable — near critical damping the peaks
        flatten and shift, and can vanish entirely."""
        m = np.diag([self.m_s, self.m_u])
        k = np.array([[self.wheel_rate_lbin, -self.wheel_rate_lbin],
                      [-self.wheel_rate_lbin,
                       self.wheel_rate_lbin + self.tire_rate_lbin]])
        lam = np.linalg.eigvals(np.linalg.solve(m, k))
        f = np.sort(np.sqrt(np.abs(np.real(lam)))) / (2.0 * np.pi)
        return float(f[0]), float(f[1])

    @property
    def body_hz(self) -> float:
        return self.modal_frequencies_hz()[0]

    @property
    def wheel_hop_hz(self) -> float:
        return self.modal_frequencies_hz()[1]

    # ---------------------------------------------------------------
    def response(self, freq_hz, damping_lbsin=None) -> dict:
        """The four transfer-function magnitudes at each frequency (Hz).

        `damping_lbsin` defaults to the mean of bump and rebound — see
        `response_band` for the honest two-curve version."""
        c = (0.5 * (self.damp_bump_lbsin + self.damp_rebound_lbsin)
             if damping_lbsin is None else float(damping_lbsin))
        f = np.atleast_1d(np.asarray(freq_hz, float))
        s = 1j * 2.0 * np.pi * f
        ks, kt = self.wheel_rate_lbin, self.tire_rate_lbin
        # [a11 a12][Zs]   [ 0 ]
        # [a21 a22][Zu] = [kt] * Zr
        a11 = self.m_s * s ** 2 + c * s + ks
        a12 = -(c * s + ks)
        a21 = a12
        a22 = self.m_u * s ** 2 + c * s + ks + kt
        det = a11 * a22 - a12 * a21
        zs = (-a12 * kt) / det                 # body / road
        zu = (a11 * kt) / det                  # wheel / road
        return {
            "freq_hz": f,
            "body_travel": np.abs(zs),
            # a in/s^2 -> g
            "body_accel": np.abs(zs * s ** 2) / G_IN_S2,
            "tire_deflection": np.abs(zu - 1.0),
            "suspension_travel": np.abs(zs - zu),
        }

    def response_band(self, freq_hz) -> dict:
        """Bump and rebound responses as a BAND.

        A real damper is asymmetric (rebound is commonly ~2-3x bump), and no
        single linear coefficient represents that. Rather than pick one and
        imply a precision the model does not have, this returns the curve for
        each: the physical car lives between them."""
        lo = self.response(freq_hz, self.damp_bump_lbsin)
        hi = self.response(freq_hz, self.damp_rebound_lbsin)
        out = {"freq_hz": lo["freq_hz"]}
        for key in TRANSFER_FUNCTIONS:
            a, b = lo[key], hi[key]
            out[f"{key}_bump"] = a
            out[f"{key}_rebound"] = b
            out[f"{key}_min"] = np.minimum(a, b)
            out[f"{key}_max"] = np.maximum(a, b)
        return out

    def peaks(self, freq_hz=None) -> dict:
        """Peak magnitude of each channel over the band, with the frequency
        it occurs at. Uses the worst (max) edge of the bump/rebound band."""
        f = default_frequencies() if freq_hz is None else np.asarray(freq_hz,
                                                                    float)
        band = self.response_band(f)
        out = {}
        for key in TRANSFER_FUNCTIONS:
            mag = band[f"{key}_max"]
            i = int(np.argmax(mag))
            out[key] = {"peak": float(mag[i]), "at_hz": float(f[i])}
        return out


def default_frequencies(f_min: float = 0.2, f_max: float = 30.0,
                        n: int = 600) -> np.ndarray:
    """Log-spaced frequency axis — a Bode plot is read on a log axis, and it
    spans the body mode (~2 Hz) through well past wheel hop (~10 Hz)."""
    return np.logspace(np.log10(f_min), np.log10(f_max), n)


def from_dynamics(v, results=None, axle: str = "front") -> QuarterCar:
    """Build the quarter car for one axle straight out of the dynamics
    layer, so it always describes the car currently being modelled.

    Everything needed is already computed there: the sprung corner weight,
    the unsprung weight, the wheel rate (spring x MR^2) and the damping at
    the wheel (damper x MR_damper^2). Nothing is invented here."""
    from . import dynamics
    if axle not in ("front", "rear"):
        raise ValueError("axle must be 'front' or 'rear'")
    r = dynamics.compute(v) if results is None else results
    mr_d = getattr(v, f"mr_damper_{axle}")
    return QuarterCar(
        name=axle,
        sprung_lb=float(r[f"corner_sprung_{axle}_lb"]),
        unsprung_lb=float(getattr(v, f"unsprung_{axle}_lb")),
        wheel_rate_lbin=float(r[f"wheel_rate_{axle}_lbin"]),
        tire_rate_lbin=float(getattr(v, f"tire_rate_{axle}_lbin")),
        # damper rates are quoted at the DAMPER; MR^2 brings them to the wheel
        damp_bump_lbsin=float(getattr(v, f"damp_bump_{axle}_lbsin")) * mr_d ** 2,
        damp_rebound_lbsin=float(
            getattr(v, f"damp_rebound_{axle}_lbsin")) * mr_d ** 2,
    )


def excitation_speed_mph(freq_hz: float, spacing_ft: float) -> float:
    """Road speed at which evenly-spaced bumps drive a given frequency.

    A wheel crossing bumps spaced `spacing_ft` apart at speed V meets them at
    f = V / spacing. Inverted: V = f * spacing. This is the whole reason the
    resonance frequencies are actionable — they map to a speed you actually
    drive."""
    return float(freq_hz) * float(spacing_ft) * 12.0 / MPH_TO_IN_S


def terrain_table(qc: QuarterCar, terrain: dict = None) -> list:
    """For each terrain feature, the speeds at which it lands on the body and
    the wheel-hop resonances. A coincidence inside the car's real speed range
    is a genuine setup problem — that is the point of the table."""
    terrain = DEFAULT_TERRAIN if terrain is None else terrain
    body, hop = qc.modal_frequencies_hz()
    rows = []
    for label, spacing in terrain.items():
        rows.append({
            "feature": label,
            "spacing_ft": float(spacing),
            "body_mph": excitation_speed_mph(body, spacing),
            "wheel_hop_mph": excitation_speed_mph(hop, spacing),
        })
    return rows


@dataclass
class RideReport:
    """Everything the Bode view shows for one axle, ready to plot or print."""
    quarter_car: QuarterCar
    freq_hz: np.ndarray
    band: dict
    body_hz: float
    wheel_hop_hz: float
    peaks: dict
    terrain: list
    notes: list = field(default_factory=list)


def ride_report(v, axle: str = "front", terrain: dict = None,
                freq_hz=None, results=None) -> RideReport:
    """The whole frequency-domain picture for one axle."""
    qc = from_dynamics(v, results=results, axle=axle)
    f = default_frequencies() if freq_hz is None else np.asarray(freq_hz,
                                                                 float)
    body, hop = qc.modal_frequencies_hz()
    notes = [
        "Linear 2-DOF quarter car: a design-target and comparison tool, not "
        "an absolute prediction. No bump stops, no progressive springs, no "
        "tire lift-off.",
        "The band spans BUMP to REBOUND damping — one linear coefficient "
        "cannot represent an asymmetric damper, so the real car lives "
        "between the two curves.",
    ]
    zb, zr = qc.zeta_bump, qc.zeta_rebound
    if zb > 0.7:
        notes.append(
            f"Bump damping ratio {zb:.2f} is very high (racing practice is "
            "~0.3-0.5 in bump): past ~0.7 the damper starts transmitting "
            "the road rather than absorbing it, which RAISES body "
            "acceleration.")
    if zr > 1.0:
        notes.append(
            f"Rebound damping ratio {zr:.2f} is over critical — the wheel "
            "may not extend fast enough to follow the ground after a bump "
            "(packing down over successive hits).")
    if hop > 0.0 and body > 0.0 and hop / body < 3.0:
        notes.append(
            "Body and wheel-hop modes are unusually close together, so they "
            "interact; check the unsprung mass and tire rate.")
    return RideReport(
        quarter_car=qc, freq_hz=f, band=qc.response_band(f),
        body_hz=body, wheel_hop_hz=hop, peaks=qc.peaks(f),
        terrain=terrain_table(qc, terrain), notes=notes)
