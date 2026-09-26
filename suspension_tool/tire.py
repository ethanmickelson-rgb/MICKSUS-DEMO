"""Tire model + estimated parameters for the team's SUN-F 23x7-10 tire.

MICKSUS is a kinematics tool, but grip is what ultimately decides the car,
so the design targets a kinematicist chases (static camber, camber-in-roll,
Ackermann, slip budgets) should trace back to a tire's force-and-moment
behaviour. This module holds (1) a small saturating lateral-force model and
(2) a table of ESTIMATED parameters for the SUN-F A051 23x7-10 ATV tire.

========================  READ THIS  ========================
There is STILL no measured force-and-moment data for this tire. SUN-F does
not publish F&M, and the 23x7-10 size is not in any TTC-style dataset. Every
number below is an ENGINEERING ESTIMATE, and each carries an explicit
`Confidence` and a `basis` string saying exactly how it was arrived at:

  MEASURED            a published manufacturer/spec value
  DERIVED             computed from a published value + a standard relation
  INFORMED_ESTIMATE   from off-road-tire literature / comparable tires
  ROUGH_GUESS         order-of-magnitude only

Treat any downstream output as QUALITATIVE (trends, relative comparisons)
until the team runs its own tire test. When real data arrives, fit
`TireModel` to it (or swap in a full Pacejka curve exposing the same
`lateral_force` interface) and replace the estimates below with basis
"MEASURED"; the rest of the tool consumes it unchanged.

Provenance: the PHYSICAL numbers (dimensions, ply, load index, max
pressure, tread depth, weight) are MEASURED — taken from SunF's published
spec for the A051 23x7-10 (6 PR tubeless, rim 10x5.5, max 14 psi, tread
depth 15/32, 15 lb, service description "35F" -> load index 35 = 121 kg =
267 lb). The FORCE-AND-MOMENT numbers remain estimates: no lateral F&M
test exists for this tire, and the published Baja-tire literature says the
same — e.g. "Estimating Pacejka (PAC2002) Tire Coefficients for Pneumatic
Tires on Soft Soils With Application to BAJA SAE Vehicles" derives its
coefficients from a Bekker-Wong soil model rather than measurement,
precisely because off-road tire data is unavailable.
Sources: sunfpower.com / sunf.com A051 product pages; motoee.com listing
(service description 35F TL 6PR); ISO/ETRTO load-index table (35 = 121 kg).

========  EXTERNAL CROSS-CHECK (v1.29, other teams' published work)  ========
A sweep of published Baja SAE senior-design and SAE/ASME papers was run to
test these estimates against somebody else's numbers. What it settled:

  CONFIRMED  Ride frequencies. Two published Baja teams report 1.95 / 2.16 Hz
             and 2.1 / 2.3 Hz (front / rear) — both with the REAR higher,
             i.e. Olley-correct. That is the band this tool recommends.
  CONFIRMED  ~3 g is the accepted maximum vertical acceleration a Baja car
             sees in competition, which is the `bump_g` default in impact.py.
  CONFIRMED  Motion ratios near 0.7 (shock/wheel) are normal — one published
             team runs spring 16.447 N/mm -> wheel 8.272 N/mm, i.e. MR 0.709.
  CONFIRMED  That measured off-road tire F&M data essentially does not exist.
             Saunders, White & Compere (ASME IMECE2019) derive Pacejka
             PAC2002 coefficients from Bekker-Wong SOIL MECHANICS precisely
             because Baja teams have no measured tire data to fit. Our
             estimates are the same kind of object, and no worse founded.
  CHALLENGED Vertical rate — see the note on that parameter. Possibly 2x low.
  NOT FOUND  Any published cornering stiffness or peak-mu number for an ATV
             knobby. SAE 961000 (Renfroe, 1996) reports ATV skid tests on
             dirt, gravel and asphalt and is the best lead, but it is
             paywalled. This remains the weakest data in the tool.

Worth pulling through a university library, in priority order:
  * SAE 961000 — Renfroe, ATV tire friction coefficients on dirt/gravel.
  * SAE 2020-32-2309 — measurement of ground forces on a Baja SAE vehicle
    (strain-gauge wheel + Quarq pressure sensors).
  * SAE 2023-01-0736 — development of a Baja SAE data acquisition system.
  * ASME IMECE2019-10682 / -10683 — Pacejka and traction on soft soils.
Teams known to run WHEEL FORCE TRANSDUCERS (the gold standard, and worth
asking directly): Michigan Tech Blizzard Baja, with Michigan Scientific
Corp hardware; Cincinnati (UC Bearcats) built their own strain-gauge wheel.
"""

from dataclasses import dataclass, field
from enum import Enum

import numpy as np

LB_TO_N = 4.4482216153
IN_TO_M = 0.0254


class Confidence(str, Enum):
    MEASURED = "measured"                    # published spec value
    DERIVED = "derived"                      # computed from a published value
    INFORMED_ESTIMATE = "informed-estimate"  # from literature / comparables
    ROUGH_GUESS = "rough-guess"              # order of magnitude only


@dataclass(frozen=True)
class TireParam:
    """One estimated tire parameter with its provenance."""
    name: str
    value: float
    units: str
    confidence: Confidence
    basis: str

    def __str__(self) -> str:
        return (f"{self.name} = {self.value:g} {self.units} "
                f"[{self.confidence.value}]")


# ----------------------------------------------------------------------
# Lateral-force model (unchanged interface — fit real data into this)
# ----------------------------------------------------------------------
@dataclass
class TireModel:
    """Smooth saturating lateral-force model: linear at the cornering
    stiffness near zero slip, saturating to mu*Fz at high slip angle. A
    stand-in for a fitted Pacejka curve — good enough to reason about
    camber/slip TARGETS, not to predict absolute grip.

    Because knobby off-road tires have a LOW cornering coefficient but a
    respectable peak mu, this tanh form naturally puts the peak at a high
    slip angle (~10-15 deg), which is the correct qualitative behaviour for
    this tire class — very different from a racing slick that peaks ~5-7 deg.
    """

    cornering_stiffness_n_per_deg: float   # C_alpha at the rated load
    peak_friction: float                   # mu_y (lateral coefficient)
    rated_load_n: float                    # Fz the stiffness is quoted at
    label: str = "estimate"

    def lateral_force(self, slip_deg, fz_n=None):
        """Lateral force (N) at slip angle `slip_deg` (deg) and vertical
        load `fz_n` (N, defaults to the rated load). Matches the cornering
        stiffness at small slip and saturates at mu*Fz. Accepts scalars or
        numpy arrays of slip."""
        fz = self.rated_load_n if fz_n is None else float(fz_n)
        # cornering stiffness scales ~linearly with load in this stand-in
        c = self.cornering_stiffness_n_per_deg * (fz / self.rated_load_n)
        fmax = self.peak_friction * fz
        if fmax <= 0.0:
            return np.zeros_like(np.asarray(slip_deg, float))
        return fmax * np.tanh(c / fmax * np.asarray(slip_deg, float))

    def peak_slip_deg(self, fz_n=None, frac=0.98):
        """Slip angle (deg) where the model reaches `frac` of its
        saturation force — a rough 'beyond here you're mostly sliding'
        marker for slip-budget reasoning."""
        fz = self.rated_load_n if fz_n is None else float(fz_n)
        c = self.cornering_stiffness_n_per_deg * (fz / self.rated_load_n)
        fmax = self.peak_friction * fz
        return float(fmax / c * np.arctanh(frac))

    def cornering_stiffness_lb_per_deg(self, fz_n=None):
        """Cornering stiffness in lb/deg at `fz_n` — the unit the dynamics
        layer's understeer-gradient inputs want."""
        fz = self.rated_load_n if fz_n is None else float(fz_n)
        c = self.cornering_stiffness_n_per_deg * (fz / self.rated_load_n)
        return c / LB_TO_N


def estimate_cornering_stiffness_n_per_deg(rated_load_n, factor=0.11):
    """Rough cornering stiffness from load via the normalized cornering
    COEFFICIENT: C_alpha ≈ factor * Fz (N per degree), i.e. `factor` is
    C_alpha/Fz per degree. Street radials sit ~0.15-0.25/deg; a knobby
    off-road tire is LOWER, ~0.08-0.13/deg, because the lugs squirm before
    the carcass builds slip force. `factor` = 0.11 is the informed midpoint;
    re-tune it the moment real data exists. This is a documented ESTIMATE."""
    return factor * float(rated_load_n)


# ----------------------------------------------------------------------
# Surface-dependent peak lateral friction (all INFORMED_ESTIMATE)
# ----------------------------------------------------------------------
# Peak lateral mu_y for a knobby ATV tire by surface. Off-road grip is
# hugely surface-dependent; these are typical dry-to-damp values from
# off-road tire behaviour, NOT measurements. On very soft sand/mud the lugs
# dig in and the effective "grip" can exceed the shear-limited value here
# because the tire is bulldozing material — treat those as especially soft.
_LEGACY_SURFACE_MU = {
    "dry_pavement":  0.80,   # knobblies squirm — well below a slick's ~1.3
    "dry_hardpack":  0.70,   # the usual Baja design surface
    "damp_hardpack": 0.65,
    "loose_gravel":  0.50,
    "sand":          0.45,   # shear-limited; paddle-like dig-in adds to this
    "mud":           0.40,
}
DESIGN_SURFACE = "dry_hardpack"   # what the default model is quoted at

# SURFACE_PEAK_MU is now DERIVED (v1.35) from the MEASURED exponential
# curves at the design load — see the bottom of this module. The legacy
# table above survives only for its surface RATIOS, which are still the
# best guide we have for the surfaces nobody measured. Bound here so the
# module imports cleanly; rebound with real values once the exponential
# models are defined.
SURFACE_PEAK_MU = dict(_LEGACY_SURFACE_MU)


# ----------------------------------------------------------------------
# SUN-F A051 23x7-10 — estimated parameter table
# ----------------------------------------------------------------------
_RATED_LB = 267.0          # load index 35 = 121 kg = 267 lb (published)
_DESIGN_FZ_LB = 150.0      # a representative loaded corner for this car

# RECALIBRATED v1.35 against MEASURED data. Our own estimate had peak mu
# 0.70 and C_alpha/Fz 0.11/deg, which together predicted 89 lb of lateral
# force at 150 lb / 8 deg. The measured Dunlop KT821 on clay hardpack says
# 66 lb: we were ~35% optimistic on grip and ~45% LOW on cornering
# stiffness at the same time. Both now come from the measured curve.
_MEASURED_C, _MEASURED_M = 4.5, -0.46      # Dunlop KT821, clay hardpack
_MU = _MEASURED_C * _DESIGN_FZ_LB ** _MEASURED_M            # ~0.449
_CALPHA_LB_PER_DEG = _DESIGN_FZ_LB * _MU ** 2               # ~30.2 lb/deg
_CALPHA_N_PER_DEG = _CALPHA_LB_PER_DEG * LB_TO_N

# The pre-recalibration model, kept so tire_data_spread() can show how far
# apart the measured and estimated tires are. The author of the measured
# set suspects it under-reports; our estimate ran optimistic; the honest
# answer for any absolute grip number is the band between them.
SUNF_23x7_LEGACY_MODEL = TireModel(
    cornering_stiffness_n_per_deg=estimate_cornering_stiffness_n_per_deg(
        _DESIGN_FZ_LB * LB_TO_N, factor=0.11),
    peak_friction=_LEGACY_SURFACE_MU[DESIGN_SURFACE],
    rated_load_n=_DESIGN_FZ_LB * LB_TO_N,
    label="SunF A051 23x7-10 (PRE-v1.35 estimate, superseded)")


@dataclass(frozen=True)
class TireEstimate:
    """The full estimated parameter set for one tire, plus the lateral
    force model built from it. `params` is the provenance-tagged table."""
    label: str
    model: TireModel
    params: list = field(default_factory=list)
    surface_mu: dict = field(default_factory=lambda: dict(SURFACE_PEAK_MU))
    design_surface: str = DESIGN_SURFACE

    def param(self, name: str) -> TireParam:
        for p in self.params:
            if p.name == name:
                return p
        raise KeyError(name)

    def cornering_stiffness_axle_lb_per_deg(self, corner_load_lb=None) -> float:
        """Per-AXLE cornering stiffness (both tires) in lb/deg — the unit the
        dynamics understeer inputs (cornering_stiff_front/rear_lbdeg) want."""
        fz_n = (self._design_fz_n if corner_load_lb is None
                else corner_load_lb * LB_TO_N)
        return 2.0 * self.model.cornering_stiffness_lb_per_deg(fz_n)

    @property
    def _design_fz_n(self) -> float:
        return _DESIGN_FZ_LB * LB_TO_N


_C = Confidence
SUNF_23x7 = TireEstimate(
    label="SUN-F A051 23x7-10 (ESTIMATED — no measured F&M data)",
    model=TireModel(
        cornering_stiffness_n_per_deg=_CALPHA_N_PER_DEG,
        peak_friction=_MU,
        rated_load_n=_DESIGN_FZ_LB * LB_TO_N,
        label="SUN-F A051 23x7-10 (ESTIMATED — no measured data)"),
    params=[
        # --- physical / construction ---------------------------------
        TireParam("overall_diameter", 23.0, "in", _C.MEASURED,
                  "size designation 23x7-10"),
        TireParam("section_width", 7.0, "in", _C.MEASURED,
                  "size designation 23x7-10"),
        TireParam("rim_diameter", 10.0, "in", _C.MEASURED,
                  "size designation 23x7-10"),
        TireParam("rim_width", 5.5, "in", _C.MEASURED,
                  "SunF lists rim 10 x 5.5 for the A051 23x7-10"),
        TireParam("ply_rating", 6.0, "PR", _C.MEASURED,
                  "SunF A051 'Power II' 23x7-10 is a 6 PR tubeless"),
        TireParam("max_load", _RATED_LB, "lb", _C.MEASURED,
                  "load index 35 on the A051 23x7-10 service description "
                  "(35F TL 6PR) = 121 kg = 267 lb at max pressure. The car "
                  "loads a tire ~130-160 lb static and up to ~300 lb on a "
                  "loaded outside corner — so a hard corner can REACH the "
                  "rating; keep pressure up for high-load events"),
        TireParam("pressure_min", 5.0, "psi", _C.INFORMED_ESTIMATE,
                  "ATV racing runs 4-8 psi; Baja stays >=5 for bead "
                  "security and pinch-flat resistance on rocks"),
        TireParam("pressure_max", 14.0, "psi", _C.MEASURED,
                  "SunF spec sheet lists max 14 psi for the A051 23x7-10"),
        TireParam("tread_depth", 15.0, "1/32 in", _C.MEASURED,
                  "SunF lists tread depth 15 (32nds) for the A051"),
        TireParam("weight", 15.0, "lb", _C.MEASURED,
                  "SunF lists 15 lb for the A051 23x7-10. NOTE this is much "
                  "heavier than a car tire of similar diameter — it drives "
                  "the UNSPRUNG mass (the dynamics sheet's 30/35 lb per "
                  "side must cover tire + wheel + upright + half the arms)"),
        # --- vertical (radial) behaviour -----------------------------
        TireParam("loaded_radius", 11.1, "in", _C.DERIVED,
                  "unloaded radius 11.5 in minus ~0.4 in deflection at "
                  "~150 lb / ~8 psi (delta = load / vertical_rate)"),
        TireParam("vertical_rate", 240.0, "lb/in", _C.INFORMED_ESTIMATE,
                  "at ~8 psi; small bias tire rate is roughly linear in "
                  "pressure: ~ carcass 65 + 22 lb/in per psi. Lands near "
                  "the 250 lb/in the dynamics sheet already assumes. "
                  "*** CONFLICT — MEASURE THIS ONE FIRST. *** A widely used "
                  "racing rule of thumb puts a BIAS tire at 60-70 lb/in per "
                  "psi, and published rig data agrees for stiffer race tires "
                  "(a 20x7.5 at 16 psi measures ~940 lb/in, i.e. ~59 lb/in "
                  "per psi). Applied at 8 psi that rule gives ~480-560 lb/in "
                  "— roughly DOUBLE the value here. Our lower figure assumes "
                  "an ATV knobby's tall, soft sidewall is far more compliant "
                  "than a race tire, which is physically reasonable but "
                  "UNVERIFIED. It matters: wheel hop goes as "
                  "sqrt(Kw + Kt), so doubling Kt raises the hop frequency "
                  "~35% (wheel hop would move from ~9.8 Hz to ~13 Hz) and changes "
                  "where terrain excites it. A press, a scale and an "
                  "afternoon settles it."),
        TireParam("vertical_rate_per_psi", 22.0, "lb/in/psi",
                  _C.INFORMED_ESTIMATE,
                  "pressure sensitivity of the vertical rate; use "
                  "rate ~ 65 + 22*psi lb/in over 5-12 psi. See the conflict "
                  "noted under vertical_rate — the racing rule of thumb for "
                  "bias tires is 60-70 lb/in per psi, about 3x this slope."),
        # --- lateral force -------------------------------------------
        TireParam("peak_mu_lateral", _MU, "-", _C.INFORMED_ESTIMATE,
                  f"peak lateral friction on {DESIGN_SURFACE}; see "
                  "SURFACE_PEAK_MU for other surfaces (0.4 mud .. 0.8 "
                  "pavement) — knobblies squirm so they never reach a "
                  "slick's ~1.3"),
        TireParam("cornering_coeff", 0.11, "1/deg", _C.INFORMED_ESTIMATE,
                  "normalized cornering stiffness C_alpha/Fz per degree. "
                  "Defensible band for a knobby is 0.10-0.15; 0.11 is the "
                  "midpoint chosen for SELF-CONSISTENCY with mu=0.7 and a "
                  "~12 deg peak (the tanh model reaches 98% of peak at "
                  "(mu/CC)*atanh(0.98) = 14.6 deg at CC=0.11, 10.7 deg at "
                  "0.15 — both in the knobby band). Context: published "
                  "FSAE-class tire studies quote 187-395 N/deg for slicks "
                  "at ~1100 N (CC ~0.17-0.36/deg); a Baja knobby sits well "
                  "BELOW that, hence 0.10-0.15"),
        TireParam("cornering_stiffness_per_tire",
                  _CALPHA_N_PER_DEG / LB_TO_N, "lb/deg", _C.DERIVED,
                  "= cornering_coeff * design corner load (150 lb); "
                  "per axle is 2x this (~33 lb/deg) — NOTE this is much "
                  "lower than the 150 lb/deg the dynamics sheet defaults "
                  "to, which would model a far grippier tire"),
        TireParam("peak_slip_angle", 12.0, "deg", _C.INFORMED_ESTIMATE,
                  "knobby off-road tires peak at high slip (~10-15 deg) vs "
                  "5-7 deg for slicks; consistent with the low cornering "
                  "coefficient + moderate peak mu"),
        TireParam("load_sensitivity", 0.0006, "1/lb", _C.ROUGH_GUESS,
                  "fractional drop in peak mu per lb of Fz above the design "
                  "load (mu falls as load rises — standard tire load "
                  "sensitivity); ~0.06 mu lost per +100 lb"),
        # --- camber / aligning / misc --------------------------------
        TireParam("camber_stiffness", 1.6, "lb/deg", _C.ROUGH_GUESS,
                  "camber thrust per degree at the design load, ~0.1x the "
                  "cornering stiffness; LOW for a knobby. DESIGN GUIDANCE: "
                  "large negative static camber lifts the inside lugs off "
                  "the ground and shrinks the contact patch, so knobbies "
                  "generally prefer NEAR-ZERO static camber (~0 to -1 deg) "
                  "— the opposite of a slick. Keep camber-in-roll small."),
        TireParam("relaxation_length", 8.0, "in", _C.ROUGH_GUESS,
                  "lateral relaxation length ~0.5-1.5 tire radii; affects "
                  "transient response only"),
        TireParam("pneumatic_trail", 0.7, "in", _C.ROUGH_GUESS,
                  "~15% of a ~4-5 in contact-patch length; sets the "
                  "self-aligning-torque contribution beyond mechanical "
                  "trail"),
        TireParam("rolling_resistance_coeff", 0.08, "-", _C.ROUGH_GUESS,
                  "Crr on dirt/hardpack is ~0.05-0.15 (vs ~0.015 on "
                  "pavement) — dominated by soil deformation"),
    ],
)

# Back-compat: the old name pointed at the bare placeholder model. Keep it
# resolving to the (now better-reasoned) estimate's force model so existing
# imports keep working.
SUNF_23x7_PLACEHOLDER = SUNF_23x7.model


def pacejka_lateral_coeffs(estimate: TireEstimate = SUNF_23x7,
                           corner_load_lb: float = _DESIGN_FZ_LB) -> dict:
    """Approximate Magic-Formula lateral coefficients (B, C, D, E) built
    from the estimated peak mu and cornering stiffness — NOT a fit. Use to
    seed a Pacejka curve that matches this tire's peak and initial slope:
        D = mu*Fz (peak), C ~ 1.4 (lateral shape), BCD = cornering
        stiffness => B = C_alpha/(C*D), E ~ 0.9.
    All INFORMED_ESTIMATE. Fz in the return is in N."""
    fz_n = corner_load_lb * LB_TO_N
    d = estimate.model.peak_friction * fz_n
    c = 1.40
    c_alpha = estimate.model.cornering_stiffness_n_per_deg * (
        fz_n / estimate.model.rated_load_n) * (180.0 / np.pi)  # N/rad
    b = c_alpha / (c * d) if d > 0 else 0.0
    return {"B": b, "C": c, "D": d, "E": 0.90, "fz_n": fz_n,
            "note": "approximate — built from estimated mu + cornering "
                    "stiffness, not fitted to data"}

# ======================================================================
# Exponential tire model + MEASURED Baja tire data (v1.35)
# ----------------------------------------------------------------------
# Everything above this line is our own ESTIMATE for the SunF. This block
# is the first real measured lateral data we have for a Baja-class tire:
# a Dunlop KT821 22x8-10 run on two surfaces in a graduate test program,
# shared publicly by an Auburn student ("dynosaur") on the Baja SAE
# Discord along with an MMM yaw-moment tool. Free to use, as-is.
#
# The model is a two-coefficient exponential:
#
#     mu(Fz)     = c * Fz**m                 (peak friction FALLS with load)
#     Fy(Fz, a)  = sign(a) * Fz * mu * (1 - exp(-mu * |a|))     a in DEGREES
#
# Two things fall straight out of it and are worth naming, because they
# are the numbers a suspension designer actually wants:
#
#     peak mu           = c * Fz**m           (the asymptote as a -> inf)
#     cornering stiff.  = dFy/da |a=0 = Fz * mu**2      (lb per DEGREE)
#
# So one pair of coefficients carries both the grip ceiling AND the
# build-up rate, with load sensitivity as a power law rather than the
# linear fudge factor our own estimate used.
#
# HEALTH WARNING, from the author himself: "I have some doubts that the
# tire model under-reports tire forces." Our own independent estimate ran
# the other way (optimistic). Treat the truth as bracketed by the two, and
# see TIRE_DATA_SPREAD below for how wide that bracket is.
# ======================================================================

@dataclass(frozen=True)
class ExponentialTire:
    """Measured (or measurement-derived) lateral tire model.

    Loads and forces in POUNDS, slip angles in DEGREES -- the units the
    source data is quoted in. Use `.lateral_force_n()` at the boundary
    with the SI-flavoured TireModel above."""

    coeff_c: float          # scale coefficient
    coeff_m: float          # load exponent (negative: mu falls with load)
    label: str
    surface: str
    source: str = ""
    vertical_rate_lb_in: float = float("nan")
    pressure_psi: float = float("nan")
    rated_load_lb: float = float("nan")
    rolling_resistance: float = float("nan")

    def peak_mu(self, fz_lb):
        """Peak lateral friction coefficient at vertical load `fz_lb`."""
        fz = np.asarray(fz_lb, float)
        return np.where(fz > 0.0, self.coeff_c * np.power(
            np.maximum(fz, 1e-9), self.coeff_m), 0.0)

    def cornering_stiffness_lb_per_deg(self, fz_lb):
        """C_alpha = dFy/d(alpha) at zero slip, lb per DEGREE.

        For this model that is exactly Fz * mu**2 -- differentiate
        Fz*mu*(1-exp(-mu*a)) and set a = 0."""
        fz = np.asarray(fz_lb, float)
        return fz * self.peak_mu(fz) ** 2

    def lateral_force_lb(self, fz_lb, slip_deg):
        """Lateral force (lb). Positive slip gives positive force."""
        fz = np.asarray(fz_lb, float)
        a = np.asarray(slip_deg, float)
        mu = self.peak_mu(fz)
        return np.sign(a) * fz * mu * (1.0 - np.exp(-mu * np.abs(a)))

    def lateral_force_n(self, fz_n, slip_deg):
        """Same, in newtons -- for the SI side of the tool."""
        return self.lateral_force_lb(
            np.asarray(fz_n, float) / LB_TO_N, slip_deg) * LB_TO_N

    def slip_at_frac_of_peak(self, fz_lb, frac=0.95):
        """Slip angle (deg) reaching `frac` of the peak force. The
        exponential inverts exactly: a = -ln(1-frac)/mu."""
        mu = self.peak_mu(fz_lb)
        return float(-np.log(1.0 - frac) / mu)


# -- the two measured datasets, verbatim from the shared YAML ----------
_KT821 = dict(vertical_rate_lb_in=350.0, pressure_psi=10.0,
              rated_load_lb=600.0, rolling_resistance=0.06,
              source="Dunlop KT821 22x8-10, graduate test programme; "
                     "shared by 'dynosaur' (Auburn) on the Baja SAE Discord")

DUNLOP_KT821_CLAY = ExponentialTire(
    coeff_c=4.5, coeff_m=-0.46, surface="dry_hardpack",
    label="Dunlop KT821 22x8-10 — clay, hardpack (MEASURED)", **_KT821)

DUNLOP_KT821_GRAVEL = ExponentialTire(
    coeff_c=6.65, coeff_m=-0.58, surface="loose_gravel",
    label="Dunlop KT821 22x8-10 — gravel, NCAT (MEASURED)", **_KT821)

MEASURED_TIRES = {
    "dunlop_kt821_clay": DUNLOP_KT821_CLAY,
    "dunlop_kt821_gravel": DUNLOP_KT821_GRAVEL,
}

# Reference load the single-number summaries are quoted at: a representative
# loaded corner on a Baja car.
DESIGN_FZ_LB = 150.0


def surface_tire(surface: str = DESIGN_SURFACE) -> ExponentialTire:
    """Exponential model for `surface`.

    Hardpack and gravel ARE the two measured surfaces. The others are the
    measured hardpack curve scaled by the peak-mu ratio our own estimate
    used, keeping the measured load exponent -- an interpolation, labelled
    as one, not a measurement."""
    if surface == "dry_hardpack":
        return DUNLOP_KT821_CLAY
    if surface == "loose_gravel":
        return DUNLOP_KT821_GRAVEL
    ratio = _LEGACY_SURFACE_MU.get(surface, _LEGACY_SURFACE_MU[DESIGN_SURFACE])
    ratio /= _LEGACY_SURFACE_MU[DESIGN_SURFACE]
    return ExponentialTire(
        coeff_c=DUNLOP_KT821_CLAY.coeff_c * ratio,
        coeff_m=DUNLOP_KT821_CLAY.coeff_m,
        surface=surface,
        label=f"KT821 hardpack curve scaled to {surface} (INTERPOLATED)",
        source="measured hardpack curve x our estimated surface-mu ratio",
        vertical_rate_lb_in=DUNLOP_KT821_CLAY.vertical_rate_lb_in,
        pressure_psi=DUNLOP_KT821_CLAY.pressure_psi,
        rated_load_lb=DUNLOP_KT821_CLAY.rated_load_lb,
        rolling_resistance=DUNLOP_KT821_CLAY.rolling_resistance)


def tire_data_spread(fz_lb: float = DESIGN_FZ_LB, slip_deg: float = 8.0):
    """How far apart the measured and estimated tires actually are.

    Worth calling before trusting any absolute cornering number: the
    measured model may under-report (the author's own caveat) and our
    estimate was optimistic, so the honest answer is a band, not a value."""
    meas = DUNLOP_KT821_CLAY.lateral_force_lb(fz_lb, slip_deg)
    est = SUNF_23x7_LEGACY_MODEL.lateral_force(
        slip_deg, fz_n=fz_lb * LB_TO_N) / LB_TO_N
    lo, hi = float(min(meas, est)), float(max(meas, est))
    return {"measured_lb": float(meas), "legacy_estimate_lb": float(est),
            "low_lb": lo, "high_lb": hi,
            "spread_frac": (hi - lo) / lo if lo > 0 else float("nan")}


# ----------------------------------------------------------------------
# Rebind the surface table from the MEASURED curves (v1.35)
# ----------------------------------------------------------------------
# Hardpack and gravel are measured outright; the rest are the measured
# hardpack curve scaled by the legacy surface ratios. Quoted at the design
# load, because peak mu is now load-dependent and a single number has to
# name the load it belongs to.
SURFACE_PEAK_MU = {name: float(surface_tire(name).peak_mu(DESIGN_FZ_LB))
                   for name in _LEGACY_SURFACE_MU}
