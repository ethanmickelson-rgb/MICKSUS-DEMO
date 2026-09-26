"""Vehicle dynamics calculations — Milliken & Milliken (RCVD) reference.

A 1:1 port of the team's suspension-design spreadsheet
(reference/suspension_design_tool_v2.xlsx): mass & CG, springs / motion
ratios / dampers / ARBs and the rates they produce, lateral and
longitudinal weight transfer, per-corner loads, the brake force chain,
roll gradient, ride / bounce / pitch frequencies, understeer gradient,
and the anti-squat/dive/lift percentages.

UNITS: everything here is in the spreadsheet's imperial units — lb, in,
lb/in, lb·s/in, lb-ft/deg, ft, mph, g, Hz, deg — with gravity 386.4 in/s²,
so every number can be checked cell-for-cell against the spreadsheet.
(The kinematics side of the tool stays in mm; the GUI converts when it
pipes live roll-centre heights / motion ratios in.)

MOTION RATIO: MR = spring travel / wheel travel, so wheel rate
Kw = Ks * MR². Since v1.28 the KINEMATIC side uses the same convention
(shock/wheel, ~0.54), so the live link now passes the value straight
through — the old 1/MR conversion at the GUI boundary is gone. Before
v1.28 the kinematics reported wheel/shock (> 1) and had to be
inverted, which is easy to forget when hand-checking against a textbook.

Pure functions + one big input dataclass, GUI-free, unit-tested against
hand-keyed spreadsheet arithmetic. Milliken chapter references are noted
inline (matching the spreadsheet's Equations sheet).

v1.22.1 correctness audit (every output re-derived against outside
sources — SAE J670, Milliken RCVD, Gillespie, OptimumG; details in the
baja-suspension-design skill). Three genuine defects were found and fixed
here, so a few numbers now DIFFER from the original spreadsheet port:
  * LATERAL LOAD TRANSFER used the total weight in the sprung roll-moment
    term (while the roll gradient correctly used the sprung weight) and
    omitted the unsprung-at-wheel-centre reaction — overstating transfer
    ~12%. Now the full Milliken 18.4 three-term form (sprung weight in
    both sprung terms + a separate unsprung term).
  * BOUNCE / PITCH frequencies used the wheel rate, putting the bounce
    frequency ABOVE both corner ride frequencies (impossible). Now the
    ride rate (tire in series), consistent with the ride frequencies.
  * BANKING sign was non-standard (positive bank increased the tyre
    demand); now positive bank = into the turn = reduces demand (SAE).
Everything else audited CORRECT or a documented convention (e.g.
brake_decel_g is brake-system CAPABILITY not achievable grip; the
simplified roll-gradient drops the small −Ws·H denominator term; the
SVSA anti approximation references one slope for squat and lift — the
exact kinematic path in vehicle.py distinguishes wheel-centre vs
contact-patch and OVERRIDES it when linked).
"""

from dataclasses import dataclass, field, asdict

import numpy as np

G_IN_S2 = 386.4          # gravity, in/s^2
RHO_AIR = 0.002378       # slug/ft^3
MPH_TO_FTS = 1.467
DEG_PER_RAD = 57.2958    # the spreadsheet's constant (not np.degrees(1))


@dataclass
class DynamicsInputs:
    """Every blue (input) cell of the spreadsheet, defaulted to its
    'Baja SAE' preset values."""

    # --- Vehicle: mass & weight distribution -------------------------
    weight_empty_lb: float = 400.0
    driver_lb: float = 180.0
    front_axle_lb: float = 260.0        # WF, with driver
    rear_axle_lb: float = 320.0         # WR, with driver
    unsprung_front_lb: float = 30.0     # per side
    unsprung_rear_lb: float = 35.0      # per side
    wheelbase_in: float = 63.0
    track_front_in: float = 52.0
    track_rear_in: float = 50.0
    cg_height_in: float = 20.0          # h, above ground
    loaded_radius_front_in: float = 11.5
    loaded_radius_rear_in: float = 11.5
    # Roll radius of gyration of the SPRUNG mass (I_x = m_s * k_x^2). Needed
    # only by the full-car ride model (`full_car.py`); nothing else reads it.
    # ESTIMATE: production cars sit near 0.3-0.4 x track. A Baja car is
    # narrow with a tall roll cage and a high-mounted driver, so it lands at
    # the upper end — 0.35 x a ~52 in track = 18 in, the default here.
    # Measure it (bifilar pendulum) and replace this when you can; the pitch
    # radius is likewise an estimate (Olley's k_y^2 = a*b, dynamic index 1).
    roll_radius_gyr_in: float = 18.0

    # --- Vehicle: roll axis (live-linked from the kinematic model) ---
    rc_front_in: float = 3.0            # zRF
    rc_rear_in: float = 5.0             # zRR

    # --- Vehicle: tires ----------------------------------------------
    tire_rate_front_lbin: float = 250.0
    tire_rate_rear_lbin: float = 250.0
    cornering_stiff_front_lbdeg: float = 150.0   # CaF, axle total
    cornering_stiff_rear_lbdeg: float = 170.0    # CaR

    # --- Vehicle: operating condition --------------------------------
    ay_g: float = 1.0
    ax_g: float = 0.7                   # + = braking
    bank_deg: float = 0.0
    speed_mph: float = 40.0
    cla_ft2: float = 0.0                # CL*A; 0 = no aero
    aero_front_frac: float = 0.5

    # --- Setup: springs (dual-rate stack supported) ------------------
    spring1_front_lbin: float = 125.0
    spring2_front_lbin: float = 0.0     # 0 = single rate
    stack_front: str = "Series"         # Single | Series | Parallel
    spring1_rear_lbin: float = 150.0
    spring2_rear_lbin: float = 0.0
    stack_rear: str = "Series"

    # --- Setup: motion ratios (MR = spring / wheel; Kw = Ks*MR^2) ----
    mr_spring_front: float = 0.7
    mr_spring_rear: float = 0.75
    mr_damper_front: float = 0.7
    mr_damper_rear: float = 0.75

    # --- Setup: dampers (Milliken Ch. 22) -----------------------------
    damp_bump_front_lbsin: float = 15.0
    damp_bump_rear_lbsin: float = 17.0
    damp_rebound_front_lbsin: float = 20.0
    damp_rebound_rear_lbsin: float = 23.0

    # --- Setup: anti-roll bars ----------------------------------------
    arb_rate_front_lbftdeg: float = 150.0   # 0 = no bar
    arb_rate_rear_lbftdeg: float = 60.0
    arb_arm_front_ft: float = 0.5
    arb_arm_rear_ft: float = 0.4
    arb_ir_front: float = 0.6               # install ratio, in/in
    arb_ir_rear: float = 0.65

    # --- Loads: anti-geometry (side-view swing arm; the GUI overrides
    # these with the exact kinematic values when linked) ---------------
    # The SVSA is the virtual arm from the side-view instant centre to the
    # wheel: `length` is its horizontal reach and `height` the IC's height
    # ABOVE THE REFERENCE POINT named by `svsa_ref`.
    #   "wheel_center"  — the standard way an SVSA is quoted (default)
    #   "contact_patch" — some sources quote the anti-dive line instead
    # Anti-SQUAT (inboard drive) needs the WHEEL-CENTRE line; anti-DIVE and
    # anti-LIFT (outboard brakes) need the CONTACT-PATCH line. The two
    # differ by the loaded radius over the SVSA length, so ONE slope cannot
    # serve both — we convert between them using the loaded radius.
    svsa_height_front_in: float = 2.0
    svsa_length_front_in: float = 30.0
    svsa_height_rear_in: float = 3.0
    svsa_length_rear_in: float = 25.0
    svsa_ref: str = "wheel_center"

    # --- Loads: brake force chain --------------------------------------
    brake_bias_front: float = 0.6       # front fraction, sums to 1 w/ rear
    pedal_force_lb: float = 150.0
    pedal_ratio: float = 4.846
    mc_bore_front_in: float = 0.75
    mc_bore_rear_in: float = 0.75
    caliper_area_front_in2: float = 1.06
    caliper_area_rear_in2: float = 1.06
    rotor_radius_front_in: float = 2.975
    rotor_radius_rear_in: float = 2.975
    pad_mu_front: float = 0.4
    pad_mu_rear: float = 0.4

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "DynamicsInputs":
        known = {k: v for k, v in (d or {}).items()
                 if k in cls.__dataclass_fields__}
        return cls(**known)


def _effective_spring(k1: float, k2: float, mode: str) -> float:
    """Single K1; Series K1*K2/(K1+K2) (K2=0 falls back to K1);
    Parallel K1+K2. (Milliken Ch. 16.3)"""
    if mode == "Parallel":
        return k1 + k2
    if mode == "Series" and k2 > 0.0:
        return k1 * k2 / (k1 + k2)
    return k1


def _series(a: float, b: float) -> float:
    return a * b / (a + b) if (a + b) > 0.0 else 0.0


def _roll_rate_lbftdeg(kw_lbin: float, kt_lbin: float, track_in: float,
                       arb_lbftdeg: float, arb_arm_ft: float,
                       arb_ir: float) -> tuple[float, float]:
    """(ARB contribution, total axle roll rate), both lb-ft/deg.

    ARB: K_phi,ARB = K_thetaB * (IB * T / L)^2      (Milliken 16.3 p.599)
    Total: series combination of (ARB + springs) with the tires, all in
    lb-ft/rad, back to lb-ft/deg.                    (Milliken 16.3 p.594)
    """
    t_ft = track_in / 12.0
    k_arb = arb_lbftdeg * (arb_ir * t_ft / arb_arm_ft) ** 2 \
        if arb_lbftdeg > 0.0 and arb_arm_ft > 0.0 else 0.0
    a_rad = k_arb * 180.0 / np.pi           # lb-ft/rad
    s_rad = 12.0 * kw_lbin * t_ft ** 2 / 2.0
    t_rad = 12.0 * kt_lbin * t_ft ** 2 / 2.0
    total = _series(a_rad + s_rad, t_rad) / DEG_PER_RAD
    return k_arb, total


def compute(v: DynamicsInputs, kinematic_anti: dict | None = None) -> dict:
    """Every derived number of the spreadsheet, key-for-key comparable.

    kinematic_anti: optional {"anti_dive_front_pct", "anti_lift_rear_pct",
    "anti_squat_rear_pct"} from the exact kinematic model; when given they
    REPLACE the SVSA approximation (and the output notes say so).
    """
    out: dict = {}
    notes: list[str] = []

    # ------------------------------------------------------------------
    # Vehicle sheet: mass, CG, sprung/unsprung split  (Milliken 18.2)
    # ------------------------------------------------------------------
    W = v.weight_empty_lb + v.driver_lb
    WF, WR = v.front_axle_lb, v.rear_axle_lb
    L = v.wheelbase_in
    out["total_weight_lb"] = W
    out["front_weight_frac"] = WF / W
    # Stated as a FRACTION, not an absolute: a 0.5 lb tolerance is tight on
    # a Baja car and meaningless on a 7 lb RC buggy, and the message has to
    # read the same whichever units the panel is showing.
    if W > 0.0 and abs(WF + WR - W) > max(0.001 * W, 1e-9):
        notes.append(f"⚠ Front + rear axle weights don't sum to the total "
                     f"— off by {abs(WF + WR - W) / W:.1%}.")
    b = WF * L / W               # CG -> rear axle
    a = L - b                    # CG -> front axle
    out["cg_to_rear_in"] = b
    out["cg_to_front_in"] = a
    wu_total = 2.0 * v.unsprung_front_lb + 2.0 * v.unsprung_rear_lb
    Ws = W - wu_total
    out["unsprung_total_lb"] = wu_total
    out["sprung_weight_lb"] = Ws
    hs = (W * v.cg_height_in
          - 2.0 * v.unsprung_front_lb * v.loaded_radius_front_in
          - 2.0 * v.unsprung_rear_lb * v.loaded_radius_rear_in) / Ws
    out["sprung_cg_height_in"] = hs
    a_s = (WF - 2.0 * v.unsprung_front_lb) / Ws   # front sprung fraction
    out["front_sprung_frac"] = a_s

    # Roll axis at the sprung-CG station; rolling moment arm H (16.5 p.603)
    z_ra = v.rc_front_in + (v.rc_rear_in - v.rc_front_in) * (1.0 - a_s)
    H = hs - z_ra
    out["roll_axis_at_cg_in"] = z_ra
    out["roll_moment_arm_in"] = H

    # ------------------------------------------------------------------
    # Setup sheet: springs -> wheel rates -> ride rates -> frequencies
    # ------------------------------------------------------------------
    ks_f = _effective_spring(v.spring1_front_lbin, v.spring2_front_lbin,
                             v.stack_front)
    ks_r = _effective_spring(v.spring1_rear_lbin, v.spring2_rear_lbin,
                             v.stack_rear)
    # A stack mode the solver doesn't recognise silently DROPS spring 2 —
    # say so rather than quietly modelling a single-rate spring.
    for end, mode, k2 in (("front", v.stack_front, v.spring2_front_lbin),
                          ("rear", v.stack_rear, v.spring2_rear_lbin)):
        if k2 > 0.0 and mode not in ("Single", "Series", "Parallel"):
            notes.append(f"⚠ {end} spring stack mode {mode!r} not "
                         "recognised (expected Single/Series/Parallel) — "
                         "spring 2 IGNORED.")
    out["spring_eff_front_lbin"] = ks_f
    out["spring_eff_rear_lbin"] = ks_r
    kw_f = ks_f * v.mr_spring_front ** 2      # Kw = Ks*MR^2 (16.3 p.596)
    kw_r = ks_r * v.mr_spring_rear ** 2
    out["wheel_rate_front_lbin"] = kw_f
    out["wheel_rate_rear_lbin"] = kw_r
    kr_f = _series(kw_f, v.tire_rate_front_lbin)   # ride rate (16.2 p.591)
    kr_r = _series(kw_r, v.tire_rate_rear_lbin)
    out["ride_rate_front_lbin"] = kr_f
    out["ride_rate_rear_lbin"] = kr_r
    ws1_f = (WF - 2.0 * v.unsprung_front_lb) / 2.0  # sprung wt / corner
    ws1_r = (WR - 2.0 * v.unsprung_rear_lb) / 2.0
    out["corner_sprung_front_lb"] = ws1_f
    out["corner_sprung_rear_lb"] = ws1_r
    f_f = np.sqrt(kr_f * G_IN_S2 / ws1_f) / (2.0 * np.pi)  # (16.2 p.601)
    f_r = np.sqrt(kr_r * G_IN_S2 / ws1_r) / (2.0 * np.pi)
    out["ride_freq_front_hz"] = f_f
    out["ride_freq_rear_hz"] = f_r
    olley = f_r / f_f
    out["olley_ratio"] = olley
    if olley < 1.0:
        notes.append("⚠ Olley: front stiffer than rear — pitching ride.")
    elif olley < 1.1:
        notes.append("○ Olley: below the 1.10–1.20 flat-ride target.")
    elif olley <= 1.2:
        notes.append("✓ Olley flat-ride criterion met (1.10–1.20).")
    else:
        notes.append("⚠ Olley: rear too stiff vs front (> 1.20).")

    # ------------------------------------------------------------------
    # Dampers (Milliken Ch. 22)
    # ------------------------------------------------------------------
    for end, cb, cr, mrd, kw, ws1 in (
            ("front", v.damp_bump_front_lbsin, v.damp_rebound_front_lbsin,
             v.mr_damper_front, kw_f, ws1_f),
            ("rear", v.damp_bump_rear_lbsin, v.damp_rebound_rear_lbsin,
             v.mr_damper_rear, kw_r, ws1_r)):
        c_wheel = cb * mrd ** 2
        c_crit = 2.0 * np.sqrt(kw * ws1 / G_IN_S2)
        out[f"damp_wheel_bump_{end}_lbsin"] = c_wheel
        out[f"damp_critical_{end}_lbsin"] = c_crit
        out[f"zeta_bump_{end}"] = c_wheel / c_crit
        out[f"zeta_rebound_{end}"] = (cr * mrd ** 2) / c_crit

    # ------------------------------------------------------------------
    # Roll rates: springs + ARB + tires in series  (Milliken 16.3)
    # ------------------------------------------------------------------
    arb_f, kphi_f = _roll_rate_lbftdeg(
        kw_f, v.tire_rate_front_lbin, v.track_front_in,
        v.arb_rate_front_lbftdeg, v.arb_arm_front_ft, v.arb_ir_front)
    arb_r, kphi_r = _roll_rate_lbftdeg(
        kw_r, v.tire_rate_rear_lbin, v.track_rear_in,
        v.arb_rate_rear_lbftdeg, v.arb_arm_rear_ft, v.arb_ir_rear)
    out["arb_roll_rate_front_lbftdeg"] = arb_f
    out["arb_roll_rate_rear_lbftdeg"] = arb_r
    out["roll_rate_front_lbftdeg"] = kphi_f
    out["roll_rate_rear_lbftdeg"] = kphi_r
    out["roll_rate_total_lbftdeg"] = kphi_f + kphi_r
    out["roll_rate_front_frac"] = kphi_f / (kphi_f + kphi_r)

    # Roll gradient (16.2 p.590): negative = leans out of the turn.
    roll_grad = -(Ws * (H / 12.0)) / (kphi_f + kphi_r)
    out["roll_gradient_deg_g"] = roll_grad
    out["body_roll_at_ay_deg"] = roll_grad * v.ay_g

    # ------------------------------------------------------------------
    # Lateral weight transfer  (Milliken 18.4 p.682) — THREE reactions per
    # axle: the SPRUNG mass reacts an elastic term (through the roll
    # stiffness) plus a geometric term (through the links at the roll
    # centre), and the UNSPRUNG mass reacts directly at the wheel centre.
    #   dW = (Ay/t)·[ Ws·H·Kφ/(Kφf+Kφr)          (sprung, elastic)
    #              +  Ws·(dist/L)·z_RC             (sprung, geometric @ RC)
    #              +  Wu·z_wc ]                    (unsprung @ wheel centre)
    # Both sprung terms carry the SPRUNG weight Ws (the same roll moment
    # the roll gradient above uses — the two must agree), NOT the total W;
    # using W and dropping the unsprung term overstated the transfer ~12%.
    # Wu is the AXLE unsprung (2× the per-side input); z_wc ≈ loaded radius.
    wuf = 2.0 * v.unsprung_front_lb
    wur = 2.0 * v.unsprung_rear_lb
    dwf = (v.ay_g / (v.track_front_in / 12.0)) * (
        Ws * (H / 12.0) * kphi_f / (kphi_f + kphi_r)
        + Ws * (b / L) * (v.rc_front_in / 12.0)
        + wuf * (v.loaded_radius_front_in / 12.0))
    dwr = (v.ay_g / (v.track_rear_in / 12.0)) * (
        Ws * (H / 12.0) * kphi_r / (kphi_f + kphi_r)
        + Ws * (a / L) * (v.rc_rear_in / 12.0)
        + wur * (v.loaded_radius_rear_in / 12.0))
    out["lat_transfer_front_lb"] = dwf
    out["lat_transfer_rear_lb"] = dwr
    out["lat_transfer_total_lb"] = dwf + dwr
    tlltd = dwf / (dwf + dwr)
    out["tlltd_front"] = tlltd
    static_f = WF / W
    if tlltd > static_f + 0.03:
        out["balance_note"] = "More LT on front → understeer trend"
    elif tlltd < static_f - 0.03:
        out["balance_note"] = "More LT on rear → oversteer trend"
    else:
        out["balance_note"] = "Roughly neutral"

    # Longitudinal transfer (18.5 p.689) and banking (18.6)
    dwx = v.ax_g * W * v.cg_height_in / L
    out["long_transfer_lb"] = dwx
    # Banking, SAE/standard sign: POSITIVE bank_deg = banked INTO the turn
    # (favourable), which REDUCES the lateral acceleration the tyres must
    # generate in the road plane — hence the MINUS sign. (Previously this
    # ADDED sin, i.e. treated positive as an adverse/off-camber bank, which
    # is the opposite of every reference.)
    out["ay_banked_g"] = (v.ay_g * np.cos(np.radians(v.bank_deg))
                          - np.sin(np.radians(v.bank_deg)))

    # Aero (Ch. 16): Fz = 0.5*rho*V^2*CL*A, drop = Fz_axle / (2*KR)
    fz = 0.5 * RHO_AIR * (v.speed_mph * MPH_TO_FTS) ** 2 * v.cla_ft2
    fz_f = fz * v.aero_front_frac
    fz_r = fz - fz_f
    out["aero_total_lb"] = fz
    out["aero_front_lb"] = fz_f
    out["aero_drop_front_in"] = fz_f / (2.0 * kr_f) if kr_f else 0.0
    out["aero_drop_rear_in"] = fz_r / (2.0 * kr_r) if kr_r else 0.0

    # ------------------------------------------------------------------
    # Dynamic corner loads: static ± lateral ± longitudinal/2 + aero/2
    # ------------------------------------------------------------------
    out["load_front_out_lb"] = WF / 2.0 + dwf + dwx / 2.0 + fz_f / 2.0
    out["load_front_in_lb"] = WF / 2.0 - dwf + dwx / 2.0 + fz_f / 2.0
    out["load_rear_out_lb"] = WR / 2.0 + dwr - dwx / 2.0 + fz_r / 2.0
    out["load_rear_in_lb"] = WR / 2.0 - dwr - dwx / 2.0 + fz_r / 2.0
    min_load = min(out["load_front_in_lb"], out["load_rear_in_lb"],
                   out["load_front_out_lb"], out["load_rear_out_lb"])
    if min_load < 0.0:
        notes.append("⚠ WHEEL LIFT-OFF: an inside wheel goes negative at "
                     "this Ay — soften that axle's ARB or lower the CG.")
    elif min_load < 0.08 * W:
        # Also relative: 50 lb was ~8% of a Baja car, but every corner of an
        # RC buggy is under 2 lb, so the fixed threshold fired always.
        notes.append(f"NOTE: the lightest wheel is down to "
                     f"{min_load / W:.1%} of the car's weight — close to "
                     f"lift-off.")

    # ------------------------------------------------------------------
    # Anti-squat / anti-dive / anti-lift  (Milliken 17.3 p.618-619)
    # ------------------------------------------------------------------
    h_over_l = v.cg_height_in / L
    if kinematic_anti:
        out["anti_squat_rear_pct"] = kinematic_anti.get("anti_squat_rear_pct")
        out["anti_dive_front_pct"] = kinematic_anti.get("anti_dive_front_pct")
        out["anti_lift_rear_pct"] = kinematic_anti.get("anti_lift_rear_pct")
        out["anti_source"] = "exact kinematic model"
    else:
        # Anti-SQUAT (inboard drive) is drawn from the WHEEL CENTRE;
        # anti-DIVE / anti-LIFT (outboard brakes) from the CONTACT PATCH.
        # For the SAME instant centre those two lines differ by the loaded
        # radius over the SVSA length, so convert the quoted height to each
        # reference instead of reusing one slope for both (which can only
        # ever be right for one of them).
        cp = str(v.svsa_ref).lower() == "contact_patch"

        def _slopes(h_in, l_in, radius):
            """(wheel-centre slope, contact-patch slope) for one axle."""
            if l_in == 0.0:
                return float("nan"), float("nan")
            h_wc = h_in - radius if cp else h_in
            return h_wc / l_in, (h_wc + radius) / l_in

        s_wc_f, s_cp_f = _slopes(v.svsa_height_front_in,
                                 v.svsa_length_front_in,
                                 v.loaded_radius_front_in)
        s_wc_r, s_cp_r = _slopes(v.svsa_height_rear_in,
                                 v.svsa_length_rear_in,
                                 v.loaded_radius_rear_in)
        out["anti_squat_rear_pct"] = 100.0 * s_wc_r / h_over_l
        out["anti_dive_front_pct"] = (100.0 * v.brake_bias_front
                                      * s_cp_f / h_over_l)
        out["anti_lift_rear_pct"] = (100.0 * (1.0 - v.brake_bias_front)
                                     * s_cp_r / h_over_l)
        out["anti_source"] = (f"SVSA approximation (heights quoted from the "
                              f"{'contact patch' if cp else 'wheel centre'})")
        notes.append(
            "○ Anti % from the SVSA approximation: squat uses the "
            "wheel-centre line, dive/lift the contact-patch line (they "
            "differ by the loaded radius). Link the kinematic model for "
            "exact values.")

    # ------------------------------------------------------------------
    # Brake force chain (practical)
    # ------------------------------------------------------------------
    total_brake = 0.0
    for end, bore, area, r_eff, mu, bias, rl in (
            ("front", v.mc_bore_front_in, v.caliper_area_front_in2,
             v.rotor_radius_front_in, v.pad_mu_front, v.brake_bias_front,
             v.loaded_radius_front_in),
            ("rear", v.mc_bore_rear_in, v.caliper_area_rear_in2,
             v.rotor_radius_rear_in, v.pad_mu_rear,
             1.0 - v.brake_bias_front, v.loaded_radius_rear_in)):
        f_mc = v.pedal_force_lb * v.pedal_ratio * bias
        press = f_mc / (np.pi * (bore / 2.0) ** 2)
        clamp = press * area
        torque = 2.0 * (2.0 * mu * clamp * r_eff)   # 2 wheels x 2 pads
        force = torque / rl
        total_brake += force
        out[f"brake_pressure_{end}_psi"] = press
        out[f"brake_clamp_{end}_lb"] = clamp
        out[f"brake_torque_{end}_lbin"] = torque
        out[f"brake_force_{end}_lb"] = force
    out["brake_decel_g"] = total_brake / W
    if abs(out["brake_decel_g"] - v.ax_g) > 0.15:
        notes.append(
            f"NOTE: the brake chain achieves {out['brake_decel_g']:.2f} g "
            f"but the load case assumes Ax = {v.ax_g:.2f} g — iterate or "
            "accept the achievable value.")

    # ------------------------------------------------------------------
    # Pitch & bounce frequencies (Milliken 16.4 — Olley)
    # ------------------------------------------------------------------
    # These are SPRUNG-mass modes, so they must use the RIDE rate (wheel
    # rate in series with the tire) — the same rate the per-corner ride
    # frequencies above use — NOT the bare wheel rate. Using the wheel
    # rate put the bounce frequency ABOVE both corner ride frequencies,
    # which is impossible (a mass-weighted blend of the two corner
    # oscillators must lie between them). k_y² = a·b is Olley's flat-ride
    # dynamic-index-1 assumption (reported, not measured — see notes).
    ms = Ws / G_IN_S2
    out["bounce_freq_hz"] = (np.sqrt((2.0 * kr_f + 2.0 * kr_r) / ms)
                             / (2.0 * np.pi))
    # a*b goes negative when the axle weights put the CG outside the
    # wheelbase — an inconsistency already flagged above, and a state the
    # panel passes through on every keystroke while a mass set is being
    # retyped. Return None rather than a NaN and a console warning.
    if a * b > 0.0:
        out["pitch_radius_gyr_in"] = np.sqrt(a * b)   # k_y^2 ~ a*b (p.604)
        out["pitch_freq_hz"] = (np.sqrt(
            (2.0 * kr_f * a ** 2 + 2.0 * kr_r * b ** 2) / (ms * a * b))
            / (2.0 * np.pi))
    else:
        out["pitch_radius_gyr_in"] = None
        out["pitch_freq_hz"] = None

    # ------------------------------------------------------------------
    # Understeer gradient (Milliken Ch. 5 / 8)
    # ------------------------------------------------------------------
    k_us = WF / v.cornering_stiff_front_lbdeg - WR / v.cornering_stiff_rear_lbdeg
    out["understeer_grad_deg_g"] = k_us
    if k_us > 0.5:
        out["understeer_note"] = "Understeer (stable, slow)"
    elif k_us > 0.0:
        out["understeer_note"] = "Mild understeer (target)"
    elif k_us > -0.5:
        out["understeer_note"] = "Mild oversteer"
    else:
        out["understeer_note"] = "Oversteer (unstable, requires skill)"

    # Tuning recommendation (roll-rate distribution vs static split)
    if out["roll_rate_front_frac"] > static_f + 0.05:
        out["tuning_note"] = ("Biased toward UNDERSTEER — stiffen the rear "
                              "ARB, soften the front ARB, or move the CG "
                              "rearward to balance.")
    elif out["roll_rate_front_frac"] < static_f - 0.05:
        out["tuning_note"] = ("Biased toward OVERSTEER — stiffen the front "
                              "ARB, soften the rear ARB, or move the CG "
                              "forward to balance.")
    else:
        out["tuning_note"] = ("Roll-rate distribution roughly matches the "
                              "static weight split → neutral baseline.")

    out["notes"] = notes
    return out
