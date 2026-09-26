---
name: optimumg-kinematics
description: >
  OptimumG / OptimumKinematics suspension-kinematics and vehicle-dynamics
  knowledge, distilled and evaluated: their tire-first design methodology,
  the roll-centre-as-load-transfer-split framing, FVSA/SVSA roles,
  anti-geometry ("the anti-antis"), and target values for camber, bump
  steer, caster/KPI/scrub/mechanical trail, Ackermann, motion ratio and
  ride frequency, plus yaw-moment-diagram thinking. Use for suspension
  kinematics/vehicle-dynamics design questions, sanity-checking target
  values, interpreting OptimumG methods, or relating them to the MICKSUS
  suspension tool in this repo.
---

# OptimumG / OptimumKinematics — distilled kinematics & vehicle dynamics

**Provenance / accuracy caveat.** This skill is distilled from web-search
summaries of OptimumG's *public* materials — the OptimumKinematics help
file and brochure, the "Generic Formula Student Car" and FSAE-front-axle
optimization case studies, the Racecar-Engineering monthly columns by
Claude Rouelle ("Rolling about", "The anti-antis", "Optimal thinking",
"Getting more from your yaw diagrams"), the Springs & Dampers Tech Tips
by Matt Giaraffa, and seminar summaries ("Getting to grips"/Slip Angle).
Direct fetching of optimumg.com was **blocked by this environment's egress
allowlist**, so numbers below are directional — verify against the primary
PDFs for anything load-bearing. Where a value is general race-engineering
practice rather than an explicit OptimumG statement, it is flagged as such.
(If deeper primary access is wanted, add `optimumg.com` and
`downloads.optimumg.com` to the session egress allowlist.)

## Who / what OptimumG is

OptimumG is Claude Rouelle's vehicle-dynamics consultancy + seminar +
software house (300+ seminars, thousands of engineers trained). Its
teaching is **tire-centric**: grip, balance and driver feedback are decided
by the four forces and moments in each contact patch, and everything else
(kinematics, springs, aero) exists to *feed the tire the loads and angles
it likes*. Software suite:

- **OptimumKinematics (OK)** — hardpoint-based suspension kinematics &
  analysis. Design / Motion / Analysis workflow; forces/moments applied at
  contact patch OR wheel centre; heave, roll, pitch, steer motions in any
  combination; 400+ data channels; an **Optimization Module** for
  multi-objective hardpoint optimization. Validated against CAD + lab data.
  Deliberately *fewer inputs than a full multibody tool* — fast iteration.
- **OptimumDynamics (a.k.a. CVD)** — steady/quasi-steady full-vehicle F&M
  simulation (weight transfer, tire deflection, yaw-moment diagrams).
- **OptimumTire** — tire F&M test-data processing / model fitting.
- **OptimumLap** — free point-mass lap-time tool (weight, engine, aero,
  tire µ).

## Design philosophy & methodology (the order matters)

1. **Tire data FIRST.** From the tire's F&M curves pick the operating
   window: the inclination (camber) angle that maximises lateral force,
   the slip-angle range, load sensitivity, and how camber/pressure/temp
   shift the peak. Kinematic *targets* fall out of this, not the reverse.
2. **Kinematics to serve the tire.** Lay out hardpoints so the wheel keeps
   the tire near its best camber/slip through roll, heave, pitch and steer.
3. **Iterate hardpoints** against targets (by hand first, then the
   Optimization Module) — good kinematics is a *set of trade-offs*, not a
   single optimum, so the optimizer takes many weighted objectives at once.

Distinctive OptimumG stance: **there is no universally "right" roll-centre
height or camber curve** — only the numbers that put *this* tire at its
happy point for *this* car's mass/aero/track. Treat published rules of
thumb as starting points, then let tire data + lap-sim move them.

## Coordinate & sign conventions

- OptimumKinematics lets the user **choose which of X/Y/Z is
  longitudinal/lateral/vertical and which sense is positive** — conventions
  are configurable, not fixed. (Contrast MICKSUS: fixed internal frame
  +X fwd / +Y left / +Z up, with display `AxisConvention`s — same idea,
  the tool just exposes a menu of presets.)
- SAE tire axis they lean on: origin at the **contact-patch centre**,
  x = wheel heading (fwd +), y lateral, z up; Fz > 0 up, Fx > 0 fwd.
- Scrub-radius sign: **positive when the steer-axis (KPI) ground pierce is
  INBOARD of the contact-patch centre**, negative if outboard. Racing
  usually runs small positive.
- Geometric-load-transfer sign (see roll centre): **positive when the roll
  centre is above ground, negative below** — a below-ground RC makes the
  geometric split negative and throws more onto the springs.

## Instant centres, FVSA & SVSA — what each controls

The whole double-wishbone geometry reduces to two "swing arms":

- **Front-View Swing Arm (FVSA)** — the virtual arm from the front-view
  instant centre to the wheel. It sets **roll-centre height, camber-change
  rate (camber gain), and lateral tire scrub**. **Longer FVSA → less camber
  gain** and a lower/steadier RC; short FVSA → aggressive camber gain but
  fast RC migration. This is *the* camber-gain-vs-RC-migration trade-off.
- **Side-View Swing Arm (SVSA)** — sets **anti-dive / anti-squat /
  anti-lift, caster-change rate, and the fore/aft wheel path (recession)**.

Because both ICs are generically at finite points that *move* through
travel, every one of these numbers is a **curve**, not a static value —
OptimumG's habit is to judge the curve (and its migration) over the real
travel/roll range, not just the static point.

## Roll centre — OptimumG's signature framing

**The roll centre is NOT "the point the body rolls about."** ("Rolling
about", "The no-way transfer".) It is a **load-path split**:

- Total lateral load transfer depends **only on** sprung+unsprung mass,
  lateral acceleration, track width and CG height — **the roll centre does
  not change the total at all.**
- What the RC height sets is the **split** of the *sprung-mass* transfer
  into:
  - **Geometric (kinematic) load transfer** — reacted through the
    suspension *links*, essentially **instantaneous** (no roll needed);
    grows with RC height above ground.
  - **Elastic load transfer** — reacted through **springs/ARBs/dampers**,
    arrives with the **time lag** of body roll.
- So RC height is really a **transient / response** knob: raise it → more
  load goes through the links → faster load transfer at that axle → less
  roll there, and (side effect) more **jacking** (unequal inside/outside
  geometric transfer pushes the body up/down). Lower/at-ground → softer,
  more roll, more of the work on the springs, less jacking.

Design implication: pick RC heights (and the roll-axis inclination front↔
rear) mainly for **balance and transient response and jacking**, and keep
**migration small** so that balance doesn't wander with roll/heave. A very
low or below-ground RC minimises jacking but loads the springs; a high RC
is twitchy and jacks. FSAE practice is typically a **low RC, slightly above
ground, front a touch lower than rear** (general practice, not a hard OG
number).

## Camber

- **Static camber**: set from the **tire's peak-lateral-force inclination**
  (from tire data), not a default. General race range is **≈ −2° to −4°**
  to bank camber thrust into the outside tire; the exact value is
  tire-specific and OptimumG insists you read it off the F&M curves.
- **Camber in roll / heave**: OptimumG optimization targets keep the
  **camber variation small — within about ±1°, ideal target 0** (case
  study). Camber-*change-on-steer* is often targeted to **0°** so the
  wheel stays at the tire's best inclination through steering.
- Camber gain is dialed with FVSA length (above). More gain recovers camber
  on the rolling outside wheel but costs RC-migration control; less gain is
  steadier but needs more static camber (which hurts the inside tire and
  straight-line grip). The classic compromise.
- **Caster also gains camber on the steered wheel** (favourable on the
  outside), but OptimumG notes this is **front-axle only** and only matters
  at **large steer** — so it does NOT replace static camber for high-speed
  (low-steer) corners. Use both.

## Bump steer & roll steer

- Definitions: **bump steer = toe change vs vertical wheel travel**
  (deg/m); **roll steer = toe change from body roll** (same construction,
  driven by roll angle). Realistic roll is **≤ ~2°**, so judge roll steer
  over that.
- Default target is **minimise** (tie-rod inner/outer on the line through
  the arm pivots so the tie-rod arcs match the wishbone arcs). Widely-cited
  acceptable band: toe within roughly **+0.2° / −1°** over the used travel.
- But OptimumG's real move is a **custom-curve target, not zero** — e.g.
  their FS example asks for **toe = +0.4° at −50 mm (bump) and −0.2° at
  +50 mm (droop)** — i.e. deliberate **passive steer** (toe-out in bump /
  toe-in in droop, or vice-versa) tuned for turn-in or stability. (This is
  exactly the MICKSUS `toe_slope` optimizer goal — see mapping below.)

## Steering-axis geometry — caster, KPI, scrub, mechanical trail

The **four steering-geometry design variables**: KPI, caster, mechanical
(caster) trail, scrub radius.

- **Scrub radius**: keep **small but non-zero**. Zero scrub means the
  driver can't feel a left/right longitudinal-force imbalance (e.g. split-µ
  braking, one wheel locking); a small positive value restores that
  feedback without loading the steering. Too large = heavy, twitchy
  steering under braking.
- **Mechanical (caster) trail**: the side-view lever between tire lateral
  force and the steer axis — it supplies **self-centring / steering feel**.
  OptimumG's optimization tends to **minimise it** to keep rack loads /
  steering effort down, while keeping enough for feedback. More trail =
  more feel but heavier wheel.
- **Caster**: gives self-aligning torque + steered-wheel camber gain +
  steer-induced ride-height change (jacks the body when steered). KPI:
  affects scrub, steer-camber, and the steering "self-lift" that
  self-centres. Split the desired scrub between KPI and caster.
- General FSAE numbers (practice, not hard OG values): caster ≈ 3–7°, KPI
  small (≈ 4–8°), scrub ≈ small positive (single-digit to ~20 mm),
  mechanical trail a few mm to a couple cm.

## Anti-geometry — "the anti-antis"

Anti-dive / anti-squat / anti-lift are **percentages of the longitudinal
load transfer reacted through the suspension links instead of the springs**
(SVSA side-view construction; anti-dive is a function of **brake bias**):

- 100% anti = *all* fore/aft load transfer goes through the arms, **none
  through the springs** → the spring never sees pitch load → suspension
  stops moving in that mode (harsh, no compliance, geometry binds).
- OptimumG's guidance ("The anti-antis"): **don't chase 100%.** Deliberately
  leave some dive/squat in. **≈ 80%** captures most of the pitch-control
  benefit while keeping the springs working — *unless aero (ride-height
  sensitivity) forces you higher.*
- So anti-geometry is a **load-sharing split between arms and springs**,
  the same mental model as the roll centre but in the side view.

## Ackermann steering

Ackermann % = f(inside steer, outside steer, front track, wheelbase). For
**FSAE, ≈ 60–80%** is cited as a good compromise (tight autocross corners
want positive/near-full Ackermann for the slow inside wheel; higher-speed
grip wants closer to parallel or even anti-Ackermann to keep both fronts
near peak slip). Tune against corner-speed distribution + tire slip curves.

## Springs, dampers, motion ratio (Giaraffa Tech Tips)

- **Motion ratio (MR)** = wheel travel / spring travel; **installation
  ratio = 1/MR** = spring / wheel. Wheel rate `Kw = Ks · IR²` (spring rate
  reduced by the square of the ratio). Keep the definition straight — the
  square is where sign errors hide.
- **Ride frequency** = undamped natural frequency of the sprung mass in
  ride; think of it as **normalised ride stiffness** (removes the mass so
  front↔rear compare fairly). Recommended bands:
  - passenger cars **0.5–1.5 Hz**
  - sedan racecars / moderate-downforce formula **1.5–2.0 Hz**
  - high-downforce **3.0–5.0+ Hz**
  Higher = stiffer/faster response but less mechanical grip; lower = more
  grip but slower transient ("lack of support").
- **Spring-rate design**: from target ride frequency,
  `Ks = 4π²·fr²·msm·MR²` (SI: `msm` sprung mass per corner in kg, `Ks` in
  N/m). Equivalent to `fr = (1/2π)√(Kw/msm)`.
- **Rear ride frequency ≈ 1.3 × front** (ride/road cars) so the faster rear
  "catches up" to the front over a bump and **reduces pitch** — a flat-ride
  idea. (Downforce cars often invert this for aero-platform reasons.)
- There are **four distinct rates/modes** to think in — **ride, single-
  wheel bump, roll, and pitch** — each with its own effective stiffness and
  its own **target damping ratio**; design the dampers per mode.

## The Optimization Module (how OG optimizes kinematics)

- Multi-objective by design (good kinematics = trade-offs), takes **many
  weighted objectives simultaneously**, and supports **custom target
  *curves*** per channel (not just "minimise/hold constant") — e.g. the
  bump-steer curve above, or camber-in-roll held to a shaped target.
- Typical FS front-axle objective set: bump steer (curve), scrub radius
  (small target, not 0), mechanical trail (minimise), camber-in-roll (≈0,
  |·|<1°), camber-on-steer (0 = tire's best inclination), Ackermann,
  RC height/migration, motion ratio — all bounded by the packaging
  envelope (track, arm lengths, frame pickups, shock).

## Vehicle level (where kinematics meets the tire)

- **Weight transfer**: total lateral = mass·ay·CGh / track — kinematics
  only moves *when/where* it acts, never the total (see RC). Front-vs-rear
  **roll-stiffness distribution (LLTD/TLLTD)** is the primary balance knob:
  more roll stiffness / higher RC at an axle → more of its share of load
  transfer → **less grip at that axle → that end "gives up" first**
  (front-biased → understeer, rear-biased → oversteer).
- **Yaw Moment Diagram (YMD / MMM method)** — OptimumG's headline analysis
  (Milliken Moment Method): plot **yaw moment (y) vs lateral acceleration
  (x)** as a grid of constant-steer and constant-body-slip-angle lines.
  From it read the car's character:
  - **Grip** — how far right the envelope reaches (max lateral accel).
  - **Balance / trim** — where the zero-yaw-moment (trimmed) line crosses;
    understeer vs oversteer.
  - **Control** — yaw moment available from steering (turn-in authority).
  - **Stability** — sign/slope of the constant-steer lines: ascending-right
    = stable, descending-right = oversteer / can go unstable past the
    critical speed.
  Kinematic + setup changes are judged by how they move these four.

## Evaluation — strengths, distinctive ideas, limits

**Strong / distinctive (worth internalising):**
- **Tire-first**: kinematic targets are *derived from tire F&M data*, not
  copied from a textbook. This is the single most valuable habit.
- The **load-transfer-split** reframing of both the roll centre (front view)
  and anti-geometry (side view): both are "how much load goes through the
  links vs the springs," which makes jacking, transient response and the
  "don't run 100% anti" rule fall out naturally.
- **Judge curves, not points** — camber, bump steer, RC, anti all evaluated
  over the real motion envelope and for *migration*.
- **Custom-curve optimization targets** (passive steer, shaped camber) —
  more nuanced than "minimise everything."
- **Ride frequency** as the normalised, comparable spring metric; the
  four-mode (ride/1-wheel/roll/pitch) spring+damper decomposition.
- **YMD/MMM** for balance/stability separated into grip/balance/control/
  stability.

**Limits / caveats to keep in mind:**
- OptimumKinematics is **pure kinematics** (rigid links, quasi-static) —
  no compliance / bushing deflection / dynamic tire lag unless you move to
  OptimumDynamics; real cars are elastokinematic.
- The methodology leans on **good tire data** (TTC / lab F&M). Without it,
  the "tire-first" chain is only as good as the model.
- Much of the published numeric guidance is **directional / FSAE-flavoured**
  — the whole point of their method is that the *right* numbers come from
  your tire + mass + aero, so don't cargo-cult the ranges.
- It's a **design/analysis** tool, not a lap-time optimizer — pair with
  OptimumLap/Dynamics to close the loop on whether a kinematic change is
  actually faster.

## Mapping to the MICKSUS tool in this repo

The MICKSUS suspension tool (see `baja-suspension-design` skill) already
mirrors much of OptimumG's approach — this cross-reference makes the
overlap explicit:

| OptimumG concept | MICKSUS equivalent |
|---|---|
| Configurable coord convention | `axes.py` `AxisConvention` (tool frame + chassis/display presets) |
| FVSA → RC height + camber gain | `front_view_ic`, `roll_center_height_mm`, camber-vs-travel plot |
| SVSA → anti-dive/squat + wheel path | `side_view_ic`, `vehicle.anti_*`, `wheel_recession_mm` |
| RC = load-transfer split (geom/elastic) | `dynamics.py` (Milliken port: geometric vs elastic LLT), roll-axis metrics |
| Anti % = arms-vs-springs share | `anti_dive_front_pct` / `anti_squat_rear_pct` (100·slope·L/h) |
| Bump steer curve / passive steer target | `bump_steer_deg_per_mm` + the `toe_slope` optimizer goal (toe changes at a target deg/mm through travel) |
| Camber-in-roll ≈0, |·|<1° | camber-vs-travel + camber-gain checklist limits |
| Motion ratio / ride frequency / spring rate | `motion_ratio`; `dynamics.py` ride-frequency↔spring-rate (`ride_freq_*_hz`; OG's design form solves it the other way, `Ks=4π²fr²·msm·MR²`) |
| RC / camber migration as a target | `optimize.py` `rc_migration`, `camber_gain` goals |
| Multi-objective, custom-curve optimization | `optimize.py` weighted goals incl. `curve_slope`/`curve_zero` kinds |
| YMD / MMM, LLTD balance | `dynamics.py` (roll gradient, TLLTD, understeer gradient) |
| Scrub small-nonzero, min mechanical trail | `desired_scrub_radius`, caster-trail readout |

Where OptimumG would push MICKSUS further: (1) drive camber/toe targets
from *actual tire F&M data* rather than fixed setup inputs; (2) evaluate
**migration** of RC/camber/anti as first-class metrics, not just static
values (MICKSUS already sweeps these — surface the migration numbers); (3)
add a lap-time / YMD feedback loop to decide if a kinematic change is worth
it.

## Sources (public OptimumG material; access was egress-limited)

- OptimumKinematics product + Help File + FS case study (optimumg.com /
  downloads.optimumg.com) — features, workflow, definitions, FS targets.
- Racecar-Engineering columns (Claude Rouelle): "Rolling about" (roll
  centre = load-transfer split), "The anti-antis" (anti-geometry, ~80%),
  "Optimal thinking", "Getting more from your yaw diagrams" (YMD/MMM),
  "Getting to grips"/Slip Angle (seminar summary).
- Springs & Dampers Tech Tips (Matt Giaraffa) — motion ratio, ride
  frequency ranges, `Ks=4π²fr²msm MR²`, rear≈1.3×front, four modes.
- FSAE-front-axle & tie-rod OptimumKinematics-Optimization case studies —
  Ackermann 60–80%, scrub small-nonzero, min mechanical trail, camber ±1°,
  custom bump-steer curve.
- Vehicle Setup & Kinematics Q&A series — camber-vs-caster, RC-vs-steering-
  rack, camber-gain-vs-RC-migration trade-offs.
