# MICKSUS — Baja SAE Suspension Kinematics & Synthesis Tool

> **This is the public demo branch.** It is a snapshot of v1.43.0 with the
> team's vehicle data removed: no `.MICK` project files for our cars, no
> frame meshes, no CAD exports, no design spreadsheets. What ships instead
> is a set of generated, generic examples in [`examples/`](examples/) — a
> plausible Baja layout built from round numbers, and a 1/10 scale RC buggy
> that exists to prove the tool is not secretly hard-coded to one vehicle
> size.
>
> The development log below is kept as written, so it occasionally cites
> measurements from the car this tool was built against. Those are cited as
> *evidence for a bug fix*, not as a design you should copy — treat every
> number in this README as one team's data point.
>
> Feedback is the whole reason this branch exists: what you liked, what got
> in your way, what you expected the tool to do and it didn't.

A desktop tool to design and analyze suspension geometry for a Baja SAE car:
go from high-level setup goals to hardpoints (synthesis), then tweak the
hardpoints and immediately see the kinematic effect (analysis). Pure
kinematics — no tire forces or loads.

Full project brief: [`suspension_tool_prompt_2.md`](suspension_tool_prompt_2.md).

## Quick start

Needs **Python 3.11 or newer**. From the repo root:

```bash
pip install -r requirements.txt
python -m suspension_tool.gui
```

That is the whole install — no build step, no compiler. On Linux you may
also need system GL libraries for Qt (`sudo apt install libegl1 libgl1
libxkbcommon-x11-0`); Windows and macOS need nothing extra.

Then **File → Open** and pick one of:

| File | What it is |
|------|------------|
| `examples/baja_demo_double_wishbone.MICK` | **Start here.** A generic full-size Baja car, double wishbone both ends. |
| `examples/baja_demo_trailing_arm.MICK` | Same front, classic trailing-arm rear. |
| `examples/rc_buggy_1_10.MICK` | A 1/10 scale RC buggy — the tool is not hard-coded to full-size cars. |

Nothing here is anyone's real vehicle; the full-size numbers are round
figures chosen to be plausible. [`examples/README.md`](examples/README.md)
explains what each one is for and what to notice in it.

### First five minutes

1. **Drag a hardpoint** in the 3D view, or type coordinates in the
   Hardpoints table. Every curve under the viewport redraws immediately —
   that live loop is the point of the tool.
2. **Pull the Droop/Bump slider** through the travel range and watch
   camber, toe, roll centre and motion ratio move.
3. **Open the Design Checklist.** The demo car deliberately fails the
   roll-centre-height row: a car with 12 in of ride height genuinely lands
   high, and the default band is a generic target. Arguing with it is the
   intended use.
4. **Hit Generate seed** in the Setup panel with your own track, wheelbase,
   ride height and shock lengths. That is the synthesis half — it hands you
   a starting geometry instead of a blank sketch.
5. **Help → User guide** explains every readout (and **Help → Equations**
   lists the formulas in-app), and
   `suspension_tool/equations.py` carries every formula with its
   Milliken / Gillespie citation.

One convention to know before you type numbers in: **motion ratio here is
shock travel per unit wheel travel**, so it is *below* 1 (~0.55). Many
teams quote the reciprocal (~1.8). Entering the wrong one will not error —
it will quietly quarter your travel budget.

### Feedback

This build exists to be argued with. What was confusing, what you expected
to find and couldn't, what your team needs that is missing, and anything
that gave you a number you did not believe — all of it is useful. Open an
issue on this repo.

## Status

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Forward double-wishbone solver | **Done — verified** |
| 2 | Parametric seed generator | **Done — validated against a real car** |
| 3 | GUI: 3D view, plots, manual editing | **Done** |
| 3.1 | Feedback: units, save/load | **Done** |
| 3.2 | Feedback: bushing axes normal to sketch plane | **Done** |
| 3.3 | Feedback: parallel axes view + larger inboard separation | **Done** |
| 4 | Synthesis / optimization | **Done** |
| 4.1 | Feedback: tweaks panel, new setup vars, RC/IC display, graph toggles, dark mode, CSV/Onshape export | **Done** |
| 5 | Suspension-type interface + rack steer; trailing arm/multilink stubs | **Done** |
| 5.1 | Front + rear axles in one project, mirroring, roll axis, CG, steer slider, Onshape push | **Done** |
| 5.2 | Roll/pitch/steering sweeps; trailing-arm + multilink kinematics | **Done** |
| 5.3 | Per-type GUI editing, anti-dive/anti-squat %, design comparison, report export | **Done** |
| 5.4 | Multilink virtual kingpin, per-type tweaks + optimization, bump-steer map | **Done** |
| 5.5 | Roll visualization, wheel-guard, fit view, help guide, anti-% graphs, undo | **Done** |
| 6 | Frame backdrop, save/load, Windows exe | **Done** |
| 7 | v1.3: camera stays put + model-centred orbit, physical right-wheel steer, Onshape-aligned seed offset, one-sketch planarity guard, CG marker, dockable panels, Milliken dynamics panel | **Done** |
| 8 | v1.4: assembly-branch solver fix (links can no longer cross), chassis (Y-fwd) coordinate convention, halfshaft CV/plunge checks, offset-aware scene, symmetric roll, ride-height/reset buttons, ghost toggle, example projects | **Done** |
| 9 | v1.5: workflow revamp (auto-clamp + live Design Checklist + autosave), drag hardpoints in 3D with axis locks, optional snap-to-frame, renamable whole-vehicle Candidates, Onshape full-design sync (hybrid path) | **Done** |
| 10 | v1.6: frame auto-align on import + move-warning, halfshaft drawn in 3D + seed-phase option, both-axle heave | **Done** |
| 11 | v1.7: halfshaft overhaul (kingpin-axis outer CV, table rows, draggable inner, hub-along-kingpin tweak, stale-inner tangle fix), self-healing orbit recenter, one-click re-square button | **Done** |
| 12 | v1.8: camera bounds ignore far IC markers (the "blue dot" orbit fix), outer CV moved onto the tire's centre axis + axis stub visual, bushing-axis yaw metric + yaw-removing re-square, steering-arm tweak, Ackermann graph + turning-radius goal in the Design Check | **Done** |
| 13 | v1.9: MICKSUS branding (splash, dark boot, blank workspace), twisted sketch plane (per-axle design yaw; verified on the user's 20° angled rear), sketch-dimension tweaks (kingpin length, inboard axis gap, in-sketch arm lengths), panel font-size setting, in-app design guide | **Done** |
| 14 | v1.10: static-toe-by-tie-rod tweak (knuckle rotates about the kingpin, link length + camber track it), passive-rear-steer optimizer goal (toe changes through travel at a target deg/mm) | **Done** |
| 15 | v1.11: Onshape geometry push — custom FeatureScript feature deposits 3D hardpoints + front/rear sketch planes from the variable table, design-notes PDF, and idempotent re-push (manifest of tab/feature ids updates the same tabs in place, never duplicates). Mock-tested; live validation pending | **Done** |
| 16 | v1.12: three new kinematic tweaks (hub offset = outer-CV→tire-centre distance, installed shock length @ ride with a live bump/droop stroke-split readout, and ride height as a rigid-z placement driven by a frame-tube-offset datum); plot hover readout (exact metric value at the cursor); Free (L/R) travel mode (independent right-wheel slider for single-wheel-bump roll centre); trimmed export menu | **Done** |
| 17 | v1.13 (v2 flexibility + cleanup): post-seed **scrub-radius** and **caster** tweaks (lean/tilt the kingpin to a target, keeping track + static alignment — no reseeding after hand-placing arms); **lockable hardpoints** (double-click a point to pin it: won't drag, tweaks won't move it, saved in the .MICK); the seed's **wheelbase** now drives the live Vehicle param; live-readout fix (Roll mode no longer blanks track/wheelbase/ride-height); dead-code + stale-label cleanup; placeholder **tire seam** (`tire.py`) for future grip-driven targets (no real SUNF data yet — clearly marked) | **Done** |
| 17.1 | v1.13.1: **steering link on the lower arm** (Setup option for a fixed-toe rear DW: the toe link's inner end bolts to the arm and swings with it — solver, 3D view and .MICK all follow; toe curve then tracks the camber curve, reported honestly instead of fake-tuned to zero); **drag-lock fix** (locking inboard arm points scrambled the sphere→hardpoint mapping so steering links wouldn't drag — the drag list now None-fills locked entries instead of removing them) | **Done** |
| 20 | v1.16 (shock hardware truth + absolute locks, verified on a real chassis file): **fixed shock spec** (`ShockSpec` min/max lives with the axle in the Halfshafts & shocks panel, persisted + auto-migrated, overrides the seed form so hardware is never lost); **stroke split + stroke-used Readout rows**; **locks are absolute** (rigid tweaks + vehicle ride height refuse with errors — vehicle RH atomically; optimizer skips locked points; reseed already honored them) and **locked points draw amber** in 3D | **Done** |
| 52 | v1.43 (**steering effort**): new `steering.py` + **Sweeps ▸ Steering effort (rack force)…**, for sizing the bolted joint that pins onto the rack. Reports kingpin torque per wheel, tie-rod axial force, **rack axial force**, and steering-wheel torque / hand force. Moment about the kingpin is the exact 3D `k·[(P−A)×F]`, NOT the textbook `Fy×trail` — that form assumes a vertical kingpin and a real one leans well off vertical, so they differ a few percent. The tie-rod moment arm is likewise exact, `k·[(T−A)×u]`, and differs from the tweaks panel's scalar "steering arm length" by **around 20% on a real knuckle** because the tie rod is not perpendicular to the kingpin — with the scalar erring light, which would under-size the joint. Load case is cornering at the grip limit, wheel loads from the dynamics sheet (so lateral load transfer is included) and load-sensitive μ from the measured tire, so the lightly loaded inside tire carries the higher μ. Loads read from the Dynamics panel with a plausibility warning — the reference file's mass block still held 1/10 RC values, which would have made every force ~200x low and plausible-looking. **VERIFIED BY VIRTUAL WORK** through the solver (`F·d(rack) = M·dθ`), sharing no code with the implementation: agreement **1.2e-4**, and the residual converges to a constant rather than shrinking with the difference step, because it is **steering jack** — the solver holds wheel-centre height, so steering about an inclined kingpin rotates the arm slightly and the ball joints translate. Pneumatic trail is handled geometrically (the force acts that far behind the contact patch) rather than as a scalar Mz, after the scalar version got the sign wrong and made a trail lighten the rack. Documented as an under-estimate on two counts: trail defaults to zero (aligning moment peaks BELOW the limit) and **parking effort is not modelled and usually governs a manual rack** | **Done** |
| 51 | v1.42 (**live-tracking audit**): after the roll-angle track bug, every derived numerical output was swept for the same fault — a number computed from a value typed once rather than from the live model. **Three more found, all the same shape.** (1+2) **The roll sweep and the steer sweep** both read `last_setup.track_width`, identical to the roll-angle readout; both now call `_axle_track` (the geometry). (3) **Front/rear track in the Dynamics panel were hand-entered** while the model already knew them, and track divides straight into lateral load transfer — on the reference file they held a 1/10 RC car's value left over from an earlier session, against the full-size track actually built. Now linked (🔗), along with **loaded tire radius**, taken from the contact patch (the kinematics carry no tire squash, so it is the free radius and reads a percent or two optimistic). Pushed as `None` rather than 0 for an unseeded axle, since the dynamics sheet divides by track. **Audited and deliberately left as inputs**: vehicle wheelbase and CG (each axle is modelled in its own local frame, so one corner's hardpoints cannot supply them — both already carry plausibility checks), masses, spring/damper rates, tire friction data, brake hardware. `impact.py` and the equation sheet were confirmed to read only live `hp`/`state`, pinned by a test that greps for `last_setup` in those modules | **Done** |
| 50 | v1.41 (**track vs overall width; live track**): reported as "the track readout disagrees with the CAD by one tire width". **Not a bug** — the two readings fit the arithmetic *identically* (`overall = 2*(wc_y + tire_w/2)`, `track = 2*wc_y`), so the larger number could be a correct track measured edge-to-edge OR a genuinely wider track whose wheel centre was entered half a tire width too far inboard. Parts of the file leaned each way, so **no amount of arithmetic could settle it** — it took one physical CAD measurement, centreline to tire mid-plane, after which the tool turned out to be right and the CAD figure was the **overall width across the outsides of the tires**, i.e. track + one tire width, matching to thousandths. Track is mid-plane to mid-plane per SAE J670 (the tool uses the contact-patch centres, so it breathes with half-track change); redefining it as centre-to-centre + tire width would have corrupted load transfer, roll rates and ARB rates, so instead **"Overall width (tire edge to edge)" is now its own labelled readout** and the two can never be mistaken again. **Real bug found while checking**: roll angle is `atan(2*travel/track)` and that track came from `last_setup.track_width` — the value ASKED for at seed time, not the geometry built. On the file in question the seed field was a whole tire width off what the linkage actually built, **biasing every roll number by 14.6%**; `_axle_track` now reads the hardpoints, and a new checklist row flags a seeded-vs-built divergence beyond 1% of the car's own track | **Done** |
| 49 | v1.40 (**frame mesh travels with the file; sketch-yaw tweak**): three requests. (1) **The chassis backdrop is embedded in the .MICK** — it was stored as a PATH, which breaks the moment a design moves to another machine, the one thing a save file exists to survive. `frame.embed_mesh_file` deflates and base64s the SOURCE file's own bytes (not a re-export: `load_frame_mesh` already knows every format we accept, and round-tripping through a writer of ours would quietly change the geometry — a byte-identical round trip is asserted). The path is kept alongside for re-linking. Blob is cached at load so saving never re-reads a multi-megabyte file; **autosave skips the embed** since it runs on a timer and a crash recovery is always same-machine. 2.4 MB mesh → ~3.0 MB in-file (.3mf is already a zip); ASCII STL compresses several-fold. (2) **Chassis opacity 0.35 → 0.60**, plus an Opacity box in the Frame panel, persisted per project: VTK does not depth-sort transparent surfaces, so several imported STL parts stacked into a muddle. (3) **Sketch plane yaw is now a tweak**, not a seed-only input — a sign error (typing +20° for tubes that run −20°) used to need a reseed. `set_sketch_yaw` performs the same minimal repair Re-square does (midpoints, spread lengths and kickup ELEVATION preserved), on the four types with a 2D design sketch: DW, C-hub, H-arm, loaded halfshaft. `sketch_yaw_changed` updates the recorded design intent so the checklist and sketch-fit readout follow. Two ordering traps found by the existing tests: the re-square must run **before** `plan_deg`/`elev_deg` (those deliberately skew the carrier pin OFF the sketch, and squaring erases them — so it lives at the top of each per-type setter, not in `_apply_common`, which runs last), and it must be a **no-op when the requested yaw equals the current one**, because callers routinely hand back the whole params dict with one value edited and squaring would straighten misalignment nobody asked to touch | **Done** |
| 48 | v1.39 (**batch hardpoint entry; derived-cell warning**): reported as "one of the 3 dimensions would let me enter a value but wouldn't actually input it". The first hypothesis — that single-cell edits walk the linkage through unbuildable intermediate states and get refused — was **checked and is false**: across 864 two-axis destinations on the example front geometry, every single-axis intermediate also solved. The real cause is the **knuckle-consistency convention**, which makes some coordinates DERIVED. Typing +10 mm into each of the 33 coordinates, what actually survives is `wheel_center` X → **0.00 mm** (the axle station is derived) and `tierod_outer` Y → **0.13 mm** (the steering-arm plane holds it). Only the wheel-centre case was ever reported, hard-coded by name; the tie-rod outer swallowed 99% of the typed value in silence — one specific dimension of one specific point, exactly as described. The check is now **general over every point and axis**, and restricted to cells the user actually TYPED, because other points shifting is the convention working as designed (moving an inner pickup legitimately re-derives the knuckle by millimetres) and reporting that too would bury the case that matters. Plus the requested feature: **batch edit** — a checkbox holds every cell until Apply, pending cells go bold amber with a live count, Revert discards, unticking applies. A **refused batch KEEPS what was typed** and marks it red rather than reverting a whole table because the last cell was wrong; live mode still reverts as before. The refusal REASON now lands under the table instead of only in a status bar that times out. Pending cells survive an unrelated refresh (travel slider, axle switch) and are dropped on a unit change, since 300 typed as mm is not 300 as inches | **Done** |
| 47 | v1.38 (**inputs commit on Enter; CAD-style CG marker**): two UI reports. (1) **Every non-hardpoint numeric field live-updated on each keystroke**, so clicking into a box showing `45`, forgetting to clear it and typing `120` walked the model through 45 → 451 → 4512 → 45123 and threw a link off to infinity before the intended number was finished. New `CommitSpin` (`gui/widgets.py`) turns Qt keyboard tracking off, so `valueChanged` waits for **Enter, Tab, focus loss or the stepper**, and selects the box's contents on the click that focuses it so typing **replaces** rather than appends — the actual root cause. **Esc** abandons a pending edit. Arrows and the stepper still act immediately. Applied to tweaks, vehicle, frame, shocks, dynamics, checklist, travel range, rack limit and the sweep dialogs. The **seed and optimizer panels keep their apply-on-button behaviour untouched**, but because a deferred spin returns its last COMMITTED value, `commit_spins()` now forces any half-typed field to interpret itself before those buttons read it — otherwise a value typed and not confirmed would have been silently read stale, which is the one way this change could lose an edit. (2) **The CG marker is now the CAD centre-of-mass symbol** — a sphere chequered into eight octants by the parity of its three sign bits, gold `#ffd020` and near-black, so opposite octants share a colour and any viewpoint shows four alternating quadrants (position *and* orientation, not a featureless blob). Built as two flat-coloured meshes rather than one scalar-mapped mesh, so there is no lookup table to fail on a driver short of texture units. Its radius is now derived from the car (1.5% of max(track, wheelbase), clamped 2–40 mm) instead of a fixed 22 mm — the old ball was 44 mm across on a 260 mm-wheelbase RC car. Sized from the TRUE half-track, not the `extent` used for the ground plane, which carries a 700 mm display floor | **Done** |
| 46 | v1.37 (**the dynamics panel goes metric**): the Dynamics panel now follows the toolbar's display-unit toggle instead of being imperial regardless of it. **mm → SI**: mass in **kg**, lengths mm, forces and wheel loads N, spring/tire rates N/mm, dampers N·s/mm, roll and ARB rates N·m/deg, speed km/h, CL·A m². **in → Milliken's imperial set**, byte-identical to before. The noun changes with the unit — "Empty weight (lb)" becomes "Empty **mass** (kg)" — because kg is a mass and the spreadsheet's lb is a weight (1 lb = 0.45359237 kg at one g). It is a **display layer only** (`dyn_units.py`, mirroring `units.py` for geometry): `current_inputs()` always returns imperial, so saved files, sweeps, the Onshape push and every formula are unchanged, and a project written in metric opens identically in imperial. The panel holds its own imperial copy of every field rather than reading values back off the spin boxes, so **flipping the toggle cannot move a number** however many times it is flipped, and editing one field never re-rounds the others. Per-quantity decimal shifts keep the metric side at least as fine as the imperial side (0.07 kg of unsprung mass and a 2.2 N/mm RC spring both survive). Also fixed two warnings that were quietly scale-broken: "inside corner loads near zero" used a fixed **50 lb** threshold, so it fired at every Ay on a 7 lb car, and the front+rear axle sum check used a fixed **0.5 lb** tolerance, far too loose at 1/10 scale — both are now percentages of the car's own weight and neither quotes a unit, so they read correctly in either system. `compute()` also returns `None` instead of a NaN for the pitch radius when the axle weights put the CG outside the wheelbase, a state the panel passes through on every keystroke while a mass set is retyped | **Done** |
| 45 | v1.36 (**three reports from the 1/10 RC design**): (1) **the steering rack limit did not survive a save** — it lived only on the window and was never written to the project, the one UI setting a saved design did not carry. Now in `ProjectState.steer_limit_mm`, with older files falling back to the default. (2) **"bump steer does not change when I move the toe link"** with the link on the LOWER ARM. Measured: moving the inner ±30 mm changes bump steer by <0.05 deg/mm, against >1.0 for a chassis mount — a ~500x difference. **Not a bug**: the inner ball joint is bolted to the same arm as the lower ball joint, so it RIDES that arm and its distance to the LBJ is constant through travel (verified to 9 decimals). The seed already knew and skips that secant deliberately; the UI never said so. The tweaks panel now states it and points at the OUTER rise, which has ~10x the authority and **crosses zero**, so bump steer can actually be dialled out. (3) **"the roll axis is really sloped, the front RC must be wrong"** — the front roll centre is **correct**, verified against an independent contact-patch→IC construction (agrees to 1e-3 mm, swing arm 2.00x half-track), and the roll axis is only −3.0°. The actual fault was the hand-entered CG: full-size values left on a 1/10 car — CG 852 mm behind the front on a **260 mm wheelbase** (3.28x, i.e. 592 mm behind the REAR axle, implying a −2.28 static front weight fraction) and a CG height 15x the tire radius. Nothing flagged it because every other readout still looked plausible. New CG plausibility check in the vehicle panel, judged against the car's OWN geometry so one check serves Baja and 1/10 alike | **Done** |
| 44 | v1.35 (**measured tire data + Milliken yaw-moment diagram**): a Baja SAE Discord post by an Auburn student ("dynosaur") supplied two things we did not have. (1) **The first MEASURED lateral data for a Baja-class tire** — a Dunlop KT821 22x8-10 on hard clay and NCAT gravel, as a two-coefficient exponential model (mu = c·Fz^m, Fy = Fz·mu·(1−e^(−mu·α))), from which peak mu AND cornering stiffness (C_α = Fz·mu², exact) both fall out with load sensitivity as a power law. This filled the exact gap the v1.29 literature search reported as unfillable. **It also settles the vertical-rate conflict I raised in v1.29**: measured 350 lb/in @ 10 psi = 35 lb/in/psi against our estimated 30 — we were right and the 60–70 lb/in/psi racing rule does not apply to this tire class, so a Baja car's wheel hop stays ~9.8 Hz. **Our SunF numbers were recalibrated** and two errors were found compounding: peak mu 0.70 → 0.45 (we were ~35%% optimistic) and cornering stiffness 16.5 → 30.2 lb/deg (~45%% low). Together they predicted 89 lb of lateral force at 150 lb/8° where measured says 66. A test asserting knobbies peak past 8° was **overturned by the data** — the measured tire peaks at 6.7°, because a tire that builds force twice as fast against a lower ceiling necessarily peaks earlier. (2) **Yaw-moment diagram (MMM)** in new `yaw_moment.py` + a Sweeps dialog: the whole manoeuvring envelope rather than one operating point, with limit Ay, trim Ay, **stability index** (CMG/LAG per Milliken SAE 760712 — NOT dCn/dβ, which is a different quantity), control moment gain and **limit balance** (understeer/oversteer AT THE LIMIT, routinely the opposite of the linear-range gradient we had). Reproduces the author's own headline observation: stability index climbs from −0.82 at 10 mph to +0.35 at 40, and control moment gain goes NEGATIVE below ~18 mph — the car fights being steered, which is the "turns into a pig at low speed" he described. The **Auburn vehicle file is a real Baja car** and is shipped as `AUBURN_EXAMPLE`, giving our load-transfer maths its first independent validation case | **Done** |
| 43 | v1.34 (**C-hub squares to its 2D sketch; caster pill**): a C-hub corner has ONE chassis bushing axis and one carrier pin, and both are meant to be normal to the same 2D design sketch — the direct analogue of a double wishbone's two arm axes. Neither half of that was honoured. The kickup tweak rotated the arm and **left the pin behind**, so 25 deg of kickup meant 25 deg of misalignment, and **Re-square refused non-wishbones outright**, so there was no way back — which is exactly why a C-hub went strange the moment the kickup was touched. Now `set_arm_axis_kickup` carries the pin (misalignment stays 0.00000 at every kickup, as the DW's does), and new `carrier_planarity` / `square_carrier_axes` in `geometry.py` give the carrier types the same repair the DW has had, wired to the same Re-square button and graded in the Design Checklist. **New caster-pill knob**: on the real car an eccentric pill where the lower arm's pin passes through the C-block turns the block — and the kingpin fixed in it — relative to the arm, so `set_chub_pill_caster` rotates the carrier about its pin's lateral axis and **total caster comes out as kickup + pill exactly** (10 + −5/−2.5/0/2.5/5 → 5/7.5/10/12.5/15, matching the real insert values), which is the RC rule 'caster = kick-up + caster-block angle'. Re-square **preserves a dialled-in pill**: carrying the block through the pin's rotation alone drifts caster ~1 deg, so the pill is re-asserted afterwards. Also made the squareness metric numerically stable — `arccos(dot)` reads ~1e-6 deg on a perfectly square corner, the half-angle `2·atan2(|a−p|,|a+p|)` reads 0. **v1.34.1**: the hardpoint table must never fail silently. Reported symptom was typing a coordinate not moving the point and locking a row doing nothing. Three separate silent-swallow paths closed: (a) `refresh()` set `_refreshing = True` with no `try/finally`, and that flag gates `_on_cell` — ONE exception partway through latched it forever, so every keystroke AND every lock was ignored from then on, with no error and no way back but a restart (the single mechanism that reproduces both symptoms at once); (b) the wheel centre's fore/aft station is DERIVED on a double wishbone — the knuckle convention keeps the spin axis in the ball-joint plane, so a typed value is projected straight back out and used to vanish without a word; (c) locking a halfshaft CV row returned silently because those rows are derived. New `note` signal wires all of them to the status bar. Also filled the **⊥ sketch column for the C-hub** (and the other carrier types) — v1.34 gave them a real sketch plane, so the column no longer reads a dash. **v1.34.2**: the tweaks panel is now grouped into labelled sections (Shock mount / Steering / geometry / **Placement**). The ride-height row has been on every type since v1.31 and works correctly there — but on a C-hub it sat at row 19 of a flat list of 21 and read as missing, which is a real usability failure at that length. Captions are display-only rows (`kind="head"`) that never reach the spin dict, the params dict or a setter | **Done** |
| 42 | v1.33 (**four decisions from the type audit**): (1) **`x_offset` now names the AXLE STATION** — the wheel centre lands on it, at every kickup and for every type. It used to be a plain translation, which anchored the inboard bushing axes instead (the seed pins those at x = 0 and kickup then carries the wheel centre fore/aft ~2.4 mm per degree, so the axle landed up to ~38 mm off). That mattered because the rear axle is exported shifted by exactly the entered wheelbase, so two axles at different kickup did not sit the entered wheelbase apart — and wheelbase feeds every anti-geometry percentage. It also disagreed with the MEASURE path, which always read x_offset back as the wheel-centre x. `z_offset` keeps naming the GROUND PLANE, so both offsets are now datums on the car. (2) **Trailing arm joins the rear-only set**: measured, its toe reads 0.200 deg at both 0 and 20 mm of rack — no tie rod, so a trailing-arm front axle cannot turn, the same failure mode as the H-arm and loaded halfshaft. The restriction is now self-verifying: a test asserts every rear-only type fails to respond to the rack and every front-legal type does. (3) **`seed_multilink` honours your shock inputs**: it inherited the DW's shock re-homed to the lower ball joint, so installed length read 653 mm against a requested 517.65 and MR 0.600 against 0.541. With the outer eye pinned to the upright the chassis end controls both — axis direction sets MR, distance along it sets length — so a scan on the inclination plus a placement now hits both to 1e-4 across a range of goals. (4) Polish: `steering_arm_x` is now a setup-form field (it was a fixed -80 mm, absurd at 1/10 scale); the Design Checklist explains the toe-vs-anti-geometry trade on the toe-holding layouts instead of silently reading 0%; and the trailing arm now uses the DW's EXACT static-spindle form, so its camber read-back went from 5.3e-06 deg to 0. **v1.33.1**: worked the GUI smoke test back to GREEN end-to-end for the first time in many versions. Fixing the x_offset datum unblocked line 400, which had been failing so long that everything after it was dead code; four more defects were queued behind it. Two were REAL app bugs — the seeded halfshaft inner CV was placed with the seed report's travel budget and then left stale when apply_hardpoints replaced that budget with the shock hardware's limits (so the shaft was ~3.9 mm off the level-at-mid-travel it exists to guarantee), and re-placing it also needed a table refresh or the CV rows showed a stale z that an edit would silently reinstate. Two were stale assertions — a column caption shortened to '⊥ sk' in v1.22.1, and an equilibrium check summing ALL mount rows instead of only the chassis ones (internal joints come in equal-and-opposite pairs, the same fix tests/test_impact.py already had). Plus a NameError from a missing import. Smoke now passes 3/3 | **Done** |
| 41 | v1.32 (**seed dispatch, axle restrictions, corner placement**): three usability fixes. (1) **"Generate seed hardpoints" ignored the type dropdown** — it always ran the double-wishbone seeder, so selecting C-hub and pressing Generate silently threw the C-hub away and built a DW; the type only survived if you never pressed Generate again. Both entry points (the dropdown and the button) now share one `_seed_as_type` path. (2) **Rear-only types are blocked on the front axle**: an H-arm grabs the upright at two outboard points and a loaded-halfshaft knuckle is closed by the shaft itself — both hold toe rigidly and have NO steering input, so a car built with one on the front physically cannot steer, yet the solver would happily pose it and plot a flat toe curve as if that were fine. New `REAR_ONLY_TYPES` / `types_for_axle` / `is_allowed_on` in the registry; the dropdown is filtered per axle from construction (not just after the first axle switch) AND the handler refuses a forced request, since a loaded project or a script can bypass the dropdown. (3) New **corner up/down** tweak (`corner_z`) beside the existing corner fore/aft, on every type including the DW — a direct rigid move, deliberately separate from the ride-height row, which is a target measured from the ground and moves the corner the other way. New `tests/test_type_restrictions.py` covers all three; its MainWindow tests **stub the VTK viewport**, so they run in the normal unit suite instead of depending on the GL-flaky smoke test. **v1.32.1**: hand audit of all five non-DW types against the DW standard (static read-backs, kingpin-metric NaN handling, branch continuity over 401 travel steps, articulation, motion ratio, IC/RC). All five pass; two behaviours documented rather than 'fixed' after measurement showed them to be deliberate — the H-arm and loaded-halfshaft seeds keep the chassis bushing axis LEVEL (kickup_deg does not reach them) because those types have no toe link, and tilting the axis costs ~0.19 deg of bump toe per degree of kickup, so holding toe is traded for ~0% anti-squat; and `from_double_wishbone` re-homes the multilink shock to the lower ball joint, which discards `shock_length_at_ride` (the v1.31 shock-length tweak is the remedy) | **Done** |
| 40 | v1.31 (**kinematic tweaks for every suspension type**): the tweaks panel had 19 rows for the double wishbone and 2-6 for everything else, so a C-hub, trailing-arm, multilink, loaded-halfshaft or H-arm corner was tunable almost only by hand-editing raw coordinates. New cross-type tweak layer in `tweaks.py` dispatches on the structure a hardpoint set actually has: installed **shock length @ ride** (slides the mount along whichever arm the type has; along the shock's own axis on a multilink, which has no single arm), **ride height** and **corner fore/aft** (already type-generic, simply never wired up), **hub offset**, **arm bushing-axis kickup**, **arm length**, and **camber-link rises**. C-hub gains the full steering set it was missing — tie-rod outer/inner rise (the bump-steer knobs the seed already secant-tunes), steering-arm length, static toe, kingpin length — going from 6 rows to 19, matching the double wishbone. Row counts now: DW 19, C-hub 19, H-arm 12, loaded-halfshaft 11, trailing arm 8, multilink 5. **Hub offset is offered only where a steering axis exists** (ball joints, a physical kingpin, or an outer CV); trailing-arm, multilink and H-arm uprights have none, so the row would read a degenerate 0 — `outer_cv_of_hp` was also extended to recognise the C-hub's physical kingpin, which had made its hub offset read 0. **Every optional row is gated**: the panel passes None for any row whose spin has not moved beyond its display quantum, so editing one row can never walk the others by the rounding error — the trap the double wishbone's hand-rolled path already guarded. Two genuinely coupled pairs are resolved by documented ordering (steering-arm length vs tie-rod outer rise; `axle_x` vs hub offset), with the more design-critical target winning. New `tests/test_tweaks_cross_type.py` round-trips every new row on every type, asserts the placement tweaks change no kinematics, pins the camber link as a strong roll-centre control (50 mm of inner-ball travel moves the roll centre ~99 mm), and drives the real panel widget headlessly for all five types | **Done** |
| 39 | v1.30 (**scale-free: the tool now designs a 1/10 RC buggy as well as a Baja car**): every seed-generator input range was floored at full-size Baja values (800 mm track, 1000 mm wheelbase, 100 mm tire radius, 100 mm shocks), so model-car dimensions could not even be typed in. All length ranges in the setup form, hardpoint scalars and vehicle panel now reach down to 1/10 scale (~250 mm track, ~285 mm wheelbase, ~31 mm tire radius, ~62 mm shocks); defaults stay full-size. **The deeper half of the fix was three fixed millimetre constants buried in the seed maths** that would have produced silently wrong geometry once the ranges allowed the values through: the C-hub tie-rod search band (±120 mm — nearly half an RC wheelbase), the loaded-halfshaft diff flange (pinned at y = 120 mm, essentially AT the wheel on a 250 mm track), and the articulation probe step (1 mm floor — most of an RC car's travel in one step). All three are now fractions of the car and reproduce their old values at Baja size to within 0.2%. The optimizer's packaging clearances were scaled the same way. **Dynamics panel too**: weights carried 0 decimals, so a 1/10 buggy's 0.15 lb unsprung corner rounded to zero and every dynamics result became garbage — weights now carry 2-3 decimals and front/rear track reach below 10 in. New `tests/test_rc_scale.py` seeds and articulates **all five suspension types** at 1/10 scale, asserts exact static read-backs and the motion-ratio goal, and guards the whole bug class by asserting no seeded hardpoint lands outside the car; a companion class pins the Baja values so the rewrites cannot drift full-size geometry. **Also fixed a pre-existing smoke-test failure** (unrelated): the hardpoint-table edit assertion demanded a 12.7 mm move to within 1e-6 mm, but read its input from a cell rounded to 3 decimal inches — a 0.0127 mm quantum. It only ever passed while the seeded wheel-centre x happened to be an exact 3-dp inch value. **v1.30.1**: chassis kickup max raised 20 -> 30 deg — RC buggies run far more kickup than a Baja car, and the spin range was the only limit (the seed maths has no clamp; verified seeding and articulating at 30 deg for the double wishbone and C-hub, at both Baja and 1/10 scale) | **Done** |
| 38 | v1.28 (**motion-ratio convention flipped to shock/wheel**): the team convention was MR = wheel/shock (**>1**) while the dynamics layer used the standard spring/wheel (**<1**), bridged by a `1/MR` conversion at the GUI boundary. The internal math was correct, but the number *displayed* was the wheel/shock one — and reading that into `K_w = K_s·MR²` from any textbook squares the mistake into a **several-fold error**. Now unified on the standard **MR = shock travel / wheel travel (<1)** everywhere: `metrics.motion_ratio` (the graph), `SetupVariables.motion_ratio_goal`, all six seed generators (double wishbone, trailing arm, multilink, C-hub, H-arm, loaded halfshaft — each had its own local `mr()` helper), the travel calculation, the optimizer target (scale raised 20→68 to keep the goal weighted identically against a ~3.42× smaller residual), panel labels/defaults/ranges, and the plot axis. **The `_mr_inv` bridge is deleted rather than documented** — the whole class of bug goes with it. **Saved-file migration**: `FORMAT_VERSION` 3 stamps `mr_convention`; files without it (or with a goal >1, which is physically impossible for shock/wheel) get the goal AND the optimizer target inverted on load, with a dialog listing exactly what changed — never a silent reinterpretation. **Also fixed a real bug in the v1.27 equation sheet**, which defined MR as wheel/spring while listing `K_w = K_s·MR²` — mutually inconsistent, and the exact trap this change removes; the virtual-work relation is now correctly `F_shock = F_wheel / MR`. Sanity check now built into the docs: since MR<1 the wheel rate is always *below* the spring rate, so an answer that comes out higher means the ratio is upside down | **Done** |
| 37 | v1.27 (equation reference — Help ▸ Equations, Shift+F1): new `suspension_tool/equations.py` holds **82 equations across 15 sections** as structured data — the kinematic solver core (Rodrigues rotation, circle∩sphere assembly closure, three-sphere trilateration, Kabsch pose), wheel angles, instant/roll-centre construction, motion ratio and rates, anti-geometry, mass/CG/roll-axis, the three-term Milliken 18.4 lateral transfer, ride/bounce/pitch frequencies and damping, understeer/Ackermann/brake chain, the tire model, CV joint angles, the whole impact load path (impulse, friction circle, two-force sign convention, the arm-axis reduced equation, min-norm indeterminacy, beam internals, rocker, virtual-work check) and both frequency-response models (quarter car, 7-DOF full car, wheelbase filtering, Froude scaling). Every entry carries its symbol definitions and the module that implements it, plus a 30-row symbol table and an explicit units warning (geometry mm, dynamics lb/in/s, **mass = W/386.4** — using 32.2 is the classic factor-of-12 error). **Single source of truth**: the in-app window and the printable PDF (`docs/Equation_Sheet.pdf`, regenerated by `tools/build_equation_sheet.py`) both render from the same module, so they cannot drift. **Drift-tested, not just proofread**: 14 tests evaluate the documented formula by hand and assert it matches what the software computes (CG, sprung mass and CG height, wheel/ride rate, roll moment arm, lateral + longitudinal transfer, roll gradient, banking sign, understeer, critical damping and ζ, pitch radius, brake chain, impulse/peak force, drop height, wheel hop, terrain excitation, ARB wheel-rate conversion); another asserts every cross-referenced module still exists | **Done** |
| 36 | v1.26.2 (Bode guide: what the y-axis should read): the first cut of the guide gave frequency (x-axis) targets but never said how TALL the peaks should be — the obvious follow-up question, and it had no answer. Added a page: **three of the four channels are dimensionless ratios per inch of road**, so their targets are identical for a Baja car, an RC car and a Formula car and are set almost entirely by damping ratio — a ζ-sweep table computed from the model itself gives peak body motion / tire deflection / suspension travel at ζ = 0.15→0.90 (e.g. at ζ 0.35: 2.5 / 1.8 / 1.9). Includes the two hard asymptotes worth memorising (**body motion is exactly 1.0 at DC**, **tire deflection is exactly 1.0 at high frequency** — a wrong asymptote means a wrong input) and, crucially, how to convert a ratio into a physical limit: dynamic tire load = tire-deflection × k_t × bump, so the Baja corner fully unloads at only **0.29 in** of sustained input at the hop frequency (the stiff-tired road car at 0.10 in) — which is exactly why periodic washboard is destructive. Also documents that **body acceleration is the one dimensional channel** and scales as f², so its g/in values (Baja 2.5, road race 6.4, 1:10 RC 34) are NOT comparable between classes until multiplied by the bump size each car actually meets — do that and they converge (~7.6 g vs ~10 g). Table row in the vehicle comparison now carries the per-class y-values. Also surfaces the non-monotonic result the model produces: past ζ ≈ 0.7 tire deflection gets WORSE again, the numerical fingerprint of over-damping. Guide is now 3 pages | **Done** |
| 35 | v1.26.1 (Bode-plot reading guide, shipped + linked in Help): a two-page PDF, `suspension_tool/docs/Reading_the_Bode_Plot.pdf`, opened from **Help ▸ Reading the Bode plot** and from a live link in the in-app user guide (`show_help` substitutes a real `file://` URL for the `__BODE_GUIDE_URL__` token, so the link works wherever the repo is checked out). Page 1 derives the curves from the quarter-car equations of motion, explains the four channels, and gives four reading rules — including the two that catch people out: **isolation only begins above √2 × f_body** (between f_body and 1.41 f_body you are amplifying the road), and **above that crossover more damping makes isolation worse, not better**. Page 2 is a sourced comparison of what "good" looks like for a **Baja SAE car** (1.5–2.5 Hz body, 9–12 Hz hop, ζ 0.25–0.45, tire-deflection-limited), a **1:10 scale RC car** (Froude scaling: frequency × 1/√λ = 3.2×, so 5–8 Hz body and 30–45 Hz hop — with the honest caveat that real RC cars are documented not to achieve dynamic similarity) and an **on-road race car** (1.5–2.5 Hz non-aero, 3–5+ Hz with downforce, 12–18 Hz hop, tire-load-variation-limited), plus a diagnostic checklist. Numbers sourced to OptimumG (Giaraffa, *Springs & Dampers* Tech Tip 1), Penske Racing Shocks, Milliken *RCVD*, Gillespie *FVD*, and scaled-vehicle similitude literature. Tests assert the PDF ships, is a real PDF, and that the help token substitutes to a valid file URL | **Done** |
| 34 | v1.26 (whole-car Bode — 7-DOF ride model): new `full_car.py` extends the quarter car to the whole vehicle — heave, pitch and roll of the sprung mass plus four unsprung masses. **Wheelbase filtering** is the payoff a quarter car structurally cannot give: the rear axle meets each bump L/V after the front, so the response is speed-dependent and combs — pitch nulls at f=n·V/L (axles in phase), heave nulls at f=(2n+1)·V/2L (axles opposed), both verified EXACT (~1e-16) for a symmetric car. **Coupled modes**: the dynamics sheet's bounce/pitch frequencies are the decoupled closed forms; this solves the coupled eigenproblem and returns all seven modes, each labelled from its eigenvector by kinetic-energy participation (heave / pitch / roll / wheel hop front+rear). **Roll and the anti-roll bars**, which a quarter car cannot represent at all: bars enter as a differential stiffness k_bar·gg^T across each axle, and the assembled roll stiffness K[1,1] reproduces `dynamics`' independently-computed ARB + spring roll rate to **0.0000%**. Also body acceleration at the axle stations (pitch makes them differ from the CG) and per-corner dynamic tire load. Cross-checks in the test suite: full-car body modes match the dynamics sheet within 5%, wheel-hop modes match the quarter car within 2%, symmetric road input never excites roll (~1e-16). New estimated input `roll_radius_gyr_in` (~0.35×track default, exposed in the Dynamics panel); pitch inertia still Olley DI=1 — both disclosed as estimates in the report notes. GUI: a model selector + speed box on the existing Bode dialog | **Done** |
| 33 | v1.25 (ride/grip frequency response — the quarter-car Bode plot): new `ride_freq.py` adds the frequency-domain view every scalar in the dynamics sheet was missing. **Model**: the standard 2-DOF quarter car (sprung mass on the wheel-rate spring + damper, unsprung mass on the tire spring), built by `from_dynamics` straight out of the existing dynamics outputs — sprung corner weight, unsprung weight, wheel rate, tire rate, damper rate × MR² — so nothing is invented and it always describes the car being modelled. **Four transfer functions**: body acceleration (harshness), tire deflection (the GRIP channel), body motion (isolation) and suspension travel used. **The wheel-hop mode**, ~10 Hz, where the unsprung mass bounces on the tire spring — previously reported nowhere in the tool, and the mode that makes a tire skip; taken from the undamped eigenproblem so it is exact and damping-independent (peak-picking on a damped response is unreliable near ζ=1). **Terrain table**: bumps spaced λ apart at speed V drive f=V/λ, so each resonance converts to the road speed that excites it, over editable spacings — on the default preset washboard hits wheel hop at ~14 mph. **Damping as a band** between the bump and rebound curves, because one linear coefficient cannot represent an asymmetric damper, plus warnings when either ratio leaves racing practice (~0.3–0.5 bump; the preset is 0.92/1.23, i.e. over-damped, which *raises* body acceleration). **Cross-validated**: the 2-DOF body mode (2.181/2.202 Hz) agrees with `dynamics`' closed-form ride frequency (2.194/2.223 Hz) to 0.6% from entirely different maths, and wheel hop matches the textbook decoupled form to 0.6% — regression-tested, so this doubles as a check on the dynamics layer. Documented as linear/small-perturbation (no bump stops, progressive springs or tire lift-off): a design-target and comparison instrument, not an absolute prediction. Sweeps ▸ Ride/grip frequency response | **Done** |
| 32 | v1.24.1 (arm bending + per-part FEA load sets + a real defect fix): prompted by "the shock applies a force onto the arm that causes a bending moment, this will change the statics". **Verdict on the statics: it does not** — for a rigid body the joint reactions follow from global equilibrium regardless of how load is distributed inside, and bending is self-equilibrating by definition; an independent from-scratch derivation (3 bodies, 20 unknowns, 18 equations, 2 documented axial indeterminacies) reproduces `_dw_corner_loads` term for term, and virtual work confirms it. **But the instinct found a real bug of exactly that shape**: `_dw_corner_loads` never read `tierod_on_lca` / `tierod_on_uca`, so when the toe link's inner ball joint bolts to a control arm (a Setup option the kinematic solver honours) its reaction on that arm was missing from the arm's moment equation and bushing split, and the mount was tagged as a frame pickup. Shock axial was off by up to 3.1% and individual pickups far more; **neither safeguard caught it** (residual 1.8e-10, chassis mounts still summed to the wheel load). Fixed and verified against virtual work at 0.0000% for every shock/toe-link mounting combination. **New capability for what the question was really after**: `BodyLoadSet` gives each part (both arms, upright) its COMPLETE self-equilibrated load set (ΣF = 0, ΣM = 0 to ~1e-10) — apply it to the part in ANSYS/Onshape with only a weak restraint and the FEA shows the bending directly; ball joints and the shock's arm end are now emitted as internal mounts (the double wishbone previously emitted none); `control_arm_beams` / `beam_internal_loads` give bending / torsion / shear / axial along each arm leg (zero at the spherical bushing, stepping at the shock mount) under a documented V-arm idealisation that honestly flags the inherited bushing indeterminacy; `shock_bending_summary` gives the assumption-light headline — on the seed the shock puts **~10,000 lb·in** into the lower arm about its ball joint, which a truss idealisation reports as zero. New File-menu export | **Done** |
| 31 | v1.24 (Lotus-style load matrix + envelope): the impact loads are extended from one-axis-at-a-time to the **friction-circle load matrix** the way Lotus Suspension Analysis and standard FSAE practice do it. A `WheelLoad` carries a full `(Fx, Fy, Fz)` at the contact patch + an optional tire couple (aligning/overturning `patch_moment`, now threaded through every solver — DW, multilink and the serial chains). `friction_circle_loads` sweeps a combined horizontal force `|F_horiz| = μ·Fz` around the traction circle (braking → drive → cornering → every combination); `standard_load_matrix` builds the full set — static, the circle, a vertical bump, and the two impulse cases — **auto-derived** from the tire (μ by surface, pneumatic trail) and dynamics (per-corner vertical incl. lateral + longitudinal transfer + aero) models so it matches the car. `envelope` runs the whole matrix **through the travel range** (the double wishbone is re-posed + re-solved at each travel from the kinematic solver — worst link angle is usually at bump/droop, not ride height) and keeps the **worst force per pickup / worst axial per member**, tracking peak tension AND compression separately and naming the governing case + pose. New CSV export (File menu). Plus an **opt-in pushrod/bellcrank** load path (`RockerGeometry` / `rocker_loads` / `corner_loads_rocker`): routes the pushrod axial through a rocker to the damper force + pivot bearing reaction. Verified: patch-moment equilibrium (force + moment to ~1e-4 rel), envelope ≥ every single case, travel-0 pose ≡ static, rocker force+moment balance to ~1e-12 and the corner still reproduces the wheel load. Research basis: fswiki Suspension Forces, Machine Design (Lotus), IRJET/IJERT Lotus SHARK papers, FSAE.com | **Done** |
| 30 | v1.23 (impact loads → FEA export): new `impact.py` turns an impact scenario into FEA boundary conditions. **Impulse chain**: F_avg = m·Δv/Δt, scaled to a design peak by a pulse-shape factor (half-sine ×1.57, triangular ×2.00), times the corner's share and a safety factor; drop heights convert via v=√(2gh). **Load path — ALL six suspension types**: double wishbone as a fork (upright's 6 equations + one moment equation per arm about its own bushing axis = 8 unknowns/8 equations, determinate); multilink as a square 6x6 (5 links + shock); and trailing arm / H-arm / C-hub / loaded halfshaft as serial rigid-body chains solved outboard-to-inboard (C-hub = steering block -> C-hub -> lower arm), each body's closing link reacting the moment its joint pair cannot. Gives a force vector at every chassis pickup plus the axial force (tension/compression) in every member; rows are tagged chassis (frame FEA load set) vs internal (ball-joint/hinge/kingpin loads for sizing the joint + upright). **CSV export** (File menu) in lb or N, in the chosen display convention, self-describing with the whole scenario. Verified 3 independent ways ON EVERY TYPE: chassis mount loads sum back to the wheel load in force AND moment to ~1e-13, and the shock force matches wheel-load × motion-ratio from the kinematic sweep to 0.00%. Documented as quasi-static (rigid links, no bushing compliance); the arm's bushing axial split is indeterminate and the total is reported | **Done** |
| 29 | v1.22.1 (dynamics audit + estimated tire data): every `dynamics.py` output was re-derived against SAE J670 / Milliken RCVD / Gillespie / OptimumG. **Three real defects fixed** (numbers now differ from the old spreadsheet port): lateral load transfer used total weight in the sprung roll-moment term + omitted the unsprung-at-wheel reaction (**overstated ~12%** → full Milliken 18.4 three-term form); bounce/pitch frequencies used the wheel rate, putting bounce **above both ride frequencies** (impossible) → ride rate; banking sign made SAE-standard (positive = into the turn). Rest audited correct/convention (documented in the module + skill). Plus **estimated SUN-F 23x7-10 tire data** in `tire.py`: vertical rate, surface-dependent peak μ (0.4 mud–0.8 pavement), cornering stiffness, peak-slip angle, camber guidance (knobbies want near-zero static camber), Pacejka-seed coeffs — every value tagged with a `Confidence` (measured/derived/informed-estimate/rough-guess) and its derivation basis; no measured F&M exists yet | **Done** |
| 28 | v1.22 (inner-joint type + directional plunge budget): the inner (diff-side) joint now has a **type selector** — Plunging CV (tripod/AAR/VL, ~25°, plunges internally), Rzeppa (fixed CV, ~47°, no plunge), Double-cardan U-joint (~45°, plunge via an extending slip-spline shaft). Picking a type presets the inner angle limit and enables/disables the plunge budget; a fixed Rzeppa inner warns if the geometry demands shaft-length change. The **plunge spec is now directional**: a total stroke + a plunge-in % measured from the ride-height shaft length (rest = pull-out), each direction checked against its own allowance — asymmetric because ride rarely sits centred in the joint's travel (shock-split analogy). `HalfshaftConfig` gains `inner_joint` / `plunge_total_mm` / `plunge_in_pct`; old `.MICK` (single `max_cv_deg`, symmetric `±max_plunge_mm`) migrate automatically (behaviour-preserving) | **Done** |
| 27 | v1.21.2 (separate inner/outer CV angle limits): a literature review of every graph output confirmed all 12 plotted channels are computed correctly (camber/toe/caster signs, motion ratio, bump steer, half-track change, wheel recession, anti-squat/dive %, CV angle, plunge — checked against SAE J670/ISO 8855, Milliken RCVD, OptimumG, etc. and numerically). The one design refinement it surfaced: the halfshaft check used a single CV-angle limit for BOTH joints, but the inner plunging joint (tripod/AAR/VL ~22–33°) tolerates far less than the outer fixed joint (Rzeppa ~45–50°, which also takes the steer angle on a driven front). Split into `max_cv_inner_deg` (default 25°) / `max_cv_outer_deg` (default 45°), each checked against its own peak with the warning naming the joint; old single-limit `.MICK` files migrate onto both (behaviour-preserving) | **Done** |
| 26 | v1.21.1 (roll-centre / anti accuracy fix): the roll centre is correctly built in the **front view** (transverse vertical plane, per SAE J670 — confirmed against Milliken RCVD, SAE/ISO, ScienceDirect, Suspension Secrets), but the double-wishbone construction collapsed the 3-D arms into that plane by slicing both arms at the single **wheel-centre plane**, which is only exact for flat (zero-kickup) arms. Now each arm is sliced at its **own ball-joint station**, matching rigid-body kinematics exactly (verified against a finite-difference rigid-body IC). Impact: front RC reads correctly with kickup (≈ +5 mm at 10°, +25 mm at 20°) and now properly **rises** with kickup instead of looking flat; same fix on `side_view_ic` for anti (unchanged for the parallel-arm seeds, so the v1.17 anti audit holds; corrected only for deliberately-skewed anti). Zero change at 0° kickup. Added exactness regression tests | **Done** |
| 25 | v1.21 (H-arm rear suspension type): a simple, effective **driven rear end** — a rigid lower **H-arm** (two inboard bushings on a fore-aft axis) grabs the upright at **two outboard points** so it holds toe and locates the bottom of the upright with no separate toe link, closed by a single **upper camber link** for camber. Driven by default (CV halfshaft angle/plunge overlay, kingpin-less so the outer CV sits at the wheel centre; disable for a dead axle). Toe stays fixed through travel (the wide grab base holds it); skewing the **grab line** in the tweaks panel adds gentle passive rear steer. Closed form via the shared hinge-carrier core, verified against an independent scipy full-pose closure to <1e-7 mm; full GUI parity (seed dropdown, table, drag, 3D, tweak box, optimizer groups, .MICK, CSV + Onshape skeleton lines); RC via the numeric IC path | **Done** |
| 24 | v1.20 (two new suspension types): **C-hub front** (RC-buggy style, AE B7 scaled up — lower arm → hinge pin → C-hub/caster block → camber link to a free chassis ball, steering block on a **physical kingpin fixed in the C-hub**; caster/KPI/axle-offset are first-class tweak knobs like swapping blocks/inserts; seed aims kingpin at desired caster+scrub, aims the camber link at an inboard IC, secant-tunes bump steer to ~0) and **Loaded halfshaft rear** (upper arm w/ shock → knuckle on a bushing pin → the **halfshaft itself closes the loop** as a structural fixed-length link from the diff flange; no toe link — the pin fixes toe, skewing it is a gentle passive-steer knob; **plunge ≡ 0 by construction**, hs_inner/hs_outer are real hardpoints synced live with the Halfshafts panel). Both closed form via a shared hinge-carrier core (`hinge_carrier.py`), verified against independent scipy full-pose closures to <1e-7; full GUI parity (seed dropdown, table, drag, 3D, tweaks box, optimizer groups, .MICK, CSV/Onshape skeleton lines); ICs/RC via the numeric Adams-style path | **Done** |
| 23 | v1.19 (shared vehicle frame for coordinate entry): the rear axle's hardpoint table and halfshaft inner-CV entry now read/write in the **shared vehicle frame** — rear shifted −wheelbase fore-aft so both axles reference ONE origin (the front/firewall datum), matching the CSV/Onshape export and the 3D scene (which already did). Fixes the "rear has no Y datum, doesn't match Onshape" complaint. Display/entry only (`HardpointTable._to_view/_from_view`, `HalfshaftPanel` per-axle `set_vehicle_dx`, driven by `MainWindow._push_vehicle_frame`); storage stays axle-local so old .MICK files are unchanged; the offset tracks the wheelbase live | **Done** |
| 22 | v1.18 (arm-mounted toe link — physics corrected): mounting the toe link's inner ball joint on a control arm does **NOT fix toe** (the v1.13.1 framing was wrong). Bolting the inner to the same arm as the ball joint forms a near-rigid BJ–inner–steering-arm triangle → the steering arm follows that arm → **large bump steer** (verified 4 independent ways: closed-form, scipy constraint solver, brute-force knuckle-rotation, analytic). Added **upper-arm option** alongside lower (mutually exclusive; ball-joint inner rides the chosen arm via its UCA/LCA bushing-axis rotation, threaded through the solver). Seed seats the **outboard link low (lower) / high (upper) on the kingpin** to keep the tie-rod IC near the arm's IC and minimize the (real) bump steer — the tuning lever is the outboard height, not the inner position; docs/tests/seed comments corrected | **Done** |
| 21 | v1.17 (independent measurement audit + fixes): a **13-cluster verification** re-derived every user-facing measurement by alternative means (from-scratch scipy constraint solver — tool states satisfy all joint constraints to 4e-13 mm; hand rotation matrices; symbolic line-plane intersections; first-principles Milliken formulas — all 12 dynamics outputs exact) and confirmed the tool correct to machine precision. Four issues found + fixed: **spindle** rebuilt so camber AND toe read back exactly for any input (was gamma·tau²/2 coupling); **anti-geometry continuity** at the parallel-arm design state (IC-at-infinity fallback now uses the arm-plane direction, killing a 2–4 pp jump on the shipped seeds); **change-channel datum** to true static (was nearest sample, ~1 mm offset); **motion-ratio/bump-steer** endpoints now 2nd-order (`edge_order=2`) | **Done** |
| 20.2 | v1.16.2: **separate Roll-centre / IC-marker view toggles** (the combined "RC/IC/axis" checkbox split in two — hide the RC dots + roll axis independently of the instant-centre markers + rays, for a decluttered view for new users) | **Done** |
| 20.1 | v1.16.1: **the travel range IS the shock's range, automatically** — with a spec set, every edit/load/reseed re-probes the exact travels where the shock hits its hardware min/max (fast bracket+bisection, ~0.1 mm accuracy, ~0.2 s) and sets the range to them, expanding AND shrinking; range spins become displays; CV/plunge remain pure warnings, never caps. On the reference file the front was posing 128% of the stroke and the rear capped the shock short of its hardware maximum — both now sweep the full hardware stroke (100%) with no user action | **Done** |
| 19.2 | v1.15.2: **vehicle ride-height knob** (Vehicle panel: set BOTH axles' measured ride height in one action — rigid z translations honoring each axle's frame-tube datum, no reseed, no kinematics change, per-axle undo; the one-click fix for a red stance row; spin tracks the front axle's live value) | **Done** |
| 19.1 | v1.15.1 (ride-height/stance trust round): **measured ride-height readout** (from the displayed geometry, identical to the tweak row — the old readout echoed the stale Setup field, so the 3D and the numbers could disagree; the two rows also differed by the frame-tube offset before); **F/R stance row** in the Vehicle panel (red when the axles' ground planes differ); **common-ground roll axis** (cross-axle metrics + dynamics link reference both RCs to the front ground plane, matching the 3D axis — a 40 mm stance error used to hide a 1.5° axis-angle error); **Ride height / Reset pose reset BOTH axles** | **Done** |
| 19 | v1.15 (reseed frustration round, tested on a real chassis file): **kickup as a live tweak** (rotates the bushing axes about their own midpoints — tube centres/spreads/yaw kept, kingpin re-planarized so caster follows; isolates kickup studies with zero reseed. Finding: front RC rises ~3.4 mm/deg of kickup, camber gain strengthens alongside, roll axis flattens ~0.3°/deg); **reseed honors locked hardpoints** (`apply_locked_points`: new seed slides onto the pinned points in X/Z then keeps them EXACTLY — welded tabs survive; kingpin re-squared + knuckle convention re-enforced around them); **landing warning** when a reseed still jumps >25 mm from the previous design | **Done** |
| 18 | v1.14 (trust + reseed-without-loss): **self-reconciling sketch-fit readout** (shows measured caster vs kickup vs yaw — off-plane ≠ caster−kickup on a yawed sketch because KPI tilts the kingpin too; the confusing number is now checkable); **live wheelbase readout** (wheel-centre to wheel-centre along chassis-forward, breathing with recession through travel); **corner fore/aft tweak** (rigid X slide of the whole corner — retune seed placement without reseeding); **Measure setup from current design** (`setup_from_hardpoints` inverts the seed: fills the Setup form from live/candidate geometry so a reseed reproduces the design — round-trips to <0.25 mm); **⊥-sketch hardpoint column** (4th table column edits each point's offset along the sketch normal — packaging moves that preserve in-sketch kinematics) | **Done** |

## Roadmap from industry-tool research

We surveyed OptimumKinematics and MSC Adams Car for features worth
porting to a pure-kinematics Baja tool. Reassuring precedent: Adams
itself has a "kinematic mode" that locks out compliance — running
suspension events without forces is standard early-design practice, so
this tool mirrors a real industry workflow. Already adopted from the
survey: half-track change + wheel recession channels, RC/IC display, the
static readout table, and Adams-Insight-style hardpoint optimization.
Next most valuable (in rough priority order):

1. **Roll sweep** (opposite wheel travel): camber/toe/RC migration vs
   body roll — needs the left+right pair modelled together.
2. **Steering sweep** on the new rack-steer input: Ackermann %, turn
   diameter, steer-camber; plus bump-steer maps at any rack position.
3. **Anti-dive / anti-squat %**: pure side-view geometry, but needs a
   few vehicle scalars (CG height, brake bias, drive type) as inputs.
4. **Design comparison**: overlay N saved designs' curves (we already
   overlay before/after for the optimizer).
5. **Report export**: the static table + curves to CSV/Excel.

Out of scope by design (they need forces/compliance): K&C compliance
outputs, force-based roll centres, and full-vehicle dynamic events.
(Wheel/ride/roll RATES joined the tool in v1.3 via the Dynamics panel —
they need masses and spring rates, not compliance.)

## v1.13: post-seed flexibility + lockable hardpoints (v2 groundwork)

The headline gripe this release attacks: certain values were baked in at
seed time and couldn't be changed without regenerating a whole new seed
(and losing hand-placed hardpoints).

* **Scrub radius** and **caster** are now Kinematic-tweaks rows. Each
  inverts the seed relationship in place — the scrub tweak leans the
  kingpin sideways (moves the UBJ in Y), the caster tweak tilts it
  fore/aft (moves the UBJ in X) — hitting the number you type while
  **track and static camber/toe stay put**. No reseed after you've
  positioned the arms. (Caster set this way reverts on Re-square in the
  default one-sketch workflow; independent-caster mode keeps it. Verified
  against `metrics.scrub_radius_mm` / `caster_deg` at ride height.)
* **Lockable hardpoints**: double-click a point's name in the Hardpoints
  table to pin it (🔒). A locked point won't drag, and the scrub/caster
  tweaks raise instead of moving it — so fixed halfshaft mounts and
  chassis points you like are protected. Locks persist in the .MICK.
* The seed's **wheelbase** now drives the live Vehicle-panel value (it was
  inert for a single corner before); keep editing it there.
* **Live-readout fix**: Roll mode no longer blanks the track/wheelbase/
  ride-height rows — all four motion modes now feed the Readouts panel
  through one path.
* **Cleanup** toward a "v2" feel: dead Onshape handlers and orphaned
  symbols removed, stale menu labels corrected, the help guide reconciled
  to the single Build/update CAD flow.
* **Tire seam** (`tire.py`): a placeholder lateral-force model + the
  cornering-stiffness hook the dynamics layer already exposes, so future
  work can make targets grip-driven. There is **no measured data** for the
  SUNF 23x7-10, so the placeholder is loudly flagged as an engineering
  guess. (Bigger structural refactor + a real tire model are staged as
  follow-ups.)

## v1.11: Onshape geometry push (points + planes + PDF), idempotent

**File → Build/update CAD in Onshape** goes beyond the variable table:
* A **Variable Studio** is the single source of truth (one length
  variable per hardpoint coordinate, in the chassis frame).
* A **Feature Studio** holds ONE custom FeatureScript feature that reads
  the variable table and regenerates **every hardpoint as a 3D point**
  plus the **front/rear 2D design sketch planes** (which inherit the
  kickup + sketch-yaw twist). Onshape has no importable free 3D-point
  and won't let a variable drive a raw sketch point, so the custom
  feature is the clean, low-call-count way to do this.
* A **design-notes PDF** (per-axle readouts, goals vs achieved, camber
  and toe curves) is dropped in as a blob tab.
* **Idempotent re-push**: a manifest of tab + feature ids is stored with
  the project, so pushing again finds those tabs and updates them **in
  place — never creating duplicates**. If the manifest is lost (a
  teammate's fresh machine), it falls back to matching tabs by name.
  Re-pushing after a redesign just moves the existing geometry.

**v1.11.3** (after live debugging): the custom feature is now
**self-contained** — MICKSUS embeds the coordinates in the feature so it
builds geometry with no need to insert the Variable Studio into each Part
Studio (a real Onshape gotcha), while still honoring a live #variable
override. Beyond points and sketch planes it now emits, per axle: the
**lower/upper A-arm planes**, the **2D design sketch plane**, the
**halfshaft inner/outer CV points**, and a full **construction-line
skeleton** (lower/upper bushing axes, kingpin axis, both legs of each
A-arm, tie rod, shock, halfshaft) as named reference wire lines.

**v1.12.3** enforces a **knuckle-consistency convention** (always on) so
the tire, CV and steering geometry are buildable: the **tire spin axis is
kept in the plane of the two ball joints and the wheel centre** (so the CV
output line is coincident with the tire centreline, not cocked at an angle
— the root of the "wheel angle from kingpin" issue), and the
**steering-arm plane sits at a set angle to that tire plane** (a new
`Steering-arm plane angle` setup knob, default 90°). It's applied by
sliding the wheel centre the minimum **fore/aft** distance onto the tire
plane — camber, toe, track and ride height are all preserved (scrub moves
<1 mm) — and swinging the tie-rod outer to the target angle (tie-rod inner
fixed, so static toe is unchanged). Idempotent, so opening an older `.MICK`
snaps it consistent on load with no other change.

**v1.12.2**: the Readouts fly-out now reads **track width live from the
displayed geometry** (measured between the tire contact patches, so it
breathes with the wheels through travel — consistent with half-track
change) instead of echoing the setup input; wheelbase and ride height sit
alongside it. The Tweaks panel's arm/stroke hints are restyled as muted
captions so they no longer read as a stray line.

**v1.12.1** fixes the **outer-CV geometry**: the joint (and its 3D
tire-axis stub) now sits **on the tire centreline** — the wheel-spin axis
through the wheel centre, at the point nearest the kingpin — so the CV
output line is **coincident with the tire centreline** instead of cocked
at a few-degree angle to it (a CV angled to the wheel is buildable but out
of scope). Both the steering link and the CV link then swing rigidly with
the knuckle about the kingpin, staying coincident through steer and
travel. The Readouts fly-out also gains **wheelbase** and **track width**
next to ride height.

**v1.12.0** adds tuning and measurement the team asked for:

* **Hub offset tweak** = the **outer-CV → tire-centre distance** (the
  hub/knuckle stickout, the number hub and CV-cup packaging is built
  around). Sliding it moves the wheel along its spin axis — scrub radius
  and half-track change, camber/caster/KPI do not. (The seed's separate
  "Hub offset (WC→BJ plane)" still seeds the lateral ball-joint position.)
* **Shock length @ ride tweak** with a live **bump/droop stroke-split**
  readout, so you can dial the installed length to hit a target travel
  split.
* **Ride height tweak** — a rigid vertical placement (no kinematic
  change), driven by a new **frame-tube offset** seed variable (default
  5⁄8 in) that sets the ride-height datum above the CAD origin; adjust it
  for a non-Baja frame. The seed X/Z offsets are relabelled as the
  initial-placement knobs they always were.
* **Plot hover readout** — mouse over any graph to see the exact metric
  value at that travel instead of eyeballing it.
* **Free (L/R) travel mode** — a second slider gives the right wheel its
  own travel, so you can hold one wheel at ride height and bump the other
  (single-wheel-bump roll centre) and read the asymmetric roll centre.
* **Export menu trimmed** to the essentials (the redundant older Onshape
  push variants are gone; Build/update CAD supersedes them).

The **anti-squat/anti-dive** percentages were also re-verified end to end
against a real exact-CAD front corner: they read tens of percent (≈ −29 %
front anti-dive on that car), not near-zero — the math tracks the
side-view geometry correctly, so a near-zero reading points to an
out-of-date build.

**v1.11.5**: a **caster/kingpin setting**. A kingpin lying in the
kickup-tilted 2D sketch necessarily has caster ≈ kickup, so by default
MICKSUS keeps the kingpin **planar** (seeds and Re-square both build a
twist-free corner; caster follows the kickup). Tick **"Independent
caster"** in Setup to instead honor your desired-caster value and allow
the kingpin to sit off the sketch (Re-square then leaves it alone).

**v1.11.4** fixes two live-found issues: (1) a re-push now **updates the
existing feature in place** — it finds the hardpoints feature by type in
the Part Studio (even if the manifest was lost) and updates it, so entity
ids and all your downstream references survive instead of a duplicate
feature being pasted in; (2) **Re-square now also pulls the kingpin into
the 2D sketch plane** (`planarize_kingpin`) — the knuckle-twist you'd
otherwise see in side view (2.7° front / 14° rear on the test car) goes to
zero, and the caster change it implies is reported.

Status: the orchestration and idempotency are unit-tested against a
stateful mock document. Live push validated through geometry generation;
format strings are centralised so any remaining fixes are one-liners.

## v1.10: toe-by-tie-rod tweak + passive-steer optimizer goal

* **Static toe by tie rod** — a new DW tweak sets static toe the way a
  mechanic does: threading the tie rod rotates the whole knuckle about
  the kingpin, so the wheel, hub and tie-rod knuckle point swing
  together, the link length re-derives, and the leaned kingpin nudges
  camber (the stored alignment tracks both). Unlike typing static toe
  (which only re-aims the spindle), this keeps the 3D geometry
  consistent with the number.
* **Passive-rear-steer optimizer goal** — the new "Toe vs travel" goal
  targets a toe SLOPE through travel instead of flat bump steer: a
  fixed-tie-rod rear can be tuned to toe out in bump and in during
  droop (target negative deg/mm) for passive rear steering. The
  classic "bump steer → 0" goal is unchanged.

## v1.9: MICKSUS + twisted sketch plane + sketch-dimension tweaks

* **The tool is now MICKSUS** — splash screen, window icon, dark theme
  by default, and a **blank boot**: no surprise default car. Load your
  frame, set the Setup values, Generate seed. `File → New` resets to
  the same blank state.
* **Twisted sketch plane** — a per-axle **Sketch plane yaw** in Setup
  for frames whose suspension-mount tubes run at an angle. Seeds
  generate at that angle (knuckle/wheel still point straight; the
  wheelbase breathes a few mm through travel — real and fine), the
  squareness metric/checklist judge deviation FROM the design angle,
  and **Re-square rotates existing geometry onto it**. Verified on the
  user's file: his rear tubes measure +20.1°; re-squaring at that angle
  moves the bushings ≤5 mm (vs 124 mm when forcing them flat).
* **Sketch-dimension tweaks** — kingpin overall length (UBJ slides
  along the axis; lower arm and wheel centre stay), the **direct gap
  between the UCA and LCA inboard pivot axes** (upper pair translates
  along the common perpendicular), and each arm's **length as
  dimensioned on the sketch** (BJ slides in-plane, caster offset kept)
  — the camber-curve dials. Move-and-flag: tweaks always apply; travel
  auto-clamps and the checklist flags anything broken.
* **Panel font size** (Panels menu, persists) so the full hardpoint
  table fits on screen; dense tables get alternating row colors.
  (v1.9.1: the X/Y/Z columns share the panel width — no more sideways
  scrolling — with taller, comfier rows and a 9 pt default.)
* **In-app guide grew up** — Help/F1 now opens a TOC'd guide with a
  step-by-step getting-started walkthrough and a "designing a Baja
  suspension" engineering section (target numbers + which knob moves
  each metric).

## v1.8: orbit "blue dot" fix + tire-axis CV + squareness + steering checks

* **Camera no longer orbits the blue dot** — the far-off point was the
  side-view instant-centre marker (legitimately tens of metres out when
  the side-view arms run near-parallel). Orbit recentering, Fit view
  and first-build framing now use MODEL bounds that exclude the
  IC/SVIC/RC/roll-axis overlays; IC markers farther than 4 m are hidden
  and the IC direction ray is clamped to that length.
* **Outer CV on the tire's centre axis** — the outer joint now sits at
  the skew-line closest point of the kingpin axis to the WHEEL SPIN
  axis (on Ethan's file the old wheel-centre projection was 30–37 mm
  off the tire axis; the two axes pass within ~2 mm, a true
  intersection). A thin violet stub from the CV to the wheel centre
  makes the on-axis placement visible.
* **Bushing-axis yaw metric + repair** — a one-sketch corner's bushing
  axes must lie in the side-view plane (kickup only). `sketch_planarity`
  now reports `axis_yaw_deg`; the checklist flags yaw > 0.25°, and
  Re-square removes the yaw (kickup, midpoints, spread lengths kept;
  ball joints — i.e. your caster — untouched).
* **Steering-arm tweak** — new DW tweak: signed distance from the
  kingpin axis to the tie-rod outer along the sketch normal. Slides
  tierod_outer along the normal (exact affine solve); the rack end
  stays put and static toe is unaffected — only effort/ratio/max angle
  change.
* **Turning radius + Ackermann in the Design Check** — new thresholds
  (rack travel ± from centre, turning-radius goal) drive a live rack
  sweep at ride height: a pass/fail row for the outside-front-wheel
  turning radius at full lock, and an embedded Ackermann-% graph.
  Thresholds persist in the project file.

## v1.7: halfshaft overhaul + orbit self-heal + re-square button

* **Tangle fixed** — enabling a halfshaft on an Onshape-offset design
  used to draw the shaft from the PRE-offset default inner position, a
  metre off-axle, slashing across the whole linkage ("tangled mess").
  Enabling now auto-places a stale inner at the axle station, level at
  mid-travel; `inner_is_stale()` guards it.
* **Outer CV on the kingpin axis** — the outer joint now sits at the
  closest point of the LBJ→UBJ axis to the wheel centre (where it
  physically lives in the hub), rides rigidly with the upright, and is
  used for all CV-angle/plunge math. Kingpin-less types fall back to
  the wheel centre.
* **Both CV joints in the hardpoint table** — two extra rows when the
  axle has a halfshaft. The inner is a free chassis point (fix it to
  the gearbox flange the powertrain team gives you); editing the OUTER
  projects the request onto the kingpin axis and slides the hub — and
  therefore the tire height — with it (undo-able).
* **Hub/CV-along-kingpin tweak** — new spin in the DW tweaks: signed
  position of the hub along the kingpin from the ball-joint midpoint;
  the wheel and outer CV translate together, knuckle angles unchanged.
* **Inner CV is draggable** in the 3D view (violet marker), with the
  same axis locks and optional frame snapping as any hardpoint.
* **Camera orbit self-heals** — the rotation pivot is re-anchored onto
  the model after EVERY orbit/pan/zoom (VTK end-interaction hook), plus
  a "Recenter orbit" rescue button next to Fit view.
* **"Re-square arms" button** in the Optimize panel: fixes one-sketch
  bushing-axis drift on demand, no goals run, undo-able.

## v1.6: frame auto-align + visible seed-phase halfshafts + both-axle heave

* **Frame imports auto-align** — a freshly loaded mesh gets its scale
  guessed AND is rotated onto the chassis convention automatically, so
  with the default seed offsets the design lands ON the frame with zero
  manual transforms (verified against the user's EthanTest.MICK: most
  chassis-side mounts sit 2–36 mm off the frame surface — welded-tab
  distance). Moving the frame off the canonical alignment raises a
  warning: the backdrop marks where the real chassis is; fix the
  suspension, not the picture.
* **Halfshafts are visible and seed-phase** — the CV axle now draws in
  the 3D view (violet line, inner joint to wheel centre, both sides).
  "Driven axle: model halfshaft" in Setup enables it at seed time with
  the inner joint auto-placed level at mid-travel (minimum plunge); the
  Halfshafts panel remains for fine-tuning. Works for the DW seed and
  the trailing-arm/multilink re-seed path.
* **"Both axles" heave** — checkbox by the travel slider moves front and
  rear together, each clamped to its own range.

## v1.5: workflow revamp + direct manipulation + candidates + Onshape design sync

* **Workflow** — the intended loop is now enforced by the tool itself:
  Seed → place mounts → **Checklist green** → Optimize → **snapshot a
  Candidate** → export/sync. Edits that shrink the reachable travel
  AUTO-CLAMP the swept range (requested range remembered; the checklist
  shows the shortfall) instead of being rejected; only geometry that
  barely articulates is refused. A debounced rolling autosave (File →
  Restore autosave) makes bad edits non-fatal.
* **Design Checklist dock** — live pass/fail per axle: articulation vs
  request, one-sketch arms, static bump steer under threshold,
  roll-centre height inside a band, halfshaft CV within limits.
  Thresholds editable in the panel; tooltips carry the numbers.
* **Drag hardpoints in the 3D view** (OptimumKinematics-style): click a
  point sphere, drag in the view plane, hold X/Y/Z to lock that axis of
  the ACTIVE display convention; live re-solve during the drag, full
  pipeline + undo on release, Esc cancels. Optional **snap-to-frame**
  (off by default — real mounts are welded tabs offset from tubes)
  pulls released points onto the frame-mesh SURFACE.
* **Candidates dock** — named whole-vehicle snapshots with frozen metric
  columns (RC heights, camber gain, bump steer, caster, travel, worst
  CV); rename in place, load to hop between designs; saved in the .MICK.
* **Onshape hybrid sync** — File → "Sync design to Onshape document"
  stores the full project JSON as an app element next to the Variable
  Studio; "Open design from Onshape document" pulls it back anywhere.
  Mock-tested end-to-end (live validation still pending API keys); a
  read-only in-Onshape viewer tab is the planned phase 2.

## v1.4: solver branch fix + chassis coords + halfshafts

* **Links-crossing bug fixed (critical, reported with EthanTest.MICK)**
  — the UBJ closure `A cos b + B sin b = C` has two roots: the real
  assembly and the upright folded over. The solver used to pick "the
  root nearest b = 0" at every pose, which silently teleported onto the
  folded branch at big travels (tie rod jumped ~170 mm between
  neighbouring slider steps, links visibly crossed). The assembly
  branch is a build-time property of the mechanism, so it is now chosen
  once from the static pose and held forever; the user's exact file now
  articulates smoothly through its whole declared range (regression
  test `tests/test_branch_stability.py` pins it).
* **Chassis coordinate convention** — toolbar "Coords" switch presents
  the table, CSV export, Onshape variables and CG readout in the
  team chassis frame convention (fore-aft on Y, lateral on X; both nose
  signs offered). Internal math/solvers/files stay in the tool frame;
  `axes.py` holds the orthonormal mappings.
* **Halfshafts** — per-axle CV axle model (inner joint at the gearbox
  flange, outer at the wheel centre, any suspension type): CV angle
  (worst joint) and plunge tracked over the sweep as graph channels,
  with user-set datasheet limits raising warnings (not hard stops).
* **Offset-aware scene** — ground plane, centreline, roll axis, CG and
  iso camera anchor to the axles' true stations instead of x = 0 (the
  "uncentered frame" report).
* **Roll mode centred** — the roll slider clamps to ±min(droop, bump)
  so both sides articulate symmetrically (one side used to freeze when
  it ran out of droop); "Ride height" and "Reset pose" buttons added.
* **Scroll-wheel edits fully blocked** on spinboxes/combos (focused or
  not), ghost overlay has a visibility toggle, `examples/` ships three
  frame-referenced example projects with a README.

## v1.3: dynamics panel + workflow feedback round

* **Dynamics panel** — a 1:1 live port of the team's Milliken & Milliken
  (RCVD) reference spreadsheet (`reference/suspension_design_tool_v2.xlsx`,
  sheets Vehicle/Setup/Loads/Dynamics): mass & CG & sprung/unsprung split,
  springs (dual-rate stacks) → wheel rates → ride rates → ride/bounce/
  pitch frequencies + Olley flat-ride check, damper ratios, ARB + spring +
  tire roll rates, roll gradient, lateral & longitudinal weight transfer,
  TLLTD, per-corner loads with lift-off warning, the full brake force
  chain, understeer gradient, and auto tuning notes. Imperial units
  (lb/in/lb-ft/deg) matching the reference formulas; `dynamics.py` is
  unit-tested against hand-keyed spreadsheet arithmetic. Default
  *kinematics mode* shows only the geometry-relevant numbers; *Full
  dynamics mode* shows everything. *Link kinematics* streams the live RC
  heights, CG, wheelbase, motion ratios (converted wheel/shock →
  spring/wheel) and the EXACT kinematic anti-percentages into it.
* **Dockable panels** — every design panel is a CAD-style dock: drag,
  float (second monitor), tab, close, re-open from the Panels menu.
* **Onshape-aligned coordinates** — new seeds shift by a user-set X/Z offset
  (user-editable in Setup) so table coordinates paste straight onto the
  team's frame origin; the ground sits at −ride-height and every
  ground-referenced metric now uses the tire contact plane instead of
  assuming z = 0 (rigid-shift invariance is unit-tested).
* **One-sketch planarity guard** — live "Sketch fit" readout under the
  hardpoint table (the UCA/LCA bushing axes must stay parallel for both
  arms to share one 2D sketch with sketch-normal bushing tubes), and the
  optimizer auto-re-squares drifted axes on entry (midpoints and spread
  lengths preserved — the minimal repair).
* **Camera fixes** — editing values no longer moves the camera (pose is
  preserved across scene rebuilds; only Fit view / view buttons reframe),
  and the orbit pivot is re-anchored onto the model every rebuild so
  rotation never orbits a stale point off in space.
* **Physical steering display** — the mirrored right wheel is now solved
  at the opposite rack sign before mirroring, so both wheels visibly
  steer into the turn like the real car.
* **CG marker** — toggleable yellow sphere at the CG with its
  coordinates shown in the Vehicle panel.

## Coordinate convention

Used everywhere in code, tests, and (later) the UI:

- **+X = forward**, **+Y = left**, **+Z = up** (right-handed)
- Origin at **ground level**, on the **vehicle centreline**, at the
  X-station of the axle being modelled
- Units: **millimetres** and **degrees**
- We model the **left corner**, so hardpoints have y > 0
- **Travel**: wheel-centre height relative to static; positive = bump
  (wheel up), negative = droop

Sign conventions for the metrics (left corner): camber negative = top of
tire leans inboard; toe positive = toe-in; caster positive = kingpin top
leans rearward; KPI positive = kingpin top leans inboard; scrub radius
positive = contact patch outboard of the kingpin ground-pierce point;
caster trail positive = contact patch behind it. Motion ratio = **wheel
travel / shock travel** (the team convention — a Baja value reads above 1).

## How the solver works (Phase 1)

`suspension_tool/solver.py` solves one double-wishbone corner through
vertical wheel travel. The mechanism has one degree of freedom (steering
fixed), parameterised by the lower-control-arm rotation angle:

1. **LBJ** — rotate the static lower ball joint about the LCA inner-pivot
   axis (Rodrigues' formula).
2. **UBJ** — circle ∩ sphere, **closed form**: on the circle swept about
   the UCA axis *and* at the rigid kingpin distance from the LBJ
   (reduces to `A·cos b + B·sin b = C`).
3. **Tie-rod outer** — three-sphere **trilateration**: rigid distances
   from LBJ and UBJ (upright) plus the fixed tie-rod length from the
   chassis-side inner end.
4. **Upright pose** — exact rigid-body fit (Kabsch/SVD) through the three
   solved points carries the wheel centre and spindle axis along.

The only iteration is a bracketed 1D root find (`scipy.optimize.brentq`)
for the LCA angle that hits a requested wheel travel.

`suspension_tool/metrics.py` computes camber, toe, caster, KPI, scrub
radius, caster trail, roll centre (classic front-view instant-centre
construction), motion ratio, and bump steer at each travel position.

## Seed generator (Phase 2)

`suspension_tool/seed.py` turns the setup variables (track width, ride
height, tire size, shock min/max/at-ride lengths, motion-ratio goal, hub
offset, desired scrub/caster, static camber/toe) into a valid starting
hardpoint set, by rules of thumb plus two tiny 1D secant tunes: the shock
mount slides along the LCA until the static motion ratio hits the goal,
and the tie-rod inner height is nudged until static bump steer is ~zero.
The generator refuses to return a seed that can't articulate through the
full shock stroke.

It was validated during development against a real, fully detailed front
corner — its motion ratio, shock hardware, track and static camber taken
from CAD — and reproduces the known camber trend (more negative in bump,
positive in droop). Those vehicle numbers are not distributed with this
demo; the tests exercise the same behaviour on generic geometry. See
`tests/test_seed.py`.

### Chassis kickup — bushing axes normal to the sketch plane (Phase 3.1 fix)

The reference front suspension is designed on a 2D sketch (the front view) taken
at the chassis **kickup angle** (~10°). The hard requirement from the team:
**each control arm's front→rear bushing line must be NORMAL to that sketch
plane**, so the linkages are clean projections off the bushings and the
suspension follows the 2D sketch path — which keeps spurious **caster gain**
off the knuckle (a double-wishbone wants none).

So the seed generator lays the whole corner out on the front-view sketch,
then spreads **only the four inboard bushings** along the sketch-plane
normal `(cos k, 0, sin k)` (front end up). The ball joints, knuckle, tie
rod, and shock stay on the sketch — they are **not** rotated. Consequences:

- each bushing axis is exactly normal to the sketch plane (tilted by the
  kickup, front mount higher); `kickup_deg` is an editable setup variable.
- the **knuckle keeps its design caster** (~4°, not inflated) — the kickup
  tilts bushings, not the kingpin. Static caster will read offset from the
  real-chassis value by the kickup, which the designers account for.
- the wheel stays exactly half-track out and one tire radius up.

This replaced an earlier "rotate the whole corner" attempt that rotated the
ball joints too, which pushed the knuckle off the 2D path and inflated
caster — exactly what the Phase 3.1 video feedback flagged. The solver
needed no change: it builds each arm's axis from the actual front/rear
bushing positions, so a tilted axis just works.

**Both inboard axes are exactly parallel** (both equal to the sketch-plane
normal — verified numerically in `tests/test_seed.py`). If they ever *look*
skewed in the 3D view that is perspective, not the model: the viewport now
defaults to **parallel (orthographic) projection** and offers **Front /
Side / Top / Iso** view buttons so parallel geometry stays parallel on
screen.

**Inboard separation** between the upper and lower axes is set by
`inboard_sep_frac` (vertical gap as a fraction of the kingpin's vertical
extent, default 0.4). It is made roomy for frame mounting but kept well
under 1.0 — past ~0.85 the front-view instant centre crosses over and the
**camber curve reverses**, so the fraction is clamped to 0.8. Raising the
inboard axes shortens the reachable bump slightly, so the seed now reports
the travel it can actually **articulate** (the `MR × stroke` budget is
optimistic because MR drops in bump) instead of rejecting an otherwise good
seed.

### Display units and project files (Phase 3 feedback)

- **Units:** a project-wide unit selector (mm / inch, default **inch**) in
  the top toolbar. Everything is stored internally in mm; only the display
  converts, so switching units never changes the geometry. Angles stay in
  degrees; rates show as deg/in or deg/mm.
- **Save/Load:** the File menu reads and writes **`.MICK`** project files
  (JSON under the hood) holding the setup variables, hardpoints, display
  unit, travel range, and a slot for Phase 4 goals. See
  `suspension_tool/project.py` and `tests/test_project.py`.

## Run it

```bash
pip install -r requirements.txt

# verification suite (hand-calculated / cross-checked / validated on a real car)
python -m unittest

# metric tables: example corner + a generated seed
python -m suspension_tool.demo

# the GUI (Phase 3)
python -m suspension_tool.gui
```

## Synthesis / optimization (Phase 4)

`suspension_tool/optimize.py` refines the current hardpoints toward
user-selected kinematic goals with `scipy.optimize.least_squares`. In the
GUI ("Optimization goals" panel): tick the goals you care about, set each
target and weight, tick which hardpoint groups are **free** to move
(everything else stays fixed), set the search range, and hit **Run
optimization**.

Goals: camber-gain curve, bump steer → 0, roll-centre height, RC
migration → 0, motion ratio, caster, scrub radius, KPI, caster trail.
(Static camber/toe are direct inputs to the model, so they are set — not
optimized.)

Design decisions that follow the brief and the team's feedback:

- **Limited free set, from the seed.** You choose a few free groups; the
  optimizer never moves everything at once.
- **The sketch-plane constraint is baked into the parameterisation**: the
  inboard bushing pairs move only via their axis *midpoint*, carrying the
  front/rear spread vector verbatim — so the axes stay exactly normal to
  the 2D sketch plane and parallel to each other no matter what the
  optimizer does.
- **Packaging bounds**: every free coordinate is boxed around its start
  value and clipped inside the wheel / above the ground. Candidates that
  cannot articulate the travel range are rejected via a penalty, and a
  result that isn't an improvement is discarded (geometry unchanged).
- **Before/after**: after a run, the plots overlay the previous curves
  (dashed) and the 3D view shows the previous linkage as a translucent
  ghost. Goal settings are saved in the `.MICK` project file.

For optimizer speed the solver gained `walk_travels()`, a warm-start sweep
that walks outward from static re-using the previous position's arm angle
(≈3× faster per candidate than the bracketed global solve, verified
identical to 1e-9).

## Frame backdrop + packaging (Phase 6)

**Frame backdrop:** the Frame panel loads an **STL or 3MF** of the
chassis (`suspension_tool/frame.py`, via trimesh) and draws it as a
static translucent backdrop behind the suspension. Because CAD exports
rarely match our vehicle frame, it carries a transform: uniform **scale**
(auto-guessed from the mesh size — a chassis 3MF in metres detects
as ×1000), **Z-rotation** in 90° steps, and an XYZ **offset** in display
units. The path + transform save into the `.MICK` project. Verified
against a real 75k-vertex chassis export.

**Save/load** has existed since Phase 3.1 (`.MICK`, now format v2 with
both axles + vehicle + frame).

**Windows executable:** PyInstaller cannot cross-compile, so the exact
build command must run ON Windows:

```bash
pip install -r requirements.txt pyinstaller
pyinstaller --noconfirm suspension_tool.spec
```

Output: `dist/BajaSuspensionTool/BajaSuspensionTool.exe` (a one-folder
app — zip the folder to share). The spec was validated by building and
launching the packaged app on Linux (same spec, same code paths), and
`.github/workflows/build-windows-exe.yml` runs the identical build on a
**Windows** GitHub runner: repo → Actions → "Build Windows exe" → Run
workflow → download the `BajaSuspensionTool-windows` artifact. The
workflow also runs the test suite and smoke-launches the exe before
uploading.

## Usability + roll visualization (Phase 5.5)

- **Roll mode** (dropdown next to ▶ Play): the travel slider becomes the
  LEFT wheel's travel — the right side goes the other way and the other
  axle rolls to the same chassis angle. With "Mirror right side" on you
  watch the whole car roll, the roll centre migrate off the centreline
  (asymmetric left/right construction), and the roll axis move. Play
  animates it.
- **Scroll-wheel guard**: spinboxes and dropdowns only respond to the
  wheel after you CLICK into them — no more accidental design edits
  while scrolling the panel.
- **Fit view** button, **Undo** (toolbar + Ctrl+Z, 15 levels per axle),
  and a **Help → User guide** (F1) that explains every panel, the three
  suspension types (including how to think about the multilink's virtual
  ball joints), sign conventions, and tips. Multilink/trailing-arm table
  rows now carry human labels ("Upper link A — chassis" instead of
  "link1 inner").
- **Anti-squat / anti-dive curves**: two new graph panels (and report
  columns) showing the percentages through travel for the active axle,
  live against the Vehicle panel's CG and brake split.
- Frame import now explains the fix when `trimesh` is missing (re-run
  `pip install -r requirements.txt`), and the exe build bundles trimesh's
  dynamic loaders.

## Virtual kingpin, per-type tuning, bump-steer map (Phase 5.4)

**Multilink virtual kingpin**: caster / KPI / scrub / caster trail now
work for multilinks via the standard virtual-ball-joint construction —
the closest point between the extended upper link pair (1 & 2) and lower
pair (3 & 4) define the steering axis. For a double wishbone expressed as
five links the pairs intersect AT the real ball joints, so the virtual
axis reproduces the physical kingpin to machine precision (the test
asserts 1e-6°); it stays finite for genuinely skewed link sets.

**Per-type tweaks**: the tweaks panel re-shapes with the suspension type —
trailing arms get the shock decomposition on the arm line plus the
**pivot-axis plan/elevation skew** (the semi-trailing knobs that dial
camber/toe gain); multilinks get the **toe-link end rises** (the
bump-steer knobs). **Per-type optimization**: the free-hardpoint list
follows the type (pivots for trailing arms, ten link ends for
multilinks), goals undefined for a type (caster on a kingpin-less
trailing arm) error clearly, and the tests repair a de-tuned multilink's
bump steer and hit a trailing-arm camber-gain target.

**Bump-steer map** (Sweeps ▾): left-wheel toe over the full travel × rack
grid, as a heatmap plus toe-vs-travel curves at fixed rack positions —
whether your bump steer stays acceptable *at steer*, not just on-centre.

## Per-type editing + anti-geometry + comparison + reports (Phase 5.3)

The **Suspension type** dropdown now actually switches the ACTIVE axle:
picking Trailing arm or Multilink re-seeds that axle from the current
setup values (trailing arm: lateral pivot axis at rail height, shock
MR-tuned; multilink: the 5-link equivalent of the double-wishbone seed —
start moving individual rod ends). The hardpoint table re-shapes to the
type's own points, the 3D view draws the arm/links, all metrics and
sweeps work (kingpin channels read blank where undefined), and mixed
front/rear types save/load in the `.MICK` file and export to CSV/Onshape.
The kinematic tweaks panel and the optimizer remain double-wishbone-only
for now and say so.

**Anti-dive / anti-lift / anti-squat %** (Vehicle panel + report): pure
side-view geometry from each axle's support line (contact patch for the
outboard brakes, wheel centre for the inboard rear drive) against the
load-transfer angle — needs only the CG inputs and the new front brake
fraction. Exact for trailing arms (IC = pivot), numeric-fallback for
parallel-axis double wishbones (hand-verified: a lateral rear pivot 28 mm
above and 350 mm ahead of the wheel gives 22.2 % anti-squat at a
representative CG height).

**Design comparison** (Compare ▾): pin up to four named snapshots of the
current curves and overlay them against the live design as you tweak.

**Report export** (File → Export report CSV): the static design table
(all channels, front vs rear), the vehicle numbers (roll axis, moment
arm, anti percentages), and the full sweep curves per axle in one
Excel-friendly file, in the current display units.

## Motion sweeps + all three suspension types (Phase 5.2)

**Sweeps ▾** (above the 3D view) opens three non-modal analysis dialogs
(`suspension_tool/sweeps.py`), re-runnable as you tweak geometry:

- **Roll sweep** (opposite wheel travel): ground-frame camber and toe
  (roll steer) for BOTH wheels vs roll angle, plus roll-centre height
  AND lateral migration computed from the true asymmetric left/right
  construction. Positive roll = body leaning onto the left wheels.
- **Pitch sweep** (needs both axles): front/rear travels coupled about
  the CG station; ground-frame front caster, camber both ends, and
  wheelbase change vs pitch angle.
- **Steering sweep**: wheel steer angles both sides across the rack
  limit, **percent Ackermann**, caster-induced camber with steer, and a
  low-speed outside-turn-diameter estimate.

**Trailing arm** (`trailing_arm.py`) and **multilink**
(`multilink.py`) kinematics are now fully implemented at the solver
level. The trailing arm is closed form with EXACT instant centres (the
pivot axis pierced through each view plane) and covers pure and
semi-trailing axes; the multilink solves the 5-link closure numerically
and is validated by reproducing the double-wishbone solver's results
when fed a double wishbone expressed as five links (to 1e-5 deg). Both
run through the same metrics pipeline (kingpin-less types read NaN for
caster/KPI/scrub/trail; roll centres come from a numeric Adams-style
instant-centre method). They are usable today via the Python API and
tests; their GUI editing panels (seed/table/tweaks per type) are the
next milestone — the UI editor stays double-wishbone for now.

## Whole-vehicle mode (Phase 5.1)

A project now holds **both axles**. The toolbar's **Editing** switch
flips every panel (setup, hardpoints, tweaks, optimizer, readouts, plots,
travel slider) between the front and rear designs — switching to an
un-designed axle auto-seeds it from the current setup values. The 3D
view can display **front, rear, or both** (rear drawn at `-wheelbase`,
green arms), each with an optional **mirrored right side**, and shows
per-axle **front-view ICs, side-view ICs** (finite only when the arm
axes are skewed for anti-geometry — by design they're parallel in seeds,
IC at infinity), **roll centres**, and the **roll axis** joining them.

The **Vehicle panel** takes wheelbase and the overall CG (height +
distance behind the front axle) and derives, live at the current travel:
roll-axis angle, roll-axis height under the CG, the **roll moment arm**,
and the static weight split. All of it saves into the `.MICK` file
(format v2; v1 single-corner files still open as the front axle).

A **front rack steer slider** (below the travel slider) turns the front
rack within a user-set ± limit that mimics the physical rack stops; the
3D pose and readouts update live (plots stay at zero steer).

CAD handoff scales up too: with both axles present the CSV export/copy
becomes a **vehicle file** (front_/rear_ prefixes, rear shifted by the
wheelbase), and **File → Push hardpoints to Onshape…** writes every
coordinate into a named **Variable Studio** via the REST API (free API
keys from dev-portal.onshape.com) — created if missing, updated in place
otherwise, so variable-driven skeleton sketches rebuild with zero clicks.
See `suspension_tool/onshape_sync.py` and `onshape/README.md`.

## Suspension types + steering (Phase 5)

`suspension_tool/suspension_types.py` defines the layout interface from
the brief — `solve(travel, steer)` / `walk_travels(travels, steer)` — and
a registry that feeds the **Suspension type** dropdown in the toolbar.
Double wishbone is fully implemented; **Trailing arm** and **Multilink**
are declared stubs that explain themselves when selected. When they get
implemented, none of the metrics/plots/3D code needs to change.

As part of the interface, the double-wishbone solver gained real **rack
steer**: `solve(travel, steer_mm)` displaces the tie-rod inner point
laterally (+ = toward the modelled left corner) and re-solves the corner
at the same wheel height. Zero steer is bit-identical to before; tests
cover steering direction, tie-rod rigidity from the displaced rack, and
caster-induced camber with steer. This is the foundation for steering
sweeps / Ackermann analysis (see the research notes below).

## Kinematic tweaks panel (Phase 4.1)

Between the hardpoint table and the optimizer sits the **relative-
dimension tweaks** panel (`suspension_tool/tweaks.py`) — adjust the
dimensions designers actually think in, and every edit re-solves live
with the same reject-and-revert safety as raw hardpoint edits:

- **Shock mount along the arm** (from the inboard axis toward the
  knuckle; bigger = closer to the knuckle = lower motion ratio),
- **Shock mount height off the arm line** and **mount angle** (0° = tab
  straight up, ± tilts it fore/aft OFF the 2D sketch plane — real tabs
  are moved for clearance and twisted when bump/droop isn't 50/50),
- **Tie-rod outer rise above the LBJ** and **inner rise above the LCA
  inboard axis** — the classic bump-steer honing dimensions.

New setup variables: **kingpin length** (LBJ–UBJ span, centred on the
wheel centre; "auto" = 0.9 × tire radius), **inboard axis gap** (direct
mm/in value on the sketch plane, overriding the fraction), and **shock on
upper arm** (the solver rotates the outer mount with whichever arm
carries it).

New display features: **RC/IC markers** in the 3D view (front-view
instant centre, roll centre on the centreline, and the contact-patch→IC
construction line — toggleable), a **live ground plane** that rises/falls
with travel plus a **ride height readout**, a **Graphs ▾ menu** to pick
which metric panels show (the grid re-flows), **dark mode**, and two new
industry-standard channels: **half-track change** (lateral tire scrub
through travel) and **wheel recession** (fore-aft wheel-centre walk).

## CAD handoff: CSV + Onshape (Phase 4.1)

`File → Export hardpoints CSV…` writes a named `name,x,y,z` table in the
current display units; `File → Copy hardpoints CSV (for Onshape)` puts
the same text on the clipboard. The [`onshape/`](onshape/) folder ships a
**"Baja Hardpoints" FeatureScript** — paste the CSV into one custom
feature at the top of your part studio and it generates named points
(and optional mate connectors, optional mirrored right side); re-paste
after a design change and everything built on the points rebuilds. See
[`onshape/README.md`](onshape/README.md), including the roadmap for a
zero-click REST-API push via Onshape Variable Studios.

## GUI (Phase 3)

`python -m suspension_tool.gui` opens the tool: setup variables +
editable hardpoint table + live readouts on the left, the 3D corner
(arms, upright, tie rod, shock, wheel, ground) top-right, and the six
metric-vs-travel plots bottom-right. The slider under the viewport
re-solves the corner live, ▶ Play animates it through full travel, and
**editing any hardpoint cell re-solves everything immediately** — edits
that make the linkage impossible are rejected with an explanation in the
status bar and the table reverts. It starts on a generated seed for the
the default setup values. Above the viewport, **Front / Side / Top / Iso** buttons
and a **Parallel projection** toggle (orthographic by default) let you check
geometry without perspective distortion. The top toolbar switches display
units project-wide and the File menu saves/loads `.MICK` projects. A
headless smoke test lives at
`tests/smoke_gui.py` (run `QT_QPA_PLATFORM=offscreen python -m
tests.smoke_gui`).

## Verification (Phase 1)

`tests/test_solver.py` checks the solver against hand calculations:

- **Parallelogram geometry** (equal parallel arms, parallel tie rod):
  camber and toe must stay exactly zero, the wheel centre must follow the
  hand-computed circular arc, and the roll centre must sit on the ground.
- **Static kingpin metrics**: caster, KPI, scrub radius, and caster trail
  computed by hand from a known kingpin line (e.g. scrub = 13 mm,
  trail = 18 mm) and matched to 1e-6.
- **Rigid-body invariants**: every fixed length (kingpin spacing, tie rod,
  upright spacings) stays constant through ±60 mm of travel.
- **Instant-centre cross-check**: the finite-difference camber rate matches
  `1/(y_IC − y_wheel)` from the independent front-view IC construction.
- **Bump-steer sanity**: a deliberately misplaced tie rod creates toe
  change; the well-placed one doesn't.
