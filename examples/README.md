# Example projects

Three ready-to-open `.MICK` projects, plus a frame mesh to sit behind
them. Open with **File → Open**, and load `demo_frame.stl` from the
Frame panel if it does not come up on its own.

Every full-size number here is **generic** — a plausible Baja SAE layout
built from round figures, not any team's actual car. The point is to
give you something that solves on the first click so you can go
straight to changing it.

| File | Front | Rear | The point of it |
|------|-------|------|-----------------|
| `baja_demo_double_wishbone.MICK` | DW, 60" track, 12" ride height, 8° kickup | DW, 59" track | The ordinary starting point: double wishbone both ends, ~+231/−139 mm of front travel. |
| `baja_demo_trailing_arm.MICK` | same DW front | Trailing arm, pivots on the lower rail | The classic Baja rear. Skew the pivot axis in **Tweaks** for semi-trailing behaviour. |
| `rc_buggy_1_10.MICK` | 1/10 scale buggy | — | 260 mm wheelbase, ~220 mm track, 37.5 mm tire radius. See below. |

## Why a 1/10 RC car is in here

It is the best test this tool has of **scale**. Nearly every default in
a suspension program is secretly a ratio of one full-size car — a
minimum track width, a "wheel is nearly unloaded" threshold in pounds, a
roll radius of gyration in inches — and none of them misbehave until you
hand the program a vehicle a tenth the size. Open the RC file and the
readouts, the checklist and the dynamics panel all have to stay
sensible. Several did not, once.

Its dynamics block is a real 1/10 buggy: 3.25 lb, 1.20/2.05 lb axle
split, 0.20/0.25 lb unsprung per corner, and tire vertical rates of
37.3/39.2 lb/in measured on a bench rig. Those tire rates give a
wheel-hop frequency near 46 Hz, which is what Froude-scaling a full-size
car's 13–15 Hz by √10 predicts — a useful sanity check that the
measurement and the model agree.

## Things worth noticing

**The rear halfshaft is modelled** on both Baja examples (45° outer /
25° inner CV, ±25 mm plunge). With the inner joint at wheel-centre
height, plunge over this much travel runs to **±50 mm — beyond any real
joint's limit**. The examples put the inner joint ~110 mm higher so the
shaft runs level near *mid-travel* rather than at ride height, which
cuts plunge dramatically. That trade — a little more static CV angle for
far less plunge — is exactly what the halfshaft panel exists to show.

**The front roll centre sits high**, around 337 mm (13.3 in), which
fails the Design Checklist's default 0–12 in band. That is not a broken
example: a car with 12 in of ride height genuinely lands there, and real
Baja cars measure in the same place. It is a good first thing to argue
with. Run **Optimize** with an `rc_height` goal — free the ball joints,
not the inboard bushings — and watch the before/after ghost.

**Caster comes out at 8°, equal to the kickup.** That is geometry, not
coincidence: with the kingpin lying in the one-sketch plane, caster
necessarily follows the kickup angle. Turn on `independent_caster` in
the setup if you want caster as its own knob.

## Regenerating

`python examples/make_examples.py` from the repo root rebuilds all of
it, including the frame mesh. Edit the constants at the top to move the
demo car around — that is why the generator ships rather than only the
files.

One convention worth knowing before you edit: `motion_ratio_goal` in
`SetupVariables` is **shock travel per unit wheel travel** (so ~0.55),
not the wheel-per-shock figure (~1.8) many teams quote. They are
reciprocals, and passing the wrong one quarters your travel budget
without any error message.
