# Claude Code Prompt: Suspension Kinematics & Synthesis Tool

I'm building a desktop tool to design and analyze suspension geometry for a Baja SAE car, similar to a stripped-down OptimumKinematics. The point is to let club members go from high-level setup goals to suspension hardpoints, then analyze and tweak them — without doing kinematics by hand or fighting CAD. It must run as a downloadable Windows executable.

The tool does two related jobs:
1. **Synthesis:** given setup variables and kinematic goals, suggest a good set of hardpoints.
2. **Analysis/tuning:** let the user edit those hardpoints and immediately see the effect via plots and animated motion.

**No tire forces / loads.** Pure kinematics and geometry only.

## Hard rules
- **KISS.** Build the simplest thing that works. No abstractions until needed.
- **Build incrementally and keep it runnable at every step.** Finish and verify one phase before the next. Do not scaffold all phases up front.
- **You cannot optimize what you cannot evaluate.** Build a correct forward kinematics solver FIRST. Optimization comes much later.
- Ask before adding any dependency beyond the stack below.

## Tech stack (fixed)
- Python 3.11+
- `numpy` + `scipy` (kinematics math AND optimization)
- `PySide6` for the GUI
- `pyvista` + `pyvistaqt` for the embedded 3D viewport and animation
- `matplotlib` (or `pyqtgraph`) for the metric-vs-travel plots
- `trimesh` (or `meshio`) to load frame mesh files
- `PyInstaller` to package a Windows exe at the end
- **Frame import = mesh only (STL or 3MF), NOT STEP.** Static visual backdrop. No CAD kernel / pythonocc.

## Setup variables the user provides
Max track width, wheelbase, ride height, tire size (radius/width), hub offsets, shock length, shock travel, motion-ratio goal. These define the packaging envelope and become the seed + the bounds for optimization.

## Kinematic goals (selectable targets, each with a weight)
Target camber curve through travel (e.g. desired camber gain), minimal bump steer (toe change vs travel), roll center height + limited roll center migration, motion ratio target, static camber/caster/toe, scrub radius, kingpin inclination (KPI), caster trail. The user picks which to target and how heavily.

## Phases

### Phase 1 — Forward double-wishbone solver (foundation)
- Hardpoints: upper inner front/rear pivots, lower inner front/rear pivots, upper + lower outer ball joints, tie-rod inner/outer, wheel center, shock inner/outer mounts, tire radius.
- Solve the corner through vertical wheel travel: each control arm rotates about its inner-pivot axis; the upright is rigid (fixed ball-joint spacing); the tie rod (fixed length) sets toe. Solve upright pose per ride height via geometric intersection or `scipy.optimize.fsolve`. State the method used.
- Compute **camber, caster, toe, bump steer, roll center, motion ratio** at each travel position.
- Verify against a hand-calculated test case before moving on.

### Phase 2 — Parametric seed generator
- From the setup variables, generate a sensible starting set of hardpoints (track width sets outer lateral positions, ride height + tire set vertical references, hub offsets set wheel center, shock length/travel + motion-ratio goal place the shock mounts, etc.).
- This is heuristic placement, NOT optimization yet. It just gives a reasonable, valid starting geometry.

### Phase 3 — Plots, animation, manual editing
- 3D viewport with hardpoints, links, upright, wheel, shock.
- Ride-height/travel slider that re-solves live; a "play" button that animates the corner through full travel.
- Plots of camber / caster / toe / roll center / motion ratio vs wheel travel.
- Let the user edit any hardpoint coordinate and see the 3D view and all plots update immediately. This manual-tuning loop is a primary feature.

### Phase 4 — Synthesis / optimization
- User selects goals (above) and weights, and selects which hardpoints are FREE to move; the rest stay fixed.
- Free hardpoints are bounded by the packaging envelope (track width, ride height, shock geometry, etc.).
- Use `scipy.optimize` (`least_squares` or `minimize`; `differential_evolution` if a global pass is needed) to minimize weighted error between achieved curves and target curves.
- **Do not optimize all hardpoints at once** — start from the Phase 2 seed and refine a limited free set. Reject geometry that violates packaging bounds or fails to solve.
- Show before/after geometry and overlaid before/after plots.

### Phase 5 — Multiple suspension types
- Refactor the solver into a small interface (base class: `solve(ride_height, steer)` -> upright pose) so layouts plug in. Double wishbone fully implemented; add stubs for trailing arm and multilink. Dropdown selects the active type.

### Phase 6 — Frame backdrop + save/load + package
- Load STL/3MF and render it static behind the geometry, with a position/scale offset so hardpoints line up.
- Save/load a full setup (variables, goals, hardpoints) as one JSON file.
- Package a standalone Windows .exe with PyInstaller; document the exact build command and confirm the exe launches and loads a mesh.

## Notes
- Define a clear coordinate convention (axes, origin) up front and show it in the UI.
- Comment the kinematics and optimization math heavily — this tool is meant to teach incoming members, so readable over clever.
- Plain UI: left panel for inputs/goals/readouts, 3D viewport top-right, plots bottom-right.

Start with Phase 1. Lay out the project structure, build and verify the forward double-wishbone solver against my test case, then stop and check with me before Phase 2. Ask anything you need before starting.
