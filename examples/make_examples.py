"""Build the example projects that ship with the public demo.

Every number here is GENERIC on purpose. The full-size car is a plausible
Baja SAE layout built from round figures, not any team's actual vehicle,
and the frame backdrop is a simple ladder generated below rather than a
real chassis export. Change them freely -- that is the point of having
the generator in the repo rather than only the .MICK files.

Run from the repo root:  python examples/make_examples.py
"""

import dataclasses
import json
import os

import numpy as np

from suspension_tool.geometry import sketch_planarity
from suspension_tool.halfshaft import HalfshaftConfig
from suspension_tool.metrics import corner_metrics, toe_deg
from suspension_tool.project import (AxleDesign, ProjectState, save_project)
from suspension_tool.seed import IN, SetupVariables, generate_seed
from suspension_tool.solver import DoubleWishboneSolver
from suspension_tool.trailing_arm import TrailingArmSolver, seed_trailing_arm

OUT = "examples"

# ----------------------------------------------------------------------
# A generic full-size Baja car. Round numbers, no team's geometry.
# ----------------------------------------------------------------------
BAJA = dict(track=60.0 * IN, wheelbase=60.0 * IN, ride_height=12.0 * IN,
            tire_radius=11.5 * IN, tire_width=7.0 * IN,
            shock_min=14.0 * IN, shock_max=22.0 * IN, shock_ride=19.0 * IN,
            frame_half_width=140.0)

# Tool-frame (Y, Z) of the inboard pickups the frame rails run through.
# Printed by this script so they can be re-measured after a change.
LOWER_RAIL = (140.0, -30.0)
UPPER_RAIL = (224.0, 75.0)

BAJA_VEHICLE = {"wheelbase": BAJA["wheelbase"], "cg_height": 20.0 * IN,
                "cg_behind_front": 0.55 * BAJA["wheelbase"],
                "brake_front_frac": 0.6}

# 600 lb with driver, 45/55 rear-biased -- typical, and deliberately not
# anyone's measured car.
BAJA_DYNAMICS = {
    "weight_empty_lb": 420.0, "driver_lb": 180.0,
    "front_axle_lb": 270.0, "rear_axle_lb": 330.0,
    "unsprung_front_lb": 30.0, "unsprung_rear_lb": 35.0,
    "tire_rate_front_lbin": 250.0, "tire_rate_rear_lbin": 250.0,
    "spring1_front_lbin": 150.0, "spring1_rear_lbin": 175.0,
    "stack_front": "Single", "stack_rear": "Single",
    "mr_damper_front": 0.55, "mr_damper_rear": 0.57,
    "damp_bump_front_lbsin": 12.0, "damp_rebound_front_lbsin": 16.0,
    "damp_bump_rear_lbsin": 12.0, "damp_rebound_rear_lbsin": 16.0,
    "cg_height_in": 20.0, "roll_radius_gyr_in": 18.0,
    "cornering_stiff_front_lbdeg": 150.0,
    "cornering_stiff_rear_lbdeg": 170.0,
    "ay_g": 1.0, "ax_g": 0.7, "speed_mph": 30.0,
}


def rezero_bump_steer(hp):
    """Secant on the tie-rod inner HEIGHT until static bump steer ~ 0."""
    def bs(z):
        cand = dataclasses.replace(
            hp, tierod_inner=np.array([hp.tierod_inner[0],
                                       hp.tierod_inner[1], z]))
        s = DoubleWishboneSolver(cand)
        return (toe_deg(s.solve(1.0)) - toe_deg(s.solve(-1.0))) / 2.0, cand
    z0, z1 = hp.tierod_inner[2], hp.tierod_inner[2] + 8.0
    f0, _ = bs(z0)
    f1, cand = bs(z1)
    for _ in range(14):
        if abs(f1) < 1e-6 or abs(f1 - f0) < 1e-15:
            break
        z0, z1, f0 = z1, z1 - f1 * (z1 - z0) / (f1 - f0), f1
        f1, cand = bs(z1)
    return cand


def reachable(solver, want, sign):
    """How much of the requested travel the linkage actually solves."""
    step = max(want / 100.0, 1.0)
    reached, t = 0.0, step
    while t <= want + 1e-9:
        try:
            solver.solve(sign * t)
        except ValueError:
            break
        reached = t
        t += step
    return min(reached, want)


def baja_dw(track=None, mr=0.55, camber=-1.0, kingpin=None):
    """`mr` is SHOCK travel / wheel travel, the convention
    `SetupVariables.motion_ratio_goal` documents (seed.py). The team
    convention of wheel/shock (~1.8) is its reciprocal; passing that
    here quarters the travel budget, which is how the previous
    generator ended up seeding 69 mm of bump on a Baja car."""
    sv = SetupVariables(
        track_width=track or BAJA["track"], wheelbase=BAJA["wheelbase"],
        ride_height=BAJA["ride_height"], tire_radius=BAJA["tire_radius"],
        tire_width=BAJA["tire_width"], shock_min_length=BAJA["shock_min"],
        shock_max_length=BAJA["shock_max"],
        shock_length_at_ride=BAJA["shock_ride"], motion_ratio_goal=mr,
        static_camber_deg=camber, kickup_deg=8.0,
        frame_half_width=BAJA["frame_half_width"], kingpin_length=kingpin)
    hp, report = generate_seed(sv)
    hp = rezero_bump_steer(hp)
    s = DoubleWishboneSolver(hp)
    bump = reachable(s, report.bump_travel, +1.0)
    droop = reachable(s, report.droop_travel, -1.0)
    assert bump > 100 and droop > 50, (bump, droop)
    assert sketch_planarity(hp)["axis_misalign_deg"] < 1e-9
    return sv, hp, bump, droop, corner_metrics(s, s.solve(0.0))


def rear_halfshaft(ride_height, tire_radius):
    """Inner CV at a plausible gearbox flange, raised above the wheel
    centre so the shaft runs level near MID-travel rather than at ride --
    a worked example of what the CV plunge check is for."""
    wc_z = tire_radius - ride_height
    return HalfshaftConfig(
        enabled=True,
        inner=np.array([0.65 * BAJA["wheelbase"] / 2.0, 120.0, wc_z + 110.0]),
        inner_joint="plunging_cv",
        max_cv_inner_deg=25.0, max_cv_outer_deg=45.0,
        plunge_total_mm=50.0, plunge_in_pct=50.0).to_dict()


def design(sv, hp, bump, droop, halfshaft=None):
    return AxleDesign(hardpoints=hp, setup=sv, droop_travel=droop,
                      bump_travel=bump, halfshaft=halfshaft)


# ----------------------------------------------------------------------
# A simple ladder frame, so the backdrop feature has something to load.
# ----------------------------------------------------------------------
def make_frame(path):
    """A ladder chassis whose rails run through the demo car's own
    inboard pickups, so the backdrop is useful rather than decorative.

    Authored in the CHASSIS convention the tool expects from an Onshape
    export -- Z up, +Y forward, X lateral -- and in METRES, so loading it
    exercises both the scale guess (1000) and the 270 degree rotation
    onto the tool's internal frame. Under that transform a mesh point
    (x, y, z) lands at tool (1000*y, -1000*x, 1000*z), which is why the
    rails sit at lateral x = -/+ the pickup Y and at the pickup Z.
    """
    import trimesh
    lower_y, lower_z = LOWER_RAIL          # tool Y, Z of the LCA pickups
    upper_y, upper_z = UPPER_RAIL          # tool Y, Z of the UCA pickups
    y0, y1 = -0.75, 1.20                   # fore-aft extent, metres
    r = 0.0125                             # 25 mm tube

    def tube(a, b):
        a, b = np.asarray(a, float), np.asarray(b, float)
        seg = trimesh.creation.cylinder(radius=r, segment=[a, b], sections=12)
        return seg

    parts = []
    for lat, hgt in ((lower_y / 1000.0, lower_z / 1000.0),
                     (upper_y / 1000.0, upper_z / 1000.0)):
        for sign in (+1.0, -1.0):
            parts.append(tube([sign * lat, y0, hgt], [sign * lat, y1, hgt]))
            # cross-members tying the two sides together
        for y in np.linspace(y0, y1, 5):
            parts.append(tube([-lat, float(y), hgt], [lat, float(y), hgt]))
    # uprights joining lower to upper at three stations
    for y in (y0 + 0.15, 0.25, y1 - 0.15):
        for sign in (+1.0, -1.0):
            parts.append(tube(
                [sign * lower_y / 1000.0, float(y), lower_z / 1000.0],
                [sign * upper_y / 1000.0, float(y), upper_z / 1000.0]))
    trimesh.util.concatenate(parts).export(path)
    return path


# ----------------------------------------------------------------------
# The 1/10 scale RC buggy.
#
# Kinematics are a real 1/10 buggy (260 mm wheelbase, ~220 mm track,
# 37.5 mm tire radius). It is here because it is the best test this tool
# has of SCALE: nearly every default in a suspension program is secretly
# a ratio of a full-size car, and the bugs only show up when you hand it
# a vehicle a tenth the size.
#
# The dynamics block is written fresh. The source file carried a
# full-size car's masses, springs and tire rates alongside the RC
# geometry -- a CG 558 mm up on a 260 mm wheelbase, sitting behind the
# rear axle -- which made every dynamics output meaningless.
# ----------------------------------------------------------------------
RC_VEHICLE = {"wheelbase": 260.0, "cg_height": 45.0,
              "cg_behind_front": 0.55 * 260.0, "brake_front_frac": 0.55}

RC_DYNAMICS = {
    "weight_empty_lb": 3.25, "driver_lb": 0.0,
    "front_axle_lb": 1.20, "rear_axle_lb": 2.05,
    "unsprung_front_lb": 0.20, "unsprung_rear_lb": 0.25,
    # Measured on a bench rig: 6.53 and 6.867 N/mm.
    "tire_rate_front_lbin": 37.287, "tire_rate_rear_lbin": 39.212,
    "spring1_front_lbin": 4.30, "spring1_rear_lbin": 2.20,
    "stack_front": "Single", "stack_rear": "Single",
    "mr_damper_front": 0.508, "mr_damper_rear": 0.61,
    "damp_bump_front_lbsin": 0.228, "damp_rebound_front_lbsin": 0.228,
    "damp_bump_rear_lbsin": 0.171, "damp_rebound_rear_lbsin": 0.171,
    "wheelbase_in": 260.0 / 25.4, "cg_height_in": 45.0 / 25.4,
    "loaded_radius_front_in": 37.5 / 25.4, "loaded_radius_rear_in": 37.5 / 25.4,
    # 0.35 x track, the same RATIO the full-size default encodes.
    "roll_radius_gyr_in": 0.35 * 220.0 / 25.4,
    "cornering_stiff_front_lbdeg": 0.70,
    "cornering_stiff_rear_lbdeg": 1.10,
    "ay_g": 1.0, "ax_g": 0.7, "speed_mph": 20.0,
}


def write_rc(src, dst, frame_path):
    """Re-body the RC project: keep its kinematics, replace the vehicle
    and dynamics blocks, and drop the absolute path its frame pointed at."""
    with open(src, encoding="utf-8") as fh:
        doc = json.load(fh)
    doc["vehicle"] = dict(RC_VEHICLE)
    dyn = dict(doc.get("dynamics") or {})
    dyn.update(RC_DYNAMICS)
    doc["dynamics"] = dyn
    doc["frame"] = {"path": frame_path, "scale": 1000.0, "rot_z_deg": 90.0,
                    "dx": 0.0, "dy": 0.0, "dz": 0.0}
    with open(dst, encoding="utf-8", mode="w") as fh:
        json.dump(doc, fh, indent=1)
    return dst


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    frame_file = make_frame(f"{OUT}/demo_frame.stl")
    frame = {"path": frame_file, "scale": 1000.0, "rot_z_deg": 270.0,
             "dx": 0.0, "dy": 0.0, "dz": 0.0}
    hs = rear_halfshaft(BAJA["ride_height"], BAJA["tire_radius"])

    svf, hpf, bf, df, mf = baja_dw()
    svr, hpr, br, dr, _ = baja_dw(track=59.0 * IN, mr=0.57, camber=-0.5)
    save_project(f"{OUT}/baja_demo_double_wishbone.MICK", ProjectState(
        front=design(svf, hpf, bf, df),
        rear=design(svr, hpr, br, dr, halfshaft=hs),
        unit_key="in", vehicle=BAJA_VEHICLE, frame=frame,
        dynamics=BAJA_DYNAMICS, coords="chassis+y"))

    svt = SetupVariables(
        track_width=59.0 * IN, wheelbase=BAJA["wheelbase"],
        ride_height=BAJA["ride_height"], tire_radius=BAJA["tire_radius"],
        tire_width=8.0 * IN, shock_min_length=BAJA["shock_min"],
        shock_max_length=BAJA["shock_max"],
        shock_length_at_ride=BAJA["shock_ride"], motion_ratio_goal=0.62,
        static_camber_deg=0.0, frame_half_width=BAJA["frame_half_width"])
    hpt = seed_trailing_arm(svt)
    shift = np.array([0.0, 0.0, 10.0 - hpt.pivot_inner[2]])
    hpt.pivot_inner = hpt.pivot_inner + shift
    hpt.pivot_outer = hpt.pivot_outer + shift
    ts = TrailingArmSolver(hpt)
    bt = reachable(ts, 5.0 * IN / 0.62, +1.0)
    dt = reachable(ts, 3.0 * IN / 0.62, -1.0)
    assert bt > 100 and dt > 50, (bt, dt)
    save_project(f"{OUT}/baja_demo_trailing_arm.MICK", ProjectState(
        front=design(svf, hpf, bf, df),
        rear=design(svt, hpt, bt, dt, halfshaft=hs),
        unit_key="in", vehicle=BAJA_VEHICLE, frame=frame,
        dynamics=BAJA_DYNAMICS, coords="chassis+y"))

    rc_src = os.environ.get("RC_SOURCE", f"{OUT}/rc_buggy_1_10.MICK")
    if os.path.exists(rc_src):
        write_rc(rc_src, f"{OUT}/rc_buggy_1_10.MICK", frame_file)

    print(f"front: +{bf:.0f}/-{df:.0f} mm, caster {mf['caster_deg']:.1f} deg, "
          f"RC {mf['roll_center_height_mm']:.0f} mm, "
          f"scrub {mf['scrub_radius_mm']:.0f} mm")
    print(f"trailing rear: +{bt:.0f}/-{dt:.0f} mm")
    print(f"frame rails at tool Y/Z {LOWER_RAIL} (lower), {UPPER_RAIL} "
          f"(upper); measured pickups "
          f"lca ({hpf.lca_inner_front[1]:.0f}, "
          f"{hpf.lca_inner_front[2]:.0f}) "
          f"uca ({hpf.uca_inner_front[1]:.0f}, "
          f"{hpf.uca_inner_front[2]:.0f})")
    print("wrote examples to", OUT)
