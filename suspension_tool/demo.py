"""Quick command-line demo: solve the example Baja front corner and a
Phase 2 generated seed (generic full-size setup) through travel and print the
metric tables.

Run from the repo root:  python -m suspension_tool.demo
"""

import numpy as np

from .geometry import example_baja_front
from .metrics import sweep_metrics
from .seed import IN, SetupVariables, generate_seed
from .solver import DoubleWishboneSolver


def print_table(solver: DoubleWishboneSolver, travels) -> None:
    m = sweep_metrics(solver, travels)
    cols = [
        ("travel", "travel_mm", "{:8.1f}"),
        ("camber", "camber_deg", "{:8.3f}"),
        ("toe", "toe_deg", "{:8.3f}"),
        ("caster", "caster_deg", "{:8.3f}"),
        ("KPI", "kpi_deg", "{:8.3f}"),
        ("scrub", "scrub_radius_mm", "{:8.2f}"),
        ("trail", "caster_trail_mm", "{:8.2f}"),
        ("RC h", "roll_center_height_mm", "{:8.2f}"),
        ("shock", "shock_length_mm", "{:8.2f}"),
        ("MR", "motion_ratio", "{:8.3f}"),
    ]
    print("".join(f"{name:>9}" for name, _, _ in cols))
    for k in range(len(travels)):
        print("".join(fmt.format(m[key][k]) for _, key, fmt in cols))


def main() -> None:
    print("Units: mm and degrees. Travel positive = bump. "
          "MR = shock/wheel (< 1, standard convention).")

    print("\n--- Example front corner (geometry.example_baja_front) ---")
    solver = DoubleWishboneSolver(example_baja_front())
    print_table(solver, np.arange(-75.0, 75.0 + 1e-9, 15.0))

    print("\n--- Phase 2 seed from a generic full-size setup ---")
    sv = SetupVariables(
        track_width=60.0 * IN, wheelbase=60.0 * IN, ride_height=12.0 * IN,
        tire_radius=11.5 * IN, tire_width=7.0 * IN,
        shock_min_length=14.0 * IN, shock_max_length=22.0 * IN,
        shock_length_at_ride=19.0 * IN, motion_ratio_goal=0.55,
        static_camber_deg=-1.0, desired_scrub_radius=30.0,
    )
    hp, report = generate_seed(sv)
    print(f"travel budget: +{report.bump_travel:.1f} / -{report.droop_travel:.1f} mm,"
          f" static MR {report.motion_ratio_static:.3f},"
          f" static bump steer {report.bump_steer_static:+.5f} deg/mm")
    print(f"chassis kickup {report.kickup_deg:.0f} deg (bushing axes normal to"
          f" the sketch plane); static caster {report.caster_static:.1f} deg"
          f" (design value, not inflated by kickup)")
    travels = np.linspace(-report.droop_travel, report.bump_travel, 11)
    print_table(DoubleWishboneSolver(hp), travels)


if __name__ == "__main__":
    main()
