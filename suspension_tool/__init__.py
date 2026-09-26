"""Suspension kinematics & synthesis tool for Baja SAE.

Phase 1: forward double-wishbone solver (no GUI yet).

Coordinate convention (used EVERYWHERE in this project):
    +X = forward (direction of travel)
    +Y = left (driver side on a LHD car)
    +Z = up
    Right-handed. Units are millimetres. The origin sits at GROUND LEVEL,
    on the vehicle centreline, at the X-station of the axle being modelled.
    We model the LEFT corner, so all hardpoints have y > 0.
"""

from .geometry import DoubleWishbonePoints, example_baja_front
from .solver import DoubleWishboneSolver, CornerState
from .metrics import corner_metrics, sweep_metrics, front_view_ic
from .seed import SetupVariables, SeedReport, generate_seed, sketch_plane_normal
from .project import ProjectState, save_project, load_project
from .suspension_types import (SUSPENSION_TYPES, CornerSolver,
                               REAR_ONLY_TYPES, types_for_axle,
                               is_allowed_on)
from .vehicle import (VehicleParams, anti_geometry, roll_axis_metrics,
                      roll_axis_points)
from .trailing_arm import (TrailingArmPoints, TrailingArmSolver,
                           seed_trailing_arm)
from .multilink import (MultilinkPoints, MultilinkSolver,
                        from_double_wishbone, seed_multilink)
from .chub_front import (CHubFrontPoints, CHubFrontSolver, seed_chub_front)
from .loaded_halfshaft import (LoadedHalfshaftPoints, LoadedHalfshaftSolver,
                               seed_loaded_halfshaft)
from .harm_rear import (HArmRearPoints, HArmRearSolver, seed_harm_rear)
from .tire import (TireModel, TireEstimate, SUNF_23x7, SURFACE_PEAK_MU,
                   pacejka_lateral_coeffs, ExponentialTire, MEASURED_TIRES,
                   DUNLOP_KT821_CLAY, DUNLOP_KT821_GRAVEL, surface_tire,
                   tire_data_spread)
from .yaw_moment import (YawMomentVehicle, YawMomentDiagram,
                         yaw_moment_diagram, solve_point, AUBURN_EXAMPLE)
from .impact import (ImpactCase, STANDARD_CASES, corner_loads, loads_csv,
                     export_loads_csv, drop_height_to_mph,
                     WheelLoad, friction_circle_loads, corner_conditions,
                     standard_load_matrix, envelope, EnvelopeResult,
                     envelope_csv, export_envelope_csv,
                     RockerGeometry, rocker_loads, corner_loads_rocker,
                     BodyLoadSet, SectionLoad, BeamResult,
                     beam_internal_loads, control_arm_beams,
                     shock_bending_summary, body_loads_csv,
                     export_body_loads_csv)
from .sweeps import (roll_sweep, pitch_sweep, steer_sweep,
                     bump_steer_map)
from .ride_freq import (QuarterCar, RideReport, ride_report, from_dynamics,
                        excitation_speed_mph, terrain_table,
                        default_frequencies, TRANSFER_FUNCTIONS,
                        DEFAULT_TERRAIN)
from .full_car import (FullCar, FullCarReport, full_car_report,
                       wheelbase_filter_speeds)
from .equations import SECTIONS as EQUATION_SECTIONS, all_equations
from .units import UNITS, DEFAULT_UNIT

__all__ = [
    "DoubleWishbonePoints",
    "example_baja_front",
    "DoubleWishboneSolver",
    "CornerState",
    "corner_metrics",
    "sweep_metrics",
    "front_view_ic",
    "SetupVariables",
    "SeedReport",
    "generate_seed",
    "sketch_plane_normal",
    "ProjectState",
    "save_project",
    "load_project",
    "SUSPENSION_TYPES",
    "REAR_ONLY_TYPES",
    "types_for_axle",
    "is_allowed_on",
    "VehicleParams",
    "roll_axis_metrics",
    "roll_axis_points",
    "CornerSolver",
    "TrailingArmPoints",
    "TrailingArmSolver",
    "MultilinkPoints",
    "MultilinkSolver",
    "CHubFrontPoints",
    "CHubFrontSolver",
    "LoadedHalfshaftPoints",
    "LoadedHalfshaftSolver",
    "HArmRearPoints",
    "HArmRearSolver",
    "seed_trailing_arm",
    "seed_multilink",
    "seed_chub_front",
    "seed_loaded_halfshaft",
    "seed_harm_rear",
    "from_double_wishbone",
    "anti_geometry",
    "roll_sweep",
    "pitch_sweep",
    "steer_sweep",
    "bump_steer_map",
    "TireModel",
    "TireEstimate",
    "SUNF_23x7",
    "SURFACE_PEAK_MU",
    "pacejka_lateral_coeffs",
    "ExponentialTire",
    "MEASURED_TIRES",
    "DUNLOP_KT821_CLAY",
    "DUNLOP_KT821_GRAVEL",
    "surface_tire",
    "tire_data_spread",
    "YawMomentVehicle",
    "YawMomentDiagram",
    "yaw_moment_diagram",
    "solve_point",
    "AUBURN_EXAMPLE",
    "ImpactCase",
    "STANDARD_CASES",
    "corner_loads",
    "loads_csv",
    "export_loads_csv",
    "drop_height_to_mph",
    "WheelLoad",
    "friction_circle_loads",
    "corner_conditions",
    "standard_load_matrix",
    "envelope",
    "EnvelopeResult",
    "envelope_csv",
    "export_envelope_csv",
    "RockerGeometry",
    "rocker_loads",
    "corner_loads_rocker",
    "BodyLoadSet",
    "SectionLoad",
    "BeamResult",
    "beam_internal_loads",
    "control_arm_beams",
    "shock_bending_summary",
    "body_loads_csv",
    "export_body_loads_csv",
    "QuarterCar",
    "RideReport",
    "ride_report",
    "from_dynamics",
    "excitation_speed_mph",
    "terrain_table",
    "default_frequencies",
    "TRANSFER_FUNCTIONS",
    "DEFAULT_TERRAIN",
    "FullCar",
    "FullCarReport",
    "full_car_report",
    "wheelbase_filter_speeds",
    "EQUATION_SECTIONS",
    "all_equations",
    "UNITS",
    "DEFAULT_UNIT",
]

# Bump this whenever the UI/behaviour changes, so a running build is easy to
# identify (shown in the window title and the view-controls row).
__version__ = "1.43.0"
