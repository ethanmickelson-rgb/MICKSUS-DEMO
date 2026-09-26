"""The suspension-type interface (Phase 5) and its implementations.

Every layout satisfies one small contract (the brief's
`solve(ride_height, steer) -> upright pose`):

    solver = SUSPENSION_TYPES["Double wishbone"](hardpoints)
    state  = solver.solve(travel_mm, steer_mm)      # one position
    states = solver.walk_travels(travels, steer_mm) # a whole sweep

`travel` is the wheel-centre height relative to static (positive = bump);
`steer` is the steering-rack displacement (mm, positive = rack moves
toward the modelled left corner). States are duck-type compatible, so the
metrics/plots/3D layers consume them without caring which layout produced
them (types without a steering knuckle report NaN kingpin metrics).

All types are implemented at the solver level:

  * Double wishbone — closed form (suspension_tool.solver), each hardpoint
    set editable throughout the GUI.
  * Trailing arm — closed form (suspension_tool.trailing_arm); the pivot
    axis may be skewed, covering pure AND semi-trailing arms, with EXACT
    instant centres (axis-pierce construction).
  * Multilink (5-link) — numeric closure (suspension_tool.multilink),
    validated against the double wishbone expressed as five links.
  * C-hub front — closed form (suspension_tool.chub_front): the RC-buggy
    front end (lower arm, C-hub on a hinge pin, camber link, steering
    block on a physical kingpin).
  * Loaded halfshaft (rear) — closed form
    (suspension_tool.loaded_halfshaft): upper arm + knuckle on a bushing
    pin, closed by the halfshaft itself as a structural lateral link.
  * H-arm (rear) — closed form (suspension_tool.harm_rear): a rigid lower
    H-arm grabbing the upright at two outboard points (holding toe),
    closed by a single upper camber link; a simple driven rear end.

GUI note: seeds, the hardpoint table, dragging, the 3D view, sweeps and
export work for every type; the deep tweak/optimizer knobs are richest
for the double wishbone.

DoubleWishboneSolver is attached as a *virtual* subclass
(CornerSolver.register) rather than by inheritance so solver.py — the
heavily-commented teaching module — keeps zero imports from here while
isinstance(solver, CornerSolver) still holds. The other two register the
same way for consistency.
"""

from abc import ABC, abstractmethod

from .chub_front import CHubFrontPoints, CHubFrontSolver
from .harm_rear import HArmRearPoints, HArmRearSolver
from .loaded_halfshaft import LoadedHalfshaftPoints, LoadedHalfshaftSolver
from .multilink import MultilinkPoints, MultilinkSolver
from .solver import DoubleWishboneSolver
from .trailing_arm import TrailingArmPoints, TrailingArmSolver


class CornerSolver(ABC):
    """Contract every suspension-type solver satisfies."""

    @abstractmethod
    def solve(self, travel: float, steer: float = 0.0):
        """Pose the corner at one wheel travel / rack position."""

    @abstractmethod
    def walk_travels(self, travels, steer: float = 0.0):
        """Efficiently pose a whole list of travels."""


CornerSolver.register(DoubleWishboneSolver)
CornerSolver.register(TrailingArmSolver)
CornerSolver.register(MultilinkSolver)
CornerSolver.register(CHubFrontSolver)
CornerSolver.register(LoadedHalfshaftSolver)
CornerSolver.register(HArmRearSolver)


def solver_for(hp):
    """The solver class for a hardpoint set, by its type."""
    if isinstance(hp, TrailingArmPoints):
        return TrailingArmSolver
    if isinstance(hp, MultilinkPoints):
        return MultilinkSolver
    if isinstance(hp, CHubFrontPoints):
        return CHubFrontSolver
    if isinstance(hp, LoadedHalfshaftPoints):
        return LoadedHalfshaftSolver
    if isinstance(hp, HArmRearPoints):
        return HArmRearSolver
    return DoubleWishboneSolver


# Dropdown registry: display name -> solver class.
SUSPENSION_TYPES = {
    "Double wishbone": DoubleWishboneSolver,
    "Trailing arm": TrailingArmSolver,
    "Multilink": MultilinkSolver,
    "C-hub front": CHubFrontSolver,
    "Loaded halfshaft (rear)": LoadedHalfshaftSolver,
    "H-arm (rear)": HArmRearSolver,
}

# Types that CANNOT go on a steered axle. An H-arm grabs the upright at two
# outboard points and a loaded-halfshaft knuckle is closed by the shaft
# itself: both hold toe rigidly by design, so neither has — or can have — a
# steering input. Offering them on the front lets an inexperienced user
# build a car that physically cannot steer, and the tool would happily
# solve it and plot a flat toe curve as if that were fine.
REAR_ONLY_TYPES = frozenset({
    "Loaded halfshaft (rear)",
    "H-arm (rear)",
    # A trailing arm has no tie rod at all: its toe reads the same at 0 and
    # 20 mm of rack, so a trailing-arm front axle physically cannot turn.
    # Same failure mode as the two above, measured the same way.
    "Trailing arm",
})


def types_for_axle(axle_key: str) -> list:
    """Suspension-type names offerable on `axle_key` ("front"/"rear")."""
    if str(axle_key).lower() == "front":
        return [n for n in SUSPENSION_TYPES if n not in REAR_ONLY_TYPES]
    return list(SUSPENSION_TYPES)


def is_allowed_on(name: str, axle_key: str) -> bool:
    """Can this suspension type be used on this axle?"""
    return name in types_for_axle(axle_key)


def steers(name: str) -> bool:
    """Does this type have a steering input at all?"""
    return name not in REAR_ONLY_TYPES


# name -> hardpoint dataclass for each type (None = uses DoubleWishbonePoints)
SUSPENSION_POINT_TYPES = {
    "Double wishbone": None,
    "Trailing arm": TrailingArmPoints,
    "Multilink": MultilinkPoints,
    "C-hub front": CHubFrontPoints,
    "Loaded halfshaft (rear)": LoadedHalfshaftPoints,
    "H-arm (rear)": HArmRearPoints,
}
