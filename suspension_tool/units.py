"""Project-wide display units.

The whole tool stores geometry INTERNALLY in millimetres and angles in
degrees — the solver, seed generator, and saved files never change. This
module only governs how lengths are shown to and entered by the user.
Angles stay in degrees in every unit system; ratios are dimensionless.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Unit:
    key: str        # short id used in saved files ("mm", "in")
    label: str      # suffix shown in the UI
    mm: float       # millimetres in one of this unit
    decimals: int   # display precision for lengths
    step: float     # spin-box step, in this unit

    def from_mm(self, v_mm: float) -> float:
        return v_mm / self.mm

    def to_mm(self, v: float) -> float:
        return v * self.mm

    def fmt(self, v_mm: float) -> str:
        return f"{v_mm / self.mm:.{self.decimals}f}"


# Only the two the team actually uses (per Phase 3 feedback), default inch.
UNITS = {
    "mm": Unit("mm", "mm", 1.0, 1, 1.0),
    "in": Unit("in", "in", 25.4, 3, 0.05),
}
DEFAULT_UNIT = "in"
