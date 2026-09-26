"""Display/export coordinate conventions.

The MATH throughout this tool lives in one frame forever: +X forward,
+Y left, +Z up (see the package docstring). But the team's chassis CAD
(the team's Onshape models) is drawn with the fore-aft axis on Y and the
lateral axis on X — so the numbers people paste between the tool and
Onshape need remapping. That remapping is PRESENTATION ONLY and happens
here: the hardpoint table, the CSV exports, the Onshape push and the CG
readout run every point through the active convention; solvers, metrics
and project files never see it (projects store tool-frame mm, always).

Each convention is an orthonormal basis change, so from_display is just
the transpose. Two chassis variants are offered because the photo of the
frame's view cube fixes the fore-aft AXIS but not its SIGN — pick the
one that matches the CAD ("+Y fwd" means the nose points toward +Y).
"""

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class AxisConvention:
    key: str
    name: str                       # shown in the toolbar combo
    labels: tuple                   # column/axis captions, display order
    mat: np.ndarray = field(repr=False)   # display = mat @ internal

    def to_display(self, p) -> np.ndarray:
        return self.mat @ np.asarray(p, dtype=float)

    def from_display(self, q) -> np.ndarray:
        return self.mat.T @ np.asarray(q, dtype=float)


CONVENTIONS = {
    # native tool frame — what every previous release displayed
    "tool": AxisConvention(
        "tool", "Tool (X fwd)",
        ("X fwd", "Y left", "Z up"),
        np.eye(3)),
    # chassis frame, nose toward +Y: X = right, Y = forward, Z = up
    # (right-handed: right x forward = up)
    "chassis+y": AxisConvention(
        "chassis+y", "Chassis (+Y fwd)",
        ("X right", "Y fwd", "Z up"),
        np.array([[0.0, -1.0, 0.0],
                  [1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0]])),
    # chassis frame, nose toward -Y: X = left, Y = rearward, Z = up
    "chassis-y": AxisConvention(
        "chassis-y", "Chassis (-Y fwd)",
        ("X left", "Y aft", "Z up"),
        np.array([[0.0, 1.0, 0.0],
                  [-1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0]])),
}

DEFAULT_CONVENTION = "chassis+y"
