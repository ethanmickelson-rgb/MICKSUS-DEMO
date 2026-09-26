"""Display units for the dynamics panel.

The dynamics math is imperial end to end — Milliken's formulas, the team's
spring catalogue, `DynamicsInputs`, and every saved .MICK file. None of
that changes here. This module is a DISPLAY layer only, exactly as
`units.py` is for geometry: the panel converts on the way to the screen
and back on the way in, and the value it hands to `compute()` is always
lb / in / lb-in.

When the display unit is mm the panel shows SI — mass in kg, force in N,
rates in N/mm, roll rates in N·m/deg, speed in km/h. Mass is the one
place the two systems disagree about what the number MEANS rather than
just how big it is: imperial "Empty weight (lb)" becomes "Empty mass
(kg)". The numeric conversion (1 lb → 0.45359237 kg) is the
weight-at-one-g equivalence, which is what the spreadsheet's pounds have
always been.

DECIMALS matter more than they look. The tool has to serve a 600 lb Baja
car and a 3 kg RC buggy from the same spin boxes, so each dimension
carries a `dec_shift` chosen to keep the metric side at least as fine as
the imperial side — a rate quoted to 0.1 lb/in needs 3 decimals in N/mm
to say as much.
"""

import math
import re
from dataclasses import dataclass

# Exact by definition, every one of them.
LB_TO_KG = 0.45359237
LBF_TO_N = 4.4482216152605
IN_TO_MM = 25.4
FT_TO_MM = 304.8
MPH_TO_KMH = 1.609344
FT2_TO_M2 = 0.09290304
IN2_TO_MM2 = 645.16
LBIN_TO_NMM = LBF_TO_N / IN_TO_MM        # lb/in → N/mm, lb·s/in → N·s/mm
LBFT_TO_NM = LBF_TO_N * 0.3048           # lb-ft → N·m


@dataclass(frozen=True)
class Dim:
    """One physical dimension, and how it is shown in each system."""

    imperial: str            # suffix when the display unit is inch
    metric: str              # suffix when it is mm
    factor: float = 1.0      # metric value of one imperial unit
    dec_shift: int = 0       # decimals to ADD in metric (may be negative)
    imperial_noun: str = ""  # {q} in a label, e.g. "weight" vs "mass"
    metric_noun: str = ""

    def unit(self, metric: bool) -> str:
        return self.metric if metric else self.imperial

    def noun(self, metric: bool) -> str:
        return self.metric_noun if metric else self.imperial_noun

    def to_display(self, v: float, metric: bool) -> float:
        return v * self.factor if metric else v

    def to_imperial(self, v: float, metric: bool) -> float:
        return v / self.factor if metric else v

    def decimals(self, base: int, metric: bool) -> int:
        return max(0, min(6, base + self.dec_shift)) if metric else base


DIMENSIONLESS = Dim("", "")

DIMS = {
    "": DIMENSIONLESS,
    # Mass is shown as MASS in metric — the noun changes, not just the unit.
    "mass": Dim("lb", "kg", LB_TO_KG, +1, "weight", "mass"),
    "force": Dim("lb", "N", LBF_TO_N, 0),
    "len": Dim("in", "mm", IN_TO_MM, -1),
    "len_ft": Dim("ft", "mm", FT_TO_MM, -2),
    # 0.1 lb/in is 0.0175 N/mm, so metric needs two more decimals to keep
    # the same resolution on a 2 N/mm RC spring.
    "rate": Dim("lb/in", "N/mm", LBIN_TO_NMM, +2),
    "damp": Dim("lb·s/in", "N·s/mm", LBIN_TO_NMM, +2),
    "cstiff": Dim("lb/deg", "N/deg", LBF_TO_N, 0),
    "rollrate": Dim("lb-ft/deg", "N·m/deg", LBFT_TO_NM, +1),
    "speed": Dim("mph", "km/h", MPH_TO_KMH, 0),
    "area_l": Dim("ft²", "m²", FT2_TO_M2, +2),
    "area_s": Dim("in²", "mm²", IN2_TO_MM2, -2),
    # Same in both systems, but still worth naming so every row carries a
    # dimension and the renderer needs no special cases.
    "deg": Dim("deg", "deg"),
    "deg_g": Dim("deg/g", "deg/g"),
    "hz": Dim("Hz", "Hz"),
    "g": Dim("g", "g"),
    "pct": Dim("%", "%"),
}


def dim(key: str | None) -> Dim:
    """The Dim for a key; unknown or empty keys are dimensionless."""
    return DIMS.get(key or "", DIMENSIONLESS)


_SPEC = re.compile(r"^\{:([-+ ]?)(\d*)\.(\d+)([fF%])\}$")


def shift_fmt(fmt: str, n: int) -> str:
    """`fmt` with its decimal count moved by n, never below zero.

    Anything that is not a plain fixed-point spec is passed through
    untouched, so an exotic format never silently loses its meaning.
    """
    if n == 0:
        return fmt
    m = _SPEC.match(fmt)
    if m is None:
        return fmt
    sign, width, dec, kind = m.groups()
    return "{:%s%s.%d%s}" % (sign, width, max(0, int(dec) + n), kind)


def nice_step(x: float) -> float:
    """Round a converted spin step to 1/2/5 × 10^n.

    A 5 lb step converts to 2.268 kg, which is a silly thing to put on an
    arrow key. This gives 2 kg instead.
    """
    if not (x > 0.0) or not math.isfinite(x):
        return 1.0
    e = math.floor(math.log10(x))
    m = x / 10.0 ** e
    m = 1.0 if m < 1.5 else 2.0 if m < 3.5 else 5.0 if m < 7.5 else 10.0
    return m * 10.0 ** e


def widen(lo: float, hi: float, decimals: int) -> tuple[float, float]:
    """Snap a converted range OUTWARD to the display precision.

    Qt rounds a spin box's min/max to its decimals. Rounding a converted
    minimum the wrong way would clip a value that is legal in imperial,
    so round the low end down and the high end up.
    """
    q = 10.0 ** -decimals
    return (math.floor(lo / q + 1e-9) * q, math.ceil(hi / q - 1e-9) * q)
