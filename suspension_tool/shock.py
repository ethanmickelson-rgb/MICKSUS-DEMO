"""Fixed shock hardware spec, and the travel limits it imposes.

The shock's compressed/extended lengths are HARDWARE numbers — known from
the datasheet before any geometry exists, and unchanged by seeding,
tweaking or dragging. Like the halfshaft config, the spec lives with the
axle (persisted in the .MICK) instead of being a seed-form field that
goes stale: the travel range, the stroke-split readout and the checklist
all read the same spec, so the displayed travel can never quietly pose
the linkage with the shock past its physical stroke (which the bare
solver would happily do — a shock is just two points to it).
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class ShockSpec:
    """Eye-to-eye hardware limits of one axle's shock (mm)."""

    min_length: float        # fully compressed (bottomed)
    max_length: float        # fully extended (topped out)

    @property
    def stroke(self) -> float:
        return float(self.max_length) - float(self.min_length)

    def to_dict(self) -> dict:
        return {"min_length": float(self.min_length),
                "max_length": float(self.max_length)}

    @classmethod
    def from_dict(cls, data: dict | None):
        if not data:
            return None
        return cls(min_length=float(data["min_length"]),
                   max_length=float(data["max_length"]))


def shock_travel_limits(solver, spec: ShockSpec,
                        cap: float = 500.0) -> dict:
    """How far the corner can ACTUALLY travel before the shock (or the
    linkage) stops it — fast enough to run on EVERY geometry edit, so
    the travel range can track the shock's stroke automatically.

    Per side: grow a bracket geometrically from static until the shock
    length crosses its hardware limit (or the linkage stops solving),
    then bisect the crossing to ~0.05 mm. Cost is a few dozen solves,
    not a step-by-step walk. Returns {'bump', 'droop', 'bump_stop',
    'droop_stop'} with stops one of 'shock' | 'linkage' | 'cap'."""

    def length_at(t):
        return float(solver.solve(t).shock_length)

    out = {}
    for name, sign, hw in (("bump", +1.0, float(spec.min_length)),
                           ("droop", -1.0, float(spec.max_length))):

        def crossed(length):
            return length <= hw if sign > 0 else length >= hw

        t_ok = 0.0                       # solvable, not yet crossed
        limit, stop = cap, "cap"
        grow = 16.0
        while abs(t_ok) + grow <= cap * 2.0:
            t_try = t_ok + sign * grow
            if abs(t_try) > cap:
                t_try = sign * cap
            try:
                length = length_at(t_try)
            except ValueError:
                # linkage locks inside (t_ok, t_try): bisect the lock,
                # then check the shock didn't cross first
                a, b = t_ok, t_try
                for _ in range(22):
                    m = 0.5 * (a + b)
                    try:
                        length_at(m)
                        a = m
                    except ValueError:
                        b = m
                if crossed(length_at(a)):
                    stop = "shock"       # crossed before the lock: fall
                    b = a                # through to the shock bisection
                    a = t_ok
                    break
                limit, stop = abs(a), "linkage"
                break
            if crossed(length):
                a, b = t_ok, t_try       # crossing inside (a, b)
                stop = "shock"
                break
            t_ok = t_try
            grow *= 2.0
            if abs(t_ok) >= cap:
                break
        if stop == "shock":
            for _ in range(22):          # bisect the length crossing
                m = 0.5 * (a + b)
                if crossed(length_at(m)):
                    b = m
                else:
                    a = m
            limit = abs(0.5 * (a + b))
        out[name] = float(limit)
        out[f"{name}_stop"] = stop
    return out
