"""Shared closed-form math for "carrier on a hinge pin" suspensions.

Two of the tool's suspension types are the SAME mechanism wearing
different clothes:

  * C-hub front (the RC-buggy front end, e.g. Team Associated B7): a
    lower control arm carries a hinge pin at its outer end; the C-hub
    (caster block) rotates on that pin; a single CAMBER LINK from the
    chassis to the top of the C-hub closes the loop. A steering block
    then pivots on a physical kingpin fixed in the C-hub.

  * Loaded-halfshaft rear: an upper control arm carries a bushing pin at
    its outer end; the knuckle rotates on that pin; the HALFSHAFT itself
    (diff output flange -> outer CV, fixed length, structurally loaded)
    closes the loop. No steering.

Chain, in both cases:

    chassis --revolute--> arm --revolute (hinge pin)--> carrier
                                                        |
    chassis --fixed-length link (camber link / halfshaft)+

Pose parameters: `a` = arm angle about its chassis bushing axis, `b` =
carrier angle about the POSED hinge pin (both zero at static, where all
link lengths are measured). Given `a`, the closing link reduces `b` to
the familiar A*cos b + B*sin b = C identity — the same one the double
wishbone and trailing arm use — so everything here is closed form.

Branch convention (see the assembly-branch lesson in the domain skill):
the identity has two roots — the real assembly and the folded-over one.
The static pose IS b = 0, and for buildable geometry the fold-over root
sits far away, so we take the root nearest zero, picked ONCE per pose
from the static branch. Never re-pick from "nearest previous" — that is
how solvers teleport onto the folded assembly at large travel.
"""

import numpy as np


def _unit(v):
    return v / np.linalg.norm(v)


def rotate_about(origin, axis, p, ang):
    """Rodrigues rotation of point `p` about the line (origin, unit axis)
    by `ang` radians."""
    r = p - origin
    return origin + rotate_dir(axis, r, ang)


def rotate_dir(axis, d, ang):
    """Rodrigues rotation of a direction vector (no origin)."""
    c, s = np.cos(ang), np.sin(ang)
    return d * c + np.cross(axis, d) * s + axis * np.dot(axis, d) * (1.0 - c)


def closing_angle(origin, axis, p, target, length):
    """The rotation `b` about the line (origin, unit axis) that puts point
    `p` at distance `length` from `target` — the closed-form closure every
    hinge-carrier type uses.

    Decompose p's circle about the axis (the double wishbone's UBJ trick):
    p(b) = c + u*cos b + v*sin b, then |p(b) - target|^2 = length^2 gives
    A*cos b + B*sin b = C. Returns the root nearest ZERO (the static
    assembly branch). Raises ValueError when the link cannot reach — the
    articulation limit."""
    rel = p - origin
    c = origin + axis * np.dot(axis, rel)
    u = p - c
    v = np.cross(axis, u)
    w = c - target
    A = 2.0 * np.dot(w, u)
    B = 2.0 * np.dot(w, v)
    C = length * length - np.dot(w, w) - np.dot(u, u)
    R = np.hypot(A, B)
    if R < 1e-12 or abs(C) > R:
        raise ValueError("closing link cannot reach this pose")
    phi = np.arctan2(B, A)
    delta = np.arccos(np.clip(C / R, -1.0, 1.0))
    return min(_wrap(phi + delta), _wrap(phi - delta), key=abs)


def _wrap(ang):
    """Wrap an angle to (-pi, pi] so 'nearest zero' compares correctly."""
    return (ang + np.pi) % (2.0 * np.pi) - np.pi


def solve_travel(wc_z_of, target_z, last_a=0.0, span=1.2):
    """Find the arm angle `a` with wc_z_of(a) == target_z: secant from the
    warm start, falling back to a bracketed scan + bisection near static.
    Every evaluation is closed form, so the scan fallback is still cheap.
    Raises ValueError when the travel is outside the linkage's reach."""
    def f(a):
        return wc_z_of(a) - target_z
    # -- secant, warm-started (covers virtually every real call) --------
    a0, a1 = last_a, last_a + 0.01
    try:
        f0, f1 = f(a0), f(a1)
        for _ in range(30):
            if abs(f1) < 1e-9:
                return a1
            if abs(f1 - f0) < 1e-15 or abs(a1) > span:
                break
            a0, a1, f0 = a1, a1 - f1 * (a1 - a0) / (f1 - f0), f1
            f1 = f(a1)
    except ValueError:
        pass                       # wandered past a closure limit: rescan
    # -- robust fallback: scan outward from static for a sign bracket ---
    f_static = f(0.0)
    if abs(f_static) < 1e-9:
        return 0.0
    # walk BOTH directions; the target's side is unknown for a skewed arm
    for direction in (1.0, -1.0):
        pa, pf = 0.0, f_static
        aa = 0.0
        while abs(aa) < span:
            aa += direction * 0.02
            try:
                fa = f(aa)
            except ValueError:
                break              # closure limit before a bracket: give up
            if pf == 0.0 or (fa < 0) != (pf < 0):
                lo, hi, flo = pa, aa, pf
                for _ in range(60):            # bisection to ~1e-18 rad
                    mid = 0.5 * (lo + hi)
                    fm = f(mid)
                    if abs(fm) < 1e-9:
                        return mid
                    if (fm < 0) == (flo < 0):
                        lo, flo = mid, fm
                    else:
                        hi = mid
                return 0.5 * (lo + hi)
            pa, pf = aa, fa
    raise ValueError(
        f"travel target z={target_z:.1f} mm is outside the linkage's reach")
