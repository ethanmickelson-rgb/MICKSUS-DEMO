"""Forward kinematics for one double-wishbone corner.

Method (stated up front, per the project brief):

The mechanism has ONE degree of freedom once the tie rod is fixed
(no steering input yet). We parameterise it by the LOWER control arm's
rotation angle about its inner-pivot axis, then resolve the rest of the
linkage with closed-form geometric intersections:

  1. LCA angle `a` -> lower ball joint (LBJ) position: a pure rotation of
     the static LBJ about the LCA's inner-pivot axis (Rodrigues' formula).
  2. Upper ball joint (UBJ): it must lie on the CIRCLE swept by the UCA
     about its own inner axis, AND on the SPHERE of radius
     |UBJ - LBJ| (the rigid upright's "kingpin" spacing) centred on the
     current LBJ. Circle ∩ sphere reduces to A·cos(b) + B·sin(b) = C,
     which we solve analytically (no iteration).
  3. Tie-rod outer (TRO): intersection of THREE spheres — fixed distance
     from LBJ and from UBJ (rigid upright) and fixed tie-rod length from
     the chassis-side tie-rod inner. Solved by standard trilateration.
  4. The upright's rigid-body pose is recovered from the three solved
     points (LBJ, UBJ, TRO) with the Kabsch algorithm (SVD best-fit
     rotation — exact here, since the upright is rigid). The wheel centre
     and spindle axis are carried along with that pose.

To solve AT A GIVEN WHEEL TRAVEL we root-find (scipy brentq) the LCA
angle that puts the wheel centre at the requested height. That is the
only iterative step, and it is a well-behaved 1D bracketed root find.

Sign/axis conventions: see the package docstring (+X fwd, +Y left, +Z up,
mm, left corner). "Travel" is the wheel-centre height relative to static:
positive = bump (wheel moves up), negative = droop.
"""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import brentq

from .geometry import DoubleWishbonePoints

Z_UP = np.array([0.0, 0.0, 1.0])


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)


def _rotate_about_line(p, origin, axis_unit, angle):
    """Rotate point `p` about the line through `origin` with direction
    `axis_unit` by `angle` (radians) — Rodrigues' rotation formula."""
    r = p - origin
    return (
        origin
        + r * np.cos(angle)
        + np.cross(axis_unit, r) * np.sin(angle)
        + axis_unit * np.dot(axis_unit, r) * (1.0 - np.cos(angle))
    )


@dataclass
class CornerState:
    """Solved pose of the corner at one travel position."""

    travel: float            # wheel-centre height above static (mm)
    steer: float             # rack displacement from centre (mm, + = left)
    lca_angle: float         # LCA rotation from static (rad)
    lbj: np.ndarray          # lower ball joint
    ubj: np.ndarray          # upper ball joint
    tro: np.ndarray          # tie-rod outer
    wheel_center: np.ndarray
    spindle: np.ndarray      # unit vector along the wheel rotation axis,
                             # pointing outboard (+Y side for the left corner)
    contact_patch: np.ndarray
    shock_outer: np.ndarray  # current position of the LCA-mounted shock end
    shock_length: float      # |shock_outer - shock_inner| (mm)
    tierod_inner_cur: np.ndarray = None  # tie-rod inner at THIS pose: equal
                             # to hp.tierod_inner when chassis-mounted, but
                             # swung with the LCA when hp.tierod_on_lca


class DoubleWishboneSolver:
    """Forward solver for one double-wishbone corner. Build it once from a
    hardpoint set; call .solve(travel) as often as you like."""

    def __init__(self, hp: DoubleWishbonePoints):
        self.hp = hp

        # --- control arm rotation axes (chassis side, never move) -------
        self._lca_origin = hp.lca_inner_front
        self._lca_axis = _unit(hp.lca_inner_rear - hp.lca_inner_front)
        self._uca_origin = hp.uca_inner_front
        self._uca_axis = _unit(hp.uca_inner_rear - hp.uca_inner_front)

        # --- circle swept by the UBJ about the UCA axis ------------------
        # Decompose the static UBJ into: centre c (its projection onto the
        # axis) + radius vector u. As the UCA rotates by angle b:
        #     UBJ(b) = c + u*cos(b) + v*sin(b),   v = axis × u
        # (u ⊥ axis by construction, and |v| = |u|.)
        rel = hp.uca_outer - self._uca_origin
        c = self._uca_origin + self._uca_axis * np.dot(self._uca_axis, rel)
        self._ubj_center = c
        self._ubj_u = hp.uca_outer - c
        self._ubj_v = np.cross(self._uca_axis, self._ubj_u)

        # --- rigid lengths fixed by the hardware -------------------------
        self._kingpin_len = np.linalg.norm(hp.uca_outer - hp.lca_outer)
        self._d_lbj_tro = np.linalg.norm(hp.tierod_outer - hp.lca_outer)
        self._d_ubj_tro = np.linalg.norm(hp.tierod_outer - hp.uca_outer)
        self._tierod_len = np.linalg.norm(hp.tierod_outer - hp.tierod_inner)

        # --- static upright reference for the Kabsch pose fit ------------
        self._upright_ref = np.vstack([hp.lca_outer, hp.uca_outer, hp.tierod_outer])
        self._upright_ref_centroid = self._upright_ref.mean(axis=0)

        # Trilateration has two mirror solutions; remember which side of the
        # (LBJ, UBJ, tie-rod-inner) plane the static TRO sits on.
        self._tro_sign = self._trilateration_sign_static()

        # --- UBJ assembly branch (fixed for the life of the mechanism) ---
        # The circle∩sphere closure A cos b + B sin b = C has two roots
        # b = phi ± delta: the real assembly and the folded-over one. The
        # static pose (b = 0) tells us which sign this corner is assembled
        # with; _solve_ubj holds that sign forever so the solver can never
        # drift onto the folded branch at large travel.
        w0 = self._ubj_center - hp.lca_outer
        A0 = 2.0 * np.dot(w0, self._ubj_u)
        B0 = 2.0 * np.dot(w0, self._ubj_v)
        C0 = (self._kingpin_len**2 - np.dot(w0, w0)
              - np.dot(self._ubj_u, self._ubj_u))
        R0 = np.hypot(A0, B0)
        if R0 < 1e-12 or abs(C0) > R0 * (1.0 + 1e-9):
            raise ValueError("static geometry does not close (UCA cannot "
                             "reach the UBJ)")
        phi0 = np.arctan2(B0, A0)
        delta0 = np.arccos(np.clip(C0 / R0, -1.0, 1.0))

        def _wrap(x):
            return np.arctan2(np.sin(x), np.cos(x))

        self._ubj_branch = (1.0 if abs(_wrap(phi0 + delta0))
                            <= abs(_wrap(phi0 - delta0)) else -1.0)

        # --- static wheel orientation ------------------------------------
        # Build the static spindle axis from the user's static camber/toe.
        # Start from pure outboard (+Y for the left corner), tilt for camber
        # (rotation about X), then steer for toe (rotation about Z). The
        # rotation signs are chosen so that the camber/toe measured back from
        # the spindle (see metrics.py) reproduce the inputs exactly.
        gamma = np.radians(hp.static_camber_deg)
        tau = np.radians(hp.static_toe_deg)
        # Exact spindle: front-view camber and top-view toe both read back
        # to machine precision for ANY (camber, toe) — the sequential
        # Rz(-tau)Rx(-gamma) form only reproduced toe exactly and put a
        # gamma*tau^2/2 error into camber (see geometry.static_spindle).
        # Identical to that form whenever toe = 0.
        s = np.array([np.tan(tau), 1.0, -np.tan(gamma)])
        self._spindle0 = s / np.linalg.norm(s)

        self._wc_z0 = hp.wheel_center[2]
        # Lazy cache for the LCA-angle grid used to bracket travel solves.
        self._angle_grid = None

    # ------------------------------------------------------------------
    # Step 2: UBJ via circle ∩ sphere, closed form
    # ------------------------------------------------------------------
    def _solve_ubj(self, lbj: np.ndarray) -> tuple[np.ndarray, float]:
        # |UBJ(b) - lbj|^2 = kingpin_len^2 with UBJ(b) = c + u cos b + v sin b.
        # Expanding (u·v = 0, |u| = |v|) gives A cos b + B sin b = C:
        w = self._ubj_center - lbj
        u, v = self._ubj_u, self._ubj_v
        A = 2.0 * np.dot(w, u)
        B = 2.0 * np.dot(w, v)
        C = self._kingpin_len**2 - np.dot(w, w) - np.dot(u, u)
        R = np.hypot(A, B)
        if R < 1e-12 or abs(C) > R:
            raise ValueError("upper arm cannot reach: geometry does not close")
        # A cos b + B sin b = R cos(b - phi); the two roots b = phi ± delta
        # are the two ASSEMBLY BRANCHES of the linkage (the second is the
        # upright folded over). The branch is a property of how the corner
        # is assembled — it can NEVER change during motion — so the ± sign
        # is chosen ONCE from the static configuration (see __init__) and
        # held. (Choosing "the root nearest b = 0" per-solve, as this used
        # to, silently swapped branches at big travels: the linkage would
        # teleport to the folded assembly and links crossed on screen.)
        # phi is continuous in lbj up to 2*pi wraps, which cos/sin absorb.
        phi = np.arctan2(B, A)
        delta = np.arccos(np.clip(C / R, -1.0, 1.0))
        b = phi + self._ubj_branch * delta
        # report b wrapped to (-pi, pi] so downstream warm starts stay sane
        b = np.arctan2(np.sin(b), np.cos(b))
        return self._ubj_center + u * np.cos(b) + v * np.sin(b), b

    # ------------------------------------------------------------------
    # Step 3: tie-rod outer via three-sphere intersection (trilateration)
    # ------------------------------------------------------------------
    def _current_tierod_inner(self, a: float, b: float, steer: float = 0.0):
        """The tie-rod inner ball-joint centre at this pose: chassis-fixed
        by default, or riding a control arm's rotation — the LOWER arm (by
        LCA angle `a`) when hp.tierod_on_lca, the UPPER arm (by UCA angle
        `b`) when hp.tierod_on_uca. The link still pivots freely at the
        ball joint; only its mount point moves with the arm. The rack's
        steer offset applies either way (zero for a rear toe link)."""
        p = self.hp.tierod_inner
        if getattr(self.hp, "tierod_on_lca", False):
            p = _rotate_about_line(p, self._lca_origin, self._lca_axis, a)
        elif getattr(self.hp, "tierod_on_uca", False):
            p = _rotate_about_line(p, self._uca_origin, self._uca_axis, b)
        return p + np.array([0.0, steer, 0.0])

    def _solve_tro(self, lbj, ubj, sign, steer=0.0, a=0.0, b=0.0):
        # Spheres: centre lbj radius d_lbj_tro, centre ubj radius d_ubj_tro,
        # centre tierod_inner radius tierod_len. Classic trilateration:
        # build a local frame (ex, ey, ez) from the three centres, solve for
        # the point's coordinates (x, y, z) in that frame.
        # Steering input = the rack sliding the tie-rod inner point
        # laterally (+Y = toward this left corner).
        p1, p2 = lbj, ubj
        p3 = self._current_tierod_inner(a, b, steer)
        r1, r2, r3 = self._d_lbj_tro, self._d_ubj_tro, self._tierod_len
        ex = _unit(p2 - p1)
        d = np.linalg.norm(p2 - p1)
        v3 = p3 - p1
        i = np.dot(ex, v3)
        ey = _unit(v3 - i * ex)
        ez = np.cross(ex, ey)
        j = np.dot(ey, v3)
        x = (r1**2 - r2**2 + d**2) / (2.0 * d)
        y = (r1**2 - r3**2 + i**2 + j**2) / (2.0 * j) - (i / j) * x
        zsq = r1**2 - x**2 - y**2
        if zsq < -1e-6:
            raise ValueError("tie rod cannot reach: geometry does not close")
        z = sign * np.sqrt(max(zsq, 0.0))
        return p1 + x * ex + y * ey + z * ez

    def _trilateration_sign_static(self) -> float:
        """Which side of the three-centre plane is the static TRO on?"""
        p1, p2, p3 = self.hp.lca_outer, self.hp.uca_outer, self.hp.tierod_inner
        ex = _unit(p2 - p1)
        v3 = p3 - p1
        ey = _unit(v3 - np.dot(ex, v3) * ex)
        ez = np.cross(ex, ey)
        return 1.0 if np.dot(self.hp.tierod_outer - p1, ez) >= 0.0 else -1.0

    # ------------------------------------------------------------------
    # Step 4: rigid-body pose of the upright from its three solved points
    # ------------------------------------------------------------------
    def _upright_pose(self, lbj, ubj, tro):
        """Best-fit (here: exact) rotation R and translation taking the
        static upright points onto the current ones — Kabsch algorithm."""
        cur = np.vstack([lbj, ubj, tro])
        cur_centroid = cur.mean(axis=0)
        a = self._upright_ref - self._upright_ref_centroid   # static, centred
        b = cur - cur_centroid                               # current, centred
        h = a.T @ b
        u_svd, _, vt = np.linalg.svd(h)
        det = np.linalg.det(vt.T @ u_svd.T)
        r = vt.T @ np.diag([1.0, 1.0, det]) @ u_svd.T
        return r, cur_centroid

    def _carry(self, point0, r, cur_centroid):
        """Move a static upright-fixed point with the solved pose."""
        return r @ (point0 - self._upright_ref_centroid) + cur_centroid

    # ------------------------------------------------------------------
    # Full pose at a given LCA angle
    # ------------------------------------------------------------------
    def solve_at_lca_angle(self, a: float, steer: float = 0.0) -> CornerState:
        hp = self.hp
        lbj = _rotate_about_line(hp.lca_outer, self._lca_origin, self._lca_axis, a)
        ubj, b = self._solve_ubj(lbj)
        # The shock's outer end rides on whichever arm it is mounted to,
        # so it rotates by that arm's angle about that arm's pivot axis.
        if hp.shock_on_uca:
            shock_outer = _rotate_about_line(
                hp.shock_outer, self._uca_origin, self._uca_axis, b)
        else:
            shock_outer = _rotate_about_line(
                hp.shock_outer, self._lca_origin, self._lca_axis, a)
        tro = self._solve_tro(lbj, ubj, self._tro_sign, steer, a, b)
        r, cur_centroid = self._upright_pose(lbj, ubj, tro)

        wheel_center = self._carry(hp.wheel_center, r, cur_centroid)
        spindle = r @ self._spindle0

        # Contact patch: from the wheel centre, go one tire radius "down"
        # WITHIN the wheel plane (so camber tips the patch sideways slightly,
        # like a real cambered wheel).
        forward = _unit(np.cross(spindle, Z_UP))      # wheel heading, in-plane
        down = _unit(np.cross(spindle, forward))      # in-plane, pointing down
        contact_patch = wheel_center + hp.tire_radius * down

        return CornerState(
            travel=wheel_center[2] - self._wc_z0,
            steer=steer,
            lca_angle=a,
            lbj=lbj,
            ubj=ubj,
            tro=tro,
            wheel_center=wheel_center,
            spindle=spindle,
            contact_patch=contact_patch,
            shock_outer=shock_outer,
            shock_length=np.linalg.norm(shock_outer - hp.shock_inner),
            tierod_inner_cur=self._current_tierod_inner(a, b, steer),
        )

    # ------------------------------------------------------------------
    # Solve at a given wheel travel (the public entry point)
    # ------------------------------------------------------------------
    def solve(self, travel: float, steer: float = 0.0) -> CornerState:
        """Solve the corner with the wheel centre `travel` mm above static
        (negative = droop) and the steering rack displaced `steer` mm
        (positive = rack moves toward this left corner). Root-finds the
        LCA angle with brentq."""
        if abs(steer) > 1e-12:
            # Steer perturbs the wheel-centre height only slightly, so the
            # steer-free solution is an excellent warm start for the local
            # root find with the rack displaced.
            base = self.solve(travel, 0.0)
            a = self._local_angle_for_height(
                self._wc_z0 + travel, base.lca_angle, steer)
            return self.solve_at_lca_angle(a, steer)
        if abs(travel) < 1e-12:
            return self.solve_at_lca_angle(0.0)

        grid, heights = self._lca_angle_grid()
        target = self._wc_z0 + travel

        def err(a: float) -> float:
            return self.solve_at_lca_angle(a).wheel_center[2] - target

        # Find every grid interval that brackets the target height. Far from
        # static the linkage can close in a flipped ("upside-down upright")
        # configuration that also reaches the target, so among all brackets
        # we refine the one CLOSEST to the static angle a = 0 — that is the
        # physical branch the suspension actually moves on.
        brackets = []
        for k in range(len(grid) - 1):
            h0, h1 = heights[k], heights[k + 1]
            if np.isnan(h0) or np.isnan(h1):
                continue
            if (h0 - target) * (h1 - target) <= 0.0:
                brackets.append((grid[k], grid[k + 1]))
        if not brackets:
            raise ValueError(f"travel {travel:+.1f} mm is outside the solvable range")
        lo, hi = min(brackets, key=lambda br: min(abs(br[0]), abs(br[1])))
        a = brentq(err, lo, hi, xtol=1e-12)
        return self.solve_at_lca_angle(a)

    # ------------------------------------------------------------------
    # Fast sweep: solve many travels by walking outward from static
    # ------------------------------------------------------------------
    def walk_travels(self, travels, steer: float = 0.0) -> list[CornerState]:
        """Solve a whole list of travels efficiently.

        solve() builds a 161-point bracket grid the first time it is called
        — fine for one slider position, far too slow inside an optimizer
        that evaluates thousands of candidate geometries. This walks the
        travels in order of increasing |travel|, warm-starting each 1D root
        find (secant on the LCA angle) from the angle of the previous,
        neighbouring solution. Because neighbouring travels have
        neighbouring angles, the secant converges in a few iterations and
        never needs the grid; solve() remains the fallback if it strays.

        Returns states in the same order as the input travels. Raises
        ValueError if any travel is unreachable (same contract as solve()).
        """
        travels = np.asarray(travels, dtype=float)
        states: list[CornerState | None] = [None] * len(travels)
        a_prev = {False: 0.0, True: 0.0}   # last angle per side (neg/pos)
        for idx in np.argsort(np.abs(travels), kind="stable"):
            t = travels[idx]
            side = t >= 0.0
            try:
                a = self._local_angle_for_height(self._wc_z0 + t,
                                                 a_prev[side], steer)
                st = self.solve_at_lca_angle(a, steer)
            except ValueError:
                st = self.solve(t, steer)  # robust global fallback
                a = st.lca_angle
            a_prev[side] = a
            states[idx] = st
        return states

    def _local_angle_for_height(self, target: float, a0: float,
                                steer: float = 0.0) -> float:
        """Secant root find for the LCA angle putting the wheel centre at
        `target` height, starting near a0. Raises ValueError if it diverges
        (caller falls back to the bracketed global solve)."""
        f0 = self.solve_at_lca_angle(a0, steer).wheel_center[2] - target
        if abs(f0) < 1e-9:
            return a0
        a1 = a0 + 1e-3
        f1 = self.solve_at_lca_angle(a1, steer).wheel_center[2] - target
        for _ in range(25):
            if abs(f1 - f0) < 1e-15:
                raise ValueError("secant stalled")
            a2 = a1 - f1 * (a1 - a0) / (f1 - f0)
            if abs(a2 - a1) > 0.3:          # jumping toward another branch
                raise ValueError("secant diverged")
            a0, f0 = a1, f1
            a1 = a2
            f1 = self.solve_at_lca_angle(a1, steer).wheel_center[2] - target
            if abs(f1) < 1e-9:
                return a1
        raise ValueError("secant did not converge")

    def _lca_angle_grid(self):
        """Cached coarse map of LCA angle -> wheel-centre height, used only
        to bracket the brentq root find."""
        if self._angle_grid is None:
            grid = np.linspace(-0.8, 0.8, 161)  # ±46 deg, far beyond any real arm
            heights = np.full_like(grid, np.nan)
            for k, a in enumerate(grid):
                try:
                    heights[k] = self.solve_at_lca_angle(a).wheel_center[2]
                except ValueError:
                    pass  # linkage doesn't close out here; leave NaN
            self._angle_grid = (grid, heights)
        return self._angle_grid
