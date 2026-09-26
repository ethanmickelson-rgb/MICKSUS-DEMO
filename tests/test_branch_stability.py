"""Regression: the solver must never swap assembly branches mid-travel.

v1.3.0 bug (reported with reference/EthanTest.MICK): _solve_ubj picked
"the root nearest b = 0" at every pose, so at big travels the folded-over
assembly's root could win and the linkage teleported — links visibly
crossed on screen and the tie rod jumped ~170 mm between neighbouring
travels. The branch is now locked from the static pose at construction.

The original fixture was a heavily hand-edited user file that is not
distributed with this copy. The property under test is general -- no
linkage may teleport between neighbouring travels -- so it now runs over
every example project that ships here, front and rear, across the full
range each file declares. Any .MICK dropped into examples/ is picked up
automatically, so re-adding the original file restores the exact
original coverage.
"""

import os
import unittest

import numpy as np

from suspension_tool.project import load_project
from suspension_tool.solver import DoubleWishboneSolver

EXAMPLES = os.path.join(os.path.dirname(__file__), "..", "examples")


def _projects():
    """Every example project, as (name, ProjectState)."""
    out = []
    for name in sorted(os.listdir(EXAMPLES)):
        if not name.endswith(".MICK"):
            continue
        try:
            out.append((name, load_project(os.path.join(EXAMPLES, name))))
        except Exception as exc:                  # a broken example is a
            raise AssertionError(                 # failure, not a skip
                f"example {name} will not load: {exc}") from exc
    return out


class TestBranchStability(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.projects = _projects()
        if not cls.projects:
            raise unittest.SkipTest("no example projects present")

    def _scan(self, hp, lo, hi):
        s = DoubleWishboneSolver(hp)
        prev = None
        step = 2.0
        for t in np.arange(lo, hi + 1e-9, step):
            st = s.solve(float(t))
            if prev is not None:
                for attr in ("tro", "ubj", "lbj", "wheel_center",
                             "shock_outer"):
                    jump = np.linalg.norm(getattr(st, attr)
                                          - getattr(prev, attr))
                    self.assertLess(
                        jump, 12.0 * step,
                        f"{attr} jumped {jump:.0f} mm at travel {t:.0f}")
            prev = st
        return s

    def _double_wishbone_axles(self):
        """(label, AxleDesign) for every axle this solver applies to."""
        from suspension_tool.geometry import DoubleWishbonePoints
        for name, st in self.projects:
            for end in ("front", "rear"):
                ax = getattr(st, end)
                if ax and isinstance(ax.hardpoints, DoubleWishbonePoints):
                    yield f"{name}:{end}", ax

    def test_full_range_is_continuous(self):
        seen = 0
        for label, ax in self._double_wishbone_axles():
            with self.subTest(axle=label):
                self._scan(ax.hardpoints, -ax.droop_travel, ax.bump_travel)
            seen += 1
        self.assertGreater(seen, 0, "no double-wishbone axle to scan")

    def test_walk_matches_solve(self):
        for label, ax in self._double_wishbone_axles():
            with self.subTest(axle=label):
                hp = ax.hardpoints
                s = DoubleWishboneSolver(hp)
                travels = np.linspace(-ax.droop_travel, ax.bump_travel, 37)
                for st in s.walk_travels(travels):
                    st2 = s.solve(st.travel)
                    np.testing.assert_allclose(st.tro, st2.tro, atol=1e-6)
                    np.testing.assert_allclose(st.ubj, st2.ubj, atol=1e-6)

    def test_static_branch_reproduces_static_pose(self):
        for label, ax in self._double_wishbone_axles():
            with self.subTest(axle=label):
                hp = ax.hardpoints
                s = DoubleWishboneSolver(hp)
                st0 = s.solve(0.0)
                np.testing.assert_allclose(st0.ubj, hp.uca_outer, atol=1e-9)
                np.testing.assert_allclose(st0.tro, hp.tierod_outer,
                                           atol=1e-6)


if __name__ == "__main__":
    unittest.main()
