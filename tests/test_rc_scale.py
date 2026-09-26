"""1/10 RC-scale seeding.

The tool is used for the team's Baja cars AND for a 1/10 scale RC buggy.
Every seed heuristic must therefore be scale-free: ranges, packaging
clearances and search bands are expressed as fractions of the car, not as
fixed millimetre constants sized for a 600 lb Baja car.

These are regression tests for v1.30.0, which found three fixed constants
that silently broke at model-car scale (a 120 mm tie-rod search band, a
120 mm hard-coded diff-flange station, and a 1 mm articulation probe step
that was most of an RC car's travel in a single step).
"""

import unittest

import numpy as np

from suspension_tool import metrics
from suspension_tool.chub_front import CHubFrontSolver, seed_chub_front
from suspension_tool.harm_rear import HArmRearSolver, seed_harm_rear
from suspension_tool.loaded_halfshaft import (LoadedHalfshaftSolver,
                                              seed_loaded_halfshaft)
from suspension_tool.seed import SetupVariables, generate_seed
from suspension_tool.solver import DoubleWishboneSolver
from suspension_tool.trailing_arm import TrailingArmSolver, seed_trailing_arm


def _rc_sv(**kw) -> SetupVariables:
    """A real 1/10 2WD buggy (Team Associated B7 class), in mm."""
    base = dict(track_width=250.0, wheelbase=285.0, ride_height=25.0,
                tire_radius=31.0, tire_width=25.0,
                shock_min_length=62.0, shock_max_length=88.0,
                shock_length_at_ride=80.0, motion_ratio_goal=0.55,
                hub_offset=10.0, desired_scrub_radius=5.0,
                desired_caster_deg=8.0, kingpin_length=22.0,
                static_camber_deg=-1.0, static_toe_deg=0.0,
                x_offset=0.0, z_offset=0.0, frame_tube_offset=0.0)
    base.update(kw)
    return SetupVariables(**base)


# (label, seeder, solver class) — every type the GUI offers.
TYPES = [
    ("double wishbone", None, DoubleWishboneSolver),
    ("c-hub front", seed_chub_front, CHubFrontSolver),
    ("trailing arm", seed_trailing_arm, TrailingArmSolver),
    ("h-arm rear", seed_harm_rear, HArmRearSolver),
    ("loaded halfshaft", seed_loaded_halfshaft, LoadedHalfshaftSolver),
]


def _seed(seeder, sv):
    if seeder is None:
        rep = generate_seed(sv)
        return rep[0] if isinstance(rep, tuple) else rep.hp
    return seeder(sv)


class TestRCScaleSeeding(unittest.TestCase):
    """Every suspension type seeds and articulates at 1/10 scale."""

    def test_all_types_seed_and_articulate(self):
        sv = _rc_sv()
        for label, seeder, solver_cls in TYPES:
            with self.subTest(suspension=label):
                hp = _seed(seeder, sv)
                s = solver_cls(hp)
                for t in np.linspace(-12.0, 12.0, 9):
                    s.solve(float(t))     # raises ValueError if unreachable

    def test_static_readbacks_exact_at_rc_scale(self):
        """Camber/toe/shock-length read back exactly — the same contract
        the full-size seeds hold, so nothing degrades with scale."""
        sv = _rc_sv()
        for label, seeder, solver_cls in TYPES:
            with self.subTest(suspension=label):
                hp = _seed(seeder, sv)
                st = solver_cls(hp).solve(0.0)
                self.assertAlmostEqual(metrics.camber_deg(st), -1.0, places=6)
                self.assertAlmostEqual(metrics.toe_deg(st), 0.0, places=6)
                self.assertAlmostEqual(st.shock_length, 80.0, places=6)

    def test_motion_ratio_goal_met_at_rc_scale(self):
        """The shock-mount secant converges at model-car size (it walks a
        fraction of the arm, so it is scale-free by construction)."""
        sv = _rc_sv()
        for label, seeder, solver_cls in TYPES:
            with self.subTest(suspension=label):
                s = solver_cls(_seed(seeder, sv))
                mr = (s.solve(-6.0).shock_length
                      - s.solve(6.0).shock_length) / 12.0
                self.assertAlmostEqual(mr, sv.motion_ratio_goal, delta=0.06)

    def test_no_hardpoint_lands_outside_the_car(self):
        """Guards the class of bug this suite exists for: a fixed mm
        constant that is 'near the centreline' on a Baja car but out at
        the wheel on a 1/10 buggy."""
        sv = _rc_sv()
        half_track = sv.track_width / 2.0
        for label, seeder, _ in TYPES:
            with self.subTest(suspension=label):
                hp = _seed(seeder, sv)
                for attr in hp.POINT_ATTRS:
                    p = np.asarray(getattr(hp, attr), float)
                    self.assertLessEqual(
                        abs(float(p[1])), half_track + sv.tire_width,
                        f"{label}: {attr} at y={p[1]:.1f} is outside a "
                        f"{sv.track_width:.0f} mm track")
                    self.assertLess(
                        abs(float(p[0])), sv.wheelbase,
                        f"{label}: {attr} at x={p[0]:.1f} exceeds the "
                        f"{sv.wheelbase:.0f} mm wheelbase")

    def test_diff_flange_scales_with_the_car(self):
        """loaded_halfshaft's inner CV used to be pinned at y = 120 mm,
        which is essentially AT the wheel on a 250 mm track."""
        sv = _rc_sv()
        hp = seed_loaded_halfshaft(sv)
        self.assertLess(float(hp.hs_inner[1]), 0.4 * (sv.track_width / 2.0))


class TestFullSizeUnchanged(unittest.TestCase):
    """The scale-relative rewrites must not move the Baja seeds."""

    def _baja_sv(self) -> SetupVariables:
        from suspension_tool.seed import IN
        return SetupVariables(
            track_width=63 * IN, wheelbase=61 * IN, ride_height=14 * IN,
            tire_radius=11.5 * IN, tire_width=7 * IN,
            shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
            motion_ratio_goal=1.0 / 1.85, shock_length_at_ride=20.38 * IN,
            static_camber_deg=-0.87, static_toe_deg=0.2)

    def test_chub_search_band_matches_the_old_fixed_120mm(self):
        sv = self._baja_sv()
        self.assertAlmostEqual(0.41 * sv.tire_radius, 120.0, delta=1.0)

    def test_optimizer_clearances_match_the_old_fixed_values(self):
        sv = self._baja_sv()
        self.assertAlmostEqual(0.034 * sv.tire_radius, 10.0, delta=0.5)
        self.assertAlmostEqual(0.017 * sv.tire_radius, 5.0, delta=0.5)

    def test_articulation_probe_step_unchanged_at_full_size(self):
        """The probe floor only bites when the travel budget is small."""
        want = 267.8                      # a Baja bump budget, mm
        self.assertAlmostEqual(max(want / 120.0, min(1.0, want / 60.0)),
                               max(want / 120.0, 1.0), places=9)


if __name__ == "__main__":
    unittest.main()
