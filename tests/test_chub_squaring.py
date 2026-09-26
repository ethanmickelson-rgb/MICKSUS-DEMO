"""C-hub squareness, kickup and the caster pill (v1.34).

A C-hub corner has ONE chassis bushing axis and one carrier (hinge) pin,
and both are meant to be normal to the same 2D design sketch -- the direct
analogue of a double wishbone's two arm axes. Three things are pinned here:

  * changing the kickup keeps the corner SQUARE. Before v1.34 the kickup
    tweak rotated the arm axis and left the pin behind, so 25 deg of kickup
    meant 25 deg of misalignment and the corner went strange as soon as the
    kickup was touched.
  * Re-square repairs a hand-skewed corner, as it always has for the DW.
    It used to refuse non-wishbones outright, so there was no way back.
  * the caster pill (the eccentric in the C-block that turns the block on
    its pin) is an independent knob, and total caster is kickup + pill --
    the same rule the RC world states as "caster = kick-up + caster-block
    angle". Re-square must PRESERVE a pill that has been dialled in.
"""

import dataclasses
import unittest

import numpy as np

from suspension_tool import metrics, tweaks
from suspension_tool.chub_front import CHubFrontSolver, seed_chub_front
from suspension_tool.geometry import carrier_planarity, square_carrier_axes
from suspension_tool.harm_rear import seed_harm_rear
from suspension_tool.loaded_halfshaft import seed_loaded_halfshaft
from suspension_tool.seed import IN, SetupVariables


def _sv(**kw) -> SetupVariables:
    base = dict(track_width=63 * IN, wheelbase=61 * IN, ride_height=14 * IN,
                tire_radius=11.5 * IN, tire_width=7 * IN,
                shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
                motion_ratio_goal=1.0 / 1.85,
                shock_length_at_ride=20.38 * IN,
                static_camber_deg=-0.87, static_toe_deg=0.2,
                desired_caster_deg=8.0)
    base.update(kw)
    return SetupVariables(**base)


class TestKickupKeepsTheCornerSquare(unittest.TestCase):
    """The regression that started this: the kickup tweak must move the
    carrier pin WITH the arm, exactly as the DW tilts both bushing axes."""

    def test_kickup_leaves_no_misalignment(self):
        hp = seed_chub_front(_sv())
        for k in (0.0, 5.0, 10.0, 20.0, 30.0):
            with self.subTest(kickup=k):
                out = tweaks.set_arm_axis_kickup(hp, k)
                pl = carrier_planarity(out)
                self.assertAlmostEqual(pl["sketch_kickup_deg"], k, places=6)
                self.assertLess(pl["axis_misalign_deg"], 1e-9,
                                "arm and carrier pin came out of square")
                CHubFrontSolver(out).solve(0.0)     # still assembles

    def test_the_other_carrier_types_stay_square_too(self):
        sv = _sv()
        for seeder in (seed_loaded_halfshaft, seed_harm_rear):
            hp = seeder(sv)
            for k in (10.0, 25.0):
                with self.subTest(suspension=type(hp).__name__, kickup=k):
                    pl = carrier_planarity(tweaks.set_arm_axis_kickup(hp, k))
                    self.assertLess(pl["axis_misalign_deg"], 1e-9)

    def test_kickup_does_not_disturb_the_alignment_settings(self):
        """Only the bushing axis and the pin move — camber and toe are
        properties of the knuckle and must be untouched."""
        hp = seed_chub_front(_sv())
        base = CHubFrontSolver(hp).solve(0.0)
        for k in (10.0, 25.0):
            st = CHubFrontSolver(tweaks.set_arm_axis_kickup(hp, k)).solve(0.0)
            self.assertAlmostEqual(metrics.camber_deg(st),
                                   metrics.camber_deg(base), places=9)
            self.assertAlmostEqual(metrics.toe_deg(st),
                                   metrics.toe_deg(base), places=9)


class TestCasterPill(unittest.TestCase):
    """total caster = chassis kickup + caster pill."""

    def _at(self, kickup, pill):
        hp = tweaks.set_arm_axis_kickup(seed_chub_front(_sv()), kickup)
        return tweaks.set_chub_pill_caster(hp, pill)

    def test_caster_is_kickup_plus_pill(self):
        for kickup in (0.0, 10.0, 20.0):
            # the real part comes in 0 / +-2.5 / +-5 deg inserts
            for pill in (-5.0, -2.5, 0.0, 2.5, 5.0):
                with self.subTest(kickup=kickup, pill=pill):
                    hp = self._at(kickup, pill)
                    st = CHubFrontSolver(hp).solve(0.0)
                    self.assertAlmostEqual(metrics.caster_deg(st),
                                           kickup + pill, places=6)
                    self.assertAlmostEqual(tweaks.chub_pill_caster_deg(hp),
                                           pill, places=6)

    def test_the_pill_turns_the_block_only(self):
        """It is the block rotating on its pin: the arm, the pin and every
        chassis point must not move, and the corner stays square."""
        hp = tweaks.set_arm_axis_kickup(seed_chub_front(_sv()), 15.0)
        out = tweaks.set_chub_pill_caster(hp, 5.0)
        for attr in ("arm_inner_front", "arm_inner_rear", "hinge_front",
                     "hinge_rear", "camber_inner", "tierod_inner",
                     "shock_inner"):
            np.testing.assert_allclose(getattr(out, attr), getattr(hp, attr),
                                       atol=1e-9, err_msg=attr)
        self.assertLess(carrier_planarity(out)["axis_misalign_deg"], 1e-9)

    def test_pill_does_not_disturb_camber_or_toe(self):
        hp = tweaks.set_arm_axis_kickup(seed_chub_front(_sv()), 15.0)
        base = CHubFrontSolver(hp).solve(0.0)
        st = CHubFrontSolver(tweaks.set_chub_pill_caster(hp, 5.0)).solve(0.0)
        self.assertAlmostEqual(metrics.camber_deg(st),
                               metrics.camber_deg(base), places=6)
        self.assertAlmostEqual(metrics.toe_deg(st),
                               metrics.toe_deg(base), places=6)


class TestResquare(unittest.TestCase):

    def _wrecked(self):
        hp = tweaks.set_chub_pill_caster(
            tweaks.set_arm_axis_kickup(seed_chub_front(_sv()), 20.0), 2.5)
        bad = dataclasses.replace(
            hp,
            arm_inner_front=hp.arm_inner_front + np.array([0.0, 12.0, 7.0]),
            hinge_front=hp.hinge_front + np.array([0.0, -5.0, 3.0]))
        return hp, bad

    def test_it_squares_the_arm_and_pin_at_the_design_yaw(self):
        _, bad = self._wrecked()
        self.assertGreater(carrier_planarity(bad)["axis_misalign_deg"], 1.0)
        for yaw in (0.0, 7.5):
            with self.subTest(yaw=yaw):
                fixed = square_carrier_axes(bad, yaw_deg=yaw)
                pl = carrier_planarity(fixed)
                self.assertLess(pl["axis_misalign_deg"], 1e-6)
                self.assertAlmostEqual(pl["axis_yaw_deg"], yaw, places=6)
                CHubFrontSolver(fixed).solve(0.0)

    def test_it_preserves_the_caster_pill(self):
        """The user's call: Re-square repairs drift without undoing a
        caster setting they chose. Carrying the block through the pin's
        rotation alone is NOT enough -- that rotation has a fore/aft
        component and drifts caster about a degree."""
        _, bad = self._wrecked()
        pill_before = tweaks.chub_pill_caster_deg(bad)
        caster_before = metrics.caster_deg(CHubFrontSolver(bad).solve(0.0))
        fixed = square_carrier_axes(bad, yaw_deg=0.0)
        self.assertAlmostEqual(tweaks.chub_pill_caster_deg(fixed),
                               pill_before, places=6)
        self.assertAlmostEqual(
            metrics.caster_deg(CHubFrontSolver(fixed).solve(0.0)),
            caster_before, places=6)

    def test_it_keeps_the_kickup(self):
        _, bad = self._wrecked()
        k = carrier_planarity(bad)["sketch_kickup_deg"]
        self.assertAlmostEqual(
            carrier_planarity(square_carrier_axes(bad, 0.0))
            ["sketch_kickup_deg"], k, places=6)

    def test_squaring_an_already_square_corner_is_a_no_op(self):
        hp = tweaks.set_arm_axis_kickup(seed_chub_front(_sv()), 12.0)
        out = square_carrier_axes(hp, yaw_deg=0.0)
        for attr in type(hp).POINT_ATTRS:
            np.testing.assert_allclose(getattr(out, attr),
                                       getattr(hp, attr), atol=1e-7,
                                       err_msg=attr)


if __name__ == "__main__":
    unittest.main()
