"""The sketch-planarity guard: a single shared 2D design sketch exists
iff the UCA and LCA bushing axes are parallel. Seeds satisfy it exactly
(caster offsets of the ball joints are legitimate and do NOT violate it);
the metric flags a bushing edit that breaks parallelism; and the
optimizer auto-repairs (re-squares) the axes on entry.
"""

import dataclasses
import unittest

import numpy as np

from suspension_tool.geometry import (planarize_kingpin, sketch_planarity,
                                      square_bushing_axes)
from suspension_tool.optimize import Goal, optimize
from suspension_tool.seed import SetupVariables, generate_seed, IN


def _setup() -> SetupVariables:
    return SetupVariables(
        track_width=63.0 * IN, wheelbase=61.0 * IN, ride_height=14.0 * IN,
        tire_radius=11.5 * IN, tire_width=7.0 * IN,
        shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
        shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85,
        kickup_deg=10.0)


def _bend(hp, dz=8.0):
    """Tilt the UCA bushing axis by moving one bushing up: the classic
    hand-edit that silently kills the shared sketch."""
    return dataclasses.replace(
        hp, uca_inner_front=hp.uca_inner_front + np.array([0.0, 0.0, dz]))


class TestSketchPlanarity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hp, cls.report = generate_seed(_setup())

    def test_seed_is_single_sketch(self):
        p = sketch_planarity(self.hp)
        self.assertLess(p["axis_misalign_deg"], 1e-9)
        # the sketch plane the axes imply IS the requested kickup
        self.assertAlmostEqual(p["sketch_kickup_deg"], 10.0, places=6)

    def test_caster_offsets_do_not_violate(self):
        # Ball joints deliberately sit fore/aft of the sketch (caster);
        # sliding one further along the axis normal must stay legal.
        n = np.array([np.cos(np.radians(10.0)), 0.0, np.sin(np.radians(10.0))])
        slid = dataclasses.replace(self.hp,
                                   uca_outer=self.hp.uca_outer + 15.0 * n)
        self.assertLess(sketch_planarity(slid)["axis_misalign_deg"], 1e-9)

    def test_metric_sees_bent_bushing_axis(self):
        bent = _bend(self.hp)
        self.assertGreater(sketch_planarity(bent)["axis_misalign_deg"], 1.0)

    def test_square_bushing_axes_repairs_minimally(self):
        bent = _bend(self.hp)
        fixed = square_bushing_axes(bent)
        self.assertLess(sketch_planarity(fixed)["axis_misalign_deg"], 1e-9)
        for attrs in (("uca_inner_front", "uca_inner_rear"),
                      ("lca_inner_front", "lca_inner_rear")):
            f0, r0 = getattr(bent, attrs[0]), getattr(bent, attrs[1])
            f1, r1 = getattr(fixed, attrs[0]), getattr(fixed, attrs[1])
            # midpoint and spread length preserved
            np.testing.assert_allclose((f0 + r0) / 2, (f1 + r1) / 2,
                                       atol=1e-9)
            self.assertAlmostEqual(np.linalg.norm(f0 - r0),
                                   np.linalg.norm(f1 - r1), places=9)
        # ball joints untouched
        np.testing.assert_allclose(fixed.uca_outer, bent.uca_outer)
        np.testing.assert_allclose(fixed.lca_outer, bent.lca_outer)

    def test_yaw_metric_and_repair(self):
        # Yaw the whole inboard picture: rotate every bushing about Z.
        # The axes stay PARALLEL (misalign 0) but lean out of the
        # side-view plane — Ethan's "you can see the other side of the
        # kingpin in side view" defect (his file: 3.7 deg F / 19.8 deg R).
        th = np.radians(12.0)
        rz = np.array([[np.cos(th), -np.sin(th), 0.0],
                       [np.sin(th), np.cos(th), 0.0],
                       [0.0, 0.0, 1.0]])
        c = (self.hp.uca_inner_front + self.hp.uca_inner_rear) / 2
        rot = {a: c + rz @ (getattr(self.hp, a) - c)
               for a in ("uca_inner_front", "uca_inner_rear",
                         "lca_inner_front", "lca_inner_rear")}
        yawed = dataclasses.replace(self.hp, **rot)
        p = sketch_planarity(yawed)
        self.assertLess(p["axis_misalign_deg"], 1e-6)
        # v1.9 signed plan-view metric: rotating the axes 12 deg about Z
        # reads EXACTLY +12 (and the kickup elevation is yaw-invariant)
        self.assertAlmostEqual(p["axis_yaw_deg"], 12.0, places=9)
        fixed = square_bushing_axes(yawed)
        pf = sketch_planarity(fixed)
        self.assertLess(pf["axis_yaw_deg"], 1e-9)
        self.assertLess(pf["axis_misalign_deg"], 1e-9)
        # kickup preserved, midpoints preserved, balljoints untouched
        self.assertAlmostEqual(pf["sketch_kickup_deg"],
                               p["sketch_kickup_deg"], places=6)
        for attrs in (("uca_inner_front", "uca_inner_rear"),
                      ("lca_inner_front", "lca_inner_rear")):
            f0, r0 = getattr(yawed, attrs[0]), getattr(yawed, attrs[1])
            f1, r1 = getattr(fixed, attrs[0]), getattr(fixed, attrs[1])
            np.testing.assert_allclose((f0 + r0) / 2, (f1 + r1) / 2,
                                       atol=1e-9)
            self.assertAlmostEqual(np.linalg.norm(f0 - r0),
                                   np.linalg.norm(f1 - r1), places=9)
        np.testing.assert_allclose(fixed.uca_outer, yawed.uca_outer)

    def test_seed_has_zero_yaw(self):
        self.assertLess(abs(sketch_planarity(self.hp)["axis_yaw_deg"]),
                        1e-9)

    def test_planarize_kingpin_zeros_twist_keeps_spindle(self):
        # Twist the knuckle out of the sketch: move the UBJ sideways so
        # the kingpin leaves the plane.
        import dataclasses
        from suspension_tool.metrics import camber_deg, toe_deg
        from suspension_tool.solver import DoubleWishboneSolver
        twisted = dataclasses.replace(
            self.hp, uca_outer=self.hp.uca_outer + np.array([40.0, 0, 0]))
        off0 = sketch_planarity(twisted)["kingpin_off_plane_deg"]
        self.assertGreater(off0, 3.0)
        fixed = planarize_kingpin(twisted)
        self.assertLess(sketch_planarity(fixed)["kingpin_off_plane_deg"],
                        1e-6)
        # lower ball joint + kingpin length preserved
        np.testing.assert_allclose(fixed.lca_outer, twisted.lca_outer)
        self.assertAlmostEqual(
            np.linalg.norm(fixed.uca_outer - fixed.lca_outer),
            np.linalg.norm(twisted.uca_outer - twisted.lca_outer), places=6)
        # camber/toe (the stored spindle) untouched
        s0 = DoubleWishboneSolver(twisted).solve(0.0)
        s1 = DoubleWishboneSolver(fixed).solve(0.0)
        self.assertAlmostEqual(camber_deg(s0), camber_deg(s1), places=6)
        self.assertAlmostEqual(toe_deg(s0), toe_deg(s1), places=6)

    def test_seed_kingpin_already_planar_after_resquare(self):
        # a squared seed at its design yaw, planarized, reads ~0 twist
        fixed = planarize_kingpin(square_bushing_axes(self.hp, 0.0))
        self.assertLess(sketch_planarity(fixed)["kingpin_off_plane_deg"],
                        1e-6)

    def test_twisted_plane_seed_and_repair(self):
        # v1.9: a seed generated with sketch_yaw_deg carries EXACTLY that
        # plan twist, is misalign-free, keeps its kickup elevation, and
        # the knuckle stays straight (BJ caster offsets are pure X).
        from suspension_tool.seed import generate_seed
        import dataclasses as _dc
        # independent caster so the ball joints aren't planarized (this
        # test is about the bushing twist, not the kingpin fix)
        sv = _dc.replace(_setup(), independent_caster=True)
        hp20, _ = generate_seed(_dc.replace(sv, sketch_yaw_deg=20.0))
        p = sketch_planarity(hp20)
        self.assertLess(p["axis_misalign_deg"], 1e-9)
        self.assertAlmostEqual(p["axis_yaw_deg"], 20.0, places=9)
        self.assertAlmostEqual(p["sketch_kickup_deg"], 10.0, places=6)
        base, _ = generate_seed(sv)
        np.testing.assert_allclose(hp20.uca_outer, base.uca_outer)
        np.testing.assert_allclose(hp20.lca_outer, base.lca_outer)
        # re-square AT the design angle is a no-op...
        same = square_bushing_axes(hp20, yaw_deg=20.0)
        for a in ("uca_inner_front", "uca_inner_rear",
                  "lca_inner_front", "lca_inner_rear"):
            np.testing.assert_allclose(getattr(same, a), getattr(hp20, a),
                                       atol=1e-9)
        # ...and re-squaring a FLAT seed TO 20 deg reproduces the twisted
        # seed's bushings exactly (same midpoints/spreads by construction)
        twisted = square_bushing_axes(base, yaw_deg=20.0)
        for a in ("uca_inner_front", "uca_inner_rear",
                  "lca_inner_front", "lca_inner_rear"):
            np.testing.assert_allclose(getattr(twisted, a),
                                       getattr(hp20, a), atol=1e-6)

    def test_ethantest_rear_resquare_at_design_yaw_barely_moves(self):
        # Ethan's acceptance test: his rear frame tubes run ~20 deg; at
        # that design yaw a re-square only absorbs the 1.6 deg misalign
        # (bushings move a few mm), while squaring to zero would drag
        # them >100 mm.
        import os
        path = os.path.join(os.path.dirname(__file__), "..",
                            "reference", "EthanTest.MICK")
        if not os.path.exists(path):
            self.skipTest("reference file not present")
        from suspension_tool.project import load_project
        hp = load_project(path).rear.hardpoints
        yaw = sketch_planarity(hp)["axis_yaw_deg"]
        self.assertGreater(yaw, 15.0)
        atts = ("uca_inner_front", "uca_inner_rear",
                "lca_inner_front", "lca_inner_rear")
        at_design = square_bushing_axes(hp, yaw_deg=yaw)
        move = max(np.linalg.norm(getattr(at_design, a) - getattr(hp, a))
                   for a in atts)
        self.assertLess(move, 10.0)
        flat = square_bushing_axes(hp, yaw_deg=0.0)
        move0 = max(np.linalg.norm(getattr(flat, a) - getattr(hp, a))
                    for a in atts)
        self.assertGreater(move0, 100.0)

    def test_optimizer_repairs_on_entry(self):
        bent = _bend(self.hp)
        travels = np.linspace(-60.0, 60.0, 7)
        res = optimize(bent, [Goal("bump_steer")], ["tierod_inner"],
                       travels, box=40.0, max_nfev=80)
        # whatever the goal outcome, the returned geometry is single-sketch
        self.assertLess(
            sketch_planarity(res.hp_after)["axis_misalign_deg"], 1e-9)


class TestEnforceKnucklePlanes(unittest.TestCase):
    """Knuckle-consistency convention: the tire spin axis lies in the
    (ball joints + wheel centre) plane, and the steering-arm plane sits at
    a set angle to it. Camber / toe / track / ride height are preserved."""

    def _measure(self, hp):
        from suspension_tool.geometry import static_spindle
        ubj, lbj = hp.uca_outer, hp.lca_outer
        wc, tro = hp.wheel_center, hp.tierod_outer
        king = (ubj - lbj) / np.linalg.norm(ubj - lbj)
        s = static_spindle(hp)
        s = s / np.linalg.norm(s)
        nA = np.cross(king, wc - lbj)
        nA = nA / np.linalg.norm(nA)
        nB = np.cross(king, tro - lbj)
        nB = nB / np.linalg.norm(nB)
        return (np.degrees(np.arcsin(abs(np.dot(s, nA)))),          # spin oop
                np.degrees(np.arccos(np.clip(abs(np.dot(nA, nB)), -1, 1))))

    def test_constraints_hold_and_camber_toe_track_kept(self):
        from suspension_tool.geometry import enforce_knuckle_planes
        from suspension_tool.metrics import (caster_deg, camber_deg,
                                             scrub_radius_mm, toe_deg)
        from suspension_tool.solver import DoubleWishboneSolver
        # start from a seed, then break the convention by hand so there is
        # something to enforce (nudge the wheel centre off the tire plane)
        hp0, _ = generate_seed(_setup())
        hp0 = dataclasses.replace(
            hp0, wheel_center=hp0.wheel_center + np.array([25.0, 0, 0]))
        oop0, dih0 = self._measure(hp0)
        self.assertGreater(oop0, 1.0)          # genuinely off-plane now
        s0 = DoubleWishboneSolver(hp0).solve(0.0)
        hp1 = enforce_knuckle_planes(hp0, 90.0)
        oop1, dih1 = self._measure(hp1)
        self.assertAlmostEqual(oop1, 0.0, places=4)     # spin axis in plane
        self.assertAlmostEqual(dih1, 90.0, places=4)    # steering arm 90 deg
        s1 = DoubleWishboneSolver(hp1).solve(0.0)
        # camber, toe, caster, track (y) and ride height (z) preserved
        self.assertAlmostEqual(camber_deg(s1), camber_deg(s0), places=4)
        self.assertAlmostEqual(toe_deg(s1), toe_deg(s0), places=4)
        self.assertAlmostEqual(caster_deg(s1), caster_deg(s0), places=4)
        self.assertAlmostEqual(hp1.wheel_center[1], hp0.wheel_center[1],
                               places=9)                # track
        self.assertAlmostEqual(hp1.wheel_center[2], hp0.wheel_center[2],
                               places=9)                # ride height
        self.assertLess(abs(scrub_radius_mm(s1) - scrub_radius_mm(s0)), 2.0)

    def test_idempotent_and_variable_angle(self):
        from suspension_tool.geometry import enforce_knuckle_planes
        hp, _ = generate_seed(_setup())
        # a fresh seed is already enforced -> re-enforcing is a no-op
        hp1 = enforce_knuckle_planes(hp, 90.0)
        for a in hp.POINT_ATTRS:
            np.testing.assert_allclose(getattr(hp1, a), getattr(hp, a),
                                       atol=1e-6)
        # the angle is a working variable
        hp80 = enforce_knuckle_planes(hp, 80.0)
        self.assertAlmostEqual(self._measure(hp80)[1], 80.0, places=4)


if __name__ == "__main__":
    unittest.main()
