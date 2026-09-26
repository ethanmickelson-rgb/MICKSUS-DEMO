"""Loaded-halfshaft rear suspension: solver correctness (constraint
residuals + an INDEPENDENT scipy full-pose closure), the zero-plunge
invariant (a loaded shaft cannot plunge), passive-steer via pin skew,
tweaks, project round-trip, plumbing."""

import dataclasses
import unittest

import numpy as np
from scipy.optimize import least_squares

from suspension_tool import metrics
from suspension_tool.loaded_halfshaft import (LoadedHalfshaftPoints,
                                              LoadedHalfshaftSolver,
                                              seed_loaded_halfshaft)
from suspension_tool.seed import IN, SetupVariables


def _sv(**kw) -> SetupVariables:
    base = dict(track_width=61 * IN, wheelbase=61 * IN, ride_height=14 * IN,
                tire_radius=11.5 * IN, tire_width=7 * IN,
                shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
                motion_ratio_goal=1.0 / 1.85, shock_length_at_ride=20.38 * IN,
                static_camber_deg=-0.5, static_toe_deg=0.1)
    base.update(kw)
    return SetupVariables(**base)


def _rodrigues(rvec):
    ang = np.linalg.norm(rvec)
    if ang < 1e-12:
        return np.eye(3)
    ax = rvec / ang
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + np.sin(ang) * k + (1 - np.cos(ang)) * (k @ k)


class TestLoadedHalfshaftSolver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sv = _sv()
        cls.hp = seed_loaded_halfshaft(cls.sv)
        cls.solver = LoadedHalfshaftSolver(cls.hp)

    def test_static_readbacks_exact(self):
        st = self.solver.solve(0.0)
        self.assertAlmostEqual(metrics.camber_deg(st), -0.5, places=9)
        self.assertAlmostEqual(metrics.toe_deg(st), 0.1, places=9)
        # no steering knuckle: kingpin metrics NaN by design
        self.assertTrue(np.isnan(metrics.caster_deg(st)))
        self.assertTrue(np.isnan(metrics.scrub_radius_mm(st)))

    def test_shaft_length_is_the_constraint(self):
        """THE defining property: the loaded shaft's length never changes
        — it is the closure, so plunge is identically zero."""
        hp = self.hp
        L = np.linalg.norm(hp.hs_outer - hp.hs_inner)
        for t in np.linspace(-100.0, 220.0, 12):
            st = self.solver.solve(t)
            self.assertAlmostEqual(np.linalg.norm(st.hs_outer - hp.hs_inner),
                                   L, places=6)

    def test_zero_plunge_through_halfshaft_module(self):
        """halfshaft.py prefers the pose's structural hs_outer, so the CV
        readout's plunge channel reads EXACTLY zero (the built-in sanity
        check) while CV angles stay real."""
        from suspension_tool.halfshaft import (HalfshaftConfig,
                                               halfshaft_curves,
                                               outer_cv_of_hp)
        np.testing.assert_allclose(outer_cv_of_hp(self.hp), self.hp.hs_outer)
        cfg = HalfshaftConfig(enabled=True, inner=self.hp.hs_inner.copy())
        states = self.solver.walk_travels(np.linspace(-100, 220, 25))
        curves = halfshaft_curves(cfg, self.hp, states)
        self.assertLess(np.max(np.abs(curves["plunge_mm"])), 1e-6)
        self.assertTrue(np.all(np.isfinite(curves["cv_max_deg"])))

    def test_constraint_residuals_over_travel(self):
        hp = self.hp
        pairs = [
            (lambda st: (st.hinge_front, st.hinge_rear),
             np.linalg.norm(hp.hinge_front - hp.hinge_rear)),
            (lambda st: (st.wheel_center, st.hinge_front),
             np.linalg.norm(hp.wheel_center - hp.hinge_front)),
            (lambda st: (st.hs_outer, st.wheel_center),
             np.linalg.norm(hp.hs_outer - hp.wheel_center)),
        ]
        for t in np.linspace(-100.0, 220.0, 9):
            st = self.solver.solve(t)
            for pts, want in pairs:
                a, b = pts(st)
                self.assertAlmostEqual(np.linalg.norm(a - b), want, places=6)
            self.assertAlmostEqual(st.wheel_center[2],
                                   hp.wheel_center[2] + t, places=6)

    def test_independent_scipy_closure(self):
        """From-scratch verification: knuckle as a free 6-DOF body plus
        the arm angle, constraints satisfied directly by scipy — the
        wheel centre must land where the closed-form solver puts it."""
        hp = self.hp
        arm_o = hp.arm_inner_front
        arm_ax = hp.arm_inner_rear - hp.arm_inner_front
        arm_ax = arm_ax / np.linalg.norm(arm_ax)
        L = np.linalg.norm(hp.hs_outer - hp.hs_inner)

        def arm_pt(p, a):
            r = p - arm_o
            return (arm_o + r * np.cos(a) + np.cross(arm_ax, r) * np.sin(a)
                    + arm_ax * np.dot(arm_ax, r) * (1 - np.cos(a)))

        def body_pt(p, pose):
            return (_rodrigues(pose[:3]) @ (p - hp.wheel_center)
                    + hp.wheel_center + pose[3:])

        def residuals(x, target_z):
            a, knuckle = x[0], x[1:7]
            res = []
            for p in (hp.hinge_front, hp.hinge_rear):   # shared pin points
                res.extend(body_pt(p, knuckle) - arm_pt(p, a))
            res.append(np.linalg.norm(body_pt(hp.hs_outer, knuckle)
                                      - hp.hs_inner) - L)
            res.append(body_pt(hp.wheel_center, knuckle)[2] - target_z)
            return np.array(res)

        for t in (90.0, -70.0, 180.0):
            st = self.solver.solve(t)
            sol = least_squares(residuals, np.zeros(7),
                                args=(hp.wheel_center[2] + t,),
                                method="lm", xtol=1e-14, ftol=1e-14)
            self.assertLess(np.max(np.abs(sol.fun)), 1e-7)
            wc = body_pt(hp.wheel_center, sol.x[1:7])
            self.assertLess(np.linalg.norm(wc - st.wheel_center), 1e-4)

    def test_straight_pin_keeps_toe(self):
        """A pure fore-aft bushing pin + fore-aft arm axis constrain toe:
        it must stay put through travel (camber does all the moving)."""
        sw = metrics.sweep_metrics(self.solver, np.linspace(-100, 200, 31))
        self.assertLess(np.max(np.abs(sw["toe_deg"] - 0.1)), 0.05)
        # ...while camber genuinely changes (the shaft is the camber link)
        self.assertGreater(np.ptp(sw["camber_deg"]), 2.0)

    def test_pin_skew_is_the_passive_steer_knob(self):
        """Skewing the knuckle pin couples the knuckle's relative swing
        into toe. The effect is GENTLE (the knuckle only rotates a little
        relative to the arm), so assert the comparative signal: several
        times the straight-pin toe drift."""
        from suspension_tool.tweaks import loaded_hs_params, set_loaded_hs
        travels = np.linspace(-80, 160, 25)
        base = np.ptp(metrics.sweep_metrics(
            LoadedHalfshaftSolver(self.hp), travels)["toe_deg"])
        p = loaded_hs_params(self.hp)
        hp2 = set_loaded_hs(self.hp, **dict(p, plan_deg=15.0))
        skewed = np.ptp(metrics.sweep_metrics(
            LoadedHalfshaftSolver(hp2), travels)["toe_deg"])
        self.assertGreater(skewed, 4.0 * base)
        self.assertGreater(skewed, 0.02)

    def test_diff_height_changes_camber_gain(self):
        """Raising the diff flange tilts the structural shaft — the
        camber-curve lever a designer would reach for."""
        def gain(hp):
            s = LoadedHalfshaftSolver(hp)
            return (metrics.camber_deg(s.solve(20.0))
                    - metrics.camber_deg(s.solve(-20.0))) / 40.0
        hi = dataclasses.replace(
            self.hp, hs_inner=self.hp.hs_inner + np.array([0, 0, 40.0]))
        self.assertGreater(abs(gain(hi) - gain(self.hp)), 5e-3)

    def test_seed_quality(self):
        ts = np.linspace(-40, 40, 21)
        sw = metrics.sweep_metrics(self.solver, ts)
        i0 = int(np.argmin(np.abs(ts)))
        self.assertLess(abs(sw["motion_ratio"][i0] - 1.0 / 1.85), 0.02)
        bump = 1.85 * (self.sv.shock_length_at_ride
                       - self.sv.shock_min_length)
        droop = 1.85 * (self.sv.shock_max_length
                        - self.sv.shock_length_at_ride)
        self.solver.solve(bump)
        self.solver.solve(-droop)


class TestLoadedHalfshaftPlumbing(unittest.TestCase):
    def test_registry(self):
        from suspension_tool.suspension_types import (SUSPENSION_TYPES,
                                                      solver_for)
        hp = seed_loaded_halfshaft(_sv())
        self.assertIs(solver_for(hp), LoadedHalfshaftSolver)
        self.assertIs(SUSPENSION_TYPES["Loaded halfshaft (rear)"],
                      LoadedHalfshaftSolver)

    def test_project_roundtrip(self):
        from suspension_tool.project import _hp_from_dict, _hp_to_dict
        hp = seed_loaded_halfshaft(_sv())
        d = _hp_to_dict(hp)
        self.assertEqual(d["suspension_type"], "loaded_halfshaft")
        hp2 = _hp_from_dict(d)
        self.assertIsInstance(hp2, LoadedHalfshaftPoints)
        for attr in LoadedHalfshaftPoints.POINT_ATTRS:
            np.testing.assert_allclose(getattr(hp2, attr),
                                       getattr(hp, attr), atol=1e-9)

    def test_tweaks_roundtrip(self):
        from suspension_tool.tweaks import loaded_hs_params, set_loaded_hs
        hp = seed_loaded_halfshaft(_sv())
        p = loaded_hs_params(hp)
        hp2 = set_loaded_hs(hp, **p)
        for attr in LoadedHalfshaftPoints.POINT_ATTRS:
            np.testing.assert_allclose(getattr(hp2, attr),
                                       getattr(hp, attr), atol=1e-9)

    def test_rigid_moves_carry_the_diff_flange(self):
        """Ride-height / corner-x are rigid whole-corner moves: hs_inner
        (the diff flange) must ride along, keeping the shaft geometry."""
        from suspension_tool.tweaks import ride_height, set_ride_height
        hp = seed_loaded_halfshaft(_sv())
        L = np.linalg.norm(hp.hs_outer - hp.hs_inner)
        hp2 = set_ride_height(hp, ride_height(hp) + 20.0)
        self.assertAlmostEqual(
            np.linalg.norm(hp2.hs_outer - hp2.hs_inner), L, places=9)

    def test_numeric_ic_dispatch(self):
        hp = seed_loaded_halfshaft(_sv())
        solver = LoadedHalfshaftSolver(hp)
        self.assertFalse(hasattr(hp, "uca_inner_front"))
        rc = metrics.roll_center_height_mm(solver, solver.solve(0.0))
        self.assertTrue(np.isfinite(rc))


if __name__ == "__main__":
    unittest.main()
