"""H-arm rear suspension: solver correctness (constraint residuals + an
INDEPENDENT scipy full-pose closure), the toe-held invariant (the H's
wide grab base holds toe), passive-steer via grab-line skew, the driven
CV overlay, tweaks, project round-trip, plumbing."""

import dataclasses
import unittest

import numpy as np
from scipy.optimize import least_squares

from suspension_tool import metrics
from suspension_tool.harm_rear import (HArmRearPoints, HArmRearSolver,
                                       seed_harm_rear)
from suspension_tool.seed import IN, SetupVariables


def _sv(**kw) -> SetupVariables:
    base = dict(track_width=61 * IN, wheelbase=61 * IN, ride_height=14 * IN,
                tire_radius=11.5 * IN, tire_width=7 * IN,
                shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
                motion_ratio_goal=1.0 / 1.85, shock_length_at_ride=20.38 * IN,
                static_camber_deg=-0.5, static_toe_deg=0.15)
    base.update(kw)
    return SetupVariables(**base)


def _rod(rvec):
    ang = np.linalg.norm(rvec)
    if ang < 1e-12:
        return np.eye(3)
    ax = rvec / ang
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + np.sin(ang) * k + (1 - np.cos(ang)) * (k @ k)


class TestHArmSolver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sv = _sv()
        cls.hp = seed_harm_rear(cls.sv)
        cls.solver = HArmRearSolver(cls.hp)

    def test_static_readbacks_exact(self):
        st = self.solver.solve(0.0)
        self.assertAlmostEqual(metrics.camber_deg(st), -0.5, places=9)
        self.assertAlmostEqual(metrics.toe_deg(st), 0.15, places=9)
        # no steering knuckle: kingpin metrics NaN by design
        self.assertTrue(np.isnan(metrics.caster_deg(st)))
        self.assertTrue(np.isnan(metrics.kpi_deg(st)))

    def test_constraint_residuals_over_travel(self):
        hp = self.hp
        cam_len = np.linalg.norm(hp.camber_outer - hp.camber_inner)
        pairs = [
            (lambda st: (st.outer_front, hp.arm_inner_front),
             np.linalg.norm(hp.outer_front - hp.arm_inner_front)),
            (lambda st: (st.outer_rear, hp.arm_inner_rear),
             np.linalg.norm(hp.outer_rear - hp.arm_inner_rear)),
            (lambda st: (st.wheel_center, st.outer_front),
             np.linalg.norm(hp.wheel_center - hp.outer_front)),
            (lambda st: (st.wheel_center, st.outer_rear),
             np.linalg.norm(hp.wheel_center - hp.outer_rear)),
            (lambda st: (st.camber_outer, st.wheel_center),
             np.linalg.norm(hp.camber_outer - hp.wheel_center)),
        ]
        for t in np.linspace(-100.0, 220.0, 9):
            st = self.solver.solve(t)
            self.assertAlmostEqual(
                np.linalg.norm(st.camber_outer - hp.camber_inner),
                cam_len, places=6)
            for pts, want in pairs:
                a, b = pts(st)
                self.assertAlmostEqual(np.linalg.norm(a - b), want, places=6)
            self.assertAlmostEqual(st.wheel_center[2],
                                   hp.wheel_center[2] + t, places=6)

    def test_independent_scipy_closure(self):
        """From-scratch verification: the upright is a free 6-DOF body plus
        the arm angle, constraints (two grab points shared with the arm,
        the camber-link length, the target height) satisfied directly by
        scipy — no closed-form closure reused."""
        hp = self.hp
        arm_o = hp.arm_inner_front
        arm_ax = hp.arm_inner_rear - hp.arm_inner_front
        arm_ax = arm_ax / np.linalg.norm(arm_ax)
        cam_len = np.linalg.norm(hp.camber_outer - hp.camber_inner)

        def arm_pt(p, a):
            r = p - arm_o
            return (arm_o + r * np.cos(a) + np.cross(arm_ax, r) * np.sin(a)
                    + arm_ax * np.dot(arm_ax, r) * (1 - np.cos(a)))

        def body(p, pose):
            return (_rod(pose[:3]) @ (p - hp.wheel_center)
                    + hp.wheel_center + pose[3:])

        def res(x, tz):
            a, up = x[0], x[1:7]
            r = []
            for p in (hp.outer_front, hp.outer_rear):
                r.extend(body(p, up) - arm_pt(p, a))
            r.append(np.linalg.norm(body(hp.camber_outer, up)
                                    - hp.camber_inner) - cam_len)
            r.append(body(hp.wheel_center, up)[2] - tz)
            return np.array(r)

        for t in (90.0, -70.0, 180.0):
            st = self.solver.solve(t)
            sol = least_squares(res, np.zeros(7),
                                args=(hp.wheel_center[2] + t,),
                                method="lm", xtol=1e-14, ftol=1e-14)
            self.assertLess(np.max(np.abs(sol.fun)), 1e-7)
            wc = body(hp.wheel_center, sol.x[1:7])
            self.assertLess(np.linalg.norm(wc - st.wheel_center), 1e-4)

    def test_the_h_arm_holds_toe(self):
        """The wide fore-aft grab base holds toe: it barely moves through
        travel while camber does the work (the type's whole point)."""
        sw = metrics.sweep_metrics(self.solver, np.linspace(-100, 200, 31))
        self.assertLess(np.max(np.abs(sw["toe_deg"] - 0.15)), 0.05)
        self.assertGreater(np.ptp(sw["camber_deg"]), 2.0)

    def test_grab_line_skew_is_the_passive_steer_knob(self):
        from suspension_tool.tweaks import harm_params, set_harm
        travels = np.linspace(-80, 160, 25)
        base = np.ptp(metrics.sweep_metrics(
            HArmRearSolver(self.hp), travels)["toe_deg"])
        p = harm_params(self.hp)
        hp2 = set_harm(self.hp, **dict(p, plan_deg=15.0))
        skewed = np.ptp(metrics.sweep_metrics(
            HArmRearSolver(hp2), travels)["toe_deg"])
        self.assertGreater(skewed, 4.0 * base)

    def test_branch_continuity(self):
        sw = metrics.sweep_metrics(self.solver, np.linspace(-110, 240, 71))
        self.assertLess(np.max(np.abs(np.diff(sw["camber_deg"]))), 1.0)
        self.assertLess(np.max(np.abs(np.diff(sw["toe_deg"]))), 0.5)

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

    def test_driven_cv_overlay(self):
        """Driven axle: the standard non-structural CV check works (outer
        CV at the wheel centre, since the type is kingpin-less), and plunge
        is a real number (NOT identically zero — unlike the loaded shaft)."""
        from suspension_tool.halfshaft import (HalfshaftConfig,
                                               halfshaft_curves,
                                               outer_cv_of_hp)
        np.testing.assert_allclose(outer_cv_of_hp(self.hp),
                                   self.hp.wheel_center)
        cfg = HalfshaftConfig(
            enabled=True,
            inner=np.array([self.hp.wheel_center[0], 120.0,
                            self.hp.wheel_center[2] + 40.0]))
        states = self.solver.walk_travels(np.linspace(-100, 200, 21))
        curves = halfshaft_curves(cfg, self.hp, states)
        self.assertTrue(np.all(np.isfinite(curves["cv_max_deg"])))
        self.assertGreater(np.ptp(curves["plunge_mm"]), 1.0)  # it DOES plunge


class TestHArmPlumbing(unittest.TestCase):
    def test_registry(self):
        from suspension_tool.suspension_types import (SUSPENSION_TYPES,
                                                      solver_for)
        hp = seed_harm_rear(_sv())
        self.assertIs(solver_for(hp), HArmRearSolver)
        self.assertIs(SUSPENSION_TYPES["H-arm (rear)"], HArmRearSolver)

    def test_project_roundtrip(self):
        from suspension_tool.project import _hp_from_dict, _hp_to_dict
        hp = seed_harm_rear(_sv())
        d = _hp_to_dict(hp)
        self.assertEqual(d["suspension_type"], "harm_rear")
        hp2 = _hp_from_dict(d)
        self.assertIsInstance(hp2, HArmRearPoints)
        for attr in HArmRearPoints.POINT_ATTRS:
            np.testing.assert_allclose(getattr(hp2, attr),
                                       getattr(hp, attr), atol=1e-9)

    def test_tweaks_roundtrip(self):
        from suspension_tool.tweaks import harm_params, set_harm
        hp = seed_harm_rear(_sv())
        hp2 = set_harm(hp, **harm_params(hp))
        for attr in HArmRearPoints.POINT_ATTRS:
            np.testing.assert_allclose(getattr(hp2, attr),
                                       getattr(hp, attr), atol=1e-9)

    def test_optimize_groups(self):
        from suspension_tool.optimize import free_groups_for
        groups = free_groups_for(seed_harm_rear(_sv()))
        self.assertIn("camber_inner", groups)
        self.assertIn("arm_inner", groups)

    def test_numeric_ic_dispatch(self):
        hp = seed_harm_rear(_sv())
        solver = HArmRearSolver(hp)
        self.assertFalse(hasattr(hp, "uca_inner_front"))
        rc = metrics.roll_center_height_mm(solver, solver.solve(0.0))
        self.assertTrue(np.isfinite(rc))


if __name__ == "__main__":
    unittest.main()
