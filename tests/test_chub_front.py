"""C-hub front suspension: solver correctness (constraint residuals +
an INDEPENDENT scipy full-pose closure), seed quality, tweaks, project
round-trip, registry plumbing."""

import dataclasses
import unittest

import numpy as np
from scipy.optimize import least_squares

from suspension_tool import metrics
from suspension_tool.chub_front import (CHubFrontPoints, CHubFrontSolver,
                                        seed_chub_front)
from suspension_tool.seed import IN, SetupVariables


def _sv(**kw) -> SetupVariables:
    base = dict(track_width=63 * IN, wheelbase=61 * IN, ride_height=14 * IN,
                tire_radius=11.5 * IN, tire_width=7 * IN,
                shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
                motion_ratio_goal=1.0 / 1.85, shock_length_at_ride=20.38 * IN,
                static_camber_deg=-0.87, static_toe_deg=0.2)
    base.update(kw)
    return SetupVariables(**base)


def _rodrigues(rvec):
    ang = np.linalg.norm(rvec)
    if ang < 1e-12:
        return np.eye(3)
    ax = rvec / ang
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + np.sin(ang) * k + (1 - np.cos(ang)) * (k @ k)


class TestCHubSolver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sv = _sv()
        cls.hp = seed_chub_front(cls.sv)
        cls.solver = CHubFrontSolver(cls.hp)

    def test_static_readbacks_exact(self):
        st = self.solver.solve(0.0)
        self.assertAlmostEqual(metrics.camber_deg(st), -0.87, places=9)
        self.assertAlmostEqual(metrics.toe_deg(st), 0.2, places=9)
        # the physical kingpin was aimed at the desired caster exactly
        self.assertAlmostEqual(metrics.caster_deg(st), 4.0, places=6)
        # scrub lands near the target (the cambered contact patch sits
        # r*sin(camber) ~ 4.4 mm from the flat-patch aim point)
        self.assertLess(abs(metrics.scrub_radius_mm(st) - 30.0), 6.0)

    def test_constraint_residuals_over_grid(self):
        """Every joint constraint holds at machine precision through
        travel AND steer — the definitive correctness check."""
        hp = self.hp
        cam_len = np.linalg.norm(hp.camber_outer - hp.camber_inner)
        tie_len = np.linalg.norm(hp.tierod_outer - hp.tierod_inner)
        rigid_pairs = [
            ("kingpin span", lambda st: (st.ubj, st.lbj),
             np.linalg.norm(hp.kingpin_upper - hp.kingpin_lower)),
            ("wc-ubj", lambda st: (st.wheel_center, st.ubj),
             np.linalg.norm(hp.wheel_center - hp.kingpin_upper)),
            ("tro-lbj", lambda st: (st.tro, st.lbj),
             np.linalg.norm(hp.tierod_outer - hp.kingpin_lower)),
            ("hinge-kingpin", lambda st: (st.hinge_front, st.lbj),
             np.linalg.norm(hp.hinge_front - hp.kingpin_lower)),
        ]
        for t in np.linspace(-100.0, 200.0, 7):
            for steer in (-30.0, 0.0, 30.0):
                st = self.solver.solve(t, steer)
                self.assertAlmostEqual(
                    np.linalg.norm(st.camber_outer - hp.camber_inner),
                    cam_len, places=6)
                rack = hp.tierod_inner + np.array([0.0, steer, 0.0])
                self.assertAlmostEqual(np.linalg.norm(st.tro - rack),
                                       tie_len, places=6)
                for name, pts, want in rigid_pairs:
                    a, b = pts(st)
                    self.assertAlmostEqual(np.linalg.norm(a - b), want,
                                           places=6, msg=name)
                self.assertAlmostEqual(st.wheel_center[2],
                                       hp.wheel_center[2] + t, places=6)

    def test_independent_scipy_closure(self):
        """From-scratch verification: pose the C-hub and steering block as
        free 6-DOF bodies (rotation vector + translation each, plus the
        arm angle) and let scipy satisfy the joint constraints directly —
        no closed-form closures involved. The wheel centre must land where
        the closed-form solver puts it."""
        hp = self.hp
        arm_o = hp.arm_inner_front
        arm_ax = hp.arm_inner_rear - hp.arm_inner_front
        arm_ax = arm_ax / np.linalg.norm(arm_ax)
        cam_len = np.linalg.norm(hp.camber_outer - hp.camber_inner)
        tie_len = np.linalg.norm(hp.tierod_outer - hp.tierod_inner)

        def arm_pt(p, a):
            r = p - arm_o
            return (arm_o + r * np.cos(a) + np.cross(arm_ax, r) * np.sin(a)
                    + arm_ax * np.dot(arm_ax, r) * (1 - np.cos(a)))

        def body_pt(p, pose, ref):
            return _rodrigues(pose[:3]) @ (p - ref) + ref + pose[3:]

        def residuals(x, target_z, steer):
            a, chub, block = x[0], x[1:7], x[7:13]
            res = []
            # C-hub shares the hinge-pin points with the arm
            for p in (hp.hinge_front, hp.hinge_rear):
                res.extend(body_pt(p, chub, hp.kingpin_lower) - arm_pt(p, a))
            # camber link length
            res.append(np.linalg.norm(
                body_pt(hp.camber_outer, chub, hp.kingpin_lower)
                - hp.camber_inner) - cam_len)
            # steering block shares the kingpin points with the C-hub
            for p in (hp.kingpin_upper, hp.kingpin_lower):
                res.extend(body_pt(p, block, hp.wheel_center)
                           - body_pt(p, chub, hp.kingpin_lower))
            # tie rod to the (steered) rack
            rack = hp.tierod_inner + np.array([0.0, steer, 0.0])
            res.append(np.linalg.norm(
                body_pt(hp.tierod_outer, block, hp.wheel_center) - rack)
                - tie_len)
            # requested wheel-centre height
            res.append(body_pt(hp.wheel_center, block,
                               hp.wheel_center)[2] - target_z)
            return np.array(res)

        for t, steer in ((80.0, 0.0), (-60.0, 0.0), (40.0, 25.0)):
            st = self.solver.solve(t, steer)
            x0 = np.zeros(13)
            sol = least_squares(residuals, x0,
                                args=(hp.wheel_center[2] + t, steer),
                                method="lm", xtol=1e-14, ftol=1e-14)
            self.assertLess(np.max(np.abs(sol.fun)), 1e-7)
            wc = _rodrigues(sol.x[7:10]) @ np.zeros(3) + hp.wheel_center \
                + sol.x[10:13]
            self.assertLess(np.linalg.norm(wc - st.wheel_center), 1e-4)

    def test_branch_continuity(self):
        """No fold-over teleports: neighbouring travel samples stay close
        (the assembly-branch regression pattern)."""
        sw = metrics.sweep_metrics(self.solver, np.linspace(-110, 240, 71))
        self.assertLess(np.max(np.abs(np.diff(sw["camber_deg"]))), 1.0)
        self.assertLess(np.max(np.abs(np.diff(sw["toe_deg"]))), 1.0)

    def test_seed_quality(self):
        """The seed hits its stated goals: motion ratio near target, bump
        steer secant-tuned to ~zero at ride, full travel budget."""
        ts = np.linspace(-40, 40, 21)
        sw = metrics.sweep_metrics(self.solver, ts)
        i0 = int(np.argmin(np.abs(ts)))
        self.assertLess(abs(sw["motion_ratio"][i0] - 1.0 / 1.85), 0.02)
        self.assertLess(abs(sw["bump_steer_deg_per_mm"][i0]), 5e-3)
        stroke = self.sv.shock_max_length - self.sv.shock_min_length
        bump = 1.85 * (self.sv.shock_length_at_ride
                       - self.sv.shock_min_length)
        droop = 1.85 * (self.sv.shock_max_length
                        - self.sv.shock_length_at_ride)
        self.solver.solve(bump)      # must not raise
        self.solver.solve(-droop)
        self.assertGreater(stroke, 0)

    def test_steer_moves_toe_like_dw(self):
        """Positive rack displacement (toward the modelled left corner)
        must steer the wheel the same direction as the double wishbone."""
        from suspension_tool.seed import generate_seed
        from suspension_tool.solver import DoubleWishboneSolver
        dw_hp, _ = generate_seed(_sv())
        dw = DoubleWishboneSolver(dw_hp)
        d_dw = (metrics.toe_deg(dw.solve(0.0, 20.0))
                - metrics.toe_deg(dw.solve(0.0, 0.0)))
        d_ch = (metrics.toe_deg(self.solver.solve(0.0, 20.0))
                - metrics.toe_deg(self.solver.solve(0.0, 0.0)))
        self.assertGreater(d_dw * d_ch, 0.0)


class TestCHubTweaks(unittest.TestCase):
    def setUp(self):
        self.hp = seed_chub_front(_sv())

    def test_params_roundtrip(self):
        from suspension_tool.tweaks import chub_params, set_chub
        p = chub_params(self.hp)
        hp2 = set_chub(self.hp, **p)
        for attr in CHubFrontPoints.POINT_ATTRS:
            np.testing.assert_allclose(getattr(hp2, attr),
                                       getattr(self.hp, attr), atol=1e-9)

    def test_kingpin_reaim_exact(self):
        from suspension_tool.tweaks import chub_params, set_chub
        p = chub_params(self.hp)
        # Drop `pill`: it and `caster` are two routes to the same angle
        # (swapping the block's bore vs turning the eccentric on its pin),
        # so sending both over-determines it. The panel gates them, so only
        # the row actually edited is ever sent -- mirror that here.
        p2 = dict(p, caster=8.0, kpi=10.0, axle_x=-5.0)
        p2.pop("pill", None)
        hp2 = set_chub(self.hp, **p2)
        q = chub_params(hp2)
        self.assertAlmostEqual(q["caster"], 8.0, places=9)
        self.assertAlmostEqual(q["kpi"], 10.0, places=9)
        self.assertAlmostEqual(q["axle_x"], -5.0, places=9)
        # kingpin length preserved; the corner still assembles + solves
        self.assertAlmostEqual(
            np.linalg.norm(hp2.kingpin_upper - hp2.kingpin_lower),
            np.linalg.norm(self.hp.kingpin_upper - self.hp.kingpin_lower))
        st = CHubFrontSolver(hp2).solve(30.0)
        self.assertAlmostEqual(metrics.caster_deg(st), 8.0, delta=1.5)


class TestCHubPlumbing(unittest.TestCase):
    def test_registry(self):
        from suspension_tool.suspension_types import (SUSPENSION_POINT_TYPES,
                                                      SUSPENSION_TYPES,
                                                      solver_for)
        hp = seed_chub_front(_sv())
        self.assertIs(solver_for(hp), CHubFrontSolver)
        self.assertIs(SUSPENSION_TYPES["C-hub front"], CHubFrontSolver)
        self.assertIs(SUSPENSION_POINT_TYPES["C-hub front"], CHubFrontPoints)

    def test_project_roundtrip(self):
        from suspension_tool.project import _hp_from_dict, _hp_to_dict
        hp = seed_chub_front(_sv())
        d = _hp_to_dict(hp)
        self.assertEqual(d["suspension_type"], "chub_front")
        hp2 = _hp_from_dict(d)
        self.assertIsInstance(hp2, CHubFrontPoints)
        for attr in CHubFrontPoints.POINT_ATTRS:
            np.testing.assert_allclose(getattr(hp2, attr),
                                       getattr(hp, attr), atol=1e-9)

    def test_optimize_groups(self):
        from suspension_tool.optimize import free_groups_for
        groups = free_groups_for(seed_chub_front(_sv()))
        self.assertIn("camber_inner", groups)
        self.assertIn("tierod_inner", groups)

    def test_optimize_panel_serializes_type_specific_groups(self):
        """Regression: OptimizePanel.to_dict / payload iterate the LIVE
        per-type free checkboxes, not the static DW FREE_ROWS — otherwise
        saving a project with a C-hub axle raised KeyError('uca_inner')."""
        import os
        # Run headless without requiring the caller to export the platform,
        # so bare `python -m unittest` still works.
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        try:
            from PySide6.QtWidgets import QApplication
        except Exception:
            self.skipTest("Qt not available")
        from suspension_tool.gui.panels import OptimizePanel
        from suspension_tool.units import DEFAULT_UNIT, UNITS
        app = QApplication.instance() or QApplication([])
        panel = OptimizePanel(UNITS[DEFAULT_UNIT])
        panel.set_free_groups_for(seed_chub_front(_sv()))
        # neither call may raise, and 'uca_inner' must not appear
        d = panel.to_dict()
        p = panel.payload()
        self.assertNotIn("uca_inner", d["free"])
        self.assertNotIn("uca_inner", p["free"])

    def test_numeric_ic_dispatch(self):
        """No uca_inner_front attribute -> metrics must take the numeric
        IC path, not the DW arm-plane construction (which would crash)."""
        hp = seed_chub_front(_sv())
        solver = CHubFrontSolver(hp)
        st = solver.solve(0.0)
        self.assertFalse(hasattr(hp, "uca_inner_front"))
        ic = metrics.front_view_ic(solver, st)
        rc = metrics.roll_center_height_mm(solver, st)
        self.assertTrue(ic is None or np.isfinite(ic[0]))
        self.assertTrue(np.isfinite(rc))


if __name__ == "__main__":
    unittest.main()
