"""Tests for the combined / friction-circle load-matrix layer (v1.24.0).

Covers the Lotus-style additions to `impact.py`:
  * contact-patch moments in the load-path solver
  * the friction-circle combined load set and the standard matrix
  * the through-travel load ENVELOPE (worst per pickup / member)
  * the opt-in rocker / bellcrank load path
"""

import unittest

import numpy as np

from suspension_tool.geometry import example_baja_front
from suspension_tool.impact import (control_arm_beams,
    WheelLoad, corner_loads, corner_conditions, friction_circle_loads,
    standard_load_matrix, envelope, envelope_csv, RockerGeometry,
    rocker_loads, corner_loads_rocker)
from suspension_tool.seed import IN, SetupVariables, generate_seed
from suspension_tool.solver import DoubleWishboneSolver


def _sv():
    return SetupVariables(
        track_width=63 * IN, wheelbase=61 * IN, ride_height=14 * IN,
        tire_radius=11.5 * IN, tire_width=7 * IN,
        shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
        shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85,
        kickup_deg=10.0, static_camber_deg=-0.87)


class TestPatchMoment(unittest.TestCase):
    def _cp(self, hp):
        wc = np.asarray(hp.wheel_center, float)
        return np.array([wc[0], wc[1], wc[2] - hp.tire_radius])

    def test_applied_couple_balances_in_force_and_moment(self):
        # With a pure couple at the patch, the chassis mounts must still
        # reproduce the wheel FORCE, and their moment about the patch must
        # equal the applied couple (whole-corner equilibrium).
        hp = generate_seed(_sv())[0]
        cp = self._cp(hp)
        w = np.array([-800.0, 500.0, 1200.0])
        M = np.array([150.0, -90.0, 4000.0])
        r = corner_loads(hp, w, cp, patch_moment=M)
        chassis = [m for m in r.mounts if m.is_chassis]
        F = sum((m.force for m in chassis), np.zeros(3))
        Mo = sum((np.cross(m.point - cp, m.force) for m in chassis),
                 np.zeros(3))
        self.assertLess(np.linalg.norm(F - w), 1e-6)
        self.assertLess(np.linalg.norm(Mo - M), 1e-4)

    def test_zero_moment_matches_no_moment(self):
        hp = generate_seed(_sv())[0]
        w = np.array([0.0, 400.0, 1000.0])
        a = corner_loads(hp, w)
        b = corner_loads(hp, w, patch_moment=np.zeros(3))
        self.assertAlmostEqual(a.worst_member().axial_lb,
                               b.worst_member().axial_lb, places=6)

    def test_moment_on_chain_type(self):
        from suspension_tool.harm_rear import seed_harm_rear
        hp = seed_harm_rear(_sv())
        cp = self._cp(hp)
        w = np.array([-300.0, 600.0, 900.0])
        M = np.array([0.0, 0.0, 3000.0])
        r = corner_loads(hp, w, cp, patch_moment=M)
        chassis = [m for m in r.mounts if m.is_chassis]
        Mo = sum((np.cross(m.point - cp, m.force) for m in chassis),
                 np.zeros(3))
        self.assertLess(np.linalg.norm(Mo - M) / 3000.0, 1e-4)


class TestFrictionCircle(unittest.TestCase):
    def test_magnitude_and_directions(self):
        loads = friction_circle_loads(300.0, 0.7, n_dir=8,
                                      pneumatic_trail_mm=18.0)
        self.assertEqual(len(loads), 8)
        h = 0.7 * 300.0
        for wl in loads:
            fx, fy, fz = wl.force
            self.assertAlmostEqual(np.hypot(fx, fy), h, places=6)
            self.assertAlmostEqual(fz, 300.0, places=6)
        # theta = 0 is pure braking (-X, no Y)
        b0 = loads[0]
        self.assertLess(b0.force[0], 0.0)
        self.assertAlmostEqual(b0.force[1], 0.0, places=6)
        # the 90-deg case is pure +Y and carries the aligning couple Mz
        q = loads[2]
        self.assertAlmostEqual(q.force[0], 0.0, places=6)
        self.assertGreater(q.force[1], 0.0)
        self.assertAlmostEqual(q.moment[2], -q.force[1] * 18.0, places=6)


class TestStandardMatrix(unittest.TestCase):
    def test_matrix_shape_and_conditions(self):
        loads, cond = standard_load_matrix("front_out", n_dir=12,
                                           bump_g=3.0)
        # static + 12 circle + bump + 2 impulse = 16
        self.assertEqual(len(loads), 16)
        names = [wl.name for wl in loads]
        self.assertIn("static", names)
        self.assertIn("bump_3g", names)
        self.assertIn("kerb_lateral", names)
        self.assertGreater(cond["fz_lb"], 0.0)
        self.assertGreater(cond["mu"], 0.0)

    def test_corner_conditions_matches_dynamics(self):
        from suspension_tool import dynamics
        dyn = dynamics.DynamicsInputs()
        cond = corner_conditions("rear_out", dyn=dyn)
        res = dynamics.compute(dyn)
        self.assertAlmostEqual(cond["fz_lb"], res["load_rear_out_lb"],
                               places=6)


class TestEnvelope(unittest.TestCase):
    def setUp(self):
        self.hp = generate_seed(_sv())[0]
        self.solver = DoubleWishboneSolver(self.hp)
        self.loads, self.cond = standard_load_matrix("front_out", n_dir=12)

    def test_envelope_is_worst_over_the_set(self):
        env = envelope(self.hp, self.loads, conditions=self.cond)
        # the envelope worst per member must be >= every single case
        per_member_worst = {}
        for wl in self.loads:
            r = corner_loads(self.hp, wl.force,
                             patch_moment=(wl.moment if np.any(wl.moment)
                                           else None))
            for m in r.members:
                per_member_worst[m.name] = max(
                    per_member_worst.get(m.name, 0.0), abs(m.axial_lb))
        for e in env.members:
            self.assertGreaterEqual(abs(e.worst_axial_lb) + 1e-6,
                                    per_member_worst[e.name])

    def test_tension_and_compression_bracket_worst(self):
        env = envelope(self.hp, self.loads, conditions=self.cond)
        for e in env.members:
            self.assertGreaterEqual(e.max_tension_lb, 0.0)
            self.assertLessEqual(e.max_compression_lb, 0.0)
            self.assertLessEqual(abs(e.worst_axial_lb),
                                 max(e.max_tension_lb,
                                     -e.max_compression_lb) + 1e-6)

    def test_travel_pose_zero_matches_static(self):
        static = envelope(self.hp, self.loads, conditions=self.cond)
        swept = envelope(self.hp, self.loads, solver=self.solver,
                         travels=[0.0], conditions=self.cond)
        self.assertEqual(swept.n_poses, 1)
        a = {m.name: m.worst_axial_lb for m in static.members}
        b = {m.name: m.worst_axial_lb for m in swept.members}
        # The travel-0 pose uses the solver's camber-accurate contact patch;
        # the static fallback uses a straight-down patch. They agree to
        # within the (tiny) camber-induced patch shift — assert relatively.
        for k in a:
            self.assertLess(abs(a[k] - b[k]) / max(abs(a[k]), 1.0), 2e-3)

    def test_travel_sweep_never_below_static(self):
        static = envelope(self.hp, self.loads, conditions=self.cond)
        swept = envelope(self.hp, self.loads, solver=self.solver,
                         travels=np.linspace(-25, 25, 11),
                         conditions=self.cond)
        self.assertGreater(swept.n_poses, 1)
        a = {m.name: abs(m.worst_axial_lb) for m in static.members}
        for e in swept.members:
            self.assertGreaterEqual(abs(e.worst_axial_lb) + 1e-6, a[e.name])

    def test_csv_has_governing_columns(self):
        env = envelope(self.hp, self.loads, solver=self.solver,
                       travels=np.linspace(-15, 15, 7), conditions=self.cond)
        text = envelope_csv(env)
        self.assertIn("governing_case", text)
        self.assertIn("governing_pose", text)
        self.assertIn("max_tension", text)
        self.assertIn("apply_to", text)


class TestArmMountedTieRod(unittest.TestCase):
    """v1.24.1 defect: `_dw_corner_loads` ignored tierod_on_lca/uca, so the
    toe link's reaction on the arm was missing from that arm's equilibrium.
    Virtual work (which uses only the kinematic solver) is the independent
    referee — it shares no code with the statics."""

    FZ = 1000.0

    def _virtual_work_shock(self, hp):
        from suspension_tool.metrics import sweep_metrics
        solver = DoubleWishboneSolver(hp)
        mr = sweep_metrics(solver, np.linspace(-15.0, 15.0, 31))[
            "motion_ratio"][15]
        return self.FZ / mr        # MR is shock/wheel (< 1)

    def _configs(self):
        import dataclasses
        base = generate_seed(_sv())[0]
        return [
            ("chassis", base),
            ("tierod_on_lca", dataclasses.replace(base, tierod_on_lca=True)),
            ("tierod_on_uca", dataclasses.replace(base, tierod_on_uca=True)),
            ("shock_uca+tierod_lca",
             dataclasses.replace(base, shock_on_uca=True, tierod_on_lca=True)),
        ]

    def test_shock_matches_virtual_work_for_every_mounting(self):
        for name, hp in self._configs():
            r = corner_loads(hp, np.array([0.0, 0.0, self.FZ]))
            shock = abs([m for m in r.members if m.name == "shock"][0].axial_lb)
            truth = self._virtual_work_shock(hp)
            self.assertAlmostEqual(shock / truth, 1.0, places=4,
                                   msg=f"{name}: virtual work disagrees")

    def test_arm_mounting_actually_changes_the_answer(self):
        # Regression guard for the exact defect: the flags were ignored, so
        # all three configurations returned an identical shock force.
        vals = []
        for _name, hp in self._configs()[:3]:
            r = corner_loads(hp, np.array([0.0, 0.0, self.FZ]))
            vals.append(abs([m for m in r.members
                             if m.name == "shock"][0].axial_lb))
        self.assertNotAlmostEqual(vals[0], vals[1], places=1)
        self.assertNotAlmostEqual(vals[0], vals[2], places=1)

    def test_arm_mounted_tierod_is_not_a_chassis_pickup(self):
        import dataclasses
        base = generate_seed(_sv())[0]
        r = corner_loads(base, np.array([0.0, 0.0, self.FZ]))
        self.assertTrue(r.mount("tierod_inner").is_chassis)
        for flag in ("tierod_on_lca", "tierod_on_uca"):
            hp = dataclasses.replace(base, **{flag: True})
            r = corner_loads(hp, np.array([0.0, 0.0, self.FZ]))
            self.assertFalse(r.mount("tierod_inner").is_chassis,
                             f"{flag}: toe link inner is on an arm, not the "
                             "frame")

    def test_chassis_mounts_still_balance(self):
        w = np.array([-800.0, 500.0, 1200.0])
        for name, hp in self._configs():
            r = corner_loads(hp, w)
            chassis = [m for m in r.mounts if m.is_chassis]
            total = sum((m.force for m in chassis), np.zeros(3))
            self.assertLess(np.linalg.norm(total - w) / np.linalg.norm(w),
                            1e-8, name)


class TestBodyLoadSets(unittest.TestCase):
    def setUp(self):
        self.hp = generate_seed(_sv())[0]
        self.r = corner_loads(self.hp, np.array([-400.0, 300.0, 1200.0]),
                              patch_moment=np.array([0.0, 0.0, 3000.0]))

    def test_every_body_is_self_equilibrated(self):
        self.assertTrue(self.r.bodies)
        for name, b in self.r.bodies.items():
            scale = max(1.0, max(float(np.linalg.norm(f))
                                 for _l, _p, f in b.loads))
            self.assertLess(b.force_residual() / scale, 1e-9, f"{name} force")
            self.assertLess(b.moment_residual(b.loads[0][1]) / scale, 1e-6,
                            f"{name} moment")

    def test_ball_joints_are_exposed_as_internal(self):
        names = {m.name: m for m in self.r.mounts}
        for k in ("uca_outer_ball_joint", "lca_outer_ball_joint",
                  "shock_outer"):
            self.assertIn(k, names)
            self.assertFalse(names[k].is_chassis, k)

    def test_internal_rows_cancel_inside_the_corner(self):
        # Each internal row has an equal-and-opposite partner on the adjacent
        # body, which is exactly why only the chassis rows sum to the wheel
        # load. Guards against anyone "fixing" the invariant by summing all.
        for arm, bj in (("uca", "uca_outer_ball_joint"),
                        ("lca", "lca_outer_ball_joint")):
            on_upright = self.r.mount(bj).force
            on_arm = [f for lab, _p, f in self.r.bodies[arm].loads
                      if lab == bj][0]
            np.testing.assert_allclose(on_upright, -on_arm, atol=1e-8)

    def test_csv_contains_bodies_and_bending(self):
        from suspension_tool.impact import body_loads_csv
        text = body_loads_csv(self.r)
        self.assertIn("lca", text)
        self.assertIn("upright", text)
        self.assertIn("bending", text)


class TestArmBending(unittest.TestCase):
    """The shock hanging off mid-span bends the arm. That bending is an
    INTERNAL stress resultant: it never appears in a joint reaction, which is
    exactly why it needs its own output."""

    def setUp(self):
        self.hp = generate_seed(_sv())[0]
        self.r = corner_loads(self.hp, np.array([0.0, 0.0, 1000.0]))

    def test_bending_vanishes_at_a_spherical_bushing(self):
        for beam in control_arm_beams(self.r.bodies["lca"]):
            self.assertLess(beam.sections[0].bending_lb_mm, 1e-6,
                            f"{beam.name} root is a spherical joint")

    def test_shock_leg_carries_more_bending_than_the_bare_leg(self):
        beams = {b.name: b for b in control_arm_beams(self.r.bodies["lca"])}
        shock_leg = [b for b in beams.values()
                     if "shock_outer" in b.carried]
        self.assertEqual(len(shock_leg), 1)
        bare = [b for b in beams.values() if "shock_outer" not in b.carried][0]
        self.assertGreater(shock_leg[0].max_bending().bending_lb_mm,
                           bare.max_bending().bending_lb_mm)

    def test_summary_matches_a_hand_moment(self):
        from suspension_tool.impact import shock_bending_summary
        d = shock_bending_summary(self.r)["lca"]
        body = self.r.bodies["lca"]
        pts = {lab: (np.asarray(p, float), np.asarray(f, float))
               for lab, p, f in body.loads}
        sh_p, sh_f = pts["shock_outer"]
        bj_p, _ = pts["lca_outer_ball_joint"]
        hand = float(np.linalg.norm(np.cross(sh_p - bj_p, sh_f)))
        self.assertAlmostEqual(d["bending_about_ball_joint_lb_mm"] / hand,
                               1.0, places=9)

    def test_bending_is_large_enough_to_matter(self):
        from suspension_tool.impact import shock_bending_summary
        d = shock_bending_summary(self.r)["lca"]
        # A truss idealisation would report zero. It is thousands of lb-in.
        self.assertGreater(d["bending_about_ball_joint_lb_in"], 1000.0)

    def test_non_arm_body_raises(self):
        with self.assertRaises(ValueError):
            control_arm_beams(self.r.bodies["upright"])


class TestRocker(unittest.TestCase):
    def _rocker(self, hp):
        return RockerGeometry(
            pivot_pt=np.array([0.0, 300.0, 500.0]),
            pivot_axis=np.array([1.0, 0.0, 0.0]),
            pushrod_rocker_pt=np.asarray(hp.shock_inner, float),
            pushrod_arm_pt=np.asarray(hp.shock_outer, float),
            damper_rocker_pt=np.array([0.0, 330.0, 470.0]),
            damper_chassis_pt=np.array([0.0, 260.0, 560.0]))

    def test_rocker_equilibrium(self):
        hp = example_baja_front()
        rk = self._rocker(hp)
        rr = rocker_loads(rk, 900.0)
        u_pr = (np.asarray(hp.shock_outer, float)
                - np.asarray(hp.shock_inner, float))
        f_pr = u_pr / np.linalg.norm(u_pr) * 900.0
        piv_on_rocker = -rr.mounts[0].force
        damp_on_rocker = -rr.mounts[1].force
        self.assertLess(
            np.linalg.norm(f_pr + damp_on_rocker + piv_on_rocker), 1e-6)
        self.assertLess(rr.residual, 1e-6)

    def test_rocker_corner_preserves_wheel_load(self):
        hp = example_baja_front()
        rk = self._rocker(hp)
        w = np.array([-400.0, 300.0, 1000.0])
        r = corner_loads_rocker(hp, w, rk)
        F = sum((m.force for m in r.mounts if m.is_chassis), np.zeros(3))
        self.assertLess(np.linalg.norm(F - w), 1e-6)
        names = [m.name for m in r.members]
        self.assertIn("pushrod", names)
        self.assertIn("damper", names)
        self.assertNotIn("shock", names)
        mounts = [m.name for m in r.mounts]
        self.assertIn("rocker_pivot", mounts)
        self.assertIn("damper_chassis", mounts)
        self.assertNotIn("shock_inner", mounts)

    def test_singular_rocker_raises(self):
        hp = example_baja_front()
        rk = self._rocker(hp)
        # damper line through the pivot along the axis makes no moment
        rk.damper_rocker_pt = np.array([0.0, 300.0, 500.0])
        rk.damper_chassis_pt = np.array([50.0, 300.0, 500.0])
        with self.assertRaises(ValueError):
            rocker_loads(rk, 500.0)


if __name__ == "__main__":
    unittest.main()
