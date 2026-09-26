"""Phase 2 verification: the seed generator, driven from a full-size
Baja setup:

  * motion ratio 0.55 (shock travel / wheel travel, the standard
    convention -- below 1)
  * shock 14" compressed .. 22" extended, 19" at ride height
  * track 60" at 12" ride height, 23" OD tire
  * static camber -1.0 deg
  * camber goes MORE NEGATIVE in bump, positive in droop

The generator was originally validated against a real car's measured
front corner; those numbers are not distributed with this demo, and none
of the checks below depended on them. What is asserted is behaviour: the
seed hits whatever setup it is given, articulates through the full travel
that setup implies, and reproduces the known camber trend.
"""

import unittest

import numpy as np

from suspension_tool.metrics import camber_deg, scrub_radius_mm, toe_deg
from suspension_tool.seed import IN, SetupVariables, generate_seed
from suspension_tool.solver import DoubleWishboneSolver


# The one source for the fixture's figures.
TRACK = 60.0 * IN
MR = 0.55          # shock travel / wheel travel
CAMBER = -1.0      # deg


def baseline_setup() -> SetupVariables:
    # Offsets zeroed: these tests hand-verify the DESIGN frame (ground at
    # z = 0). The Onshape-alignment offset is a pure rigid shift, tested
    # separately in TestSeedOffset.
    return SetupVariables(
        track_width=TRACK,
        wheelbase=60.0 * IN,           # not used yet; typical Baja value
        ride_height=12.0 * IN,
        tire_radius=11.5 * IN,         # 23" OD
        tire_width=7.0 * IN,
        shock_min_length=14.0 * IN,
        shock_max_length=22.0 * IN,
        shock_length_at_ride=19.0 * IN,
        motion_ratio_goal=MR,
        static_camber_deg=CAMBER,
        desired_scrub_radius=30.0,
        x_offset=0.0, z_offset=0.0,
    )


class TestSeedFullSize(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hp, cls.report = generate_seed(baseline_setup())
        cls.solver = DoubleWishboneSolver(cls.hp)

    def test_travel_budget_from_shock_and_mr_clamped_to_articulation(self):
        # The optimistic budget is MR * shock stroke each way; the reported
        # travel is that, CLAMPED to what the linkage can actually reach. So
        # it must not exceed the budget, must stay a sensible fraction of it,
        # and the seed must solve right at the reported extremes.
        opt_bump = (19.0 - 14.0) * IN / MR
        opt_droop = (22.0 - 19.0) * IN / MR
        self.assertLessEqual(self.report.bump_travel, opt_bump + 1e-6)
        self.assertGreater(self.report.bump_travel, 0.85 * opt_bump)
        self.assertLessEqual(self.report.droop_travel, opt_droop + 1e-6)
        self.assertGreater(self.report.droop_travel, 0.85 * opt_droop)
        self.solver.solve(self.report.bump_travel)
        self.solver.solve(-self.report.droop_travel)

    def test_articulates_through_full_real_car_travel(self):
        # generate_seed already checks the extremes; sweep the whole range
        # to make sure there is no dead spot in the middle.
        for t in np.linspace(-self.report.droop_travel, self.report.bump_travel, 21):
            self.solver.solve(t)

    def test_track_width_and_tire_on_ground(self):
        # Only the bushings carry the kickup, so the wheel centre is exactly
        # half-track out and one tire radius up; the contact patch is on the
        # ground to within the tiny camber-induced offset.
        self.assertAlmostEqual(self.hp.wheel_center[1], TRACK / 2.0, places=6)
        self.assertAlmostEqual(self.hp.wheel_center[2], self.hp.tire_radius, places=6)
        self.assertAlmostEqual(self.solver.solve(0.0).contact_patch[2], 0.0, delta=0.2)

    def test_static_alignment(self):
        st = self.solver.solve(0.0)
        self.assertAlmostEqual(camber_deg(st), CAMBER, places=6)
        self.assertAlmostEqual(toe_deg(st), 0.0, places=6)

    def test_motion_ratio_hits_goal(self):
        self.assertAlmostEqual(self.report.motion_ratio_static,
                               MR, delta=0.02)

    def test_bump_steer_tuned_out(self):
        self.assertLess(abs(self.report.bump_steer_static), 0.005)  # deg/mm

    def test_scrub_radius_near_requested(self):
        st = self.solver.solve(0.0)
        # Static camber tips the contact patch slightly, so allow a few mm.
        self.assertAlmostEqual(scrub_radius_mm(st), 30.0, delta=5.0)

    def test_camber_trend_matches_real_car(self):
        # Team data: more negative in bump, more positive in droop.
        c_droop = camber_deg(self.solver.solve(-0.8 * self.report.droop_travel))
        c_static = camber_deg(self.solver.solve(0.0))
        c_bump = camber_deg(self.solver.solve(+0.8 * self.report.bump_travel))
        self.assertLess(c_bump, c_static)
        self.assertGreater(c_droop, c_static)

    def test_packaging_sanity(self):
        # Chassis-side points stay inside the frame envelope; everything is
        # above ground and inside the half track.
        hp = self.hp
        for p in (hp.uca_inner_front, hp.uca_inner_rear, hp.lca_inner_front,
                  hp.lca_inner_rear, hp.tierod_inner, hp.shock_inner):
            self.assertGreater(p[1], 0.0)
            self.assertLess(p[1], 0.35 * TRACK)
            self.assertGreater(p[2], 0.0)
        for p in (hp.uca_outer, hp.lca_outer, hp.tierod_outer):
            self.assertLess(p[1], TRACK / 2.0)


class TestKickup(unittest.TestCase):
    """Phase 3.1 feedback: each arm's front->rear bushing line must be NORMAL
    to the 2D sketch plane (tilted by the chassis kickup), and ONLY the
    bushings tilt — the ball joints, knuckle and shock stay on the sketch so
    the suspension follows the 2D path and no caster is baked into the
    knuckle."""

    @staticmethod
    def _axis_kickup_deg(front, rear):
        """Angle of the fore-aft bushing axis above horizontal, side view."""
        d = front - rear
        return np.degrees(np.arctan2(d[2], d[0]))

    def test_bushing_axes_normal_to_sketch_plane(self):
        from suspension_tool.seed import sketch_plane_normal
        sv = baseline_setup(); sv.kickup_deg = 10.0
        hp, report = generate_seed(sv)
        self.assertAlmostEqual(report.kickup_deg, 10.0, places=9)
        n = sketch_plane_normal(10.0)
        for front, rear in [(hp.uca_inner_front, hp.uca_inner_rear),
                            (hp.lca_inner_front, hp.lca_inner_rear)]:
            axis = (front - rear) / np.linalg.norm(front - rear)
            # axis must be parallel to the sketch-plane normal...
            self.assertAlmostEqual(abs(float(np.dot(axis, n))), 1.0, places=9)
            # ...which means tilted by the kickup, front mount higher.
            self.assertAlmostEqual(self._axis_kickup_deg(front, rear), 10.0, places=4)
            self.assertGreater(front[2], rear[2])

    def test_zero_kickup_gives_flat_axes(self):
        sv = baseline_setup()
        sv.kickup_deg = 0.0
        hp, _ = generate_seed(sv)
        self.assertAlmostEqual(hp.uca_inner_front[2], hp.uca_inner_rear[2], places=6)
        self.assertAlmostEqual(hp.lca_inner_front[2], hp.lca_inner_rear[2], places=6)

    def test_kickup_does_not_move_knuckle_or_caster(self):
        # The fix: tilting the bushings must NOT rotate the ball joints,
        # knuckle, or shock, and must NOT inflate the knuckle's caster (the
        # bug in the previous "rotate whole corner" build). Caster stays at
        # the design value regardless of kickup.
        # In INDEPENDENT-caster mode the kingpin is NOT planarized, so
        # caster stays at the design value regardless of kickup (the old
        # invariant). (In the default follow-kickup mode caster tracks the
        # kickup by design — tested separately.)
        flat = baseline_setup(); flat.kickup_deg = 0.0
        flat.independent_caster = True
        kicked = baseline_setup(); kicked.kickup_deg = 10.0
        kicked.independent_caster = True
        hp_flat, r_flat = generate_seed(flat)
        hp_kick, r_kick = generate_seed(kicked)
        self.assertAlmostEqual(r_kick.caster_static, r_flat.caster_static, delta=0.05)
        self.assertAlmostEqual(r_kick.caster_static, 4.0, delta=0.2)  # ~design
        # The knuckle (ball joints + steering arm) and wheel are pure geometry
        # and must be IDENTICAL with and without kickup. (The shock and
        # tie-rod inner are re-tuned for MR / zero bump steer, so they may
        # shift slightly — that is correct, not a rotation of the knuckle.)
        for attr in ("uca_outer", "lca_outer", "tierod_outer", "wheel_center"):
            np.testing.assert_allclose(getattr(hp_kick, attr),
                                       getattr(hp_flat, attr), atol=1e-6)

    def test_inboard_axes_exactly_parallel(self):
        # Both inboard axes must be parallel (both normal to the same sketch
        # plane). Any non-parallel APPEARANCE in the 3D view is perspective,
        # not the model — so assert it numerically here.
        sv = baseline_setup(); sv.kickup_deg = 10.0
        hp, _ = generate_seed(sv)
        ua = hp.uca_inner_front - hp.uca_inner_rear
        la = hp.lca_inner_front - hp.lca_inner_rear
        ua /= np.linalg.norm(ua); la /= np.linalg.norm(la)
        self.assertLess(float(np.linalg.norm(np.cross(ua, la))), 1e-9)

    def test_inboard_separation_larger_but_under_kingpin(self):
        # Default seed must give a roomy inboard axis separation (frame
        # mounting) that stays safely under the kingpin length (camber
        # direction). A bigger fraction must give a bigger separation.
        sv = baseline_setup()
        hp, _ = generate_seed(sv)
        umid = (hp.uca_inner_front + hp.uca_inner_rear) / 2
        lmid = (hp.lca_inner_front + hp.lca_inner_rear) / 2
        sep = np.linalg.norm(umid - lmid)
        kingpin = np.linalg.norm(hp.uca_outer - hp.lca_outer)
        self.assertGreater(sep, 100.0)            # roomy for the frame
        self.assertLess(sep, 0.85 * kingpin)      # margin below the kingpin

        wider = baseline_setup(); wider.inboard_sep_frac = 0.55
        hp2, _ = generate_seed(wider)
        umid2 = (hp2.uca_inner_front + hp2.uca_inner_rear) / 2
        lmid2 = (hp2.lca_inner_front + hp2.lca_inner_rear) / 2
        self.assertGreater(np.linalg.norm(umid2 - lmid2), sep)

    def test_camber_direction_correct_across_separations(self):
        # The camber curve must keep its direction (negative in bump) for the
        # whole allowed separation range — the constraint the team flagged.
        for frac in (0.2, 0.35, 0.5, 0.65):
            sv = baseline_setup(); sv.inboard_sep_frac = frac
            hp, rep = generate_seed(sv)
            s = DoubleWishboneSolver(hp)
            cd = camber_deg(s.solve(-0.7 * rep.droop_travel))
            cs = camber_deg(s.solve(0.0))
            cb = camber_deg(s.solve(0.7 * rep.bump_travel))
            self.assertTrue(cb < cs < cd, f"camber direction wrong at frac {frac}")

    def test_kickup_preserves_track_and_ground(self):
        sv = baseline_setup(); sv.kickup_deg = 10.0
        hp, _ = generate_seed(sv)
        solver = DoubleWishboneSolver(hp)
        self.assertAlmostEqual(hp.wheel_center[1], TRACK / 2.0, places=6)
        # Wheel centre sits exactly one tire radius up; the contact patch is
        # on the ground to within the tiny camber-induced offset.
        self.assertAlmostEqual(hp.wheel_center[2], hp.tire_radius, places=6)
        self.assertAlmostEqual(solver.solve(0.0).contact_patch[2], 0.0, delta=0.2)


class TestNewSetupVariables(unittest.TestCase):
    """Phase 4.1: direct kingpin length, direct inboard gap, shock on UCA."""

    def test_kingpin_length_honoured(self):
        sv = baseline_setup()
        sv.kingpin_length = 250.0
        sv.independent_caster = True   # keep the raw vertical kingpin span
        hp, _ = generate_seed(sv)
        self.assertAlmostEqual(hp.uca_outer[2] - hp.lca_outer[2], 250.0, places=6)
        # still centred on the wheel centre
        self.assertAlmostEqual((hp.uca_outer[2] + hp.lca_outer[2]) / 2,
                               hp.wheel_center[2], places=6)

    def test_inboard_gap_honoured_and_clamped(self):
        sv = baseline_setup()
        sv.inboard_sep = 110.0
        sv.independent_caster = True   # raw vertical kingpin span reference
        hp, _ = generate_seed(sv)
        umid = (hp.uca_inner_front + hp.uca_inner_rear) / 2
        lmid = (hp.lca_inner_front + hp.lca_inner_rear) / 2
        self.assertAlmostEqual(umid[2] - lmid[2], 110.0, places=6)
        # an absurd request clamps to 0.8 of the kingpin span, not beyond
        sv.inboard_sep = 5000.0
        hp2, _ = generate_seed(sv)
        umid2 = (hp2.uca_inner_front + hp2.uca_inner_rear) / 2
        lmid2 = (hp2.lca_inner_front + hp2.lca_inner_rear) / 2
        span = hp2.uca_outer[2] - hp2.lca_outer[2]
        self.assertLessEqual(umid2[2] - lmid2[2], 0.8 * span + 1e-6)

    def test_default_seed_has_planar_kingpin(self):
        # v1.11.5 default: caster follows the kickup -> the kingpin lies
        # in the 2D sketch (no knuckle twist), and caster ~= kickup.
        from suspension_tool.geometry import sketch_planarity
        from suspension_tool.metrics import caster_deg
        from suspension_tool.solver import DoubleWishboneSolver
        sv = baseline_setup(); sv.kickup_deg = 10.0
        sv.independent_caster = False
        hp, _ = generate_seed(sv)
        self.assertLess(sketch_planarity(hp)["kingpin_off_plane_deg"], 1e-6)
        self.assertAlmostEqual(
            caster_deg(DoubleWishboneSolver(hp).solve(0.0)), 10.0, delta=0.5)

    def test_independent_caster_keeps_desired_caster(self):
        from suspension_tool.metrics import caster_deg
        from suspension_tool.solver import DoubleWishboneSolver
        sv = baseline_setup(); sv.kickup_deg = 10.0
        sv.desired_caster_deg = 4.0
        sv.independent_caster = True
        hp, _ = generate_seed(sv)
        self.assertAlmostEqual(
            caster_deg(DoubleWishboneSolver(hp).solve(0.0)), 4.0, delta=0.2)

    def test_shock_on_uca(self):
        sv = baseline_setup()
        sv.shock_on_uca = True
        hp, report = generate_seed(sv)
        self.assertTrue(hp.shock_on_uca)
        self.assertAlmostEqual(report.motion_ratio_static,
                               sv.motion_ratio_goal, delta=0.06)
        # the outer mount must be rigid with the UPPER arm through travel
        s = DoubleWishboneSolver(hp)
        d0 = np.linalg.norm(s.solve(0.0).shock_outer - s.solve(0.0).ubj)
        for t in (-80.0, 60.0, 150.0):
            st = s.solve(t)
            self.assertAlmostEqual(
                np.linalg.norm(st.shock_outer - st.ubj), d0, places=6)


class TestSeedOtherCar(unittest.TestCase):
    """The generator must work for setups other than the default."""

    def test_smaller_car(self):
        sv = SetupVariables(
            track_width=1300.0,
            wheelbase=1500.0,
            ride_height=250.0,
            tire_radius=280.0,
            tire_width=180.0,
            shock_min_length=350.0,
            shock_max_length=500.0,
            motion_ratio_goal=1.0 / 1.5,
            static_camber_deg=-1.0,
        )
        hp, report = generate_seed(sv)
        self.assertAlmostEqual(report.motion_ratio_static,
                               1.0 / 1.5, delta=0.03)
        self.assertLess(abs(report.bump_steer_static), 0.005)
        solver = DoubleWishboneSolver(hp)
        for t in np.linspace(-report.droop_travel, report.bump_travel, 11):
            solver.solve(t)


class TestSeedOffset(unittest.TestCase):
    """The Onshape-alignment offset: a pure rigid translation of the seed.

    Default is +39 in X / -14 in Z (user-editable), placing the ground at
    z = -ride_height for the team's 14 in ride height. Every kinematic
    metric must be EXACTLY unchanged — ground-referenced metrics use the
    tire contact plane, not z = 0."""

    def test_default_offset_is_exact_rigid_shift(self):
        base = baseline_setup()                       # offsets zeroed
        shifted_sv = baseline_setup()
        shifted_sv.x_offset, shifted_sv.z_offset = 39.0 * IN, -14.0 * IN
        hp0, _ = generate_seed(base)
        hp1, _ = generate_seed(shifted_sv)
        off = np.array([39.0 * IN, 0.0, -14.0 * IN])
        for attr in hp0.POINT_ATTRS:
            np.testing.assert_allclose(getattr(hp1, attr),
                                       getattr(hp0, attr) + off, atol=1e-9)
        # ground plane = contact patch z = -ride height (14 in tire radius
        # cancels: wc_z = tire_radius - 14 in, patch = wc_z - tire_radius)
        s1 = DoubleWishboneSolver(hp1)
        self.assertAlmostEqual(s1.solve(0.0).contact_patch[2],
                               -14.0 * IN, delta=0.2)

    def test_metrics_invariant_under_offset(self):
        from suspension_tool.metrics import corner_metrics
        hp0, _ = generate_seed(baseline_setup())      # offsets zeroed
        hp1, _ = generate_seed(baseline_setup().__class__(**{
            **{f: getattr(baseline_setup(), f) for f in vars(baseline_setup())},
            "x_offset": 39.0 * IN, "z_offset": -14.0 * IN}))
        s0, s1 = DoubleWishboneSolver(hp0), DoubleWishboneSolver(hp1)
        for t in (-60.0, 0.0, 90.0):
            m0 = corner_metrics(s0, s0.solve(t))
            m1 = corner_metrics(s1, s1.solve(t))
            for key in ("camber_deg", "toe_deg", "caster_deg", "kpi_deg",
                        "scrub_radius_mm", "caster_trail_mm",
                        "roll_center_height_mm", "shock_length_mm"):
                self.assertAlmostEqual(m0[key], m1[key], places=6,
                                       msg=f"{key} at travel {t}")


class TestSetupFromHardpoints(unittest.TestCase):
    """v1.14: measure a SetupVariables back from live geometry so a
    reseed reproduces the design instead of resetting to defaults."""

    def test_tweaked_design_round_trips_through_reseed(self):
        import dataclasses
        from suspension_tool.seed import setup_from_hardpoints
        from suspension_tool.tweaks import (corner_x, inboard_axis_sep,
                                            kingpin_length, scrub_radius,
                                            set_corner_x, set_kingpin_length,
                                            set_scrub_radius)
        sv0 = baseline_setup()
        hp, _ = generate_seed(sv0)
        hp = set_scrub_radius(hp, 42.0)
        hp = set_kingpin_length(hp, 280.0)
        hp = set_corner_x(hp, corner_x(hp) + 30.0)

        sv1 = setup_from_hardpoints(hp, sv0)
        hp2, report = generate_seed(sv1)

        self.assertAlmostEqual(2 * abs(hp2.wheel_center[1]),
                               2 * abs(hp.wheel_center[1]), delta=0.01)
        self.assertAlmostEqual(scrub_radius(hp2), scrub_radius(hp),
                               delta=0.1)
        self.assertAlmostEqual(kingpin_length(hp2), kingpin_length(hp),
                               delta=0.5)
        self.assertAlmostEqual(inboard_axis_sep(hp2), inboard_axis_sep(hp),
                               delta=0.1)
        self.assertAlmostEqual(corner_x(hp2), corner_x(hp), delta=0.5)
        self.assertAlmostEqual(hp2.wheel_center[2], hp.wheel_center[2],
                               delta=0.01)
        self.assertAlmostEqual(report.motion_ratio_static,
                               sv0.motion_ratio_goal, delta=0.1)

    def test_carries_seed_only_flags(self):
        import dataclasses
        from suspension_tool.seed import setup_from_hardpoints
        sv0 = dataclasses.replace(baseline_setup(), shock_on_uca=True)
        hp, _ = generate_seed(sv0)
        sv1 = setup_from_hardpoints(hp, sv0)
        self.assertTrue(sv1.shock_on_uca)
        self.assertAlmostEqual(sv1.tire_radius, hp.tire_radius, places=9)
        self.assertAlmostEqual(sv1.static_camber_deg, hp.static_camber_deg,
                               places=9)


class TestApplyLockedPoints(unittest.TestCase):
    """v1.15: reseed honors pinned hardpoints (welded tabs survive)."""

    def setUp(self):
        import dataclasses
        from suspension_tool.tweaks import set_ride_height, ride_height
        self.sv = baseline_setup()
        hp, _ = generate_seed(self.sv)
        # simulate a session: raise the whole design 30 mm and drag the
        # lower tabs + shock mount to "welded" positions
        hp = set_ride_height(hp, ride_height(hp) - 30.0)
        self.ref = dataclasses.replace(
            hp,
            lca_inner_front=hp.lca_inner_front + np.array([5.0, -8.0, 12.0]),
            lca_inner_rear=hp.lca_inner_rear + np.array([5.0, -8.0, 12.0]),
            shock_inner=hp.shock_inner + np.array([-20.0, 10.0, 15.0]))
        self.locked = {"lca_inner_front", "lca_inner_rear", "shock_inner"}

    def test_locked_points_kept_exactly(self):
        import dataclasses
        from suspension_tool.seed import apply_locked_points
        hp_new, _ = generate_seed(dataclasses.replace(self.sv,
                                                      kickup_deg=14.0))
        adj = apply_locked_points(hp_new, self.ref, self.locked)
        for a in self.locked:
            np.testing.assert_allclose(getattr(adj, a), getattr(self.ref, a),
                                       atol=1e-9, err_msg=a)
        DoubleWishboneSolver(adj).solve(0.0)      # still assembles

    def test_bulk_height_jump_removed(self):
        import dataclasses
        from suspension_tool.seed import apply_locked_points
        hp_new, _ = generate_seed(dataclasses.replace(self.sv,
                                                      kickup_deg=14.0))
        raw_dz = abs(hp_new.wheel_center[2] - self.ref.wheel_center[2])
        adj = apply_locked_points(hp_new, self.ref, self.locked)
        adj_dz = abs(adj.wheel_center[2] - self.ref.wheel_center[2])
        self.assertGreater(raw_dz, 25.0)          # the complaint
        self.assertLess(adj_dz, raw_dz / 2.0)     # the fix

    def test_track_not_shifted_by_locking(self):
        import dataclasses
        from suspension_tool.seed import apply_locked_points
        hp_new, _ = generate_seed(self.sv)
        adj = apply_locked_points(hp_new, self.ref, self.locked)
        self.assertAlmostEqual(adj.wheel_center[1], hp_new.wheel_center[1],
                               places=9)          # y translation forbidden

    def test_no_locks_is_identity(self):
        from suspension_tool.seed import apply_locked_points
        hp_new, _ = generate_seed(self.sv)
        self.assertIs(apply_locked_points(hp_new, self.ref, set()), hp_new)
        self.assertIs(apply_locked_points(hp_new, None, self.locked), hp_new)


class TestTierodOnArm(unittest.TestCase):
    """Toe link's inner ball joint mounted ON a control arm (lower OR
    upper). v1.18 corrects the v1.13.1 framing: this does NOT fix toe — it
    generally gives LARGE bump steer (the near-rigid BJ-inner-steering-arm
    triangle makes the steering arm follow that arm), minimized by seating
    the outboard link near that arm's ball joint on the kingpin."""

    @classmethod
    def setUpClass(cls):
        import dataclasses
        sv = dataclasses.replace(baseline_setup(), tierod_on_lca=True)
        cls.hp, cls.report = generate_seed(sv)
        cls.solver = DoubleWishboneSolver(cls.hp)
        cls.hp_chassis, _ = generate_seed(baseline_setup())
        svu = dataclasses.replace(baseline_setup(), tierod_on_uca=True)
        cls.hp_u, cls.report_u = generate_seed(svu)

    def test_flag_set_and_serializes(self):
        from suspension_tool.project import (AxleDesign, ProjectState,
                                             dict_to_project,
                                             project_to_dict)
        self.assertTrue(self.hp.tierod_on_lca)
        self.assertFalse(self.hp_chassis.tierod_on_lca)
        ps = ProjectState(front=AxleDesign(self.hp, None, 75.0, 75.0),
                          rear=None, unit_key="mm")
        back = dict_to_project(project_to_dict(ps))
        self.assertTrue(back.front.hardpoints.tierod_on_lca)

    def test_inner_sits_on_the_arm(self):
        mid = (self.hp.lca_inner_front + self.hp.lca_inner_rear) / 2.0
        seg = self.hp.lca_outer - mid
        t = np.dot(self.hp.tierod_inner - mid, seg) / np.dot(seg, seg)
        off = np.linalg.norm(self.hp.tierod_inner - (mid + t * seg))
        self.assertLess(off, 1e-6)          # ON the arm line
        self.assertGreater(t, 0.1)          # off the pivot axis
        self.assertLess(t, 0.95)            # inside the ball joint

    def test_inner_swings_with_the_arm(self):
        st0 = self.solver.solve(0.0)
        st = self.solver.solve(40.0)
        np.testing.assert_allclose(st0.tierod_inner_cur,
                                   self.hp.tierod_inner, atol=1e-9)
        moved = np.linalg.norm(st.tierod_inner_cur - st0.tierod_inner_cur)
        self.assertGreater(moved, 5.0)
        # and the link is rigid to the MOVING end, not the static point
        L0 = np.linalg.norm(self.hp.tierod_outer - self.hp.tierod_inner)
        L = np.linalg.norm(st.tro - st.tierod_inner_cur)
        self.assertAlmostEqual(L, L0, places=6)

    def test_chassis_mounted_inner_stays_put(self):
        s = DoubleWishboneSolver(self.hp_chassis)
        st = s.solve(40.0)
        np.testing.assert_allclose(st.tierod_inner_cur,
                                   self.hp_chassis.tierod_inner, atol=1e-9)

    def test_arm_mount_does_not_fix_toe(self):
        # An arm-mounted link generally has substantial bump steer — much
        # larger than a chassis link tuned to ~zero. The seed reports the
        # real value (does NOT pretend it is fixed to zero).
        chas = generate_seed(baseline_setup())[1].bump_steer_static
        self.assertLess(abs(chas), 5e-3)            # chassis tuned ~flat
        self.assertGreater(abs(self.report.bump_steer_static), 0.01)

    def test_outboard_seated_low_for_lower_high_for_upper(self):
        # lower mount -> outboard link near the LBJ (low on the kingpin);
        # upper mount -> near the UBJ (high). Keeps the tie-rod IC close to
        # the arm's IC so the (unavoidable) bump steer is minimized.
        span_l = self.hp.uca_outer[2] - self.hp.lca_outer[2]
        f_low = (self.hp.tierod_outer[2] - self.hp.lca_outer[2]) / span_l
        self.assertLess(f_low, 0.2)                 # low, near LBJ
        f_high = (self.hp_u.tierod_outer[2]
                  - self.hp_u.lca_outer[2]) / span_l
        self.assertGreater(f_high, 0.8)             # high, near UBJ

    def test_bump_steer_tunable_by_outboard_height(self):
        # dropping the outboard link lower on the kingpin reduces lower-arm
        # bump steer (the design lever, since the inner position can't).
        import dataclasses

        from suspension_tool.metrics import toe_deg
        king = self.hp.uca_outer - self.hp.lca_outer

        def bs(frac):
            on = self.hp.lca_outer + frac * king
            hp = dataclasses.replace(self.hp, tierod_outer=np.array(
                [self.hp.tierod_outer[0], on[1], on[2]]))
            s = DoubleWishboneSolver(hp)
            return abs(toe_deg(s.solve(40.0)) - toe_deg(s.solve(-40.0)))
        self.assertLess(bs(0.02), bs(0.5))          # lower = less bump steer

    def test_upper_arm_mount_inner_rides_uca(self):
        self.assertTrue(self.hp_u.tierod_on_uca)
        self.assertFalse(self.hp_u.tierod_on_lca)
        s = DoubleWishboneSolver(self.hp_u)
        st0, st40 = s.solve(0.0), s.solve(40.0)
        self.assertGreater(np.linalg.norm(st40.tierod_inner_cur
                                          - st0.tierod_inner_cur), 5.0)
        # it truly RIDES the UPPER arm: its distance to the UCA bushing
        # axis is preserved through travel (a rigid rotation about it), and
        # NOT the LCA axis
        uax = (self.hp_u.uca_inner_rear - self.hp_u.uca_inner_front)
        uax = uax / np.linalg.norm(uax)
        uo = self.hp_u.uca_inner_front

        def axis_dist(p):
            v = p - uo
            return np.linalg.norm(v - np.dot(v, uax) * uax)
        self.assertAlmostEqual(axis_dist(st40.tierod_inner_cur),
                               axis_dist(self.hp_u.tierod_inner), places=6)

    def test_articulates_full_travel(self):
        for hp, rep in ((self.hp, self.report), (self.hp_u, self.report_u)):
            s = DoubleWishboneSolver(hp)
            for t in np.linspace(-rep.droop_travel, rep.bump_travel, 15):
                s.solve(float(t))


if __name__ == "__main__":
    unittest.main()
