"""Verification for the relative-dimension tweaks (Phase 4.1)."""

import unittest

import numpy as np

from suspension_tool import metrics
from suspension_tool.seed import IN, SetupVariables, generate_seed
from suspension_tool.solver import DoubleWishboneSolver
from suspension_tool.tweaks import (LockedPointError, caster_angle_deg,
                                    scrub_radius, set_caster_angle_deg,
                                    set_scrub_radius, set_shock_mount,
                                    set_steer_arm_length, set_tierod_rises,
                                    shock_mount_params, steer_arm_length,
                                    tierod_rises)


def mk4_setup(**over) -> SetupVariables:
    kw = dict(track_width=63.0 * IN, wheelbase=61.0 * IN, ride_height=14.0 * IN,
              tire_radius=11.5 * IN, tire_width=7.0 * IN,
              shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
              shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85,
              static_camber_deg=-0.87, kickup_deg=10.0)
    kw.update(over)
    return SetupVariables(**kw)


class TestShockMountTweaks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hp, _ = generate_seed(mk4_setup())

    def test_read_write_round_trip(self):
        p = shock_mount_params(self.hp)
        hp2 = set_shock_mount(self.hp, p["d"], p["h"], p["ang"])
        np.testing.assert_allclose(hp2.shock_outer, self.hp.shock_outer,
                                   atol=1e-9)

    def test_set_values_read_back(self):
        hp2 = set_shock_mount(self.hp, 400.0, 25.0, 15.0)
        p = shock_mount_params(hp2)
        self.assertAlmostEqual(p["d"], 400.0, places=9)
        self.assertAlmostEqual(p["h"], 25.0, places=9)
        self.assertAlmostEqual(p["ang"], 15.0, places=9)

    def test_larger_d_moves_mount_toward_knuckle_and_drops_mr(self):
        p = shock_mount_params(self.hp)
        hp2 = set_shock_mount(self.hp, p["d"] + 60.0, p["h"], p["ang"])
        # closer to the knuckle -> more shock motion per wheel motion ->
        # LOWER wheel/shock motion ratio
        def mr(hp):
            s = DoubleWishboneSolver(hp)
            return 2.0 / (s.solve(-1.0).shock_length - s.solve(1.0).shock_length)
        self.assertLess(mr(hp2), mr(self.hp))
        # and the mount really is closer to the LBJ
        self.assertLess(np.linalg.norm(hp2.shock_outer - hp2.lca_outer),
                        np.linalg.norm(self.hp.shock_outer - self.hp.lca_outer))

    def test_angle_moves_mount_out_of_sketch_plane(self):
        p = shock_mount_params(self.hp)
        hp2 = set_shock_mount(self.hp, p["d"], max(p["h"], 20.0), 20.0)
        self.assertGreater(hp2.shock_outer[0], self.hp.shock_outer[0])
        hp3 = set_shock_mount(self.hp, p["d"], max(p["h"], 20.0), -20.0)
        self.assertLess(hp3.shock_outer[0], self.hp.shock_outer[0])

    def test_works_on_uca_mounted_shock(self):
        hp_u, _ = generate_seed(mk4_setup(shock_on_uca=True))
        p = shock_mount_params(hp_u)
        hp2 = set_shock_mount(hp_u, p["d"] + 30.0, p["h"] + 10.0, 10.0)
        q = shock_mount_params(hp2)
        self.assertAlmostEqual(q["d"], p["d"] + 30.0, places=8)
        self.assertAlmostEqual(q["h"], p["h"] + 10.0, places=8)
        self.assertAlmostEqual(q["ang"], 10.0, places=8)
        DoubleWishboneSolver(hp2).solve(25.0)   # still articulates


class TestTierodTweaks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hp, _ = generate_seed(mk4_setup())

    def test_read_write_round_trip(self):
        r = tierod_rises(self.hp)
        hp2 = set_tierod_rises(self.hp, r["outer_rise"], r["inner_rise"])
        np.testing.assert_allclose(hp2.tierod_outer, self.hp.tierod_outer, atol=1e-9)
        np.testing.assert_allclose(hp2.tierod_inner, self.hp.tierod_inner, atol=1e-9)

    def test_rise_changes_bump_steer(self):
        from suspension_tool.metrics import toe_deg
        def bs(hp):
            s = DoubleWishboneSolver(hp)
            return (toe_deg(s.solve(1.0)) - toe_deg(s.solve(-1.0))) / 2.0
        r = tierod_rises(self.hp)
        self.assertLess(abs(bs(self.hp)), 0.001)      # seed is tuned
        hp2 = set_tierod_rises(self.hp, r["outer_rise"] + 20.0, r["inner_rise"])
        self.assertGreater(abs(bs(hp2)), 0.005)       # de-tuned as expected
        # only z moved
        self.assertAlmostEqual(hp2.tierod_outer[0], self.hp.tierod_outer[0])
        self.assertAlmostEqual(hp2.tierod_outer[1], self.hp.tierod_outer[1])


class TestSteerArmTweak(unittest.TestCase):
    """Steering-arm length: kingpin axis -> tierod_outer along the sketch
    normal — the lever the rack pushes on."""

    @classmethod
    def setUpClass(cls):
        cls.hp, _ = generate_seed(mk4_setup())

    def test_hand_math(self):
        # independent computation of the same dimension
        hp = self.hp
        au = hp.uca_inner_front - hp.uca_inner_rear
        al = hp.lca_inner_front - hp.lca_inner_rear
        au /= np.linalg.norm(au)
        al /= np.linalg.norm(al)
        if np.dot(au, al) < 0:
            al = -al
        n = (au + al) / np.linalg.norm(au + al)
        a, b = hp.lca_outer, hp.uca_outer
        d = (b - a) / np.linalg.norm(b - a)
        p = a + np.dot(hp.tierod_outer - a, d) * d
        self.assertAlmostEqual(steer_arm_length(hp),
                               float(np.dot(hp.tierod_outer - p, n)),
                               places=9)

    def test_read_write_round_trip(self):
        hp2 = set_steer_arm_length(self.hp, steer_arm_length(self.hp))
        np.testing.assert_allclose(hp2.tierod_outer, self.hp.tierod_outer,
                                   atol=1e-9)

    def test_set_moves_only_along_sketch_normal(self):
        L0 = steer_arm_length(self.hp)
        hp2 = set_steer_arm_length(self.hp, L0 + 25.0)
        self.assertAlmostEqual(steer_arm_length(hp2), L0 + 25.0, places=9)
        # the move is purely along the sketch normal
        delta = hp2.tierod_outer - self.hp.tierod_outer
        dn = delta / np.linalg.norm(delta)
        au = self.hp.uca_inner_front - self.hp.uca_inner_rear
        self.assertAlmostEqual(
            abs(float(np.dot(dn, au / np.linalg.norm(au)))), 1.0, places=9)
        # nothing else moved
        np.testing.assert_allclose(hp2.tierod_inner, self.hp.tierod_inner)
        np.testing.assert_allclose(hp2.lca_outer, self.hp.lca_outer)

    def test_longer_arm_slows_the_steering(self):
        # same rack input -> smaller road-wheel angle with a longer arm
        from suspension_tool.metrics import toe_deg
        L0 = steer_arm_length(self.hp)
        sgn = 1.0 if L0 >= 0 else -1.0
        long_arm = set_steer_arm_length(self.hp, L0 + sgn * 30.0)
        a0 = abs(toe_deg(DoubleWishboneSolver(self.hp).solve(0.0, 20.0))
                 - toe_deg(DoubleWishboneSolver(self.hp).solve(0.0)))
        a1 = abs(toe_deg(DoubleWishboneSolver(long_arm).solve(0.0, 20.0))
                 - toe_deg(DoubleWishboneSolver(long_arm).solve(0.0)))
        self.assertLess(a1, a0)


class TestSketchDimensionTweaks(unittest.TestCase):
    """v1.9: kingpin length, inboard UC-LC axis gap, in-sketch arm
    lengths — the numbers dimensioned directly on the 2D sketch."""

    @classmethod
    def setUpClass(cls):
        cls.hp, _ = generate_seed(mk4_setup())

    def test_kingpin_length_moves_only_ubj_along_axis(self):
        from suspension_tool.tweaks import (kingpin_length,
                                            set_kingpin_length)
        L0 = kingpin_length(self.hp)
        hp2 = set_kingpin_length(self.hp, L0 + 25.0)
        self.assertAlmostEqual(kingpin_length(hp2), L0 + 25.0, places=9)
        np.testing.assert_allclose(hp2.lca_outer, self.hp.lca_outer)
        np.testing.assert_allclose(hp2.wheel_center, self.hp.wheel_center)
        d0 = (self.hp.uca_outer - self.hp.lca_outer)
        d0 /= np.linalg.norm(d0)
        d2 = (hp2.uca_outer - hp2.lca_outer)
        d2 /= np.linalg.norm(d2)
        np.testing.assert_allclose(d2, d0, atol=1e-12)   # same axis dir

    def test_axis_sep_hand_math_and_setter(self):
        from suspension_tool.tweaks import (inboard_axis_sep,
                                            set_inboard_axis_sep)
        hp = self.hp
        # seed axes are parallel: hand math = perpendicular distance
        dl = hp.lca_inner_front - hp.lca_inner_rear
        dl /= np.linalg.norm(dl)
        rel = ((hp.uca_inner_front + hp.uca_inner_rear) / 2
               - (hp.lca_inner_front + hp.lca_inner_rear) / 2)
        hand = np.linalg.norm(rel - np.dot(rel, dl) * dl)
        s0 = inboard_axis_sep(hp)
        self.assertAlmostEqual(s0, hand, places=9)
        hp2 = set_inboard_axis_sep(hp, s0 + 20.0)
        self.assertAlmostEqual(inboard_axis_sep(hp2), s0 + 20.0, places=9)
        # lower pair + BJs untouched; upper pair translated rigidly
        np.testing.assert_allclose(hp2.lca_inner_front, hp.lca_inner_front)
        np.testing.assert_allclose(hp2.uca_outer, hp.uca_outer)
        np.testing.assert_allclose(
            hp2.uca_inner_front - hp2.uca_inner_rear,
            hp.uca_inner_front - hp.uca_inner_rear, atol=1e-9)

    def test_arm_length_dials_camber_gain(self):
        from suspension_tool.metrics import camber_deg
        from suspension_tool.tweaks import (arm_length_sketch,
                                            set_arm_length_sketch)
        L0 = arm_length_sketch(self.hp, True)
        hp2 = set_arm_length_sketch(self.hp, True, L0 - 20.0)
        self.assertAlmostEqual(arm_length_sketch(hp2, True), L0 - 20.0,
                               places=9)
        # caster offset (sketch-normal component) preserved
        au = self.hp.uca_inner_front - self.hp.uca_inner_rear
        au /= np.linalg.norm(au)
        mid = (self.hp.uca_inner_front + self.hp.uca_inner_rear) / 2
        self.assertAlmostEqual(
            float(np.dot(hp2.uca_outer - mid, au)),
            float(np.dot(self.hp.uca_outer - mid, au)), places=9)
        # a shorter upper arm gains more negative camber in bump
        def gain(hp):
            s = DoubleWishboneSolver(hp)
            return (camber_deg(s.solve(40.0)) - camber_deg(s.solve(0.0)))
        self.assertLess(gain(hp2), gain(self.hp))

    def test_round_trips(self):
        from suspension_tool.tweaks import (
            arm_length_sketch, inboard_axis_sep, kingpin_length,
            set_arm_length_sketch, set_inboard_axis_sep,
            set_kingpin_length)
        hp = set_kingpin_length(self.hp, kingpin_length(self.hp))
        hp = set_inboard_axis_sep(hp, inboard_axis_sep(hp))
        hp = set_arm_length_sketch(hp, True, arm_length_sketch(hp, True))
        hp = set_arm_length_sketch(hp, False,
                                   arm_length_sketch(hp, False))
        for a in self.hp.POINT_ATTRS:
            np.testing.assert_allclose(getattr(hp, a),
                                       getattr(self.hp, a), atol=1e-9)


class TestToeByTierod(unittest.TestCase):
    """v1.10: the mechanic's toe adjustment — thread the tie rod, the
    knuckle rotates about the kingpin, geometry and stored alignment
    stay consistent."""

    @classmethod
    def setUpClass(cls):
        cls.hp, _ = generate_seed(mk4_setup())

    def test_toe_lands_exactly_and_geometry_is_consistent(self):
        from suspension_tool.metrics import toe_deg
        from suspension_tool.tweaks import set_toe_by_tierod
        hp2 = set_toe_by_tierod(self.hp, 0.75)
        self.assertAlmostEqual(hp2.static_toe_deg, 0.75, places=9)
        st = DoubleWishboneSolver(hp2).solve(0.0)
        self.assertAlmostEqual(toe_deg(st), 0.75, places=6)
        # the link really changed length (that IS the adjustment)...
        L0 = np.linalg.norm(self.hp.tierod_outer - self.hp.tierod_inner)
        L1 = np.linalg.norm(hp2.tierod_outer - hp2.tierod_inner)
        self.assertGreater(abs(L1 - L0), 0.5)
        # ...while the knuckle stayed rigid (distances to both BJs kept)
        for p in ("lca_outer", "uca_outer"):
            self.assertAlmostEqual(
                np.linalg.norm(hp2.tierod_outer - getattr(hp2, p)),
                np.linalg.norm(self.hp.tierod_outer - getattr(self.hp, p)),
                places=9)
            self.assertAlmostEqual(
                np.linalg.norm(hp2.wheel_center - getattr(hp2, p)),
                np.linalg.norm(self.hp.wheel_center - getattr(self.hp, p)),
                places=9)
        # camber couples slightly through the leaned kingpin (real
        # effect), and the stored value tracks it
        self.assertNotAlmostEqual(hp2.static_camber_deg,
                                  self.hp.static_camber_deg, places=3)

    def test_round_trip(self):
        from suspension_tool.tweaks import set_toe_by_tierod
        back = set_toe_by_tierod(set_toe_by_tierod(self.hp, 1.0), 0.0)
        self.assertAlmostEqual(back.static_toe_deg, 0.0, places=9)
        np.testing.assert_allclose(back.tierod_outer,
                                   self.hp.tierod_outer, atol=1e-6)


class TestHubCvOffset(unittest.TestCase):
    """v1.12: hub offset = outer-CV -> tire-centre distance (the hub/knuckle
    stickout). Sliding the wheel centre along its spin axis changes scrub
    but leaves camber / caster / KPI alone."""

    @classmethod
    def setUpClass(cls):
        cls.hp, _ = generate_seed(mk4_setup())

    def test_read_write_round_trip(self):
        from suspension_tool.tweaks import hub_cv_offset, set_hub_cv_offset
        hp2 = set_hub_cv_offset(self.hp, hub_cv_offset(self.hp))
        np.testing.assert_allclose(hp2.wheel_center, self.hp.wheel_center,
                                   atol=1e-6)

    def test_set_lands_and_preserves_kingpin_geometry(self):
        from suspension_tool.metrics import camber_deg, caster_deg
        from suspension_tool.tweaks import hub_cv_offset, set_hub_cv_offset
        d0 = hub_cv_offset(self.hp)
        hp2 = set_hub_cv_offset(self.hp, d0 + 20.0)
        self.assertAlmostEqual(hub_cv_offset(hp2), d0 + 20.0, places=6)
        # ball joints and kingpin untouched
        np.testing.assert_allclose(hp2.lca_outer, self.hp.lca_outer)
        np.testing.assert_allclose(hp2.uca_outer, self.hp.uca_outer)
        # camber & caster unchanged (spindle direction preserved)
        s0 = DoubleWishboneSolver(self.hp).solve(0.0)
        s2 = DoubleWishboneSolver(hp2).solve(0.0)
        self.assertAlmostEqual(camber_deg(s2), camber_deg(s0), places=6)
        self.assertAlmostEqual(caster_deg(s2), caster_deg(s0), places=6)
        # the wheel moved outboard (bigger stickout)
        self.assertGreater(hp2.wheel_center[1], self.hp.wheel_center[1])


class TestShockLengthAtRide(unittest.TestCase):
    """v1.12: set the installed shock length at ride by sliding the arm
    mount, and report the bump/droop stroke split."""

    @classmethod
    def setUpClass(cls):
        cls.sv = mk4_setup()
        cls.hp, _ = generate_seed(cls.sv)

    def test_round_trip(self):
        from suspension_tool.tweaks import (set_shock_length_at_ride,
                                            shock_length_at_ride)
        L0 = shock_length_at_ride(self.hp)
        hp2 = set_shock_length_at_ride(self.hp, L0)
        np.testing.assert_allclose(hp2.shock_outer, self.hp.shock_outer,
                                   atol=1e-6)

    def test_set_lands_and_keeps_h_ang(self):
        from suspension_tool.tweaks import (set_shock_length_at_ride,
                                            shock_length_at_ride,
                                            shock_mount_params)
        p0 = shock_mount_params(self.hp)
        L0 = shock_length_at_ride(self.hp)
        hp2 = set_shock_length_at_ride(self.hp, L0 - 15.0)
        self.assertAlmostEqual(shock_length_at_ride(hp2), L0 - 15.0, places=6)
        p2 = shock_mount_params(hp2)
        self.assertAlmostEqual(p2["h"], p0["h"], places=6)
        self.assertAlmostEqual(p2["ang"], p0["ang"], places=6)
        # chassis end never moves
        np.testing.assert_allclose(hp2.shock_inner, self.hp.shock_inner)

    def test_split_sums_to_one_and_tracks_length(self):
        from suspension_tool.tweaks import (set_shock_length_at_ride,
                                            shock_travel_split)
        lo, hi = self.sv.shock_min_length, self.sv.shock_max_length
        s = shock_travel_split(self.hp, lo, hi)
        self.assertAlmostEqual(s["bump_frac"] + s["droop_frac"], 1.0, places=9)
        # a longer installed length reserves MORE bump (compression) budget
        longer = set_shock_length_at_ride(self.hp, s["length"] + 20.0)
        s2 = shock_travel_split(longer, lo, hi)
        self.assertGreater(s2["bump_frac"], s["bump_frac"])


class TestRideHeightTweak(unittest.TestCase):
    """v1.12: ride height is a PLACEMENT knob — a rigid z shift that changes
    no kinematics, datum-corrected by the frame-tube offset."""

    @classmethod
    def setUpClass(cls):
        cls.hp, _ = generate_seed(mk4_setup())

    def test_round_trip(self):
        from suspension_tool.tweaks import ride_height, set_ride_height
        rh = ride_height(self.hp, 15.875)
        hp2 = set_ride_height(self.hp, rh, 15.875)
        for a in self.hp.POINT_ATTRS:
            np.testing.assert_allclose(getattr(hp2, a), getattr(self.hp, a),
                                       atol=1e-9)

    def test_frame_tube_offset_shifts_the_datum(self):
        from suspension_tool.tweaks import ride_height
        self.assertAlmostEqual(
            ride_height(self.hp, 0.0) - ride_height(self.hp, 10.0),
            10.0, places=9)

    def test_set_is_a_rigid_z_translation_no_kinematics(self):
        from suspension_tool.metrics import camber_deg
        from suspension_tool.tweaks import ride_height, set_ride_height
        rh = ride_height(self.hp, 15.875)
        hp2 = set_ride_height(self.hp, rh + 30.0, 15.875)
        self.assertAlmostEqual(ride_height(hp2, 15.875), rh + 30.0, places=6)
        # every point shifted by the SAME z, x and y untouched
        deltas = np.array([getattr(hp2, a) - getattr(self.hp, a)
                           for a in self.hp.POINT_ATTRS])
        np.testing.assert_allclose(deltas[:, 0], 0.0, atol=1e-9)
        np.testing.assert_allclose(deltas[:, 1], 0.0, atol=1e-9)
        self.assertTrue(np.allclose(deltas[:, 2], deltas[0, 2]))
        # camber curve identical (pure translation)
        def gain(hp):
            s = DoubleWishboneSolver(hp)
            return camber_deg(s.solve(30.0)) - camber_deg(s.solve(0.0))
        self.assertAlmostEqual(gain(hp2), gain(self.hp), places=9)


class TestScrubRadiusTweak(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hp, _ = generate_seed(mk4_setup(desired_scrub_radius=20.0))

    def _scrub_metric(self, hp):
        return metrics.scrub_radius_mm(DoubleWishboneSolver(hp).solve(0.0))

    def test_read_matches_displayed_metric(self):
        self.assertAlmostEqual(scrub_radius(self.hp),
                               self._scrub_metric(self.hp), places=1)

    def test_set_reads_back(self):
        hp2 = set_scrub_radius(self.hp, 35.0)
        self.assertAlmostEqual(scrub_radius(hp2), 35.0, places=4)
        self.assertAlmostEqual(self._scrub_metric(hp2), 35.0, places=1)

    def test_preserves_track_and_alignment(self):
        hp2 = set_scrub_radius(self.hp, 40.0)
        self.assertAlmostEqual(hp2.wheel_center[1], self.hp.wheel_center[1],
                               places=9)               # track unchanged
        self.assertAlmostEqual(hp2.static_camber_deg,
                               self.hp.static_camber_deg, places=9)
        self.assertAlmostEqual(hp2.static_toe_deg,
                               self.hp.static_toe_deg, places=9)
        np.testing.assert_allclose(hp2.lca_outer, self.hp.lca_outer, atol=1e-9)

    def test_does_not_disturb_caster(self):
        c0 = caster_angle_deg(self.hp)
        hp2 = set_scrub_radius(self.hp, 45.0)
        self.assertAlmostEqual(caster_angle_deg(hp2), c0, places=6)

    def test_round_trip_is_identity(self):
        hp2 = set_scrub_radius(self.hp, scrub_radius(self.hp))
        np.testing.assert_allclose(hp2.uca_outer, self.hp.uca_outer, atol=1e-6)

    def test_locked_ubj_raises(self):
        with self.assertRaises(LockedPointError) as cm:
            set_scrub_radius(self.hp, 30.0, locked={"uca_outer"})
        self.assertEqual(cm.exception.point, "uca_outer")


class TestCornerXTweak(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hp, _ = generate_seed(mk4_setup())

    def test_reads_wheel_centre_station(self):
        from suspension_tool.tweaks import corner_x
        self.assertAlmostEqual(corner_x(self.hp),
                               float(self.hp.wheel_center[0]), places=9)

    def test_rigid_x_shift_only(self):
        from suspension_tool.tweaks import corner_x, set_corner_x
        hp2 = set_corner_x(self.hp, corner_x(self.hp) + 40.0)
        deltas = np.array([getattr(hp2, a) - getattr(self.hp, a)
                           for a in self.hp.POINT_ATTRS])
        np.testing.assert_allclose(deltas[:, 0], 40.0, atol=1e-9)
        np.testing.assert_allclose(deltas[:, 1:], 0.0, atol=1e-9)

    def test_kinematics_unchanged(self):
        from suspension_tool.metrics import camber_deg
        from suspension_tool.tweaks import corner_x, set_corner_x
        hp2 = set_corner_x(self.hp, corner_x(self.hp) - 60.0)

        def gain(hp):
            s = DoubleWishboneSolver(hp)
            return camber_deg(s.solve(30.0)) - camber_deg(s.solve(0.0))
        self.assertAlmostEqual(gain(hp2), gain(self.hp), places=9)


class TestRigidMovesRefuseLocks(unittest.TestCase):
    """v1.16: NOTHING moves a locked point — rigid whole-corner
    translations (ride height, corner X) must refuse, not drag it."""

    @classmethod
    def setUpClass(cls):
        cls.hp, _ = generate_seed(mk4_setup())

    def test_ride_height_refuses(self):
        from suspension_tool.tweaks import ride_height, set_ride_height
        with self.assertRaises(LockedPointError) as cm:
            set_ride_height(self.hp, ride_height(self.hp) + 20.0,
                            locked={"lca_inner_front"})
        self.assertEqual(cm.exception.point, "lca_inner_front")

    def test_corner_x_refuses(self):
        from suspension_tool.tweaks import corner_x, set_corner_x
        with self.assertRaises(LockedPointError):
            set_corner_x(self.hp, corner_x(self.hp) + 20.0,
                         locked={"shock_inner"})

    def test_noop_targets_pass_even_when_locked(self):
        from suspension_tool.tweaks import (corner_x, ride_height,
                                            set_corner_x, set_ride_height)
        hp2 = set_ride_height(self.hp, ride_height(self.hp),
                              locked={"lca_inner_front"})
        self.assertIs(hp2, self.hp)
        hp3 = set_corner_x(self.hp, corner_x(self.hp),
                           locked={"shock_inner"})
        self.assertIs(hp3, self.hp)


class TestSketchKickupTweak(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hp, _ = generate_seed(mk4_setup())

    def test_set_reads_back_and_caster_follows(self):
        from suspension_tool.tweaks import set_sketch_kickup, sketch_kickup
        hp2 = set_sketch_kickup(self.hp, 15.0)
        self.assertAlmostEqual(sketch_kickup(hp2), 15.0, places=6)
        self.assertAlmostEqual(
            metrics.caster_deg(DoubleWishboneSolver(hp2).solve(0.0)),
            15.0, delta=0.01)      # planarize: caster follows kickup

    def test_midpoints_spreads_yaw_and_rest_unchanged(self):
        from suspension_tool.geometry import sketch_planarity
        from suspension_tool.tweaks import set_sketch_kickup
        hp2 = set_sketch_kickup(self.hp, 17.0)
        for f, r in (("uca_inner_front", "uca_inner_rear"),
                     ("lca_inner_front", "lca_inner_rear")):
            np.testing.assert_allclose(
                (getattr(hp2, f) + getattr(hp2, r)) / 2.0,
                (getattr(self.hp, f) + getattr(self.hp, r)) / 2.0,
                atol=1e-9)         # tube centres stay put
            self.assertAlmostEqual(
                np.linalg.norm(getattr(hp2, f) - getattr(hp2, r)),
                np.linalg.norm(getattr(self.hp, f) - getattr(self.hp, r)),
                places=9)          # spreads kept
        self.assertAlmostEqual(sketch_planarity(hp2)["axis_yaw_deg"],
                               sketch_planarity(self.hp)["axis_yaw_deg"],
                               places=6)
        # knuckle/shock side untouched (UBJ moves via planarize only)
        for attr in ("lca_outer", "wheel_center", "shock_inner",
                     "shock_outer", "tierod_inner"):
            np.testing.assert_allclose(getattr(hp2, attr),
                                       getattr(self.hp, attr), atol=1e-9)

    def test_no_planarize_keeps_ubj(self):
        from suspension_tool.tweaks import set_sketch_kickup
        hp2 = set_sketch_kickup(self.hp, 15.0, planarize=False)
        np.testing.assert_allclose(hp2.uca_outer, self.hp.uca_outer,
                                   atol=1e-9)

    def test_locked_bushing_raises(self):
        from suspension_tool.tweaks import set_sketch_kickup
        with self.assertRaises(LockedPointError) as cm:
            set_sketch_kickup(self.hp, 15.0, locked={"lca_inner_front"})
        self.assertEqual(cm.exception.point, "lca_inner_front")


class TestCasterTweak(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hp, _ = generate_seed(mk4_setup())

    def _caster_metric(self, hp):
        return metrics.caster_deg(DoubleWishboneSolver(hp).solve(0.0))

    def test_read_matches_displayed_metric(self):
        self.assertAlmostEqual(caster_angle_deg(self.hp),
                               self._caster_metric(self.hp), places=1)

    def test_set_reads_back(self):
        hp2 = set_caster_angle_deg(self.hp, 6.0)
        self.assertAlmostEqual(caster_angle_deg(hp2), 6.0, places=4)
        self.assertAlmostEqual(self._caster_metric(hp2), 6.0, places=1)

    def test_does_not_disturb_scrub_or_track(self):
        s0 = scrub_radius(self.hp)
        hp2 = set_caster_angle_deg(self.hp, 7.0)
        self.assertAlmostEqual(scrub_radius(hp2), s0, places=4)
        self.assertAlmostEqual(hp2.wheel_center[1], self.hp.wheel_center[1],
                               places=9)

    def test_locked_ubj_raises(self):
        with self.assertRaises(LockedPointError) as cm:
            set_caster_angle_deg(self.hp, 5.0, locked={"uca_outer"})
        self.assertEqual(cm.exception.point, "uca_outer")


if __name__ == "__main__":
    unittest.main()
