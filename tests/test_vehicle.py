"""Verification for vehicle-level math (roll axis, CG numbers), the
side-view IC, and the Onshape Variable-Studio sync (mocked transport)."""

import unittest

import numpy as np

from suspension_tool.onshape_sync import (OnshapeClient, build_variables,
                                          parse_document_url,
                                          push_hardpoints)
from suspension_tool.metrics import front_view_ic, side_view_ic
from suspension_tool.seed import IN, SetupVariables, generate_seed
from suspension_tool.solver import DoubleWishboneSolver
from suspension_tool.vehicle import (VehicleParams, roll_axis_metrics,
                                     roll_axis_points)


def mk4_setup() -> SetupVariables:
    return SetupVariables(
        track_width=63.0 * IN, wheelbase=61.0 * IN, ride_height=14.0 * IN,
        tire_radius=11.5 * IN, tire_width=7.0 * IN,
        shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
        shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85,
        static_camber_deg=-0.87, kickup_deg=10.0)


class TestRollAxis(unittest.TestCase):
    def test_points_and_metrics_by_hand(self):
        # front RC 300 mm, rear RC 400 mm, wheelbase 1500 mm, CG 600 mm
        # high at 40% of the wheelbase behind the front axle:
        #   axis height at CG = 300 + (400-300)*0.4 = 340
        #   moment arm        = 600 - 340 = 260
        #   angle             = atan(100/1500) = 3.8141 deg
        params = VehicleParams(wheelbase=1500.0, cg_height=600.0,
                               cg_behind_front=600.0)
        m = roll_axis_metrics(300.0, 400.0, params)
        self.assertAlmostEqual(m["roll_axis_height_at_cg_mm"], 340.0, places=9)
        self.assertAlmostEqual(m["roll_moment_arm_mm"], 260.0, places=9)
        self.assertAlmostEqual(m["roll_axis_angle_deg"],
                               np.degrees(np.arctan2(100.0, 1500.0)), places=9)
        self.assertAlmostEqual(m["front_weight_frac"], 0.6, places=9)
        pf, pr = roll_axis_points(300.0, 400.0, 1500.0)
        np.testing.assert_allclose(pf, [0.0, 0.0, 300.0])
        np.testing.assert_allclose(pr, [-1500.0, 0.0, 400.0])


class TestSideViewIC(unittest.TestCase):
    def test_parallel_axes_give_ic_at_infinity(self):
        # This tool's seeds keep both bushing axes normal to the sketch
        # plane, so the side-view traces are parallel -> IC is None.
        hp, _ = generate_seed(mk4_setup())
        solver = DoubleWishboneSolver(hp)
        self.assertIsNone(side_view_ic(solver, solver.solve(0.0)))

    def test_skewed_axes_give_finite_ic(self):
        # Skew the upper axis (anti-dive style) and the IC must appear.
        import dataclasses
        hp, _ = generate_seed(mk4_setup())
        hp = dataclasses.replace(
            hp, uca_inner_front=hp.uca_inner_front + np.array([0, 0, 25.0]))
        solver = DoubleWishboneSolver(hp)
        ic = side_view_ic(solver, solver.solve(0.0))
        self.assertIsNotNone(ic)
        self.assertTrue(np.all(np.isfinite(ic)))


class TestAntiGeometryKinematic(unittest.TestCase):
    """The kinematic anti percentages must reflect the real side-view
    geometry, NOT collapse toward zero. Guards the v1.11.5 'anti reads
    extremely small' report: a kicked-up seed has parallel side-view arms
    (IC at infinity), yet the anti still comes out substantial through the
    straight-line-path fallback."""

    def test_kicked_up_seed_has_substantial_anti(self):
        from suspension_tool.vehicle import VehicleParams, anti_geometry
        f, _ = generate_seed(mk4_setup())
        r, _ = generate_seed(mk4_setup())
        fs, rs = DoubleWishboneSolver(f), DoubleWishboneSolver(r)
        anti = anti_geometry((fs, fs.solve(0.0)), (rs, rs.solve(0.0)),
                             VehicleParams())
        # 10 deg kickup -> the side-view wheel path is steeply inclined, so
        # the anti magnitudes are tens of percent, never a rounding-to-zero
        self.assertGreater(abs(anti["anti_dive_front_pct"]), 10.0)
        self.assertGreater(abs(anti["anti_squat_rear_pct"]), 10.0)
        self.assertGreater(abs(anti["anti_lift_rear_pct"]), 5.0)

    def test_near_parallel_import_still_finite_and_substantial(self):
        # CAD-imported points are ALMOST parallel in side view (rounded),
        # so side_view_ic returns a very distant finite IC — the anti must
        # still be substantial, matching the exactly-parallel seed.
        import dataclasses

        from suspension_tool.vehicle import VehicleParams, anti_geometry
        hp, _ = generate_seed(mk4_setup())
        # nudge one bushing by a hair: breaks exact parallelism
        hp = dataclasses.replace(
            hp, uca_inner_front=hp.uca_inner_front + np.array([0, 0, 1e-3]))
        s = DoubleWishboneSolver(hp)
        anti = anti_geometry((s, s.solve(0.0)), (s, s.solve(0.0)),
                             VehicleParams())
        self.assertGreater(abs(anti["anti_dive_front_pct"]), 10.0)

    def test_anti_continuous_through_parallel_arm_state(self):
        # REGRESSION: the IC-at-infinity fallback used to finite-difference
        # the wheel path, disagreeing with the finite arm-plane branch by
        # ~2-4 percentage points right at the designed (parallel-arm) seed.
        # Now the fallback uses the arm-plane line direction, so the anti %
        # is continuous as the arms go parallel and equals the tan(kickup)
        # geometric limit.
        import dataclasses

        from suspension_tool.vehicle import VehicleParams, anti_geometry
        p = VehicleParams(wheelbase=1549.4, cg_height=558.8,
                          brake_front_frac=0.6)
        hp, _ = generate_seed(mk4_setup())          # kickup 10 deg, parallel
        s = DoubleWishboneSolver(hp)
        a0 = anti_geometry((s, s.solve(0.0)), (s, s.solve(0.0)), p)
        # geometric limit: support slope = tan(kickup)
        want = 100.0 * np.tan(np.radians(10.0)) * 1549.4 / 558.8
        self.assertAlmostEqual(a0["anti_squat_rear_pct"], want, delta=0.05)
        # an infinitesimal skew flips to the finite branch: must NOT jump
        r2 = dataclasses.replace(
            hp, lca_inner_rear=hp.lca_inner_rear + np.array([0.02, 0, 0]))
        s2 = DoubleWishboneSolver(r2)
        a1 = anti_geometry((s, s.solve(0.0)), (s2, s2.solve(0.0)), p)
        self.assertLess(
            abs(a1["anti_squat_rear_pct"] - a0["anti_squat_rear_pct"]), 0.1)


class TestViewICExact(unittest.TestCase):
    """The analytic front-/side-view instant centres (double wishbone) are
    constructed by slicing each arm plane at THAT arm's ball-joint station.
    They must therefore equal the true rigid-body instant centre found by
    finite-differencing two rigid upright points — even when the arms are
    inclined fore/aft (kickup, anti, independent caster), the case where
    the old shared-wheel-centre-plane construction was wrong by tens of mm.
    """

    @staticmethod
    def _rigid_ic(solver, state, axes, h=0.25):
        """Ground-truth IC: central-difference two RIGID upright points
        (ubj, lbj) and intersect the perpendiculars in the projection."""
        lo, hi = solver.solve(state.travel - h), solver.solve(state.travel + h)
        i, j = axes
        lines = []
        for attr in ("ubj", "lbj"):
            p = np.array([getattr(state, attr)[i], getattr(state, attr)[j]])
            v = np.array([getattr(hi, attr)[i] - getattr(lo, attr)[i],
                          getattr(hi, attr)[j] - getattr(lo, attr)[j]])
            lines.append((p, np.array([-v[1], v[0]])))
        (p1, n1), (p2, n2) = lines
        s = np.linalg.solve(np.column_stack([n1, -n2]), p2 - p1)
        ic = p1 + s[0] * n1
        return float(ic[0]), float(ic[1])

    def _cases(self):
        base = dict(track_width=63 * IN, wheelbase=61 * IN, ride_height=14 * IN,
                    tire_radius=11.5 * IN, tire_width=7 * IN,
                    shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
                    motion_ratio_goal=1.0 / 1.85, shock_length_at_ride=20.38 * IN,
                    static_camber_deg=-0.87)
        for kw in (dict(kickup_deg=10.0), dict(kickup_deg=20.0),
                   dict(kickup_deg=15.0, desired_caster_deg=12.0,
                        independent_caster=True),
                   dict(kickup_deg=10.0, sketch_yaw_deg=20.0)):
            hp, _ = generate_seed(SetupVariables(**{**base, **kw}))
            yield kw, DoubleWishboneSolver(hp)

    def test_front_view_ic_matches_rigid_body(self):
        for kw, s in self._cases():
            st = s.solve(0.0)
            ic = front_view_ic(s, st)
            rig = self._rigid_ic(s, st, (1, 2))
            self.assertLess(abs(ic[0] - rig[0]), 0.6, kw)   # y (mm)
            self.assertLess(abs(ic[1] - rig[1]), 0.6, kw)   # z (mm)

    def test_side_view_ic_matches_rigid_body(self):
        # skew the arms so the side-view IC is finite (anti geometry)
        import dataclasses
        base = dict(track_width=63 * IN, wheelbase=61 * IN, ride_height=14 * IN,
                    tire_radius=11.5 * IN, tire_width=7 * IN,
                    shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
                    motion_ratio_goal=1.0 / 1.85, shock_length_at_ride=20.38 * IN,
                    static_camber_deg=-0.87, kickup_deg=10.0)
        hp, _ = generate_seed(SetupVariables(**base))
        hp = dataclasses.replace(
            hp, lca_inner_rear=hp.lca_inner_rear + np.array([0.0, 0.0, 30.0]),
            uca_inner_rear=hp.uca_inner_rear + np.array([0.0, 0.0, 15.0]))
        s = DoubleWishboneSolver(hp)
        st = s.solve(0.0)
        ic = side_view_ic(s, st)
        self.assertIsNotNone(ic)               # finite (skewed) side-view IC
        rig = self._rigid_ic(s, st, (0, 2))
        # far-out IC: a small relative tolerance on both coordinates
        self.assertLess(abs(ic[0] - rig[0]), 0.02 * abs(rig[0]) + 1.0)
        self.assertLess(abs(ic[1] - rig[1]), 0.02 * abs(rig[1]) + 1.0)


class TestOnshapeSync(unittest.TestCase):
    def test_parse_document_url(self):
        did, wid = parse_document_url(
            "https://cad.onshape.com/documents/0123456789abcdef01234567"
            "/w/89abcdef0123456789abcdef/e/deadbeefdeadbeefdeadbeef")
        self.assertEqual(did, "0123456789abcdef01234567")
        self.assertEqual(wid, "89abcdef0123456789abcdef")
        with self.assertRaises(ValueError):
            parse_document_url("https://cad.onshape.com/whatever")

    def test_build_variables_vehicle_frame(self):
        hp, _ = generate_seed(mk4_setup())
        wb = 1549.4
        variables = build_variables({"front": hp, "rear": hp}, wb)
        self.assertEqual(len(variables), 2 * 11 * 3)
        by_name = {v["name"]: v for v in variables}
        fx = float(by_name["front_wheel_center_x"]["expression"].split()[0])
        rx = float(by_name["rear_wheel_center_x"]["expression"].split()[0])
        self.assertAlmostEqual(fx - rx, wb, places=3)
        self.assertTrue(all(v["type"] == "LENGTH" for v in variables))
        self.assertTrue(all(v["expression"].endswith(" mm")
                            for v in variables))

    def test_push_updates_existing_studio(self):
        hp, _ = generate_seed(mk4_setup())
        calls = []

        def fake_request(method, path, body=None):
            calls.append((method, path, body))
            if method == "GET":
                return [{"id": "e123", "name": "SuspensionHardpoints",
                         "elementType": "VARIABLESTUDIO"},
                        {"id": "e456", "name": "Part Studio 1",
                         "elementType": "PARTSTUDIO"}]
            return {}

        client = OnshapeClient("ak", "sk", request_fn=fake_request)
        summary = push_hardpoints(
            client, "https://cad.onshape.com/documents/aaaaaaaaaaaaaaaaaaaaaaaa"
            "/w/bbbbbbbbbbbbbbbbbbbbbbbb/e/cccccccccccccccccccccccc",
            {"front": hp, "rear": None}, 1549.4)
        self.assertIn("updated", summary)
        methods = [c[0] for c in calls]
        self.assertEqual(methods, ["GET", "POST"])   # no create call
        self.assertIn("/e/e123/variables", calls[1][1])
        self.assertEqual(len(calls[1][2]), 11 * 3)

    def test_push_creates_missing_studio(self):
        hp, _ = generate_seed(mk4_setup())
        calls = []

        def fake_request(method, path, body=None):
            calls.append((method, path, body))
            if method == "GET":
                return []
            if path.endswith("/variablestudio"):
                return {"id": "new99"}
            return {}

        client = OnshapeClient("ak", "sk", request_fn=fake_request)
        summary = push_hardpoints(
            client, "https://cad.onshape.com/documents/aaaaaaaaaaaaaaaaaaaaaaaa"
            "/w/bbbbbbbbbbbbbbbbbbbbbbbb", {"front": hp}, 1549.4)
        self.assertIn("created", summary)
        self.assertIn("/e/new99/variables", calls[-1][1])


if __name__ == "__main__":
    unittest.main()
