"""Phase 1 verification: hand-calculated checks of the forward solver.

Run from the repo root:  python -m unittest

Each test states the hand calculation it checks against. Uses only the
standard-library unittest runner (no extra dependency).
"""

import unittest

import numpy as np

from suspension_tool.geometry import DoubleWishbonePoints, _pt, example_baja_front
from suspension_tool.metrics import (
    camber_deg,
    caster_deg,
    caster_trail_mm,
    corner_metrics,
    front_view_ic,
    kpi_deg,
    roll_center_height_mm,
    scrub_radius_mm,
    sweep_metrics,
    toe_deg,
)
from suspension_tool.solver import DoubleWishboneSolver


def parallelogram_corner(tierod_inner_z: float = 310.0) -> DoubleWishbonePoints:
    """Equal-length, parallel, horizontal arms with a vertical kingpin and
    a parallel equal-length tie rod: a perfect parallelogram. The upright
    TRANSLATES without rotating, so camber and toe must stay exactly zero,
    and every upright point follows the same circular arc as the ball
    joints (radius 380 mm about y = 220 mm). Fully hand-checkable.
    """
    return DoubleWishbonePoints(
        uca_inner_front=_pt(120.0, 220.0, 420.0),
        uca_inner_rear=_pt(-120.0, 220.0, 420.0),
        uca_outer=_pt(0.0, 600.0, 420.0),
        lca_inner_front=_pt(150.0, 220.0, 200.0),
        lca_inner_rear=_pt(-150.0, 220.0, 200.0),
        lca_outer=_pt(0.0, 600.0, 200.0),
        tierod_inner=_pt(-180.0, 220.0, tierod_inner_z),
        tierod_outer=_pt(-180.0, 600.0, 310.0),
        wheel_center=_pt(0.0, 650.0, 310.0),
        shock_inner=_pt(0.0, 250.0, 500.0),
        shock_outer=_pt(0.0, 480.0, 200.0),
        # Tire radius = wheel-centre height, so the tire sits ON the ground.
        tire_radius=310.0,
    )


class TestParallelogram(unittest.TestCase):
    """Hand calculation: with a parallelogram, camber/toe never change and
    the wheel centre's lateral position is 650 + (sqrt(380^2 - dz^2) - 380)
    — the ball joints' arc about the inner pivots, radius 380 mm."""

    def setUp(self):
        self.solver = DoubleWishboneSolver(parallelogram_corner())

    def test_camber_and_toe_stay_zero(self):
        for dz in np.linspace(-50.0, 50.0, 11):
            st = self.solver.solve(dz)
            self.assertAlmostEqual(camber_deg(st), 0.0, places=8)
            self.assertAlmostEqual(toe_deg(st), 0.0, places=8)

    def test_wheel_center_follows_hand_calculated_arc(self):
        for dz in (-50.0, -20.0, 10.0, 35.0, 50.0):
            st = self.solver.solve(dz)
            y_hand = 650.0 + (np.sqrt(380.0**2 - dz**2) - 380.0)
            self.assertAlmostEqual(st.wheel_center[2], 310.0 + dz, places=8)
            self.assertAlmostEqual(st.wheel_center[1], y_hand, places=6)
            self.assertAlmostEqual(st.wheel_center[0], 0.0, places=8)

    def test_roll_center_on_ground(self):
        # Horizontal parallel arms -> IC at infinity, horizontally -> the
        # contact-patch line runs flat -> roll centre at ground level.
        st = self.solver.solve(0.0)
        self.assertIsNone(front_view_ic(self.solver, st))
        self.assertAlmostEqual(roll_center_height_mm(self.solver, st), 0.0, places=8)


class TestStaticMetricsByHand(unittest.TestCase):
    """Static geometry with hand-computed kingpin metrics.

    Kingpin: LBJ (0, 600, 180) -> UBJ (-20, 570, 380), k = (-20, -30, 200).
      caster = atan(20/200) = 5.71059 deg (top rearward)
      KPI    = atan(30/200) = 8.53077 deg (top inboard)
    Ground pierce: t = -180/200 = -0.9 ->
      x = 0 + (-20)(-0.9) = 18,  y = 600 + (-30)(-0.9) = 627.
    Wheel centre (0, 640, 300), zero camber/toe, R = 300 ->
      contact patch (0, 640, 0).
      scrub = 640 - 627 = 13 mm,  trail = 18 - 0 = 18 mm.
    """

    def setUp(self):
        hp = DoubleWishbonePoints(
            uca_inner_front=_pt(150.0, 250.0, 390.0),
            uca_inner_rear=_pt(-150.0, 250.0, 390.0),
            uca_outer=_pt(-20.0, 570.0, 380.0),
            lca_inner_front=_pt(150.0, 250.0, 170.0),
            lca_inner_rear=_pt(-150.0, 250.0, 170.0),
            lca_outer=_pt(0.0, 600.0, 180.0),
            tierod_inner=_pt(-150.0, 280.0, 250.0),
            tierod_outer=_pt(-160.0, 590.0, 270.0),
            wheel_center=_pt(0.0, 640.0, 300.0),
            shock_inner=_pt(0.0, 300.0, 550.0),
            shock_outer=_pt(0.0, 480.0, 175.0),
            tire_radius=300.0,
        )
        self.solver = DoubleWishboneSolver(hp)
        self.static = self.solver.solve(0.0)

    def test_caster(self):
        self.assertAlmostEqual(caster_deg(self.static), np.degrees(np.arctan(20.0 / 200.0)), places=6)

    def test_kpi(self):
        self.assertAlmostEqual(kpi_deg(self.static), np.degrees(np.arctan(30.0 / 200.0)), places=6)

    def test_scrub_radius(self):
        self.assertAlmostEqual(scrub_radius_mm(self.static), 13.0, places=6)

    def test_caster_trail(self):
        self.assertAlmostEqual(caster_trail_mm(self.static), 18.0, places=6)

    def test_static_camber_toe_inputs_round_trip(self):
        hp = example_baja_front()
        hp.static_camber_deg = -1.5
        hp.static_toe_deg = 0.25
        st = DoubleWishboneSolver(hp).solve(0.0)
        # Camber is measured as a front-view projection, so with nonzero toe
        # it differs from the applied rotation by O(toe^2 * camber) ~ 1e-5 deg.
        self.assertAlmostEqual(camber_deg(st), -1.5, places=3)
        self.assertAlmostEqual(toe_deg(st), 0.25, places=6)


class TestRigidBodyInvariants(unittest.TestCase):
    """Every fixed length in the mechanism must stay fixed through travel:
    arm radii, kingpin spacing, tie-rod length, upright point spacings."""

    def test_lengths_constant_through_sweep(self):
        hp = example_baja_front()
        solver = DoubleWishboneSolver(hp)
        ref = {
            "kingpin": np.linalg.norm(hp.uca_outer - hp.lca_outer),
            "tierod": np.linalg.norm(hp.tierod_outer - hp.tierod_inner),
            "lbj_tro": np.linalg.norm(hp.tierod_outer - hp.lca_outer),
            "ubj_wc": np.linalg.norm(hp.wheel_center - hp.uca_outer),
            "lbj_wc": np.linalg.norm(hp.wheel_center - hp.lca_outer),
        }
        for dz in np.linspace(-60.0, 60.0, 13):
            st = solver.solve(dz)
            self.assertAlmostEqual(np.linalg.norm(st.ubj - st.lbj), ref["kingpin"], places=7)
            self.assertAlmostEqual(np.linalg.norm(st.tro - hp.tierod_inner), ref["tierod"], places=7)
            self.assertAlmostEqual(np.linalg.norm(st.tro - st.lbj), ref["lbj_tro"], places=7)
            self.assertAlmostEqual(np.linalg.norm(st.wheel_center - st.ubj), ref["ubj_wc"], places=7)
            self.assertAlmostEqual(np.linalg.norm(st.wheel_center - st.lbj), ref["lbj_wc"], places=7)
            self.assertAlmostEqual(st.wheel_center[2] - hp.wheel_center[2], dz, places=7)


class TestCamberRateMatchesInstantCentre(unittest.TestCase):
    """Cross-check the solver against the front-view instant-centre theory:
    in the front view the upright rotates about the IC, so the camber rate
    must equal 1 / (y_ic - y_wc) rad per mm of wheel travel. The solver and
    the IC construction are computed by independent code paths, so
    agreement validates both.

    Caveat: the relation is only exact when bump steer is ~zero (toe change
    rotates the wheel about the tilted kingpin, which leaks into front-view
    camber and is not part of the IC construction). The example geometry has
    its tie rod placed for near-zero static bump steer, on purpose."""

    def test_camber_rate(self):
        solver = DoubleWishboneSolver(example_baja_front())
        st0 = solver.solve(0.0)
        ic = front_view_ic(solver, st0)
        self.assertIsNotNone(ic)
        y_ic, _ = ic
        predicted = np.degrees(1.0 / (y_ic - st0.wheel_center[1]))  # deg/mm
        h = 0.5
        rate = (camber_deg(solver.solve(h)) - camber_deg(solver.solve(-h))) / (2 * h)
        self.assertAlmostEqual(rate, predicted, delta=abs(predicted) * 0.02)


class TestBumpSteerSanity(unittest.TestCase):
    """Misplacing the tie rod must CREATE bump steer; the well-placed
    parallelogram tie rod has none. Catches a frozen/ignored tie rod."""

    def test_raised_tierod_inner_causes_toe_change(self):
        good = DoubleWishboneSolver(parallelogram_corner(tierod_inner_z=310.0))
        bad = DoubleWishboneSolver(parallelogram_corner(tierod_inner_z=360.0))
        self.assertAlmostEqual(toe_deg(good.solve(40.0)), 0.0, places=8)
        self.assertGreater(abs(toe_deg(bad.solve(40.0))), 0.05)


class TestSweepOutputs(unittest.TestCase):
    """The sweep helper returns consistent, finite curves."""

    def test_sweep_shapes_and_motion_ratio(self):
        solver = DoubleWishboneSolver(example_baja_front())
        travels = np.linspace(-60.0, 60.0, 25)
        m = sweep_metrics(solver, travels)
        for key in ("camber_deg", "toe_deg", "caster_deg", "kpi_deg",
                    "roll_center_height_mm", "motion_ratio",
                    "bump_steer_deg_per_mm", "shock_length_mm"):
            self.assertEqual(len(m[key]), len(travels))
            self.assertTrue(np.all(np.isfinite(m[key])), key)
        # The shock sits inboard on the LCA, so wheel/shock ratio > 1.
        # MR is shock/wheel since v1.28, so it is BELOW 1 for an
        # inboard-mounted shock.
        self.assertTrue(np.all(m["motion_ratio"] < 1.0))
        self.assertTrue(np.all(m["motion_ratio"] > 0.0))
        self.assertTrue(np.all(m["motion_ratio"] < 4.0))


class TestSpindleExactReadback(unittest.TestCase):
    """REGRESSION: the spindle is built so BOTH front-view camber and
    top-view toe read back EXACTLY for any input — the old sequential
    Rz(-tau)Rx(-gamma) form coupled a gamma*tau^2/2 error into camber
    (0.037 deg at -10/5, exceeding the geometry tolerance)."""

    def test_camber_and_toe_read_back_exactly(self):
        import dataclasses

        from suspension_tool.metrics import camber_deg, toe_deg
        from suspension_tool.seed import IN, SetupVariables, generate_seed
        base, _ = generate_seed(SetupVariables(
            track_width=63 * IN, wheelbase=61 * IN, ride_height=14 * IN,
            tire_radius=11.5 * IN, tire_width=7 * IN,
            shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
            shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85))
        for g in (-10.0, -3.0, -0.87, 0.0, 4.0):
            for t in (-5.0, -1.0, 0.0, 2.0, 5.0):
                hp = dataclasses.replace(base, static_camber_deg=g,
                                         static_toe_deg=t)
                st = DoubleWishboneSolver(hp).solve(0.0)
                self.assertAlmostEqual(camber_deg(st), g, places=6,
                                       msg=f"camber ({g},{t})")
                self.assertAlmostEqual(toe_deg(st), t, places=6,
                                       msg=f"toe ({g},{t})")


class TestChangeChannelDatum(unittest.TestCase):
    """REGRESSION: half_track_change / wheel_recession are referenced to
    TRUE static (travel=0), not the nearest grid sample — on asymmetric
    -droop..+bump sweeps the old datum offset the whole curve by ~1 mm."""

    def test_channels_zero_at_true_static(self):
        from suspension_tool.metrics import sweep_metrics
        from suspension_tool.seed import IN, SetupVariables, generate_seed
        hp, _ = generate_seed(SetupVariables(
            track_width=63 * IN, wheelbase=61 * IN, ride_height=14 * IN,
            tire_radius=11.5 * IN, tire_width=7 * IN,
            shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
            shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85,
            static_camber_deg=-0.87))
        s = DoubleWishboneSolver(hp)
        # deliberately asymmetric range so travel=0 is NOT a grid node
        tr = np.linspace(-121.5, 327.2, 41)
        sw = sweep_metrics(s, tr)
        htc0 = float(np.interp(0.0, tr, sw["half_track_change_mm"]))
        rec0 = float(np.interp(0.0, tr, sw["wheel_recession_mm"]))
        # curve passes through ~0 at true static (interp curvature only)
        self.assertLess(abs(htc0), 0.05)
        self.assertLess(abs(rec0), 0.05)


if __name__ == "__main__":
    unittest.main()
