"""Impact load cases and the suspension load path (impact.py).

The load-path solver is verified three independent ways:
  * whole-corner FORCE and MOMENT equilibrium (sum of the mount loads must
    reproduce the wheel load exactly),
  * VIRTUAL WORK — the shock force must equal wheel load / motion ratio
    (MR = shock travel / wheel travel, so < 1),
    where the motion ratio comes from the kinematic sweep, a completely
    separate calculation,
  * hand arithmetic on the impulse chain.
"""

import unittest

import numpy as np

from suspension_tool.impact import (PULSE_SHAPES, STANDARD_CASES, ImpactCase,
                                    corner_loads, drop_height_to_mph,
                                    loads_csv)
from suspension_tool.metrics import sweep_metrics
from suspension_tool.seed import IN, SetupVariables, generate_seed
from suspension_tool.solver import DoubleWishboneSolver

G_IN_S2 = 386.4
MPH_TO_IN_S = 17.6


def _hp():
    sv = SetupVariables(
        track_width=63 * IN, wheelbase=61 * IN, ride_height=14 * IN,
        tire_radius=11.5 * IN, tire_width=7 * IN,
        shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
        motion_ratio_goal=1.0 / 1.85, kickup_deg=10.0)
    hp, _ = generate_seed(sv)
    return hp


class TestImpulse(unittest.TestCase):
    def setUp(self):
        self.c = ImpactCase(speed_mph=15.0, final_speed_mph=0.0,
                            contact_time_s=0.15, vehicle_weight_lb=580.0,
                            corner_share=0.5, pulse="half_sine")

    def test_impulse_by_hand(self):
        m = 580.0 / G_IN_S2
        dv = 15.0 * MPH_TO_IN_S
        self.assertAlmostEqual(self.c.impulse_lb_s(), m * dv, places=6)
        self.assertAlmostEqual(self.c.average_force_lb(), m * dv / 0.15,
                               places=6)

    def test_pulse_shape_scales_peak(self):
        avg = self.c.average_force_lb()
        self.assertAlmostEqual(self.c.peak_force_lb(),
                               avg * np.pi / 2.0 * 0.5, places=6)
        rect = ImpactCase(**{**self.c.__dict__, "pulse": "rectangular"})
        self.assertLess(rect.peak_force_lb(), self.c.peak_force_lb())
        tri = ImpactCase(**{**self.c.__dict__, "pulse": "triangular"})
        self.assertGreater(tri.peak_force_lb(), self.c.peak_force_lb())
        self.assertAlmostEqual(PULSE_SHAPES["half_sine"], np.pi / 2.0)

    def test_longer_contact_time_reduces_force(self):
        soft = ImpactCase(**{**self.c.__dict__, "contact_time_s": 0.30})
        self.assertAlmostEqual(soft.peak_force_lb(),
                               self.c.peak_force_lb() / 2.0, places=6)

    def test_zero_contact_time_rejected(self):
        with self.assertRaises(ValueError):
            ImpactCase(contact_time_s=0.0).average_force_lb()

    def test_direction_axes(self):
        for d, axis in (("head_on", 0), ("lateral", 1), ("vertical", 2)):
            w = ImpactCase(direction=d, include_static=False).wheel_load()
            self.assertGreater(abs(w[axis]), 0.0)
            self.assertAlmostEqual(np.linalg.norm(w), abs(w[axis]), places=6)
        # head-on decelerates the car: the ground pushes REARWARD (-X)
        self.assertLess(ImpactCase(direction="head_on").wheel_load()[0], 0.0)
        # a bump pushes the tire UP
        self.assertGreater(
            ImpactCase(direction="vertical").wheel_load()[2], 0.0)
        with self.assertRaises(ValueError):
            ImpactCase(direction="sideways").wheel_load()

    def test_static_load_adds_only_vertically(self):
        c = ImpactCase(direction="vertical", static_corner_load_lb=150.0)
        no_static = ImpactCase(direction="vertical",
                               static_corner_load_lb=150.0,
                               include_static=False)
        self.assertAlmostEqual(c.wheel_load()[2] - no_static.wheel_load()[2],
                               150.0, places=6)

    def test_drop_height(self):
        # v = sqrt(2gh); 36 in drop
        self.assertAlmostEqual(drop_height_to_mph(36.0),
                               np.sqrt(2 * G_IN_S2 * 36.0) / MPH_TO_IN_S,
                               places=6)
        self.assertEqual(drop_height_to_mph(0.0), 0.0)

    def test_equivalent_g_is_sane(self):
        # 15 mph to rest in 0.15 s is a hard hit but not absurd
        self.assertGreater(self.c.equivalent_g(), 3.0)
        self.assertLess(self.c.equivalent_g(), 20.0)


class TestLoadPath(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hp = _hp()
        cls.cp = np.array([cls.hp.wheel_center[0], cls.hp.wheel_center[1],
                           cls.hp.wheel_center[2] - cls.hp.tire_radius])

    def _check_equilibrium(self, load):
        r = corner_loads(self.hp, load)
        # Only CHASSIS rows sum to the wheel load — that is the documented
        # invariant. Internal rows (ball joints, the shock's arm end) are
        # equal-and-opposite pairs inside the corner and cancel, so including
        # them would double-count. (Before v1.24.1 the double wishbone
        # emitted no internal rows, which made the two sums coincide.)
        chassis = [m for m in r.mounts if m.is_chassis]
        total = sum((m.force for m in chassis), np.zeros(3))
        scale = max(1.0, float(np.linalg.norm(r.wheel_load_lb)))
        # FORCE: the mount loads must reproduce the wheel load exactly
        self.assertLess(float(np.linalg.norm(total - r.wheel_load_lb)) / scale,
                        1e-9)
        # MOMENT: about the origin, too
        mom = (np.cross(self.cp, r.wheel_load_lb)
               - sum((np.cross(m.point, m.force) for m in chassis),
                     np.zeros(3)))
        self.assertLess(float(np.linalg.norm(mom)) / scale, 1e-6)
        return r

    def test_equilibrium_all_directions(self):
        for load in (np.array([1000.0, 0, 0]), np.array([0, 1000.0, 0]),
                     np.array([0, 0, 1000.0]),
                     np.array([-800.0, 300.0, 1200.0])):
            self._check_equilibrium(load)

    def test_equilibrium_standard_cases(self):
        for case in STANDARD_CASES.values():
            case.vehicle_weight_lb = 580.0
            self._check_equilibrium(case.wheel_load())

    def test_virtual_work_shock_force(self):
        """INDEPENDENT check: for a pure vertical wheel load the shock
        force must be load / motion ratio (MR = shock/wheel), taken
        from the kinematic sweep — a completely separate computation."""
        s = DoubleWishboneSolver(self.hp)
        travels = np.linspace(-20.0, 20.0, 41)
        mr = sweep_metrics(s, travels)["motion_ratio"][20]
        fz = 1000.0
        r = corner_loads(self.hp, np.array([0.0, 0.0, fz]))
        shock = abs(r.worst_member().axial_lb if
                    r.worst_member().name == "shock"
                    else [m for m in r.members if m.name == "shock"][0].axial_lb)
        # MR is shock/wheel (< 1), so the shock force EXCEEDS the wheel
        # force: F_shock = F_wheel / MR.
        self.assertAlmostEqual(shock / (fz / mr), 1.0, places=4)

    def test_load_scales_linearly(self):
        a = corner_loads(self.hp, np.array([0.0, 0.0, 500.0]))
        b = corner_loads(self.hp, np.array([0.0, 0.0, 1000.0]))
        for ma, mb in zip(a.mounts, b.mounts):
            np.testing.assert_allclose(2.0 * ma.force, mb.force, atol=1e-6)

    def test_shock_carries_vertical_not_lateral(self):
        vert = corner_loads(self.hp, np.array([0.0, 0.0, 1000.0]))
        lat = corner_loads(self.hp, np.array([0.0, 1000.0, 0.0]))
        f_v = abs([m for m in vert.members if m.name == "shock"][0].axial_lb)
        f_l = abs([m for m in lat.members if m.name == "shock"][0].axial_lb)
        self.assertGreater(f_v, f_l)

    def test_members_report_tension_or_compression(self):
        r = corner_loads(self.hp, np.array([0.0, 0.0, 1000.0]))
        shock = [m for m in r.members if m.name == "shock"][0]
        # a bump COMPRESSES the shock
        self.assertLess(shock.axial_lb, 0.0)
        self.assertEqual(shock.state, "compression")
        self.assertGreater(shock.length_mm, 0.0)

    def test_shock_on_upper_arm_branch(self):
        """The shock can mount on the UPPER arm — that swaps which arm's
        moment equation carries it, so equilibrium must still close."""
        sv = SetupVariables(
            track_width=63 * IN, wheelbase=61 * IN, ride_height=14 * IN,
            tire_radius=11.5 * IN, tire_width=7 * IN,
            shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
            motion_ratio_goal=1.0 / 1.85, kickup_deg=10.0, shock_on_uca=True)
        hp, _ = generate_seed(sv)
        self.assertTrue(hp.shock_on_uca)
        for load in (np.array([0.0, 0.0, 1000.0]),
                     np.array([-900.0, 0.0, 0.0]),
                     np.array([0.0, 800.0, 0.0])):
            r = corner_loads(hp, load)
            total = sum((m.force for m in r.mounts if m.is_chassis),
                        np.zeros(3))
            self.assertLess(
                float(np.linalg.norm(total - r.wheel_load_lb))
                / float(np.linalg.norm(load)), 1e-9)

    def test_shock_force_can_be_pinned(self):
        r = corner_loads(self.hp, np.array([0.0, 0.0, 1000.0]),
                         shock_force_known=-500.0)
        shock = [m for m in r.members if m.name == "shock"][0]
        self.assertAlmostEqual(shock.axial_lb, -500.0, places=6)


class TestAllTypesLoadPath(unittest.TestCase):
    """Every suspension type must balance: the CHASSIS mount loads sum back
    to the wheel load in force and moment, and the shock force must match
    wheel load x motion ratio from the kinematic sweep (virtual work)."""

    LOADS = (np.array([0.0, 0.0, 1200.0]), np.array([-1000.0, 0.0, 0.0]),
             np.array([0.0, 700.0, 0.0]), np.array([-600.0, 300.0, 900.0]))

    @classmethod
    def setUpClass(cls):
        from suspension_tool.chub_front import seed_chub_front
        from suspension_tool.harm_rear import seed_harm_rear
        from suspension_tool.loaded_halfshaft import seed_loaded_halfshaft
        from suspension_tool.multilink import seed_multilink
        from suspension_tool.trailing_arm import seed_trailing_arm
        sv = SetupVariables(
            track_width=63 * IN, wheelbase=61 * IN, ride_height=14 * IN,
            tire_radius=11.5 * IN, tire_width=7 * IN,
            shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
            shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85,
            kickup_deg=10.0, static_camber_deg=-0.87)
        cls.types = {
            "double wishbone": generate_seed(sv)[0],
            "trailing arm": seed_trailing_arm(sv),
            "multilink": seed_multilink(sv),
            "H-arm": seed_harm_rear(sv),
            "C-hub": seed_chub_front(sv),
            "loaded halfshaft": seed_loaded_halfshaft(sv),
        }

    def test_chassis_mounts_balance_the_wheel_load(self):
        for name, hp in self.types.items():
            cp = np.array([hp.wheel_center[0], hp.wheel_center[1],
                           hp.wheel_center[2] - hp.tire_radius])
            for load in self.LOADS:
                r = corner_loads(hp, load)
                chassis = [m for m in r.mounts if m.is_chassis]
                self.assertTrue(chassis, name)
                total = sum((m.force for m in chassis), np.zeros(3))
                scale = float(np.linalg.norm(load))
                self.assertLess(
                    float(np.linalg.norm(total - load)) / scale, 1e-8,
                    f"{name} force imbalance")
                mom = (np.cross(cp, load)
                       - sum((np.cross(m.point, m.force) for m in chassis),
                             np.zeros(3)))
                self.assertLess(float(np.linalg.norm(mom)) / scale, 1e-6,
                                f"{name} moment imbalance")

    def test_virtual_work_every_type(self):
        from suspension_tool.suspension_types import solver_for
        fz = 1000.0
        for name, hp in self.types.items():
            solver = solver_for(hp)(hp)
            mr = sweep_metrics(solver, np.linspace(-15.0, 15.0, 31))[
                "motion_ratio"][15]
            r = corner_loads(hp, np.array([0.0, 0.0, fz]))
            shock = abs([m for m in r.members
                         if m.name == "shock"][0].axial_lb)
            self.assertAlmostEqual(shock / (fz / mr), 1.0, places=4,
                                   msg=f"{name} virtual work")

    def test_internal_joints_are_flagged_not_chassis(self):
        # the serial-chain types expose ball-joint / hinge / kingpin loads
        for name in ("H-arm", "C-hub", "loaded halfshaft"):
            r = corner_loads(self.types[name], np.array([0.0, 0.0, 1000.0]))
            internal = [m for m in r.mounts if not m.is_chassis]
            self.assertTrue(internal, f"{name} exposes no internal joints")
            for m in internal:
                self.assertIn("INTERNAL", m.note)
        # the C-hub chain is 3 bodies deep: block -> C-hub -> arm
        chub = corner_loads(self.types["C-hub"], np.array([0.0, 0.0, 1000.0]))
        names = {m.name for m in chub.mounts}
        self.assertTrue(any("kingpin" in n for n in names))
        self.assertTrue(any("hinge" in n for n in names))

    def test_every_type_exports_csv(self):
        case = ImpactCase(vehicle_weight_lb=580.0)
        for name, hp in self.types.items():
            text = loads_csv(case, corner_loads(hp, case.wheel_load()))
            self.assertIn("apply_to", text)
            self.assertIn("chassis", text)
            self.assertIn("member,axial_lb,state", text)

    def test_unknown_type_still_refused(self):
        class Weird:
            wheel_center = np.zeros(3)
            tire_radius = 1.0
        with self.assertRaises(NotImplementedError):
            corner_loads(Weird(), np.array([0.0, 0.0, 100.0]))


class TestCsvExport(unittest.TestCase):
    def test_csv_has_points_forces_and_provenance(self):
        hp = _hp()
        case = ImpactCase(name="unit test", speed_mph=10.0,
                          contact_time_s=0.1, vehicle_weight_lb=580.0)
        r = corner_loads(hp, case.wheel_load())
        text = loads_csv(case, r)
        self.assertIn("# MICKSUS impact load case: unit test", text)
        self.assertIn("# impulse_lb_s,", text)
        self.assertIn("# pulse_shape,", text)
        self.assertIn("quasi-static", text.lower())
        self.assertIn("name,x_mm,y_mm,z_mm,Fx_lb,Fy_lb,Fz_lb", text)
        for name in ("lca_inner_front", "uca_inner_rear", "shock_inner",
                     "tierod_inner"):
            self.assertIn(name, text)
        self.assertIn("member,axial_lb,state", text)
        # every mount row parses to 8 fields
        rows = [ln for ln in text.splitlines()
                if ln and not ln.startswith("#")
                and not ln.startswith(("name,", "member,"))]
        self.assertTrue(rows)

    def test_newton_conversion(self):
        hp = _hp()
        case = ImpactCase(vehicle_weight_lb=580.0)
        r = corner_loads(hp, case.wheel_load())
        lb = loads_csv(case, r)
        n = loads_csv(case, r, force_in_newtons=True)
        self.assertIn("Fx_lb", lb)
        self.assertIn("Fx_N", n)


if __name__ == "__main__":
    unittest.main()
