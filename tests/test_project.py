"""Phase 3 cleanup: units conversion and .MICK project save/load.

Neither needs a GUI, so both are checked directly here.
"""

import os
import tempfile
import unittest

import numpy as np

from suspension_tool.project import (
    AxleDesign, ProjectState, dict_to_project, load_project,
    project_to_dict, save_project)
from suspension_tool.seed import IN, SetupVariables, generate_seed
from suspension_tool.units import UNITS


def mk4_setup() -> SetupVariables:
    return SetupVariables(
        track_width=63.0 * IN, wheelbase=61.0 * IN, ride_height=14.0 * IN,
        tire_radius=11.5 * IN, tire_width=7.0 * IN,
        shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
        shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85,
        static_camber_deg=-0.87, kickup_deg=10.0)


class TestLockedPersistence(unittest.TestCase):
    def setUp(self):
        self.hp, _ = generate_seed(mk4_setup())

    def _round_trip(self, locked):
        ad = AxleDesign(hardpoints=self.hp, setup=None, droop_travel=75.0,
                        bump_travel=75.0, locked=locked)
        ps = ProjectState(front=ad, rear=None, unit_key="mm")
        return dict_to_project(project_to_dict(ps)).front

    def test_locked_round_trips(self):
        back = self._round_trip(["uca_outer", "lca_inner_front"])
        self.assertEqual(set(back.locked), {"uca_outer", "lca_inner_front"})

    def test_empty_locks_serialize_as_none(self):
        self.assertIsNone(self._round_trip([]).locked)

    def test_old_file_without_locked_key_loads(self):
        ps = ProjectState(front=AxleDesign(self.hp, None, 75.0, 75.0),
                          rear=None, unit_key="mm")
        d = project_to_dict(ps)
        d["front"].pop("locked", None)          # simulate a pre-v2 file
        self.assertIsNone(dict_to_project(d).front.locked)


class TestUnits(unittest.TestCase):
    def test_inch_round_trip(self):
        inch = UNITS["in"]
        self.assertAlmostEqual(inch.to_mm(1.0), 25.4, places=9)
        self.assertAlmostEqual(inch.from_mm(25.4), 1.0, places=9)
        self.assertAlmostEqual(inch.from_mm(inch.to_mm(7.3)), 7.3, places=9)

    def test_mm_is_identity(self):
        mm = UNITS["mm"]
        self.assertEqual(mm.to_mm(123.4), 123.4)
        self.assertEqual(mm.from_mm(123.4), 123.4)

    def test_format_precision(self):
        self.assertEqual(UNITS["in"].fmt(25.4), "1.000")
        self.assertEqual(UNITS["mm"].fmt(25.4), "25.4")


class TestProjectRoundTrip(unittest.TestCase):
    def setUp(self):
        self.setup = mk4_setup()
        self.hp, _ = generate_seed(self.setup)

    def _state(self):
        front = AxleDesign(hardpoints=self.hp, setup=self.setup,
                           droop_travel=108.0, bump_travel=268.0)
        rear = AxleDesign(hardpoints=self.hp, setup=None,
                          droop_travel=90.0, bump_travel=200.0)
        return ProjectState(front=front, rear=rear, unit_key="in",
                            goals=None,
                            vehicle={"wheelbase": 1549.4,
                                     "cg_height": 558.8,
                                     "cg_behind_front": 852.2})

    def test_dict_round_trip_preserves_both_axles(self):
        restored = dict_to_project(project_to_dict(self._state()))
        for axle in (restored.front, restored.rear):
            for attr in ("uca_inner_front", "lca_outer", "tierod_outer",
                         "wheel_center", "shock_inner", "shock_outer"):
                np.testing.assert_allclose(
                    getattr(axle.hardpoints, attr), getattr(self.hp, attr),
                    rtol=0, atol=1e-9)
        self.assertEqual(restored.unit_key, "in")
        self.assertAlmostEqual(restored.front.bump_travel, 268.0)
        self.assertAlmostEqual(restored.rear.bump_travel, 200.0)
        self.assertAlmostEqual(restored.vehicle["wheelbase"], 1549.4)

    def test_setup_survives_round_trip(self):
        restored = dict_to_project(project_to_dict(self._state()))
        self.assertIsNotNone(restored.front.setup)
        self.assertAlmostEqual(restored.front.setup.kickup_deg, 10.0)
        self.assertIsNone(restored.rear.setup)

    def test_file_save_and_load(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "design.MICK")
            save_project(path, self._state())
            self.assertTrue(os.path.exists(path))
            restored = load_project(path)
        np.testing.assert_allclose(
            restored.front.hardpoints.uca_outer, self.hp.uca_outer, atol=1e-9)
        self.assertEqual(restored.unit_key, "in")

    def test_unknown_setup_field_is_ignored(self):
        data = project_to_dict(self._state())
        data["front"]["setup"]["some_future_field"] = 999  # forward-compat
        restored = dict_to_project(data)
        self.assertFalse(hasattr(restored.front.setup, "some_future_field"))

    def test_dynamics_inputs_round_trip(self):
        from suspension_tool.dynamics import DynamicsInputs
        state = self._state()
        state.dynamics = {**DynamicsInputs(ay_g=1.4).to_dict(),
                          "_linked": False}
        restored = dict_to_project(project_to_dict(state))
        self.assertEqual(restored.dynamics["ay_g"], 1.4)
        self.assertIs(restored.dynamics["_linked"], False)
        v = DynamicsInputs.from_dict(restored.dynamics)  # tolerant of extras
        self.assertEqual(v.ay_g, 1.4)
        # old files without the key still load
        d = project_to_dict(self._state())
        del d["dynamics"]
        self.assertIsNone(dict_to_project(d).dynamics)

    def test_bad_version_rejected(self):
        data = project_to_dict(self._state())
        data["version"] = 999
        with self.assertRaises(ValueError):
            dict_to_project(data)

    def test_v1_file_loads_as_front_axle(self):
        # Files saved by builds <= 0.6.0 held one corner at the top level.
        from suspension_tool.project import _hp_to_dict
        import dataclasses as dc
        v1 = {
            "format": "baja-suspension-kinematics",
            "version": 1,
            "unit": "in",
            "travel_mm": {"droop": 108.0, "bump": 268.0},
            "hardpoints_mm": _hp_to_dict(self.hp),
            "setup": dc.asdict(self.setup),
            "goals": None,
        }
        restored = dict_to_project(v1)
        self.assertIsNone(restored.rear)
        np.testing.assert_allclose(
            restored.front.hardpoints.lca_outer, self.hp.lca_outer, atol=1e-9)
        self.assertAlmostEqual(restored.front.droop_travel, 108.0)

    def test_rear_only_project_loads(self):
        state = self._state()
        state.front = None
        restored = dict_to_project(project_to_dict(state))
        self.assertIsNone(restored.front)
        self.assertIsNotNone(restored.rear)


class TestCsvExport(unittest.TestCase):
    def setUp(self):
        self.hp, _ = generate_seed(mk4_setup())

    def test_csv_round_trip_mm(self):
        from suspension_tool.project import hardpoints_csv
        point_attrs = type(self.hp).POINT_ATTRS
        text = hardpoints_csv(self.hp)
        rows = {}
        for line in text.splitlines():
            if line.startswith("#") or line.startswith("name,") or not line:
                continue
            name, x, y, z = line.split(",")
            rows[name] = np.array([float(x), float(y), float(z)])
        self.assertEqual(set(rows), set(point_attrs))
        for attr in point_attrs:
            np.testing.assert_allclose(rows[attr], getattr(self.hp, attr),
                                       atol=1e-3)

    def test_csv_respects_units(self):
        from suspension_tool.project import hardpoints_csv
        text = hardpoints_csv(self.hp, unit_mm=25.4, unit_label="in")
        line = [l for l in text.splitlines() if l.startswith("wheel_center")][0]
        _, x, y, z = line.split(",")
        self.assertAlmostEqual(float(y), self.hp.wheel_center[1] / 25.4, places=3)
        self.assertIn("(in)", text.splitlines()[0])

    def test_csv_file_export(self):
        from suspension_tool.project import export_hardpoints_csv
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "hp.csv")
            export_hardpoints_csv(path, self.hp)
            with open(path) as f:
                content = f.read()
        self.assertIn("lca_outer,", content)

    def test_shock_on_uca_round_trips_in_project(self):
        import dataclasses
        hp = dataclasses.replace(self.hp, shock_on_uca=True)
        state = ProjectState(
            front=AxleDesign(hardpoints=hp, setup=None,
                             droop_travel=50.0, bump_travel=50.0),
            rear=None, unit_key="mm")
        restored = dict_to_project(project_to_dict(state))
        self.assertTrue(restored.front.hardpoints.shock_on_uca)
        # and old files without the field default to False
        data = project_to_dict(state)
        del data["front"]["hardpoints_mm"]["shock_on_uca"]
        self.assertFalse(
            dict_to_project(data).front.hardpoints.shock_on_uca)

    def test_vehicle_csv_places_rear_behind_front(self):
        from suspension_tool.project import vehicle_csv
        wb = 1549.4
        text = vehicle_csv({"front": self.hp, "rear": self.hp}, wb)
        rows = {}
        for line in text.splitlines():
            if line.startswith("#") or line.startswith("name,") or not line:
                continue
            name, x, y, z = line.split(",")
            rows[name] = (float(x), float(y), float(z))
        self.assertAlmostEqual(
            rows["front_wheel_center"][0] - rows["rear_wheel_center"][0],
            wb, places=3)
        self.assertAlmostEqual(rows["front_wheel_center"][1],
                               rows["rear_wheel_center"][1], places=6)


if __name__ == "__main__":
    unittest.main()
