"""Onshape design sync (hybrid path): the whole .MICK project JSON is
stored in the document as an app element, create-or-replace by name, and
pulled back as a loadable project. Mock-tested like the studio push."""

import unittest

from suspension_tool.onshape_sync import (DEFAULT_DESIGN_NAME,
                                          OnshapeClient, pull_design,
                                          push_design)
from suspension_tool.project import (dict_to_project, project_to_dict,
                                     AxleDesign, ProjectState)
from suspension_tool.seed import IN, SetupVariables, generate_seed

URL = ("https://cad.onshape.com/documents/aaaaaaaaaaaaaaaaaaaaaaaa"
       "/w/bbbbbbbbbbbbbbbbbbbbbbbb/e/cccccccccccccccccccccccc")


def _project_dict():
    sv = SetupVariables(
        track_width=63 * IN, wheelbase=61 * IN, ride_height=14 * IN,
        tire_radius=11.5 * IN, tire_width=7 * IN,
        shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
        shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85)
    hp, _ = generate_seed(sv)
    state = ProjectState(
        front=AxleDesign(hardpoints=hp, setup=sv, droop_travel=107.0,
                         bump_travel=268.0),
        rear=None, unit_key="in", coords="chassis+y",
        candidates=[{"name": "baseline", "metrics": {}}])
    return project_to_dict(state)


class TestDesignSync(unittest.TestCase):
    def test_push_creates_when_absent(self):
        calls = []

        def fake(method, path, body=None):
            calls.append((method, path, body))
            if method == "GET":
                return [{"id": "v1", "name": "SuspensionHardpoints",
                         "elementType": "VARIABLESTUDIO"}]
            if "/appelements/" in path and method == "POST":
                return {"id": "app77"}
            return {}

        client = OnshapeClient("ak", "sk", request_fn=fake)
        summary = push_design(client, URL, _project_dict())
        self.assertIn("created", summary)
        create = next(c for c in calls if c[0] == "POST")
        self.assertIn("/appelements/", create[1])
        self.assertEqual(create[2]["name"], DEFAULT_DESIGN_NAME)
        self.assertEqual(create[2]["jsonTree"]["format"],
                         "baja-suspension-kinematics")

    def test_push_replaces_existing(self):
        calls = []

        def fake(method, path, body=None):
            calls.append((method, path, body))
            if method == "GET":
                return [{"id": "old1", "name": DEFAULT_DESIGN_NAME}]
            if method == "POST":
                return {"id": "app78"}
            return {}

        client = OnshapeClient("ak", "sk", request_fn=fake)
        summary = push_design(client, URL, _project_dict())
        self.assertIn("replaced", summary)
        methods = [c[0] for c in calls]
        self.assertEqual(methods, ["GET", "DELETE", "POST"])
        self.assertIn("/e/old1", calls[1][1])

    def test_pull_round_trips_a_loadable_project(self):
        stored = _project_dict()

        def fake(method, path, body=None):
            if method == "GET" and path.endswith("/elements"):
                return [{"id": "app77", "name": DEFAULT_DESIGN_NAME}]
            if method == "GET" and "/content/json" in path:
                return {"tree": stored}
            return {}

        client = OnshapeClient("ak", "sk", request_fn=fake)
        tree = pull_design(client, URL)
        state = dict_to_project(tree)
        self.assertIsNotNone(state.front)
        self.assertEqual(state.coords, "chassis+y")
        self.assertEqual(state.candidates[0]["name"], "baseline")
        import numpy as np
        np.testing.assert_allclose(
            state.front.hardpoints.wheel_center,
            dict_to_project(stored).front.hardpoints.wheel_center)

    def test_pull_missing_element_is_actionable(self):
        def fake(method, path, body=None):
            return [] if method == "GET" else {}

        client = OnshapeClient("ak", "sk", request_fn=fake)
        with self.assertRaises(RuntimeError) as ctx:
            pull_design(client, URL)
        self.assertIn("Sync", str(ctx.exception))


class _MockDoc:
    """A stateful fake Onshape workspace: tabs live in a dict, POSTs to a
    create endpoint add a tab, POSTs to an /e/{id} endpoint update in
    place. Lets us prove that a re-push creates NOTHING new."""

    def __init__(self):
        self.elements = []      # list of {id, name, elementType}
        self.n = 0
        self.variable_sets = 0
        self.feature_updates = 0
        self.feature_adds = 0
        self.blob_uploads = 0
        self.ps_features = []   # featureIds present in the part studio

    def _new(self, name, etype):
        self.n += 1
        eid = f"{etype[:2].lower()}{self.n}"
        self.elements.append({"id": eid, "name": name,
                              "elementType": etype})
        return eid

    def __call__(self, method, path, body=None):
        has_e = "/e/" in path
        if method == "GET" and path.endswith("/elements"):
            return list(self.elements)
        if method == "GET" and path.endswith("/featurespecs"):
            return {"featureSpecs": [
                {"featureType": "suspensionHardpoints",
                 "namespace": "d1::w1::e1::m1"}]}
        if method == "POST" and path.endswith("/variablestudio"):
            return {"id": self._new(body["name"], "VARIABLESTUDIO")}
        if method == "POST" and "/variables/" in path and has_e:
            self.variable_sets += 1
            return {}
        if method == "POST" and "/featurestudios/" in path:
            if has_e:                          # update code in place
                return {"microversionId": "m123"}
            return {"id": self._new("MICKSUS Kinematics", "FEATURESTUDIO")}
        if method == "GET" and path.endswith("/features"):
            return {"features": [{"featureType": "suspensionHardpoints",
                                  "featureId": f} for f in self.ps_features]}
        if method == "POST" and "/features/featureid/" in path:
            self.feature_updates += 1
            return {"feature": {"featureId": path.rsplit("/", 1)[-1]}}
        if method == "POST" and path.endswith("/features"):
            self.feature_adds += 1
            fid = f"F{len(self.ps_features)}"
            self.ps_features.append(fid)
            return {"feature": {"featureId": fid}}
        if method == "POST" and "/partstudios/" in path and not has_e:
            return {"id": self._new("Suspension Hardpoints", "PARTSTUDIO")}
        if method == "POST" and "/blobelements/" in path:
            self.blob_uploads += 1
            if has_e:                          # update in place
                return {}
            return {"id": self._new("Suspension Design Notes.pdf", "BLOB")}
        return {}


class TestGeometryPushIdempotent(unittest.TestCase):
    """The behaviour Ethan asked for: the FIRST push builds the tabs; a
    SECOND push finds them and updates in place — zero new tabs."""

    def _axles(self):
        sv = SetupVariables(
            track_width=63 * IN, wheelbase=61 * IN, ride_height=14 * IN,
            tire_radius=11.5 * IN, tire_width=7 * IN,
            shock_min_length=14.68 * IN, shock_max_length=22.68 * IN,
            shock_length_at_ride=20.38 * IN, motion_ratio_goal=1.0 / 1.85)
        hp, _ = generate_seed(sv)
        return {"front": hp, "rear": None}

    def test_create_then_update_makes_no_duplicates(self):
        from suspension_tool.onshape_sync import push_geometry
        from suspension_tool.report_pdf import design_notes_pdf
        doc = _MockDoc()
        client = OnshapeClient("ak", "sk", request_fn=doc)
        axles = self._axles()
        pdf = design_notes_pdf({"front": {"hp": axles["front"]}}, 1549.4)

        first = push_geometry(client, URL, axles, 1549.4, pdf_bytes=pdf)
        n_after_first = len(doc.elements)
        # 4 tabs: variable studio, feature studio, part studio, pdf
        self.assertEqual(n_after_first, 4, doc.elements)
        self.assertEqual(first["manifest"]["feature_id"], "F0")
        self.assertEqual(doc.feature_adds, 1)

        # Re-push with the returned manifest: NOTHING new is created and
        # the feature is UPDATED in place (references preserved).
        second = push_geometry(client, URL, axles, 1549.4, pdf_bytes=pdf,
                               manifest=first["manifest"])
        self.assertEqual(len(doc.elements), n_after_first,
                         "re-push created a duplicate tab")
        self.assertEqual(doc.feature_adds, 1, "feature was re-added!")
        self.assertEqual(doc.feature_updates, 1)
        self.assertTrue(any("in place" in s for s in second["summary"]))
        self.assertEqual(doc.variable_sets, 2)
        self.assertEqual(doc.blob_uploads, 2)

    def test_update_survives_lost_manifest_via_names(self):
        # If the manifest is lost (e.g. a teammate's fresh machine), the
        # push must still find the existing tabs by name AND the existing
        # feature by type — never duplicating either.
        from suspension_tool.onshape_sync import push_geometry
        doc = _MockDoc()
        client = OnshapeClient("ak", "sk", request_fn=doc)
        axles = self._axles()
        push_geometry(client, URL, axles, 1549.4)
        n = len(doc.elements)
        self.assertEqual(doc.feature_adds, 1)
        push_geometry(client, URL, axles, 1549.4, manifest=None)
        self.assertEqual(len(doc.elements), n,
                         "name-based match failed; duplicated tabs")
        # the KEY fix: without a manifest, the existing feature is still
        # found (by type) and updated, not pasted anew
        self.assertEqual(doc.feature_adds, 1,
                         "manifest-less re-push duplicated the feature")
        self.assertEqual(doc.feature_updates, 1)

    def test_feature_source_is_self_contained_with_skeleton(self):
        # v1.11.3: the FS embeds coordinates (works without inserting the
        # Variable Studio), reads a variable override first, and emits the
        # full named skeleton + halfshaft points.
        from suspension_tool.halfshaft import (HalfshaftConfig,
                                               default_inner_for)
        from suspension_tool.onshape_featurescript import \
            feature_studio_source
        from suspension_tool.onshape_sync import hardpoint_values
        hp = self._axles()["front"]
        cfg = HalfshaftConfig(enabled=True,
                              inner=default_inner_for(hp, 268.0, 107.0))
        vals = hardpoint_values({"front": hp, "rear": None}, 1549.4,
                                halfshafts={"front": cfg})
        self.assertIn("front_halfshaft_inner", vals)
        src = feature_studio_source(vals)
        for token in ("onshape/std/common.fs", "const EMBEDDED",
                      "try silent(getVariable", "opPoint", "opFitSpline",
                      "Front LowerAArmPlane", "Front UpperAArmPlane",
                      "Front Lower Bushing Axis", "Front Kingpin Axis",
                      "Front 2D Sketch Plane", "Front Halfshaft"):
            self.assertIn(token, src)
        self.assertNotIn("geometry.fs", src)


if __name__ == "__main__":
    unittest.main()
