"""Push a MICKSUS suspension design into an Onshape document over the
REST API. Three levels, smallest to fullest:

  * push_hardpoints  — write just the Variable Studio (one length
    variable per hardpoint coordinate). Your own Onshape sketches can
    reference #front_lca_outer_x etc. and rebuild.
  * push_design      — store the whole .MICK project as a hidden app
    element so teammates pull the kinematic model back into MICKSUS.
  * push_geometry (v1.11) — the full build Ethan asked for: a Variable
    Studio (source of truth) + a Feature Studio holding ONE custom
    FeatureScript feature that reads the variable table and regenerates
    every hardpoint (as a point body) plus the front/rear 2D sketch
    planes, + a design-notes PDF. Crucially IDEMPOTENT: a stored
    manifest of tab/feature ids means a re-push updates the SAME tabs in
    place — no duplicates — so re-pushing after a redesign just moves
    the existing geometry. This also keeps the API call count per push
    tiny, which matters: Onshape bills an ANNUAL per-user quota.

Auth is HTTP Basic with free API keys (dev-portal.onshape.com/keys).
Networking is a thin, injectable `_request` so the orchestration is
unit-tested offline against a stateful mock (see tests/test_design_sync).
The geometry push's exact BTM/FeatureScript strings were built from the
API docs but not yet exercised live; first-run errors are surfaced
verbatim and the format strings are centralised (here + in
onshape_featurescript.py) so a live fix is a one-line edit.
"""

import base64
import json
import re
import urllib.error
import urllib.request


DEFAULT_BASE = "https://cad.onshape.com"
DEFAULT_STUDIO_NAME = "SuspensionHardpoints"
APP_FORMAT_ID = "com.bajasae.suspension-design"
DEFAULT_DESIGN_NAME = "Suspension Design (Baja Tool)"
DEFAULT_PARTSTUDIO_NAME = "Suspension Hardpoints"
DEFAULT_PDF_NAME = "Suspension Design Notes.pdf"
# Where the create-vs-update manifest (tab + feature ids) lives, so a
# re-push finds prior tabs and updates them in place instead of making
# new ones. Stored inside the design app-element JSON.
MANIFEST_KEY = "_onshape_manifest"


def _encode_multipart(filename: str, blob: bytes) -> tuple:
    """Minimal multipart/form-data body for a blob upload."""
    import uuid
    boundary = "----MICKSUS" + uuid.uuid4().hex
    pre = (f'--{boundary}\r\n'
           f'Content-Disposition: form-data; name="file"; '
           f'filename="{filename}"\r\n'
           f'Content-Type: application/octet-stream\r\n\r\n').encode()
    post = f'\r\n--{boundary}--\r\n'.encode()
    return pre + blob + post, f"multipart/form-data; boundary={boundary}"


def parse_document_url(url: str) -> tuple[str, str]:
    """Extract (document_id, workspace_id) from an Onshape document URL,
    e.g. https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>."""
    m = re.search(r"/documents/([0-9a-f]+)/w/([0-9a-f]+)", url)
    if not m:
        raise ValueError(
            "could not find /documents/<id>/w/<id> in the URL — copy it "
            "from the browser address bar while the document is open")
    return m.group(1), m.group(2)


def hardpoint_values(axles: dict, wheelbase: float, conv=None,
                     halfshafts: dict = None) -> dict:
    """Every hardpoint's position as {base_name: (x, y, z)} in MILLIMETRES
    in the document (chassis) frame: rear axle shifted rearward by the
    wheelbase, then remapped through `conv`. Includes the halfshaft inner
    and outer CV when that axle is driven. This is the single source both
    the Variable Studio payload and the FeatureScript embedding read, so
    they can never disagree."""
    import numpy as np
    if conv is None:
        from .axes import CONVENTIONS
        conv = CONVENTIONS["tool"]
    halfshafts = halfshafts or {}
    out = {}
    for axle_name, hp in axles.items():
        if hp is None:
            continue
        dx = 0.0 if axle_name == "front" else -wheelbase

        def place(p):
            return conv.to_display(np.asarray(p, float)
                                   + np.array([dx, 0.0, 0.0]))
        for attr in type(hp).POINT_ATTRS:
            out[f"{axle_name}_{attr}"] = tuple(
                float(v) for v in place(getattr(hp, attr)))
        cfg = halfshafts.get(axle_name)
        if cfg is not None and getattr(cfg, "enabled", False):
            from .halfshaft import outer_cv_of_hp
            out[f"{axle_name}_halfshaft_inner"] = tuple(
                float(v) for v in place(cfg.inner))
            out[f"{axle_name}_halfshaft_outer"] = tuple(
                float(v) for v in place(outer_cv_of_hp(hp)))
    return out


def build_variables(axles: dict, wheelbase: float, conv=None,
                    halfshafts: dict = None) -> list:
    """Variable Studio payload: one LENGTH variable per coordinate (from
    hardpoint_values), so Onshape sketches can reference #front_lca_outer_x
    etc."""
    out = []
    for base, (x, y, z) in hardpoint_values(
            axles, wheelbase, conv, halfshafts).items():
        for coord, value in (("x", x), ("y", y), ("z", z)):
            out.append({
                "name": f"{base}_{coord}",
                "type": "LENGTH",
                "expression": f"{value:.4f} mm",
                "description": "Baja Suspension Tool hardpoint",
            })
    return out


class OnshapeClient:
    """Minimal signed client. API keys go in via Basic auth (supported by
    Onshape for API keys), so there is no HMAC dance to get wrong."""

    def __init__(self, access_key: str, secret_key: str,
                 base: str = DEFAULT_BASE, request_fn=None):
        token = base64.b64encode(
            f"{access_key}:{secret_key}".encode()).decode()
        self._headers = {
            "Authorization": f"Basic {token}",
            "Accept": "application/json;charset=UTF-8; qs=0.09",
            "Content-Type": "application/json",
        }
        self._base = base.rstrip("/")
        # injectable for tests
        self._request = request_fn if request_fn is not None else self._urllib

    def _urllib(self, method: str, path: str, body=None):
        headers = dict(self._headers)
        if isinstance(body, tuple) and body and body[0] == "__multipart__":
            _, filename, blob = body
            data, ctype = _encode_multipart(filename, blob)
            headers["Content-Type"] = ctype
            headers.pop("Accept", None)
        elif body is not None:
            data = json.dumps(body).encode()
        else:
            data = None
        req = urllib.request.Request(self._base + path, data=data,
                                     headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=60) as resp:
                return json.loads(resp.read().decode() or "null")
        except urllib.error.HTTPError as e:
            detail = e.read().decode(errors="replace")[:300]
            raise RuntimeError(f"Onshape API {e.code} on {method} {path}: "
                               f"{detail}") from e

    # -- API calls -----------------------------------------------------
    def list_elements(self, did: str, wid: str) -> list:
        return self._request(
            "GET", f"/api/v6/documents/d/{did}/w/{wid}/elements")

    def create_variable_studio(self, did: str, wid: str, name: str) -> dict:
        return self._request(
            "POST", f"/api/v6/variables/d/{did}/w/{wid}/variablestudio",
            {"name": name})

    def set_variables(self, did: str, wid: str, eid: str, variables: list):
        return self._request(
            "POST", f"/api/v6/variables/d/{did}/w/{wid}/e/{eid}/variables",
            variables)

    # App elements: JSON-native storage inside a document — where the
    # full design file lives so teammates can pull it back into the tool.
    def create_app_element(self, did: str, wid: str, name: str,
                           json_tree: dict) -> dict:
        return self._request(
            "POST", f"/api/v6/appelements/d/{did}/w/{wid}",
            {"formatId": APP_FORMAT_ID, "name": name,
             "jsonTree": json_tree})

    def get_app_element_json(self, did: str, wid: str, eid: str) -> dict:
        return self._request(
            "GET",
            f"/api/v6/appelements/d/{did}/w/{wid}/e/{eid}/content/json")

    def delete_element(self, did: str, wid: str, eid: str):
        return self._request(
            "DELETE", f"/api/v6/elements/d/{did}/w/{wid}/e/{eid}")

    # -- geometry push (v1.11): Feature Studio + Part Studio + blob ----
    # These use the v10 surface (current) per the API research; the
    # variable/app-element calls above stay on v6 where they were
    # validated. Paths are centralised so a live fix is one line.
    def create_part_studio(self, did: str, wid: str, name: str) -> dict:
        return self._request(
            "POST", f"/api/v10/partstudios/d/{did}/w/{wid}", {"name": name})

    def create_feature_studio(self, did: str, wid: str, name: str) -> dict:
        return self._request(
            "POST", f"/api/v10/featurestudios/d/{did}/w/{wid}",
            {"name": name})

    def update_feature_studio(self, did: str, wid: str, eid: str,
                              contents: str) -> dict:
        return self._request(
            "POST", f"/api/v10/featurestudios/d/{did}/w/{wid}/e/{eid}",
            {"contents": contents})

    def feature_specs(self, did: str, wid: str, eid: str) -> dict:
        """The custom features a Feature Studio exports, INCLUDING the
        exact `namespace` string a Part Studio must use to instantiate
        them — so we never have to construct that string by hand."""
        return self._request(
            "GET",
            f"/api/v10/featurestudios/d/{did}/w/{wid}/e/{eid}/featurespecs")

    def part_studio_features(self, did: str, wid: str, eid: str) -> dict:
        return self._request(
            "GET",
            f"/api/v10/partstudios/d/{did}/w/{wid}/e/{eid}/features")

    def add_part_studio_feature(self, did: str, wid: str, eid: str,
                                feature: dict) -> dict:
        return self._request(
            "POST", f"/api/v10/partstudios/d/{did}/w/{wid}/e/{eid}/features",
            {"btType": "BTFeatureDefinitionCall-1406", "feature": feature})

    def update_part_studio_feature(self, did: str, wid: str, eid: str,
                                   fid: str, feature: dict) -> dict:
        return self._request(
            "POST",
            f"/api/v10/partstudios/d/{did}/w/{wid}/e/{eid}/features/"
            f"featureid/{fid}",
            {"btType": "BTFeatureDefinitionCall-1406", "feature": feature})

    def upload_blob(self, did: str, wid: str, filename: str,
                    data: bytes, eid: str = None) -> dict:
        """Create (eid None) or replace-in-place (eid given) a blob
        element. Multipart is handled by the request layer via a
        ('__multipart__', ...) sentinel so the injectable _request stays
        JSON-only for tests."""
        path = (f"/api/v10/blobelements/d/{did}/w/{wid}"
                + (f"/e/{eid}" if eid else ""))
        return self._request("POST", path,
                             ("__multipart__", filename, data))


def push_hardpoints(client: OnshapeClient, document_url: str, axles: dict,
                    wheelbase: float,
                    studio_name: str = DEFAULT_STUDIO_NAME,
                    conv=None) -> str:
    """Create-or-update `studio_name` in the document with the current
    hardpoints. Returns a human-readable summary of what happened."""
    did, wid = parse_document_url(document_url)
    elements = client.list_elements(did, wid)
    studio = next(
        (e for e in elements
         if e.get("elementType") == "VARIABLESTUDIO"
         and e.get("name") == studio_name), None)
    created = False
    if studio is None:
        made = client.create_variable_studio(did, wid, studio_name)
        studio_id = made.get("id") or made.get("elementId")
        if not studio_id:
            raise RuntimeError(
                f"could not create Variable Studio {studio_name!r} — create "
                "an empty Variable Studio tab with that exact name in the "
                "document, then push again")
        created = True
    else:
        studio_id = studio["id"]
    variables = build_variables(axles, wheelbase, conv=conv)
    client.set_variables(did, wid, studio_id, variables)
    which = " + ".join(k for k, v in axles.items() if v is not None)
    return (f"{'created' if created else 'updated'} Variable Studio "
            f"{studio_name!r} with {len(variables)} variables ({which})")


def push_design(client: OnshapeClient, document_url: str,
                project_dict: dict,
                name: str = DEFAULT_DESIGN_NAME) -> str:
    """Store the FULL design (the .MICK project as JSON) inside the
    Onshape document as an application element, create-or-replace by
    name. Teammates open the same document and pull the design straight
    back into the tool — the CAD and the kinematic model travel
    together. (Like the Variable Studio push, exercised against mocks;
    first live errors will be verbatim and actionable.)"""
    did, wid = parse_document_url(document_url)
    elements = client.list_elements(did, wid)
    existing = next((e for e in elements if e.get("name") == name), None)
    replaced = False
    if existing is not None:
        client.delete_element(did, wid, existing["id"])
        replaced = True
    made = client.create_app_element(did, wid, name, project_dict)
    eid = made.get("id") or made.get("elementId")
    if not eid:
        raise RuntimeError(
            f"Onshape did not return an element id creating {name!r}")
    return (f"{'replaced' if replaced else 'created'} design element "
            f"{name!r} in the document")


def _feature_instance(namespace: str) -> dict:
    """BTM for instantiating the custom hardpoints feature in a Part
    Studio. `namespace` is the STRING Onshape returns from featurespecs
    (e.g. 'd.../w.../e.../m...') locating the Feature Studio's code."""
    from .onshape_featurescript import FEATURE_NAME, FEATURE_TYPE_ID
    return {
        "btType": "BTMFeature-134",
        "featureType": FEATURE_TYPE_ID,
        "name": FEATURE_NAME,
        "namespace": namespace or "",
        "parameters": [],
    }


def _resolve_namespace(client, did, wid, fs_eid, micro=None) -> str:
    """The `namespace` string a Part Studio needs to instantiate our
    custom feature. Preferred source: the Feature Studio's featurespecs
    (Onshape hands back the exact string). Its shape isn't formally
    documented, so search it generically for our featureType. Fallback:
    construct the same-document reference `d..::w..::e..[::m..]` from the
    Feature Studio's element id (+ microversion when known)."""
    from .onshape_featurescript import FEATURE_TYPE_ID
    found = {"ns": ""}
    try:
        specs = client.feature_specs(did, wid, fs_eid)
    except Exception:
        specs = None

    def walk(node, ns_hint=""):
        if isinstance(node, dict):
            ns = node.get("namespace")
            ns = ns if isinstance(ns, str) else ns_hint
            ft = (node.get("featureType") or node.get("id")
                  or node.get("featureTypeName"))
            if ft == FEATURE_TYPE_ID and ns:
                found["ns"] = ns
            for v in node.values():
                walk(v, ns)
        elif isinstance(node, list):
            for v in node:
                walk(v, ns_hint)

    if specs is not None:
        walk(specs)
    if found["ns"]:
        return found["ns"]
    ref = f"d{did}::w{wid}::e{fs_eid}"
    if micro:
        ref += f"::m{micro}"
    return ref


def _feature_id_of(res) -> str:
    """Pull the featureId out of an add-feature response, whatever the
    nesting (top level, under 'feature', or under 'feature'.'message')."""
    if not isinstance(res, dict):
        return None
    for node in (res, res.get("feature"),
                 (res.get("feature") or {}).get("message")
                 if isinstance(res.get("feature"), dict) else None):
        if isinstance(node, dict) and node.get("featureId"):
            return node["featureId"]
    return None


def _find_existing_feature(client, did, wid, ps) -> str:
    """featureId of the hardpoints feature already in this Part Studio
    (matched by featureType), or None. Lets a re-push update in place even
    when the manifest is gone — the difference between preserving the
    user's references and duplicating the feature."""
    from .onshape_featurescript import FEATURE_TYPE_ID
    try:
        data = client.part_studio_features(did, wid, ps)
    except Exception:
        return None
    found = {"fid": None}

    def walk(node):
        if isinstance(node, dict):
            ft = node.get("featureType")
            fid = node.get("featureId")
            if ft == FEATURE_TYPE_ID and fid:
                found["fid"] = fid
            for v in node.values():
                walk(v)
        elif isinstance(node, list):
            for v in node:
                walk(v)

    walk(data)
    return found["fid"]


def push_geometry(client: OnshapeClient, document_url: str, axles: dict,
                  wheelbase: float, *, pdf_bytes: bytes = None,
                  manifest: dict = None, conv=None,
                  halfshafts: dict = None,
                  studio_name: str = DEFAULT_STUDIO_NAME,
                  partstudio_name: str = DEFAULT_PARTSTUDIO_NAME,
                  featurestudio_name: str = None,
                  pdf_name: str = DEFAULT_PDF_NAME) -> dict:
    """Build (first run) or UPDATE-IN-PLACE (every later run) the full CAD
    representation in the document: a Variable Studio of hardpoints, a
    Feature Studio holding the custom feature, a Part Studio that
    instantiates it (points + sketch planes), and a design-notes PDF.

    Idempotency is the whole point: a `manifest` (stored in the design
    app-element by the caller) remembers each tab's elementId and the
    feature instance's featureId. On a re-push we find those tabs, update
    the variables + code + PDF in place, and CREATE NOTHING NEW — so
    pushing again just moves the existing geometry. Returns
    {"summary": [...lines...], "manifest": {...}} — persist the manifest.

    halfshafts: {axle: HalfshaftConfig} so driven axles also get their
    inner/outer CV points + halfshaft line."""
    from .onshape_featurescript import (FEATURE_STUDIO_NAME,
                                         feature_studio_source)
    if featurestudio_name is None:
        featurestudio_name = FEATURE_STUDIO_NAME
    did, wid = parse_document_url(document_url)
    man = dict(manifest or {})
    elements = client.list_elements(did, wid)
    by_id = {e.get("id"): e for e in elements}
    summary = []

    def locate(key, name, etype):
        """Prior element for `key`: stored manifest id first (survives
        renames), else a name+type match. None => create it."""
        eid = man.get(key)
        if eid and eid in by_id:
            return eid
        hit = next((e for e in elements
                    if e.get("name") == name
                    and e.get("elementType") == etype), None)
        return hit["id"] if hit else None

    # Coordinates once — both the Variable Studio payload and the
    # FeatureScript embedding read the SAME numbers so they can't drift.
    values = hardpoint_values(axles, wheelbase, conv=conv,
                              halfshafts=halfshafts)

    # 1. Variable Studio — reference table (the feature is self-contained
    #    and doesn't depend on it being inserted, but power users can
    #    reference #variables in their own sketches).
    variables = build_variables(axles, wheelbase, conv=conv,
                                halfshafts=halfshafts)
    vs = locate("variable_studio", studio_name, "VARIABLESTUDIO")
    if vs is None:
        made = client.create_variable_studio(did, wid, studio_name)
        vs = made.get("id") or made.get("elementId")
        summary.append(f"created Variable Studio {studio_name!r}")
    else:
        summary.append(f"updated Variable Studio {studio_name!r}")
    client.set_variables(did, wid, vs, variables)
    man["variable_studio"] = vs
    summary[-1] += f" ({len(variables)} variables)"

    # 2. Feature Studio — holds the custom feature code (idempotent: we
    #    overwrite the contents each push so code fixes propagate).
    code = feature_studio_source(values)
    fs = locate("feature_studio", featurestudio_name, "FEATURESTUDIO")
    if fs is None:
        made = client.create_feature_studio(did, wid, featurestudio_name)
        fs = made.get("id") or made.get("elementId")
        summary.append(f"created Feature Studio {featurestudio_name!r}")
    else:
        summary.append(f"updated Feature Studio {featurestudio_name!r}")
    fs_resp = client.update_feature_studio(did, wid, fs, code)
    micro = None
    if isinstance(fs_resp, dict):
        micro = (fs_resp.get("microversionId")
                 or fs_resp.get("sourceMicroversion"))
    man["feature_studio"] = fs

    # 3. Part Studio + the custom-feature instance. The feature reads the
    #    variables, so once it exists we NEVER recreate it — updated
    #    variables regenerate all geometry on rebuild.
    ps = locate("part_studio", partstudio_name, "PARTSTUDIO")
    if ps is None:
        made = client.create_part_studio(did, wid, partstudio_name)
        ps = made.get("id") or made.get("elementId")
        summary.append(f"created Part Studio {partstudio_name!r}")
    else:
        summary.append(f"updated Part Studio {partstudio_name!r}")
    man["part_studio"] = ps
    namespace = _resolve_namespace(client, did, wid, fs, micro=micro)
    feat = _feature_instance(namespace)
    # CRITICAL for keeping the user's downstream references: find the
    # EXISTING hardpoints feature and update it in place. The stored
    # manifest id is tried first, but we also scan the Part Studio's
    # feature list for our featureType — so even if the manifest was lost
    # (a fresh machine, an app restart before saving), a re-push updates
    # the same feature (same entity ids -> references survive) instead of
    # pasting a duplicate.
    fid = man.get("feature_id") or _find_existing_feature(client, did,
                                                          wid, ps)
    if fid:
        client.update_part_studio_feature(did, wid, ps, fid, feat)
        man["feature_id"] = fid
        summary.append("  refreshed hardpoints feature in place "
                       "(references preserved)")
    else:
        res = client.add_part_studio_feature(did, wid, ps, feat)
        man["feature_id"] = _feature_id_of(res)
        summary.append("  added hardpoints feature (points/lines/planes)")

    # 4. Design-notes PDF blob, replaced in place.
    if pdf_bytes:
        blob = locate("pdf", pdf_name, "BLOB")
        res = client.upload_blob(did, wid, pdf_name, pdf_bytes, eid=blob)
        # keep the id: the create response carries the new one; on update
        # the tab id is unchanged (`blob`).
        new_id = None
        if isinstance(res, dict):
            new_id = res.get("id") or res.get("elementId")
        man["pdf"] = blob or new_id
        summary.append(f"{'updated' if blob else 'uploaded'} {pdf_name!r}")

    return {"summary": summary, "manifest": man}


def pull_design(client: OnshapeClient, document_url: str,
                name: str = DEFAULT_DESIGN_NAME) -> dict:
    """Fetch the design JSON stored by push_design. Raises with a clear
    message if the document has no design element of that name."""
    did, wid = parse_document_url(document_url)
    elements = client.list_elements(did, wid)
    existing = next((e for e in elements if e.get("name") == name), None)
    if existing is None:
        raise RuntimeError(
            f"no element named {name!r} in the document — use 'Sync "
            "design to Onshape' from the machine that has the design "
            "first")
    content = client.get_app_element_json(did, wid, existing["id"])
    tree = content.get("tree") if isinstance(content, dict) else None
    if tree is None and isinstance(content, dict):
        tree = content.get("jsonTree", content)
    if not isinstance(tree, dict) or "format" not in tree:
        raise RuntimeError(
            f"element {name!r} does not contain a Baja Tool design")
    return tree
