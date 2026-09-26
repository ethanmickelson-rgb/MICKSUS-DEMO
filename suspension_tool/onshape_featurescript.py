"""The custom FeatureScript feature MICKSUS builds inside the Onshape
document.

Design (v1.11.3, after live debugging with Ethan):

  * SELF-CONTAINED. MICKSUS embeds every hardpoint coordinate directly in
    the generated feature, so it ALWAYS builds geometry — no need to
    insert the Variable Studio into each Part Studio (the friction Ethan
    hit). It still reads a matching Variable Studio variable first when
    one is in scope, so live #variable edits override the embedded value;
    every push refreshes both.
  * It emits, per axle: named 3D points for every hardpoint (incl. the
    halfshaft inner/outer CV when driven); the lower/upper A-arm planes
    (through their three points); the 2D design sketch plane (normal to
    the bushing axis, so it carries the kickup + sketch-yaw); and a full
    reference skeleton of construction wire lines — lower/upper bushing
    axes, kingpin axis, both legs of each A-arm, tie rod, shock and
    halfshaft — as visual/reference lines (the real tubes don't run
    exactly on these, but they anchor the modelling).

`evaluateFeatureScript` is read-only, so this lives as an instantiated
custom FEATURE. The exact op signatures/import were validated from the
FsDoc; the one thing most likely to ever need a bump is STD_VERSION.
"""

# Feature type id (the export const name) and the human name shown on the
# feature. Keep STABLE — the Part Studio's feature instance references the
# type id, and re-push relies on finding it.
FEATURE_TYPE_ID = "suspensionHardpoints"
FEATURE_STUDIO_NAME = "MICKSUS Kinematics"
FEATURE_NAME = "Suspension hardpoints"

# std version for the source header + import. Backward compatibility means
# any REAL released version works; this is the single most likely thing to
# need a live bump if it's ever too new for the target document.
STD_VERSION = "2581"

# The reference skeleton, per axle: (slug, display suffix, pointA, pointB).
# Guarded in FS — any line whose endpoints are missing is skipped, so the
# halfshaft line just doesn't appear on an undriven axle.
_LINES = [
    ("lower_bushing_axis", "Lower Bushing Axis", "lca_inner_front",
     "lca_inner_rear"),
    ("upper_bushing_axis", "Upper Bushing Axis", "uca_inner_front",
     "uca_inner_rear"),
    ("kingpin_axis", "Kingpin Axis", "lca_outer", "uca_outer"),
    ("lca_front_leg", "LCA Front Leg", "lca_inner_front", "lca_outer"),
    ("lca_rear_leg", "LCA Rear Leg", "lca_inner_rear", "lca_outer"),
    ("uca_front_leg", "UCA Front Leg", "uca_inner_front", "uca_outer"),
    ("uca_rear_leg", "UCA Rear Leg", "uca_inner_rear", "uca_outer"),
    ("tie_rod", "Tie Rod", "tierod_inner", "tierod_outer"),
    ("shock", "Shock", "shock_inner", "shock_outer"),
    ("halfshaft", "Halfshaft", "halfshaft_inner", "halfshaft_outer"),
    # Hinge-carrier types (C-hub front / loaded halfshaft rear). Guarded
    # like everything else: absent on other types, they simply skip.
    ("arm_bushing_axis", "Arm Bushing Axis", "arm_inner_front",
     "arm_inner_rear"),
    ("arm_front_leg", "Arm Front Leg", "arm_inner_front", "hinge_front"),
    ("arm_rear_leg", "Arm Rear Leg", "arm_inner_rear", "hinge_rear"),
    ("hinge_pin", "Hinge Pin", "hinge_front", "hinge_rear"),
    ("camber_link", "Camber Link", "camber_inner", "camber_outer"),
    ("chub_kingpin", "Kingpin Axis (C-hub)", "kingpin_lower",
     "kingpin_upper"),
    ("loaded_halfshaft", "Loaded Halfshaft", "hs_inner", "hs_outer"),
    # H-arm rear: the lower H's outboard grab points + their cross member.
    ("harm_rail_front", "H-arm Rail Front", "arm_inner_front", "outer_front"),
    ("harm_rail_rear", "H-arm Rail Rear", "arm_inner_rear", "outer_rear"),
    ("harm_grab_line", "H-arm Grab Line", "outer_front", "outer_rear"),
]
# (slug, display suffix, p1, p2, p3) — planes through three points.
_PLANES3 = [
    ("lower_aarm_plane", "LowerAArmPlane", "lca_inner_front",
     "lca_inner_rear", "lca_outer"),
    ("upper_aarm_plane", "UpperAArmPlane", "uca_inner_front",
     "uca_inner_rear", "uca_outer"),
]


def _embedded_map_fs(values: dict) -> str:
    rows = [f'        "{base}" : [{x:.4f}, {y:.4f}, {z:.4f}]'
            for base, (x, y, z) in values.items()]
    return "{\n" + ",\n".join(rows) + "\n    }" if rows else "{}"


def feature_studio_source(values: dict) -> str:
    """Full FeatureScript source. `values` maps every point base name
    (e.g. 'front_lca_outer', 'front_halfshaft_inner') to its (x, y, z) in
    MILLIMETRES, in the document's chassis frame — the same numbers that
    go into the Variable Studio. Points/lines/planes are emitted only for
    the axles present in `values`."""
    axles = [a for a in ("front", "rear")
             if any(k.startswith(a + "_") for k in values)]
    point_bases = list(values.keys())
    bases_fs = ", ".join(f'"{b}"' for b in point_bases)

    body = []
    for a in axles:
        title = a.capitalize()
        for slug, disp, ba, bb in _LINES:
            body.append(
                f'        makeLine(context, id, "{a}_{slug}", '
                f'"{title} {disp}", "{a}_{ba}", "{a}_{bb}");')
        for slug, disp, p1, p2, p3 in _PLANES3:
            body.append(
                f'        makePlane3(context, id, "{a}_{slug}", '
                f'"{title} {disp}", "{a}_{p1}", "{a}_{p2}", "{a}_{p3}");')
        body.append(
            f'        makeSketchPlane(context, id, "{a}", '
            f'"{title} 2D Sketch Plane");')
    body_fs = "\n".join(body)

    return f'''FeatureScript {STD_VERSION};
import(path : "onshape/std/common.fs", version : "{STD_VERSION}.0");

// Auto-generated by MICKSUS (the Baja suspension tool). Reads hardpoint
// coordinates (embedded below, or from a Variable Studio variable of the
// same name when one is in scope) and builds named points, A-arm and
// design-sketch planes, and a construction-line skeleton. Do NOT hand
// edit — re-pushing from MICKSUS overwrites this file.

// Embedded coordinates in millimetres, document (chassis) frame.
const EMBEDDED = {_embedded_map_fs(values)};

const POINT_BASES = [{bases_fs}];

// A hardpoint position: a live Variable Studio variable wins (so #var
// edits move geometry); otherwise the embedded value. undefined when the
// point isn't part of this design (e.g. an undriven axle's halfshaft).
function pt(context is Context, base is string)
{{
    const vx = try silent(getVariable(context, base ~ "_x"));
    const vy = try silent(getVariable(context, base ~ "_y"));
    const vz = try silent(getVariable(context, base ~ "_z"));
    if (vx != undefined && vy != undefined && vz != undefined)
    {{
        return vector(vx, vy, vz);
    }}
    const e = EMBEDDED[base];
    if (e != undefined)
    {{
        return vector(e[0], e[1], e[2]) * millimeter;
    }}
    return undefined;
}}

function nameBody(context is Context, id is Id, name is string)
{{
    setProperty(context, {{
            "entities" : qCreatedBy(id, EntityType.BODY),
            "propertyType" : PropertyType.NAME,
            "value" : name
    }});
}}

function makePoint(context is Context, id is Id, base is string)
{{
    const p = pt(context, base);
    if (p == undefined) {{ return; }}
    opPoint(context, id + base, {{ "point" : p }});
    nameBody(context, id + base, base);
}}

// A construction/reference wire line between two hardpoints (the real
// tube rarely runs exactly on it, but it anchors the model).
function makeLine(context is Context, id is Id, slug is string, name is string, baseA is string, baseB is string)
{{
    const a = pt(context, baseA);
    const b = pt(context, baseB);
    if (a == undefined || b == undefined) {{ return; }}
    if (norm(b - a) < 1e-5 * meter) {{ return; }}
    opFitSpline(context, id + slug, {{ "points" : [a, b] }});
    nameBody(context, id + slug, name);
}}

function makePlane3(context is Context, id is Id, slug is string, name is string, b1 is string, b2 is string, b3 is string)
{{
    const p1 = pt(context, b1);
    const p2 = pt(context, b2);
    const p3 = pt(context, b3);
    if (p1 == undefined || p2 == undefined || p3 == undefined) {{ return; }}
    const n = cross(p2 - p1, p3 - p1);
    if (norm(n) < 1e-9 * meter * meter) {{ return; }}
    opPlane(context, id + slug, {{
            "plane" : plane(p1, normalize(n)),
            "width" : 10 * inch,
            "height" : 10 * inch
    }});
    nameBody(context, id + slug, name);
}}

// The 2D design sketch plane: origin at the lower ball joint, normal =
// the lower bushing-axis direction, so it inherits the kickup + any
// sketch-plane yaw.
function makeSketchPlane(context is Context, id is Id, axle is string, name is string)
{{
    const f = pt(context, axle ~ "_lca_inner_front");
    const r = pt(context, axle ~ "_lca_inner_rear");
    const o = pt(context, axle ~ "_lca_outer");
    if (f == undefined || r == undefined || o == undefined) {{ return; }}
    if (norm(f - r) < 1e-5 * meter) {{ return; }}
    opPlane(context, id + (axle ~ "_sketch_plane"), {{
            "plane" : plane(o, normalize(f - r)),
            "width" : 10 * inch,
            "height" : 10 * inch
    }});
    nameBody(context, id + (axle ~ "_sketch_plane"), name);
}}

annotation {{ "Feature Type Name" : "{FEATURE_NAME}" }}
export const {FEATURE_TYPE_ID} = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {{
    }}
    {{
        for (var base in POINT_BASES)
        {{
            makePoint(context, id, base);
        }}
{body_fs}
    }});
'''
