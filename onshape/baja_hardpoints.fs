// Baja Hardpoints — paste-CSV custom feature for Onshape
//
// Creates one named point body (and optionally a mate connector) per
// suspension hardpoint from CSV text pasted straight out of the Baja
// Suspension Tool (File -> "Copy hardpoints CSV (for Onshape)").
//
// Workflow: keep ONE instance of this feature at the top of your
// suspension Part Studio and build everything else off the points it
// makes. When the design changes, re-paste the new CSV into this feature
// and every downstream sketch/extrude/mate that references the points
// rebuilds automatically — no tedious rework.
//
// Input format (comment lines starting with # and the header row are
// ignored):   name,x,y,z      one hardpoint per line.
// Units are selected in the dialog and must match what the tool exported.
// Coordinates are the tool's vehicle frame: +X forward, +Y left, +Z up,
// origin on the ground at the vehicle centreline. The optional mirror
// checkbox also creates the right-side points (Y negated).
//
// To install: Onshape -> create a Feature Studio tab -> paste this file's
// contents -> commit. The feature then appears in that document's custom
// feature dropdown (share the document to use it across the team).

FeatureScript 2260;
import(path : "onshape/std/common.fs", version : "2260.0");

annotation { "Feature Type Name" : "Baja Hardpoints",
             "Feature Type Description" : "Suspension hardpoints from pasted CSV" }
export const bajaHardpoints = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Hardpoints CSV (name,x,y,z)",
                     "Description" : "Paste from the Baja Suspension Tool" }
        definition.csv is string;

        annotation { "Name" : "Units of the pasted values" }
        definition.units is LengthUnit;

        annotation { "Name" : "Mirror to right side (-Y)" }
        definition.mirror is boolean;

        annotation { "Name" : "Also create mate connectors" }
        definition.mateConnectors is boolean;
    }
    {
        const rows = parseCsv(definition.csv);
        if (size(rows) == 0)
            throw regenError("No hardpoint rows found - paste the CSV from the tool.");

        var index = 0;
        for (var row in rows)
        {
            const sides = definition.mirror ? [1, -1] : [1];
            for (var side in sides)
            {
                const suffix = side == 1 ? "" : "_R";
                const pointId = id + ("pt_" ~ index ~ suffix);
                const location = vector(row.x, side * row.y, row.z) * definition.units;
                opPoint(context, pointId, { "point" : location });
                setProperty(context, {
                    "entities" : qCreatedBy(pointId, EntityType.VERTEX),
                    "propertyType" : PropertyType.NAME,
                    "value" : row.name ~ suffix
                });
                if (definition.mateConnectors)
                {
                    opMateConnector(context, id + ("mc_" ~ index ~ suffix), {
                        "coordSystem" : coordSystem(location, vector(1, 0, 0), vector(0, 0, 1)),
                        "owner" : qNothing()
                    });
                }
            }
            index += 1;
        }
    });

// Parse "name,x,y,z" lines; skip blanks, comment lines (#...), and the
// header row. Tolerates Windows line endings and stray spaces.
function parseCsv(csv is string) returns array
{
    var rows = [];
    for (var line in splitString(csv, "\n"))
    {
        line = replace(line, "\r", "");
        line = replace(line, " ", "");
        if (line == "" || match(line, "#.*").hasMatch)
            continue;
        const parts = splitString(line, ",");
        if (size(parts) != 4)
            continue;
        const x = stringToNumber(parts[1]);
        const y = stringToNumber(parts[2]);
        const z = stringToNumber(parts[3]);
        if (x == undefined || y == undefined || z == undefined)
            continue;   // header row ("name,x,y,z") lands here
        rows = append(rows, { "name" : parts[0], "x" : x, "y" : y, "z" : z });
    }
    return rows;
}
