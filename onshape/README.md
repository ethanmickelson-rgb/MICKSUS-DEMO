# Getting hardpoints into Onshape (and keeping them updatable)

The goal: design/iterate kinematics in the Baja Suspension Tool, and have
your Onshape part studios **rebuild automatically** when hardpoints change
— no redoing tedious CAD work when packaging forces a geometry change.

Onshape has no native "import 3D points" — the community-standard answer
is a small custom FeatureScript feature, which is what we ship here.

## Route 1 (recommended to start): paste-CSV custom feature

1. **Install once:** in your Onshape suspension document, create a new
   **Feature Studio** tab, paste the contents of
   [`baja_hardpoints.fs`](baja_hardpoints.fs) into it, and commit. A
   custom feature named **"Baja Hardpoints"** now appears in that
   document's custom-feature dropdown (top toolbar).
2. **Use it:** in your suspension Part Studio, insert **Baja Hardpoints**
   as the FIRST feature. In the tool: `File → Copy hardpoints CSV (for
   Onshape)`, then paste into the feature's text box, set the units to
   match your display units, and accept. You get one **named point** per
   hardpoint (plus optional mate connectors, plus an optional mirrored
   right side).
3. **Build off the points:** sketch/loft/extrude your control arms,
   knuckle, and tabs referencing those points (or mate parts to the mate
   connectors in assemblies) — the standard "skeleton part studio"
   pattern FSAE/Baja teams use.
4. **Update:** when the kinematics change, copy the new CSV and re-paste
   it into the same feature. Everything downstream rebuilds.

Note: the tool models the LEFT corner in vehicle coordinates (+X forward,
+Y left, +Z up, origin on the ground at the centreline). Tick "Mirror" in
the feature to also get the right-side points.

> The FeatureScript is written against `onshape/std` v2260 syntax but was
> authored offline. If Onshape's editor flags anything on first commit,
> send the error text back and we'll fix it in one pass.

## Route 2 (nicer long-term): Variable Studio + REST API push

Onshape **Variable Studios** hold named, unit-aware variables that any
sketch dimension or mate-connector offset can reference (`#name`), and
they are writable from outside via Onshape's REST API (`setVariables`
endpoint) using free per-account API keys from
[dev-portal.onshape.com](https://dev-portal.onshape.com/keys).

The pipeline looks like: a Variable Studio holds 33 variables
(`#lca_outer_x`, `#lca_outer_y`, … — x/y/z per hardpoint), a skeleton
Part Studio's points are dimensioned by those variables, and the desktop
tool pushes new values over the API — every consuming tab rebuilds with
**zero clicks in Onshape**. Two caveats from our research: keep the
Variable Studio **in the same document** as the skeleton (cross-document
links are version-pinned), and don't use the uploaded-CSV-blob route
(FeatureScript file imports pin to a blob version and go stale).

Adding a "Push to Onshape" button to the tool is very doable (the
`onshape-client` PyPI package or a ~100-line signed-requests client) —
it needs your document/element IDs and an API key pair, so we'll wire it
up together when you want it.

## Also included

`File → Export hardpoints CSV…` writes the same table to a `.csv` file
(in your current display units) for Excel, SolidWorks equations, or
anything else.
