"""3D viewport: draws the whole vehicle's suspension — front and/or rear
axle, left side and optionally the mirrored right side — plus the roll
axis, per-axle instant-centre / roll-centre markers, ground planes, and
an optional "before" ghost. pyvista embedded in Qt.

Layout convention in the scene: each axle's hardpoints live in their own
axle-local frame (origin at that axle); the scene places the FRONT axle
at x = 0 and the REAR axle at x = -wheelbase. The right side is a pure
display mirror (y -> -y) of the solved left corner.

Drawing strategy (fast slider dragging): every displayed corner owns a
fixed set of polydata whose point counts never change, so updating a
frame just overwrites `.points` arrays — actors are only created in
rebuild(), never per frame.
"""

import numpy as np
import pyvista as pv
from PySide6.QtCore import Qt, Signal
from pyvistaqt import QtInteractor

GROUND_COLOR = "#3a4a3a"
# Chassis backdrop. It used to sit at 0.35, which reads fine for a single
# tube frame but goes strange when several STL parts are imported to show
# mounting points: VTK draws transparent surfaces without depth sorting,
# so overlapping shells stack into a muddle. 0.60 is solid enough to kill
# most of that and to read a mount face, while links behind the chassis
# stay visible. Adjustable live in the Frame panel and saved per project.
FRAME_OPACITY = 0.60
ARM_COLOR = "#4f8fd0"
ARM_COLOR_REAR = "#7fd04f"
UPRIGHT_COLOR = "#c0c0c0"
TIEROD_COLOR = "#e0a030"
SHOCK_COLOR = "#d05050"
POINT_COLOR = "#ffffff"
LOCK_COLOR = "#ffb703"    # locked hardpoints: amber, unmistakably not white
WHEEL_COLOR = "#404040"
IC_COLOR = "#ffa040"      # front-view instant centre
SVIC_COLOR = "#40d0e0"    # side-view instant centre
RC_COLOR = "#ff50e0"      # roll centre
ROLL_AXIS_COLOR = "#ff50e0"
CG_COLOR = "#ffd020"      # centre of gravity marker, light octants
CG_DARK = "#1a1a1a"       # ... and its dark ones
HALFSHAFT_COLOR = "#b070ff"  # CV axle (inner joint -> outer CV)

# Instant-centre markers farther than this from the axle are hidden (the
# side-view IC legitimately runs tens of metres out as the arms go
# parallel — a dot there is meaningless and used to drag the camera's
# idea of "the scene" with it). The IC direction line is clamped to this
# length instead so the geometry stays readable.
MARKER_MAX_DIST = 4000.0


def _add(plotter, mesh, **kw):
    """add_mesh without the per-actor camera reset/render (pyvista renders
    on every add otherwise — slow, and it thrashes fragile GL contexts).
    The scene resets the camera exactly once at the end of rebuild()."""
    kw.setdefault("reset_camera", False)
    kw.setdefault("render", False)
    return plotter.add_mesh(mesh, **kw)


def _cg_octant_meshes(radius: float):
    """The CAD centre-of-mass ball, as two flat-coloured half-meshes.

    A sphere chequered into eight octants by the parity of the three sign
    bits, so opposite octants share a colour — the symbol SolidWorks (and
    every other CAD package) uses. From any viewpoint you see four
    alternating quadrants, which reads as a position AND an orientation
    instead of the featureless blob a single-colour sphere gives.

    Two meshes rather than one scalar-mapped mesh on purpose: flat colours
    need no lookup table, so there is nothing to go wrong on a driver that
    is short of texture units.
    """
    s = pv.Sphere(radius=float(radius), center=(0.0, 0.0, 0.0),
                  theta_resolution=48, phi_resolution=48)
    c = s.cell_centers().points
    parity = ((c[:, 0] >= 0).astype(int) + (c[:, 1] >= 0).astype(int)
              + (c[:, 2] >= 0).astype(int)) % 2
    return (s.extract_cells(np.flatnonzero(parity == 0)),
            s.extract_cells(np.flatnonzero(parity == 1)))


def _segments(*pairs):
    """PolyData from (start, end) point pairs as independent line segments."""
    pts = np.vstack([p for pair in pairs for p in pair]).astype(float)
    n = len(pairs)
    lines = np.column_stack([np.full(n, 2), np.arange(0, 2 * n, 2),
                             np.arange(1, 2 * n, 2)]).ravel()
    return pv.PolyData(pts, lines=lines)


def _wheel_pose(center, spindle) -> np.ndarray:
    """4x4 matrix taking the template wheel (cylinder along +Y at the
    origin) onto the given centre/spindle — Rodrigues' formula."""
    y = np.array([0.0, 1.0, 0.0])
    s = np.asarray(spindle, float)
    s = s / np.linalg.norm(s)
    v = np.cross(y, s)
    c = float(np.dot(y, s))
    if np.linalg.norm(v) < 1e-12:
        r = np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    else:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        r = np.eye(3) + vx + vx @ vx * ((1 - c) / (np.dot(v, v)))
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = center
    return m


def _corner_layout(hp, state):
    """Per-suspension-type drawing recipe: {group: (color_key, width,
    [point pairs])} plus the hardpoint cloud. Pair counts are constant per
    type, so per-frame updates only overwrite point arrays."""
    from ..chub_front import CHubFrontPoints
    from ..harm_rear import HArmRearPoints
    from ..loaded_halfshaft import LoadedHalfshaftPoints
    from ..multilink import MultilinkPoints
    from ..trailing_arm import TrailingArmPoints
    if isinstance(hp, TrailingArmPoints):
        groups = {
            "arms": ("arm", 6, [(hp.pivot_inner, hp.pivot_outer),
                                (hp.pivot_inner, state.wheel_center),
                                (hp.pivot_outer, state.wheel_center)]),
            "shock": ("shock", 7, [(hp.shock_inner, state.shock_outer)]),
        }
        points = [hp.pivot_inner, hp.pivot_outer, hp.shock_inner,
                  state.wheel_center, state.shock_outer]
    elif isinstance(hp, CHubFrontPoints):
        groups = {
            "arms": ("arm", 5, [(hp.arm_inner_front, state.hinge_front),
                                (hp.arm_inner_rear, state.hinge_rear),
                                (state.hinge_front, state.hinge_rear)]),
            # C-hub + steering block: hinge pin up to the kingpin, the
            # kingpin itself, the camber-link perch, and the block out to
            # the wheel centre + steering arm.
            "upright": ("upright", 3, [
                (state.hinge_front, state.lbj),
                (state.hinge_rear, state.lbj),
                (state.lbj, state.ubj),
                (state.ubj, state.camber_outer),
                (state.lbj, state.wheel_center),
                (state.ubj, state.wheel_center),
                (state.tro, state.lbj)]),
            "tierod": ("tierod", 5, [(hp.tierod_inner, state.tro),
                                     (hp.camber_inner, state.camber_outer)]),
            "shock": ("shock", 7, [(hp.shock_inner, state.shock_outer)]),
        }
        points = [hp.arm_inner_front, hp.arm_inner_rear,
                  hp.camber_inner, hp.tierod_inner, hp.shock_inner,
                  state.hinge_front, state.hinge_rear, state.camber_outer,
                  state.ubj, state.lbj, state.tro, state.wheel_center,
                  state.shock_outer]
    elif isinstance(hp, LoadedHalfshaftPoints):
        groups = {
            "arms": ("arm", 5, [(hp.arm_inner_front, state.hinge_front),
                                (hp.arm_inner_rear, state.hinge_rear),
                                (state.hinge_front, state.hinge_rear)]),
            # knuckle: bushing pin down to the hub/outer CV + wheel centre
            "upright": ("upright", 3, [
                (state.hinge_front, state.hs_outer),
                (state.hinge_rear, state.hs_outer),
                (state.hs_outer, state.wheel_center)]),
            # the LOADED shaft — drawn as part of the linkage because it
            # IS one (diff flange to the outer CV, fixed length)
            "tierod": ("halfshaft", 7, [(hp.hs_inner, state.hs_outer)]),
            "shock": ("shock", 7, [(hp.shock_inner, state.shock_outer)]),
        }
        points = [hp.arm_inner_front, hp.arm_inner_rear, hp.hs_inner,
                  hp.shock_inner, state.hinge_front, state.hinge_rear,
                  state.hs_outer, state.wheel_center, state.shock_outer]
    elif isinstance(hp, HArmRearPoints):
        # lower H: two side rails + the outboard cross member (the grab
        # line that holds toe); the upright spans the two outboard grab
        # points up to the wheel centre + camber-link top.
        groups = {
            "arms": ("arm", 5, [(hp.arm_inner_front, state.outer_front),
                                (hp.arm_inner_rear, state.outer_rear),
                                (state.outer_front, state.outer_rear)]),
            "upright": ("upright", 3, [
                (state.outer_front, state.wheel_center),
                (state.outer_rear, state.wheel_center),
                (state.outer_front, state.camber_outer),
                (state.camber_outer, state.wheel_center)]),
            "tierod": ("tierod", 5, [(hp.camber_inner, state.camber_outer)]),
            "shock": ("shock", 7, [(hp.shock_inner, state.shock_outer)]),
        }
        points = [hp.arm_inner_front, hp.arm_inner_rear, hp.camber_inner,
                  hp.shock_inner, state.outer_front, state.outer_rear,
                  state.camber_outer, state.wheel_center, state.shock_outer]
    elif isinstance(hp, MultilinkPoints):
        outer = state.outer_points
        groups = {
            "arms": ("arm", 5, [(getattr(hp, f"link{k}_inner"), outer[k])
                                for k in range(1, 5)]),
            "tierod": ("tierod", 5, [(hp.link5_inner, outer[5])]),
            "upright": ("upright", 3, [(outer[k], state.wheel_center)
                                       for k in range(1, 6)]),
            "shock": ("shock", 7, [(hp.shock_inner, state.shock_outer)]),
        }
        points = ([getattr(hp, f"link{k}_inner") for k in range(1, 6)]
                  + [outer[k] for k in range(1, 6)]
                  + [state.wheel_center, hp.shock_inner, state.shock_outer])
    else:                                   # double wishbone
        # The tie-rod inner is chassis-fixed by default, but when the toe
        # link mounts ON a control arm (lower/upper) its ball-joint centre
        # rides that arm — draw the state-carried point so the link stays
        # attached through travel.
        tri = getattr(state, "tierod_inner_cur", None)
        if tri is None:
            tri = hp.tierod_inner
        groups = {
            "arms": ("arm", 5, [(hp.uca_inner_front, state.ubj),
                                (hp.uca_inner_rear, state.ubj),
                                (hp.lca_inner_front, state.lbj),
                                (hp.lca_inner_rear, state.lbj)]),
            "upright": ("upright", 3, [(state.lbj, state.ubj),
                                       (state.lbj, state.wheel_center),
                                       (state.ubj, state.wheel_center),
                                       (state.tro, state.lbj),
                                       (state.tro, state.ubj)]),
            "tierod": ("tierod", 5, [(tri, state.tro)]),
            "shock": ("shock", 7, [(hp.shock_inner, state.shock_outer)]),
        }
        points = [hp.uca_inner_front, hp.uca_inner_rear, hp.lca_inner_front,
                  hp.lca_inner_rear, tri, hp.shock_inner,
                  state.ubj, state.lbj, state.tro, state.wheel_center,
                  state.shock_outer]
    return groups, points


def drag_attrs_for(hp) -> list:
    """Hardpoint attribute per displayed point index — MUST match the
    `points` order _corner_layout builds for this type. Dragging happens
    at ride height, where the state-carried points (ball joints, wheel
    centre...) coincide with their hardpoint definitions exactly."""
    from ..chub_front import CHubFrontPoints
    from ..harm_rear import HArmRearPoints
    from ..loaded_halfshaft import LoadedHalfshaftPoints
    from ..multilink import MultilinkPoints
    from ..trailing_arm import TrailingArmPoints
    if isinstance(hp, TrailingArmPoints):
        return ["pivot_inner", "pivot_outer", "shock_inner",
                "wheel_center", "shock_outer"]
    if isinstance(hp, HArmRearPoints):
        return ["arm_inner_front", "arm_inner_rear", "camber_inner",
                "shock_inner", "outer_front", "outer_rear",
                "camber_outer", "wheel_center", "shock_outer"]
    if isinstance(hp, CHubFrontPoints):
        return ["arm_inner_front", "arm_inner_rear",
                "camber_inner", "tierod_inner", "shock_inner",
                "hinge_front", "hinge_rear", "camber_outer",
                "kingpin_upper", "kingpin_lower", "tierod_outer",
                "wheel_center", "shock_outer"]
    if isinstance(hp, LoadedHalfshaftPoints):
        return ["arm_inner_front", "arm_inner_rear", "hs_inner",
                "shock_inner", "hinge_front", "hinge_rear",
                "hs_outer", "wheel_center", "shock_outer"]
    if isinstance(hp, MultilinkPoints):
        return ([f"link{k}_inner" for k in range(1, 6)]
                + [f"link{k}_outer" for k in range(1, 6)]
                + ["wheel_center", "shock_inner", "shock_outer"])
    return ["uca_inner_front", "uca_inner_rear", "lca_inner_front",
            "lca_inner_rear", "tierod_inner", "shock_inner",
            "uca_outer", "lca_outer", "tierod_outer", "wheel_center",
            "shock_outer"]


class _CornerVisual:
    """Actors + meshes for ONE displayed corner: an axle side, placed at
    an x offset and optionally mirrored to the right (-Y) side. The
    drawing recipe comes from _corner_layout, so every suspension type
    renders through the same machinery."""

    def __init__(self, plotter, hp, state, x_offset: float, mirror: bool,
                 arm_color: str, halfshaft_inner=None, locked_idx=None):
        self.x_offset = x_offset
        self.mirror = mirror
        self._hp = hp
        self._locked_idx = list(locked_idx or ())
        self._hs_inner = (None if halfshaft_inner is None
                          else np.asarray(halfshaft_inner, float))
        colors = {"arm": arm_color, "upright": UPRIGHT_COLOR,
                  "tierod": TIEROD_COLOR, "shock": SHOCK_COLOR,
                  "halfshaft": HALFSHAFT_COLOR}
        groups, points = _corner_layout(hp, state)
        z = np.zeros(3)
        self._meshes = {}
        for name, (color_key, width, pairs) in groups.items():
            mesh = _segments(*[(z, z)] * len(pairs))
            self._meshes[name] = mesh
            _add(plotter, mesh, color=colors[color_key], line_width=width,
                 style="wireframe")
        if self._hs_inner is not None:
            z3 = np.zeros(3)
            hs = _segments((z3, z3))
            self._meshes["halfshaft"] = hs
            _add(plotter, hs, color=HALFSHAFT_COLOR, line_width=7,
                 style="wireframe")
            # visible CV joints: [inner, outer] — the inner is pickable
            # for dragging, both show in the hardpoint table
            hs_pts = pv.PolyData(np.zeros((2, 3)))
            self._meshes["hs_points"] = hs_pts
            _add(plotter, hs_pts, color=HALFSHAFT_COLOR, point_size=12,
                 render_points_as_spheres=True)
            # tire-axis stub: outer CV -> wheel centre, so it's visually
            # obvious the CV sits ON the tire's centre (spin) axis
            hs_axis = _segments((z3, z3))
            self._meshes["hs_axis"] = hs_axis
            _add(plotter, hs_axis, color=HALFSHAFT_COLOR, line_width=2,
                 style="wireframe", opacity=0.55)
        pts = pv.PolyData(np.zeros((len(points), 3)))
        self._meshes["points"] = pts
        _add(plotter, pts, color=POINT_COLOR, point_size=9,
             render_points_as_spheres=True)
        if self._locked_idx:
            # locked hardpoints re-drawn on top in amber, slightly larger,
            # so a pinned point is visually distinct from the free ones
            lk = pv.PolyData(np.zeros((len(self._locked_idx), 3)))
            self._meshes["locked_pts"] = lk
            _add(plotter, lk, color=LOCK_COLOR, point_size=13,
                 render_points_as_spheres=True)
        wheel = pv.Cylinder(center=(0, 0, 0), direction=(0, 1, 0),
                            radius=hp.tire_radius, height=hp.tire_width)
        self._wheel_actor = _add(plotter, 
            wheel, color=WHEEL_COLOR, opacity=0.55, smooth_shading=True)

    def _tf(self, p):
        """Axle-local point -> scene point (x offset, optional mirror)."""
        q = np.asarray(p, float).copy()
        q[0] += self.x_offset
        if self.mirror:
            q[1] = -q[1]
        return q

    def set_state(self, state) -> None:
        tf = self._tf
        groups, points = _corner_layout(self._hp, state)
        for name, (_, _, pairs) in groups.items():
            self._meshes[name].points = np.vstack(
                [tf(p) for pair in pairs for p in pair])
        self._meshes["points"].points = np.vstack([tf(p) for p in points])
        if self._locked_idx and "locked_pts" in self._meshes:
            self._meshes["locked_pts"].points = np.vstack(
                [tf(points[i]) for i in self._locked_idx
                 if i < len(points)])
        if self._hs_inner is not None and "halfshaft" in self._meshes:
            from ..halfshaft import outer_cv_of_state
            outer = outer_cv_of_state(state)
            ends = np.vstack([tf(self._hs_inner), tf(outer)])
            self._meshes["halfshaft"].points = ends
            if "hs_points" in self._meshes:
                self._meshes["hs_points"].points = ends.copy()
            if "hs_axis" in self._meshes:
                self._meshes["hs_axis"].points = np.vstack(
                    [tf(outer), tf(state.wheel_center)])
        spindle = np.asarray(state.spindle, float).copy()
        if self.mirror:
            spindle[1] = -spindle[1]
        self._wheel_actor.user_matrix = _wheel_pose(
            self._tf(state.wheel_center), spindle)


class _AxleMarkers:
    """RC / front-view IC / side-view IC markers for one axle."""

    def __init__(self, plotter, x_offset: float):
        self.x_offset = x_offset
        self.ic = pv.PolyData(np.zeros((1, 3)))
        self.svic = pv.PolyData(np.zeros((1, 3)))
        self.rc = pv.PolyData(np.zeros((1, 3)))
        self.line = _segments((np.zeros(3), np.zeros(3)))
        self.actors = [
            _add(plotter, self.ic, color=IC_COLOR, point_size=14,
                             render_points_as_spheres=True),
            _add(plotter, self.svic, color=SVIC_COLOR, point_size=14,
                             render_points_as_spheres=True),
            _add(plotter, self.rc, color=RC_COLOR, point_size=14,
                             render_points_as_spheres=True),
            _add(plotter, self.line, color=IC_COLOR, line_width=2,
                             style="wireframe", opacity=0.6),
        ]

    def update(self, state, rc_info, show_rc: bool,
               show_ic: bool) -> None:
        """rc_info: {"ic": (y,z)|None, "svic": (x,z)|None, "rc_h": float,
        optional "rc_y" (roll mode: the RC leaves the centreline)}.
        show_rc gates the roll-centre dot; show_ic gates the two
        instant-centre markers and the patch->IC ray — independently
        hideable so new users can declutter the view."""
        ic_a, svic_a, rc_a, line_a = self.actors
        if rc_info is None or not (show_rc or show_ic):
            for a in self.actors:
                a.SetVisibility(False)
            return
        dx = self.x_offset
        x0 = state.wheel_center[0] + dx
        ic = rc_info.get("ic")
        has_ic = ic is not None and show_ic
        line_a.SetVisibility(bool(has_ic))
        if has_ic:
            p = np.array([x0, ic[0], ic[1]])
            cp = np.asarray(state.contact_patch, float).copy()
            cp[0] += dx
            v = p - cp
            dist = float(np.linalg.norm(v))
            near = dist <= MARKER_MAX_DIST
            ic_a.SetVisibility(bool(near))
            if near:
                self.ic.points = p.reshape(1, 3)
            else:                       # clamp the ray, hide the dot
                p = cp + v * (MARKER_MAX_DIST / max(dist, 1e-9))
            self.line.points = np.vstack([cp, p])
        else:
            ic_a.SetVisibility(False)
        rc_h = rc_info.get("rc_h", np.nan)
        rc_y = rc_info.get("rc_y", 0.0)
        rc_ok = bool(show_rc and np.isfinite(rc_h) and np.isfinite(rc_y))
        rc_a.SetVisibility(rc_ok)
        if rc_ok:
            self.rc.points = np.array([[x0, rc_y, rc_h]])
        svic = rc_info.get("svic")
        has_sv = svic is not None and show_ic
        if has_sv:
            p = np.array([svic[0] + dx, state.wheel_center[1], svic[1]])
            wc = np.asarray(state.wheel_center, float).copy()
            wc[0] += dx
            has_sv = bool(np.linalg.norm(p - wc) <= MARKER_MAX_DIST)
            if has_sv:
                self.svic.points = p.reshape(1, 3)
        svic_a.SetVisibility(bool(has_sv))


class VehicleScene:
    """Builds and updates the whole-vehicle scene on ANY pyvista plotter."""

    def __init__(self, plotter):
        self.plotter = plotter
        plotter.set_background("#1e1e26")
        plotter.add_axes(xlabel="X fwd", ylabel="Y left", zlabel="Z up")
        plotter.enable_parallel_projection()
        self.show_rc = True    # roll-centre dots + the roll axis line
        self.show_ic = True    # front/side-view instant centres + rays
        self.show_cg = True
        self.show_ghost = True
        self.show_live_ground = True
        self._cg_actors = []
        self._cg_point = None
        self._ghost_actor = None
        self._corners = {}        # (axle, side) -> _CornerVisual
        self._markers = {}        # axle -> _AxleMarkers
        self._roll_axis_mesh = None
        self._roll_axis_actor = None
        self._live_ground_actor = None
        self._extent = 900.0
        self._wheel_z = 300.0
        self._frame_mesh = None      # chassis backdrop (persists rebuilds)
        self.frame_opacity = FRAME_OPACITY
        self._frame_matrix = np.eye(4)
        self._frame_actor = None
        self.show_frame = True
        self._overlay_actors = []    # IC/SVIC/RC/roll-axis actors: excluded
                                     # from camera bounds (can sit km away)
        self._built = False          # first rebuild frames the scene; later
                                     # rebuilds keep the user's camera
        # Self-healing orbit: after EVERY mouse interaction (orbit, pan,
        # zoom) drag the focal point back onto the view axis nearest the
        # model, so the NEXT rotation always pivots around the car — the
        # focal point can otherwise drift arbitrarily far away and the
        # camera orbits some point off in space.
        try:
            self.plotter.iren.add_observer(
                "EndInteractionEvent",
                lambda *_: self._recenter_rotation())
        except (AttributeError, RuntimeError):
            pass    # headless/offscreen plotters have no interactor
        self._ground_z = 0.0         # absolute z of the static ground plane

    # ------------------------------------------------------------------
    def rebuild(self, corners: dict, wheelbase: float, ghost=None) -> None:
        """Recreate the scene.

        corners: {"front"/"rear": {"hp":…, "state":…, "mirror": bool}} for
        every axle that should be DISPLAYED. ghost: (axle_key, hp, state)
        of a before-geometry overlay, drawn on that axle's station."""
        cam = None
        if self._built:
            c = self.plotter.camera
            cam = (tuple(c.position), tuple(c.focal_point), tuple(c.up),
                   float(c.parallel_scale))
        self.plotter.clear()
        self._corners = {}
        self._markers = {}
        self._overlay_actors = []
        offsets = {"front": 0.0, "rear": -wheelbase}
        colors = {"front": ARM_COLOR, "rear": ARM_COLOR_REAR}

        half_track = 700.0          # display FLOOR, so the ground plane
        true_half_track = 0.0       # never collapses; this one is the car
        gz = 0.0    # ground plane z: through the tire contact patches (the
                    # Onshape-aligned coordinate offset puts it below z = 0)
        # The hardpoints carry the Onshape-alignment x offset, so each
        # axle's true scene station is its wheel-centre x plus its display
        # offset — NOT simply 0 / -wheelbase. Everything that used to
        # assume "front axle at x = 0" (ground centre, centreline, roll
        # axis, CG) anchors to these stations instead.
        stations = []
        for key, info in corners.items():
            half_track = max(half_track, abs(info["hp"].wheel_center[1])
                             + info["hp"].tire_radius)
            true_half_track = max(true_half_track,
                                  abs(info["hp"].wheel_center[1])
                                  + info["hp"].tire_radius)
            self._wheel_z = float(info["hp"].wheel_center[2])
            gz = float(info["hp"].wheel_center[2]) - info["hp"].tire_radius
            stations.append(float(info["hp"].wheel_center[0]) + offsets[key])
        self._ground_z = gz
        cx = float(np.mean(stations)) if stations else 0.0
        self._center_x = cx
        extent = max(half_track, (wheelbase if len(corners) > 1 else 0.0))
        self._extent = extent

        ground = pv.Plane(center=(cx, 0, gz),
                          direction=(0, 0, 1),
                          i_size=2.5 * extent, j_size=2.6 * half_track)
        _add(self.plotter, ground, color=GROUND_COLOR, opacity=0.5)
        centerline = _segments((np.array([cx + 1.3 * extent, 0, gz]),
                                np.array([cx - 1.3 * extent, 0, gz])))
        _add(self.plotter, centerline, color="#888888", line_width=2,
                              style="wireframe")
        live = pv.Plane(center=(cx, 0, gz),
                        direction=(0, 0, 1),
                        i_size=2.5 * extent, j_size=2.6 * half_track)
        self._live_ground_actor = _add(self.plotter, 
            live, color="#7a8a5a", opacity=0.3)
        self._live_ground_actor.SetVisibility(False)

        for key, info in corners.items():
            sides = [False] + ([True] if info.get("mirror") else [])
            for mirror in sides:
                self._corners[(key, mirror)] = _CornerVisual(
                    self.plotter, info["hp"], info["state"], offsets[key],
                    mirror, colors[key],
                    halfshaft_inner=info.get("halfshaft_inner"),
                    locked_idx=info.get("locked_idx"))
            self._markers[key] = _AxleMarkers(self.plotter, offsets[key])
            self._overlay_actors.extend(self._markers[key].actors)

        # Chassis backdrop, if one has been loaded (persists rebuilds).
        self._frame_actor = None
        if self._frame_mesh is not None:
            self._frame_actor = _add(self.plotter, self._frame_mesh,
                                     color="#8a8a99",
                                     opacity=self.frame_opacity,
                                     smooth_shading=True)
            self._frame_actor.user_matrix = self._frame_matrix
            self._frame_actor.SetVisibility(bool(self.show_frame))

        # CG marker: the CAD centre-of-mass symbol, moved by actor matrix.
        # Sized from the CAR (real track / wheelbase), not from `extent`,
        # which carries a 700 mm half-track floor for the ground plane and
        # would leave a 44 mm ball sitting on a 260 mm-wheelbase RC car.
        car = max(2.0 * true_half_track, wheelbase if len(corners) > 1 else 0.0)
        cg_r = float(np.clip(0.015 * car, 2.0, 40.0)) if car > 0 else 22.0
        self._cg_actors = []
        for mesh, col in zip(_cg_octant_meshes(cg_r), (CG_COLOR, CG_DARK)):
            a = _add(self.plotter, mesh, color=col, opacity=0.95)
            a.SetVisibility(False)
            self._cg_actors.append(a)
        if self._cg_point is not None:      # re-show across rebuilds
            self.set_cg(self._cg_point, _render=False)

        # Roll axis: RC front to RC rear, extended a little past both.
        self._roll_axis_mesh = _segments((np.zeros(3), np.zeros(3)))
        self._roll_axis_actor = _add(self.plotter, 
            self._roll_axis_mesh, color=ROLL_AXIS_COLOR, line_width=3,
            style="wireframe", opacity=0.8)
        self._roll_axis_actor.SetVisibility(False)
        self._overlay_actors.append(self._roll_axis_actor)

        self._ghost_actor = None
        if ghost is not None:
            gkey, ghp, gst = ghost
            dx = offsets.get(gkey, 0.0)
            def tf(p):
                q = np.asarray(p, float).copy()
                q[0] += dx
                return q
            ggroups, _ = _corner_layout(ghp, gst)
            pairs = [pair for _, _, ps in ggroups.values() for pair in ps]
            self._ghost_actor = _add(self.plotter,
                _segments(*[(tf(a), tf(b)) for a, b in pairs]),
                color="#aab4c8", opacity=0.35, line_width=2,
                style="wireframe")
            self._ghost_actor.SetVisibility(bool(self.show_ghost))

        for key, info in corners.items():
            self.set_state(key, info["state"], info.get("rc_info"),
                           state_right=info.get("state_right"),
                           _render=False)
        if cam is None:
            # First build: frame the scene WITHOUT forcing a render — the
            # GL context may not be realized yet (offscreen/startup), and
            # Qt repaints on first expose anyway.
            e = self._extent
            self.plotter.camera_position = [
                (self._center_x + 2.0 * e, 2.2 * e, 1.2 * e),
                (self._center_x, 0.3 * e, self._wheel_z),
                (0.0, 0.0, 1.0)]
            # NOT plotter.reset_camera(): that wrapper ALWAYS renders
            # afterwards, and the GL context may not exist yet at startup.
            self.plotter.renderer.reset_camera(render=False,
                                               bounds=self._model_bounds())
        else:
            # Editing a number must NOT move the view: restore the exact
            # camera the user had, then quietly drag the orbit pivot back
            # onto the model (see _recenter_rotation).
            c = self.plotter.camera
            c.position, c.focal_point, c.up = cam[0], cam[1], cam[2]
            c.parallel_scale = cam[3]
            self._recenter_rotation()
            self._render()
        self._built = True

    def _render(self) -> None:
        """Render ONLY when it can matter: the window must have drawn
        itself at least once AND (for the Qt-embedded plotter) be visible.
        Explicit renders on a hidden/never-realized GL context are
        pointless — Qt repaints on show — and they are exactly what
        crashes fragile (offscreen/startup) drivers mid-__init__."""
        rw = self.plotter.render_window
        if rw is None or rw.GetNeverRendered():
            return
        vis = getattr(self.plotter, "isVisible", None)
        if vis is not None and not vis():
            return
        self.plotter.render()

    def _model_bounds(self):
        """Bounds of the CAR (linkages, wheels, frame, ground) — the
        IC/SVIC/RC/roll-axis overlays are excluded because a side-view IC
        can legitimately sit tens of metres out, and letting it into the
        bounds dragged both the orbit pivot and Fit view off to it (the
        'blue dot way off in space' problem). Returns None when the scene
        holds nothing measurable."""
        skip = {id(a) for a in self._overlay_actors}
        b = None
        try:
            actors = list(self.plotter.renderer.actors.values())
        except AttributeError:
            actors = []
        for a in actors:
            if id(a) in skip or not a.GetVisibility():
                continue
            ab = a.GetBounds()
            if ab is None or ab[0] > ab[1]:
                continue
            if b is None:
                b = list(ab)
            else:
                for i in (0, 2, 4):
                    b[i] = min(b[i], ab[i])
                    b[i + 1] = max(b[i + 1], ab[i + 1])
        return b

    def _recenter_rotation(self) -> None:
        """Move the camera focal point to the point on the view axis nearest
        the MODEL centre. This leaves the image untouched (the view direction
        is unchanged; in parallel projection zoom is parallel_scale, and in
        perspective the focal distance doesn't alter the frustum) — but VTK
        orbits about the focal point, which otherwise drifts far off into
        space as the user pans/zooms across rebuilds, making rotation pivot
        around a weird unknown point."""
        b = self._model_bounds()
        if b is None or b[0] > b[1]:
            return
        center = np.array([(b[0] + b[1]) / 2.0, (b[2] + b[3]) / 2.0,
                           (b[4] + b[5]) / 2.0])
        cam = self.plotter.camera
        p = np.asarray(cam.position, float)
        d = np.asarray(cam.focal_point, float) - p
        n = np.linalg.norm(d)
        if n < 1e-9:
            return
        d /= n
        cam.focal_point = tuple(p + d * float(np.dot(center - p, d)))

    # ------------------------------------------------------------------
    def set_state(self, axle: str, state, rc_info=None,
                  state_right=None, _render: bool = True) -> None:
        """Update one axle's displayed corners for a solved pose (cheap).

        state_right, when given, poses the mirrored right side with its
        OWN solution (roll visualization: left in bump while right is in
        droop); otherwise the right side mirrors the left pose."""
        for (key, mirror), visual in self._corners.items():
            if key == axle:
                visual.set_state(state_right if (mirror and
                                                 state_right is not None)
                                 else state)
        if axle in self._markers:
            self._markers[axle].update(state, rc_info, self.show_rc,
                                       self.show_ic)
        if _render:
            self._render()

    def fit(self) -> None:
        """Zoom-to-fit: frame the CAR (ignoring far-flung IC markers)."""
        b = self._model_bounds()
        if b is not None:
            self.plotter.renderer.reset_camera(render=False, bounds=b)
        else:
            self.plotter.renderer.reset_camera(render=False)
        self._recenter_rotation()
        self._render()

    def recenter_orbit(self) -> None:
        """Explicit rescue: put the orbit pivot back on the model
        WITHOUT changing what's on screen."""
        self._recenter_rotation()
        self._render()

    def set_live_ground(self, travel: float) -> None:
        """Ground plane at the ACTIVE axle's current travel."""
        if self._live_ground_actor is None:
            return
        m = np.eye(4)
        m[2, 3] = travel
        self._live_ground_actor.user_matrix = m
        self._live_ground_actor.SetVisibility(
            bool(self.show_live_ground and abs(travel) > 1e-6))
        self._render()

    def set_cg(self, point, _render: bool = True) -> None:
        """Place (or hide, with None) the CG marker at an absolute
        (x, y, z) mm position."""
        self._cg_point = None if point is None else np.asarray(point, float)
        if not self._cg_actors:
            return
        ok = bool(self.show_cg and self._cg_point is not None
                  and np.all(np.isfinite(self._cg_point)))
        m = np.eye(4)
        if ok:
            m[:3, 3] = self._cg_point
        for actor in self._cg_actors:       # both octant groups move as one
            actor.SetVisibility(ok)
            if ok:
                actor.user_matrix = m
        if _render:
            self._render()

    def set_show_cg(self, on: bool) -> None:
        self.show_cg = bool(on)
        self.set_cg(self._cg_point)

    def set_frame_opacity(self, value: float) -> None:
        """0 = invisible, 1 = solid. Applied to the live actor so the
        slider moves the chassis without a scene rebuild."""
        self.frame_opacity = float(np.clip(value, 0.0, 1.0))
        if self._frame_actor is not None:
            self._frame_actor.GetProperty().SetOpacity(self.frame_opacity)
            self._render()

    def set_show_ghost(self, on: bool) -> None:
        """Show/hide the before-optimization ghost overlay (it can sit
        right on top of the tuned geometry and get in the way)."""
        self.show_ghost = bool(on)
        if self._ghost_actor is not None:
            self._ghost_actor.SetVisibility(bool(on))
        self._render()

    def set_axis_convention(self, conv) -> None:
        """Relabel the axes gizmo in the chosen display convention. The
        scene itself always renders in tool coordinates; each gizmo arrow
        gets the name that direction carries in the display frame (e.g.
        the forward arrow reads '+Y fwd' in the chassis convention)."""
        names = []
        for i in range(3):
            col = conv.mat[:, i]      # display components of tool axis i
            j = int(np.argmax(np.abs(col)))
            sign = "-" if col[j] < 0 else "+"
            names.append(f"{sign}{conv.labels[j]}")
        self.plotter.add_axes(xlabel=names[0], ylabel=names[1],
                              zlabel=names[2])
        self._render()

    def set_roll_axis(self, p_front, p_rear) -> None:
        """Show the roll axis through the two roll centres (or hide it)."""
        if self._roll_axis_actor is None:
            return
        ok = bool(self.show_rc and p_front is not None and p_rear is not None
                  and np.all(np.isfinite(p_front))
                  and np.all(np.isfinite(p_rear)))
        self._roll_axis_actor.SetVisibility(ok)
        if ok:
            a, b = np.asarray(p_front, float), np.asarray(p_rear, float)
            d = b - a
            self._roll_axis_mesh.points = np.vstack([a - 0.25 * d,
                                                     b + 0.25 * d])
        self._render()

    # ------------------------------------------------------------------
    def set_frame(self, mesh, matrix) -> None:
        """Install/replace the chassis backdrop mesh (None removes it).
        Takes effect at the next rebuild; the transform applies now."""
        self._frame_mesh = mesh
        self._frame_matrix = np.asarray(matrix, dtype=float)
        if self._frame_actor is not None:
            if mesh is None:
                self._frame_actor.SetVisibility(False)
            else:
                self._frame_actor.user_matrix = self._frame_matrix
        self._render()

    def set_frame_matrix(self, matrix) -> None:
        self._frame_matrix = np.asarray(matrix, dtype=float)
        if self._frame_actor is not None:
            self._frame_actor.user_matrix = self._frame_matrix
        self._render()

    def set_show_frame(self, on: bool) -> None:
        self.show_frame = bool(on)
        if self._frame_actor is not None:
            self._frame_actor.SetVisibility(bool(on))
        self._render()

    # ------------------------------------------------------------------
    def set_parallel(self, on: bool) -> None:
        if on:
            self.plotter.enable_parallel_projection()
        else:
            self.plotter.disable_parallel_projection()
        self._render()

    def view_named(self, name: str) -> None:
        """Standard views. 'front' looks down the car's X axis (the 2D
        sketch view), 'side' shows X-Z, 'top' shows X-Y, 'iso' is 3D."""
        if name == "front":
            self.plotter.view_yz(negative=True)
        elif name == "side":
            self.plotter.view_xz()
        elif name == "top":
            self.plotter.view_xy()
        else:
            e = self._extent
            self.plotter.camera_position = [
                (self._center_x + 2.0 * e, 2.2 * e, 1.2 * e),
                (self._center_x, 0.3 * e, self._wheel_z),
                (0.0, 0.0, 1.0)]
        self.plotter.reset_camera()
        self._render()


class CornerViewport(QtInteractor):
    """The Qt-embedded 3D view: a QtInteractor delegating to VehicleScene
    (shared with off-screen rendering in tests/screenshots).

    v1.5 adds OptimumKinematics-style direct manipulation: click a
    hardpoint sphere of the ACTIVE axle and drag it. Motion follows the
    view plane; holding X, Y or Z locks it to that axis of the ACTIVE
    DISPLAY CONVENTION (so 'Y' is fore-aft when chassis coords are on).
    The geometry re-solves live during the drag; releasing runs the full
    pipeline (undo, sweep, checklist). Escape cancels."""

    drag_started = Signal(str)                 # attr name
    drag_moved = Signal(str, object)           # attr, new point (mm, tool)
    drag_finished = Signal(str, object)        # attr, final point
    drag_cancelled = Signal()

    PICK_RADIUS_PX = 14.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene = VehicleScene(self)
        self.setFocusPolicy(Qt.StrongFocus)    # receive X/Y/Z lock keys
        self._drag_ctx = None    # {"axle", "attrs": [attr per point idx]}
        self._conv = None        # display convention for axis-lock keys
        self._drag = None        # live drag state dict
        self._keys = set()

    # -- wiring from the main window -----------------------------------
    def set_drag_context(self, axle_key: str, attrs: list | None) -> None:
        """Which axle is editable and, per displayed point index, which
        hardpoint attribute it manipulates (None disables dragging)."""
        self._drag_ctx = (None if attrs is None
                          else {"axle": axle_key, "attrs": attrs})

    def set_drag_convention(self, conv) -> None:
        self._conv = conv

    # -- picking helpers -------------------------------------------------
    def _dpr(self) -> float:
        try:
            return float(self.devicePixelRatioF())
        except AttributeError:
            return 1.0

    def _display_of(self, world) -> tuple:
        ren = self.scene.plotter.renderer
        ren.SetWorldPoint(world[0], world[1], world[2], 1.0)
        ren.WorldToDisplay()
        return ren.GetDisplayPoint()

    def _world_at(self, xd: float, yd: float, zd: float) -> np.ndarray:
        ren = self.scene.plotter.renderer
        ren.SetDisplayPoint(xd, yd, zd)
        ren.DisplayToWorld()
        wx, wy, wz, w = ren.GetWorldPoint()
        if abs(w) > 1e-12:
            wx, wy, wz = wx / w, wy / w, wz / w
        return np.array([wx, wy, wz])

    def _qt_to_display(self, pos) -> tuple:
        r = self._dpr()
        return pos.x() * r, (self.height() - pos.y()) * r

    def _pick_point(self, pos):
        """Nearest displayed hardpoint of the active corner within
        PICK_RADIUS_PX of the cursor; returns (index, scene_point)."""
        if self._drag_ctx is None:
            return None
        visual = self.scene._corners.get((self._drag_ctx["axle"], False))
        if visual is None:
            return None
        xq, yq = self._qt_to_display(pos)
        pts = np.asarray(visual._meshes["points"].points)
        best = None
        for i, p in enumerate(pts):
            xd, yd, _ = self._display_of(p)
            d = np.hypot(xd - xq, yd - yq)
            if d < self.PICK_RADIUS_PX * self._dpr() and (
                    best is None or d < best[0]):
                best = (d, i, p.copy())
        # the halfshaft's inner CV is draggable too (special index -1)
        if "hs_points" in visual._meshes:
            p = np.asarray(visual._meshes["hs_points"].points)[0]
            xd, yd, _ = self._display_of(p)
            d = np.hypot(xd - xq, yd - yq)
            if d < self.PICK_RADIUS_PX * self._dpr() and (
                    best is None or d < best[0]):
                best = (d, -1, p.copy())
        return None if best is None else (best[1], best[2])

    # -- mouse / key events ----------------------------------------------
    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton and self._drag_ctx is not None:
            hit = self._pick_point(ev.position().toPoint()
                                   if hasattr(ev, "position") else ev.pos())
            if hit is not None:
                idx, scene_pt = hit
                attrs = self._drag_ctx["attrs"]
                if idx == -1:
                    attrs = {-1: "halfshaft_inner"}
                if (idx == -1 or idx < len(attrs)) and attrs[idx] is not None:
                    visual = self.scene._corners[(self._drag_ctx["axle"],
                                                  False)]
                    self._drag = {
                        "attr": attrs[idx],
                        "x_offset": visual.x_offset,
                        "scene_start": scene_pt,
                        "depth": self._display_of(scene_pt)[2],
                        "point": scene_pt - np.array(
                            [visual.x_offset, 0.0, 0.0]),
                    }
                    self._drag["orig"] = self._drag["point"].copy()
                    self.drag_started.emit(self._drag["attr"])
                    ev.accept()
                    return
        super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        if self._drag is not None:
            xq, yq = self._qt_to_display(
                ev.position().toPoint() if hasattr(ev, "position")
                else ev.pos())
            world = self._world_at(xq, yq, self._drag["depth"])
            delta = world - self._drag["scene_start"]
            delta = self._lock_axis(delta)
            new_pt = self._drag["orig"] + delta
            self._drag["point"] = new_pt
            self.drag_moved.emit(self._drag["attr"], new_pt)
            ev.accept()
            return
        super().mouseMoveEvent(ev)

    def mouseReleaseEvent(self, ev):
        if self._drag is not None and ev.button() == Qt.LeftButton:
            drag, self._drag = self._drag, None
            self.drag_finished.emit(drag["attr"], drag["point"])
            ev.accept()
            return
        super().mouseReleaseEvent(ev)

    def _lock_axis(self, delta: np.ndarray) -> np.ndarray:
        """Constrain the drag to a display-convention axis while its key
        is held (the labels the user sees are the axes they get)."""
        keymap = {Qt.Key_X: 0, Qt.Key_Y: 1, Qt.Key_Z: 2}
        axes = [keymap[k] for k in self._keys if k in keymap]
        if not axes or self._conv is None:
            return delta
        out = np.zeros(3)
        for j in axes:
            direction = self._conv.from_display(np.eye(3)[j])
            out += np.dot(delta, direction) * direction
        return out

    def keyPressEvent(self, ev):
        if ev.key() in (Qt.Key_X, Qt.Key_Y, Qt.Key_Z):
            self._keys.add(ev.key())
            ev.accept()
            return                      # don't feed VTK's key bindings
        if ev.key() == Qt.Key_Escape and self._drag is not None:
            self._drag = None
            self.drag_cancelled.emit()
            ev.accept()
            return
        super().keyPressEvent(ev)

    def keyReleaseEvent(self, ev):
        self._keys.discard(ev.key())
        super().keyReleaseEvent(ev)

    def preview_hardpoints(self, axle_key: str, hp, state) -> None:
        """Live drag feedback: repoint the corner visuals (both sides) at
        the candidate hardpoints and pose them with `state` — no rebuild,
        no sweep, just the cheap per-frame update."""
        for mirror in (False, True):
            visual = self.scene._corners.get((axle_key, mirror))
            if visual is not None:
                visual._hp = hp
                visual.set_state(state)
        self.scene._render()

    def rebuild(self, corners, wheelbase, ghost=None):
        self.scene.rebuild(corners, wheelbase, ghost=ghost)

    def set_state(self, axle, state, rc_info=None, state_right=None):
        self.scene.set_state(axle, state, rc_info=rc_info,
                             state_right=state_right)

    def fit(self):
        self.scene.fit()

    def recenter_orbit(self):
        self.scene.recenter_orbit()

    def set_live_ground(self, travel):
        self.scene.set_live_ground(travel)

    def set_roll_axis(self, p_front, p_rear):
        self.scene.set_roll_axis(p_front, p_rear)

    def set_cg(self, point):
        self.scene.set_cg(point)

    def set_show_cg(self, on):
        self.scene.set_show_cg(on)

    def set_frame_opacity(self, value):
        self.scene.set_frame_opacity(value)

    def set_show_ghost(self, on):
        self.scene.set_show_ghost(on)

    def set_axis_convention(self, conv):
        self.scene.set_axis_convention(conv)

    def set_parallel(self, on):
        self.scene.set_parallel(on)

    def view_named(self, name):
        self.scene.view_named(name)

    def set_frame(self, mesh, matrix):
        self.scene.set_frame(mesh, matrix)

    def set_frame_matrix(self, matrix):
        self.scene.set_frame_matrix(matrix)

    def set_show_frame(self, on):
        self.scene.set_show_frame(on)

    def set_show_rc(self, on):
        self.scene.show_rc = on

    def set_show_ic(self, on):
        self.scene.show_ic = on

    def set_show_live_ground(self, on):
        self.scene.show_live_ground = on
