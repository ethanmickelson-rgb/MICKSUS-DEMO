"""Main window: wires the panels, viewport, and plots together.

Since v0.7.0 a project holds BOTH axles. The toolbar's "Editing" switch
flips every panel (setup, hardpoints, tweaks, optimizer, readouts, plots,
travel slider) between the FRONT and REAR designs, while the 3D view can
display either or both — with an optional mirrored right side — plus the
roll axis joining the two roll centres. A steering slider turns the front
rack within user-set limits.

State model: `self.axles` holds one Axle record per end (hardpoints,
solver, sweep, travel range, current travel, optimizer baseline/ghost).
Exactly one axle is "active" (being edited); the compatibility properties
`hp/solver/sweep/current_travel` refer to it. All coordinates stay in
each axle's local frame (origin at that axle); only the 3D scene and the
CAD exports place the rear at x = -wheelbase.
"""

import dataclasses

import numpy as np
from PySide6.QtCore import Qt, QTimer, QSettings
from PySide6.QtGui import QAction
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDockWidget,
    QFileDialog, QFormLayout, QHBoxLayout, QLabel,
    QLineEdit, QMainWindow, QMessageBox, QPushButton, QScrollArea,
    QSlider, QSplitter, QTabWidget, QToolBar, QVBoxLayout, QWidget,
)

from ..metrics import corner_metrics, front_view_ic, side_view_ic, sweep_metrics
from ..optimize import Goal, optimize
from ..project import (FILE_EXTENSION, AxleDesign, ProjectState,
                       hardpoints_csv, load_project, save_project,
                       vehicle_csv)
from ..chub_front import CHubFrontPoints, seed_chub_front
from ..geometry import DoubleWishbonePoints
from ..harm_rear import HArmRearPoints, seed_harm_rear
from ..loaded_halfshaft import LoadedHalfshaftPoints, seed_loaded_halfshaft
from ..multilink import MultilinkPoints, MultilinkSolver, seed_multilink
from ..seed import IN, SetupVariables, generate_seed
from .widgets import CommitIntSpin, CommitSpin, commit_spins
from ..solver import DoubleWishboneSolver
from ..trailing_arm import (TrailingArmPoints, TrailingArmSolver,
                            seed_trailing_arm)
from ..vehicle import anti_geometry
from ..units import DEFAULT_UNIT, UNITS
from ..vehicle import VehicleParams, roll_axis_metrics, roll_axis_points
from .panels import (FramePanel, HardpointTable, OptimizePanel, Readouts,
                     SetupForm, TweaksPanel, VehiclePanel)
from .plots import PANELS, MetricPlots
from .viewport import CornerViewport

SWEEP_POINTS = 41
ANIM_INTERVAL_MS = 30
UNDO_DEPTH = 15


class _WheelGuard(__import__("PySide6.QtCore", fromlist=["QObject"]).QObject):
    """Swallows scroll-wheel events over spinboxes and combo boxes unless
    the widget is focused (clicked into) — accidental scrolling over the
    left panel was silently rewriting designs."""

    def eventFilter(self, obj, event):
        from PySide6.QtCore import QEvent
        from PySide6.QtWidgets import QAbstractSpinBox, QComboBox
        # Block wheel edits on value widgets UNCONDITIONALLY (v1.3.1: the
        # old "only when unfocused" rule still let a focused spinbox eat a
        # scroll and silently wreck a design). Values change by typing or
        # clicking the arrows only; the wheel just scrolls the panel.
        if (event.type() == QEvent.Type.Wheel
                and isinstance(obj, (QAbstractSpinBox, QComboBox))):
            return True
        return False

# What a blank workspace opens with. Deliberately ROUND numbers for a
# plausible Baja car rather than any particular vehicle's spec: this is
# the first screen a new user sees, and a real car's measurements sitting
# there reads as a recommendation. Change them to your own car and save a
# project -- the defaults only apply to a workspace that has never been
# told otherwise.
DEFAULT_SETUP = SetupVariables(
    track_width=60.0 * IN, wheelbase=60.0 * IN, ride_height=12.0 * IN,
    tire_radius=11.5 * IN, tire_width=7.0 * IN,
    shock_min_length=14.0 * IN, shock_max_length=22.0 * IN,
    shock_length_at_ride=19.0 * IN, motion_ratio_goal=0.55,
    static_camber_deg=-1.0, kickup_deg=8.0,
)


@dataclasses.dataclass
class Axle:
    """Everything the window tracks per axle end."""

    name: str
    hp: object = None
    solver: object = None
    sweep: dict | None = None
    last_setup: SetupVariables | None = None
    droop: float = 75.0          # mm — ACTUAL swept range (may be clamped)
    bump: float = 75.0           # mm
    req_droop: float = 75.0      # mm — what the user ASKED for; when the
    req_bump: float = 75.0       # geometry can't reach it, droop/bump are
                                 # clamped and the checklist flags the gap
    hs_warns: list | None = None  # last halfshaft warnings (None = off)
    travel: float = 0.0          # mm, currently displayed (LEFT wheel)
    travel_right: float = 0.0    # mm, RIGHT wheel in "Free (L/R)" mode —
                                 # independent of the left so a single-wheel
                                 # bump can be posed to read the roll centre
    rc_h: float = float("nan")   # roll-centre height at the current travel
    state: object = None         # last solved pose (for vehicle-level calcs)
    baseline: tuple | None = None
    ghost: tuple | None = None
    undo: list = dataclasses.field(default_factory=list)  # hp history
    halfshaft: object = None     # HalfshaftConfig (set in MainWindow.__init__)
    shock_spec: object = None    # ShockSpec: FIXED hardware min/max lengths
                                 # (persisted; clamps the travel range so a
                                 # pose can never over-stroke the shock)
    locked: frozenset = frozenset()  # pinned hardpoint attrs (won't drag /
                                     # tweak); persisted in the .MICK


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        from .. import __version__
        from .branding import APP_NAME, app_icon
        self._version = __version__
        self._app_name = APP_NAME
        self.setWindowIcon(app_icon())
        self._retitle()
        self.unit = UNITS[DEFAULT_UNIT]
        from ..axes import CONVENTIONS, DEFAULT_CONVENTION
        self.conv = CONVENTIONS[DEFAULT_CONVENTION]
        from PySide6.QtWidgets import QApplication
        self._wheel_guard = _WheelGuard(self)
        QApplication.instance().installEventFilter(self._wheel_guard)
        self.axles = {"front": Axle("front"), "rear": Axle("rear")}
        self.active_key = "front"
        self.current_path = None
        self._onshape_manifest = None    # Onshape push tab/feature ids
        self._retitle()

        # --- left panel (top to bottom = the design workflow) ------------
        self.setup_form = SetupForm(self.unit)
        self.hardpoint_table = HardpointTable(self.unit)
        self.tweaks_panel = TweaksPanel(self.unit)
        self.optimize_panel = OptimizePanel(self.unit)
        self.vehicle_panel = VehiclePanel(self.unit)
        self.frame_panel = FramePanel(self.unit)
        self._frame_blob = None   # packed mesh bytes for saving
        from .dynamics_panel import DynamicsPanel
        self.dynamics_panel = DynamicsPanel(self.unit)
        from .panels import HalfshaftPanel
        self.halfshaft_panel = HalfshaftPanel(self.unit)
        from .checklist import ChecklistPanel
        self.checklist = ChecklistPanel(self.unit)
        from .candidates import CandidatesPanel
        self.candidates_panel = CandidatesPanel(self.unit)
        self.candidates_panel.snapshot_requested.connect(
            self.snapshot_candidate)
        self.candidates_panel.load_requested.connect(self.load_candidate)
        self.candidates_panel.changed.connect(self._autosave_soon)
        self.checklist.changed.connect(lambda: self._update_checklist())
        # Rolling autosave: debounce writes so rapid edits cost nothing.
        self._autosave_timer = QTimer(self)
        self._autosave_timer.setSingleShot(True)
        self._autosave_timer.setInterval(4000)
        self._autosave_timer.timeout.connect(self._write_autosave)
        from ..halfshaft import HalfshaftConfig
        for key, axle in self.axles.items():
            axle.halfshaft = HalfshaftConfig()
        self.halfshaft_panel.load(self.axles["front"].halfshaft,
                                  self.axles["rear"].halfshaft)
        self.readouts = Readouts(self.unit)

        # --- right: viewport over plots ----------------------------------
        self.viewport = CornerViewport()
        self.plots = MetricPlots(self.unit)

        # View controls row.
        view_controls = QWidget()
        vc = QHBoxLayout(view_controls)
        vc.setContentsMargins(4, 0, 4, 0)
        vc.addWidget(QLabel(f"<b>v{self._version}</b>"))
        fit_btn = QPushButton("Fit view")
        fit_btn.clicked.connect(lambda: self.viewport.fit())
        vc.addWidget(fit_btn)
        recenter_btn = QPushButton("Recenter orbit")
        recenter_btn.setToolTip(
            "Put the rotation pivot back on the car without moving the "
            "view (it also self-heals after every orbit/pan/zoom now)")
        recenter_btn.clicked.connect(lambda: self.viewport.recenter_orbit())
        vc.addWidget(recenter_btn)
        vc.addWidget(QLabel("View:"))
        for label, name in [("Front (sketch)", "front"), ("Side", "side"),
                            ("Top", "top"), ("Iso", "iso")]:
            b = QPushButton(label)
            b.clicked.connect(lambda _=False, n=name: self.viewport.view_named(n))
            vc.addWidget(b)
        self.show_front_check = QCheckBox("Show front")
        self.show_front_check.setChecked(True)
        self.show_rear_check = QCheckBox("Show rear")
        self.show_rear_check.setChecked(True)
        self.mirror_check = QCheckBox("Mirror right side")
        for cb in (self.show_front_check, self.show_rear_check,
                   self.mirror_check):
            cb.toggled.connect(lambda _=False: self.rebuild_scene())
            vc.addWidget(cb)
        self.parallel_check = QCheckBox("Parallel proj.")
        self.parallel_check.setChecked(True)
        self.parallel_check.toggled.connect(self.viewport.set_parallel)
        vc.addWidget(self.parallel_check)
        self.rc_check = QCheckBox("Roll centres")
        self.rc_check.setChecked(True)
        self.rc_check.setToolTip(
            "Show/hide the roll-centre dots and the roll axis line")
        self.rc_check.toggled.connect(self._on_rc_toggled)
        self.ic_check = QCheckBox("IC markers")
        self.ic_check.setChecked(True)
        self.ic_check.setToolTip(
            "Show/hide the front/side-view instant-centre markers and "
            "their construction rays")
        self.ic_check.toggled.connect(self._on_ic_toggled)
        vc.addWidget(self.rc_check)
        vc.addWidget(self.ic_check)
        self.cg_check = QCheckBox("CG")
        self.cg_check.setChecked(True)
        self.cg_check.toggled.connect(
            lambda on: self.viewport.set_show_cg(on))
        vc.addWidget(self.cg_check)
        self.snap_check = QCheckBox("Snap to frame")
        self.snap_check.setChecked(False)
        self.snap_check.setToolTip(
            "When releasing a dragged hardpoint, snap it to the nearest "
            "frame-mesh point (within ~1.2 in). Off by default: real "
            "mounts sit on welded tabs OFFSET from the tubes.")
        vc.addWidget(self.snap_check)
        self.ghost_check = QCheckBox("Ghost")
        self.ghost_check.setChecked(True)
        self.ghost_check.setToolTip(
            "Show the before-optimization geometry overlay")
        self.ghost_check.toggled.connect(
            lambda on: self.viewport.set_show_ghost(on))
        vc.addWidget(self.ghost_check)
        self.live_ground_check = QCheckBox("Live ground")
        self.live_ground_check.setChecked(True)
        self.live_ground_check.toggled.connect(self._on_live_ground_toggled)
        vc.addWidget(self.live_ground_check)
        from PySide6.QtWidgets import QMenu, QToolButton
        graphs_btn = QToolButton()
        graphs_btn.setText("Graphs ▾")
        graphs_btn.setPopupMode(QToolButton.InstantPopup)
        menu = QMenu(graphs_btn)
        self._graph_actions = {}
        for key, label, _ in PANELS:
            act = QAction(label, self, checkable=True)
            act.setChecked(True)
            act.toggled.connect(self._on_graphs_changed)
            menu.addAction(act)
            self._graph_actions[key] = act
        graphs_btn.setMenu(menu)
        vc.addWidget(graphs_btn)
        sweeps_btn = QToolButton()
        sweeps_btn.setText("Sweeps ▾")
        sweeps_btn.setPopupMode(QToolButton.InstantPopup)
        sweeps_menu = QMenu(sweeps_btn)
        for text, slot in [("Roll sweep…", self.open_roll_sweep),
                           ("Pitch sweep…", self.open_pitch_sweep),
                           ("Steering sweep…", self.open_steer_sweep),
                           ("Bump-steer map…", self.open_bump_steer_map),
                           ("Ride/grip frequency response (Bode)…",
                            self.open_ride_bode),
                           ("Yaw-moment diagram (MMM)…",
                            self.open_yaw_moment),
                           ("Steering effort (rack force)…",
                            self.open_steering_effort)]:
            act = QAction(text, self)
            act.triggered.connect(slot)
            sweeps_menu.addAction(act)
        sweeps_btn.setMenu(sweeps_menu)
        vc.addWidget(sweeps_btn)
        compare_btn = QToolButton()
        compare_btn.setText("Compare ▾")
        compare_btn.setPopupMode(QToolButton.InstantPopup)
        compare_menu = QMenu(compare_btn)
        for text, slot in [("Snapshot current curves", self.snapshot_curves),
                           ("Clear snapshots", self.clear_snapshots)]:
            act = QAction(text, self)
            act.triggered.connect(slot)
            compare_menu.addAction(act)
        compare_btn.setMenu(compare_menu)
        vc.addWidget(compare_btn)
        vc.addStretch()
        self._snapshot_count = 0
        self._sweep_dialogs = []   # keep references so they stay alive

        # Travel bar (drives the ACTIVE axle).
        travel_bar = QWidget()
        tb = QHBoxLayout(travel_bar)
        tb.setContentsMargins(4, 0, 4, 0)
        self.play_btn = QPushButton("▶ Play")
        self.play_btn.setCheckable(True)
        self.play_btn.toggled.connect(self._on_play)
        tb.addWidget(self.play_btn)
        self.motion_combo = QComboBox()
        self.motion_combo.addItems(["Heave", "Roll", "Free (L/R)"])
        self.motion_combo.setToolTip(
            "Heave: both wheels together. Roll: left wheel up while the "
            "right goes down (both axles roll with the chassis). "
            "Free (L/R): the right wheel gets its OWN travel slider so you "
            "can hold one wheel at ride height and bump the other — the "
            "single-wheel-bump roll-centre check. Turn on 'Mirror right "
            "side' to see both wheels.")
        self.motion_combo.currentTextChanged.connect(self._on_motion_mode)
        tb.addWidget(self.motion_combo)
        self.both_axles_check = QCheckBox("Both axles")
        self.both_axles_check.setToolTip(
            "Heave moves the front AND rear suspension together (each "
            "clamped to its own travel range) — see the whole car move.")
        self.both_axles_check.toggled.connect(
            lambda _=False: self.show_travel(self.active.travel))
        tb.addWidget(self.both_axles_check)
        tb.addWidget(QLabel("Droop"))
        self.slider = QSlider(Qt.Horizontal)   # integer mm
        self.slider.valueChanged.connect(lambda v: self.show_travel(float(v)))
        tb.addWidget(self.slider, stretch=1)
        tb.addWidget(QLabel("Bump"))
        self.travel_label = QLabel("0")
        self.travel_label.setMinimumWidth(80)
        tb.addWidget(self.travel_label)
        # Right-wheel travel — only used/visible in "Free (L/R)" mode.
        self.right_label_lo = QLabel("R droop")
        tb.addWidget(self.right_label_lo)
        self.right_slider = QSlider(Qt.Horizontal)
        self.right_slider.valueChanged.connect(
            lambda v: self.show_travel_right(float(v)))
        tb.addWidget(self.right_slider, stretch=1)
        self.right_label_hi = QLabel("R bump")
        tb.addWidget(self.right_label_hi)
        self.right_travel_label = QLabel("0")
        self.right_travel_label.setMinimumWidth(80)
        tb.addWidget(self.right_travel_label)
        for w in (self.right_label_lo, self.right_slider,
                  self.right_label_hi, self.right_travel_label):
            w.setVisible(False)
        ride_btn = QPushButton("Ride height")
        ride_btn.setToolTip("Back to static travel (0)")
        ride_btn.clicked.connect(self.go_ride_height)
        tb.addWidget(ride_btn)
        reset_btn = QPushButton("Reset pose")
        reset_btn.setToolTip("Ride height AND steering centred")
        reset_btn.clicked.connect(self.reset_pose)
        tb.addWidget(reset_btn)
        tb.addWidget(QLabel("range -"))
        self.droop_spin = CommitSpin()
        tb.addWidget(self.droop_spin)
        tb.addWidget(QLabel("+"))
        self.bump_spin = CommitSpin()
        tb.addWidget(self.bump_spin)
        self.droop_spin.editingFinished.connect(self._on_range_changed)
        self.bump_spin.editingFinished.connect(self._on_range_changed)

        # Steering bar (drives the FRONT axle's rack, within user limits).
        steer_bar = QWidget()
        sb = QHBoxLayout(steer_bar)
        sb.setContentsMargins(4, 0, 4, 0)
        sb.addWidget(QLabel("Front rack steer"))
        self.steer_slider = QSlider(Qt.Horizontal)  # integer mm of rack
        self.steer_slider.valueChanged.connect(
            lambda v: self.set_steer(float(v)))
        sb.addWidget(self.steer_slider, stretch=1)
        self.steer_label = QLabel("0")
        self.steer_label.setMinimumWidth(80)
        sb.addWidget(self.steer_label)
        center_btn = QPushButton("Center")
        center_btn.clicked.connect(lambda: self.steer_slider.setValue(0))
        sb.addWidget(center_btn)
        sb.addWidget(QLabel("rack limit ±"))
        self.steer_limit_spin = CommitSpin()
        self.steer_limit_spin.editingFinished.connect(self._on_steer_limit)
        sb.addWidget(self.steer_limit_spin)
        self.steer = 0.0             # mm of rack displacement (front axle)
        self.steer_limit = 38.0      # mm (~1.5 in each way, editable)

        view_box = QWidget()
        vb = QVBoxLayout(view_box)
        vb.setContentsMargins(0, 0, 0, 0)
        vb.addWidget(view_controls)
        vb.addWidget(self.viewport, stretch=1)
        vb.addWidget(travel_bar)
        vb.addWidget(steer_bar)

        right = QSplitter(Qt.Vertical)
        right.addWidget(view_box)
        right.addWidget(self.plots)
        right.setSizes([550, 350])
        self.setCentralWidget(right)

        # --- dockable panels (CAD-style): each design panel lives in its
        # own dock — drag it out, float it, tab it with others, close it,
        # or bring it back from the Panels menu. Default layout: one tabbed
        # stack on the left with the hardpoint table showing, and the live
        # readouts always visible below it.
        self.setDockOptions(QMainWindow.AnimatedDocks
                            | QMainWindow.AllowTabbedDocks
                            | QMainWindow.AllowNestedDocks)
        self.setTabPosition(Qt.LeftDockWidgetArea, QTabWidget.North)
        self._docks = {}

        def _dock(key: str, title: str, panel) -> QDockWidget:
            d = QDockWidget(title, self)
            d.setObjectName(f"dock_{key}")
            sc = QScrollArea()
            sc.setWidget(panel)
            sc.setWidgetResizable(True)
            sc.setMinimumWidth(360)
            d.setWidget(sc)
            self._docks[key] = d
            return d

        tabbed = [
            _dock("setup", "Setup / Seed", self.setup_form),
            _dock("hardpoints", "Hardpoints", self.hardpoint_table),
            _dock("tweaks", "Tweaks", self.tweaks_panel),
            _dock("optimize", "Optimize", self.optimize_panel),
            _dock("vehicle", "Vehicle", self.vehicle_panel),
            _dock("dynamics", "Dynamics", self.dynamics_panel),
            _dock("halfshafts", "Halfshafts", self.halfshaft_panel),
            _dock("candidates", "Candidates", self.candidates_panel),
            _dock("frame", "Frame", self.frame_panel),
        ]
        for d in tabbed:
            self.addDockWidget(Qt.LeftDockWidgetArea, d)
        for d in tabbed[1:]:
            self.tabifyDockWidget(tabbed[0], d)
        readouts_dock = _dock("readouts", "Readouts", self.readouts)
        self.addDockWidget(Qt.LeftDockWidgetArea, readouts_dock)
        self.splitDockWidget(tabbed[0], readouts_dock, Qt.Vertical)
        checklist_dock = _dock("checklist", "Checklist", self.checklist)
        self.addDockWidget(Qt.LeftDockWidgetArea, checklist_dock)
        self.tabifyDockWidget(readouts_dock, checklist_dock)
        checklist_dock.raise_()
        self._docks["hardpoints"].raise_()

        self._build_menu_and_toolbar()

        # --- signals -------------------------------------------------------
        self.setup_form.generate_requested.connect(self.generate_seed)
        self.setup_form.measure_requested.connect(self.measure_setup_from_design)
        self.optimize_panel.optimize_requested.connect(self.run_optimization)
        self.optimize_panel.resquare_requested.connect(self.resquare_arms)
        self.hardpoint_table.edited.connect(self.apply_hardpoints)
        self.hardpoint_table.set_halfshaft_provider(
            lambda: self.active.halfshaft)
        self.hardpoint_table.set_design_yaw_provider(
            lambda: self.design_yaw_of(self.active))
        self.hardpoint_table.set_arm_plane_provider(
            lambda: getattr(self.active.last_setup,
                            "steering_arm_plane_deg", 90.0))
        self.hardpoint_table.halfshaft_inner_edited.connect(
            self._on_hs_inner_edited)
        self.hardpoint_table.halfshaft_outer_edited.connect(
            self._on_hs_outer_edited)
        self.tweaks_panel.edited.connect(self.apply_hardpoints)
        self.tweaks_panel.blocked.connect(
            lambda msg: self.statusBar().showMessage(msg, 6000))
        self.hardpoint_table.locks_changed.connect(self._on_locks_changed)
        self.hardpoint_table.note.connect(
            lambda msg: self.statusBar().showMessage(msg, 10000))
        self.vehicle_panel.changed.connect(self._on_vehicle_changed)
        self.vehicle_panel.ride_height_requested.connect(
            self.set_vehicle_ride_height)
        self.halfshaft_panel.changed.connect(self._on_halfshaft_changed)
        self.viewport.drag_started.connect(self._on_drag_started)
        self.viewport.drag_moved.connect(self._on_drag_moved)
        self.viewport.drag_finished.connect(self._on_drag_finished)
        self.viewport.drag_cancelled.connect(self._on_drag_cancelled)
        self.tweaks_panel.sketch_yaw_changed.connect(self._on_sketch_yaw)
        self.frame_panel.load_requested.connect(self.load_frame_dialog)
        self.frame_panel.changed.connect(self._on_frame_changed)
        self._anim = QTimer(self)
        self._anim.setInterval(ANIM_INTERVAL_MS)
        self._anim.timeout.connect(self._anim_tick)
        self._anim_dir = 1.0

        self._apply_range_unit()
        self._apply_steer_unit()
        # Default to the team's chassis coordinate convention everywhere.
        self.set_convention(self.conv.key)
        # MICKSUS boots dark (v1.9). Checked HERE, after the plots widget
        # exists — the toggle's slot styles it too.
        self.dark_check.setChecked(True)
        # Boot BLANK (v1.9): no surprise default seed. The user loads
        # their frame, sets their numbers, and hits Generate seed.
        self.setup_form.load_setup(DEFAULT_SETUP)
        self.hardpoint_table.refresh(None)
        self.rebuild_scene()
        self.statusBar().showMessage(
            "Blank workspace — load your chassis (Frame panel), set the "
            "Setup values, then Generate seed", 15000)

    def set_panel_font(self, pt: int) -> None:
        """Apply a font size to the dock panels only (the 3D view, plots
        and toolbar keep theirs). Persists across sessions."""
        for dock in self._docks.values():
            dock.setStyleSheet(f"QWidget {{ font-size: {pt}pt; }}")
        if hasattr(self, "_ui_settings"):
            self._ui_settings.setValue("panel_pt2", int(pt))
        # rows scale with the font, with breathing room for readability
        self.hardpoint_table.table.verticalHeader().setDefaultSectionSize(
            max(24, int(pt * 2.9)))

    def _retitle(self) -> None:
        """MICKSUS — <file> vX.Y.Z, following the loaded project."""
        import os
        name = (os.path.basename(self.current_path)
                if getattr(self, "current_path", None) else "untitled")
        self.setWindowTitle(
            f"{self._app_name} — {name}  ·  v{self._version}")

    # ------------------------------------------------------------------
    # Active-axle plumbing
    # ------------------------------------------------------------------
    @property
    def active(self) -> Axle:
        return self.axles[self.active_key]

    # Compatibility accessors (tests/scripts predating the two-axle model).
    @property
    def hp(self):
        return self.active.hp

    @property
    def solver(self):
        return self.active.solver

    @property
    def sweep(self):
        return self.active.sweep

    @property
    def current_travel(self):
        return self.active.travel

    @property
    def last_setup(self):
        return self.active.last_setup

    def switch_axle(self, key: str) -> None:
        """Point every editing panel at the other axle. If that axle has
        no geometry yet, seed it from the current setup-form values so the
        user lands on something editable."""
        if key == self.active_key:
            return
        self.active_key = key
        axle = self.active
        if axle.hp is None:
            self.statusBar().showMessage(
                f"{key} axle has no geometry yet — set the Setup values "
                "and hit Generate seed", 8000)
            self._sync_range_spins()
            self._sync_type_combo()
            self.rebuild_scene()
            self.hardpoint_table.refresh(None)
            self._update_checklist()
            return
        if axle.last_setup is not None:
            self.setup_form.load_setup(axle.last_setup)
        self._resync_active_ui()

    def _resync_active_ui(self) -> None:
        """Point every editing panel/plot/scene at the ACTIVE axle's
        current state (used on axle switch and after any operation that
        edits a non-active axle behind the scenes)."""
        axle = self.active
        self._sync_range_spins()
        self._sync_type_combo()
        self.hardpoint_table.refresh(axle.hp)
        self.tweaks_panel.set_context(axle.last_setup)
        self.tweaks_panel.refresh(axle.hp)
        self._push_locks(axle)
        self._push_vehicle_frame()
        if axle.sweep is not None:
            self.plots.set_sweep(axle.sweep["travel_mm"], axle.sweep,
                                 baseline=axle.baseline)
        self.rebuild_scene()
        self.show_travel(axle.travel)

    def _on_axle_combo(self, name: str) -> None:
        self.switch_axle("front" if name.startswith("Front") else "rear")

    # ------------------------------------------------------------------
    def _build_menu_and_toolbar(self) -> None:
        file_menu = self.menuBar().addMenu("&File")
        for text, slot, shortcut in [
            ("&New (blank)", self.new_project, "Ctrl+N"),
            ("&Open…", self.open_project, "Ctrl+O"),
            ("&Save", self.save_project, "Ctrl+S"),
            ("Save &As…", self.save_project_as, "Ctrl+Shift+S"),
        ]:
            act = QAction(text, self)
            act.triggered.connect(slot)
            act.setShortcut(shortcut)
            file_menu.addAction(act)
        restore_act = QAction("Restore &autosave", self)
        restore_act.triggered.connect(self.restore_autosave)
        file_menu.addAction(restore_act)
        file_menu.addSeparator()
        # Export / CAD handoff, trimmed to the essentials: the older
        # "Push hardpoints" and "Sync design" Onshape variants are
        # superseded by "Build/update CAD" (idempotent points+planes+PDF).
        for text, slot in [
            ("&Export hardpoints CSV…", self.export_csv),
            ("&Copy hardpoints CSV (for Onshape)", self.copy_csv),
            ("Export &report CSV…", self.export_report),
            ("Export &impact loads for FEA (ANSYS/Onshape)…",
             self.export_impact_loads),
            ("Export load-case &envelope (friction-circle matrix)…",
             self.export_load_envelope),
            ("Export per-&part loads + arm bending (component FEA)…",
             self.export_body_loads),
            ("&Build/update CAD in Onshape (points + planes + PDF)…",
             self.build_cad_in_onshape),
            ("Open design &from Onshape document…", self.pull_design_from_onshape),
        ]:
            act = QAction(text, self)
            act.triggered.connect(slot)
            file_menu.addAction(act)

        # Panels menu: re-open closed docks, or hide the ones you don't
        # use. Each action tracks its dock's visibility automatically.
        panels_menu = self.menuBar().addMenu("&Panels")
        for dock in self._docks.values():
            panels_menu.addAction(dock.toggleViewAction())
        panels_menu.addSeparator()
        # Panel font size (v1.9): the hardpoint table is dense — a notch
        # smaller fits every row without shrinking the whole app.
        from PySide6.QtCore import QSettings
        self._ui_settings = QSettings("BajaSuspensionTool", "ui")
        size_menu = panels_menu.addMenu("Panel font size")
        from PySide6.QtGui import QActionGroup
        grp = QActionGroup(self)
        saved_pt = int(self._ui_settings.value("panel_pt2", 9))
        for pt in (7, 8, 9, 10, 11):
            act = QAction(f"{pt} pt", self)
            act.setCheckable(True)
            act.setChecked(pt == saved_pt)
            act.triggered.connect(
                lambda _=False, p=pt: self.set_panel_font(p))
            grp.addAction(act)
            size_menu.addAction(act)
        self.set_panel_font(saved_pt)

        help_menu = self.menuBar().addMenu("&Help")
        guide_act = QAction("&User guide", self)
        guide_act.setShortcut("F1")
        guide_act.triggered.connect(self.show_help)
        help_menu.addAction(guide_act)
        eq_act = QAction("&Equations", self)
        eq_act.setShortcut("Shift+F1")
        eq_act.triggered.connect(self.show_equations)
        help_menu.addAction(eq_act)
        bode_act = QAction("Reading the &Bode plot (PDF)…", self)
        bode_act.triggered.connect(self.open_bode_guide)
        help_menu.addAction(bode_act)

        bar = QToolBar("Main")
        self.addToolBar(bar)
        bar.addWidget(QLabel("Editing: "))
        self.axle_combo = QComboBox()
        self.axle_combo.addItems(["Front suspension", "Rear suspension"])
        self.axle_combo.currentTextChanged.connect(self._on_axle_combo)
        bar.addWidget(self.axle_combo)
        bar.addWidget(QLabel("   Display units: "))
        self.unit_combo = QComboBox()
        for key, u in UNITS.items():
            self.unit_combo.addItem(u.label, key)
        self.unit_combo.setCurrentText(self.unit.label)
        self.unit_combo.currentIndexChanged.connect(
            lambda _: self.set_units(self.unit_combo.currentData()))
        bar.addWidget(self.unit_combo)
        bar.addWidget(QLabel("   Coords: "))
        from ..axes import CONVENTIONS as _CONVS
        self.coords_combo = QComboBox()
        for key, c in _CONVS.items():
            self.coords_combo.addItem(c.name, key)
        self.coords_combo.setCurrentText(self.conv.name)
        self.coords_combo.setToolTip(
            "Which axis convention the table, CSV export and Onshape push "
            "use. Chassis = the team frame convention (fore-aft on Y, "
            "lateral on X). The math underneath never changes.")
        self.coords_combo.currentIndexChanged.connect(
            lambda _: self.set_convention(self.coords_combo.currentData()))
        bar.addWidget(self.coords_combo)
        bar.addWidget(QLabel("   Suspension type: "))
        from ..suspension_types import types_for_axle
        self.type_combo = QComboBox()
        # Seeded for the axle that is active at construction (the front), so
        # the rear-only types are absent from the very first paint rather
        # than only after the first axle switch. _sync_type_combo refills it
        # whenever the active axle changes.
        for name in types_for_axle(self.active_key):
            self.type_combo.addItem(name)
        self.type_combo.setToolTip(
            "Suspension type for the ACTIVE axle. The rear-only layouts "
            "(H-arm, loaded halfshaft) hold toe by design and have no "
            "steering input, so they are offered on the rear axle only.")
        self.type_combo.currentTextChanged.connect(self._on_type_changed)
        bar.addWidget(self.type_combo)
        self.dark_check = QCheckBox("  Dark theme")
        self.dark_check.setToolTip("Dark theme for the whole app "
                                   "(plots, panels and viewport)")
        self.dark_check.toggled.connect(self.set_dark)
        bar.addWidget(self.dark_check)
        undo_btn = QPushButton(" ↩ Undo ")
        undo_btn.setToolTip(f"Undo the last geometry change on the active "
                            f"axle (Ctrl+Z, up to {UNDO_DEPTH} steps)")
        undo_btn.clicked.connect(self.undo)
        bar.addWidget(undo_btn)
        undo_act = QAction("Undo", self)
        undo_act.setShortcut("Ctrl+Z")
        undo_act.triggered.connect(self.undo)
        self.addAction(undo_act)
        self._undoing = False

    # ------------------------------------------------------------------
    # Units
    # ------------------------------------------------------------------
    @property
    def travel_range(self) -> tuple[float, float]:
        return (-self.unit.to_mm(self.droop_spin.value()),
                self.unit.to_mm(self.bump_spin.value()))

    def _apply_range_unit(self) -> None:
        for spin in (self.droop_spin, self.bump_spin):
            spin.blockSignals(True)
            spin.setRange(0, self.unit.from_mm(800.0))
            spin.setDecimals(self.unit.decimals)
            spin.setSingleStep(self.unit.step)
            spin.setSuffix(f" {self.unit.label}")
            spin.blockSignals(False)
        self._sync_range_spins()

    def _sync_range_spins(self) -> None:
        axle = self.active
        auto = axle.shock_spec is not None
        for spin, v in ((self.droop_spin, axle.droop),
                        (self.bump_spin, axle.bump)):
            spin.blockSignals(True)
            spin.setValue(self.unit.from_mm(v))
            # With a shock spec the range FOLLOWS the hardware stroke
            # automatically — the spins become displays, not inputs.
            spin.setEnabled(not auto)
            spin.setToolTip(
                "Travel range follows the shock's hardware stroke "
                "(min/max in the Halfshafts & shocks panel)" if auto
                else "")
            spin.blockSignals(False)
        lo, hi = -axle.droop, axle.bump
        if self.motion_combo.currentText() == "Roll":
            # Roll drives the two sides to OPPOSITE travels, so the slider
            # can only go as far as BOTH directions allow — otherwise one
            # side runs out of droop and freezes while the other keeps
            # moving (the "uneven roll" report).
            m = min(axle.droop, axle.bump)
            lo, hi = -m, m
        self.slider.blockSignals(True)
        self.slider.setRange(int(np.ceil(lo)), int(np.floor(hi)))
        self.slider.setValue(int(round(axle.travel)))
        self.slider.blockSignals(False)
        # right wheel (Free mode) spans the axle's full range independently
        self.right_slider.blockSignals(True)
        self.right_slider.setRange(int(np.ceil(-axle.droop)),
                                   int(np.floor(axle.bump)))
        self.right_slider.setValue(int(round(axle.travel_right)))
        self.right_slider.blockSignals(False)

    def _apply_steer_unit(self) -> None:
        s = self.steer_limit_spin
        s.blockSignals(True)
        s.setRange(self.unit.from_mm(1.0), self.unit.from_mm(150.0))
        s.setDecimals(self.unit.decimals)
        s.setSingleStep(self.unit.step)
        s.setSuffix(f" {self.unit.label}")
        s.setValue(self.unit.from_mm(self.steer_limit))
        s.blockSignals(False)
        lim = int(np.floor(self.steer_limit))
        self.steer_slider.blockSignals(True)
        self.steer_slider.setRange(-lim, lim)
        self.steer_slider.setValue(int(round(self.steer)))
        self.steer_slider.blockSignals(False)
        self._update_steer_label()

    def set_units(self, key: str) -> None:
        new = UNITS[key]
        self.unit = new
        self._apply_range_unit()
        self._apply_steer_unit()
        for panel in (self.setup_form, self.optimize_panel,
                      self.hardpoint_table, self.readouts, self.plots,
                      self.tweaks_panel, self.vehicle_panel,
                      self.frame_panel, self.halfshaft_panel,
                      self.checklist, self.candidates_panel,
                      self.dynamics_panel):
            panel.set_units(new)
        if self.unit_combo.currentData() != key:
            self.unit_combo.setCurrentText(new.label)
        self._update_travel_label(self.active.travel)
        self._update_vehicle_readouts()

    def set_convention(self, key: str) -> None:
        """Switch the display/export axis convention (tool vs chassis
        frame). Presentation only: solvers and saved files stay in the
        tool frame."""
        from ..axes import CONVENTIONS
        self.conv = CONVENTIONS[key]
        self.hardpoint_table.set_convention(self.conv)
        self.halfshaft_panel.set_convention(self.conv)
        self.viewport.set_axis_convention(self.conv)
        if self.coords_combo.currentData() != key:
            idx = self.coords_combo.findData(key)
            if idx >= 0:
                self.coords_combo.setCurrentIndex(idx)
        self._update_vehicle_readouts()   # CG readout re-formats
        self.statusBar().showMessage(
            f"Coordinates shown/exported as: {self.conv.name}", 5000)

    # ------------------------------------------------------------------
    # Geometry lifecycle (all against the ACTIVE axle)
    # ------------------------------------------------------------------
    def set_vehicle_ride_height(self, target_mm: float) -> None:
        """Rigidly translate EVERY seeded axle in z until each reads
        `target_mm` ride height (ground -> frame datum, per-axle
        frame-tube datum honored). One knob for the whole car's stance —
        also the one-click fix for a red F/R stance row. No kinematics
        change; each axle's move lands in its own undo history."""
        from ..tweaks import ride_height, set_ride_height
        # ATOMIC lock check first: a locked point may NEVER be moved, and
        # a half-applied stance (front moved, rear refused) would be worse
        # than refusing outright — so neither axle moves if either is
        # pinned and needs to move.
        for key in ("front", "rear"):
            axle = self.axles[key]
            if axle.hp is None or not axle.locked:
                continue
            fto = (float(getattr(axle.last_setup, "frame_tube_offset",
                                 0.0) or 0.0)
                   if axle.last_setup is not None else 0.0)
            if abs(ride_height(axle.hp, fto) - target_mm) >= 0.01:
                self.statusBar().showMessage(
                    f"Vehicle ride height refused: the {key} axle has "
                    f"locked point(s) ({', '.join(sorted(axle.locked))}) "
                    "that a rigid stance move would drag along — unlock "
                    "them first (nothing ever moves a locked point)",
                    10000)
                return
        cur = self.active_key
        changed = []
        try:
            for key in ("front", "rear"):
                axle = self.axles[key]
                if axle.hp is None:
                    continue
                fto = (float(getattr(axle.last_setup, "frame_tube_offset",
                                     0.0) or 0.0)
                       if axle.last_setup is not None else 0.0)
                if abs(ride_height(axle.hp, fto) - target_mm) < 0.01:
                    continue
                self.active_key = key
                self.apply_hardpoints(
                    set_ride_height(axle.hp, target_mm, fto))
                changed.append(key)
        finally:
            self.active_key = cur
        if changed:
            self._resync_active_ui()
            self._update_roll_axis()
            self.statusBar().showMessage(
                f"Vehicle ride height set to "
                f"{self.unit.fmt(target_mm)} {self.unit.label} "
                f"({' + '.join(changed)} translated rigidly — no "
                "kinematics change)", 8000)
        else:
            self.statusBar().showMessage(
                "Vehicle ride height: both axles already there", 5000)

    def measure_setup_from_design(self) -> None:
        """Fill the Setup form FROM the active axle's live geometry, so a
        reseed reproduces the current design (flip one seed-only option,
        Generate, land where you were). Works on whatever is displayed —
        load a candidate first to measure that instead."""
        hp = self.active.hp
        if hp is None:
            self.statusBar().showMessage("Nothing to measure — no geometry "
                                         "on this axle yet", 6000)
            return
        if not isinstance(hp, DoubleWishbonePoints):
            self.statusBar().showMessage(
                "Measure-setup works on double-wishbone axles only", 6000)
            return
        from ..seed import setup_from_hardpoints
        sv = setup_from_hardpoints(hp, self.setup_form.current_setup())
        self.setup_form.load_setup(sv)
        self.statusBar().showMessage(
            "Setup form now matches the measured design — change what you "
            "need and Generate to reseed onto it. (One-sketch mode will "
            "still planarize caster to the kickup.)", 10000)

    def generate_seed(self, sv: SetupVariables) -> None:
        """Seed the active axle from the setup form.

        Honours the TYPE DROPDOWN. Until v1.32 this always ran the double
        wishbone seeder, so selecting (say) C-hub and then pressing
        "Generate seed hardpoints" silently threw the C-hub away and built
        a double wishbone — the type only survived if you never touched
        Generate again."""
        wanted = self.type_combo.currentText()
        from ..suspension_types import is_allowed_on
        if not is_allowed_on(wanted, self.active_key):
            # e.g. a rear-only type left selected while switching to front
            wanted = "Double wishbone"
            self._sync_type_combo()
        if wanted != "Double wishbone":
            self._seed_as_type(wanted, sv)
            return
        # The shock's hardware min/max are FIXED axle properties (set in
        # the Halfshafts & shocks panel) — when a spec exists it overrides
        # whatever the form fields say, so reseeding can never lose the
        # real stroke. First seed with no spec: the form values CREATE it.
        if self.active.shock_spec is not None:
            spec = self.active.shock_spec
            sv = dataclasses.replace(sv, shock_min_length=spec.min_length,
                                     shock_max_length=spec.max_length)
        try:
            hp, report = generate_seed(sv)
        except ValueError as e:
            self.statusBar().showMessage(f"Seed generation failed: {e}", 8000)
            return
        axle = self.active
        reseed_note = ""
        if axle.hp is not None and type(axle.hp) is type(hp):
            # Reseeding over an existing design: honor the pinned points —
            # the new seed is slid onto them and they are kept EXACTLY, so
            # welded tabs / fixed mounts survive the reseed.
            if axle.locked:
                from ..seed import apply_locked_points
                hp = apply_locked_points(
                    hp, axle.hp, axle.locked, sv.independent_caster,
                    sv.steering_arm_plane_deg)
                reseed_note = (f" — {len(axle.locked)} locked point(s) "
                               "honored")
            jump = float(np.linalg.norm(
                np.asarray(hp.wheel_center) - np.asarray(axle.hp.wheel_center)))
            if jump > 25.0:
                reseed_note += (f" — landed {jump:.0f} mm from the previous "
                                "design; use 'Measure setup from current "
                                "design' first (and lock fixed tabs) to "
                                "reseed in place")
        axle.last_setup = sv
        if axle.shock_spec is None:
            from ..shock import ShockSpec
            axle.shock_spec = ShockSpec(sv.shock_min_length,
                                        sv.shock_max_length)
            self.halfshaft_panel.load_shock_specs(
                self.axles["front"].shock_spec,
                self.axles["rear"].shock_spec)
        # Make the seed's wheelbase actually drive the LIVE vehicle param
        # (rear-axle station, CG split, CAD export). It was inert for a
        # single corner before — a longstanding "why is wheelbase locked?"
        # confusion. It stays freely editable afterward in the Vehicle panel.
        if getattr(sv, "wheelbase", 0):
            p = self.vehicle_panel.params()
            p.wheelbase = float(sv.wheelbase)
            self.vehicle_panel.load_params(p)
        self.tweaks_panel.set_context(sv)
        axle.baseline = None
        axle.ghost = None
        axle.droop = axle.req_droop = report.droop_travel
        axle.bump = axle.req_bump = report.bump_travel
        # Halfshaft is a SEED-TIME decision (you know whether this axle is
        # driven before you draw geometry): auto-place the inner CV joint
        # so the shaft runs level at MID-travel — the placement that
        # minimizes plunge over a bump-biased range (see examples/README).
        from ..halfshaft import HalfshaftConfig, default_inner_for
        if getattr(sv, "has_halfshaft", False):
            axle.halfshaft = HalfshaftConfig(
                enabled=True,
                inner=default_inner_for(hp, report.bump_travel,
                                        report.droop_travel))
        else:
            axle.halfshaft.enabled = False
        self.halfshaft_panel.load(self.axles["front"].halfshaft,
                                  self.axles["rear"].halfshaft)
        self.setup_form.load_setup(sv)
        self._sync_range_spins()
        self.apply_hardpoints(hp)
        # Re-place the inner CV now that the travel budget is FINAL. The
        # block above used the seed report's budget, but apply_hardpoints
        # then replaces it with the shock hardware's real limits -- so the
        # shaft was left a few mm off level at mid-travel, which is the one
        # thing this placement exists to guarantee.
        if (axle.halfshaft is not None and axle.halfshaft.enabled
                and axle.hp is not None):
            axle.halfshaft = HalfshaftConfig(
                enabled=True,
                inner=default_inner_for(axle.hp, axle.bump, axle.droop))
            self.halfshaft_panel.load(self.axles["front"].halfshaft,
                                      self.axles["rear"].halfshaft)
            # apply_hardpoints already refreshed the table against the OLD
            # config, so the CV rows would otherwise show a stale z -- and
            # editing one reads all three columns back off the table, which
            # would silently reinstate the stale value.
            self.hardpoint_table.refresh(axle.hp)
        self.statusBar().showMessage(
            f"{axle.name} seed: MR {report.motion_ratio_static:.2f}, "
            f"caster {report.caster_static:.1f} deg, travel "
            f"+{self.unit.fmt(report.bump_travel)}/"
            f"-{self.unit.fmt(report.droop_travel)} {self.unit.label}"
            f"{reseed_note}", 12000)

    @staticmethod
    def _probe_reachable(solver, want: float, sign: float) -> float:
        """Largest |travel| the linkage can SWEEP toward `sign`, up to
        `want`. Probes with walk_travels — the same warm-started
        machinery the sweeps use — so the clamped range is exactly what
        the sweep can deliver (cold solve() brackets on a bounded angle
        grid and gives up earlier than a warm-started walk can reach).
        Binary search, backed off 1% for margin."""
        def ok(end: float) -> bool:
            try:
                solver.walk_travels(np.linspace(0.0, sign * end, 25))
                return True
            except ValueError:
                return False
        if ok(want):
            return want
        lo, hi = 0.0, want          # lo = feasible, hi = infeasible
        for _ in range(12):
            mid = 0.5 * (lo + hi)
            if ok(mid):
                lo = mid
            else:
                hi = mid
        return max(0.99 * lo, 0.0)

    def apply_hardpoints(self, hp) -> None:
        """Adopt a new hardpoint set for the active axle.

        Workflow rule (v1.5): an edit that shrinks the reachable travel
        does NOT get rejected — the swept range auto-clamps to what the
        geometry can do and the Design Checklist flags the shortfall.
        Only geometry that barely articulates at all is refused."""
        axle = self.active
        # Knuckle-consistency convention (always on for double wishbones):
        # keep the tire spin axis in the ball-joint/wheel-centre plane and
        # the steering arm at its set angle to it. Idempotent, so re-adopting
        # already-consistent geometry is a no-op.
        if isinstance(hp, DoubleWishbonePoints):
            from ..geometry import enforce_knuckle_planes
            hp = enforce_knuckle_planes(
                hp, float(getattr(axle.last_setup, "steering_arm_plane_deg",
                                  90.0) or 90.0))
        from ..suspension_types import solver_for
        try:
            solver = solver_for(hp)(hp)
        except ValueError as e:
            self.statusBar().showMessage(
                f"Rejected: geometry does not close at ride height ({e})",
                8000)
            # The table decides what to do with a refusal: live mode reverts
            # as it always has, batch mode keeps every typed cell. Either
            # way the REASON now lands in the panel instead of only in a
            # status bar that times out in the far corner of the window.
            self.hardpoint_table.edit_rejected(str(e))
            if axle.hp is not None:
                self.tweaks_panel.refresh(axle.hp)
            return
        # With a shock spec, the travel range IS the shock's range: probe
        # the exact travels where the shock reaches its hardware min/max
        # (or the linkage locks first) and use them — automatically, both
        # expanding AND shrinking. The shock must sweep its whole stroke;
        # CV angle / plunge issues over that range stay WARNINGS (the
        # Halfshafts panel + checklist), never travel caps.
        if axle.shock_spec is not None:
            from ..shock import shock_travel_limits
            lim = shock_travel_limits(solver, axle.shock_spec)
            old_b, old_d = axle.req_bump, axle.req_droop
            axle.req_bump, axle.req_droop = lim["bump"], lim["droop"]
            if (abs(lim["bump"] - old_b) > 1.0
                    or abs(lim["droop"] - old_d) > 1.0):
                why = {"shock": "shock stroke", "linkage": "linkage lock",
                       "cap": "probe cap"}
                self.statusBar().showMessage(
                    f"Travel follows the shock: "
                    f"+{self.unit.fmt(lim['bump'])} "
                    f"({why[lim['bump_stop']]}) / "
                    f"-{self.unit.fmt(lim['droop'])} "
                    f"({why[lim['droop_stop']]}) {self.unit.label}", 8000)
        lo, hi = -axle.req_droop, axle.req_bump
        try:
            sweep = sweep_metrics(solver, np.linspace(lo, hi, SWEEP_POINTS))
            axle.droop, axle.bump = axle.req_droop, axle.req_bump
        except ValueError:
            bump = self._probe_reachable(solver, axle.req_bump, +1.0)
            droop = self._probe_reachable(solver, axle.req_droop, -1.0)
            if bump < 15.0 and droop < 15.0:
                self.statusBar().showMessage(
                    "Rejected: linkage can barely articulate "
                    f"(+{bump:.0f}/-{droop:.0f} mm)", 8000)
                if axle.hp is not None:
                    self.hardpoint_table.refresh(axle.hp)
                    self.tweaks_panel.refresh(axle.hp)
                return
            lo, hi = -droop, bump
            try:
                sweep = sweep_metrics(solver,
                                      np.linspace(lo, hi, SWEEP_POINTS))
            except ValueError as e:   # pathological edge: refuse after all
                self.statusBar().showMessage(f"Rejected: {e}", 8000)
                if axle.hp is not None:
                    self.hardpoint_table.refresh(axle.hp)
                return
            axle.droop, axle.bump = droop, bump
            self.statusBar().showMessage(
                f"Travel auto-clamped to +{self.unit.fmt(bump)}/"
                f"-{self.unit.fmt(droop)} {self.unit.label} (requested "
                f"+{self.unit.fmt(axle.req_bump)}/"
                f"-{self.unit.fmt(axle.req_droop)}) — see the Checklist",
                9000)
        if axle.hp is not None and not self._undoing and axle.hp is not hp:
            axle.undo.append(axle.hp)
            del axle.undo[:-UNDO_DEPTH]
        axle.hp, axle.solver, axle.sweep = hp, solver, sweep
        self._augment_anti(axle)
        self.hardpoint_table.refresh(hp)
        self.tweaks_panel.set_context(axle.last_setup)
        self.tweaks_panel.refresh(hp)
        self.optimize_panel.set_free_groups_for(hp)
        self.plots.set_sweep(sweep["travel_mm"], sweep, baseline=axle.baseline)
        self._sync_range_spins()
        self.rebuild_scene()
        self.show_travel(float(np.clip(axle.travel, lo, hi)))
        self._push_locks(axle)
        self._push_vehicle_frame()
        self.viewport.set_drag_convention(self.conv)
        self._update_checklist()
        self._autosave_soon()

    def _axle_track(self, axle: Axle) -> float:
        """Track of the car ACTUALLY being modelled, from the geometry.

        It used to prefer `last_setup.track_width`, which is the value
        that was ASKED for when the axle was seeded. Any hardpoint edit
        moves the real track away from it, and a batch entry of CAD
        coordinates can leave the two inches apart — 63 in recorded
        against 55 in built, on the file that turned this up. Roll angle
        is derived from this (phi = atan(2*travel/track)), so a stale
        value quietly biases every roll number by the same ratio.
        """
        if axle.hp is not None:
            return 2.0 * abs(float(axle.hp.wheel_center[1]))
        if axle.last_setup is not None:
            return float(axle.last_setup.track_width)
        return 0.0

    def track_mismatch(self, axle: Axle) -> float | None:
        """How far the seeded track intent is from the built geometry, in
        mm, or None when there is nothing to compare."""
        if axle.hp is None or axle.last_setup is None:
            return None
        return float(axle.last_setup.track_width) - self._axle_track(axle)

    def _apply_readouts(self, axle: "Axle", state, travel: float) -> None:
        """Single source of truth for the Readouts panel: live corner
        metrics at `state`, the four sweep-derived channels sampled at
        `travel`, and the whole-car context rows (track / wheelbase / ride
        height). EVERY motion mode routes through here so no code path can
        silently drop a row — the old Roll path did exactly that, blanking
        track/wheelbase/ride-height whenever you rolled."""
        values = corner_metrics(axle.solver, state)
        for key in ("motion_ratio", "bump_steer_deg_per_mm",
                    "half_track_change_mm", "wheel_recession_mm"):
            values[key] = float(np.interp(
                travel, axle.sweep["travel_mm"], axle.sweep[key]))
        values.update(self._context_readouts(axle, travel, state))
        self.readouts.update_values(values)

    def show_travel(self, travel: float) -> None:
        """Re-solve at one travel position. In Heave mode the slider moves
        the ACTIVE axle's wheels together; in Roll mode it is the LEFT
        wheel's travel — the right side goes the other way and the other
        axle rolls to the same chassis angle."""
        axle = self.active
        if axle.solver is None:
            return
        mode = self.motion_combo.currentText()
        if mode == "Roll":
            self._show_roll(travel)
            return
        if mode == "Free (L/R)":
            self._show_free(travel)
            return
        try:
            state = self._solve_display(axle, travel)
        except ValueError:
            return  # edge of articulation: hold the last good pose
        axle.travel = travel
        self._push_axle_visual(axle, state)
        if self.both_axles_check.isChecked():
            other = self.axles["rear" if self.active_key == "front"
                               else "front"]
            if other.solver is not None:
                t2 = float(np.clip(travel, -other.droop, other.bump))
                try:
                    st2 = self._solve_display(other, t2)
                except ValueError:
                    st2 = None
                if st2 is not None:
                    other.travel = t2
                    self._push_axle_visual(other, st2)
        self._apply_readouts(axle, state, travel)
        self.plots.set_marker(travel)
        self.viewport.set_live_ground(travel)
        self._update_travel_label(travel)
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(travel)))
        self.slider.blockSignals(False)
        self._update_roll_axis()

    def _show_roll(self, travel: float) -> None:
        """Roll pose: active axle's left wheel at +travel, right at -travel;
        the other axle rolls to the SAME chassis angle (its travel scales
        with its track). RC markers use the asymmetric construction."""
        from ..sweeps import asymmetric_roll_center
        active = self.active
        phi = np.arctan2(2.0 * travel, self._axle_track(active))
        for key, axle in self.axles.items():
            if axle.solver is None:
                continue
            t_axle = (travel if key == self.active_key
                      else np.tan(phi) * self._axle_track(axle) / 2.0)
            try:
                left = axle.solver.solve(t_axle)
                right = axle.solver.solve(-t_axle)
            except ValueError:
                continue
            axle.travel = t_axle if key == self.active_key else axle.travel
            info = self._rc_info(axle, left)
            rc_y, rc_z = asymmetric_roll_center(axle.solver, left, right)
            info["rc_h"] = rc_z          # absolute z for the 3D marker
            info["rc_y"] = rc_y
            # metric height is measured from the STATIC ground plane
            axle.rc_h = rc_z - (float(axle.hp.wheel_center[2])
                                - axle.hp.tire_radius)
            self.viewport.set_state(key, left, info, state_right=right)
        active_state = active.state
        if active_state is not None:
            self._apply_readouts(active, active_state, travel)
        self.plots.set_marker(travel)
        self._update_travel_label(travel)
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(travel)))
        self.slider.blockSignals(False)
        self.viewport.set_live_ground(0.0)   # ground stays put in pure roll
        self._update_roll_axis()

    def _show_free(self, travel: float) -> None:
        """Asymmetric per-wheel pose on the active axle: the LEFT wheel sits
        at the main slider's travel, the RIGHT wheel at its own slider's
        travel. The roll centre uses the asymmetric construction from the
        two wheel states — the single-wheel-bump roll-centre check. The
        other axle keeps whatever pose it was left in."""
        from ..sweeps import asymmetric_roll_center
        axle = self.active
        axle.travel = travel
        tr = axle.travel_right
        steer = self.steer if axle.name == "front" else 0.0
        steer_r = -steer if axle.name == "front" else 0.0
        try:
            left = axle.solver.solve(travel, steer)
            right = axle.solver.solve(tr, steer_r)
        except ValueError:
            return
        info = self._rc_info(axle, left)
        rc_y, rc_z = asymmetric_roll_center(axle.solver, left, right)
        info["rc_h"] = rc_z          # absolute z for the 3D marker
        info["rc_y"] = rc_y
        axle.rc_h = rc_z - (float(axle.hp.wheel_center[2])
                            - axle.hp.tire_radius)
        self.viewport.set_state(axle.name, left, info, state_right=right)
        self._apply_readouts(axle, left, travel)
        self.plots.set_marker(travel)
        self.viewport.set_live_ground(travel)
        self._update_travel_label(travel)
        self._update_right_travel_label(tr)
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(travel)))
        self.slider.blockSignals(False)
        self._update_roll_axis()

    def show_travel_right(self, travel_right: float) -> None:
        """Right-wheel slider (Free mode): pose the right wheel on its own."""
        axle = self.active
        if axle.solver is None:
            return
        axle.travel_right = float(np.clip(travel_right, -axle.droop, axle.bump))
        if self.motion_combo.currentText() == "Free (L/R)":
            self._show_free(axle.travel)

    def _update_right_travel_label(self, travel_mm: float) -> None:
        self.right_travel_label.setText(
            f"{self.unit.from_mm(travel_mm):+.{self.unit.decimals}f} "
            f"{self.unit.label}")

    def _on_motion_mode(self, mode: str) -> None:
        free = mode == "Free (L/R)"
        if mode in ("Roll", "Free (L/R)") and not self.mirror_check.isChecked():
            self.mirror_check.setChecked(True)   # need both sides to see it
        for w in (self.right_label_lo, self.right_slider,
                  self.right_label_hi, self.right_travel_label):
            w.setVisible(free)
        axle = self.active
        if mode == "Roll":
            m = min(axle.droop, axle.bump)
            axle.travel = float(np.clip(axle.travel, -m, m))
        self._sync_range_spins()                 # re-clamps the slider range
        self.show_travel(axle.travel)

    def go_ride_height(self) -> None:
        """Back to static: travel -> 0 on BOTH axles (the old
        active-axle-only reset left the other axle posed wherever its
        slider last was — wheels then sat at visibly different heights in
        3D while the readouts looked fine)."""
        for a in self.axles.values():
            a.travel = 0.0
            a.travel_right = 0.0
        self.right_slider.blockSignals(True)
        self.right_slider.setValue(0)
        self.right_slider.blockSignals(False)
        self._update_right_travel_label(0.0)
        if self.motion_combo.currentText() == "Roll":
            self.show_travel(0.0)
        else:
            self.slider.setValue(0)
            self.show_travel(0.0)
        self.rebuild_scene()          # re-pose the OTHER axle at 0 too

    def reset_pose(self) -> None:
        """Full static pose: ride height AND centred steering."""
        self.steer_slider.setValue(0)
        self.go_ride_height()

    def _solve_display(self, axle: Axle, travel: float):
        """Solve for DISPLAY: the front axle sees the steering input."""
        steer = self.steer if axle.name == "front" else 0.0
        return axle.solver.solve(travel, steer)

    def _solve_display_right(self, axle: Axle, travel: float):
        """The RIGHT wheel's pose when steering. The display mirrors y -> -y,
        so a corner solved with the OPPOSITE rack sign and then mirrored
        shows the right wheel swinging the same physical direction as the
        left (both wheels point into the turn), instead of a mirror image.
        Returns None when there is no steer (plain mirror is then exact)."""
        if axle.name != "front" or abs(self.steer) < 1e-9:
            return None
        try:
            return axle.solver.solve(travel, -self.steer)
        except ValueError:
            return None

    def _rc_info(self, axle: Axle, state) -> dict:
        # axle.rc_h is the METRIC (height above the ground plane); the 3D
        # marker needs the absolute z, so add the contact-patch z back
        # (the Onshape-aligned offset puts the ground below z = 0).
        rc_h = corner_metrics(axle.solver, state)["roll_center_height_mm"]
        info = {
            "ic": front_view_ic(axle.solver, state),
            "svic": side_view_ic(axle.solver, state),
            "rc_h": rc_h + float(state.contact_patch[2]),
        }
        axle.rc_h = rc_h
        axle.state = state
        return info

    def _push_axle_visual(self, axle: Axle, state) -> None:
        self.viewport.set_state(axle.name, state, self._rc_info(axle, state),
                                state_right=self._solve_display_right(
                                    axle, axle.travel))

    def set_steer(self, steer: float) -> None:
        """Steering slider: re-pose the FRONT axle at its current travel."""
        self.steer = float(np.clip(steer, -self.steer_limit, self.steer_limit))
        self._update_steer_label()
        front = self.axles["front"]
        if front.solver is None:
            return
        try:
            state = front.solver.solve(front.travel, self.steer)
        except ValueError:
            return
        self._push_axle_visual(front, state)
        if self.active_key == "front":
            self._apply_readouts(front, state, front.travel)
        self._update_roll_axis()

    def _on_steer_limit(self) -> None:
        self.steer_limit = self.unit.to_mm(self.steer_limit_spin.value())
        self.steer = float(np.clip(self.steer, -self.steer_limit,
                                   self.steer_limit))
        self._apply_steer_unit()
        self.set_steer(self.steer)

    def _update_steer_label(self) -> None:
        self.steer_label.setText(
            f"{self.unit.from_mm(self.steer):+.{self.unit.decimals}f} "
            f"{self.unit.label}")

    # ------------------------------------------------------------------
    # Scene / vehicle-level display
    # ------------------------------------------------------------------
    def rebuild_scene(self) -> None:
        """Recreate the 3D scene from every displayed axle."""
        corners = {}
        shown = {"front": self.show_front_check.isChecked(),
                 "rear": self.show_rear_check.isChecked()}
        mirror = self.mirror_check.isChecked()
        for key, axle in self.axles.items():
            if axle.solver is None or not shown[key]:
                continue
            try:
                state = self._solve_display(axle, axle.travel)
            except ValueError:
                state = axle.solver.solve(0.0)
            from .viewport import drag_attrs_for
            locked_idx = ([i for i, a in enumerate(drag_attrs_for(axle.hp))
                           if a in axle.locked] if axle.locked else None)
            corners[key] = {"hp": axle.hp, "state": state, "mirror": mirror,
                            "rc_info": self._rc_info(axle, state),
                            "state_right": self._solve_display_right(
                                axle, axle.travel),
                            "locked_idx": locked_idx,
                            # A structural shaft (loaded halfshaft) is
                            # drawn by the type's own layout — no overlay.
                            "halfshaft_inner": (
                                axle.halfshaft.inner
                                if axle.halfshaft is not None
                                and axle.halfshaft.enabled
                                and not hasattr(axle.hp, "hs_inner")
                                else None)}
        ghost = None
        if self.active.ghost is not None:
            ghp, gst = self.active.ghost
            ghost = (self.active_key, ghp, gst)
        self.viewport.rebuild(
            corners, self.vehicle_panel.params().wheelbase, ghost=ghost)
        self.viewport.set_live_ground(self.active.travel)
        self._update_roll_axis()

    def _ground_z(self, axle: Axle) -> float:
        """Absolute z of the static ground plane under this axle (the
        Onshape-aligned offset puts it below z = 0)."""
        if axle.hp is None:
            return 0.0
        return float(axle.hp.wheel_center[2]) - axle.hp.tire_radius

    def _front_station_x(self) -> float:
        """Scene x of the FRONT axle: the hardpoints carry the Onshape
        coordinate offset, so the axle is NOT at x = 0 — it is at its
        wheel-centre x. (The rear axle's wheel-centre x reads the same in
        its own local frame, and the scene shifts it back by the
        wheelbase, so this value anchors both.)"""
        for axle in (self.axles["front"], self.axles["rear"]):
            if axle.hp is not None:
                return float(axle.hp.wheel_center[0])
        return 0.0

    def _cg_point(self) -> np.ndarray | None:
        """Absolute (x, y, z) of the CG in model coordinates: cg_height is
        entered above ground, cg_behind_front behind the front axle."""
        anchor = (self.axles["front"] if self.axles["front"].hp is not None
                  else self.axles["rear"])
        if anchor.hp is None:
            return None
        params = self.vehicle_panel.params()
        return np.array([self._front_station_x() - params.cg_behind_front,
                         0.0,
                         params.cg_height + self._ground_z(anchor)])

    def _update_roll_axis(self) -> None:
        """Roll axis + CG-derived numbers, live at the current travels."""
        f, r = self.axles["front"], self.axles["rear"]
        params = self.vehicle_panel.params()
        cg = self._cg_point()
        self.viewport.set_cg(cg)               # 3D marker: tool frame
        # panel readout: shown in the active display convention
        cg_disp = None if cg is None else self.conv.to_display(cg)
        have_both = (f.solver is not None and r.solver is not None
                     and np.isfinite(f.rc_h) and np.isfinite(r.rc_h))
        # Front/rear STANCE mismatch: the two axles' static ground planes
        # should coincide (one car, one ground). When they don't, every
        # cross-axle number needs a common reference — and the designer
        # needs to see the mismatch instead of a silently floating tire.
        stance = None
        if f.hp is not None and r.hp is not None:
            stance = self._ground_z(f) - self._ground_z(r)
        # keep the vehicle ride-height spin tracking the FRONT axle's
        # measured value (the canonical reference plane)
        anchor = f if f.hp is not None else r
        if anchor.hp is not None:
            from ..tweaks import ride_height as _rh_meas
            fto = (float(getattr(anchor.last_setup, "frame_tube_offset",
                                 0.0) or 0.0)
                   if anchor.last_setup is not None else 0.0)
            self.vehicle_panel.update_ride_height_display(
                _rh_meas(anchor.hp, fto))
        # CG plausibility, judged against THIS car's geometry (v1.36).
        if anchor.hp is not None:
            self.vehicle_panel.show_cg_warnings(
                self.vehicle_panel.check_cg_plausible(
                    wheelbase_mm=params.wheelbase,
                    track_mm=2.0 * abs(float(anchor.hp.wheel_center[1])),
                    tire_radius_mm=float(anchor.hp.tire_radius)))
        if have_both:
            # rc_h values are heights above EACH AXLE'S OWN ground; the 3D
            # axis needs absolute z, so lift each end by its ground plane —
            # and shift both ends onto the true axle stations (the
            # Onshape offset moves them off x = 0 / -wheelbase).
            pf, pr = roll_axis_points(f.rc_h + self._ground_z(f),
                                      r.rc_h + self._ground_z(r),
                                      params.wheelbase)
            pf[0] += self._front_station_x()
            pr[0] += self._front_station_x()
            self.viewport.set_roll_axis(pf, pr)
            anti = None
            if f.state is not None and r.state is not None:
                anti = anti_geometry((f.solver, f.state),
                                     (r.solver, r.state), params)
            # Cross-axle metrics (axis angle, height at CG, moment arm)
            # reference BOTH RC heights to the FRONT axle's ground plane —
            # the same plane the CG height is anchored to — so the panel's
            # numbers always agree with the absolute 3D axis even when the
            # stance mismatches. (Equal grounds: correction is zero.)
            rc_r_common = r.rc_h + (self._ground_z(r) - self._ground_z(f))
            self.vehicle_panel.show_derived(
                f.rc_h, r.rc_h,
                roll_axis_metrics(f.rc_h, rc_r_common, params),
                anti=anti, cg=cg_disp, stance=stance)
            self._push_dynamics_link(anti, rc_rear_common=rc_r_common)
        else:
            self.viewport.set_roll_axis(None, None)
            self.vehicle_panel.show_derived(
                f.rc_h if np.isfinite(f.rc_h) else None,
                r.rc_h if np.isfinite(r.rc_h) else None, None, cg=cg_disp,
                stance=stance)
            self._push_dynamics_link(None)

    def _push_dynamics_link(self, anti: dict | None,
                            rc_rear_common: float | None = None) -> None:
        """Feed the dynamics panel the live kinematic values (in the
        spreadsheet's inches; motion ratio flipped to spring/wheel).
        rc_rear_common: rear RC referenced to the FRONT ground plane so
        the spreadsheet's single-ground model stays consistent when the
        two axles' stances mismatch."""
        f, r = self.axles["front"], self.axles["rear"]
        params = self.vehicle_panel.params()

        def _mr(axle: Axle):
            """Live motion ratio at ride height. Since v1.28 the kinematic
            side reports SHOCK/WHEEL — the same convention the dynamics
            layer wants — so this passes straight through. The old 1/MR
            bridge, and the whole class of bug it invited, is gone."""
            if axle.sweep is None:
                return None
            mr = float(np.interp(0.0, axle.sweep["travel_mm"],
                                 axle.sweep["motion_ratio"]))
            return mr if np.isfinite(mr) and abs(mr) > 1e-9 else None

        rc_rear = rc_rear_common if rc_rear_common is not None else r.rc_h
        def _track_in(axle):
            """None, not zero, when an axle has no geometry yet — the
            dynamics sheet divides by track and a pushed 0 is a crash."""
            if axle.hp is None:
                return None
            v = self._axle_track(axle)
            return v / IN if v > 0.0 else None

        def _loaded_r(axle):
            """Wheel-centre height above the contact patch, in inches."""
            if axle.hp is None or axle.state is None:
                return None
            r = float(axle.state.wheel_center[2]
                      - axle.state.contact_patch[2])
            return r / IN if r > 0.0 else None

        self.dynamics_panel.set_kinematic({
            "rc_front_in": f.rc_h / IN if np.isfinite(f.rc_h) else None,
            "rc_rear_in": rc_rear / IN if np.isfinite(rc_rear) else None,
            "cg_height_in": params.cg_height / IN,
            "wheelbase_in": params.wheelbase / IN,
            "mr_spring_front": _mr(f),
            "mr_spring_rear": _mr(r),
            # Track and loaded radius were typed by hand while the model
            # already knew both, so they went stale exactly the way the
            # roll-angle track did. Loaded radius comes from the contact
            # patch, which is where every other ground-referenced metric
            # is measured; the kinematics carry no tire squash, so this is
            # the free radius and reads a percent or two optimistic.
            "track_front_in": _track_in(self.axles["front"]),
            "track_rear_in": _track_in(self.axles["rear"]),
            "loaded_radius_front_in": _loaded_r(self.axles["front"]),
            "loaded_radius_rear_in": _loaded_r(self.axles["rear"]),
            "anti": anti,
        })

    def _update_vehicle_readouts(self) -> None:
        self._update_roll_axis()

    def _on_vehicle_changed(self) -> None:
        # wheelbase moves the rear axle's displayed station, and the CG /
        # brake-split inputs change the anti-percentage curves
        for axle in self.axles.values():
            self._augment_anti(axle)
        active = self.active
        if active.sweep is not None:
            self.plots.set_sweep(active.sweep["travel_mm"], active.sweep,
                                 baseline=active.baseline)
        self._push_vehicle_frame()   # wheelbase changed the rear datum
        self.rebuild_scene()
        self._update_checklist()

    def _context_readouts(self, axle: Axle, travel: float, state=None) -> dict:
        """Whole-car readouts that track the DISPLAYED geometry, not the
        setup inputs: track width measured live between the tire contact
        patches (updates as the wheels move), the vehicle wheelbase shown
        in the scene, and ride height at this travel."""
        out = {}
        st = state if state is not None else axle.state
        if st is not None:
            # left corner at +y, mirrored to -y: track = 2 * |contact y|,
            # measured at the contact patch (the real track datum) so it
            # breathes with half-track change through travel
            out["track_width_mm"] = 2.0 * abs(float(st.contact_patch[1]))
            # OVERALL WIDTH across the outsides of the tires. Not a
            # synonym for track, and the distinction has bitten once
            # already: a CAD model measured edge-to-edge reads one whole
            # tire width MORE than its track, so a 55 in track car
            # measures 62 in across the tires and the two numbers look
            # like a 7 in error in something. Camber tilts the tread band
            # and is ignored here (worth ~0.35 in at 0.9 deg on a 23 in
            # tire), because this is the nominal dimension CAD quotes.
            out["overall_width_mm"] = 2.0 * (
                abs(float(st.wheel_center[1])) + 0.5 * float(axle.hp.tire_width))
        # Wheelbase is LIVE: the static value plus each axle's wheel
        # recession at its displayed pose (wheel centres move fore/aft
        # through travel), measured along the chassis-forward axis.
        wb = float(self.vehicle_panel.params().wheelbase)
        for key, sign in (("front", +1.0), ("rear", -1.0)):
            other = self.axles[key]
            if other.state is not None and other.hp is not None:
                wb += sign * float(other.state.wheel_center[0]
                                   - other.hp.wheel_center[0])
        out["wheelbase_mm"] = wb
        # Ride height is MEASURED from the displayed geometry (ground
        # plane under the flat contact patch up to the frame datum) — not
        # the setup form's stored number, which goes stale the moment a
        # tweak or drag moves the corner in z. Same convention as the
        # ride-height tweak, so the readout, the tweak row and the 3D
        # view can never disagree.
        fto = float(getattr(axle.last_setup, "frame_tube_offset", 0.0)
                    or 0.0) if axle.last_setup is not None else 0.0
        if st is not None and axle.hp is not None:
            out["ride_height_mm"] = (-(float(st.wheel_center[2])
                                       - float(axle.hp.tire_radius)) - fto)
        elif axle.hp is not None:
            from ..tweaks import ride_height as _rh
            out["ride_height_mm"] = _rh(axle.hp, fto) - travel
        # Shock hardware rows: how the installed length splits the stroke
        # at ride, and how much of the stroke the CURRENT travel range
        # actually sweeps (94% on a real rear was the complaint).
        spec = axle.shock_spec
        if spec is not None and axle.hp is not None and spec.stroke > 1e-9:
            from ..tweaks import shock_travel_split
            sp = shock_travel_split(axle.hp, spec.min_length,
                                    spec.max_length)
            if np.isfinite(sp["bump_frac"]):
                out["shock_split_txt"] = (f"{100 * sp['bump_frac']:.0f}% / "
                                          f"{100 * sp['droop_frac']:.0f}%")
            if axle.sweep is not None and "shock_length_mm" in axle.sweep:
                length = axle.sweep["shock_length_mm"]
                used = float(np.max(length) - np.min(length))
                u = self.unit
                out["shock_use_txt"] = (
                    f"{100 * used / spec.stroke:.0f}% "
                    f"({u.fmt(used)}/{u.fmt(spec.stroke)} {u.label})")
        return out

    def _push_vehicle_frame(self) -> None:
        """Put the coordinate-entry surfaces (hardpoint table, halfshaft
        panel) into the SHARED VEHICLE FRAME: the rear axle reads/enters
        relative to the FRONT (firewall) origin — shifted by -wheelbase
        fore-aft — so both axles share one datum and match the CSV /
        Onshape export and the 3D scene. Storage stays axle-local."""
        wb = float(self.vehicle_panel.params().wheelbase)
        self.hardpoint_table.set_vehicle_dx(
            0.0 if self.active_key == "front" else -wb)
        self.halfshaft_panel.set_vehicle_dx(0.0, -wb)

    def _push_locks(self, axle: Axle) -> None:
        """Propagate an axle's pinned hardpoints to the table (read-only +
        🔒), the tweaks panel (scrub/caster refuse to move them) and the
        3D drag list (locked points can't be grabbed)."""
        self.hardpoint_table.set_locked(axle.locked)
        self.tweaks_panel.set_locked(axle.locked)
        if axle.hp is not None:
            from .viewport import drag_attrs_for
            # attrs is INDEX-ALIGNED with the displayed spheres — a locked
            # point must become None (undraggable), NEVER be removed, or
            # every later sphere grabs the wrong hardpoint.
            attrs = [None if a in axle.locked else a
                     for a in drag_attrs_for(axle.hp)]
            self.viewport.set_drag_context(axle.name, attrs)

    def _on_locks_changed(self, locked) -> None:
        """A hardpoint was pinned/unpinned in the table (double-click)."""
        self.active.locked = frozenset(locked)
        self._push_locks(self.active)
        self.rebuild_scene()          # re-color the pinned spheres (amber)
        self._autosave_soon()

    def _update_travel_label(self, travel_mm: float) -> None:
        self.travel_label.setText(
            f"{self.unit.from_mm(travel_mm):+.{self.unit.decimals}f} "
            f"{self.unit.label}")

    # ------------------------------------------------------------------
    # Optimization (active axle)
    # ------------------------------------------------------------------
    def design_yaw_of(self, axle) -> float:
        """The axle's DESIGN sketch-plane yaw (deg): squareness is judged
        against this, zero for a classic front-parallel sketch."""
        return float(getattr(axle.last_setup, "sketch_yaw_deg", 0.0)
                     or 0.0)

    def resquare_arms(self) -> None:
        """One-sketch repair on demand: rotate the DW bushing axes onto
        their mean direction AT THE DESIGN SKETCH YAW (kickup kept)
        without touching goals or ball joints. Reads the yaw from the
        setup form so 'set the angle, hit Re-square' twists existing
        geometry onto an angled frame."""
        import dataclasses as _dc
        from ..geometry import (DoubleWishbonePoints, planarize_kingpin,
                                sketch_planarity, square_bushing_axes)
        from ..metrics import caster_deg
        axle = self.active
        yaw = float(self.setup_form.current_setup().sketch_yaw_deg)
        if not isinstance(axle.hp, DoubleWishbonePoints):
            # Single-arm carrier types (C-hub, loaded halfshaft, H-arm) have
            # ONE bushing axis and a carrier pin, and those two are meant to
            # be parallel -- normal to the same 2D sketch. Squaring them is
            # the same repair, so Re-square works here too now (v1.34); it
            # used to refuse outright, which left no way to fix a corner
            # after the kickup tweak had skewed it.
            from ..geometry import carrier_planarity, square_carrier_axes
            try:
                b = carrier_planarity(axle.hp)
            except (ValueError, AttributeError):
                self.statusBar().showMessage(
                    "Re-square needs a bushing axis and a carrier pin — "
                    "this type has neither", 6000)
                return
            if (b["axis_misalign_deg"] <= 1e-9
                    and abs(b["axis_yaw_deg"] - yaw) <= 1e-9):
                self.statusBar().showMessage(
                    f"Arm and carrier pin already square at {yaw:.1f} deg "
                    "yaw — nothing to fix", 6000)
                return
            fixed = square_carrier_axes(axle.hp, yaw_deg=yaw)
            self.apply_hardpoints(fixed)
            a = carrier_planarity(self.active.hp)
            self.statusBar().showMessage(
                f"Re-squared to {yaw:.1f} deg yaw: arm/pin misalign "
                f"{b['axis_misalign_deg']:.3f}->{a['axis_misalign_deg']:.2g} "
                f"deg, kickup {a['sketch_kickup_deg']:.1f} deg kept "
                "(caster pill preserved; Ctrl+Z undoes)", 12000)
            return
        if axle.last_setup is not None:
            axle.last_setup = _dc.replace(axle.last_setup,
                                          sketch_yaw_deg=yaw)
        else:
            axle.last_setup = self.setup_form.current_setup()
        indep = bool(getattr(axle.last_setup, "independent_caster", False))
        b = sketch_planarity(axle.hp)
        square_ok = (b["axis_misalign_deg"] <= 1e-9
                     and abs(b["axis_yaw_deg"] - yaw) <= 1e-9)
        if square_ok and (indep or b["kingpin_off_plane_deg"] <= 1e-6):
            self.statusBar().showMessage(
                f"Arms already share one square 2D sketch at {yaw:.1f} deg "
                "yaw — nothing to fix", 6000)
            return
        caster0 = caster_deg(axle.solver.solve(0.0))
        # square the bushings to the design yaw, THEN (unless the axle is
        # in independent-caster mode) pull the kingpin into the resulting
        # sketch plane to kill the knuckle twist
        fixed = square_bushing_axes(axle.hp, yaw_deg=yaw)
        if not indep:
            fixed = planarize_kingpin(fixed)
        self.apply_hardpoints(fixed)
        a = sketch_planarity(self.active.hp)
        caster1 = caster_deg(self.active.solver.solve(0.0))
        note = ""
        if abs(caster1 - caster0) > 0.5:
            note = (f"; caster shifted {caster0:.1f}->{caster1:.1f} deg as "
                    "the kingpin came into plane — re-dial it in the sketch")
        self.statusBar().showMessage(
            f"Re-squared to {yaw:.1f} deg yaw: misalign "
            f"{b['axis_misalign_deg']:.3f}->{a['axis_misalign_deg']:.2g}, "
            f"kingpin twist {b['kingpin_off_plane_deg']:.2f}->"
            f"{a['kingpin_off_plane_deg']:.2g} deg{note} (Ctrl+Z undoes)",
            12000)

    def run_optimization(self, payload: dict) -> None:
        axle = self.active
        if axle.hp is None:
            return
        goals = [Goal(g["key"], g["target"], g["weight"])
                 for g in payload["goals"]]
        # locked points may NEVER move — drop any free GROUP that touches
        # a locked attr (groups like "uca_inner" cover both bushings)
        from ..optimize import free_groups_for
        groups = free_groups_for(axle.hp)

        def _touches_lock(name: str) -> bool:
            _, attrs = groups.get(name, (None, ()))
            return any(a in axle.locked for a in attrs)

        free = [g for g in payload["free"] if not _touches_lock(g)]
        dropped = [g for g in payload["free"] if _touches_lock(g)]
        if dropped:
            self.statusBar().showMessage(
                f"Optimizer: skipping locked point(s) "
                f"{', '.join(dropped)}", 7000)
        payload = {**payload, "free": free}
        if not goals or not payload["free"]:
            self.statusBar().showMessage(
                "Select at least one goal and one free (unlocked) "
                "hardpoint", 6000)
            return
        travels = np.linspace(-axle.droop, axle.bump, 13)
        before_hp, before_sweep = axle.hp, axle.sweep
        self.statusBar().showMessage("Optimizing…")
        from PySide6.QtWidgets import QApplication
        QApplication.setOverrideCursor(Qt.WaitCursor)
        QApplication.processEvents()
        try:
            result = optimize(axle.hp, goals, payload["free"], travels,
                              box=payload["box_mm"],
                              sketch_yaw_deg=self.design_yaw_of(axle))
        except ValueError as e:
            self.statusBar().showMessage(f"Optimization failed: {e}", 8000)
            return
        finally:
            QApplication.restoreOverrideCursor()
        if not result.success:
            self.statusBar().showMessage(f"Optimization: {result.message}", 8000)
            return
        axle.baseline = (before_sweep["travel_mm"], before_sweep)
        axle.ghost = (before_hp, DoubleWishboneSolver(before_hp).solve(0.0))
        self.apply_hardpoints(result.hp_after)
        self.statusBar().showMessage(
            f"Optimization done: {result.message} ({result.n_evals} "
            "evaluations). Dashed curves / ghost linkage show before.", 12000)

    # ------------------------------------------------------------------
    # Suspension type dropdown
    # ------------------------------------------------------------------
    def _type_name_of(self, hp) -> str:
        if isinstance(hp, TrailingArmPoints):
            return "Trailing arm"
        if isinstance(hp, MultilinkPoints):
            return "Multilink"
        if isinstance(hp, CHubFrontPoints):
            return "C-hub front"
        if isinstance(hp, LoadedHalfshaftPoints):
            return "Loaded halfshaft (rear)"
        if isinstance(hp, HArmRearPoints):
            return "H-arm (rear)"
        return "Double wishbone"

    def _sync_type_combo(self) -> None:
        """Point the dropdown at the active axle's type AND restrict the
        list to what that axle can physically use — the rear-only types
        (H-arm, loaded halfshaft) hold toe by design and have no steering
        input, so they must never be offerable on the steered front."""
        from ..suspension_types import types_for_axle
        allowed = types_for_axle(self.active_key)
        self.type_combo.blockSignals(True)
        if [self.type_combo.itemText(i)
                for i in range(self.type_combo.count())] != allowed:
            self.type_combo.clear()
            self.type_combo.addItems(allowed)
        want = (self._type_name_of(self.active.hp)
                if self.active.hp is not None else "Double wishbone")
        self.type_combo.setCurrentText(
            want if want in allowed else "Double wishbone")
        self.type_combo.blockSignals(False)

    def _on_type_changed(self, name: str) -> None:
        """Re-seed the ACTIVE axle as the selected suspension type from the
        current setup-form values. Reverts the dropdown if the seed cannot
        be built or cannot articulate the travel budget."""
        axle = self.active
        if axle.hp is not None and self._type_name_of(axle.hp) == name:
            return
        from ..suspension_types import is_allowed_on
        if not is_allowed_on(name, self.active_key):
            self.statusBar().showMessage(
                f"{name} has no steering input — it holds toe by design, so "
                f"it cannot go on the steered FRONT axle. Switch to the rear "
                f"axle to use it.", 9000)
            self._sync_type_combo()
            return
        sv = self.setup_form.current_setup()
        if name == "Double wishbone":
            self.generate_seed(sv)       # DW path sets ranges from report
            self._sync_type_combo()
            return
        self._seed_as_type(name, sv)

    # Seeders for the non-double-wishbone types, by dropdown name.
    _TYPED_SEEDERS = {
        "Trailing arm": staticmethod(seed_trailing_arm),
        "Multilink": staticmethod(seed_multilink),
        "C-hub front": staticmethod(seed_chub_front),
        "Loaded halfshaft (rear)": staticmethod(seed_loaded_halfshaft),
        "H-arm (rear)": staticmethod(seed_harm_rear),
    }

    def _seed_as_type(self, name: str, sv: SetupVariables) -> None:
        """Seed the active axle as `name` and adopt it. Shared by the type
        dropdown AND by the Generate button, so pressing Generate honours
        the selected type instead of silently producing a double wishbone
        (which it did until v1.32)."""
        axle = self.active
        seeder = self._TYPED_SEEDERS.get(name)
        if seeder is None:
            self.generate_seed(sv)
            return
        try:
            hp = seeder.__func__(sv) if isinstance(seeder, staticmethod) \
                else seeder(sv)
        except (ValueError, TypeError) as e:
            self.statusBar().showMessage(f"{name} seed failed: {e}", 8000)
            self._sync_type_combo()
            return
        stroke = sv.shock_max_length - sv.shock_min_length
        l_ride = (sv.shock_length_at_ride
                  if sv.shock_length_at_ride is not None
                  else sv.shock_min_length + 0.7 * stroke)
        axle.last_setup = sv
        axle.baseline = None
        axle.ghost = None
        # wheel travel = shock travel / MR  (MR is shock/wheel, < 1)
        axle.droop = axle.req_droop = (sv.shock_max_length - l_ride) / sv.motion_ratio_goal
        axle.bump = axle.req_bump = (l_ride - sv.shock_min_length) / sv.motion_ratio_goal
        # Halfshaft is a seed-time decision for these types too (a
        # trailing-arm rear is THE classic driven Baja axle).
        from ..halfshaft import HalfshaftConfig, default_inner_for
        if isinstance(hp, LoadedHalfshaftPoints):
            # The shaft IS a suspension member here — always modeled, its
            # inner CV is the hs_inner HARDPOINT (single source of truth).
            axle.halfshaft = HalfshaftConfig(enabled=True,
                                             inner=hp.hs_inner.copy())
        elif getattr(sv, "has_halfshaft", False) or isinstance(
                hp, HArmRearPoints):
            # The H-arm rear is a driven axle by convention — model the CV
            # halfshaft (a non-structural angle/plunge check) even if the
            # setup form's 'driven' box wasn't ticked. The user can disable
            # it in the Halfshafts panel for a dead axle.
            axle.halfshaft = HalfshaftConfig(
                enabled=True,
                inner=default_inner_for(hp, axle.bump, axle.droop))
        else:
            axle.halfshaft.enabled = False
        self.halfshaft_panel.load(self.axles["front"].halfshaft,
                                  self.axles["rear"].halfshaft)
        prev = axle.hp
        self._sync_range_spins()
        self.apply_hardpoints(hp)
        if axle.hp is not prev and axle.hp is hp:
            self.statusBar().showMessage(
                f"{self.active_key} axle re-seeded as {name} — edit its "
                "points in the table (tweaks/optimizer stay DW-only)", 9000)
        self._sync_type_combo()

    @staticmethod
    def equation_sheet_path() -> str:
        """Absolute path to the bundled printable equation sheet."""
        import os
        return os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), "docs", "Equation_Sheet.pdf")

    def show_equations(self) -> None:
        """Help > Equations — every formula the tool uses, for hand checks.

        Rendered live from `suspension_tool.equations`, which is the single
        source of truth the printable PDF is built from too, so the two
        cannot drift."""
        import os
        from PySide6.QtCore import QUrl
        from PySide6.QtWidgets import (QHBoxLayout, QPushButton, QTextBrowser,
                                       QVBoxLayout)
        from ..equations import all_equations, to_html
        dlg = QDialog(self)
        dlg.setWindowTitle("Equation reference")
        dlg.resize(820, 760)
        lay = QVBoxLayout(dlg)
        browser = QTextBrowser()
        browser.setHtml(to_html())
        browser.setOpenExternalLinks(True)
        lay.addWidget(browser)
        row = QHBoxLayout()
        row.addWidget(QLabel(f"{len(all_equations())} equations · "
                             "geometry in mm, dynamics in lb/in/s "
                             "(g = 386.4 in/s²)"))
        row.addStretch()
        pdf = QPushButton("Open printable PDF…")
        pdf.clicked.connect(self.open_equation_sheet)
        pdf.setEnabled(os.path.exists(self.equation_sheet_path()))
        row.addWidget(pdf)
        lay.addLayout(row)
        dlg.setModal(False)
        dlg.show()
        self._sweep_dialogs.append(dlg)      # keep alive

    def open_equation_sheet(self) -> None:
        """Open the printable equation sheet in the system PDF viewer."""
        import os
        from PySide6.QtCore import QUrl
        from PySide6.QtGui import QDesktopServices
        path = self.equation_sheet_path()
        if not os.path.exists(path):
            QMessageBox.warning(
                self, "Equation sheet",
                f"The PDF is missing from this install:\n{path}")
            return
        if not QDesktopServices.openUrl(QUrl.fromLocalFile(path)):
            QMessageBox.information(
                self, "Equation sheet",
                f"Could not launch a PDF viewer. The sheet is at:\n{path}")

    @staticmethod
    def bode_guide_path() -> str:
        """Absolute path to the bundled 'Reading the Bode Plot' PDF."""
        import os
        return os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), "docs", "Reading_the_Bode_Plot.pdf")

    def open_bode_guide(self) -> None:
        """Open the two-page Bode-plot guide in the system PDF viewer."""
        import os
        from PySide6.QtCore import QUrl
        from PySide6.QtGui import QDesktopServices
        path = self.bode_guide_path()
        if not os.path.exists(path):
            QMessageBox.warning(
                self, "Bode guide",
                f"The guide PDF is missing from this install:\n{path}")
            return
        if not QDesktopServices.openUrl(QUrl.fromLocalFile(path)):
            QMessageBox.information(
                self, "Bode guide",
                f"Could not launch a PDF viewer. The guide is at:\n{path}")

    def show_help(self) -> None:
        """The in-app user guide (also on F1)."""
        import os
        from PySide6.QtCore import QUrl
        from PySide6.QtWidgets import QTextBrowser, QVBoxLayout
        from .help_text import HELP_HTML
        dlg = QDialog(self)
        dlg.setWindowTitle("User guide")
        dlg.resize(760, 720)
        lay = QVBoxLayout(dlg)
        browser = QTextBrowser()
        # Point the in-guide link at the real file on THIS machine, so it
        # opens in the system PDF viewer rather than 404-ing on a fixed path.
        guide = self.bode_guide_path()
        html = HELP_HTML.replace(
            "__BODE_GUIDE_URL__",
            QUrl.fromLocalFile(guide).toString() if os.path.exists(guide)
            else "")
        browser.setHtml(html)
        browser.setOpenExternalLinks(True)
        lay.addWidget(browser)
        dlg.setModal(False)
        dlg.show()
        self._sweep_dialogs.append(dlg)   # keep alive

    def undo(self) -> None:
        """Step the ACTIVE axle back one geometry change (up to 15 kept)."""
        axle = self.active
        if not axle.undo:
            self.statusBar().showMessage("nothing to undo on this axle", 4000)
            return
        hp = axle.undo.pop()
        remaining = len(axle.undo)
        self._undoing = True
        try:
            self.apply_hardpoints(hp)
        finally:
            self._undoing = False
        self.statusBar().showMessage(
            f"undid 1 geometry change on the {axle.name} axle "
            f"({remaining} more available)", 5000)

    # ------------------------------------------------------------------
    # Direct manipulation (drag a hardpoint in the 3D view)
    # ------------------------------------------------------------------
    def _on_drag_started(self, attr: str) -> None:
        if self.motion_combo.currentText() == "Roll":
            self.motion_combo.setCurrentText("Heave")
        self.show_travel(0.0)          # design edits happen at ride height
        self.statusBar().showMessage(
            f"Dragging {attr} — hold X/Y/Z to lock that axis "
            f"({self.conv.name}); Esc cancels")

    def _on_drag_moved(self, attr: str, point) -> None:
        """Live preview: re-solve the static pose only (cheap); the full
        sweep waits for the release."""
        axle = self.active
        if axle.hp is None:
            return
        if attr == "halfshaft_inner":
            axle.halfshaft.inner = np.asarray(point, float)
            try:
                st = axle.solver.solve(0.0)
            except ValueError:
                return
            for mirror in (False, True):
                vis = self.viewport.scene._corners.get((axle.name, mirror))
                if vis is not None and vis._hs_inner is not None:
                    vis._hs_inner = np.asarray(point, float)
                    vis.set_state(st)
            self.viewport.scene._render()
            return
        cand = dataclasses.replace(axle.hp,
                                   **{attr: np.asarray(point, float)})
        from ..suspension_types import solver_for
        try:
            st = solver_for(cand)(cand).solve(0.0)
        except ValueError:
            return                     # hold the last valid preview
        self.viewport.preview_hardpoints(axle.name, cand, st)

    def _on_drag_finished(self, attr: str, point) -> None:
        axle = self.active
        if axle.hp is None:
            return
        p = np.asarray(point, float)
        if attr == "halfshaft_inner":
            if self.snap_check.isChecked():
                snapped = self._snap_to_frame(axle, p)
                if snapped is not None:
                    p = snapped
            axle.halfshaft.inner = p
            self.halfshaft_panel.load(self.axles["front"].halfshaft,
                                      self.axles["rear"].halfshaft)
            self._augment_halfshaft(axle)
            if axle.sweep is not None:
                self.plots.set_sweep(axle.sweep["travel_mm"], axle.sweep,
                                     baseline=axle.baseline)
            self.hardpoint_table.refresh(axle.hp)
            self.rebuild_scene()
            self._update_checklist()
            self._autosave_soon()
            return
        if self.snap_check.isChecked():
            snapped = self._snap_to_frame(axle, p)
            if snapped is not None:
                p = snapped
                self.statusBar().showMessage(
                    f"{attr} snapped to the frame", 4000)
        self.apply_hardpoints(dataclasses.replace(axle.hp, **{attr: p}))

    def _on_drag_cancelled(self) -> None:
        axle = self.active
        if axle.solver is not None:
            self.show_travel(axle.travel)   # redraw with the real hp
            self.rebuild_scene()
        self.statusBar().showMessage("Drag cancelled", 3000)

    def _snap_to_frame(self, axle: Axle, p_local, max_dist: float = 30.0):
        """Nearest point ON the frame mesh SURFACE within max_dist (not
        just a vertex — tube walls have few vertices along straight
        runs). Works in scene space; the rear axle's local frame is
        offset by the wheelbase. None = no frame or nothing close."""
        scene = self.viewport.scene
        if scene._frame_mesh is None:
            return None
        visual = scene._corners.get((axle.name, False))
        dx = visual.x_offset if visual is not None else 0.0
        p_scene = np.asarray(p_local, float) + np.array([dx, 0.0, 0.0])
        # query in MESH space (the backdrop actor carries the transform)
        m = np.asarray(scene._frame_matrix)
        inv = np.linalg.inv(m)
        q = inv[:3, :3] @ p_scene + inv[:3, 3]
        _, closest = scene._frame_mesh.find_closest_cell(
            q, return_closest_point=True)
        hit_scene = m[:3, :3] @ np.asarray(closest) + m[:3, 3]
        if np.linalg.norm(hit_scene - p_scene) > max_dist:
            return None
        return hit_scene - np.array([dx, 0.0, 0.0])

    # ------------------------------------------------------------------
    # Design candidates (whole-vehicle snapshots)
    # ------------------------------------------------------------------
    def _candidate_metrics(self) -> dict:
        out = {}
        for key, axle in self.axles.items():
            if axle.sweep is None:
                continue
            s = axle.sweep
            k0 = int(np.argmin(np.abs(s["travel_mm"])))
            suffix = "f" if key == "front" else "r"
            out[f"rc_{suffix}"] = float(s["roll_center_height_mm"][k0])
            out[f"travel_{suffix}"] = (f"+{self.unit.fmt(axle.bump)}/"
                                       f"-{self.unit.fmt(axle.droop)} "
                                       f"{self.unit.label}")
            if key == "front":
                out["camber_gain_f"] = float(
                    np.gradient(s["camber_deg"], s["travel_mm"])[k0])
                out["bump_steer_f"] = float(s["bump_steer_deg_per_mm"][k0])
                out["caster_f"] = float(s["caster_deg"][k0])
            cv = s.get("cv_max_deg")
            if cv is not None and np.any(np.isfinite(cv)):
                worst = float(np.nanmax(cv))
                out["cv_worst"] = max(out.get("cv_worst", 0.0), worst)
        return out

    def snapshot_candidate(self) -> None:
        from ..project import _hp_to_dict
        n = len(self.candidates_panel.candidates()) + 1
        cand = {
            "name": f"candidate {n}",
            "front": (_hp_to_dict(self.axles["front"].hp)
                      if self.axles["front"].hp is not None else None),
            "rear": (_hp_to_dict(self.axles["rear"].hp)
                     if self.axles["rear"].hp is not None else None),
            "vehicle": dataclasses.asdict(self.vehicle_panel.params()),
            "metrics": self._candidate_metrics(),
        }
        self.candidates_panel.add_candidate(cand)
        self.statusBar().showMessage(
            f"Snapshot saved as '{cand['name']}' — double-click the name "
            "to rename it", 6000)

    def load_candidate(self, index: int) -> None:
        from ..project import _hp_from_dict
        cands = self.candidates_panel.candidates()
        if not (0 <= index < len(cands)):
            return
        cand = cands[index]
        if cand.get("vehicle"):
            known = {f.name for f in dataclasses.fields(VehicleParams)}
            self.vehicle_panel.load_params(VehicleParams(
                **{k: v for k, v in cand["vehicle"].items() if k in known}))
        restore_key = self.active_key
        for key in ("front", "rear"):
            if cand.get(key) is None:
                continue
            self.active_key = key
            self.apply_hardpoints(_hp_from_dict(cand[key]))
        self.active_key = restore_key
        axle = self.active
        if axle.hp is not None:
            self.hardpoint_table.refresh(axle.hp)
            self.tweaks_panel.refresh(axle.hp)
            if axle.sweep is not None:
                self.plots.set_sweep(axle.sweep["travel_mm"], axle.sweep,
                                     baseline=axle.baseline)
        self._sync_type_combo()
        self.rebuild_scene()
        self.statusBar().showMessage(
            f"Loaded candidate '{cand.get('name', index)}'", 6000)

    def _on_hs_inner_edited(self, point) -> None:
        """Table edit of the halfshaft inner CV (chassis point)."""
        axle = self.active
        axle.halfshaft.inner = np.asarray(point, float)
        self.halfshaft_panel.load(self.axles["front"].halfshaft,
                                  self.axles["rear"].halfshaft)
        self._augment_halfshaft(axle)
        if axle.sweep is not None:
            self.plots.set_sweep(axle.sweep["travel_mm"], axle.sweep,
                                 baseline=axle.baseline)
        self.hardpoint_table.refresh(axle.hp)
        self.rebuild_scene()
        self._update_checklist()
        self._autosave_soon()

    def _on_hs_outer_edited(self, point) -> None:
        """Table edit of the OUTER CV: it lives on the kingpin axis, so
        the request is projected onto the axis and the hub (wheel centre
        + outer CV together) slides there — the tire moves with it."""
        from ..geometry import DoubleWishbonePoints
        from ..tweaks import hub_along_kingpin, set_hub_along_kingpin
        from ..halfshaft import outer_cv_of_hp
        axle = self.active
        if not isinstance(axle.hp, DoubleWishbonePoints):
            self.statusBar().showMessage(
                "Outer CV rides at the wheel centre on this suspension "
                "type — edit the wheel centre instead", 6000)
            self.hardpoint_table.refresh(axle.hp)
            return
        a, b = axle.hp.lca_outer, axle.hp.uca_outer
        d = (b - a) / np.linalg.norm(b - a)
        delta = float(np.dot(np.asarray(point, float)
                             - outer_cv_of_hp(axle.hp), d))
        self.apply_hardpoints(set_hub_along_kingpin(
            axle.hp, hub_along_kingpin(axle.hp) + delta))
        self.statusBar().showMessage(
            "Outer CV request projected onto the kingpin axis — the hub "
            "and tire slid with it (Ctrl+Z undoes)", 7000)

    def _update_checklist(self) -> None:
        self.checklist.set_steering(self._front_steer_sweep())
        self.checklist.evaluate(self.axles)

    def _front_steer_sweep(self) -> dict | None:
        """Rack sweep at ride height for the checklist's turning-radius
        row and Ackermann graph. None when the front can't steer (no
        geometry, unsteerable type, zero rack travel)."""
        front = self.axles.get("front")
        if front is None or front.solver is None or front.hp is None:
            return None
        rack = float(self.checklist.rack_travel_mm)
        if rack <= 0.0 or not hasattr(front.hp, "tierod_inner"):
            return None
        from ..sweeps import steer_sweep
        try:
            return steer_sweep(
                front.solver, rack, 0.0,
                2.0 * abs(float(front.hp.wheel_center[1])),
                self.vehicle_panel.params().wheelbase, n=13)
        except Exception:
            return None    # a mid-edit unsolvable pose must not crash

    def _autosave_soon(self) -> None:
        """(Re)arm the debounced rolling autosave."""
        self._autosave_timer.start()

    def _autosave_path(self) -> str:
        import os
        return os.path.join(os.path.expanduser("~"),
                            ".baja_suspension_autosave.MICK")

    def _write_autosave(self) -> None:
        """Crash insurance: the full project, silently, to a rolling file
        (File -> Restore autosave brings it back)."""
        try:
            # Autosave runs on a timer, so it skips the embed: writing a
            # 3 MB mesh every few seconds is not worth it, and the path
            # still works for a crash recovery on the SAME machine, which
            # is the only situation an autosave is for.
            self._write(self._autosave_path(), quiet=True,
                        embed_frame=False)
        except Exception:
            pass    # autosave must never interrupt the user

    def restore_autosave(self) -> None:
        import os
        path = self._autosave_path()
        if not os.path.exists(path):
            QMessageBox.information(self, "Restore autosave",
                                    "No autosave found yet.")
            return
        self._load_project_file(path)
        self.current_path = None   # force Save As, don't clobber anything
        self._retitle()
        self.statusBar().showMessage("Autosave restored (unsaved)", 8000)

    def _augment_anti(self, axle: Axle) -> None:
        """Attach per-travel anti-squat / anti-dive % curves to the axle's
        sweep so they plot and report like any other channel."""
        if axle.sweep is None or "states" not in axle.sweep:
            return
        from ..vehicle import anti_percent_curves
        squat, dive = anti_percent_curves(
            axle.solver, axle.sweep["states"],
            self.vehicle_panel.params(), axle.name)
        axle.sweep["anti_squat_pct"] = squat
        axle.sweep["anti_dive_pct"] = dive
        self._augment_halfshaft(axle)

    def _augment_halfshaft(self, axle: Axle) -> None:
        """Attach CV-angle / plunge curves for this axle's halfshaft (if
        enabled) and surface limit warnings in the halfshaft panel."""
        if axle.sweep is None or "states" not in axle.sweep:
            return
        from ..halfshaft import halfshaft_curves, halfshaft_warnings
        cfg = axle.halfshaft
        if hasattr(axle.hp, "hs_inner"):
            # Loaded halfshaft: the shaft is STRUCTURAL, its inner CV is
            # the hs_inner hardpoint — the config mirrors it (the panel
            # then shows the live value; plunge reads exactly 0).
            cfg.enabled = True
            cfg.inner = np.asarray(axle.hp.hs_inner, float).copy()
            self.halfshaft_panel.load(self.axles["front"].halfshaft,
                                      self.axles["rear"].halfshaft)
        n = len(axle.sweep["travel_mm"])
        if not cfg.enabled:
            # keep plot channels well-defined (hidden panels show flat 0)
            axle.sweep["cv_max_deg"] = np.full(n, np.nan)
            axle.sweep["plunge_mm"] = np.full(n, np.nan)
            axle.hs_warns = None
            self.halfshaft_panel.show_status(axle.name, None)
            return
        curves = halfshaft_curves(cfg, axle.hp, axle.sweep["states"])
        axle.sweep["cv_inner_deg"] = curves["cv_inner_deg"]
        axle.sweep["cv_outer_deg"] = curves["cv_outer_deg"]
        axle.sweep["cv_max_deg"] = curves["cv_max_deg"]
        axle.sweep["plunge_mm"] = curves["plunge_mm"]
        warns = halfshaft_warnings(cfg, curves)
        axle.hs_warns = warns
        self.halfshaft_panel.show_status(axle.name, warns)
        if warns:
            self.statusBar().showMessage(
                f"{axle.name} halfshaft: " + "; ".join(warns), 8000)

    def _on_halfshaft_changed(self) -> None:
        from ..halfshaft import default_inner_for, inner_is_stale
        # Shock hardware spec (lives in the same panel): update each
        # axle's spec, keep the setup fields in sync (one source), and
        # re-clamp any axle whose spec actually changed.
        respec = []
        for key, spec in self.halfshaft_panel.shock_specs().items():
            axle = self.axles[key]
            old = axle.shock_spec
            axle.shock_spec = spec
            if spec is not None and axle.last_setup is not None:
                axle.last_setup = dataclasses.replace(
                    axle.last_setup, shock_min_length=spec.min_length,
                    shock_max_length=spec.max_length)
            differs = ((old is None) != (spec is None)
                       or (old is not None and spec is not None
                           and (abs(old.min_length - spec.min_length) > 1e-6
                                or abs(old.max_length - spec.max_length)
                                > 1e-6)))
            if differs and axle.hp is not None:
                respec.append(key)
        if respec:
            cur = self.active_key
            try:
                for key in respec:
                    self.active_key = key
                    self.apply_hardpoints(self.axles[key].hp)
            finally:
                self.active_key = cur
            self._resync_active_ui()
        replaced = False
        for key, cfg in self.halfshaft_panel.configs().items():
            axle = self.axles[key]
            if axle.hp is not None and hasattr(axle.hp, "hs_inner"):
                # Loaded halfshaft: the inner CV is a structural hardpoint,
                # so a panel edit MOVES THE SUSPENSION — route it through
                # apply_hardpoints (re-solves, re-clamps, undo-able). The
                # shaft cannot be un-modeled: it is a suspension member.
                cfg.enabled = True
                axle.halfshaft.inner_joint = cfg.inner_joint
                axle.halfshaft.max_cv_inner_deg = cfg.max_cv_inner_deg
                axle.halfshaft.max_cv_outer_deg = cfg.max_cv_outer_deg
                axle.halfshaft.plunge_total_mm = cfg.plunge_total_mm
                axle.halfshaft.plunge_in_pct = cfg.plunge_in_pct
                if not np.allclose(cfg.inner, axle.hp.hs_inner):
                    cur = self.active_key
                    try:
                        self.active_key = key
                        self.apply_hardpoints(dataclasses.replace(
                            axle.hp,
                            hs_inner=np.asarray(cfg.inner, float)))
                    finally:
                        self.active_key = cur
                    self._resync_active_ui()
                else:
                    self._augment_halfshaft(axle)
                continue
            if (cfg.enabled and axle.hp is not None
                    and inner_is_stale(cfg, axle.hp)):
                # e.g. the pre-offset default (0,150,0) while the design
                # sits at x ~ +990: the shaft would slash across the
                # whole scene. Auto-place it sensibly instead.
                cfg.inner = default_inner_for(axle.hp, axle.bump,
                                              axle.droop)
                replaced = True
            axle.halfshaft = cfg
            self._augment_halfshaft(axle)
        if replaced:
            self.halfshaft_panel.load(self.axles["front"].halfshaft,
                                      self.axles["rear"].halfshaft)
            self.statusBar().showMessage(
                "Inner CV was far off this axle — auto-placed at the "
                "axle station, level at mid-travel (edit it in the table "
                "or drag the violet point)", 9000)
        if self.active.hp is not None:
            self.hardpoint_table.refresh(self.active.hp)
        active = self.active
        if active.sweep is not None:
            self.plots.set_sweep(active.sweep["travel_mm"], active.sweep,
                                 baseline=active.baseline)
        self.rebuild_scene()      # show/hide/move the 3D halfshaft line
        self._update_checklist()

    # ------------------------------------------------------------------
    # Design comparison snapshots
    # ------------------------------------------------------------------
    def snapshot_curves(self) -> None:
        axle = self.active
        if axle.sweep is None:
            return
        self._snapshot_count += 1
        label = f"{self.active_key} #{self._snapshot_count}"
        self.plots.add_snapshot(label, axle.sweep["travel_mm"], axle.sweep)
        self.statusBar().showMessage(
            f"Snapshot '{label}' pinned to the plots — tweak away and "
            "compare (up to 4 kept)", 6000)

    def clear_snapshots(self) -> None:
        self.plots.clear_snapshots()

    # ------------------------------------------------------------------
    # Motion sweeps (roll / pitch / steering)
    # ------------------------------------------------------------------
    def _open_sweep(self, dialog_cls) -> None:
        dlg = dialog_cls(self, parent=self)
        dlg.setModal(False)
        dlg.show()
        self._sweep_dialogs.append(dlg)

    def open_roll_sweep(self) -> None:
        from .sweep_dialogs import RollSweepDialog
        self._open_sweep(RollSweepDialog)

    def open_pitch_sweep(self) -> None:
        from .sweep_dialogs import PitchSweepDialog
        self._open_sweep(PitchSweepDialog)

    def open_steer_sweep(self) -> None:
        from .sweep_dialogs import SteerSweepDialog
        self._open_sweep(SteerSweepDialog)

    def open_bump_steer_map(self) -> None:
        from .sweep_dialogs import BumpSteerMapDialog
        self._open_sweep(BumpSteerMapDialog)

    def open_ride_bode(self) -> None:
        """Quarter-car frequency response: the two resonances (body and
        wheel hop), the ride/grip tradeoff, and the road speeds that excite
        them. Driven by the Dynamics panel, so it needs no geometry."""
        from .sweep_dialogs import RideBodeDialog
        self._open_sweep(RideBodeDialog)

    def open_steering_effort(self) -> None:
        """Rack force and driver effort at ride height, from the live
        front geometry and the dynamics sheet's wheel loads."""
        axle = self.axles["front"]
        if axle.hp is None or axle.state is None:
            self.statusBar().showMessage(
                "Steering effort needs front geometry — generate or load "
                "a design first", 6000)
            return
        try:
            _SteeringEffortDialog(self, axle).exec()
        except ValueError as e:
            QMessageBox.critical(self, "Steering effort", str(e))

    def open_yaw_moment(self) -> None:
        """Milliken Moment Method: the whole manoeuvring envelope, plus
        stability index, control moment gain and limit balance. Driven by
        the Dynamics panel and the measured tire data, so it needs no
        geometry."""
        from .sweep_dialogs import YawMomentDialog
        self._open_sweep(YawMomentDialog)

    # ------------------------------------------------------------------
    # View / display options
    # ------------------------------------------------------------------
    def _on_rc_toggled(self, on: bool) -> None:
        self.viewport.set_show_rc(on)
        self.rebuild_scene()

    def _on_ic_toggled(self, on: bool) -> None:
        self.viewport.set_show_ic(on)
        self.rebuild_scene()

    def _on_live_ground_toggled(self, on: bool) -> None:
        self.viewport.set_show_live_ground(on)
        self.viewport.set_live_ground(self.active.travel)

    def _on_graphs_changed(self, *_):
        keys = [k for k, a in self._graph_actions.items() if a.isChecked()]
        self.plots.set_visible_panels(keys)
        self.plots.set_marker(self.active.travel)

    def set_dark(self, on: bool) -> None:
        from PySide6.QtWidgets import QApplication
        from .branding import apply_dark_theme
        app = QApplication.instance()
        if on:
            apply_dark_theme(app)      # the MICKSUS palette (default)
        else:
            app.setPalette(app.style().standardPalette())
        self.plots.set_dark(on)
        self.plots.set_marker(self.active.travel)

    # ------------------------------------------------------------------
    # Frame backdrop (Phase 6)
    # ------------------------------------------------------------------
    def _on_sketch_yaw(self, yaw_deg: float) -> None:
        """Re-squaring to a new yaw changes the corner's DESIGN INTENT.
        Record it on the axle's setup, or the checklist and the sketch-fit
        readout keep grading the new geometry against the old target."""
        import dataclasses as _dc
        axle = self.active
        if axle.last_setup is not None:
            axle.last_setup = _dc.replace(axle.last_setup,
                                          sketch_yaw_deg=float(yaw_deg))
        self.setup_form.set_sketch_yaw(float(yaw_deg))
        self.statusBar().showMessage(
            f"Sketch plane re-squared to {yaw_deg:+.1f} deg yaw — design "
            "target updated to match (Ctrl+Z undoes)", 8000)

    def load_frame_dialog(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Load frame mesh", "",
            "Mesh files (*.stl *.3mf);;All files (*)")
        if path:
            self.load_frame(path)

    def load_frame(self, path: str, remember_path: str | None = None) -> None:
        from ..frame import guess_scale, load_frame_mesh
        try:
            mesh = load_frame_mesh(path)
        except Exception as e:
            QMessageBox.critical(self, "Frame load failed", str(e))
            return
        self.frame_panel.path = remember_path or path
        # Cache the packed bytes once, at load, so saving never has to
        # re-read and re-compress a multi-megabyte file.
        from ..frame import embed_mesh_file
        try:
            self._frame_blob = embed_mesh_file(path)
        except OSError:
            self._frame_blob = None
        if self.frame_panel.transform.scale == 1.0:
            # First load: guess the unit from the mesh size AND auto-align
            # to the chassis convention — team frames are modeled Y-forward
            # in Onshape, so rotating 270 about Z drops them straight onto
            # the (X-forward) kinematics with zero manual fiddling.
            self.frame_panel.transform.scale = guess_scale(
                np.ptp(mesh.points, axis=0))
            self.frame_panel.transform.rot_z_deg = 270.0
        self.frame_panel.refresh()
        self.viewport.set_frame(mesh, self.frame_panel.transform.matrix())
        self.rebuild_scene()
        self.statusBar().showMessage(
            f"Frame loaded ({mesh.n_points} vertices), auto-aligned to the "
            "chassis convention — it should already sit on the design",
            8000)

    def _on_frame_changed(self) -> None:
        self.viewport.set_show_frame(self.frame_panel.show_check.isChecked())
        self.viewport.set_frame_matrix(self.frame_panel.transform.matrix())
        self.viewport.set_frame_opacity(
            self.frame_panel.opacity_spin.value() / 100.0)

    # ------------------------------------------------------------------
    # CAD handoff
    # ------------------------------------------------------------------
    def _axle_hps(self) -> dict:
        return {k: a.hp for k, a in self.axles.items() if a.hp is not None}

    def _csv_text(self) -> str | None:
        axles = self._axle_hps()
        if not axles:
            return None
        if len(axles) == 1:
            return hardpoints_csv(next(iter(axles.values())),
                                  self.unit.mm, self.unit.label,
                                  conv=self.conv)
        return vehicle_csv(axles, self.vehicle_panel.params().wheelbase,
                           self.unit.mm, self.unit.label, conv=self.conv)

    def export_csv(self) -> None:
        text = self._csv_text()
        if text is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export hardpoints CSV", "", "CSV (*.csv)")
        if not path:
            return
        if not path.endswith(".csv"):
            path += ".csv"
        try:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(text)
        except OSError as e:
            QMessageBox.critical(self, "Export failed", str(e))
            return
        self.statusBar().showMessage(
            f"Exported hardpoints ({self.unit.label}) to {path}", 6000)

    def export_impact_loads(self) -> None:
        """Impact scenario -> load at every chassis pickup -> CSV for FEA.

        Asks for the scenario (speed in, speed out, contact time, how much
        of the car this corner takes, pulse shape, safety factor), runs the
        impulse + load-path solve on the ACTIVE axle's geometry, and writes
        a self-describing CSV that ANSYS / Onshape Simulation can consume."""
        from ..impact import (PULSE_SHAPES, ImpactCase, corner_loads,
                              loads_csv)
        axle = self.active
        if axle.hp is None:
            self.statusBar().showMessage(
                "Generate or load a design first — impact loads need "
                "geometry", 6000)
            return
        dlg = _ImpactDialog(self, self.dynamics_panel.current_inputs())
        if not dlg.exec():
            return
        case = dlg.case()
        try:
            result = corner_loads(axle.hp, case.wheel_load())
        except (ValueError, NotImplementedError) as e:
            QMessageBox.critical(self, "Impact loads", str(e))
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export impact loads CSV", "", "CSV (*.csv)")
        if not path:
            return
        if not path.endswith(".csv"):
            path += ".csv"
        try:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(loads_csv(case, result, unit_mm=self.unit.mm,
                                   unit_label=self.unit.label,
                                   force_in_newtons=dlg.newtons(),
                                   conv=self.conv))
        except OSError as e:
            QMessageBox.critical(self, "Export failed", str(e))
            return
        worst = result.worst_mount()
        member = result.worst_member()
        self.statusBar().showMessage(
            f"Impact loads → {path}   (corner force "
            f"{case.peak_force_lb():.0f} lb ≈ {case.equivalent_g():.1f} g; "
            f"worst mount {worst.name} {worst.magnitude:.0f} lb; worst "
            f"member {member.name} {member.axial_lb:+.0f} lb "
            f"{member.state})", 15000)

    def export_load_envelope(self) -> None:
        """Lotus-style load matrix → the design ENVELOPE → CSV for FEA.

        Builds the friction-circle load matrix (braking, cornering, drive,
        every combination, plus bump and the impulse cases) from the tire +
        dynamics models, runs the load path at each case THROUGH the travel
        range, and writes the worst force per pickup / worst axial per member
        — the number that actually sizes the hardware."""
        from ..impact import (standard_load_matrix, envelope, envelope_csv)
        from ..suspension_types import solver_for
        axle = self.active
        if axle.hp is None:
            self.statusBar().showMessage(
                "Generate or load a design first — the load envelope needs "
                "geometry", 6000)
            return
        default_corner = ("front_out" if self.active_key == "front"
                          else "rear_out")
        dlg = _EnvelopeDialog(self, default_corner)
        if not dlg.exec():
            return
        dyn = self.dynamics_panel.current_inputs()
        try:
            loads, cond = standard_load_matrix(
                corner=dlg.corner(), dyn=dyn, surface=dlg.surface(),
                n_dir=dlg.n_dir(), bump_g=dlg.bump_g(),
                vehicle_weight_lb=float(getattr(dyn, "weight_empty_lb", 400.0))
                + float(getattr(dyn, "driver_lb", 180.0)))
            solver = travels = None
            if dlg.sweep_travel():
                from ..geometry import DoubleWishbonePoints
                if isinstance(axle.hp, DoubleWishbonePoints):
                    solver = solver_for(axle.hp)(axle.hp)
                    travels = np.linspace(-axle.droop, axle.bump, 11)
            env = envelope(axle.hp, loads, solver=solver, travels=travels,
                           conditions=cond)
        except (ValueError, NotImplementedError, KeyError) as e:
            QMessageBox.critical(self, "Load envelope", str(e))
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export load envelope CSV", "", "CSV (*.csv)")
        if not path:
            return
        if not path.endswith(".csv"):
            path += ".csv"
        try:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(envelope_csv(env, unit_mm=self.unit.mm,
                                      unit_label=self.unit.label,
                                      force_in_newtons=dlg.newtons(),
                                      conv=self.conv))
        except OSError as e:
            QMessageBox.critical(self, "Export failed", str(e))
            return
        wm = env.worst_mount()
        wmem = env.worst_member()
        self.statusBar().showMessage(
            f"Load envelope → {path}   ({len(env.case_names)} cases × "
            f"{env.n_poses} pose(s); worst mount {wm.name} {wm.magnitude:.0f} "
            f"lb via {wm.governing_case}; worst member {wmem.name} "
            f"{wmem.worst_axial_lb:+.0f} lb {wmem.state})", 15000)

    def export_body_loads(self) -> None:
        """Per-part self-equilibrated load sets + arm bending → CSV.

        The chassis export gives you frame pickups; this one gives you what
        to apply to an INDIVIDUAL part (a control arm, the upright) in a
        component FEA — including the bending a mid-span shock mount induces,
        which never shows up in a joint reaction because it is an internal
        stress resultant."""
        from ..impact import body_loads_csv, corner_loads, shock_bending_summary
        axle = self.active
        if axle.hp is None:
            self.statusBar().showMessage(
                "Generate or load a design first — per-part loads need "
                "geometry", 6000)
            return
        dlg = _ImpactDialog(self, self.dynamics_panel.current_inputs())
        if not dlg.exec():
            return
        case = dlg.case()
        try:
            result = corner_loads(axle.hp, case.wheel_load())
        except (ValueError, NotImplementedError) as e:
            QMessageBox.critical(self, "Per-part loads", str(e))
            return
        if not result.bodies:
            QMessageBox.information(
                self, "Per-part loads",
                "Per-part load sets are currently produced for the double "
                "wishbone only; the other types report chassis and internal "
                "joint loads through the impact export.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export per-part loads CSV", "", "CSV (*.csv)")
        if not path:
            return
        if not path.endswith(".csv"):
            path += ".csv"
        try:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(body_loads_csv(result, unit_mm=self.unit.mm,
                                        unit_label=self.unit.label,
                                        force_in_newtons=dlg.newtons(),
                                        conv=self.conv))
        except OSError as e:
            QMessageBox.critical(self, "Export failed", str(e))
            return
        bend = shock_bending_summary(result)
        extra = ""
        for arm, d in bend.items():
            extra = (f"; shock bends the {arm} by "
                     f"{d['bending_about_ball_joint_lb_in']:.0f} lb·in about "
                     f"its ball joint")
        self.statusBar().showMessage(
            f"Per-part loads → {path}   ({len(result.bodies)} bodies, each "
            f"self-equilibrated{extra})", 15000)

    def copy_csv(self) -> None:
        text = self._csv_text()
        if text is None:
            return
        from PySide6.QtWidgets import QApplication
        QApplication.clipboard().setText(text)
        self.statusBar().showMessage(
            f"Hardpoints CSV ({self.unit.label}) copied — paste into the "
            "Onshape hardpoints feature", 6000)

    def _project_state_now(self, embed_frame: bool = True):
        """The current design as a ProjectState (same builder as saving)."""
        def design(axle: Axle) -> AxleDesign | None:
            if axle.hp is None:
                return None
            return AxleDesign(hardpoints=axle.hp, setup=axle.last_setup,
                              droop_travel=axle.droop, bump_travel=axle.bump,
                              halfshaft=axle.halfshaft.to_dict(),
                              locked=sorted(axle.locked) or None,
                              shock=(axle.shock_spec.to_dict()
                                     if axle.shock_spec is not None
                                     else None))
        return ProjectState(
            front=design(self.axles["front"]),
            rear=design(self.axles["rear"]),
            unit_key=self.unit.key,
            goals={**self.optimize_panel.to_dict(),
                   "checklist": self.checklist.to_dict()},
            vehicle=dataclasses.asdict(self.vehicle_panel.params()),
            frame=self._frame_state(embed=embed_frame),
            dynamics={**self.dynamics_panel.current_inputs().to_dict(),
                      "_linked": self.dynamics_panel.link_check.isChecked()},
            coords=self.conv.key,
            candidates=self.candidates_panel.candidates() or None,
            onshape=self._onshape_manifest,
            steer_limit_mm=float(self.steer_limit),
        )

    def _frame_state(self, embed: bool = True) -> dict | None:
        """The frame block for a project file.

        A backdrop used to be stored as a PATH, which breaks the moment
        the project moves to another machine. The mesh now rides inside
        the file (deflated, see `frame.embed_mesh_file`), with the path
        kept alongside so it can still be re-linked to a live export.
        `embed=False` is for autosave, which runs on a timer.
        """
        if not self.frame_panel.path:
            return None
        out = {"path": self.frame_panel.path,
               "opacity": float(self.frame_panel.opacity_spin.value()) / 100.0,
               **self.frame_panel.transform.to_dict()}
        if embed and self._frame_blob is not None:
            out["mesh"] = self._frame_blob
        return out

    def _design_notes_axles(self) -> dict:
        """Assemble the per-axle data the PDF generator wants from the
        live sweeps/metrics (None where an axle has no geometry)."""
        from ..metrics import corner_metrics
        out = {}
        for name, axle in self.axles.items():
            if axle.hp is None or axle.solver is None:
                out[name] = None
                continue
            try:
                static = corner_metrics(axle.solver, axle.solver.solve(0.0))
            except Exception:
                static = None
            hs = None
            if axle.sweep is not None and "cv_max_deg" in axle.sweep:
                import numpy as np
                cv = axle.sweep["cv_max_deg"]
                pl = axle.sweep.get("plunge_mm")
                if np.any(np.isfinite(cv)):
                    hs = {"cv_max": float(np.nanmax(cv)),
                          "plunge": (float(np.nanmax(np.abs(pl)))
                                     if pl is not None else None)}
            out[name] = {"hp": axle.hp, "static": static,
                         "sweep": axle.sweep,
                         "travel": (axle.droop, axle.bump), "halfshaft": hs}
        return out

    def build_cad_in_onshape(self) -> None:
        """The full geometry push (Ethan's v1.10 request): deposit the 3D
        hardpoints + front/rear sketch planes via a custom FeatureScript
        feature, drop in a design-notes PDF, and — crucially — UPDATE an
        existing push in place instead of duplicating it."""
        axles = self._axle_hps()
        if not axles:
            QMessageBox.information(self, "Build CAD in Onshape",
                                    "Generate at least one axle first.")
            return
        dlg = OnshapeDialog(self)
        if dlg.exec() != QDialog.Accepted:
            return
        from ..onshape_sync import push_geometry, OnshapeClient
        from ..report_pdf import design_notes_pdf
        cfg = dlg.values()
        wheelbase = self.vehicle_panel.params().wheelbase
        from PySide6.QtWidgets import QApplication
        QApplication.setOverrideCursor(Qt.WaitCursor)
        self.statusBar().showMessage("Building CAD in Onshape…")
        try:
            pdf = design_notes_pdf(self._design_notes_axles(), wheelbase,
                                   version=self._version)
            client = OnshapeClient(cfg["access"], cfg["secret"])
            halfshafts = {name: ax.halfshaft
                          for name, ax in self.axles.items()
                          if ax.hp is not None and ax.halfshaft is not None}
            result = push_geometry(
                client, cfg["url"], axles, wheelbase, pdf_bytes=pdf,
                manifest=self._onshape_manifest, conv=self.conv,
                halfshafts=halfshafts, studio_name=cfg["studio"],
                partstudio_name=cfg["partstudio"])
        except Exception as e:
            QMessageBox.critical(self, "Onshape CAD build failed", str(e))
            self.statusBar().clearMessage()
            return
        finally:
            QApplication.restoreOverrideCursor()
        # remember the manifest so the NEXT push updates these same tabs
        self._onshape_manifest = result["manifest"]
        self._autosave_soon()
        QMessageBox.information(
            self, "Onshape CAD build",
            "Done:\n\n" + "\n".join(result["summary"])
            + "\n\nRe-running this updates these same tabs in place.")
        self.statusBar().showMessage("Onshape CAD build complete", 8000)

    def pull_design_from_onshape(self) -> None:
        dlg = OnshapeDialog(self)
        if dlg.exec() != QDialog.Accepted:
            return
        from ..onshape_sync import OnshapeClient, pull_design
        from ..project import dict_to_project
        cfg = dlg.values()
        from PySide6.QtWidgets import QApplication
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            client = OnshapeClient(cfg["access"], cfg["secret"])
            state = dict_to_project(pull_design(client, cfg["url"]))
        except Exception as e:
            QMessageBox.critical(self, "Onshape pull failed", str(e))
            return
        finally:
            QApplication.restoreOverrideCursor()
        self.current_path = None    # pulled design: Save As decides where
        self._retitle()
        self._apply_project_state(state)
        self.statusBar().showMessage(
            "Design pulled from the Onshape document (unsaved)", 8000)

    def export_report(self) -> None:
        """Static design table + vehicle numbers + full curves, one CSV."""
        from ..project import report_csv
        from ..vehicle import roll_axis_metrics as _ram
        axles = {k: a.sweep for k, a in self.axles.items()
                 if a.sweep is not None}
        if not axles:
            return
        params = self.vehicle_panel.params()
        u = self.unit
        rows = {
            f"wheelbase_{u.label}": f"{u.from_mm(params.wheelbase):.3f}",
            f"cg_height_{u.label}": f"{u.from_mm(params.cg_height):.3f}",
            f"cg_behind_front_{u.label}":
                f"{u.from_mm(params.cg_behind_front):.3f}",
            "brake_front_frac": f"{params.brake_front_frac:.2f}",
        }
        f, r = self.axles["front"], self.axles["rear"]
        if np.isfinite(f.rc_h) and np.isfinite(r.rc_h):
            m = _ram(f.rc_h, r.rc_h, params)
            rows["roll_axis_angle_deg"] = f"{m['roll_axis_angle_deg']:.3f}"
            rows[f"roll_moment_arm_{u.label}"] = (
                f"{u.from_mm(m['roll_moment_arm_mm']):.3f}")
            if f.state is not None and r.state is not None:
                anti = anti_geometry((f.solver, f.state),
                                     (r.solver, r.state), params)
                for k, v in anti.items():
                    rows[k] = f"{v:.2f}" if np.isfinite(v) else "-"
        path, _ = QFileDialog.getSaveFileName(
            self, "Export report CSV", "", "CSV (*.csv)")
        if not path:
            return
        if not path.endswith(".csv"):
            path += ".csv"
        try:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(report_csv(axles, rows, u.mm, u.label))
        except OSError as e:
            QMessageBox.critical(self, "Export failed", str(e))
            return
        self.statusBar().showMessage(f"Report exported to {path}", 6000)

    # ------------------------------------------------------------------
    # File menu
    # ------------------------------------------------------------------
    def _fresh_axles(self) -> dict:
        from ..halfshaft import HalfshaftConfig
        axles = {"front": Axle("front"), "rear": Axle("rear")}
        for a in axles.values():
            a.halfshaft = HalfshaftConfig()
        return axles

    def new_project(self) -> None:
        self.current_path = None
        self._onshape_manifest = None    # a new design pushes to new tabs
        self._retitle()
        self.axles = self._fresh_axles()
        self.halfshaft_panel.load(self.axles["front"].halfshaft,
                                  self.axles["rear"].halfshaft)
        self.halfshaft_panel.load_shock_specs(None, None)
        self.active_key = "front"
        self.axle_combo.setCurrentIndex(0)
        self.setup_form.load_setup(DEFAULT_SETUP)
        self.hardpoint_table.refresh(None)
        self.rebuild_scene()
        self._update_checklist()
        self.statusBar().showMessage(
            "New blank project — load your chassis, set the Setup "
            "values, then Generate seed", 10000)

    def open_project(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Open project", "",
            f"Suspension project (*{FILE_EXTENSION});;All files (*)")
        if not path:
            return
        self._load_project_file(path)

    def _load_project_file(self, path: str) -> None:
        try:
            state = load_project(path)
        except (OSError, ValueError, KeyError) as e:
            QMessageBox.critical(self, "Open failed", f"Could not open:\n{e}")
            return
        self.current_path = path
        self._retitle()
        self._apply_project_state(state)
        notes = list(getattr(state, "migration_notes", []) or [])
        if notes:
            # Never reinterpret saved data silently — say what changed.
            QMessageBox.information(
                self, "Project updated on load",
                "This file was saved before the motion-ratio convention "
                "changed (v1.28). MICKSUS now uses the standard "
                "SHOCK/WHEEL ratio (below 1), which drops straight into "
                "Kw = Ks·MR² with no inversion.\n\n" + "\n".join(notes)
                + "\n\nThe geometry itself is unchanged — only the way the "
                  "ratio is written. Re-save to store it in the new form.")
            self.statusBar().showMessage(
                f"Opened {path} — motion-ratio convention migrated "
                f"({len(notes)} value(s))", 12000)
        else:
            self.statusBar().showMessage(f"Opened {path}", 6000)

    def _apply_project_state(self, state) -> None:
        if state.unit_key in UNITS:
            self.set_units(state.unit_key)
        if state.vehicle:
            known = {f.name for f in dataclasses.fields(VehicleParams)}
            self.vehicle_panel.load_params(VehicleParams(
                **{k: v for k, v in state.vehicle.items() if k in known}))
        if state.goals:
            self.optimize_panel.load_dict(state.goals)
            self.checklist.load_dict(state.goals.get("checklist") or {})
        if state.dynamics:
            self.dynamics_panel.load_inputs(
                state.dynamics, linked=state.dynamics.get("_linked", True))
        from ..axes import CONVENTIONS as _convs
        if state.coords in _convs:
            self.set_convention(state.coords)
        self.candidates_panel.load_list(state.candidates)
        if state.steer_limit_mm:
            # v1.36: the rack limit used to live only on the window, so it
            # silently reset to the 38 mm default on every reload -- the
            # one UI setting a saved file did not carry.
            self.steer_limit = float(state.steer_limit_mm)
            self._apply_steer_unit()
            self._sync_range_spins()
        self._onshape_manifest = state.onshape   # re-push updates in place
        if state.frame and state.frame.get("path"):
            from ..frame import FrameTransform, extract_mesh_file
            self.frame_panel.transform = FrameTransform.from_dict(state.frame)
            if state.frame.get("opacity") is not None:
                self.frame_panel.opacity_spin.blockSignals(True)
                self.frame_panel.opacity_spin.setValue(
                    float(state.frame["opacity"]) * 100.0)
                self.frame_panel.opacity_spin.blockSignals(False)
            import os
            blob = state.frame.get("mesh")
            if blob:
                # The embedded copy wins: it is what this design was drawn
                # against, and it is there precisely so the file opens on a
                # machine that has never seen the original CAD export.
                import tempfile
                d = tempfile.mkdtemp(prefix="micksus-frame-")
                try:
                    self.load_frame(extract_mesh_file(blob, d),
                                    remember_path=state.frame["path"])
                except (OSError, ValueError, KeyError) as e:
                    self.statusBar().showMessage(
                        f"Embedded frame mesh could not be read ({e})", 8000)
            elif os.path.exists(state.frame["path"]):
                self.load_frame(state.frame["path"])
            else:
                self.statusBar().showMessage(
                    f"Frame mesh not found: {state.frame['path']} — this "
                    "project was saved before meshes were embedded", 8000)
        self.axles = self._fresh_axles()
        for key, design in (("front", state.front), ("rear", state.rear)):
            if design is None:
                continue
            axle = self.axles[key]
            axle.last_setup = design.setup
            axle.droop = axle.req_droop = design.droop_travel
            axle.bump = axle.req_bump = design.bump_travel
            axle.locked = frozenset(design.locked or ())
            from ..halfshaft import HalfshaftConfig as _HS
            from ..shock import ShockSpec as _SS
            axle.halfshaft = _HS.from_dict(design.halfshaft)
            axle.shock_spec = _SS.from_dict(design.shock)
            if axle.shock_spec is None and design.setup is not None:
                # older file: migrate the setup's shock fields into the
                # fixed hardware spec (same numbers, now first-class)
                axle.shock_spec = _SS(design.setup.shock_min_length,
                                      design.setup.shock_max_length)
            self.active_key = key
            self.apply_hardpoints(design.hardpoints)
        self.halfshaft_panel.load(self.axles["front"].halfshaft,
                                  self.axles["rear"].halfshaft)
        self.halfshaft_panel.load_shock_specs(
            self.axles["front"].shock_spec, self.axles["rear"].shock_spec)
        for a in self.axles.values():
            self._augment_halfshaft(a)
        self.active_key = "front" if state.front is not None else "rear"
        self.axle_combo.blockSignals(True)
        self.axle_combo.setCurrentIndex(0 if self.active_key == "front" else 1)
        self.axle_combo.blockSignals(False)
        axle = self.active
        if axle.last_setup is not None:
            self.setup_form.load_setup(axle.last_setup)
        self._sync_range_spins()
        if axle.hp is not None:
            self.hardpoint_table.refresh(axle.hp)
            self.tweaks_panel.refresh(axle.hp)
            self.plots.set_sweep(axle.sweep["travel_mm"], axle.sweep)
        self._push_locks(axle)
        self._push_vehicle_frame()   # reset to the FRONT datum (the batch
                                     # apply above left it on the rear axle)
        self.rebuild_scene()

    def save_project(self) -> None:
        if self.current_path is None:
            self.save_project_as()
        else:
            self._write(self.current_path)

    def save_project_as(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "Save project", "",
            f"Suspension project (*{FILE_EXTENSION})")
        if not path:
            return
        if not path.endswith(FILE_EXTENSION):
            path += FILE_EXTENSION
        self.current_path = path
        self._retitle()
        self._write(path)

    def _write(self, path: str, quiet: bool = False,
               embed_frame: bool = True) -> None:
        def design(axle: Axle) -> AxleDesign | None:
            if axle.hp is None:
                return None
            return AxleDesign(hardpoints=axle.hp, setup=axle.last_setup,
                              droop_travel=axle.droop, bump_travel=axle.bump,
                              halfshaft=axle.halfshaft.to_dict(),
                              locked=sorted(axle.locked) or None,
                              shock=(axle.shock_spec.to_dict()
                                     if axle.shock_spec is not None
                                     else None))
        state = ProjectState(
            front=design(self.axles["front"]),
            rear=design(self.axles["rear"]),
            unit_key=self.unit.key,
            goals={**self.optimize_panel.to_dict(),
                   "checklist": self.checklist.to_dict()},
            vehicle=dataclasses.asdict(self.vehicle_panel.params()),
            frame=self._frame_state(embed=embed_frame),
            dynamics={**self.dynamics_panel.current_inputs().to_dict(),
                      "_linked": self.dynamics_panel.link_check.isChecked()},
            coords=self.conv.key,
            candidates=self.candidates_panel.candidates() or None,
            onshape=self._onshape_manifest,
            steer_limit_mm=float(self.steer_limit),
        )
        try:
            save_project(path, state)
        except OSError as e:
            if not quiet:
                QMessageBox.critical(self, "Save failed",
                                     f"Could not save:\n{e}")
            return
        if not quiet:
            self.statusBar().showMessage(f"Saved {path}", 6000)

    # ------------------------------------------------------------------
    def _on_range_changed(self) -> None:
        axle = self.active
        axle.droop = axle.req_droop = self.unit.to_mm(self.droop_spin.value())
        axle.bump = axle.req_bump = self.unit.to_mm(self.bump_spin.value())
        self._sync_range_spins()
        if axle.hp is not None:
            self.apply_hardpoints(axle.hp)

    def _on_play(self, playing: bool) -> None:
        self.play_btn.setText("⏸ Pause" if playing else "▶ Play")
        if playing:
            self._anim.start()
        else:
            self._anim.stop()

    def _anim_tick(self) -> None:
        axle = self.active
        lo, hi = -axle.droop, axle.bump
        step = max(1.0, (hi - lo) / 120.0)
        t = axle.travel + self._anim_dir * step
        if t >= hi:
            t, self._anim_dir = hi, -1.0
        elif t <= lo:
            t, self._anim_dir = lo, 1.0
        self.show_travel(t)


class OnshapeDialog(QDialog):
    """Document URL + API keys + tab names for the Onshape CAD build/pull.
    Everything persists in QSettings so it's a one-time setup."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Onshape document & API keys")
        self._settings = QSettings("BajaSuspensionTool", "onshape")
        form = QFormLayout(self)
        self.url = QLineEdit(self._settings.value("url", ""))
        self.access = QLineEdit(self._settings.value("access", ""))
        self.secret = QLineEdit(self._settings.value("secret", ""))
        self.secret.setEchoMode(QLineEdit.Password)
        self.studio = QLineEdit(self._settings.value(
            "studio", "SuspensionHardpoints"))
        self.partstudio = QLineEdit(self._settings.value(
            "partstudio", "Suspension Hardpoints"))
        form.addRow("Document URL", self.url)
        form.addRow("API access key", self.access)
        form.addRow("API secret key", self.secret)
        form.addRow("Variable Studio name", self.studio)
        form.addRow("Part Studio name", self.partstudio)
        form.addRow(QLabel(
            "Get free API keys at dev-portal.onshape.com/keys.\n"
            "Tabs are created if missing, else updated IN PLACE — pushing\n"
            "again moves the existing geometry, never duplicates it."))
        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self._accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def _accept(self) -> None:
        for key, w in (("url", self.url), ("access", self.access),
                       ("secret", self.secret), ("studio", self.studio),
                       ("partstudio", self.partstudio)):
            self._settings.setValue(key, w.text())
        self.accept()

    def values(self) -> dict:
        return {"url": self.url.text().strip(),
                "access": self.access.text().strip(),
                "secret": self.secret.text().strip(),
                "studio": self.studio.text().strip() or "SuspensionHardpoints",
                "partstudio": (self.partstudio.text().strip()
                               or "Suspension Hardpoints")}


class _ImpactDialog(QDialog):
    """Ask for one impact scenario, then hand back an ImpactCase.

    Defaults come from the Dynamics panel (vehicle weight, corner load) so
    the numbers already match the car being modelled."""

    def __init__(self, parent, dyn):
        super().__init__(parent)
        from ..impact import PULSE_SHAPES, STANDARD_CASES, drop_height_to_mph
        self.setWindowTitle("Impact load case → FEA")
        self.resize(520, 0)
        from PySide6.QtWidgets import QFormLayout, QVBoxLayout
        lay = QVBoxLayout(self)
        blurb = QLabel(
            "Impulse: an impact that changes the car's speed by Δv in a "
            "contact time Δt needs an average force <b>F = m·Δv/Δt</b>. The "
            "pulse shape scales that average up to the design PEAK, the "
            "corner share is how much of the car this wheel takes, and the "
            "result is pushed through the linkage to every chassis pickup."
            "<br><i>Quasi-static: the peak force is applied as a static "
            "load — no structural dynamics or bushing compliance.</i>")
        blurb.setWordWrap(True)
        lay.addWidget(blurb)
        form = QFormLayout()

        self.preset = QComboBox()
        self.preset.addItem("(custom)", None)
        for key, c in STANDARD_CASES.items():
            self.preset.addItem(c.name, key)
        self.preset.currentIndexChanged.connect(self._apply_preset)
        form.addRow(QLabel("Preset"), self.preset)

        self.direction = QComboBox()
        for key, label in (("head_on", "Head-on (fore/aft)"),
                           ("vertical", "Vertical (bump / landing)"),
                           ("lateral", "Lateral (kerb / slide)")):
            self.direction.addItem(label, key)
        form.addRow(QLabel("Direction"), self.direction)

        def _spin_local(lo, hi, dec, step, val, suffix=""):
            s = CommitSpin()
            s.setRange(lo, hi)
            s.setDecimals(dec)
            s.setSingleStep(step)
            s.setValue(val)
            if suffix:
                s.setSuffix(suffix)
            return s

        self.v0 = _spin_local(0, 200, 2, 1.0, 15.0, " mph")
        self.v1 = _spin_local(0, 200, 2, 1.0, 0.0, " mph")
        self.dt = _spin_local(0.001, 5.0, 3, 0.01, 0.15, " s")
        w = float(getattr(dyn, "weight_empty_lb", 400.0)) + float(
            getattr(dyn, "driver_lb", 180.0))
        self.weight = _spin_local(1, 5000, 1, 10.0, w, " lb")
        self.share = _spin_local(0.0, 1.0, 3, 0.05, 0.5)
        self.sf = _spin_local(1.0, 10.0, 2, 0.1, 1.0)
        self.pulse = QComboBox()
        for key in PULSE_SHAPES:
            self.pulse.addItem(f"{key} (×{PULSE_SHAPES[key]:.2f})", key)
        self.pulse.setCurrentIndex(list(PULSE_SHAPES).index("half_sine"))
        self.drop = _spin_local(0.0, 200.0, 1, 1.0, 0.0, " in")
        self.drop.setToolTip("Optional: a free-fall drop height fills the "
                             "impact speed from v = √(2gh)")
        self.drop.valueChanged.connect(self._from_drop)
        self.newton = QCheckBox("Write forces in newtons (else lb)")

        form.addRow(QLabel("Speed at impact"), self.v0)
        form.addRow(QLabel("Speed after impact"), self.v1)
        form.addRow(QLabel("…or drop height"), self.drop)
        form.addRow(QLabel("Contact time Δt"), self.dt)
        form.addRow(QLabel("Vehicle weight"), self.weight)
        form.addRow(QLabel("Share taken by THIS corner"), self.share)
        form.addRow(QLabel("Pulse shape (peak/avg)"), self.pulse)
        form.addRow(QLabel("Safety factor"), self.sf)
        form.addRow(self.newton)
        lay.addLayout(form)

        self.readout = QLabel("")
        self.readout.setWordWrap(True)
        lay.addWidget(self.readout)
        for wdg in (self.v0, self.v1, self.dt, self.weight, self.share,
                    self.sf):
            wdg.valueChanged.connect(self._update_readout)
        self.pulse.currentIndexChanged.connect(self._update_readout)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok
                                   | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        lay.addWidget(buttons)
        self._update_readout()

    def _from_drop(self, height_in):
        from ..impact import drop_height_to_mph
        if height_in > 0.0:
            self.v0.setValue(drop_height_to_mph(height_in))
            self.v1.setValue(0.0)
            self.direction.setCurrentIndex(1)      # vertical
            self._update_readout()

    def _apply_preset(self):
        from ..impact import STANDARD_CASES
        key = self.preset.currentData()
        if key is None:
            return
        c = STANDARD_CASES[key]
        self.v0.setValue(c.speed_mph)
        self.v1.setValue(c.final_speed_mph)
        self.dt.setValue(c.contact_time_s)
        self.share.setValue(c.corner_share)
        idx = self.direction.findData(c.direction)
        if idx >= 0:
            self.direction.setCurrentIndex(idx)
        pidx = self.pulse.findData(c.pulse)
        if pidx >= 0:
            self.pulse.setCurrentIndex(pidx)
        self._update_readout()

    def case(self):
        from ..impact import ImpactCase
        return ImpactCase(
            name=(self.preset.currentText() if self.preset.currentData()
                  else "custom impact"),
            direction=self.direction.currentData(),
            speed_mph=self.v0.value(), final_speed_mph=self.v1.value(),
            contact_time_s=self.dt.value(),
            vehicle_weight_lb=self.weight.value(),
            corner_share=self.share.value(),
            pulse=self.pulse.currentData(),
            safety_factor=self.sf.value(), include_static=False)

    def newtons(self) -> bool:
        return self.newton.isChecked()

    def _update_readout(self, *_):
        try:
            c = self.case()
            self.readout.setText(
                f"<b>Impulse</b> {c.impulse_lb_s():.1f} lb·s → average "
                f"{c.average_force_lb():.0f} lb → <b>corner design force "
                f"{c.peak_force_lb():.0f} lb</b> "
                f"(≈ {c.equivalent_g():.1f} g on the whole car)")
        except ValueError as e:
            self.readout.setText(f"<span style='color:#c1121f'>{e}</span>")


class _EnvelopeDialog(QDialog):
    """Pick the corner + surface for the friction-circle load matrix. The
    grip and per-corner vertical load come from the tire + dynamics models,
    so the numbers already match the car being modelled."""

    def __init__(self, parent, default_corner="front_out"):
        super().__init__(parent)
        from PySide6.QtWidgets import (QFormLayout, QVBoxLayout)
        from ..tire import SURFACE_PEAK_MU, SUNF_23x7
        self.setWindowTitle("Load-case envelope → FEA")
        self.resize(540, 0)
        lay = QVBoxLayout(self)
        blurb = QLabel(
            "The worst force a member sees is a <b>combined</b> load from the "
            "traction circle — F<sub>x</sub>, F<sub>y</sub> and "
            "F<sub>z</sub> together — not one axis at a time. This runs the "
            "whole matrix (braking, cornering, drive, every combination, "
            "plus bump and the impulse cases) through the travel range and "
            "keeps the <b>worst per pickup and per member</b> — the envelope "
            "that sizes the hardware. Grip and corner load come from the tire "
            "+ dynamics models.")
        blurb.setWordWrap(True)
        lay.addWidget(blurb)
        form = QFormLayout()

        self.corner_box = QComboBox()
        for key, label in (("front_out", "Front outside"),
                           ("front_in", "Front inside"),
                           ("rear_out", "Rear outside"),
                           ("rear_in", "Rear inside")):
            self.corner_box.addItem(label, key)
        idx = self.corner_box.findData(default_corner)
        if idx >= 0:
            self.corner_box.setCurrentIndex(idx)
        form.addRow(QLabel("Loaded corner"), self.corner_box)

        self.surface_box = QComboBox()
        for key in SURFACE_PEAK_MU:
            self.surface_box.addItem(f"{key} (μ {SURFACE_PEAK_MU[key]:.2f})",
                                     key)
        sidx = self.surface_box.findData(SUNF_23x7.design_surface)
        if sidx >= 0:
            self.surface_box.setCurrentIndex(sidx)
        form.addRow(QLabel("Surface"), self.surface_box)

        self.ndir = CommitIntSpin()
        self.ndir.setRange(4, 72)
        self.ndir.setValue(12)
        self.ndir.setToolTip("How many directions around the friction circle")
        form.addRow(QLabel("Friction-circle directions"), self.ndir)

        self.bump = CommitSpin()
        self.bump.setRange(1.0, 10.0)
        self.bump.setDecimals(1)
        self.bump.setSingleStep(0.5)
        self.bump.setValue(3.0)
        self.bump.setSuffix(" g")
        form.addRow(QLabel("Vertical bump factor"), self.bump)

        self.sweep = QCheckBox("Sweep the travel range (double wishbone) — "
                               "worst link angle is usually at bump/droop")
        self.sweep.setChecked(True)
        self.newton = QCheckBox("Write forces in newtons (else lb)")
        lay.addLayout(form)
        lay.addWidget(self.sweep)
        lay.addWidget(self.newton)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok
                                   | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        lay.addWidget(buttons)

    def corner(self) -> str:
        return self.corner_box.currentData()

    def surface(self) -> str:
        return self.surface_box.currentData()

    def n_dir(self) -> int:
        return int(self.ndir.value())

    def bump_g(self) -> float:
        return float(self.bump.value())

    def sweep_travel(self) -> bool:
        return self.sweep.isChecked()

    def newtons(self) -> bool:
        return self.newton.isChecked()


class _SteeringEffortDialog(QDialog):
    """Rack force at the grip limit, live from the front geometry.

    Loads come from the Dynamics panel so the number follows the car, and
    the panel says so loudly when those loads are not plausible for the
    car being modelled -- which is exactly the failure this tool has hit
    before, an axle weight left over from a different vehicle.
    """

    def __init__(self, parent, axle):
        super().__init__(parent)
        from PySide6.QtWidgets import QFormLayout, QVBoxLayout
        from ..steering import IN as _SIN
        self.setWindowTitle("Steering effort — rack force")
        self.resize(600, 0)
        self._win = parent
        self._axle = axle
        lay = QVBoxLayout(self)
        lay.addWidget(QLabel(
            "<b>Cornering at the grip limit.</b> Lateral force at each front "
            "contact patch makes a moment about the <b>kingpin axis</b>; the "
            "tie rod holds it, and its lateral component is what the rack "
            "pushes. Both moments are computed in 3D from the actual points "
            "— <i>k·[(P−A)×F]</i> — not the textbook <i>Fy×trail</i>, which "
            "assumes a vertical kingpin and is 4% out on this geometry."))
        form = QFormLayout()
        self._trail = _spin_pneu = CommitSpin()
        _spin_pneu.setRange(0.0, 200.0)
        _spin_pneu.setDecimals(1)
        _spin_pneu.setSuffix(" mm")
        _spin_pneu.setToolTip(
            "Zero at the limit (the tire is saturated). The aligning "
            "moment PEAKS BELOW the limit though, so put a few percent of "
            "the tire radius in to see that worse case.")
        form.addRow(QLabel("Pneumatic trail"), _spin_pneu)
        self._travel = CommitSpin()
        self._travel.setRange(0.0, 500.0)
        self._travel.setDecimals(1)
        self._travel.setSuffix(" mm")
        self._travel.setValue(76.2)
        self._travel.setToolTip("Rack travel for one turn of the wheel; "
                                "0 hides the driver-effort rows")
        form.addRow(QLabel("Rack travel per steering-wheel turn"),
                    self._travel)
        self._wheel = CommitSpin()
        self._wheel.setRange(0.0, 800.0)
        self._wheel.setDecimals(0)
        self._wheel.setSuffix(" mm")
        self._wheel.setValue(280.0)
        form.addRow(QLabel("Steering-wheel diameter"), self._wheel)
        lay.addLayout(form)
        for w in (self._trail, self._travel, self._wheel):
            w.valueChanged.connect(self._recompute)
        self._out = QLabel("")
        self._out.setTextFormat(Qt.RichText)
        self._out.setWordWrap(True)
        lay.addWidget(self._out)
        box = QDialogButtonBox(QDialogButtonBox.Close)
        box.rejected.connect(self.reject)
        lay.addWidget(box)
        self._recompute()

    def result_now(self):
        """The computed effort, or a ValueError explaining why not."""
        from ..dynamics import compute
        from ..steering import cornering_effort
        from ..tire import surface_tire
        win = self._win
        dyn = win.dynamics_panel.current_inputs()
        res = compute(dyn)
        loads = (float(res["load_front_out_lb"]),
                 float(res["load_front_in_lb"]))
        surface = getattr(win.checklist, "surface", None)
        tire = surface_tire(surface() if callable(surface)
                            else "dry_hardpack")
        return dyn, loads, cornering_effort(
            self._axle.hp, self._axle.state, loads, tire,
            pneumatic_trail_mm=self._trail.value(),
            rack_travel_per_turn_mm=self._travel.value(),
            wheel_diameter_mm=self._wheel.value())

    def _plausibility(self, dyn) -> list:
        """Front axle weight sanity, judged against the car's own size --
        the same relative test the CG check uses, so it means the same
        thing on a Baja car and a 1/10 model."""
        msgs = []
        wf = float(dyn.front_axle_lb)
        track = win_track = self._win._axle_track(self._axle)
        # a wheel carrying less than a pound per inch of track is not a
        # car anyone drives; this catches a mass block left from another
        # vehicle, which is the real-world failure
        if track > 0 and wf < 0.25 * (track / 25.4):
            msgs.append(
                f"Front axle weight is {wf:.2f} lb on a "
                f"{track / 25.4:.0f} in track car — that is not this "
                f"vehicle. Fix the Dynamics panel's mass block; every "
                f"force below scales directly with it.")
        return msgs

    def _recompute(self, *_):
        u = self._win.unit
        try:
            dyn, loads, eff = self.result_now()
        except (ValueError, KeyError, ZeroDivisionError) as e:
            self._out.setText(f"<span style='color:#c02020'>{e}</span>")
            return
        warn = self._plausibility(dyn)
        rows = ["<table cellpadding=3>"]
        rows.append("<tr><th align=left>Wheel</th><th align=right>Fz</th>"
                    "<th align=right>μ</th><th align=right>Fy</th>"
                    "<th align=right>Kingpin torque</th>"
                    "<th align=right>Tie-rod force</th></tr>")
        for c in eff.corners:
            rows.append(
                f"<tr><td>{c.name}</td><td align=right>{c.fz_lb:.0f} lb</td>"
                f"<td align=right>{c.mu:.2f}</td>"
                f"<td align=right>{c.fy_lb:.0f} lb</td>"
                f"<td align=right>{c.kingpin_torque_lbin:.0f} lb-in</td>"
                f"<td align=right>{c.tierod_force_lb:.0f} lb</td></tr>")
        rows.append("</table>")
        arm = eff.corners[0].effective_arm_mm if eff.corners else 0.0
        rows.append(
            f"<p><b>Rack axial force {eff.rack_force_lb:.0f} lb "
            f"({eff.rack_force_n:.0f} N)</b> — size the bolted joint and "
            f"the rack on this.<br>Effective steering arm about the "
            f"kingpin: {u.fmt(arm)} {u.label}.</p>")
        if eff.steering_wheel_torque_lbin is not None:
            rows.append(
                f"<p>Steering wheel: "
                f"{eff.steering_wheel_torque_lbin:.1f} lb-in "
                f"({eff.steering_wheel_torque_lbin / 12.0:.1f} lb-ft)"
                + (f", {eff.hand_force_lb:.1f} lb at the rim"
                   if eff.hand_force_lb is not None else "") + ".</p>")
        for w in warn:
            rows.append(f"<p style='color:#c02020'><b>⚠ {w}</b></p>")
        for n in eff.notes:
            rows.append(f"<p style='color:#888888'>{n}</p>")
        self._out.setText("".join(rows))
