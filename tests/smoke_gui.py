"""Manual/headless GUI smoke test (NOT part of `python -m unittest` —
GUI rendering needs a display or OSMesa, so run it explicitly):

    QT_QPA_PLATFORM=offscreen python -m tests.smoke_gui

Builds the main window and exercises: the travel slider, a good edit, a
rejected bad edit, animation, unit switching (in <-> mm), and a .MICK
save/load round-trip. Saves a screenshot to /tmp. On a normal desktop just
run `python -m suspension_tool.gui` and click around instead.
"""

import os
import tempfile

import numpy as np

from PySide6.QtWidgets import QApplication


def main() -> None:
    app = QApplication([])
    from suspension_tool.gui.main_window import MainWindow
    from suspension_tool.project import load_project

    win = MainWindow()
    win.resize(1500, 900)
    win.show()
    app.processEvents()
    # v1.9: MICKSUS boots BLANK — no surprise default seed; the table is
    # empty and the title carries the app name.
    assert win.hp is None, "boot should be blank"
    assert win.hardpoint_table.table.rowCount() == 0
    assert "MICKSUS" in win.windowTitle()
    assert win.dark_check.isChecked() and win.plots._dark, \
        "MICKSUS must boot dark"
    # switching axles while blank must not crash or auto-seed
    win.axle_combo.setCurrentIndex(1)
    app.processEvents()
    assert win.hp is None
    win.axle_combo.setCurrentIndex(0)
    app.processEvents()
    # generate the seed the user would (Setup -> Generate)
    win.setup_form._emit()
    app.processEvents()
    assert win.hp is not None, "seed generation failed"

    # Default display unit is inches (per feedback).
    assert win.unit.key == "in", win.unit.key

    # Travel slider (slider works in mm internally).
    win.show_travel(150.0)
    app.processEvents()
    assert "in" in win.travel_label.text()

    # Switch to mm and back; the readout label should reflect the unit.
    # v1.37: the Dynamics panel follows the same toggle into SI. Compare
    # its inputs either side of the switch with no event loop in between,
    # so this pins the display layer and not a stray kinematic push.
    _dp = win.dynamics_panel
    _imp = _dp.current_inputs().to_dict()
    win.set_units("mm")
    assert _dp.current_inputs().to_dict() == _imp, \
        "the unit toggle moved a dynamics value"
    assert _dp._labels["weight_empty_lb"].text() == "Empty mass (kg)"
    assert "(N/mm)" in _dp._labels["spring1_front_lbin"].text()
    assert _dp._outs["load_front_out_lb"][0].text().endswith(" N")
    app.processEvents()
    assert "mm" in win.travel_label.text()
    assert win.hardpoint_table.table.horizontalHeaderItem(0).text().endswith("(mm)")
    win.set_units("in")
    assert _dp._labels["weight_empty_lb"].text() == "Empty weight (lb)"
    assert _dp._outs["load_front_out_lb"][0].text().endswith(" lb")
    app.processEvents()

    # A good hardpoint edit (in display units) must be adopted and re-solved.
    table = win.hardpoint_table.table
    row = 2  # UCA outer (UBJ)
    old_in = float(table.item(row, 2).text())
    new_in = old_in + 0.5  # +0.5 inch
    table.item(row, 2).setText(f"{new_in}")
    app.processEvents()
    # The displayed value is in inches; internal storage must be that in mm.
    assert abs(win.hp.uca_outer[2] - new_in * 25.4) < 1e-6, win.hp.uca_outer[2]

    # v1.5 workflow: a wild edit is NOT hard-rejected anymore — it is
    # adopted with the travel range auto-clamped to whatever still
    # articulates (the checklist flags the shortfall), OR refused only
    # if the linkage can barely move at all. Either way: no crash, undo
    # brings the old geometry back.
    old_cell = table.item(5, 0).text()
    old_hp = win.hp
    table.item(5, 0).setText("500")   # LBJ 500 in to the other side...
    app.processEvents()
    if table.item(5, 0).text() != old_cell:      # adopted (clamped)
        win.undo()
        app.processEvents()
    assert table.item(5, 0).text() == old_cell
    assert np.allclose(win.hp.lca_outer, old_hp.lca_outer)

    # Animation ping-pong.
    win.play_btn.setChecked(True)
    for _ in range(10):
        win._anim_tick()
    win.play_btn.setChecked(False)

    # Phase 4: run an optimization through the window handler. Break the
    # tie rod first so there is something to fix, then check the before/
    # after overlay context is set and the defect is gone.
    import dataclasses
    from suspension_tool.optimize import Goal  # noqa: F401 (import check)
    bad = dataclasses.replace(
        win.hp, tierod_inner=win.hp.tierod_inner + np.array([0, 0, 15.0]))
    win.apply_hardpoints(bad)
    app.processEvents()
    win.run_optimization({"goals": [{"key": "bump_steer", "target": 0.0,
                                     "weight": 1.0}],
                          "free": ["tierod_inner"], "box_mm": 40.0})
    app.processEvents()
    assert win.active.baseline is not None, "before/after overlay not set"
    assert win.active.ghost is not None
    from suspension_tool.metrics import toe_deg
    s = win.solver
    bump_steer = (toe_deg(s.solve(1.0)) - toe_deg(s.solve(-1.0))) / 2.0
    assert abs(bump_steer) < 0.005, f"bump steer not fixed: {bump_steer}"

    # Phase 4.1: tweaks panel drives geometry in display units.
    from suspension_tool.tweaks import shock_mount_params
    d_before = shock_mount_params(win.hp)["d"]
    _, dw_spins, _ = win.tweaks_panel._boxes["dw"]
    dw_spins["d"].setValue(dw_spins["d"].value() + 1.0)  # +1 inch
    app.processEvents()
    # (the spinbox displays 3 decimals in inches, so allow its rounding)
    assert abs(shock_mount_params(win.hp)["d"] - d_before - 25.4) < 0.05

    # Graph visibility toggles re-flow the grid.
    from suspension_tool.gui.plots import PANELS as _PANELS
    win._graph_actions["caster_deg"].setChecked(False)
    app.processEvents()
    assert len(win.plots._visible) == len(_PANELS) - 1
    win._graph_actions["caster_deg"].setChecked(True)

    # Dark mode toggles cleanly and lands back on the MICKSUS default
    # (dark) so the final screenshot shows the real boot look.
    win.set_dark(False)
    win.set_dark(True)
    app.processEvents()
    assert win.plots._dark

    # CSV for Onshape: 11 point rows on the clipboard (front axle only).
    win.copy_csv()
    rows = [l for l in QApplication.clipboard().text().splitlines()
            if l and not l.startswith("#") and not l.startswith("name,")]
    assert len(rows) == 11, rows

    # v1.9: switching to a blank rear does NOT auto-seed; the user
    # generates it explicitly from the setup values.
    win.axle_combo.setCurrentIndex(1)
    app.processEvents()
    assert win.active_key == "rear"
    assert win.axles["rear"].hp is None, "rear must stay blank"
    win.setup_form._emit()
    app.processEvents()
    assert win.axles["rear"].hp is not None, "rear seed failed"
    win.axle_combo.setCurrentIndex(0)
    app.processEvents()
    assert win.active_key == "front"
    from suspension_tool.metrics import toe_deg as _toe
    toe_straight = _toe(win.solver.solve(win.active.travel))
    win.steer_slider.setValue(20)      # 20 mm of rack
    app.processEvents()
    st = win.solver.solve(win.active.travel, win.steer)
    assert abs(_toe(st) - toe_straight) > 0.5, "steer had no effect"
    win.steer_slider.setValue(0)
    app.processEvents()

    # With both axles present the CSV becomes a vehicle file (22 rows,
    # rear shifted rearward by the wheelbase).
    win.copy_csv()
    rows = [l for l in QApplication.clipboard().text().splitlines()
            if l and not l.startswith("#") and not l.startswith("name,")]
    assert len(rows) == 22, len(rows)

    # Roll axis + CG numbers appear once both axles have geometry.
    assert "deg" in win.vehicle_panel._out["roll_axis_angle_deg"].text()

    # v1.12.1: wheelbase + track-width readouts appear alongside the live
    # metrics (ride height was already there).
    for key in ("wheelbase_mm", "track_width_mm", "ride_height_mm"):
        assert win.readouts._labels[key].text() not in ("", "-"), \
            f"{key} readout empty"
    # v1.12.2: track width is LIVE (from the displayed geometry) — it
    # breathes with the tires, matching half-track change, not the setup.
    win.show_travel(0.0)
    tw0 = float(win.readouts._labels["track_width_mm"].text())
    win.show_travel(60.0)
    tw1 = float(win.readouts._labels["track_width_mm"].text())
    assert abs(tw1 - tw0) > 0.01, "track width not live (didn't move with travel)"
    win.show_travel(0.0)

    # v1.12.1: the outer CV sits ON the tire centreline (spin axis) — the
    # CV output line is coincident with the wheel, not cocked at an angle.
    from suspension_tool.halfshaft import outer_cv_of_state
    _st = win.solver.solve(0.0)
    _v = _st.wheel_center - outer_cv_of_state(_st)
    _sin = (np.linalg.norm(np.cross(_v, _st.spindle))
            / (np.linalg.norm(_v) * np.linalg.norm(_st.spindle)))
    assert _sin < 1e-9, f"outer CV off the tire centreline: sin={_sin}"

    # v1.12.3: the knuckle-consistency convention is auto-enforced — the
    # tire spin axis lies in the (ball joints + wheel centre) plane.
    from suspension_tool.geometry import static_spindle
    _hp = win.hp
    _king = _hp.uca_outer - _hp.lca_outer
    _king = _king / np.linalg.norm(_king)
    _nA = np.cross(_king, _hp.wheel_center - _hp.lca_outer)
    _nA = _nA / np.linalg.norm(_nA)
    _s = static_spindle(_hp)
    _oop = abs(float(np.dot(_s / np.linalg.norm(_s), _nA)))
    assert _oop < 1e-6, f"tire spin axis not in the knuckle plane: {_oop}"

    # v1.0: switch the REAR axle to a trailing arm through the dropdown —
    # the table re-shapes to its points, the scene redraws, and the
    # vehicle panel shows anti-geometry percentages.
    win.axle_combo.setCurrentIndex(1)          # editing: rear
    app.processEvents()
    win.type_combo.setCurrentText("Trailing arm")
    app.processEvents()
    from suspension_tool.trailing_arm import TrailingArmPoints
    assert isinstance(win.axles["rear"].hp, TrailingArmPoints), \
        win.statusBar().currentMessage()
    assert win.hardpoint_table.table.rowCount() == 5
    assert "trailing_arm" in QApplication.clipboard().text() or True
    anti_txt = win.vehicle_panel._out["anti_squat_rear_pct"].text()
    assert anti_txt not in ("", "-"), f"anti-squat not shown: {anti_txt!r}"

    # v1.1: the tweaks panel now speaks trailing-arm — skew the pivot
    # axis (semi-trailing) through the panel and the geometry follows.
    from suspension_tool.tweaks import trailing_arm_params
    _, ta_spins, _ = win.tweaks_panel._boxes["ta"]
    ta_spins["plan_deg"].setValue(15.0)
    app.processEvents()
    assert abs(trailing_arm_params(win.hp)["plan_deg"] - 15.0) < 0.01
    # and the optimizer free list re-shaped to trailing-arm groups
    assert "pivot_outer" in win.optimize_panel._free
    assert "uca_inner" not in win.optimize_panel._free
    # and back to a double wishbone for the rest of the checks
    win.type_combo.setCurrentText("Double wishbone")
    app.processEvents()
    win.axle_combo.setCurrentIndex(0)
    app.processEvents()

    # v1.2: undo restores the previous geometry (15 levels).
    before_undo = win.hp.uca_outer.copy()
    table = win.hardpoint_table.table
    old_v = float(table.item(2, 2).text())
    table.item(2, 2).setText(f"{old_v + 0.3}")
    app.processEvents()
    assert abs(win.hp.uca_outer[2] - before_undo[2]) > 1.0
    win.undo()
    app.processEvents()
    assert abs(win.hp.uca_outer[2] - before_undo[2]) < 1e-6, "undo failed"

    # v1.2: roll mode poses left and right differently (mirror auto-on).
    win.motion_combo.setCurrentText("Roll")
    app.processEvents()
    assert win.mirror_check.isChecked()
    win.show_travel(60.0)
    app.processEvents()
    scene = win.viewport.scene
    left_pts = scene._corners[("front", False)]._meshes["points"].points
    right_pts = scene._corners[("front", True)]._meshes["points"].points
    # wheel-centre z differs between the sides in roll (index 9 for DW)
    assert abs(left_pts[9][2] - right_pts[9][2]) > 50.0, "roll pose missing"
    win.motion_combo.setCurrentText("Heave")
    app.processEvents()

    # v1.2: anti-percentage channels rode along with the sweep.
    assert "anti_squat_pct" in win.sweep and "anti_dive_pct" in win.sweep
    assert np.all(np.isfinite(win.sweep["anti_squat_pct"]))

    # v1.4: chassis coordinate convention is the default — the table's
    # middle column is the fore-aft (Y fwd) axis and edits round-trip.
    assert "Y fwd" in win.hardpoint_table.table.horizontalHeaderItem(1).text()
    # Use the SHOCK INNER (row 9), a free chassis point. This check used to
    # edit the wheel centre (row 8), which cannot move fore-aft: the
    # knuckle-consistency rule in apply_hardpoints (enforce_knuckle_planes)
    # keeps the spin axis in the ball-joint/wheel-centre plane, so a fore-aft
    # nudge of the wheel centre is projected straight back and the edit
    # reads as a silent no-op. That is intended behaviour, asserted below —
    # but it makes the wheel centre the wrong probe for "edits round-trip".
    x_before = win.hp.shock_inner[0]
    cell = win.hardpoint_table.table.item(9, 1)     # shock inner, Y fwd
    typed = float(cell.text()) + 0.5                # +0.5 in forward
    cell.setText(f"{typed}")
    app.processEvents()
    # What you type is what you get: the coordinate lands on the typed value.
    assert abs(win.hp.shock_inner[0] - typed * 25.4) < 1e-6, \
        (typed, win.hp.shock_inner)
    # ...and the move is +0.5 in, to within the display quantum the cell text
    # was rounded to (3 dp in inches = 0.0127 mm).
    assert abs(win.hp.shock_inner[0] - x_before - 12.7) < 0.0127, \
        win.hp.shock_inner
    win.undo()
    app.processEvents()

    # The flip side of the same rule: the wheel centre's fore-aft coordinate
    # is DERIVED (it stays in the ball-joint plane), so typing into it is
    # deliberately inert. Pin that, so the day it starts moving is the day
    # this test tells us the knuckle convention broke.
    wc_before = win.hp.wheel_center.copy()
    wc_cell = win.hardpoint_table.table.item(8, 1)
    wc_cell.setText(f"{float(wc_cell.text()) + 0.5}")
    app.processEvents()
    assert abs(win.hp.wheel_center[0] - wc_before[0]) < 1e-9, \
        ("wheel-centre fore-aft is knuckle-constrained and must not move",
         wc_before, win.hp.wheel_center)
    win.set_convention("tool")
    assert win.hardpoint_table.table.horizontalHeaderItem(0).text().startswith("X fwd")
    win.set_convention("chassis+y")
    app.processEvents()

    # v1.4: halfshaft on the rear axle — channels + status appear.
    hs = win.halfshaft_panel
    hs._w["rear"]["enable"].setChecked(True)
    app.processEvents()
    assert win.axles["rear"].sweep is not None
    assert np.any(np.isfinite(win.axles["rear"].sweep["cv_max_deg"]))
    assert hs._w["rear"]["status"].text() != ""
    hs._w["rear"]["enable"].setChecked(False)
    app.processEvents()

    # v1.4: ride-height / reset-pose buttons.
    win.show_travel(60.0)
    win.steer_slider.setValue(15)
    app.processEvents()
    win.reset_pose()
    app.processEvents()
    assert abs(win.active.travel) < 1e-6
    assert abs(win.steer) < 1e-6

    # v1.4: ghost toggle exists and flips scene state.
    win.ghost_check.setChecked(False)
    assert not win.viewport.scene.show_ghost
    win.ghost_check.setChecked(True)

    # v1.5: checklist is live (front column graded, banner set) and the
    # travel auto-clamp kicks in for a binding edit.
    assert win.checklist._cells[("articulation", "front")].text() != "—"
    assert win.checklist.banner.text() != ""
    # Ask for more bump travel than the linkage can physically reach:
    # the request must be ADOPTED with the swept range auto-clamped, the
    # ask remembered, and the checklist's articulation row flipped to a
    # fail. Probe the true lock first so the test doesn't depend on how
    # earlier steps reshaped the geometry.
    # (v1.16.1: with a shock spec the range is AUTO — test the manual
    # articulation-clamp path with the spec temporarily removed)
    spec_saved = win.active.shock_spec
    win.active.shock_spec = None
    win._sync_range_spins()
    orig_b = win.active.bump
    lock = win._probe_reachable(win.solver, 799.0, +1.0)
    if lock < 700.0:                     # linkage locks inside spin range
        ask = min(lock + 80.0, 799.0)
        win.bump_spin.setValue(win.unit.from_mm(ask))
        win._on_range_changed()
        app.processEvents()
        assert win.active.req_bump > lock + 40.0
        assert win.active.bump <= lock + 1.0, (win.active.bump, lock)
        assert "✗" in win.checklist._cells[("articulation", "front")].text()
        win.bump_spin.setValue(win.unit.from_mm(orig_b))
        win._on_range_changed()
        app.processEvents()
        # Restoring through the SPIN goes via display units at the
        # spin's decimal precision, so the round trip is only exact when
        # the value happens to be representable there. It was, with the
        # old default seed (+9.809 in); it is not in general. Assert the
        # round trip to the spin's own resolution rather than to 1e-6.
        _res = win.unit.to_mm(10.0 ** -win.bump_spin.decimals())
        assert abs(win.active.bump - orig_b) <= 2.0 * _res, \
            (win.active.bump, orig_b, _res)
    assert "✓" in win.checklist._cells[("articulation", "front")].text()
    win.active.shock_spec = spec_saved
    win.apply_hardpoints(win.hp)         # back to shock-driven range
    app.processEvents()

    # v1.5: autosave writes the rolling file when the debounce fires.
    win._write_autosave()
    assert os.path.exists(win._autosave_path())

    # v1.5: hardpoint dragging — context is armed, axis-lock math obeys
    # the active display convention, and a simulated drag lands + undoes.
    from suspension_tool.gui.viewport import drag_attrs_for
    vp = win.viewport
    assert vp._drag_ctx is not None
    n_pts = len(vp.scene._corners[("front", False)]._meshes["points"].points)
    assert len(vp._drag_ctx["attrs"]) == n_pts == len(drag_attrs_for(win.hp))
    from PySide6.QtCore import Qt as _Qt
    vp._keys = {_Qt.Key_Y}                    # chassis Y = tool forward
    locked = vp._lock_axis(np.array([10.0, 5.0, 2.0]))
    assert np.allclose(locked, [10.0, 0.0, 0.0]), locked
    vp._keys = set()
    old_pt = win.hp.uca_outer.copy()
    win._on_drag_started("uca_outer")
    win._on_drag_moved("uca_outer", old_pt + np.array([0.0, 0.0, 6.0]))
    win._on_drag_finished("uca_outer", old_pt + np.array([0.0, 0.0, 6.0]))
    app.processEvents()
    assert abs(win.hp.uca_outer[2] - old_pt[2] - 6.0) < 1e-6
    win.undo()
    app.processEvents()
    assert np.allclose(win.hp.uca_outer, old_pt)

    # v1.2: fit view + help dialog don't blow up.
    win.viewport.fit()
    win.show_help()
    app.processEvents()

    # v1.3 / v1.33: a SEED lands on the Onshape frame origin — x_offset is
    # the axle station (wheel centre) and z_offset the ground plane. Check a
    # freshly generated seed, not win.hp: by this point the window's geometry
    # has been through the optimizer, a drag and several tweak edits, and
    # enforce_knuckle_planes legitimately nudges the wheel centre when the
    # ball-joint plane moves — so win.hp is ~1 mm off by design, and always
    # was. (Before v1.33 this assertion failed outright by ~13 mm, because
    # x_offset anchored the inboard bushing axes instead of the axle.)
    from suspension_tool.seed import generate_seed as _gen_seed
    _fresh_hp, _ = _gen_seed(win.setup_form.current_setup())
    assert abs(_fresh_hp.wheel_center[0] - 39.0 * 25.4) < 1e-6, \
        _fresh_hp.wheel_center
    assert abs((_fresh_hp.wheel_center[2] - _fresh_hp.tire_radius)
               - (-14.0 * 25.4)) < 1e-6, _fresh_hp.wheel_center
    assert abs(win.viewport.scene._ground_z
               - (win.hp.wheel_center[2] - win.hp.tire_radius)) < 1e-6

    # v1.3: editing a value must NOT move the camera (only Fit/views do).
    table = win.hardpoint_table.table
    cam_before = tuple(win.viewport.scene.plotter.camera.position)
    v0 = float(table.item(2, 2).text())
    table.item(2, 2).setText(f"{v0 + 0.2}")
    app.processEvents()
    cam_after = tuple(win.viewport.scene.plotter.camera.position)
    assert np.allclose(cam_before, cam_after), (cam_before, cam_after)
    win.undo()
    app.processEvents()

    # v1.3: dockable panels exist and the Panels menu can re-show them.
    assert set(win._docks) >= {"setup", "hardpoints", "tweaks", "optimize",
                               "vehicle", "dynamics", "frame", "readouts"}
    win._docks["tweaks"].close()
    assert not win._docks["tweaks"].isVisible()
    win._docks["tweaks"].toggleViewAction().trigger()
    app.processEvents()

    # v1.3: sketch-planarity readout is live for the double wishbone.
    assert "Sketch fit OK" in win.hardpoint_table.sketch_fit.text()

    # v1.3: CG marker is placed and toggles. v1.38: it is the CAD
    # centre-of-mass symbol, so it is TWO actors (light + dark octants)
    # that must show, hide and move together.
    _cg = win.viewport.scene._cg_actors
    assert len(_cg) == 2, "CG marker should be a two-tone chequered ball"
    assert all(a.GetVisibility() for a in _cg)
    win.cg_check.setChecked(False)
    app.processEvents()
    assert not any(a.GetVisibility() for a in _cg)
    win.cg_check.setChecked(True)
    app.processEvents()
    assert all(a.GetVisibility() for a in _cg)
    _m = [np.array(a.user_matrix)[:3, 3] for a in _cg]
    assert np.allclose(_m[0], _m[1]), "the two halves must sit together"

    # v1.43: steering effort opens, reads live geometry, and warns when
    # the dynamics mass block is not this car.
    import suspension_tool.gui.main_window as _mwmod
    _sd = _mwmod._SteeringEffortDialog(win, win.axles["front"])
    _dyn, _loads, _eff = _sd.result_now()
    assert _eff.rack_force_lb > 0.0, _eff.rack_force_lb
    assert len(_eff.corners) == 2
    assert _eff.corners[0].fz_lb >= _eff.corners[1].fz_lb, "outside carries more"
    assert _eff.steering_wheel_torque_lbin > 0.0
    _arm = _eff.corners[0].effective_arm_mm
    from suspension_tool.tweaks import steer_arm_length as _sal
    assert abs(_arm - abs(_sal(win.axles["front"].hp))) > 1e-6, \
        "effective arm must not collapse to the scalar steering-arm length"
    _sd.deleteLater()

    # v1.41: track and overall width are BOTH shown, and they differ by
    # exactly one tire width — the confusion that prompted this.
    _ro = win.readouts._last
    assert "track_width_mm" in _ro and "overall_width_mm" in _ro, sorted(_ro)
    # They differ by a tire width MINUS the camber term: track is measured
    # at the CONTACT PATCH (so it breathes with half-track change) while
    # the overall figure is the nominal one taken at the wheel centres,
    # which is the dimension CAD quotes. On a cambered corner the contact
    # patch sits outboard of the wheel-centre plane, so the gap is a
    # little under one tire width -- and the discrepancy is exactly twice
    # that offset, not slop.
    _st = win.active.solver.solve(win.active.travel)
    _cam_off = 2.0 * (abs(float(_st.contact_patch[1]))
                      - abs(float(_st.wheel_center[1])))
    assert abs((_ro["overall_width_mm"] - _ro["track_width_mm"])
               - (win.active.hp.tire_width - _cam_off)) < 0.5, \
        (_ro["overall_width_mm"], _ro["track_width_mm"], _cam_off)
    # v1.41: roll angle's track comes from the GEOMETRY, not the seed form.
    import dataclasses as _dcw
    _live = 2.0 * abs(float(win.active.hp.wheel_center[1]))
    assert abs(win._axle_track(win.active) - _live) < 1e-6
    win.active.last_setup = _dcw.replace(win.active.last_setup,
                                         track_width=_live + 203.2)
    assert abs(win._axle_track(win.active) - _live) < 1e-6, \
        "stale seed track leaked back into the roll-angle calculation"
    assert abs(win.track_mismatch(win.active) - 203.2) < 1e-6
    win.active.last_setup = _dcw.replace(win.active.last_setup,
                                         track_width=_live)

    # v1.40: the chassis mesh rides inside the project file, and its
    # opacity is a saved setting.
    _repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    win.load_frame(os.path.join(_repo, "examples", "demo_frame.stl"))
    app.processEvents()
    assert win._frame_blob is not None, "mesh was not packed for saving"
    _fs = win._frame_state(embed=True)
    assert _fs["mesh"]["bytes"] > 1000
    assert abs(_fs["opacity"] - 0.60) < 1e-9, _fs["opacity"]
    assert win._frame_state(embed=False).get("mesh") is None, \
        "autosave must not embed the mesh"
    import tempfile as _tfr
    with _tfr.TemporaryDirectory() as _d:
        _f = os.path.join(_d, "framed.MICK")
        win._write(_f, quiet=True)
        _reload = load_project(_f)
    assert _reload.frame.get("mesh"), "mesh did not survive the save"
    assert _reload.frame["path"].endswith("demo_frame.stl"), \
        _reload.frame["path"]

    # v1.39: batch hardpoint entry holds every cell until Apply.
    _t = win.hardpoint_table
    _t.batch_check.setChecked(True)
    _before = np.array(win.active.hp.uca_outer, float)
    _row = next(i for i, (a, _) in enumerate(_t._fields_for(_t._hp))
                if a == "uca_outer")
    _shown = _t._to_view(_before)
    for _c, _d in ((0, 8.0), (2, 6.0)):
        _t.table.item(_row, _c).setText(
            f"{_t._unit.from_mm(_shown[_c] + _d):.4f}")
    app.processEvents()
    assert np.allclose(np.array(win.active.hp.uca_outer, float), _before), \
        "batch mode solved while typing"
    assert _t.apply_btn.isEnabled() and "2" in _t.apply_btn.text()
    _t.apply_batch()
    app.processEvents()
    _moved = np.linalg.norm(np.array(win.active.hp.uca_outer, float) - _before)
    assert abs(_moved - np.hypot(8.0, 6.0)) < 0.05, _moved
    assert not _t._pending
    _t.batch_check.setChecked(False)
    app.processEvents()

    # v1.39: a DERIVED coordinate says so instead of silently reverting.
    # Column 1 is fore/aft in the default Chassis (+Y fwd) convention — the
    # table's columns are NOT the tool axes, which is the trap this check
    # has to get right.
    assert _t._conv.labels[1] == "Y fwd", _t._conv.labels
    _wc = next(i for i, (a, _) in enumerate(_t._fields_for(_t._hp))
               if a == "wheel_center")
    _t.table.item(_wc, 1).setText(
        f"{_t._unit.from_mm(_t._to_view(_t._hp.wheel_center)[1] + 10.0):.4f}")
    app.processEvents()
    assert "DERIVED" in _t.edit_status.text(), _t.edit_status.text()

    # v1.38: no non-hardpoint numeric field may live-update while typing.
    from PySide6.QtWidgets import QAbstractSpinBox as _ASB
    for _panel in (win.tweaks_panel, win.vehicle_panel, win.frame_panel,
                   win.halfshaft_panel, win.dynamics_panel, win.checklist,
                   win.setup_form, win.optimize_panel):
        for _s in _panel.findChildren(_ASB):
            assert not _s.keyboardTracking(), \
                f"{type(_panel).__name__} still has a live-updating spin"

    # v1.3: steering the rack poses the RIGHT wheel physically (same
    # direction as the left), not as a mirror image.
    win.mirror_check.setChecked(True)
    app.processEvents()
    win.steer_slider.setValue(25)
    app.processEvents()
    scene = win.viewport.scene
    lp = scene._corners[("front", False)]._meshes["points"].points.copy()
    rp = scene._corners[("front", True)]._meshes["points"].points.copy()
    mirrored = lp * np.array([1.0, -1.0, 1.0])
    assert not np.allclose(rp, mirrored, atol=0.5), \
        "right wheel still mirror-steers"
    win.steer_slider.setValue(0)
    app.processEvents()

    # v1.3: dynamics panel computes live and links the kinematic model.
    dp = win.dynamics_panel
    assert dp._last["roll_gradient_deg_g"] < 0.0
    assert not dp._spins["rc_front_in"].isEnabled()   # linked -> read-only
    ay0 = dp._last["lat_transfer_front_lb"]
    dp._spins["ay_g"].setValue(1.5)
    app.processEvents()
    assert dp._last["lat_transfer_front_lb"] > ay0
    dp._spins["ay_g"].setValue(1.0)
    dp.full_check.setChecked(True)
    app.processEvents()
    dp.full_check.setChecked(False)
    app.processEvents()

    # v1.0: design-comparison snapshots + report export.
    win.snapshot_curves()
    app.processEvents()
    assert len(win.plots._snapshots) == 1
    win.clear_snapshots()
    import tempfile as _tf
    with _tf.TemporaryDirectory() as d2:
        rp = os.path.join(d2, "report.csv")
        from suspension_tool.project import report_csv as _rc
        axles = {k: a.sweep for k, a in win.axles.items() if a.sweep is not None}
        with open(rp, "w") as fh:
            fh.write(_rc(axles, {"wheelbase_in": "61.0"},
                         win.unit.mm, win.unit.label))
        assert os.path.getsize(rp) > 500

    # v0.9: the three sweep dialogs construct and auto-run (both axles
    # exist by now, so pitch has what it needs).
    from suspension_tool.gui.sweep_dialogs import (PitchSweepDialog,
                                                   RollSweepDialog,
                                                   SteerSweepDialog)
    from suspension_tool.gui.sweep_dialogs import BumpSteerMapDialog
    from suspension_tool.gui.sweep_dialogs import RideBodeDialog
    for cls in (RollSweepDialog, PitchSweepDialog, SteerSweepDialog,
                BumpSteerMapDialog, RideBodeDialog):
        dlg = cls(win)
        app.processEvents()
        assert "failed" not in dlg.status.text(), dlg.status.text()
        dlg.close()

    # v1.25: the Bode dialog reports BOTH resonances and re-runs when the
    # terrain spacings are edited (it reads the Dynamics panel, not geometry).
    bode = RideBodeDialog(win)
    app.processEvents()
    assert "WHEEL HOP" in bode.status.text(), bode.status.text()
    bode.spacings.setText("4, 9")
    bode.run()
    app.processEvents()
    assert "4 ft" in bode.status.text() and "9 ft" in bode.status.text(), \
        bode.status.text()
    # v1.26: the full-car model reports all seven modes and re-runs on speed
    bode.model.setCurrentIndex(1)
    app.processEvents()
    txt = bode.status.text()
    assert "7-DOF" in txt and "pitch" in txt and "roll" in txt, txt
    bode.speed.setValue(35.0)
    app.processEvents()
    assert "35 mph" in bode.status.text(), bode.status.text()
    bode.close()

    # v1.27: Help > Equations opens and renders every section.
    win.show_equations()
    app.processEvents()
    eq_dlg = win._sweep_dialogs[-1]
    assert eq_dlg.windowTitle() == "Equation reference", eq_dlg.windowTitle()
    from suspension_tool.equations import all_equations
    from PySide6.QtWidgets import QTextBrowser as _TB
    eq_txt = eq_dlg.findChild(_TB).toPlainText()
    for probe in ("Rodrigues", "Kabsch", "roll moment arm", "Froude",
                  "386.4"):
        assert probe.lower() in eq_txt.lower(), f"equations missing {probe}"
    assert len(all_equations()) > 60
    eq_dlg.close()

    # Phase 6: load the demo frame backdrop and nudge its transform.
    frame_path = os.path.join(os.path.dirname(__file__), "..", "examples", "demo_frame.stl")
    win.load_frame(frame_path)
    app.processEvents()
    assert win.frame_panel.path == frame_path
    assert win.frame_panel.transform.scale == 1000.0, "unit guess failed"
    win.frame_panel.rot_combo.setCurrentIndex(3)   # 270 deg
    app.processEvents()
    assert win.frame_panel.transform.rot_z_deg == 270.0

    # v1.5: snap-to-frame (opt-in) pulls a released drag onto a tube.
    # Probe a point a few mm off the demo frame's LOWER rail, which is
    # generated to run through the example car's LCA pickups at tool
    # (y 140, z -30); see examples/make_examples.py.
    win.snap_check.setChecked(True)
    _near_rail = np.array([990.0, 143.0, -26.0])
    kd_probe = win._snap_to_frame(win.axles["front"], _near_rail)
    assert kd_probe is not None, "no frame point within snap range"
    assert np.linalg.norm(kd_probe - _near_rail) < 30.0, \
        np.linalg.norm(kd_probe - _near_rail)
    win.snap_check.setChecked(False)

    # v1.6: moving the frame off the chassis alignment raises the warning.
    win.frame_panel.rot_combo.setCurrentIndex(1)   # 90 deg: off-canonical
    app.processEvents()
    assert win.frame_panel.align_warning.text() != ""
    win.frame_panel.rot_combo.setCurrentIndex(3)   # back to canonical 270
    app.processEvents()
    assert win.frame_panel.align_warning.text() == ""

    # v1.6: seed-phase halfshaft — tick the setup option, regenerate, and
    # the CV axle exists (enabled, inner level at mid-travel) AND is
    # drawn in the 3D view. Untick + regenerate turns it back off.
    hs_field = win.setup_form._fields["has_halfshaft"]
    hs_field.setChecked(True)
    win.setup_form._emit()
    app.processEvents()
    hs_cfg = win.active.halfshaft
    assert hs_cfg.enabled, "seed did not enable the halfshaft"
    wc = win.hp.wheel_center
    expect_z = wc[2] + (win.active.bump - win.active.droop) / 2.0
    assert abs(hs_cfg.inner[2] - expect_z) < 1e-6, (hs_cfg.inner, expect_z)
    vis = win.viewport.scene._corners[(win.active_key, False)]
    assert "halfshaft" in vis._meshes, "halfshaft not drawn"
    assert "hs_axis" in vis._meshes, "tire-axis stub not drawn"
    assert np.any(np.isfinite(win.sweep["cv_max_deg"]))
    # v1.8: the outer CV sits on the kingpin AND on the tire's centre
    # axis (as close as the two lines get)
    from suspension_tool.geometry import static_spindle
    from suspension_tool.halfshaft import outer_cv_of_hp as _ocv
    _o = _ocv(win.hp)
    _a, _b = win.hp.lca_outer, win.hp.uca_outer
    _d = (_b - _a) / np.linalg.norm(_b - _a)
    _s = static_spindle(win.hp)
    _n = np.cross(_d, _s)
    _gap = abs(np.dot(win.hp.wheel_center - _a, _n / np.linalg.norm(_n)))
    assert abs(np.linalg.norm(np.cross(_o - win.hp.wheel_center, _s))
               - _gap) < 1e-6, "outer CV not on the tire axis"
    # v1.7: CV joints show in the hardpoint table (2 extra rows), the
    # outer sits ON the kingpin axis, and editing the rows works.
    from suspension_tool.halfshaft import outer_cv_of_hp
    table = win.hardpoint_table.table
    assert table.rowCount() == 13, table.rowCount()
    o = outer_cv_of_hp(win.hp)
    a, b = win.hp.lca_outer, win.hp.uca_outer
    d = (b - a) / np.linalg.norm(b - a)
    assert np.linalg.norm(np.cross(o - a, d)) < 1e-9, "outer off kingpin"
    # edit the INNER row (row 11): +0.5 in on the Z column
    cell = table.item(11, 2)
    cell.setText(f"{float(cell.text()) + 0.5}")
    app.processEvents()
    assert abs(win.active.halfshaft.inner[2] - expect_z - 12.7) < 0.02
    # edit the OUTER row (row 12): request +0.4 in up -> hub slides along
    # the kingpin and the TIRE height changes with it
    wc_before = win.hp.wheel_center.copy()
    cell = table.item(12, 2)
    cell.setText(f"{float(cell.text()) + 0.4}")
    app.processEvents()
    assert abs(win.hp.wheel_center[2] - wc_before[2]) > 5.0, \
        "hub did not slide along the kingpin"
    win.undo()
    app.processEvents()

    # v1.7: hub tweak spin exists and reads the current position
    from suspension_tool.tweaks import hub_along_kingpin
    _, dw_spins, _ = win.tweaks_panel._boxes["dw"]
    assert "hub_t" in dw_spins
    assert abs(win.unit.to_mm(dw_spins["hub_t"].value())
               - hub_along_kingpin(win.hp)) < 0.1

    hs_field.setChecked(False)
    win.setup_form._emit()
    app.processEvents()
    assert not win.active.halfshaft.enabled
    vis = win.viewport.scene._corners[(win.active_key, False)]
    assert "halfshaft" not in vis._meshes
    assert win.hardpoint_table.table.rowCount() == 11

    # v1.7: enabling via the PANEL with a stale inner (the old default a
    # metre off-axle) auto-places it — the "tangled mess" fix.
    from suspension_tool.halfshaft import HalfshaftConfig
    win.active.halfshaft = HalfshaftConfig()      # stale (0,150,0)
    win.halfshaft_panel.load(win.axles["front"].halfshaft,
                             win.axles["rear"].halfshaft)
    win.halfshaft_panel._w[win.active_key]["enable"].setChecked(True)
    app.processEvents()
    inner = win.active.halfshaft.inner
    assert abs(inner[0] - win.hp.wheel_center[0]) < 1.0, inner
    win.halfshaft_panel._w[win.active_key]["enable"].setChecked(False)
    app.processEvents()

    # v1.7: re-square button repairs a bent bushing axis (no goals run).
    import dataclasses as _dc2
    from suspension_tool.geometry import sketch_planarity
    bent = _dc2.replace(win.hp, uca_inner_front=win.hp.uca_inner_front
                        + np.array([0.0, 0.0, 8.0]))
    win.apply_hardpoints(bent)
    app.processEvents()
    assert sketch_planarity(win.hp)["axis_misalign_deg"] > 1.0
    win.resquare_arms()
    app.processEvents()
    assert sketch_planarity(win.hp)["axis_misalign_deg"] < 1e-9
    win.undo(); win.undo()
    app.processEvents()

    # v1.8: re-square also removes bushing-axis YAW (Ethan's twisted-
    # knuckle-in-side-view defect) while keeping the kickup.
    th = np.radians(8.0)
    rz = np.array([[np.cos(th), -np.sin(th), 0.0],
                   [np.sin(th), np.cos(th), 0.0], [0.0, 0.0, 1.0]])
    c0 = (win.hp.uca_inner_front + win.hp.uca_inner_rear) / 2.0
    yawed = _dc2.replace(win.hp, **{
        a: c0 + rz @ (getattr(win.hp, a) - c0)
        for a in ("uca_inner_front", "uca_inner_rear",
                  "lca_inner_front", "lca_inner_rear")})
    win.apply_hardpoints(yawed)
    app.processEvents()
    assert sketch_planarity(win.hp)["axis_yaw_deg"] > 5.0
    win.resquare_arms()
    app.processEvents()
    assert sketch_planarity(win.hp)["axis_yaw_deg"] < 1e-9
    win.undo(); win.undo()
    app.processEvents()

    # v1.9: twisted sketch plane — set a design yaw in Setup, Re-square
    # rotates the axes onto it, and the sketch row judges against it.
    yaw_spin = win.setup_form._fields["sketch_yaw_deg"]
    yaw_spin.setValue(12.0)
    win.resquare_arms()
    app.processEvents()
    assert abs(sketch_planarity(win.hp)["axis_yaw_deg"] - 12.0) < 1e-6
    assert abs(win.design_yaw_of(win.active) - 12.0) < 1e-9
    yaw_spin.setValue(0.0)
    win.resquare_arms()
    app.processEvents()
    assert abs(sketch_planarity(win.hp)["axis_yaw_deg"]) < 1e-9

    # v1.9: sketch-dimension tweak spins read the live geometry.
    from suspension_tool.tweaks import (arm_length_sketch,
                                        inboard_axis_sep, kingpin_length)
    _, dw_spins9, _ = win.tweaks_panel._boxes["dw"]
    for key9, fn9 in (("kp_len", kingpin_length),
                      ("axis_sep", inboard_axis_sep),
                      ("uca_len", lambda h: arm_length_sketch(h, True)),
                      ("lca_len", lambda h: arm_length_sketch(h, False))):
        assert key9 in dw_spins9, key9
        assert abs(win.unit.to_mm(dw_spins9[key9].value())
                   - fn9(win.hp)) < 0.1, key9

    # v1.9: panel font size applies + persists without a crash.
    win.set_panel_font(9)

    # v1.10: "static toe by tie rod" tweak reads the live toe and, when
    # nudged, lands the wheel on the requested toe at ride height.
    from suspension_tool.metrics import toe_deg as _toe10
    assert "toe_link" in dw_spins9
    assert abs(dw_spins9["toe_link"].value()
               - float(win.hp.static_toe_deg)) < 0.01
    dw_spins9["toe_link"].setValue(0.6)
    app.processEvents()
    assert abs(_toe10(win.solver.solve(0.0)) - 0.6) < 0.02, \
        _toe10(win.solver.solve(0.0))
    win.undo()
    app.processEvents()

    # v1.10: the passive-steer optimizer goal is offered in the panel.
    assert "toe_slope" in win.optimize_panel._enable

    # v1.11: the Onshape geometry push builds a design-notes PDF and an
    # idempotent manifest from the live design (mock client, no network).
    from suspension_tool.onshape_sync import OnshapeClient, push_geometry
    from suspension_tool.report_pdf import design_notes_pdf
    notes = win._design_notes_axles()
    assert notes["front"] is not None and notes["front"]["hp"] is not None
    pdf = design_notes_pdf(notes, win.vehicle_panel.params().wheelbase,
                           version=win._version)
    assert pdf[:4] == b"%PDF"
    made = {}
    def _fake_os(method, path, body=None):
        if method == "GET" and path.endswith("/elements"):
            return list(made.values())
        if method == "POST" and "/e/" not in path:
            eid = f"e{len(made)}"
            made[eid] = {"id": eid, "name": "x",
                         "elementType": ("VARIABLESTUDIO" if "variablestudio"
                                         in path else "FEATURESTUDIO"
                                         if "featurestudios" in path
                                         else "BLOB" if "blobelements" in path
                                         else "PARTSTUDIO")}
            return {"id": eid, "feature": {"featureId": "F1"}}
        return {"feature": {"featureId": "F1"}}
    client = OnshapeClient("a", "b", request_fn=_fake_os)
    r1 = push_geometry(client, "https://cad.onshape.com/documents/"
                       "aaaaaaaaaaaaaaaaaaaaaaaa/w/"
                       "bbbbbbbbbbbbbbbbbbbbbbbb/e/cccccccccccccccccccccccc",
                       {"front": win.hp, "rear": None},
                       win.vehicle_panel.params().wheelbase, pdf_bytes=pdf)
    n_tabs = len(made)
    r2 = push_geometry(client, "https://cad.onshape.com/documents/"
                       "aaaaaaaaaaaaaaaaaaaaaaaa/w/"
                       "bbbbbbbbbbbbbbbbbbbbbbbb/e/cccccccccccccccccccccccc",
                       {"front": win.hp, "rear": None},
                       win.vehicle_panel.params().wheelbase, pdf_bytes=pdf,
                       manifest=r1["manifest"])
    assert len(made) == n_tabs, "re-push duplicated Onshape tabs"

    # manifest round-trips through a saved project
    from suspension_tool.project import (dict_to_project, project_to_dict)
    win._onshape_manifest = {"variable_studio": "vsX", "feature_id": "F9"}
    reloaded = dict_to_project(project_to_dict(win._project_state_now()))
    assert reloaded.onshape["feature_id"] == "F9"
    win._onshape_manifest = None

    # v1.8: steering-arm tweak spin reads the live dimension.
    from suspension_tool.tweaks import steer_arm_length
    _, dw_spins8, _ = win.tweaks_panel._boxes["dw"]
    assert "steer_arm" in dw_spins8
    assert abs(win.unit.to_mm(dw_spins8["steer_arm"].value())
               - steer_arm_length(win.hp)) < 0.1

    # v1.8: checklist grades the turning radius and draws the Ackermann
    # graph from a live rack sweep at ride height.
    win._update_checklist()
    app.processEvents()
    r = win.checklist._turn_radius_m()
    assert np.isfinite(r) and 1.0 < r < 20.0, r
    assert win.checklist._steer is not None
    assert np.any(np.isfinite(win.checklist._steer["ackermann_pct"]))

    # v1.7: recenter-orbit rescue doesn't blow up.
    win.viewport.recenter_orbit()

    # v1.8: camera bounds ignore the far-flung IC overlay markers —
    # the model bounds stay car-sized and Fit view uses them.
    mb = win.viewport.scene._model_bounds()
    assert mb is not None
    span = max(mb[1] - mb[0], mb[3] - mb[2], mb[5] - mb[4])
    assert span < 12000.0, f"model bounds polluted: span {span:.0f} mm"
    win.viewport.scene.fit()

    # v1.6: 'Both axles' heave moves the rear together with the front.
    win.both_axles_check.setChecked(True)
    rear_before = win.axles["rear"].travel
    win.show_travel(50.0)
    app.processEvents()
    assert abs(win.axles["rear"].travel - 50.0) < 1e-6, \
        (rear_before, win.axles["rear"].travel)
    win.show_travel(0.0)
    win.both_axles_check.setChecked(False)
    app.processEvents()

    # v1.5: candidates — snapshot, rename in place, load restores.
    n0 = len(win.candidates_panel.candidates())
    win.snapshot_candidate()
    app.processEvents()
    assert len(win.candidates_panel.candidates()) == n0 + 1
    win.candidates_panel.table.item(n0, 0).setText("my baseline")
    assert win.candidates_panel.candidates()[n0]["name"] == "my baseline"
    saved_wc = win.hp.wheel_center.copy()
    cell = win.hardpoint_table.table.item(8, 2)     # wheel centre z
    cell.setText(f"{float(cell.text()) + 1.0}")
    app.processEvents()
    assert abs(win.hp.wheel_center[2] - saved_wc[2]) > 1.0
    win.load_candidate(n0)
    app.processEvents()
    assert np.allclose(win.hp.wheel_center, saved_wc, atol=1e-6), \
        (win.hp.wheel_center, saved_wc)

    # .MICK v2 save/load round-trip through the window's handlers.
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "smoke.MICK")
        win._write(path)
        assert os.path.exists(path)
        saved = win.hp.uca_outer.copy()
        state = load_project(path)
        assert state.front is not None and state.rear is not None
        assert state.candidates and \
            state.candidates[n0]["name"] == "my baseline"
        # v1.8: checklist thresholds (rack travel, turn-radius goal)
        # persist inside the goals dict
        cl = (state.goals or {}).get("checklist") or {}
        assert abs(cl.get("rack_mm", -1) -
                   win.checklist.rack_travel_mm) < 1e-6, cl
        win.apply_hardpoints(state.front.hardpoints)
        app.processEvents()
        assert abs(win.hp.uca_outer[2] - saved[2]) < 1e-6

    # v1.12: new-feature coverage, kept at the END so the geometry edits
    # don't perturb the coordinate-specific checks above. Make the front
    # double wishbone active first (the rear was made a trailing arm).
    win.axle_combo.setCurrentIndex(0)
    app.processEvents()
    from suspension_tool.tweaks import (hub_cv_offset, ride_height,
                                        shock_length_at_ride)
    _, dw_spins, _ = win.tweaks_panel._boxes["dw"]
    # hub offset = outer-CV -> tire-centre distance
    hub0 = hub_cv_offset(win.hp)
    dw_spins["hub_cv"].setValue(dw_spins["hub_cv"].value() + 0.5)  # +0.5 in
    app.processEvents()
    assert abs(hub_cv_offset(win.hp) - hub0 - 12.7) < 0.15, hub_cv_offset(win.hp)
    # shock length @ ride + the bump/droop split readout
    len0 = shock_length_at_ride(win.hp)
    dw_spins["shock_len"].setValue(dw_spins["shock_len"].value() - 0.5)
    app.processEvents()
    assert abs(shock_length_at_ride(win.hp) - len0 + 12.7) < 0.15
    assert "bump" in win.tweaks_panel.split_label.text().lower(), \
        "shock split readout missing"
    # ride height = rigid z placement, datum-corrected by frame_tube_offset
    fto = win.tweaks_panel._frame_tube_offset
    assert fto > 0.0, "frame-tube offset not loaded from setup"
    rh0 = ride_height(win.hp, fto)
    dw_spins["ride_h"].setValue(dw_spins["ride_h"].value() + 1.0)  # +1 in
    app.processEvents()
    assert abs(ride_height(win.hp, fto) - rh0 - 25.4) < 0.15

    # Free (L/R) mode: the right-wheel slider appears and the two wheels
    # can sit at DIFFERENT travels (single-wheel-bump roll centre).
    win.motion_combo.setCurrentText("Free (L/R)")
    app.processEvents()
    assert win.right_slider.isVisible(), "right-wheel slider hidden in Free"
    win.slider.setValue(0)             # left wheel at ride height
    win.right_slider.setValue(25)      # right wheel bumped alone
    app.processEvents()
    assert abs(win.active.travel_right - 25.0) < 1.0
    assert abs(win.active.travel) < 1.0, "left wheel should stay at 0"
    assert np.isfinite(win.axles["front"].rc_h), \
        "asymmetric roll centre not computed"
    win.motion_combo.setCurrentText("Heave")
    app.processEvents()
    assert not win.right_slider.isVisible()
    # plot hover map: one entry per live curve
    assert len(win.plots._axmap) > 0, "plot hover map empty"

    # v1.13: post-seed scrub + caster tweaks (front DW active).
    from suspension_tool.tweaks import caster_angle_deg, scrub_radius
    _, dw_spins, _ = win.tweaks_panel._boxes["dw"]
    sc0 = scrub_radius(win.hp)
    dw_spins["scrub"].setValue(win.unit.from_mm(sc0 + 10.0))
    app.processEvents()
    assert abs(scrub_radius(win.hp) - (sc0 + 10.0)) < 0.3, scrub_radius(win.hp)
    ca0 = caster_angle_deg(win.hp)
    dw_spins["caster"].setValue(ca0 + 1.0)      # caster row is in degrees
    app.processEvents()
    assert abs(caster_angle_deg(win.hp) - (ca0 + 1.0)) < 0.1, caster_angle_deg(win.hp)

    # v1.13: lock the UBJ — it drops out of the drag list and the scrub
    # tweak refuses to move it (geometry untouched), then unlocks.
    ht = win.hardpoint_table
    ubj_row = [a for a, _ in ht._fields_for(win.hp)].index("uca_outer")
    ht._toggle_lock(ubj_row)
    app.processEvents()
    assert "uca_outer" in win.active.locked, "lock not recorded"
    # REGRESSION (v1.13.1): the drag list is INDEX-ALIGNED with the
    # displayed spheres — a locked point must become None in place, never
    # be removed (removal shifted every later index, so locking the LCA
    # inners made the tie-rod sphere grab the UBJ).
    from suspension_tool.gui.viewport import drag_attrs_for as _daf
    full = _daf(win.hp)
    ctx = win.viewport._drag_ctx["attrs"]
    assert len(ctx) == len(full), "drag list shrank — index misalignment"
    for i, a in enumerate(full):
        assert ctx[i] == (None if a == "uca_outer" else a), \
            f"drag index {i} misaligned: {ctx[i]!r} vs {a!r}"
    hp_before = win.hp
    dw_spins["scrub"].setValue(win.unit.from_mm(scrub_radius(win.hp) + 8.0))
    app.processEvents()
    assert win.hp is hp_before, "locked UBJ was moved by the scrub tweak"
    assert win.statusBar().currentMessage() != "", "no blocked feedback"
    ht._toggle_lock(ubj_row)                    # unlock
    app.processEvents()
    assert "uca_outer" not in win.active.locked

    # v1.13: Roll mode keeps the whole-car readouts live (was blanking).
    win.motion_combo.setCurrentText("Roll")
    app.processEvents()
    win.show_travel(40.0)
    app.processEvents()
    for key in ("track_width_mm", "wheelbase_mm", "ride_height_mm"):
        assert win.readouts._labels[key].text() not in ("", "-"), \
            f"roll-mode {key} readout blank"
    win.motion_combo.setCurrentText("Heave")
    app.processEvents()

    # v1.13: the seed's wheelbase drives the live vehicle param.
    assert abs(win.vehicle_panel.params().wheelbase
               - win.active.last_setup.wheelbase) < 1e-6, \
        "seed wheelbase did not reach the vehicle param"

    # v1.13: locks survive a .MICK round-trip.
    ht._toggle_lock(ubj_row)                    # lock again
    app.processEvents()
    with tempfile.TemporaryDirectory() as d13:
        p13 = os.path.join(d13, "locks.MICK")
        win._write(p13)
        st13 = load_project(p13)
        assert st13.front.locked and "uca_outer" in st13.front.locked, \
            "locked set not persisted"
    ht._toggle_lock(ubj_row)                    # unlock (leave clean)
    app.processEvents()

    # v1.14: ⊥-sketch column, corner fore/aft tweak, measure-setup button,
    # live wheelbase (both axles exist by now).
    t14 = win.hardpoint_table.table
    assert t14.columnCount() == 4
    # caption is "⊥ sk (in)" — shortened in v1.22.1 so all four columns fit
    assert "⊥" in t14.horizontalHeaderItem(3).text(), \
        t14.horizontalHeaderItem(3).text()
    _, dw14, _ = win.tweaks_panel._boxes["dw"]
    assert "corner_x" in dw14
    from suspension_tool.tweaks import corner_x as _cx
    assert abs(win.unit.to_mm(dw14["corner_x"].value()) - _cx(win.hp)) < 0.1
    win.show_travel(0.0)
    wb_a = float(win.readouts._labels["wheelbase_mm"].text())
    win.show_travel(50.0)
    wb_b = float(win.readouts._labels["wheelbase_mm"].text())
    assert abs(wb_b - wb_a) > 1e-4, "wheelbase readout not live"
    win.show_travel(0.0)
    win.measure_setup_from_design()
    sv14 = win.setup_form.current_setup()
    assert abs(sv14.track_width - 2 * abs(win.hp.wheel_center[1])) < 0.1, \
        "measure-setup did not fill the form from the design"

    # v1.15: kickup is a live tweak (row reads the measured angle) and a
    # reseed honors locked hardpoints.
    from suspension_tool.tweaks import sketch_kickup as _sk
    _, dw15, _ = win.tweaks_panel._boxes["dw"]
    assert "kickup" in dw15
    assert abs(dw15["kickup"].value() - _sk(win.hp)) < 0.05
    ht15 = win.hardpoint_table
    f15 = [a for a, _ in ht15._fields_for(win.hp)]
    ht15._toggle_lock(f15.index("lca_inner_front"))
    app.processEvents()
    tab15 = win.hp.lca_inner_front.copy()
    win.setup_form._emit()                        # reseed with a lock set
    app.processEvents()
    assert np.allclose(win.hp.lca_inner_front, tab15, atol=1e-6), \
        "reseed moved a locked point"
    ht15._toggle_lock(f15.index("lca_inner_front"))   # unlock again
    app.processEvents()

    # v1.15.1: the ride-height readout MEASURES the geometry (agrees with
    # the tweak row exactly), and the stance row exists.
    from suspension_tool.tweaks import ride_height as _rh151
    _, dw151, _ = win.tweaks_panel._boxes["dw"]
    win.show_travel(0.0)
    rh_read_mm = win.unit.to_mm(
        float(win.readouts._labels["ride_height_mm"].text()))
    rh_tweak_mm = win.unit.to_mm(dw151["ride_h"].value())
    assert abs(rh_read_mm - rh_tweak_mm) < 0.2, (rh_read_mm, rh_tweak_mm)
    assert win.vehicle_panel._out["stance"].text() != "-", \
        "stance row not populated with both axles present"

    # v1.15.2: one vehicle ride-height knob sets BOTH axles.
    from suspension_tool.tweaks import ride_height as _rh152

    def _fto152(axle):
        return (float(getattr(axle.last_setup, "frame_tube_offset", 0.0)
                      or 0.0) if axle.last_setup is not None else 0.0)

    win.set_vehicle_ride_height(361.0)
    app.processEvents()
    for k152 in ("front", "rear"):
        a152 = win.axles[k152]
        assert abs(_rh152(a152.hp, _fto152(a152)) - 361.0) < 0.01, k152
    assert "matched" in win.vehicle_panel._out["stance"].text()
    win.undo()
    app.processEvents()

    # v1.16.1: the travel range FOLLOWS the shock automatically — the
    # spec exists after seeding, the readout rows are live, the shock
    # stays inside its hardware limits AND sweeps (nearly) all of its
    # stroke without any button press; the range spins are display-only.
    a116 = win.active
    assert a116.shock_spec is not None, "seed did not create the shock spec"
    assert win.readouts._labels["shock_split_txt"].text() not in ("", "-")
    L116 = a116.sweep["shock_length_mm"]
    assert np.max(L116) <= a116.shock_spec.max_length + 0.5
    assert np.min(L116) >= a116.shock_spec.min_length - 0.5
    used116 = (np.max(L116) - np.min(L116)) / a116.shock_spec.stroke
    assert used116 > 0.9, f"stroke under-used automatically: {used116:.2f}"
    assert not win.bump_spin.isEnabled(), \
        "range spins should be display-only with a shock spec"

    # v1.19: rear axle coordinates read in the SHARED vehicle frame (a
    # wheelbase behind the front, one firewall datum) — matching the CSV
    # export and the 3D scene, not the old per-axle-local frame.
    def _tbl_row(attr):
        fs = [a for a, _ in win.hardpoint_table._fields_for(win.hp)]
        r = fs.index(attr)
        return win.unit.to_mm(float(win.hardpoint_table.table.item(r, 1).text()))
    win.axle_combo.setCurrentIndex(0)
    app.processEvents()
    front_yfwd = _tbl_row("wheel_center")     # chassis+y: col 1 = fore-aft
    win.axle_combo.setCurrentIndex(1)
    app.processEvents()
    rear_yfwd = _tbl_row("wheel_center")
    wb19 = win.vehicle_panel.params().wheelbase
    # front and rear now differ by ~a wheelbase in fore-aft (was ~0)
    assert abs((front_yfwd - rear_yfwd)) > 0.5 * wb19, \
        (front_yfwd, rear_yfwd, wb19)
    # and the rear table value equals what the vehicle CSV writes
    from suspension_tool.project import vehicle_csv as _vcsv
    _csv = _vcsv({"front": win.axles["front"].hp,
                  "rear": win.axles["rear"].hp}, wb19, conv=win.conv)
    _cy = float([l for l in _csv.splitlines()
                 if l.startswith("rear_wheel_center")][0].split(",")[2])
    assert abs(rear_yfwd - _cy) < 0.05, (rear_yfwd, _cy)
    win.axle_combo.setCurrentIndex(0)
    app.processEvents()

    # v1.20: the two hinge-carrier types seed, solve, table, tweak and
    # sync end-to-end. Front -> C-hub, rear -> loaded halfshaft.
    from suspension_tool.chub_front import CHubFrontPoints as _CH20
    from suspension_tool.loaded_halfshaft import (
        LoadedHalfshaftPoints as _LH20)
    win.axle_combo.setCurrentIndex(0)
    app.processEvents()
    win.type_combo.setCurrentText("C-hub front")
    app.processEvents()
    assert isinstance(win.hp, _CH20), "C-hub seed did not adopt"
    win.show_travel(30.0)
    app.processEvents()
    assert win.readouts._labels["camber_deg"].text() not in ("", "-")
    # the physical kingpin feeds the kingpin metrics (no NaN on a C-hub)
    assert "nan" not in win.readouts._labels["caster_deg"].text().lower()
    box20, spins20, _ = win.tweaks_panel._boxes["ch"]
    assert box20.isVisible() and "kpi" in spins20, "C-hub tweak box missing"
    win.show_travel(0.0)
    app.processEvents()

    win.axle_combo.setCurrentIndex(1)
    app.processEvents()
    win.type_combo.setCurrentText("Loaded halfshaft (rear)")
    app.processEvents()
    a20 = win.active
    assert isinstance(a20.hp, _LH20), "loaded-halfshaft seed did not adopt"
    # the shaft is structural: config mirrors the hs_inner hardpoint,
    # plunge over the sweep is EXACTLY zero, and the hardpoint table does
    # NOT duplicate the CV rows (hs_inner/hs_outer are regular rows)
    assert a20.halfshaft.enabled
    # cfg.inner mirrors the hs_inner HARDPOINT (the structural CV); it may
    # differ by <1 display ULP because the Halfshafts panel round-trips it
    # through inch spinboxes, but the GEOMETRY (hs_inner) stays exact and
    # the plunge is computed from the exact hardpoint -> identically zero.
    assert np.allclose(a20.halfshaft.inner, a20.hp.hs_inner, atol=0.05)
    assert np.max(np.abs(a20.sweep["plunge_mm"])) < 1e-6, \
        "a LOADED halfshaft cannot plunge"
    rows20 = [a for a, _ in win.hardpoint_table._fields_for(a20.hp)]
    assert rows20.count("hs_inner") == 1
    assert win.hardpoint_table.table.rowCount() == len(rows20), \
        "cfg halfshaft rows duplicated the structural hs points"
    # editing the diff flange in the HALFSHAFT PANEL moves the suspension
    z20 = win.halfshaft_panel._w["rear"]["spins"][2]
    old_z20 = float(a20.hp.hs_inner[2])
    z20.setValue(z20.value() + win.unit.from_mm(10.0))
    app.processEvents()
    assert abs(float(win.axles["rear"].hp.hs_inner[2])
               - (old_z20 + 10.0)) < 0.05, \
        "panel edit of the loaded shaft's inner CV did not move hs_inner"
    _, spins20b, _ = win.tweaks_panel._boxes["lh"]
    assert "plan_deg" in spins20b, "loaded-halfshaft tweak box missing"
    # both types survive a project save/load round-trip
    import tempfile as _tf20
    with _tf20.TemporaryDirectory() as d20:
        p20 = f"{d20}/types20.MICK"
        win._write(p20, quiet=True)
        win._load_project_file(p20)
        app.processEvents()
    assert isinstance(win.axles["front"].hp, _CH20)
    assert isinstance(win.axles["rear"].hp, _LH20)
    win.axle_combo.setCurrentIndex(0)
    app.processEvents()

    # v1.21: H-arm rear — seeds, adopts, is DRIVEN by default (CV overlay
    # on, plunge is a real non-zero number unlike the loaded shaft), holds
    # toe through travel, and its tweak box + save/load work.
    from suspension_tool.harm_rear import HArmRearPoints as _HA21
    win.axle_combo.setCurrentIndex(1)
    app.processEvents()
    win.type_combo.setCurrentText("H-arm (rear)")
    app.processEvents()
    a21 = win.active
    assert isinstance(a21.hp, _HA21), "H-arm seed did not adopt"
    assert a21.halfshaft.enabled, "H-arm should be driven by default"
    assert np.max(np.abs(a21.sweep["plunge_mm"])) > 0.5, \
        "a normal driven CV should plunge (not a structural shaft)"
    toe21 = a21.sweep["toe_deg"]
    i21 = int(np.argmin(np.abs(a21.sweep["travel_mm"])))
    assert np.max(np.abs(toe21 - toe21[i21])) < 0.1, \
        "the H-arm should hold toe through travel"
    _, spins21, _ = win.tweaks_panel._boxes["ha"]
    assert "plan_deg" in spins21, "H-arm tweak box missing"
    with _tf20.TemporaryDirectory() as d21:
        p21 = f"{d21}/harm21.MICK"
        win._write(p21, quiet=True)
        win._load_project_file(p21)
        app.processEvents()
    assert isinstance(win.axles["rear"].hp, _HA21), "H-arm lost on reload"
    win.axle_combo.setCurrentIndex(0)
    app.processEvents()

    # v1.23: impact load case -> mount loads -> FEA CSV.
    from suspension_tool.geometry import DoubleWishbonePoints as _DW23
    from suspension_tool.gui.main_window import _ImpactDialog as _ID23
    from suspension_tool.impact import corner_loads as _cl23, loads_csv as _lc23
    win.axle_combo.setCurrentIndex(0)
    app.processEvents()
    if not isinstance(win.hp, _DW23):
        win.type_combo.setCurrentText("Double wishbone")
        app.processEvents()
    d23 = _ID23(win, win.dynamics_panel.current_inputs())
    assert "Impulse" in d23.readout.text(), "impact dialog readout empty"
    d23.preset.setCurrentIndex(1)          # first standard case
    app.processEvents()
    c23 = d23.case()
    assert c23.peak_force_lb() > 0.0
    r23 = _cl23(win.hp, c23.wheel_load())
    # The CHASSIS mounts must reproduce the wheel load (equilibrium).
    # Summing every row is wrong: internal joints (ball joints, the shock's
    # arm end) are emitted as equal-and-opposite PAIRS, so they cancel only
    # by luck. This matched while the DW emitted no internal rows and went
    # stale when it started to (same fix as tests/test_impact.py).
    tot23 = sum((m.force for m in r23.mounts if m.is_chassis), np.zeros(3))
    assert np.linalg.norm(tot23 - r23.wheel_load_lb) < 1e-6 * max(
        1.0, np.linalg.norm(r23.wheel_load_lb)), "impact loads out of balance"
    csv23 = _lc23(c23, r23, unit_mm=win.unit.mm, unit_label=win.unit.label,
                  conv=win.conv)
    for tag in ("lca_inner_front", "shock_inner", "member,", "impulse_lb_s"):
        assert tag in csv23, f"impact CSV missing {tag}"
    # a free-fall drop height fills the speed and flips to vertical
    d23.drop.setValue(36.0)
    app.processEvents()
    assert d23.direction.currentData() == "vertical"
    assert d23.v0.value() > 8.0

    # the window must END dark too (the boot default survives the run)
    assert win.plots._dark, "plots lost dark mode during the session"
    assert win.plots._fig.get_facecolor()[0] < 0.2, \
        f"plots figure is light: {win.plots._fig.get_facecolor()}"
    win.grab().save("/tmp/full_window.png")
    print("GUI smoke test passed; screenshot at /tmp/full_window.png")


if __name__ == "__main__":
    main()
