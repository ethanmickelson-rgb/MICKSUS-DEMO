"""Steering effort: rack force and driver effort (v1.43).

Built to size the bolted joint that pins onto the rack, so the number
that matters is a FORCE and it had better be right. The headline test
here is a virtual-work check: perturb the rack in the actual solver,
measure how far the knuckle turns about its kingpin, and confirm that
    F_rack * d(rack) == M_kingpin * d(theta)
That route never touches the implementation's formula -- it only uses
the solver and conservation of energy -- so agreement is real evidence
rather than the code agreeing with itself.
"""

import os
import unittest

import numpy as np

from suspension_tool import metrics, steering as S
from suspension_tool.geometry import example_baja_front
from suspension_tool.project import load_project
from suspension_tool.suspension_types import solver_for
from suspension_tool.tire import surface_tire

_DEMO = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "examples", "baja_demo_double_wishbone.MICK")
IN = 25.4


def _kingpin_angle(state) -> float:
    """Rotation of the tie-rod ball about the kingpin axis, radians."""
    k = S.kingpin_axis(state)
    v = np.asarray(state.tro, float) - np.asarray(state.lbj, float)
    v = v - np.dot(v, k) * k
    ref = np.array([1.0, 0.0, 0.0])
    e1 = ref - np.dot(ref, k) * k
    e1 /= np.linalg.norm(e1)
    return float(np.arctan2(np.dot(v, np.cross(k, e1)), np.dot(v, e1)))


class TestVirtualWork(unittest.TestCase):
    """The proof. Energy in at the rack equals energy out at the kingpin."""

    def _case(self, hp):
        sol = solver_for(hp)(hp)
        return sol, sol.solve(0.0)

    def test_rack_force_matches_the_energy_route(self):
        for hp in (example_baja_front(),
                   load_project(_DEMO).front.hardpoints):
            sol, state = self._case(hp)
            h = 0.5
            dtheta = (_kingpin_angle(sol.solve(0.0, +h))
                      - _kingpin_angle(sol.solve(0.0, -h))) / (2.0 * h)
            m = S.kingpin_moment(state, [0.0, -100.0, 0.0])
            energy = abs(m * dtheta)
            arm = S.tierod_effective_arm(state, hp)
            geometry = abs(m / arm) * S.rack_axis_fraction(state, hp)
            self.assertAlmostEqual(energy / geometry, 1.0, places=3,
                                   msg=f"{type(hp).__name__}: {energy} vs "
                                       f"{geometry}")

    def test_the_arm_itself_is_the_reciprocal_of_the_ratio(self):
        """d(theta)/d(rack) must equal rack_fraction / effective_arm --
        the same statement, checked directly on the geometry."""
        hp = load_project(_DEMO).front.hardpoints
        sol, state = self._case(hp)
        h = 0.1
        dtheta = (_kingpin_angle(sol.solve(0.0, +h))
                  - _kingpin_angle(sol.solve(0.0, -h))) / (2.0 * h)
        want = S.rack_axis_fraction(state, hp) / abs(
            S.tierod_effective_arm(state, hp))
        # 1.2e-4, and it CONVERGES to that rather than shrinking with h,
        # so it is not truncation -- see the test below for what it is.
        self.assertLess(abs(abs(dtheta) - want) / want, 1e-3)

    def test_the_residual_is_steering_jack_not_an_error(self):
        """The analytic arm treats the knuckle as rotating about a FIXED
        kingpin. It does not quite: the solver holds wheel-centre HEIGHT
        constant, and steering about an inclined kingpin tries to raise
        the corner, so the arm rotates a little to compensate and the ball
        joints translate. That coupling is the whole residual.

        Two things prove it. The error CONVERGES as the difference step
        shrinks rather than blowing up (truncation would fall as h^2,
        solver noise would grow as 1/h, and neither happens), and the
        ball joints demonstrably move.

        Whether it converges from above or below depends on how inclined
        the kingpin is, so monotonicity is not asserted -- on a car with
        little kingpin lean the jack term is small and other terms of
        similar size decide the sign. What must hold is that it stays
        bounded and small: at 1e-4 it is irrelevant for sizing a bolt,
        but it should be named rather than papered over."""
        hp = load_project(_DEMO).front.hardpoints
        sol = solver_for(hp)(hp)
        errs = []
        for h in (0.5, 0.1, 0.02):
            state = sol.solve(0.0)
            d = (_kingpin_angle(sol.solve(0.0, +h))
                 - _kingpin_angle(sol.solve(0.0, -h))) / (2.0 * h)
            want = S.rack_axis_fraction(state, hp) / abs(
                S.tierod_effective_arm(state, hp))
            errs.append(abs(abs(d) - want) / want)
        self.assertLess(max(errs), 1e-3,
                        f"residual must stay negligible, got {errs}")
        self.assertLess(errs[-1] / errs[0], 3.0,
                        f"must not grow like 1/h as the step shrinks: {errs}")
        self.assertLess(abs(errs[-1] - errs[1]) / errs[1], 0.30,
                        f"should CONVERGE, not keep moving: {errs}")
        a, b = sol.solve(0.0, -1.0), sol.solve(0.0, 1.0)
        self.assertGreater(
            float(np.linalg.norm(np.asarray(b.lbj, float)
                                 - np.asarray(a.lbj, float))), 0.01,
            "the ball joints must move, or the explanation is wrong")
        self.assertAlmostEqual(float(a.wheel_center[2]),
                               float(b.wheel_center[2]), places=6)


class TestExactVersusTextbook(unittest.TestCase):
    """The 3D cross product against the Fy x trail every book prints."""

    def setUp(self):
        self.hp = load_project(_DEMO).front.hardpoints
        sol = solver_for(self.hp)(self.hp)
        self.state = sol.solve(0.0)
        self.m = metrics.corner_metrics(sol, self.state)

    def test_they_agree_to_a_few_percent_on_a_real_car(self):
        exact = S.kingpin_moment(self.state, [0.0, -100.0, 0.0])
        classic = S.classical_kingpin_moment(100.0,
                                             self.m["caster_trail_mm"])
        self.assertLess(abs(exact - classic) / abs(exact), 0.10)

    def test_the_textbook_form_is_exact_for_a_vertical_kingpin(self):
        """Its assumption, stated: with the kingpin vertical and the force
        horizontal, the cross product reduces to Fy x trail exactly."""
        import dataclasses
        hp = example_baja_front()
        sol = solver_for(hp)(hp)
        st = sol.solve(0.0)
        k = S.kingpin_axis(st)
        # only trust this where the kingpin really is near vertical
        if abs(k[2]) < 0.999:
            self.skipTest("example geometry's kingpin is not vertical")
        exact = S.kingpin_moment(st, [0.0, -100.0, 0.0])
        trail = metrics.corner_metrics(sol, st)["caster_trail_mm"]
        self.assertAlmostEqual(exact,
                               S.classical_kingpin_moment(100.0, trail),
                               places=3)

    def test_the_effective_arm_is_not_the_scalar_steering_arm(self):
        """The scalar length is a perpendicular distance and ignores that
        the tie rod is not perpendicular to the kingpin. Using it would
        UNDER-predict the force, which is the wrong way to be wrong.

        The DIRECTION of the error is the property worth pinning: the
        scalar always over-states the arm, so it always under-states the
        rack force. The SIZE of the gap is geometry -- it depends on how
        far the tie rod sits from perpendicular to the kingpin, and runs
        from a few percent to about 20% across the cars this has been
        measured on. Asserting a fixed 10% was pinning one car's
        knuckle, so only the direction and a non-trivial floor are
        asserted here."""
        from suspension_tool.tweaks import steer_arm_length
        exact = abs(S.tierod_effective_arm(self.state, self.hp))
        scalar = abs(steer_arm_length(self.hp))
        self.assertGreater(scalar, exact,
                           "the scalar must over-state the arm")
        self.assertGreater(scalar / exact - 1.0, 0.02,
                           "and by enough to matter when sizing a joint")


class TestKingpinMoment(unittest.TestCase):

    def setUp(self):
        self.hp = load_project(_DEMO).front.hardpoints
        self.state = solver_for(self.hp)(self.hp).solve(0.0)

    def test_it_is_linear_in_the_force(self):
        a = S.kingpin_moment(self.state, [0.0, -50.0, 0.0])
        b = S.kingpin_moment(self.state, [0.0, -100.0, 0.0])
        self.assertAlmostEqual(b, 2.0 * a, places=9)

    def test_a_force_along_the_kingpin_makes_no_moment(self):
        k = S.kingpin_axis(self.state)
        self.assertAlmostEqual(S.kingpin_moment(self.state, k * 100.0),
                               0.0, places=6)

    def test_vertical_load_acts_through_the_scrub_radius(self):
        """Fz alone still makes a moment, because the contact patch is
        offset from where the kingpin meets the ground."""
        self.assertGreater(
            abs(S.kingpin_moment(self.state, [0.0, 0.0, 300.0])), 1.0)

    def test_pneumatic_trail_adds_to_the_moment(self):
        """It EXTENDS the mechanical trail, so it must make the steering
        heavier. Got this backwards first time by adding a scalar Mz whose
        sense I reasoned about instead of measuring -- the term subtracted
        and a trail made the rack lighter, which is nonsense."""
        bare = abs(S.kingpin_moment(self.state, [0.0, -100.0, 0.0]))
        for tp in (5.0, 20.0, 40.0):
            m = abs(S.kingpin_moment(self.state, [0.0, -100.0, 0.0],
                                     pneumatic_trail=tp))
            self.assertGreater(m, bare, f"trail {tp} mm made it lighter")
            bare = m

    def test_the_trail_shift_is_along_the_wheel_heading(self):
        f = S.wheel_forward(self.state)
        self.assertAlmostEqual(float(np.linalg.norm(f)), 1.0, places=9)
        self.assertAlmostEqual(float(f[2]), 0.0, places=9)
        self.assertGreater(float(f[0]), 0.9, "should point forward")


class TestCorneringEffort(unittest.TestCase):

    def setUp(self):
        self.hp = load_project(_DEMO).front.hardpoints
        self.state = solver_for(self.hp)(self.hp).solve(0.0)
        self.tire = surface_tire("dry_hardpack")

    def _eff(self, loads=(331.6, 88.2), **kw):
        return S.cornering_effort(self.hp, self.state, loads, self.tire, **kw)

    def test_it_produces_a_sane_rack_force_for_mk4(self):
        eff = self._eff()
        self.assertGreater(eff.rack_force_lb, 50.0)
        self.assertLess(eff.rack_force_lb, 400.0)
        self.assertAlmostEqual(eff.rack_force_n,
                               eff.rack_force_lb * 4.4482216152605, places=6)

    def test_both_wheels_contribute(self):
        eff = self._eff()
        self.assertEqual(len(eff.corners), 2)
        self.assertAlmostEqual(
            eff.rack_force_lb,
            sum(c.rack_share_lb for c in eff.corners), places=9)

    def test_grip_is_load_sensitive_so_mu_is_higher_inside(self):
        """The lightly loaded inside tire has the HIGHER friction
        coefficient -- the whole reason load transfer costs grip."""
        out, ins = self._eff().corners
        self.assertGreater(out.fz_lb, ins.fz_lb)
        self.assertGreater(ins.mu, out.mu)

    def test_a_lifted_wheel_contributes_nothing(self):
        eff = self._eff(loads=(420.0, 0.0))
        self.assertEqual(eff.corners[1].fy_lb, 0.0)
        self.assertEqual(eff.corners[1].rack_share_lb, 0.0)

    def test_negative_load_is_clamped_not_propagated(self):
        eff = self._eff(loads=(420.0, -30.0))
        self.assertEqual(eff.corners[1].fz_lb, 0.0)

    def test_pneumatic_trail_raises_the_rack_force(self):
        bare = self._eff().rack_force_lb
        self.assertGreater(self._eff(pneumatic_trail_mm=25.0).rack_force_lb,
                           bare)

    def test_the_limitations_are_stated_not_hidden(self):
        notes = " ".join(self._eff().notes).lower()
        self.assertIn("pneumatic trail", notes)
        self.assertIn("parking", notes)

    def test_driver_effort_follows_virtual_work(self):
        """One turn of the wheel moves the rack one travel, so
        T = F * travel / (2*pi). Checked against the returned value."""
        eff = self._eff(rack_travel_per_turn_mm=76.2, wheel_diameter_mm=280.0)
        want = eff.rack_force_lb * (76.2 / IN) / (2.0 * np.pi)
        self.assertAlmostEqual(eff.steering_wheel_torque_lbin, want, places=9)
        self.assertAlmostEqual(eff.hand_force_lb,
                               want / (140.0 / IN), places=9)

    def test_driver_rows_are_omitted_without_a_ratio(self):
        eff = self._eff()
        self.assertIsNone(eff.steering_wheel_torque_lbin)
        self.assertIsNone(eff.hand_force_lb)

    def test_a_tie_rod_through_the_kingpin_is_refused_with_a_reason(self):
        """Zero steering arm means no steering at all; silently dividing
        by it would print an infinite force."""
        import dataclasses
        kp = np.asarray(self.state.lbj, float)
        hp2 = dataclasses.replace(self.hp, tierod_outer=kp.copy())
        st2 = solver_for(hp2)(hp2).solve(0.0)
        with self.assertRaises(ValueError):
            S.cornering_effort(hp2, st2, (300.0, 100.0), self.tire)


class TestDialogUsesLiveDataAndWarns(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _win(self):
        from unittest import mock
        from PySide6.QtWidgets import QWidget

        class _Stub(QWidget):
            def __init__(self, *a, **k):
                super().__init__()
                self.scene = mock.MagicMock()

            def __getattr__(self, n):
                return mock.MagicMock()

        import suspension_tool.gui.main_window as mw
        with mock.patch.object(mw, "CornerViewport", _Stub):
            win = mw.MainWindow()
        self.app.processEvents()
        win._load_project_file(_DEMO)
        self.app.processEvents()
        return mw, win

    def test_it_warns_when_the_mass_block_is_another_car(self):
        """The failure this guards: a dynamics block left on a different
        vehicle makes every force ~200x low and entirely plausible-looking.

        It used to lean on the fixture project itself carrying a 1/10 RC
        car's masses alongside full-size geometry -- a real defect at the
        time. A test that needs its fixture to stay BROKEN stops testing
        the moment someone fixes it, so the mismatch is injected here."""
        mw, win = self._win()
        win.dynamics_panel._imperial.update(
            dict(weight_empty_lb=3.25, driver_lb=0.0, front_axle_lb=1.2,
                 rear_axle_lb=2.05, unsprung_front_lb=0.2,
                 unsprung_rear_lb=0.25))
        dlg = mw._SteeringEffortDialog(win, win.axles["front"])
        dyn, _loads, _eff = dlg.result_now()
        self.assertLess(dyn.front_axle_lb, 10.0)
        msgs = dlg._plausibility(dyn)
        self.assertTrue(msgs)
        self.assertIn("not this vehicle", msgs[0])

    def test_real_masses_clear_the_warning_and_give_sane_numbers(self):
        mw, win = self._win()
        win.dynamics_panel._imperial.update(
            dict(weight_empty_lb=400.0, driver_lb=180.0, front_axle_lb=260.0,
                 rear_axle_lb=320.0, unsprung_front_lb=30.0,
                 unsprung_rear_lb=35.0, ay_g=1.0))
        dlg = mw._SteeringEffortDialog(win, win.axles["front"])
        dyn, loads, eff = dlg.result_now()
        self.assertEqual(dlg._plausibility(dyn), [])
        self.assertGreater(loads[0], loads[1], "outside wheel carries more")
        self.assertGreater(eff.rack_force_lb, 50.0)
        self.assertLess(eff.rack_force_lb, 400.0)
        self.assertLess(eff.hand_force_lb, 60.0, "a driver has to turn it")


if __name__ == "__main__":
    unittest.main()
