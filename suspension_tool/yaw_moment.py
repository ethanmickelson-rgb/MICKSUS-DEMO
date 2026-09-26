"""Milliken Moment Method (MMM) yaw-moment diagrams.

WHAT IT ANSWERS. Everything else in this tool describes the car at one
operating point: a roll gradient, an understeer gradient in the LINEAR
range, a load transfer. None of that tells you what the car does when the
tires are actually saturated -- and a car's balance at the limit is
routinely the opposite of its balance in the linear range.

A yaw-moment diagram is a portrait of the whole manoeuvring envelope. The
vehicle is swept over a grid of body slip angles (beta) and steer angles
(delta); at each point the tires are loaded, slipped and summed, giving

    Cy = sum(Fy) / W                 lateral acceleration, g
    Cn = N / (W * L)                 yaw moment coefficient, dimensionless

Plotting Cn against Cy draws a curved envelope. Read it as:

  * the WIDTH is how much lateral g the car can hold;
  * the HEIGHT is spare yaw moment -- how hard it can still rotate;
  * Cn = 0 is TRIM (a steady turn: no leftover yaw moment);
  * the top/bottom edges are the limit of control authority.

Three numbers come out of the origin of the diagram, and they are the ones
worth putting on a design review slide:

  * STABILITY INDEX = CMG / LAG, in Cn per g. Milliken's definition (SAE
    760712), NOT dCn/d(beta) -- several texts use that name for the other
    quantity, so both are exposed and dcn_dbeta_per_deg is the other one.
    Negative = stable: generating lateral acceleration also generates a
    yaw moment opposing further rotation. Positive = the car diverges and
    the driver is doing the stabilising.
  * CONTROL MOMENT GAIN, dCn/d(delta) per radian. How much yaw moment a
    unit of steer buys. Too little and the car will not rotate; too much
    and it is nervous. It can go NEGATIVE at low speed, which means the
    car actively resists being steered.
  * LIMIT BALANCE, the sign of Cn at maximum Cy. Positive means the car
    still wants to rotate INTO the corner at the limit (loose/oversteer);
    negative means it is pushing (understeer).

Reference: Milliken & Milliken, "Race Car Vehicle Dynamics", ch. 8, and
SAE 760712. Prompted by an MMM implementation shared publicly on the Baja
SAE Discord by an Auburn student ("dynosaur"), which also supplied the
first MEASURED lateral tire data we have for a Baja-class tire -- see
tire.ExponentialTire.

UNITS. Pounds, feet, seconds, degrees -- the units the source data and
Milliken both use. The kinematics half of this tool is mm; conversions
happen at the boundary, never inside.
"""

from dataclasses import dataclass, field

import numpy as np

from .tire import DUNLOP_KT821_CLAY, ExponentialTire

G_FT_S2 = 32.174          # gravity, ft/s^2
MPH_TO_FT_S = 5280.0 / 3600.0


@dataclass
class YawMomentVehicle:
    """A car, in the units Milliken's chapter 8 uses.

    Corner weights are STATIC and in lb; tracks, wheelbase, CG height and
    roll-centre heights in INCHES; roll rates in ft-lb per degree of body
    roll (suspension only, ARB included)."""

    weight_fl_lb: float
    weight_fr_lb: float
    weight_rl_lb: float
    weight_rr_lb: float
    track_front_in: float
    track_rear_in: float
    wheelbase_in: float
    cg_height_in: float
    roll_centre_front_in: float
    roll_centre_rear_in: float
    roll_rate_front_ftlb_deg: float
    roll_rate_rear_ftlb_deg: float
    tire_front: ExponentialTire = field(default=DUNLOP_KT821_CLAY)
    tire_rear: ExponentialTire = field(default=DUNLOP_KT821_CLAY)
    name: str = "vehicle"

    # -- derived ------------------------------------------------------
    @property
    def weight_lb(self) -> float:
        return (self.weight_fl_lb + self.weight_fr_lb
                + self.weight_rl_lb + self.weight_rr_lb)

    @property
    def weight_front_lb(self) -> float:
        return self.weight_fl_lb + self.weight_fr_lb

    @property
    def wheelbase_ft(self) -> float:
        return self.wheelbase_in / 12.0

    @property
    def a_ft(self) -> float:
        """CG to FRONT axle. Follows from the static weight split: the
        front axle carries more when the CG is closer to it."""
        return self.wheelbase_ft * (1.0 - self.weight_front_lb
                                    / self.weight_lb)

    @property
    def b_ft(self) -> float:
        """CG to REAR axle."""
        return self.wheelbase_ft - self.a_ft

    @property
    def roll_axis_at_cg_in(self) -> float:
        """Roll-axis height under the CG, interpolated between the two
        roll centres."""
        f = self.a_ft / self.wheelbase_ft      # fraction of L back from front
        return (self.roll_centre_front_in
                + f * (self.roll_centre_rear_in - self.roll_centre_front_in))

    @property
    def roll_moment_arm_ft(self) -> float:
        """CG height above the roll axis -- the lever the sprung weight
        rolls about (Milliken's H1)."""
        return (self.cg_height_in - self.roll_axis_at_cg_in) / 12.0

    def lateral_load_transfer_lb(self, ay_g: float):
        """(front, rear) lateral load transfer in lb at `ay_g`.

        The standard three-term split: each axle carries its share of the
        SPRUNG roll couple in proportion to its roll stiffness, plus the
        geometric term its own roll centre reacts directly. Written the
        way Milliken 18.4 does, which is also what dynamics.py uses."""
        w = self.weight_lb
        h1 = self.roll_moment_arm_ft
        tf, tr = self.track_front_in / 12.0, self.track_rear_in / 12.0
        kf = self.roll_rate_front_ftlb_deg * 180.0 / np.pi   # per radian
        kr = self.roll_rate_rear_ftlb_deg * 180.0 / np.pi
        k_tot = kf + kr - w * h1            # net roll stiffness
        if abs(k_tot) < 1e-9:               # the car would roll over
            k_tot = 1e-9
        zf, zr = self.roll_centre_front_in / 12.0, self.roll_centre_rear_in / 12.0
        # geometric term uses the axle's OWN weight share
        wf, wr = self.weight_front_lb, w - self.weight_front_lb
        d_front = ay_g * (w * h1 * kf / k_tot + wf * zf) / tf
        d_rear = ay_g * (w * h1 * kr / k_tot + wr * zr) / tr
        return float(d_front), float(d_rear)

    def wheel_loads_lb(self, ay_g: float):
        """[fl, fr, rl, rr] vertical loads at `ay_g`, clamped at zero --
        a lifted wheel carries no load and makes no force."""
        df, dr = self.lateral_load_transfer_lb(ay_g)
        # positive ay = left turn = load moves to the RIGHT (outside)
        fz = [self.weight_fl_lb - df, self.weight_fr_lb + df,
              self.weight_rl_lb - dr, self.weight_rr_lb + dr]
        return [max(0.0, v) for v in fz]


def _slip_angles_deg(veh, beta_deg, steer_deg, speed_ft_s, yaw_rate_rad_s):
    """Front and rear slip angles (deg).

    alpha_f = delta - beta - a*r/V ;  alpha_r = -beta + b*r/V
    -- the standard bicycle-model kinematics, with the yaw rate term being
    what makes the front and rear axles see different slip in a turn."""
    beta = np.radians(beta_deg)
    delta = np.radians(steer_deg)
    v = max(float(speed_ft_s), 1e-6)
    af = delta - beta - veh.a_ft * yaw_rate_rad_s / v
    ar = -beta + veh.b_ft * yaw_rate_rad_s / v
    return float(np.degrees(af)), float(np.degrees(ar))


def solve_point(veh: YawMomentVehicle, beta_deg: float, steer_deg: float,
                speed_mph: float, radius_ft: float = None,
                max_iter: int = 60, tol: float = 1e-8) -> dict:
    """One (beta, delta) point of the diagram.

    The lateral acceleration is not known in advance: it sets the load
    transfer, which sets the tire loads, which set the forces, which set
    the lateral acceleration. So iterate to a fixed point.

    radius_ft None  -> CONSTANT SPEED sweep. The yaw rate follows the
                       achieved acceleration (r = Ay*g/V), which is the
                       general 'anywhere on the track' diagram.
    radius_ft given -> CONSTANT RADIUS sweep, r = V/R fixed. This is the
                       diagram for one specific corner.
    """
    v = max(float(speed_mph) * MPH_TO_FT_S, 1e-6)
    w = veh.weight_lb
    ay = 0.0
    for _ in range(max_iter):
        if radius_ft is None:
            r = ay * G_FT_S2 / v
        else:
            r = v / float(radius_ft)
        af, ar = _slip_angles_deg(veh, beta_deg, steer_deg, v, r)
        fz = veh.wheel_loads_lb(ay)
        fy = [float(veh.tire_front.lateral_force_lb(fz[0], af)),
              float(veh.tire_front.lateral_force_lb(fz[1], af)),
              float(veh.tire_rear.lateral_force_lb(fz[2], ar)),
              float(veh.tire_rear.lateral_force_lb(fz[3], ar))]
        ay_new = sum(fy) / w
        if abs(ay_new - ay) < tol:
            ay = ay_new
            break
        # damped update: the load-transfer feedback can oscillate at the
        # limit, where a small Ay change lifts a wheel outright
        ay += 0.5 * (ay_new - ay)
    n_ftlb = veh.a_ft * (fy[0] + fy[1]) - veh.b_ft * (fy[2] + fy[3])
    return {
        "beta_deg": float(beta_deg), "steer_deg": float(steer_deg),
        "ay_g": float(ay),
        "cn": float(n_ftlb / (w * veh.wheelbase_ft)),
        "yaw_moment_ftlb": float(n_ftlb),
        "alpha_front_deg": af, "alpha_rear_deg": ar,
        "fz_lb": fz, "fy_lb": fy,
        "wheel_lifted": any(v_ <= 0.0 for v_ in fz),
    }


@dataclass
class YawMomentDiagram:
    """A solved grid, plus everything read off it."""
    betas_deg: np.ndarray
    steers_deg: np.ndarray
    ay_g: np.ndarray            # [beta, steer]
    cn: np.ndarray              # [beta, steer]
    speed_mph: float
    radius_ft: float = None
    vehicle_name: str = ""

    # -- the numbers worth quoting ------------------------------------
    @property
    def limit_ay_g(self) -> float:
        """Largest lateral acceleration anywhere in the envelope."""
        return float(np.nanmax(np.abs(self.ay_g)))

    @property
    def control_moment_gain(self) -> float:
        """CMG = dCn/d(delta) at the origin, per RADIAN of steer. How much
        yaw moment a unit of steer buys -- the car's control authority."""
        return self._slope(self.cn, axis=1) * 180.0 / np.pi

    @property
    def lateral_accel_gain(self) -> float:
        """LAG = dAy/d(delta) at the origin, g per RADIAN. How much
        lateral acceleration a unit of steer buys."""
        return self._slope(self.ay_g, axis=1) * 180.0 / np.pi

    @property
    def stability_index(self) -> float:
        """SI = CMG / LAG, in Cn per g.

        This is Milliken's definition (SAE 760712) and the one the
        reference implementation uses -- NOT dCn/d(beta), which measures
        something related but different. Read it as the slope of the
        Cn-Ay curve: how much unbalanced yaw moment appears per g of
        lateral acceleration as the driver steers.

        NEGATIVE is stable/restoring: as the car generates lateral
        acceleration it also generates a yaw moment opposing further
        rotation. Positive means the car helps itself around, and the
        driver is doing the stabilising."""
        lag = self.lateral_accel_gain
        if abs(lag) < 1e-12:
            return float("nan")
        return self.control_moment_gain / lag

    @property
    def dcn_dbeta_per_deg(self) -> float:
        """The static directional stability derivative, dCn/d(beta) per
        degree. Kept separately because several texts call THIS the
        stability index; it is not what SI means above."""
        return self._slope(self.cn, axis=0)

    def _slope(self, field: np.ndarray, axis: int) -> float:
        """Central-difference slope of `field` at the grid origin.
        axis 0 = with respect to beta, axis 1 = with respect to steer."""
        i = int(np.argmin(np.abs(self.betas_deg)))
        j = int(np.argmin(np.abs(self.steers_deg)))
        if axis == 0:
            xs, ys, k = self.betas_deg, field[:, j], i
        else:
            xs, ys, k = self.steers_deg, field[i, :], j
        if len(xs) < 3:
            return float("nan")
        lo, hi = max(0, k - 1), min(len(xs) - 1, k + 1)
        if hi == lo:
            return float("nan")
        return float((ys[hi] - ys[lo]) / (xs[hi] - xs[lo]))

    @property
    def limit_balance(self) -> float:
        """Cn where |Ay| is greatest. > 0 the car still wants to rotate
        into the corner at the limit (loose); < 0 it is pushing."""
        i, j = np.unravel_index(np.nanargmax(np.abs(self.ay_g)),
                                self.ay_g.shape)
        return float(self.cn[i, j])

    def trim_ay_g(self) -> float:
        """Largest |Ay| reachable at TRIM (Cn = 0) -- a steady turn the
        car will actually hold, as opposed to a transient peak."""
        best = 0.0
        for i in range(len(self.betas_deg)):
            row_cn, row_ay = self.cn[i, :], self.ay_g[i, :]
            sign = np.sign(row_cn)
            for j in range(len(sign) - 1):
                if sign[j] == 0.0:
                    best = max(best, abs(row_ay[j]))
                elif sign[j] != sign[j + 1] and np.isfinite(row_cn[j]):
                    # linear interpolation onto Cn = 0
                    f = row_cn[j] / (row_cn[j] - row_cn[j + 1])
                    best = max(best, abs(row_ay[j]
                                         + f * (row_ay[j + 1] - row_ay[j])))
        return float(best)

    def summary(self) -> dict:
        return {
            "limit_ay_g": self.limit_ay_g,
            "trim_ay_g": self.trim_ay_g(),
            "stability_index": self.stability_index,
            "control_moment_gain": self.control_moment_gain,
            "lateral_accel_gain": self.lateral_accel_gain,
            "dcn_dbeta_per_deg": self.dcn_dbeta_per_deg,
            "limit_balance": self.limit_balance,
            "stable": self.stability_index < 0.0,
            "balance": ("oversteer at the limit" if self.limit_balance > 0
                        else "understeer at the limit"),
        }


def yaw_moment_diagram(veh: YawMomentVehicle, speed_mph: float,
                       betas_deg=None, steers_deg=None,
                       radius_ft: float = None) -> YawMomentDiagram:
    """Sweep the (beta, delta) grid and solve every point."""
    betas = (np.linspace(-10.0, 10.0, 21) if betas_deg is None
             else np.asarray(betas_deg, float))
    steers = (np.linspace(-20.0, 20.0, 21) if steers_deg is None
              else np.asarray(steers_deg, float))
    ay = np.zeros((len(betas), len(steers)))
    cn = np.zeros_like(ay)
    for i, b in enumerate(betas):
        for j, d in enumerate(steers):
            pt = solve_point(veh, float(b), float(d), speed_mph,
                             radius_ft=radius_ft)
            ay[i, j] = pt["ay_g"]
            cn[i, j] = pt["cn"]
    return YawMomentDiagram(betas_deg=betas, steers_deg=steers, ay_g=ay,
                            cn=cn, speed_mph=float(speed_mph),
                            radius_ft=radius_ft, vehicle_name=veh.name)


# ----------------------------------------------------------------------
# The Auburn reference car -- an ACTUAL Baja vehicle, used as an
# independent validation case for our dynamics defaults.
# ----------------------------------------------------------------------
AUBURN_EXAMPLE = YawMomentVehicle(
    weight_fl_lb=154.0, weight_fr_lb=154.0,
    weight_rl_lb=142.0, weight_rr_lb=142.0,
    track_front_in=51.0, track_rear_in=44.0, wheelbase_in=59.0,
    cg_height_in=21.91,
    roll_centre_front_in=6.427, roll_centre_rear_in=6.58,
    roll_rate_front_ftlb_deg=64.0, roll_rate_rear_ftlb_deg=77.0,
    tire_front=DUNLOP_KT821_CLAY, tire_rear=DUNLOP_KT821_CLAY,
    name="Auburn example (hard clay, with rollbar)")


def from_dynamics(inputs, results, tire_front=None, tire_rear=None,
                  name="MICKSUS design") -> YawMomentVehicle:
    """Build a YawMomentVehicle from the dynamics panel's inputs/results,
    so the diagram runs on the car currently open rather than on a
    separately-typed copy of it."""
    wf = float(getattr(inputs, "front_axle_lb", 0.0))
    wr = float(getattr(inputs, "rear_axle_lb", 0.0))
    tf = float(getattr(inputs, "track_front_in", 51.0))
    tr = float(getattr(inputs, "track_rear_in", 51.0))
    rc_f = float(getattr(inputs, "rc_front_in", 0.0))
    rc_r = float(getattr(inputs, "rc_rear_in", 0.0))
    kf = float(results.get("roll_rate_front_lbftdeg", 0.0) or 0.0)
    kr = float(results.get("roll_rate_rear_lbftdeg", 0.0) or 0.0)
    t = tire_front or DUNLOP_KT821_CLAY
    return YawMomentVehicle(
        weight_fl_lb=wf / 2.0, weight_fr_lb=wf / 2.0,
        weight_rl_lb=wr / 2.0, weight_rr_lb=wr / 2.0,
        track_front_in=tf, track_rear_in=tr,
        wheelbase_in=float(getattr(inputs, "wheelbase_in", 59.0)),
        cg_height_in=float(getattr(inputs, "cg_height_in", 21.0)),
        roll_centre_front_in=rc_f, roll_centre_rear_in=rc_r,
        roll_rate_front_ftlb_deg=kf, roll_rate_rear_ftlb_deg=kr,
        tire_front=t, tire_rear=tire_rear or t, name=name)
