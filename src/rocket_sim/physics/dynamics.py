"""Equations of motion: 3-DOF point mass and 6-DOF rigid body.

Both classes expose the same interface so the simulation loop, recorder and sensors do not
care which is used:

    initial_state()          -> state vector y0
    derivative(t, y)         -> dy/dt           (uses ``self.controls`` and ``self.chute_start``)
    evaluate(t, y)           -> Eval            (forces, environment, mass; for logging/sensors)
    position(y) / velocity(y) / quaternion(y) / omega(y)
    post_step(y)             -> y               (renormalise quaternion, rail clamp)

6-DOF state (13): [x y z | vx vy vz | qw qx qy qz | p q r]  (launch frame / body frame, SI)
3-DOF state (6):  [x y z | vx vy vz]

Equations (6-DOF), launch frame L (ENU), body frame B (x nose, FRD):

    r_dot = v
    m v_dot = R(q) (F_thrust + F_aero)_B + F_chute_L + m g_L,        g_L = (0, 0, -g(h))
    q_dot = 1/2 q (x) (0, omega)
    I omega_dot = M - omega x (I omega),      I = 3x3 tensor (diag(Ixx, Iyy, Iyy) on the fast path)

Thrust is an external force (momentum thrust is inside the thrust-curve value); the mass-flow
induced terms (jet damping, dI/dt * omega, propellant relative momentum) are neglected.
While ``on_rail`` the vehicle is constrained to slide along the rail axis (no rotation, no
sliding backwards). See docs/physics.md for assumptions and references.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import NamedTuple

import numpy as np

from ..environment import Environment
from ..vehicle import Vehicle
from .math3d import Quat, Vec, quat_from_pointing, quat_rotate, quat_rotate_inv


def euler_angular_acceleration(mp, m: Vec, w: Vec) -> Vec:
    """Solve I w_dot = M - w x (I w) for the full symmetric inertia tensor (body axes)."""
    ixx, iyy, izz = mp.ixx, mp.iyy, mp.izz_eff
    ixy, ixz, iyz = mp.ixy, mp.ixz, mp.iyz  # products of inertia (matrix entries are the negatives)
    p, q, r = w
    hx = ixx * p - ixy * q - ixz * r
    hy = -ixy * p + iyy * q - iyz * r
    hz = -ixz * p - iyz * q + izz * r
    rx = m[0] - (q * hz - r * hy)
    ry = m[1] - (r * hx - p * hz)
    rz = m[2] - (p * hy - q * hx)
    a11, a12, a13 = ixx, -ixy, -ixz
    a22, a23, a33 = iyy, -iyz, izz
    det = a11 * (a22 * a33 - a23 * a23) - a12 * (a12 * a33 - a23 * a13) + a13 * (a12 * a23 - a22 * a13)
    i11 = (a22 * a33 - a23 * a23) / det
    i12 = (a13 * a23 - a12 * a33) / det
    i13 = (a12 * a23 - a13 * a22) / det
    i22 = (a11 * a33 - a13 * a13) / det
    i23 = (a12 * a13 - a11 * a23) / det
    i33 = (a11 * a22 - a12 * a12) / det
    return (
        i11 * rx + i12 * ry + i13 * rz,
        i12 * rx + i22 * ry + i23 * rz,
        i13 * rx + i23 * ry + i33 * rz,
    )


class Eval(NamedTuple):
    """Instantaneous derived quantities at (t, y). All SI, vectors in the stated frame."""

    a: Vec  # inertial (coordinate) acceleration, launch frame [m/s^2]
    wdot: Vec  # angular acceleration, body frame [rad/s^2]
    spec_force: Vec  # specific force (a - g), body frame [m/s^2] -> what an accelerometer reads
    thrust: float  # thrust magnitude [N]
    drag: float  # total drag (body + parachute) [N]
    lift: float  # aerodynamic force perpendicular to the relative wind [N]
    chute_drag: float  # parachute drag [N]
    mach: float
    alpha: float  # total angle of attack [rad]
    beta: float  # sideslip [rad]
    qdyn: float  # dynamic pressure [Pa]
    rho: float
    temp: float
    pres: float
    sos: float  # speed of sound [m/s]
    wind: Vec  # air velocity in launch frame [m/s]
    airspeed: float  # |v - wind| [m/s]
    g: float  # gravitational acceleration magnitude [m/s^2]
    mass: float
    x_cg: float  # [m] aft of nose
    x_cp: float  # [m] aft of nose
    ixx: float
    iyy: float
    static_margin: float  # calibers


@dataclass
class Controls:
    """Actuator state applied to the physics (held constant across an integration step)."""

    tvc_y: float = 0.0  # thrust deflection about body y [rad] (ACTUAL actuator state)
    tvc_z: float = 0.0  # thrust deflection about body z [rad]
    fin: list[float] = field(default_factory=list)  # actual deflection of each control fin [rad]


@dataclass(frozen=True)
class LaunchSetup:
    position: Vec
    quaternion: Quat
    rail_length: float
    on_rail: bool = True


class _BaseDynamics:
    n_state = 0

    def __init__(self, vehicle: Vehicle, env: Environment, launch: LaunchSetup) -> None:
        self.vehicle = vehicle
        self.env = env
        self.launch = launch
        self.controls = Controls()
        # simulation time at which each parachute begins inflating (None = not deployed)
        self.chute_starts: list[float | None] = [None] * len(vehicle.parachutes)
        self.on_rail = launch.on_rail
        self.rail_axis: Vec = quat_rotate(launch.quaternion, (1.0, 0.0, 0.0))
        self.rail_origin: Vec = launch.position
        self.rail_length = launch.rail_length
        self._body_len = vehicle.body.length
        self._cache: tuple | None = None

    def evaluate(self, t: float, y: np.ndarray) -> Eval:
        """Forces/environment at (t, y). One-entry memo: the simulator evaluates the state at the
        start of a step for logging and RK4's first stage needs the same value."""
        c = self.controls
        key = (t, y.tobytes(), c.tvc_y, c.tvc_z, tuple(c.fin), self.on_rail, tuple(self.chute_starts))
        hit = self._cache
        if hit is not None and hit[0] == key:
            return hit[1]
        e = self._evaluate(t, y)
        self._cache = (key, e)
        return e

    def _evaluate(self, t: float, y: np.ndarray) -> Eval:  # pragma: no cover - abstract
        raise NotImplementedError

    def chute_cda(self, t: float) -> float:
        total = 0.0
        for p, t0 in zip(self.vehicle.parachutes, self.chute_starts):
            if t0 is None or t < t0:
                continue
            f = 1.0 if p.inflation_time <= 0 else min((t - t0) / p.inflation_time, 1.0)
            total += p.cd * p.area * f
        return total

    def rail_distance(self, y: np.ndarray) -> float:
        o, ax = self.rail_origin, self.rail_axis
        return (y[0] - o[0]) * ax[0] + (y[1] - o[1]) * ax[1] + (y[2] - o[2]) * ax[2]

    def position(self, y: np.ndarray) -> Vec:
        return (float(y[0]), float(y[1]), float(y[2]))

    def velocity(self, y: np.ndarray) -> Vec:
        return (float(y[3]), float(y[4]), float(y[5]))

    def _rail_clamp(self, y: np.ndarray) -> np.ndarray:
        """While on the rail, remove any velocity component off-axis or back down the rail."""
        ax = self.rail_axis
        s_dot = y[3] * ax[0] + y[4] * ax[1] + y[5] * ax[2]
        s_dot = max(s_dot, 0.0)
        y[3], y[4], y[5] = s_dot * ax[0], s_dot * ax[1], s_dot * ax[2]
        return y


class PointMass3DOF(_BaseDynamics):
    """Three-degree-of-freedom point mass (translation only).

    Thrust acts along the rail axis while on the rail; after that along the relative-wind
    direction ("zero angle of attack", i.e. perfect weathercocking) when aerodynamics are
    enabled, otherwise along the fixed launch axis. Drag acts along -v_rel with the coefficient
    from the vehicle aero model. ``vertical_only`` constrains motion to the z axis (1-D).
    """

    n_state = 6

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        launch: LaunchSetup,
        aero_enabled: bool = True,
        wind_enabled: bool = True,
        vertical_only: bool = False,
    ) -> None:
        super().__init__(vehicle, env, launch)
        self.aero_enabled = aero_enabled
        self.wind_enabled = wind_enabled
        self.vertical_only = vertical_only
        if vertical_only:
            self.rail_axis = (0.0, 0.0, 1.0)

    def initial_state(self) -> np.ndarray:
        y = np.zeros(6)
        y[:3] = self.launch.position
        return y

    def quaternion(self, y: np.ndarray, t: float = 0.0) -> Quat:
        """Kinematic attitude: nose along the thrust/flight-path direction, zero roll."""
        d = self._nose_direction(t, y)
        el = math.asin(max(-1.0, min(1.0, d[2])))
        az = math.atan2(d[0], d[1]) if (d[0] or d[1]) else 0.0
        return quat_from_pointing(el, az, 0.0)

    def omega(self, y: np.ndarray) -> Vec:
        return (0.0, 0.0, 0.0)

    def post_step(self, y: np.ndarray) -> np.ndarray:
        if self.vertical_only:
            y[0] = self.launch.position[0]
            y[1] = self.launch.position[1]
            y[3] = y[4] = 0.0
        if self.on_rail:
            y = self._rail_clamp(y)
        return y

    def _nose_direction(self, t: float, y: np.ndarray) -> Vec:
        if self.on_rail or not self.aero_enabled or self.vertical_only:
            return self.rail_axis
        w = self._wind(t, y)
        vx, vy, vz = y[3] - w[0], y[4] - w[1], y[5] - w[2]
        v = math.sqrt(vx * vx + vy * vy + vz * vz)
        if v < 1.0:
            return self.rail_axis
        return (vx / v, vy / v, vz / v)

    def _wind(self, t: float, y: np.ndarray) -> Vec:
        if not (self.aero_enabled and self.wind_enabled):
            return (0.0, 0.0, 0.0)
        return self.env.wind.at(t, float(y[2]))

    def _evaluate(self, t: float, y: np.ndarray) -> Eval:
        env, veh = self.env, self.vehicle
        z = float(y[2])
        h_msl = env.site_elevation + z
        g = env.gravity.g(h_msl)
        mp = veh.mass_props(t)
        thrust = veh.thrust_at(t)
        powered = thrust > 0.0
        nd = self._nose_direction(t, y)

        fx, fy, fz = thrust * nd[0], thrust * nd[1], thrust * nd[2]
        drag = chute = 0.0
        mach = qdyn = airspeed = 0.0
        rho = temp = pres = sos = 0.0
        w = (0.0, 0.0, 0.0)
        cp = -1.0  # filled from the aero model below (or the static value if no airflow)
        if self.aero_enabled:
            atm = env.atmosphere.at(h_msl)
            rho, temp, pres, sos = atm.density, atm.temperature, atm.pressure, atm.speed_of_sound
            w = self._wind(t, y)
            vx, vy, vz = y[3] - w[0], y[4] - w[1], y[5] - w[2]
            airspeed = math.sqrt(vx * vx + vy * vy + vz * vz)
            if airspeed > 1e-9:
                mach = airspeed / sos
                re = rho * airspeed * self._body_len / atm.viscosity
                c = veh.aero.coefficients(mach, re, powered)
                cp = c.x_cp
                qdyn = 0.5 * rho * airspeed * airspeed
                body_drag = qdyn * veh.aero.ref_area * c.cd0
                chute = qdyn * self.chute_cda(t)
                drag = body_drag + chute
                k = -drag / airspeed
                fx += k * vx
                fy += k * vy
                fz += k * vz
        if cp < 0.0:
            cp = veh.aero.coefficients(0.0, 1e6, powered).x_cp if self.aero_enabled else mp.x_cg
        ax, ay, az = fx / mp.mass, fy / mp.mass, fz / mp.mass - g
        if self.vertical_only:
            ax = ay = 0.0
        if self.on_rail:
            ax_, ay_, az_ = self._rail_accel(y, ax, ay, az)
            ax, ay, az = ax_, ay_, az_
        # 3-DOF has no body axes: sensors (which need specific force in the body frame) require 6-DOF
        return Eval(
            (ax, ay, az),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            thrust,
            drag,
            0.0,
            chute,
            mach,
            0.0,
            0.0,
            qdyn,
            rho,
            temp,
            pres,
            sos,
            w,
            airspeed,
            g,
            mp.mass,
            mp.x_cg,
            cp,
            mp.ixx,
            mp.iyy,
            (cp - mp.x_cg) / veh.aero.ref_diameter,
        )

    def _rail_accel(self, y: np.ndarray, ax: float, ay: float, az: float) -> Vec:
        axis = self.rail_axis
        a_ax = ax * axis[0] + ay * axis[1] + az * axis[2]
        s_dot = y[3] * axis[0] + y[4] * axis[1] + y[5] * axis[2]
        if s_dot <= 1e-12 and a_ax <= 0.0:
            return (0.0, 0.0, 0.0)  # held by the pad/rail
        return (a_ax * axis[0], a_ax * axis[1], a_ax * axis[2])

    def derivative(self, t: float, y: np.ndarray) -> np.ndarray:
        e = self.evaluate(t, y)
        vx, vy, vz = float(y[3]), float(y[4]), float(y[5])
        if self.on_rail:
            ax = self.rail_axis
            s_dot = max(vx * ax[0] + vy * ax[1] + vz * ax[2], 0.0)
            vx, vy, vz = s_dot * ax[0], s_dot * ax[1], s_dot * ax[2]
        return np.array([vx, vy, vz, e.a[0], e.a[1], e.a[2]])


class RigidBody6DOF(_BaseDynamics):
    """Six-degree-of-freedom rigid body with Barrowman aerodynamics and optional TVC."""

    n_state = 13

    def __init__(self, vehicle: Vehicle, env: Environment, launch: LaunchSetup) -> None:
        super().__init__(vehicle, env, launch)
        self._mis = vehicle.thrust_misalignment

    def initial_state(self) -> np.ndarray:
        y = np.zeros(13)
        y[:3] = self.launch.position
        y[6:10] = self.launch.quaternion
        return y

    def quaternion(self, y: np.ndarray, t: float = 0.0) -> Quat:
        return (float(y[6]), float(y[7]), float(y[8]), float(y[9]))

    def omega(self, y: np.ndarray) -> Vec:
        return (float(y[10]), float(y[11]), float(y[12]))

    def post_step(self, y: np.ndarray) -> np.ndarray:
        n = math.sqrt(y[6] ** 2 + y[7] ** 2 + y[8] ** 2 + y[9] ** 2)
        y[6:10] /= n
        if self.on_rail:
            y = self._rail_clamp(y)
            y[10:13] = 0.0
        return y

    def _evaluate(self, t: float, y: np.ndarray) -> Eval:
        env, veh = self.env, self.vehicle
        x, yy, z, vx, vy, vz, qw, qx, qy, qz, p, q, r = y.tolist()
        qn = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        quat = (qw / qn, qx / qn, qy / qn, qz / qn)
        h_msl = env.site_elevation + z
        g = env.gravity.g(h_msl)
        mp = veh.mass_props(t)
        thrust = veh.thrust_at(t)
        powered = thrust > 0.0
        atm = env.atmosphere.at(h_msl)
        w = env.wind.at(t, z)
        # relative air velocity of the vehicle, in body axes: (u, v, w) with u along the nose
        vrel_l = (vx - w[0], vy - w[1], vz - w[2])
        u, v, ww = quat_rotate_inv(quat, vrel_l)
        airspeed = math.sqrt(u * u + v * v + ww * ww)

        # thrust: gimballed (ACTUAL actuator state) + fixed misalignment, applied at the nozzle exit plane
        th_y = self.controls.tvc_y + self._mis[0]
        th_z = self.controls.tvc_z + self._mis[1]
        cz = math.cos(th_z)
        tdx, tdy, tdz = cz * math.cos(th_y), math.sin(th_z), -cz * math.sin(th_y)
        fbx, fby, fbz = thrust * tdx, thrust * tdy, thrust * tdz
        rn = mp.x_cg - veh.nozzle_x  # nozzle position forward of CG (negative: it is aft)
        yc, zc = mp.y_cg, mp.z_cg  # CG offsets from the nose axis (0 for axisymmetric vehicles)
        # moment = r x F with r = (rn, -yc, -zc) from the CG to the nozzle exit
        mx = zc * fby - yc * fbz
        my = -zc * fbx - rn * fbz
        mz = rn * fby + yc * fbx

        drag = lift = chute = 0.0
        mach = qdyn = alpha = beta = 0.0
        x_cp = -1.0
        cfx = cfy = cfz = 0.0  # parachute force, launch frame
        if airspeed > 1e-6:
            mach = airspeed / atm.speed_of_sound
            re = atm.density * airspeed * self._body_len / atm.viscosity
            qdyn = 0.5 * atm.density * airspeed * airspeed
            s_ref = veh.aero.ref_area
            lat = math.hypot(v, ww)
            alpha = math.atan2(lat, u)
            beta = math.asin(max(-1.0, min(1.0, v / airspeed)))
            fc = veh.aero.force_coefficients(mach, alpha, re, powered)
            x_cp = fc.x_cp_static
            # axial force (drag at alpha = 0) and the normal force opposing the lateral air velocity
            fax = -qdyn * s_ref * fc.ca
            f_n = qdyn * s_ref * fc.cn
            if lat > 1e-9:
                ly, lz = -v / lat, -ww / lat
            else:
                ly = lz = 0.0
            fay, faz = f_n * ly, f_n * lz
            # aerodynamic force acts at the centre of pressure of the normal force: r = (rx, -yc, -zc)
            rx = mp.x_cg - fc.x_cp_force
            mx += zc * fay - yc * faz
            my += -zc * fax - rx * faz
            mz += rx * fay + yc * fax
            # movable control surfaces (canards / tail fins): force along the fin normal, at the fin AC
            cfins = veh.aero.control_fins
            if cfins:
                kmach = veh.aero.control_cn_alpha(mach)
                for cf, dlt in zip(cfins, self.controls.fin):
                    if dlt == 0.0:
                        continue
                    lift_i = qdyn * s_ref * cf.cn_alpha_single * kmach * dlt
                    sphi, cphi = math.sin(cf.phi), math.cos(cf.phi)
                    fx_i, fy_i, fz_i = (
                        -abs(lift_i * dlt),
                        -lift_i * sphi,
                        lift_i * cphi,
                    )  # |L d|: induced drag
                    fax += fx_i
                    fay += fy_i
                    faz += fz_i
                    rxi, py, pz = mp.x_cg - cf.x_ac, cf.rho * cphi - yc, cf.rho * sphi - zc
                    mx += py * fz_i - pz * fy_i
                    my += pz * fx_i - rxi * fz_i
                    mz += rxi * fy_i - py * fx_i
            fbx += fax
            fby += fay
            fbz += faz
            # damping from strip theory: c = 1/2 rho V S sum(CNa_i x_i^2); roll: rho V S CNa r^2
            cdamp = 0.0
            for cn_i, x_i in veh.aero.damping_surfaces:
                arm = mp.x_cg - x_i
                cdamp += cn_i * arm * arm
            cdamp *= 0.5 * atm.density * airspeed * s_ref
            my -= cdamp * q
            mz -= cdamp * r
            mx -= (
                atm.density
                * airspeed
                * s_ref
                * veh.aero.roll_damping_cn
                * veh.aero.roll_damping_radius**2
                * p
            )
            # diagnostics: project the total aerodynamic force onto the relative wind
            fa_dot_v = (fax * u + fay * v + faz * ww) / airspeed
            drag = -fa_dot_v
            f2 = fax * fax + fay * fay + faz * faz
            lift = math.sqrt(max(f2 - fa_dot_v * fa_dot_v, 0.0))
            cda = self.chute_cda(t)
            if cda > 0.0:
                chute = qdyn * cda
                k = -chute / airspeed
                cfx, cfy, cfz = k * vrel_l[0], k * vrel_l[1], k * vrel_l[2]
                drag += chute
                # shock-cord load acts at the attachment point, forward of the CG: this makes the
                # airframe hang from the canopy (stable equilibrium) instead of tumbling
                cb = quat_rotate_inv(quat, (cfx, cfy, cfz))
                r_att = mp.x_cg - veh.parachutes[0].attach
                mx += zc * cb[1] - yc * cb[2]
                my += -zc * cb[0] - r_att * cb[2]
                mz += r_att * cb[1] + yc * cb[0]

        if x_cp < 0.0:
            x_cp = veh.aero.coefficients(0.0, 1e6, powered).x_cp
        fl = quat_rotate(quat, (fbx, fby, fbz))
        inv_m = 1.0 / mp.mass
        ax = (fl[0] + cfx) * inv_m
        ay = (fl[1] + cfy) * inv_m
        az = (fl[2] + cfz) * inv_m - g
        if self.on_rail:
            axis = self.rail_axis
            a_ax = ax * axis[0] + ay * axis[1] + az * axis[2]
            s_dot = vx * axis[0] + vy * axis[1] + vz * axis[2]
            if s_dot <= 1e-12 and a_ax <= 0.0:
                a_ax = 0.0
            ax, ay, az = a_ax * axis[0], a_ax * axis[1], a_ax * axis[2]
            wdot = (0.0, 0.0, 0.0)
        elif mp.izz is None and mp.ixy == 0.0 and mp.ixz == 0.0 and mp.iyz == 0.0:
            ixx, iyy = mp.ixx, mp.iyy  # axisymmetric: the Euler equations reduce to the V1 form
            wdot = (
                mx / ixx,
                (my + (iyy - ixx) * r * p) / iyy,
                (mz + (ixx - iyy) * p * q) / iyy,
            )
        else:
            wdot = euler_angular_acceleration(mp, (mx, my, mz), (p, q, r))
        sf = quat_rotate_inv(quat, (ax, ay, az + g))
        return Eval(
            (ax, ay, az),
            wdot,
            sf,
            thrust,
            drag,
            lift,
            chute,
            mach,
            alpha,
            beta,
            qdyn,
            atm.density,
            atm.temperature,
            atm.pressure,
            atm.speed_of_sound,
            w,
            airspeed,
            g,
            mp.mass,
            mp.x_cg,
            x_cp,
            mp.ixx,
            mp.iyy,
            (x_cp - mp.x_cg) / veh.aero.ref_diameter,
        )

    def derivative(self, t: float, y: np.ndarray) -> np.ndarray:
        e = self.evaluate(t, y)
        vx, vy, vz = float(y[3]), float(y[4]), float(y[5])
        qw, qx, qy, qz = float(y[6]), float(y[7]), float(y[8]), float(y[9])
        p, q, r = float(y[10]), float(y[11]), float(y[12])
        if self.on_rail:
            ax = self.rail_axis
            s_dot = max(vx * ax[0] + vy * ax[1] + vz * ax[2], 0.0)
            vx, vy, vz = s_dot * ax[0], s_dot * ax[1], s_dot * ax[2]
            dq = (0.0, 0.0, 0.0, 0.0)
        else:
            dq = (
                0.5 * (-qx * p - qy * q - qz * r),
                0.5 * (qw * p + qy * r - qz * q),
                0.5 * (qw * q - qx * r + qz * p),
                0.5 * (qw * r + qx * q - qy * p),
            )
        return np.array(
            [
                vx,
                vy,
                vz,
                e.a[0],
                e.a[1],
                e.a[2],
                dq[0],
                dq[1],
                dq[2],
                dq[3],
                e.wdot[0],
                e.wdot[1],
                e.wdot[2],
            ]
        )
