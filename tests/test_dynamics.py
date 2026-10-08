"""Equations of motion verified against closed-form solutions (not against the simulator)."""

import math

import numpy as np
import pytest

from rocket_sim.constants import G0
from rocket_sim.motor import Motor
from rocket_sim.physics.integrators import rk4_step
from rocket_sim.vehicle import ConstantAero
from tests.conftest import (
    free_launch,
    make_3dof,
    make_6dof,
    make_env,
    make_vehicle,
    tiny_motor,
)


def integrate(dyn, y, t_end, dt, t0=0.0):
    t = t0
    n = int(round((t_end - t0) / dt))
    for _ in range(n):
        y = dyn.post_step(rk4_step(dyn.derivative, t, y, dt))
        t += dt
    return t, y


def test_free_fall_constant_gravity():
    veh = make_vehicle(tiny_motor())
    dyn = make_3dof(veh, make_env(rho=None, g=9.81), free_launch(z0=1000.0), aero_enabled=False)
    y = dyn.initial_state()
    t, y = integrate(dyn, y, 5.0, 0.01)
    assert y[2] == pytest.approx(1000.0 - 0.5 * 9.81 * 25.0, abs=1e-6)
    assert y[5] == pytest.approx(-9.81 * 5.0, abs=1e-8)


def test_vacuum_projectile_range_and_apogee():
    veh = make_vehicle(tiny_motor())
    dyn = make_3dof(veh, make_env(g=9.81), free_launch(), aero_enabled=False)
    y = dyn.initial_state()
    v0, ang = 100.0, math.radians(40.0)
    y[3], y[5] = v0 * math.cos(ang), v0 * math.sin(ang)
    t_flight = 2 * v0 * math.sin(ang) / 9.81
    t, y = integrate(dyn, y, round(t_flight / 0.005) * 0.005, 0.005)  # whole number of steps
    assert y[2] == pytest.approx(v0 * math.sin(ang) * t - 0.5 * 9.81 * t * t, abs=1e-6)
    assert y[0] == pytest.approx(v0 * math.cos(ang) * t, rel=1e-12)
    assert abs(y[2]) < 0.05 and y[0] == pytest.approx(v0**2 * math.sin(2 * ang) / 9.81, rel=1e-3)
    # apogee height at half time
    dyn2 = make_3dof(veh, make_env(g=9.81), free_launch(), aero_enabled=False)
    y2 = dyn2.initial_state()
    y2[3], y2[5] = v0 * math.cos(ang), v0 * math.sin(ang)
    t2, y2 = integrate(dyn2, y2, round(t_flight / 2 / 0.005) * 0.005, 0.005)
    assert y2[2] == pytest.approx(v0 * math.sin(ang) * t2 - 0.5 * 9.81 * t2 * t2, abs=1e-6)
    assert y2[2] == pytest.approx((v0 * math.sin(ang)) ** 2 / (2 * 9.81), rel=1e-4)


def test_rocket_equation_constant_thrust_vertical_vacuum():
    """v(t) = c ln(m0/m) - g t with c = I_total/m_p for impulse-proportional mass flow."""
    thrust, burn, prop = 100.0, 2.0, 0.2
    motor = Motor("C", 0.03, 0.2, prop, 0.3, [0.0, burn], [thrust, thrust])
    veh = make_vehicle(motor, dry_mass=0.8)
    g = 9.81
    dyn = make_3dof(veh, make_env(g=g), free_launch(), aero_enabled=False)
    y = dyn.initial_state()
    t, y = integrate(dyn, y, burn, 0.002)
    m0 = 0.8 + 0.1 + prop
    mf = 0.8 + 0.1
    c = motor.total_impulse / prop
    v_expect = c * math.log(m0 / mf) - g * burn
    assert y[5] == pytest.approx(v_expect, rel=1e-8)
    # altitude: integral of the analytic velocity history
    from scipy.integrate import quad

    mdot = prop / burn
    z_expect = quad(lambda s_: c * math.log(m0 / (m0 - mdot * s_)) - g * s_, 0, burn, epsabs=1e-12)[0]
    assert y[2] == pytest.approx(z_expect, rel=1e-8)


def test_mass_depletion_matches_motor_model_through_vehicle():
    motor = Motor("C", 0.03, 0.2, 0.2, 0.3, [0.0, 1.0, 2.0], [50.0, 150.0, 50.0])
    veh = make_vehicle(motor, dry_mass=0.8)
    assert veh.mass_props(0.0).mass == pytest.approx(0.8 + 0.3)
    assert veh.mass_props(2.0).mass == pytest.approx(0.8 + 0.1)
    assert veh.mass_props(10.0).mass == pytest.approx(0.9)
    # symmetric thrust curve -> half the propellant gone at the midpoint
    assert veh.mass_props(1.0).mass == pytest.approx(0.8 + 0.1 + 0.1)


def test_quadratic_drag_free_fall_matches_analytic():
    rho, cd, d, mass = 1.0, 0.8, 0.1, 1.5
    veh = make_vehicle(tiny_motor(), dry_mass=mass, aero=ConstantAero(d, cd), diameter=d)
    env = make_env(rho=rho, g=9.81)
    dyn = make_3dof(veh, env, free_launch(z0=5000.0))
    m = veh.mass_props(0.0).mass
    s = math.pi * d * d / 4
    vt = math.sqrt(2 * m * 9.81 / (rho * s * cd))
    y = dyn.initial_state()
    # (nose direction is fixed to the launch axis only when aero is off; with aero the thrust
    #  direction follows the airflow, thrust here is ~1e-9 N so it is irrelevant)
    for T in (3.0, 6.0):
        dyn = make_3dof(veh, env, free_launch(z0=5000.0))
        y = dyn.initial_state()
        _, y = integrate(dyn, y, T, 0.005)
        v_expect = -vt * math.tanh(9.81 * T / vt)
        z_expect = 5000.0 - (vt**2 / 9.81) * math.log(math.cosh(9.81 * T / vt))
        assert y[5] == pytest.approx(v_expect, rel=2e-6)
        assert y[2] == pytest.approx(z_expect, rel=1e-8)


def test_wind_enters_through_relative_velocity():
    """Stationary vehicle in a steady wind feels drag along the wind, F = 1/2 rho w^2 Cd S."""
    from rocket_sim.environment import ConstantWind

    rho, cd, d = 1.2, 0.5, 0.1
    veh = make_vehicle(tiny_motor(), dry_mass=1.0, aero=ConstantAero(d, cd), diameter=d)
    env = make_env(rho=rho, g=0.0)
    env.wind = ConstantWind(10.0, 270.0)  # blows toward +x
    dyn = make_3dof(veh, env, free_launch(z0=100.0))
    y = dyn.initial_state()
    e = dyn.evaluate(0.0, y)
    s = math.pi * d * d / 4
    assert e.drag == pytest.approx(0.5 * rho * 100.0 * cd * s, rel=1e-12)
    m = veh.mass_props(0.0).mass
    assert e.a[0] == pytest.approx(e.drag / m, rel=1e-6) and e.a[0] > 0
    assert e.a[1] == pytest.approx(0.0, abs=1e-12)
    assert e.airspeed == pytest.approx(10.0)
    # relaxes toward the wind speed with quadratic drag: delta(t) = d0 / (1 + k d0 t)
    k = 0.5 * rho * cd * s / m
    _, y = integrate(dyn, y, 400.0, 0.05)
    assert y[3] == pytest.approx(10.0 - 10.0 / (1.0 + k * 10.0 * 400.0), rel=1e-6)
    # and the SAME wind applied to a vehicle already moving with the air produces no drag
    y2 = dyn.initial_state()
    y2[3] = 10.0
    assert dyn.evaluate(0.0, y2).drag == pytest.approx(0.0, abs=1e-12)


def test_mach_logged_from_relative_speed():
    veh = make_vehicle(tiny_motor(), aero=ConstantAero(0.05, 0.3))
    dyn = make_3dof(veh, make_env(rho=1.225), free_launch(z0=10.0))
    y = dyn.initial_state()
    y[5] = 170.147
    e = dyn.evaluate(0.0, y)
    assert e.mach == pytest.approx(0.5, rel=1e-5)
    assert e.sos == pytest.approx(340.294, rel=1e-5)


def test_6dof_torque_free_axisymmetric_precession():
    """Euler's equations for Ixx != Iyy: transverse rate rotates at -lambda*p, |w| conserved."""
    ixx, iyy = 0.002, 0.08
    veh = make_vehicle(tiny_motor(), dry_mass=1.0, ixx=ixx, iyy=iyy, aero=ConstantAero(0.05, 0.0))
    # zero density atmosphere: no aerodynamic moments
    env = make_env(rho=1e-12, g=0.0)
    dyn = make_6dof(veh, env, free_launch(z0=1e5))
    mp = veh.mass_props(0.0)
    y = dyn.initial_state()
    p0, q0 = 40.0, 0.5
    y[10], y[11] = p0, q0
    lam = (mp.iyy - mp.ixx) / mp.iyy
    t_end = 0.5
    t, y = integrate(dyn, y, t_end, 0.0005)
    assert y[10] == pytest.approx(p0, rel=1e-9)
    assert y[11] == pytest.approx(q0 * math.cos(lam * p0 * t_end), abs=2e-6)
    assert y[12] == pytest.approx(-q0 * math.sin(lam * p0 * t_end), abs=2e-6)
    assert math.hypot(y[11], y[12]) == pytest.approx(q0, rel=1e-9)
    # rotational kinetic energy and |H| conserved
    h = math.sqrt((mp.ixx * y[10]) ** 2 + (mp.iyy * y[11]) ** 2 + (mp.iyy * y[12]) ** 2)
    assert h == pytest.approx(math.sqrt((mp.ixx * p0) ** 2 + (mp.iyy * q0) ** 2), rel=1e-9)
    assert np.linalg.norm(y[6:10]) == pytest.approx(1.0, abs=1e-12)


def test_6dof_constant_rate_rotation_integrates_quaternion():
    veh = make_vehicle(tiny_motor(), ixx=0.01, iyy=0.01)  # symmetric
    dyn = make_6dof(veh, make_env(rho=1e-12, g=0.0), free_launch(z0=1e5))
    y = dyn.initial_state()
    y[10] = 2.0  # roll about the nose axis at 2 rad/s
    t, y = integrate(dyn, y, 1.5, 0.001)
    q = y[6:10]
    nose0 = np.array([0, 0, 1.0])
    from rocket_sim.physics.math3d import quat_rotate

    assert np.allclose(quat_rotate(tuple(q), (1, 0, 0)), nose0, atol=1e-9)  # nose unchanged
    # total rotation angle = w t
    q0 = dyn.launch.quaternion
    qrel = np.array(q) @ np.array(q0)  # |<q, q0>| = cos(angle/2)
    assert abs(qrel) == pytest.approx(abs(math.cos(2.0 * 1.5 / 2)), abs=1e-9)


def test_6dof_static_stability_oscillation_frequency():
    """Small-angle weathercock oscillation: wn^2 = q S CNa (x_cp - x_cg)/Iyy, with damping."""
    d, cna, xcp, cg = 0.05, 12.0, 0.8, 0.5
    aero = ConstantAero(d, 0.0, cn_alpha=cna, x_cp=xcp)
    iyy = 0.08
    veh = make_vehicle(tiny_motor(), dry_mass=1.0, ixx=0.002, iyy=iyy, aero=aero, cg=cg, diameter=d)
    mp = veh.mass_props(0.0)
    rho, v = 1.2, 100.0
    env = make_env(rho=rho, g=0.0)
    dyn = make_6dof(veh, env, free_launch(z0=1e5, elevation=0.0, azimuth=0.0))  # level, heading north
    y = dyn.initial_state()
    y[4] = v  # flying north at 100 m/s
    # small pitch perturbation: rotate the body about its y axis by 2 degrees (nose up)
    from rocket_sim.physics.math3d import quat_from_axis_angle, quat_mul

    q0 = tuple(y[6:10])
    th0 = math.radians(2.0)
    q1 = quat_mul(q0, quat_from_axis_angle((0, 1, 0), -th0))  # negative about +y_B raises the nose
    y[6:10] = q1
    s = math.pi * d * d / 4
    arm = xcp - mp.x_cg
    q_dyn = 0.5 * rho * v * v
    wn2 = q_dyn * s * cna * arm / mp.iyy
    c = 0.5 * rho * v * s * cna * arm**2
    wd = math.sqrt(wn2 - (c / (2 * mp.iyy)) ** 2)
    # integrate and record pitch angle (angle between nose and velocity) zero crossings
    dt, t = 0.0005, 0.0
    angs, ts = [], []
    from rocket_sim.physics.math3d import angle_between, quat_rotate

    for _ in range(int(2.0 / dt)):
        y = dyn.post_step(rk4_step(dyn.derivative, t, y, dt))
        t += dt
        nose = quat_rotate(tuple(y[6:10]), (1, 0, 0))
        vel = tuple(y[3:6])
        sign = 1.0 if nose[2] > vel[2] / np.linalg.norm(vel) else -1.0
        angs.append(sign * angle_between(nose, vel))
        ts.append(t)
    angs = np.array(angs)
    crossings = [ts[i] for i in range(1, len(angs)) if angs[i - 1] > 0 >= angs[i]]
    assert len(crossings) >= 3
    period = np.mean(np.diff(crossings))
    assert 2 * math.pi / wd == pytest.approx(period, rel=0.01)
    assert np.max(np.abs(angs[: int(0.05 / dt)])) <= th0 * 1.01


def test_6dof_unstable_when_cp_ahead_of_cg():
    d = 0.05
    aero = ConstantAero(d, 0.0, cn_alpha=12.0, x_cp=0.3)  # cp forward of cg=0.5
    veh = make_vehicle(tiny_motor(), aero=aero, cg=0.5, diameter=d)
    assert veh.static_margin(0.0) < 0
    dyn = make_6dof(veh, make_env(rho=1.2, g=0.0), free_launch(z0=1e5, elevation=0.0))
    y = dyn.initial_state()
    y[4] = 100.0
    from rocket_sim.physics.math3d import angle_between, quat_from_axis_angle, quat_mul, quat_rotate

    y[6:10] = quat_mul(tuple(y[6:10]), quat_from_axis_angle((0, 1, 0), -math.radians(1.0)))
    t = 0.0
    for _ in range(int(1.0 / 0.001)):
        y = dyn.post_step(rk4_step(dyn.derivative, t, y, 0.001))
        t += 0.001
    ang = angle_between(quat_rotate(tuple(y[6:10]), (1, 0, 0)), tuple(y[3:6]))
    assert ang > math.radians(20)  # diverged from the initial 1 degree


def test_tvc_moment_direction_and_magnitude():
    thrust = 200.0
    motor = Motor("C", 0.03, 0.2, 0.05, 0.1, [0.0, 1.0], [thrust, thrust])
    veh = make_vehicle(motor, dry_mass=1.0, cg=0.5)
    dyn = make_6dof(veh, make_env(rho=1e-12, g=0.0), free_launch(z0=1e5))
    dyn.on_rail = False
    y = dyn.initial_state()
    mp = veh.mass_props(0.0)
    th = math.radians(3.0)
    dyn.controls.tvc_y = th
    e = dyn.evaluate(0.0, y)
    lever = veh.nozzle_x - mp.x_cg
    # thrust deflected by +th about y_B: F_z = -T sin(th); nozzle aft by `lever` => M_y = -lever*T*sin(th)... sign check
    expected_my = -lever * thrust * math.sin(th)
    assert e.wdot[1] * mp.iyy == pytest.approx(expected_my, rel=1e-9)
    assert e.wdot[2] == pytest.approx(0.0, abs=1e-12)
    # axial thrust component reduced by cos(th)
    assert e.thrust == pytest.approx(thrust)


def test_rail_holds_vehicle_until_thrust_exceeds_weight():
    motor = Motor("C", 0.03, 0.2, 0.05, 0.1, [0.0, 1.0], [5.0, 5.0])  # T = 5 N << weight
    veh = make_vehicle(motor, dry_mass=1.0)
    from rocket_sim.physics.dynamics import LaunchSetup
    from rocket_sim.physics.math3d import quat_from_pointing

    launch = LaunchSetup((0, 0, 0), quat_from_pointing(math.pi / 2, 0.0), 1.0, True)
    dyn = make_6dof(veh, make_env(g=9.81, rho=1.2), launch)
    y = dyn.initial_state()
    t, y2 = integrate(dyn, y, 0.5, 0.01)
    assert np.allclose(y2[:6], y[:6], atol=1e-12)
    assert G0 * 1.15 > 5.0  # sanity of the test setup


def test_tail_first_flight_is_unstable_and_flips_to_nose_first():
    """A statically stable rocket (CP aft of CG) is unstable when flying backwards."""
    from rocket_sim.physics.math3d import angle_between, quat_from_axis_angle, quat_mul, quat_rotate

    d = 0.05
    aero = ConstantAero(d, 0.0, cn_alpha=12.0, x_cp=0.8)
    veh = make_vehicle(tiny_motor(), aero=aero, cg=0.5, diameter=d)
    assert veh.static_margin(0.0) > 0
    dyn = make_6dof(veh, make_env(rho=1.2, g=0.0), free_launch(z0=1e5, elevation=0.0))
    y = dyn.initial_state()
    y[4] = -100.0  # flying south while the nose points north: alpha = 180 deg
    y[6:10] = quat_mul(tuple(y[6:10]), quat_from_axis_angle((0, 1, 0), math.radians(2.0)))
    ang0 = angle_between(quat_rotate(tuple(y[6:10]), (1, 0, 0)), tuple(y[3:6]))
    assert ang0 > math.radians(177)
    t = 0.0
    for _ in range(2000):
        y = dyn.post_step(rk4_step(dyn.derivative, t, y, 0.001))
        t += 0.001
    ang = angle_between(quat_rotate(tuple(y[6:10]), (1, 0, 0)), tuple(y[3:6]))
    assert ang < math.radians(150)  # left the 180-degree equilibrium
