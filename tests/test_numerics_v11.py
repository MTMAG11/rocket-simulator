"""V1.1 numerics: quaternion algebra, conservation laws in isolated-force limits, timestep convergence ladder."""

import math

import numpy as np
import pytest

from rocket_sim.motor import Motor
from rocket_sim.physics.integrators import rk4_step
from rocket_sim.physics.math3d import (
    quat_conj,
    quat_from_axis_angle,
    quat_from_pointing,
    quat_mul,
    quat_normalize,
    quat_rotate,
    quat_to_euler,
)
from rocket_sim.simulation import run_simulation
from rocket_sim.vehicle import ConstantAero
from tests.conftest import cfg_from, free_launch, make_3dof, make_6dof, make_env, make_vehicle, tiny_motor


def integrate(dyn, y, t_end, dt):
    t = 0.0
    for _ in range(int(round(t_end / dt))):
        y = dyn.post_step(rk4_step(dyn.derivative, t, y, dt))
        t += dt
    return y


# -------------------------------------------------------------------------------- quaternions
def test_identity_quaternion_is_neutral():
    v = (0.3, -1.2, 2.5)
    assert quat_rotate((1.0, 0.0, 0.0, 0.0), v) == pytest.approx(v)
    q = quat_from_axis_angle((0.2, 0.5, -0.4), 0.9)
    assert quat_mul((1.0, 0, 0, 0), q) == pytest.approx(q) and quat_mul(q, (1.0, 0, 0, 0)) == pytest.approx(q)


def test_quaternion_times_conjugate_is_identity():
    q = quat_normalize((0.4, -0.3, 0.8, 0.1))
    assert quat_mul(q, quat_conj(q)) == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-15)


@pytest.mark.parametrize("axis", [(1, 0, 0), (0, 1, 0), (0, 0, 1)])
def test_90_and_180_degree_rotations(axis):
    n = np.array(axis, float)
    other = np.roll(n, 1)
    q90 = quat_from_axis_angle(axis, math.pi / 2)
    q180 = quat_from_axis_angle(axis, math.pi)
    # 90 deg about a basis axis maps the next basis vector onto the cross product
    assert quat_rotate(q90, tuple(other)) == pytest.approx(tuple(np.cross(n, other)), abs=1e-14)
    assert quat_rotate(q180, tuple(other)) == pytest.approx(tuple(-other), abs=1e-14)
    assert quat_rotate(q180, axis) == pytest.approx(axis, abs=1e-14)  # the axis itself is unchanged
    assert quat_mul(q90, q90) == pytest.approx(q180, abs=1e-14) or quat_mul(q90, q90) == pytest.approx(
        tuple(-x for x in q180), abs=1e-14
    )


def test_repeated_integration_keeps_unit_norm_and_exact_angle():
    dyn = make_6dof(
        make_vehicle(tiny_motor(), ixx=0.01, iyy=0.01), make_env(rho=1e-12, g=0.0), free_launch(z0=1e5)
    )
    y = dyn.initial_state()
    y[10], y[11] = 1.5, 0.7  # tumbling about a tilted axis
    q0 = np.array(y[6:10])
    t_end = 20.0
    y = integrate(dyn, y, t_end, 0.005)
    assert np.linalg.norm(y[6:10]) == pytest.approx(1.0, abs=1e-12)  # 4000 steps, no drift
    # symmetric body: |w| constant, total angle = |w| t about the (body-fixed) axis -> compare with exact exponential map
    w = np.array([1.5, 0.7, 0.0])
    qe = quat_mul(
        tuple(q0), quat_from_axis_angle(tuple(w / np.linalg.norm(w)), float(np.linalg.norm(w)) * t_end)
    )
    assert abs(float(np.dot(qe, y[6:10]))) == pytest.approx(1.0, abs=1e-6)


@pytest.mark.parametrize(
    "el,az,roll", [(0.3, 0.2, -0.5), (1.0, 2.5, 0.7), (-0.4, -1.0, 2.0), (0.0, 0.0, 0.0)]
)
def test_euler_conversion_roundtrip(el, az, roll):
    # (heading clockwise from north, pitch above horizon, roll about the nose) <- quat_from_pointing
    heading, pitch, r = quat_to_euler(quat_from_pointing(el, az, roll))
    assert (heading, pitch, r) == pytest.approx((az, el, roll), abs=1e-12)


def test_euler_gimbal_lock_is_finite():
    q = quat_from_pointing(math.pi / 2, 0.0, 0.4)
    assert all(math.isfinite(x) for x in quat_to_euler(q))


# ---------------------------------------------------------------------------------- conservation
def test_no_forces_means_constant_velocity_and_zero_rate():
    veh = make_vehicle(tiny_motor(), aero=ConstantAero(0.05, 0.0))
    dyn = make_6dof(veh, make_env(rho=1e-12, g=0.0), free_launch(z0=1e5))
    y = dyn.initial_state()
    y[3:6] = (12.0, -7.0, 30.0)
    y2 = integrate(dyn, y.copy(), 10.0, 0.01)
    assert y2[3:6] == pytest.approx((12.0, -7.0, 30.0), abs=1e-6)
    assert y2[0:3] == pytest.approx(y[0:3] + 10.0 * np.array([12.0, -7.0, 30.0]), abs=1e-6)
    assert y2[10:13] == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_gravity_only_conserves_mechanical_energy_and_horizontal_momentum():
    veh = make_vehicle(tiny_motor())
    dyn = make_3dof(veh, make_env(g=9.81), free_launch(z0=100.0), aero_enabled=False)
    y = dyn.initial_state()
    y[3], y[4], y[5] = 20.0, 5.0, 40.0
    e0 = 0.5 * float(np.dot(y[3:6], y[3:6])) + 9.81 * y[2]
    y = integrate(dyn, y, 6.0, 0.01)
    assert 0.5 * float(np.dot(y[3:6], y[3:6])) + 9.81 * y[2] == pytest.approx(e0, rel=1e-9)
    assert (y[3], y[4]) == pytest.approx((20.0, 5.0), abs=1e-12)


def test_thrust_only_impulse_equals_momentum_change():
    mass_prop = 0.05
    motor = Motor("C", 0.03, 0.2, mass_prop, 0.1, [0.0, 1.0, 2.0], [100.0, 100.0, 100.0])
    # constant 100 N for 2 s: impulse 200 N s; the propellant mass is tiny so use the rocket equation
    veh = make_vehicle(motor, dry_mass=2.0, cg=0.5)
    dyn = make_3dof(veh, make_env(g=0.0), free_launch(z0=1e5), aero_enabled=False)
    y = dyn.initial_state()
    y = integrate(dyn, y, 2.0, 0.002)
    isp_ve = 200.0 / mass_prop  # effective exhaust velocity
    assert y[5] == pytest.approx(
        isp_ve * math.log(dyn.vehicle.mass_props(0.0).mass / dyn.vehicle.mass_props(2.0).mass), rel=2e-3
    )


def test_drag_only_energy_decreases_monotonically_with_exact_rate():
    cd, rho, m = 0.5, 1.2, 1.0
    veh = make_vehicle(tiny_motor(), dry_mass=m, aero=ConstantAero(0.1, cd), diameter=0.1)
    dyn = make_3dof(veh, make_env(rho=rho, g=0.0), free_launch(z0=1e5))
    y = dyn.initial_state()
    y[5] = 100.0
    t, e_prev = 0.0, 0.5 * m * 100.0**2
    k = 0.5 * rho * cd * veh.aero.ref_area / m
    for _ in range(500):
        y = dyn.post_step(rk4_step(dyn.derivative, t, y, 0.01))
        t += 0.01
        e = 0.5 * m * y[5] ** 2
        assert e < e_prev
        e_prev = e
    assert y[5] == pytest.approx(1.0 / (k * t + 1.0 / 100.0), rel=1e-8)  # v(t) = 1/(k t + 1/v0)


def test_torque_only_constant_torque_gives_exact_angular_rate():
    # TVC torque with no aero: use the 6-DOF with a pure constant gimbal torque via a short, light motor
    motor = Motor("C", 0.03, 0.2, 0.05, 0.1, [0.0, 10.0], [50.0, 50.0])
    veh = make_vehicle(motor, dry_mass=1.0, cg=0.5, aero=ConstantAero(0.05, 0.0))
    dyn = make_6dof(veh, make_env(rho=1e-12, g=0.0), free_launch(z0=1e5))
    dyn.controls.tvc_y = 0.02
    y = dyn.initial_state()
    mp0 = dyn.vehicle.mass_props(0.0)
    dt, n = 0.0005, 100  # 0.05 s: mass change is negligible
    y = integrate(dyn, y, dt * n, dt)
    lever = veh.nozzle_x - mp0.x_cg
    torque = -lever * 50.0 * math.sin(0.02)
    assert y[11] == pytest.approx(torque / mp0.iyy * dt * n, rel=2e-2)  # q = M t / I (I changes by <1 %)


# ----------------------------------------------------------------------------- convergence ladder
LADDER_3DOF = (0.1, 0.05, 0.01, 0.005, 0.001)
LADDER_6DOF = (0.05, 0.01, 0.005, 0.001)  # config validation caps 6-DOF dt at 0.05 s


def _run(fid, dt):
    s = run_simulation(
        cfg_from(
            fidelity=fid,
            rocket={"parachutes": []},
            simulation={"dt_s": dt},
            environment={"wind": {"model": "constant", "speed_ms": 3, "direction_from_deg": 270}},
        ),
        seed=0,
    ).summary
    return s["apogee_m"], s["apogee_time_s"], s["max_velocity_ms"]


@pytest.mark.parametrize("fid,ladder", [(2, LADDER_3DOF), (3, LADDER_6DOF)])
def test_convergence_ladder(fid, ladder):
    ref = _run(fid, 0.0005)
    errs = [abs(_run(fid, dt)[0] - ref[0]) for dt in ladder]
    # the finest step is converged to < 1 cm and no coarser step beats it; the coarsest is clearly worse than the finest
    assert errs[-1] < 0.01 and errs[-1] <= min(errs[:-1]) + 1e-12
    assert errs[0] > 10 * errs[-1] + 0.01  # the ladder actually spans resolved -> unresolved
    # the middle of the ladder (0.01 s) is within the documented ~3.5 cm thrust-corner / event floor
    assert errs[ladder.index(0.01)] < 0.05
