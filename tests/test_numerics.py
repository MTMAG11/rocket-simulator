"""Numerical validation: integrator order of accuracy and full-flight timestep convergence."""

import math

import numpy as np
import pytest

from rocket_sim.physics.integrators import euler_step, midpoint_step, refine_event, rk4_step
from rocket_sim.simulation import run_simulation
from tests.conftest import cfg_from


@pytest.mark.parametrize("stepper,order", [(euler_step, 1), (midpoint_step, 2), (rk4_step, 4)])
def test_integrator_order_of_accuracy(stepper, order):
    """y' = -y, y(0)=1: measured convergence order matches the theoretical order."""

    def f(t, y):
        return -y

    errs = []
    for n in (4, 8, 16) if order == 4 else (20, 40, 80):  # RK4 reaches round-off on finer grids
        h = 2.0 / n
        y, t = np.array([1.0]), 0.0
        for _ in range(n):
            y = stepper(f, t, y, h)
            t += h
        errs.append(abs(y[0] - math.exp(-2.0)))
    rate = math.log2(errs[0] / errs[1]), math.log2(errs[1] / errs[2])
    assert all(r == pytest.approx(order, abs=0.5) for r in rate), rate


def test_refine_event_finds_root_with_reintegration():
    f = lambda t, y: np.array([-9.81 * t * 0 - 9.81])  # noqa: E731 - falling body, y' = -g (v state omitted)
    # state is height with constant descent rate: y(t) = 10 - 9.81 t ; root at t = 10/9.81
    g = lambda t, y: float(y[0])  # noqa: E731
    te, ye = refine_event(rk4_step, f, g, 0.0, np.array([10.0]), 2.0)
    assert te == pytest.approx(10.0 / 9.81, abs=1e-12) and abs(ye[0]) < 1e-12


def _apogee(dt, integrator="rk4", fidelity=3):
    cfg = cfg_from(
        fidelity=fidelity,
        rocket={"parachutes": []},
        simulation={"dt_s": dt, "integrator": integrator},
        environment={"wind": {"model": "constant", "speed_ms": 3, "direction_from_deg": 270}},
    )
    r = run_simulation(cfg, seed=0)
    return (
        r.summary["apogee_m"],
        r.summary["impact_speed_ms"],
        r.summary["landing_x_m"],
        r.summary["apogee_time_s"],
    )


@pytest.mark.parametrize("fidelity,dts", [(2, (0.1, 0.01, 0.001)), (3, (0.05, 0.01, 0.001))])
def test_timestep_convergence_rk4(fidelity, dts):
    """Coarse -> fine timesteps against a fine reference: errors shrink monotonically and are small.

    (6-DOF is capped at dt <= 0.05 s by config validation; 3-DOF supports 0.1 s.)
    """
    ref = _apogee(0.0005, fidelity=fidelity)
    results = {dt: _apogee(dt, fidelity=fidelity) for dt in dts}
    err = {dt: abs(results[dt][0] - ref[0]) for dt in dts}
    coarse, mid, fine = dts
    # errors shrink to a ~cm floor (thrust-curve corners, event location); allow 5 cm of slack
    assert err[fine] <= err[mid] + 0.05 and err[mid] <= err[coarse] + 0.05
    assert err[fine] < err[coarse] or err[coarse] < 0.05
    assert err[mid] < 0.5 and err[fine] < 0.05
    assert abs(results[mid][3] - ref[3]) < 0.02  # apogee time
    assert abs(results[fine][2] - ref[2]) < 0.5  # landing position
    assert abs(results[fine][1] - ref[1]) < 0.05  # impact speed


@pytest.mark.slow
def test_euler_is_visibly_worse_than_rk4_at_same_dt():
    ref = _apogee(0.0005)[0]
    e_euler = abs(_apogee(0.01, "euler")[0] - ref)
    e_rk4 = abs(_apogee(0.01, "rk4")[0] - ref)
    assert e_rk4 < e_euler / 5


def test_event_times_do_not_depend_on_step_alignment():
    """Apogee/landing are located by root finding, not snapped to the output grid."""
    a = _apogee(0.01)
    b = _apogee(0.013)
    assert a[0] == pytest.approx(b[0], abs=0.3)
    assert a[3] == pytest.approx(b[3], abs=0.03)
