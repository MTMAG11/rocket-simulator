"""Drag coefficient from the coast phase: window selection, uncertainty and recovery of a known Cd."""

from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.integrate import solve_ivp

from rocket_sim.config import apply_overrides, config_from_dict, read_mapping
from rocket_sim.environment.atmosphere import ISAAtmosphere
from rocket_sim.errors import ConfigError
from rocket_sim.resources import default_config
from rocket_sim.simulation import Simulation
from rocket_sim.validation import drag_fit
from rocket_sim.validation.drag_fit import estimate_cd

G = 9.80665
MASS, DIAMETER = 0.62, 0.0762  # test vehicle (not a measured rocket)
AREA = math.pi * DIAMETER**2 / 4.0


def oracle_cd(mass, rho, speed, area, drag_decel):
    """Test-local inverse of D = 1/2 rho v^2 S Cd (NOT the owner's TODO)."""
    return 2.0 * mass * drag_decel / (rho * speed**2 * area)


@pytest.fixture
def with_oracle(monkeypatch):
    monkeypatch.setattr(drag_fit, "solve_cd", oracle_cd)


def synthetic_flight(cd=0.55, fs=25.0, noise=0.3, seed=0, boost=2.0, v_burnout=110.0):
    """Altitude log of a vertical flight: constant-acceleration boost, then a quadratic-drag coast integrated with SciPy."""
    atm = ISAAtmosphere()

    def rhs(_t, y):
        h, v = y
        rho = atm.at(max(h, 0.0)).density
        return [v, -G - 0.5 * rho * v * abs(v) * AREA * cd / MASS]

    a_boost = v_burnout / boost
    t_b = np.arange(0.0, boost, 1.0 / fs)
    h_b = 0.5 * a_boost * t_b**2
    h0 = 0.5 * a_boost * boost**2

    def apogee(_t, y):
        return y[1]

    apogee.terminal, apogee.direction = True, -1
    sol = solve_ivp(
        rhs, (boost, boost + 60.0), [h0, v_burnout], events=apogee, dense_output=True, rtol=1e-9, atol=1e-9
    )
    t_c = np.arange(boost, sol.t[-1], 1.0 / fs)
    h_c = sol.sol(t_c)[0]
    t = np.concatenate([t_b, t_c])
    h = np.concatenate([h_b, h_c])
    # a short descent so the apogee is interior to the log
    t = np.concatenate([t, t[-1] + np.arange(1, 40) / fs])
    h = np.concatenate([h, h[-1] - 0.5 * G * (np.arange(1, 40) / fs) ** 2])
    rng = np.random.default_rng(seed)
    truth_f = np.interp(t_c, sol.t, np.array([rhs(0, y)[1] for y in sol.y.T]))  # dv/dt along the coast
    return t, h + rng.normal(0.0, noise, len(h)), (t_c, truth_f)


# ---------------------------------------------------------------------------------------- user TODO
@pytest.mark.user_todo
@pytest.mark.xfail(strict=True, raises=NotImplementedError, reason="TODO(you): implement drag_fit.solve_cd")
def test_todo_solve_cd_hand_value():
    """m = 0.6 kg, rho = 1.0, v = 100 m/s, S = 0.004 m^2, a_D = 20 m/s^2  ->  Cd = 0.6 (docs/derivations/drag_from_coast.md)."""
    assert drag_fit.solve_cd(0.6, 1.0, 100.0, 0.004, 20.0) == pytest.approx(0.6)
    assert drag_fit.solve_cd(0.6, 1.0, 100.0, 0.004, 0.0) == pytest.approx(0.0)  # no deceleration, no drag
    cd = drag_fit.solve_cd(
        np.array([0.6, 0.6]), np.array([1.0, 1.0]), np.array([100.0, 50.0]), 0.004, np.array([20.0, 5.0])
    )
    assert cd == pytest.approx(
        [0.6, 0.6]
    )  # works on arrays too (second element: 2 * 0.6 * 5 / (50^2 * 0.004) = 0.6)


def test_oracle_hand_value():
    assert oracle_cd(0.6, 1.0, 100.0, 0.004, 20.0) == pytest.approx(0.6)


# ------------------------------------------------------------------------------------ recovery
def test_baro_route_recovers_a_known_cd(with_oracle):
    errs = []
    for seed in range(6):
        t, h, _ = synthetic_flight(cd=0.55, seed=seed)
        r = estimate_cd(t, h, mass_kg=MASS, ref_diameter_m=DIAMETER)
        assert r.bins and r.window[0] > 2.3  # starts after burnout (t = 2 s) plus the skip
        errs.append(r.combined()[0] / 0.55 - 1.0)
    assert abs(np.mean(errs)) < 0.05, errs
    assert max(abs(e) for e in errs) < 0.12, errs


def test_accelerometer_route_is_more_precise(with_oracle):
    t, h, (t_c, dvdt) = synthetic_flight(cd=0.55, seed=3)
    rng = np.random.default_rng(9)
    t_acc = np.arange(0.0, t[-1], 0.01)
    f_true = np.interp(
        t_acc, t_c, dvdt + G, left=0.0, right=0.0
    )  # specific force along the nose during the coast
    f_x = f_true + rng.normal(0.0, 0.1, len(t_acc))
    r = estimate_cd(t, h, mass_kg=MASS, ref_diameter_m=DIAMETER, accel_axial=(t_acc, f_x))
    cd = r.combined()[0]
    assert cd == pytest.approx(0.55, rel=0.04)
    assert "accelerometer" in r.method


def test_the_reported_random_error_matches_the_scatter(with_oracle):
    cds, ses = [], []
    for seed in range(12):
        t, h, _ = synthetic_flight(cd=0.55, seed=seed)
        r = estimate_cd(t, h, mass_kg=MASS, ref_diameter_m=DIAMETER, mach_bin_width=1.0, n_bootstrap=200)
        (b,) = r.bins
        cds.append(b.cd)
        ses.append(b.se_random)
    ratio = np.std(cds, ddof=1) / np.mean(ses)
    assert 0.4 < ratio < 2.5, ratio  # the bootstrap standard error is the right order of magnitude


def test_systematic_error_adds_in_quadrature(with_oracle):
    t, h, _ = synthetic_flight()
    r = estimate_cd(
        t,
        h,
        mass_kg=MASS,
        ref_diameter_m=DIAMETER,
        rel_sigma_mass=0.02,
        rel_sigma_area=0.01,
        rel_sigma_rho=0.03,
    )
    b = r.bins[0]
    assert b.se_systematic == pytest.approx(abs(b.cd) * math.sqrt(0.02**2 + 0.01**2 + 0.03**2))
    assert b.se_total == pytest.approx(math.hypot(b.se_random, b.se_systematic))
    r2 = estimate_cd(t, h, mass_kg=MASS, ref_diameter_m=DIAMETER)
    assert r2.bins[0].se_systematic is None and r2.bins[0].se_total is None
    assert any("systematic" in n for n in r2.notes)


def test_report_text_lists_every_bin(with_oracle):
    t, h, _ = synthetic_flight()
    r = estimate_cd(t, h, mass_kg=MASS, ref_diameter_m=DIAMETER)
    text = r.report()
    assert "drag from coast" in text and f"{r.bins[0].n:4d}" in text


# ------------------------------------------------------------------------------------ simulator
def test_recovers_the_cd_used_by_the_simulator(with_oracle):
    """End to end: simulated barometer (noise, rate) from the 6-DOF simulation with a constant-Cd aerodynamic model."""
    p = default_config()
    cfg = config_from_dict(
        apply_overrides(
            read_mapping(p),
            {
                "fidelity": 4,
                "rocket.aero.model": "simplified",
                "rocket.aero.cd": 0.50,
                "environment.wind.model": "none",
            },
        ),
        base_dir=p.parent,
    )
    rec = Simulation(cfg, seed=1).run()
    new = rec.col("meas_baro_new") > 0
    t, h = rec.col("t")[new], rec.col("meas_baro_altitude")[new]
    burnout = rec.summary["burnout_time_s"]
    mass = float(rec.col("mass")[np.searchsorted(rec.col("t"), burnout) + 5])
    r = estimate_cd(t, h, mass_kg=mass, ref_diameter_m=cfg.rocket.body_diameter_m)
    cd = r.combined()[0]
    assert cd == pytest.approx(0.50, rel=0.15)
    assert r.window[0] > burnout


# --------------------------------------------------------------------------------------- inputs
def test_bad_inputs_are_rejected(with_oracle):
    t, h, _ = synthetic_flight()
    with pytest.raises(ConfigError, match="equal length"):
        estimate_cd(t, h[:-1], mass_kg=MASS, ref_diameter_m=DIAMETER)
    with pytest.raises(ConfigError, match="strictly increasing"):
        estimate_cd(np.r_[t[:10], t[9], t[10:-1]], h, mass_kg=MASS, ref_diameter_m=DIAMETER)
    with pytest.raises(ConfigError, match="mass_kg"):
        estimate_cd(t, h, mass_kg=0.0, ref_diameter_m=DIAMETER)
    with pytest.raises(ConfigError, match="coast"):
        estimate_cd(t, np.linspace(0.0, 100.0, len(t)), mass_kg=MASS, ref_diameter_m=DIAMETER)
    with pytest.raises(ConfigError, match="no Mach bin"):
        estimate_cd(t, h, mass_kg=MASS, ref_diameter_m=DIAMETER, min_samples_per_bin=10_000)


def test_a_gap_in_the_log_is_reported(with_oracle):
    t, h, _ = synthetic_flight()
    keep = (t < 3.0) | (t > 3.4)
    r = estimate_cd(t[keep], h[keep], mass_kg=MASS, ref_diameter_m=DIAMETER)
    assert any("gaps" in n for n in r.notes)


def test_solve_cd_is_required(monkeypatch):
    t, h, _ = synthetic_flight()
    with pytest.raises(NotImplementedError, match="solve_cd"):
        estimate_cd(t, h, mass_kg=MASS, ref_diameter_m=DIAMETER)


def test_every_bin_has_a_nonzero_bootstrap_error_and_covers_the_truth(with_oracle):
    """Regression: bins with fewer samples than one bootstrap block used to resample identical data (error = 0)."""
    for seed in range(4):
        t, h, _ = synthetic_flight(cd=0.55, seed=seed)
        r = estimate_cd(t, h, mass_kg=MASS, ref_diameter_m=DIAMETER)
        assert all(b.se_random > 1e-4 for b in r.bins), [b.se_random for b in r.bins]
        cd, se = r.combined()
        assert abs(cd - 0.55) < 3 * se, (cd, se)
