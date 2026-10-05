"""Validation tooling, uncertainty formatting and the command-line interface."""

import json
from pathlib import Path

import numpy as np
import pytest

from rocket_sim.cli import main
from rocket_sim.errors import ConfigError
from rocket_sim.simulation import run_simulation
from rocket_sim.uncertainty import format_value
from rocket_sim.validation.compare import compare_flight, run_validation
from rocket_sim.validation.metrics import crossing_time, scalar_error, series_error, smooth_derivative
from rocket_sim.validation.telemetry import Channel, Telemetry, load_telemetry, telemetry_from_record
from tests.conftest import CONFIG_G80, ROOT, cfg_from

VAL = ROOT / "validation_data"


def test_series_metrics_against_hand_computation():
    real = np.array([0.0, 10.0, 20.0, 30.0])
    sim = np.array([1.0, 9.0, 22.0, 30.0])
    e = series_error(real, sim)
    assert e.rmse == pytest.approx(np.sqrt((1 + 1 + 4 + 0) / 4)) and e.mae == pytest.approx(1.0)
    assert (
        e.max_abs == 2.0 and e.bias == pytest.approx(0.5) and e.nrmse_pct == pytest.approx(100 * e.rmse / 30)
    )
    s = scalar_error(100.0, 95.0)
    assert s["abs_error"] == -5.0 and s["pct_error"] == pytest.approx(-5.0)


def test_crossing_and_derivative_helpers():
    t = np.linspace(0, 10, 101)
    assert crossing_time(t, t**2, 25.0) == pytest.approx(5.0, abs=0.02)
    assert crossing_time(t, 100 - t**2, 75.0, rising=False) == pytest.approx(5.0, abs=0.02)
    d = smooth_derivative(t, 3 * t**2, 1.0)
    assert d[50] == pytest.approx(30.0, rel=1e-6)  # exact for quadratics


def test_telemetry_import_with_unit_scaling_and_bad_inputs(tmp_path):
    p = tmp_path / "log.csv"
    p.write_text("t, alt_ft\n0,0\n1,100\n2,300\nbad,row\n3,500\n")
    tel = load_telemetry(p, {"altitude": {"time_column": "t", "column": "alt_ft", "scale": 0.3048}})
    assert tel["altitude"].y[-1] == pytest.approx(152.4) and len(tel["altitude"].t) == 4  # bad row skipped
    with pytest.raises(ConfigError, match="not found"):
        load_telemetry(tmp_path / "x.csv", {"altitude": {"time_column": 0, "column": 1}})
    with pytest.raises(ConfigError, match="unknown telemetry channel"):
        load_telemetry(p, {"bogus": {"time_column": 0, "column": 1}})
    with pytest.raises(ConfigError, match="not in header"):
        load_telemetry(p, {"altitude": {"time_column": "t", "column": "nope"}})


def test_simulated_telemetry_representation_and_csv_export(tmp_path):
    rec = run_simulation(cfg_from(fidelity=3), seed=0)
    tel = telemetry_from_record(rec)
    assert set(tel.channels) == {
        "altitude",
        "altitude_baro",
        "velocity_z",
        "speed",
        "accel_axial",
        "pressure",
    }
    assert tel["accel_axial"].y[0] == pytest.approx(9.80665, abs=0.05)  # +g specific force on the pad
    files = tel.to_csv(tmp_path)
    assert len(files) == 6 and files[0].read_text().startswith("time_s,altitude_m")
    with pytest.raises(ConfigError, match="sensors"):
        telemetry_from_record(rec, use_sensors=True)


def test_self_consistency_simulated_flight_vs_itself_has_zero_error():
    """Pipeline sanity: comparing a simulation against its own telemetry gives ~0 error."""
    rec = run_simulation(cfg_from(fidelity=3), seed=0)
    sim = telemetry_from_record(rec)
    res = compare_flight(
        Telemetry({k: Channel(v.t.copy(), v.y.copy(), v.unit) for k, v in sim.channels.items()}),
        rec,
        {"name": "self", "telemetry": {"launch_threshold_altitude_m": 15, "altitude_reference": "geometric"}},
    )
    m = res.metrics
    assert m["apogee"]["abs_error"] == pytest.approx(0.0, abs=1e-9)
    assert m["altitude_all"]["rmse"] < 1e-6 and m["alignment"]["shift_s"] == pytest.approx(0.0, abs=1e-9)
    assert m["accel_axial"]["rmse"] < 1e-3 and m["burnout_time"]["abs_error"] == pytest.approx(0.0, abs=1e-6)


@pytest.mark.slow
@pytest.mark.parametrize(
    "flight,limits",
    [
        ("bella_lui", {"apogee": 1.5, "alt_nrmse": 1.5}),
        ("ndrt_2020", {"apogee": 15.0, "alt_nrmse": 11.0}),
        ("prometheus", {"apogee": 6.0, "alt_nrmse": 4.5}),
    ],
)
def test_real_flight_regression(flight, limits):
    """Regression guard on the documented validation results (physics v1.1.0, uncalibrated)."""
    r = run_validation(VAL / f"{flight}.yaml", plot=False)
    assert abs(r.metrics["apogee"]["pct_error"]) < limits["apogee"]
    assert r.metrics["altitude_all"]["nrmse_pct"] < limits["alt_nrmse"]
    assert "source" in r.definition and r.definition["source"]["quality"]


def test_uncertainty_formatting_never_fakes_precision():
    s = format_value(801.4734197, "apogee_m", "m")
    assert "801.47" not in s and "+/-" in s and s.startswith("8")
    assert format_value(None, "apogee_m", "m") == "n/a"
    assert "+/-" not in format_value(3.14159, "unknown_metric", "x")
    assert "+/-" not in format_value(
        1.0, "apogee_m", "m", fidelity=1
    )  # fidelity < 2: no model uncertainty claimed


# ------------------------------------------------------------------------------------ CLI
def test_cli_simulate_exports_and_reports(tmp_path, capsys):
    rc = main(
        [
            "simulate",
            str(CONFIG_G80),
            "--out",
            str(tmp_path),
            "--format",
            "csv,parquet",
            "--seed",
            "3",
            "--set",
            "rocket.dry_mass_kg=0.5",
            "--fidelity",
            "2",
            "--dt",
            "0.02",
        ]
    )
    out = capsys.readouterr().out
    assert rc == 0 and "Apogee" in out and "+/-" in out and "fidelity=2" in out
    assert len(list(tmp_path.glob("*.csv"))) == 1 and len(list(tmp_path.glob("*.parquet"))) == 1


def test_cli_errors_are_clean(capsys):
    assert main(["simulate", "nope.yaml"]) == 1
    assert "error:" in capsys.readouterr().err
    assert main(["simulate", str(CONFIG_G80), "--set", "rocket.dry_mass_kg=-3"]) == 1
    assert "dry_mass_kg" in capsys.readouterr().err
    with pytest.raises(SystemExit):
        main(["simulate", str(CONFIG_G80), "--format"])


def test_cli_check_schema_motors(capsys):
    assert main(["check", str(CONFIG_G80)]) == 0 and "OK" in capsys.readouterr().out
    assert main(["schema"]) == 0 and "Telemetry schema" in capsys.readouterr().out
    assert main(["schema", "--json"]) == 0
    assert json.loads(capsys.readouterr().out)["schema_version"]
    assert main(["motors"]) == 0 and "G80NBT" in capsys.readouterr().out


def test_cli_batch_inspect_export(tmp_path, capsys):
    spec = tmp_path / "spec.yaml"
    spec.write_text(
        "name: cli\nbase_config: " + CONFIG_G80.as_posix() + "\nmaster_seed: 1\nshard_runs: 2\nworkers: 1\n"
        "overrides: {fidelity: 2, fast: true, simulation: {dt_s: 0.02, descent_dt_s: 0.1}}\n"
        "parameters:\n  - {path: rocket.dry_mass_kg, dist: normal, rel_std: 0.03}\n"
        "dataset: {kind: telemetry}\noutput_dir: " + (tmp_path / "out").as_posix() + "\n"
    )
    assert main(["batch", str(spec), "--runs", "4"]) == 0
    assert "4/4 accepted" in capsys.readouterr().out
    assert main(["inspect", str(tmp_path / "out")]) == 0
    assert "cli-train-00000003" in capsys.readouterr().out
    assert main(["inspect", str(tmp_path / "out"), "--run", "cli-train-00000001"]) == 0
    assert "rocket.dry_mass_kg" in capsys.readouterr().out
    assert (
        main(
            [
                "export",
                str(tmp_path / "out"),
                "--run",
                "cli-train-00000001",
                "--out",
                str(tmp_path / "x"),
                "--format",
                "parquet,csv",
            ]
        )
        == 0
    )
    assert (tmp_path / "x" / "cli-train-00000001.parquet").exists()
    assert main(["batch", str(spec), "--runs", "4", "--no-resume"]) == 1
    assert "already contains results" in capsys.readouterr().err


def test_main_entry_point_is_registered():
    import tomllib

    meta = tomllib.loads((ROOT / "pyproject.toml").read_text())
    assert meta["project"]["scripts"]["rocketsim"] == "rocket_sim.cli:main"
    assert Path(VAL / "bella_lui.yaml").exists()


def test_barometer_equivalent_altitude_known_pressures():
    from rocket_sim.environment import ISAAtmosphere
    from rocket_sim.validation.telemetry import _baro_equivalent_altitude

    isa = ISAAtmosphere()
    p = np.array([101325.0, isa.at(1000.0).pressure, isa.at(3000.0).pressure])
    h = _baro_equivalent_altitude(p)
    assert h == pytest.approx([0.0, 1000.0, 3000.0], abs=1e-3)
    # a warm day (+15 K) makes a standard barometer read HIGH-vs-geometric above the pad by ~ dT/T
    warm = ISAAtmosphere(temperature_offset=15.0)
    pw = np.array([warm.at(0.0).pressure, warm.at(3000.0).pressure])
    assert _baro_equivalent_altitude(pw)[1] == pytest.approx(3000.0 / (1 + 15.0 / 288.15), rel=0.02)


def test_self_consistency_barometric_mode():
    rec = run_simulation(cfg_from(fidelity=3), seed=0)
    sim = telemetry_from_record(rec)
    real = Telemetry({k: Channel(v.t.copy(), v.y.copy(), v.unit) for k, v in sim.channels.items()})
    real.channels["altitude"] = real.channels["altitude_baro"]  # a perfect barometric altimeter
    res = compare_flight(real, rec, {"name": "self", "telemetry": {"launch_threshold_altitude_m": 15}})
    assert res.metrics["apogee"]["abs_error"] == pytest.approx(0.0, abs=1e-9)
