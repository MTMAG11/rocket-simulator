"""Flight-log importer: units, time base and wraparound, gaps and duplicates, saturation, launch detection, and `validate`."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from rocket_sim.cli import main
from rocket_sim.config import apply_overrides, config_from_dict, read_mapping
from rocket_sim.constants import G0
from rocket_sim.environment.atmosphere import ISAAtmosphere
from rocket_sim.errors import ConfigError
from rocket_sim.resources import default_config
from rocket_sim.simulation import Simulation
from rocket_sim.validation import flightlog
from rocket_sim.validation.compare import run_validation
from rocket_sim.validation.flightlog import import_flight_log, load_log_format, write_import

ISA = ISAAtmosphere()


def isa_altitude(p, p0):
    """Library oracle (NOT the owner's TODO): the repository's ISA inversion, relative to the pad pressure."""
    return np.array(
        [ISA.pressure_to_altitude(float(x)) for x in np.atleast_1d(p)]
    ) - ISA.pressure_to_altitude(float(p0))


# ----------------------------------------------------------------------------------- the TODO
@pytest.mark.user_todo
@pytest.mark.xfail(
    strict=True, raises=NotImplementedError, reason="TODO(you): implement flightlog.pressure_to_altitude_m"
)
def test_todo_pressure_to_altitude_hand_value():
    """p0 = 101325 Pa, p = 100000 Pa -> 110.88 m (docs/derivations/baro_altitude.md)."""
    f = flightlog.pressure_to_altitude_m
    assert f(100000.0, 101325.0) == pytest.approx(110.88, abs=0.05)
    assert f(101325.0, 101325.0) == pytest.approx(0.0, abs=1e-9)
    assert f(np.array([100000.0, 95000.0]), 101325.0) == pytest.approx([110.88, 540.34], abs=0.1)


def test_the_hand_value_agrees_with_the_library_oracle():
    assert float(isa_altitude(100000.0, 101325.0)[0]) == pytest.approx(110.88, abs=0.05)
    assert float(isa_altitude(95000.0, 101325.0)[0]) == pytest.approx(540.34, abs=0.1)


# --------------------------------------------------------------------------- synthetic raw logs
class Raw:
    """A made-up logger layout (NOT the owner's): wrapping 32-bit microsecond clock, a counter, accelerometer in g with the board
    mounted upside down (axial = -az), gyro in deg/s, pressure in hPa updated at half the log rate."""

    HEADER = ["us", "seq", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "p_hpa"]
    FORMAT = {
        "name": "synthetic_logger",
        "time_column": "us",
        "time_unit": "us",
        "time_wrap": 4294967296,
        "counter_column": "seq",
        "accel_columns": ["ay_g", "ax_g", "az_g"],
        "accel_unit": "g",
        "accel_axial": 2,
        "accel_axial_sign": -1,
        "accel_full_scale": 16.0,
        "gyro_columns": ["gx_dps", "gy_dps", "gz_dps"],
        "gyro_unit": "deg/s",
        "pressure_column": "p_hpa",
        "pressure_unit": "hPa",
        "pressure_repeats_are_holds": True,
        "pad_s": 1.5,
    }


@pytest.fixture(scope="module")
def sim():
    p = default_config()
    cfg = config_from_dict(
        apply_overrides(
            read_mapping(p),
            {
                "fidelity": 4,
                "motor.ignition_delay_s": 2.0,
                "environment.wind.model": "none",
                "simulation.stop_after_apogee_s": 3.0,
            },
        ),
        base_dir=p.parent,
    )
    return Simulation(cfg, seed=3).run()


def held(rec, name, t):
    """Value of the (sample-and-hold) logged column at times ``t``."""
    ts = rec.col("t")
    idx = np.clip(np.searchsorted(ts, t, side="right") - 1, 0, len(ts) - 1)
    return rec.col(name)[idx]


def make_log(
    rec, path: Path, rate=100.0, drop=0.01, gap=(6.0, 0.3), dup=5, seed=0, start_us=2**32 - 3_000_000
):
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, float(rec.col("t")[-1]), 1.0 / rate)
    g = lambda c: held(rec, c, t)  # noqa: E731
    ax, ay, az = g("meas_accel_x") / G0, g("meas_accel_y") / G0, g("meas_accel_z") / G0
    raw_az = (
        -ax
    )  # the logger's az_g axis points tail-ward: the simulator's axial reading appears with the opposite sign
    pressure = g("meas_baro_pressure") / 100.0
    rows = []
    for i in range(len(t)):
        us = int(round(t[i] * 1e6) + start_us) % 2**32
        rows.append(
            [
                us,
                i,
                ay[i],
                az[i],
                raw_az[i],
                *(np.degrees([g("meas_gyro_x")[i], g("meas_gyro_y")[i], g("meas_gyro_z")[i]])),
                pressure[i],
            ]
        )
    keep = rng.random(len(rows)) > drop
    in_gap = (t >= gap[0]) & (t < gap[0] + gap[1])
    keep &= ~in_gap
    keep[0] = True
    kept = [r for r, k in zip(rows, keep, strict=True) if k]
    n_dropped = len(rows) - len(kept)
    lines = [",".join(Raw.HEADER)]
    for k, r in enumerate(kept):
        lines.append(",".join(f"{v:.9g}" if isinstance(v, float | np.floating) else str(v) for v in r))
        if k in range(100, 100 + dup):  # duplicated rows
            lines.append(lines[-1])
    lines.insert(60, ",".join(Raw.HEADER))  # a repeated header
    swap = 200  # one out-of-order row
    lines[swap], lines[swap + 1] = lines[swap + 1], lines[swap]
    lines.append("123456,77")  # truncated last line
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {"n_dropped": n_dropped, "dups": dup, "rate": rate, "t": t, "gap": gap}


def fmt_file(tmp_path: Path, **changes) -> Path:
    d = {**Raw.FORMAT, **changes}
    d = {k: v for k, v in d.items() if v is not None}
    p = tmp_path / "format.yaml"
    p.write_text(yaml.safe_dump(d), encoding="utf-8")
    return p


@pytest.fixture(scope="module")
def imported(sim, tmp_path_factory):
    tmp = tmp_path_factory.mktemp("log")
    info = make_log(sim, tmp / "raw.csv")
    fmt = load_log_format(fmt_file(tmp))
    res = import_flight_log(tmp / "raw.csv", fmt, pressure_to_altitude=isa_altitude)
    return res, info, tmp


# ------------------------------------------------------------------------------ the checks
def test_time_base_wraparound_duplicates_order_and_bad_rows(imported):
    res, info, _ = imported
    r = res.report
    assert r.n_wraps == 1  # the 32-bit microsecond counter wrapped once during the log
    assert r.n_duplicate_times == info["dups"]
    assert r.n_out_of_order == 1
    assert r.n_bad_rows == 2  # the repeated header and the truncated last line
    assert np.all(np.diff(res.t) > 0)
    assert r.median_dt_s == pytest.approx(0.01, rel=0.05)


def test_gaps_and_dropped_samples_are_reported_not_filled(imported):
    res, info, _ = imported
    r = res.report
    assert len(r.gaps) == 1
    t0, dur, missing = r.gaps[0]
    assert dur == pytest.approx(0.3, abs=0.03) and missing == pytest.approx(29, abs=3)
    assert t0 > 0  # a time on the log's own clock
    # the counter sees every missing row, including those inside the gap (the out-of-order swap does not change the total)
    assert (
        r.n_dropped_by_counter == info["n_dropped"] + 1
    )  # +1: the row that was swapped out of order is discarded
    assert any("gap" in w for w in r.warnings)
    # no sample was invented: the gap is a hole in the time axis
    d = np.diff(res.t)
    assert d.max() == pytest.approx(0.3, abs=0.03)


def test_units_axes_and_sign(imported, sim):
    res, _, _ = imported
    # axial +1 g at rest on the pad, whatever the mounting: the sign and the column are in the format file
    pad = res.t < -0.5
    assert np.median(res.accel_axial[pad]) == pytest.approx(G0, abs=0.5)
    assert res.report.pad_accel_g == pytest.approx(1.0, abs=0.05)
    # the imported axial acceleration equals the simulated accelerometer reading at the same instant
    t_sim = res.t - res.t[0]  # simulated seconds: the log starts at simulated t = 0
    # a log time that falls exactly on a simulator row time may take either neighbouring held value (float rounding), so
    # accept the value from just before or just after it
    ok = np.isfinite(res.accel_axial_valid)

    def match(name, got, tol):
        lo, hi = held(sim, name, t_sim - 1e-7), held(sim, name, t_sim + 1e-7)
        return np.minimum(np.abs(got - lo), np.abs(got - hi)) < tol

    assert match("meas_accel_x", res.accel_axial, 1e-3)[ok].mean() > 0.999
    assert res.gyro is not None
    assert match("meas_gyro_x", res.gyro[:, 0], 1e-5)[ok].mean() > 0.999  # deg/s converted to rad/s


def test_saturated_samples_are_flagged_and_excluded(imported, sim):
    res, _, _ = imported
    full = 0.995 * 16.0 * G0
    expect = np.abs(res.accel_axial) >= full - 1e-6
    assert res.saturated.any() and res.report.n_saturated == int(res.saturated.sum())
    assert np.all(res.saturated[expect])
    assert np.all(np.isnan(res.accel_axial_valid[res.saturated]))
    assert np.all(np.isfinite(res.accel_axial_valid[~res.saturated]))
    assert res.report.saturated_intervals and any("saturat" in w for w in res.report.warnings)
    assert res.accel_axial_valid[~res.saturated].max() < 16.0 * G0


def test_saturation_flag_column(tmp_path, sim):
    make_log(sim, tmp_path / "raw.csv", drop=0.0, gap=(1e9, 0), dup=0)
    text = (tmp_path / "raw.csv").read_text().splitlines()
    # append a flag column that is 1 on rows 300-320 only, and no full-scale detection
    out = [text[0] + ",sat"]
    for i, line in enumerate(text[1:], start=1):
        out.append(line + ("," + ("1" if 300 <= i < 320 else "0") if line.count(",") == 8 else ""))
    (tmp_path / "raw.csv").write_text("\n".join(out) + "\n")
    fmt = load_log_format(fmt_file(tmp_path, accel_full_scale=None, accel_saturation_flag="sat"))
    res = import_flight_log(tmp_path / "raw.csv", fmt, pressure_to_altitude=isa_altitude)
    assert res.report.n_saturated >= 18 and np.all(np.isnan(res.accel_axial_valid[res.saturated]))


def test_launch_is_detected_from_the_acceleration(imported, sim):
    res, _, _ = imported
    fsp = np.sqrt(sum(sim.col(f"fsp_{a}") ** 2 for a in "xyz")) / G0
    t_sim = sim.col("t")
    t_truth = float(t_sim[np.argmax(fsp > 2.5)])  # first true specific force above 2.5 g
    # t = 0 of the imported time axis is the launch, and the log starts at simulated t = 0, so -res.t[0] is the simulated launch time
    assert -res.t[0] == pytest.approx(t_truth, abs=0.05)
    assert res.t[np.argmin(np.abs(res.t))] == pytest.approx(0.0, abs=0.01)  # t = 0 is launch
    assert res.accel_axial[res.t > 0.2][:3].max() > 2.0 * G0  # and the boost follows


def test_altitude_and_velocity_from_pressure(imported, sim):
    res, _, _ = imported
    t_sim = res.t - res.t[0]
    ok = np.isfinite(res.altitude)
    alt_true = held(sim, "altitude", t_sim)
    err = res.altitude[ok] - alt_true[ok]
    assert np.sqrt(np.mean(err**2)) < 2.0, np.sqrt(np.mean(err**2))
    assert abs(np.nanmax(res.altitude) - sim.summary["apogee_m"]) < 5.0
    vz = held(sim, "vel_z", t_sim)
    okv = ok & (res.t > 0.5) & (res.t < 10.0)
    assert np.sqrt(np.mean((res.velocity_z[okv] - vz[okv]) ** 2)) < 5.0
    assert res.report.pad_pressure_pa == pytest.approx(101325.0, rel=0.01)


def test_pressure_holds_are_not_new_samples(imported):
    res, _, _ = imported
    n_p = int(np.isfinite(res.pressure).sum())
    assert 0.3 * len(res.t) < n_p < 0.75 * len(res.t)  # about half the rows carry a new pressure sample


def test_the_pressure_function_todo_is_required_for_a_log_with_pressure(tmp_path, sim):
    make_log(sim, tmp_path / "raw.csv")
    fmt = load_log_format(fmt_file(tmp_path))
    with pytest.raises(NotImplementedError, match="pressure_to_altitude_m"):
        import_flight_log(tmp_path / "raw.csv", fmt)
    nop = load_log_format(fmt_file(tmp_path, pressure_column=None, pressure_unit="Pa"))
    res = import_flight_log(tmp_path / "raw.csv", nop)  # no pressure column: nothing to convert
    assert res.altitude is None and res.report.launch_time_s is not None


# ----------------------------------------------------------------------------- other conventions
def test_other_time_and_accel_units_without_a_wrap(tmp_path):
    rng = np.random.default_rng(1)
    t = np.arange(0.0, 6.0, 0.005)
    a = np.full_like(t, G0)
    a[t > 3.0] = 6 * G0
    lines = ["ms,a_ms2"] + [
        f"{ti * 1000:.3f},{ai + rng.normal(0, 0.05):.4f},0,0" for ti, ai in zip(t, a, strict=True)
    ]
    lines[0] = "ms,ax,ay,az"
    (tmp_path / "log.csv").write_text("\n".join(lines))
    fmt = load_log_format(
        fmt_file(
            tmp_path,
            time_column="ms",
            time_unit="ms",
            time_wrap=None,
            counter_column=None,
            accel_columns=["ax", "ay", "az"],
            accel_unit="m/s2",
            accel_axial=0,
            accel_axial_sign=1,
            accel_full_scale=None,
            gyro_columns=[],
            pressure_column=None,
        )
    )
    res = import_flight_log(tmp_path / "log.csv", fmt)
    assert res.report.n_wraps == 0 and res.report.n_saturated == 0
    assert res.report.launch_time_s == pytest.approx(3.0, abs=0.02)
    assert res.report.pad_accel_g == pytest.approx(1.0, abs=0.01)


def test_wrong_acceleration_unit_is_noticed(tmp_path):
    t = np.arange(0.0, 4.0, 0.01)
    (tmp_path / "log.csv").write_text(
        "t,ax,ay,az\n" + "\n".join(f"{x},9.81,0,0" for x in t)
    )  # m/s^2, declared as g
    fmt = load_log_format(
        fmt_file(
            tmp_path,
            time_column="t",
            time_unit="s",
            time_wrap=None,
            counter_column=None,
            accel_columns=["ax", "ay", "az"],
            accel_axial=0,
            accel_axial_sign=1,
            accel_full_scale=None,
            gyro_columns=[],
            pressure_column=None,
        )
    )
    res = import_flight_log(tmp_path / "log.csv", fmt)
    assert any("not about 1 g" in w for w in res.report.warnings)
    assert (
        res.report.launch_time_s is not None
    )  # 9.81 "g" looks like a launch, which is exactly why the warning matters


def test_a_log_without_a_launch_is_reported(tmp_path):
    t = np.arange(0.0, 4.0, 0.01)
    (tmp_path / "log.csv").write_text("t,ax,ay,az\n" + "\n".join(f"{x},1.0,0,0" for x in t))
    fmt = load_log_format(
        fmt_file(
            tmp_path,
            time_column="t",
            time_unit="s",
            time_wrap=None,
            counter_column=None,
            accel_columns=["ax", "ay", "az"],
            accel_axial=0,
            accel_axial_sign=1,
            accel_full_scale=None,
            gyro_columns=[],
            pressure_column=None,
        )
    )
    res = import_flight_log(tmp_path / "log.csv", fmt)
    assert res.report.launch_time_s is None and any("no launch" in w for w in res.report.warnings)
    assert res.t[0] == 0.0  # time is then relative to the first sample


# ------------------------------------------------------------------------------------- the format
def test_format_validation(tmp_path):
    base = fmt_file(tmp_path)
    assert load_log_format(base).name == "synthetic_logger"
    for bad, msg in (
        ({"time_unit": "min"}, "time_unit"),
        ({"accel_columns": ["a", "b"]}, "three columns"),
        ({"accel_axial_sign": 2}, "accel_axial"),
        ({"launch_threshold_g": 0.0}, "launch_threshold_g"),
    ):
        with pytest.raises(ConfigError, match=msg):
            load_log_format(fmt_file(tmp_path, **bad))
    p = tmp_path / "typo.yaml"
    p.write_text(yaml.safe_dump({**Raw.FORMAT, "accel_unit_typo": "g"}))
    with pytest.raises(ConfigError, match="unknown log-format keys"):
        load_log_format(p)
    q = tmp_path / "missing.yaml"
    q.write_text(yaml.safe_dump({"name": "x"}))
    with pytest.raises(ConfigError, match="time_column"):
        load_log_format(q)


def test_missing_column_and_file_errors(tmp_path):
    (tmp_path / "log.csv").write_text("a,b\n" + "\n".join("1,2" for _ in range(30)))
    with pytest.raises(ConfigError, match="not in the header"):
        import_flight_log(tmp_path / "log.csv", load_log_format(fmt_file(tmp_path)))
    with pytest.raises(ConfigError, match="not found"):
        import_flight_log(tmp_path / "none.csv", load_log_format(fmt_file(tmp_path)))


# ---------------------------------------------------------------------------- validate end to end
def test_imported_log_feeds_rocketsim_validate(imported, sim, tmp_path):
    res, _, _ = imported
    paths = write_import(res, tmp_path, "flight")
    d = yaml.safe_load(paths["flight"].read_text())
    assert (
        d["sim_config"].startswith("TODO")
        and d["telemetry"]["channels"]["altitude"]["column"] == "altitude_m"
    )
    d["sim_config"] = str(default_config())
    d["sim_overrides"] = {"fidelity": 3, "environment.wind.model": "none"}
    d["name"] = "synthetic"
    d["telemetry"]["landing_altitude_m"] = 5.0
    d["telemetry"]["compare_landing"] = False
    paths["flight"].write_text(yaml.safe_dump(d))
    out = run_validation(paths["flight"], plot=False)
    ap = out.metrics["apogee"]
    assert abs(ap["pct_error"]) < 5.0, (
        ap
    )  # the "real" log came from the same model, through sensors and file conversion
    assert out.metrics["alignment"]["channel"] == "altitude"


def test_cli_import_log(sim, tmp_path, capsys):
    make_log(sim, tmp_path / "raw.csv", seed=2)
    fmt = fmt_file(tmp_path, pressure_column=None, pressure_unit="Pa")
    assert (
        main(["import-log", str(tmp_path / "raw.csv"), "--format", str(fmt), "--out", str(tmp_path / "out")])
        == 0
    )
    out = capsys.readouterr().out
    assert "launch at" in out and "edit the TODO fields" in out
    assert (tmp_path / "out" / "raw.csv").exists() and (tmp_path / "out" / "raw_import.json").exists()
    assert (tmp_path / "out" / "raw.flight.yaml").exists()
    full = fmt_file(tmp_path)
    assert (
        main(["import-log", str(tmp_path / "raw.csv"), "--format", str(full), "--out", str(tmp_path / "o2")])
        == 1
    )
    assert "pressure_to_altitude_m" in capsys.readouterr().err


def test_csv_round_trip_keeps_nan_as_empty(imported, tmp_path):
    res, _, _ = imported
    p = res.to_csv(tmp_path / "x.csv")
    rows = p.read_text().splitlines()
    assert rows[0].startswith("time_s,accel_axial_ms2,accel_axial_valid_ms2")
    sat_row = next(r for r in rows[1:] if r.split(",")[3] == "1")
    assert sat_row.split(",")[2] == ""  # saturated sample: empty valid column
    assert math.isclose(float(rows[1].split(",")[0]), float(res.t[0]), rel_tol=1e-8, abs_tol=1e-8)


def test_channel_loader_skips_rows_with_an_empty_value_without_misaligning_time(tmp_path):
    """Regression: a row with a valid time but an empty value used to leave the time and value arrays different lengths."""
    from rocket_sim.validation.telemetry import load_channel

    p = tmp_path / "wide.csv"
    p.write_text("t,a,b\n1,10,\n2,,5\n3,30,6\n4,40,7\n5,50,8\n6,nan,9\n", encoding="utf-8")
    a = load_channel(p, ",", {"time_column": "t", "column": "a"}, "m")
    b = load_channel(p, ",", {"time_column": "t", "column": "b"}, "m")
    assert list(a.t) == [1.0, 3.0, 4.0, 5.0] and list(a.y) == [10.0, 30.0, 40.0, 50.0]
    assert list(b.t) == [2.0, 3.0, 4.0, 5.0, 6.0] and list(b.y) == [5.0, 6.0, 7.0, 8.0, 9.0]
