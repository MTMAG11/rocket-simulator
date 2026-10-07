"""V1.2 aerodynamics: provenance of every coefficient, interface consistency, bounds, regression pins, RocketPy cross-check."""

import json
import math
import sys
import types
from pathlib import Path

import numpy as np
import pytest

from rocket_sim.config import config_from_dict, load_config
from rocket_sim.errors import ConfigError
from rocket_sim.simulation import run_simulation
from rocket_sim.simulation.builder import build_vehicle
from rocket_sim.vehicle.aero import (
    ESTIMATE_KINDS,
    PROVENANCE_KINDS,
    AeroProvenance,
    ProvItem,
    default_provenance,
)
from tests.conftest import cfg_from

ROOT = Path(__file__).resolve().parents[1]
DEMO = ROOT / "vehicles" / "example_tvc_demo.json"
TABLE = [[0.0, 0.50], [0.8, 0.55], [1.2, 0.80], [3.0, 0.60]]


def vehicle(**aero):
    return build_vehicle(cfg_from(fidelity=3, rocket={"aero": aero}))


# ------------------------------------------------------------------------------------------- provenance
@pytest.mark.parametrize(
    "aero,kind",
    [
        ({"model": "barrowman"}, "estimate"),
        ({"model": "buildup"}, "estimate"),
        ({"model": "enhanced"}, "estimate"),
        ({"model": "simplified", "cd": 0.5}, "mixed"),  # user drag + Barrowman stability
        ({"model": "constant", "cd": 0.5}, "mixed"),
        ({"model": "table", "cd_table": TABLE}, "mixed"),
        ({"model": "table", "cd_table": TABLE, "cn_alpha": 12.0, "x_cp_from_nose_m": 0.8}, "mixed"),
    ],
)
def test_every_model_declares_what_its_numbers_are(aero, kind):
    p = vehicle(**aero).aero.provenance
    assert isinstance(p, AeroProvenance)
    assert p.kind == kind
    d = p.to_dict()
    json.dumps(d)  # serialisable
    for item in ("drag", "normal_force_cp", "damping"):
        assert d[item]["kind"] in PROVENANCE_KINDS and d[item]["source"] and d[item]["confidence"]
    assert d["reynolds_dependence"]  # always stated, never left implicit


def test_estimates_are_never_labelled_as_measured_data():
    for aero in ({"model": "barrowman"}, {"model": "enhanced"}):
        p = vehicle(**aero).aero.provenance
        assert all(i.is_estimate for i in (p.drag, p.normal_force_cp, p.damping)) and p.kind == "estimate"
        assert p.drag.kind not in ("experimental", "cfd")
    assert set(ESTIMATE_KINDS) == {"analytical", "barrowman", "empirical"}
    assert (
        not ProvItem("experimental", "wind tunnel").is_estimate and not ProvItem("cfd", "run 7").is_estimate
    )


def test_declared_provenance_of_imported_data_is_carried_through():
    prov = {"kind": "experimental", "source": "wind tunnel run 12, 2026-05", "confidence": "high"}
    p = vehicle(model="table", cd_table=TABLE, provenance=prov).aero.provenance
    assert (p.drag.kind, p.drag.source, p.drag.confidence) == (
        "experimental",
        "wind tunnel run 12, 2026-05",
        "high",
    )
    assert p.mach_range == (0.0, 3.0)  # applicable Mach range taken from the table itself
    assert (
        p.normal_force_cp.kind == "barrowman"
    )  # stability still comes from geometry: not claimed to be measured
    assert p.kind == "mixed"  # so the dataset flag does not call it measured
    assert (
        vehicle(model="table", cd_table=TABLE).aero.provenance.drag.kind == "user_defined"
    )  # undeclared: unknown, stated


def test_unknown_provenance_kind_is_rejected():
    with pytest.raises(ConfigError, match="provenance kind"):
        default_provenance("table", {"kind": "magic"})
    with pytest.raises(ConfigError, match="provenance kind"):
        ProvItem("guess", "x")


def test_declared_reynolds_dependence_matches_actual_behaviour():
    """The provenance says how Re enters. Check that the code does exactly that and nothing else."""
    bar = vehicle(model="barrowman")
    assert "skin friction only" in bar.aero.provenance.reynolds_dependence
    c_lo, c_hi = bar.aero.coefficients(0.3, 2e5, False), bar.aero.coefficients(0.3, 5e7, False)
    assert c_lo.cd0 != pytest.approx(c_hi.cd0, rel=1e-3)  # drag depends on Re (skin friction)
    assert (c_lo.cn_alpha, c_lo.x_cp) == (c_hi.cn_alpha, c_hi.x_cp)  # lift slope and CP do not
    for aero in ({"model": "table", "cd_table": TABLE}, {"model": "constant", "cd": 0.5}):
        v = vehicle(**aero)
        assert "none" in v.aero.provenance.reynolds_dependence
        a, b = v.aero.coefficients(0.3, 2e5, False), v.aero.coefficients(0.3, 5e7, False)
        assert (a.cd0, a.cn_alpha, a.x_cp) == (b.cd0, b.cn_alpha, b.x_cp)  # truly Re-independent


def test_enhanced_model_carries_its_unvalidated_status_in_the_provenance():
    p = vehicle(model="enhanced").aero.provenance
    assert p.drag.confidence == "low" and any("NO real-flight validation" in n for n in p.notes)
    assert p.mach_range == (0.0, 2.0)
    b = vehicle(model="barrowman").aero.provenance
    assert b.mach_range == (0.0, 0.9) and b.drag.confidence == "medium"


def test_provenance_reaches_the_flight_record_and_the_dataset_rows(tmp_path):
    rec = run_simulation(cfg_from(fidelity=3, simulation={"t_max_s": 3}), seed=0)
    assert rec.meta.aero_provenance["kind"] == "estimate" and rec.meta.aero_provenance["model"] == "barrowman"
    from rocket_sim.config import config_to_dict, from_dict
    from rocket_sim.data.batch import read_runs, run_batch
    from rocket_sim.data.spec import BatchSpec

    spec = from_dict(
        BatchSpec,
        {
            "name": "prov",
            "config": config_to_dict(
                cfg_from(fidelity=2, fast=True, simulation={"dt_s": 0.02, "descent_dt_s": 0.1})
            ),
            "master_seed": 3,
            "shard_runs": 2,
            "workers": 1,
            "output_dir": str(tmp_path / "o"),
            "runs": 2,
            "parameters": [{"path": "rocket.dry_mass_kg", "dist": "normal", "rel_std": 0.02}],
            "dataset": {"kind": "telemetry"},
        },
    )
    run_batch(spec, tmp_path)
    rows = read_runs(tmp_path / "o").to_pylist()
    assert {r["aero_provenance_kind"] for r in rows} == {"estimate"} and {r["aero_model"] for r in rows} == {
        "barrowman"
    }
    assert {r["controller_state_source"] for r in rows} == {"none"}
    # reloading a saved record keeps the provenance
    from rocket_sim.data.export import export_record, read_record_parquet

    f = export_record(rec, tmp_path, "r", ("parquet",))[0]
    assert read_record_parquet(f).meta.aero_provenance["kind"] == "estimate"


# ---------------------------------------------------------------------- interface: coefficients, bounds, trends
MODELS = [
    {"model": "barrowman"},
    {"model": "enhanced"},
    {"model": "simplified", "cd": 0.5},
    {"model": "table", "cd_table": TABLE},
    {"model": "constant", "cd": 0.5},
]


@pytest.mark.parametrize("aero", MODELS, ids=lambda a: a["model"])
def test_coefficients_are_finite_bounded_and_consistent_over_the_whole_envelope(aero):
    v = vehicle(**aero)
    a = v.aero
    x_ref = v.mass_props(0.0).x_cg
    for mach in (0.0, 0.1, 0.5, 0.9, 1.1, 2.0, 3.0):
        c0 = a.coefficients(mach, 1e6, False)
        assert 0.0 < c0.cd0 < 2.5 and c0.cn_alpha > 0 and 0.0 < c0.x_cp < v.body.length
        for alpha in np.radians([0, 2, 5, 10, 20, 45, 90, 135, 180]):
            f = a.force_coefficients(mach, float(alpha), 1e6, False)
            cd, cl, cm = (
                a.cd(mach, float(alpha), 1e6),
                a.cl(mach, float(alpha), 1e6),
                a.cm(mach, float(alpha), 1e6, x_ref),
            )
            assert all(math.isfinite(x) for x in (f.ca, f.cn, cd, cl, cm))
            assert f.cn >= 0.0  # normal force is defined opposing the lateral velocity
            cn_max = (
                0.5 * c0.cn_alpha * 1.5 + a.crossflow_cd * a.planform_area / a.ref_area * 1.5 + 1e-9
            )  # slope x 0.5 x margin, + crossflow
            assert f.cn <= cn_max
            # wind-axis / body-axis rotation is exact
            assert cd == pytest.approx(f.ca * math.cos(alpha) + f.cn * math.sin(alpha), abs=1e-12)
            assert cl == pytest.approx(f.cn * math.cos(alpha) - f.ca * math.sin(alpha), abs=1e-12)
            if 0 < alpha <= math.radians(10) and v.static_margin(0.0) > 0:
                # small-angle statically stable nose-first flight: restoring moment. (At larger alpha (and supersonic, where the fin lift slope falls) the crossflow force acts at
                # the planform centroid, ahead of the CG, so the moment legitimately turns destabilising: not asserted.)
                assert cm <= 1e-12


def test_drag_rises_with_angle_of_attack_and_transonic_drag_rise_exists_for_the_analytic_models():
    for aero in ({"model": "barrowman"}, {"model": "enhanced"}):
        a = vehicle(**aero).aero
        cds = [a.cd(0.3, float(np.radians(x)), 1e6) for x in (0, 2, 5, 10, 15, 20)]
        assert all(b >= c for c, b in zip(cds, cds[1:])) and cds[-1] > 1.1 * cds[0]
        assert a.coefficients(1.1, 1e6, False).cd0 > 1.15 * a.coefficients(0.5, 1e6, False).cd0
    flat = vehicle(model="constant", cd=0.5).aero  # a constant model has NO such behaviour (and says so)
    assert flat.coefficients(1.1, 1e6, False).cd0 == flat.coefficients(0.1, 1e6, False).cd0


def test_barrowman_regression_pins_and_hand_calculation():
    g80 = build_vehicle(load_config(ROOT / "configs" / "example_g80.yaml")).aero
    c = g80.coefficients(0.1, 2e5, False)
    assert (c.cd0, c.cn_alpha, c.x_cp) == pytest.approx(
        (0.8112573264591236, 25.248367289163756, 0.8029411909556069), rel=1e-9
    )
    # CNa by hand: ogive nose 2 + fin set Barrowman (4 fins, span 0.07, root 0.10, tip 0.045, sweep 0.06) x body interference
    d, s, cr, ct, sw, n = 0.041, 0.07, 0.10, 0.045, 0.06, 4
    lf = math.hypot(s, sw + 0.5 * (ct - cr))  # fin mid-chord line length
    k = 1.0 + (0.5 * d) / (s + 0.5 * d)
    cn_fins = k * 4 * n * (s / d) ** 2 / (1.0 + math.sqrt(1.0 + (2.0 * lf / (cr + ct)) ** 2))
    assert c.cn_alpha == pytest.approx(2.0 + cn_fins, rel=1e-9)


def test_demo_vehicle_regression_pin():
    cfg = config_from_dict({"config_version": 1, "fidelity": 3, "vehicle_file": str(DEMO)}, base_dir=ROOT)
    c = build_vehicle(cfg).aero.coefficients(0.1, 2e5, False)
    assert (c.cd0, c.cn_alpha, c.x_cp) == pytest.approx(
        (0.5585063876267502, 13.8901295099331, 1.2027859223207906), rel=1e-9
    )


# --------------------------------------------------------------------------------------------- cross-check
def test_vehicle_file_stability_matches_rocketpy_on_the_same_geometry():
    """Independent code: RocketPy builds the SAME nose / fins / boat-tail from the vehicle file's numbers (CNa and CP)."""
    try:
        import netCDF4  # noqa: F401
    except ImportError:
        sys.modules["netCDF4"] = types.ModuleType("netCDF4")
    rocketpy = pytest.importorskip("rocketpy")
    d = json.loads(DEMO.read_text(encoding="utf-8"))
    comp = {c["id"]: c for c in d["components"]}
    length = 1.60  # airframe length; RocketPy origin at the aft end, z up toward the nose
    r = rocketpy.Rocket(
        radius=0.0381,
        mass=3.0,
        inertia=(0.01, 0.7, 0.7),
        power_off_drag=0.5,
        power_on_drag=0.5,
        center_of_mass_without_motor=0.6,
        coordinate_system_orientation="tail_to_nose",
    )
    r.add_nose(length=comp["nose"]["length_m"], kind="vonKarman", position=length)
    f = comp["fins"]
    r.add_trapezoidal_fins(
        n=f["count"],
        span=f["span_m"],
        root_chord=f["root_chord_m"],
        tip_chord=f["tip_chord_m"],
        sweep_length=f["sweep_m"],
        position=length - f["x_root_le_m"],
    )
    bt = comp["boat_tail"]
    r.add_tail(
        top_radius=0.5 * bt["fore_diameter_m"],
        bottom_radius=0.5 * bt["aft_diameter_m"],
        length=bt["length_m"],
        position=length - bt["x_start_m"],
    )
    cna_rp, cp_rp = float(r.total_lift_coeff_der(0.0)), length - float(r.cp_position(0.0))
    cfg = config_from_dict({"config_version": 1, "fidelity": 3, "vehicle_file": str(DEMO)}, base_dir=ROOT)
    c = build_vehicle(cfg).aero.coefficients(0.05, 5e6, False)
    assert c.cn_alpha == pytest.approx(
        cna_rp, rel=2e-3
    )  # same Barrowman equations, independent implementation
    assert (
        abs(c.x_cp - cp_rp) / 0.0762 < 0.06
    )  # CP within 0.06 cal (von Karman vs ogive nose CP is the known difference)


# ---------------------------------------------------------------------- stated ranges are enforced at run time
def test_flight_beyond_the_stated_mach_range_is_warned_about():
    cfg = config_from_dict(
        {"config_version": 1, "fidelity": 3, "vehicle_file": str(DEMO), "simulation": {"t_max_s": 30}},
        base_dir=ROOT,
    )
    rec = run_simulation(cfg, seed=0)
    assert rec.col("mach").max() > 0.9
    assert any("stated Mach range" in w and "barrowman" in w for w in rec.meta.warnings)
    slow = run_simulation(
        cfg_from(fidelity=3, simulation={"t_max_s": 30}), seed=0
    )  # example G80 peaks near Mach 0.6
    assert slow.col("mach").max() < 0.9 and not any("Mach range" in w for w in slow.meta.warnings)


def test_tumbling_flight_is_warned_about_the_angle_of_attack_range():
    rec = run_simulation(
        cfg_from(
            fidelity=3,
            rocket={"cg_from_nose_m": 0.80, "parachutes": []},
            motor={"misalignment_deg": [0.6, 0.0]},
            environment={"wind": {"model": "none"}},
            launch={"elevation_deg": 90.0, "rail_length_m": 1.5},
            simulation={"t_max_s": 6.0},
        ),
        seed=0,
    )
    assert np.degrees(rec.col("aoa")).max() > 15.0
    assert any("angle of attack reached" in w for w in rec.meta.warnings)


def test_nominal_flights_do_not_trigger_spurious_range_warnings():
    """Regression: the angle-of-attack check once counted the pad (speed 0, alpha = 90 deg) and warned on every flight."""
    rec = run_simulation(
        cfg_from(
            fidelity=3,
            environment={"wind": {"model": "constant", "speed_ms": 4.0, "direction_from_deg": 270}},
        ),
        seed=0,
    )
    assert rec.col("aoa").max() > np.radians(60)  # the pad artefact really is in the record...
    assert not any(
        "angle of attack reached" in w or "Mach range" in w for w in rec.meta.warnings
    )  # ...and is ignored
    for name in ("bella_lui", "ndrt_2020"):  # development flights are subsonic and nose-first
        from rocket_sim.config import load_config as lc

        r = run_simulation(lc(ROOT / "validation_data" / f"{name}_sim.yaml"), seed=0)
        assert not any("angle of attack reached" in w for w in r.meta.warnings), name


def test_table2d_provenance_takes_its_ranges_from_the_table_axes(tmp_path):
    rows = ["mach,alpha_deg,cd,cl"] + [
        f"{m},{a},{0.5 + 0.01 * a},{0.1 * a}" for m in (0.1, 0.5) for a in (0, 10, 20)
    ]
    f = tmp_path / "t.csv"
    f.write_text("\n".join(rows), encoding="utf-8")
    p = vehicle(model="table2d", table2d_file=str(f)).aero.provenance
    assert p.mach_range == pytest.approx((0.1, 0.5)) and p.alpha_range_deg == pytest.approx((0.0, 20.0))
    assert "none" in p.reynolds_dependence


def test_uncertainty_is_marked_extrapolated_outside_the_compared_envelope():
    from rocket_sim.reporting import outside_envelope, summary_text

    rec = run_simulation(
        config_from_dict(
            {"config_version": 1, "fidelity": 3, "vehicle_file": str(DEMO), "simulation": {"t_max_s": 30}},
            base_dir=ROOT,
        ),
        seed=0,
    )
    why = outside_envelope(rec)
    assert why and "Mach" in why and "kg" in why  # supersonic AND a 4.7 kg vehicle
    text = summary_text(rec)
    assert "EXTRAPOLATED" in text and "+/-" in text
    apogee_line = next(x for x in text.splitlines() if x.strip().startswith("Apogee (AGL)"))
    assert "*" in apogee_line  # provisional star even for the normally 'validated' apogee band
