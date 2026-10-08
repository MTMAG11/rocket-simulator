"""V1.2 vehicle file: component mass summation, CG, inertia, geometry placement, validation, determinism.

Expected values are computed here with plain formulas / numpy, not with the code under test.
"""

import copy
import json
import math
import shutil
from pathlib import Path

import numpy as np
import pytest

from rocket_sim.config import config_from_dict, config_hash
from rocket_sim.errors import ConfigError
from rocket_sim.simulation import run_simulation
from rocket_sim.simulation.builder import build_vehicle
from rocket_sim.vehicle.report import mass_properties_report
from rocket_sim.vehicle.vehicle_file import compile_vehicle, load_vehicle, shape_inertia

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "vehicles" / "example_tvc_demo.json"
RHO = 1850.0


def base_vehicle() -> dict:
    """A small, hand-checkable vehicle (all masses specified unless a test derives them)."""
    return {
        "format": "rocket-sim-vehicle",
        "format_version": 1,
        "metadata": {"name": "t", "data_quality": "design"},
        "materials": {"glass": RHO},
        "components": [
            {
                "id": "nose",
                "type": "nose",
                "shape": "ogive",
                "length_m": 0.2,
                "base_diameter_m": 0.06,
                "mass_kg": 0.05,
            },
            {"id": "tube", "type": "body_tube", "length_m": 0.8, "diameter_m": 0.06, "mass_kg": 0.30},
            {
                "id": "fins",
                "type": "fin_set",
                "count": 4,
                "root_chord_m": 0.1,
                "tip_chord_m": 0.05,
                "span_m": 0.06,
                "sweep_m": 0.05,
                "thickness_m": 0.002,
                "x_root_le_m": 0.85,
                "mass_kg": 0.08,
            },
            {
                "id": "avionics",
                "type": "avionics",
                "mass_kg": 0.10,
                "cg_x_m": 0.40,
                "shape": "box",
                "dimensions_m": [0.10, 0.05, 0.03],
            },
            {
                "id": "battery",
                "type": "battery",
                "mass_kg": 0.08,
                "cg_x_m": 0.50,
                "offset_m": [0.01, -0.005],
                "shape": "point",
            },
        ],
        "motor": {"file": str(ROOT / "data" / "motors" / "AeroTech_G80T.eng"), "aft_position_m": 1.0},
    }


def build(d: dict, tmp_path: Path):
    p = tmp_path / "v.json"
    p.write_text(json.dumps(d), encoding="utf-8")
    cfg = config_from_dict({"config_version": 1, "fidelity": 3, "vehicle_file": p.name}, base_dir=tmp_path)
    return cfg, build_vehicle(cfg)


def test_total_mass_and_cg_equal_independent_summation(tmp_path):
    d = base_vehicle()
    cfg, veh = build(d, tmp_path)
    motor = veh.motor
    comps = [(c["mass_kg"], c.get("cg_x_m")) for c in d["components"]]
    # CG positions by the rules of the format: sections at their midpoint (default), fins at LE + sweep/centroid + mean chord/2
    x_nose, x_tube = 0.1, 0.2 + 0.4
    fin = d["components"][2]
    r_, t_, s_ = fin["root_chord_m"], fin["tip_chord_m"], fin["sweep_m"]
    x_fin = fin["x_root_le_m"] + s_ / 3.0 * (r_ + 2 * t_) / (r_ + t_) + 0.5 * (0.5 * (r_ + t_))
    masses = [0.05, 0.30, 0.08, 0.10, 0.08]
    xs = [x_nose, x_tube, x_fin, 0.40, 0.50]
    nozzle = 1.0
    xm = nozzle - 0.5 * motor.length
    masses += [motor.casing_mass]
    xs += [xm]
    m_dry = sum(masses)
    x_dry = sum(m * x for m, x in zip(masses, xs)) / m_dry
    assert veh.mass_model.static_mass == pytest.approx(m_dry, rel=1e-12)
    mp = veh.mass_props(0.0)
    assert mp.mass == pytest.approx(m_dry + motor.propellant_mass, rel=1e-12)
    assert mp.x_cg == pytest.approx(
        (m_dry * x_dry + motor.propellant_mass * xm) / (m_dry + motor.propellant_mass), rel=1e-12
    )
    del comps


def test_cg_moves_forward_during_burn_and_matches_independent_formula(tmp_path):
    cfg, veh = build(base_vehicle(), tmp_path)
    m_s = veh.mass_model.static_mass
    x_s = veh.mass_model._sx / m_s
    xm = veh.mass_model.px
    p0 = veh.motor.propellant_mass
    prev = None
    for frac in (1.0, 0.75, 0.5, 0.25, 0.0):
        mp = veh.mass_model.at(frac * p0)
        expected = (m_s * x_s + frac * p0 * xm) / (m_s + frac * p0)
        assert mp.x_cg == pytest.approx(expected, rel=1e-12)
        assert mp.mass == pytest.approx(m_s + frac * p0)
        if prev is not None:
            assert mp.x_cg < prev  # propellant sits aft of the structure CG: burning it moves the CG forward
        prev = mp.x_cg


def test_off_axis_components_give_cg_offset_and_products_of_inertia(tmp_path):
    cfg, veh = build(base_vehicle(), tmp_path)
    mp = veh.mass_props(0.0)
    m = mp.mass
    # only the battery is off axis: (y, z) = (0.01, -0.005) m, mass 0.08 kg
    assert (mp.y_cg, mp.z_cg) == pytest.approx((0.08 * 0.01 / m, -0.08 * 0.005 / m), rel=1e-12)
    assert not mp.is_axisymmetric
    t = np.array(mp.tensor())
    assert np.allclose(t, t.T)  # symmetric
    assert np.all(np.linalg.eigvalsh(t) > 0)  # positive definite
    # independent full-tensor summation over every body of the vehicle
    mm = veh.mass_model
    bodies = [
        (
            c.mass,
            np.array([-c.x_cg, c.y, c.z]),
            np.array([[c.ixx, -c.ixy, -c.ixz], [-c.ixy, c.iyy, -c.iyz], [-c.ixz, -c.iyz, c.izz_eff]]),
        )
        for c in mm.static
    ]
    from rocket_sim.vehicle.mass import solid_cylinder_inertia

    ixp, iyp = solid_cylinder_inertia(veh.motor.propellant_mass, mm.pr, mm.pl)
    bodies.append((veh.motor.propellant_mass, np.array([-mm.px, 0.0, 0.0]), np.diag([ixp, iyp, iyp])))
    mt = sum(b[0] for b in bodies)
    cg = sum(b[0] * b[1] for b in bodies) / mt
    ref = sum(
        b[2] + b[0] * ((np.dot(b[1] - cg, b[1] - cg)) * np.eye(3) - np.outer(b[1] - cg, b[1] - cg))
        for b in bodies
    )
    assert t == pytest.approx(ref, rel=1e-10, abs=1e-14)


def test_shape_inertia_matches_closed_forms():
    m, lx, ly, lz = 0.2, 0.10, 0.05, 0.03
    ixx, iyy, izz = shape_inertia(m, "box", [lx, ly, lz], "c")
    assert (ixx, iyy, izz) == pytest.approx(
        (m / 12 * (ly**2 + lz**2), m / 12 * (lx**2 + lz**2), m / 12 * (lx**2 + ly**2))
    )
    L, d = 0.3, 0.06
    ixx, iyy, izz = shape_inertia(m, "cylinder", [L, d], "c")
    assert (ixx, iyy, izz) == pytest.approx(
        (0.5 * m * (d / 2) ** 2, m * (3 * (d / 2) ** 2 + L**2) / 12, m * (3 * (d / 2) ** 2 + L**2) / 12)
    )
    assert shape_inertia(m, "point", [], "c") == (0.0, 0.0, 0.0)


def test_shell_and_plate_masses_are_derived_from_material_and_geometry():
    d = base_vehicle()
    del d["components"][1]["mass_kg"]
    d["components"][1].update({"material": "glass", "wall_thickness_m": 0.002})
    del d["components"][2]["mass_kg"]
    d["components"][2]["material"] = "glass"
    del d["components"][0]["mass_kg"]
    d["components"][0].update({"material": "glass", "wall_thickness_m": 0.002})
    v = compile_vehicle(d)
    tube = next(c for c in v.components if c.id == "tube")
    ro, ri, L = 0.03, 0.028, 0.8
    assert tube.mass_kg == pytest.approx(math.pi * (ro**2 - ri**2) * L * RHO, rel=1e-12)
    assert tube.mass_source == "derived_shell"
    fins = next(c for c in v.components if c.id == "fins")
    assert fins.mass_kg == pytest.approx(4 * 0.5 * (0.1 + 0.05) * 0.06 * 0.002 * RHO, rel=1e-12)
    assert fins.mass_source == "derived_plate"
    nose = next(c for c in v.components if c.id == "nose")
    from rocket_sim.vehicle.assembly import Section

    area = Section("nose", 0.0, 0.2, 0.0, 0.06, "ogive").wetted_area()
    assert nose.mass_kg == pytest.approx(area * 0.002 * RHO, rel=1e-12)
    assert next(c for c in v.components if c.id == "avionics").mass_source == "specified"


def test_explicit_tensor_with_products_roundtrips_into_the_mass_model(tmp_path):
    d = base_vehicle()
    tensor = [
        [2e-4, -3e-5, 1e-5],
        [-3e-5, 9e-4, -2e-5],
        [1e-5, -2e-5, 9.5e-4],
    ]  # matrix (off-diagonals = -products)
    d["components"][3]["inertia_kgm2"] = tensor
    del d["components"][3]["shape"], d["components"][3]["dimensions_m"]
    cfg, veh = build(d, tmp_path)
    c = next(c for c in veh.mass_model.static if c.name == "avionics")
    assert (c.ixx, c.iyy, c.izz_eff) == pytest.approx((2e-4, 9e-4, 9.5e-4))
    assert (c.ixy, c.ixz, c.iyz) == pytest.approx(
        (3e-5, -1e-5, 2e-5)
    )  # products are the negated off-diagonals
    assert np.array(
        ((c.ixx, -c.ixy, -c.ixz), (-c.ixy, c.iyy, -c.iyz), (-c.ixz, -c.iyz, c.izz_eff))
    ) == pytest.approx(np.array(tensor))


def test_cad_inertia_on_a_geometry_component_rides_on_a_mass_item(tmp_path):
    d = base_vehicle()
    d["components"][1]["inertia_kgm2"] = [[1e-4, 0, 0], [0, 0.02, 0], [0, 0, 0.02]]
    d["components"][1]["cg_x_m"] = 0.62
    cfg, veh = build(d, tmp_path)
    c = next(c for c in veh.mass_model.static if c.name == "tube")
    assert c.mass == pytest.approx(0.30) and c.x_cg == pytest.approx(0.62) and c.iyy == pytest.approx(0.02)
    # the section itself carries no mass, so no thin-shell estimate was used for it
    assert not any("tube" in n for n in veh.notes)


def test_component_placement_and_dimensions_reach_the_assembly(tmp_path):
    cfg, veh = build(base_vehicle(), tmp_path)
    a = veh.assembly
    assert a is not None
    secs = a.sections
    assert [s.kind for s in secs] == ["nose", "body"]
    assert (secs[0].x, secs[0].length, secs[1].x, secs[1].length) == pytest.approx((0.0, 0.2, 0.2, 0.8))
    assert secs[1].d_fore == secs[1].d_aft == pytest.approx(0.06)
    assert a.length == pytest.approx(1.0)
    f = a.fins
    assert (
        f.count,
        f.root_chord,
        f.tip_chord,
        f.span,
        f.sweep,
        f.thickness,
        f.leading_edge_from_nose,
    ) == pytest.approx((4, 0.1, 0.05, 0.06, 0.05, 0.002, 0.85))
    assert veh.nozzle_x == pytest.approx(1.0) and veh.body.diameter == pytest.approx(0.06)


def test_transition_and_boat_tail_from_components(tmp_path):
    d = base_vehicle()
    d["components"].insert(
        2,
        {
            "id": "bt",
            "type": "transition",
            "length_m": 0.05,
            "fore_diameter_m": 0.06,
            "aft_diameter_m": 0.05,
            "mass_kg": 0.01,
        },
    )
    d["components"][3]["x_root_le_m"] = 0.8  # fins start on the straight tube
    d["motor"]["aft_position_m"] = 1.05
    cfg, veh = build(d, tmp_path)
    s = veh.assembly.sections[-1]
    assert s.kind == "transition" and s.d_fore == pytest.approx(0.06) and s.d_aft == pytest.approx(0.05)
    assert s.half_angle < 0  # narrowing: a boat-tail
    assert veh.assembly.length == pytest.approx(1.05)


def test_example_vehicle_derives_everything_and_is_internally_consistent(tmp_path):
    v = load_vehicle(EXAMPLE)
    assert v.data_quality == "placeholder"
    cfg = config_from_dict({"config_version": 1, "fidelity": 3, "vehicle_file": str(EXAMPLE)}, base_dir=ROOT)
    veh = build_vehicle(cfg)
    rep = mass_properties_report(veh)
    comp_sum = sum(c["mass_kg"] for c in rep["components"])
    assert comp_sum == pytest.approx(rep["liftoff_mass_kg"], rel=1e-12)
    assert rep["dry_mass_kg"] + rep["propellant_mass_kg"] == pytest.approx(rep["liftoff_mass_kg"])
    assert (
        rep["static_margin_burnout_cal"] > rep["static_margin_ignition_cal"] > 1.0
    )  # stable, margin grows as CG moves forward
    assert rep["at_burnout"]["cg_x_m"] < rep["at_ignition"]["cg_x_m"]
    t = np.array(rep["at_ignition"]["inertia_tensor_kgm2"])
    assert np.allclose(t, t.T) and np.all(np.linalg.eigvalsh(t) > 0)
    assert rep["aero_provenance"]["kind"] == "estimate"


@pytest.mark.parametrize(
    "mutate,match",
    [
        (lambda d: d.update(format_version=2), "format_version"),
        (lambda d: d.update(format="other"), "format"),
        (lambda d: d.update(bogus=1), "unknown top-level"),
        (lambda d: d["components"][1].update(x_start_m=0.3), "gap/overlap"),
        (lambda d: d["components"][1].update(diameter_m=0.05), "does not match"),
        (lambda d: d["components"][1].pop("mass_kg"), "needs 'mass_kg' or a 'material'"),
        (lambda d: d["components"][1].update(colour="red"), "unknown field"),
        (lambda d: d["components"][3].update(cg_x_m=5.0), "outside the airframe"),
        (lambda d: d["components"][3].pop("cg_x_m"), "cg_x_m"),
        (lambda d: d["components"][3].update(id="nose"), "unique"),
        (lambda d: d["components"][3].update(mass_kg=-1.0), "mass_kg"),
        (lambda d: d["components"][3].update(type="warp_drive"), "type must be"),
        (lambda d: d["motor"].update(aft_position_m=3.0), "within the airframe"),
        (lambda d: d["components"].pop(0), "exactly one 'nose'"),
        (lambda d: d["components"][3].pop("shape"), None),  # allowed: point mass with a warning
    ],
)
def test_vehicle_file_validation(mutate, match):
    d = copy.deepcopy(base_vehicle())
    mutate(d)
    if match is None:
        v = compile_vehicle(d)
        assert any("point mass" in w for w in v.warnings)
        return
    with pytest.raises(ConfigError, match=match):
        compile_vehicle(d)


@pytest.mark.parametrize(
    "tensor,match",
    [
        ([[1e-4, 1e-5, 0], [2e-5, 1e-4, 0], [0, 0, 1e-4]], "symmetric"),
        ([[1e-4, 0, 0], [0, 1e-4, 0], [0, 0, 1e-3]], "triangle"),
        ([[-1e-4, 0, 0], [0, 1e-4, 0], [0, 0, 1e-4]], "diagonal"),
        ([[1, 2], [3, 4]], "3x3"),
    ],
)
def test_unphysical_inertia_tensors_are_rejected(tensor, match):
    d = base_vehicle()
    d["components"][3]["inertia_kgm2"] = tensor
    del d["components"][3]["shape"], d["components"][3]["dimensions_m"]
    with pytest.raises(ConfigError, match=match):
        compile_vehicle(d)


def test_bad_json_and_missing_file_messages(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text("{not json", encoding="utf-8")
    with pytest.raises(ConfigError, match="not valid JSON"):
        load_vehicle(p)
    with pytest.raises(ConfigError, match="not found"):
        config_from_dict({"config_version": 1, "vehicle_file": "nope.json"}, base_dir=tmp_path)


def test_compilation_is_deterministic_and_line_ending_insensitive(tmp_path):
    p = tmp_path / "v.json"
    text = json.dumps(base_vehicle(), indent=2)
    p.write_text(text, encoding="utf-8")
    a, b = load_vehicle(p), load_vehicle(p)
    assert a.rocket == b.rocket and a.sha256 == b.sha256
    q = tmp_path / "w.json"
    q.write_bytes(text.replace("\n", "\r\n").encode())
    assert load_vehicle(q).sha256 == a.sha256


def test_vehicle_source_is_recorded_and_config_hash_is_machine_independent(tmp_path):
    d = base_vehicle()
    d["motor"]["file"] = "m/AeroTech_G80T.eng"
    for sub in ("a", "b"):
        (tmp_path / sub / "m").mkdir(parents=True)
        shutil.copy(ROOT / "data" / "motors" / "AeroTech_G80T.eng", tmp_path / sub / "m")
        (tmp_path / sub / "v.json").write_text(json.dumps(d), encoding="utf-8")
    hashes = []
    for sub in ("a", "b"):
        cfg = config_from_dict(
            {"config_version": 1, "fidelity": 3, "vehicle_file": "v.json"}, base_dir=tmp_path / sub
        )
        src = cfg.rocket.vehicle_source
        assert (
            src is not None
            and len(src["sha256"]) == 64
            and src["data_quality"] == "design"
            and src["format_version"] == 1
        )
        hashes.append(config_hash(cfg))
    assert hashes[0] == hashes[1]  # no absolute paths leak into the configuration hash


def test_vehicle_file_and_rocket_section_conflict(tmp_path):
    (tmp_path / "v.json").write_text(json.dumps(base_vehicle()), encoding="utf-8")
    with pytest.raises(ConfigError, match="mutually exclusive"):
        config_from_dict(
            {"config_version": 1, "vehicle_file": "v.json", "rocket": {"dry_mass_kg": 1.0}}, base_dir=tmp_path
        )


def test_vehicle_file_vehicle_flies_identically_to_the_equivalent_hand_written_config(tmp_path):
    d = base_vehicle()
    d["parachutes"] = []
    p = tmp_path / "v.json"
    p.write_text(json.dumps(d), encoding="utf-8")
    c1 = config_from_dict(
        {"config_version": 1, "fidelity": 3, "vehicle_file": p.name, "simulation": {"t_max_s": 25}},
        base_dir=tmp_path,
    )
    comp = compile_vehicle(d)
    raw = {
        "config_version": 1,
        "fidelity": 3,
        "simulation": {"t_max_s": 25},
        "rocket": comp.rocket,
        "motor": comp.motor,
    }
    c2 = config_from_dict(raw, base_dir=tmp_path)
    r1, r2 = run_simulation(c1, seed=1), run_simulation(c2, seed=1)
    assert r1.summary["apogee_m"] == r2.summary["apogee_m"]
    assert np.array_equal(r1.col("pos_z"), r2.col("pos_z"))
    assert (
        r1.meta.config["rocket"]["vehicle_source"] is not None
        and r2.meta.config["rocket"]["vehicle_source"] is None
    )


def test_vehicle_cli_prints_the_derived_report(capsys):
    from rocket_sim.cli import main

    assert main(["vehicle", str(EXAMPLE)]) == 0
    out = capsys.readouterr().out
    assert (
        "At ignition" in out and "inertia tensor" in out and "static margin" in out and "placeholder" in out
    )
    assert main(["vehicle", str(EXAMPLE), "--json"]) == 0
    rep = json.loads(capsys.readouterr().out)
    assert rep["liftoff_mass_kg"] > rep["dry_mass_kg"]


def test_published_json_schema_matches_the_compiler_tables():
    """vehicles/vehicle.schema.json is generated from the compiler's tables; they must not drift."""
    from rocket_sim.vehicle import vehicle_file as vf

    schema = json.loads((ROOT / "vehicles" / "vehicle.schema.json").read_text(encoding="utf-8"))
    assert schema["properties"]["format_version"]["const"] == vf.FORMAT_VERSION
    for t in vf.ALL_TYPES:
        defs = schema["$defs"][t]
        assert set(defs["properties"]) == vf._ALLOWED[t] | {"type"} and defs["additionalProperties"] is False
    assert set(schema["properties"]) == vf._TOP
    assert set(schema["properties"]["metadata"]["properties"]) == vf._META
    assert set(schema["properties"]["motor"]["properties"]) == vf._MOTOR


def test_example_validates_against_the_json_schema_when_jsonschema_is_available():
    jsonschema = pytest.importorskip("jsonschema")
    schema = json.loads((ROOT / "vehicles" / "vehicle.schema.json").read_text(encoding="utf-8"))
    jsonschema.validate(json.loads(EXAMPLE.read_text(encoding="utf-8")), schema)
    bad = json.loads(EXAMPLE.read_text(encoding="utf-8"))
    bad["components"][0]["colour"] = "red"
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(bad, schema)


def test_non_positive_semidefinite_tensor_is_rejected():
    d = base_vehicle()
    d["components"][3]["inertia_kgm2"] = [
        [1.0, 2.0, 0.0],
        [2.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]  # eigenvalue -1, passes the triangle test
    del d["components"][3]["shape"], d["components"][3]["dimensions_m"]
    with pytest.raises(ConfigError, match="positive semi-definite"):
        compile_vehicle(d)


@pytest.mark.parametrize(
    "mutate,match",
    [
        (lambda d: d["materials"].update(glass=-5.0), "density"),
        (lambda d: d["materials"].update(glass=float("nan")), "density"),
        (
            lambda d: (
                d["components"][0].update(material="glass", wall_thickness_m=0.5)
                or d["components"][0].pop("mass_kg")
            ),
            "smaller than the smallest radius",
        ),
        (lambda d: d["components"][2].update(count=2.5), "integer"),
        (lambda d: d["components"][2].update(count=True), "integer"),
        (lambda d: d["components"][3].update(dimensions_m=[0.1, -0.05, 0.03]), "positive dimensions"),
        (lambda d: d["components"][3].update(dimensions_m=[0.1, 0.05]), "3 positive dimensions"),
        (lambda d: d["components"][3].update(dimensions_m=["a", "b", "c"]), "list of numbers"),
        (lambda d: d["components"][3].update(offset_m=[1.0, 0.0]), "outside the airframe"),
        (lambda d: d["components"][3].update(offset_m="ab"), "offset_m must be"),
        (
            lambda d: d["components"][3].update(inertia_kgm2=[[1e-4, 0, 0], [0, 1e-4, 0], [0, 0, 1e-4]]),
            "EITHER",
        ),
        (lambda d: d["motor"].update(mount="motor_mount"), "unknown motor key"),
    ],
)
def test_more_validation_cases(mutate, match):
    d = base_vehicle()
    mutate(d)
    with pytest.raises(ConfigError, match=match):
        compile_vehicle(d)


def test_compile_does_not_mutate_its_input_and_fin_cg_matches_the_builder(tmp_path):
    d = base_vehicle()
    before = copy.deepcopy(d)
    compile_vehicle(d)
    assert d == before
    cfg, veh = build(d, tmp_path)
    fins_row = next(c for c in compile_vehicle(d).components if c.id == "fins")
    built = next(c for c in veh.mass_model.static if c.name == "fins")
    assert fins_row.x_cg_m == pytest.approx(
        built.x_cg, rel=1e-12
    )  # the report and the physics place the fins identically


def test_demo_vehicle_report_table_equals_the_physics_cg():
    cfg = config_from_dict({"config_version": 1, "fidelity": 3, "vehicle_file": str(EXAMPLE)}, base_dir=ROOT)
    veh = build_vehicle(cfg)
    rows = {c.id: c for c in load_vehicle(EXAMPLE).components}
    for c in veh.mass_model.static:
        if c.name in rows:
            assert rows[c.name].mass_kg == pytest.approx(c.mass, rel=1e-12), c.name
            assert rows[c.name].x_cg_m == pytest.approx(c.x_cg, rel=1e-12), c.name


LITERAL_CG = (
    0.5333333333,
    0.0266666667,
    0.01,
)  # aft of nose, body y, body z: from a separate hand calculation
LITERAL_TENSOR = [
    [0.0188666667, -0.0226666667, 0.0080000000],
    [-0.0226666667, 0.1322666667, 0.0014000000],
    [0.0080000000, 0.0014000000, 0.1449333333],
]


def test_vehicle_tensor_equals_literal_hand_calculated_numbers():
    """Two bodies with products of inertia: the expected matrix was computed once, offline, and is written down as constants,
    so a sign error in the production convention cannot cancel against a shared helper."""
    from rocket_sim.vehicle.mass import MassComponent, MassModel

    a = MassComponent(
        "A", 2.0, 0.4, 0.01, 0.02, y=0.05, z=0.0, izz=0.03, ixy=0.004
    )  # product +0.004 -> matrix entry -0.004
    b = MassComponent("B", 1.0, 0.8, 0.005, 0.005, y=-0.02, z=0.03, izz=0.005)
    mp = MassModel([a, b], 0.5, 0.01, 0.1).at(0.0)
    assert (mp.x_cg, mp.y_cg, mp.z_cg) == pytest.approx(LITERAL_CG, abs=1e-9)
    assert np.array(mp.tensor()) == pytest.approx(np.array(LITERAL_TENSOR), abs=1e-9)


def test_vehicle_json_tensor_sign_convention_reaches_the_physics_with_the_literal_numbers(tmp_path):
    d = base_vehicle()
    d["components"] = [d["components"][0], d["components"][1]]
    d["components"][0]["base_diameter_m"] = d["components"][1]["diameter_m"] = (
        0.2  # wide enough for the 5 cm offsets
    )
    d["components"][1]["mass_kg"] = 1e-9  # negligible structure so only the two items matter
    d["components"][0]["mass_kg"] = 1e-9
    d["components"] += [
        {
            "id": "A",
            "type": "payload",
            "mass_kg": 2.0,
            "cg_x_m": 0.4,
            "offset_m": [0.05, 0.0],
            "inertia_kgm2": [[0.01, -0.004, 0.0], [-0.004, 0.02, 0.0], [0.0, 0.0, 0.03]],
        },
        {
            "id": "B",
            "type": "payload",
            "mass_kg": 1.0,
            "cg_x_m": 0.8,
            "offset_m": [-0.02, 0.03],
            "inertia_kgm2": [[0.005, 0, 0], [0, 0.005, 0], [0, 0, 0.005]],
        },
    ]
    cfg, veh = build(d, tmp_path)
    items = {c.name: c for c in veh.mass_model.static}
    assert (items["A"].ixy, items["A"].ixz, items["A"].iyz) == pytest.approx((0.004, 0.0, 0.0))
    # compare the two payload bodies' contribution: remove the (negligible) structure and motor by differencing against a model of A+B only
    from rocket_sim.vehicle.mass import MassModel

    mm = MassModel([items["A"], items["B"]], 0.5, 0.01, 0.1)
    assert np.array(mm.at(0.0).tensor()) == pytest.approx(np.array(LITERAL_TENSOR), abs=1e-9)


@pytest.mark.parametrize(
    "key,val",
    [
        ("cg_x_m", 1.4),
        ("offset_m", [0.0, 0.0]),
        ("inertia_kgm2", [[1e-3, 0, 0], [0, 1e-3, 0], [0, 0, 1e-3]]),
        ("mass_source", "x"),
    ],
)
def test_keys_that_would_be_ignored_are_errors_for_fin_sets(key, val):
    d = base_vehicle()
    d["components"][2][key] = val
    with pytest.raises(ConfigError, match="unknown field"):
        compile_vehicle(d)


def test_absurd_fin_span_is_flagged_as_a_probable_unit_error():
    d = base_vehicle()
    d["components"][2]["span_m"] = 3.0
    v = compile_vehicle(d)
    assert any("exceeds two body diameters" in w for w in v.warnings)
