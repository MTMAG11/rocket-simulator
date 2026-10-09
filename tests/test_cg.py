"""CG recomputation used to check a vehicle file against a CAD mass report."""

import pytest

from rocket_sim.config import config_from_dict, config_to_dict, load_config
from rocket_sim.simulation import Simulation
from rocket_sim.vehicle import cg
from rocket_sim.vehicle.cg import cg_from_report
from rocket_sim.vehicle.report import mass_properties_report
from tests.conftest import ROOT


def oracle_cg(masses, positions):
    """Test-local mass-weighted mean (NOT the owner's TODO)."""
    return sum(m * x for m, x in zip(masses, positions, strict=True)) / sum(masses)


@pytest.mark.user_todo
@pytest.mark.xfail(strict=True, raises=NotImplementedError, reason="TODO(you): implement center_of_gravity")
def test_todo_center_of_gravity_hand_value():
    """masses (0.2, 0.3, 0.5) kg at (0.1, 0.5, 0.9) m -> 0.62 m (docs/derivations/center_of_gravity.md)."""
    f = cg.center_of_gravity
    assert f([0.2, 0.3, 0.5], [0.1, 0.5, 0.9]) == pytest.approx(0.62)
    assert f([2.0], [0.7]) == pytest.approx(0.7)  # one mass: its own position
    assert f([1.0, 1.0], [0.0, 1.0]) == pytest.approx(0.5)  # equal masses: the midpoint
    assert f([1.0, 3.0], [0.0, 1.0]) == pytest.approx(0.75)  # nearer the heavier one


def test_oracle_hand_value():
    assert oracle_cg([0.2, 0.3, 0.5], [0.1, 0.5, 0.9]) == pytest.approx(0.62)


def test_the_vehicle_report_cg_is_the_mass_weighted_mean_of_its_components(monkeypatch):
    monkeypatch.setattr(cg, "center_of_gravity", oracle_cg)
    cfg = load_config(ROOT / "configs" / "example_vehicle_sim.yaml")
    veh = Simulation(config_from_dict(config_to_dict(cfg), base_dir=ROOT / "configs"), seed=0).vehicle
    rep = mass_properties_report(veh)
    for with_motor in (True, False):
        out = cg_from_report(rep, include_motor=with_motor)
        assert abs(out["difference_m"]) < 1e-9, out


def test_a_wrong_position_in_the_file_shows_up_as_a_difference(monkeypatch):
    monkeypatch.setattr(cg, "center_of_gravity", oracle_cg)
    rep = {
        "components": [
            {"name": "a", "mass_kg": 1.0, "cg_x_m": 0.2},
            {"name": "b", "mass_kg": 1.0, "cg_x_m": 0.6},
            {"name": "propellant", "mass_kg": 0.5, "cg_x_m": 0.9, "burns_off": True},
        ],
        "at_ignition": {"cg_x_m": 0.52},  # the true CG would be (0.2 + 0.6 + 0.45) / 2.5 = 0.50
        "at_burnout": {"cg_x_m": 0.40},
    }
    assert cg_from_report(rep)["difference_m"] == pytest.approx(-0.02)
    assert cg_from_report(rep, include_motor=False)["difference_m"] == pytest.approx(0.0)
