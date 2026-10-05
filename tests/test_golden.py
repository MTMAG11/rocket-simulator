"""Golden-output regression keyed to PHYSICS_VERSION.

If this test fails, the physics changed. Either it is a bug, or you must bump PHYSICS_VERSION (and the change log in
docs/physics.md), re-run the validation, and update these numbers: datasets from different physics versions are not
comparable (spec section 48).
"""

import pytest

from rocket_sim.simulation import run_simulation
from rocket_sim.version import PHYSICS_VERSION
from tests.conftest import cfg_from

GOLDEN = {
    "1.2.1": {
        3: {
            "apogee_m": 918.4653338991054,
            "max_velocity_ms": 197.84451611991202,
            "impact_speed_ms": 5.314115246465192,
        },
        2: {
            "apogee_m": 905.7451037354892,
            "max_velocity_ms": 197.79518750606735,
            "impact_speed_ms": 5.314115246465169,
        },
    },
    "1.2.0": {
        3: {
            "apogee_m": 918.4653338991054,
            "max_velocity_ms": 197.84451611991202,
            "impact_speed_ms": 5.314115246465192,
        },
        2: {
            "apogee_m": 905.7451037354892,
            "max_velocity_ms": 197.79518750606735,
            "impact_speed_ms": 5.314115246465169,
        },
    },
    "1.1.0": {
        3: {
            "apogee_m": 918.5620493414768,
            "max_velocity_ms": 197.84631612485572,
            "impact_speed_ms": 5.314115246465169,
        },
        2: {
            "apogee_m": 905.7451037354892,
            "max_velocity_ms": 197.79518750606735,
            "impact_speed_ms": 5.314115246465169,
        },
    },
}


@pytest.mark.parametrize("fidelity", [2, 3])
def test_golden_summary(fidelity):
    assert PHYSICS_VERSION in GOLDEN, f"no golden numbers recorded for physics {PHYSICS_VERSION}"
    s = run_simulation(cfg_from(fidelity=fidelity), seed=0).summary
    for k, v in GOLDEN[PHYSICS_VERSION][fidelity].items():
        assert s[k] == pytest.approx(v, rel=1e-9), f"{k} changed: physics changed without a version bump?"
