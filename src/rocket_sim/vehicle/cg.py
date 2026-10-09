"""Independent centre-of-gravity check of a vehicle description (used to test a vehicle file against a CAD mass report).

The mass-weighted mean itself, ``center_of_gravity``, is left as a TODO for the project owner
(docs/derivations/center_of_gravity.md).
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any


def center_of_gravity(masses_kg: Sequence[float], positions_m: Sequence[float]) -> float:
    """TODO(you): the position of the centre of gravity [m] of point masses ``masses_kg`` at ``positions_m`` (one axis).

    Derive it in docs/derivations/center_of_gravity.md; check it against the hand-calculated value in tests/test_cg.py
    (``pytest --runxfail -m user_todo`` shows that test failing until this is implemented).
    """
    raise NotImplementedError("center_of_gravity: see the TODO in rocket_sim/vehicle/cg.py")


def cg_from_report(report: dict[str, Any], include_motor: bool = True) -> dict[str, float]:
    """CG recomputed from the component table of ``mass_properties_report`` and compared with the CG the report states.

    Returns the recomputed CG, the reported CG and their difference [m]; ``include_motor`` adds the propellant row (the vehicle
    at ignition), otherwise the dry components only."""
    comps = [c for c in report["components"] if include_motor or not c.get("burns_off")]
    cg = center_of_gravity([c["mass_kg"] for c in comps], [c["cg_x_m"] for c in comps])
    stated = report["at_ignition"]["cg_x_m"] if include_motor else report["at_burnout"]["cg_x_m"]
    return {"recomputed_m": float(cg), "reported_m": float(stated), "difference_m": float(cg) - float(stated)}
