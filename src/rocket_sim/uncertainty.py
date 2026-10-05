"""Honest reporting: numbers are shown only to the precision the model can support.

``MODEL_UNCERTAINTY`` holds the *model-form* relative 1-sigma uncertainty of headline outputs
when the inputs (mass, motor, drag, wind...) are known exactly. Input uncertainty is separate
(that is what Monte Carlo is for). The values come from the validation campaign documented in
docs/validation.md; entries marked ``provisional`` (shown with ``*``) are NOT confirmed by flight data.

A result is printed with ``format_value``: the uncertainty is rounded to one significant digit
(two if it starts with 1) and the value is rounded to the same decimal place, so a vehicle
apogee is shown as ``800 +/- 80 m`` and never as ``801.4734 m``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class RelUncertainty:
    rel: float  # relative 1-sigma
    basis: str  # where the number comes from
    provisional: bool = True


# Compared with 3 real flights (docs/validation.md; physics v1.1.0, BAROMETER-equivalent altitude like the
# real altimeters): apogee errors +0.5 / +13.4 / -4.9 % (RMS 8.3 %), max velocity -5.7 / +13.1 / -0.3 %
# (RMS 8.3 %), apogee time -1.0 / +5.3 / -5.6 % (RMS 4.5 %). These INCLUDE the unknown input errors of the
# reconstructions (thrust tolerance, atmosphere temperature offsets that were estimated, not measured, mass), so
# they describe "predict a flight from catalogue inputs". n = 3 flights, motors 54-98 mm, 20 kg class, subsonic:
# order-of-magnitude only, NOT a confidence interval, and NOT validated for small model rockets (the 29 mm
# example vehicle) or supersonic flight, where these figures are extrapolations.
_V = "3-flight validation (physics v1.1.0)"
MODEL_UNCERTAINTY: dict[str, RelUncertainty] = {
    "apogee_m": RelUncertainty(0.08, _V, provisional=False),
    "apogee_event_altitude_m": RelUncertainty(0.08, _V, provisional=False),
    "max_velocity_ms": RelUncertainty(0.08, _V, provisional=False),
    "burnout_velocity_ms": RelUncertainty(0.08, "as max_velocity_ms", provisional=False),
    "burnout_altitude_m": RelUncertainty(0.08, "extrapolated from apogee/velocity validation"),
    "max_acceleration_ms2": RelUncertainty(
        0.10, "NDRT accelerometer comparison (+9.7 %, sensor scale unknown)"
    ),
    "max_load_factor_g": RelUncertainty(0.10, "NDRT accelerometer comparison (+9.7 %, sensor scale unknown)"),
    "apogee_time_s": RelUncertainty(0.05, _V, provisional=False),
    "burnout_time_s": RelUncertainty(0.02, "NDRT accelerometer zero crossing (-1.9 %), one flight only"),
    "impact_speed_ms": RelUncertainty(0.20, "parachute Cd/area uncertainty (no descent-rate truth yet)"),
    "impact_vertical_speed_ms": RelUncertainty(
        0.20, "parachute Cd/area uncertainty (no descent-rate truth yet)"
    ),
    "flight_time_s": RelUncertainty(0.10, "landing-time errors +1.3 / +16.7 / -4.2 % (descent assumptions)"),
    "landing_distance_m": RelUncertainty(0.50, "wind unknown; dominated by input uncertainty"),
    "max_mach": RelUncertainty(0.08, "as max_velocity_ms", provisional=False),
    "rail_exit_velocity_ms": RelUncertainty(0.03, "RocketPy cross-check agrees to 0.3 %; mass/thrust inputs"),
}


def apply_validated(table: dict[str, float], note: str) -> None:
    """Replace provisional uncertainties with validated ones (used by the validation tools)."""
    for k, rel in table.items():
        MODEL_UNCERTAINTY[k] = RelUncertainty(rel, note, provisional=False)


def _round_sig(x: float, n: int) -> float:
    if x == 0 or not math.isfinite(x):
        return x
    return round(x, n - 1 - int(math.floor(math.log10(abs(x)))))


def format_value(value: float | None, metric: str, unit: str = "", fidelity: int = 3) -> str:
    """Value rounded to its uncertainty, e.g. '800 +/- 80 m'. Falls back to 3 s.f. if unknown."""
    if value is None:
        return "n/a"
    unc = MODEL_UNCERTAINTY.get(metric)
    if unc is None or fidelity < 2:
        return f"{value:.3g} {unit}".strip()
    sigma = abs(value) * unc.rel
    if sigma == 0:
        return f"{value:.3g} {unit}".strip()
    first = f"{sigma:.1e}"[0]
    sig = 2 if first == "1" else 1
    s_r = _round_sig(sigma, sig)
    decimals = max(0, sig - 1 - int(math.floor(math.log10(s_r))))
    v_r = round(value, decimals)
    star = "*" if unc.provisional else ""
    return f"{v_r:.{decimals}f} +/- {s_r:.{decimals}f}{star} {unit}".strip()


def provisional_note() -> str:
    return "* provisional model uncertainty (see docs/validation.md)"
