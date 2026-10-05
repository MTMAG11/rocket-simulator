"""Fixed-step explicit integrators and event-time refinement.

All steppers have signature ``step(f, t, y, h) -> y_new`` for ``dy/dt = f(t, y)`` with NumPy
state arrays. Orders of accuracy: euler 1, midpoint 2, rk4 4 (verified by the convergence
tests in tests/test_numerics.py against analytic solutions).

Non-smooth inputs (thrust-curve corners, parachute opening, controller/sensor ticks) reduce
the *effective* order of any one-step method if a step straddles them; the simulator
therefore truncates steps so that these instants coincide with step boundaries.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

Deriv = Callable[[float, np.ndarray], np.ndarray]
Stepper = Callable[[Deriv, float, np.ndarray, float], np.ndarray]


def euler_step(f: Deriv, t: float, y: np.ndarray, h: float) -> np.ndarray:
    return y + h * f(t, y)


def midpoint_step(f: Deriv, t: float, y: np.ndarray, h: float) -> np.ndarray:
    k1 = f(t, y)
    return y + h * f(t + 0.5 * h, y + 0.5 * h * k1)


def rk4_step(f: Deriv, t: float, y: np.ndarray, h: float) -> np.ndarray:
    k1 = f(t, y)
    k2 = f(t + 0.5 * h, y + 0.5 * h * k1)
    k3 = f(t + 0.5 * h, y + 0.5 * h * k2)
    k4 = f(t + h, y + h * k3)
    return y + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


STEPPERS: dict[str, Stepper] = {"euler": euler_step, "midpoint": midpoint_step, "rk4": rk4_step}


def refine_event(
    step: Stepper,
    f: Deriv,
    g: Callable[[float, np.ndarray], float],
    t0: float,
    y0: np.ndarray,
    h: float,
    iterations: int = 48,
) -> tuple[float, np.ndarray]:
    """Locate the root of g within the step [t0, t0+h] by bisection with re-integration.

    Requires g(t0, y0) and g(t0+h, y(t0+h)) to have opposite signs (or zero at the end).
    Returns (t_event, y_event). Accuracy ~ h * 2^-iterations (below float resolution for the
    default), so event times carry no step-size-dependent interpolation error beyond the
    integrator's own truncation error.
    """
    g0 = g(t0, y0)
    lo, hi = 0.0, h
    for _ in range(iterations):
        mid = 0.5 * (lo + hi)
        ym = step(f, t0, y0, mid)
        gm = g(t0 + mid, ym)
        if (gm > 0.0) == (g0 > 0.0) and gm != 0.0:
            lo = mid
        else:
            hi = mid
    y_hi = step(f, t0, y0, hi)
    return t0 + hi, y_hi
