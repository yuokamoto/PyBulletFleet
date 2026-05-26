"""Trapezoidal-velocity profile primitives shared by batch controllers.

Scalar :func:`trapezoid_params` solves a symmetric profile (``v0 = ve = 0``)
for one trajectory. Vectorised :func:`trapezoid_distance` evaluates the
distance traveled at time ``tau`` for ``(N,)`` arrays of trajectories.

These functions used to live inside ``batch_omni.py``; they are extracted
here so ``batch_differential.py`` (and any future batch controller) can
import them without a sibling-module dependency.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def trapezoid_params(distance: float, vmax: float, accel: float) -> Tuple[float, float, float]:
    """Solve a symmetric trapezoidal profile.

    Parameters
    ----------
    distance, vmax, accel
        Non-negative scalars. Distance must be ``>= 0``.

    Returns
    -------
    (t_accel, t_const, t_total)
        Phase durations. All zero when ``distance`` is negligible or when
        ``vmax``/``accel`` is non-finite or non-positive.
    """
    if distance <= 1e-12 or vmax <= 0.0 or accel <= 0.0 or not math.isfinite(vmax) or not math.isfinite(accel):
        return 0.0, 0.0, 0.0
    d_to_vmax = vmax * vmax / (2.0 * accel)
    if 2.0 * d_to_vmax <= distance:
        t_accel = vmax / accel
        t_const = (distance - 2.0 * d_to_vmax) / vmax
        t_total = 2.0 * t_accel + t_const
    else:
        v_peak = math.sqrt(accel * distance)
        t_accel = v_peak / accel
        t_const = 0.0
        t_total = 2.0 * t_accel
    return t_accel, t_const, t_total


def trapezoid_distance(
    tau: np.ndarray,
    t_accel: np.ndarray,
    t_const: np.ndarray,
    t_total: np.ndarray,
    accel: np.ndarray,
    total_distance: np.ndarray,
) -> np.ndarray:
    """Vectorised distance-traveled lookup for trapezoidal profiles.

    All inputs are ``(N,)`` arrays; returns ``(N,)``.
    """
    out = np.zeros_like(tau)
    # phase masks
    in_const = (tau > t_accel) & (tau <= t_accel + t_const)
    in_decel = (tau > t_accel + t_const) & (tau < t_total)
    done = tau >= t_total
    in_accel = (tau > 0.0) & ~in_const & ~in_decel & ~done

    # accel phase: 0.5 * a * t^2
    if in_accel.any():
        out[in_accel] = 0.5 * accel[in_accel] * tau[in_accel] * tau[in_accel]
    # const phase: 0.5 * a * t_accel^2 + v_peak * (t - t_accel),
    # where v_peak = a * t_accel
    if in_const.any():
        ta = t_accel[in_const]
        a = accel[in_const]
        out[in_const] = 0.5 * a * ta * ta + (a * ta) * (tau[in_const] - ta)
    # decel phase: total_distance - 0.5 * a * (t_total - t)^2
    if in_decel.any():
        a = accel[in_decel]
        t_left = t_total[in_decel] - tau[in_decel]
        out[in_decel] = total_distance[in_decel] - 0.5 * a * t_left * t_left
    if done.any():
        out[done] = total_distance[done]
    return out
