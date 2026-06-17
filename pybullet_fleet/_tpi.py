"""Trapezoidal-velocity profile primitives shared by per-agent and batched controllers.

:func:`build_tpi` is the canonical factory wrapping the external
``TwoPointInterpolation`` package — used by both per-agent
``KinematicController`` and the vectorised ``BatchKinematicController``
subclasses to construct a single trajectory.

:func:`extract_phase_params` lifts the symmetric trapezoidal phase scalars
(``t_accel``, ``t_const``, ``t_total``, ``accel``) out of a constructed
``TwoPointInterpolation`` so the batched controllers can store them in
``(N,)`` arrays for vectorised evaluation.

:func:`trapezoid_distance` evaluates the distance traveled at time ``tau``
for ``(N,)`` arrays of such trajectories — the batched eval hot path.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np
from two_point_interpolation import TwoPointInterpolation


def build_tpi(
    p0: float,
    pe: float,
    vmax: float,
    accel: float,
    t0: float,
    v0: float = 0.0,
    ve: float = 0.0,
) -> TwoPointInterpolation:
    """Construct a ready-to-evaluate :class:`TwoPointInterpolation`.

    Wraps the ``init() + calc_trajectory()`` pair with a zero-distance
    fallback (returns a degenerate ``p0 → p0`` trajectory if the requested
    motion is infeasible). This is the single canonical entry point for
    building a TPI in both per-agent and batched controllers.
    """
    tpi = TwoPointInterpolation()
    try:
        tpi.init(p0=p0, pe=pe, acc_max=accel, vmax=vmax, t0=t0, v0=v0, ve=ve, dec_max=accel)
        tpi.calc_trajectory()
    except ValueError:
        tpi = TwoPointInterpolation()
        tpi.init(p0=p0, pe=p0, acc_max=accel, vmax=vmax, t0=t0, v0=0.0, ve=0.0, dec_max=accel)
        tpi.calc_trajectory()
    return tpi


def extract_phase_params(tpi: TwoPointInterpolation) -> Tuple[float, float, float, float]:
    """Read symmetric trapezoidal phase scalars from a constructed TPI.

    Returns ``(t_accel, t_const, t_total, accel)`` relative to ``tpi.t0``
    in the form expected by :func:`trapezoid_distance` for batched eval.

    Handles all three TPI cases:

    - ``case == -1`` (degenerate / zero distance): all zeros.
    - ``case == 0`` (triangle, ``vmax`` not reached): ``t_const = 0``.
    - ``case == 1`` (full trapezoid): three populated phases.

    Assumes the TPI was built with symmetric accel/dec (the only case
    used by current per-agent and batched controllers).
    """
    dt = tpi.dt
    accel = float(tpi.amax_accel)
    if len(dt) == 0:
        return 0.0, 0.0, 0.0, accel
    if len(dt) == 2:
        t_accel = float(dt[0])
        t_decel = float(dt[1])
        return t_accel, 0.0, t_accel + t_decel, accel
    # len == 3 (full trapezoid)
    t_accel = float(dt[0])
    t_const = float(dt[1])
    t_total = float(dt[0] + dt[1] + dt[2])
    return t_accel, t_const, t_total, accel


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
