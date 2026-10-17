"""Trapezoidal-velocity profile primitives shared by per-agent and batched controllers.

:func:`build_tpi` is the canonical factory wrapping the external
``TwoPointInterpolation`` package — used by both per-agent
``KinematicController`` and the vectorised ``BatchKinematicController``
subclasses to construct a single trajectory. :func:`try_build_tpi` is the
same construction without the fallback, for callers that have to tell an
infeasible request apart from a trajectory that simply finishes early.

:func:`extract_phase_params` lifts the symmetric trapezoidal phase scalars
(``t_accel``, ``t_const``, ``t_total``, ``accel``) out of a constructed
``TwoPointInterpolation`` so the batched controllers can store them in
``(N,)`` arrays for vectorised evaluation.

:func:`trapezoid_distance` evaluates the distance traveled at time ``tau``
for ``(N,)`` arrays of such trajectories — the batched eval hot path.

:func:`build_tpi_from_speed` starts a trajectory from a body already moving
along the travel axis, for a goal replaced mid-motion, and
:func:`try_build_tpi_from_speed` says whether it can stop in time.
"""

from __future__ import annotations

from typing import Optional, Tuple

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
    decel: Optional[float] = None,
) -> TwoPointInterpolation:
    """Construct a ready-to-evaluate :class:`TwoPointInterpolation`.

    Wraps the ``init() + calc_trajectory()`` pair with a zero-distance
    fallback (returns a degenerate ``p0 → p0`` trajectory if the requested
    motion is infeasible). This is the single canonical entry point for
    building a TPI in both per-agent and batched controllers.

    Args:
        p0: Start position along the travel axis.
        pe: End position along the same axis.
        vmax: Speed cap for the cruise phase.
        accel: Acceleration magnitude for the ramp up.
        t0: Simulation time the trajectory starts at; evaluation is relative
            to it.
        v0: Speed at ``p0`` (default: at rest).
        ve: Speed to arrive at ``pe`` with (default: at rest).
        decel: Deceleration magnitude for the ramp down. ``None`` (the
            default) reuses ``accel``, giving the symmetric trapezoid that was
            the only profile this helper could build previously. Pass a
            different value for vehicles that brake harder, or softer, than
            they accelerate.

    Returns:
        A ``TwoPointInterpolation`` with its trajectory already calculated.
    """
    tpi = try_build_tpi(p0=p0, pe=pe, vmax=vmax, accel=accel, t0=t0, v0=v0, ve=ve, decel=decel)
    if tpi is not None:
        return tpi
    dec = accel if decel is None else decel
    tpi = TwoPointInterpolation()
    tpi.init(p0=p0, pe=p0, acc_max=accel, vmax=vmax, t0=t0, v0=0.0, ve=0.0, dec_max=dec)
    tpi.calc_trajectory()
    return tpi


def try_build_tpi(
    p0: float,
    pe: float,
    vmax: float,
    accel: float,
    t0: float,
    v0: float = 0.0,
    ve: float = 0.0,
    decel: Optional[float] = None,
) -> Optional[TwoPointInterpolation]:
    """Build a :class:`TwoPointInterpolation`, or ``None`` if there is none.

    Same arguments and same solver as :func:`build_tpi`, without its
    fallback. A request is infeasible when ``v0`` is too high to reach ``pe``
    at ``ve`` within ``decel`` — the gap between ``p0`` and ``pe`` is shorter
    than the braking distance the speed demands.

    :func:`build_tpi` answers that with a degenerate ``p0 → p0`` trajectory,
    which is the right default for a controller that re-plans from where it
    stands. A caller that reads "trajectory over" as "target reached" needs
    the two cases kept apart, and gets ``None`` here instead.

    Returns:
        The calculated trajectory, or ``None`` when the solver rejects the
        request as infeasible.
    """
    dec = accel if decel is None else decel
    tpi = TwoPointInterpolation()
    try:
        tpi.init(p0=p0, pe=pe, acc_max=accel, vmax=vmax, t0=t0, v0=v0, ve=ve, dec_max=dec)
        tpi.calc_trajectory()
    except ValueError:
        return None
    if _ends_at(tpi, pe, ve):
        return tpi
    # A body starting away from pe (v0 against the travel direction) gives
    # the solver two positive roots for the first phase, and it takes the
    # smaller one, which does not end at pe at rest. The larger one does:
    # keep accelerating toward pe through zero speed. Rebuild with it.
    if _rebuild_with_larger_root(tpi) and _ends_at(tpi, pe, ve):
        return tpi
    return None


def _ends_at(tpi: TwoPointInterpolation, pe: float, ve: float) -> bool:
    """Whether a calculated trajectory really ends at ``(pe, ve)``."""
    if not tpi.dt:
        return True
    if any(d < -1e-12 for d in tpi.dt):
        return False
    p_end = tpi.p[-1] + tpi.v[-1] * tpi.dt[-1] + 0.5 * tpi.a[-1] * tpi.dt[-1] ** 2
    v_end = tpi.v[-1] + tpi.a[-1] * tpi.dt[-1]
    scale = max(1.0, abs(pe - tpi.p0))
    return abs(p_end - pe) <= 1e-6 * scale and abs(v_end - ve) <= 1e-6 * max(1.0, abs(tpi.vmax))


def _rebuild_with_larger_root(tpi: TwoPointInterpolation) -> bool:
    """Redo the solver's phases with the larger first-phase root.

    Mirrors ``TwoPointInterpolation.calc_trajectory`` for constant
    acceleration (both of its cases); only the root choice differs.
    """
    p0, pe, v0, ve, vmax = float(tpi.p0), float(tpi.pe), float(tpi.v0), float(tpi.ve), float(tpi.vmax)
    dp = pe - p0
    if dp == 0.0:
        return False
    sign = 1.0 if dp > 0 else -1.0
    acc = float(tpi.amax_accel) * sign
    dec = float(tpi.amax_decel) * sign
    ratio = acc / dec
    a_coeff = 0.5 * acc * (1 + ratio)
    b_coeff = v0 * (1 + ratio)
    c_coeff = -dp + (v0**2 - ve**2) / (2 * dec)
    disc = b_coeff**2 - 4 * a_coeff * c_coeff
    if disc <= 0:
        return False
    root = max((-b_coeff + np.sqrt(disc)) / (2 * a_coeff), (-b_coeff - np.sqrt(disc)) / (2 * a_coeff))
    if root <= 0:
        return False
    v1 = v0 + acc * root
    if abs(v1) < vmax:
        p1 = p0 + v0 * root + 0.5 * acc * root**2
        tpi.case = 0
        tpi.dt = [root, abs((v1 - ve) / dec)]
        tpi.a = [acc, -dec]
        tpi.v = [v0, v1]
        tpi.p = [p0, p1]
    else:
        v1 = vmax * sign
        dt01 = abs((v1 - v0) / acc)
        p1 = p0 + v0 * dt01 + 0.5 * acc * dt01**2
        dt2e = abs((v1 - ve) / dec)
        dp2e = v1 * dt2e - 0.5 * dec * dt2e**2
        dt12 = (pe - p1 - dp2e) / v1
        if dt12 < 0:
            return False
        tpi.case = 1
        tpi.dt = [dt01, dt12, dt2e]
        tpi.a = [acc, 0.0, -dec]
        tpi.v = [v0, v1, v1]
        tpi.p = [p0, p1, pe - dp2e]
    tpi.trajectory_calced = True
    tpi.duration = float(sum(tpi.dt))
    tpi.te = tpi.t0 + tpi.duration
    return True


#: Fraction below ``vmax`` an initial speed is clamped to. The solver treats an
#: initial speed equal to ``vmax`` as exceeding it (and warns), so a body that
#: is cruising at the cap starts a hair below it instead.
_V0_CAP = 1.0 - 1e-9


def try_build_tpi_from_speed(
    p0: float,
    pe: float,
    vmax: float,
    accel: float,
    t0: float,
    v0: float,
    decel: Optional[float] = None,
) -> Optional[TwoPointInterpolation]:
    """A trajectory to ``pe`` for a body already moving at ``v0``, or ``None``.

    ``v0`` is signed along ``p0 -> pe``: negative is moving away from ``pe``,
    and the trajectory then slows, reverses and drives to ``pe`` (that first
    slowing uses ``accel``, the solver's first-phase limit). Its magnitude is
    clamped to ``vmax``. ``None`` means no trajectory within the limits
    exists -- ``pe`` is closer than the braking distance, or the body is
    moving with nowhere to go -- and no limit is ever raised to make one fit;
    see :mod:`pybullet_fleet._replan`.
    """
    v0 = max(-vmax * _V0_CAP, min(float(v0), vmax * _V0_CAP))
    return try_build_tpi(p0=p0, pe=pe, vmax=vmax, accel=accel, t0=t0, v0=v0, decel=decel)


def build_tpi_from_speed(
    p0: float,
    pe: float,
    vmax: float,
    accel: float,
    t0: float,
    v0: float,
    decel: Optional[float] = None,
) -> TwoPointInterpolation:
    """A trajectory to ``pe`` for a body already moving at ``v0`` toward it.

    For a goal replaced mid-motion: the body keeps its (signed) speed instead
    of dropping to rest and accelerating again. From rest this is
    :func:`build_tpi`.

    Raises:
        ValueError: no trajectory within the limits exists (see
            :func:`try_build_tpi_from_speed`). Callers check first with
            :func:`pybullet_fleet._replan.replacement_rejection`; no limit is
            ever raised to make a goal fit.
    """
    if v0 == 0.0:
        return build_tpi(p0=p0, pe=pe, vmax=vmax, accel=accel, t0=t0, decel=decel)
    tpi = try_build_tpi_from_speed(p0=p0, pe=pe, vmax=vmax, accel=accel, t0=t0, v0=v0, decel=decel)
    if tpi is None:
        raise ValueError(
            f"no trajectory over {abs(pe - p0):.6g} from speed {v0:.6g} within accel "
            f"{accel:.6g} / decel {accel if decel is None else decel:.6g}"
        )
    return tpi


def extract_phase_params(tpi: TwoPointInterpolation) -> Tuple[float, float, float, float, float]:
    """Read trapezoidal phase scalars from a constructed TPI.

    Returns ``(t_accel, t_const, t_total, accel, decel)`` relative to
    ``tpi.t0`` in the form expected by :func:`trapezoid_distance` for batched
    eval.

    Handles all three TPI cases:

    - ``case == -1`` (degenerate / zero distance): all zeros.
    - ``case == 0`` (triangle, ``vmax`` not reached): ``t_const = 0``.
    - ``case == 1`` (full trapezoid): three populated phases.

    An asymmetric profile needs nothing special here. ``tpi.dt`` already holds
    the phase durations ``TwoPointInterpolation`` computed from both scalars;
    only ``decel`` had to be carried out alongside ``accel`` for
    :func:`trapezoid_distance` to integrate the braking phase with it.
    """
    dt = tpi.dt
    accel = float(tpi.amax_accel)
    decel = float(tpi.amax_decel)
    if len(dt) == 0:
        return 0.0, 0.0, 0.0, accel, decel
    if len(dt) == 2:
        t_accel = float(dt[0])
        t_decel = float(dt[1])
        return t_accel, 0.0, t_accel + t_decel, accel, decel
    # len == 3 (full trapezoid)
    t_accel = float(dt[0])
    t_const = float(dt[1])
    t_total = float(dt[0] + dt[1] + dt[2])
    return t_accel, t_const, t_total, accel, decel


def trapezoid_distance(
    tau: np.ndarray,
    t_accel: np.ndarray,
    t_const: np.ndarray,
    t_total: np.ndarray,
    accel: np.ndarray,
    total_distance: np.ndarray,
    decel: Optional[np.ndarray] = None,
    v0: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Vectorised distance-traveled lookup for trapezoidal profiles.

    All inputs are ``(N,)`` arrays; returns ``(N,)``.

    ``v0`` is the speed each trajectory starts at (``None``: at rest), as built
    by :func:`build_tpi_from_speed`; the ramp up then starts from it.

    ``decel`` is the braking scalar for the final phase. ``None`` reuses
    ``accel``, which is the symmetric profile and what every caller got before
    this parameter existed.
    """
    if decel is None:
        decel = accel
    out = np.zeros_like(tau)
    # phase masks
    in_const = (tau > t_accel) & (tau <= t_accel + t_const)
    in_decel = (tau > t_accel + t_const) & (tau < t_total)
    done = tau >= t_total
    in_accel = (tau > 0.0) & ~in_const & ~in_decel & ~done

    if v0 is None:
        v0 = np.zeros_like(tau)
    # accel phase: v0 * t + 0.5 * a * t^2
    if in_accel.any():
        t = tau[in_accel]
        out[in_accel] = v0[in_accel] * t + 0.5 * accel[in_accel] * t * t
    # const phase: v0 * t_accel + 0.5 * a * t_accel^2 + v_peak * (t - t_accel),
    # where v_peak = v0 + a * t_accel
    if in_const.any():
        ta = t_accel[in_const]
        a = accel[in_const]
        u = v0[in_const]
        out[in_const] = u * ta + 0.5 * a * ta * ta + (u + a * ta) * (tau[in_const] - ta)
    # decel phase: total_distance - 0.5 * a * (t_total - t)^2
    if in_decel.any():
        d = decel[in_decel]
        t_left = t_total[in_decel] - tau[in_decel]
        out[in_decel] = total_distance[in_decel] - 0.5 * d * t_left * t_left
    if done.any():
        out[done] = total_distance[done]
    return out
