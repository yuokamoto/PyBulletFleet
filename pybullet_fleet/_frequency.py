"""When a frequency-limited handler is due, on simulated time.

Simulated time is accumulated one timestep at a time, so it carries floating
point error: ten steps of 0.1 s add up to 0.9999999999999999 s. A check of
``now - last >= interval`` would miss a beat whenever that error falls short,
and measuring from the time a handler actually ran would let its phase drift
when the interval is not a whole number of timesteps.

:class:`FrequencyGate` instead fires on a fixed grid of due times, ``k *
interval`` from time zero, with a tolerance far below one timestep. It fires
at most once per call (one call per step), and after a gap -- a pause, a
handler registered late, a restore -- it moves on to the next grid point
rather than firing once for every point it missed.
"""

import math
from typing import Optional

from pybullet_fleet.logging_utils import get_lazy_logger

logger = get_lazy_logger(__name__)

#: Fraction of a timestep by which simulated time may fall short of a due time
#: and still count as reaching it. Accumulated error is many orders of magnitude
#: smaller than this; a real shortfall is a whole timestep.
_TOLERANCE = 1e-6


class FrequencyGate:
    """Decide, once per step, whether a handler running at ``frequency`` Hz is due.

    ``frequency`` ``None`` means every step. The gate keeps no reference to
    the simulation; callers pass the step's simulated time and timestep.
    An owner creates its gate once and changes the rate with
    :meth:`set_frequency`.
    """

    __slots__ = ("_frequency", "_next_index", "_last_time", "_warned")

    def __init__(self, frequency: Optional[float] = None) -> None:
        self._frequency = self._checked(frequency)
        self._warned = False
        self.reset()

    @property
    def frequency(self) -> Optional[float]:
        """Rate in Hz, or ``None`` for every step. Change it with :meth:`set_frequency`."""
        return self._frequency

    def set_frequency(self, frequency: Optional[float]) -> None:
        """Run at ``frequency`` from now on; a new rate starts a fresh schedule.

        Setting the current rate again changes nothing.
        """
        frequency = self._checked(frequency)
        if frequency == self._frequency:
            return
        self._frequency = frequency
        self.reset()

    @staticmethod
    def _checked(frequency: Optional[float]) -> Optional[float]:
        if frequency is not None and not (math.isfinite(frequency) and frequency > 0):
            raise ValueError("frequency must be None or a positive finite number, got {!r}".format(frequency))
        return frequency

    def reset(self) -> None:
        """Forget the schedule, as at the start of a run."""
        self._next_index: Optional[int] = None
        self._last_time: Optional[float] = None

    def due(self, now: float, timestep: float) -> Optional[float]:
        """Whether the handler runs at ``now``, and with which ``dt``.

        Returns:
            ``None`` when the handler is not due at ``now``. When it is due, the
            simulated time since it last ran (``dt_since_last_run``), to pass to
            the handler: always positive, and one interval (at least one
            timestep) on the first run. Test with ``is not None``.

        A handler first runs one interval into the run (the first grid point
        after time zero), or at once when its first call is already on a due
        time.
        """
        if self.frequency is None:
            dt = timestep if self._last_time is None else now - self._last_time
            last = self._last_time
            self._last_time = now
            return self._positive(dt, now, last, timestep)

        # Grid points within ``tolerance`` of ``now`` count as ``now`` itself, in
        # either direction, so float error in the accumulated time neither skips
        # a due time nor runs the same one twice.
        interval = 1.0 / self.frequency
        tolerance = _TOLERANCE * timestep
        if self._next_index is None:
            # Baseline: the first grid point at or after now, and never the
            # run's start itself -- a handler first runs one interval into the
            # run. A first call already on a due time (after a restore, or a
            # post-step schedule whose first time is one step in) runs at once.
            # ceil((now - tolerance) / interval): the first grid point at or after now.
            self._next_index = max(1, math.ceil((now - tolerance) / interval))
        if now + tolerance < self._next_index * interval:
            return None
        # Due. Schedule the next grid point after now: normally the following
        # one, after a gap the first one still ahead. floor((now + tolerance) /
        # interval) is the last grid point at or before now.
        self._next_index = max(self._next_index + 1, math.floor((now + tolerance) / interval) + 1)
        last = self._last_time
        self._last_time = now
        # The first run has no previous one: report the interval, or one step
        # when the interval is shorter than a step.
        dt = now - last if last is not None else max(interval, timestep)
        return self._positive(dt, now, last, timestep)

    def _positive(self, dt: float, now: float, last: Optional[float], timestep: float) -> float:
        """``dt``, or one timestep with a warning when it is not positive.

        A non-positive ``dt`` means the gate was asked twice at the same time or
        the clock went back without :meth:`reset` -- a caller error. Handlers
        still get a usable ``dt``; the warning is logged once per gate.
        """
        if dt > 0:
            return dt
        if not self._warned:
            self._warned = True
            logger.warning(
                "FrequencyGate: non-positive dt %.9g at time %.9g (previous run at %r); "
                "was the gate called twice in one step, or the clock moved back without reset()? "
                "Using one timestep (%.9g) instead.",
                dt,
                now,
                last,
                timestep,
            )
        return timestep
