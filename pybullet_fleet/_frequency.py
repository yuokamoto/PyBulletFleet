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

#: Fraction of a timestep by which simulated time may fall short of a due time
#: and still count as reaching it. Accumulated error is many orders of magnitude
#: smaller than this; a real shortfall is a whole timestep.
_TOLERANCE = 1e-6


class FrequencyGate:
    """Decide, once per step, whether a handler running at ``frequency`` Hz is due.

    ``frequency`` ``None`` means every step. The gate keeps no reference to
    the simulation; callers pass the step's simulated time and timestep.
    """

    __slots__ = ("frequency", "_next_index", "_last_time")

    def __init__(self, frequency: Optional[float] = None) -> None:
        if frequency is not None and not (math.isfinite(frequency) and frequency > 0):
            raise ValueError("frequency must be None or a positive finite number, got {!r}".format(frequency))
        self.frequency = frequency
        self.reset()

    def reset(self) -> None:
        """Forget the schedule, as at the start of a run."""
        self._next_index: Optional[int] = None
        self._last_time: Optional[float] = None

    def due(self, now: float, timestep: float) -> Optional[float]:
        """Return the time since the handler last ran if it is due at ``now``, else ``None``.

        A handler first runs one interval into the run (the first grid point
        after time zero), or at once when its first call is already on a due
        time. The returned ``dt`` is the
        simulated time since the previous run; the first run reports one
        interval (at least one timestep). It is never zero or negative.
        """
        if self.frequency is None:
            dt = timestep if self._last_time is None else now - self._last_time
            self._last_time = now
            return dt if dt > 0 else timestep

        interval = 1.0 / self.frequency
        slack = _TOLERANCE * timestep
        if self._next_index is None:
            # Baseline: the first grid point at or after now, and never the
            # run's start itself -- a handler first runs one interval into the
            # run. A first call already on a due time (after a restore, or a
            # post-step schedule whose first time is one step in) runs at once.
            self._next_index = max(1, math.ceil((now - slack) / interval))
        if now + slack < self._next_index * interval:
            return None
        # Due. Schedule the next grid point after now: normally the following
        # one, after a gap the first one still ahead.
        self._next_index = max(self._next_index + 1, math.floor((now + slack) / interval) + 1)
        last = self._last_time
        self._last_time = now
        # The first run has no previous one: report the interval, or one step
        # when the interval is shorter than a step.
        dt = now - last if last is not None else max(interval, timestep)
        return dt if dt > 0 else timestep
