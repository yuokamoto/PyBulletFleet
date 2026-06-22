"""Lightweight event pub/sub for simulation lifecycle events.

Two usage levels:

- **Global bus** (``sim.events``): sim-wide events (pre_step, agent_spawned, etc.)
- **Per-entity bus** (``agent.events``): entity-specific hooks for subclasses

Handlers execute in priority order (lower = first). Same priority = FIFO.
Exceptions in handlers are logged but do not block other handlers.

Pre-defined event names are available as :class:`SimEvents` constants.
Custom event names (any ``str``) are also supported for user-defined extensions.
"""

from collections import defaultdict
from typing import Any, Callable, Dict, List, Optional

import logging

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Pre-defined event name constants
# ---------------------------------------------------------------------------


class SimEvents:
    """Pre-defined event name constants.

    Using these constants prevents typos and enables IDE auto-completion,
    while the :class:`EventBus` still accepts any ``str`` for user-defined
    events.

    **Simulation lifecycle** (emitted on ``sim.events``)::

        sim.events.on(SimEvents.PRE_STEP, handler)
        sim.events.on(SimEvents.POST_STEP, handler)

    **Entity lifecycle** (emitted on both ``sim.events`` and ``entity.events``)::

        sim.events.on(SimEvents.OBJECT_SPAWNED, handler)
        agent.events.on(SimEvents.ACTION_COMPLETED, handler)

    **Custom events** (any string — no constant needed)::

        sim.events.on("my_plugin_event", handler)
    """

    # --- Simulation step -------------------------------------------------
    PRE_STEP: str = "pre_step"
    """Emitted at the start of each step.  kwargs: ``dt``, ``sim_time``."""

    POST_STEP: str = "post_step"
    """Emitted at the end of each step.  kwargs: ``dt``, ``sim_time``."""

    PAUSED: str = "paused"
    """Emitted when simulation is paused.  No kwargs."""

    RESUMED: str = "resumed"
    """Emitted when simulation is resumed.  No kwargs."""

    # --- Object / Agent lifecycle ----------------------------------------
    OBJECT_SPAWNED: str = "object_spawned"
    """Global: ``obj``.  Per-entity: no kwargs."""

    OBJECT_REMOVED: str = "object_removed"
    """Global: ``obj``.  Per-entity: no kwargs."""

    AGENT_SPAWNED: str = "agent_spawned"
    """Global only: ``agent``."""

    AGENT_REMOVED: str = "agent_removed"
    """Global only: ``agent``."""

    # --- Collision -------------------------------------------------------
    COLLISION_STARTED: str = "collision_started"
    """Global: ``obj_a``, ``obj_b``.  Per-entity: ``other``."""

    COLLISION_ENDED: str = "collision_ended"
    """Global: ``obj_a``, ``obj_b``.  Per-entity: ``other``."""

    # --- Agent update -----------------------------------------------------
    PRE_UPDATE: str = "pre_update"
    """Per-entity only: ``dt``.  Emitted before agent movement/action processing."""

    POST_UPDATE: str = "post_update"
    """Per-entity only: ``dt``, ``moved`` (bool).  Emitted after agent update completes."""

    # --- Action ----------------------------------------------------------
    ACTION_STARTED: str = "action_started"
    """Global: ``agent``, ``action``.  Per-entity: ``action``."""

    ACTION_COMPLETED: str = "action_completed"
    """Global: ``agent``, ``action``, ``status``.  Per-entity: ``action``, ``status``."""


class EventBus:
    """Lightweight event pub/sub bus.

    Usage::

        bus = EventBus()
        bus.on("collision_started", my_handler, priority=0)
        bus.emit("collision_started", obj_a=a, obj_b=b)
        bus.off("collision_started", my_handler)
    """

    __slots__ = ("_handlers", "_sorted")

    def __init__(self) -> None:
        self._handlers: Dict[str, List[tuple]] = defaultdict(list)
        self._sorted: Dict[str, bool] = {}

    def on(self, event: str, handler: Callable, priority: int = 0) -> None:
        """Register handler. Lower priority = executed first."""
        self._handlers[event].append((priority, handler))
        self._sorted[event] = False

    def off(self, event: str, handler: Callable) -> None:
        """Unregister handler."""
        self._handlers[event] = [(p, h) for p, h in self._handlers[event] if h is not handler]

    def emit(self, event: str, **kwargs: Any) -> None:
        """Fire event. Handlers called in priority order."""
        handlers = self._handlers.get(event)
        if not handlers:
            return
        if not self._sorted.get(event, True):
            handlers.sort(key=lambda x: x[0])
            self._sorted[event] = True
        for _, handler in handlers:
            try:
                handler(**kwargs)
            except Exception:
                logger.exception("EventBus handler error for event '%s'", event)

    def clear(self, event: Optional[str] = None) -> None:
        """Clear handlers. If event given, clear only that event."""
        if event:
            self._handlers.pop(event, None)
            self._sorted.pop(event, None)
        else:
            self._handlers.clear()
            self._sorted.clear()

    def has_handlers(self, event: str) -> bool:
        """Check if any handlers are registered for an event."""
        return bool(self._handlers.get(event))
