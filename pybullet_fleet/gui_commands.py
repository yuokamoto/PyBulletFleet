"""Toolkit-neutral monitor commands and read-only monitor frames.

The monitor window may run on a GUI thread, while simulation advancement and
PyBullet access always remain on the simulation thread.  These small value
objects are the boundary between them and intentionally do not reference a
renderer or a GUI toolkit.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class GuiCommandType(str, Enum):
    """Operations a monitor UI may request from a simulation."""

    PAUSE = "pause"
    RESUME = "resume"
    SINGLE_STEP = "single_step"
    SELECT_ENTITY = "select_entity"
    SET_FOLLOW = "set_follow"
    SET_SIMULATION_PACING = "set_simulation_pacing"
    SET_ENTITY_POSE = "set_entity_pose"


@dataclass(frozen=True)
class GuiCommand:
    """A monitor-originated request, consumed at a simulation step boundary."""

    command: GuiCommandType
    source: str = "monitor"
    entity_id: Optional[int] = None
    enabled: Optional[bool] = None
    target_rtf: Optional[float] = None
    timestep: Optional[float] = None
    position: Optional[Tuple[float, float, float]] = None
    yaw_radians: Optional[float] = None
    rpy_radians: Optional[Tuple[float, float, float]] = None


@dataclass(frozen=True)
class MonitorEntity:
    """Stable entity summary for an inspector."""

    object_id: int
    name: str
    kind: str


@dataclass(frozen=True)
class SelectedEntityMonitorState:
    """Detailed state for the one selected entity."""

    entity: MonitorEntity
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]
    action_type: Optional[str]
    action_status: Optional[str]
    queued_actions: int
    attached_objects: int
    active_collisions: int


@dataclass(frozen=True)
class MonitorFrame:
    """Read-only, toolkit-neutral summary of the current simulation state."""

    sim_time: float
    real_time: float
    target_rtf: float
    actual_rtf: float
    timestep: float
    physics_enabled: bool
    agents: int
    objects: int
    active_collisions: int
    collisions: int
    steps: int
    paused: bool
    entities: Tuple[MonitorEntity, ...] = ()
    selected_entity: Optional[SelectedEntityMonitorState] = None
    follow_enabled: bool = False
