"""Transport-neutral client contract for the RMF fleet adapter."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass
class RobotUpdateData:
    """Snapshot of robot state for EasyFullControl updates."""

    map: str
    position: list  # [x, y, yaw]
    battery_soc: float
    last_completed_cmd_id: int

    def is_command_completed(self, cmd_id: int) -> bool:
        """Check if the given command has been completed."""
        return self.last_completed_cmd_id >= cmd_id


class RmfRobotClient(Protocol):
    """Per-robot client contract consumed by :class:`RobotAdapter`.

    Implementations may use per-robot ROS actions/services, fleet-level ROS
    endpoints, or direct Python APIs. The RMF adapter should not need to know
    which transport is behind this interface.
    """

    def get_data(self) -> RobotUpdateData | None:
        """Return latest robot state, or ``None`` until state is available."""
        ...

    def set_map_name(self, map_name: str) -> None:
        """Update the map/level associated with subsequent RMF state updates."""
        ...

    def navigate(self, cmd_id: int, position: list, map_name: str, speed_limit: float = 0.0) -> bool:
        """Command navigation to ``position`` on ``map_name``."""
        ...

    def stop(self) -> bool:
        """Stop the robot's active navigation command."""
        ...

    def start_charge(self, cmd_id: int) -> bool:
        """Start charging and complete ``cmd_id`` once accepted."""
        ...

    def stop_charge(self) -> bool:
        """Stop charging."""
        ...

    def toggle_attach(self, attach: bool, cmd_id: int) -> bool:
        """Request simple attach/detach for delivery actions."""
        ...


class RmfClientFactory(Protocol):
    """Factory for per-robot RMF clients sharing a transport backend."""

    def robot(self, robot_name: str) -> RmfRobotClient:
        """Return a client for ``robot_name``."""
        ...
