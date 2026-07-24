"""Transport-neutral fleet state snapshot types."""

from __future__ import annotations

from dataclasses import dataclass

from pybullet_fleet.commands import Quat, Vec2, Vec3


@dataclass(frozen=True)
class RobotState2D:
    """Minimal planar state for one robot."""

    name: str
    object_id: int
    position: Vec2
    yaw: float
    linear_velocity: Vec2 = (0.0, 0.0)
    angular_velocity: float = 0.0
    is_moving: bool = False
    battery_soc: float | None = None
    is_charging: bool | None = None


@dataclass(frozen=True)
class RobotState3D:
    """Minimal 3D state for one robot."""

    name: str
    object_id: int
    position: Vec3
    orientation: Quat
    linear_velocity: Vec3 = (0.0, 0.0, 0.0)
    angular_velocity: Vec3 = (0.0, 0.0, 0.0)
    is_moving: bool = False
    battery_soc: float | None = None
    is_charging: bool | None = None
