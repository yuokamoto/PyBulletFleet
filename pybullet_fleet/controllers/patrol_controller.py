"""Waypoint patrol controller."""

from __future__ import annotations

from pybullet_fleet.controller import Controller
from pybullet_fleet.geometry import Pose
from pybullet_fleet.logging_utils import get_lazy_logger

logger = get_lazy_logger(__name__)


class PatrolController(Controller):
    """Cycle through waypoints. Delegates actual movement to the base controller
    by calling ``agent.set_goal_pose()``.

    Args:
        waypoints: List of [x, y, z] positions.
        wait_time: Seconds to wait at each waypoint before advancing.
        loop: If True, restart from first waypoint after reaching last.
    """

    _registry_name = "patrol"

    def __init__(
        self,
        waypoints: list | None = None,
        wait_time: float = 0.0,
        loop: bool = True,
    ):
        self._waypoints = waypoints or []
        self._current_idx = 0
        self._wait_time = wait_time
        self._wait_timer = 0.0
        self._loop = loop
        self._started = False

    def compute(self, agent, dt: float) -> bool:
        if not self._waypoints:
            return False

        if self._wait_timer > 0:
            self._wait_timer -= dt
            return False

        # First call: set initial waypoint
        if not self._started:
            self._started = True
            target = self._waypoints[self._current_idx]
            agent.set_goal_pose(Pose.from_xyz(*target))
            return False

        # Advance when agent stops moving
        if not agent.is_moving:
            self._current_idx += 1
            if self._current_idx >= len(self._waypoints):
                if self._loop:
                    self._current_idx = 0
                else:
                    return False  # done
            target = self._waypoints[self._current_idx]
            agent.set_goal_pose(Pose.from_xyz(*target))
            self._wait_timer = self._wait_time

        return False  # actual movement handled by base controller
