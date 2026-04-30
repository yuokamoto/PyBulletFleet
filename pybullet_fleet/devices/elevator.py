"""Elevator device — Agent subclass with prismatic Z joint and auto-attach.

Objects (agents, sim_objects) on the platform are automatically attached
before movement and detached on arrival, so they physically ride the
elevator without any external teleport logic.
"""

from __future__ import annotations

import pybullet as p

from dataclasses import dataclass, field, fields
from typing import TYPE_CHECKING, Any, Dict, List, Optional

from pybullet_fleet.action import JointAction
from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.logging_utils import get_lazy_logger
from pybullet_fleet.types import ActionStatus

if TYPE_CHECKING:
    from pybullet_fleet.sim_object import SimObject

logger = get_lazy_logger(__name__)


@dataclass
class ElevatorParams(AgentSpawnParams):
    """Spawn parameters for an Elevator device.

    Extends AgentSpawnParams with elevator-specific floor/joint config.

    Attributes:
        floors: Floor name → joint position mapping.
            Required — ``None`` raises TypeError in ``__post_init__``.
        initial_floor: Name of the floor the elevator starts at.
        joint_name: Name of the prismatic joint that moves the platform.
        platform_link: Name of the link with the platform collision box.
    """

    floors: Optional[Dict[str, float]] = None
    initial_floor: str = ""
    joint_name: str = "lift"
    platform_link: str = "platform"

    def __post_init__(self):
        super().__post_init__()
        if self.floors is None:
            raise TypeError("ElevatorParams requires floors (Dict[str, float])")
        if not self.initial_floor:
            self.initial_floor = next(iter(self.floors), "")

    @classmethod
    def from_dict(cls, config: Dict[str, Any]) -> "ElevatorParams":
        """Create ElevatorParams from a config dict.

        Delegates base-field parsing to ``AgentSpawnParams.from_dict``
        and adds elevator-specific fields (``floors``, ``initial_floor``,
        ``joint_name``, ``platform_link``).
        """
        base = AgentSpawnParams.from_dict(config)
        base_kwargs = {f.name: getattr(base, f.name) for f in fields(base)}
        return cls(
            **base_kwargs,
            floors=config.get("floors"),
            initial_floor=config.get("initial_floor", ""),
            joint_name=config.get("joint_name", "lift"),
            platform_link=config.get("platform_link", "platform"),
        )


class Elevator(Agent):
    """Elevator with named floors and automatic passenger attachment.

    Uses JointAction for floor transitions.  Before movement starts,
    all sim_objects whose AABB overlaps the platform collision box are
    attached to the platform link.  On arrival they are detached.

    Config (YAML)::

        type: elevator
        urdf_path: robots/elevator.urdf
        use_fixed_base: true
        floors: {L1: 0.0, L2: 8.0, L3: 16.0}
        initial_floor: L1
        joint_name: lift
        platform_link: platform
    """

    _spawn_params_cls = ElevatorParams
    _entity_type_name = "elevator"

    @classmethod
    def from_params(cls, spawn_params: "ElevatorParams", sim_core=None) -> "Elevator":
        """Create an Elevator from ElevatorParams.

        Raises:
            TypeError: If *spawn_params* is not an ``ElevatorParams`` instance
                or ``floors`` is missing.
        """
        if not isinstance(spawn_params, ElevatorParams):
            raise TypeError(f"Elevator.from_params requires ElevatorParams, got {type(spawn_params).__name__}")
        agent = super().from_params(spawn_params, sim_core)
        agent._floors: Dict[str, float] = spawn_params.floors  # type: ignore[assignment]
        agent._joint_name: str = spawn_params.joint_name
        agent._platform_link: str = spawn_params.platform_link
        agent._current_floor_name: str = spawn_params.initial_floor
        agent._target_floor_name: str = spawn_params.initial_floor
        agent._passengers: List[SimObject] = []  # Currently attached passengers
        agent._moving: bool = False
        return agent

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def current_floor(self) -> str:
        """Floor where the elevator currently is (or departed from while moving).

        While the elevator is in transit this returns the **departure** floor,
        not the destination.  Use :pyattr:`target_floor` for the destination.
        """
        for name, pos in self._floors.items():
            if self.are_joints_at_targets({self._joint_name: pos}):
                self._current_floor_name = name
                return name
        return self._current_floor_name

    @property
    def target_floor(self) -> str:
        """Destination floor (equals :pyattr:`current_floor` when idle)."""
        return self._target_floor_name

    @property
    def available_floors(self) -> List[str]:
        """List of available floor names."""
        return list(self._floors.keys())

    @property
    def is_moving(self) -> bool:
        """True while elevator is transitioning between floors."""
        return self._moving

    @property
    def passengers(self) -> List["SimObject"]:
        """Currently attached passengers (read-only copy)."""
        return list(self._passengers)

    # ------------------------------------------------------------------
    # Floor request
    # ------------------------------------------------------------------

    def request_floor(self, floor_name: str) -> None:
        """Request the elevator to move to the given floor.

        Attaches all objects on the platform, then starts the joint action.

        Args:
            floor_name: Name of the target floor (must exist in ``floors``).
        """
        if floor_name not in self._floors:
            logger.warning("Unknown floor: %s (available: %s)", floor_name, list(self._floors.keys()))
            return
        if floor_name == self.current_floor:
            return
        if self._moving and floor_name == self._target_floor_name:
            return

        self._target_floor_name = floor_name
        target_pos = self._floors[floor_name]

        # Attach all objects currently on the platform
        self._attach_platform_objects()

        self._moving = True
        self.clear_actions()
        self.add_action(JointAction(target_joint_positions={self._joint_name: target_pos}))
        logger.info(
            "Elevator '%s': moving '%s' -> '%s' (%d passengers)",
            self.name,
            self.current_floor,
            floor_name,
            len(self._passengers),
        )

    # ------------------------------------------------------------------
    # Update — detach on arrival
    # ------------------------------------------------------------------

    def update(self, dt: float) -> None:
        """Per-step update.  Detaches passengers when movement completes."""
        super().update(dt)

        if self._moving:
            action = self.get_current_action()
            still_moving = action is not None and isinstance(action, JointAction) and action.status == ActionStatus.IN_PROGRESS
            if not still_moving:
                self._moving = False
                n = len(self._passengers)
                self._detach_all_passengers()
                logger.info(
                    "Elevator '%s': arrived at '%s', %d passengers released",
                    self.name,
                    self.current_floor,
                    n,
                )

    # ------------------------------------------------------------------
    # Platform object detection & attach/detach
    # ------------------------------------------------------------------

    def _get_platform_link_index(self) -> int:
        """Resolve the platform link name to a PyBullet link index."""
        num_joints = p.getNumJoints(self.body_id, physicsClientId=self._pid)
        for i in range(num_joints):
            info = p.getJointInfo(self.body_id, i, physicsClientId=self._pid)
            link_name = info[12].decode("utf-8")
            if link_name == self._platform_link:
                return i
        logger.warning("Platform link '%s' not found, using link 0", self._platform_link)
        return 0

    def _get_platform_aabb(self) -> tuple:
        """Get the AABB of the platform link.

        Returns:
            Tuple of ((min_x, min_y, min_z), (max_x, max_y, max_z)).
        """
        link_idx = self._get_platform_link_index()
        return p.getAABB(self.body_id, link_idx, physicsClientId=self._pid)

    def _find_objects_on_platform(self) -> List["SimObject"]:
        """Find sim_objects whose AABB overlaps the platform AABB.

        Only considers objects that are not the elevator itself and are
        not already attached to something.
        """
        if self.sim_core is None:
            return []

        platform_min, platform_max = self._get_platform_aabb()

        # Expand platform AABB slightly upward to catch objects standing on it
        # (robot feet may be just above the platform top surface).
        search_min = (platform_min[0], platform_min[1], platform_min[2])
        search_max = (platform_max[0], platform_max[1], platform_max[2] + 1.0)

        candidates = []
        for obj in self.sim_core.sim_objects:
            if obj is self:
                continue
            if obj.is_attached():
                continue
            try:
                obj_min, obj_max = p.getAABB(obj.body_id, physicsClientId=self._pid)
            except Exception:
                continue

            # AABB overlap test
            if (
                obj_min[0] <= search_max[0]
                and obj_max[0] >= search_min[0]
                and obj_min[1] <= search_max[1]
                and obj_max[1] >= search_min[1]
                and obj_min[2] <= search_max[2]
                and obj_max[2] >= search_min[2]
            ):
                candidates.append(obj)

        return candidates

    def _attach_platform_objects(self) -> None:
        """Attach all objects currently on the platform."""
        objects = self._find_objects_on_platform()
        link_idx = self._get_platform_link_index()

        for obj in objects:
            # Temporarily enable pickable for the attach call
            # (agents have pickable=False by default, but elevator attachment is valid)
            was_pickable = obj.pickable
            obj.pickable = True
            success = self.attach_object(obj, parent_link_index=link_idx, keep_world_pose=True)
            if not was_pickable:
                obj.pickable = was_pickable

            if success:
                self._passengers.append(obj)
                logger.info(
                    "Elevator '%s': attached '%s' (body %d) to platform",
                    self.name,
                    getattr(obj, "name", "?"),
                    obj.body_id,
                )

    def _detach_all_passengers(self) -> None:
        """Detach all currently attached passengers."""
        for obj in list(self._passengers):
            self.detach_object(obj)
            logger.info(
                "Elevator '%s': detached '%s' (body %d)",
                self.name,
                getattr(obj, "name", "?"),
                obj.body_id,
            )
        self._passengers.clear()
