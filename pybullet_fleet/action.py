"""
action.py
Base classes and implementations for robot actions.

Provides a unified interface for different types of robot actions:
- MoveAction: Path following
- PickAction: Object picking with approach
- DropAction: Object dropping with approach
- WaitAction: Timed waiting (e.g., charging)

Actions are executed through an action queue system in the Agent class.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Union, TYPE_CHECKING
import logging
import numpy as np
import pybullet as p

from .geometry import Pose, Path
from .types import ActionStatus, MovementDirection
from . import tools

if TYPE_CHECKING:
    from .sim_object import SimObject

# Create logger for this module
logger = logging.getLogger(__name__)

# Note: ActionStatus and MovementDirection have been moved to types.py


class PickPhase(Enum):
    """Phases of pick action execution."""

    INIT = "init"
    APPROACHING = "approaching"
    MOVING_TO_PICK = "moving_to_pick"
    PICKING = "picking"
    RETREATING = "retreating"


class DropPhase(Enum):
    """Phases of drop action execution."""

    INIT = "init"
    APPROACHING = "approaching"
    MOVING_TO_DROP = "moving_to_drop"
    DROPPING = "dropping"
    RETREATING = "retreating"


class Action(ABC):
    """
    Base class for all robot actions.

    Actions represent high-level tasks that agents can perform:
    - MoveAction: Navigate along a path
    - PickAction: Pick up an object
    - DropAction: Drop an object
    - WaitAction: Wait for a duration

    Actions are executed through Agent.add_action() or Agent.add_action_sequence().
    Each action manages its own state and returns True when complete.

    Attributes:
        status: Current execution status
        start_time: When action started (simulation time)
        end_time: When action completed (simulation time)
        error_message: Error description if action failed
    """

    def __init__(self):
        self.status = ActionStatus.NOT_STARTED
        self.start_time: Optional[float] = None
        self.end_time: Optional[float] = None
        self.error_message: Optional[str] = None

    @abstractmethod
    def execute(self, agent, dt: float) -> bool:
        """
        Execute action for one timestep.

        Args:
            agent: Agent instance executing this action
            dt: Timestep duration in seconds

        Returns:
            True if action is complete (success or failure), False if still executing
        """
        pass

    @abstractmethod
    def reset(self):
        """Reset action state for re-execution."""
        self.status = ActionStatus.NOT_STARTED
        self.start_time = None
        self.end_time = None
        self.error_message = None

    def cancel(self):
        """Cancel action execution."""
        self.status = ActionStatus.CANCELLED
        if self.start_time is not None and self.end_time is None:
            # Action was in progress, set end time
            self.end_time = self.start_time  # Use start time as placeholder

    def get_duration(self) -> Optional[float]:
        """
        Get action execution duration.

        Returns:
            Duration in seconds, or None if not yet completed
        """
        if self.start_time is None or self.end_time is None:
            return None
        return self.end_time - self.start_time

    def is_complete(self) -> bool:
        """Check if action has completed (successfully or with failure)."""
        return self.status in [ActionStatus.COMPLETED, ActionStatus.FAILED, ActionStatus.CANCELLED]


@dataclass
class MoveAction(Action):
    """
    Move along a path.

    Wraps the existing Agent path following functionality into the Action interface.

    Args:
        path: Path object with waypoints to follow
        auto_approach: Add approach waypoint if far from start (default: True)
        final_orientation_align: Align to final orientation after reaching end (default: True)
        direction: Movement direction - MovementDirection.FORWARD (default, face movement direction)
                  or MovementDirection.BACKWARD (maintain orientation, move backward).
                  Can also accept string "forward" or "backward" for convenience.

    Example:
        # Move forward (normal)
        path = Path.from_positions([[0, 0, 0], [1, 0, 0], [1, 1, 0]])
        action = MoveAction(path=path)
        agent.add_action(action)

        # Move backward
        action = MoveAction(path=path, direction=MovementDirection.BACKWARD)
        agent.add_action(action)
    """

    path: Path
    auto_approach: bool = True
    final_orientation_align: bool = True
    direction: Union[MovementDirection, str] = MovementDirection.FORWARD

    def __post_init__(self):
        super().__init__()
        # Normalize string to enum if needed
        if isinstance(self.direction, str):
            self.direction = MovementDirection(self.direction)

    def execute(self, agent, dt: float) -> bool:
        """Execute move action."""
        if self.status == ActionStatus.NOT_STARTED:
            # Start action: set path on agent
            self.status = ActionStatus.IN_PROGRESS
            self.start_time = agent.sim_core.sim_time if agent.sim_core else 0.0

            # Use agent's existing path following system
            # Path visualization is handled by agent based on agent.path_visualize setting
            agent.set_path(
                self.path,
                auto_approach=self.auto_approach,
                final_orientation_align=self.final_orientation_align,
                direction=self.direction,  # Pass direction parameter (enum or string)
            )

            direction_str = "backward" if self.direction == MovementDirection.BACKWARD else "forward"
            logger.info(f"Started path following with {len(self.path)} waypoints ({direction_str})")

        # Check if agent has completed the path
        # Path is complete when agent is not moving and path queue is empty
        if not agent.is_moving and not agent._path:
            self.status = ActionStatus.COMPLETED
            self.end_time = agent.sim_core.sim_time if agent.sim_core else 0.0

            logger.info(f"Completed path following in {self.get_duration():.2f}s")
            return True

        return False

    def reset(self):
        """Reset action state."""
        super().reset()


@dataclass
class WaitAction(Action):
    """
    Wait for a specified duration.

    Useful for simulating charging, loading, or other time-based operations.

    Args:
        duration: Time to wait in seconds
        action_type: Type of wait action (default: "idle")
                    Options: "idle", "charge", "loading", "custom"
        show_indicator: Show progress indicator (default: True)
        indicator_color: RGB color for indicator [r, g, b] (default: yellow)

    Example:
        # Wait for 5 seconds (charging)
        action = WaitAction(duration=5.0, action_type="charge")
        agent.add_action(action)
    """

    duration: float
    action_type: str = "idle"

    # Internal state
    _elapsed_time: float = field(default=0.0, init=False)

    def __post_init__(self):
        super().__init__()

    def execute(self, agent, dt: float) -> bool:
        """Execute wait action."""
        if self.status == ActionStatus.NOT_STARTED:
            self.status = ActionStatus.IN_PROGRESS
            self.start_time = agent.sim_core.sim_time if agent.sim_core else 0.0
            self._elapsed_time = 0.0
            print(f"[WaitAction] Starting {self.action_type} for {self.duration:.1f}s")

        # Update elapsed time
        self._elapsed_time += dt

        # Check if duration has elapsed
        if self._elapsed_time >= self.duration:
            self.status = ActionStatus.COMPLETED
            self.end_time = agent.sim_core.sim_time if agent.sim_core else 0.0
            print(f"[WaitAction] Completed {self.action_type}")
            return True

        return False

    def reset(self):
        """Reset action state."""
        super().reset()
        self._elapsed_time = 0.0


@dataclass
class PickAction(Action):
    """
    Pick an object and attach it to the robot.

    Kinematic implementation:
    1. Move to approach pose (if specified or auto-calculated)
    2. Teleport object to attachment position
    3. Attach object using constraint

    Args:
        target_object_id: Specific object body ID to pick (optional)
        target_position: Pick from position - auto-select nearest pickable object (optional)
        search_radius: Search radius when using target_position (default: 0.5m)
        approach_pose: Explicit approach pose (optional, auto-calculated if None)
        approach_offset: Auto-calculate approach offset distance (default: 1.0m)
        pick_offset: Distance from target where pick is executed (default: 0.0m)
        attach_link: Link to attach to - can be index (int) or name (str). -1 or "base_link" for base link (default: -1)
        attach_relative_pose: Position and orientation offset in link's frame as Pose object

    Note:
        Path visualization is controlled by agent.path_visualize setting.

    Example:
        # Pick specific object with position offset (using link index)
        action = PickAction(
            target_object_id=pallet.body_id,
            approach_offset=1.0,
            pick_offset=0.3,
            attach_link=-1,  # Base link
            attach_relative_pose=Pose.from_euler(0.6, 0, -0.2, roll=np.pi/2, pitch=0, yaw=0)
        )
        agent.add_action(action)

        # Pick using link name
        action = PickAction(
            target_object_id=pallet.body_id,
            attach_link="sensor_mast",  # Attach to named link
            attach_relative_pose=Pose.from_euler(0, 0, 0.5)
        )
        agent.add_action(action)

        # Pick nearest object at location
        action = PickAction(
            target_position=[5, 0, 0.1],
            search_radius=0.5
        )
        agent.add_action(action)
    """

    # Target specification (one of these must be provided)
    target_object_id: Optional[int] = None
    target_position: Optional[List[float]] = None
    search_radius: float = 0.5

    # Approach configuration
    use_approach: bool = True  # Whether to use approach phase
    approach_pose: Optional[Pose] = None  # Explicit approach pose (if None, auto-calculated)
    approach_offset: float = 1.0  # Distance from target for approach pose (used if approach_pose is None)
    pick_offset: float = 0.0  # Distance from target where pick is executed (0 = at target position)

    # Attachment configuration
    attach_link: Union[int, str] = -1  # Link index or name to attach to (-1 or "base_link" for base link)
    attach_relative_pose: Pose = field(
        default_factory=lambda: Pose(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
    )  # Relative pose of attached object in parent frame

    # Internal state
    _phase: PickPhase = field(default=PickPhase.INIT, init=False)
    _attach_link_index: int = field(default=-1, init=False)  # Resolved link index
    _target_object: Optional["SimObject"] = field(default=None, init=False)
    _approach_action: Optional[MoveAction] = field(default=None, init=False)
    _forward_action: Optional[MoveAction] = field(default=None, init=False)
    _retreat_action: Optional[MoveAction] = field(default=None, init=False)
    _original_pose: Optional[Pose] = field(default=None, init=False)
    _pick_pose: Optional[Pose] = field(default=None, init=False)  # Pose where pick is executed

    def __post_init__(self):
        super().__init__()
        # Resolve link name to index will be done during execution when agent is available
        self._attach_link_index = -1 if isinstance(self.attach_link, str) else self.attach_link

    def execute(self, agent, dt: float) -> bool:
        """Execute pick action."""
        if self.status == ActionStatus.NOT_STARTED:
            self.status = ActionStatus.IN_PROGRESS
            self.start_time = agent.sim_core.sim_time if agent.sim_core else 0.0
            self._phase = PickPhase.INIT

            # Resolve link name to index using tools function
            self._attach_link_index = tools.resolve_link_index(agent.body_id, self.attach_link)
            if isinstance(self.attach_link, str):
                logger.debug(f"Resolved link '{self.attach_link}' to index {self._attach_link_index}")

        # Phase 1: Initialize - find target object and calculate poses
        if self._phase == PickPhase.INIT:
            if not self._find_target_object(agent):
                self.status = ActionStatus.FAILED
                self.error_message = "Target object not found or not pickable"
                self.end_time = agent.sim_core.sim_time if agent.sim_core else 0.0
                logger.info(f" Failed: {self.error_message}")
                return True

            # Save original pose for retreat
            self._original_pose = agent.get_pose()

            # Calculate pick pose (where pick is executed)
            self._pick_pose = self._calculate_pick_pose(agent)

            obj_pos = self._target_object.get_pose().position
            pick_pos = self._pick_pose.position
            logger.info(" Object at %s, pick pose at %s", obj_pos[:2], pick_pos[:2])

            if self.use_approach:
                # Calculate or use approach pose
                if self.approach_pose is None:
                    self.approach_pose = self._calculate_approach_pose(agent)

                approach_pos = self.approach_pose.position
                logger.info(f"Approach pose at {approach_pos[:2]} (offset={self.approach_offset:.3f}m)")

                # Create move action for approach (move forward)
                approach_path = Path([self.approach_pose])
                self._approach_action = MoveAction(path=approach_path, auto_approach=False, final_orientation_align=True)
                self._phase = PickPhase.APPROACHING
                logger.info(f"Target object {self._target_object.body_id} found, approaching...")
            else:
                # Skip approach, go directly to moving to pick pose
                self._phase = PickPhase.MOVING_TO_PICK
                logger.info(f" Target object {self._target_object.body_id} found, moving to pick position...")

        # Phase 2: Approach target (move forward to approach_pose)
        if self._phase == PickPhase.APPROACHING:
            if self._approach_action.execute(agent, dt):
                self._phase = PickPhase.MOVING_TO_PICK
                logger.info(" Reached approach pose, moving to pick position...")

        # Phase 3: Move forward to pick pose
        if self._phase == PickPhase.MOVING_TO_PICK:
            # Create forward action once to move from current position to pick pose
            if self._forward_action is None:
                forward_path = Path([self._pick_pose])

                self._forward_action = MoveAction(
                    path=forward_path, auto_approach=False, final_orientation_align=False, direction=MovementDirection.FORWARD
                )
                current_pos = agent.get_pose().position
                pick_pos = self._pick_pose.position
                distance = np.linalg.norm(np.array(pick_pos[:2]) - np.array(current_pos[:2]))
                logger.info("Moving forward %.3fm to pick pose...", distance)

            if self._forward_action.execute(agent, dt):
                self._phase = PickPhase.PICKING
                logger.info(" Reached pick position, picking...")

        # Phase 4: Pick object
        if self._phase == PickPhase.PICKING:
            # Get attachment position in world coordinates
            if self._attach_link_index == -1:
                parent_pos, parent_orn = p.getBasePositionAndOrientation(agent.body_id)
            else:
                link_state = p.getLinkState(agent.body_id, self._attach_link_index)
                parent_pos, parent_orn = link_state[0], link_state[1]

            # Calculate world position for attached object
            attach_world_pos, attach_world_orn = p.multiplyTransforms(
                parent_pos, parent_orn, self.attach_relative_pose.position, self.attach_relative_pose.orientation
            )

            # Teleport object to attachment position
            self._target_object.set_pose(Pose.from_pybullet(attach_world_pos, attach_world_orn))

            # Attach object using relative_pose
            success = agent.attach_object(
                self._target_object, parent_link_index=self._attach_link_index, relative_pose=self.attach_relative_pose
            )

            if success:
                if self.use_approach:
                    # Create retreat action (move backward to approach pose)
                    current_pos = agent.get_pose().position
                    approach_pos = self.approach_pose.position
                    retreat_distance = np.linalg.norm(np.array(approach_pos[:2]) - np.array(current_pos[:2]))
                    logger.info(
                        f"Successfully picked object {self._target_object.body_id}, retreating {retreat_distance:.3f}m..."
                    )

                    retreat_path = Path([self.approach_pose])
                    self._retreat_action = MoveAction(
                        path=retreat_path,
                        auto_approach=False,
                        final_orientation_align=False,  # Keep current orientation
                        direction=MovementDirection.BACKWARD,  # Move backward
                    )
                    self._phase = PickPhase.RETREATING
                else:
                    # No approach, complete immediately
                    self.status = ActionStatus.COMPLETED
                    self.end_time = agent.sim_core.sim_time if agent.sim_core else 0.0
                    logger.info(f"Successfully picked object {self._target_object.body_id}")
                    return True
            else:
                self.status = ActionStatus.FAILED
                self.error_message = "Failed to attach object"
                self.end_time = agent.sim_core.sim_time if agent.sim_core else 0.0
                logger.error(f"Failed: {self.error_message}")
                return True

        # Phase 5: Retreat to approach pose (move backward)
        if self._phase == PickPhase.RETREATING:
            if self._retreat_action.execute(agent, dt):
                self.status = ActionStatus.COMPLETED
                self.end_time = agent.sim_core.sim_time if agent.sim_core else 0.0
                logger.info(" Returned to approach pose")
                return True

        return False

    def _find_target_object(self, agent) -> bool:
        """Find target object based on ID or position."""
        if self.target_object_id is not None:
            # Find by ID
            if agent.sim_core is None:
                return False

            for obj in agent.sim_core.sim_objects:
                if obj.body_id == self.target_object_id and obj.pickable:
                    self._target_object = obj
                    return True
            return False

        elif self.target_position is not None:
            # Find nearest pickable object within search radius
            if agent.sim_core is None:
                return False

            target_pos = np.array(self.target_position)
            min_distance = self.search_radius
            nearest_obj = None

            for obj in agent.sim_core.sim_objects:
                if not obj.pickable or obj == agent:
                    continue

                obj_pose = obj.get_pose()
                obj_pos = np.array(obj_pose.position)
                distance = np.linalg.norm(obj_pos - target_pos)

                if distance < min_distance:
                    min_distance = distance
                    nearest_obj = obj

            if nearest_obj is not None:
                self._target_object = nearest_obj
                return True
            return False

        else:
            return False

    def _calculate_pick_pose(self, agent) -> Pose:
        """Calculate pose where pick is executed (pick_offset away from target)."""
        obj_pose = self._target_object.get_pose()
        return tools.calculate_offset_pose(
            target_position=obj_pose.position,
            current_position=agent.get_pose().position,
            offset=self.pick_offset,
            keep_height=True,
        )

    def _calculate_approach_pose(self, agent) -> Pose:
        """Calculate approach pose based on target object position."""
        obj_pose = self._target_object.get_pose()
        return tools.calculate_approach_pose(
            target_position=obj_pose.position,
            current_position=agent.get_pose().position,
            approach_offset=self.approach_offset,
            keep_height=True,
        )

    def reset(self):
        """Reset action state."""
        super().reset()
        self._phase = PickPhase.INIT
        self._target_object = None
        self._approach_action = None
        self._forward_action = None
        self._retreat_action = None
        self._original_pose = None
        self._pick_pose = None


@dataclass
class DropAction(Action):
    """
    Drop an attached object at a specified location.

    Kinematic implementation:
    1. Move to approach pose (if specified or auto-calculated)
    2. Detach object
    3. Teleport object to drop position

    Args:
        drop_position: Where to drop the object [x, y, z] (required)
        drop_orientation: Orientation as quaternion [x, y, z, w] (optional)
        approach_pose: Explicit approach pose (optional, auto-calculated if None)
        approach_offset: Auto-calculate approach offset distance (default: 0.2m)
        place_gently: Place at exact position vs drop from height (default: True)
        drop_height: Height above drop_position to release if not gentle (default: 0.1m)
        target_object_id: Specific object to drop (None = drop first attached)
        path_visualize: Whether to visualize approach/drop paths (default: None = use agent's setting)
        path_color: RGB color for path visualization (default: None = yellow for drop)
        path_width: Line width for path visualization (default: None = use agent's setting)

    Example:
        # Drop at specific location
        action = DropAction(
            drop_position=[10, 5, 0.1],
            place_gently=True
        )
        agent.add_action(action)

    Note:
        Path visualization is controlled by agent.path_visualize setting.
    """

    # Drop location (required)
    drop_position: List[float]
    drop_orientation: Optional[List[float]] = None

    # Approach configuration
    use_approach: bool = True  # Whether to use approach phase
    approach_pose: Optional[Pose] = None  # Explicit approach pose (if None, auto-calculated)
    approach_offset: float = 1.0  # Distance from target for approach pose (used if approach_pose is None)
    drop_offset: float = 0.0  # Distance from drop_position where drop is executed (0 = at drop_position)

    # Drop behavior
    place_gently: bool = True
    drop_height: float = 0.1

    # Target object (None = drop first attached object)
    target_object_id: Optional[int] = None

    # Internal state
    _phase: DropPhase = field(default=DropPhase.INIT, init=False)
    _target_object: Optional["SimObject"] = field(default=None, init=False)
    _approach_action: Optional[MoveAction] = field(default=None, init=False)
    _forward_action: Optional[MoveAction] = field(default=None, init=False)
    _retreat_action: Optional[MoveAction] = field(default=None, init=False)
    _original_pose: Optional[Pose] = field(default=None, init=False)
    _drop_pose: Optional[Pose] = field(default=None, init=False)  # Pose where drop is executed

    def __post_init__(self):
        super().__init__()

    def execute(self, agent, dt: float) -> bool:
        """Execute drop action."""
        if self.status == ActionStatus.NOT_STARTED:
            self.status = ActionStatus.IN_PROGRESS
            self.start_time = agent.sim_core.sim_time if agent.sim_core else 0.0
            self._phase = DropPhase.INIT

        # Phase 1: Initialize - find target object to drop and calculate poses
        if self._phase == DropPhase.INIT:
            if not self._find_attached_object(agent):
                self.status = ActionStatus.FAILED
                self.error_message = "No attached object found to drop"
                self.end_time = agent.sim_core.sim_time if agent.sim_core else 0.0
                logger.info(f" Failed: {self.error_message}")
                return True

            # Save original pose for retreat
            self._original_pose = agent.get_pose()

            # Calculate drop pose (where drop is executed)
            self._drop_pose = self._calculate_drop_pose(agent)

            target_pos = self.drop_position
            drop_pos = self._drop_pose.position
            logger.info(f" Target at {target_pos[:2]}, drop pose at {drop_pos[:2]}")

            if self.use_approach:
                # Calculate or use approach pose
                if self.approach_pose is None:
                    self.approach_pose = self._calculate_approach_pose(agent)

                approach_pos = self.approach_pose.position
                logger.info(f"Approach pose at {approach_pos[:2]} (offset={self.approach_offset:.3f}m)")

                # Create move action for approach (move forward)
                approach_path = Path([self.approach_pose])
                self._approach_action = MoveAction(path=approach_path, auto_approach=False, final_orientation_align=True)
                self._phase = DropPhase.APPROACHING
                logger.info("Approaching drop position...")
            else:
                # Skip approach, go directly to moving to drop pose
                self._phase = DropPhase.MOVING_TO_DROP
                logger.info("Moving to drop position...")

        # Phase 2: Approach drop position (move forward to approach_pose)
        if self._phase == DropPhase.APPROACHING:
            if self._approach_action.execute(agent, dt):
                self._phase = DropPhase.MOVING_TO_DROP
                logger.info(" Reached approach pose, moving to drop position...")

        # Phase 3: Move forward to drop pose
        if self._phase == DropPhase.MOVING_TO_DROP:
            # Create forward action once to move from current position to drop pose
            if self._forward_action is None:
                forward_path = Path([self._drop_pose])
                self._forward_action = MoveAction(
                    path=forward_path, auto_approach=False, final_orientation_align=False, direction=MovementDirection.FORWARD
                )
                current_pos = agent.get_pose().position
                drop_pos = self._drop_pose.position
                distance = np.linalg.norm(np.array(drop_pos[:2]) - np.array(current_pos[:2]))
                logger.info("Moving forward %.3fm to drop pose...", distance)

            if self._forward_action.execute(agent, dt):
                self._phase = DropPhase.DROPPING
                logger.info("Reached drop position, dropping...")

        # Phase 4: Drop object
        if self._phase == DropPhase.DROPPING:
            # Detach object
            success = agent.detach_object(self._target_object)

            if not success:
                self.status = ActionStatus.FAILED
                self.error_message = "Failed to detach object"
                self.end_time = agent.sim_core.sim_time if agent.sim_core else 0.0
                logger.info(f" Failed: {self.error_message}")
                return True

            # Calculate final drop pose
            if self.place_gently:
                final_position = self.drop_position
            else:
                # Drop from height
                final_position = [self.drop_position[0], self.drop_position[1], self.drop_position[2] + self.drop_height]

            # Determine orientation
            if self.drop_orientation is None:
                drop_orientation = [0, 0, 0, 1]  # Default: no rotation
            else:
                drop_orientation = self.drop_orientation

            # Teleport object to drop position
            drop_pose = Pose(position=final_position, orientation=drop_orientation)
            self._target_object.set_pose(drop_pose)

            # If not placing gently, give object a small downward velocity
            if not self.place_gently:
                p.resetBaseVelocity(self._target_object.body_id, linearVelocity=[0, 0, -0.5], angularVelocity=[0, 0, 0])

            if self.use_approach:
                # Create retreat action (move backward to approach pose)
                current_pos = agent.get_pose().position
                approach_pos = self.approach_pose.position
                retreat_distance = np.linalg.norm(np.array(approach_pos[:2]) - np.array(current_pos[:2]))
                logger.info(
                    f" Successfully dropped object {self._target_object.body_id}, retreating {retreat_distance:.3f}m..."
                )

                # Use visualization settings from action or agent defaults

                retreat_path = Path([self.approach_pose])
                self._retreat_action = MoveAction(
                    path=retreat_path,
                    auto_approach=False,
                    final_orientation_align=False,  # Keep current orientation
                    direction=MovementDirection.BACKWARD,  # Move backward
                )
                self._phase = DropPhase.RETREATING
            else:
                # No approach, complete immediately
                self.status = ActionStatus.COMPLETED
                self.end_time = agent.sim_core.sim_time if agent.sim_core else 0.0
                logger.info(f"Successfully dropped object {self._target_object.body_id}")
                return True

        # Phase 5: Retreat to approach pose (move backward)
        if self._phase == DropPhase.RETREATING:
            if self._retreat_action.execute(agent, dt):
                self.status = ActionStatus.COMPLETED
                self.end_time = agent.sim_core.sim_time if agent.sim_core else 0.0
                logger.info("Returned to approach pose")
                return True

        return False

    def _find_attached_object(self, agent) -> bool:
        """Find attached object to drop."""
        attached = agent.get_attached_objects()

        if len(attached) == 0:
            return False

        if self.target_object_id is not None:
            # Find specific object
            for obj in attached:
                if obj.body_id == self.target_object_id:
                    self._target_object = obj
                    return True
            return False
        else:
            # Drop first attached object
            self._target_object = attached[0]
            return True

    def _calculate_drop_pose(self, agent) -> Pose:
        """Calculate pose where drop is executed (drop_offset away from drop_position)."""
        return tools.calculate_offset_pose(
            target_position=self.drop_position,
            current_position=agent.get_pose().position,
            offset=self.drop_offset,
            keep_height=True,
        )

    def _calculate_approach_pose(self, agent) -> Pose:
        """Calculate approach pose based on drop position."""
        return tools.calculate_approach_pose(
            target_position=self.drop_position,
            current_position=agent.get_pose().position,
            approach_offset=self.approach_offset,
            keep_height=True,
        )

    def reset(self):
        """Reset action state."""
        super().reset()
        self._phase = DropPhase.INIT
        self._target_object = None
        self._approach_action = None
        self._forward_action = None
        self._retreat_action = None
        self._original_pose = None
        self._drop_pose = None
