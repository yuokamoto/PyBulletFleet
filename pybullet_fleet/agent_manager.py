"""
core/agent_manager.py
Management classes for simulation objects and agents.
- SimObjectManager: Base manager for all simulation objects
- AgentManager: Extended manager with movement control for agents
"""

from dataclasses import dataclass, replace
from typing import Any, Callable, Dict, Generic, List, Optional, Tuple, TypeVar

from .agent import Agent, AgentSpawnParams, Pose
from .sim_object import SimObject, SimObjectSpawnParams

# Type variable for generic SimObjectManager
T = TypeVar("T", bound=SimObject)


@dataclass
class GridSpawnParams:
    """
    Grid parameters for object spawning in 3D space.

    Attributes:
        x_min: Minimum X grid index
        x_max: Maximum X grid index
        y_min: Minimum Y grid index
        y_max: Maximum Y grid index
        spacing: Grid spacing [spacing_x, spacing_y, spacing_z] in meters
        offset: Grid offset [offset_x, offset_y, offset_z] in meters
        z_min: Minimum Z grid index (defaults to 0)
        z_max: Maximum Z grid index (defaults to 0 for 2D grids)

    Note:
        Objects spawn in X → Y → Z order.
        For 2D grids, set z_min=z_max=0 (default).
        For 3D grids, set z_max > z_min to enable vertical layers.
    """

    x_min: int
    x_max: int
    y_min: int
    y_max: int
    spacing: List[float]  # [spacing_x, spacing_y, spacing_z]
    offset: List[float]  # [offset_x, offset_y, offset_z]
    z_min: int = 0
    z_max: int = 0


class SimObjectManager(Generic[T]):
    """
    Base manager class for simulation objects (SimObject and Agent).

    This class provides centralized control of multiple simulation objects.

    Features:
    - Grid-based object spawning
    - Query all object poses
    - Get object count

    Type Parameter:
        T: Type of objects managed (must be SimObject or its subclass)
    """

    def __init__(self, sim_core=None):
        """
        Initialize SimObjectManager.

        Args:
            sim_core: Reference to simulation core (optional)
        """
        self.sim_core = sim_core
        self.objects: List[T] = []
        self.object_ids: Dict[int, T] = {}  # body_id -> object mapping

    def add_object(self, obj: T) -> None:
        """
        Add an object to the manager.

        Args:
            obj: SimObject or Agent instance to add
        """
        self.objects.append(obj)
        self.object_ids[obj.body_id] = obj

    def spawn_objects_grid(
        self, num_objects: int, grid_params: GridSpawnParams, spawn_params: SimObjectSpawnParams
    ) -> List[T]:
        """
        Spawn multiple SimObjects in a grid pattern.

        This is a convenience wrapper around spawn_grid_mixed() for spawning
        a single object type with 100% probability.

        Args:
            num_objects: Number of objects to spawn
            grid_params: GridSpawnParams instance with grid configuration
            spawn_params: SimObjectSpawnParams instance with object parameters

        Returns:
            List of spawned SimObject instances
        """
        # Wrapper: Call spawn_grid_mixed with single spawn_params at probability 1.0
        return self.spawn_grid_mixed(num_objects, grid_params, [(spawn_params, 1.0)])

    def spawn_grid_mixed(
        self, num_objects: int, grid_params: GridSpawnParams, spawn_params_list: List[Tuple[SimObjectSpawnParams, float]]
    ) -> List[T]:
        """
        Spawn multiple SimObjects in a grid pattern with mixed types.

        This method allows spawning different types of objects with specified probabilities.
        If probabilities don't sum to 1.0, some grid positions may remain empty.

        Args:
            num_objects: Maximum number of objects to spawn
            grid_params: GridSpawnParams instance with grid configuration
            spawn_params_list: List of (spawn_params, probability) tuples
                             - spawn_params: SimObjectSpawnParams for this type
                             - probability: Spawn probability (0.0 to 1.0)
                             Note: Probabilities don't need to sum to 1.0

        Returns:
            List of spawned SimObject instances

        Example:
            # Spawn 100 objects: 60% typeA, 30% typeB, 10% empty
            spawn_params_list = [
                (typeA_params, 0.6),
                (typeB_params, 0.3),
            ]
            manager.spawn_grid_mixed(100, grid_params, spawn_params_list)
        """

        return self._spawn_grid_mixed_impl(
            num_objects=num_objects,
            grid_params=grid_params,
            spawn_params_list=spawn_params_list,
            object_class=SimObject,
            manager_name="SimObjectManager",
        )

    def _spawn_grid_mixed_impl(
        self,
        num_objects: int,
        grid_params: GridSpawnParams,
        spawn_params_list: List[Tuple[Any, float]],
        object_class: type,
        manager_name: str,
    ) -> List[Any]:
        """
        Internal implementation for spawn_grid_mixed methods.

        This method contains the common logic for spawning mixed object types in a grid.

        Args:
            num_objects: Maximum number of objects to spawn
            grid_params: GridSpawnParams instance with grid configuration
            spawn_params_list: List of (spawn_params, probability) tuples
            object_class: Class to instantiate (SimObject or Agent)
            manager_name: Name for logging messages

        Returns:
            List of spawned object instances
        """
        import random

        from .tools import grid_to_world

        # Normalize probabilities if total > 1.0
        total_prob = sum(prob for _, prob in spawn_params_list)
        if total_prob > 1.0:
            print(f"[{manager_name}] Warning: Total probability {total_prob:.2f} > 1.0, normalizing to 1.0")
            # Normalize: divide each probability by total
            spawn_params_list = [(params, prob / total_prob) for params, prob in spawn_params_list]
            total_prob = 1.0

        # Extract grid parameters
        x_min = grid_params.x_min
        x_max = grid_params.x_max
        y_min = grid_params.y_min
        y_max = grid_params.y_max
        z_min = grid_params.z_min
        z_max = grid_params.z_max
        spacing = grid_params.spacing
        offset = grid_params.offset

        # Spawn objects sequentially
        spawned_objects = []
        current_x = x_min
        current_y = y_min
        current_z = z_min

        for i in range(num_objects):
            # Calculate grid position
            spawn_grid = [current_x, current_y, current_z]
            spawn_pos = grid_to_world(spawn_grid, spacing, offset)

            # Randomly select spawn params based on probabilities
            rand = random.random()
            cumulative_prob = 0.0
            selected_params = None

            for spawn_params, prob in spawn_params_list:
                cumulative_prob += prob
                if rand < cumulative_prob:
                    selected_params = spawn_params
                    break

            # If selected, spawn object
            if selected_params is not None:
                # Create Pose from spawn position
                if selected_params.initial_pose is not None:
                    # Preserve orientation from spawn_params
                    spawn_pose = Pose(position=spawn_pos, orientation=selected_params.initial_pose.orientation)
                else:
                    spawn_pose = Pose.from_xyz(spawn_pos[0], spawn_pos[1], spawn_pos[2])

                # Create a copy of spawn_params with the calculated pose
                grid_spawn_params = replace(selected_params, initial_pose=spawn_pose)

                # Spawn object using from_params()
                obj = object_class.from_params(spawn_params=grid_spawn_params, sim_core=self.sim_core)

                # Track object
                self.add_object(obj)
                spawned_objects.append(obj)

            # Increment grid position: X → Y → Z
            current_x += 1
            if current_x > x_max:
                current_x = x_min
                current_y += 1
                if current_y > y_max:
                    current_y = y_min
                    current_z += 1
                    if current_z > z_max:
                        # Grid exhausted
                        if i < num_objects - 1:
                            print(
                                f"[{manager_name}] Warning: Grid exhausted after {i + 1} iterations "
                                f"(requested {num_objects})"
                            )
                        break

        return spawned_objects

    def get_pose(self, object_index: int) -> Optional[Pose]:
        """
        Get current pose of a specific object.

        Args:
            object_index: Index of object in self.objects list

        Returns:
            Current Pose or None if invalid index
        """
        if 0 <= object_index < len(self.objects):
            return self.objects[object_index].get_pose()
        else:
            print(f"[SimObjectManager] Warning: Invalid object index {object_index}")
            return None

    def get_all_poses(self) -> List[Pose]:
        """
        Get current poses of all objects.

        Returns:
            List of Pose objects for all objects
        """
        return [obj.get_pose() for obj in self.objects]

    def get_poses_dict(self) -> Dict[int, Pose]:
        """
        Get current poses of all objects as a dictionary.

        Returns:
            Dictionary mapping body_id to Pose
        """
        return {obj.body_id: obj.get_pose() for obj in self.objects}

    def get_object_count(self) -> int:
        """Get total number of managed objects."""
        return len(self.objects)

    def __repr__(self):
        return f"SimObjectManager(total={len(self.objects)})"


class AgentManager(SimObjectManager[Agent]):
    """
    Extended manager class for agents with update callback system.

    This class extends SimObjectManager with agent-specific features:
    - Grid-based agent spawning with AgentSpawnParams
    - Pose goal assignment to individual agents
    - Callback system for custom update logic (goals, state tracking, etc.)
    - Query moving/stopped agents

    Key Features:
    - Auto-registers update callback to simulation loop
    - Callback receives (agents, manager, dt) for easy agent operations
    - Configurable update frequency
    - Direct access to manager methods within callback

    Note:
    - Agent.update() is automatically called by MultiRobotSimulationCore every step
    - AgentManager callbacks are for high-level logic (goals, coordination, etc.)
    - Agent-specific state can be stored in each Agent.user_data dict
    """

    def __init__(self, sim_core=None, update_frequency: float = 10.0):
        """
        Initialize AgentManager.

        Args:
            sim_core: Reference to simulation core (optional)
            update_frequency: Default update callback frequency in Hz (default: 10.0)
        """
        super().__init__(sim_core)
        self._callbacks: List[Dict[str, Any]] = []  # List of registered callbacks
        self._update_frequency: float = update_frequency  # Default callback frequency in Hz

    def spawn_agents_grid(self, num_agents: int, grid_params: GridSpawnParams, spawn_params: AgentSpawnParams) -> List[Agent]:
        """
        Spawn multiple agents in a grid pattern.

        This is a convenience wrapper around spawn_agents_grid_mixed() for spawning
        a single agent type with 100% probability.

        Args:
            num_agents: Number of agents to spawn
            grid_params: GridSpawnParams instance with grid configuration
            spawn_params: AgentSpawnParams instance with agent parameters

        Returns:
            List of spawned Agent instances
        """
        # Wrapper: Call spawn_agents_grid_mixed with single spawn_params at probability 1.0
        return self.spawn_agents_grid_mixed(num_agents, grid_params, [(spawn_params, 1.0)])

    def spawn_agents_grid_mixed(
        self, num_agents: int, grid_params: GridSpawnParams, spawn_params_list: List[Tuple[AgentSpawnParams, float]]
    ) -> List[Agent]:
        """
        Spawn multiple agents in a grid pattern with mixed types.

        This method allows spawning different types of agents with specified probabilities.
        If probabilities don't sum to 1.0, some grid positions may remain empty.

        Args:
            num_agents: Maximum number of agents to spawn
            grid_params: GridSpawnParams instance with grid configuration
            spawn_params_list: List of (spawn_params, probability) tuples
                             - spawn_params: AgentSpawnParams for this type
                             - probability: Spawn probability (0.0 to 1.0)
                             Note: Probabilities don't need to sum to 1.0

        Returns:
            List of spawned Agent instances

        Example:
            # Spawn 100 agents: 60% mobile, 30% arm, 10% empty
            spawn_params_list = [
                (mobile_params, 0.6),
                (arm_params, 0.3),
            ]
            manager.spawn_agents_grid_mixed(100, grid_params, spawn_params_list)
        """

        return self._spawn_grid_mixed_impl(
            num_objects=num_agents,
            grid_params=grid_params,
            spawn_params_list=spawn_params_list,
            object_class=Agent,
            manager_name="AgentManager",
        )

    def set_goal_pose(self, agent_index: int, goal: Pose):
        """
        Set goal pose for a specific agent.

        Args:
            agent_index: Index of agent in self.objects list
            goal: Target Pose
        """
        if 0 <= agent_index < len(self.objects):
            self.objects[agent_index].set_goal_pose(goal)
        else:
            print(f"[AgentManager] Warning: Invalid agent index {agent_index}")

    def set_goal_pose_by_id(self, body_id: int, goal: Pose):
        """
        Set goal pose for an agent by its PyBullet body ID.

        Args:
            body_id: PyBullet body ID
            goal: Target Pose
        """
        if body_id in self.object_ids:
            self.object_ids[body_id].set_goal_pose(goal)
        else:
            print(f"[AgentManager] Warning: Unknown agent body_id {body_id}")

    def stop_all(self):
        """Stop all agents and clear their goals."""
        for agent in self.objects:
            agent.stop()

    def get_moving_count(self) -> int:
        """Get number of agents currently moving."""
        return sum(1 for agent in self.objects if agent.is_moving)

    def register_callback(self, callback: Callable[["AgentManager", float], None], frequency: Optional[float] = None):
        """
        Register a callback for custom agent update logic.

        Multiple callbacks can be registered, each with their own frequency.
        This method automatically registers with sim_core if available.

        The callback provides access to the AgentManager instance, making it easy to:
        - Access all agents via manager.objects
        - Use manager methods (set_goal_pose, query states, etc.)
        - Store manager-level state in agent.user_data

        The callback should have signature:
            callback(manager: AgentManager, dt: float) -> None

        Args:
            callback: Function to call for agent updates
            frequency: Update frequency in Hz. If None, uses the default frequency
                      from __init__ (default: None)

        Example (Goal management):
            def goal_update_logic(manager, dt):
                for agent in manager.objects:
                    if not agent.is_moving:
                        # Set new goal for stopped agents
                        new_goal = calculate_next_goal(agent)
                        agent.set_goal_pose(new_goal)

            manager.register_callback(goal_update_logic, frequency=4.0)

        Example (State tracking):
            def state_tracker(manager, dt):
                for agent in manager.objects:
                    # Track agent statistics
                    if 'total_distance' not in agent.user_data:
                        agent.user_data['total_distance'] = 0.0
                    agent.user_data['total_distance'] += np.linalg.norm(agent.velocity) * dt

            manager.register_callback(state_tracker, frequency=10.0)

        Comparison with sim_core.register_callback():
        - AgentManager callback: Provides manager reference and filtered agent list
        - sim_core callback: Provides sim_core reference for all objects
        - Use AgentManager for agent-specific operations
        - Use sim_core for general simulation-wide operations
        """
        # Use default frequency if not specified
        if frequency is None:
            frequency = self._update_frequency

        # Store callback info
        self._callbacks.append({"func": callback, "frequency": frequency})

        # Register with sim_core (let sim_core handle frequency management)
        if self.sim_core is not None:
            # Create wrapper to match sim_core callback signature
            def _callback_wrapper(core, dt):
                callback(self, dt)

            self.sim_core.register_callback(_callback_wrapper, frequency=frequency)
            print(f"[AgentManager] Registered callback at {frequency} Hz (total: {len(self._callbacks)} callback(s))")
        else:
            print(
                f"[AgentManager] Warning: sim_core not set, callback registered but not active. "
                f"Total: {len(self._callbacks)} callback(s)"
            )

    def setup_camera(self, camera_config: Optional[Dict] = None) -> None:
        """
        Setup camera to view all managed objects in the simulation.

        This method automatically extracts positions from all managed objects
        and configures the camera to fit them in the view.

        Args:
            camera_config: Dictionary with camera settings (from yaml config).
                          If None, uses default settings.

        Example:
            # Basic usage with default settings
            agent_manager.setup_camera()

            # With custom camera config
            camera_config = {
                'camera_mode': 'auto',
                'camera_view_type': 'top_down',
                'camera_auto_scale': 0.8
            }
            agent_manager.setup_camera(camera_config)

        Note:
            Requires sim_core to be set during AgentManager initialization.
            Only works when GUI is enabled.
        """
        if self.sim_core is None:
            print("[AgentManager] Warning: Cannot setup camera - sim_core is not set")
            return

        if camera_config is None:
            camera_config = {}

        # Extract positions from all managed objects
        try:
            poses = self.get_all_poses()
            entity_positions = [pose.position for pose in poses]

            if len(entity_positions) == 0:
                print("[AgentManager] Warning: No objects to setup camera for")
                return

            # Call sim_core's setup_camera with extracted positions
            self.sim_core.setup_camera(camera_config=camera_config, entity_positions=entity_positions)

            print(f"[AgentManager] Camera setup complete for {len(entity_positions)} objects")

        except Exception as e:
            print(f"[AgentManager] Error setting up camera: {e}")

    def __repr__(self):
        return f"AgentManager(total={len(self.objects)}, " f"moving={self.get_moving_count()})"
