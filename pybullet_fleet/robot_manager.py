"""
core/robot_manager.py
Agent management class for controlling multiple agents (robots, people, conveyors, etc.).
Designed to be eventually migrated to a ROS2 node.
"""
import numpy as np
from dataclasses import dataclass, field, replace
from typing import List, Dict, Optional, Tuple, Callable, Any
from .robot import Agent, Pose, AgentSpawnParams


@dataclass
class GridSpawnParams:
    """
    Grid parameters for robot spawning in 3D space.
    
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
        Robots spawn in X → Y → Z order.
        For 2D grids, set z_min=z_max=0 (default).
        For 3D grids, set z_max > z_min to enable vertical layers.
    """
    x_min: int
    x_max: int
    y_min: int
    y_max: int
    spacing: List[float]  # [spacing_x, spacing_y, spacing_z]
    offset: List[float]   # [offset_x, offset_y, offset_z]
    z_min: int = 0
    z_max: int = 0


class AgentManager:
    """
    Manager class for multiple agents (robots, people, conveyors, etc.).
    
    This class provides centralized control of multiple agents and is designed
    to eventually become a ROS2 node that:
    - Publishes agent poses (geometry_msgs/Pose)
    - Subscribes to goal poses (geometry_msgs/PoseStamped)
    
    Features:
    - Grid-based agent spawning
    - Pose goal assignment to individual agents
    - Query all agent poses
    - Update all agents in a single call
    - Callback system for custom goal update logic
    
    Note: Robot-specific state is stored in each Agent.user_data dict
    """
    
    def __init__(self, sim_core=None):
        """
        Initialize AgentManager.
        
        Args:
            sim_core: Reference to simulation core (optional)
        """
        self.sim_core = sim_core
        self.robots: List[Agent] = []
        self.robot_ids: Dict[int, Agent] = {}  # body_id -> robot mapping
        self._goal_update_callback: Optional[Callable] = None  # User-defined goal logic
    
    def add_robot(self, robot: Agent) -> None:
        """
        Add a robot to the manager.
        
        This method properly registers a robot in both the list and dict,
        preventing inconsistencies from manual additions.
        
        Args:
            robot: Agent instance to add
        """
        self.robots.append(robot)
        self.robot_ids[robot.body_id] = robot
    
    def spawn_robots_grid(self, num_robots: int,
                         grid_params: GridSpawnParams, 
                         spawn_params: AgentSpawnParams) -> List[Agent]:
        """
        Spawn multiple robots in a grid pattern.
        
        Args:
            num_robots: Number of robots to spawn
            grid_params: GridSpawnParams instance with grid configuration
            spawn_params: AgentSpawnParams instance with robot parameters
        
        Returns:
            List of spawned Robot instances
        """
        from .tools import grid_to_world
        
        # Extract grid parameters from dataclass
        x_min = grid_params.x_min
        x_max = grid_params.x_max
        y_min = grid_params.y_min
        y_max = grid_params.y_max
        z_min = grid_params.z_min
        z_max = grid_params.z_max
        spacing = grid_params.spacing
        offset = grid_params.offset
        
        # Validate that either mesh_path or urdf_path is provided in spawn_params
        if spawn_params.mesh_path is None and spawn_params.urdf_path is None:
            raise ValueError("Either spawn_params.mesh_path or spawn_params.urdf_path must be provided")
        
        # Spawn robots sequentially
        # Note: Shared shapes are automatically cached on first robot spawn
        spawned_robots = []
        current_x = x_min
        current_y = y_min
        current_z = z_min
        
        print(f"[AgentManager] Spawning {num_robots} robots in grid pattern...")
        print(f"  Grid: X=[{x_min}..{x_max}] Y=[{y_min}..{y_max}] Z=[{z_min}..{z_max}]")
        
        for i in range(num_robots):
            # Calculate grid position
            spawn_grid = [current_x, current_y, current_z]
            spawn_pos = grid_to_world(spawn_grid, spacing, offset)
            
            # Create a copy of spawn_params with the calculated position
            grid_spawn_params = replace(spawn_params, initial_position=spawn_pos)
            
            # Spawn robot using from_params()
            robot = Agent.from_params(
                spawn_params=grid_spawn_params,
                sim_core=self.sim_core
            )
            
            # Track robot
            self.add_robot(robot)
            spawned_robots.append(robot)
            
            # Increment grid position: X → Y → Z
            current_x += 1
            if current_x > x_max:
                current_x = x_min
                current_y += 1
                if current_y > y_max:
                    current_y = y_min
                    current_z += 1
                    if current_z > z_max:
                        # Grid exhausted, stop spawning
                        if i < num_robots - 1:
                            print(f"[AgentManager] Warning: Grid exhausted after {i + 1} robots "
                                  f"(requested {num_robots})")
                        break
            
            # Progress feedback
            if i < 10 or (i + 1) % 20 == 0 or i == num_robots - 1:
                print(f"  Spawned {i + 1}/{num_robots} robots... "
                      f"Last: grid {spawn_grid}, world {spawn_pos}")
        
        print(f"[AgentManager] Successfully spawned {len(spawned_robots)} robots!")
        return spawned_robots
    
    def set_goal_pose(self, robot_index: int, goal: Pose):
        """
        Set goal pose for a specific robot.
        
        Args:
            robot_index: Index of robot in self.robots list
            goal: Target Pose
        """
        if 0 <= robot_index < len(self.robots):
            self.robots[robot_index].set_goal_pose(goal)
        else:
            print(f"[AgentManager] Warning: Invalid robot index {robot_index}")
    
    def set_goal_pose_by_id(self, body_id: int, goal: Pose):
        """
        Set goal pose for a robot by its PyBullet body ID.
        
        Args:
            body_id: PyBullet body ID
            goal: Target Pose
        """
        if body_id in self.robot_ids:
            self.robot_ids[body_id].set_goal_pose(goal)
        else:
            print(f"[AgentManager] Warning: Unknown robot body_id {body_id}")
    
    def get_pose(self, robot_index: int) -> Optional[Pose]:
        """
        Get current pose of a specific robot.
        
        Args:
            robot_index: Index of robot in self.robots list
        
        Returns:
            Current Pose or None if invalid index
        """
        if 0 <= robot_index < len(self.robots):
            return self.robots[robot_index].get_pose()
        else:
            print(f"[AgentManager] Warning: Invalid robot index {robot_index}")
            return None
    
    def get_all_poses(self) -> List[Pose]:
        """
        Get current poses of all robots.
        
        Returns:
            List of Pose objects for all robots
        """
        return [robot.get_pose() for robot in self.robots]
    
    def get_poses_dict(self) -> Dict[int, Pose]:
        """
        Get current poses of all robots as a dictionary.
        
        Returns:
            Dictionary mapping body_id to Pose
        """
        return {robot.body_id: robot.get_pose() for robot in self.robots}
    
    def update_all(self, dt: float):
        """
        Update all robots (move towards goals with velocity constraints).
        
        Args:
            dt: Time step (seconds)
        """
        for robot in self.robots:
            robot.update(dt)
    
    def stop_all(self):
        """Stop all robots and clear their goals."""
        for robot in self.robots:
            robot.stop()
    
    def get_robot_count(self) -> int:
        """Get total number of managed robots."""
        return len(self.robots)
    
    def get_moving_count(self) -> int:
        """Get number of robots currently moving."""
        return sum(1 for robot in self.robots if robot.is_moving)
    
    def register_goal_update_callback(self, callback: Callable[[List[Agent], 'AgentManager', float], None]):
        """
        Register a callback for custom goal update logic.
        
        The callback will be called during update_all_with_goals() and should have signature:
            callback(robots: List[Agent], manager: AgentManager, dt: float) -> None
        
        Example:
            def my_goal_logic(robots, manager, dt):
                for robot in robots:
                    if not robot.is_moving and not robot.static:
                        # Set new goal based on custom logic
                        robot.set_goal_pose(some_goal)
                        # Use robot.user_data for custom state
                        robot.user_data['last_goal_time'] = time.time()
            
            manager.register_goal_update_callback(my_goal_logic)
        
        Args:
            callback: Function to call for goal updates
        """
        self._goal_update_callback = callback
    
    def update_all_with_goals(self, dt: float):
        """
        Update all robots and call goal update callback if registered.
        
        This is a convenience method that:
        1. Calls the goal update callback (if registered)
        2. Updates all robot positions
        
        Args:
            dt: Time step (seconds)
        """
        # Call user-defined goal update logic
        if self._goal_update_callback is not None:
            self._goal_update_callback(self.robots, self, dt)
        
        # Update all robot positions
        self.update_all(dt)
    
    def __repr__(self):
        return (f"AgentManager(total={len(self.robots)}, "
                f"moving={self.get_moving_count()})")

