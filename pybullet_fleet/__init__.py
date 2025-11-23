"""
pybullet_fleet package
General-purpose PyBullet simulation library for multi-robot fleets
"""

# Base classes
from pybullet_fleet.sim_object import SimObject, Pose, MeshObject, URDFObject

# Core simulation classes
from pybullet_fleet.core_simulation import (
    MultiRobotSimulationCore,
    SimulationParams,
    LogLevelManager
)

# Robot/Agent management
from pybullet_fleet.agent import (
    Agent,
    Pose,
    AgentSpawnParams
)

from pybullet_fleet.agent_manager import (
    AgentManager,
    GridSpawnParams
)

# Utilities
from pybullet_fleet.tools import (
    world_to_grid,
    grid_to_world,
    world_to_grid_2d,
    grid_to_world_2d,
    grid_execution,
    grid_spawn,
    grid_spawn_urdf,
    grid_spawn_mesh
)

# Visualization and monitoring
from pybullet_fleet.collision_visualizer import CollisionVisualizer
from pybullet_fleet.data_monitor import DataMonitor

__all__ = [
    # Core simulation
    'MultiRobotSimulationCore',
    'SimulationParams',
    'SimObject',
    'MeshObject',
    'URDFObject',
    'Pose',
    'LogLevelManager',
    
    # Agent management
    'Agent',
    'AgentSpawnParams',
    'AgentManager',
    'GridSpawnParams',
    
    # Utilities
    'grid_to_world',
    'world_to_grid'
]
