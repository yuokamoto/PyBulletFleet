"""
pybullet_fleet package
General-purpose PyBullet simulation library for multi-robot fleets
"""

# Type definitions
from pybullet_fleet.types import (
    ActionStatus,
    DifferentialPhase,
    MotionMode,
    MovementDirection,
    SpatialHashCellSizeMode,
)

# Geometry primitives
from pybullet_fleet.geometry import Path, Pose

# Robot/Agent management
from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams

# Core simulation classes
from pybullet_fleet.core_simulation import (
    MultiRobotSimulationCore,
    SimulationParams,
)

# Base classes
from pybullet_fleet.sim_object import SimObject

# Utilities
from pybullet_fleet.tools import grid_to_world, world_to_grid, normalize_vector_param

__all__ = [
    # Type definitions
    "ActionStatus",
    "DifferentialPhase",
    "MotionMode",
    "MovementDirection",
    # Geometry
    "Pose",
    "Path",
    # Core simulation
    "MultiRobotSimulationCore",
    "SimulationParams",
    "SpatialHashCellSizeMode",
    "SimObject",
    # Agent management
    "Agent",
    "AgentSpawnParams",
    "AgentManager",
    "GridSpawnParams",
    # Utilities
    "grid_to_world",
    "world_to_grid",
    "normalize_vector_param",
]
