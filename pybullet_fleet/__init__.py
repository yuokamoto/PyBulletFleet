"""
pybullet_fleet package
General-purpose PyBullet simulation library for multi-robot fleets
"""

# Type definitions
from pybullet_fleet.types import ActionStatus, DifferentialPhase, MotionMode, MovementDirection

# Geometry primitives
from pybullet_fleet.geometry import Path, Pose

# Robot/Agent management
from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams

# Core simulation classes
from pybullet_fleet.core_simulation import LogLevelManager, MultiRobotSimulationCore, SimulationParams

# Base classes
from pybullet_fleet.sim_object import MeshObject, SimObject, URDFObject

# Utilities
from pybullet_fleet.tools import grid_to_world, world_to_grid

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
    "SimObject",
    "MeshObject",
    "URDFObject",
    "LogLevelManager",
    # Agent management
    "Agent",
    "AgentSpawnParams",
    "AgentManager",
    "GridSpawnParams",
    # Utilities
    "grid_to_world",
    "world_to_grid",
]
