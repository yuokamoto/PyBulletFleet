"""
pybullet_fleet package
General-purpose PyBullet simulation library for multi-robot fleets
"""

# Type definitions
from pybullet_fleet.types import (
    ActionStatus,
    ControllerMode,
    DifferentialPhase,
    MotionMode,
    MovementDirection,
    SpatialHashCellSizeMode,
)

# Geometry primitives
from pybullet_fleet.geometry import Path, Pose

# Robot/Agent management
from pybullet_fleet.agent import Agent, AgentSpawnParams, IKParams
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams

# Core simulation classes
from pybullet_fleet.core_simulation import (
    MultiRobotSimulationCore,
    SimulationParams,
)

# Base classes
from pybullet_fleet.sim_object import SimObject

# Controller system
from pybullet_fleet.controller import (
    Controller,
    KinematicController,
    DifferentialController,
    OmniController,
    create_controller,
    register_controller,
)

# Utilities
from pybullet_fleet.tools import (
    body_to_world_velocity_2d,
    body_to_world_velocity_3d,
    grid_to_world,
    world_to_grid,
    normalize_vector_param,
)

# Config utilities
from pybullet_fleet.config_utils import load_yaml_config

# Entity registry
from pybullet_fleet.entity_registry import register_entity_class

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
    "IKParams",
    "AgentManager",
    "GridSpawnParams",
    # Controller system
    "Controller",
    "ControllerMode",
    "KinematicController",
    "OmniController",
    "DifferentialController",
    "create_controller",
    "register_controller",
    # Utilities
    "body_to_world_velocity_2d",
    "body_to_world_velocity_3d",
    "grid_to_world",
    "world_to_grid",
    "normalize_vector_param",
    # Config utilities
    "load_yaml_config",
    # Entity registry
    "register_entity_class",
]
