"""
pybullet_fleet package
General-purpose PyBullet simulation library for multi-robot fleets
"""

# Type definitions
from pybullet_fleet.types import (
    ActionStatus,
    ControllerMode,
    MotionMode,
    MovementDirection,
    PosePhase,
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

# Robot model resolution
from pybullet_fleet.robot_models import (
    ModelEntry,
    RobotProfile,
    add_search_path,
    auto_detect_profile,
    detect_robot_type,
    discover_models,
    get_search_paths,
    list_all_models,
    register_model,
    remove_search_path,
    resolve_urdf,
    unregister_model,
)

__all__ = [
    # Type definitions
    "ActionStatus",
    "MotionMode",
    "MovementDirection",
    "PosePhase",
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
    # Robot models
    "resolve_urdf",
    "list_all_models",
    "ModelEntry",
    "RobotProfile",
    "auto_detect_profile",
    "detect_robot_type",
    "register_model",
    "unregister_model",
    "discover_models",
    "add_search_path",
    "remove_search_path",
    "get_search_paths",
]
