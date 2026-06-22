"""Package-level constants for pybullet_fleet_ros.

Centralises simulation_interfaces feature declarations so they are
easy to find and update when new capabilities are added.
"""

from simulation_interfaces.msg import SimulatorFeatures

# Features reported by GetSimulatorFeatures service
SUPPORTED_FEATURES = [
    SimulatorFeatures.SPAWNING,
    SimulatorFeatures.DELETING,
    SimulatorFeatures.ENTITY_STATE_GETTING,
    SimulatorFeatures.ENTITY_STATE_SETTING,
    SimulatorFeatures.ENTITY_INFO_GETTING,
    SimulatorFeatures.ENTITY_BOUNDS,
    SimulatorFeatures.SPAWNABLES,
    SimulatorFeatures.SIMULATION_STATE_GETTING,
    SimulatorFeatures.SIMULATION_STATE_SETTING,
    SimulatorFeatures.SIMULATION_STATE_PAUSE,
    SimulatorFeatures.STEP_SIMULATION_SINGLE,
    SimulatorFeatures.STEP_SIMULATION_MULTIPLE,
    SimulatorFeatures.STEP_SIMULATION_ACTION,
    SimulatorFeatures.SIMULATION_RESET,
]

# Supported spawn resource formats
SUPPORTED_SPAWN_FORMATS = ["urdf"]

# Default URDF when SpawnEntity request has no URI
DEFAULT_SPAWN_URDF = "robots/mobile_robot.urdf"

# Fallback URDF when requested file does not exist on disk (mesh-less body)
FALLBACK_SPAWN_URDF = "robots/simple_cube.urdf"
