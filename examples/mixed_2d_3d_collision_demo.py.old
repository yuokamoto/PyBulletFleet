#!/usr/bin/env python3
"""
mixed_2d_3d_collision_demo.py
Demonstrates per-object collision_check_2d configuration.

This example shows how to optimize collision detection for mixed scenarios:
- 2D agents (AGV, mobile robots): collision_check_2d=True (9 neighbors)
- 3D agents (drones, arms): collision_check_2d=False (27 neighbors)
- 3D objects (lifted pallets): collision_check_2d=False (27 neighbors)

Performance benefit:
- 2D agents: ~67% faster collision checks (9 vs 27 neighbors)
- 3D agents/objects: Full 3D collision detection maintained
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from pybullet_fleet.agent import Agent, AgentSpawnParams, MotionMode
from pybullet_fleet.sim_object import SimObject, SimObjectSpawnParams, ShapeParams
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Pose

print("=" * 70)
print("Mixed 2D/3D Collision Detection Demo")
print("=" * 70)

# Simulation parameters with global default = False (3D)
params = SimulationParams(
    timestep=0.01,
    duration=10.0,
    gui=True,
    collision_check_2d=False,  # Global default: 3D collision (27 neighbors)
    speed=1.0,
    enable_profiling=True,  # Enable to see performance difference
    log_level="INFO",
)

sim_core = MultiRobotSimulationCore(params=params)
agent_manager = AgentManager(sim_core)

# ========================================
# Spawn 2D Agents (AGVs)
# ========================================

mobile_urdf = os.path.join(os.path.dirname(__file__), "..", "robots/mobile_robot.urdf")

# AGV agents: Use 2D collision optimization
agv_params = AgentSpawnParams(
    urdf_path=mobile_urdf,
    initial_pose=Pose.from_xyz(0, 0, 0.3),
    use_fixed_base=False,
    max_linear_vel=2.0,
    max_linear_accel=5.0,
    motion_mode=MotionMode.OMNIDIRECTIONAL,
    collision_check_2d=True,  # 2D optimization (9 neighbors)
    user_data={"type": "agv", "description": "2D mobile robot"},
)

print("\n✓ Spawning 2D AGVs (collision_check_2d=True, 9 neighbors)...")
agv_grid = GridSpawnParams(
    x_min=0, x_max=4,
    y_min=0, y_max=4,
    z_min=0, z_max=0,
    spacing=[2.0, 2.0, 0.0],
    offset=[0.0, 0.0, 0.0],
)
agvs = agent_manager.spawn_agents_grid(
    num_agents=25,
    grid_params=agv_grid,
    spawn_params=agv_params
)
print(f"  Spawned {len(agvs)} AGVs with 2D collision detection")

# ========================================
# Spawn 3D Agents (Drones - simulated with fixed-base agents at different heights)
# ========================================

# Drone agents: Use 3D collision (default)
drone_params = AgentSpawnParams(
    urdf_path=mobile_urdf,  # Using mobile robot URDF for demo
    initial_pose=Pose.from_xyz(0, 0, 2.0),  # Elevated
    use_fixed_base=False,
    max_linear_vel=1.0,
    max_linear_accel=2.0,
    collision_check_2d=False,  # 3D collision (27 neighbors) - explicit
    user_data={"type": "drone", "description": "3D flying robot"},
)

print("\n✓ Spawning 3D Drones (collision_check_2d=False, 27 neighbors)...")
drone_grid = GridSpawnParams(
    x_min=0, x_max=2,
    y_min=0, y_max=2,
    z_min=0, z_max=0,
    spacing=[4.0, 4.0, 0.0],
    offset=[1.0, 1.0, 2.0],  # Elevated
)
drones = agent_manager.spawn_agents_grid(
    num_agents=9,
    grid_params=drone_grid,
    spawn_params=drone_params
)
print(f"  Spawned {len(drones)} drones with 3D collision detection")

# ========================================
# Spawn 3D Objects (Pallets that can be lifted)
# ========================================

# Pallet objects: Use 3D collision (can be lifted)
pallet_params = SimObjectSpawnParams(
    visual_shape=ShapeParams(
        shape_type="box",
        half_extents=[0.6, 0.4, 0.1],
        rgba_color=[0.8, 0.6, 0.4, 1.0],
    ),
    collision_shape=ShapeParams(
        shape_type="box",
        half_extents=[0.6, 0.4, 0.1],
    ),
    initial_pose=Pose.from_xyz(0, 0, 0.1),
    mass=5.0,  # Dynamic object
    pickable=True,
    collision_check_2d=False,  # 3D collision (can be lifted)
)

print("\n✓ Spawning 3D Pallets (collision_check_2d=False, 27 neighbors)...")
pallets = []
for i in range(5):
    x = 2.0 + i * 1.5
    y = 8.0
    pallet_params.initial_pose = Pose.from_xyz(x, y, 0.1)
    pallet = SimObject.from_params(pallet_params, sim_core)
    pallets.append(pallet)
print(f"  Spawned {len(pallets)} pallets with 3D collision detection")

# ========================================
# Spawn 2D Static Objects (Floor tiles - never move vertically)
# ========================================

tile_params = SimObjectSpawnParams(
    visual_shape=ShapeParams(
        shape_type="box",
        half_extents=[0.5, 0.5, 0.05],
        rgba_color=[0.3, 0.3, 0.3, 1.0],
    ),
    collision_shape=ShapeParams(
        shape_type="box",
        half_extents=[0.5, 0.5, 0.05],
    ),
    initial_pose=Pose.from_xyz(0, 0, 0.05),
    mass=0.0,  # Static
    pickable=False,
    collision_check_2d=True,  # 2D optimization (fixed Z-position)
)

print("\n✓ Spawning 2D Floor Tiles (collision_check_2d=True, 9 neighbors)...")
tiles = []
for i in range(3):
    x = 6.0 + i * 1.5
    y = 6.0
    tile_params.initial_pose = Pose.from_xyz(x, y, 0.05)
    tile = SimObject.from_params(tile_params, sim_core)
    tiles.append(tile)
print(f"  Spawned {len(tiles)} tiles with 2D collision detection")

# ========================================
# Setup Camera
# ========================================

agent_manager.setup_camera({
    "distance": 20.0,
    "yaw": 45,
    "pitch": -45,
    "target": [5, 5, 0],
})

# ========================================
# Movement Callback
# ========================================

def random_movement_callback(sim_core, dt):
    """Random movement for different agent types."""
    import random
    
    for obj in sim_core.sim_objects:
        if not isinstance(obj, Agent):
            continue
        
        agent_type = obj.user_data.get("type", "unknown")
        
        if random.random() < 0.02:  # 2% chance to update goal
            if agent_type == "agv":
                # AGVs move on ground (Z=0.3)
                new_goal = Pose.from_xyz(
                    random.uniform(0, 10),
                    random.uniform(0, 10),
                    0.3
                )
            elif agent_type == "drone":
                # Drones move in 3D space (Z=1.0 to 3.0)
                new_goal = Pose.from_xyz(
                    random.uniform(0, 10),
                    random.uniform(0, 10),
                    random.uniform(1.0, 3.0)
                )
            else:
                continue
            
            agent.set_goal_pose(new_goal)

sim_core.register_callback(random_movement_callback)

# ========================================
# Run Simulation
# ========================================

print("\n" + "=" * 70)
print("Collision Detection Configuration Summary:")
print("=" * 70)
print(f"  Global default: collision_check_2d = {params.collision_check_2d} (3D)")
print(f"  AGVs (25):      collision_check_2d = True  (2D, 9 neighbors)")
print(f"  Drones (9):     collision_check_2d = False (3D, 27 neighbors)")
print(f"  Pallets (5):    collision_check_2d = False (3D, 27 neighbors)")
print(f"  Tiles (3):      collision_check_2d = True  (2D, 9 neighbors)")
print("=" * 70)
print("\nRunning simulation for 10 seconds...")
print("Watch the profiling output to see collision check performance!")
print("=" * 70)

sim_core.run_simulation()

print("\n" + "=" * 70)
print("Simulation complete!")
print("=" * 70)
print("\nKey Takeaways:")
print("  1. AGVs and floor tiles use 2D collision (9 neighbors) - faster")
print("  2. Drones and pallets use 3D collision (27 neighbors) - accurate")
print("  3. Each object can override the global collision_check_2d setting")
print("  4. Mixed 2D/3D scenarios get optimized automatically per-object")
print("=" * 70)
