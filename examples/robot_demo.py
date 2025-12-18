#!/usr/bin/env python3
"""
robot_demo.py
Comprehensive demo showing SimObject and Agent classes with various configurations:
- SimObject with mesh (visual)
- SimObject without mesh (simple box)
- Agent with mesh (mobile)
- Agent with URDF (fixed arm)
"""
import os
import sys

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pybullet as p

from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams

# Initialize simulation
params = SimulationParams(gui=True, timestep=0.01)
sim_core = MultiRobotSimulationCore(params)

# 1. SimObject with mesh (pallet visual)
pallet_mesh_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../mesh/11pallet.obj"))
pallet_sim = SimObject.from_mesh(
    visual_shape=ShapeParams(
        shape_type="mesh", mesh_path=pallet_mesh_path, mesh_scale=[0.5, 0.5, 0.5], rgba_color=[0.8, 0.6, 0.4, 1.0]
    ),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.4, 0.1]),
    pose=Pose.from_xyz(-2, 0, 0.1),
    sim_core=sim_core,
)

# 2. SimObject without mesh (simple red box with direct PyBullet API)
box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.3])
box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.3], rgbaColor=[1.0, 0.0, 0.0, 1.0])
box_body = p.createMultiBody(0.0, box_collision, box_visual, [-2, 2, 0.3])
box_sim = SimObject(body_id=box_body, sim_core=sim_core)

# 3. Agent with mesh (mobile robot with cube visual)
cube_mesh_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../mesh/cube.obj"))
cube_agent = Agent.from_mesh(
    visual_shape=ShapeParams(
        shape_type="mesh", mesh_path=cube_mesh_path, mesh_scale=[0.3, 0.3, 0.3], rgba_color=[0.0, 1.0, 0.0, 1.0]
    ),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.3, 0.3, 0.3]),
    pose=Pose.from_xyz(0, 0, 0.5),
    max_linear_vel=1.5,
    sim_core=sim_core,
)

# 4. Agent with URDF (fixed arm robot)
arm_agent = Agent.from_urdf(
    urdf_path=os.path.join(os.path.dirname(__file__), "../robots/arm_robot.urdf"),
    pose=Pose.from_xyz(3, 0, 0),
    use_fixed_base=True,
    sim_core=sim_core,
)

# Setup camera
p.resetDebugVisualizerCamera(10.0, 45, -25, [0.5, 0.5, 0.5])


# Callbacks
def cube_random_movement_callback(sim_core, dt):
    """Update cube with random goals."""
    workspace_bounds = {"x_min": -3, "x_max": 4, "y_min": -2, "y_max": 4, "z": 0.5}
    if sim_core.step_count % 10 == 0:
        random_x = np.random.uniform(workspace_bounds["x_min"], workspace_bounds["x_max"])
        random_y = np.random.uniform(workspace_bounds["y_min"], workspace_bounds["y_max"])
        random_goal = Pose.from_xyz(random_x, random_y, workspace_bounds["z"])
        cube_agent.set_goal_pose(random_goal)


def arm_joint_control_callback(sim_core, dt):
    """Control arm joints with sinusoidal motion."""
    if sim_core.step_count % 10 == 0:
        t = sim_core.sim_time
        joint_targets = [0.5 * np.sin(t + j * 0.5) for j in range(arm_agent.get_num_joints())]
        arm_agent.set_all_joint_targets(joint_targets)


# Register and run
sim_core.register_callback(cube_random_movement_callback, frequency=10)
sim_core.register_callback(arm_joint_control_callback, frequency=10)
sim_core.run_simulation()
