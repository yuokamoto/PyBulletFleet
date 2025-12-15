"""
Test link name resolution in PickAction.

This example demonstrates using link names instead of indices
when attaching objects.
"""

import numpy as np
import pybullet as p

from pybullet_fleet.action import DropAction, PickAction
from pybullet_fleet.agent import Agent, AgentSpawnParams, MotionMode
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, SimObjectSpawnParams

print("=" * 70)
print("Link Name Test")
print("=" * 70)
print("Testing PickAction with link name instead of index:")
print("1. Pick object and attach to 'sensor_mast' link (by name)")
print("2. Drop object")
print("=" * 70)
print()

# Initialize simulation
params = SimulationParams(gui=True, timestep=0.1, speed=3)
sim = MultiRobotSimulationCore(params)

# Camera setup
p.resetDebugVisualizerCamera(
    cameraDistance=10.0,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[2, 0, 0],
)

# Spawn agent
agent_params = AgentSpawnParams(
    urdf_path="robots/mobile_robot.urdf",
    initial_pose=Pose.from_xyz(0, 0, 0.3),
    rgba_color=[0.2, 0.5, 1.0, 1.0],  # Blue
    motion_mode=MotionMode.DIFFERENTIAL,
    max_linear_vel=2.0,
    max_linear_accel=5.0,
    max_angular_vel=2.0,
    max_angular_accel=5.0,
    mass=0.0,  # Kinematic control
)
agent = Agent.from_params(agent_params, sim_core=sim)

# Spawn pickable object
pallet_params = SimObjectSpawnParams(
    mesh_path="mesh/11pallet.obj",
    initial_pose=Pose.from_euler(3, 0, 0.1, roll=np.pi / 2, pitch=0, yaw=0),
    collision_half_extents=[0.5, 0.4, 0.1],
    mesh_scale=[0.5, 0.5, 0.5],
    rgba_color=[0.8, 0.6, 0.4, 1.0],
    mass=0.0,
    pickable=True,
)
pallet = SimObject.from_params(pallet_params, sim_core=sim)

# Print available links
print("Available links on robot:")
num_joints = p.getNumJoints(agent.body_id)
for i in range(num_joints):
    joint_info = p.getJointInfo(agent.body_id, i)
    link_name = joint_info[12].decode("utf-8")
    print(f"  [{i}] {link_name}")
print()

# Get pallet orientation quaternion
pallet_orientation_quat = p.getQuaternionFromEuler([np.pi / 2, 0, 0])

# Create actions using link NAME instead of INDEX
print("Creating PickAction with attach_link='sensor_mast'...")
task1 = PickAction(
    target_object_id=pallet.body_id,
    approach_offset=1.0,
    pick_offset=0.3,
    attach_link="sensor_mast",  # Use link name!
    attach_relative_pose=Pose(position=[0.6, 0, -0.2], orientation=pallet_orientation_quat),
)

print("Creating DropAction...")
task2 = DropAction(drop_position=[10, 5, 0.1], place_gently=True)

# Add actions
agent.add_action(task1)
agent.add_action(task2)

print(f"\n✅ Added {len(agent._action_queue)} actions to agent queue\n")

# Run simulation
print("Starting simulation...")
print("Press Ctrl+C to exit\n")

# Status tracking
last_status_time = [0.0]


def status_display_callback(sim_objects, sim_core, dt):
    """Display action status periodically."""
    if sim_core.sim_time - last_status_time[0] >= 2.0:
        current_action = agent.get_current_action()
        queue_size = agent.get_action_queue_size()

        if current_action:
            action_name = current_action.__class__.__name__
            status = current_action.status.value
            print(f"[t={sim_core.sim_time:.1f}s] Current: {action_name} ({status}), Queue: {queue_size} actions")
        else:
            if queue_size > 0:
                print(f"[t={sim_core.sim_time:.1f}s] Idle, Queue: {queue_size} actions waiting")
            else:
                print(f"[t={sim_core.sim_time:.1f}s] All tasks completed!")

        last_status_time[0] = sim_core.sim_time


try:
    sim.register_callback(status_display_callback, frequency=None)
    sim.run_simulation()

except KeyboardInterrupt:
    print("\n\nSimulation interrupted by user")

finally:
    p.disconnect()
    print("Simulation ended")
