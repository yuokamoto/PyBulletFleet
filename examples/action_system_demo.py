"""
action_system_demo.py
Demo of the action queue system with Pick/Drop/Wait actions.

Demonstrates:
1. MoveAction: Navigate to locations
2. WaitAction: Wait for specified duration (e.g., charging)
3. PickAction: Pick objects with auto-approach
4. DropAction: Drop objects at specified locations
5. Action sequencing: Execute multiple actions in order
"""

import os
import logging
import numpy as np
import pybullet as p

from pybullet_fleet.agent import Agent, AgentSpawnParams, MotionMode
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Pose, Path
from pybullet_fleet.sim_object import SimObject, SimObjectSpawnParams, ShapeParams
from pybullet_fleet.action import MoveAction, WaitAction, PickAction, DropAction

# Setup logging
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")


def main():
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

    print("\n" + "=" * 70)
    print("Action System Demo")
    print("=" * 70)
    print("Demonstration of high-level action queue system:")
    print("1. Agent picks up object (with auto-approach)")
    print("2. Agent drops object at target location (with auto-approach)")
    print("3. Agent moves to charging station")
    print("4. Agent waits (simulates charging)")
    print("=" * 70 + "\n")

    # Spawn differential drive agent (blue)
    urdf_path = os.path.join(os.path.dirname(__file__), "../robots/mobile_robot.urdf")
    agent_params = AgentSpawnParams(
        urdf_path=urdf_path,
        initial_pose=Pose.from_xyz(0, 0, 0.3),
        motion_mode=MotionMode.DIFFERENTIAL,
        max_linear_vel=2.0,
        max_linear_accel=5.0,
        max_angular_vel=2.0,
        max_angular_accel=5.0,
        mass=0.0,  # Kinematic control (no physics)
    )
    agent = Agent.from_params(agent_params, sim_core=sim)

    # Enable path visualization for all actions
    agent.path_visualize = False
    agent.path_visualize_width = 3.0

    # Get mesh paths (absolute)
    mesh_dir = os.path.join(os.path.dirname(__file__), "../mesh")
    pallet_mesh_path = os.path.join(mesh_dir, "11pallet.obj")
    cube_mesh_path = os.path.join(mesh_dir, "cube.obj")

    # Spawn pickable object (pallet, rotated 90 degrees for horizontal placement)
    pallet_params = SimObjectSpawnParams(
        visual_shape=ShapeParams(
            shape_type="mesh", mesh_path=pallet_mesh_path, mesh_scale=[0.5, 0.5, 0.5], rgba_color=[0.8, 0.6, 0.4, 1.0]
        ),
        collision_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.4, 0.1]),
        initial_pose=Pose.from_euler(5, 0, 0.1, roll=np.pi / 2, pitch=0, yaw=0),  # Rotated 90° around X-axis
        mass=0.0,  # Kinematic (no physics)
        pickable=True,  # Can be picked up
    )
    pallet = SimObject.from_params(pallet_params, sim_core=sim)

    # Spawn dropoff marker (not pickable)
    dropoff_marker_params = SimObjectSpawnParams(
        visual_shape=ShapeParams(
            shape_type="mesh",
            mesh_path=cube_mesh_path,
            mesh_scale=[0.2, 0.2, 0.01],
            rgba_color=[0.0, 1.0, 0.0, 0.5],  # Green, transparent
        ),
        collision_shape=None,  # No collision
        initial_pose=Pose.from_xyz(5, 5, 0.05),
        mass=0.0,
        pickable=False,  # Cannot be picked
    )
    dropoff_marker = SimObject.from_params(dropoff_marker_params, sim_core=sim)

    # Define warehouse task sequence
    print("Creating task sequence...")

    # Task 1: Pick pallet (with auto-approach from current position)
    # PickAction automatically calculates approach pose from target position
    # Keep pallet's horizontal orientation when attached (90 degrees around X-axis)
    pallet_orientation_quat = p.getQuaternionFromEuler([np.pi / 2, 0, 0])
    task1 = PickAction(
        target_object_id=pallet.body_id,
        use_approach=True,  # Use approach phase
        approach_offset=1.5,  # Approach from 1.5m away
        pick_offset=1.0,  # Pick at 1.0m from object (robot doesn't overlap with object)
        attach_relative_pose=Pose.from_euler(
            0.6, 0, -0.2, roll=np.pi / 2, pitch=0, yaw=0
        ),  # 0.6m front, 0.2m below, 90° around X
    )

    # Task 2: Drop pallet at dropoff location (with auto-approach)
    # DropAction automatically calculates approach pose from drop position
    # Keep pallet's horizontal orientation when dropped
    task2 = DropAction(
        drop_position=[5, 5, 0.1],
        drop_orientation=list(pallet_orientation_quat),  # Keep horizontal orientation
        place_gently=True,
        use_approach=True,  # Use approach phase
        approach_offset=1.5,  # Approach from 1.5m away
        drop_offset=1.0,  # Drop at 1.0m from target position
    )

    # Task 3: Move to charging station
    path_to_charge = Path.from_positions([[2.5, 2.5, 0.3], [0, 0, 0.3]])
    task3 = MoveAction(path=path_to_charge, final_orientation_align=False)

    # Task 4: Wait (simulates charging)
    task4 = WaitAction(
        duration=3.0,
        action_type="charge",
    )

    # Add all tasks to agent
    agent.add_action_sequence([task1, task2, task3, task4])

    print(f"✅ Added {agent.get_action_queue_size()} actions to agent queue\n")

    # Add labels
    p.addUserDebugText(
        "Pickup",
        [5, 0, 1.0],
        textColorRGB=[1, 1, 1],
        textSize=1.5,
        lifeTime=0,
    )
    p.addUserDebugText(
        "Dropoff",
        [5, 5, 1.0],
        textColorRGB=[0, 1, 0],
        textSize=1.5,
        lifeTime=0,
    )
    p.addUserDebugText(
        "Charging",
        [0, 0, 1.0],
        textColorRGB=[1, 1, 0],
        textSize=1.5,
        lifeTime=0,
    )

    # Status tracking
    last_status_time = [0.0]  # Use list to allow modification in callback

    # Callback for status display
    def status_display_callback(sim_core, dt):
        """Display action status periodically."""
        # Print status every 2 seconds
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

    # Register callback and run simulation
    print("Starting simulation...")
    print("Press Ctrl+C to exit\n")

    sim.register_callback(status_display_callback, frequency=None)  # Call every step
    sim.run_simulation()


if __name__ == "__main__":
    main()
