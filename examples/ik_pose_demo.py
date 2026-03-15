#!/usr/bin/env python3
"""
ik_pose_demo.py
Demo: Robot arm control using IK (Inverse Kinematics) - PoseAction and move_end_effector.

Demonstrates:
- PoseAction: Move end-effector to target position via IK
- Agent.move_end_effector(): Direct EE position command (fire-and-forget)
- PickAction with ee_target_position: Position-based pick
- DropAction with ee_target_position: Position-based drop
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pybullet as p
from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams
from pybullet_fleet.action import PoseAction, PickAction, DropAction, WaitAction

# Simulation setup (kinematic mode, fast execution)
params = SimulationParams(gui=True, timestep=0.1, physics=False, target_rtf=10, log_level="info")
sim_core = MultiRobotSimulationCore(params)

# Spawn robot arm (fixed base)
arm_urdf = os.path.join(os.path.dirname(__file__), "../robots/arm_robot.urdf")
arm_agent = Agent.from_urdf(
    urdf_path=arm_urdf,
    pose=Pose.from_xyz(0, 0, 0),
    use_fixed_base=True,
    sim_core=sim_core,
)

# Spawn box to pick/drop
box = SimObject.from_mesh(
    visual_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05], rgba_color=[1, 0, 0, 1]),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
    pose=Pose.from_xyz(0.3, 0, 0.1),
    mass=0.0,
    sim_core=sim_core,
)

# End-effector link = last link
EE_LINK = p.getNumJoints(arm_agent.body_id) - 1
HOME_POS = [0.0, 0.0, 0.75]  # EE home position (arm fully upright)
PICK_EE_POS = [0.0, 0.0, 0.75]  # EE position for picking (near box)
DROP_EE_POS = [0.0, 0.0, 0.75]  # EE position for dropping

# ============================================================
# Action sequence: IK-based pick/drop cycle
# ============================================================
# 1. PoseAction  → Move EE to pick position via IK
# 2. PickAction  → Pick box using ee_target_position
# 3. PoseAction  → Move EE to drop position
# 4. DropAction  → Drop box using ee_target_position
# 5. PoseAction  → Return to home
# 6. Repeat
# ============================================================

step_count = 0
cycle = 0
MAX_CYCLES = 3


def demo_callback(sim_core, dt):
    global step_count, cycle
    step_count += 1

    # Queue actions when idle
    if not arm_agent._current_action and not arm_agent._action_queue:
        if cycle >= MAX_CYCLES:
            print(f"\n[DONE] Completed {MAX_CYCLES} pick/drop cycles")
            sim_core.stop()
            return

        cycle += 1
        print(f"\n{'='*50}")
        print(f"Cycle {cycle}/{MAX_CYCLES} — IK-based pick/drop")
        print(f"{'='*50}")

        # Queue the full cycle
        arm_agent.add_action(PoseAction(target_position=PICK_EE_POS, tolerance=0.02))
        arm_agent.add_action(
            PickAction(
                target_object_id=box.body_id,
                use_approach=False,
                ee_target_position=PICK_EE_POS,
                attach_link=EE_LINK,
                attach_relative_pose=Pose.from_xyz(0, 0, 0.14),
            )
        )
        arm_agent.add_action(WaitAction(duration=0.5))
        arm_agent.add_action(PoseAction(target_position=DROP_EE_POS, tolerance=0.02))
        arm_agent.add_action(
            DropAction(
                drop_pose=Pose.from_xyz(-0.3, 0, 0.1),
                use_approach=False,
                ee_target_position=DROP_EE_POS,
            )
        )
        arm_agent.add_action(WaitAction(duration=0.5))
        arm_agent.add_action(PoseAction(target_position=HOME_POS, tolerance=0.02))
        arm_agent.add_action(WaitAction(duration=0.5))


# Register callback and run
sim_core.register_callback(demo_callback, frequency=1.0)

print("IK Pose Demo — PoseAction + move_end_effector()")
print(f"Robot arm: {arm_agent.get_num_joints()} joints")
print(f"EE home position: {HOME_POS}")
print()

# Also demonstrate direct move_end_effector() before starting the action loop
print("Direct move_end_effector() call (fire-and-forget):")
arm_agent.move_end_effector(HOME_POS)
print(f"  Targets set: {len(arm_agent._kinematic_joint_targets)} joints")
print()

sim_core.run_simulation(duration=30.0)
