#!/usr/bin/env python3
"""
pick_drop_arm_demo.py
Demo: Robot arm picks and drops a box at the end-effector (link).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pybullet as p
from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose, SimObject, ShapeParams

# Simulation setup
params = SimulationParams(gui=True, timestep=0.01)
sim_core = MultiRobotSimulationCore(params)

# Spawn robot arm (fixed base)
arm_urdf = os.path.join(os.path.dirname(__file__), "../robots/arm_robot.urdf")
arm_agent = Agent.from_urdf(
    urdf_path=arm_urdf,
    pose=Pose.from_xyz(0, 0, 0),
    use_fixed_base=True,
    sim_core=sim_core,
)


# 目標関節角度（例）
pick_joints = [1.5, 1.5, 1.5, 0.0]  # box方向に伸ばす
place_joints = [-1.5, 1.5, 1.5, 0.0]  # 反対側に伸ばす
moving_to_pick = True
# Spawn box to pick/drop
box_sim = SimObject.from_mesh(
    visual_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05], rgba_color=[1, 0, 0, 1]),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
    pose=Pose.from_xyz(0.3, 0, 0.1),
    mass=0.0,
    sim_core=sim_core,
)

# Pick/drop logic
PICK_LINK_INDEX = p.getNumJoints(arm_agent.body_id) - 1  # End-effector link
PICK_INTERVAL = 50  # steps


picked = False
# 0: box_pick_poseへ, 1:pick, 2:box_place_poseへ, 3:drop, 4:初期姿勢
# 5:box_place_poseへ, 6:pick, 7:box_pick_poseへ, 8:drop, 9:初期姿勢
step_state = 0
wait_counter = 0
WAIT_STEPS = int(0.05 / params.timestep)  # 各動作間の待機(0.5秒)
box_pick_pose = Pose.from_xyz(0.3, 0, 0.1)
box_place_pose = Pose.from_xyz(-0.3, 0, 0.1)
joint_init = [0.0, 0.0, 0.0, 0.0]
box_offset = 0.14
offset_pose = Pose.from_xyz(0, 0, box_offset)


def pick_drop_callback(sim_core, dt):
    global picked, step_state, wait_counter
    step = sim_core.step_count

    # ステートマシンで順序制御
    if step_state == 0:
        # 1. box_pick_poseに移動
        arm_agent.set_all_joints_targets(joint_init)
        box_sim.set_pose(box_pick_pose)
        wait_counter += 1
        if wait_counter >= WAIT_STEPS:
            step_state = 1
            wait_counter = 0
            print(f"[STEP {step}] Move to box_pick_pose")
    elif step_state == 1:
        # 2. pick
        arm_agent.set_all_joints_targets(pick_joints)
        wait_counter += 1
        if wait_counter >= WAIT_STEPS:
            arm_agent.attach_object(box_sim, parent_link_index=PICK_LINK_INDEX, relative_pose=offset_pose)
            picked = True
            step_state = 2
            wait_counter = 0
            print(f"[STEP {step}] Picked box (attached)")
    elif step_state == 2:
        # 3. box_place_poseに移動
        arm_agent.set_all_joints_targets(place_joints)
        box_sim.set_pose(box_place_pose)
        wait_counter += 1
        if wait_counter >= WAIT_STEPS:
            step_state = 3
            wait_counter = 0
            print(f"[STEP {step}] Move to box_place_pose")
    elif step_state == 3:
        # 4. drop
        arm_agent.detach_object(box_sim)
        picked = False
        wait_counter += 1
        if wait_counter >= WAIT_STEPS:
            step_state = 4
            wait_counter = 0
            print(f"[STEP {step}] Dropped box (detached)")
    elif step_state == 4:
        # 5. 初期姿勢に移動
        arm_agent.set_all_joints_targets(joint_init)
        wait_counter += 1
        if wait_counter >= WAIT_STEPS:
            step_state = 5
            wait_counter = 0
            print(f"[STEP {step}] Move to joint_init")
    elif step_state == 5:
        # 6. box_place_poseに移動
        arm_agent.set_all_joints_targets(joint_init)
        box_sim.set_pose(box_place_pose)
        wait_counter += 1
        if wait_counter >= WAIT_STEPS:
            step_state = 6
            wait_counter = 0
            print(f"[STEP {step}] Move to box_place_pose (reverse)")
    elif step_state == 6:
        # 7. pick
        arm_agent.set_all_joints_targets(place_joints)
        wait_counter += 1
        if wait_counter >= WAIT_STEPS:
            arm_agent.attach_object(box_sim, parent_link_index=PICK_LINK_INDEX, relative_pose=offset_pose)
            picked = True
            step_state = 7
            wait_counter = 0
            print(f"[STEP {step}] Picked box (attached, reverse)")
    elif step_state == 7:
        # 8. box_pick_poseに移動
        arm_agent.set_all_joints_targets(pick_joints)
        box_sim.set_pose(box_pick_pose)
        wait_counter += 1
        if wait_counter >= WAIT_STEPS:
            step_state = 8
            wait_counter = 0
            print(f"[STEP {step}] Move to box_pick_pose (reverse)")
    elif step_state == 8:
        # 9. drop
        arm_agent.detach_object(box_sim)
        picked = False
        wait_counter += 1
        if wait_counter >= WAIT_STEPS:
            step_state = 9
            wait_counter = 0
            print(f"[STEP {step}] Dropped box (detached, reverse)")
    elif step_state == 9:
        # 10. 初期姿勢に移動
        arm_agent.set_all_joints_targets(joint_init)
        wait_counter += 1
        if wait_counter >= WAIT_STEPS:
            step_state = 0
            wait_counter = 0
            print(f"[STEP {step}] Move to joint_init (loop)")


# Register callback
sim_core.register_callback(pick_drop_callback, frequency=10)

# Camera setup
p.resetDebugVisualizerCamera(2.0, 45, -30, [0, 0, 0.2])

sim_core.run_simulation()
