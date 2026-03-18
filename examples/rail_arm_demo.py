#!/usr/bin/env python3
"""
rail_arm_demo.py
Demo: Rail arm robot (prismatic + revolute joints) controlled via JointAction and PoseAction.

Shows:
1. JointAction — raise the rail, move arm joints
2. PoseAction  — IK-based end-effector positioning at different rail heights
3. JointAction — return to home position
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pybullet_fleet.action import JointAction, PoseAction
from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import Pose

# ---------------------------------------------------------------------------
# Simulation setup
# ---------------------------------------------------------------------------

params = SimulationParams(gui=True, timestep=0.1, physics=False, target_rtf=10)
sim_core = MultiRobotSimulationCore(params)

# Spawn rail arm (fixed base)
rail_arm_urdf = os.path.join(os.path.dirname(__file__), "../robots/rail_arm_robot.urdf")
agent = Agent.from_urdf(
    urdf_path=rail_arm_urdf,
    pose=Pose.from_xyz(0, 0, 0),
    use_fixed_base=True,
    sim_core=sim_core,
)


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------


def print_joint_states(agent, label: str) -> None:
    """Print current joint positions with a label."""
    n = agent.get_num_joints()
    positions = [agent.get_joint_state(i)[0] for i in range(n)]
    names = [agent.joint_info[i][1].decode() for i in range(n)]
    print(f"\n--- {label} ---")
    for name, pos in zip(names, positions):
        print(f"  {name}: {pos:.4f}")


# ---------------------------------------------------------------------------
# Action sequence
# ---------------------------------------------------------------------------

print_joint_states(agent, "Initial state")

# Step 1: Raise rail to 0.5 m via JointAction
print("\n=== Step 1: Raise rail to 0.5 m ===")
agent.add_action(
    JointAction(
        target_joint_positions={"rail_joint": 0.5},
        tolerance=0.02,
    )
)

# Step 2: Move arm to a pose via JointAction (all 5 joints)
print("=== Step 2: Move arm joints ===")
agent.add_action(
    JointAction(
        target_joint_positions=[0.5, 0.8, 0.5, -0.3, 0.0],
        tolerance=0.02,
    )
)

# Step 3: PoseAction — IK-based EE positioning (high target)
print("=== Step 3: PoseAction to [0.0, 0.0, 1.2] ===")
agent.add_action(
    PoseAction(
        target_position=[0.0, 0.0, 1.2],
        tolerance=0.03,
    )
)

# Step 4: PoseAction — different height (requires rail adjustment via IK)
print("=== Step 4: PoseAction to [0.2, 0.0, 0.6] ===")
agent.add_action(
    PoseAction(
        target_position=[0.2, 0.0, 0.6],
        tolerance=0.03,
    )
)

# Step 5: Return to home via JointAction
print("=== Step 5: Return home ===")
agent.add_action(
    JointAction(
        target_joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0],
        tolerance=0.02,
    )
)


# ---------------------------------------------------------------------------
# Callbacks for status reporting
# ---------------------------------------------------------------------------

_last_queue_size = -1


def report_progress(sim_core) -> None:
    """Print status when the action queue changes."""
    global _last_queue_size
    current = agent.get_action_queue_size()
    if current != _last_queue_size:
        _last_queue_size = current
        if current == 0:
            print_joint_states(agent, "All actions completed")
        else:
            print(f"  Actions remaining: {current}")


sim_core.register_callback(report_progress, frequency=10)

# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------

print("\nStarting simulation (press Ctrl+C to stop)...")
try:
    sim_core.run_simulation(duration=60.0)
except KeyboardInterrupt:
    pass

print("\nDone.")
