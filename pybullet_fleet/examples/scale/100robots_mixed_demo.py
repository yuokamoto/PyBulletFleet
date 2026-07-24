#!/usr/bin/env python3
"""
100robots_mixed_demo.py
Config-driven representative mixed fleet demo.

The scene is loaded from config/100robots_mixed_config.yaml and uses
entities[].grid for both robot groups:

- 50 mobile robots
- 50 fixed-base arm robots

Examples:
  python 100robots_mixed_demo.py
  python 100robots_mixed_demo.py --duration=10
  python 100robots_mixed_demo.py --controller=per_agent --duration=10
"""

import argparse
import os
import sys

import numpy as np
import pybullet as p

# Run from a source checkout without installing: fall back to the repo root
# so `import pybullet_fleet` resolves. Installed/editable users never hit this.
try:
    import pybullet_fleet  # noqa: F401
except ModuleNotFoundError:
    sys.path.insert(
        0,
        os.path.join(os.path.dirname(__file__), "..", "..", ".."),
    )
    import pybullet_fleet  # noqa: F401

from pybullet_fleet.agent import Agent, Pose
from pybullet_fleet.config_utils import load_yaml_config, merge_configs
from pybullet_fleet.core_simulation import MultiRobotSimulationCore

parser = argparse.ArgumentParser(description="100 Robots Mixed Demo")
parser.add_argument(
    "--duration",
    type=float,
    default=None,
    help="Simulation duration in seconds (default: run forever)",
)
parser.add_argument(
    "--controller",
    choices=["per_agent", "batch"],
    default="batch",
    help="Controller implementation for mobile robots (default: batch)",
)
parser.add_argument(
    "--config",
    default="config/100robots_mixed_config.yaml",
    help="Demo config merged over config/config.yaml",
)
args = parser.parse_args()


config = merge_configs(
    load_yaml_config("config/config.yaml"),
    load_yaml_config(args.config),
)
entities = config.get("entities") or []
if not entities:
    raise ValueError("100robots_mixed_demo.py requires a config with entities[].grid")

for entity in entities:
    robot_type = (entity.get("user_data") or {}).get("robot_type")
    if robot_type != "mobile_robot":
        continue
    if args.controller == "batch":
        entity.setdefault("batch_controller", "batch_omni")
    else:
        entity.pop("batch_controller", None)

config.setdefault("simulation", {})["physics"] = True
num_robots = sum(int((entry.get("grid") or {}).get("count", 1)) for entry in entities)

print("=== 100 Robots Mixed Demo ===")
print(f"Config: {args.config}")
print(f"Total robots: {num_robots}")
print(f"Config entities: {len(entities)} group(s)")
print(f"Controller implementation: {args.controller}")

sim_core = MultiRobotSimulationCore.from_dict(config)
agents = [obj for obj in sim_core.sim_objects if isinstance(obj, Agent)]
mobile_agents = [agent for agent in agents if agent.user_data.get("robot_type") == "mobile_robot"]
arm_agents = [agent for agent in agents if agent.user_data.get("robot_type") == "arm_robot"]
print(f"✓ Loaded {len(mobile_agents)} mobile robots and " f"{len(arm_agents)} arm robots from entities[].grid")

camera_config = sim_core.params.camera_config
if camera_config:
    agent_positions = [agent.get_pose().position for agent in agents]
    sim_core.setup_camera(
        camera_config=camera_config,
        entity_positions=agent_positions,
    )
    print("✓ Camera configured from config file: " f"{camera_config.get('camera_mode', 'auto')} mode")


def _move_mobile_random(agent: Agent, dt: float) -> None:
    pose = agent.get_pose()
    pos, orn = pose.as_tuple()
    yaw = p.getEulerFromQuaternion(orn)[2]
    forward_vel = np.random.uniform(-2.0, 2.0)
    yaw_vel = np.random.uniform(-1.5, 1.5)
    new_x = pos[0] + forward_vel * np.cos(yaw) * dt
    new_y = pos[1] + forward_vel * np.sin(yaw) * dt
    new_yaw = yaw + yaw_vel * dt
    agent.set_pose(
        Pose.from_pybullet(
            [new_x, new_y, 0.3],
            p.getQuaternionFromEuler([0, 0, new_yaw]),
        )
    )


def _move_arm_random(agent: Agent) -> None:
    for joint_index in range(p.getNumJoints(agent.body_id)):
        info = p.getJointInfo(agent.body_id, joint_index)
        if info[2] != p.JOINT_REVOLUTE:
            continue
        lower, upper = info[8:10]
        if lower < upper:
            target_pos = np.random.uniform(lower, upper)
        else:
            target_pos = np.random.uniform(-1.0, 1.0)
        p.setJointMotorControl2(
            agent.body_id,
            joint_index,
            p.POSITION_CONTROL,
            targetPosition=target_pos,
            force=500,
        )


def mobile_movement_callback(_sim_core, dt):
    for agent in mobile_agents:
        _move_mobile_random(agent, dt)


def arm_movement_callback(_sim_core, _dt):
    for agent in arm_agents:
        _move_arm_random(agent)


sim_core.register_callback(mobile_movement_callback, frequency=None)
sim_core.register_callback(arm_movement_callback, frequency=10)
print("✓ Registered mobile movement callback")
print("✓ Registered arm joint callback")

sim_core.run_simulation(duration=args.duration)
