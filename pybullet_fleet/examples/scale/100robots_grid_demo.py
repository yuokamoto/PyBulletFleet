#!/usr/bin/env python3
"""
100robots_grid_demo.py
Config-driven scale demo for spawning 100 mobile robots from entities[].grid.

Command comparison:
  --controller=per_agent|batch chooses the simulation controller.
  --command-interface=per_agent|fleet chooses how repeated goals are submitted.

Examples:
  python 100robots_grid_demo.py
  python 100robots_grid_demo.py --movement=goal --duration=10
"""
import argparse
import os
import sys
import time

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
from pybullet_fleet.commands import RobotGoalCommand2D
from pybullet_fleet.config_utils import load_yaml_config, merge_configs
from pybullet_fleet.core_simulation import MultiRobotSimulationCore
from pybullet_fleet.fleet_api import FleetCommandDispatcher, FleetStateProvider


parser = argparse.ArgumentParser(description="100 Robots Grid Demo")
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
    "--command-interface",
    choices=["per_agent", "fleet"],
    default="fleet",
    help="Command API used when --movement=goal (default: fleet)",
)
parser.add_argument(
    "--movement",
    choices=["random", "goal", "none"],
    default="random",
    help="Movement mode: random callback, repeated goals, or spawn only",
)
parser.add_argument(
    "--config",
    default="config/100robots_config.yaml",
    help="Demo config merged over config/config.yaml",
)
args = parser.parse_args()


config = merge_configs(
    load_yaml_config("config/config.yaml"),
    load_yaml_config(args.config),
)
entities = config.get("entities") or []
if not entities:
    raise ValueError("100robots_grid_demo.py requires a config with entities[].grid")

for entity in entities:
    if args.controller == "batch":
        entity.setdefault("batch_controller", "batch_omni")
    else:
        entity.pop("batch_controller", None)

config.setdefault("simulation", {})["physics"] = False
num_robots = sum(int((entry.get("grid") or {}).get("count", 1)) for entry in entities)

print("=== 100 Robots Grid Demo ===")
print(f"Config: {args.config}")
print(f"Total robots: {num_robots}")
print(f"Config entities: {len(entities)} group(s)")
print(f"Controller implementation: {args.controller}")
print(f"Command interface: {args.command_interface}")
print(f"Movement: {args.movement}")
print("Mode: entities[].grid mobile robots")

sim_core = MultiRobotSimulationCore.from_dict(config)
agents = [obj for obj in sim_core.sim_objects if isinstance(obj, Agent)]
print(f"✓ Loaded {len(agents)} agents from entities[].grid")

fleet_dispatcher = FleetCommandDispatcher(sim_core)
fleet_state_provider = FleetStateProvider(sim_core)


def _goal_for_agent(agent: Agent, cycle: int) -> Pose:
    pose = agent.get_pose()
    direction = 1.0 if cycle % 2 == 0 else -1.0
    return Pose.from_yaw(pose.x + direction * 0.5, pose.y, pose.z, pose.yaw)


def _send_goal_commands(cycle: int) -> None:
    if args.command_interface == "per_agent":
        start = time.perf_counter()
        for agent in agents:
            agent.set_goal_pose(_goal_for_agent(agent, cycle))
        elapsed = time.perf_counter() - start
        accepted = len(agents)
        rejected = 0
    else:
        commands = []
        for agent in agents:
            goal = _goal_for_agent(agent, cycle)
            commands.append(
                RobotGoalCommand2D(
                    name=agent.name,
                    position=(goal.x, goal.y),
                    yaw=goal.yaw,
                    z=goal.z,
                )
            )
        start = time.perf_counter()
        ack = fleet_dispatcher.navigate(
            commands,
            source="100robots-grid-demo",
            command_id=f"grid-demo-nav-{cycle}",
        )
        elapsed = time.perf_counter() - start
        accepted = len(ack.accepted_names)
        rejected = len(ack.rejected)

    state_start = time.perf_counter()
    state_count = len(fleet_state_provider.get_states())
    state_elapsed = time.perf_counter() - state_start
    print(
        "[goal_cycle=%d | controller_impl=%s | command_interface=%s | "
        "mobile_goals=%d | accepted=%d | rejected=%d | "
        "command_setup=%.6fs | state_snapshot=%d in %.6fs]"
        % (
            cycle,
            args.controller,
            args.command_interface,
            len(agents),
            accepted,
            rejected,
            elapsed,
            state_count,
            state_elapsed,
        )
    )


camera_config = sim_core.params.camera_config
if camera_config:
    agent_positions = [agent.get_pose().position for agent in agents]
    sim_core.setup_camera(
        camera_config=camera_config,
        entity_positions=agent_positions,
    )
    print("✓ Camera configured from config file: " f"{camera_config.get('camera_mode', 'auto')} mode")


def random_movement_callback(_sim_core, dt):
    for agent in agents:
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


goal_state = {"cycle": 0, "next_time": 0.0}
GOAL_PERIOD_S = 3.0


def goal_command_callback(_sim_core, _dt):
    if sim_core.sim_time + 1e-9 < goal_state["next_time"]:
        return
    _send_goal_commands(goal_state["cycle"])
    goal_state["cycle"] += 1
    goal_state["next_time"] = sim_core.sim_time + GOAL_PERIOD_S


if args.movement == "random":
    sim_core.register_callback(random_movement_callback, frequency=None)
    print("✓ Registered random movement callback")
elif args.movement == "goal":
    sim_core.register_callback(goal_command_callback, frequency=1)
    print("✓ Registered repeated goal command callback " f"(period={GOAL_PERIOD_S:.1f}s)")
else:
    state_start = time.perf_counter()
    state_count = len(fleet_state_provider.get_states())
    state_elapsed = time.perf_counter() - state_start
    print(f"✓ Spawn-only mode; state_snapshot={state_count} " f"in {state_elapsed:.6f}s")

sim_core.run_simulation(duration=args.duration)
