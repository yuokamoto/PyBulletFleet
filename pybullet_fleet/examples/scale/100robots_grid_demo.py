#!/usr/bin/env python3
"""
100robots_grid_demo.py
Config-driven scale demo for spawning 100 robots with AgentManager.

Modes:
  --mode=mixed (default): Spawn mixed robot types (mobile 60% + arm 40%)
  --mode=single: Spawn single robot type (mobile only)
  --movement=random (default): Existing random pose/joint movement
  --movement=goal: Repeatedly send goals to mobile robots via per-agent or fleet API.
                   In mixed mode, arm robots keep moving with random joint commands.
  --movement=none: Spawn only and print a fleet state snapshot.

Command comparison:
  --controller=per_agent|batch chooses the simulation controller implementation.
  --command-interface=per_agent|fleet chooses how goal commands are submitted.
  The standard scene is loaded from config/100robots_config.yaml; --config can
  point to another config with the same top-level keys.

Examples:
  python 100robots_grid_demo.py              # Mixed mode (default)
  python 100robots_grid_demo.py --mode=mixed # Mixed mode (explicit)
  python 100robots_grid_demo.py --mode=single # Single type mode
  python 100robots_grid_demo.py --mode=single --movement=goal --command-interface=fleet
  python 100robots_grid_demo.py --movement=goal --command-interface=per_agent --duration=10
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
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    import pybullet_fleet  # noqa: F401

from pybullet_fleet.agent import Agent, AgentSpawnParams, Pose
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.config_utils import load_yaml_config, merge_configs
from pybullet_fleet.core_simulation import MultiRobotSimulationCore
from pybullet_fleet.fleet_api import FleetCommandDispatcher, FleetStateProvider, RobotGoalCommand2D
from pybullet_fleet.robot_models import resolve_model

# ========================================
# Parse Arguments
# ========================================

parser = argparse.ArgumentParser(description="100 Robots Grid Demo")
parser.add_argument(
    "--mode",
    type=str,
    default="mixed",
    choices=["mixed", "single"],
    help="Spawn mode: mixed (mobile+arm) or single (mobile only)",
)
parser.add_argument("--robot", default="husky", help="Robot name (e.g. husky, racecar) or URDF path for mobile robots")
parser.add_argument(
    "--arm-robot",
    default="panda",
    help="Arm robot name (e.g. panda, kuka_iiwa, arm_robot) or URDF path",
)
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
    help="Movement mode: random callback, repeated goal commands, or spawn only",
)
parser.add_argument(
    "--config",
    default="config/100robots_config.yaml",
    help="Demo config merged over config/config.yaml",
)
args = parser.parse_args()

# ========================================
# Load Configuration
# ========================================

# Bundled configs — resolved against the packaged config root by load_yaml_config.
base_config_path = "config/config.yaml"
robots_config_path = args.config

# Load both configs (100robots_config overrides config.yaml)
config = merge_configs(load_yaml_config(base_config_path), load_yaml_config(robots_config_path))
mode = args.mode

# Extract parameters from config
num_robots = config.get("num_robots", 100)
grid_size = int(np.ceil(np.sqrt(num_robots)))

# Grid parameters
grid_config = config.get("grid", {})
spacing_config = config.get("spacing", [2.0, 2.0, 0.0])
offset_config = config.get("offset", [0.0, 0.0, 0.0])

# Robot parameters
mobile_robot_config = config.get("mobile_robot", {})
arm_robot_config = config.get("arm_robot", {})
mixed_mode_config = config.get("mixed_mode", {})

# Mixed mode probabilities
mobile_robot_prob = mixed_mode_config.get("mobile_robot_prob", 0.5)
arm_robot_prob = mixed_mode_config.get("arm_robot_prob", 0.5)

print("=== 100 Robots Grid Demo ===")
print(f"Config: {robots_config_path}")
print(f"Mode: {mode}")
print(f"Total robots: {num_robots}")
print(f"Grid size: {grid_size}x{grid_size}")
print(f"Controller implementation: {args.controller}")
print(f"Command interface: {args.command_interface}")
print(f"Movement: {args.movement}")
if mode == "mixed":
    print(f"Mobile robot probability: {mobile_robot_prob*100:.0f}%")
    print(f"Arm robot probability: {arm_robot_prob*100:.0f}%")
    if arm_robot_prob > 0:
        config.setdefault("simulation", {})["physics"] = True
        print("Mode: mixed (mobile + arm) - Physics enabled for robot arm")
else:
    config.setdefault("simulation", {})["physics"] = False
    print("Mode: mixed (mobile + arm) - Physics disabled for maximum performance")

sim_core = MultiRobotSimulationCore.from_dict(config)

# ========================================
# Setup Spawn Parameters
# ========================================

batch_controller = "batch_omni" if args.controller == "batch" else None
agent_manager = AgentManager(sim_core=sim_core, batch_controller=batch_controller)

# Create grid parameters from config
grid_params = GridSpawnParams(
    x_min=grid_config.get("x_min", 0),
    x_max=grid_config.get("x_max", grid_size - 1),
    y_min=grid_config.get("y_min", 0),
    y_max=grid_config.get("y_max", grid_size - 1),
    z_min=grid_config.get("z_min", 0),
    z_max=grid_config.get("z_max", 0),
    spacing=spacing_config,
    offset=offset_config,
)

# Mobile robot spawn params (shared by both modes) - from config
mobile_urdf = resolve_model(args.robot)
print(f"Using robot: {args.robot} -> {mobile_urdf}")
if not os.path.exists(mobile_urdf):
    raise FileNotFoundError(f"Mobile robot URDF not found: {mobile_urdf}")

mobile_params = AgentSpawnParams(
    urdf_path=mobile_urdf,
    initial_pose=Pose.from_xyz(0, 0, mobile_robot_config.get("initial_z", 0.3)),
    use_fixed_base=mobile_robot_config.get("use_fixed_base", False),
    controller=mobile_robot_config.get("controller"),
    user_data={"robot_type": "mobile_robot"},
)

# ========================================
# Prepare Spawn Parameters List
# ========================================

if mode == "mixed":
    # Mixed Mode: Spawn mixed robot types
    # Arm robot spawn params - resolve via robot_models
    arm_urdf = resolve_model(args.arm_robot)
    print(f"Using arm robot: {args.arm_robot} -> {arm_urdf}")

    arm_params = AgentSpawnParams(
        urdf_path=arm_urdf,
        initial_pose=Pose.from_xyz(0, 0, arm_robot_config.get("initial_z", 0.0)),
        use_fixed_base=arm_robot_config.get("use_fixed_base", True),
        controller=arm_robot_config.get("controller"),
        mass=arm_robot_config.get("mass", 1.0),  # Use URDF mass (1.0) for physics simulation
        user_data={"robot_type": "arm_robot"},
    )

    print("\n✓ Using URDFs:")
    print(f"  - Mobile: {mobile_urdf}")
    print(f"  - Arm: {arm_urdf}")

    # Create spawn params list with probabilities
    spawn_params_list = [(mobile_params, mobile_robot_prob), (arm_params, arm_robot_prob)]
    print(f"\nSpawning {num_robots} agents with mixed types...")

else:  # mode == 'single'
    # Single Mode: Spawn single robot type only
    print(f"\n✓ Using URDF: {mobile_urdf}")

    # Create spawn params list with 100% probability for mobile robot
    spawn_params_list = [(mobile_params, 1.0)]
    print(f"\nSpawning {num_robots} agents (mobile only)...")

# ========================================
# Spawn Agents (unified call)
# ========================================

spawned_agents = agent_manager.spawn_agents_grid_mixed(
    num_agents=num_robots,
    grid_params=grid_params,
    spawn_params_list=spawn_params_list,
    name_prefix="robot",
)

print(f"✓ Spawned {len(spawned_agents)} agents using spawn_agents_grid_mixed()")

print("\n=== AgentManager Status ===")
print(agent_manager)


def _mobile_agents() -> list[Agent]:
    return [agent for agent in spawned_agents if agent.user_data.get("robot_type", "mobile_robot") == "mobile_robot"]


def _arm_agents() -> list[Agent]:
    return [agent for agent in spawned_agents if agent.user_data.get("robot_type") == "arm_robot"]


mobile_agents = _mobile_agents()
arm_agents = _arm_agents()
fleet_dispatcher = FleetCommandDispatcher(sim_core)
fleet_state_provider = FleetStateProvider(sim_core)


def _goal_for_agent(agent: Agent, cycle: int) -> Pose:
    pose = agent.get_pose()
    direction = 1.0 if cycle % 2 == 0 else -1.0
    return Pose.from_yaw(pose.x + direction * 0.5, pose.y, pose.z, pose.yaw)


def _send_goal_commands(cycle: int) -> None:
    if args.command_interface == "per_agent":
        start = time.perf_counter()
        for agent in mobile_agents:
            agent.set_goal_pose(_goal_for_agent(agent, cycle))
        elapsed = time.perf_counter() - start
        accepted = len(mobile_agents)
        rejected = 0
    else:
        commands = []
        for agent in mobile_agents:
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
        ack = fleet_dispatcher.navigate(commands, source="100robots-grid-demo", command_id=f"grid-demo-nav-{cycle}")
        elapsed = time.perf_counter() - start
        accepted = len(ack.accepted_names)
        rejected = len(ack.rejected)

    state_start = time.perf_counter()
    state_count = len(fleet_state_provider.get_states())
    state_elapsed = time.perf_counter() - state_start
    print(
        "[goal_cycle=%d | controller_impl=%s | command_interface=%s | mobile_goals=%d | accepted=%d | rejected=%d | "
        "command_setup=%.6fs | state_snapshot=%d in %.6fs]"
        % (
            cycle,
            args.controller,
            args.command_interface,
            len(mobile_agents),
            accepted,
            rejected,
            elapsed,
            state_count,
            state_elapsed,
        )
    )


# ========================================
# Setup Camera
# ========================================

# Setup camera view using config file settings (auto mode with agent positions)
camera_config = sim_core.params.camera_config
if camera_config:
    # Provide agent positions for auto mode
    agent_positions = [agent.get_pose().position for agent in agent_manager.objects]
    sim_core.setup_camera(camera_config=camera_config, entity_positions=agent_positions)
    print(f"✓ Camera configured from config file: {camera_config.get('camera_mode', 'auto')} mode")
else:
    # Fallback: use agent_manager's setup_camera (auto mode)
    agent_manager.setup_camera({})
    print("✓ Camera configured with default auto mode")

# ========================================
# Movement Callback
# ========================================


def _move_arm_random(agent: Agent) -> None:
    num_joints = p.getNumJoints(agent.body_id)
    for j in range(num_joints):
        info = p.getJointInfo(agent.body_id, j)
        joint_type = info[2]
        if joint_type == p.JOINT_REVOLUTE:
            lower, upper = info[8:10]
            if lower < upper:
                target_pos = np.random.uniform(lower, upper)
            else:
                target_pos = np.random.uniform(-1.0, 1.0)
            p.setJointMotorControl2(agent.body_id, j, p.POSITION_CONTROL, targetPosition=target_pos, force=500)


def batch_agent_movement_callback(sim_core, dt):
    """
    Callback for mixed robot types.
    - Mobile robots: random forward/backward movement with rotation
    - Arm robots: random joint movements
    """
    for obj in sim_core.sim_objects:
        if not isinstance(obj, Agent):
            continue

        agent = obj
        robot_type = agent.user_data.get("robot_type", "mobile_robot")

        if robot_type == "mobile_robot":
            # Mobile robot movement (position-based)
            pose = agent.get_pose()
            pos, orn = pose.as_tuple()
            euler = p.getEulerFromQuaternion(orn)
            yaw = euler[2]

            max_linear_speed = 2.0
            max_angular_speed = 1.5
            forward_vel = np.random.uniform(-max_linear_speed, max_linear_speed)
            yaw_vel = np.random.uniform(-max_angular_speed, max_angular_speed)

            # Calculate new position based on velocity and dt
            forward_x = forward_vel * np.cos(yaw) * dt
            forward_y = forward_vel * np.sin(yaw) * dt
            new_x = pos[0] + forward_x
            new_y = pos[1] + forward_y

            # Calculate new orientation
            new_yaw = yaw + yaw_vel * dt
            corrected_orn = p.getQuaternionFromEuler([0, 0, new_yaw])

            # Set new pose (position-based control)
            new_pose = Pose.from_pybullet([new_x, new_y, 0.3], corrected_orn)
            agent.set_pose(new_pose)

        elif robot_type == "arm_robot":
            _move_arm_random(agent)


goal_state = {"cycle": 0, "next_time": 0.0}
GOAL_PERIOD_S = 3.0


def goal_command_callback(sim_core, dt):
    for agent in arm_agents:
        _move_arm_random(agent)
    if sim_core.sim_time + 1e-9 < goal_state["next_time"]:
        return
    _send_goal_commands(goal_state["cycle"])
    goal_state["cycle"] += 1
    goal_state["next_time"] = sim_core.sim_time + GOAL_PERIOD_S


if args.movement == "random":
    sim_core.register_callback(batch_agent_movement_callback, frequency=1)
    print("✓ Registered random movement callback")
elif args.movement == "goal":
    sim_core.register_callback(goal_command_callback, frequency=1)
    print(f"✓ Registered repeated goal command callback (period={GOAL_PERIOD_S:.1f}s)")
else:
    state_start = time.perf_counter()
    state_count = len(fleet_state_provider.get_states())
    state_elapsed = time.perf_counter() - state_start
    print(f"✓ Spawn-only mode; state_snapshot={state_count} in {state_elapsed:.6f}s")

sim_core.run_simulation(duration=args.duration)
