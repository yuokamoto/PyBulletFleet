#!/usr/bin/env python3
"""
robot_descriptions_demo.py
Demo: Using Tier 3 models from the ``robot_descriptions`` pip package.

Shows the Tier 3 workflow:
1. First run without the package → helpful error with install instructions
2. After ``pip install robot_descriptions`` → model auto-downloads on first use
3. Subsequent runs use the cached URDF instantly

Available Tier 3 models (registered in KNOWN_MODELS):
  - tiago   (PAL TIAGo service robot)
  - pr2     (Willow Garage PR2)

The ``robot_descriptions`` package supports 80+ robots. To add more, register
them in ``KNOWN_MODELS`` in ``pybullet_fleet/robot_models.py``.

Usage::

    python robot_descriptions_demo.py                     # default: tiago
    python robot_descriptions_demo.py --robot pr2
    python robot_descriptions_demo.py --robot panda       # Tier 1 also works
    python robot_descriptions_demo.py --list              # show all models

Prerequisites::

    pip install pybullet-fleet[models]
    # or: pip install robot_descriptions
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from pybullet_fleet.robot_models import auto_detect_profile, list_all_models, resolve_urdf

parser = argparse.ArgumentParser(description="Tier 3 robot_descriptions demo")
parser.add_argument("--robot", default="tiago", help="Robot name (default: tiago)")
parser.add_argument("--list", action="store_true", help="List all available models and exit")
args = parser.parse_args()

# --list mode
if args.list:
    models = list_all_models()
    print(f"\n{'Name':<22} {'Tier':<22} {'Available':<10} {'Path / Error'}")
    print("-" * 100)
    for name, info in models.items():
        available = "yes" if info["available"] else "no"
        detail = info.get("path", info.get("error", ""))
        # Truncate long paths
        if len(detail) > 50:
            detail = "..." + detail[-47:]
        print(f"{name:<22} {info['tier']:<22} {available:<10} {detail}")
    print()
    sys.exit(0)

# Resolve URDF — this is where the magic happens:
#   - If robot_descriptions is not installed → FileNotFoundError with install hint
#   - If installed but model not cached → git clone runs (progress bar shown)
#   - If cached → instant return
print(f"Resolving '{args.robot}'...")
try:
    urdf_path = resolve_urdf(args.robot)
except FileNotFoundError as e:
    print(f"\n  ERROR: {e}")
    print("\n  To fix, run:")
    print("    pip install robot_descriptions")
    print("  or:")
    print("    pip install pybullet-fleet[models]")
    print()
    sys.exit(1)

print(f"  → {urdf_path}")

if not os.path.isfile(urdf_path):
    print(f"  ERROR: File not found: {urdf_path}")
    sys.exit(1)

# --- Spawn and inspect ---
from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Pose

params = SimulationParams(gui=True, timestep=0.1, physics=False, target_rtf=1.0)
sim = MultiRobotSimulationCore(params)

# Auto-detect profile
profile = auto_detect_profile(urdf_path, sim.client)
print(f"\nRobotProfile for '{args.robot}':")
print(f"  type:       {profile.robot_type}")
print(f"  num_joints: {profile.num_joints}")
if profile.ee_link_name:
    print(f"  ee_link:    {profile.ee_link_name} (index={profile.ee_link_index})")
if profile.movable_joint_names:
    print(f"  joints:     {profile.movable_joint_names[:6]}{'...' if len(profile.movable_joint_names) > 6 else ''}")

# Spawn agent — use profile hints for spawn params
is_mobile = profile.robot_type in ("mobile", "mobile_manipulator")
spawn_params = AgentSpawnParams(
    urdf_path=urdf_path,
    initial_pose=Pose.from_xyz(0, 0, 0),
    use_fixed_base=not is_mobile,
    name=args.robot,
)
agent = Agent.from_params(spawn_params, sim)
print(f"\nSpawned '{agent.name}' with {agent.get_num_joints()} joints")

# If the robot has movable joints, gently cycle them so it's not static
if profile.movable_joint_names:
    step: list[float] = [0.0] * len(profile.movable_joint_names)
    directions: list[float] = [1.0] * len(profile.movable_joint_names)

    def _wave_joints(sim_core, dt):
        """Slowly oscillate each joint within ±20% of its range."""
        states = agent.get_all_joints_state()
        if not states:
            return
        targets = [s[0] for s in states]  # current positions
        dt = 0.3  # fraction increment per call
        for i, idx in enumerate(profile._movable_joint_indices):
            lo = profile.joint_lower_limits[i]
            hi = profile.joint_upper_limits[i]
            mid = (lo + hi) / 2.0
            half_range = (hi - lo) / 2.0
            step[i] += directions[i] * dt * 0.05
            if abs(step[i]) > 0.2:
                directions[i] *= -1
            if idx < len(targets):
                targets[idx] = mid + step[i] * half_range
        agent.set_all_joints_targets(targets)

    sim.register_callback(_wave_joints, frequency=10.0)
    print("Joints oscillating — close the PyBullet GUI window to exit.")
else:
    print("Close the PyBullet GUI window to exit.")

# Run until user closes window
sim.run_simulation()
