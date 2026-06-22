#!/usr/bin/env python3
"""Smoke test for a *clean* (pip-installed, no repo mount) pybullet-fleet.

Run from a directory that is NOT the repo so any reliance on the source tree
(CWD-relative ``robots/...`` / ``config/...`` paths) fails loudly. Verifies that
bundled assets ship in the wheel and resolve correctly:

  - ``resolve_model("mobile_robot")``        — bundled model name
  - ``resolve_model("robots/arm_robot.urdf")`` — bundled relative path
  - ``load_yaml_config("config/config.yaml")`` — bundled config
  - ``Agent.from_params(urdf_path=...)``      — spawn a bundled robot end-to-end

Exits non-zero on the first failure. Used by scripts/test_clean_install.sh and
the docker `pip-test` service.
"""
import os
import sys


def main() -> int:
    # Guard: do not run from the repo checkout (would mask packaging bugs).
    if os.path.isdir(os.path.join(os.getcwd(), "pybullet_fleet")):
        print("WARNING: running from the repo root — run from elsewhere to test the installed wheel.")

    from pybullet_fleet.agent import Agent, AgentSpawnParams
    from pybullet_fleet.config_utils import load_yaml_config
    from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
    from pybullet_fleet.geometry import Pose
    from pybullet_fleet.robot_models import resolve_model

    checks = []

    p1 = resolve_model("mobile_robot")
    checks.append(("resolve_model('mobile_robot') is a real file", os.path.isfile(p1)))

    p2 = resolve_model("robots/arm_robot.urdf")
    checks.append(("resolve_model('robots/arm_robot.urdf') is a real file", os.path.isfile(p2)))

    cfg = load_yaml_config("config/config.yaml")
    checks.append(("bundled config/config.yaml loads", isinstance(cfg, dict) and "simulation" in cfg))

    sim = MultiRobotSimulationCore(SimulationParams(gui=False, physics=False, monitor=False))
    agent = Agent.from_params(
        AgentSpawnParams(urdf_path="robots/mobile_robot.urdf", initial_pose=Pose.from_xyz(0, 0, 0.1), name="smoke"),
        sim,
    )
    checks.append(("spawn bundled mobile_robot", agent is not None and agent.body_id >= 0))

    ok = True
    for name, passed in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}")
        ok = ok and passed

    print("=== clean-install smoke test:", "PASSED ===" if ok else "FAILED ===")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
