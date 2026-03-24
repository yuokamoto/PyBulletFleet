#!/usr/bin/env python3
"""Send a NavigateToPose goal to a robot (Phase 2 — requires nav action server).

Usage:
    python3 scripts/send_nav_goal.py --robot robot_0 --x 5.0 --y 5.0

NOTE: This requires the NavigateToPose action server which is deferred to Phase 2.
For now, use cmd_vel or set_entity_state service to move robots.
"""

import argparse


def main():
    parser = argparse.ArgumentParser(description="Send NavigateToPose goal (Phase 2)")
    parser.add_argument("--robot", default="robot_0")
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, default=0.05)
    args = parser.parse_args()

    print("NavigateToPose action server is deferred to Phase 2.")
    print("For now, use cmd_vel or SetEntityState service:")
    print(f"  python3 scripts/teleop_cmd_vel.py --robot {args.robot} --vx 1.0 --duration 5")
    print(
        f"  ros2 service call /sim/set_entity_state "
        f"simulation_interfaces/srv/SetEntityState "
        f"\"{{name: '{args.robot}', state: {{pose: {{position: "
        f'{{x: {args.x}, y: {args.y}, z: {args.z}}}}}}}}}"'
    )


if __name__ == "__main__":
    main()
