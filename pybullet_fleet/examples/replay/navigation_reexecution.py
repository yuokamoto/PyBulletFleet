"""Representative navigation/stop scenario; no historical failure is claimed.

Run: python -m pybullet_fleet.examples.replay.navigation_reexecution /tmp/pbf-replay-demo
The output directory must not exist. No ROS, GUI, or USO is required.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from pybullet_fleet.replay import ReplayInput, ReplaySession, compare, reexecute


def run(directory: Path) -> dict:
    directory.mkdir(parents=True, exist_ok=False)
    initial = {
        "world": {
            "entities": [
                {"entity_id": "robot-a", "name": "amr_a", "position": [0, 0, 0.1]},
                {"entity_id": "robot-b", "name": "amr_b", "position": [0, 2, 0.1]},
            ]
        },
        "pbf": {"controller": "batch_omni", "timestep": 0.1},
    }
    original = directory / "original"
    with ReplaySession.create(initial, output=original) as session:
        for step in range(60):
            inputs = []
            if step == 0:
                inputs = [
                    ReplayInput.navigate("amr_a", (5, 0), source="external", command_id="a-go"),
                    ReplayInput.navigate("amr_b", (4, 2), source="ros", command_id="b-go"),
                ]
            elif step == 10:
                inputs = [ReplayInput.stop(["amr_a", "unknown"], command_id="partial-stop")]
            elif step == 15:
                inputs = [ReplayInput.navigate("amr_a", (2, 0), command_id="a-restart")]
            session.step(inputs)
    repeated = reexecute(original, directory / "repeated")
    variant = reexecute(original, directory / "slower", pbf_overrides={"limits": {"max_linear_vel": 0.5}})
    return {"repeated": compare(original, repeated), "variant": compare(original, variant)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    results = run(args.output)
    for label, result in results.items():
        print(label, result.status, result.first_difference)
    if results["repeated"].status != "matched" or results["variant"].status != "different":
        raise SystemExit("unexpected comparison result")


if __name__ == "__main__":
    main()
