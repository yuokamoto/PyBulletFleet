#!/usr/bin/env python3
"""Plugin demo — reusable simulation and per-agent behavior.

Demonstrates:
- A config-loaded SimPlugin with on_init/on_step/on_shutdown hooks
- Frequency-controlled SimPlugin updates
- A BatteryPlugin attached to an Agent through its YAML-style plugins entry
- A normal MoveAction running alongside both plugin types

Usage:
    python examples/basics/plugin_demo.py
"""

import os
import sys

# Run from a source checkout without installing: fall back to the repo root
# so `import pybullet_fleet` resolves. Installed/editable users never hit this.
try:
    import pybullet_fleet  # noqa: F401
except ModuleNotFoundError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    import pybullet_fleet  # noqa: F401

from pybullet_fleet import MultiRobotSimulationCore
from pybullet_fleet.action import MoveAction
from pybullet_fleet.geometry import Path
from pybullet_fleet.sim_plugin import SimPlugin


class StatusReporterPlugin(SimPlugin):
    """Small simulation-wide plugin used only by this demo."""

    def __init__(self, sim_core, label: str = "status"):
        super().__init__(sim_core)
        self.label = label
        self.calls = 0

    def on_init(self) -> None:
        names = [agent.name for agent in self.sim_core.agents]
        print(f"[{self.label}] on_init: agents={names}")

    def on_step(self, dt: float) -> None:
        self.calls += 1
        agent = self.sim_core.agents[0]
        print(
            f"[{self.label}] on_step #{self.calls}: "
            f"dt={dt:.2f}s, position={agent.get_pose().position[:2]}, "
            f"battery={agent.battery_soc:.1%}"
        )

    def on_shutdown(self) -> None:
        print(f"[{self.label}] on_shutdown after {self.calls} calls")


def main() -> None:
    # ``from_dict()`` creates the simulation, agent, and SimPlugin from one
    # YAML-shaped configuration. A real YAML file would use the same keys.
    config = {
        "simulation": {
            "gui": False,
            "monitor": False,
            "physics": False,
            "enable_floor": False,
            "timestep": 0.1,
            "target_rtf": 0,
            "duration": 1.2,
        },
        "entities": [
            {
                "name": "delivery_01",
                "urdf_path": "robots/mobile_robot.urdf",
                "pose": [0.0, 0.0, 0.1],
                "controller": {"type": "omni", "max_linear_vel": 1.0},
                "plugins": [
                    {
                        "type": "battery",
                        "config": {"initial_soc": 0.8, "discharge_rate": 0.05},
                    }
                ],
            }
        ],
        "plugins": [
            {
                # In a real YAML file, use an importable package class rather
                # than this demo module's __main__ path.
                "class": f"{__name__}.StatusReporterPlugin",
                "frequency": 2.0,
                "config": {"label": "sim-plugin"},
            }
        ],
    }

    sim = MultiRobotSimulationCore.from_dict(config)
    sim.initialize_simulation()
    agent = sim.agents[0]

    # The AgentPlugin was created from entities[].plugins before the agent was
    # announced to observers. Its state is exposed by Agent convenience APIs.
    print(f"[agent-plugin] BatteryPlugin attached: initial SOC={agent.battery_soc:.1%}")

    # Plugins do not replace normal robot behavior. The agent moves through its
    # controller/action lifecycle while StatusReporterPlugin observes the sim.
    agent.add_action(MoveAction(path=Path.from_positions([[1.0, 0.0, 0.1]])))
    sim.run_simulation()


if __name__ == "__main__":
    main()
