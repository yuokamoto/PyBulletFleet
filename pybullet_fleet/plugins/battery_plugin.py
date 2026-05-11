"""BatteryPlugin — per-agent battery simulation.

Drains/charges battery SOC each step based on the plugin's
rate attributes and the agent's current motion/charging state.

The plugin **owns** all battery state (rates, SOC, charging flag).
Agent exposes delegate properties for convenience::

    agent.battery_soc       # → plugin.soc
    agent.battery_plugin    # → the BatteryPlugin instance (or None)
    agent.is_charging       # → plugin.is_charging
    agent.set_charging(b)   # → plugin.set_charging(b)

Created automatically by :meth:`Agent.from_params` when listed in
``AgentSpawnParams.plugins``, or attached manually.

Example (via plugins list in YAML)::

    plugins:
      - type: battery
        config:
          initial_soc: 0.8
          discharge_rate: 0.002

Example (manual)::

    from pybullet_fleet.plugins.battery_plugin import BatteryPlugin
    agent.add_plugin(BatteryPlugin(agent, discharge_rate=0.002))

Example (custom subclass)::

    class TemperatureBattery(BatteryPlugin):
        def on_update(self, dt: float) -> None:
            temp_factor = 1.0 + 0.02 * (self.agent.user_data.get("temp", 25) - 25)
            if self.is_charging:
                self.soc = min(1.0, self.soc + self.charge_rate * dt)
            elif self.agent.is_moving:
                self.soc = max(0.0, self.soc - self.discharge_rate * dt * temp_factor)

    agent.remove_plugin(BatteryPlugin)
    agent.add_plugin(TemperatureBattery(agent, initial_soc=0.9, discharge_rate=0.005))
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from pybullet_fleet.agent_plugin import AgentPlugin

if TYPE_CHECKING:
    from pybullet_fleet.agent import Agent

logger = logging.getLogger(__name__)


class BatteryPlugin(AgentPlugin):
    """Default linear battery drain/charge plugin.

    Owns all battery state as plain attributes:

    - :attr:`discharge_rate` — SOC drain per second while moving
    - :attr:`charge_rate` — SOC gain per second while charging
    - :attr:`idle_rate` — SOC drain per second while idle
    - :attr:`soc` — current state of charge [0.0, 1.0]
    - :attr:`is_charging` — whether the agent is charging

    Update logic:

    - **charging** → ``soc += charge_rate * dt``
    - **moving** → ``soc -= discharge_rate * dt``
    - **idle** → ``soc -= idle_rate * dt``

    SOC is clamped to ``[0.0, 1.0]``.
    """

    _registry_name = "battery"

    def __init__(
        self,
        agent: "Agent",
        initial_soc: float = 1.0,
        discharge_rate: float = 0.001,
        charge_rate: float = 0.005,
        idle_rate: float = 0.0,
    ) -> None:
        super().__init__(agent)
        self.discharge_rate: float = discharge_rate
        self.charge_rate: float = charge_rate
        self.idle_rate: float = idle_rate
        self.soc: float = initial_soc
        self.is_charging: bool = False

    def set_charging(self, charging: bool) -> None:
        """Start or stop charging.

        Args:
            charging: ``True`` to start, ``False`` to stop.
        """
        if self.is_charging != charging:
            logger.info("'%s': charging %s", self.agent.name, "started" if charging else "stopped")
        self.is_charging = charging

    def on_update(self, dt: float) -> None:
        """Update battery SOC based on agent's current state."""
        if self.is_charging:
            self.soc = min(1.0, self.soc + self.charge_rate * dt)
        elif self.agent.is_moving:
            self.soc = max(0.0, self.soc - self.discharge_rate * dt)
        else:
            self.soc = max(0.0, self.soc - self.idle_rate * dt)
