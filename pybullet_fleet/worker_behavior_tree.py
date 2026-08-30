"""Worker-specific behavior-tree actions."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from xml.etree import ElementTree

from pybullet_fleet.agent_behavior_tree import AgentBehaviorTree
from pybullet_fleet.behavior_tree_core import ActionHandler, TickResult


class WorkerActionId(str, Enum):
    """Worker-specific portable Action IDs."""

    SELECT_GOAL_FROM_WAYPOINT_SET = "SelectGoalFromWaypointSet"
    WAIT_RANDOM = "WaitRandom"


@dataclass
class WorkerBehaviorTree(AgentBehaviorTree):
    """Worker-specific zone selection and wait actions."""

    waypoint_sets: dict[str, list[tuple[float, float, float]]] = field(default_factory=dict)

    def _action_handlers(self) -> dict[str | None, ActionHandler]:
        return {
            **super()._action_handlers(),
            WorkerActionId.SELECT_GOAL_FROM_WAYPOINT_SET.value: (
                lambda node, _sim_time: self._select_goal_from_waypoint_set(node)
            ),
            WorkerActionId.WAIT_RANDOM.value: self._wait_random,
        }

    def _select_goal_from_waypoint_set(self, node: ElementTree.Element) -> TickResult:
        waypoint_set = self._port(node.attrib.get("waypoint_set", ""))
        choices = self.waypoint_sets.get(str(waypoint_set), [])
        if not choices:
            return TickResult.FAILURE
        output = node.attrib.get("output_goal", "{goal}")
        self._set_port(output, tuple(self._rng.choice(choices)))
        return TickResult.SUCCESS

    def _wait_random(self, node: ElementTree.Element, sim_time: float) -> TickResult:
        state = self._state.setdefault(id(node), {})
        if "until" not in state:
            low = float(node.attrib.get("min_seconds", "0"))
            high = float(node.attrib.get("max_seconds", str(low)))
            if high < low:
                raise ValueError("WaitRandom max_seconds must be >= min_seconds")
            state["until"] = sim_time + self._rng.uniform(low, high)
        if sim_time < state["until"]:
            return TickResult.RUNNING
        self._reset(node)
        return TickResult.SUCCESS
