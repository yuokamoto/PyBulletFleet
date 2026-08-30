"""Compatibility imports for behavior-tree runners.

The implementation is split by portability boundary: ``behavior_tree_core``
has no PyBulletFleet dependency, while Agent and worker actions remain here.
"""

from pybullet_fleet.agent_behavior_tree import AgentActionId, AgentBehaviorTree, PyBulletFleetNavigationAdapter
from pybullet_fleet.behavior_tree_core import (
    BehaviorTree,
    BehaviorTreeNodeType,
    NavigationAdapter,
    NavigationStatus,
    TickResult,
)
from pybullet_fleet.worker_behavior_tree import WorkerActionId, WorkerBehaviorTree

__all__ = [
    "AgentBehaviorTree",
    "AgentActionId",
    "BehaviorTree",
    "BehaviorTreeNodeType",
    "NavigationAdapter",
    "NavigationStatus",
    "PyBulletFleetNavigationAdapter",
    "TickResult",
    "WorkerBehaviorTree",
    "WorkerActionId",
]
