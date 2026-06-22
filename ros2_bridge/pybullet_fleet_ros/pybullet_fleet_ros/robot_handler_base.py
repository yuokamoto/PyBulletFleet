"""Abstract base class for per-robot ROS handlers.

Defines the lifecycle protocol that BridgeNode calls on each handler:

    PRE_STEP  event → handler.pre_step(dt, stamp)
    sim.step()
    POST_STEP event → handler.post_step(dt, stamp)

BridgeNode converts ``sim_time`` (float) → ``stamp`` (builtin_interfaces/Time)
once per step, so handlers receive a ready-to-use ROS timestamp.
If raw simulation time is needed, use ``self.agent._sim_core.sim_time``.

Subclasses must implement :meth:`destroy` to clean up ROS resources.
:meth:`pre_step` and :meth:`post_step` are no-ops by default — override
only those you need.
"""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from builtin_interfaces.msg import Time as TimeMsg

    from pybullet_fleet.agent import Agent
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster


class RobotHandlerBase(ABC):
    """Abstract base for per-robot ROS 2 interface handlers.

    Attributes:
        node: The ROS 2 node (BridgeNode) this handler belongs to.
        agent: The PyBulletFleet Agent this handler manages.
        ns: ROS namespace derived from the agent name.
        tf_broadcaster: Shared TF broadcaster (optional).
    """

    def __init__(
        self,
        node: "Node",
        agent: "Agent",
        tf_broadcaster: Optional["TransformBroadcaster"] = None,
    ) -> None:
        self.node = node
        self.agent = agent
        self.ns: str = agent.name or f"robot{agent.object_id}"
        self.tf_broadcaster = tf_broadcaster

    def pre_step(self, dt: float, stamp: "TimeMsg") -> None:
        """Called before each simulation step (PRE_STEP event).

        Override to apply pending commands (e.g. cmd_vel) before the
        simulation advances.  Default is a no-op.

        Args:
            dt: Time delta of this step.
            stamp: ROS timestamp for this step.
        """

    def post_step(self, dt: float, stamp: "TimeMsg") -> None:
        """Called after each simulation step (POST_STEP event).

        Override to publish state (odom, TF, diagnostics, sensor data).
        Default is a no-op.

        Args:
            dt: Time delta of this step.
            stamp: ROS timestamp for this step.
        """

    @abstractmethod
    def destroy(self) -> None:
        """Clean up all ROS resources (publishers, subscribers, action servers).

        Must be implemented by every subclass.
        """
