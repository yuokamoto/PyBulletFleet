"""Abstract base class for singleton bridge plugins.

Bridge plugins are loaded once by :class:`BridgeNode` and provide
global ROS functionality (as opposed to per-robot
:class:`RobotHandlerBase` instances).

Lifecycle::

    __init__(node, sim_core, config)   # construction
    post_step(sim_time)                # called each sim step (after robot handlers)
    destroy()                          # cleanup before shutdown/reset

Example::

    class MyBridgePlugin(BridgePluginBase):
        def __init__(self, node, sim_core, config=None):
            super().__init__(node, sim_core, config)
            self._pub = node.create_publisher(String, "/my_topic", 10)

        def post_step(self, sim_time: float) -> None:
            self._pub.publish(String(data=f"t={sim_time:.2f}"))

        def destroy(self) -> None:
            self._node.destroy_publisher(self._pub)

YAML config::

    bridge_plugins:
      - class: my_pkg.my_bridge_plugin.MyBridgePlugin
        config:
          some_param: 42
"""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Dict, Optional

if TYPE_CHECKING:
    from pybullet_fleet.core_simulation import MultiRobotSimulationCore
    from rclpy.node import Node


class BridgePluginBase(ABC):
    """Abstract base for singleton bridge plugins.

    Subclasses must implement :meth:`post_step` and :meth:`destroy`.

    Attributes:
        node: The ROS 2 BridgeNode instance.
        sim_core: The PyBulletFleet simulation core.
        config: Plugin-specific configuration dict.
    """

    def __init__(
        self,
        node: "Node",
        sim_core: "MultiRobotSimulationCore",
        config: Optional[Dict[str, Any]] = None,
    ) -> None:
        self.node = node
        self.sim_core = sim_core
        self.config: Dict[str, Any] = config or {}

    @abstractmethod
    def post_step(self, sim_time: float) -> None:
        """Called after each simulation step.

        Publish ROS messages, update state, etc.

        Args:
            sim_time: Current simulation time in seconds.
        """

    @abstractmethod
    def destroy(self) -> None:
        """Clean up all ROS resources (publishers, subscribers, timers).

        Called when BridgeNode resets or shuts down.
        """
