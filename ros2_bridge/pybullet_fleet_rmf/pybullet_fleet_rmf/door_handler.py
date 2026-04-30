"""Door handler — RMF door protocol bridge via handler_map.

Replaces the standalone ``door_adapter`` node.  Runs inside BridgeNode
with direct access to the :class:`Door` agent, so it can call
``request_open()`` / ``request_close()`` to animate URDF joints.

Lifecycle:
    - Subscribes to ``/adapter_door_requests`` (from rmf_door_supervisor).
    - On request: calls ``Door.request_open()`` or ``request_close()``.
    - ``post_step()``: publishes ``/door_states`` (throttled to ~1 Hz).

Config (bridge YAML)::

    handler_map:
      L*_door*: pybullet_fleet_ros.door_handler.DoorHandler

The handler reads ``open_positions``/``close_positions`` from the agent's
``user_data`` (already parsed by Door via DoorParams).
"""

import logging

from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from rmf_door_msgs.msg import DoorMode, DoorRequest, DoorState

from pybullet_fleet_ros.robot_handler_base import RobotHandlerBase

from pybullet_fleet.types import DoorState as SimDoorState

logger = logging.getLogger(__name__)


class DoorHandler(RobotHandlerBase):
    """Per-door handler that bridges RMF door protocol to Door.

    Each door agent gets its own DoorHandler instance.  All handlers
    share one ``/door_states`` publisher and one ``/adapter_door_requests``
    subscription, de-multiplexing by ``door_name``.
    """

    # Class-level shared ROS resources (created once, shared across instances).
    _shared_pub = None  # DoorState publisher
    _shared_sub = None  # DoorRequest subscription
    _instances: dict[str, "DoorHandler"] = {}  # door_name → handler

    def __init__(self, node, agent, tf_broadcaster=None):
        super().__init__(node, agent, tf_broadcaster)
        from pybullet_fleet.devices.door import Door

        if not isinstance(agent, Door):
            raise TypeError(f"DoorHandler requires Door, got {type(agent).__name__} " f"for '{agent.name}'")

        self._door: Door = agent
        self._last_publish_time: float = -1.0
        self._publish_interval: float = 1.0  # seconds

        # Register this instance for request dispatch
        DoorHandler._instances[self.ns] = self

        # Lazily create shared pub/sub on first instance
        if DoorHandler._shared_pub is None:
            transient_qos = QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            DoorHandler._shared_pub = node.create_publisher(DoorState, "door_states", qos_profile=transient_qos)
            DoorHandler._shared_sub = node.create_subscription(
                DoorRequest, "adapter_door_requests", DoorHandler._on_request, 10
            )

        # Publish initial state (closed)
        self._publish_state()

        logger.info("DoorHandler for '%s'", self.ns)

    # ------------------------------------------------------------------
    # Request handling (class-level dispatch)
    # ------------------------------------------------------------------

    @staticmethod
    def _on_request(msg: DoorRequest):
        """Dispatch incoming door requests to the correct handler instance."""
        handler = DoorHandler._instances.get(msg.door_name)
        if handler is None:
            # Unknown door — still publish instantaneous response
            # (matches standalone door_adapter behaviour for implicit doors)
            logger.debug("DoorHandler: door '%s' not managed, ignoring", msg.door_name)
            return
        handler._handle_request(msg)

    def _handle_request(self, msg: DoorRequest):
        """Process a door request for this door device."""
        requested_mode = msg.requested_mode.value

        if requested_mode == DoorMode.MODE_OPEN:
            self._door.request_open()
            logger.info("Door '%s' opening", self.ns)
        elif requested_mode == DoorMode.MODE_CLOSED:
            self._door.request_close()
            logger.info("Door '%s' closing", self.ns)
        else:
            logger.warning("Unknown door mode %d for '%s'", requested_mode, self.ns)
            return

        # Publish immediately so supervisor doesn't wait for periodic tick
        self._publish_state()

    # ------------------------------------------------------------------
    # Sim-step callbacks
    # ------------------------------------------------------------------

    def post_step(self, dt: float, stamp) -> None:
        """Publish door state periodically (~1 Hz)."""
        sim_time = self.agent.sim_core.sim_time
        if sim_time - self._last_publish_time >= self._publish_interval:
            self._publish_state()
            self._last_publish_time = sim_time

    # ------------------------------------------------------------------
    # State publishing
    # ------------------------------------------------------------------

    def _publish_state(self):
        """Build and publish a DoorState message for this door."""
        msg = DoorState()
        msg.door_time = self.node.get_clock().now().to_msg()
        msg.door_name = self.ns

        # Map Door state to RMF DoorMode
        mode = DoorMode()
        state = self._door.door_state
        if state in (SimDoorState.OPEN, SimDoorState.OPENING):
            mode.value = DoorMode.MODE_OPEN
        else:
            mode.value = DoorMode.MODE_CLOSED
        msg.current_mode = mode

        DoorHandler._shared_pub.publish(msg)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy(self) -> None:
        """Remove this handler from the shared instance map."""
        DoorHandler._instances.pop(self.ns, None)
        # Only destroy shared resources when last instance is removed
        if not DoorHandler._instances:
            if DoorHandler._shared_pub is not None:
                self.node.destroy_publisher(DoorHandler._shared_pub)
                DoorHandler._shared_pub = None
            if DoorHandler._shared_sub is not None:
                self.node.destroy_subscription(DoorHandler._shared_sub)
                DoorHandler._shared_sub = None
        logger.info("DoorHandler destroyed for '%s'", self.ns)
