"""Lift handler — RMF lift protocol bridge via handler_map.

Pure interface bridge between the RMF lift protocol and
:class:`Elevator`.  All simulation logic (attach/detach of
passengers, kinematic movement) is handled by Elevator itself.

Key behaviours:
    - Subscribes to ``/adapter_lift_requests`` (from rmf_lift_supervisor).
    - On request: calls ``Elevator.request_floor()`` which
      auto-attaches platform objects and drives the prismatic joint.
    - Doors stay **closed** while ``Elevator.is_moving`` is True;
      they open after the ``JointAction`` completes and passengers
      are detached.
    - ``post_step()``: publishes ``/lift_states`` (throttled to ~2 Hz).

Config (bridge YAML)::

    handler_map:
      Lift*: pybullet_fleet_rmf.lift_handler.LiftHandler

The handler reads ``floors``, ``initial_floor`` from the agent's ``user_data``.
"""

import logging

from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from rmf_lift_msgs.msg import LiftRequest, LiftState

from pybullet_fleet_ros.robot_handler_base import RobotHandlerBase

logger = logging.getLogger(__name__)


class LiftHandler(RobotHandlerBase):
    """Per-elevator handler that bridges RMF lift protocol to Elevator.

    This is a pure interface bridge — it translates RMF messages into
    Elevator API calls and publishes state back.  All simulation
    logic (attach/detach of passengers, kinematic Z movement) lives in
    Elevator.

    Each elevator agent gets its own LiftHandler instance.  All handlers
    share one ``/lift_states`` publisher and one ``/adapter_lift_requests``
    subscription, de-multiplexing by ``lift_name``.
    """

    # Class-level shared ROS resources (created once, shared across instances).
    _shared_pub = None  # LiftState publisher
    _shared_sub = None  # LiftRequest subscription
    _instances: dict[str, "LiftHandler"] = {}  # lift_name → handler

    def __init__(self, node, agent, tf_broadcaster=None):
        super().__init__(node, agent, tf_broadcaster)
        from pybullet_fleet.devices.elevator import Elevator

        if not isinstance(agent, Elevator):
            raise TypeError(f"LiftHandler requires Elevator, got {type(agent).__name__} " f"for '{agent.name}'")

        self._elevator: Elevator = agent
        self._session_id: str = ""
        self._current_mode: int = LiftState.MODE_AGV
        self._door_state: int = LiftState.DOOR_CLOSED
        self._last_publish_time: float = -1.0
        self._publish_interval: float = 0.5  # seconds (publish at ~2 Hz)

        # Register this instance for request dispatch
        LiftHandler._instances[self.ns] = self

        # Lazily create shared pub/sub on first instance
        if LiftHandler._shared_pub is None:
            transient_qos = QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            LiftHandler._shared_pub = node.create_publisher(LiftState, "lift_states", qos_profile=transient_qos)
            LiftHandler._shared_sub = node.create_subscription(
                LiftRequest, "adapter_lift_requests", LiftHandler._on_request, 10
            )

        # Publish initial state immediately
        self._publish_state()

        logger.info(
            "LiftHandler for '%s': floors=%s, initial=%s",
            self.ns,
            self._elevator.available_floors,
            self._elevator.current_floor,
        )

    # ------------------------------------------------------------------
    # Request handling (class-level dispatch)
    # ------------------------------------------------------------------

    @staticmethod
    def _on_request(msg: LiftRequest):
        """Dispatch incoming lift requests to the correct handler instance."""
        handler = LiftHandler._instances.get(msg.lift_name)
        if handler is None:
            logger.warning("LiftHandler: unknown lift '%s'", msg.lift_name)
            return
        handler._handle_request(msg)

    def _handle_request(self, msg: LiftRequest):
        """Process a lift request for this elevator.

        All simulation logic (attach/detach, Z movement) is delegated to
        Elevator — this method only manages RMF session state and
        door presentation.
        """
        if msg.request_type == LiftRequest.REQUEST_END_SESSION:
            logger.info("Lift '%s': session '%s' ended", self.ns, msg.session_id)
            self._session_id = ""
            self._door_state = LiftState.DOOR_CLOSED
            self._publish_state()
            return

        if msg.request_type in (LiftRequest.REQUEST_AGV_MODE, LiftRequest.REQUEST_HUMAN_MODE):
            self._session_id = msg.session_id
            self._current_mode = (
                LiftState.MODE_AGV if msg.request_type == LiftRequest.REQUEST_AGV_MODE else LiftState.MODE_HUMAN
            )

            destination = msg.destination_floor
            if destination and destination in self._elevator.available_floors:
                current = self._elevator.current_floor
                if current != destination:
                    # Close doors before moving — Elevator handles
                    # attach/detach and joint movement internally.
                    self._door_state = LiftState.DOOR_CLOSED
                    self._elevator.request_floor(destination)
                    logger.info(
                        "Lift '%s': %s -> %s (session '%s')",
                        self.ns,
                        current,
                        destination,
                        msg.session_id,
                    )
                else:
                    # Already at destination — open doors
                    self._door_state = LiftState.DOOR_OPEN
            elif destination:
                logger.warning(
                    "Lift '%s': unknown floor '%s', available: %s",
                    self.ns,
                    destination,
                    self._elevator.available_floors,
                )

            self._publish_state()

    # ------------------------------------------------------------------
    # Sim-step callbacks
    # ------------------------------------------------------------------

    def post_step(self, dt: float, stamp) -> None:
        """Manage door state transitions and publish lift state.

        No simulation logic here — Elevator.update() handles
        attach/detach and kinematic movement automatically.
        """
        # Check if elevator finished moving → open doors
        if self._door_state == LiftState.DOOR_CLOSED and not self._elevator.is_moving:
            # Elevator stopped and doors are closed — check if we should open.
            # Doors should open when the elevator has arrived (not moving)
            # and a session is active.
            if self._session_id:
                self._door_state = LiftState.DOOR_OPEN
                logger.info(
                    "Lift '%s': arrived at '%s', doors open",
                    self.ns,
                    self._elevator.current_floor,
                )
                # Force immediate publish so RMF knows doors are open
                self._publish_state()
                self._last_publish_time = self.agent.sim_core.sim_time

        # Throttled periodic publish
        sim_time = self.agent.sim_core.sim_time
        if sim_time - self._last_publish_time >= self._publish_interval:
            self._publish_state()
            self._last_publish_time = sim_time

    # ------------------------------------------------------------------
    # State publishing
    # ------------------------------------------------------------------

    def _publish_state(self):
        """Build and publish a LiftState message for this elevator."""
        msg = LiftState()
        msg.lift_time = self.node.get_clock().now().to_msg()
        msg.lift_name = self.ns
        msg.available_floors = self._elevator.available_floors
        msg.current_floor = self._elevator.current_floor
        msg.destination_floor = self._elevator.target_floor
        msg.door_state = self._door_state
        msg.motion_state = LiftState.MOTION_UP if self._elevator.is_moving else LiftState.MOTION_STOPPED
        msg.available_modes = [LiftState.MODE_AGV, LiftState.MODE_HUMAN]
        msg.current_mode = self._current_mode
        msg.session_id = self._session_id
        LiftHandler._shared_pub.publish(msg)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy(self) -> None:
        """Remove this handler from the shared instance map."""
        LiftHandler._instances.pop(self.ns, None)
        # Only destroy shared resources when last instance is removed
        if not LiftHandler._instances:
            if LiftHandler._shared_pub is not None:
                self.node.destroy_publisher(LiftHandler._shared_pub)
                LiftHandler._shared_pub = None
            if LiftHandler._shared_sub is not None:
                self.node.destroy_subscription(LiftHandler._shared_sub)
                LiftHandler._shared_sub = None
        logger.info("LiftHandler destroyed for '%s'", self.ns)
