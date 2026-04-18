"""Stub dispenser/ingestor adapter for PyBulletFleet — completes instantly.

In Gazebo-based rmf_demos, ``TeleportDispenser`` and ``TeleportIngestor``
plugins handle pick-up and drop-off by teleporting items between the
workcell and the robot. Since PyBulletFleet does not simulate item
transfer, this adapter listens to requests and immediately returns
SUCCESS, allowing delivery tasks to complete.

Usage::

    ros2 run pybullet_fleet_ros workcell_adapter

Handles both ``/dispenser_requests`` and ``/ingestor_requests``.
"""

import logging

import rclpy
from rclpy.node import Node

from rmf_dispenser_msgs.msg import DispenserRequest, DispenserResult, DispenserState
from rmf_ingestor_msgs.msg import IngestorRequest, IngestorResult, IngestorState

logger = logging.getLogger(__name__)


class WorkcellAdapter(Node):
    """Stub adapter that acknowledges and completes dispenser/ingestor requests instantly.

    Publishes IDLE state for all known workcells at 1 Hz and responds
    to requests with immediate SUCCESS.
    """

    def __init__(self):
        super().__init__("pybullet_workcell_adapter")

        # Track known workcells for periodic state publishing
        self._dispensers: set[str] = set()
        self._ingestors: set[str] = set()

        # --- Dispenser ---
        self._dispenser_req_sub = self.create_subscription(
            DispenserRequest,
            "dispenser_requests",
            self._on_dispenser_request,
            10,
        )
        self._dispenser_result_pub = self.create_publisher(DispenserResult, "dispenser_results", 10)
        self._dispenser_state_pub = self.create_publisher(DispenserState, "dispenser_states", 10)

        # --- Ingestor ---
        self._ingestor_req_sub = self.create_subscription(
            IngestorRequest,
            "ingestor_requests",
            self._on_ingestor_request,
            10,
        )
        self._ingestor_result_pub = self.create_publisher(IngestorResult, "ingestor_results", 10)
        self._ingestor_state_pub = self.create_publisher(IngestorState, "ingestor_states", 10)

        # Periodic state publish
        self._timer = self.create_timer(1.0, self._publish_states)

        self.get_logger().info("Workcell adapter started (instant dispense/ingest)")

    def _on_dispenser_request(self, msg: DispenserRequest):
        """Handle a dispenser request — acknowledge and complete instantly."""
        guid = msg.request_guid
        target = msg.target_guid
        self._dispensers.add(target)
        self.get_logger().info(f"Dispenser '{target}' request {guid} — completing instantly")

        # Publish SUCCESS result
        result = DispenserResult()
        result.time = self.get_clock().now().to_msg()
        result.request_guid = guid
        result.source_guid = target
        result.status = DispenserResult.SUCCESS
        self._dispenser_result_pub.publish(result)

    def _on_ingestor_request(self, msg: IngestorRequest):
        """Handle an ingestor request — acknowledge and complete instantly."""
        guid = msg.request_guid
        target = msg.target_guid
        self._ingestors.add(target)
        self.get_logger().info(f"Ingestor '{target}' request {guid} — completing instantly")

        # Publish SUCCESS result
        result = IngestorResult()
        result.time = self.get_clock().now().to_msg()
        result.request_guid = guid
        result.source_guid = target
        result.status = IngestorResult.SUCCESS
        self._ingestor_result_pub.publish(result)

    def _publish_states(self):
        """Periodic publish of IDLE state for all known workcells."""
        now = self.get_clock().now().to_msg()

        for name in self._dispensers:
            state = DispenserState()
            state.time = now
            state.guid = name
            state.mode = DispenserState.IDLE
            state.request_guid_queue = []
            state.seconds_remaining = 0.0
            self._dispenser_state_pub.publish(state)

        for name in self._ingestors:
            state = IngestorState()
            state.time = now
            state.guid = name
            state.mode = IngestorState.IDLE
            state.request_guid_queue = []
            state.seconds_remaining = 0.0
            self._ingestor_state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = WorkcellAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
