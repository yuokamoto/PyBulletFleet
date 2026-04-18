"""Stub door adapter for PyBulletFleet — opens doors instantly on request.

In Gazebo-based rmf_demos, a Gazebo plugin animates doors and publishes
``/door_states``. Since PyBulletFleet does not simulate door physics,
this adapter listens to ``/adapter_door_requests`` and immediately
publishes the requested state to ``/door_states``, allowing robots to
pass through without waiting.

Usage::

    ros2 run pybullet_fleet_ros door_adapter

All doors start in CLOSED state and transition instantly when requested.
"""

import logging

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from rmf_door_msgs.msg import DoorMode, DoorRequest, DoorState

logger = logging.getLogger(__name__)


class DoorAdapter(Node):
    """Publishes door states in response to adapter_door_requests.

    Maintains a dict of known door states and publishes them at 1 Hz.
    When a request arrives, the door transitions instantly.
    """

    def __init__(self):
        super().__init__("pybullet_door_adapter")

        # Track door states: door_name -> DoorMode value
        self._door_states: dict[str, int] = {}

        # Subscribe to adapter_door_requests (from door_supervisor)
        self._request_sub = self.create_subscription(
            DoorRequest,
            "adapter_door_requests",
            self._on_door_request,
            10,
        )

        # Publish door states
        transient_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_pub = self.create_publisher(DoorState, "door_states", qos_profile=transient_qos)

        # Publish all known door states at 1 Hz
        self._timer = self.create_timer(1.0, self._publish_states)

        self.get_logger().info("Door adapter started (instant open/close)")

    def _on_door_request(self, msg: DoorRequest):
        """Handle a door open/close request — transition instantly."""
        door_name = msg.door_name
        requested_mode = msg.requested_mode.value

        if requested_mode == DoorMode.MODE_OPEN:
            self._door_states[door_name] = DoorMode.MODE_OPEN
            self.get_logger().info(f"Door '{door_name}' opened")
        elif requested_mode == DoorMode.MODE_CLOSED:
            self._door_states[door_name] = DoorMode.MODE_CLOSED
            self.get_logger().info(f"Door '{door_name}' closed")
        else:
            self.get_logger().warn(f"Unknown door mode {requested_mode} for '{door_name}'")
            return

        # Publish immediately so the supervisor doesn't have to wait
        self._publish_door_state(door_name)

    def _publish_door_state(self, door_name: str):
        """Publish state for a single door."""
        msg = DoorState()
        now = self.get_clock().now().to_msg()
        msg.door_time = now
        msg.door_name = door_name
        mode = DoorMode()
        mode.value = self._door_states.get(door_name, DoorMode.MODE_CLOSED)
        msg.current_mode = mode
        self._state_pub.publish(msg)

    def _publish_states(self):
        """Periodic publish of all known door states."""
        for door_name in self._door_states:
            self._publish_door_state(door_name)


def main(args=None):
    rclpy.init(args=args)
    node = DoorAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
