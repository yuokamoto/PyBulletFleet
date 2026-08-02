"""Render fleet state snapshots as lightweight RViz markers."""

from __future__ import annotations

from pybullet_fleet_msgs.msg import FleetState
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class FleetRvizNode(Node):
    """Publish one cube marker per robot from the Fleet ROS state stream."""

    def __init__(self) -> None:
        super().__init__("pybullet_fleet_fleet_rviz")
        self._marker_pub = self.create_publisher(MarkerArray, "/fleet/markers", 10)
        self._state_sub = self.create_subscription(FleetState, "/fleet/states", self._on_fleet_state, 10)
        self._marker_ids_by_name: dict[str, int] = {}
        self._next_marker_id = 0

    def _marker_id(self, robot_name: str) -> int:
        marker_id = self._marker_ids_by_name.get(robot_name)
        if marker_id is None:
            marker_id = self._next_marker_id
            self._marker_ids_by_name[robot_name] = marker_id
            self._next_marker_id += 1
        return marker_id

    def _on_fleet_state(self, state: FleetState) -> None:
        markers = MarkerArray()
        active_ids: set[int] = set()
        active_names = set()
        for robot in state.robots:
            marker_id = self._marker_id(robot.name)
            active_ids.add(marker_id)
            active_names.add(robot.name)
            markers.markers.append(self._robot_marker(marker_id, state, robot))

        previous_ids = set(self._marker_ids_by_name.values())
        for marker_id in previous_ids - active_ids:
            marker = Marker()
            marker.header = state.header
            marker.ns = "pybullet_fleet"
            marker.id = marker_id
            marker.action = Marker.DELETE
            markers.markers.append(marker)

        self._marker_ids_by_name = {
            name: marker_id for name, marker_id in self._marker_ids_by_name.items() if name in active_names
        }
        self._marker_pub.publish(markers)

    @staticmethod
    def _robot_marker(marker_id: int, state: FleetState, robot) -> Marker:
        marker = Marker()
        marker.header = state.header
        marker.ns = "pybullet_fleet"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = robot.pose
        marker.scale.x = 0.45
        marker.scale.y = 0.30
        marker.scale.z = 0.15
        marker.color.a = 1.0
        marker.color.r = 0.15 if robot.is_moving else 0.45
        marker.color.g = 0.75 if robot.is_moving else 0.45
        marker.color.b = 1.0
        return marker


def main() -> None:
    import rclpy

    rclpy.init()
    node = FleetRvizNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
