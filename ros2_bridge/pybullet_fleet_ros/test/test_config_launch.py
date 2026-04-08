"""Launch test for config_yaml with mixed robots (mobile + cube).

Verifies that bridge_node correctly spawns robots from a YAML config file
with heterogeneous robot types, and that the 6-DoF OmniController
correctly handles angular velocities (wx, wy, wz) on the cube.

Run with:
    cd /opt/bridge_ws && colcon test --packages-select pybullet_fleet_ros
"""

import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import yaml
from launch_ros.actions import Node


TIMEOUT_SEC = 10.0
ROBOT_URDF = "/opt/pybullet_fleet/robots/mobile_robot.urdf"
CUBE_URDF = "/opt/pybullet_fleet/robots/simple_cube.urdf"


def _write_test_config(tmp_dir: str) -> str:
    """Write a temporary bridge config YAML with mixed robots."""
    config = {
        "simulation": {"gui": False, "physics": False},
        "robots": [
            {"name": "robot0", "urdf_path": ROBOT_URDF, "pose": [0.0, 0.0, 0.05]},
            {"name": "cube0", "urdf_path": CUBE_URDF, "pose": [2.0, 0.0, 0.5]},
        ],
    }
    config_path = f"{tmp_dir}/test_bridge_config.yaml"
    with open(config_path, "w") as f:
        yaml.dump(config, f)
    return config_path


# Create temp config before launch
_tmp_dir = tempfile.mkdtemp()
_config_path = _write_test_config(_tmp_dir)


@pytest.mark.launch_test
def generate_test_description():
    """Launch bridge_node with config_yaml (mixed robots), headless."""
    bridge_node = Node(
        package="pybullet_fleet_ros",
        executable="bridge_node",
        name="pybullet_fleet_bridge",
        parameters=[
            {
                "config_yaml": _config_path,
                "gui": False,
                "physics": False,
                "publish_rate": 10.0,
                "target_rtf": 0.0,
            }
        ],
        output="screen",
    )
    return launch.LaunchDescription(
        [
            bridge_node,
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"bridge_node": bridge_node}


class TestConfigDrivenBridge(unittest.TestCase):
    """Verify config_yaml spawns mixed robots correctly."""

    @classmethod
    def setUpClass(cls):
        import rclpy

        rclpy.init()
        cls.node = rclpy.create_node("test_config_bridge")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        import rclpy

        rclpy.shutdown()

    def _wait_for_topics_exist(self, expected_topics, timeout=TIMEOUT_SEC):
        import rclpy

        end = time.time() + timeout
        while time.time() < end:
            names = [name for name, _ in self.node.get_topic_names_and_types()]
            if all(t in names for t in expected_topics):
                return names
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return [name for name, _ in self.node.get_topic_names_and_types()]

    def _wait_for_topic(self, topic_name, msg_type, timeout=TIMEOUT_SEC):
        msgs = []
        sub = self.node.create_subscription(msg_type, topic_name, msgs.append, 10)
        try:
            end = time.time() + timeout
            import rclpy

            while time.time() < end and not msgs:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            return msgs
        finally:
            self.node.destroy_subscription(sub)

    # -- topic existence for both robot types --

    def test_robot0_topics_exist(self):
        """Mobile robot robot0 has odom and cmd_vel topics."""
        expected = ["/robot0/odom", "/robot0/cmd_vel"]
        topics = self._wait_for_topics_exist(expected)
        for t in expected:
            self.assertIn(t, topics, f"Missing topic: {t}")

    def test_cube0_topics_exist(self):
        """Cube cube0 has odom and cmd_vel topics."""
        expected = ["/cube0/odom", "/cube0/cmd_vel"]
        topics = self._wait_for_topics_exist(expected)
        for t in expected:
            self.assertIn(t, topics, f"Missing topic: {t}")

    def test_cube0_initial_position(self):
        """cube0 spawns at configured position (2.0, 0.0, 0.5)."""
        from nav_msgs.msg import Odometry

        msgs = self._wait_for_topic("/cube0/odom", Odometry)
        self.assertGreater(len(msgs), 0, "/cube0/odom did not publish")
        pos = msgs[0].pose.pose.position
        self.assertAlmostEqual(pos.x, 2.0, places=1)
        self.assertAlmostEqual(pos.z, 0.5, places=1)

    def test_cube0_6dof_angular_velocity(self):
        """Publishing angular.x (wx) to cube0 changes its roll orientation.

        This verifies OmniController 6-DoF: body-frame wx
        produces roll rotation via quaternion integration.
        """
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        import rclpy

        # Get initial orientation
        initial_msgs = self._wait_for_topic("/cube0/odom", Odometry)
        self.assertGreater(len(initial_msgs), 0)
        q0 = initial_msgs[-1].pose.pose.orientation

        # Publish angular.x (roll) velocity
        pub = self.node.create_publisher(Twist, "/cube0/cmd_vel", 10)
        twist = Twist()
        twist.angular.x = 1.0  # wx = 1.0 rad/s (roll)

        end = time.time() + 3.0
        while time.time() < end:
            pub.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # Check orientation changed
        later_msgs = self._wait_for_topic("/cube0/odom", Odometry, timeout=3.0)
        self.assertGreater(len(later_msgs), 0)
        q1 = later_msgs[-1].pose.pose.orientation

        # Quaternion should have changed from initial (roll rotation)
        # Check qx or qy changed significantly (roll rotation changes qx)
        quat_diff = abs(q1.x - q0.x) + abs(q1.y - q0.y) + abs(q1.w - q0.w)
        self.assertGreater(quat_diff, 0.01, "Cube orientation did not change after angular.x cmd_vel")

        self.node.destroy_publisher(pub)

    def test_cube0_vertical_movement(self):
        """Publishing linear.z to cube0 moves it vertically (3D omni)."""
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        import rclpy

        # Get initial z
        initial_msgs = self._wait_for_topic("/cube0/odom", Odometry)
        self.assertGreater(len(initial_msgs), 0)
        initial_z = initial_msgs[-1].pose.pose.position.z

        # Publish upward velocity
        pub = self.node.create_publisher(Twist, "/cube0/cmd_vel", 10)
        twist = Twist()
        twist.linear.z = 1.0  # Move up

        end = time.time() + 2.0
        while time.time() < end:
            pub.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # Check z increased
        later_msgs = self._wait_for_topic("/cube0/odom", Odometry, timeout=3.0)
        self.assertGreater(len(later_msgs), 0)
        later_z = later_msgs[-1].pose.pose.position.z
        self.assertGreater(later_z, initial_z + 0.1, "Cube did not move upward after linear.z cmd_vel")

        self.node.destroy_publisher(pub)


@launch_testing.post_shutdown_test()
class TestConfigBridgeShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info, bridge_node):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, 1, -2, -15],
            process=bridge_node,
        )
