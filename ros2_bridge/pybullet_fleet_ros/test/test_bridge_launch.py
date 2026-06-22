"""Integration tests using launch_testing.

Launches bridge_node as a ROS 2 node and verifies topics, services, and
message publishing via the rclpy API — no shell commands needed.

Run with:
    cd /rmf_demos_ws && colcon test --packages-select pybullet_fleet_ros
    # or directly:
    launch_test src/pybullet_fleet_ros/test/test_bridge_launch.py
"""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
from launch_ros.actions import Node


NUM_ROBOTS = 3
TIMEOUT_SEC = 10.0
# Absolute path because colcon test cwd is /rmf_demos_ws, not /opt/pybullet_fleet
ROBOT_URDF = "/opt/pybullet_fleet/robots/mobile_robot.urdf"


def _create_test_config() -> str:
    """Write a temporary YAML config for launch tests and return its path."""
    import os
    import tempfile

    import yaml

    config = {
        "simulation": {"gui": False, "physics": False},
        "entities": [
            {
                "name": f"robot{i}",
                "urdf_path": ROBOT_URDF,
                "pose": [i * 2.0, 0.0, 0.05],
                "motion_mode": "differential",
                "max_linear_vel": 2.0,
                "max_angular_vel": 3.0,
            }
            for i in range(NUM_ROBOTS)
        ],
    }
    fd, path = tempfile.mkstemp(suffix=".yaml", prefix="test_bridge_")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(config, f)
    return path


_TEST_CONFIG_PATH = _create_test_config()


@pytest.mark.launch_test
def generate_test_description():
    """Launch bridge_node with 3 robots, headless, for integration testing."""
    bridge_node = Node(
        package="pybullet_fleet_ros",
        executable="bridge_node",
        name="pybullet_fleet_bridge",
        parameters=[
            {
                "config_yaml": _TEST_CONFIG_PATH,
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


class TestBridgeTopics(unittest.TestCase):
    """Verify that expected topics are published after launch."""

    @classmethod
    def setUpClass(cls):
        import rclpy

        rclpy.init()
        cls.node = rclpy.create_node("test_bridge_topics")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        import rclpy

        rclpy.shutdown()

    # -- helpers --

    def _wait_for_topic(self, topic_name, msg_type, timeout=TIMEOUT_SEC):
        """Block until at least one message is received on *topic_name*."""
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

    def _wait_for_topics_exist(self, expected_topics, timeout=TIMEOUT_SEC):
        """Wait until all *expected_topics* appear in ``get_topic_names_and_types``."""
        import rclpy

        end = time.time() + timeout
        while time.time() < end:
            names = [name for name, _ in self.node.get_topic_names_and_types()]
            if all(t in names for t in expected_topics):
                return names
            rclpy.spin_once(self.node, timeout_sec=0.2)
        # Return whatever we found for assertion message
        return [name for name, _ in self.node.get_topic_names_and_types()]

    def _wait_for_services_exist(self, expected_services, timeout=TIMEOUT_SEC):
        """Wait until all *expected_services* appear in ``get_service_names_and_types``."""
        import rclpy

        end = time.time() + timeout
        while time.time() < end:
            names = [name for name, _ in self.node.get_service_names_and_types()]
            if all(s in names for s in expected_services):
                return names
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return [name for name, _ in self.node.get_service_names_and_types()]

    # -- topic existence --

    def test_clock_topic_exists(self):
        """Bridge publishes /clock."""
        topics = self._wait_for_topics_exist(["/clock"])
        self.assertIn("/clock", topics)

    def test_robot_odom_topics_exist(self):
        """Each robot has an odom topic."""
        expected = [f"/robot{i}/odom" for i in range(NUM_ROBOTS)]
        topics = self._wait_for_topics_exist(expected)
        for t in expected:
            self.assertIn(t, topics, f"Missing topic: {t}")

    def test_robot_cmd_vel_topics_exist(self):
        """Each robot has a cmd_vel topic."""
        expected = [f"/robot{i}/cmd_vel" for i in range(NUM_ROBOTS)]
        topics = self._wait_for_topics_exist(expected)
        for t in expected:
            self.assertIn(t, topics, f"Missing topic: {t}")

    def test_robot_joint_states_topics_exist(self):
        """Each robot has a joint_states topic."""
        expected = [f"/robot{i}/joint_states" for i in range(NUM_ROBOTS)]
        topics = self._wait_for_topics_exist(expected)
        for t in expected:
            self.assertIn(t, topics, f"Missing topic: {t}")

    def test_tf_topic_exists(self):
        """Bridge publishes /tf."""
        topics = self._wait_for_topics_exist(["/tf"])
        self.assertIn("/tf", topics)

    # -- services existence --

    def test_sim_services_exist(self):
        """simulation_interfaces services are advertised."""
        expected = [
            "/sim/get_simulator_features",
            "/sim/spawn_entity",
            "/sim/get_entities",
            "/sim/step_simulation",
            "/sim/get_simulation_state",
        ]
        services = self._wait_for_services_exist(expected)
        for s in expected:
            self.assertIn(s, services, f"Missing service: {s}")

    # -- message publishing --

    def test_clock_publishes_messages(self):
        """/clock publishes rosgraph_msgs/Clock messages."""
        from rosgraph_msgs.msg import Clock

        msgs = self._wait_for_topic("/clock", Clock)
        self.assertGreater(len(msgs), 0, "/clock did not publish any messages")

    def test_odom_publishes_messages(self):
        """/robot0/odom publishes Odometry messages with correct frame."""
        from nav_msgs.msg import Odometry

        msgs = self._wait_for_topic("/robot0/odom", Odometry)
        self.assertGreater(len(msgs), 0, "/robot0/odom did not publish any messages")
        self.assertEqual(msgs[0].header.frame_id, "odom")
        self.assertEqual(msgs[0].child_frame_id, "robot0/base_link")

    # -- cmd_vel → controller --

    def test_cmd_vel_moves_robot(self):
        """Publishing cmd_vel changes robot position over time."""
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        import rclpy

        # Get initial position
        initial_msgs = self._wait_for_topic("/robot0/odom", Odometry)
        self.assertGreater(len(initial_msgs), 0)
        initial_x = initial_msgs[-1].pose.pose.position.x

        # Publish forward velocity
        pub = self.node.create_publisher(Twist, "/robot0/cmd_vel", 10)
        twist = Twist()
        twist.linear.x = 1.0

        end = time.time() + 3.0
        while time.time() < end:
            pub.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # Check position changed
        later_msgs = self._wait_for_topic("/robot0/odom", Odometry, timeout=3.0)
        self.assertGreater(len(later_msgs), 0)
        later_x = later_msgs[-1].pose.pose.position.x
        self.assertGreater(later_x, initial_x, "Robot did not move forward after cmd_vel")
        self.node.destroy_publisher(pub)

    # -- service call --

    def test_get_entities_returns_robots(self):
        """GetEntities service returns all spawned robots."""
        from simulation_interfaces.srv import GetEntities

        import rclpy

        client = self.node.create_client(GetEntities, "/sim/get_entities")
        self.assertTrue(
            client.wait_for_service(timeout_sec=TIMEOUT_SEC),
            "GetEntities service not available",
        )

        request = GetEntities.Request()
        future = client.call_async(request)

        end = time.time() + TIMEOUT_SEC
        while time.time() < end and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(future.done(), "GetEntities call did not complete")
        result = future.result()
        self.assertGreaterEqual(len(result.entities), NUM_ROBOTS)
        self.node.destroy_client(client)


@launch_testing.post_shutdown_test()
class TestBridgeShutdown(unittest.TestCase):
    """Verify clean shutdown."""

    def test_exit_code(self, proc_info, bridge_node):
        """Bridge node exits cleanly (SIGINT/SIGTERM = OK)."""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, 1, -2, -15],  # 0, error, SIGINT, SIGTERM
            process=bridge_node,
        )
