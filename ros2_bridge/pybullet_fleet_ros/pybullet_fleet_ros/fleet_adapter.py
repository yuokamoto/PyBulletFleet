#!/usr/bin/env python3
"""Open-RMF EasyFullControl fleet adapter for PyBulletFleet.

Bridges Open-RMF task dispatching with PyBulletFleet's ROS 2 bridge
using the **EasyFullControl** API (introduced in rmf_fleet_adapter 2.3).

Each simulated robot is registered via ``RobotCallbacks`` that forward
navigate/stop/action commands as ``NavigateToPose`` action goals
to the bridge node's per-robot action servers.

Usage::

    ros2 run pybullet_fleet_ros fleet_adapter \\
        -c /path/to/fleet_config.yaml -n /path/to/nav_graph

    # Or via launch file (preferred):
    ros2 launch pybullet_fleet_ros office_pybullet.launch.py
"""

import argparse
import asyncio
import faulthandler
import logging
import math
import sys
import threading
import time

import rclpy
import rclpy.node
import yaml
from rclpy.duration import Duration
from rclpy.parameter import Parameter

logger = logging.getLogger(__name__)

try:
    import rmf_adapter
    from rmf_adapter import Adapter
    import rmf_adapter.easy_full_control as rmf_easy

    HAS_RMF = True
except ImportError:
    HAS_RMF = False

if HAS_RMF:
    from rclpy.qos import qos_profile_system_default
    from rclpy.qos import QoSDurabilityPolicy as Durability
    from rclpy.qos import QoSHistoryPolicy as History
    from rclpy.qos import QoSProfile
    from rclpy.qos import QoSReliabilityPolicy as Reliability
    from rmf_fleet_msgs.msg import ClosedLanes
    from rmf_fleet_msgs.msg import LaneRequest
    from rmf_fleet_msgs.msg import ModeRequest
    from rmf_fleet_msgs.msg import RobotMode
    from rmf_fleet_msgs.msg import SpeedLimitRequest

    from .robot_client_api import RobotClientAPI


def main(argv=sys.argv):
    """Entry point for fleet_adapter executable."""
    faulthandler.enable()  # Print traceback on segfault (rmf_adapter / PyBullet C extensions)
    rclpy.init(args=argv)

    if not HAS_RMF:
        node = rclpy.node.Node("pybullet_fleet_adapter")
        node.get_logger().error("rmf_adapter not available. " "Build rmf_fleet_adapter_python from source (see docs/README).")
        node.destroy_node()
        rclpy.shutdown()
        return

    rmf_adapter.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="PyBulletFleet Open-RMF EasyFullControl adapter",
    )
    parser.add_argument(
        "-c",
        "--config_file",
        type=str,
        required=True,
        help="Path to the fleet config YAML file",
    )
    parser.add_argument(
        "-n",
        "--nav_graph",
        type=str,
        default="",
        help="Path to the nav_graph for this fleet (optional for PyBulletFleet)",
    )
    parser.add_argument(
        "-sim",
        "--use_sim_time",
        action="store_true",
        help="Use sim time (default: false)",
    )
    args = parser.parse_args(args_without_ros[1:])

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    # Parse config for EasyFullControl
    fleet_config = rmf_easy.FleetConfiguration.from_config_files(config_path, nav_graph_path)
    if fleet_config is None:
        print(f"[ERROR] Failed to parse config file [{config_path}]")
        rclpy.shutdown()
        return

    # Parse the YAML in Python to get PyBulletFleet-specific settings
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    fleet_name = config_yaml.get("rmf_fleet", {}).get("name", "pybullet_fleet")

    # ROS 2 node for the command handle
    node = rclpy.node.Node(f"{fleet_name}_command_handle")

    adapter = Adapter.make(f"{fleet_name}_fleet_adapter")
    if adapter is None:
        node.get_logger().error("Unable to initialize fleet adapter. " "Please ensure RMF Schedule Node is running.")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Enable sim time
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])
        adapter.node.use_sim_time()

    adapter.start()
    time.sleep(1.0)

    # Forward server_uri to EasyFullControl so the fleet adapter pushes
    # fleet-state / fleet-log updates to the rmf-web API server via WebSocket.
    node.declare_parameter("server_uri", "")
    server_uri = node.get_parameter("server_uri").get_parameter_value().string_value
    if server_uri:
        fleet_config.server_uri = server_uri
        node.get_logger().info(f"Fleet adapter server_uri: {server_uri}")
    else:
        node.get_logger().info("No server_uri configured — fleet state will not be pushed to API server")

    fleet_handle = adapter.add_easy_fleet(fleet_config)
    fleet_handle.more().set_planner_cache_reset_size(2500)

    # Build RobotClientAPI wrappers for each robot
    pybullet_config = config_yaml.get("pybullet_fleet", {})
    update_period = 1.0 / pybullet_config.get("robot_state_update_frequency", 10.0)

    robots: dict[str, "RobotAdapter"] = {}
    for robot_name in fleet_config.known_robots:
        robot_config = fleet_config.get_known_robot_configuration(robot_name)
        api = RobotClientAPI(robot_name=robot_name, node=node)
        robots[robot_name] = RobotAdapter(robot_name, robot_config, node, api, fleet_handle)

    node.get_logger().info(f"EasyFullControl fleet adapter started: {len(robots)} robots")

    # Background update loop
    reassign_task_interval = config_yaml.get("rmf_fleet", {}).get(
        "reassign_task_interval", 60
    )  # seconds — periodically re-optimize task assignments

    def update_loop():
        asyncio.set_event_loop(asyncio.new_event_loop())
        last_task_replan = node.get_clock().now()
        while rclpy.ok():
            now = node.get_clock().now()
            update_jobs = []
            for robot in robots.values():
                update_jobs.append(_update_robot(robot))
            asyncio.get_event_loop().run_until_complete(asyncio.wait(update_jobs))

            # Periodically reassign dispatched tasks for better allocation
            interval_sec = (now.nanoseconds - last_task_replan.nanoseconds) / 1e9
            if interval_sec > reassign_task_interval:
                fleet_handle.more().reassign_dispatched_tasks()
                last_task_replan = now

            next_wakeup = now + Duration(nanoseconds=update_period * 1e9)
            while node.get_clock().now() < next_wakeup:
                time.sleep(0.001)

    update_thread = threading.Thread(target=update_loop, daemon=True)
    update_thread.start()

    # Connect to ROS 2 topics for lane closure, speed limits, and mode changes
    _connections = ros_connections(node, robots, fleet_handle)

    # Spin
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)
    try:
        rclpy_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy_executor.shutdown()
        rclpy.shutdown()


class RobotAdapter:
    """Per-robot state bridge between EasyFullControl and PyBulletFleet.

    Follows the ``rmf_demos`` RobotAdapter pattern with:
    - Retry-until-success for all API commands
    - Teleop support (override_schedule tracking)
    - Dock fallback (treat as regular navigate)
    - Thread-safe command cancellation
    """

    def __init__(self, name: str, configuration, node, api, fleet_handle):
        self.name = name
        self.execution = None
        self.teleoperation = None
        self.cmd_id = 0
        self.update_handle = None
        self.configuration = configuration
        self.node = node
        self.api = api
        self.fleet_handle = fleet_handle
        self.override = None
        self.issue_cmd_thread = None
        self.cancel_cmd_event = threading.Event()

    def update(self, state, data):
        """Update RMF with robot state and check command completion."""
        activity_identifier = None
        if self.execution:
            if data.is_command_completed(self.cmd_id):
                self.execution.finished()
                self.execution = None
                self.teleoperation = None
            else:
                activity_identifier = self.execution.identifier

        if self.teleoperation is not None:
            self.teleoperation.update(data)

        if self.update_handle is not None:
            self.update_handle.update(state, activity_identifier)

    def make_callbacks(self):
        """Create RobotCallbacks for EasyFullControl registration."""
        return rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(destination, execution),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(category, description, execution),
        )

    def navigate(self, destination, execution):
        """Navigate to a destination via NavigateToPose action.

        If a dock name is specified, logs a warning and falls back to
        regular navigation (PyBulletFleet has no docking server).
        """
        self.cmd_id += 1
        self.execution = execution
        self.node.get_logger().info(
            f"Commanding [{self.name}] to navigate to "
            f"{destination.position} on map [{destination.map}]: "
            f"cmd_id {self.cmd_id}"
        )

        if destination.dock is not None:
            self.node.get_logger().info(
                f"[{self.name}] Dock '{destination.dock}' requested, " "treating as regular navigate (docking not supported)"
            )

        self.attempt_cmd_until_success(
            cmd=self.api.navigate,
            args=(
                self.cmd_id,
                destination.position,
                destination.map,
                destination.speed_limit,
            ),
        )

    def stop(self, activity):
        """Cancel current navigation with retry."""
        if self.execution is not None:
            if self.execution.identifier.is_same(activity):
                self.execution = None
                self.cmd_id += 1
                self.attempt_cmd_until_success(
                    cmd=self.api.stop,
                    args=(),
                )

    def execute_action(self, category: str, description: dict, execution):
        """Handle custom actions dispatched by RMF.

        Supported categories:
        - ``teleop``: Track robot position and override RMF schedule.
        - ``delivery_pickup`` / ``delivery_dropoff``: Log and finish
          (pick/drop not yet wired to PyBulletFleet actions).
        - ``clean``: Log and finish (no cleaning simulation).
        - Others: Log warning and finish immediately.
        """
        self.cmd_id += 1
        self.execution = execution

        if category == "teleop":
            self.teleoperation = Teleoperation(execution)
            self.node.get_logger().info(f"[{self.name}] Teleop mode activated")
            # No api.toggle_teleop needed — sim robots can be moved
            # externally; we just track position via override_schedule.
        elif category == "delivery_pickup":
            self.node.get_logger().info(f"[{self.name}] Delivery pickup requested (no-op in sim), " "finishing.")
            execution.finished()
            self.execution = None
        elif category == "delivery_dropoff":
            self.node.get_logger().info(f"[{self.name}] Delivery dropoff requested (no-op in sim), " "finishing.")
            execution.finished()
            self.execution = None
        elif category == "clean":
            zone = description.get("zone", "unknown")
            self.node.get_logger().info(f"[{self.name}] Clean zone '{zone}' requested " "(no cleaning simulation), finishing.")
            execution.finished()
            self.execution = None
        else:
            self.node.get_logger().warn(f"[{self.name}] Action '{category}' not supported, " "finishing.")
            execution.finished()
            self.execution = None

    def finish_action(self):
        """Finish the current action (triggered by ModeRequest callback).

        Typically used by human operators to signal teleop completion
        via the rmf-web dashboard.
        """
        if self.execution is not None:
            self.execution.finished()
            self.execution = None
            if self.teleoperation:
                self.node.get_logger().info(f"[{self.name}] Teleop finished via ModeRequest")
                self.teleoperation = None

    def attempt_cmd_until_success(self, cmd, args):
        """Retry a command until it succeeds (returns True).

        Runs in a background thread so the adapter loop is not blocked.
        Previous retry threads are cancelled before starting a new one.
        """
        self.cancel_cmd_attempt()

        def loop():
            while not cmd(*args):
                self.node.get_logger().warn(f"Failed to contact robot [{self.name}], retrying...")
                if self.cancel_cmd_event.wait(1.0):
                    break

        self.issue_cmd_thread = threading.Thread(target=loop, daemon=True)
        self.issue_cmd_thread.start()

    def cancel_cmd_attempt(self):
        """Cancel any in-flight retry thread."""
        if self.issue_cmd_thread is not None:
            self.cancel_cmd_event.set()
            if self.issue_cmd_thread.is_alive():
                self.issue_cmd_thread.join()
                self.issue_cmd_thread = None
        self.cancel_cmd_event.clear()


class Teleoperation:
    """Track a teleoperated robot and keep RMF schedule up to date.

    When the robot moves more than 0.1 m from its last reported position,
    ``override_schedule`` is called so that RMF's traffic system knows
    the robot's actual trajectory and can avoid collisions.
    """

    def __init__(self, execution):
        self.execution = execution
        self.override = None
        self.last_position = None

    def update(self, data):
        """Called every update cycle with the latest robot state."""
        if self.last_position is None:
            self.override = self.execution.override_schedule(data.map, [data.position], 30.0)
            self.last_position = data.position
        else:
            dx = self.last_position[0] - data.position[0]
            dy = self.last_position[1] - data.position[1]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > 0.1:
                self.override = self.execution.override_schedule(data.map, [data.position], 30.0)
                self.last_position = data.position


def ros_connections(node, robots, fleet_handle):
    """Subscribe to ROS 2 topics for lane closure, speed limits, and mode changes.

    These subscriptions allow the rmf-web dashboard and external tools to:
    - Open / close navigation lanes
    - Set per-lane speed limits
    - Signal teleop completion via ModeRequest
    """
    fleet_name = fleet_handle.more().fleet_name

    transient_qos = QoSProfile(
        history=History.KEEP_LAST,
        depth=1,
        reliability=Reliability.RELIABLE,
        durability=Durability.TRANSIENT_LOCAL,
    )

    closed_lanes_pub = node.create_publisher(ClosedLanes, "closed_lanes", qos_profile=transient_qos)

    closed_lanes: set[int] = set()

    def lane_request_cb(msg):
        if msg.fleet_name and msg.fleet_name != fleet_name:
            return

        if msg.open_lanes:
            node.get_logger().info(f"Opening lanes: {msg.open_lanes}")
        if msg.close_lanes:
            node.get_logger().info(f"Closing lanes: {msg.close_lanes}")

        fleet_handle.more().open_lanes(msg.open_lanes)
        fleet_handle.more().close_lanes(msg.close_lanes)

        for lane_idx in msg.close_lanes:
            closed_lanes.add(lane_idx)
        for lane_idx in msg.open_lanes:
            closed_lanes.discard(lane_idx)

        state_msg = ClosedLanes()
        state_msg.fleet_name = fleet_name
        state_msg.closed_lanes = list(closed_lanes)
        closed_lanes_pub.publish(state_msg)

    def speed_limit_request_cb(msg):
        if msg.fleet_name is None or msg.fleet_name != fleet_name:
            return

        requests = []
        for limit in msg.speed_limits:
            request = rmf_adapter.fleet_update_handle.SpeedLimitRequest(limit.lane_index, limit.speed_limit)
            requests.append(request)
        fleet_handle.more().limit_lane_speeds(requests)
        fleet_handle.more().remove_speed_limits(msg.remove_limits)

    def mode_request_cb(msg):
        if msg.fleet_name is None or msg.fleet_name != fleet_name or msg.robot_name is None:
            return

        if msg.mode.mode == RobotMode.MODE_IDLE:
            robot = robots.get(msg.robot_name)
            if robot is None:
                return
            robot.finish_action()

    lane_request_sub = node.create_subscription(
        LaneRequest,
        "lane_closure_requests",
        lane_request_cb,
        qos_profile=qos_profile_system_default,
    )

    speed_limit_request_sub = node.create_subscription(
        SpeedLimitRequest,
        "speed_limit_requests",
        speed_limit_request_cb,
        qos_profile=qos_profile_system_default,
    )

    mode_request_sub = node.create_subscription(
        ModeRequest,
        "action_execution_notice",
        mode_request_cb,
        qos_profile=qos_profile_system_default,
    )

    return [lane_request_sub, speed_limit_request_sub, mode_request_sub]


def _parallel(f):
    """Run function in thread pool executor."""

    def run_in_parallel(*args, **kwargs):
        return asyncio.get_event_loop().run_in_executor(None, f, *args, **kwargs)

    return run_in_parallel


@_parallel
def _update_robot(robot: RobotAdapter):
    """Fetch robot state and push update to RMF."""
    data = robot.api.get_data()
    if data is None:
        return

    state = rmf_easy.RobotState(data.map, data.position, data.battery_soc)

    if robot.update_handle is None:
        robot.update_handle = robot.fleet_handle.add_robot(robot.name, state, robot.configuration, robot.make_callbacks())
        return

    robot.update(state, data)


if __name__ == "__main__":
    main(sys.argv)
