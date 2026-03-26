"""Main bridge node — wraps MultiRobotSimulationCore as a ROS 2 node.

The ROS timer drives ``step_once()`` calls; ``initialize_simulation()``
prepares the sim without entering the blocking loop.
RTF is controlled by the timer period.

Usage::

    ros2 run pybullet_fleet_ros bridge_node --ros-args -p num_robots:=5
"""

import logging
from typing import Dict

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster

from pybullet_fleet import (
    Agent,
    AgentSpawnParams,
    MultiRobotSimulationCore,
    Pose,
    SimulationParams,
)
from pybullet_fleet.config_utils import load_yaml_config
from pybullet_fleet.controller import OmniController
from pybullet_fleet.types import MotionMode

from .conversions import sim_time_to_ros_time
from .robot_handler import RobotHandler

logger = logging.getLogger(__name__)


class BridgeNode(Node):
    """ROS 2 node that drives PyBulletFleet simulation and exposes per-robot topics.

    Parameters (ROS):
        config_yaml (str): Path to PyBulletFleet YAML config file.
        num_robots (int): Number of mobile robots to spawn at startup.
        robot_urdf (str): URDF path for default mobile robot.
        publish_rate (float): State publish rate in Hz.
        gui (bool): Enable PyBullet GUI window.
        physics (bool): Enable physics simulation.
        target_rtf (float): Target real-time factor (1.0 = real time).
        enable_sim_services (bool): Create simulation_interfaces services.
    """

    def __init__(self) -> None:
        super().__init__("pybullet_fleet_bridge")

        # Declare parameters
        self.declare_parameter("config_yaml", "")
        self.declare_parameter("num_robots", 1)
        self.declare_parameter("robot_urdf", "robots/mobile_robot.urdf")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("gui", False)
        self.declare_parameter("physics", False)
        self.declare_parameter("target_rtf", 1.0)
        self.declare_parameter("enable_sim_services", True)

        # Read parameters
        config_yaml = self.get_parameter("config_yaml").get_parameter_value().string_value
        num_robots = self.get_parameter("num_robots").get_parameter_value().integer_value
        robot_urdf = self.get_parameter("robot_urdf").get_parameter_value().string_value
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        gui = self.get_parameter("gui").get_parameter_value().bool_value
        physics = self.get_parameter("physics").get_parameter_value().bool_value
        target_rtf = self.get_parameter("target_rtf").get_parameter_value().double_value
        enable_sim_services = self.get_parameter("enable_sim_services").get_parameter_value().bool_value

        # Create simulation core
        if config_yaml:
            bridge_config = load_yaml_config(config_yaml)
            sim_overrides = bridge_config.get("simulation", {})
            self.sim = MultiRobotSimulationCore(
                SimulationParams(
                    gui=sim_overrides.get("gui", gui),
                    physics=sim_overrides.get("physics", physics),
                    monitor=False,
                    target_rtf=sim_overrides.get("target_rtf", target_rtf),
                )
            )
        else:
            bridge_config = {}
            self.sim = MultiRobotSimulationCore(
                SimulationParams(
                    gui=gui,
                    physics=physics,
                    monitor=False,
                    target_rtf=target_rtf,
                )
            )

        # Initialize simulation (prepare state without entering blocking loop)
        self.sim.initialize_simulation()

        # TF broadcaster (shared by all handlers)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Robot handlers (object_id → RobotHandler)
        self._handlers: Dict[int, "RobotHandler"] = {}

        # Spawn robots: prefer config_yaml robots list, fallback to num_robots
        robots_config = bridge_config.get("robots", [])
        if robots_config:
            robots_params = [AgentSpawnParams.from_dict(d) for d in robots_config]
        else:
            robots_params = [
                AgentSpawnParams(
                    urdf_path=robot_urdf,
                    initial_pose=Pose.from_xyz(i * 2.0, 0.0, 0.05),
                    motion_mode=MotionMode.DIFFERENTIAL,
                    max_linear_vel=2.0,
                    max_angular_vel=3.0,
                    name=f"robot{i}",
                )
                for i in range(num_robots)
            ]

        with self.sim.batch_spawn():
            for params in robots_params:
                agent = Agent.from_params(params, sim_core=self.sim)
                self._register_handler(agent)
        actual_count = len(robots_params)

        # /clock publisher
        self._clock_pub = self.create_publisher(Clock, "/clock", 10)

        # simulation_interfaces services
        self._sim_services = None
        if enable_sim_services:
            from .sim_services import SimServices

            self._sim_services = SimServices(self, self.sim, self)

        # Simulation step timer — period controls RTF
        dt = self.sim.params.timestep
        effective_rtf = target_rtf if target_rtf > 0 else 0.0
        if effective_rtf > 0:
            timer_period = dt / effective_rtf
        else:
            timer_period = 0.0  # As fast as possible
        self._step_timer = self.create_timer(timer_period, self._step_callback)

        # Publish timer (may be slower than step rate)
        publish_period = 1.0 / publish_rate
        self._publish_timer = self.create_timer(publish_period, self._publish_callback)

        self.get_logger().info(
            f"BridgeNode started: {actual_count} robots, dt={dt:.4f}s, "
            f"rtf={target_rtf}, publish_rate={publish_rate}Hz, gui={gui}, physics={physics}"
        )

    def _register_handler(self, agent: Agent) -> None:
        """Create a RobotHandler for *agent* and register it.

        If the agent already has a KinematicController (e.g. from
        ``controller_config``), it is passed to RobotHandler so that
        ``cmd_vel`` messages drive the controller.

        If no controller is present, an :class:`OmniController`
        is attached **only for omnidirectional agents**. Differential-drive
        agents use TPI-based navigation directly and do not need a controller
        for ``cmd_vel`` (they use ``goal_pose`` / ``path`` topics instead).

        This is the **post-hook** that adds ROS interfaces after
        ``spawn_from_config`` / ``Agent.from_params`` creates the
        simulation-layer agent.
        """
        vel_ctrl = agent._controller
        if vel_ctrl is None and agent._motion_mode == MotionMode.OMNIDIRECTIONAL:
            vel_ctrl = OmniController()
            agent.set_controller(vel_ctrl)
        handler = RobotHandler(
            self,
            agent,
            vel_controller=vel_ctrl,
            tf_broadcaster=self._tf_broadcaster,
        )
        self._handlers[agent.object_id] = handler

    def spawn_robot(self, spawn_params: AgentSpawnParams) -> Agent:
        """Spawn a robot dynamically (e.g., via SpawnEntity service)."""
        agent = Agent.from_params(spawn_params, sim_core=self.sim)
        self._register_handler(agent)
        self.get_logger().info(f"Spawned robot '{agent.name}' (id={agent.object_id})")
        return agent

    def remove_robot(self, object_id: int) -> bool:
        """Remove a robot and its ROS interfaces."""
        handler = self._handlers.pop(object_id, None)
        if handler is None:
            return False
        handler.destroy()
        self.sim.remove_object(handler.agent)
        self.get_logger().info(f"Removed robot '{handler.agent.name}' (id={object_id})")
        return True

    def _step_callback(self) -> None:
        """Called every sim timestep — advance simulation."""
        # Apply pending cmd_vel commands
        for handler in self._handlers.values():
            handler.apply_cmd_vel()

        # Step the simulation
        self.sim.step_once()

        # Publish /clock
        clock_msg = Clock()
        clock_msg.clock = sim_time_to_ros_time(self.sim.sim_time)
        self._clock_pub.publish(clock_msg)

    def _publish_callback(self) -> None:
        """Called at publish_rate — publish state for all robots."""
        stamp = sim_time_to_ros_time(self.sim.sim_time)
        for handler in self._handlers.values():
            handler.publish_state(stamp)

    @property
    def handlers(self) -> Dict[int, "RobotHandler"]:
        """Access robot handlers (used by SimServices for spawn/delete)."""
        return self._handlers


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    # MultiThreadedExecutor allows action server execute callbacks
    # (which use time.sleep polling) to run on separate threads without
    # blocking the simulation step timer.
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
