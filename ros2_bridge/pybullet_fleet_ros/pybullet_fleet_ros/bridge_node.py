"""Main bridge node — wraps MultiRobotSimulationCore as a ROS 2 node.

The ROS timer drives ``step_once()`` calls; ``initialize_simulation()``
prepares the sim without entering the blocking loop.
RTF is controlled by the timer period.

Usage::

    ros2 run pybullet_fleet_ros bridge_node \
        --ros-args -p config_yaml:=/path/to/config.yaml
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
        config_yaml (str): Path to PyBulletFleet YAML config file (required).
        publish_rate (float): State publish rate in Hz.
        gui (bool): Enable PyBullet GUI window (overrides YAML).
        physics (bool): Enable physics simulation (overrides YAML).
        target_rtf (float): Target real-time factor (overrides YAML).
        enable_sim_services (bool): Create simulation_interfaces services.
    """

    def __init__(self) -> None:
        super().__init__("pybullet_fleet_bridge")

        # Declare parameters — use dynamic_typing so LaunchConfiguration
        # string overrides don't conflict with the default type.
        from rcl_interfaces.msg import ParameterDescriptor

        _dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("config_yaml", "")
        self.declare_parameter("publish_rate", 50.0, _dyn)
        self.declare_parameter("gui", False, _dyn)
        self.declare_parameter("physics", False, _dyn)
        self.declare_parameter("target_rtf", 1.0, _dyn)
        self.declare_parameter("enable_sim_services", True, _dyn)

        # Read parameters (handle string overrides from launch files)
        config_yaml = self.get_parameter("config_yaml").get_parameter_value().string_value
        publish_rate = self._get_float("publish_rate", 50.0)
        gui = self._get_bool("gui", False)
        physics = self._get_bool("physics", False)
        target_rtf = self._get_float("target_rtf", 1.0)
        enable_sim_services = self._get_bool("enable_sim_services", True)

        if not config_yaml:
            raise RuntimeError(
                "config_yaml parameter is required. "
                "Usage: ros2 run pybullet_fleet_ros bridge_node "
                "--ros-args -p config_yaml:=/path/to/config.yaml"
            )

        # Create simulation core
        # ROS parameters (from launch/CLI) override YAML config values.
        bridge_config = load_yaml_config(config_yaml)
        sim_overrides = bridge_config.get("simulation", {})
        effective_gui = gui if gui else sim_overrides.get("gui", False)
        effective_physics = physics if physics else sim_overrides.get("physics", False)
        effective_rtf = target_rtf if target_rtf != 1.0 else sim_overrides.get("target_rtf", 1.0)

        # Apply ROS parameter overrides into the config dict
        sim_overrides = dict(sim_overrides)
        sim_overrides["gui"] = effective_gui
        sim_overrides["physics"] = effective_physics
        sim_overrides["target_rtf"] = effective_rtf
        sim_overrides["monitor"] = False

        # Delegate world loading & robot spawning to core
        full_config = dict(bridge_config)
        full_config["simulation"] = sim_overrides
        self.sim = MultiRobotSimulationCore.from_dict(full_config)

        # Set up camera view (must be called after from_dict)
        self.sim.setup_camera()

        # Enable rendering — from_dict disables it during setup for
        # performance.  run_simulation() re-enables it, but BridgeNode
        # drives step_once() via ROS timer, so we must do it here.
        self.sim.enable_rendering()

        # TF broadcaster (shared by all handlers)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Robot handlers (object_id → RobotHandler)
        self._handlers: Dict[int, "RobotHandler"] = {}

        # Register ROS handlers for all agents already in the sim
        for agent in self.sim.agents:
            self._register_handler(agent)
        actual_count = len(self.sim.agents)

        # EventBus: auto-register/unregister handlers when agents spawn/despawn
        self.sim.events.on("agent_spawned", self._on_agent_spawned)
        self.sim.events.on("agent_removed", self._on_agent_removed)

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

    # ------------------------------------------------------------------
    # Parameter helpers — handle string overrides from launch files
    # ------------------------------------------------------------------

    def _get_float(self, name: str, default: float) -> float:
        """Read a parameter as float, coercing from string if needed."""
        val = self.get_parameter(name).value
        if isinstance(val, (int, float)):
            return float(val)
        if isinstance(val, str):
            try:
                return float(val)
            except ValueError:
                pass
        return default

    def _get_bool(self, name: str, default: bool) -> bool:
        """Read a parameter as bool, coercing from string if needed."""
        val = self.get_parameter(name).value
        if isinstance(val, bool):
            return val
        if isinstance(val, str):
            return val.lower() in ("true", "1", "yes")
        return default

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

    def _on_agent_spawned(self, agent):
        """Auto-register RobotHandler when agent spawns via EventBus."""
        if agent.object_id not in self._handlers:
            self._register_handler(agent)
            self.get_logger().info(f"EventBus: auto-registered handler for {agent.name}")

    def _on_agent_removed(self, agent):
        """Auto-cleanup when agent is removed via EventBus."""
        handler = self._handlers.pop(agent.object_id, None)
        if handler is not None:
            handler.destroy()
            self.get_logger().info(f"EventBus: removed handler for {agent.name}")

    def _step_callback(self) -> None:
        """Called every sim timestep — advance simulation."""
        try:
            # Apply pending cmd_vel commands
            for handler in self._handlers.values():
                handler.apply_cmd_vel()

            # Step the simulation
            self.sim.step_once()

            # Publish /clock
            clock_msg = Clock()
            clock_msg.clock = sim_time_to_ros_time(self.sim.sim_time)
            self._clock_pub.publish(clock_msg)
        except Exception as e:
            if "Not connected" in str(e) or "physics server" in str(e):
                self.get_logger().warn("PyBullet disconnected, requesting shutdown")
                raise SystemExit(0)
            raise

    def _publish_callback(self) -> None:
        """Called at publish_rate — publish state for all robots."""
        try:
            stamp = sim_time_to_ros_time(self.sim.sim_time)
            for handler in self._handlers.values():
                handler.publish_state(stamp)
        except Exception as e:
            if "Not connected" in str(e) or "physics server" in str(e):
                self.get_logger().warn("PyBullet disconnected, skipping publish")
                return
            raise

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
