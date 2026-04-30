"""Main bridge node — wraps MultiRobotSimulationCore as a ROS 2 node.

The simulation loop (``run_simulation()``) runs on a background thread.
Bridge publishes state via EventBus callbacks (``POST_STEP``).

Usage::

    ros2 run pybullet_fleet_ros bridge_node \
        --ros-args -p config_yaml:=/path/to/config.yaml
"""

import logging
import threading
from typing import Any, Dict, List, Type

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster

from pybullet_fleet import (
    Agent,
    AgentSpawnParams,
    MultiRobotSimulationCore,
    SimEvents,
)
from pybullet_fleet.config_utils import load_yaml_config, resolve_class
from pybullet_fleet.controller import OmniController
from pybullet_fleet.types import MotionMode

from .bridge_plugin_base import BridgePluginBase
from .conversions import sim_time_to_ros_time
from .handler_registry import HandlerMap, load_handler_map_from_config, resolve_handler_classes
from .param_utils import get_bool_param, get_float_param
from .robot_handler import RobotHandler
from .robot_handler_base import RobotHandlerBase

logger = logging.getLogger(__name__)


class BridgeNode(Node):
    """ROS 2 node that drives PyBulletFleet simulation and exposes per-robot topics.

    Parameters (ROS):
        config_yaml (str): Path to PyBulletFleet YAML config file (required).
        publish_rate (float): State publish rate in Hz (default: 1/timestep, i.e. every step).
        gui (bool): Enable PyBullet GUI window (overrides YAML).
        physics (bool): Enable physics simulation (overrides YAML).
        target_rtf (float): Target real-time factor (overrides YAML).
        enable_sim_services (bool): Create simulation_interfaces services.
    """

    #: Default handler class(es) for all robots.  Override in subclasses
    #: or pass ``handler_map`` to use per-robot handlers.
    handler_class: Type[RobotHandler] = RobotHandler

    def __init__(self, handler_map: HandlerMap | None = None) -> None:
        super().__init__("pybullet_fleet_bridge")
        self._handler_map: HandlerMap = handler_map or {}

        # -- Step 1: Read config_yaml and load YAML config --
        self.declare_parameter("config_yaml", "")
        config_yaml = self.get_parameter("config_yaml").get_parameter_value().string_value

        if not config_yaml:
            self.get_logger().info("config_yaml not set, starting with defaults")
            bridge_config: Dict[str, Any] = {}
        else:
            bridge_config = load_yaml_config(config_yaml)

        sim_cfg = bridge_config.get("simulation", {})

        # -- Step 2: Declare params with YAML values as defaults --
        # ROS 2 parameter overrides (from launch file or -p flag)
        # automatically take precedence over these defaults.
        # Priority: code default → YAML config → command-line parameter.
        from rcl_interfaces.msg import ParameterDescriptor

        _dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("publish_rate", float(sim_cfg.get("publish_rate", 0.0)), _dyn)
        self.declare_parameter("gui", sim_cfg.get("gui", False), _dyn)
        self.declare_parameter("physics", sim_cfg.get("physics", False), _dyn)
        self.declare_parameter("target_rtf", float(sim_cfg.get("target_rtf", 1.0)), _dyn)
        self.declare_parameter("enable_sim_services", True, _dyn)

        # -- Step 3: Read final resolved values --
        publish_rate_param = get_float_param(self, "publish_rate", 0.0)
        enable_sim_services = get_bool_param(self, "enable_sim_services", True)
        effective_gui = get_bool_param(self, "gui", sim_cfg.get("gui", False))
        effective_physics = get_bool_param(self, "physics", sim_cfg.get("physics", False))
        effective_rtf = get_float_param(self, "target_rtf", float(sim_cfg.get("target_rtf", 1.0)))

        # Apply resolved values into the config dict
        sim_overrides = dict(sim_cfg)
        sim_overrides["gui"] = effective_gui
        sim_overrides["physics"] = effective_physics
        sim_overrides["target_rtf"] = effective_rtf

        # Load handler_map from config (if present) and merge with
        # programmatic handler_map.  Config entries have lower priority
        # (programmatic handler_map wins on key conflict).
        config_handler_map = bridge_config.get("handler_map", {})
        if config_handler_map:
            config_map = load_handler_map_from_config(config_handler_map)
            # Programmatic handler_map takes priority over config
            config_map.update(self._handler_map)
            self._handler_map = config_map

        # Delegate world loading & robot spawning to core
        full_config = dict(bridge_config)
        full_config["simulation"] = sim_overrides
        self.sim = MultiRobotSimulationCore.from_dict(full_config)

        # Resolve publish_rate: 0 (default) = every simulation step
        timestep = self.sim.params.timestep
        publish_rate = publish_rate_param if publish_rate_param > 0 else 1.0 / timestep

        # TF broadcaster (shared by all handlers)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Robot handlers (object_id → list of RobotHandlerBase instances)
        self._handlers: Dict[int, List["RobotHandlerBase"]] = {}

        # /clock publisher and throttle state
        self._clock_pub = self.create_publisher(Clock, "/clock", 10)
        self._last_clock_time: float = -1.0
        self._clock_min_interval: float = 1.0 / publish_rate  # throttle /clock

        # Register ROS handlers for all agents already in the sim
        for agent in self.sim.agents:
            self._register_robot_handler(agent)
        actual_count = len(self.sim.agents)

        # Bridge plugins (singleton handlers loaded from config)
        self._bridge_plugins: List[BridgePluginBase] = self._load_bridge_plugins(bridge_config)

        # Auto-register/unregister handlers when agents spawn/despawn
        self.sim.events.on(SimEvents.AGENT_SPAWNED, self._on_agent_spawned)
        self.sim.events.on(SimEvents.AGENT_REMOVED, self._on_agent_removed)

        # Drive publish from simulation step
        self.sim.events.on(SimEvents.PRE_STEP, self._on_pre_step)
        self.sim.events.on(SimEvents.POST_STEP, self._on_post_step)

        # simulation_interfaces services
        self._sim_services = None
        if enable_sim_services:
            from .sim_services import SimServices

            self._sim_services = SimServices(self, self.sim, self)

        # Start simulation loop on a daemon thread.
        # rclpy.spin() runs on the main thread for action server callbacks.
        self._sim_thread = threading.Thread(target=self._run_simulation_loop, daemon=True, name="sim_loop")
        self._sim_thread.start()

        self.get_logger().info(
            f"BridgeNode started: {actual_count} robots, "
            f"dt={timestep:.4f}s, target_rtf={effective_rtf}, "
            f"publish_rate={publish_rate:.1f}Hz, gui={effective_gui}, physics={effective_physics}"
        )

    # ------------------------------------------------------------------
    # Bridge plugins (singleton handlers loaded from config)
    # ------------------------------------------------------------------

    def _load_bridge_plugins(self, bridge_config: Dict[str, Any]) -> List[BridgePluginBase]:
        """Dynamically load bridge plugins from config.

        Config format::

            bridge:
              bridge_plugins:
                - class: pybullet_fleet_ros.workcell_handler.WorkcellHandler
                  config:
                    item_search_radius: 1.0

        Each plugin class must be a :class:`BridgePluginBase` subclass.
        """
        entries: List[Dict[str, Any]] = list(bridge_config.get("bridge_plugins", []))
        import sys

        print(f"[bridge_plugins] Found {len(entries)} entries: {entries}", file=sys.stderr, flush=True)

        plugins: List[BridgePluginBase] = []
        for entry in entries:
            cls_path = entry.get("class", "")
            if not cls_path:
                continue
            try:
                cls = resolve_class(cls_path)
                if not (isinstance(cls, type) and issubclass(cls, BridgePluginBase)):
                    raise TypeError(
                        f"Bridge plugin class must be a BridgePluginBase subclass, "
                        f"got {cls!r}. Did you inherit from BridgePluginBase?"
                    )
                plugin = cls(self, self.sim, entry.get("config", {}))
                plugins.append(plugin)
                print(f"[bridge_plugins] Loaded: {cls_path}", file=sys.stderr, flush=True)
                logger.info("Loaded bridge plugin: %s", cls_path)
            except ImportError as e:
                print(f"[bridge_plugins] Import error: {cls_path}: {e}", file=sys.stderr, flush=True)
                logger.warning("Bridge plugin %s not available: %s", cls_path, e)
            except Exception as e:
                print(f"[bridge_plugins] FAILED: {cls_path}: {e}", file=sys.stderr, flush=True)
                import traceback as tb

                tb.print_exc(file=sys.stderr)
                logger.error("Failed to load bridge plugin %s: %s", cls_path, e)

        print(f"[bridge_plugins] Total loaded: {len(plugins)}", file=sys.stderr, flush=True)
        return plugins

    # ------------------------------------------------------------------
    # Robot handler registration
    # ------------------------------------------------------------------

    def _register_robot_handler(self, agent: Agent) -> None:
        """Create a RobotHandler for *agent* and register it.

        If the agent already has a KinematicController (e.g. from
        ``controller_config``), it is passed to RobotHandler so that
        ``cmd_vel`` messages drive the controller.
        If no controller is present, an :class:`OmniController`
        is attached **only for omnidirectional agents**. Differential-drive
        agents use TPI-based navigation directly and do not need a controller
        for ``cmd_vel`` (they use ``goal_pose`` / ``path`` topics instead).
        """
        motion_ctrl = agent._controller
        if motion_ctrl is None and agent._motion_mode == MotionMode.OMNIDIRECTIONAL:
            motion_ctrl = OmniController()
            agent.set_controller(motion_ctrl)
        classes = resolve_handler_classes(agent, self._handler_map, [self.handler_class])
        handlers = []
        for cls in classes:
            handler = cls(
                self,
                agent,
                tf_broadcaster=self._tf_broadcaster,
            )
            handlers.append(handler)
            if cls is not RobotHandler:
                logger.info("Using custom handler %s for '%s'", cls.__name__, agent.name)
        self._handlers[agent.object_id] = handlers

    def spawn_robot(self, spawn_params: AgentSpawnParams) -> Agent:
        """Spawn a robot dynamically (e.g., via SpawnEntity service)."""
        agent = Agent.from_params(spawn_params, sim_core=self.sim)
        self._register_robot_handler(agent)
        self.get_logger().info(f"Spawned robot '{agent.name}' (id={agent.object_id})")
        return agent

    def remove_robot(self, object_id: int) -> bool:
        """Remove a robot and its ROS interfaces."""
        handlers = self._handlers.pop(object_id, None)
        if handlers is None:
            return False
        agent = handlers[0].agent
        for h in handlers:
            h.destroy()
        self.sim.remove_object(agent)
        self.get_logger().info(f"Removed robot '{agent.name}' (id={object_id})")
        return True

    def _on_agent_spawned(self, agent, **kwargs):
        """Auto-register RobotHandler when a new agent spawns."""
        if agent.object_id not in self._handlers:
            self._register_robot_handler(agent)
            n = len(self._handlers[agent.object_id])
            self.get_logger().info(f"Registered {n} handler(s) for {agent.name}")

    def _on_agent_removed(self, agent, **kwargs):
        """Auto-cleanup when agent is removed."""
        handlers = self._handlers.pop(agent.object_id, None)
        if handlers is not None:
            for h in handlers:
                h.destroy()
            self.get_logger().info(f"Removed handler(s) for {agent.name}")

    def _run_simulation_loop(self) -> None:
        """Run ``sim.run_simulation()`` on a daemon thread."""
        try:
            self.sim.run_simulation()
        except Exception as e:
            if "Not connected" in str(e) or "physics server" in str(e):
                self.get_logger().warn("PyBullet disconnected, sim thread exiting")
            else:
                self.get_logger().error(f"Simulation error: {e}")

    def _on_pre_step(self, dt, sim_time, **kwargs) -> None:
        """PRE_STEP — delegate to handler.pre_step()."""
        stamp = sim_time_to_ros_time(sim_time)
        for handlers in self._handlers.values():
            for h in handlers:
                h.pre_step(dt=dt, stamp=stamp)

    def _on_post_step(self, dt, sim_time, **kwargs) -> None:
        """POST_STEP — publish /clock and delegate to handler.post_step()."""
        try:
            stamp = sim_time_to_ros_time(sim_time)

            # Publish /clock (throttled)
            if sim_time - self._last_clock_time >= self._clock_min_interval:
                clock_msg = Clock()
                clock_msg.clock = stamp
                self._clock_pub.publish(clock_msg)
                self._last_clock_time = sim_time

            # Per-robot update (odom, TF, joint_states, diagnostics)
            for handlers in self._handlers.values():
                for h in handlers:
                    h.post_step(dt=dt, stamp=stamp)

            # Bridge plugins (workcell handler, etc.)
            for bp in self._bridge_plugins:
                bp.post_step(sim_time)
        except Exception as e:
            if "Not connected" in str(e) or "physics server" in str(e):
                return
            raise

    def reset(self) -> None:
        """Destroy all robot handlers and reset the simulation."""
        for handlers in list(self._handlers.values()):
            for h in handlers:
                h.destroy()
        self._handlers.clear()
        for bp in self._bridge_plugins:
            bp.destroy()
        self._bridge_plugins.clear()
        self.sim.reset()
        self.get_logger().info("BridgeNode reset: all handlers destroyed, simulation reset")

    @property
    def handlers(self) -> Dict[int, List["RobotHandlerBase"]]:
        """Access robot handlers (used by SimServices for spawn/delete)."""
        return self._handlers


def main(args=None):
    # Enable Python logging so that logger.info() from WorkcellHandler,
    # DoorHandler, etc. is visible in docker logs.
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(levelname)s: %(message)s")
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
