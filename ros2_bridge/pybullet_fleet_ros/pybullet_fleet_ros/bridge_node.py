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
from pybullet_fleet.config_utils import load_yaml_config
from pybullet_fleet.controller import OmniController
from pybullet_fleet.types import MotionMode

from .bridge_plugin import BridgePlugin
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
        # "" (empty) is a sentinel meaning "not set" — lets us tell an explicit
        # enable_monitor_gui:=false from the unset case (so we can follow gui by default).
        self.declare_parameter("enable_monitor_gui", "", _dyn)

        # -- Step 3: Read final resolved values --
        publish_rate_param = get_float_param(self, "publish_rate", 0.0)
        enable_sim_services = get_bool_param(self, "enable_sim_services", True)
        effective_gui = get_bool_param(self, "gui", sim_cfg.get("gui", False))
        effective_physics = get_bool_param(self, "physics", sim_cfg.get("physics", False))
        effective_rtf = get_float_param(self, "target_rtf", float(sim_cfg.get("target_rtf", 1.0)))

        # Monitor GUI (tkinter window). Precedence: explicit enable_monitor_gui
        # param → YAML enable_monitor_gui → follow gui (headless gui:=false hides it).
        monitor_gui_raw = self.get_parameter("enable_monitor_gui").value
        if isinstance(monitor_gui_raw, str) and monitor_gui_raw == "":
            effective_monitor_gui = bool(sim_cfg.get("enable_monitor_gui", effective_gui))
        else:
            effective_monitor_gui = get_bool_param(self, "enable_monitor_gui", effective_gui)

        # Apply resolved values into the config dict
        sim_overrides = dict(sim_cfg)
        sim_overrides["gui"] = effective_gui
        sim_overrides["physics"] = effective_physics
        sim_overrides["target_rtf"] = effective_rtf
        sim_overrides["enable_monitor_gui"] = effective_monitor_gui

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

        # Offset between the sim (world/local) frame and the RMF (nav-graph/map)
        # frame. Georeferenced maps (e.g. campus) place the nav graph far from
        # the world origin, so robots spawn in local coords but RMF expects map
        # coords. RobotHandler shifts published poses by +offset (sim→RMF) and
        # incoming nav goals by -offset (RMF→sim). Default (0, 0) = same frame.
        off = bridge_config.get("rmf_frame_offset", [0.0, 0.0])
        self.rmf_frame_offset = (float(off[0]), float(off[1]))

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

        # Interface selection: Pattern 1 (per-robot RobotHandler, default) vs
        # Pattern 2 (one fleet-level FleetHandler with O(1) /fleet/* endpoints).
        self._fleet_handler = None
        self._batch_mode = str(bridge_config.get("fleet_interface", "per_robot")).lower() == "batch"
        if self._batch_mode:
            from .fleet_handler import FleetHandler

            self._fleet_handler = FleetHandler(self, self.sim, publish_rate)
        else:
            # Register per-robot ROS handlers for all agents already in the sim
            for agent in self.sim.agents:
                self._register_robot_handler(agent)
        actual_count = len(self.sim.agents)

        # Bridge plugins (singleton handlers loaded from config)
        self._bridge_plugins: List[BridgePlugin] = self._load_bridge_plugins(bridge_config)
        for bp in self._bridge_plugins:
            bp.on_init()

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
            f"publish_rate={publish_rate:.1f}Hz, gui={effective_gui}, "
            f"enable_monitor_gui={effective_monitor_gui}, physics={effective_physics}"
        )

    # ------------------------------------------------------------------
    # Bridge plugins (singleton handlers loaded from config)
    # ------------------------------------------------------------------

    def _load_bridge_plugins(self, bridge_config: Dict[str, Any]) -> List[BridgePlugin]:
        """Dynamically load bridge plugins from config.

        Config format::

            bridge_plugins:
              - class: my_pkg.workcell_handler.WorkcellHandler  # cross-package
                config:
                  item_search_radius: 1.0
              - type: my_plugin        # registry shorthand (same package only)
                config: {}

        Each plugin class must be a :class:`BridgePlugin` subclass.

        Note: ``type:`` shorthand only works when the plugin class module
        has already been imported (so ``__init_subclass__`` can register it).
        For cross-package plugins, use the ``class:`` dotted-path format.
        """
        from .bridge_plugin import create_bridge_plugin_from_entry

        entries: List[Dict[str, Any]] = list(bridge_config.get("bridge_plugins", []))

        plugins: List[BridgePlugin] = []
        for entry in entries:
            if not (entry.get("type") or entry.get("class")):
                continue
            label = entry.get("type") or entry.get("class", "?")
            try:
                plugin = create_bridge_plugin_from_entry(entry, self, self.sim)
                plugins.append(plugin)
                self.get_logger().info(f"Loaded bridge plugin: {label}")
            except ImportError as e:
                self.get_logger().warning(f"Bridge plugin {label} not available: {e}")
            except Exception as e:
                self.get_logger().error(f"Failed to load bridge plugin {label}: {e}", exc_info=True)

        return plugins

    # ------------------------------------------------------------------
    # Robot handler registration
    # ------------------------------------------------------------------

    def _register_robot_handler(self, agent: Agent) -> None:
        """Create a RobotHandler for *agent* and register it.

        If the agent already has a KinematicController (e.g. from
        ``controller``), it is passed to RobotHandler so that
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
        # from_params emits AGENT_SPAWNED at the end of construction, which
        # _on_agent_spawned already turns into a RobotHandler. Register here only
        # if that didn't happen (idempotent — avoids a double registration).
        if agent.object_id not in self._handlers:
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
        if self._batch_mode:
            return  # Pattern 2: the FleetHandler picks up new agents automatically
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

            # Fleet-level update (Pattern 2): aggregated /fleet/states
            if self._fleet_handler is not None:
                self._fleet_handler.post_step(dt=dt, stamp=stamp)

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
        if self._fleet_handler is not None:
            self._fleet_handler.destroy()
            self._fleet_handler = None
        for bp in self._bridge_plugins:
            bp.on_reset()
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
