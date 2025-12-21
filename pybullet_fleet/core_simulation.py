"""
core_simulation.py
Reusable core simulation logic for multi-robot PyBullet environments.
Integrates generation, management, transport, attach/detach, collision detection,
coordinate conversion, occupied judgment, transport path generation, debugging,
monitoring, and log control for various robots, pallets, and meshes.
"""

import json
import logging
import os
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

# --- All imports at the top for PEP8 compliance ---
import pybullet as p
import yaml

from pybullet_fleet.collision_visualizer import CollisionVisualizer
from pybullet_fleet.data_monitor import DataMonitor
from pybullet_fleet.sim_object import SimObject
from pybullet_fleet.agent import Agent

# Global log_level (default: 'info')
GLOBAL_LOG_LEVEL = "INFO"
if "PYBULLET_LOG_LEVEL" in os.environ:
    GLOBAL_LOG_LEVEL = os.environ["PYBULLET_LOG_LEVEL"].upper()

logging.basicConfig(level=logging.getLevelName(GLOBAL_LOG_LEVEL), format="%(asctime)s %(levelname)s %(message)s")

# Create module logger
logger = logging.getLogger(__name__)


# Log level management class


class LogLevelManager:
    @staticmethod
    def set_global_log_level(level_str: str) -> None:
        global GLOBAL_LOG_LEVEL
        level_name = str(level_str).upper()
        GLOBAL_LOG_LEVEL = level_name
        logging.getLogger().setLevel(logging.getLevelName(level_name))

    @staticmethod
    def set_log_level_from_params(params: "SimulationParams") -> None:
        level_str = getattr(params, "log_level", GLOBAL_LOG_LEVEL)
        LogLevelManager.set_global_log_level(level_str)


class SimulationParams:
    @classmethod
    def from_config(cls, config_path: str = "config.yaml") -> "SimulationParams":
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        return cls.from_dict(config)

    @classmethod
    def from_dict(cls, config: Dict[str, Any]) -> "SimulationParams":
        """
        Create SimulationParams from a configuration dictionary.

        Args:
            config: Configuration dictionary

        Returns:
            SimulationParams instance
        """
        return cls(
            speed=config.get("speed", 1.0),
            timestep=config.get("timestep", 1.0 / 10.0),
            duration=config.get("duration", 0),
            gui=config.get("gui", True),
            physics=config.get("physics", False),
            monitor=config.get("monitor", True),
            collision_check_frequency=config.get("collision_check_frequency", None),
            log_level=config.get("log_level", "warn"),
            max_steps_per_frame=config.get("max_steps_per_frame", 10),
            gui_min_fps=config.get("gui_min_fps", 30),
            # Visualizer settings
            enable_collision_shapes=config.get("enable_collision_shapes", False),
            enable_structure_transparency=config.get("enable_structure_transparency", False),
            enable_shadows=config.get("enable_shadows", True),
            enable_gui_panel=config.get("enable_gui_panel", False),  # Default: hide GUI panel
        )

    def __init__(
        self,
        num_robots: int = 10,
        speed: float = 1.0,
        timestep: float = 1.0 / 240.0,
        duration: float = 0,
        gui: bool = True,
        physics: bool = False,
        monitor: bool = True,
        collision_check_frequency: Optional[float] = None,
        log_level: str = "warn",
        max_steps_per_frame: int = 10,
        gui_min_fps: int = 30,
        enable_collision_shapes: bool = False,
        enable_structure_transparency: bool = False,
        enable_shadows: bool = True,
        enable_gui_panel: bool = False,
    ) -> None:
        self.speed = speed if speed > 0 else 1.0  # If speed <= 0, set to 1.0
        self.timestep = timestep
        self.duration = duration
        self.gui = gui
        self.physics = physics
        self.monitor = monitor
        self.collision_check_frequency = collision_check_frequency
        self.log_level = log_level
        self.max_steps_per_frame = max_steps_per_frame  # Maximum simulation steps per rendering frame
        self.gui_min_fps = gui_min_fps  # Minimum FPS for GUI responsiveness (default: 30 FPS = 33ms)
        # Visualizer settings
        self.enable_collision_shapes = enable_collision_shapes
        self.enable_structure_transparency = enable_structure_transparency
        self.enable_shadows = enable_shadows
        self.enable_gui_panel = enable_gui_panel


class MultiRobotSimulationCore:
    @classmethod
    def from_yaml(cls, yaml_path: str = "config.yaml") -> "MultiRobotSimulationCore":
        params = SimulationParams.from_config(yaml_path)
        return cls(params)

    @classmethod
    def from_dict(cls, config: Dict[str, Any]) -> "MultiRobotSimulationCore":
        """
        Create MultiRobotSimulationCore from a configuration dictionary.

        Args:
            config: Configuration dictionary

        Returns:
            MultiRobotSimulationCore instance
        """
        params = SimulationParams.from_dict(config)
        return cls(params)

    def __init__(self, params: SimulationParams, collision_color: List[float] = [0, 0, 1, 1]) -> None:
        # Initialize log level
        LogLevelManager.set_log_level_from_params(params)
        self.client: Optional[int] = None
        self.sim_objects: List[SimObject] = []  # List of all simulation objects (Agent, SimObject, etc.)
        self.robot_bodies: List[int] = []  # For code exchange (AABB, collision, etc.)
        self._last_collided: set = set()
        self._robot_original_colors: Dict[int, List[float]] = {}  # body_id: rgbaColor
        self.collision_count: int = 0
        self.step_count: int = 0
        self.sim_time: float = 0.0  # Simulation time
        self.start_time: Optional[float] = None
        self.monitor_enabled: bool = params.monitor
        self.last_monitor_update: float = 0
        self.callbacks: List[Dict[str, Any]] = []  # List of callback functions
        self.data_monitor: Optional[DataMonitor] = None
        self.collision_check_frequency: Optional[float] = params.collision_check_frequency  # If None, check every step
        self.last_collision_check: float = 0.0
        self.log_level: str = params.log_level
        self.params: SimulationParams = params
        self.collision_color: List[float] = collision_color
        self._rendering_enabled: bool = False  # Track rendering state
        self.collision_visualizer: CollisionVisualizer = CollisionVisualizer()  # Collision shape visualizer
        self._collision_shapes_enabled: bool = False  # Default: collision shapes OFF
        self._keyboard_events_registered: bool = False  # Track if keyboard events are registered
        self._original_visual_colors: Dict[Tuple[int, int], List[float]] = (
            {}
        )  # Store original colors: (body_id, link_id) -> rgba
        self._structure_body_ids: set = set()  # Set of structure body IDs (not robots) - use set for O(1) lookup
        self._structure_transparent: bool = False  # Track if structure is transparent
        self._simulation_paused: bool = False  # Track if simulation is paused
        self.setup_pybullet()
        self.setup_monitor()

    def setup_monitor(self) -> None:
        # If monitor: true and console_monitor: false, start DataMonitor
        from pybullet_fleet.data_monitor import DataMonitor

        if self.params.monitor:
            self.data_monitor = DataMonitor("PyBullet Simulation Monitor")
            self.data_monitor.start()
        else:
            self.data_monitor = None

    def register_callback(self, callback: Callable, frequency: Optional[float] = None) -> None:
        """
        Register a callback function to be called during simulation.

        Args:
            callback: Function with signature callback(sim_core, dt) -> None
                     - sim_core: Reference to MultiRobotSimulationCore instance
                     - dt: Time elapsed since last callback execution (seconds)
            frequency: Callback frequency in Hz. If None, called every simulation step.

        Example:
            def my_callback(sim_core, dt):
                for obj in sim_core.sim_objects:
                    if isinstance(obj, Agent):
                        # Update agent logic
                        pass

            sim_core.register_callback(my_callback, frequency=10.0)  # 10 Hz
        """
        self.callbacks.append({"func": callback, "frequency": frequency, "last_exec": 0.0})

    def set_collision_check_frequency(self, frequency: Optional[float] = None) -> None:
        """
        Set the frequency (Hz, number of times per second) for collision detection. If None, check every step.
        Default value is 1Hz (every second).
        """
        if frequency is None:
            frequency = 1
        self.collision_check_frequency = frequency

    def setup_pybullet(self) -> None:
        """Initialize PyBullet with GUI panels hidden."""
        self.client = p.connect(p.GUI if self.params.gui else p.DIRECT)

        # Hide all debug UI panels immediately after connection
        if self.params.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        import pybullet_data

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81 if self.params.physics else 0)
        p.setTimeStep(self.params.timestep)
        p.setRealTimeSimulation(0)
        # High-speed parameter settings
        p.setPhysicsEngineParameter(enableFileCaching=True)
        p.setPhysicsEngineParameter(deterministicOverlappingPairs=True)
        p.setPhysicsEngineParameter(allowedCcdPenetration=0.01)
        if not self.params.physics:
            p.setPhysicsEngineParameter(numSubSteps=1)
            p.setPhysicsEngineParameter(numSolverIterations=1)
            p.setPhysicsEngineParameter(enableConeFriction=False)
        self.plane_id = p.loadURDF("plane.urdf")
        # Disable rendering during setup for better performance
        if self.params.gui:
            self.disable_rendering()

    def disable_rendering(self) -> None:
        """Disable rendering during object spawning for better performance."""
        if self.params.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
            self._rendering_enabled = False
            logger.info("Rendering disabled for setup/spawning phase")

    def enable_rendering(self) -> None:
        """Enable rendering before starting simulation."""
        if self.params.gui and not self._rendering_enabled:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            self._rendering_enabled = True
            logger.info("Rendering enabled for simulation")

    def register_structure_body(self, body_id: int) -> None:
        """
        Register a body ID as part of the structure (not a robot).
        Structure bodies can be made transparent with 't' key.

        Args:
            body_id (int): PyBullet body ID to register as structure
        """
        self._structure_body_ids.add(body_id)

    def configure_visualizer(
        self,
        enable_collision_shapes: Optional[bool] = None,
        enable_structure_transparency: Optional[bool] = None,
        enable_shadows: Optional[bool] = None,
    ) -> None:
        """
        Configure PyBullet visualizer settings with keyboard control.

        Args:
            enable_collision_shapes: Initial state for collision shapes (None=use config)
            enable_structure_transparency: Initial state for structure transparency (None=use config)
            enable_shadows: Enable shadows (None=use config)

        Keyboard shortcuts (active during simulation):
        - Press 'c' to toggle collision shapes ON/OFF
        - Press 't' to toggle structure transparency ON/OFF
        """
        if not self.params.gui:
            return

        # Use config values if parameters are None
        if enable_collision_shapes is None:
            enable_collision_shapes = self.params.enable_collision_shapes
        if enable_structure_transparency is None:
            enable_structure_transparency = self.params.enable_structure_transparency
        if enable_shadows is None:
            enable_shadows = self.params.enable_shadows

        # Save original colors of all visual shapes ONCE
        if not self._original_visual_colors:
            self._save_original_visual_colors()

        # Store initial states
        self._collision_shapes_enabled = enable_collision_shapes
        self._structure_transparent = enable_structure_transparency

        # Configure shadows
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1 if enable_shadows else 0)

        # Apply initial visibility states
        # self._set_visual_shapes_visibility(self._visual_shapes_enabled)
        self._set_collision_shapes_visibility(self._collision_shapes_enabled)

        # Apply initial structure transparency
        self._set_structure_transparency(self._structure_transparent)

        # Enable keyboard event handling in step_once()
        self._keyboard_events_registered = True

        logger.info(
            "Visualizer configured: collision=%s, transparency=%s, shadows=%s",
            enable_collision_shapes,
            enable_structure_transparency,
            enable_shadows,
        )
        logger.info("Keyboard controls registered: SPACE=pause, v=visual, c=collision, t=transparency")
        print("\n[KEYBOARD CONTROLS]")
        print("  Press SPACE to pause/play simulation")
        print(f"  Press 'c' to toggle collision shapes (current: {'ON' if self._collision_shapes_enabled else 'OFF'})")
        print("  Press 't' to toggle structure transparency " f"(current: {'ON' if self._structure_transparent else 'OFF'})")
        if len(self._structure_body_ids) > 100:
            print(
                "             ⚠️  Warning: %d structure bodies detected - " "toggling may be slow (not recommended)",
                len(self._structure_body_ids),
            )

    def _save_original_visual_colors(self) -> None:
        """
        Save original colors of all visual shapes for fast restoration.
        Called once during configure_visualizer().
        """
        num_bodies = p.getNumBodies()
        for body_id in range(num_bodies):
            visual_data = p.getVisualShapeData(body_id)
            for shape in visual_data:
                link_index = shape[1]
                rgba = shape[7]  # Original RGBA color
                key = (body_id, link_index)
                self._original_visual_colors[key] = rgba

        logger.info(f"Saved original colors for {len(self._original_visual_colors)} visual shapes")

    def _handle_keyboard_events(self) -> None:
        """
        Handle keyboard events for toggling visual/collision shapes and pausing simulation.
        Called during simulation step.
        """
        if not self.params.gui:
            return

        try:
            # Get keyboard events
            keys = p.getKeyboardEvents()
        except p.error:
            # PyBullet disconnected (window closed), stop handling events
            return

        # Space key (ASCII 32) - toggle pause/play
        if ord(" ") in keys and keys[ord(" ")] & p.KEY_WAS_TRIGGERED:
            self._simulation_paused = not self._simulation_paused
            print(f"\n[PAUSE] Simulation: {'PAUSED' if self._simulation_paused else 'PLAYING'}")

        # # 'v' key (ASCII 118) - toggle visual shapes
        # if ord("v") in keys and keys[ord("v")] & p.KEY_WAS_TRIGGERED:
        #     self._visual_shapes_enabled = not self._visual_shapes_enabled
        #     self._set_visual_shapes_visibility(self._visual_shapes_enabled)
        #     print(f"\n[TOGGLE] Visual shapes: {'ON' if self._visual_shapes_enabled else 'OFF'}")

        # 'c' key (ASCII 99) - toggle collision shapes
        if ord("c") in keys and keys[ord("c")] & p.KEY_WAS_TRIGGERED:
            self._collision_shapes_enabled = not self._collision_shapes_enabled
            self._set_collision_shapes_visibility(self._collision_shapes_enabled)
            print(f"\n[TOGGLE] Collision shapes: {'ON' if self._collision_shapes_enabled else 'OFF'}")

        # 't' key (ASCII 116) - toggle structure transparency
        if ord("t") in keys and keys[ord("t")] & p.KEY_WAS_TRIGGERED:
            num_structures = len(self._structure_body_ids)
            if num_structures > 100:
                print(f"\n[TOGGLE] ⚠️  Warning: Toggling transparency for {num_structures} structure bodies...")
                print("         This may take several seconds (not recommended for large scenes)")
                print("         Consider setting 'enable_structure_transparency' in config.yaml instead")

            self._structure_transparent = not self._structure_transparent
            # self._set_transparency(self._structure_transparent)
            self._set_structure_transparency(self._structure_transparent)
            print(f"\n[TOGGLE] Structure transparency: {'ON' if self._structure_transparent else 'OFF'}")
            print(f"  Structure bodies affected: {num_structures}")

    # def _set_visual_shapes_visibility(self, visible: bool) -> None:
    #     """
    #     Set visibility of visual shapes independently from collision shapes.
    #     Uses pre-saved colors for fast restoration.
    #     Respects structure transparency setting when restoring visibility.

    #     Args:
    #         visible: True to show visual shapes, False to hide them
    #     """
    #     if not self.params.gui:
    #         return

    #     # Use saved color data for FAST toggling
    #     for key, rgba in self._original_visual_colors.items():
    #         body_id, link_index = key
    #         try:
    #             if visible:
    #                 # Check if this body is a structure and transparency is enabled
    #                 if body_id in self._structure_body_ids and self._structure_transparent:
    #                     # Restore with transparency
    #                     p.changeVisualShape(body_id, link_index, rgbaColor=[rgba[0], rgba[1], rgba[2], 0.3])
    #                 else:
    #                     # Restore original color with full opacity
    #                     p.changeVisualShape(body_id, link_index, rgbaColor=[rgba[0], rgba[1], rgba[2], 1.0])
    #             else:
    #                 # Make transparent (alpha = 0)
    #                 p.changeVisualShape(body_id, link_index, rgbaColor=[rgba[0], rgba[1], rgba[2], 0.0])
    #         except Exception:
    #             pass
    #             pass

    #     logger.info(f"Visual shapes {'enabled' if visible else 'disabled'}")

    def _set_collision_shapes_visibility(self, visible: bool) -> None:
        """
        Set visibility of collision shapes using PyBullet's configureDebugVisualizer.

        Args:
            visible: True to show collision shapes, False to hide them
        """
        if not self.params.gui:
            return

        # Use PyBullet's configureDebugVisualizer for robust collision shape toggling
        try:
            p.configureDebugVisualizer(3, 1 if visible else 0)
            logger.info(f"Collision shapes {'enabled' if visible else 'disabled'} (PyBullet visualizer)")
        except Exception as e:
            logger.warning(f"Failed to toggle collision shapes with PyBullet visualizer: {e}")

    # def _set_transparency(self, transparent: bool) -> None:
    #     try:
    #         p.configureDebugVisualizer(4, 1 if transparent else 0)
    #         logger.info(f"Transparency {'enabled' if transparent else 'disabled'} (PyBullet visualizer)")
    #     except Exception as e:
    #         logger.warning(f"Failed to toggle transparency with PyBullet visualizer: {e}")

    def _set_structure_transparency(self, transparent: bool) -> None:
        """
        Set transparency of structure bodies (not robots).
        Uses pre-saved colors with modified alpha channel.
        FAST implementation using pre-saved color dictionary.

        Args:
            transparent: True to make structure semi-transparent, False for opaque
        """
        if not self.params.gui:
            return

        alpha = 0.3 if transparent else 1.0

        print(f"[TRANSPARENCY] Applying alpha={alpha} to {len(self._structure_body_ids)} structure bodies...")

        # Apply alpha only to structure bodies using pre-saved colors (FAST)
        processed = 0
        for key, rgba in self._original_visual_colors.items():
            body_id, link_index = key

            # Only process structure bodies (O(1) lookup with set)
            if body_id not in self._structure_body_ids:
                continue

            try:
                # Apply new alpha to the original color
                p.changeVisualShape(body_id, link_index, rgbaColor=[rgba[0], rgba[1], rgba[2], alpha])
                processed += 1
            except Exception:
                pass

        print(f"[TRANSPARENCY] Complete: {processed} visual shapes updated")
        logger.info(f"Structure transparency {'enabled (alpha=0.3)' if transparent else 'disabled (alpha=1.0)'}")

    def setup_camera(
        self, camera_config: Optional[Dict[str, Any]] = None, entity_positions: Optional[List[List[float]]] = None
    ) -> None:
        """
        Set up camera view based on configuration.

        Args:
            camera_config: Dictionary with camera settings (from yaml config)
            entity_positions: List of [x, y, z] positions for auto camera calculation
        """
        if not self.params.gui:
            return  # No camera setup needed without GUI

        if camera_config is None:
            camera_config = {}

        camera_mode = camera_config.get("camera_mode", "none")

        if camera_mode == "none":
            return  # Skip camera setup

        elif camera_mode == "manual":
            # Use manual camera settings
            distance = camera_config.get("camera_distance", 10.0)
            yaw = camera_config.get("camera_yaw", 0)
            pitch = camera_config.get("camera_pitch", -89)
            target = camera_config.get("camera_target", [0, 0, 0])

            p.resetDebugVisualizerCamera(
                cameraDistance=distance, cameraYaw=yaw, cameraPitch=pitch, cameraTargetPosition=target
            )

            logger.info(f"Camera set to manual mode: distance={distance:.2f}m, yaw={yaw}°, pitch={pitch}°, target={target}")

        elif camera_mode == "auto":
            # Calculate camera from entity positions
            if entity_positions is None or len(entity_positions) == 0:
                logger.warning("Auto camera mode requested but no entity positions provided")
                return

            positions_array = np.array(entity_positions)
            center = positions_array.mean(axis=0)
            extent = positions_array.max(axis=0) - positions_array.min(axis=0)

            # Get camera settings
            view_type = camera_config.get("camera_view_type", "top_down")
            auto_scale = camera_config.get("camera_auto_scale", 0.8)

            if view_type == "top_down":
                # Top-down orthographic view
                distance = max(extent[0], extent[1]) * auto_scale
                yaw = 0
                pitch = -89
            else:
                # Perspective view
                distance = max(extent[0], extent[1], extent[2]) * auto_scale
                yaw = 45
                pitch = -30

            target = [center[0], center[1], center[2]]

            p.resetDebugVisualizerCamera(
                cameraDistance=distance, cameraYaw=yaw, cameraPitch=pitch, cameraTargetPosition=target
            )

            logger.info(f"Camera set to auto mode ({view_type}): distance={distance:.2f}m, target={target}")
            print(f"\n--- Camera View ({view_type}) ---")
            print(f"  Structure extent: [{extent[0]:.2f} x {extent[1]:.2f} x {extent[2]:.2f}] meters")
            print(f"  Center: [{center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}]")
            print(f"  Camera distance: {distance:.2f}m")
            print(f"  Camera target: [{target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}]")
            print(f"  Yaw: {yaw}°, Pitch: {pitch}°")

    def get_aabbs(self) -> List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
        return [p.getAABB(body_id) for body_id in self.robot_bodies]

    def filter_aabb_pairs(self) -> List[Tuple[int, int]]:
        aabbs = self.get_aabbs()
        pairs = []
        for i in range(len(self.robot_bodies)):
            aabb_i = aabbs[i]
            for j in range(i + 1, len(self.robot_bodies)):
                aabb_j = aabbs[j]
                if (
                    aabb_i[1][0] < aabb_j[0][0]
                    or aabb_i[0][0] > aabb_j[1][0]
                    or aabb_i[1][1] < aabb_j[0][1]
                    or aabb_i[0][1] > aabb_j[1][1]
                    or aabb_i[1][2] < aabb_j[0][2]
                    or aabb_i[0][2] > aabb_j[1][2]
                ):
                    continue
                pairs.append((i, j))
        return pairs

    def check_collisions(self, collision_color: Optional[List[float]] = None) -> List[Tuple[int, int]]:
        if collision_color is None:
            collision_color = self.collision_color
        collision_pairs = []
        collided = set()

        try:
            # Record: save initial color
            for idx, body_id in enumerate(self.robot_bodies):
                if body_id not in self._robot_original_colors:
                    # Get current color (assumed as default value since PyBullet API cannot retrieve it)
                    self._robot_original_colors[body_id] = [0.0, 0.0, 0.0, 1]
            for i, j in self.filter_aabb_pairs():
                contact_points = p.getContactPoints(self.robot_bodies[i], self.robot_bodies[j])
                if contact_points:
                    collision_pairs.append((i, j))
                    collided.add(i)
                    collided.add(j)
            # Color update (collision: blue, normal: original)
            for idx, body_id in enumerate(self.robot_bodies):
                was_collided = idx in self._last_collided
                is_collided = idx in collided
                if was_collided != is_collided:
                    if is_collided:
                        p.changeVisualShape(body_id, -1, rgbaColor=collision_color)
                    else:
                        orig_color = self._robot_original_colors.get(body_id, [0, 0, 0, 1])
                        p.changeVisualShape(body_id, -1, rgbaColor=orig_color)
            self._last_collided = collided
            self.collision_count += len(collision_pairs)
        except p.error:
            # PyBullet disconnected, skip collision checking
            pass

        return collision_pairs

    def update_monitor(self, suppress_console: bool = False) -> None:
        now = time.time()
        sim_time = self.step_count * self.params.timestep
        # --- Speed history buffer ---
        if not hasattr(self, "_speed_history"):
            self._speed_history = []  # [(real_time, sim_time)]
        self._speed_history.append((now, sim_time))
        # Keep only history within 10 seconds
        self._speed_history = [(rt, st) for rt, st in self._speed_history if now - rt <= 10.0]
        # Calculate speed for the last 10 seconds
        if len(self._speed_history) >= 2:
            rt0, st0 = self._speed_history[0]
            rt1, st1 = self._speed_history[-1]
            elapsed = rt1 - rt0
            sim_elapsed = st1 - st0
            actual_speed = sim_elapsed / elapsed if elapsed > 0 else 0
        else:
            actual_speed = 0
        elapsed_time = now - self.start_time if self.start_time else 0
        monitor_data = {
            "sim_time": sim_time,
            "real_time": elapsed_time,
            "target_speed": self.params.speed,
            "actual_speed": actual_speed,
            "time_step": self.params.timestep,
            "frequency": 1 / self.params.timestep,
            "physics": "enabled" if self.params.physics else "disabled",
            "robots": len(self.robot_bodies),
            "collisions": self.collision_count,
            "steps": self.step_count,
        }
        # Log level control (minimum: warn, info for details, debug for all output)
        log_level = self.log_level if not suppress_console else "warn"
        logging.debug("MONITOR:")
        logging.debug(json.dumps(monitor_data, indent=2, ensure_ascii=False))
        logging.info(
            "sim_time=%.2f, real_time=%.2f, speed=%.2f, collisions=%d, steps=%d",
            sim_time,
            elapsed_time,
            actual_speed,
            self.collision_count,
            self.step_count,
        )
        if self.collision_count > 0:
            logging.info(f"collisions={self.collision_count} at sim_time={sim_time:.2f}")
        # Also output to DataMonitor window
        if self.data_monitor:
            self.data_monitor.write_data(monitor_data)

    def run_simulation(self, duration: Optional[float] = None) -> None:
        if duration is None:
            duration = self.params.duration
        self.start_time = time.time()
        self.step_count = 0
        self.collision_count = 0

        # Configure visualizer after all objects are spawned
        # This ensures transparency and other visual settings are applied correctly
        self.configure_visualizer()

        # Enable rendering before starting simulation
        self.enable_rendering()
        try:
            start_time = time.time()
            last_step_process_time = 0.0  # Track processing time excluding sleep

            while True:
                loop_start = time.time()
                current_time = time.time()
                elapsed_time = current_time - start_time
                if duration != 0 and elapsed_time >= duration:
                    break

                # Check if PyBullet connection is still active (e.g., GUI window not closed)
                try:
                    p.getConnectionInfo()
                except p.error:
                    logging.info("PyBullet connection lost (GUI window closed)")
                    break

                # Calculate target and current simulation times
                target_sim_time = elapsed_time * self.params.speed
                current_sim_time = self.step_count * self.params.timestep
                time_diff = target_sim_time - current_sim_time

                actual_sleep = 0.0
                if time_diff > 0:
                    # Behind target: execute multiple steps to catch up
                    steps_needed = int(time_diff / self.params.timestep)
                    steps_needed = max(1, min(steps_needed, self.params.max_steps_per_frame))
                    for _ in range(steps_needed):
                        self.step_once()
                else:
                    # Ahead of or at target: sleep until next frame
                    # Calculate sleep time: time_diff - last processing time
                    sleep_time = abs(time_diff) - last_step_process_time

                    # Determine minimum sleep for GUI responsiveness
                    if self.params.gui:
                        min_sleep = 1.0 / self.params.gui_min_fps
                    else:
                        # For non-GUI, use timestep as minimum
                        min_sleep = self.params.timestep

                    # Sleep strategy based on how much we're ahead/behind
                    if sleep_time > 0:
                        # Ahead of target: sleep exactly the calculated time (no forced minimum)
                        actual_sleep = sleep_time
                    elif sleep_time > -min_sleep:
                        # Ahead of target but less than min_sleep: sleep minimum for GUI responsiveness
                        actual_sleep = min_sleep
                    time.sleep(actual_sleep)

                # Record processing time for this iteration (excluding sleep)
                last_step_process_time = time.time() - loop_start - actual_sleep

        except KeyboardInterrupt:
            logging.warning("Simulation interrupted by user")
        except RuntimeError as e:
            if "PyBullet disconnected" in str(e):
                logging.info("PyBullet connection lost (GUI window closed)")
            else:
                raise
        except p.error:
            logging.info("PyBullet connection lost (GUI window closed)")
        self.update_monitor()

        # Disconnect from PyBullet if still connected
        try:
            p.getConnectionInfo()
            p.disconnect()
        except p.error:
            # Already disconnected
            pass

    def step_once(self) -> None:
        # Check if PyBullet is still connected
        try:
            p.getConnectionInfo()
        except p.error:
            # PyBullet disconnected, skip this step
            return

        # Handle keyboard events for visual/collision shape toggling and pause
        if self._keyboard_events_registered:
            self._handle_keyboard_events()

        # Skip physics simulation and callbacks if paused
        if self._simulation_paused:
            return

        # Synchronize robot_bodies from sim_objects every step
        self.robot_bodies = [obj.body_id for obj in self.sim_objects]
        self.sim_time = self.step_count * self.params.timestep

        # Update all simulation objects that have update() method
        # Agent instances are automatically updated every step for movement control
        for obj in self.sim_objects:
            if isinstance(obj, Agent):
                obj.update(self.params.timestep)

        # Global callbacks (frequency control)
        for cbinfo in self.callbacks:
            freq = cbinfo.get("frequency", None)
            last_exec = cbinfo.get("last_exec", 0.0)
            interval = 1.0 / freq if freq else 0.0
            # Judge based on self.sim_time
            if freq is None or self.sim_time - last_exec >= interval:
                dt = self.sim_time - last_exec if last_exec > 0 else self.params.timestep
                cbinfo["func"](self, dt)
                cbinfo["last_exec"] = self.sim_time

        # Check if PyBullet is still connected before stepping
        if not p.isConnected():
            raise RuntimeError("PyBullet disconnected (GUI window closed)")

        p.stepSimulation()
        # Collision check frequency control
        freq = self.collision_check_frequency
        interval = 1.0 / freq if freq else 0.0
        # Collision check also judged based on self.sim_time
        if freq is None or self.sim_time - self.last_collision_check >= interval:
            self.check_collisions()
            self.last_collision_check = self.sim_time
        self.step_count += 1
        # Monitor: every step if GUI enabled, otherwise every second
        if self.monitor_enabled:
            interval = self.params.timestep if self.params.gui else 1.0
            if self.sim_time - self.last_monitor_update > interval:
                self.update_monitor()
                self.last_monitor_update = self.sim_time
