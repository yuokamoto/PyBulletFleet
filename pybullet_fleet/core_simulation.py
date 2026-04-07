"""
core_simulation.py
Reusable core simulation logic for multi-robot PyBullet environments.
Integrates generation, management, transport, attach/detach, collision detection,
coordinate conversion, occupied judgment, transport path generation, debugging,
monitoring, and log control for various robots, pallets, and meshes.
"""

import logging
import math
import os
import statistics
import time
import tracemalloc  # Memory profiling (imported once at module level)
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Set, Tuple, Union, cast

import numpy as np

# --- All imports at the top for PEP8 compliance ---
import pybullet as p
import pybullet_data

import yaml

from pybullet_fleet.collision_visualizer import CollisionVisualizer  # deprecated, unused
from pybullet_fleet.data_monitor import DataMonitor
from pybullet_fleet.logging_utils import get_lazy_logger
from pybullet_fleet.sim_object import SimObject
from pybullet_fleet.agent import Agent
from pybullet_fleet.types import SpatialHashCellSizeMode, CollisionDetectionMethod, CollisionMode
from pybullet_fleet._defaults import SIMULATION as _SIM_D

# Global log_level (default: 'info')
GLOBAL_LOG_LEVEL = "INFO"
if "PYBULLET_LOG_LEVEL" in os.environ:
    GLOBAL_LOG_LEVEL = os.environ["PYBULLET_LOG_LEVEL"].upper()

logging.basicConfig(level=logging.getLevelName(GLOBAL_LOG_LEVEL), format="%(asctime)s %(levelname)s %(message)s")

# Create module logger (LazyLogger: avoids expensive f-string evaluation when log level is disabled)
logger = get_lazy_logger(__name__)


@dataclass
class SimulationParams:
    """Simulation configuration parameters.

    All fields have sensible defaults for headless kinematics-based simulation.
    Use ``from_config`` / ``from_dict`` to load from YAML files.
    """

    target_rtf: float = _SIM_D["target_rtf"]  # target_rtf=0 means maximum speed (no sleep)
    timestep: float = _SIM_D["timestep"]
    duration: float = _SIM_D["duration"]
    gui: bool = _SIM_D["gui"]
    physics: bool = _SIM_D["physics"]
    monitor: bool = _SIM_D["monitor"]
    enable_monitor_gui: bool = _SIM_D["enable_monitor_gui"]
    collision_check_frequency: Optional[float] = None
    log_level: str = _SIM_D["log_level"]
    max_steps_per_frame: int = _SIM_D["max_steps_per_frame"]
    gui_min_fps: int = _SIM_D["gui_min_fps"]
    # Visualizer settings
    enable_collision_shapes: bool = _SIM_D["enable_collision_shapes"]
    enable_structure_transparency: bool = _SIM_D["enable_structure_transparency"]
    enable_shadows: bool = _SIM_D["enable_shadows"]
    enable_gui_panel: bool = _SIM_D["enable_gui_panel"]
    ignore_static_collision: bool = _SIM_D["ignore_static_collision"]
    enable_time_profiling: bool = _SIM_D["enable_time_profiling"]
    profiling_interval: int = _SIM_D["profiling_interval"]
    enable_memory_profiling: bool = _SIM_D["enable_memory_profiling"]
    enable_collision_color_change: bool = _SIM_D["enable_collision_color_change"]
    spatial_hash_cell_size_mode: SpatialHashCellSizeMode = SpatialHashCellSizeMode.AUTO_INITIAL
    spatial_hash_cell_size: Optional[float] = None  # Fixed cell size (for mode=CONSTANT)
    collision_detection_method: Optional[CollisionDetectionMethod] = None  # Auto-select based on physics
    collision_margin: float = _SIM_D["collision_margin"]
    multi_cell_threshold: float = _SIM_D["multi_cell_threshold"]
    enable_floor: bool = _SIM_D["enable_floor"]
    camera_config: Optional[Dict[str, Any]] = None  # Camera configuration from config file
    model_paths: Optional[List[str]] = None  # Additional directories to scan for URDF/mesh models
    # Window size parameters
    window_width: int = _SIM_D["window_width"]
    window_height: int = _SIM_D["window_height"]
    # DataMonitor window parameters
    monitor_width: int = _SIM_D["monitor_width"]
    monitor_height: int = _SIM_D["monitor_height"]
    monitor_x: int = _SIM_D["monitor_x"]
    monitor_y: int = _SIM_D["monitor_y"]

    def __post_init__(self) -> None:
        # Auto-select collision detection method based on physics mode if not explicitly set
        if self.collision_detection_method is None:
            # Physics ON: use CONTACT_POINTS (actual contact manifold)
            # Physics OFF: use CLOSEST_POINTS (distance-based, kinematics-safe)
            self.collision_detection_method = (
                CollisionDetectionMethod.CONTACT_POINTS if self.physics else CollisionDetectionMethod.CLOSEST_POINTS
            )
        # Ensure camera_config is always a dict (avoid mutable default)
        if self.camera_config is None:
            self.camera_config = {}
        # Ensure model_paths is always a list (avoid mutable default)
        if self.model_paths is None:
            self.model_paths = []

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
            target_rtf=config.get("target_rtf", _SIM_D["target_rtf"]),
            timestep=config.get("timestep", _SIM_D["timestep"]),
            duration=config.get("duration", _SIM_D["duration"]),
            gui=config.get("gui", _SIM_D["gui"]),
            physics=config.get("physics", _SIM_D["physics"]),
            monitor=config.get("monitor", _SIM_D["monitor"]),
            enable_monitor_gui=config.get("enable_monitor_gui", _SIM_D["enable_monitor_gui"]),
            collision_check_frequency=config.get("collision_check_frequency", _SIM_D["collision_check_frequency"]),
            log_level=config.get("log_level", _SIM_D["log_level"]),
            max_steps_per_frame=config.get("max_steps_per_frame", _SIM_D["max_steps_per_frame"]),
            gui_min_fps=config.get("gui_min_fps", _SIM_D["gui_min_fps"]),
            # Visualizer settings
            enable_collision_shapes=config.get("enable_collision_shapes", _SIM_D["enable_collision_shapes"]),
            enable_structure_transparency=config.get("enable_structure_transparency", _SIM_D["enable_structure_transparency"]),
            enable_shadows=config.get("enable_shadows", _SIM_D["enable_shadows"]),
            enable_gui_panel=config.get("enable_gui_panel", _SIM_D["enable_gui_panel"]),
            ignore_static_collision=config.get("ignore_static_collision", _SIM_D["ignore_static_collision"]),
            enable_time_profiling=config.get("enable_time_profiling", _SIM_D["enable_time_profiling"]),
            profiling_interval=config.get("profiling_interval", _SIM_D["profiling_interval"]),
            enable_memory_profiling=config.get("enable_memory_profiling", _SIM_D["enable_memory_profiling"]),
            enable_collision_color_change=config.get("enable_collision_color_change", _SIM_D["enable_collision_color_change"]),
            spatial_hash_cell_size_mode=SpatialHashCellSizeMode(
                config.get("spatial_hash_cell_size_mode", _SIM_D["spatial_hash_cell_size_mode"])
            ),
            spatial_hash_cell_size=config.get("spatial_hash_cell_size", _SIM_D["spatial_hash_cell_size"]),
            # Collision detection method - let __post_init__ auto-select if not in config
            collision_detection_method=(
                CollisionDetectionMethod(config["collision_detection_method"])
                if "collision_detection_method" in config
                else None
            ),
            collision_margin=config.get("collision_margin", _SIM_D["collision_margin"]),
            multi_cell_threshold=config.get("multi_cell_threshold", _SIM_D["multi_cell_threshold"]),
            enable_floor=config.get("enable_floor", _SIM_D["enable_floor"]),
            camera_config=config.get("camera", _SIM_D["camera_config"]),  # Camera configuration
            model_paths=config.get("model_paths", _SIM_D["model_paths"]),  # Additional model directories
            # Window size parameters
            window_width=config.get("window_width", _SIM_D["window_width"]),
            window_height=config.get("window_height", _SIM_D["window_height"]),
            # DataMonitor window parameters
            monitor_width=config.get("monitor_width", _SIM_D["monitor_width"]),
            monitor_height=config.get("monitor_height", _SIM_D["monitor_height"]),
            monitor_x=config.get("monitor_x", _SIM_D["monitor_x"]),
            monitor_y=config.get("monitor_y", _SIM_D["monitor_y"]),
        )


class MultiRobotSimulationCore:
    # Precomputed neighbor offsets for spatial hashing (class constants)
    _NEIGHBOR_OFFSETS_3D = tuple(
        (dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
    )  # 27 neighbors for 3D collision detection

    _NEIGHBOR_OFFSETS_2D = tuple(
        (dx, dy, 0) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
    )  # 9 neighbors for 2D collision detection (Z=0 only)

    _EMPTY_SET: frozenset = frozenset()  # Reusable empty set for dict.get() default (avoids per-call allocation)

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

    def __init__(self, params: SimulationParams, collision_color: Optional[List[float]] = None) -> None:
        # Initialize log level from params
        level_str = getattr(params, "log_level", GLOBAL_LOG_LEVEL)
        level_name = str(level_str).upper()
        logging.getLogger().setLevel(logging.getLevelName(level_name))

        # --- Public (read-only via @property, internal via self._X) ---
        self._client: Optional[int] = None
        self._sim_objects: List[SimObject] = []  # List of all simulation objects (Agent, SimObject, etc.)
        self._sim_objects_dict: Dict[int, SimObject] = {}  # Dict for O(1) lookup: object_id -> SimObject
        self._agents: List[Agent] = []  # List of Agent instances only for O(M) iteration in step_once()
        self._robot_original_colors: Dict[int, List[float]] = {}  # body_id: rgbaColor
        self._collision_count: int = 0
        self._step_count: int = 0
        self.sim_time: float = 0.0  # Simulation time (kept public for hot-path access)
        self._enable_time_profiling: bool = params.enable_time_profiling

        # --- Fully private (no external access) ---
        self._start_time: Optional[float] = None
        self._next_object_id: int = 0  # Counter for unique object IDs (read/incremented by SimObject.__init__)
        self._last_logged_collision_count: int = 0  # Track last logged collision count
        self._monitor_enabled: bool = params.monitor
        self._last_monitor_update: float = 0
        self._callbacks: List[Dict[str, Any]] = []  # List of callback functions
        self._data_monitor: Optional[DataMonitor] = None
        self._collision_check_frequency: Optional[float] = params.collision_check_frequency  # If None, check every step
        self._last_collision_check: float = 0.0
        self._log_level: str = params.log_level
        self._params: SimulationParams = params
        self._collision_color: List[float] = collision_color if collision_color is not None else [0, 0, 1, 1]
        self._rendering_enabled: bool = False  # Track rendering state
        self._batch_spawning: bool = False  # True inside batch_spawn() context
        self._collision_visualizer: CollisionVisualizer = CollisionVisualizer()  # Collision shape visualizer

        self._keyboard_events_registered: bool = False  # Track if keyboard events are registered
        self._original_visual_colors: Dict[Tuple[int, int], List[float]] = (
            {}
        )  # Store original colors: (body_id, link_index) -> rgba
        self._structure_transparent: bool = False  # Track if structure is transparent
        self._ignore_static_collision: bool = (
            params.ignore_static_collision
        )  # If True, skip collision checks with static objects
        self._simulation_paused: bool = False  # Track if simulation is paused

        # --- Profiling configuration ---
        self._profiling_interval: int = params.profiling_interval  # Report average every N steps
        self._enable_memory_profiling: bool = params.enable_memory_profiling  # Memory profiling switch

        # --- Profiling statistics ---
        self._profiling_stats: Dict[str, List[float]] = {
            "agent_update": [],
            "callbacks": [],
            "step_simulation": [],
            "collision_check": [],
            "monitor_update": [],
            "total": [],
        }

        # --- Memory profiling statistics (O(1) memory, aggregator-based) ---
        self._memory_stats: Dict[str, Union[int, float]] = {
            "count": 0,  # Number of samples
            "current_sum": 0.0,  # Sum of current memory (for average)
            "current_min": float("inf"),  # Minimum current memory
            "current_max": 0.0,  # Maximum current memory
            "current_first": 0.0,  # First sample (for growth calculation)
            "current_last": 0.0,  # Last sample (for growth calculation)
            "peak_sum": 0.0,  # Sum of peak memory (for average)
            "peak_max": 0.0,  # Maximum peak memory
        }
        self._memory_tracemalloc_started: bool = False  # Track if WE started tracemalloc

        # --- Monitor speed history (deque for O(1) popleft) ---
        self._speed_history: deque = deque()  # [(real_time, sim_time)]

        # --- Movement tracking and collision optimization ---
        self._moved_this_step: set = set()  # object_ids that moved this step
        self._physics_objects: set = set()  # object_ids with physics enabled (can move via collisions)
        self._kinematic_objects: set = set()  # object_ids that move only via set_pose/update
        self._static_collision_objects: set = set()  # object_ids with CollisionMode.STATIC
        self._disabled_collision_objects: set = set()  # object_ids with collision completely disabled

        # --- Collision detection cache ---
        self._cached_collision_modes: Dict[int, CollisionMode] = {}  # object_id -> CollisionMode
        self._cached_cell_size: Optional[float] = None  # Cached cell size for spatial hashing
        self._dynamic_collision_objects: set = set()  # object_ids with NORMAL_3D or NORMAL_2D (excludes STATIC and DISABLED)

        # Incremental AABB and spatial grid cache
        self._cached_aabbs_dict: Dict[int, Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = (
            {}
        )  # object_id -> AABB
        self._cached_spatial_grid: Dict[Tuple[int, int, int], Set[int]] = {}  # Spatial hash grid: cell -> {object_ids}
        self._cached_object_to_cell: Dict[int, List[Tuple[int, int, int]]] = (
            {}
        )  # object_id -> list of cells (supports multi-cell registration)
        self._aabb_cache_valid: bool = False  # Flag indicating if AABB cache is valid
        self._last_ignore_static: Optional[bool] = None  # Track last ignore_static value to detect mode changes

        # Incremental collision tracking
        self._active_collision_pairs: Set[Tuple[int, int]] = set()  # Currently colliding object_id pairs (sorted: i < j)

        # --- Recording ---
        self._recorder: Optional[Any] = None  # Optional[SimulationRecorder] (lazy import to avoid circular)

        self.setup_pybullet()
        self.setup_monitor()

    def setup_monitor(self) -> None:
        # If monitor: true and console_monitor: false, start DataMonitor

        if self._params.monitor:
            self._data_monitor = DataMonitor(
                "PyBullet Simulation Monitor",
                enable_gui=self._params.enable_monitor_gui,
                width=self._params.monitor_width,
                height=self._params.monitor_height,
                x=self._params.monitor_x,
                y=self._params.monitor_y,
            )
            self._data_monitor.start()
        else:
            self._data_monitor = None

    # --- Read-only public properties ---

    @property
    def client(self) -> Optional[int]:
        """PyBullet physics client ID."""
        return self._client

    @property
    def sim_objects(self) -> List[SimObject]:
        """List of all simulation objects (Agent, SimObject, etc.)."""
        return self._sim_objects

    @property
    def agents(self) -> List[Agent]:
        """List of Agent instances."""
        return self._agents

    @property
    def params(self) -> SimulationParams:
        """Simulation parameters."""
        return self._params

    @property
    def step_count(self) -> int:
        """Current simulation step count."""
        return self._step_count

    @property
    def collision_count(self) -> int:
        """Total number of collisions detected."""
        return self._collision_count

    @property
    def enable_memory_profiling(self) -> bool:
        """Whether memory profiling is enabled."""
        return self._enable_memory_profiling

    def register_callback(self, callback: Callable, frequency: Optional[float] = None) -> None:
        """
        Register a callback function to be called during simulation.

        Args:
            callback: Function with signature callback(sim_core, dt) -> None
                     - sim_core: Reference to MultiRobotSimulationCore instance
                     - dt: Time elapsed since last callback execution (seconds)
            frequency: Callback frequency in Hz. If None, called every simulation step.

        Example::

            def my_callback(sim_core, dt):
                for obj in sim_core.sim_objects:
                    if isinstance(obj, Agent):
                        # Update agent logic
                        pass

            sim_core.register_callback(my_callback, frequency=10.0)  # 10 Hz
        """
        self._callbacks.append({"func": callback, "frequency": frequency, "last_exec": 0.0})

    def unregister_callback(self, callback_func: Callable) -> bool:
        """Remove a registered callback by function reference.

        Args:
            callback_func: The callback function to remove (matched by identity).

        Returns:
            True if the callback was found and removed, False otherwise.
        """
        for i, cbinfo in enumerate(self._callbacks):
            if cbinfo["func"] is callback_func:
                self._callbacks.pop(i)
                return True
        return False

    # ------------------------------------------------------------------
    # Recording
    # ------------------------------------------------------------------

    def start_recording(
        self,
        output: str = "recording.gif",
        width: int = 800,
        height: int = 600,
        fps: int = 15,
        duration: Optional[float] = None,
        camera_mode: str = "auto",
        time_base: str = "sim",
        **kwargs: Any,
    ) -> Any:
        """Start recording the simulation as an animated GIF or MP4.

        Frames are captured via ``getCameraImage(ER_TINY_RENDERER)`` at the
        specified *fps* frequency. Works in both ``p.DIRECT`` and ``p.GUI``.

        Output format is auto-detected from the file extension:

        - ``.gif`` — Animated GIF via Pillow (always available)
        - ``.mp4`` — H.264 MP4 via imageio + pyav (requires ``pip install imageio[pyav]``)

        Time bases:

        - ``"sim"`` (default) — Capture at fixed sim-time intervals.
          Playback always looks like 1× speed.
        - ``"real"`` — Capture at wall-clock intervals.
          Playback reflects actual execution speed (e.g. 10× sim → 10× video).

        Args:
            output: Output file path (``.gif`` or ``.mp4``). Parent directories created automatically.
            width: Frame width in pixels.
            height: Frame height in pixels.
            fps: Capture frequency and playback FPS.
            duration: Recording duration in sim-seconds (``time_base="sim"``) or
                wall-clock seconds (``time_base="real"``). ``None`` for manual stop.
            camera_mode: ``"auto"`` | ``"gui"`` | ``"orbit"`` | ``"manual"``.
                ``"auto"`` is promoted to ``"gui"`` when running in GUI mode.
            time_base: ``"sim"`` or ``"real"``. Controls capture timing.
            **kwargs: Passed to ``SimulationRecorder`` (e.g. ``orbit_degrees``,
                     ``camera_params``).

        Returns:
            :class:`~pybullet_fleet.recorder.SimulationRecorder` instance.

        Example::

            sim.start_recording("output.gif", duration=4.0)
            sim.start_recording("output.mp4", duration=4.0, fps=30)
            sim.start_recording("perf.mp4", duration=4.0, time_base="real")
            sim.run_simulation(duration=5.0)
            # File is auto-saved when run_simulation ends
        """
        from pybullet_fleet.recorder import SimulationRecorder

        if self._recorder is not None:
            self.stop_recording()  # stop existing recording
        self._recorder = SimulationRecorder(
            sim_core=self,
            output=output,
            width=width,
            height=height,
            fps=fps,
            duration=duration,
            camera_mode=camera_mode,
            time_base=time_base,
            **kwargs,
        )
        self._recorder.start()
        logger.info("Recording started: %s", output)
        return self._recorder

    def stop_recording(self) -> Optional[str]:
        """Stop recording and save the output file.

        Returns:
            Output file path if a recording was active, ``None`` otherwise.
        """
        if self._recorder is None:
            return None
        recorder = self._recorder
        recorder.stop()
        try:
            path = recorder.save()
            logger.info("Recording saved: %s (%d frames)", path, recorder.frame_count)
        except ValueError:
            logger.warning("Recording stopped but no frames were captured")
            path = None
        self._recorder = None
        return path

    def set_collision_check_frequency(self, frequency: Optional[float] = None) -> None:
        """
        Set the frequency (Hz, number of times per second) for collision detection.

        Args:
            frequency: Collision check frequency in Hz (times per second)
                      - None: Check every step (highest accuracy, highest cost)
                      - 0: Disable collision checking (no collision detection)
                      - > 0: Check at specified frequency (e.g., 1.0 = once per second)

        Example::

            sim.set_collision_check_frequency(None)  # Every step
            sim.set_collision_check_frequency(1.0)   # 1 Hz (once per second)
            sim.set_collision_check_frequency(10.0)  # 10 Hz
            sim.set_collision_check_frequency(0)     # Disable

        Note: Higher frequency increases computational cost but improves collision detection accuracy.
        """
        self._collision_check_frequency = frequency

        # Log the setting for clarity
        if frequency is None:
            logger.info("Collision check frequency set to: EVERY STEP (highest accuracy)")
        elif frequency == 0:
            logger.info("Collision check frequency set to: DISABLED (no collision detection)")
        else:
            logger.info(f"Collision check frequency set to: {frequency} Hz ({1.0/frequency:.3f}s interval)")

    def setup_pybullet(self) -> None:
        """Initialize PyBullet with GUI panels hidden."""
        # Force headless mode when RECORD env var is set (enables zero-code-change recording).
        # RECORD_GUI=1 overrides this to keep the GUI window open so the recorder
        # can capture via ER_BULLET_HARDWARE_OPENGL (GPU-quality frames).
        if os.environ.get("RECORD") and self._params.gui and not os.environ.get("RECORD_GUI"):
            logger.info("RECORD env var detected: forcing gui=False for headless recording")
            self._params.gui = False

        options = ""
        if self._params.gui and self._params.window_width > 0 and self._params.window_height > 0:
            options = f"--width={self._params.window_width} --height={self._params.window_height}"
        self._client = p.connect(p.GUI if self._params.gui else p.DIRECT, options=options)

        # Hide all debug UI panels immediately after connection
        if self._params.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self._client)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81 if self._params.physics else 0, physicsClientId=self._client)
        p.setTimeStep(self._params.timestep, physicsClientId=self._client)
        p.setRealTimeSimulation(0, physicsClientId=self._client)

        # Physics engine parameter tuning
        self._apply_physics_engine_params()
        if self._params.enable_floor:
            p.loadURDF("plane.urdf", physicsClientId=self._client)

        # Register user-supplied model search paths.
        # NOTE: add_search_path() mutates the global _user_search_paths registry
        # in robot_models, so paths registered here persist across sim instances.
        if self._params.model_paths:
            from pybullet_fleet.robot_models import add_search_path

            for path in self._params.model_paths:
                add_search_path(path)

        # Disable rendering during setup for better performance
        if self._params.gui:
            self.disable_rendering()

    def _apply_physics_engine_params(self) -> None:
        """Apply physics engine parameter tuning.

        Shared by ``setup_pybullet()`` and ``reset()`` to ensure
        consistent engine configuration after ``p.resetSimulation()``.
        """
        p.setPhysicsEngineParameter(enableFileCaching=True, physicsClientId=self._client)
        if self._params.physics:
            # Physics ON: deterministic broadphase + CCD for reproducibility and tunneling prevention
            p.setPhysicsEngineParameter(
                deterministicOverlappingPairs=True,
                allowedCcdPenetration=0.01,
                physicsClientId=self._client,
            )
        else:
            # Physics OFF (kinematics only): minimize solver overhead
            p.setPhysicsEngineParameter(
                numSubSteps=1,
                numSolverIterations=1,
                enableConeFriction=False,
                physicsClientId=self._client,
            )

    def disable_rendering(self) -> None:
        """Disable rendering during object spawning for better performance."""
        if self._params.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=self._client)
            self._rendering_enabled = False
            logger.info("Rendering disabled for setup/spawning phase")

    def enable_rendering(self) -> None:
        """Enable rendering before starting simulation."""
        if self._params.gui and not self._rendering_enabled:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=self._client)
            self._rendering_enabled = True
            logger.info("Rendering enabled for simulation")

    @contextmanager
    def batch_spawn(self):
        """Context manager for optimised batch spawning.

        Disables rendering and defers spatial-grid rebuild until
        the context exits.  Use when spawning many objects at once.

        Rendering state is saved on entry and restored on exit.
        If rendering was already disabled (e.g. during the initial
        setup phase before ``run_simulation()``), it stays disabled
        after ``batch_spawn()`` exits.

        Example::

            with sim_core.batch_spawn():
                for params in params_list:
                    Agent.from_params(params, sim_core)
            # rendering restored to previous state, spatial grid rebuilt once
        """
        was_adaptive = self._params.spatial_hash_cell_size_mode == SpatialHashCellSizeMode.AUTO_ADAPTIVE
        was_rendering = self._rendering_enabled
        self._batch_spawning = True
        self.disable_rendering()
        try:
            yield
        finally:
            self._batch_spawning = False
            if was_adaptive:
                # set_collision_spatial_hash_cell_size_mode() already calls _rebuild_spatial_grid()
                self.set_collision_spatial_hash_cell_size_mode()
            else:
                self._rebuild_spatial_grid()
            if was_rendering:
                self.enable_rendering()

    def _calculate_cell_size_from_aabbs(
        self, aabbs: Optional[List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]] = None
    ) -> Optional[float]:
        """
        Calculate optimal cell size from AABBs using median extent.

        Args:
            aabbs: List of AABB tuples ((min_x, min_y, min_z), (max_x, max_y, max_z)).
                   If None, uses self._cached_aabbs_dict.values() (default: None)

        Returns:
            Calculated cell_size, or None if no AABBs available

        Example::

            # Use cached AABBs
            cell_size = sim._calculate_cell_size_from_aabbs()

            # Use custom AABBs
            custom_aabbs = [p.getAABB(body_id) for body_id in body_ids]
            cell_size = sim._calculate_cell_size_from_aabbs(custom_aabbs)
        """
        # Use cached AABBs if not provided
        if aabbs is None:
            aabbs = list(self._cached_aabbs_dict.values())

        if not aabbs:
            return None

        extents = []
        for aabb in aabbs:
            extent_x = aabb[1][0] - aabb[0][0]
            extent_y = aabb[1][1] - aabb[0][1]
            extent_z = aabb[1][2] - aabb[0][2]
            extents.append(max(extent_x, extent_y, extent_z))

        median_extent = sorted(extents)[len(extents) // 2]
        cell_size = max(median_extent * 2.0, 0.5)

        return cell_size

    def _rebuild_spatial_grid(self) -> None:
        """Rebuild entire spatial grid using current cell_size and AABBs."""
        self._cached_spatial_grid.clear()
        self._cached_object_to_cell.clear()

        for object_id in self._cached_aabbs_dict.keys():
            self._update_object_spatial_grid(object_id)

        logger.debug(f"Collision spatial hash: Rebuilt grid with {len(self._cached_spatial_grid)} cells")

    def _mark_object_moved(self, object_id: int) -> None:
        """
        Mark an object as moved in the current step.
        Called by SimObject.set_pose() when movement is detected.

        Args:
            object_id: The object_id that moved
        """
        self._moved_this_step.add(object_id)
        logger.debug(f"Object {object_id} marked as moved")

    def _update_object_aabb(self, object_id: int, update_grid: bool = True) -> None:
        """
        Update AABB for a single object immediately.
        Optionally updates spatial grid if cell_size is initialized.

        Called when kinematic objects move or physics objects are updated.

        Args:
            object_id: The object_id to update
            update_grid: If True, also update spatial grid (default: True)
                        Set to False to avoid circular calls from _update_object_spatial_grid
        """
        obj = self._sim_objects_dict.get(object_id)
        if obj is None:
            logger.warning(f"Cannot update AABB for object {object_id}: object not found")
            return

        try:
            self._cached_aabbs_dict[object_id] = p.getAABB(obj.body_id, physicsClientId=self._client)
            logger.debug(f"Updated AABB for object {object_id}")

            # Also update spatial grid if requested and cell_size is initialized
            if update_grid and self._cached_cell_size is not None:
                self._update_object_spatial_grid(object_id)
        except (p.error, Exception):
            # In kinematics mode (physics=false), getAABB() may fail for newly
            # spawned bodies because PyBullet has not yet computed collision info.
            # This is harmless — the AABB will be populated on the first
            # performCollisionDetection() / stepSimulation() call.
            logger.debug(f"Deferred AABB init for object {object_id} (body {obj.body_id})")

    def _remove_object_aabb(self, object_id: int, update_grid: bool = True) -> None:
        """
        Remove AABB and spatial grid entry for an object.
        Used when objects become static (no longer need collision tracking).

        Args:
            object_id: The object_id to remove from AABB cache
        """
        # Remove from AABB cache
        if object_id in self._cached_aabbs_dict:
            self._cached_aabbs_dict.pop(object_id)
            logger.debug(f"Removed object {object_id} from AABB cache")

        # Remove from spatial grid using unified method
        if update_grid:
            self._update_object_spatial_grid(object_id, remove_only=True)

    def _should_use_multi_cell_registration(self, object_id: int) -> bool:
        """
        Determine if object should be registered to multiple cells
        based on its AABB size relative to cell size.

        Large objects (AABB extent > cell_size * threshold) are registered
        to all overlapping cells to ensure collision detection with objects
        in neighboring cells.

        Args:
            object_id: The object_id to check

        Returns:
            True if object should use multi-cell registration, False otherwise

        Note:

            - Threshold is controlled by params.multi_cell_threshold (default: 1.5)
            - Only considers XY plane (ignores Z for 2D collision compatibility)
            - Automatically called during spatial grid updates
        """
        if self._cached_cell_size is None:
            return False

        # Get AABB (should already be cached)
        if object_id not in self._cached_aabbs_dict:
            return False

        aabb = self._cached_aabbs_dict[object_id]
        aabb_min, aabb_max = aabb[0], aabb[1]

        # Calculate max extent in XY plane (ignore Z for 2D collision)
        extent_x = aabb_max[0] - aabb_min[0]
        extent_y = aabb_max[1] - aabb_min[1]
        max_extent = max(extent_x, extent_y)

        # Use multi-cell if object is larger than threshold * cell_size
        threshold_size = self._cached_cell_size * self._params.multi_cell_threshold

        return max_extent > threshold_size

    def _discard_from_cells(self, object_id: int, cells: List[Tuple[int, int, int]]) -> None:
        """Remove an object from the given spatial grid cells.

        For each cell, discards *object_id* from the cell's set.
        If the set becomes empty the cell key is deleted from the grid.

        Args:
            object_id: The object to remove
            cells: List of (x, y, z) cell keys to remove *object_id* from
        """
        for cell in cells:
            if cell in self._cached_spatial_grid:
                self._cached_spatial_grid[cell].discard(object_id)
                if not self._cached_spatial_grid[cell]:
                    del self._cached_spatial_grid[cell]

    def _coord_to_cell(self, coord: float) -> int:
        """Convert a world coordinate to a spatial grid cell index.

        Uses floor division so that negative coordinates map toward -inf
        (e.g. -0.1 / 2.0 → cell -1).

        Args:
            coord: World-space coordinate value (x, y, or z)

        Returns:
            Integer cell index along the given axis
        """
        return int(math.floor(coord / cast(float, self._cached_cell_size)))

    def _get_overlapping_cells(self, object_id: int) -> List[Tuple[int, int, int]]:
        """
        Get all grid cells that overlap with object's AABB.

        Used for large objects that span multiple cells to ensure
        collision detection with objects in all neighboring cells.

        Args:
            object_id: The object_id to get overlapping cells for

        Returns:
            List of cell tuples (x, y, z) that overlap with the object's AABB

        Example::

            # Object with 5m x 5m AABB, cell_size=2m
            cells = sim._get_overlapping_cells(obj_id)
            # Returns: [(0,0,0), (0,1,0), (1,0,0), (1,1,0), ...] (9 cells in XY)
        """
        if self._cached_cell_size is None:
            return []

        # Get AABB (should already be cached)
        if object_id not in self._cached_aabbs_dict:
            return []

        aabb = self._cached_aabbs_dict[object_id]
        aabb_min, aabb_max = aabb[0], aabb[1]

        # Calculate cell range for each dimension
        x_range = range(self._coord_to_cell(aabb_min[0]), self._coord_to_cell(aabb_max[0]) + 1)
        y_range = range(self._coord_to_cell(aabb_min[1]), self._coord_to_cell(aabb_max[1]) + 1)
        z_range = range(self._coord_to_cell(aabb_min[2]), self._coord_to_cell(aabb_max[2]) + 1)

        # Generate all cell combinations
        return [(x, y, z) for x in x_range for y in y_range for z in z_range]

    def _update_object_spatial_grid(self, object_id: int, remove_only: bool = False) -> None:
        """
        Update an object's position in the spatial grid.

        This unified method handles:
        - Initial addition: old_cells=None → add to new cell(s)
        - Movement update: old_cells exists → remove from old, add to new
          (skipped when new cells == old cells for performance)
        - Removal: remove_only=True → remove from current cell(s) only
        - Multi-cell registration: Large objects registered to all overlapping cells

        Args:
            object_id: The object_id to update in spatial grid
            remove_only: If True, only remove from grid without adding to new cell(s)
                        Used when objects become static or are deleted

        Example::

            # First time (add)
            sim._update_object_spatial_grid(obj_id)  # Adds to cell(s)

            # After movement (update)
            sim._update_object_spatial_grid(obj_id)  # Removes from old cell(s), adds to new

            # Remove from grid
            sim._update_object_spatial_grid(obj_id, remove_only=True)  # Only removes
        """
        old_cells = self._cached_object_to_cell.get(object_id)

        # If remove_only, remove from old cells and return
        if remove_only:
            if old_cells is not None:
                self._discard_from_cells(object_id, old_cells)
                del self._cached_object_to_cell[object_id]
                logger.debug(f"Removed object {object_id} from {len(old_cells)} spatial grid cell(s)")
            return

        # For add/update: ensure AABB exists
        if object_id not in self._cached_aabbs_dict:
            # Safeguard against stale object references
            if object_id not in self._sim_objects_dict:
                logger.warning(f"Cannot update spatial grid for object {object_id}: object not found")
                return
            # Update AABB without triggering spatial grid update (avoid circular call)
            self._update_object_aabb(object_id, update_grid=False)
            if object_id not in self._cached_aabbs_dict:
                logger.warning(f"Cannot update spatial grid for object {object_id}: AABB update failed")
                return

        if self._cached_cell_size is None:
            return

        # Compute new cells
        use_multi_cell = self._should_use_multi_cell_registration(object_id)

        if use_multi_cell:
            cells = self._get_overlapping_cells(object_id)
        else:
            aabb = self._cached_aabbs_dict[object_id]
            center = [0.5 * (aabb[0][d] + aabb[1][d]) for d in range(3)]
            cell = tuple(self._coord_to_cell(center[d]) for d in range(3))
            cells = [cell]

        # Same-cell optimization: skip grid operations if cells haven't changed
        if old_cells is not None and cells == old_cells:
            return

        # Remove from old cells (if exists)
        if old_cells is not None:
            self._discard_from_cells(object_id, old_cells)

        # Register to new cells
        self._cached_object_to_cell[object_id] = cells
        for cell in cells:
            self._cached_spatial_grid.setdefault(cell, set()).add(object_id)

        # Log
        if old_cells is None:
            logger.debug(f"Added object {object_id} to {len(cells)} spatial grid cell(s) (multi-cell: {use_multi_cell})")
        else:
            logger.debug(
                f"Updated object {object_id} spatial grid: {len(old_cells)} -> {len(cells)} cell(s) "
                f"(multi-cell: {use_multi_cell})"
            )

    def _register_object_movement_type(self, obj: SimObject) -> None:
        """
        Register (or re-register) an object's movement type for optimization.

        This method is **idempotent**: it first clears any existing movement-type
        entries for the object, then classifies it afresh.  Safe to call both on
        initial add and on subsequent mode changes.

        Movement types:
        - STATIC: Never moves (registered via obj.is_static)
        - PHYSICS: Moves via physics engine (mass > 0, not kinematic)
        - KINEMATIC: Moves via set_pose() (mass == 0 or is_kinematic=True)

        Note: Collision mode (DISABLED) is handled in collision system methods,
              not here (this only handles movement classification).

        Args:
            obj: The SimObject instance to register
        """
        object_id = obj.object_id

        # Idempotent: clear previous classification (no-op on first call)
        self._physics_objects.discard(object_id)
        self._kinematic_objects.discard(object_id)

        # Register based on movement type (not collision mode)
        # Note: _static_collision_objects is managed by collision-domain methods,
        # not here. This only handles _physics_objects / _kinematic_objects.
        if obj.is_static:
            # STATIC: Never moves (no movement tracking needed)
            logger.debug(f"Object {object_id} registered as static (movement type)")
        else:
            # Dynamic objects: Determine if physics-enabled or kinematic
            is_physics = obj.mass > 0 and not obj.is_kinematic

            if is_physics:
                self._physics_objects.add(object_id)
                logger.debug(f"Object {object_id} registered as physics-enabled (movement type)")
            else:
                self._kinematic_objects.add(object_id)
                logger.debug(f"Object {object_id} registered as kinematic (movement type)")

    def _update_object_collision_mode(self, object_id: int, old_mode: CollisionMode, new_mode: CollisionMode) -> None:
        """
        Update collision system when object's collision mode changes.

        Called by SimObject.set_collision_mode() when mode is changed.
        Handles all cache updates (AABB, spatial grid, movement tracking).

        Args:
            object_id: Object ID
            old_mode: Previous CollisionMode
            new_mode: New CollisionMode
        """
        # Early return if mode hasn't changed
        if old_mode == new_mode:
            return

        obj = self._sim_objects_dict.get(object_id)
        if obj is None:
            logger.warning(f"Cannot update collision mode for object {object_id}: not found")
            return

        # Re-register movement type (physics / kinematic classification)
        self._register_object_movement_type(obj)

        # Update collision mode cache
        self._cached_collision_modes[object_id] = new_mode

        # Handle collision system updates based on mode transition
        # _static_collision_objects and _dynamic_collision_objects are managed here (collision domain)
        # Structure: First check old_mode, then check new_mode within each case

        if old_mode == CollisionMode.DISABLED:
            # DISABLED -> *: Classification + AABB/grid handled inside
            self._add_object_to_collision_system(object_id)

        elif old_mode == CollisionMode.STATIC:
            # STATIC -> *: STATIC already has AABB/grid
            self._static_collision_objects.discard(object_id)
            if new_mode == CollisionMode.DISABLED:
                # STATIC -> DISABLED: Remove from collision system
                self._remove_object_from_collision_system(object_id)
            else:
                # STATIC -> NORMAL_3D/2D: Add to dynamic set, mark as moved
                self._dynamic_collision_objects.add(object_id)
                self._moved_this_step.add(object_id)

        else:
            # NORMAL_3D/2D -> *: Currently in dynamic set
            if new_mode == CollisionMode.DISABLED:
                # NORMAL -> DISABLED: Remove from collision system and dynamic set
                self._dynamic_collision_objects.discard(object_id)
                self._remove_object_from_collision_system(object_id)
            elif new_mode == CollisionMode.STATIC:
                # NORMAL -> STATIC: Remove from dynamic set, add to static set, keep AABB/grid
                self._dynamic_collision_objects.discard(object_id)
                self._static_collision_objects.add(object_id)
            else:
                # NORMAL_3D <-> NORMAL_2D: mode cached in _cached_collision_modes, mark as moved
                self._moved_this_step.add(object_id)

        logger.info(f"Updated collision mode for object {object_id}: {old_mode.value} -> {new_mode.value}")

    def _add_object_to_collision_system(self, object_id: int) -> None:
        """
        Add object to collision detection system.

        Classifies the object by its current ``collision_mode`` into the
        appropriate collision set / dict, then initialises AABB cache,
        spatial grid entry, and marks the object as moved.

        Called when an object is first added (``add_object``) or when its
        collision mode transitions from DISABLED to an enabled mode.

        Note: Must NOT be called for DISABLED collision mode objects.
              The caller is responsible for handling DISABLED separately.

        Args:
            object_id: Object ID to add
        """
        obj = self._sim_objects_dict.get(object_id)
        if obj is None:
            logger.warning(f"Cannot add object {object_id} to collision system: not found")
            return

        # Remove from disabled set (transitioning from DISABLED to enabled mode)
        self._disabled_collision_objects.discard(object_id)

        # Classify into collision-domain set based on current mode
        mode = obj.collision_mode
        if mode == CollisionMode.STATIC:
            self._static_collision_objects.add(object_id)
        else:
            # NORMAL_3D or NORMAL_2D
            self._dynamic_collision_objects.add(object_id)

        self._update_object_aabb(object_id, update_grid=True)
        self._moved_this_step.add(object_id)
        logger.debug(f"Added object {object_id} to collision system (mode={mode.value})")

    def _remove_object_from_collision_system(self, object_id: int) -> None:
        """
        Remove object from collision detection system.

        Removes AABB cache, spatial grid entry, and movement tracking.
        If object is transitioning to DISABLED mode, adds to _disabled_collision_objects.
        If object is being completely removed, cleans up _disabled_collision_objects.

        Used when object is removed or when collision mode changes to disable collision.

        Args:
            object_id: Object ID to remove
        """
        obj = self._sim_objects_dict.get(object_id)
        if obj is not None and obj.collision_mode == CollisionMode.DISABLED:
            # Object exists and is DISABLED: Add to disabled set
            self._disabled_collision_objects.add(object_id)
            logger.debug(f"Removed object {object_id} from collision system (now DISABLED)")
        else:
            # Object removed or transitioning to non-DISABLED mode: Remove from disabled set
            self._disabled_collision_objects.discard(object_id)
            logger.debug(f"Removed object {object_id} from collision system")

        self._remove_object_aabb(object_id)
        self._moved_this_step.discard(object_id)

        # Remove all active collision pairs referencing this object
        self._active_collision_pairs = {(i, j) for i, j in self._active_collision_pairs if i != object_id and j != object_id}

    def set_collision_spatial_hash_cell_size_mode(
        self, mode: Optional[SpatialHashCellSizeMode] = None, cell_size: Optional[float] = None
    ) -> float:
        """
        Set collision detection spatial hash cell size mode and recalculate/rebuild grid.

        This method allows dynamic mode switching and always recalculates the cell_size
        based on the spatial_hash_cell_size_mode setting (or override via parameters).

        Args:
            mode: Override spatial_hash_cell_size_mode for this calculation (optional)
            cell_size: Override spatial_hash_cell_size when mode=CONSTANT (optional)

        Returns:
            The calculated cell_size (always > 0, falls back to 1.0 if no AABBs available)

        Example::

            # Recalculate with current mode
            sim_core.set_collision_spatial_hash_cell_size_mode()

            # Switch to constant mode with specific size
            sim_core.set_collision_spatial_hash_cell_size_mode(
                mode=SpatialHashCellSizeMode.CONSTANT,
                cell_size=3.0
            )

            # Switch to auto_adaptive mode
            sim_core.set_collision_spatial_hash_cell_size_mode(
                mode=SpatialHashCellSizeMode.AUTO_ADAPTIVE
            )
        """
        # Update params if mode or cell_size is provided
        if mode is not None:
            old_mode = self._params.spatial_hash_cell_size_mode
            self._params.spatial_hash_cell_size_mode = mode
            logger.debug(f"Updated mode: {old_mode} -> {mode}")

        if cell_size is not None:
            old_size = self._params.spatial_hash_cell_size
            self._params.spatial_hash_cell_size = cell_size

            # Warn if cell_size is provided but mode is not CONSTANT
            current_mode = self._params.spatial_hash_cell_size_mode
            if current_mode != SpatialHashCellSizeMode.CONSTANT:
                logger.warning(
                    f"cell_size parameter ({cell_size}) will be ignored in {current_mode.value} mode. "
                    "To use a fixed cell_size, set mode=SpatialHashCellSizeMode.CONSTANT"
                )
            else:
                logger.debug(f"Updated cell_size: {old_size} -> {cell_size}")

        mode = self._params.spatial_hash_cell_size_mode
        old_cell_size = self._cached_cell_size

        # Calculate cell_size based on mode
        if mode == SpatialHashCellSizeMode.CONSTANT:
            # CONSTANT mode: use user-provided value
            if self._params.spatial_hash_cell_size is None:
                logger.error(
                    "spatial_hash_cell_size_mode=CONSTANT requires spatial_hash_cell_size to be set. "
                    "Falling back to default 1.0m"
                )
                cell_size = 1.0
            else:
                cell_size = self._params.spatial_hash_cell_size
            self._cached_cell_size = cell_size
            logger.info(f"Collision spatial hash: Using constant cell size: {cell_size:.3f}m")

        elif mode in [SpatialHashCellSizeMode.AUTO_ADAPTIVE, SpatialHashCellSizeMode.AUTO_INITIAL]:
            # AUTO modes: Calculate from median AABB extent
            cell_size = self._calculate_cell_size_from_aabbs()
            if cell_size is not None:
                self._cached_cell_size = cell_size
                logger.info(
                    f"Collision spatial hash: {'Recalculated' if old_cell_size else 'Calculated'} cell size ({mode.value}): "
                    f"{cell_size:.3f}m (num_objects: {len(self._cached_aabbs_dict)})"
                )
            else:
                cell_size = 1.0
                self._cached_cell_size = cell_size
                logger.warning("Collision spatial hash: No AABBs available, using default cell size: 1.0m")
        else:
            logger.error(f"Unknown spatial_hash_cell_size_mode: {mode}, using default 1.0m")
            cell_size = 1.0
            self._cached_cell_size = cell_size

        # Rebuild spatial grid with new cell_size
        self._rebuild_spatial_grid()

        return cell_size

    def add_object(self, obj: SimObject) -> None:
        """
        Add an object to simulation and register it with all necessary caches.

        This method centralizes object registration logic for consistency and maintainability.
        Called automatically by SimObject.__init__() when sim_core is provided.

        Args:
            obj: The SimObject instance to add

        Example::

            # Manual addition (not recommended - SimObject does this automatically)
            obj = SimObject(body_id=body_id, sim_core=None)
            sim_core.add_object(obj)

            # Automatic addition (recommended)
            obj = SimObject(body_id=body_id, sim_core=sim_core)  # Calls add_object() internally
        """
        # Add to sim_objects list and dict
        if obj in self._sim_objects:
            logger.warning(f"Object {obj.object_id} already added to simulation")
            return

        self._sim_objects.append(obj)
        self._sim_objects_dict[obj.object_id] = obj

        # Add to agents list if this is an Agent instance (for O(M) iteration)
        if isinstance(obj, Agent):
            self._agents.append(obj)

        # Store collision mode in cache
        self._cached_collision_modes[obj.object_id] = obj.collision_mode

        # Register movement type based on collision mode
        self._register_object_movement_type(obj)

        # Add to collision system based on collision mode
        if obj.collision_mode == CollisionMode.DISABLED:
            # DISABLED: No collision detection at all
            self._disabled_collision_objects.add(obj.object_id)
        else:
            # STATIC / NORMAL_3D / NORMAL_2D: classification + AABB/grid
            self._add_object_to_collision_system(obj.object_id)

        # Trigger cell_size recalculation in auto_adaptive mode (deferred during batch spawn)
        if self._params.spatial_hash_cell_size_mode == SpatialHashCellSizeMode.AUTO_ADAPTIVE:
            if not self._batch_spawning:
                self.set_collision_spatial_hash_cell_size_mode()

        # Save original color once when object is added (for collision visualization)
        if obj.body_id not in self._robot_original_colors:
            try:
                # Try to get current visual color, fallback to default if not available
                visual_data = p.getVisualShapeData(obj.body_id, physicsClientId=self._client)
                if visual_data:
                    self._robot_original_colors[obj.body_id] = list(visual_data[0][7])  # RGBA color
                else:
                    self._robot_original_colors[obj.body_id] = [0.0, 0.0, 0.0, 1.0]
            except (p.error, IndexError):
                self._robot_original_colors[obj.body_id] = [0.0, 0.0, 0.0, 1.0]

        logger.debug(f"Added object {obj.object_id} (body {obj.body_id}) to simulation")

    def remove_object(self, obj: SimObject) -> None:
        """
        Remove an object from simulation and clean up all caches.

        Args:
            obj: The SimObject instance to remove

        Example::

            obj = sim_core.spawn_robot(...)
            # ... later ...
            sim_core.remove_object(obj)
        """
        obj_id = obj.object_id

        # Remove from sim_objects list and dict
        try:
            self._sim_objects.remove(obj)
        except ValueError:
            logger.warning(f"Object {obj_id} not found in sim_objects")
            return

        self._sim_objects_dict.pop(obj_id, None)

        # Remove from agents list if this is an Agent instance
        if isinstance(obj, Agent):
            try:
                self._agents.remove(obj)
            except ValueError:
                pass  # Already removed or not in list

        # Remove from movement tracking sets
        self._moved_this_step.discard(obj_id)
        self._physics_objects.discard(obj_id)
        self._kinematic_objects.discard(obj_id)
        self._static_collision_objects.discard(obj_id)
        self._disabled_collision_objects.discard(obj_id)

        # Remove from collision mode cache
        self._cached_collision_modes.pop(obj_id, None)

        # Remove from collision system (AABB, spatial grid)
        self._remove_object_from_collision_system(obj_id)

        # Remove from dynamic collision set
        self._dynamic_collision_objects.discard(obj_id)

        # Remove original color cache (prevents stale color on body_id reuse)
        self._robot_original_colors.pop(obj.body_id, None)

        # Remove PyBullet body
        if obj.body_id is not None:
            try:
                p.removeBody(obj.body_id, physicsClientId=self._client)
            except p.error:
                logger.warning(f"Failed to remove PyBullet body {obj.body_id}")

        # Trigger cell_size recalculation in auto_adaptive mode
        if self._params.spatial_hash_cell_size_mode == SpatialHashCellSizeMode.AUTO_ADAPTIVE:
            self.set_collision_spatial_hash_cell_size_mode()

        logger.info(f"Removed object {obj_id} (body {obj.body_id}) from simulation")

    def configure_visualizer(
        self,
        enable_structure_transparency: Optional[bool] = None,
        enable_shadows: Optional[bool] = None,
    ) -> None:
        """
        Configure PyBullet visualizer settings with keyboard control.

        Args:
            enable_structure_transparency: Initial state for structure transparency (None=use config)
            enable_shadows: Enable shadows (None=use config)

        Custom keyboard shortcuts (active during simulation):
        - Press SPACE to pause/play simulation
        - Press 't' to toggle structure transparency ON/OFF

        PyBullet built-in keyboard shortcuts (always available):
        - Press 'w' to toggle wireframe / collision shape display
        - Press 'g' to toggle grid display
        - Press 'j' to toggle joint axes display
        """
        if not self._params.gui:
            return

        # Use config values if parameters are None
        if enable_structure_transparency is None:
            enable_structure_transparency = self._params.enable_structure_transparency
        if enable_shadows is None:
            enable_shadows = self._params.enable_shadows

        # Save original colors of all visual shapes ONCE
        if not self._original_visual_colors:
            self._save_original_visual_colors()

        # Store initial states
        self._structure_transparent = enable_structure_transparency

        # Configure shadows
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1 if enable_shadows else 0, physicsClientId=self._client)

        # Apply initial collision shape visibility from config
        if self._params.enable_collision_shapes:
            p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1, physicsClientId=self._client)

        # Apply initial structure transparency
        self._set_structure_transparency(self._structure_transparent)

        # Enable keyboard event handling in step_once()
        self._keyboard_events_registered = True

        logger.info(
            "Visualizer configured: transparency=%s, shadows=%s",
            enable_structure_transparency,
            enable_shadows,
        )
        logger.info("Keyboard controls registered: SPACE=pause, t=transparency")
        print("\n[KEYBOARD CONTROLS]")
        print("  Custom:")
        print("    Press SPACE to pause/play simulation")
        print("    Press 't' to toggle structure transparency " f"(current: {'ON' if self._structure_transparent else 'OFF'})")
        if len(self._static_collision_objects) > 100:
            print(
                f"             ⚠️  Warning: {len(self._static_collision_objects)} static objects detected - "
                "toggling may be slow (not recommended)"
            )
        print("  PyBullet built-in:")
        print("    Press 'w' to toggle wireframe / collision shapes")
        print("    Press 'g' to toggle grid")
        print("    Press 'j' to toggle joint axes")

    def _save_original_visual_colors(self) -> None:
        """
        Save original colors of all visual shapes for fast restoration.
        Called once during configure_visualizer().
        """
        num_bodies = p.getNumBodies(physicsClientId=self._client)
        for body_id in range(num_bodies):
            visual_data = p.getVisualShapeData(body_id, physicsClientId=self._client)
            for shape in visual_data:
                link_index = shape[1]
                rgba = shape[7]  # Original RGBA color
                key = (body_id, link_index)
                self._original_visual_colors[key] = rgba

        logger.info(f"Saved original colors for {len(self._original_visual_colors)} visual shapes")

    def _handle_keyboard_events(self) -> None:
        """
        Handle keyboard events for pausing simulation and toggling transparency.
        Called during simulation step.
        """
        if not self._params.gui:
            return

        try:
            # Get keyboard events
            keys = p.getKeyboardEvents(physicsClientId=self._client)
        except p.error:
            # PyBullet disconnected (window closed), stop handling events
            return

        # Space key (ASCII 32) - toggle pause/play
        if ord(" ") in keys and keys[ord(" ")] & p.KEY_WAS_TRIGGERED:
            if self.is_paused:
                self.resume()
            else:
                self.pause()
            print(f"\n[PAUSE] Simulation: {'PAUSED' if self.is_paused else 'PLAYING'}")

        # 't' key (ASCII 116) - toggle structure transparency
        if ord("t") in keys and keys[ord("t")] & p.KEY_WAS_TRIGGERED:
            num_structures = len(self._static_collision_objects)
            if num_structures > 100:
                print(f"\n[TOGGLE] ⚠️  Warning: Toggling transparency for {num_structures} static objects...")
                print("         This may take several seconds (not recommended for large scenes)")
                print("         Consider setting 'enable_structure_transparency' in config.yaml instead")

            self._structure_transparent = not self._structure_transparent
            self._set_structure_transparency(self._structure_transparent)
            print(f"\n[TOGGLE] Structure transparency: {'ON' if self._structure_transparent else 'OFF'}")

    def _set_structure_transparency(self, transparent: bool) -> None:
        """
        Set transparency of static bodies (structures).
        Uses pre-saved colors with modified alpha channel.
        Disables rendering during batch update for ~175x speedup in GUI mode.

        Args:
            transparent: True to make static objects semi-transparent, False for opaque
        """
        if not self._params.gui:
            return

        alpha = 0.3 if transparent else 1.0

        logger.info(f"[TRANSPARENCY] Applying alpha={alpha} to {len(self._static_collision_objects)} static objects...")

        # Apply alpha only to static bodies using pre-saved colors (FAST)
        # Convert object_ids to body_ids for visual shape manipulation
        static_body_ids = {
            self._sim_objects_dict[obj_id].body_id
            for obj_id in self._static_collision_objects
            if obj_id in self._sim_objects_dict
        }

        processed = 0
        # Disable rendering during batch update to avoid per-call OpenGL re-render
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=self._client)
        try:
            for key, rgba in self._original_visual_colors.items():
                body_id, link_index = key

                # Only process static bodies (O(1) lookup with set)
                if body_id not in static_body_ids:
                    continue

                try:
                    # Apply new alpha to the original color
                    p.changeVisualShape(
                        body_id, link_index, rgbaColor=[rgba[0], rgba[1], rgba[2], alpha], physicsClientId=self._client
                    )
                    processed += 1
                except Exception:
                    pass
        finally:
            # Always re-enable rendering, even if an error occurred
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=self._client)

        logger.info(f"[TRANSPARENCY] Complete: {processed} visual shapes updated")
        logger.info(f"Static objects transparency {'enabled (alpha=0.3)' if transparent else 'disabled (alpha=1.0)'}")

    def compute_scene_bounds(self) -> Tuple[List[float], List[float]]:
        """Compute scene bounding box from all sim_object positions.

        Returns a ``(center, extent)`` tuple where *center* is the mean position
        ``[cx, cy, cz]`` and *extent* is the axis-aligned size ``[ex, ey, ez]``
        (i.e. ``max - min`` per axis).

        When the scene is empty both lists are ``[0.0, 0.0, 0.0]``.

        This is used by :meth:`setup_camera` (GUI) and
        :class:`~pybullet_fleet.recorder.SimulationRecorder` (offscreen) so that
        both share the same scene-framing logic.

        Returns:
            Tuple of ``([cx, cy, cz], [ex, ey, ez])``.
        """
        positions: List[List[float]] = []
        for obj in self._sim_objects:
            try:
                pos, _ = p.getBasePositionAndOrientation(obj.body_id, physicsClientId=self._client)
                positions.append(list(pos))
            except Exception:
                continue

        if not positions:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

        arr = np.array(positions)
        center = arr.mean(axis=0)
        extent = arr.max(axis=0) - arr.min(axis=0)
        return [float(center[0]), float(center[1]), float(center[2])], [
            float(extent[0]),
            float(extent[1]),
            float(extent[2]),
        ]

    def setup_camera(
        self, camera_config: Optional[Dict[str, Any]] = None, entity_positions: Optional[List[List[float]]] = None
    ) -> None:
        """
        Set up camera view based on configuration.

        Args:
            camera_config: Dictionary with camera settings (from yaml config)
            entity_positions: List of [x, y, z] positions for auto camera calculation
        """
        if not self._params.gui:
            return  # No camera setup needed without GUI

        # Use self._params.camera_config if camera_config is not provided
        if camera_config is None:
            camera_config = self._params.camera_config

        camera_mode = camera_config.get("camera_mode", "none")

        valid_modes = ("none", "manual", "auto")
        if camera_mode not in valid_modes:
            logger.warning(f"Unknown camera_mode '{camera_mode}'. Valid modes: {valid_modes}")
            return

        if camera_mode == "none":
            return  # Skip camera setup

        elif camera_mode == "manual":
            # Use manual camera settings
            distance = camera_config.get("camera_distance", 10.0)
            yaw = camera_config.get("camera_yaw", 0)
            pitch = camera_config.get("camera_pitch", -89)
            target = camera_config.get("camera_target", [0, 0, 0])

            p.resetDebugVisualizerCamera(
                cameraDistance=distance,
                cameraYaw=yaw,
                cameraPitch=pitch,
                cameraTargetPosition=target,
                physicsClientId=self._client,
            )

            logger.info(f"Camera set to manual mode: distance={distance:.2f}m, yaw={yaw}°, pitch={pitch}°, target={target}")

        elif camera_mode == "auto":
            # Calculate camera from entity positions
            if entity_positions is not None and len(entity_positions) > 0:
                # Use explicitly provided positions
                positions_array = np.array(entity_positions)
                center = list(positions_array.mean(axis=0).astype(float))
                extent = list((positions_array.max(axis=0) - positions_array.min(axis=0)).astype(float))
            else:
                # Use shared bounds computation from sim_objects
                center, extent = self.compute_scene_bounds()
                if center == [0.0, 0.0, 0.0] and extent == [0.0, 0.0, 0.0] and len(self._sim_objects) == 0:
                    logger.warning("Auto camera mode requested but no objects in simulation")
                    return
                logger.info(f"Auto camera: calculated from {len(self._sim_objects)} objects")

            # Get camera settings
            view_type = camera_config.get("camera_view_type", "top_down")
            auto_scale = camera_config.get("camera_auto_scale", 0.8)

            _MIN_CAMERA_DISTANCE = 1.0  # Prevent distance=0 when all objects are co-located
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

            distance = max(distance, _MIN_CAMERA_DISTANCE)
            target = [center[0], center[1], center[2]]

            p.resetDebugVisualizerCamera(
                cameraDistance=distance,
                cameraYaw=yaw,
                cameraPitch=pitch,
                cameraTargetPosition=target,
                physicsClientId=self._client,
            )

            logger.info(f"Camera set to auto mode ({view_type}): distance={distance:.2f}m, target={target}")
            logger.info(f"  Structure extent: [{extent[0]:.2f} x {extent[1]:.2f} x {extent[2]:.2f}] meters")
            logger.info(f"  Center: [{center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}]")
            logger.info(f"  Yaw: {yaw}°, Pitch: {pitch}°")

    def get_aabbs(self) -> List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
        """Get AABBs for all simulation objects."""
        return [p.getAABB(obj.body_id, physicsClientId=self._client) for obj in self._sim_objects]

    def filter_aabb_pairs(
        self, ignore_static: Optional[bool] = None, moved_objects: Optional[Set[int]] = None, return_profiling: bool = False
    ) -> Tuple[List[Tuple[int, int]], Optional[Dict[str, float]]]:
        """
        Filter AABB pairs for collision detection with moved-based optimization.
        Only checks pairs involving moved objects.

        Args:
            ignore_static: If True, skip collision checks with static objects
            moved_objects: Set of object IDs that moved since last check.
                          If None, uses self._moved_this_step (default behavior)
            return_profiling: If True, include timing information in return value

        Returns:
            Tuple of (pairs, timings):
            - pairs: List of (obj_id_i, obj_id_j) tuples for candidate collision pairs
            - timings: dict with get_aabbs, spatial_hashing, aabb_filtering (in ms) if return_profiling=True, else None

        Example::

            # Without profiling
            pairs, _ = sim.filter_aabb_pairs()

            # With profiling
            pairs, timings = sim.filter_aabb_pairs(return_profiling=True)
            print(f"AABB filtering took {timings['total']:.2f}ms")
        """
        # Profiling timings (always create dict for consistent return type)
        timings: Dict[str, float] = {}

        if ignore_static is None:
            ignore_static = self._ignore_static_collision

        # 1. Handle mode changes
        # Check if ignore_static mode has changed (skip if this is the first call)
        if self._last_ignore_static is not None and self._last_ignore_static != ignore_static:
            # Mode changed: force full recalculation regardless of moved_objects argument
            # Override moved_objects directly so the current call scans all objects
            # (setting self._moved_this_step alone would only affect the *next* call)
            moved_objects = set(self._sim_objects_dict.keys())
            # Clear existing collision pairs (will be recalculated with new filter rules)
            self._active_collision_pairs.clear()
            logger.debug(f"ignore_static mode changed to {ignore_static}, resetting collision state and marking all as moved")

        # Update last mode (do this after the check)
        self._last_ignore_static = ignore_static

        # 2. Incremental AABB update
        if return_profiling:
            t0 = time.perf_counter()

        # AABB cache is maintained incrementally by:
        # - add_object(): updates AABB for new objects
        # - remove_object(): removes AABB for deleted objects
        # - set_pose(): updates AABB for kinematic objects when moved
        # - step_once(): updates AABB for physics objects after physics step
        # Only rebuild if cache was explicitly invalidated (mode change)
        if not self._aabb_cache_valid:
            # Full rebuild needed only if cache is completely invalid (e.g., mode change)
            # In kinematics mode, call performCollisionDetection() first to ensure
            # PyBullet's broadphase has initialised collision info for all bodies.
            # Without this, getAABB() emits noisy C++ b3Warning messages.
            if not self._params.physics:
                p.performCollisionDetection(physicsClientId=self._client)
            logger.debug(f"Full AABB rebuild for {len(self._sim_objects)} objects")
            for obj in self._sim_objects:
                self._cached_aabbs_dict[obj.object_id] = p.getAABB(obj.body_id, physicsClientId=self._client)
            # Mark cache as valid after update
            self._aabb_cache_valid = True

        if return_profiling:
            timings["get_aabbs"] = (time.perf_counter() - t0) * 1000  # ms

        # 3. Cell size calculation and spatial grid initialization
        if return_profiling:
            t0 = time.perf_counter()

        # Initialize collision spatial hash if not yet initialized
        if self._cached_cell_size is None:
            self.set_collision_spatial_hash_cell_size_mode()

        # Spatial grid is now maintained incrementally by:
        # - add_object(): adds new objects to grid
        # - remove_object(): removes objects from grid
        # - set_pose(): updates kinematic objects' grid cells
        # - step_once(): updates physics objects' grid cells
        # Initial construction is handled above when cell_size is first calculated

        if return_profiling:
            timings["spatial_hashing"] = (time.perf_counter() - t0) * 1000  # ms

        # 4. AABB overlap filtering (OPTIMIZED: only check pairs involving moved objects)
        if return_profiling:
            t0 = time.perf_counter()

        # Use provided moved_objects or default to self._moved_this_step
        if moved_objects is None:
            moved_objects = self._moved_this_step

        pairs = set()
        tested_pairs = set()  # Track already-tested (i, j) pairs to avoid duplicate AABB checks

        # OPTIMIZATION: Only iterate through moved objects instead of all objects
        # This dramatically reduces computation when few objects move (e.g., 10 moved out of 1000 total)
        for obj_id_i in moved_objects:
            # Safeguard against stale moved_objects entries
            if obj_id_i not in self._sim_objects_dict:
                continue

            # Skip static objects in ignore_static mode
            # Note: In most cases, moved_this_step won't contain static objects,
            # but they may be added during spawn or mode changes
            if ignore_static and obj_id_i in self._static_collision_objects:
                continue

            # Skip disabled collision objects
            if obj_id_i in self._disabled_collision_objects:
                continue

            # Get cells for this moved object (may be multiple for large objects)
            cells_i = self._cached_object_to_cell.get(obj_id_i)
            if cells_i is None or len(cells_i) == 0:
                continue

            mode_i = self._cached_collision_modes.get(obj_id_i, CollisionMode.NORMAL_3D)
            aabb_i = self._cached_aabbs_dict.get(obj_id_i)
            if aabb_i is None:
                continue

            # Choose appropriate neighbor offsets based on collision mode (2D: 9 neighbors, 3D: 27 neighbors)
            offsets = self._NEIGHBOR_OFFSETS_2D if mode_i == CollisionMode.NORMAL_2D else self._NEIGHBOR_OFFSETS_3D

            # Check neighbors in same and adjacent cells for all cells this object occupies
            for cell_i in cells_i:
                for offset in offsets:
                    neighbor_cell = (cell_i[0] + offset[0], cell_i[1] + offset[1], cell_i[2] + offset[2])

                    for obj_id_j in self._cached_spatial_grid.get(neighbor_cell, self._EMPTY_SET):
                        # Skip self-pairs
                        if obj_id_j == obj_id_i:
                            continue

                        # Skip if j is static in ignore_static mode
                        if ignore_static and obj_id_j in self._static_collision_objects:
                            continue

                        # Skip disabled collision objects
                        if obj_id_j in self._disabled_collision_objects:
                            continue

                        # Create sorted pair to avoid duplicates (i < j)
                        pair_key = (obj_id_i, obj_id_j) if obj_id_i < obj_id_j else (obj_id_j, obj_id_i)

                        # Skip already tested pairs
                        if pair_key in tested_pairs:
                            continue
                        tested_pairs.add(pair_key)

                        # Safeguard against stale spatial grid entries
                        if obj_id_j not in self._sim_objects_dict:
                            continue

                        mode_j = self._cached_collision_modes.get(obj_id_j, CollisionMode.NORMAL_3D)

                        # Skip this pair if object j uses 2D mode and offset has Z component
                        if mode_j == CollisionMode.NORMAL_2D and offset[2] != 0:
                            continue

                        # AABB overlap test
                        aabb_j = self._cached_aabbs_dict.get(obj_id_j)
                        if aabb_j is None:
                            continue

                        # Check if AABBs overlap in all 3 axes (continue if NO overlap)
                        # Note: NORMAL_2D's optimisation is in neighbour *search* (9 vs 27 cells),
                        # NOT in skipping the Z-axis AABB check.  Full XYZ overlap is always required
                        # so that objects at different heights within the same Z-cell are correctly
                        # rejected when their AABBs don't actually overlap.
                        if (
                            aabb_i[1][0] < aabb_j[0][0]
                            or aabb_i[0][0] > aabb_j[1][0]
                            or aabb_i[1][1] < aabb_j[0][1]
                            or aabb_i[0][1] > aabb_j[1][1]
                            or aabb_i[1][2] < aabb_j[0][2]
                            or aabb_i[0][2] > aabb_j[1][2]
                        ):
                            continue  # No overlap, skip this pair

                        # AABBs overlap - add to candidate pairs
                        pairs.add(pair_key)

        if return_profiling:
            timings["aabb_filtering"] = (time.perf_counter() - t0) * 1000  # ms
            return list(pairs), timings
        else:
            return list(pairs), None

    def check_collisions(
        self,
        collision_color: Optional[List[float]] = None,
        ignore_static: Optional[bool] = None,
        return_profiling: bool = False,
    ) -> Tuple[List[Tuple[int, int]], Optional[Dict[str, float]]]:
        """
        Check for collisions between robots using incremental updates.

        Only re-checks collision pairs involving moved objects for efficiency.
        Maintains _active_collision_pairs as persistent state.

        Args:
            collision_color: RGB color for collision visualization
            ignore_static: If True, ignore structure collisions
            return_profiling: If True, include timing information in return value

        Returns:
            Tuple of (pairs, timings):
            - pairs: List of currently active collision pairs (obj_id_i, obj_id_j)
            - timings: dict with get_aabbs, spatial_hashing, aabb_filtering, contact_points, total (in ms)
              if return_profiling=True, else None

        Example::

            # Without profiling
            active_pairs, _ = sim.check_collisions()

            # With profiling
            active_pairs, timings = sim.check_collisions(return_profiling=True)
            print(f"Collision check took {timings['total']:.2f}ms")
        """
        t0 = time.perf_counter()

        # Profiling timings (always create dict for consistent return type)
        timings: Dict[str, float] = {}

        # Clear movement tracking from previous collision check
        # This accumulates all movements since the last collision check,
        # which is important when collision_check_frequency < step_frequency
        moved_since_last_check = self._moved_this_step.copy()
        self._moved_this_step.clear()

        if collision_color is None:
            collision_color = self._collision_color

        # Determine ignore_static flag
        if ignore_static is None:
            ignore_static = self._ignore_static_collision

        try:
            # Get candidate pairs (moved-based optimization)
            # Pass moved_since_last_check as argument instead of using self._moved_this_step
            if return_profiling:
                candidate_pairs, filter_timings = self.filter_aabb_pairs(
                    ignore_static=ignore_static, moved_objects=moved_since_last_check, return_profiling=True
                )
                if filter_timings:
                    timings.update(filter_timings)
            else:
                candidate_pairs, _ = self.filter_aabb_pairs(
                    ignore_static=ignore_static, moved_objects=moved_since_last_check, return_profiling=False
                )

            if return_profiling:
                t_contact0 = time.perf_counter()

            # --- Incremental collision detection ---
            # candidate_pairs already contains only pairs involving moved objects (filtered in filter_aabb_pairs)
            # We just need to add existing collision pairs for resolution detection
            pairs_to_check = set(candidate_pairs)

            # Also re-check existing collision pairs involving moved objects
            # (to detect when collisions are resolved)
            # Skip static-static pairs (they never change after initial check)
            for obj_id_i, obj_id_j in list(self._active_collision_pairs):
                # Skip if both objects are static (no need to re-check)
                if obj_id_i in self._static_collision_objects and obj_id_j in self._static_collision_objects:
                    continue

                # Re-check if at least one object moved
                if obj_id_i in moved_since_last_check or obj_id_j in moved_since_last_check:
                    pairs_to_check.add((obj_id_i, obj_id_j))

            # Check contact points for all pairs
            new_collisions = set()
            resolved_collisions = set()

            detection_method = self._params.collision_detection_method

            for obj_id_i, obj_id_j in pairs_to_check:
                obj_i = self._sim_objects_dict.get(obj_id_i)
                obj_j = self._sim_objects_dict.get(obj_id_j)
                if obj_i is None or obj_j is None:
                    # Object was removed — clean up stale pair
                    self._active_collision_pairs.discard((obj_id_i, obj_id_j))
                    continue

                # Select collision detection method based on configuration
                has_collision = False

                if detection_method == CollisionDetectionMethod.CONTACT_POINTS:
                    # Method 1: getContactPoints (physics mode, actual contact manifold)
                    # Best for: physics simulation, requires stepSimulation()
                    # Note: May be unstable for kinematic-kinematic pairs
                    contact_points = p.getContactPoints(obj_i.body_id, obj_j.body_id, physicsClientId=self._client)
                    has_collision = len(contact_points) > 0

                elif detection_method == CollisionDetectionMethod.CLOSEST_POINTS:
                    # Method 2: getClosestPoints (kinematics mode, distance-based)
                    # Best for: kinematics motion, safety margin detection
                    # Works with: Physics ON/OFF, stable with resetBasePositionAndOrientation
                    closest_points = p.getClosestPoints(
                        obj_i.body_id,
                        obj_j.body_id,
                        distance=self._params.collision_margin,  # Safety clearance
                        physicsClientId=self._client,
                    )
                    has_collision = len(closest_points) > 0

                elif detection_method == CollisionDetectionMethod.HYBRID:
                    # Method 3: Hybrid (physics uses getContactPoints, kinematic uses getClosestPoints)
                    # Best for: Mixed physics/kinematics with different detection needs
                    is_physics_i = obj_id_i in self._physics_objects
                    is_physics_j = obj_id_j in self._physics_objects

                    if is_physics_i or is_physics_j:
                        # At least one is physics object - use getContactPoints
                        contact_points = p.getContactPoints(obj_i.body_id, obj_j.body_id, physicsClientId=self._client)
                        has_collision = len(contact_points) > 0
                    else:
                        # Both are kinematic - use getClosestPoints with safety margin
                        closest_points = p.getClosestPoints(
                            obj_i.body_id, obj_j.body_id, distance=self._params.collision_margin, physicsClientId=self._client
                        )
                        has_collision = len(closest_points) > 0

                pair = (obj_id_i, obj_id_j)

                if has_collision:
                    # Collision detected
                    if pair not in self._active_collision_pairs:
                        # New collision
                        new_collisions.add(pair)
                        self._active_collision_pairs.add(pair)
                        logger.info(
                            f"NEW COLLISION: object {obj_id_i} (body {obj_i.body_id}) "
                            f"<-> object {obj_id_j} (body {obj_j.body_id})"
                        )
                else:
                    # No collision
                    if pair in self._active_collision_pairs:
                        # Collision resolved
                        resolved_collisions.add(pair)
                        self._active_collision_pairs.discard(pair)
                        logger.info(
                            f"COLLISION RESOLVED: object {obj_id_i} (body {obj_i.body_id}) "
                            f"<-> object {obj_id_j} (body {obj_j.body_id})"
                        )

            if return_profiling:
                timings["contact_points"] = (time.perf_counter() - t_contact0) * 1000  # ms

            # Update collision count (track total new collisions)
            self._collision_count += len(new_collisions)

            # Color update (collision: blue, normal: original) only if enabled
            if self._params.enable_collision_color_change:
                # Get all currently colliding objects
                currently_colliding = set()
                for obj_id_i, obj_id_j in self._active_collision_pairs:
                    currently_colliding.add(obj_id_i)
                    currently_colliding.add(obj_id_j)

                # Update colors for new collisions (set to collision_color)
                for obj_id_i, obj_id_j in new_collisions:
                    for obj_id in (obj_id_i, obj_id_j):
                        obj = self._sim_objects_dict.get(obj_id)
                        if obj is not None:
                            p.changeVisualShape(obj.body_id, -1, rgbaColor=collision_color, physicsClientId=self._client)

                # Update colors for resolved collisions (restore original color)
                # Only restore if object is not involved in any other collision
                for obj_id_i, obj_id_j in resolved_collisions:
                    for obj_id in (obj_id_i, obj_id_j):
                        if obj_id not in currently_colliding:
                            obj = self._sim_objects_dict.get(obj_id)
                            if obj is not None:
                                orig_color = self._robot_original_colors.get(obj.body_id, [0, 0, 0, 1])
                                p.changeVisualShape(obj.body_id, -1, rgbaColor=orig_color, physicsClientId=self._client)

        except p.error:
            # PyBullet disconnected, skip collision checking
            pass

        if return_profiling:
            timings["total"] = (time.perf_counter() - t0) * 1000  # ms
            return list(self._active_collision_pairs), timings
        else:
            return list(self._active_collision_pairs), None

    def get_active_collision_pairs(self) -> List[Tuple[int, int]]:
        """
        Get currently active collision pairs.

        Returns:
            List of (object_id_i, object_id_j) tuples for all active collisions.
            Pairs are sorted such that object_id_i < object_id_j.

        Example::

            # Check if any collisions are active
            if sim_core.get_active_collision_pairs():
                print("Collisions detected!")

            # Get detailed collision info
            for obj_id_i, obj_id_j in sim_core.get_active_collision_pairs():
                obj_i = sim_core._sim_objects_dict[obj_id_i]
                obj_j = sim_core._sim_objects_dict[obj_id_j]
                print(f"Collision: {obj_i} <-> {obj_j}")
        """
        return list(self._active_collision_pairs)

    def update_monitor(self) -> None:
        """Update simulation monitor with current statistics.

        Calculates actual simulation speed from a 10-second sliding window,
        logs monitor data via logger, and writes to DataMonitor if enabled.
        """
        now = time.time()
        sim_time = self._step_count * self._params.timestep
        # --- Speed history buffer ---
        self._speed_history.append((now, sim_time))
        # Keep only history within 10 seconds (O(1) popleft with deque)
        while self._speed_history and now - self._speed_history[0][0] > 10.0:
            self._speed_history.popleft()
        # Calculate actual RTF for the last 10 seconds
        if len(self._speed_history) >= 2:
            rt0, st0 = self._speed_history[0]
            rt1, st1 = self._speed_history[-1]
            elapsed = rt1 - rt0
            sim_elapsed = st1 - st0
            actual_rtf = sim_elapsed / elapsed if elapsed > 0 else 0
        else:
            actual_rtf = 0
        elapsed_time = now - self._start_time if self._start_time else 0

        # Use len(self._agents) for O(1) agent count instead of O(N) iteration
        num_agents = len(self._agents)
        num_objects = len(self._sim_objects) - num_agents

        monitor_data = {
            "sim_time": sim_time,
            "real_time": elapsed_time,
            "target_rtf": self._params.target_rtf,
            "actual_rtf": actual_rtf,
            "time_step": self._params.timestep,
            "frequency": 1 / self._params.timestep,
            "physics": "enabled" if self._params.physics else "disabled",
            "agents": num_agents,
            "objects": num_objects,
            "active_collisions": len(self._active_collision_pairs),
            "collisions": self._collision_count,
            "steps": self._step_count,
        }
        logger.debug(
            "[MONITOR] sim_time=%.2f, real_time=%.2f, rtf=%.2f, agents=%d, objects=%d, "
            "active_collisions=%d, total_collisions=%d, steps=%d",
            sim_time,
            elapsed_time,
            actual_rtf,
            num_agents,
            num_objects,
            len(self._active_collision_pairs),
            self._collision_count,
            self._step_count,
        )
        # Log only when collision count increases (new collision detected)
        if self._collision_count > self._last_logged_collision_count:
            logger.info(f"NEW COLLISION: total={self._collision_count} at sim_time={sim_time:.2f}")
            self._last_logged_collision_count = self._collision_count
        # Also output to DataMonitor window
        if self._data_monitor:
            self._data_monitor.write_data(monitor_data)

    def initialize_simulation(self) -> None:
        """
        Initialize simulation state before running.

        This method prepares the simulation for execution by:
        - Resetting simulation counters (step_count, collision_count, sim_time)
        - Clearing movement tracking state
        - Configuring visualizer settings (transparency, collision shapes)
        - Enabling rendering for GUI mode

        Called automatically by run_simulation(), but can also be called manually
        for custom simulation loops or when restarting simulation.

        Example::

            # Automatic (recommended)
            sim.run_simulation(duration=10.0)

            # Manual initialization for custom loop
            sim.initialize_simulation()
            while custom_condition:
                sim.step_once()
        """
        # Reset simulation state
        self._start_time = time.time()
        self._step_count = 0
        self._collision_count = 0
        self.sim_time = 0.0
        self._last_collision_check = 0.0
        self._last_monitor_update = 0.0
        self._last_logged_collision_count = 0

        # Reset pause state (#1: prevents stuck-paused on re-run)
        self.resume()

        # Clear movement tracking from any previous simulation
        self._moved_this_step.clear()

        # Clear active collision pairs (fresh start)
        self._active_collision_pairs.clear()

        # Reset filter_aabb_pairs mode-change detection (#6: fresh start)
        self._last_ignore_static = None

        # Clear speed history (#2: prevents stale data in update_monitor)
        self._speed_history.clear()

        # Reset callback last_exec (#3: prevents stale dt on restart)
        for cbinfo in self._callbacks:
            cbinfo["last_exec"] = 0.0

        # Clear profiling statistics (#4: prevents mixing data across runs)
        for key in self._profiling_stats:
            self._profiling_stats[key].clear()

        # Reset memory statistics (#5: prevents stale growth/average data)
        self._memory_stats = {
            "count": 0,
            "current_sum": 0.0,
            "current_min": float("inf"),
            "current_max": 0.0,
            "current_first": 0.0,
            "current_last": 0.0,
            "peak_sum": 0.0,
            "peak_max": 0.0,
        }

        # Configure visualizer after all objects are spawned
        # This ensures transparency and other visual settings are applied correctly
        self.configure_visualizer()

        # Enable rendering before starting simulation
        self.enable_rendering()

        # Start memory profiling if enabled (using is_tracing() for robustness)
        if self._enable_memory_profiling and not tracemalloc.is_tracing():
            # Start with 10 frames for traceback (useful for debugging memory issues)
            tracemalloc.start(10)
            self._memory_tracemalloc_started = True
            logger.info("Memory profiling started using tracemalloc (10 frames)")

        logger.info(f"Simulation initialized: {len(self._sim_objects)} objects, timestep={self._params.timestep}s")

    def reset(self) -> None:
        """Reset simulation to a clean state (no objects, fresh PyBullet world).

        Removes all objects from the simulation, resets PyBullet, re-applies
        physics-engine parameters, optionally reloads the ground plane (only
        when ``enable_floor`` is *True*), and re-initialises counters via
        :meth:`initialize_simulation`.

        After calling ``reset()`` the simulation is ready for new objects to be
        spawned.  Callbacks registered via :meth:`register_callback` are **preserved**
        (their ``last_exec`` is zeroed by ``initialize_simulation``).

        Example::

            sim.reset()
            # All objects removed, PyBullet world cleared
            agent = Agent.from_params(params, sim)
            sim.step_once()
        """
        # 1. Clear all Python-side object tracking
        self._sim_objects.clear()
        self._sim_objects_dict.clear()
        self._agents.clear()

        # Movement / collision caches
        self._moved_this_step.clear()
        self._physics_objects.clear()
        self._kinematic_objects.clear()
        self._static_collision_objects.clear()
        self._disabled_collision_objects.clear()
        self._dynamic_collision_objects.clear()
        self._cached_collision_modes.clear()
        self._cached_aabbs_dict.clear()
        self._cached_spatial_grid.clear()
        self._cached_object_to_cell.clear()
        self._cached_cell_size = None
        self._aabb_cache_valid = False
        self._active_collision_pairs.clear()
        self._robot_original_colors.clear()
        self._original_visual_colors.clear()

        # 2. Reset PyBullet world (removes all bodies including ground plane)
        p.resetSimulation(physicsClientId=self._client)

        # 3. Re-configure PyBullet (gravity, timestep, physics params, ground plane)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81 if self._params.physics else 0, physicsClientId=self._client)
        p.setTimeStep(self._params.timestep, physicsClientId=self._client)
        p.setRealTimeSimulation(0, physicsClientId=self._client)
        self._apply_physics_engine_params()
        if self._params.enable_floor:
            p.loadURDF("plane.urdf", physicsClientId=self._client)

        # 4. Reset object ID counter so IDs start fresh
        self._next_object_id = 0

        # 5. Re-initialise counters, profiling, and rendering
        self.initialize_simulation()

        logger.info("Simulation reset: all objects removed, world reloaded")

    def run_simulation(self, duration: Optional[float] = None) -> None:
        """
        Run the simulation for a specified duration.

        Args:
            duration: Simulation duration in seconds (simulation time, not real time).
                     If None, uses self._params.duration. If duration <= 0, runs indefinitely.

        Example::

            # Run for 10 seconds (simulation time)
            sim.run_simulation(duration=10.0)

            # Run indefinitely (until Ctrl+C or GUI closed)
            sim.run_simulation(duration=0)
        """
        if duration is None:
            duration = self._params.duration

        # Auto-recording from RECORD environment variable (zero-code-change recording)
        record_path = os.environ.get("RECORD")
        if record_path and self._recorder is None:
            record_duration = float(os.environ.get("RECORD_DURATION", "4.0"))
            record_fps = int(os.environ.get("RECORD_FPS", "15"))
            record_width = int(os.environ.get("RECORD_WIDTH", "800"))
            record_height = int(os.environ.get("RECORD_HEIGHT", "600"))
            record_time_base = os.environ.get("RECORD_TIME_BASE", "sim")
            self.start_recording(
                output=record_path,
                duration=record_duration,
                fps=record_fps,
                width=record_width,
                height=record_height,
                time_base=record_time_base,
            )

        # Initialize simulation state (counters, visualizer, rendering)
        # Note: initialize_simulation() already starts tracemalloc if needed
        self.initialize_simulation()

        try:
            start_time = time.time()
            last_step_process_time = 0.0  # Track processing time excluding sleep
            last_pause_state = False  # Track pause state to detect resume

            while True:
                current_sim_time = self._step_count * self._params.timestep

                # Check duration based on simulation time (not real time)
                if duration > 0 and current_sim_time >= duration:
                    break

                # Check if PyBullet connection is still active (e.g., GUI window not closed)
                try:
                    p.getConnectionInfo(physicsClientId=self.client)
                except p.error:
                    logger.info("PyBullet connection lost (GUI window closed)")
                    break

                # target_rtf=0: Run as fast as possible (no synchronization, no sleep)
                if self._params.target_rtf <= 0:
                    self.step_once()
                else:
                    # Speed>0: Synchronize with real time using absolute time calculation
                    loop_start = time.time()
                    current_time = time.time()

                    # Detect pause state change: reset start_time on resume
                    if last_pause_state and not self.is_paused:
                        # Just resumed from pause: reset start_time to avoid jump
                        start_time = current_time - current_sim_time / self._params.target_rtf
                        logger.info("Resumed from pause: start_time reset for smooth continuation")
                    last_pause_state = self.is_paused

                    # Calculate target sim_time using absolute time (stable control)
                    elapsed_time = current_time - start_time
                    target_sim_time = elapsed_time * self._params.target_rtf
                    time_diff = target_sim_time - current_sim_time

                    actual_sleep = 0.0

                    if time_diff > 0:
                        # Behind target: execute multiple steps to catch up
                        steps_needed = int(time_diff / self._params.timestep)
                        steps_needed = max(1, min(steps_needed, self._params.max_steps_per_frame))
                        for _ in range(steps_needed):
                            self.step_once()
                    else:
                        # Ahead of or at target: sleep until next frame
                        # Calculate sleep time: time_diff - last processing time
                        sleep_time = abs(time_diff) - last_step_process_time

                        # Determine minimum sleep for GUI responsiveness
                        if self._params.gui:
                            min_sleep = 1.0 / self._params.gui_min_fps
                        else:
                            # For non-GUI, use timestep as minimum
                            min_sleep = self._params.timestep

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
            logger.warning("Simulation interrupted by user")
        except RuntimeError as e:
            if "PyBullet disconnected" in str(e):
                logger.info("PyBullet connection lost (GUI window closed)")
            else:
                raise
        except p.error:
            logger.info("PyBullet connection lost (GUI window closed)")
        self.update_monitor()

        # Auto-save recording if active
        if self._recorder is not None:
            self.stop_recording()

        # Disconnect from PyBullet if still connected
        try:
            p.getConnectionInfo(physicsClientId=self.client)
            p.disconnect(self.client)
        except p.error:
            # Already disconnected
            pass

    # ------------------------------------------------------------------
    # Pause / Resume
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Custom Profiling Fields
    # ------------------------------------------------------------------

    def record_profiling(self, name: str, value_ms: float) -> None:
        """Record a custom profiling measurement (in milliseconds).

        Call this from within Agent.update() or callback code to accumulate
        timing data. Multiple calls per step are summed (e.g., 100 agents each
        recording their custom_logic time). Auto-registers the field on first use.

        The recorded field appears in both _print_profiling_summary() output and
        step_once(return_profiling=True) results alongside built-in fields.

        Args:
            name: Field name (e.g., 'custom_logic', 'planner').
                  Auto-registered on first call — no separate setup needed.
            value_ms: Time measurement in milliseconds

        Example::

            # Inside your custom Agent.update():
            t0 = time.perf_counter()
            self._do_custom_logic(dt)
            self.sim_core.record_profiling('custom_logic', (time.perf_counter() - t0) * 1000)
        """
        data = self._profiling_stats.get(name)
        if data is None:
            # Auto-register: create with initial 0.0 slot for current step
            self._profiling_stats[name] = [value_ms]
            return
        if data:
            data[-1] += value_ms

    def pause(self) -> None:
        """Pause the simulation. step_once will be skipped while paused."""
        self._simulation_paused = True

    def resume(self) -> None:
        """Resume the simulation after a pause."""
        self._simulation_paused = False

    @property
    def is_paused(self) -> bool:
        """Return True if the simulation is currently paused."""
        return self._simulation_paused

    def step_once(self, return_profiling: bool = False) -> Optional[Dict[str, Any]]:
        """
        Execute one simulation step.

        Performs agent updates, callbacks, physics step, collision detection, and monitoring.
        If enable_time_profiling is True, detailed timing information is logged.

        Args:
            return_profiling: If True, return timing breakdown dictionary instead of logging.
                             Useful for external profiling tools.

        Returns:
            If return_profiling=True, returns dict with timing breakdown in milliseconds::

                {
                    'agent_update': float,
                    'callbacks': float,
                    'step_simulation': float,
                    'collision_check': float,
                    'collision_breakdown': {       # per-phase detail (present when collision ran)
                        'get_aabbs': float,
                        'spatial_hashing': float,
                        'aabb_filtering': float,
                        'contact_points': float,
                        'total': float,
                    },
                    '<custom_field>': float,       # present when custom fields registered
                    'monitor_update': float,
                    'total': float,
                }

            Otherwise returns None.

        Performance note: time.perf_counter() calls have negligible overhead (<0.1% for 10k objects).
        The profiling measurements themselves do not significantly impact simulation performance.
        """
        # Profiling: step start time (measure even if return_profiling=True)
        measure_timing = self._enable_time_profiling or return_profiling
        if measure_timing:
            t_step = time.perf_counter()

        # Check if PyBullet is still connected
        try:
            p.getConnectionInfo(physicsClientId=self.client)
        except p.error:
            # PyBullet disconnected, skip this step
            return

        # Handle keyboard events for visual/collision shape toggling and pause
        if self._keyboard_events_registered:
            self._handle_keyboard_events()

        # Skip physics simulation and callbacks if paused
        if self.is_paused:
            return

        # Initialize profiling accumulators for this step (append 0.0 slot for ALL fields)
        # Placed after pause/disconnect checks so paused steps don't accumulate stale entries
        if measure_timing:
            for data in self._profiling_stats.values():
                data.append(0.0)

        # Physics objects are always considered as potentially moved (conservative approach)
        # This avoids expensive pose comparison every step
        # Note: _moved_this_step is cleared in check_collisions() to accumulate movements
        # across multiple steps when collision_check_frequency < step_frequency
        self._moved_this_step.update(self._physics_objects)

        self.sim_time = self._step_count * self._params.timestep

        # Update all simulation objects that have update() method
        # Agent instances are automatically updated every step for movement control
        # OPTIMIZATION: Use self._agents list for O(M) iteration instead of O(N) with isinstance() check
        if measure_timing:
            t0 = time.perf_counter()

        for agent in self._agents:
            moved = agent.update(self._params.timestep)
            # Track kinematic agent movement
            if moved and agent.object_id in self._kinematic_objects:
                self._moved_this_step.add(agent.object_id)

        if measure_timing:
            t1 = time.perf_counter()
            self._profiling_stats["agent_update"][-1] = (t1 - t0) * 1000

        # Global callbacks (frequency control)
        if measure_timing:
            t_cb0 = time.perf_counter()

        for cbinfo in list(self._callbacks):
            freq = cbinfo.get("frequency", None)
            last_exec = cbinfo.get("last_exec", 0.0)
            interval = 1.0 / freq if freq else 0.0
            # Judge based on self.sim_time
            if freq is None or self.sim_time - last_exec >= interval:
                dt = self.sim_time - last_exec if last_exec > 0 else self._params.timestep
                cbinfo["func"](self, dt)
                cbinfo["last_exec"] = self.sim_time

        if measure_timing:
            t_cb1 = time.perf_counter()
            self._profiling_stats["callbacks"][-1] = (t_cb1 - t_cb0) * 1000

        # Check if PyBullet is still connected before stepping
        if not p.isConnected(physicsClientId=self.client):
            raise RuntimeError("PyBullet disconnected (GUI window closed)")

        if measure_timing:
            t_sim0 = time.perf_counter()

        # stepSimulation() control based on physics mode
        # Physics ON: Call stepSimulation() every step (rigid body integration, contact resolution)
        # Physics OFF: Skip stepSimulation() for pure kinematics (position updates via reset API)
        if self._params.physics:
            p.stepSimulation(physicsClientId=self._client)
        # Note: Even in Physics OFF mode, collision detection still works via getClosestPoints()
        # which queries geometry directly without requiring stepSimulation()

        # Update AABBs and spatial grid for physics objects AFTER stepSimulation()
        # This ensures collision detection uses current-frame positions, not stale ones.
        # Kinematic objects update their AABBs and spatial grid in set_pose() for immediate consistency.
        for obj_id in self._physics_objects:
            # Safeguard against stale _physics_objects entries
            if obj_id not in self._sim_objects_dict:
                continue
            self._update_object_aabb(obj_id)
            # Update spatial grid if cell_size is initialized
            if self._cached_cell_size is not None:
                self._update_object_spatial_grid(obj_id)

        if measure_timing:
            t_sim1 = time.perf_counter()
            self._profiling_stats["step_simulation"][-1] = (t_sim1 - t_sim0) * 1000

        # Collision check frequency control
        if measure_timing:
            t_col0 = time.perf_counter()

        collision_breakdown = None  # per-phase breakdown (only when return_profiling=True)
        freq = self._collision_check_frequency
        # freq = None: check every step
        # freq = 0: disabled (never check)
        # freq > 0: check at specified frequency (Hz)
        if freq is None:
            # Check every step
            _, collision_breakdown = self.check_collisions(return_profiling=return_profiling)
            self._last_collision_check = self.sim_time
        elif freq > 0:
            # Check at specified frequency
            interval = 1.0 / freq
            if self.sim_time - self._last_collision_check >= interval:
                _, collision_breakdown = self.check_collisions(return_profiling=return_profiling)
                self._last_collision_check = self.sim_time
        # else: freq = 0, skip collision checks entirely

        if measure_timing:
            t_col1 = time.perf_counter()
            self._profiling_stats["collision_check"][-1] = (t_col1 - t_col0) * 1000

        self._step_count += 1
        # Monitor: every step if GUI enabled, otherwise every second
        if measure_timing:
            t_mon0 = time.perf_counter()

        if self._monitor_enabled:
            interval = self._params.timestep if self._params.gui else 1.0
            if self.sim_time - self._last_monitor_update > interval:
                self.update_monitor()
                self._last_monitor_update = self.sim_time

        if measure_timing:
            t_mon1 = time.perf_counter()
            t_end = time.perf_counter()
            self._profiling_stats["monitor_update"][-1] = (t_mon1 - t_mon0) * 1000
            self._profiling_stats["total"][-1] = (t_end - t_step) * 1000

        # Return profiling data if requested (pop values from _profiling_stats)
        if return_profiling:
            result: Dict[str, Any] = {name: data.pop() for name, data in self._profiling_stats.items() if data}
            if collision_breakdown is not None:
                result["collision_breakdown"] = collision_breakdown
            return result

        # Profiling output (only when time profiling is enabled and not returning data)
        if self._enable_time_profiling:
            # All values already written to _profiling_stats via [-1] assignment/accumulation

            # Print average statistics every N steps
            if self._step_count % self._profiling_interval == 0 and len(self._profiling_stats["total"]) > 0:
                self._print_profiling_summary()

        # Memory profiling output (O(1) aggregator-based statistics)
        if self._enable_memory_profiling and tracemalloc.is_tracing():
            # Get memory usage (no import needed - already at top)
            current, peak = tracemalloc.get_traced_memory()
            current_mb = current / 1024 / 1024  # Convert to MB
            peak_mb = peak / 1024 / 1024  # Convert to MB

            # Update aggregators (O(1) memory)
            count = self._memory_stats["count"] + 1
            self._memory_stats["count"] = count
            self._memory_stats["current_sum"] += current_mb
            self._memory_stats["current_min"] = min(self._memory_stats["current_min"], current_mb)
            self._memory_stats["current_max"] = max(self._memory_stats["current_max"], current_mb)
            self._memory_stats["peak_sum"] += peak_mb
            self._memory_stats["peak_max"] = max(self._memory_stats["peak_max"], peak_mb)

            # Track first and last samples for growth calculation
            if count == 1:
                self._memory_stats["current_first"] = current_mb
            self._memory_stats["current_last"] = current_mb

            # Print average statistics every N steps
            if self._step_count % self._profiling_interval == 0 and count > 0:
                self._print_memory_profiling_summary()

    def _print_profiling_summary(self) -> None:
        """Print profiling statistics summary (average over last N steps).

        Prints all fields in _profiling_stats (built-in + custom) in a single line.
        Custom fields recorded via record_profiling() appear alongside
        built-in components automatically.
        """
        if not self._profiling_stats["total"]:
            return

        total_avg = statistics.mean(self._profiling_stats["total"])
        num_samples = len(self._profiling_stats["total"])

        # Print all fields from _profiling_stats dict (preserves insertion order)
        # Built-in fields come first (defined in __init__), custom fields follow
        stats_info = []
        for name, data in self._profiling_stats.items():
            if not data:
                continue
            avg = statistics.mean(data)
            percentage = (avg / total_avg * 100) if total_avg > 0 else 0
            stats_info.append(f"{name}={avg:.2f}ms ({percentage:.1f}%)")

        logger.info(f"[PROFILING] Last {num_samples} steps average: " + ", ".join(stats_info))

        # Clear all statistics for next interval
        for data in self._profiling_stats.values():
            data.clear()

    def _print_memory_profiling_summary(self) -> None:
        """
        Print memory profiling statistics summary (aggregator-based, O(1) memory).

        Note: Reports Python heap memory tracked by tracemalloc, not RSS (process memory).
        For RSS, use psutil.Process().memory_info().rss separately.
        """
        count = self._memory_stats["count"]
        if count == 0:
            return

        # Calculate statistics from aggregators
        current_avg = self._memory_stats["current_sum"] / count
        current_min = self._memory_stats["current_min"]
        current_max = self._memory_stats["current_max"]
        peak_avg = self._memory_stats["peak_sum"] / count
        peak_max = self._memory_stats["peak_max"]

        # Calculate memory growth (difference between first and last sample)
        memory_growth = self._memory_stats["current_last"] - self._memory_stats["current_first"]
        growth_str = f", growth={memory_growth:+.2f}MB"

        logger.info(
            f"[MEMORY] Last {count} steps: "
            f"current={current_avg:.2f}MB (min={current_min:.2f}, max={current_max:.2f}), "
            f"peak={peak_avg:.2f}MB (max={peak_max:.2f}){growth_str}"
        )

        # Reset aggregators for next interval (O(1) memory)
        self._memory_stats["count"] = 0
        self._memory_stats["current_sum"] = 0.0
        self._memory_stats["current_min"] = float("inf")
        self._memory_stats["current_max"] = 0.0
        self._memory_stats["current_first"] = 0.0
        self._memory_stats["current_last"] = 0.0
        self._memory_stats["peak_sum"] = 0.0
        self._memory_stats["peak_max"] = 0.0

        # Reset peak memory for interval-based peak tracking (requires Python 3.9+)
        # This makes "peak" meaningful for each interval rather than cumulative
        if tracemalloc.is_tracing() and hasattr(tracemalloc, "reset_peak"):
            tracemalloc.reset_peak()
        # Note: Without reset_peak (Python <3.9), peak becomes cumulative over entire run

    def get_memory_usage(self) -> Optional[Dict[str, float]]:
        """
        Get current memory usage (requires enable_memory_profiling=True).

        Returns:
            Dictionary with current and peak memory usage in MB, or None if not enabled.
            Keys: 'current_mb', 'peak_mb'

        Note:

            - Returns **Python heap memory** tracked by tracemalloc, NOT RSS (process memory).
            - tracemalloc tracks Python object allocations, not C extensions or OS-level memory.
            - For total process memory (RSS), use: psutil.Process().memory_info().rss
            - Peak is cumulative since last reset_peak() call (interval-based in profiling).

        Example::

            >>> sim = MultiRobotSimulationCore.from_dict(config)
            >>> # ... run simulation with enable_memory_profiling=True ...
            >>> mem = sim.get_memory_usage()
            >>> if mem:
            >>>     print(f"Python heap: {mem['current_mb']:.2f} MB, Peak: {mem['peak_mb']:.2f} MB")
        """
        if not self._enable_memory_profiling or not tracemalloc.is_tracing():
            return None

        current, peak = tracemalloc.get_traced_memory()
        return {"current_mb": current / 1024 / 1024, "peak_mb": peak / 1024 / 1024}
