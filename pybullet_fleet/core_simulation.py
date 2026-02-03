"""
core_simulation.py
Reusable core simulation logic for multi-robot PyBullet environments.
Integrates generation, management, transport, attach/detach, collision detection,
coordinate conversion, occupied judgment, transport path generation, debugging,
monitoring, and log control for various robots, pallets, and meshes.
"""

import logging
import os
import time
from typing import Any, Callable, Dict, List, Optional, Set, Tuple, Union

import numpy as np

# --- All imports at the top for PEP8 compliance ---
import pybullet as p
import pybullet_data

import yaml

from pybullet_fleet.collision_visualizer import CollisionVisualizer
from pybullet_fleet.data_monitor import DataMonitor
from pybullet_fleet.sim_object import SimObject
from pybullet_fleet.agent import Agent
from pybullet_fleet.types import SpatialHashCellSizeMode, CollisionDetectionMethod, CollisionMode

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
            enable_monitor_gui=config.get("enable_monitor_gui", True),
            collision_check_frequency=config.get("collision_check_frequency", None),
            log_level=config.get("log_level", "warn"),
            max_steps_per_frame=config.get("max_steps_per_frame", 10),
            gui_min_fps=config.get("gui_min_fps", 30),
            # Visualizer settings
            enable_collision_shapes=config.get("enable_collision_shapes", False),
            enable_structure_transparency=config.get("enable_structure_transparency", False),
            enable_shadows=config.get("enable_shadows", True),
            enable_gui_panel=config.get("enable_gui_panel", False),  # Default: hide GUI panel
            ignore_static_collision=config.get("ignore_static_collision", True),
            enable_profiling=config.get("enable_profiling", False),
            profiling_interval=config.get("profiling_interval", 100),
            enable_collision_color_change=config.get("enable_collision_color_change", False),
            spatial_hash_cell_size_mode=SpatialHashCellSizeMode(config.get("spatial_hash_cell_size_mode", "auto_initial")),
            spatial_hash_cell_size=config.get("spatial_hash_cell_size", None),
            # Collision detection method - auto-select based on physics mode
            collision_detection_method=CollisionDetectionMethod(
                config.get(
                    "collision_detection_method", "contact_points" if config.get("physics", False) else "closest_points"
                )
            ),
            collision_margin=config.get("collision_margin", 0.02),  # Safety clearance (meters)
            multi_cell_threshold=config.get("multi_cell_threshold", 1.5),  # Multi-cell registration threshold
            camera_config=config.get("camera", {}),  # Camera configuration
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
        enable_monitor_gui: bool = True,
        collision_check_frequency: Optional[float] = None,
        log_level: str = "warn",
        max_steps_per_frame: int = 10,
        gui_min_fps: int = 30,
        enable_collision_shapes: bool = False,
        enable_structure_transparency: bool = False,
        enable_shadows: bool = True,
        enable_gui_panel: bool = False,
        ignore_static_collision: bool = True,
        enable_profiling: bool = False,
        profiling_interval: int = 100,
        enable_collision_color_change: bool = False,
        spatial_hash_cell_size_mode: SpatialHashCellSizeMode = SpatialHashCellSizeMode.AUTO_INITIAL,
        spatial_hash_cell_size: Optional[float] = None,  # Used when mode=CONSTANT
        collision_detection_method: Optional[CollisionDetectionMethod] = None,  # Auto-select based on physics
        collision_margin: float = 0.02,  # Safety clearance for getClosestPoints (meters, default: 2cm)
        multi_cell_threshold: float = 1.5,  # Threshold multiplier for multi-cell registration (>= 1.0)
        camera_config: Optional[Dict[str, Any]] = None,  # Camera configuration from config file
    ) -> None:
        self.speed = speed  # speed=0 means maximum speed (no sleep)
        self.timestep = timestep
        self.duration = duration
        self.gui = gui
        self.physics = physics
        self.monitor = monitor
        self.enable_monitor_gui = enable_monitor_gui
        self.collision_check_frequency = collision_check_frequency
        self.log_level = log_level
        self.max_steps_per_frame = max_steps_per_frame  # Maximum simulation steps per rendering frame
        self.gui_min_fps = gui_min_fps  # Minimum FPS for GUI responsiveness (default: 30 FPS = 33ms)
        # Visualizer settings
        self.enable_collision_shapes = enable_collision_shapes
        self.enable_structure_transparency = enable_structure_transparency
        self.enable_shadows = enable_shadows
        self.enable_gui_panel = enable_gui_panel
        self.ignore_static_collision = ignore_static_collision
        self.enable_profiling = enable_profiling
        self.profiling_interval = profiling_interval  # Steps between profiling reports
        self.enable_collision_color_change = enable_collision_color_change
        self.spatial_hash_cell_size_mode = spatial_hash_cell_size_mode
        self.spatial_hash_cell_size = spatial_hash_cell_size  # Fixed cell size (for mode=CONSTANT)
        self.multi_cell_threshold = multi_cell_threshold  # Threshold for multi-cell registration

        # Auto-select collision detection method based on physics mode if not explicitly set
        if collision_detection_method is None:
            # Physics ON: use CONTACT_POINTS (actual contact manifold)
            # Physics OFF: use CLOSEST_POINTS (distance-based, kinematics-safe)
            collision_detection_method = (
                CollisionDetectionMethod.CONTACT_POINTS if physics else CollisionDetectionMethod.CLOSEST_POINTS
            )
        self.collision_detection_method = collision_detection_method
        self.collision_margin = collision_margin  # Safety clearance for collision detection
        self.camera_config = camera_config if camera_config is not None else {}  # Camera configuration


class MultiRobotSimulationCore:
    # Precomputed neighbor offsets for spatial hashing (class constants)
    _NEIGHBOR_OFFSETS_3D = tuple(
        (dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
    )  # 27 neighbors for 3D collision detection

    _NEIGHBOR_OFFSETS_2D = tuple(
        (dx, dy, 0) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
    )  # 9 neighbors for 2D collision detection (Z=0 only)

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
        self._sim_objects_dict: Dict[int, SimObject] = {}  # Dict for O(1) lookup: object_id -> SimObject
        self.agents: List[Agent] = []  # List of Agent instances only for O(M) iteration in step_once()
        self._last_collided: set = set()
        self._robot_original_colors: Dict[int, List[float]] = {}  # body_id: rgbaColor
        self.collision_count: int = 0
        self.step_count: int = 0
        self.sim_time: float = 0.0  # Simulation time
        self.start_time: Optional[float] = None
        self._next_object_id: int = 0  # Counter for unique object IDs
        self._last_logged_collision_count: int = 0  # Track last logged collision count
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
        )  # Store original colors: (body_id, link_index) -> rgba
        self._structure_transparent: bool = False  # Track if structure is transparent
        self.ignore_static_collision: bool = (
            params.ignore_static_collision
        )  # If True, skip collision checks with static objects
        self._simulation_paused: bool = False  # Track if simulation is paused
        
        # --- Profiling configuration ---
        self.enable_profiling: bool = params.enable_profiling  # Master profiling switch
        self._profiling_interval: int = params.profiling_interval  # Report average every N steps
        
        # --- Profiling statistics ---
        self._profiling_stats: Dict[str, List[float]] = {
            'agent_update': [],
            'callbacks': [],
            'step_simulation': [],
            'collision_check': [],
            'monitor_update': [],
            'total': []
        }

        # --- Movement tracking and collision optimization ---
        self._moved_this_step: set = set()  # object_ids that moved this step
        self._physics_objects: set = set()  # object_ids with physics enabled (can move via collisions)
        self._kinematic_objects: set = set()  # object_ids that move only via set_pose/update
        self._static_objects: set = set()  # object_ids that never move (static structures)
        self._disabled_collision_objects: set = set()  # object_ids with collision completely disabled

        # --- Collision detection cache ---
        self._cached_collision_modes: Dict[int, CollisionMode] = {}  # object_id -> CollisionMode
        self._cached_cell_size: Optional[float] = None  # Cached cell size for spatial hashing
        self._cached_non_static_dict: Dict[int, Tuple[SimObject, CollisionMode]] = (
            {}
        )  # object_id -> (SimObject, collision_mode) for O(1) add/remove (excludes STATIC and DISABLED)

        # Incremental AABB and spatial grid cache
        self._cached_aabbs_dict: Dict[int, Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = (
            {}
        )  # object_id -> AABB
        self._cached_spatial_grid: Dict[Tuple[int, int, int], Set[int]] = {}  # Spatial hash grid: cell -> {object_ids}
        self._cached_object_to_cell: Dict[int, List[Tuple[int, int, int]]] = {}  # object_id -> list of cells (supports multi-cell registration)
        self._aabb_cache_valid: bool = False  # Flag indicating if AABB cache is valid
        self._last_ignore_static: Optional[bool] = None  # Track last ignore_static value to detect mode changes

        # Incremental collision tracking
        self._active_collision_pairs: Set[Tuple[int, int]] = set()  # Currently colliding object_id pairs (sorted: i < j)

        self.setup_pybullet()
        self.setup_monitor()

    def setup_monitor(self) -> None:
        # If monitor: true and console_monitor: false, start DataMonitor

        if self.params.monitor:
            self.data_monitor = DataMonitor("PyBullet Simulation Monitor", enable_gui=self.params.enable_monitor_gui)
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
        Set the frequency (Hz, number of times per second) for collision detection.

        Args:
            frequency: Collision check frequency in Hz (times per second)
                      - None: Check every step (highest accuracy, highest cost)
                      - 0: Disable collision checking (no collision detection)
                      - > 0: Check at specified frequency (e.g., 1.0 = once per second)

        Example:
            sim.set_collision_check_frequency(None)  # Every step
            sim.set_collision_check_frequency(1.0)   # 1 Hz (once per second)
            sim.set_collision_check_frequency(10.0)  # 10 Hz
            sim.set_collision_check_frequency(0)     # Disable

        Note: Higher frequency increases computational cost but improves collision detection accuracy.
        """
        self.collision_check_frequency = frequency

        # Log the setting for clarity
        if frequency is None:
            logger.info("Collision check frequency set to: EVERY STEP (highest accuracy)")
        elif frequency == 0:
            logger.info("Collision check frequency set to: DISABLED (no collision detection)")
        else:
            logger.info(f"Collision check frequency set to: {frequency} Hz ({1.0/frequency:.3f}s interval)")

    def set_profiling_log_frequency(self, frequency: int = 10) -> None:
        """
        Set the frequency for profiling log output (in steps).

        Args:
            frequency: Log profiling info every N steps (default: 10)
                      - 1: Every step (high overhead, detailed)
                      - 10: Every 10 steps (low overhead, recommended)
                      - 100: Every 100 steps (minimal overhead)

        Note: Profiling must be enabled (enable_profiling=True) for logs to appear.
        """
        self._profiling_log_frequency = max(1, frequency)
        logger.info(f"Profiling log frequency set to every {self._profiling_log_frequency} steps")

    def setup_pybullet(self) -> None:
        """Initialize PyBullet with GUI panels hidden."""
        self.client = p.connect(p.GUI if self.params.gui else p.DIRECT)

        # Hide all debug UI panels immediately after connection
        if self.params.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

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

        Example:
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
            self._cached_aabbs_dict[object_id] = p.getAABB(obj.body_id, physicsClientId=self.client)
            logger.debug(f"Updated AABB for object {object_id}")

            # Also update spatial grid if requested and cell_size is initialized
            if update_grid and self._cached_cell_size is not None:
                self._update_object_spatial_grid(object_id)
        except p.error:
            logger.warning(f"Failed to update AABB for object {object_id} (body {obj.body_id})")

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
        threshold_size = self._cached_cell_size * self.params.multi_cell_threshold

        return max_extent > threshold_size

    def _get_overlapping_cells(self, object_id: int) -> List[Tuple[int, int, int]]:
        """
        Get all grid cells that overlap with object's AABB.

        Used for large objects that span multiple cells to ensure
        collision detection with objects in all neighboring cells.

        Args:
            object_id: The object_id to get overlapping cells for

        Returns:
            List of cell tuples (x, y, z) that overlap with the object's AABB

        Example:
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
        import math

        cells = []
        for d in range(3):  # x, y, z
            min_cell = int(math.floor(aabb_min[d] / self._cached_cell_size))
            max_cell = int(math.floor(aabb_max[d] / self._cached_cell_size))

            if d == 0:
                x_range = range(min_cell, max_cell + 1)
            elif d == 1:
                y_range = range(min_cell, max_cell + 1)
            else:  # d == 2
                z_range = range(min_cell, max_cell + 1)

        # Generate all cell combinations
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    cells.append((x, y, z))

        return cells

    def _update_object_spatial_grid(self, object_id: int, remove_only: bool = False) -> None:
        """
        Update an object's position in the spatial grid.

        This unified method handles:
        - Initial addition: old_cells=None → add to new cell(s)
        - Movement update: old_cells exists → remove from old, add to new
        - Removal: remove_only=True → remove from current cell(s) only
        - Multi-cell registration: Large objects registered to all overlapping cells

        Args:
            object_id: The object_id to update in spatial grid
            remove_only: If True, only remove from grid without adding to new cell(s)
                        Used when objects become static or are deleted

        Example:
            # First time (add)
            sim._update_object_spatial_grid(obj_id)  # Adds to cell(s)

            # After movement (update)
            sim._update_object_spatial_grid(obj_id)  # Removes from old cell(s), adds to new

            # Remove from grid
            sim._update_object_spatial_grid(obj_id, remove_only=True)  # Only removes
        """
        # Remove from old cells (if exists)
        old_cells = self._cached_object_to_cell.get(object_id)
        if old_cells is not None:
            for old_cell in old_cells:
                if old_cell in self._cached_spatial_grid:
                    self._cached_spatial_grid[old_cell].discard(object_id)
                    # Clean up empty cells
                    if not self._cached_spatial_grid[old_cell]:
                        del self._cached_spatial_grid[old_cell]
            del self._cached_object_to_cell[object_id]

        # If remove_only, skip addition and return
        if remove_only:
            if old_cells is not None:
                logger.debug(f"Removed object {object_id} from {len(old_cells)} spatial grid cell(s)")
            return

        # For add/update: ensure AABB exists
        if object_id not in self._cached_aabbs_dict:
            obj = self._sim_objects_dict.get(object_id)
            if obj is None:
                logger.warning(f"Cannot update spatial grid for object {object_id}: object not found")
                return
            # Update AABB without triggering spatial grid update (avoid circular call)
            self._update_object_aabb(object_id, update_grid=False)
            if object_id not in self._cached_aabbs_dict:
                logger.warning(f"Cannot update spatial grid for object {object_id}: AABB update failed")
                return

        if self._cached_cell_size is None:
            return

        # Determine if multi-cell registration is needed
        use_multi_cell = self._should_use_multi_cell_registration(object_id)

        if use_multi_cell:
            # Large object: register to all overlapping cells
            cells = self._get_overlapping_cells(object_id)
        else:
            # Normal object: register to single cell (center)
            aabb = self._cached_aabbs_dict[object_id]
            center = [0.5 * (aabb[0][d] + aabb[1][d]) for d in range(3)]
            cell = tuple(int(center[d] // self._cached_cell_size) for d in range(3))
            cells = [cell]

        # Register to all cells
        self._cached_object_to_cell[object_id] = cells
        for cell in cells:
            self._cached_spatial_grid.setdefault(cell, set()).add(object_id)

        # Log appropriate message based on whether this is add or update
        if old_cells is None:
            logger.debug(f"Added object {object_id} to {len(cells)} spatial grid cell(s) (multi-cell: {use_multi_cell})")
        else:
            logger.debug(
                f"Updated object {object_id} spatial grid: {len(old_cells)} -> {len(cells)} cell(s) (multi-cell: {use_multi_cell})"
            )

    def _register_object_movement_type(self, obj: SimObject) -> None:
        """
        Register an object's movement type for optimization.

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

        # Register based on movement type (not collision mode)
        if obj.is_static:
            # STATIC: Never moves
            self._static_objects.add(object_id)
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
        obj = self._sim_objects_dict.get(object_id)
        if obj is None:
            logger.warning(f"Cannot update collision mode for object {object_id}: not found")
            return

        # Remove from old movement type sets
        self._static_objects.discard(object_id)
        self._physics_objects.discard(object_id)
        self._kinematic_objects.discard(object_id)

        # Re-register with new mode
        self._register_object_movement_type(obj)

        # Update collision mode cache
        self._cached_collision_modes[object_id] = new_mode

        # Handle collision system updates based on mode transition
        # Structure: First check old_mode, then check new_mode within each case

        if old_mode == CollisionMode.DISABLED:
            # DISABLED -> *: Need to add object to collision system
            if new_mode == CollisionMode.DISABLED:
                # No change
                pass
            elif new_mode == CollisionMode.STATIC:
                # DISABLED -> STATIC: Add to collision system (STATIC not in non_static_dict)
                self._add_object_to_collision_system(object_id)
            else:
                # DISABLED -> NORMAL_3D/2D: Add to collision system and non_static_dict
                self._cached_non_static_dict[object_id] = (obj, new_mode)
                self._add_object_to_collision_system(object_id)

        elif old_mode == CollisionMode.STATIC:
            # STATIC -> *: STATIC already has AABB/grid
            if new_mode == CollisionMode.DISABLED:
                # STATIC -> DISABLED: Remove from collision system
                self._remove_object_from_collision_system(object_id)
            elif new_mode == CollisionMode.STATIC:
                # No change
                pass
            else:
                # STATIC -> NORMAL_3D/2D: Add to non_static_dict, mark as moved
                self._cached_non_static_dict[object_id] = (obj, new_mode)
                self._moved_this_step.add(object_id)

        else:
            # NORMAL_3D/2D -> *: Currently in non_static_dict
            if new_mode == CollisionMode.DISABLED:
                # NORMAL -> DISABLED: Remove from collision system and non_static_dict
                self._cached_non_static_dict.pop(object_id, None)
                self._remove_object_from_collision_system(object_id)
            elif new_mode == CollisionMode.STATIC:
                # NORMAL -> STATIC: Remove from non_static_dict, keep AABB/grid
                self._cached_non_static_dict.pop(object_id, None)
            else:
                # NORMAL_3D <-> NORMAL_2D: Just update mode in non_static_dict
                self._cached_non_static_dict[object_id] = (obj, new_mode)
                self._moved_this_step.add(object_id)

        logger.info(f"Updated collision mode for object {object_id}: {old_mode.value} -> {new_mode.value}")

    def _add_object_to_collision_system(self, object_id: int) -> None:
        """
        Add object to collision detection system.

        Updates AABB cache, spatial grid, and marks as moved.
        Called when object is added or when collision mode changes from DISABLED to NORMAL/STATIC.

        Note: This method should NEVER be called for DISABLED collision mode objects.
              When transitioning from DISABLED to enabled mode, removes from _disabled_collision_objects.

        Args:
            object_id: Object ID to add
        """
        # Remove from disabled set (transitioning from DISABLED to enabled mode)
        self._disabled_collision_objects.discard(object_id)

        self._update_object_aabb(object_id, update_grid=True)
        self._moved_this_step.add(object_id)
        logger.debug(f"Added object {object_id} to collision system")

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

    def set_collision_spatial_hash_cell_size_mode(
        self, mode: Optional[SpatialHashCellSizeMode] = None, cell_size: Optional[float] = None
    ) -> Optional[float]:
        """
        Set collision detection spatial hash cell size mode and recalculate/rebuild grid.

        This method allows dynamic mode switching and always recalculates the cell_size
        based on the spatial_hash_cell_size_mode setting (or override via parameters).

        Args:
            mode: Override spatial_hash_cell_size_mode for this calculation (optional)
            cell_size: Override spatial_hash_cell_size when mode=CONSTANT (optional)

        Returns:
            The calculated cell_size, or None if no AABBs are available

        Example:
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
            old_mode = self.params.spatial_hash_cell_size_mode
            self.params.spatial_hash_cell_size_mode = mode
            logger.debug(f"Updated mode: {old_mode} -> {mode}")

        if cell_size is not None:
            old_size = self.params.spatial_hash_cell_size
            self.params.spatial_hash_cell_size = cell_size

            # Warn if cell_size is provided but mode is not CONSTANT
            current_mode = self.params.spatial_hash_cell_size_mode
            if current_mode != SpatialHashCellSizeMode.CONSTANT:
                logger.warning(
                    f"cell_size parameter ({cell_size}) will be ignored in {current_mode.value} mode. "
                    "To use a fixed cell_size, set mode=SpatialHashCellSizeMode.CONSTANT"
                )
            else:
                logger.debug(f"Updated cell_size: {old_size} -> {cell_size}")

        mode = self.params.spatial_hash_cell_size_mode
        old_cell_size = self._cached_cell_size

        # Calculate cell_size based on mode
        if mode == SpatialHashCellSizeMode.CONSTANT:
            # CONSTANT mode: use user-provided value
            if self.params.spatial_hash_cell_size is None:
                logger.error(
                    "spatial_hash_cell_size_mode=CONSTANT requires spatial_hash_cell_size to be set. "
                    "Falling back to default 1.0m"
                )
                cell_size = 1.0
            else:
                cell_size = self.params.spatial_hash_cell_size
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

        Example:
            # Manual addition (not recommended - SimObject does this automatically)
            obj = SimObject(body_id=body_id, sim_core=None)
            sim_core.add_object(obj)

            # Automatic addition (recommended)
            obj = SimObject(body_id=body_id, sim_core=sim_core)  # Calls add_object() internally
        """
        # Add to sim_objects list and dict
        if obj in self.sim_objects:
            logger.warning(f"Object {obj.object_id} already added to simulation")
            return

        self.sim_objects.append(obj)
        self._sim_objects_dict[obj.object_id] = obj

        # Add to agents list if this is an Agent instance (for O(M) iteration)
        if isinstance(obj, Agent):
            self.agents.append(obj)

        # Store collision mode in cache
        self._cached_collision_modes[obj.object_id] = obj.collision_mode

        # Register movement type based on collision mode
        self._register_object_movement_type(obj)

        # Add to collision system based on collision mode
        if obj.collision_mode == CollisionMode.DISABLED:
            # DISABLED: No collision detection at all
            # Add to disabled set, no AABB, no spatial grid, no tracking
            self._disabled_collision_objects.add(obj.object_id)

        elif obj.collision_mode == CollisionMode.STATIC:
            # STATIC: Add AABB/grid once, never update
            # Static objects are excluded from _cached_non_static_dict
            self._add_object_to_collision_system(obj.object_id)

        else:
            # NORMAL_3D or NORMAL_2D: Add to non-static dict and collision system
            self._cached_non_static_dict[obj.object_id] = (obj, obj.collision_mode)
            self._add_object_to_collision_system(obj.object_id)

        # Trigger cell_size recalculation in auto_adaptive mode
        if self.params.spatial_hash_cell_size_mode == SpatialHashCellSizeMode.AUTO_ADAPTIVE:
            self.set_collision_spatial_hash_cell_size_mode()

        # Save original color once when object is added (for collision visualization)
        if obj.body_id not in self._robot_original_colors:
            try:
                # Try to get current visual color, fallback to default if not available
                visual_data = p.getVisualShapeData(obj.body_id)
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

        Example:
            obj = sim_core.spawn_robot(...)
            # ... later ...
            sim_core.remove_object(obj)
        """
        obj_id = obj.object_id

        # Remove from sim_objects list and dict
        try:
            self.sim_objects.remove(obj)
        except ValueError:
            logger.warning(f"Object {obj_id} not found in sim_objects")
            return

        self._sim_objects_dict.pop(obj_id, None)

        # Remove from agents list if this is an Agent instance
        if isinstance(obj, Agent):
            try:
                self.agents.remove(obj)
            except ValueError:
                pass  # Already removed or not in list

        # Remove from movement tracking sets
        self._moved_this_step.discard(obj_id)
        self._physics_objects.discard(obj_id)
        self._kinematic_objects.discard(obj_id)
        self._static_objects.discard(obj_id)
        self._disabled_collision_objects.discard(obj_id)

        # Remove from collision mode cache
        self._cached_collision_modes.pop(obj_id, None)

        # Remove from collision system (AABB, spatial grid)
        self._remove_object_from_collision_system(obj_id)

        # Remove from non-static dict (O(1) dict removal)
        self._cached_non_static_dict.pop(obj_id, None)

        # Remove from active collision pairs
        self._active_collision_pairs = {(i, j) for i, j in self._active_collision_pairs if i != obj_id and j != obj_id}

        # Remove PyBullet body
        if obj.body_id is not None:
            try:
                p.removeBody(obj.body_id)
            except p.error:
                logger.warning(f"Failed to remove PyBullet body {obj.body_id}")

        # Trigger cell_size recalculation in auto_adaptive mode
        if self.params.spatial_hash_cell_size_mode == SpatialHashCellSizeMode.AUTO_ADAPTIVE:
            self.set_collision_spatial_hash_cell_size_mode()

        logger.info(f"Removed object {obj_id} (body {obj.body_id}) from simulation")

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
        if len(self._static_objects) > 100:
            print(
                "             ⚠️  Warning: %d static objects detected - " "toggling may be slow (not recommended)",
                len(self._static_objects),
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
            num_structures = len(self._static_objects)
            if num_structures > 100:
                print(f"\n[TOGGLE] ⚠️  Warning: Toggling transparency for {num_structures} static objects...")
                print("         This may take several seconds (not recommended for large scenes)")
                print("         Consider setting 'enable_structure_transparency' in config.yaml instead")

            self._structure_transparent = not self._structure_transparent
            # self._set_transparency(self._structure_transparent)
            self._set_structure_transparency(self._structure_transparent)
            print(f"\n[TOGGLE] Structure transparency: {'ON' if self._structure_transparent else 'OFF'}")

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
        Set transparency of static bodies (structures).
        Uses pre-saved colors with modified alpha channel.
        FAST implementation using pre-saved color dictionary.

        Args:
            transparent: True to make static objects semi-transparent, False for opaque
        """
        if not self.params.gui:
            return

        alpha = 0.3 if transparent else 1.0

        print(f"[TRANSPARENCY] Applying alpha={alpha} to {len(self._static_objects)} static objects...")

        # Apply alpha only to static bodies using pre-saved colors (FAST)
        # Convert object_ids to body_ids for visual shape manipulation
        static_body_ids = {
            self._sim_objects_dict[obj_id].body_id for obj_id in self._static_objects if obj_id in self._sim_objects_dict
        }

        processed = 0
        for key, rgba in self._original_visual_colors.items():
            body_id, link_index = key

            # Only process static bodies (O(1) lookup with set)
            if body_id not in static_body_ids:
                continue

            try:
                # Apply new alpha to the original color
                p.changeVisualShape(body_id, link_index, rgbaColor=[rgba[0], rgba[1], rgba[2], alpha])
                processed += 1
            except Exception:
                pass

        print(f"[TRANSPARENCY] Complete: {processed} visual shapes updated")
        logger.info(f"Static objects transparency {'enabled (alpha=0.3)' if transparent else 'disabled (alpha=1.0)'}")

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

        # Use self.params.camera_config if camera_config is not provided
        if camera_config is None:
            camera_config = self.params.camera_config

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
            # If entity_positions is None, calculate from all simulation objects
            if entity_positions is None or len(entity_positions) == 0:
                if len(self.sim_objects) == 0:
                    logger.warning("Auto camera mode requested but no objects in simulation")
                    return
                # Calculate positions from all simulation objects
                entity_positions = []
                for obj in self.sim_objects:
                    pose = obj.get_pose()
                    entity_positions.append(pose.position)
                logger.info(f"Auto camera: calculated from {len(entity_positions)} objects")

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
        """Get AABBs for all simulation objects."""
        return [p.getAABB(obj.body_id) for obj in self.sim_objects]

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

        Example:
            # Without profiling
            pairs, _ = sim.filter_aabb_pairs()

            # With profiling
            pairs, timings = sim.filter_aabb_pairs(return_profiling=True)
            print(f"AABB filtering took {timings['total']:.2f}ms")
        """
        # Profiling timings (always create dict for consistent return type)
        timings: Dict[str, float] = {}

        if ignore_static is None:
            ignore_static = self.ignore_static_collision

        # 1. Handle mode changes
        # Check if ignore_static mode has changed (skip if this is the first call)
        if self._last_ignore_static is not None and self._last_ignore_static != ignore_static:
            # Mode changed: reset collision state and mark all objects for re-check
            # Note: No need to clear AABB/spatial grid caches - they are position-based, not filter-based
            # Mark all objects as moved to trigger full collision recalculation
            self._moved_this_step = set(self._sim_objects_dict.keys())
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
            logger.debug(f"Full AABB rebuild for {len(self.sim_objects)} objects")
            for obj in self.sim_objects:
                self._cached_aabbs_dict[obj.object_id] = p.getAABB(obj.body_id)
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
        processed_pairs = set()  # Track (obj_id_i, obj_id_j) to avoid duplicates

        # OPTIMIZATION: Only iterate through moved objects instead of all objects
        # This dramatically reduces computation when few objects move (e.g., 10 moved out of 1000 total)
        for obj_id_i in moved_objects:
            obj_i = self._sim_objects_dict.get(obj_id_i)
            if obj_i is None:
                continue

            # Skip static objects in ignore_static mode
            # Note: In most cases, moved_this_step won't contain static objects,
            # but they may be added during spawn or mode changes
            if ignore_static and obj_id_i in self._static_objects:
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

                    for obj_id_j in self._cached_spatial_grid.get(neighbor_cell, set()):
                        # Skip self-pairs
                        if obj_id_j == obj_id_i:
                            continue

                        # Skip if j is static in ignore_static mode
                        if ignore_static and obj_id_j in self._static_objects:
                            continue

                        # Skip disabled collision objects
                        if obj_id_j in self._disabled_collision_objects:
                            continue

                        # Create sorted pair to avoid duplicates (i < j)
                        pair_key = (min(obj_id_i, obj_id_j), max(obj_id_i, obj_id_j))

                        # Skip already processed pairs
                        if pair_key in processed_pairs:
                            continue
                        processed_pairs.add(pair_key)

                        obj_j = self._sim_objects_dict.get(obj_id_j)
                        if obj_j is None:
                            continue

                        mode_j = self._cached_collision_modes.get(obj_id_j, CollisionMode.NORMAL_3D)

                        # Skip this pair if object j uses 2D mode and offset has Z component
                        if mode_j == CollisionMode.NORMAL_2D and offset[2] != 0:
                            continue

                        # AABB overlap test
                        aabb_j = self._cached_aabbs_dict.get(obj_id_j)
                        if aabb_j is None:
                            continue

                        # Check if AABBs overlap (continue if NO overlap)
                        # For 2D modes, check XY only; for 3D modes, check XYZ
                        aabb_overlap_xy = not (
                            aabb_i[1][0] < aabb_j[0][0]
                            or aabb_i[0][0] > aabb_j[1][0]
                            or aabb_i[1][1] < aabb_j[0][1]
                            or aabb_i[0][1] > aabb_j[1][1]
                        )
                        
                        # Check Z-axis overlap only if at least one object uses 3D mode
                        if mode_i == CollisionMode.NORMAL_3D or mode_j == CollisionMode.NORMAL_3D:
                            aabb_overlap_z = not (
                                aabb_i[1][2] < aabb_j[0][2]
                                or aabb_i[0][2] > aabb_j[1][2]
                            )
                            # 3D mode: require both XY and Z overlap
                            if not (aabb_overlap_xy and aabb_overlap_z):
                                continue  # No overlap, skip this pair
                        else:
                            # Both use 2D mode: only check XY overlap
                            if not aabb_overlap_xy:
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
            - timings: dict with get_aabbs, spatial_hashing, aabb_filtering, contact_points, total (in ms) if return_profiling=True, else None

        Example:
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
            collision_color = self.collision_color
        collision_pairs = []
        collided = set()

        # Determine ignore_static flag
        if ignore_static is None:
            ignore_static = self.ignore_static_collision

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
                if obj_id_i in self._static_objects and obj_id_j in self._static_objects:
                    continue

                # Re-check if at least one object moved
                if obj_id_i in moved_since_last_check or obj_id_j in moved_since_last_check:
                    pairs_to_check.add((obj_id_i, obj_id_j))

            # Check contact points for all pairs
            new_collisions = set()
            resolved_collisions = set()

            for obj_id_i, obj_id_j in pairs_to_check:
                obj_i = self._sim_objects_dict.get(obj_id_i)
                obj_j = self._sim_objects_dict.get(obj_id_j)
                if obj_i is None or obj_j is None:
                    continue

                # Select collision detection method based on configuration
                has_collision = False

                if self.params.collision_detection_method == CollisionDetectionMethod.CONTACT_POINTS:
                    # Method 1: getContactPoints (physics mode, actual contact manifold)
                    # Best for: physics simulation, requires stepSimulation()
                    # Note: May be unstable for kinematic-kinematic pairs
                    contact_points = p.getContactPoints(obj_i.body_id, obj_j.body_id, physicsClientId=self.client)
                    has_collision = len(contact_points) > 0

                elif self.params.collision_detection_method == CollisionDetectionMethod.CLOSEST_POINTS:
                    # Method 2: getClosestPoints (kinematics mode, distance-based)
                    # Best for: kinematics motion, safety margin detection
                    # Works with: Physics ON/OFF, stable with resetBasePositionAndOrientation
                    closest_points = p.getClosestPoints(
                        obj_i.body_id, obj_j.body_id, distance=self.params.collision_margin,  # Safety clearance
                        physicsClientId=self.client
                    )
                    has_collision = len(closest_points) > 0

                elif self.params.collision_detection_method == CollisionDetectionMethod.HYBRID:
                    # Method 3: Hybrid (physics uses getContactPoints, kinematic uses getClosestPoints)
                    # Best for: Mixed physics/kinematics with different detection needs
                    is_physics_i = obj_id_i in self._physics_objects
                    is_physics_j = obj_id_j in self._physics_objects

                    if is_physics_i or is_physics_j:
                        # At least one is physics object - use getContactPoints
                        contact_points = p.getContactPoints(obj_i.body_id, obj_j.body_id, physicsClientId=self.client)
                        has_collision = len(contact_points) > 0
                    else:
                        # Both are kinematic - use getClosestPoints with safety margin
                        closest_points = p.getClosestPoints(
                            obj_i.body_id, obj_j.body_id, distance=self.params.collision_margin,
                            physicsClientId=self.client
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
                            f"NEW COLLISION: object {obj_id_i} (body {obj_i.body_id}) <-> object {obj_id_j} (body {obj_j.body_id})"
                        )
                else:
                    # No collision
                    if pair in self._active_collision_pairs:
                        # Collision resolved
                        resolved_collisions.add(pair)
                        self._active_collision_pairs.discard(pair)
                        logger.info(
                            f"COLLISION RESOLVED: object {obj_id_i} (body {obj_i.body_id}) <-> object {obj_id_j} (body {obj_j.body_id})"
                        )

            if return_profiling:
                timings["contact_points"] = (time.perf_counter() - t_contact0) * 1000  # ms

            # Update collision count (track total new collisions)
            self.collision_count += len(new_collisions)

            # Color update (collision: blue, normal: original) only if enabled
            if self.params.enable_collision_color_change:
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
                            p.changeVisualShape(obj.body_id, -1, rgbaColor=collision_color)

                # Update colors for resolved collisions (restore original color)
                # Only restore if object is not involved in any other collision
                for obj_id_i, obj_id_j in resolved_collisions:
                    for obj_id in (obj_id_i, obj_id_j):
                        if obj_id not in currently_colliding:
                            obj = self._sim_objects_dict.get(obj_id)
                            if obj is not None:
                                orig_color = self._robot_original_colors.get(obj.body_id, [0, 0, 0, 1])
                                p.changeVisualShape(obj.body_id, -1, rgbaColor=orig_color)

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

        Example:
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
        
        # Use len(self.agents) for O(1) agent count instead of O(N) iteration
        num_agents = len(self.agents)
        num_objects = len(self.sim_objects) - num_agents
        
        monitor_data = {
            "sim_time": sim_time,
            "real_time": elapsed_time,
            "target_speed": self.params.speed,
            "actual_speed": actual_speed,
            "time_step": self.params.timestep,
            "frequency": 1 / self.params.timestep,
            "physics": "enabled" if self.params.physics else "disabled",
            "agents": num_agents,
            "objects": num_objects,
            "active_collisions": len(self._active_collision_pairs),
            "collisions": self.collision_count,
            "steps": self.step_count,
        }
        logging.debug(
            "[MONITOR] sim_time=%.2f, real_time=%.2f, speed=%.2f, agents=%d, objects=%d, active_collisions=%d, total_collisions=%d, steps=%d",
            sim_time,
            elapsed_time,
            actual_speed,
            num_agents,
            num_objects,
            len(self._active_collision_pairs),
            self.collision_count,
            self.step_count,
        )
        # Log only when collision count increases (new collision detected)
        if self.collision_count > self._last_logged_collision_count:
            logging.info(f"NEW COLLISION: total={self.collision_count} at sim_time={sim_time:.2f}")
            self._last_logged_collision_count = self.collision_count
        # Also output to DataMonitor window
        if self.data_monitor:
            self.data_monitor.write_data(monitor_data)

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

        Example:
            # Automatic (recommended)
            sim.run_simulation(duration=10.0)

            # Manual initialization for custom loop
            sim.initialize_simulation()
            while custom_condition:
                sim.step_once()
        """
        # Reset simulation state
        self.start_time = time.time()
        self.step_count = 0
        self.collision_count = 0
        self.sim_time = 0.0
        self.last_collision_check = 0.0
        self.last_monitor_update = 0.0
        self._last_logged_collision_count = 0

        # Clear movement tracking from any previous simulation
        self._moved_this_step.clear()

        # Clear active collision pairs (fresh start)
        self._active_collision_pairs.clear()

        # Configure visualizer after all objects are spawned
        # This ensures transparency and other visual settings are applied correctly
        self.configure_visualizer()

        # Enable rendering before starting simulation
        self.enable_rendering()

        logger.info(f"Simulation initialized: {len(self.sim_objects)} objects, timestep={self.params.timestep}s")

    def run_simulation(self, duration: Optional[float] = None) -> None:
        """
        Run the simulation for a specified duration.

        Args:
            duration: Simulation duration in seconds (simulation time, not real time).
                     If None, uses self.params.duration. If duration <= 0, runs indefinitely.

        Example:
            # Run for 10 seconds (simulation time)
            sim.run_simulation(duration=10.0)

            # Run indefinitely (until Ctrl+C or GUI closed)
            sim.run_simulation(duration=0)
        """
        if duration is None:
            duration = self.params.duration

        # Initialize simulation state (counters, visualizer, rendering)
        self.initialize_simulation()

        try:
            start_time = time.time()
            last_step_process_time = 0.0  # Track processing time excluding sleep
            last_pause_state = False  # Track pause state to detect resume

            while True:
                current_sim_time = self.step_count * self.params.timestep

                # Check duration based on simulation time (not real time)
                if duration > 0 and current_sim_time >= duration:
                    break

                # Check if PyBullet connection is still active (e.g., GUI window not closed)
                try:
                    p.getConnectionInfo()
                except p.error:
                    logging.info("PyBullet connection lost (GUI window closed)")
                    break

                # Speed=0: Run as fast as possible (no synchronization, no sleep)
                if self.params.speed <= 0:
                    self.step_once()
                else:
                    # Speed>0: Synchronize with real time using absolute time calculation
                    loop_start = time.time()
                    current_time = time.time()
                    
                    # Detect pause state change: reset start_time on resume
                    if last_pause_state and not self._simulation_paused:
                        # Just resumed from pause: reset start_time to avoid jump
                        start_time = current_time - current_sim_time / self.params.speed
                        logger.info("Resumed from pause: start_time reset for smooth continuation")
                    last_pause_state = self._simulation_paused
                    
                    # Calculate target sim_time using absolute time (stable control)
                    elapsed_time = current_time - start_time
                    target_sim_time = elapsed_time * self.params.speed
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

    def step_once(self, return_profiling: bool = False) -> Optional[Dict[str, float]]:
        """
        Execute one simulation step.

        Performs agent updates, callbacks, physics step, collision detection, and monitoring.
        If enable_profiling is True, detailed timing information is logged.

        Args:
            return_profiling: If True, return timing breakdown dictionary instead of logging.
                             Useful for external profiling tools.

        Returns:
            If return_profiling=True, returns dict with timing breakdown in milliseconds:
                {
                    'agent_update': float,
                    'callbacks': float,
                    'step_simulation': float,
                    'collision_check': float,
                    'monitor_update': float,
                    'total': float,
                }
            Otherwise returns None.

        Performance note: time.perf_counter() calls have negligible overhead (<0.1% for 10k objects).
        The profiling measurements themselves do not significantly impact simulation performance.
        """
        # Profiling: step start time (measure even if return_profiling=True)
        measure_timing = self.enable_profiling or return_profiling
        if measure_timing:
            t_step = time.perf_counter()

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

        # Physics objects are always considered as potentially moved (conservative approach)
        # This avoids expensive pose comparison every step
        # Note: _moved_this_step is cleared in check_collisions() to accumulate movements
        # across multiple steps when collision_check_frequency < step_frequency
        self._moved_this_step.update(self._physics_objects)

        # Update AABBs and spatial grid for physics objects immediately after physics step (batch operation)
        # Kinematic objects update their AABBs and spatial grid in set_pose() for immediate consistency
        for obj_id in self._physics_objects:
            # O(1) dict lookup instead of O(N) linear search
            obj = self._sim_objects_dict.get(obj_id)
            if obj is not None:
                self._update_object_aabb(obj_id)
                # Update spatial grid if cell_size is initialized
                if self._cached_cell_size is not None:
                    self._update_object_spatial_grid(obj_id)

        self.sim_time = self.step_count * self.params.timestep

        # Update all simulation objects that have update() method
        # Agent instances are automatically updated every step for movement control
        # OPTIMIZATION: Use self.agents list for O(M) iteration instead of O(N) with isinstance() check
        if measure_timing:
            t0 = time.perf_counter()

        for agent in self.agents:
            moved = agent.update(self.params.timestep)
            # Track kinematic agent movement
            if moved and agent.object_id in self._kinematic_objects:
                self._moved_this_step.add(agent.object_id)

        if measure_timing:
            t1 = time.perf_counter()

        # Global callbacks (frequency control)
        if measure_timing:
            t_cb0 = time.perf_counter()

        for cbinfo in self.callbacks:
            freq = cbinfo.get("frequency", None)
            last_exec = cbinfo.get("last_exec", 0.0)
            interval = 1.0 / freq if freq else 0.0
            # Judge based on self.sim_time
            if freq is None or self.sim_time - last_exec >= interval:
                dt = self.sim_time - last_exec if last_exec > 0 else self.params.timestep
                cbinfo["func"](self, dt)
                cbinfo["last_exec"] = self.sim_time

        if measure_timing:
            t_cb1 = time.perf_counter()

        # Check if PyBullet is still connected before stepping
        if not p.isConnected():
            raise RuntimeError("PyBullet disconnected (GUI window closed)")

        if measure_timing:
            t_sim0 = time.perf_counter()

        # stepSimulation() control based on physics mode
        # Physics ON: Call stepSimulation() every step (rigid body integration, contact resolution)
        # Physics OFF: Skip stepSimulation() for pure kinematics (position updates via reset API)
        if self.params.physics:
            p.stepSimulation()
        # Note: Even in Physics OFF mode, collision detection still works via getClosestPoints()
        # which queries geometry directly without requiring stepSimulation()

        if measure_timing:
            t_sim1 = time.perf_counter()

        # Collision check frequency control
        if measure_timing:
            t_col0 = time.perf_counter()

        collision_timings: Dict[str, float] = {}
        freq = self.collision_check_frequency
        # freq = None: check every step
        # freq = 0: disabled (never check)
        # freq > 0: check at specified frequency (Hz)
        if freq is None:
            # Check every step
            _, collision_timings = self.check_collisions(return_profiling=self.enable_profiling)
            self.last_collision_check = self.sim_time
        elif freq > 0:
            # Check at specified frequency
            interval = 1.0 / freq
            if self.sim_time - self.last_collision_check >= interval:
                _, collision_timings = self.check_collisions(return_profiling=self.enable_profiling)
                self.last_collision_check = self.sim_time
        # else: freq = 0, skip collision checks entirely

        if measure_timing:
            t_col1 = time.perf_counter()

        self.step_count += 1
        # Monitor: every step if GUI enabled, otherwise every second
        if measure_timing:
            t_mon0 = time.perf_counter()

        if self.monitor_enabled:
            interval = self.params.timestep if self.params.gui else 1.0
            if self.sim_time - self.last_monitor_update > interval:
                self.update_monitor()
                self.last_monitor_update = self.sim_time

        if measure_timing:
            t_mon1 = time.perf_counter()
            t_end = time.perf_counter()

        # Return profiling data if requested
        if return_profiling:
            return {
                "agent_update": 1000 * (t1 - t0),  # type: ignore[possibly-unbound]
                "callbacks": 1000 * (t_cb1 - t_cb0),  # type: ignore[possibly-unbound]
                "step_simulation": 1000 * (t_sim1 - t_sim0),  # type: ignore[possibly-unbound]
                "collision_check": 1000 * (t_col1 - t_col0),  # type: ignore[possibly-unbound]
                "monitor_update": 1000 * (t_mon1 - t_mon0),  # type: ignore[possibly-unbound]
                "total": 1000 * (t_end - t_step),  # type: ignore[possibly-unbound]
            }

        # Profiling output (only when profiling is enabled and not returning data)
        if self.enable_profiling:
            # Collect statistics for averaging
            self._profiling_stats['agent_update'].append(1000*(t1-t0))  # type: ignore[possibly-unbound]
            self._profiling_stats['callbacks'].append(1000*(t_cb1-t_cb0))  # type: ignore[possibly-unbound]
            self._profiling_stats['step_simulation'].append(1000*(t_sim1-t_sim0))  # type: ignore[possibly-unbound]
            self._profiling_stats['collision_check'].append(1000*(t_col1-t_col0))  # type: ignore[possibly-unbound]
            self._profiling_stats['monitor_update'].append(1000*(t_mon1-t_mon0))  # type: ignore[possibly-unbound]
            self._profiling_stats['total'].append(1000*(t_end-t_step))  # type: ignore[possibly-unbound]
            
            # Print average statistics every N steps
            if self.step_count % self._profiling_interval == 0 and len(self._profiling_stats['total']) > 0:
                self._print_profiling_summary()

    def _print_profiling_summary(self) -> None:
        """Print profiling statistics summary (average over last N steps)."""
        import statistics
        
        if not self._profiling_stats['total']:
            return
        
        # Calculate statistics for each component
        components = ['agent_update', 'callbacks', 'step_simulation', 'collision_check', 'monitor_update', 'total']
        stats_info = []
        
        total_avg = statistics.mean(self._profiling_stats['total'])
        num_samples = len(self._profiling_stats['total'])
        
        for comp in components:
            data = self._profiling_stats[comp]
            if not data:
                continue
            
            avg = statistics.mean(data)
            
            # Calculate percentage of total
            percentage = (avg / total_avg * 100) if total_avg > 0 else 0
            
            stats_info.append(f"{comp}={avg:.2f}ms ({percentage:.1f}%)")
        
        logger.info(
            f"[PROFILING] Last {num_samples} steps average: " +
            ", ".join(stats_info)
        )
        
        # Clear statistics for next interval
        for comp in components:
            self._profiling_stats[comp].clear()

