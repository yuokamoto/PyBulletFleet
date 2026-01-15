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
            ignore_structure_collision=config.get("ignore_structure_collision", True),
            enable_profiling=config.get("enable_profiling", False),
            enable_collision_color_change=config.get("enable_collision_color_change", False),
            collision_check_2d=config.get("collision_check_2d", False),  # Default: 3D collision (27 neighbors)
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
        ignore_structure_collision: bool = True,
        enable_profiling: bool = False,
        enable_collision_color_change: bool = False,
        collision_check_2d: bool = False,  # Default False: full 3D collision (27 neighbors)
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
        self.ignore_structure_collision = ignore_structure_collision
        self.enable_profiling = enable_profiling
        self.enable_collision_color_change = enable_collision_color_change
        self.collision_check_2d = collision_check_2d  # Store 2D/3D collision check mode


class MultiRobotSimulationCore:
    # Precomputed neighbor offsets for spatial hashing (3D: 27 neighbors, constant)
    _NEIGHBOR_OFFSETS_3D = tuple(
        (dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
    )
    
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
        )  # Store original colors: (body_id, link_id) -> rgba
        self._structure_body_ids: set = set()  # Set of structure body IDs (not robots) - use set for O(1) lookup
        self._structure_transparent: bool = False  # Track if structure is transparent
        self.ignore_structure_collision: bool = (
            params.ignore_structure_collision
        )  # If True, skip collision checks with structure objects
        self._simulation_paused: bool = False  # Track if simulation is paused
        self.enable_profiling: bool = params.enable_profiling  # Optional profiling output
        self._profiling_log_frequency: int = 10  # Log profiling info every N steps (default: 10)
        
        # --- Movement tracking and collision optimization ---
        self._moved_this_step: set = set()  # object_ids that moved this step
        self._physics_objects: set = set()  # object_ids with physics enabled (can move via collisions)
        self._kinematic_objects: set = set()  # object_ids that move only via set_pose/update
        self._static_objects: set = set()  # object_ids that never move (structures)
        
        # --- Collision detection cache ---
        self._cached_collision_modes: Dict[int, bool] = {}  # object_id -> use_2d (True=2D, False=3D)
        self._cached_cell_size: Optional[float] = None  # Cached cell size for spatial hashing
        self._cached_non_structure_objects: List[SimObject] = []  # Cached non-structure objects (robots) for collision detection
        self._cached_non_structure_modes: List[bool] = []  # Cached collision modes for non-structure objects (parallel to _cached_non_structure_objects)
        self._cached_all_objects_modes: List[bool] = []  # Cached collision modes for all objects (parallel to sim_objects)
        self._non_structure_cache_valid: bool = False  # Flag indicating if non-structure object cache is valid
        
        # Incremental AABB and spatial grid cache
        self._cached_aabbs_dict: Dict[int, Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = {}  # object_id -> AABB
        self._cached_spatial_grid: Dict[Tuple[int, int, int], Set[int]] = {}  # Spatial hash grid: cell -> {object_ids}
        self._cached_object_to_cell: Dict[int, Tuple[int, int, int]] = {}  # object_id -> cell mapping
        self._aabb_cache_valid: bool = False  # Flag indicating if AABB cache is valid
        self._last_ignore_structure: Optional[bool] = None  # Track last ignore_structure value to detect mode changes
        
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
        Set the frequency (Hz, number of times per second) for collision detection. If None, check every step.
        Default value is 1Hz (every second).
        """
        if frequency is None:
            frequency = 1
        self.collision_check_frequency = frequency

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

    def register_structure_body(self, body_id: int) -> None:
        """
        Register a body ID as part of the structure (not a robot).
        Structure bodies can be made transparent with 't' key.

        Args:
            body_id (int): PyBullet body ID to register as structure
        """
        self._structure_body_ids.add(body_id)
        # Invalidate cache to force rebuild (simpler than selective removal)
        self._cached_non_structure_objects = []
        self._cached_non_structure_modes = []
        self._non_structure_cache_valid = False  # Invalidate non-structure cache
        self._aabb_cache_valid = False  # Force AABB cache rebuild
        # Note: _cached_all_objects_modes doesn't need clearing (structure registration doesn't change sim_objects order)

    def _mark_object_moved(self, object_id: int) -> None:
        """
        Mark an object as moved in the current step.
        Called by SimObject.set_pose() when movement is detected.
        
        Args:
            object_id: The object_id that moved
        """
        self._moved_this_step.add(object_id)
        logger.debug(f"Object {object_id} marked as moved")
    
    def _update_object_aabb(self, object_id: int, body_id: int) -> None:
        """
        Update AABB for a single object immediately.
        Called when kinematic objects move or physics objects are updated.
        
        Args:
            object_id: The object_id to update
            body_id: PyBullet body_id for AABB query
        """
        try:
            self._cached_aabbs_dict[object_id] = p.getAABB(body_id)
            logger.debug(f"Updated AABB for object {object_id}")
        except p.error:
            logger.warning(f"Failed to update AABB for object {object_id} (body {body_id})")
    
    def _add_object_to_spatial_grid(self, object_id: int) -> None:
        """
        Add an object to the spatial grid based on its current AABB.
        Called when objects are added or moved.
        
        Args:
            object_id: The object_id to add to spatial grid
        """
        if object_id not in self._cached_aabbs_dict:
            logger.warning(f"Cannot add object {object_id} to spatial grid: AABB not found")
            return
        
        # Ensure cell size is calculated
        if self._cached_cell_size is None:
            logger.warning(f"Cannot add object {object_id} to spatial grid: cell_size not initialized")
            return
        
        aabb = self._cached_aabbs_dict[object_id]
        center = [0.5 * (aabb[0][d] + aabb[1][d]) for d in range(3)]
        cell = tuple(int(center[d] // self._cached_cell_size) for d in range(3))
        
        self._cached_object_to_cell[object_id] = cell
        self._cached_spatial_grid.setdefault(cell, set()).add(object_id)
        logger.debug(f"Added object {object_id} to spatial grid cell {cell}")
    
    def _update_object_spatial_grid(self, object_id: int) -> None:
        """
        Update an object's position in the spatial grid after movement.
        Removes from old cell and adds to new cell.
        
        Args:
            object_id: The object_id to update in spatial grid
        """
        if object_id not in self._cached_aabbs_dict:
            return
        
        if self._cached_cell_size is None:
            return
        
        # Remove from old cell
        old_cell = self._cached_object_to_cell.get(object_id)
        if old_cell is not None and old_cell in self._cached_spatial_grid:
            self._cached_spatial_grid[old_cell].discard(object_id)
            if not self._cached_spatial_grid[old_cell]:
                del self._cached_spatial_grid[old_cell]
        
        # Add to new cell
        aabb = self._cached_aabbs_dict[object_id]
        center = [0.5 * (aabb[0][d] + aabb[1][d]) for d in range(3)]
        cell = tuple(int(center[d] // self._cached_cell_size) for d in range(3))
        
        self._cached_object_to_cell[object_id] = cell
        self._cached_spatial_grid.setdefault(cell, set()).add(object_id)
        logger.debug(f"Updated object {object_id} spatial grid: {old_cell} -> {cell}")

    def _register_object_movement_type(self, object_id: int, is_physics: bool = False, is_static: bool = False) -> None:
        """
        Register an object's movement type for optimization.
        
        Args:
            object_id: The object_id to register
            is_physics: True if object has physics enabled (can move via collisions)
            is_static: True if object never moves (e.g., structures)
        """
        if is_static:
            self._static_objects.add(object_id)
            logger.debug(f"Object {object_id} registered as static")
        elif is_physics:
            self._physics_objects.add(object_id)
            logger.debug(f"Object {object_id} registered as physics-enabled")
        else:
            self._kinematic_objects.add(object_id)
            logger.debug(f"Object {object_id} registered as kinematic")
    
    def _update_collision_mode_cache(self, object_id: int, obj: SimObject) -> None:
        """
        Update cached collision mode for an object.
        Called when object is added or collision_check_2d is changed.
        
        Args:
            object_id: The object_id to update
            obj: The SimObject instance
        """
        mode = obj.collision_check_2d if obj.collision_check_2d is not None else self.params.collision_check_2d
        self._cached_collision_modes[object_id] = mode
        
        # Also update list caches (parallel to sim_objects and _cached_non_structure_objects)
        self._cached_all_objects_modes.append(mode)
        
        # Add to non-structure cache if this object is not a structure
        if obj.body_id not in self._structure_body_ids:
            self._cached_non_structure_objects.append(obj)
            self._cached_non_structure_modes.append(mode)
        
        logger.debug(f"Collision mode for object {object_id}: {'2D' if mode else '3D'}")

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
        
        # Update collision mode cache (also handles list cache updates)
        self._update_collision_mode_cache(obj.object_id, obj)
        
        # Register movement type based on object properties
        is_physics = (obj.mass > 0 and not obj.is_kinematic)
        self._register_object_movement_type(
            obj.object_id,
            is_physics=is_physics,
            is_static=False  # Default: not static (override with register_structure_body)
        )
        
        # Update AABB immediately for newly added object
        # This eliminates the need for safety fallback in filter_aabb_pairs()
        self._update_object_aabb(obj.object_id, obj.body_id)
        
        # Add to spatial grid if cell_size is initialized
        # Note: cell_size is calculated on first filter_aabb_pairs() call
        if self._cached_cell_size is not None:
            self._add_object_to_spatial_grid(obj.object_id)
        
        logger.debug(f"Added object {obj.object_id} (body {obj.body_id}) to simulation [physics={is_physics}]")

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
        
        # Remove from movement tracking sets
        self._moved_this_step.discard(obj_id)
        self._physics_objects.discard(obj_id)
        self._kinematic_objects.discard(obj_id)
        self._static_objects.discard(obj_id)
        
        # Remove from collision mode cache
        self._cached_collision_modes.pop(obj_id, None)
        
        # Remove from AABB cache
        self._cached_aabbs_dict.pop(obj_id, None)
        
        # Remove from spatial grid
        if obj_id in self._cached_object_to_cell:
            old_cell = self._cached_object_to_cell[obj_id]
            if old_cell in self._cached_spatial_grid:
                self._cached_spatial_grid[old_cell].discard(obj_id)
                # Clean up empty cells
                if not self._cached_spatial_grid[old_cell]:
                    del self._cached_spatial_grid[old_cell]
            del self._cached_object_to_cell[obj_id]
        
        # Invalidate caches that use object indices (simpler than rebuilding)
        self._non_structure_cache_valid = False
        self._cached_non_structure_objects = []
        self._cached_non_structure_modes = []
        # Note: We keep _aabb_cache_valid=True because dict-based cache handles removal
        
        # Remove PyBullet body
        if obj.body_id is not None:
            try:
                p.removeBody(obj.body_id)
            except p.error:
                logger.warning(f"Failed to remove PyBullet body {obj.body_id}")
        
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
        """Get AABBs for all simulation objects."""
        return [p.getAABB(obj.body_id) for obj in self.sim_objects]

    def filter_aabb_pairs(
        self, ignore_structure: Optional[bool] = None, return_profiling: bool = False
    ) -> Union[Tuple[List[Tuple[int, int]], List[SimObject]], Tuple[List[Tuple[int, int]], List[SimObject], Dict[str, float]]]:
        """
        Filter AABB pairs for collision detection with incremental updates.
        Only recomputes AABBs and spatial grid for moved objects.
        
        Args:
            ignore_structure: If True, ignore structure collisions
            return_profiling: If True, return (pairs, objects, timings) tuple
        
        Returns:
            (pairs, active_objects) or (pairs, active_objects, timings) if return_profiling=True
            - pairs: List of (i, j) index pairs (indices into active_objects)
            - active_objects: List of SimObject instances to check
            - timings: dict with get_aabbs, spatial_hashing, aabb_filtering (in ms)
        """
        # Profiling timings (always create dict for consistent return type)
        timings: Dict[str, float] = {}
        
        # If ignoring structure collisions, only consider non-structure bodies (robots) for AABB pairing
        if ignore_structure is None:
            ignore_structure = self.ignore_structure_collision

        # 1. Get active objects and collision modes
        # Check if ignore_structure mode has changed
        if self._last_ignore_structure != ignore_structure:
            # Mode changed: clear all caches
            self._cached_aabbs_dict.clear()
            self._cached_spatial_grid.clear()
            self._cached_object_to_cell.clear()
            self._aabb_cache_valid = False
            self._last_ignore_structure = ignore_structure
            logger.debug(f"ignore_structure mode changed to {ignore_structure}, clearing AABB caches")
        
        if ignore_structure:
            # Use cached non-structure objects if available
            if not self._non_structure_cache_valid:
                # Build cache: filter out structure bodies
                active_objects = [obj for obj in self.sim_objects if obj.body_id not in self._structure_body_ids]
                self._cached_non_structure_objects = active_objects
                # Also cache collision modes
                self._cached_non_structure_modes = [
                    self._cached_collision_modes[obj.object_id] for obj in active_objects
                ]
                self._non_structure_cache_valid = True  # Mark cache as valid
                self._aabb_cache_valid = False  # Force full AABB rebuild
                logger.debug(f"Built non-structure object cache: {len(active_objects)} objects")
            # Use cached lists (objects + modes)
            active_objects = self._cached_non_structure_objects
            collision_modes = self._cached_non_structure_modes
        else:
            # Include all objects (structures + robots)
            active_objects = self.sim_objects
            collision_modes = self._cached_all_objects_modes

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
            logger.debug(f"Full AABB rebuild for {len(active_objects)} objects")
            for obj in active_objects:
                self._cached_aabbs_dict[obj.object_id] = p.getAABB(obj.body_id)
            # Mark cache as valid after update
            self._aabb_cache_valid = True
        
        if return_profiling:
            timings["get_aabbs"] = (time.perf_counter() - t0) * 1000  # ms

        # 3. Cell size calculation and spatial grid initialization
        if return_profiling:
            t0 = time.perf_counter()
        
        # Calculate cell size if not cached (one-time initialization)
        if self._cached_cell_size is None:
            # Calculate cell size from median AABB extent (use ALL objects for global statistics)
            aabbs = list(self._cached_aabbs_dict.values())
            if aabbs:
                extents = []
                for aabb in aabbs:
                    extent_x = aabb[1][0] - aabb[0][0]
                    extent_y = aabb[1][1] - aabb[0][1]
                    extent_z = aabb[1][2] - aabb[0][2]
                    extents.append(max(extent_x, extent_y, extent_z))
                
                median_extent = sorted(extents)[len(extents) // 2]
                cell_size = max(median_extent * 2.0, 0.5)
                self._cached_cell_size = cell_size
                logger.debug(f"Calculated cell size: {cell_size:.3f}m (median extent: {median_extent:.3f}m)")
                
                # Initial spatial grid construction after cell_size is determined
                # Subsequent updates are handled by add_object(), set_pose(), and step_once()
                self._cached_spatial_grid.clear()
                self._cached_object_to_cell.clear()
                for obj in active_objects:
                    self._add_object_to_spatial_grid(obj.object_id)
                logger.debug(f"Initial spatial grid built with {len(active_objects)} objects")
            else:
                cell_size = 1.0
                self._cached_cell_size = cell_size
                logger.debug("No AABBs available, using default cell size: 1.0m")
        
        # Spatial grid is now maintained incrementally by:
        # - add_object(): adds new objects to grid
        # - remove_object(): removes objects from grid
        # - set_pose(): updates kinematic objects' grid cells
        # - step_once(): updates physics objects' grid cells
        # Initial construction is handled above when cell_size is first calculated
        
        if return_profiling:
            timings["spatial_hashing"] = (time.perf_counter() - t0) * 1000  # ms
        
        # 4. AABB overlap filtering
        if return_profiling:
            t0 = time.perf_counter()
        
        pairs = set()
        processed_pairs = set()  # Track (obj_id_i, obj_id_j) to avoid duplicates
        
        # Iterate through all cells in spatial grid
        for cell, object_ids in self._cached_spatial_grid.items():
            for obj_id_i in object_ids:
                obj_i = self._sim_objects_dict.get(obj_id_i)
                if obj_i is None:
                    continue
                
                id_i = obj_i.body_id
                use_2d_i = self._cached_collision_modes.get(obj_id_i, False)
                
                # Check neighbors in same and adjacent cells
                for offset in self._NEIGHBOR_OFFSETS_3D:
                    # Skip Z-axis neighbors if object i uses 2D mode
                    if use_2d_i and offset[2] != 0:
                        continue
                    
                    neighbor_cell = (cell[0] + offset[0], cell[1] + offset[1], cell[2] + offset[2])
                    for obj_id_j in self._cached_spatial_grid.get(neighbor_cell, set()):
                        # Skip duplicates and self-pairs
                        if obj_id_j <= obj_id_i:
                            continue
                        
                        # Skip already processed pairs
                        pair_key = (obj_id_i, obj_id_j)
                        if pair_key in processed_pairs:
                            continue
                        processed_pairs.add(pair_key)
                        
                        obj_j = self._sim_objects_dict.get(obj_id_j)
                        if obj_j is None:
                            continue
                        
                        id_j = obj_j.body_id
                        use_2d_j = self._cached_collision_modes.get(obj_id_j, False)
                        
                        # Skip this pair if object j uses 2D mode and offset has Z component
                        if use_2d_j and offset[2] != 0:
                            continue
                        
                        # Always skip structure-structure pairs
                        if id_i in self._structure_body_ids and id_j in self._structure_body_ids:
                            continue
                        
                        # AABB overlap test (using dict lookup)
                        aabb_i = self._cached_aabbs_dict.get(obj_id_i)
                        aabb_j = self._cached_aabbs_dict.get(obj_id_j)
                        if aabb_i is None or aabb_j is None:
                            continue
                        
                        if (
                            aabb_i[1][0] < aabb_j[0][0]
                            or aabb_i[0][0] > aabb_j[1][0]
                            or aabb_i[1][1] < aabb_j[0][1]
                            or aabb_i[0][1] > aabb_j[1][1]
                            or aabb_i[1][2] < aabb_j[0][2]
                            or aabb_i[0][2] > aabb_j[1][2]
                        ):
                            continue
                        # Store object_id pairs (not indices)
                        pairs.add((obj_id_i, obj_id_j))
        
        if return_profiling:
            timings["aabb_filtering"] = (time.perf_counter() - t0) * 1000  # ms
            return list(pairs), active_objects, timings
        
        return list(pairs), active_objects

    def check_collisions(
        self, collision_color: Optional[List[float]] = None, ignore_structure: Optional[bool] = None, 
        return_profiling: bool = False
    ) -> Union[List[Tuple[int, int]], Tuple[List[Tuple[int, int]], Dict[str, float]]]:
        """
        Check for collisions between robots.
        
        Args:
            collision_color: RGB color for collision visualization
            ignore_structure: If True, ignore structure collisions
            return_profiling: If True, return (pairs, timings) tuple
        
        Returns:
            List of collision pairs, or (pairs, timings) if return_profiling=True
            timings dict contains: get_aabbs, spatial_hashing, aabb_filtering, contact_points, total (in ms)
            If return_profiling=False, returns (pairs, {}) for consistent unpacking
        """
        t0 = time.perf_counter()
        
        # Profiling timings (always create dict for consistent return type)
        timings: Dict[str, float] = {}
        
        if collision_color is None:
            collision_color = self.collision_color
        collision_pairs = []
        collided = set()

        # Determine ignore_structure flag
        if ignore_structure is None:
            ignore_structure = self.ignore_structure_collision

        try:
            # Record: save initial color (now using obj.body_id directly)
            for obj in self.sim_objects:
                if obj.body_id not in self._robot_original_colors:
                    self._robot_original_colors[obj.body_id] = [0.0, 0.0, 0.0, 1]
            
            # Get candidate pairs and active objects
            if return_profiling:
                candidate_pairs, active_objects, filter_timings = self.filter_aabb_pairs(
                    ignore_structure=ignore_structure, return_profiling=True
                )
                timings.update(filter_timings)
            else:
                candidate_pairs, active_objects = self.filter_aabb_pairs(
                    ignore_structure=ignore_structure, return_profiling=False
                )
            
            if return_profiling:
                t_contact0 = time.perf_counter()
            
            # Contact point checks (using object_ids from candidate_pairs)
            for obj_id_i, obj_id_j in candidate_pairs:
                obj_i = self._sim_objects_dict.get(obj_id_i)
                obj_j = self._sim_objects_dict.get(obj_id_j)
                if obj_i is None or obj_j is None:
                    continue
                
                contact_points = p.getContactPoints(obj_i.body_id, obj_j.body_id)
                if contact_points:
                    collision_pairs.append((obj_id_i, obj_id_j))
                    collided.add(obj_id_i)
                    collided.add(obj_id_j)
            
            if return_profiling:
                timings["contact_points"] = (time.perf_counter() - t_contact0) * 1000  # ms
            
            # Log collision events
            if collision_pairs:
                for obj_id_i, obj_id_j in collision_pairs:
                    obj_i = self._sim_objects_dict.get(obj_id_i)
                    obj_j = self._sim_objects_dict.get(obj_id_j)
                    if obj_i and obj_j:
                        logger.info(f"Collision detected: object {obj_i.object_id} (body {obj_i.body_id}) <-> object {obj_j.object_id} (body {obj_j.body_id})")
            
            # Color update (collision: blue, normal: original) only if enabled
            if self.params.enable_collision_color_change:
                for obj_id, obj in self._sim_objects_dict.items():
                    was_collided = obj_id in self._last_collided
                    is_collided = obj_id in collided
                    if was_collided != is_collided:
                        if is_collided:
                            p.changeVisualShape(obj.body_id, -1, rgbaColor=collision_color)
                        else:
                            orig_color = self._robot_original_colors.get(obj.body_id, [0, 0, 0, 1])
                            p.changeVisualShape(obj.body_id, -1, rgbaColor=orig_color)
            self._last_collided = collided
            self.collision_count += len(collision_pairs)
        except p.error:
            # PyBullet disconnected, skip collision checking
            pass

        if return_profiling:
            timings["total"] = (time.perf_counter() - t0) * 1000  # ms
        
        return collision_pairs, timings

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
            "robots": len(self.sim_objects),
            "collisions": self.collision_count,
            "steps": self.step_count,
        }
        logging.debug(
            "[MONITOR] sim_time=%.2f, real_time=%.2f, speed=%.2f, collisions=%d, steps=%d",
            sim_time,
            elapsed_time,
            actual_speed,
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
                    # Speed>0: Synchronize with real time
                    loop_start = time.time()
                    current_time = time.time()
                    elapsed_time = current_time - start_time

                    # Calculate target and current simulation times
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

        # Clear movement tracking from previous step
        self._moved_this_step.clear()
        
        # Physics objects are always considered as potentially moved (conservative approach)
        # This avoids expensive pose comparison every step
        self._moved_this_step.update(self._physics_objects)
        
        # Update AABBs and spatial grid for physics objects immediately after physics step (batch operation)
        # Kinematic objects update their AABBs and spatial grid in set_pose() for immediate consistency
        for obj_id in self._physics_objects:
            # O(1) dict lookup instead of O(N) linear search
            obj = self._sim_objects_dict.get(obj_id)
            if obj is not None:
                self._update_object_aabb(obj_id, obj.body_id)
                # Update spatial grid if cell_size is initialized
                if self._cached_cell_size is not None:
                    self._update_object_spatial_grid(obj_id)

        self.sim_time = self.step_count * self.params.timestep

        # Update all simulation objects that have update() method
        # Agent instances are automatically updated every step for movement control
        if measure_timing:
            t0 = time.perf_counter()

        for obj in self.sim_objects:
            if isinstance(obj, Agent):
                moved = obj.update(self.params.timestep)
                # Track kinematic agent movement
                if moved and obj.object_id in self._kinematic_objects:
                    self._moved_this_step.add(obj.object_id)

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

        p.stepSimulation()

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
                'agent_update': 1000 * (t1 - t0),  # type: ignore[possibly-unbound]
                'callbacks': 1000 * (t_cb1 - t_cb0),  # type: ignore[possibly-unbound]
                'step_simulation': 1000 * (t_sim1 - t_sim0),  # type: ignore[possibly-unbound]
                'collision_check': 1000 * (t_col1 - t_col0),  # type: ignore[possibly-unbound]
                'monitor_update': 1000 * (t_mon1 - t_mon0),  # type: ignore[possibly-unbound]
                'total': 1000 * (t_end - t_step),  # type: ignore[possibly-unbound]
            }

        # Profiling output (only when profiling is enabled and not returning data)
        if self.enable_profiling:
            # Print profiling info at specified frequency
            # Note: All timing variables (t0, t1, etc.) are guaranteed to be defined
            # because they are all set within measure_timing=True blocks above
            if self.step_count % self._profiling_log_frequency == 0:
                # Build collision detail string if available
                collision_detail = ""
                if collision_timings:
                    collision_detail = (
                        f" [GetAABBs={collision_timings.get('get_aabbs', 0):.2f}ms, "
                        f"SpatialHash={collision_timings.get('spatial_hashing', 0):.2f}ms, "
                        f"AABBFilter={collision_timings.get('aabb_filtering', 0):.2f}ms, "
                        f"ContactPts={collision_timings.get('contact_points', 0):.2f}ms]"
                    )
                
                logger.debug(
                    f"[PROFILE] step {self.step_count}: "
                    f"Agent.update={1000*(t1-t0):.2f}ms, "  # type: ignore[possibly-unbound]
                    f"Callbacks={1000*(t_cb1-t_cb0):.2f}ms, "  # type: ignore[possibly-unbound]
                    f"stepSimulation={1000*(t_sim1-t_sim0):.2f}ms, "  # type: ignore[possibly-unbound]
                    f"Collisions={1000*(t_col1-t_col0):.2f}ms{collision_detail}, "  # type: ignore[possibly-unbound]
                    f"Monitor={1000*(t_mon1-t_mon0):.2f}ms, "  # type: ignore[possibly-unbound]
                    f"Total={1000*(t_end-t_step):.2f}ms"  # type: ignore[possibly-unbound]
                )
