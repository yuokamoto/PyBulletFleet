import random
from typing import Any, Callable, Dict, List, Optional, Union, TYPE_CHECKING
import logging

import numpy as np
import pybullet as p

from pybullet_fleet.sim_object import MeshObject, URDFObject

if TYPE_CHECKING:
    from pybullet_fleet.geometry import Pose

# Create logger for this module
logger = logging.getLogger(__name__)


def resolve_link_index(body_id: int, link: Union[int, str]) -> int:
    """
    Resolve link name to index.

    Args:
        body_id: PyBullet body ID
        link: Link index (int) or name (str). -1 or "base_link" for base link.

    Returns:
        Link index (int). Returns -1 if link is "base_link" or if link name not found.

    Example:
        >>> index = resolve_link_index(robot.body_id, "sensor_mast")
        >>> index = resolve_link_index(robot.body_id, 2)  # Also accepts int
        >>> index = resolve_link_index(robot.body_id, "base_link")  # Returns -1
    """
    if isinstance(link, int):
        return link

    # Handle special case for base link
    if link.lower() == "base_link":
        return -1

    # Search for link by name
    num_joints = p.getNumJoints(body_id)
    for i in range(num_joints):
        joint_info = p.getJointInfo(body_id, i)
        link_name = joint_info[12].decode("utf-8")  # Link name is at index 12
        if link_name == link:
            return i

    # Link not found
    logger.warning(f"Link '{link}' not found on body {body_id}, using base link (-1)")
    return -1


def calculate_offset_pose(
    target_position: List[float], current_position: List[float], offset: float, keep_height: bool = True
) -> "Pose":
    """
    Calculate pose at specified offset distance from target.

    This unified function can be used for various offset calculations:
    - Approach poses (approach_offset): Position before reaching target
    - Pick poses (pick_offset): Position where pick operation occurs
    - Drop poses (drop_offset): Position where drop operation occurs

    Args:
        target_position: Target position [x, y, z]
        current_position: Current agent position [x, y, z]
        offset: Distance from target (positive = away from target, 0 = at target)
        keep_height: Whether to keep current height (default: True)

    Returns:
        Pose object at the offset position, oriented to face the target

    Example:
        >>> # Approach pose (1.5m away from target)
        >>> approach = calculate_offset_pose([5, 0, 0.1], [0, 0, 0.3], offset=1.5)
        >>>
        >>> # Pick pose (0.3m away from target)
        >>> pick = calculate_offset_pose([5, 0, 0.1], [0, 0, 0.3], offset=0.3)
        >>>
        >>> # At target position (0m offset)
        >>> drop = calculate_offset_pose([5, 0, 0.1], [0, 0, 0.3], offset=0.0)
    """
    from pybullet_fleet.geometry import Pose

    target_pos = np.array(target_position)
    current_pos = np.array(current_position)

    # Calculate direction from current to target
    direction = target_pos - current_pos
    direction[2] = 0  # Project to XY plane

    direction_norm = np.linalg.norm(direction)
    if direction_norm > 1e-6:
        direction = direction / direction_norm
        yaw = np.arctan2(direction[1], direction[0])
    else:
        yaw = 0.0

    # Calculate offset position (away from target by offset distance)
    offset_pos = target_pos - direction * offset

    # Use current height or target height
    z = current_pos[2] if keep_height else target_pos[2]
    offset_pos[2] = z

    # Orientation: face the target with horizontal orientation
    return Pose.from_euler(x=offset_pos[0], y=offset_pos[1], z=offset_pos[2], roll=0.0, pitch=0.0, yaw=yaw)


def calculate_approach_pose(
    target_position: List[float], current_position: List[float], approach_offset: float, keep_height: bool = True
) -> "Pose":
    """
    Calculate approach pose for pick/drop operations.

    .. deprecated::
        This function is deprecated and kept for backward compatibility.
        Use :func:`calculate_offset_pose` instead.

    Args:
        target_position: Target position [x, y, z]
        current_position: Current agent position [x, y, z]
        approach_offset: Distance from target for approach pose
        keep_height: Whether to keep current height (default: True)

    Returns:
        Pose object representing the approach pose

    Example:
        >>> # Old way (deprecated but still works)
        >>> approach = calculate_approach_pose([5, 0, 0.1], [0, 0, 0.3], approach_offset=1.0)
        >>>
        >>> # New way (recommended)
        >>> approach = calculate_offset_pose([5, 0, 0.1], [0, 0, 0.3], offset=1.0)
    """
    return calculate_offset_pose(target_position, current_position, approach_offset, keep_height)


def world_to_grid(pos: List[float], spacing: List[float], offset: Optional[List[float]] = None) -> List[int]:
    """
    Convert world coordinates to grid indices (always 3D), with offset.

    Args:
        pos: [x, y, z] or [x, y] world coordinates
        spacing: [spacing_x, spacing_y, spacing_z] or [spacing_x, spacing_y] grid spacing
        offset: [offset_x, offset_y, offset_z] or [offset_x, offset_y] offset (optional)

    Returns:
        [ix, iy, iz] grid indices
    """
    x = pos[0]
    y = pos[1]
    z = pos[2] if len(pos) > 2 else 0.0
    spacing_x = spacing[0]
    spacing_y = spacing[1]
    spacing_z = spacing[2] if len(spacing) > 2 else 1.0
    offset_x = offset[0] if offset and len(offset) > 0 else 0.0
    offset_y = offset[1] if offset and len(offset) > 1 else 0.0
    offset_z = offset[2] if offset and len(offset) > 2 else 0.0
    grid_ix = int(round((x - offset_x) / spacing_x))
    grid_iy = int(round((y - offset_y) / spacing_y))
    grid_iz = int(round((z - offset_z) / spacing_z))
    return [grid_ix, grid_iy, grid_iz]


def grid_to_world(grid: List[int], spacing: List[float], offset: Optional[List[float]] = None) -> List[float]:
    """
    Convert grid indices to world coordinates (always 3D), with offset.

    Args:
        grid: [ix, iy, iz] or [ix, iy] grid indices
        spacing: [spacing_x, spacing_y, spacing_z] or [spacing_x, spacing_y] grid spacing
        offset: [offset_x, offset_y, offset_z] or [offset_x, offset_y] offset (optional)

    Returns:
        [x, y, z] world coordinates
    """
    ix = grid[0]
    iy = grid[1]
    iz = grid[2] if len(grid) > 2 else 0
    spacing_x = spacing[0]
    spacing_y = spacing[1]
    spacing_z = spacing[2] if len(spacing) > 2 else 1.0
    offset_x = offset[0] if offset and len(offset) > 0 else 0.0
    offset_y = offset[1] if offset and len(offset) > 1 else 0.0
    offset_z = offset[2] if offset and len(offset) > 2 else 0.0
    x = ix * spacing_x + offset_x
    y = iy * spacing_y + offset_y
    z = iz * spacing_z + offset_z
    return [x, y, z]


def world_to_grid_2d(pos: List[float], spacing: List[float]) -> List[int]:
    """
    2D wrapper for world_to_grid.

    Args:
        pos: [x, y] world coordinates
        spacing: [spacing_x, spacing_y] grid spacing

    Returns:
        [ix, iy] grid indices
    """
    return world_to_grid(pos[:2], spacing[:2])


def grid_to_world_2d(grid: List[int], spacing: List[float]) -> List[float]:
    """
    2D wrapper for grid_to_world.

    Args:
        grid: [ix, iy] grid indices
        spacing: [spacing_x, spacing_y] grid spacing

    Returns:
        [x, y] world coordinates
    """
    return grid_to_world(grid[:2], spacing[:2])


def grid_execution(
    grid_num: List[int] = None,
    spacing: List[float] = None,
    offset: Optional[List[float]] = None,
    func: Optional[Callable] = None,
    args: Optional[Dict] = None,
) -> None:
    """
    Calls the callback function func(grid_index, world_pos, **args) at each grid point.

    Args:
        grid_num: [x_num, y_num, z_num] number of grids in each axis (default: [1, 1, 1])
        spacing: [spacing_x, spacing_y, spacing_z] grid spacing (default: [1.0, 1.0, 1.0])
        offset: [offset_x, offset_y, offset_z] offset (default: [0.0, 0.0, 0.0])
        func: Callback function(grid_index: List[int], world_pos: List[float], **args)
              - grid_index: [ix, iy, iz] grid indices
              - world_pos: [x, y, z] world coordinates
        args: Additional arguments to pass to the callback function
    """
    if func is None:
        raise ValueError("Please specify func (callback function)")
    if grid_num is None:
        grid_num = [1, 1, 1]
    if spacing is None:
        spacing = [1.0, 1.0, 1.0]
    if offset is None:
        offset = [0.0, 0.0, 0.0]

    x_num, y_num, z_num = grid_num[0], grid_num[1], grid_num[2]
    for ix in range(x_num):
        for iy in range(y_num):
            for iz in range(z_num):
                grid_index = [ix, iy, iz]
                world_pos = grid_to_world(grid_index, spacing, offset)
                if args:
                    func(grid_index, world_pos, **args)
                else:
                    func(grid_index, world_pos)


def grid_spawn(
    model_class: Any,
    model_paths: List[str],
    sim_core: Any,
    grid_num: List[int] = None,
    spacing: List[float] = None,
    offset: Optional[List[float]] = None,
    spawn_probability: float = 1.0,
    orientation_euler: Optional[List[float]] = None,
    extra_args: Optional[Dict] = None,
    max_spawn: Optional[int] = None,
) -> List[Any]:
    """
    model_class: Class such as URDFObject, MeshObject
    model_paths: List of model file paths (can be just one)
    sim_core: MultiRobotSimulationCore instance
    grid_num: [x_num, y_num, z_num] number of grids in each axis (default: [1, 1, 1])
    spacing: [spacing_x, spacing_y, spacing_z] grid spacing (default: [1.0, 1.0, 1.0])
    offset: [offset_x, offset_y, offset_z] offset (default: [0.0, 0.0, 0.0])
    spawn_probability: Probability of spawning (0.0~1.0, default=1.0)
    orientation_euler: [roll, pitch, yaw] (default [0,0,0])
    extra_args: dict for additional parameters (specific to URDFObject/MeshObject)
    max_spawn: Maximum number of objects to spawn (None for unlimited)
    """
    if grid_num is None:
        grid_num = [1, 1, 1]
    if orientation_euler is None:
        orientation_euler = [0, 0, 0]
    if spacing is None:
        spacing = [1.0, 1.0, 1.0]
    if offset is None:
        offset = [0.0, 0.0, 0.0]
    if extra_args is None:
        extra_args = {}

    x_num, y_num, z_num = grid_num[0], grid_num[1], grid_num[2]
    spawned_objects = []
    total_grids = x_num * y_num * z_num
    spawn_count = [0]  # mutable for closure

    def spawn_func(grid_index: List[int], world_pos: List[float], **args):
        ix, iy, iz = grid_index[0], grid_index[1], grid_index[2]
        x, y, z = world_pos[0], world_pos[1], world_pos[2]

        # 1) If max_spawn is set and reached, do not spawn further
        if max_spawn is not None and spawn_count[0] >= max_spawn:
            return
        # 2) If remaining grids == remaining spawn, always spawn
        remaining_grids = total_grids - (ix * y_num * z_num + iy * z_num + iz)
        remaining_spawn = max_spawn - spawn_count[0] if max_spawn is not None else None
        force_spawn = False
        if max_spawn is not None:
            if remaining_spawn is not None and remaining_grids <= remaining_spawn:
                force_spawn = True
        # 3) If max_spawn > total_grids, warning and spawn in all grids
        if max_spawn is not None and max_spawn > total_grids:
            import warnings

            warnings.warn(f"max_spawn ({max_spawn}) is greater than grid count ({total_grids}). Spawning in all grids.")
            force_spawn = True
        # random spawn or forced spawn
        if force_spawn or random.random() <= spawn_probability:
            model_path = random.choice(model_paths)
            orientation = None
            if orientation_euler is not None:
                orientation = p.getQuaternionFromEuler(orientation_euler)
            obj_args = {"position": [x, y, z], "orientation": orientation}
            obj_args.update(args)
            obj_args["sim_core"] = sim_core
            # Branch based on whether it's a class or function
            if hasattr(model_class, "__name__") and model_class.__name__ == "MeshObject":
                obj_args["mesh_path"] = model_path
                obj = model_class.from_mesh(**obj_args)
                sim_core.mesh_objects.append(obj)
            elif hasattr(model_class, "__name__") and model_class.__name__ == "URDFObject":
                obj_args["urdf_path"] = model_path
                obj = model_class.from_urdf(**obj_args)
                sim_core.robots.append(obj)
            else:
                # Factory function (for speedup)
                obj = model_class(position=[x, y, z], orientation=orientation, sim_core=sim_core)
                sim_core.mesh_objects.append(obj)
            spawned_objects.append(obj)
            spawn_count[0] += 1

    grid_execution(grid_num=grid_num, spacing=spacing, offset=offset, func=spawn_func, args=extra_args)
    return spawned_objects


def grid_spawn_urdf(
    model_paths: List[str],
    sim_core: Any,
    grid_num: List[int] = None,
    spacing: List[float] = None,
    offset: Optional[List[float]] = None,
    spawn_probability: float = 1.0,
    orientation_euler: List[float] = None,
    useFixedBase: bool = False,
    set_mass_zero: bool = False,
    meta_data: Optional[Dict] = None,
    max_spawn: Optional[int] = None,
) -> List[Any]:
    """
    Spawn URDF objects in a grid pattern.

    Args:
        model_paths: List of URDF file paths
        sim_core: MultiRobotSimulationCore instance
        grid_num: [x_num, y_num, z_num] number of grids in each axis (default: [1, 1, 1])
        spacing: [spacing_x, spacing_y, spacing_z] grid spacing (default: [1.0, 1.0, 1.0])
        offset: [offset_x, offset_y, offset_z] offset (default: [0.0, 0.0, 0.0])
        spawn_probability: Probability of spawning (0.0~1.0, default=1.0)
        orientation_euler: [roll, pitch, yaw] (default [0,0,0])
        useFixedBase: Whether to fix the base
        set_mass_zero: Whether to set mass to zero
        meta_data: Additional metadata
        max_spawn: Maximum number of objects to spawn (None for unlimited)
    """
    if orientation_euler is None:
        orientation_euler = [0, 0, 0]
    extra_args = {"useFixedBase": useFixedBase, "set_mass_zero": set_mass_zero}
    if meta_data is not None:
        extra_args["meta_data"] = meta_data
    return grid_spawn(
        model_class=URDFObject,
        model_paths=model_paths,
        sim_core=sim_core,
        grid_num=grid_num,
        spacing=spacing,
        offset=offset,
        spawn_probability=spawn_probability,
        orientation_euler=orientation_euler,
        extra_args=extra_args,
        max_spawn=max_spawn,
    )


def grid_spawn_mesh(
    model_paths: List[str],
    sim_core: Any,
    grid_num: List[int] = None,
    spacing: List[float] = None,
    offset: Optional[List[float]] = None,
    spawn_probability: float = 1.0,
    orientation_euler: List[float] = None,
    base_mass: float = 0.0,
    mesh_scale: List[float] = None,
    rgbaColor: List[float] = None,
    max_spawn: Optional[int] = None,
) -> List[Any]:
    """
    Spawn mesh objects in a grid pattern.

    Args:
        model_paths: List of mesh file paths
        sim_core: MultiRobotSimulationCore instance
        grid_num: [x_num, y_num, z_num] number of grids in each axis (default: [1, 1, 1])
        spacing: [spacing_x, spacing_y, spacing_z] grid spacing (default: [1.0, 1.0, 1.0])
        offset: [offset_x, offset_y, offset_z] offset (default: [0.0, 0.0, 0.0])
        spawn_probability: Probability of spawning (0.0~1.0, default=1.0)
        orientation_euler: [roll, pitch, yaw] (default [0,0,0])
        base_mass: Mass of the mesh object
        mesh_scale: [scale_x, scale_y, scale_z] (default: [1.0, 1.0, 1.0])
        rgbaColor: [r, g, b, a] color (default: [1.0, 1.0, 1.0, 1.0])
        max_spawn: Maximum number of objects to spawn (None for unlimited)
    """
    if orientation_euler is None:
        orientation_euler = [0, 0, 0]
    if mesh_scale is None:
        mesh_scale = [1, 1, 1]
    if rgbaColor is None:
        rgbaColor = [1, 1, 1, 1]
    # Create visual/collision shape only once
    mesh_path = model_paths[0] if isinstance(model_paths, list) else model_paths
    vis_id = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=mesh_path, meshScale=mesh_scale, rgbaColor=rgbaColor)
    col_id = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=mesh_path, meshScale=mesh_scale)

    def mesh_object_factory(position, orientation, sim_core=None, **kwargs):
        body_id = p.createMultiBody(
            baseMass=base_mass,
            baseCollisionShapeIndex=col_id,
            baseVisualShapeIndex=vis_id,
            basePosition=position,
            baseOrientation=orientation,
        )
        return MeshObject(body_id=body_id, mesh_path=mesh_path, visual_id=vis_id, collision_id=col_id, sim_core=sim_core)

    # Pass factory function to grid_spawn's model_class argument
    return grid_spawn(
        model_class=mesh_object_factory,
        model_paths=[mesh_path],
        sim_core=sim_core,
        grid_num=grid_num,
        spacing=spacing,
        offset=offset,
        spawn_probability=spawn_probability,
        orientation_euler=orientation_euler,
        extra_args={},
        max_spawn=max_spawn,
    )
