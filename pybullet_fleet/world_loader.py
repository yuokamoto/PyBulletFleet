# pybullet_fleet/world_loader.py
"""Environment loading utilities for simulation worlds.

Provides helpers to load pre-built environment assets (OBJ meshes, SDF models)
into a PyBulletFleet simulation.
"""

import glob
import logging
import os
from pathlib import Path
from typing import List, Optional

from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import SimObject, ShapeParams
from pybullet_fleet.types import CollisionMode

logger = logging.getLogger(__name__)


def load_mesh_directory(
    mesh_dir: str,
    sim_core=None,
    collision_mode: CollisionMode = CollisionMode.NORMAL_3D,
    mesh_scale: Optional[List[float]] = None,
    rgba_color: Optional[List[float]] = None,
    pattern: str = "*.obj",
) -> List[SimObject]:
    """Load mesh files from a directory as static SimObjects.

    Scans ``mesh_dir`` for files matching ``pattern`` and creates a static
    SimObject for each one.  Object names are derived from filenames (stem).

    Originally written for rmf_demos environments where
    ``rmf_building_map_tools`` generates OBJ meshes with world-space
    vertices, but works with any directory of mesh files.

    Args:
        mesh_dir: Directory containing mesh files
        sim_core: Simulation core for registration
        collision_mode: Collision mode for environment objects
        mesh_scale: Optional scale override [sx, sy, sz] (default: [1,1,1])
        rgba_color: Optional color override [r,g,b,a] (default: grey)
        pattern: Glob pattern for mesh files (default: ``'*.obj'``)

    Returns:
        List of SimObject instances (one per mesh file)

    Raises:
        FileNotFoundError: If ``mesh_dir`` does not exist.

    Example::

        # Load OBJ meshes from a directory
        objects = load_mesh_directory('meshes/', sim_core=sim)

        # Load with custom pattern and colour
        objects = load_mesh_directory(
            'assets/', sim_core=sim, pattern='*.stl',
            rgba_color=[0.9, 0.9, 0.9, 1.0],
        )
    """
    mesh_dir = os.path.expanduser(mesh_dir)
    if not os.path.isdir(mesh_dir):
        raise FileNotFoundError(f"Mesh directory not found: {mesh_dir}")

    mesh_files = sorted(glob.glob(os.path.join(mesh_dir, pattern)))
    if not mesh_files:
        logger.warning("No %s files found in %s", pattern, mesh_dir)
        return []

    scale = mesh_scale or [1.0, 1.0, 1.0]
    objects: List[SimObject] = []

    def _load_meshes() -> None:
        for mesh_path in mesh_files:
            name = Path(mesh_path).stem

            visual = ShapeParams(
                shape_type="mesh",
                mesh_path=mesh_path,
                mesh_scale=scale,
                rgba_color=rgba_color or [0.7, 0.7, 0.7, 1.0],
            )
            collision = ShapeParams(
                shape_type="mesh",
                mesh_path=mesh_path,
                mesh_scale=scale,
            )

            # Origin placement: rmf_building_map_tools bakes world coordinates
            # into OBJ vertex data, so no per-mesh pose offset is needed.
            obj = SimObject.from_mesh(
                visual_shape=visual,
                collision_shape=collision,
                pose=Pose.from_xyz(0, 0, 0),
                mass=0.0,  # static
                sim_core=sim_core,
                collision_mode=collision_mode,
                name=name,
            )
            objects.append(obj)

    # Wrap in batch_spawn to disable rendering during bulk loading (GUI speedup)
    if sim_core is not None:
        with sim_core.batch_spawn():
            _load_meshes()
    else:
        _load_meshes()

    logger.info("Loaded %d environment objects from %s", len(objects), mesh_dir)
    return objects


def load_rmf_world(*args, **kwargs) -> List[SimObject]:
    """Backward-compatible alias for :func:`load_mesh_directory`.

    .. deprecated::
        Use :func:`load_mesh_directory` instead.
    """
    return load_mesh_directory(*args, **kwargs)
