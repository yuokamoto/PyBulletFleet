# pybullet_fleet/sdf_loader.py
"""SDF / mesh world loader for PyBulletFleet.

**Why self-implemented?**  PyBullet's ``loadSDF()`` crashes on multi-root-link
models (segfault) and rejects Gazebo SDF joints without ``<dynamics>`` defaults.
No usable Python SDF→URDF library exists — ``sdformat_urdf`` is C++-only with
no Python bindings, ``pysdf`` (PyPI) fails to build on Python 3.12, and
``libsdformat14`` is a C++ template API not callable via ctypes.

This module provides:

- :func:`load_mesh_directory` — OBJ/STL mesh directory loader (from world_loader).
- :func:`load_sdf_world` — Load a single SDF ``<model>`` (building) as meshes.
- :func:`load_sdf_world_file` — Load a ``.world`` file, resolving ``<include>``
  elements (Gazebo Fuel URIs, ``model://`` local refs) into SimObjects.
- :func:`resolve_sdf_to_urdf` — Convert a robot SDF into a temporary URDF.

Backward-compatible aliases:

- :func:`load_rmf_world` → :func:`load_mesh_directory`
"""

import glob
import logging
import math
import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import SimObject, ShapeParams
from pybullet_fleet.types import CollisionMode

logger = logging.getLogger(__name__)

_FUEL_CACHE = os.path.expanduser("~/.gz/fuel")

#: Default color for scenery objects without material info.
#: Matches Gazebo Sim (Harmonic) default — a neutral light grey
#: that renders well under PyBullet's default lighting.
DEFAULT_SCENERY_COLOR = [0.7, 0.7, 0.7, 1.0]


# ---------------------------------------------------------------------------
# model:// URI resolution
# ---------------------------------------------------------------------------


def _resolve_model_uri(uri: str, model_dir: str, model_name: str) -> str:
    """Resolve a ``model://ModelName/path`` URI to an absolute filesystem path.

    Also handles Gazebo Fuel HTTPS URIs like
    ``https://fuel.gazebosim.org/1.0/owner/models/name/ver/files/path``
    by mapping them to the local Fuel cache (``~/.gz/fuel/...``).

    Args:
        uri: Raw URI from SDF.
        model_dir: Absolute directory containing the SDF file.
        model_name: The ``<model name="...">`` attribute from the SDF.

    Returns:
        Absolute filesystem path.
    """
    if uri.startswith("https://fuel.gazebosim.org/"):
        # https://fuel.gazebosim.org/1.0/owner/models/name/ver/files/rest
        # → ~/.gz/fuel/fuel.gazebosim.org/owner/models/name/ver/rest
        stripped = uri.replace("https://", "")
        # Remove the "1.0/" API version segment
        stripped = stripped.replace("fuel.gazebosim.org/1.0/", "fuel.gazebosim.org/", 1)
        # Convert owner to lowercase (Fuel cache uses lowercase)
        parts = stripped.split("/")
        if len(parts) >= 2:
            parts[1] = parts[1].lower()  # owner
        if len(parts) >= 4:
            parts[3] = parts[3].lower()  # model name
        # Remove "files" segment if present
        if "files" in parts:
            parts.remove("files")
        local = os.path.join(_FUEL_CACHE, *parts[0:])
        # Also check model_dir for meshes subdir as fallback
        if os.path.isfile(local):
            return local
        # Try without lowercasing model name
        return os.path.join(os.path.expanduser("~/.gz/fuel"), *stripped.split("/"))

    if not uri.startswith("model://"):
        return uri

    # model://ModelName/rest/of/path  OR  model://other_model/...
    rest = uri[len("model://") :]
    parts = rest.split("/", 1)
    if len(parts) == 2 and parts[0].lower() == model_name.lower():
        return os.path.join(model_dir, parts[1])
    # Fallback: try relative to model_dir's parent (co-located models)
    return os.path.join(os.path.dirname(model_dir), rest)


def _parse_pose(
    pose_text: Optional[str],
) -> Tuple[float, float, float, float, float, float]:
    """Parse an SDF ``<pose>`` string ``'x y z roll pitch yaw'``.

    Returns:
        ``(x, y, z, roll, pitch, yaw)`` — always a 6-tuple.
        Missing orientation components default to ``0.0``.
    """
    if not pose_text or not pose_text.strip():
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    parts = pose_text.strip().split()
    x = float(parts[0]) if len(parts) > 0 else 0.0
    y = float(parts[1]) if len(parts) > 1 else 0.0
    z = float(parts[2]) if len(parts) > 2 else 0.0
    roll = float(parts[3]) if len(parts) > 3 else 0.0
    pitch = float(parts[4]) if len(parts) > 4 else 0.0
    yaw = float(parts[5]) if len(parts) > 5 else 0.0
    return (x, y, z, roll, pitch, yaw)


# ---------------------------------------------------------------------------
# Mesh directory loader  (moved from world_loader.py)
# ---------------------------------------------------------------------------


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
                rgba_color=rgba_color or list(DEFAULT_SCENERY_COLOR),
            )
            collision = ShapeParams(
                shape_type="mesh",
                mesh_path=mesh_path,
                mesh_scale=scale,
            )
            obj = SimObject.from_mesh(
                visual_shape=visual,
                collision_shape=collision,
                pose=Pose.from_xyz(0, 0, 0),
                mass=0.0,
                sim_core=sim_core,
                collision_mode=collision_mode,
                name=name,
            )
            objects.append(obj)

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


# ---------------------------------------------------------------------------
# World (static building) loader
# ---------------------------------------------------------------------------


def load_sdf_world(
    sdf_path: str,
    sim_core=None,
    collision_mode: CollisionMode = CollisionMode.DISABLED,
    rgba_color: Optional[List[float]] = None,
) -> List[SimObject]:
    """Load static building models from an SDF file.

    Parses all ``<model>`` elements, iterates over their ``<link>`` children,
    extracts mesh ``<uri>`` from the *first* ``<visual>`` and the
    link-level ``<pose>``, then creates a :class:`SimObject` for each.

    Supports SDF files containing multiple top-level ``<model>`` elements.

    This correctly handles rmf_building_map_tools output where floor
    meshes have world-baked vertices (pose = 0) and wall meshes have
    local vertices with per-link pose offsets.

    Args:
        sdf_path: Absolute path to ``model.sdf``.
        sim_core: Simulation core for object registration.
        collision_mode: Collision mode for environment objects
            (default: ``DISABLED`` — static scenery).
        rgba_color: Optional colour override ``[r, g, b, a]``.

    Returns:
        List of created SimObject instances.

    Example::

        from pybullet_fleet.sdf_loader import load_sdf_world

        objects = load_sdf_world(
            "/rmf_demos_ws/.../building_L1/model.sdf",
            sim_core=sim,
        )
    """
    sdf_path = os.path.expanduser(sdf_path)
    if not os.path.isfile(sdf_path):
        raise FileNotFoundError(f"SDF file not found: {sdf_path}")

    model_dir = os.path.dirname(sdf_path)
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    model_els = root.findall(".//model")
    if not model_els:
        raise ValueError(f"No <model> element found in {sdf_path}")

    color = rgba_color or list(DEFAULT_SCENERY_COLOR)
    objects: List[SimObject] = []

    def _load() -> None:
        for model_el in model_els:
            model_name = model_el.get("name", "")
            for link in model_el.findall("link"):
                link_name = link.get("name", "unknown")

                # Extract pose
                pose_el = link.find("pose")
                x, y, z, roll, pitch, yaw = _parse_pose(pose_el.text if pose_el is not None else None)

                # Find first visual mesh URI and scale
                mesh_uri = None
                visual_scale = [1.0, 1.0, 1.0]
                for visual in link.findall("visual"):
                    mesh_el = visual.find(".//mesh/uri")
                    if mesh_el is not None and mesh_el.text:
                        mesh_uri = mesh_el.text
                        scale_el = visual.find(".//mesh/scale")
                        if scale_el is not None and scale_el.text:
                            parts = scale_el.text.strip().split()
                            if len(parts) >= 3:
                                visual_scale = [float(parts[0]), float(parts[1]), float(parts[2])]
                        break

                if mesh_uri is None:
                    logger.debug("Link '%s' has no visual mesh, skipping", link_name)
                    continue

                mesh_path = _resolve_model_uri(mesh_uri, model_dir, model_name)
                if not os.path.isfile(mesh_path):
                    logger.warning("Mesh file not found: %s (from %s)", mesh_path, mesh_uri)
                    continue

                # Find collision mesh and scale (use visual mesh as fallback)
                collision_uri = None
                collision_scale = list(visual_scale)
                for coll in link.findall("collision"):
                    coll_mesh = coll.find(".//mesh/uri")
                    if coll_mesh is not None and coll_mesh.text:
                        collision_uri = coll_mesh.text
                        coll_scale_el = coll.find(".//mesh/scale")
                        if coll_scale_el is not None and coll_scale_el.text:
                            parts = coll_scale_el.text.strip().split()
                            if len(parts) >= 3:
                                collision_scale = [float(parts[0]), float(parts[1]), float(parts[2])]
                        break
                collision_path = _resolve_model_uri(collision_uri, model_dir, model_name) if collision_uri else mesh_path

                visual_shape = ShapeParams(
                    shape_type="mesh",
                    mesh_path=mesh_path,
                    mesh_scale=visual_scale,
                    rgba_color=color,
                )
                collision_shape = ShapeParams(
                    shape_type="mesh",
                    mesh_path=collision_path if os.path.isfile(collision_path) else mesh_path,
                    mesh_scale=collision_scale,
                )

                obj = SimObject.from_mesh(
                    visual_shape=visual_shape,
                    collision_shape=collision_shape,
                    pose=Pose.from_euler(x, y, z, roll, pitch, yaw),
                    mass=0.0,
                    sim_core=sim_core,
                    collision_mode=collision_mode,
                    name=link_name,
                )
                objects.append(obj)

    if sim_core is not None:
        with sim_core.batch_spawn():
            _load()
    else:
        _load()

    logger.info("Loaded %d links from SDF %s", len(objects), sdf_path)
    return objects


# ---------------------------------------------------------------------------
# .world file loader  (handles <include> + Gazebo Fuel URIs)
# ---------------------------------------------------------------------------


def _resolve_fuel_uri(uri: str) -> Optional[str]:
    """Resolve a Gazebo Fuel ``https://fuel.gazebosim.org/...`` URI to a local model dir.

    Downloads the model with ``gz fuel download`` if not already cached.
    Returns the local directory containing ``model.sdf``, or *None* on failure.
    """
    if not uri.startswith("https://fuel.gazebosim.org/"):
        return None

    # URL structure: https://fuel.gazebosim.org/1.0/<owner>/models/<model>
    # Local cache:   ~/.gz/fuel/fuel.gazebosim.org/<owner>/models/<model>/<ver>/
    parts = uri.replace("https://fuel.gazebosim.org/1.0/", "").split("/")
    if len(parts) < 3:  # owner/models/ModelName
        return None
    owner = parts[0].lower()
    model_name = parts[2].lower()

    cache_dir = os.path.join(_FUEL_CACHE, "fuel.gazebosim.org", owner, "models", model_name)

    # Check cache first
    if os.path.isdir(cache_dir):
        versions = sorted(os.listdir(cache_dir))
        if versions:
            candidate = os.path.join(cache_dir, versions[-1])
            if os.path.isfile(os.path.join(candidate, "model.sdf")):
                return candidate

    # Download via gz fuel
    if shutil.which("gz") is None:
        logger.warning("'gz' CLI not found — cannot download Fuel model: %s", uri)
        return None

    try:
        subprocess.run(
            ["gz", "fuel", "download", "-u", uri],
            capture_output=True,
            timeout=60,
            check=False,
        )
    except Exception as e:
        logger.warning("Failed to download Fuel model %s: %s", uri, e)
        return None

    # Re-check cache
    if os.path.isdir(cache_dir):
        versions = sorted(os.listdir(cache_dir))
        if versions:
            candidate = os.path.join(cache_dir, versions[-1])
            if os.path.isfile(os.path.join(candidate, "model.sdf")):
                return candidate

    logger.warning("Fuel model not found after download: %s", uri)
    return None


def _resolve_include_uri(
    uri: str,
    resource_paths: Optional[List[str]] = None,
) -> Optional[str]:
    """Resolve ``<uri>`` from a ``<include>`` element to a local model directory.

    Handles:
    - ``https://fuel.gazebosim.org/...`` — Gazebo Fuel download/cache
    - ``model://ModelName`` — search ``GZ_SIM_RESOURCE_PATH`` and *resource_paths*
    """
    if uri.startswith("https://fuel.gazebosim.org/"):
        return _resolve_fuel_uri(uri)

    if uri.startswith("model://"):
        model_name = uri[len("model://") :]
        paths = list(resource_paths or [])
        env_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
        if env_path:
            paths.extend(env_path.split(":"))
        for base in paths:
            candidate = os.path.join(base, model_name)
            if os.path.isfile(os.path.join(candidate, "model.sdf")):
                return candidate
        logger.warning("model://%s not found in resource paths", model_name)
        return None

    # Absolute or relative path
    if os.path.isdir(uri) and os.path.isfile(os.path.join(uri, "model.sdf")):
        return uri

    return None


def load_sdf_world_file(
    world_path: str,
    sim_core=None,
    collision_mode: CollisionMode = CollisionMode.DISABLED,
    skip_models: Optional[List[str]] = None,
    resource_paths: Optional[List[str]] = None,
) -> List[SimObject]:
    """Load a Gazebo ``.world`` file with ``<include>`` elements.

    Parses the ``<world>`` element, resolves each ``<include>`` to a local
    model directory (via Fuel download or ``model://`` search), then calls
    :func:`load_sdf_world` for the building model and creates SimObjects
    for furniture/prop models.

    Args:
        world_path: Absolute path to the ``.world`` file.
        sim_core: Simulation core for registration.
        collision_mode: Collision mode for loaded objects.
        skip_models: Model names to skip (e.g. robot names like ``TinyRobot``).
        resource_paths: Additional directories to search for ``model://`` URIs.

    Returns:
        All SimObjects loaded from the world file.
    """
    world_path = os.path.expanduser(world_path)
    if not os.path.isfile(world_path):
        raise FileNotFoundError(f"World file not found: {world_path}")

    tree = ET.parse(world_path)
    root = tree.getroot()

    world_el = root.find(".//world")
    if world_el is None:
        raise ValueError(f"No <world> element found in {world_path}")

    # Auto-discover model:// resource paths from the world file's directory.
    # rmf_demos_maps convention: <map_dir>/models/ contains building SDF models.
    effective_paths = list(resource_paths or [])
    world_dir = os.path.dirname(world_path)
    models_dir = os.path.join(world_dir, "models")
    if os.path.isdir(models_dir) and models_dir not in effective_paths:
        effective_paths.insert(0, models_dir)

    skip = {s.lower() for s in (skip_models or [])}
    all_objects: List[SimObject] = []

    # --- Phase 1: prefetch ------------------------------------------------
    # Resolve all include URIs (downloading Fuel models as needed) BEFORE
    # touching PyBullet.  This avoids long network waits while the GUI
    # rendering thread is running, which can crash on Intel GPUs.

    @dataclass
    class _IncludeInfo:
        name: str
        model_type: str
        sdf_path: str
        ix: float
        iy: float
        iz: float
        iroll: float
        ipitch: float
        iyaw: float

    resolved: List[_IncludeInfo] = []

    for include in world_el.findall("include"):
        name_el = include.find("name")
        uri_el = include.find("uri")
        if uri_el is None or not uri_el.text:
            continue

        include_name = name_el.text.strip() if name_el is not None else ""
        uri = uri_el.text.strip()

        # Extract model type name from URI for skip check
        model_type = uri.rstrip("/").split("/")[-1]
        if model_type.lower() in skip or include_name.lower() in skip:
            logger.debug("Skipping model: %s (%s)", include_name, model_type)
            continue

        # Resolve URI to local directory (may download from Fuel)
        model_dir = _resolve_include_uri(uri, effective_paths)
        if model_dir is None:
            logger.warning("Cannot resolve include URI: %s", uri)
            continue

        sdf_path = os.path.join(model_dir, "model.sdf")

        # Parse include-level pose
        pose_el = include.find("pose")
        pose_text = pose_el.text if pose_el is not None else None
        ix, iy, iz, iroll, ipitch, iyaw = _parse_pose(pose_text)

        resolved.append(
            _IncludeInfo(
                name=include_name,
                model_type=model_type,
                sdf_path=sdf_path,
                ix=ix,
                iy=iy,
                iz=iz,
                iroll=iroll,
                ipitch=ipitch,
                iyaw=iyaw,
            )
        )

    logger.info("Prefetched %d models, loading into PyBullet...", len(resolved))

    # --- Phase 2: load into PyBullet --------------------------------------
    for info in resolved:
        try:
            model_objs = load_sdf_world(
                info.sdf_path,
                sim_core=sim_core,
                collision_mode=collision_mode,
            )
        except Exception as e:
            logger.warning("Failed to load model SDF %s: %s", info.sdf_path, e)
            continue

        # Offset each object by the include-level pose
        cos_y, sin_y = math.cos(info.iyaw), math.sin(info.iyaw)
        for obj in model_objs:
            obj_pose = obj.get_pose()
            ox, oy, oz = obj_pose.position
            rx = ox * cos_y - oy * sin_y + info.ix
            ry = ox * sin_y + oy * cos_y + info.iy
            rz = oz + info.iz
            obj.set_pose(Pose.from_euler(rx, ry, rz, info.iroll, info.ipitch, info.iyaw))
            base_name = info.name or info.model_type
            obj.name = f"{base_name}/{obj.name}"

        all_objects.extend(model_objs)

    logger.info("Loaded %d objects from world %s", len(all_objects), world_path)
    return all_objects


# ---------------------------------------------------------------------------
# Robot SDF → temporary URDF converter
# ---------------------------------------------------------------------------


def resolve_sdf_to_urdf(sdf_path: str) -> str:
    """Convert an SDF robot model to a temporary URDF file.

    PyBullet cannot reliably load Gazebo SDF files (``model://`` URIs,
    missing joint dynamics defaults). This function:

    1. Parses the SDF XML.
    2. Resolves ``model://`` mesh URIs to absolute filesystem paths.
    3. Adds missing ``<dynamics>`` defaults that PyBullet requires.
    4. Writes a minimal URDF to a temporary file.

    The caller is responsible for deleting the temp file when done
    (or it will be cleaned up on process exit by the OS).

    Args:
        sdf_path: Absolute path to the robot ``model.sdf``.

    Returns:
        Absolute path to the generated temporary URDF file.

    Example::

        from pybullet_fleet.sdf_loader import resolve_sdf_to_urdf

        urdf_path = resolve_sdf_to_urdf("/path/to/TinyRobot/model.sdf")
        agent = Agent.from_urdf(urdf_path, ...)
    """
    sdf_path = os.path.expanduser(sdf_path)
    if not os.path.isfile(sdf_path):
        raise FileNotFoundError(f"SDF file not found: {sdf_path}")

    model_dir = os.path.dirname(sdf_path)
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    model_el = root.find(".//model")
    if model_el is None:
        raise ValueError(f"No <model> element found in {sdf_path}")
    model_name = model_el.get("name", "unknown")

    # Collect links and joints
    links: Dict[str, ET.Element] = {}
    joints: List[ET.Element] = []
    for link in model_el.findall("link"):
        links[link.get("name", "")] = link
    for joint in model_el.findall("joint"):
        joints.append(joint)

    # Build URDF string
    lines: List[str] = [
        '<?xml version="1.0"?>',
        f'<robot name="{model_name}">',
    ]

    for link_name, link_el in links.items():
        lines.append(f'  <link name="{link_name}">')

        # Inertial
        inertial = link_el.find("inertial")
        if inertial is not None:
            mass_el = inertial.find("mass")
            mass_val = mass_el.text.strip() if mass_el is not None else "1.0"
            pose_el = inertial.find("pose")
            ipose = pose_el.text.strip().split() if pose_el is not None else ["0"] * 6
            origin_xyz = " ".join(ipose[:3])
            origin_rpy = " ".join(ipose[3:6]) if len(ipose) >= 6 else "0 0 0"

            inertia_el = inertial.find("inertia")
            ixx = iyy = izz = "0.01"
            ixy = ixz = iyz = "0"
            if inertia_el is not None:
                for tag in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"):
                    el = inertia_el.find(tag)
                    if el is not None:
                        locals()[tag] = el.text.strip()

            lines.append("    <inertial>")
            lines.append(f'      <mass value="{mass_val}"/>')
            lines.append(f'      <origin xyz="{origin_xyz}" rpy="{origin_rpy}"/>')
            lines.append(f'      <inertia ixx="{ixx}" ixy="{ixy}" ixz="{ixz}" iyy="{iyy}" iyz="{iyz}" izz="{izz}"/>')
            lines.append("    </inertial>")

        # Visuals
        for visual in link_el.findall("visual"):
            vis_name = visual.get("name", "visual")
            pose_el = visual.find("pose")
            vpose = pose_el.text.strip().split() if pose_el is not None else ["0"] * 6
            origin_xyz = " ".join(vpose[:3])
            origin_rpy = " ".join(vpose[3:6]) if len(vpose) >= 6 else "0 0 0"

            # Extract color from SDF <material>
            mat_color = _extract_sdf_material_color(visual)

            mesh_el = visual.find(".//mesh")
            if mesh_el is not None:
                uri_el = mesh_el.find("uri")
                scale_el = mesh_el.find("scale")
                if uri_el is not None and uri_el.text:
                    abs_path = _resolve_model_uri(uri_el.text, model_dir, model_name)
                    scale = scale_el.text.strip() if scale_el is not None else "1 1 1"
                    # If no SDF <material>, try extracting color from DAE mesh
                    if not mat_color and abs_path.lower().endswith(".dae") and os.path.isfile(abs_path):
                        mat_color = _extract_dae_diffuse_color(abs_path)
                    lines.append(f'    <visual name="{vis_name}">')
                    lines.append(f'      <origin xyz="{origin_xyz}" rpy="{origin_rpy}"/>')
                    lines.append(f'      <geometry><mesh filename="{abs_path}" scale="{scale}"/></geometry>')
                    if mat_color:
                        lines.append(f'      <material name="{vis_name}_mat"><color rgba="{mat_color}"/></material>')
                    lines.append("    </visual>")
            else:
                # Check for primitive shapes
                geom = _sdf_geometry_to_urdf(visual.find("geometry"))
                if geom:
                    lines.append(f'    <visual name="{vis_name}">')
                    lines.append(f'      <origin xyz="{origin_xyz}" rpy="{origin_rpy}"/>')
                    lines.append(f"      {geom}")
                    if mat_color:
                        lines.append(f'      <material name="{vis_name}_mat"><color rgba="{mat_color}"/></material>')
                    lines.append("    </visual>")

        # Collisions
        for coll in link_el.findall("collision"):
            pose_el = coll.find("pose")
            cpose = pose_el.text.strip().split() if pose_el is not None else ["0"] * 6
            origin_xyz = " ".join(cpose[:3])
            origin_rpy = " ".join(cpose[3:6]) if len(cpose) >= 6 else "0 0 0"

            geom = _sdf_geometry_to_urdf(coll.find("geometry"), model_dir, model_name)
            if geom:
                lines.append("    <collision>")
                lines.append(f'      <origin xyz="{origin_xyz}" rpy="{origin_rpy}"/>')
                lines.append(f"      {geom}")
                lines.append("    </collision>")

        lines.append("  </link>")

    # Joints
    for joint_el in joints:
        jname = joint_el.get("name", "joint")
        jtype = joint_el.get("type", "fixed")
        # SDF "revolute" maps to URDF "continuous" (unlimited) or "revolute" (limited)
        urdf_type = jtype
        if jtype == "revolute":
            limit_el = joint_el.find(".//limit")
            if limit_el is not None:
                lower = limit_el.find("lower")
                upper = limit_el.find("upper")
                if lower is not None and upper is not None:
                    lo = float(lower.text.strip())
                    hi = float(upper.text.strip())
                    if abs(lo) > 1e10 or abs(hi) > 1e10:
                        urdf_type = "continuous"

        child = joint_el.find("child")
        parent = joint_el.find("parent")
        if child is None or parent is None:
            continue
        child_name = child.text.strip()
        parent_name = parent.text.strip()

        # Joint pose (child frame relative to parent)
        # In SDF, the link itself has a <pose> that defines its frame.
        # For URDF, the joint <origin> is the child link's pose relative to parent.
        child_link = links.get(child_name)
        if child_link is not None:
            child_pose_el = child_link.find("pose")
            cpose = child_pose_el.text.strip().split() if child_pose_el is not None else ["0"] * 6
        else:
            cpose = ["0"] * 6
        origin_xyz = " ".join(cpose[:3])
        origin_rpy = " ".join(cpose[3:6]) if len(cpose) >= 6 else "0 0 0"

        axis_el = joint_el.find(".//axis/xyz")
        axis = axis_el.text.strip() if axis_el is not None else "0 0 1"

        lines.append(f'  <joint name="{jname}" type="{urdf_type}">')
        lines.append(f'    <parent link="{parent_name}"/>')
        lines.append(f'    <child link="{child_name}"/>')
        lines.append(f'    <origin xyz="{origin_xyz}" rpy="{origin_rpy}"/>')
        lines.append(f'    <axis xyz="{axis}"/>')

        # Add dynamics (PyBullet requires damping/friction for revolute/continuous)
        lines.append('    <dynamics damping="0.0" friction="0.0"/>')

        # Limits for revolute joints
        if urdf_type == "revolute":
            limit_el = joint_el.find(".//limit")
            if limit_el is not None:
                lower = limit_el.find("lower")
                upper = limit_el.find("upper")
                lo = lower.text.strip() if lower is not None else "-3.14159"
                hi = upper.text.strip() if upper is not None else "3.14159"
                lines.append(f'    <limit lower="{lo}" upper="{hi}" effort="100" velocity="10"/>')

        lines.append("  </joint>")

    lines.append("</robot>")

    # Write to temp file
    urdf_content = "\n".join(lines)
    fd, tmp_path = tempfile.mkstemp(suffix=".urdf", prefix=f"sdf_{model_name}_")
    with os.fdopen(fd, "w") as f:
        f.write(urdf_content)

    logger.info("Converted SDF %s → temp URDF %s (%d links, %d joints)", sdf_path, tmp_path, len(links), len(joints))
    return tmp_path


def _extract_sdf_material_color(visual_el: ET.Element) -> Optional[str]:
    """Extract RGBA color string from SDF ``<material>`` inside a ``<visual>``.

    Looks for ``<diffuse>``, ``<ambient>``, or ``<emissive>`` (in that order).
    Returns ``"r g b a"`` string suitable for URDF ``<color rgba="..."/>``
    or *None* if no color information is found.
    """
    mat = visual_el.find("material")
    if mat is None:
        return None

    for tag in ("diffuse", "ambient", "emissive"):
        el = mat.find(tag)
        if el is not None and el.text:
            parts = el.text.strip().split()
            if len(parts) >= 3:
                r, g, b = parts[0], parts[1], parts[2]
                a = parts[3] if len(parts) >= 4 else "1"
                return f"{r} {g} {b} {a}"

    # Gazebo classic <script><name> (e.g. "Gazebo/Grey") — no direct RGBA
    return None


def _extract_dae_diffuse_color(dae_path: str) -> Optional[str]:
    """Extract the first diffuse RGBA color from a COLLADA (``.dae``) file.

    PyBullet sometimes ignores colours embedded in DAE meshes when loading
    via URDF.  This helper reads the ``<diffuse><color>`` from the first
    ``<phong>``/``<lambert>`` effect and returns an ``"r g b a"`` string
    that can be injected into a URDF ``<material><color rgba="..."/>``.

    Returns *None* if the file cannot be parsed or contains no colour data.
    """
    try:
        tree = ET.parse(dae_path)
    except Exception:
        return None

    root = tree.getroot()
    # COLLADA uses a namespace — strip it for simpler searches
    ns = ""
    if root.tag.startswith("{"):
        ns = root.tag.split("}")[0] + "}"

    # Search for <diffuse><color> inside <phong> or <lambert>
    for technique in root.iter(f"{ns}technique"):
        for shader in ("phong", "lambert", "blinn"):
            shader_el = technique.find(f"{ns}{shader}")
            if shader_el is None:
                continue
            diffuse_el = shader_el.find(f"{ns}diffuse")
            if diffuse_el is None:
                continue
            color_el = diffuse_el.find(f"{ns}color")
            if color_el is not None and color_el.text:
                parts = color_el.text.strip().split()
                if len(parts) >= 3:
                    r, g, b = parts[0], parts[1], parts[2]
                    a = parts[3] if len(parts) >= 4 else "1"
                    return f"{r} {g} {b} {a}"
    return None


def _sdf_geometry_to_urdf(
    geom_el: Optional[ET.Element],
    model_dir: str = "",
    model_name: str = "",
) -> str:
    """Convert an SDF ``<geometry>`` element to a URDF geometry string."""
    if geom_el is None:
        return ""

    box = geom_el.find("box/size")
    if box is not None:
        return f'<geometry><box size="{box.text.strip()}"/></geometry>'

    sphere = geom_el.find("sphere/radius")
    if sphere is not None:
        return f'<geometry><sphere radius="{sphere.text.strip()}"/></geometry>'

    cylinder = geom_el.find("cylinder")
    if cylinder is not None:
        r = cylinder.find("radius")
        length = cylinder.find("length")
        if r is not None and length is not None:
            return f'<geometry><cylinder radius="{r.text.strip()}" length="{length.text.strip()}"/></geometry>'

    mesh = geom_el.find("mesh")
    if mesh is not None:
        uri_el = mesh.find("uri")
        if uri_el is not None and uri_el.text:
            abs_path = _resolve_model_uri(uri_el.text, model_dir, model_name)
            scale_el = mesh.find("scale")
            scale = scale_el.text.strip() if scale_el is not None else "1 1 1"
            return f'<geometry><mesh filename="{abs_path}" scale="{scale}"/></geometry>'

    return ""
