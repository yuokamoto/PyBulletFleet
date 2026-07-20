"""URI resolution utilities for pybullet_fleet_ros.

Handles ``package://`` URI resolution via ``ament_index_python``
so that SpawnEntity requests can reference models from installed
ROS 2 packages.
"""

import os
from collections.abc import Mapping
from typing import Any


def resolve_uri(uri: str) -> str:
    """Resolve a ``package://`` URI to an absolute filesystem path.

    Non-``package://`` URIs (plain paths, ``file://``, etc.) are returned
    unchanged.

    Args:
        uri: Resource URI string.  ``package://pkg_name/rel/path.urdf``
             is resolved via ``ament_index_python``.

    Returns:
        Resolved filesystem path.

    Raises:
        PackageNotFoundError: If the ROS 2 package cannot be found.
    """
    if not uri.startswith("package://"):
        return uri
    rest = uri[len("package://") :]
    pkg_name, _, rel_path = rest.partition("/")
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory(pkg_name), rel_path)


def resolve_package_uris(value: Any) -> Any:
    """Recursively resolve ``package://`` strings in a config structure."""
    if isinstance(value, str):
        return resolve_uri(value)
    if isinstance(value, list):
        return [resolve_package_uris(item) for item in value]
    if isinstance(value, tuple):
        return tuple(resolve_package_uris(item) for item in value)
    if isinstance(value, Mapping):
        return {key: resolve_package_uris(item) for key, item in value.items()}
    return value
