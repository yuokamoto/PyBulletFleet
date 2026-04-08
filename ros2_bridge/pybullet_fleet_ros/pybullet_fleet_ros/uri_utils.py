"""URI resolution utilities for pybullet_fleet_ros.

Handles ``package://`` URI resolution via ``ament_index_python``
so that SpawnEntity requests can reference models from installed
ROS 2 packages.
"""

import os


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
