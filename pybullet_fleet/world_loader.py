# pybullet_fleet/world_loader.py
"""Backward-compatibility shim — all functions moved to :mod:`sdf_loader`."""

from pybullet_fleet.sdf_loader import load_mesh_directory, load_rmf_world  # noqa: F401

__all__ = ["load_mesh_directory", "load_rmf_world"]
