"""Canonical names and helpers for Fleet ROS API endpoints."""

from __future__ import annotations


FLEET_API_NAMESPACE = "/fleet"


def normalize_fleet_namespace(namespace: str | None) -> str:
    """Return an absolute Fleet namespace, defaulting to ``/fleet``."""
    normalized = (namespace or "").strip().rstrip("/")
    if not normalized:
        return FLEET_API_NAMESPACE
    return normalized if normalized.startswith("/") else f"/{normalized}"


def fleet_endpoint(namespace: str | None, name: str) -> str:
    """Return the endpoint named *name* below a Fleet namespace."""
    return f"{normalize_fleet_namespace(namespace)}/{name.lstrip('/')}"


def manager_fleet_namespace(manager: str) -> str:
    """Return the manager-scoped Fleet namespace for a validated manager name."""
    return fleet_endpoint(FLEET_API_NAMESPACE, manager)
