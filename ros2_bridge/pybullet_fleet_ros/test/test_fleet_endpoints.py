"""Tests for ROS-independent Fleet endpoint naming helpers."""

from pybullet_fleet_ros.fleet_endpoints import (
    FLEET_API_NAMESPACE,
    fleet_endpoint,
    manager_fleet_namespace,
    normalize_fleet_namespace,
)


def test_fleet_namespace_helpers_normalize_and_join_endpoints():
    assert normalize_fleet_namespace(None) == FLEET_API_NAMESPACE
    assert normalize_fleet_namespace("fleet/delivery/") == "/fleet/delivery"
    assert fleet_endpoint("/fleet/delivery/", "/states") == "/fleet/delivery/states"
    assert manager_fleet_namespace("inspection") == "/fleet/inspection"
