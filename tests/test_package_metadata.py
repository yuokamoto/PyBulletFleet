"""Tests for package-level metadata exposed by pybullet_fleet."""

import re

import pybullet_fleet


def test_version_is_exposed():
    assert isinstance(pybullet_fleet.__version__, str)
    assert pybullet_fleet.__version__


def test_version_looks_like_semver_or_fallback():
    # Either a real installed version (e.g. "0.4.1", possibly with a local/dev
    # suffix) or the not-installed fallback.
    v = pybullet_fleet.__version__
    assert v == "0.0.0+unknown" or re.match(r"^\d+\.\d+\.\d+", v), v


def test_version_in_all():
    assert "__version__" in pybullet_fleet.__all__


def test_fleet_command_types_are_reexported():
    assert pybullet_fleet.RobotState2D.__name__ == "RobotState2D"
    assert pybullet_fleet.RobotState3D.__name__ == "RobotState3D"
    assert pybullet_fleet.RobotAttachCommand.__name__ == "RobotAttachCommand"
    assert pybullet_fleet.RobotActionCommand.__name__ == "RobotActionCommand"
    assert "RobotState2D" in pybullet_fleet.__all__
    assert "RobotState3D" in pybullet_fleet.__all__
    assert pybullet_fleet.DEFAULT_ATTACH_SEARCH_RADIUS == 0.5
    assert "RobotAttachCommand" in pybullet_fleet.__all__
    assert "RobotActionCommand" in pybullet_fleet.__all__
    assert "DEFAULT_ATTACH_SEARCH_RADIUS" in pybullet_fleet.__all__


def test_fleet_api_keeps_command_import_compatibility():
    from pybullet_fleet.commands import RobotAttachCommand as CommandsAttach
    from pybullet_fleet.fleet_api import RobotAttachCommand as FleetApiAttach

    assert FleetApiAttach is CommandsAttach


def test_fleet_api_keeps_state_import_compatibility():
    from pybullet_fleet.fleet_api import RobotState3D as FleetApiState
    from pybullet_fleet.states import RobotState3D as StatesState

    assert FleetApiState is StatesState
