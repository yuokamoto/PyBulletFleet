"""Tests for Door device."""

import pytest
from pybullet_fleet import Agent, AgentSpawnParams, Pose
from pybullet_fleet.types import DoorState, MotionMode
from pybullet_fleet.devices.door import Door, DoorParams


@pytest.fixture
def sim_core(pybullet_env):
    from tests.conftest import MockSimCore

    sc = MockSimCore()
    sc._client = pybullet_env
    return sc


class TestDoor:
    def test_door_initial_state_closed(self, sim_core):
        """Door starts in 'closed' state."""
        params = DoorParams(
            urdf_path="robots/door_hinge.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            use_fixed_base=True,
            open_positions={"hinge": 1.57},
            close_positions={"hinge": 0.0},
        )
        door = Door.from_params(params, sim_core)
        assert door.door_state == DoorState.CLOSED

    def test_door_open_close_cycle(self, sim_core):
        """Door: request_open -> open, request_close -> closed."""
        params = DoorParams(
            urdf_path="robots/door_hinge.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            use_fixed_base=True,
            open_positions={"hinge": 1.57},
            close_positions={"hinge": 0.0},
        )
        door = Door.from_params(params, sim_core)
        door.request_open()
        sim_core.tick(200)
        assert door.door_state == DoorState.OPEN

        door.request_close()
        sim_core.tick(200)
        assert door.door_state == DoorState.CLOSED

    def test_door_is_agent_subclass(self, sim_core):
        """Door inherits from Agent."""
        params = DoorParams(
            urdf_path="robots/door_hinge.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            use_fixed_base=True,
            open_positions={"hinge": 1.57},
            close_positions={"hinge": 0.0},
        )
        door = Door.from_params(params, sim_core)
        assert isinstance(door, Agent)
        assert isinstance(door, Door)

    def test_slide_door(self, sim_core):
        """Door works with prismatic slide joints."""
        params = DoorParams(
            urdf_path="robots/door_slide.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            use_fixed_base=True,
            open_positions={"slide": 1.0},
            close_positions={"slide": 0.0},
        )
        door = Door.from_params(params, sim_core)
        assert door.door_state == "closed"
        door.request_open()
        sim_core.tick(200)
        assert door.door_state == "open"

    def test_door_from_dict(self, sim_core):
        """Door.from_dict creates Door with DoorParams automatically."""
        config = {
            "name": "test_door",
            "urdf_path": "robots/door_hinge.urdf",
            "pose": [0, 0, 0],
            "use_fixed_base": True,
            "open_positions": {"hinge": 1.57},
            "close_positions": {"hinge": 0.0},
        }
        door = Door.from_dict(config, sim_core)
        assert isinstance(door, Door)
        assert door.door_state == "closed"


class TestDoorParams:
    """Tests for DoorParams dataclass validation."""

    def test_direct_construction(self):
        """DoorParams direct construction with typed fields."""
        params = DoorParams(
            urdf_path="robots/door_hinge.urdf",
            open_positions={"hinge": 1.57},
            close_positions={"hinge": 0.0},
        )
        assert params.open_positions == {"hinge": 1.57}
        assert params.close_positions == {"hinge": 0.0}
        assert isinstance(params, AgentSpawnParams)

    def test_minimal_construction(self):
        """DoorParams with only open_positions (close defaults to {})."""
        params = DoorParams(
            urdf_path="robots/door_hinge.urdf",
            open_positions={"hinge": 1.57},
        )
        assert params.open_positions == {"hinge": 1.57}
        assert params.close_positions == {}

    def test_missing_open_positions_raises(self):
        """DoorParams raises TypeError when open_positions is None."""
        with pytest.raises(TypeError, match="open_positions"):
            DoorParams(urdf_path="robots/door_hinge.urdf")

    def test_from_dict_top_level(self):
        """DoorParams.from_dict reads door fields at top level."""
        params = DoorParams.from_dict(
            {
                "name": "test_door",
                "urdf_path": "robots/door_hinge.urdf",
                "open_positions": {"hinge": 1.57},
                "close_positions": {"hinge": 0.0},
            }
        )
        assert params.open_positions == {"hinge": 1.57}
        assert params.close_positions == {"hinge": 0.0}

    def test_door_from_params_wrong_type(self, sim_core):
        """Door.from_params raises TypeError when given AgentSpawnParams."""
        params = AgentSpawnParams(
            urdf_path="robots/door_hinge.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            use_fixed_base=True,
        )
        with pytest.raises(TypeError, match="DoorParams"):
            Door.from_params(params, sim_core)
