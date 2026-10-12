"""Fresh-process coverage for built-in physics-off primitive objects."""

from __future__ import annotations

import copy
import json
import subprocess
import sys
from pathlib import Path

import pybullet as p
import pytest

from pybullet_fleet.replay.kinematic_profile import KinematicProfile
from pybullet_fleet.replay.state_recording import load_recording_manifest


def _run(script: str, *args: str) -> dict | None:
    result = subprocess.run(
        [sys.executable, "-c", script, *args],
        capture_output=True,
        text=True,
        check=True,
        timeout=30,
    )
    marked = [line.removeprefix("RESULT:") for line in result.stdout.splitlines() if line.startswith("RESULT:")]
    return json.loads(marked[-1]) if marked else None


@pytest.fixture
def primitive_artifact(tmp_path: Path) -> Path:
    directory = tmp_path / "primitives"
    _run(
        """import sys
from pathlib import Path
from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Pose
from pybullet_fleet.replay.kinematic_profile import KinematicProfile
from pybullet_fleet.sim_object import ShapeParams, SimObject, SimObjectSpawnParams
s = MultiRobotSimulationCore(SimulationParams(gui=False, physics=False, timestep=0.1,
    collision_check_frequency=0, target_rtf=0))
agent = Agent.from_params(AgentSpawnParams(name='robot',
    urdf_path=str(Path('pybullet_fleet/robots/mobile_robot.urdf').resolve()),
    initial_pose=Pose.from_xyz(-2, 0, 0.05), mass=0), s)
objects = {
    'sphere': (ShapeParams(shape_type='sphere', radius=0.2, rgba_color=[1, 0, 0, 1],
                           frame_pose=Pose.from_xyz(0.01, 0.02, 0.03)),
               ShapeParams(shape_type='sphere', radius=0.18,
                           frame_pose=Pose.from_xyz(-0.01, 0, 0))),
    'cylinder': (ShapeParams(shape_type='cylinder', radius=0.15, height=0.4,
                             frame_pose=Pose.from_yaw(0, 0, 0.03, 0.4)),
                 ShapeParams(shape_type='cylinder', radius=0.13, height=0.35)),
    'mixed': (ShapeParams(shape_type='box', half_extents=[0.2, 0.1, 0.3]),
              ShapeParams(shape_type='sphere', radius=0.12)),
    'visual-only': (ShapeParams(shape_type='sphere', radius=0.1), None),
}
for index, (name, (visual, collision)) in enumerate(objects.items()):
    obj = SimObject.from_params(SimObjectSpawnParams(name=name, mass=0,
        initial_pose=Pose.from_xyz(index, 1, 0.5),
        visual_shape=visual, collision_shape=collision), s)
    if name == 'sphere':
        assert agent.attach_object(obj, parent_link_index='base_link',
            relative_pose=Pose.from_xyz(0.2, 0, 0.1))
s.configure_state_recording(profile=KinematicProfile(), output=sys.argv[1])
s.run_simulation(duration=0.2)
""",
        str(directory),
    )
    assert load_recording_manifest(directory)["complete"] is True
    return directory


def test_primitives_restore_in_fresh_process(primitive_artifact: Path) -> None:
    checkpoint = json.loads((primitive_artifact / "checkpoints" / "000000001.json").read_text())
    source_objects = checkpoint["objects"]
    assert {name: entry["type"] for name, entry in source_objects.items()} == {
        name: "kinematic_primitive" for name in ("sphere", "cylinder", "mixed", "visual-only")
    }
    assert source_objects["cylinder"]["state"]["construction"]["visual_shape"]["frame_pose"] is not None
    assert source_objects["sphere"]["state"]["attachment"]["parent_name"] == "robot"
    assert source_objects["mixed"]["state"]["construction"]["collision_shape"]["type"] == "sphere"
    assert source_objects["visual-only"]["state"]["construction"]["collision_shape"] is None
    restored = _run(
        """import json, sys, pybullet as p
from pybullet_fleet.replay import load_supported_simulation
from pybullet_fleet.replay.kinematic_profile import capture_primitive_state
s = load_supported_simulation(sys.argv[1], at_or_before=0.1)
objects = {obj.name: obj for obj in s.sim_objects if obj not in s.agents}
print('RESULT:' + json.dumps({
    'states': {name: capture_primitive_state(obj) for name, obj in objects.items()},
    'geometry': {name: {
        'visual': [shape[2] for shape in p.getVisualShapeData(obj.body_id, physicsClientId=s.client)],
        'collision': [shape[2] for shape in p.getCollisionShapeData(obj.body_id, -1, physicsClientId=s.client)],
        'visual_offsets': [list(shape[5]) for shape in p.getVisualShapeData(obj.body_id, physicsClientId=s.client)],
        'collision_offsets': [list(shape[5]) for shape in p.getCollisionShapeData(obj.body_id, -1, physicsClientId=s.client)]
    } for name, obj in objects.items()}
}))
""",
        str(primitive_artifact),
    )
    assert restored is not None
    assert restored["states"] == {name: entry["state"] for name, entry in source_objects.items()}
    geometry = restored["geometry"]
    assert geometry["sphere"]["visual"] == [p.GEOM_SPHERE]
    assert geometry["sphere"]["collision"] == [p.GEOM_SPHERE]
    assert geometry["cylinder"]["visual"] == [p.GEOM_CYLINDER]
    assert geometry["cylinder"]["collision"] == [p.GEOM_CYLINDER]
    assert geometry["mixed"]["visual"] == [p.GEOM_BOX]
    assert geometry["mixed"]["collision"] == [p.GEOM_SPHERE]
    assert geometry["sphere"]["visual_offsets"][0] == pytest.approx([0.01, 0.02, 0.03])
    assert geometry["sphere"]["collision_offsets"][0] == pytest.approx([-0.01, 0, 0])


@pytest.mark.parametrize(
    "mutation",
    [
        lambda state: state["objects"]["sphere"]["state"]["construction"]["visual_shape"].update(type="mesh"),
        lambda state: state["objects"]["cylinder"]["state"]["construction"]["collision_shape"].update(height=-1),
        lambda state: state["objects"]["mixed"]["state"]["construction"]["visual_shape"].update(radius=1),
    ],
)
def test_unsupported_or_invalid_shape_is_rejected_before_restore(primitive_artifact: Path, mutation) -> None:
    manifest = load_recording_manifest(primitive_artifact)
    state = json.loads((primitive_artifact / "checkpoints" / "000000001.json").read_text())
    state = copy.deepcopy(state)
    mutation(state)
    with pytest.raises(ValueError):
        KinematicProfile().validate_checkpoint(manifest, state)


def test_invalid_agent_angular_velocity_is_rejected(primitive_artifact: Path) -> None:
    manifest = load_recording_manifest(primitive_artifact)
    checkpoint = json.loads((primitive_artifact / "checkpoints" / "000000001.json").read_text())
    for invalid in (True, "0", None, float("nan"), float("inf")):
        state = copy.deepcopy(checkpoint)
        state["agents"]["robot"]["state"]["angular_velocity"] = invalid
        with pytest.raises(ValueError, match="Invalid Agent movement state"):
            KinematicProfile().validate_checkpoint(manifest, state)


def test_primitive_only_world_records_and_restores_without_an_agent(tmp_path: Path) -> None:
    from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
    from pybullet_fleet.geometry import Pose
    from pybullet_fleet.replay import load_supported_simulation
    from pybullet_fleet.sim_object import ShapeParams, SimObject, SimObjectSpawnParams

    path = tmp_path / "primitive-only"
    sim = MultiRobotSimulationCore(
        SimulationParams(gui=False, physics=False, collision_check_frequency=0, target_rtf=0, timestep=0.1)
    )
    try:
        SimObject.from_params(
            SimObjectSpawnParams(
                name="box",
                mass=0,
                initial_pose=Pose.from_xyz(1, 2, 0.5),
                visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            ),
            sim,
        )
        sim.configure_state_recording(profile=KinematicProfile(), output=str(path))
        sim.run_simulation(duration=0.1)
        assert load_recording_manifest(path)["construction"]["agents"] == {}
        restored = load_supported_simulation(path, at_or_before=0.1)
        try:
            assert restored.agents == []
            assert restored.get_unique_object_by_name("box").get_pose().position == pytest.approx([1, 2, 0.5])
            assert restored.get_completed_step_boundary() == (1, pytest.approx(0.1))
        finally:
            p.disconnect(restored.client)
    finally:
        if p.isConnected(sim.client):
            p.disconnect(sim.client)
