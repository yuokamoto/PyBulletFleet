"""Structure transparency keeps what a model authored, and works headless.

Turning transparency off used to force alpha 1.0, so a model that authored
its own translucency lost it -- and since ``transparent=False`` is the
default, simply calling ``configure_visualizer()`` repainted every static
body. The setter was also GUI-only, which left offscreen renders of a
multi-level scene with an opaque lid over everything below the top.
"""

import pybullet as p
import pytest

from pybullet_fleet import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import ShapeParams, SimObject, SimObjectSpawnParams
from pybullet_fleet.types import CollisionMode

AUTHORED_ALPHA = 0.45


@pytest.fixture
def sim_core():
    sim = MultiRobotSimulationCore(SimulationParams(gui=False, monitor=False, enable_monitor_gui=False))
    sim.initialize_simulation()
    yield sim
    try:
        p.disconnect(sim.client)
    except p.error:
        pass


@pytest.fixture
def translucent(sim_core):
    """One static body whose material asks for a translucent colour."""
    return SimObject.from_params(
        SimObjectSpawnParams(
            visual_shape=ShapeParams(
                shape_type="box",
                half_extents=[0.5, 0.5, 0.05],
                rgba_color=[0.07, 0.07, 0.07, AUTHORED_ALPHA],
            ),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.05]),
            name="deck",
            initial_pose=Pose.from_xyz(0.0, 0.0, 0.0),
            collision_mode=CollisionMode.STATIC,
        ),
        sim_core=sim_core,
    )


def _alpha(sim_core, obj):
    return p.getVisualShapeData(obj.body_id, physicsClientId=sim_core.client)[0][7][3]


class TestStructureTransparency:
    def test_it_works_without_a_gui(self, sim_core, translucent):
        """Offscreen renders honour alpha, so headless needs this too."""
        sim_core.set_structure_transparency(True)
        assert _alpha(sim_core, translucent) == pytest.approx(0.3)

    def test_turning_it_off_restores_the_authored_alpha(self, sim_core, translucent):
        sim_core.set_structure_transparency(True)
        sim_core.set_structure_transparency(False)
        assert _alpha(sim_core, translucent) == pytest.approx(AUTHORED_ALPHA)

    def test_turning_it_off_first_changes_nothing(self, sim_core, translucent):
        """transparent=False is the default, so this is what a bare
        configure_visualizer() call does to a scene."""
        sim_core.set_structure_transparency(False)
        assert _alpha(sim_core, translucent) == pytest.approx(AUTHORED_ALPHA)

    def test_an_opaque_body_stays_opaque_through_a_round_trip(self, sim_core):
        opaque = SimObject.from_params(
            SimObjectSpawnParams(
                visual_shape=ShapeParams(shape_type="box", half_extents=[0.2, 0.2, 0.2], rgba_color=[0.8, 0.12, 0.12, 1.0]),
                collision_shape=ShapeParams(shape_type="box", half_extents=[0.2, 0.2, 0.2]),
                name="post",
                initial_pose=Pose.from_xyz(2.0, 0.0, 0.0),
                collision_mode=CollisionMode.STATIC,
            ),
            sim_core=sim_core,
        )
        sim_core.set_structure_transparency(True)
        assert _alpha(sim_core, opaque) == pytest.approx(0.3)
        sim_core.set_structure_transparency(False)
        assert _alpha(sim_core, opaque) == pytest.approx(1.0)

    def test_the_flag_is_reported(self, sim_core, translucent):
        sim_core.set_structure_transparency(True)
        assert sim_core._structure_transparent is True
        sim_core.set_structure_transparency(False)
        assert sim_core._structure_transparent is False


class TestLateAddedBodies:
    """Cases raised in review: the colour cache was built once, from body ids
    assumed to be contiguous."""

    def _static_box(self, sim_core, name, x, alpha):
        return SimObject.from_params(
            SimObjectSpawnParams(
                visual_shape=ShapeParams(shape_type="box", half_extents=[0.2, 0.2, 0.2], rgba_color=[0.3, 0.3, 0.3, alpha]),
                collision_shape=ShapeParams(shape_type="box", half_extents=[0.2, 0.2, 0.2]),
                name=name,
                initial_pose=Pose.from_xyz(x, 0.0, 0.0),
                collision_mode=CollisionMode.STATIC,
            ),
            sim_core=sim_core,
        )

    def test_a_body_added_after_the_first_call_goes_transparent_too(self, sim_core, translucent):
        sim_core.set_structure_transparency(True)
        late = self._static_box(sim_core, "late", 3.0, 1.0)

        sim_core.set_structure_transparency(True)

        assert _alpha(sim_core, late) == pytest.approx(0.3)

    def test_and_its_own_alpha_is_restorable(self, sim_core, translucent):
        sim_core.set_structure_transparency(True)
        late = self._static_box(sim_core, "late", 3.0, 0.6)
        sim_core.set_structure_transparency(True)

        sim_core.set_structure_transparency(False)

        assert _alpha(sim_core, late) == pytest.approx(0.6)

    def test_repainted_colours_never_become_the_originals(self, sim_core, translucent):
        """Re-capturing must not overwrite: a body already at 0.3 would
        otherwise have that recorded as what it was authored with, and the
        real colour would be gone for the rest of the run."""
        sim_core.set_structure_transparency(True)
        self._static_box(sim_core, "late", 3.0, 1.0)
        sim_core.set_structure_transparency(True)

        sim_core.set_structure_transparency(False)

        assert _alpha(sim_core, translucent) == pytest.approx(AUTHORED_ALPHA)

    def test_body_ids_are_not_assumed_contiguous(self, sim_core, translucent):
        """Removing a body leaves ids with a hole; indexing by position would
        skip a real body and query an id that does not exist."""
        doomed = self._static_box(sim_core, "doomed", 2.0, 1.0)
        survivor = self._static_box(sim_core, "survivor", 4.0, 0.7)
        p.removeBody(doomed.body_id, physicsClientId=sim_core.client)

        sim_core.set_structure_transparency(True)
        assert _alpha(sim_core, survivor) == pytest.approx(0.3)
        sim_core.set_structure_transparency(False)
        assert _alpha(sim_core, survivor) == pytest.approx(0.7)
