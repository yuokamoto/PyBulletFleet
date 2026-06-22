# SDF Loader Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Add `SimObject.from_sdf()` factory method returning `List[SimObject]`, `load_rmf_world()` OBJ directory loader, and `resolve_model()` alias for URDF/SDF dual resolution.

**Architecture:** `from_sdf()` wraps `p.loadSDF()` and creates a SimObject per body_id via existing `__init__(body_id=...)`. `load_rmf_world()` uses existing `SimObject.from_mesh()` for OBJ files. `resolve_model()` is a thin alias for `resolve_model()`. All objects auto-register via `sim_core.add_object()`.

**Tech Stack:** Python stdlib (glob, os, pathlib). No new dependencies. PyBullet's `p.loadSDF()`.

**Depends on:** EventBus (PR 1) — `object_spawned` event fires on `add_object()`.

---

### Task 1: Tests for `resolve_model()` (SERIAL)

**Files:**
- Modify: `tests/test_robot_models.py` — add resolve_model tests

**Step 1: Write failing test**

Add to `tests/test_robot_models.py`:

```python
import os

from pybullet_fleet.robot_models import resolve_model


class TestResolveModel:
    def test_resolve_model_sdf(self):
        """resolve_model resolves SDF model names."""
        path = resolve_model("kiva_shelf")
        assert path.endswith(".sdf")
        assert os.path.exists(path)

    def test_resolve_model_urdf(self):
        """resolve_model resolves URDF model names."""
        path = resolve_model("panda")
        assert path.endswith(".urdf")

    def test_resolve_model_alias_of_resolve_model(self):
        """resolve_model returns same result as resolve_model."""
        from pybullet_fleet.robot_models import resolve_model

        assert resolve_model("kiva_shelf") == resolve_model("kiva_shelf")
```

**Step 2: Run test to verify it fails**

```bash
pytest tests/test_robot_models.py::TestResolveModel -v
```
Expected: FAIL — `ImportError: cannot import name 'resolve_model'`

**Step 3: Commit test**

```bash
git add tests/test_robot_models.py
git commit -m "test: add resolve_model tests (red)"
```

---

### Task 2: Implement `resolve_model()` (SERIAL, depends on Task 1)

**Files:**
- Modify: `pybullet_fleet/robot_models.py` — add `resolve_model` function after `resolve_model`
- Modify: `pybullet_fleet/__init__.py` — add export

**Step 1: Add resolve_model alias**

In `pybullet_fleet/robot_models.py`, right after the `resolve_model()` function (after its closing line, around line 290):

```python
def resolve_model(name_or_path: str) -> str:
    """Resolve a model name or path to an absolute file path.

    Supports both URDF and SDF files. Alias for :func:`resolve_model` with
    extended SDF awareness.

    Args:
        name_or_path: Model name (``"kiva_shelf"``) or path (``"robots/arm.urdf"``)

    Returns:
        Absolute path to the model file

    Raises:
        FileNotFoundError: If the model cannot be resolved.
    """
    return resolve_model(name_or_path)
```

**Step 2: Add export in `__init__.py`**

Add import:
```python
from pybullet_fleet.robot_models import resolve_model
```

Add to `__all__`:
```python
    "resolve_model",
```

**Step 3: Run tests**

```bash
pytest tests/test_robot_models.py::TestResolveModel -v
```
Expected: ALL PASS

**Step 4: Commit**

```bash
git add pybullet_fleet/robot_models.py pybullet_fleet/__init__.py
git commit -m "feat(sdf): add resolve_model() alias for URDF/SDF resolution"
```

---

### Task 3: Tests for `SimObject.from_sdf()` (SERIAL)

**Files:**
- Create: `tests/test_sdf_loader.py`

**Step 1: Write failing tests**

```python
# tests/test_sdf_loader.py
"""Tests for SDF loading and world loading utilities."""

import os

import pybullet as p
import pybullet_data
import pytest

from pybullet_fleet import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import SimObject, ShapeParams
from pybullet_fleet.types import CollisionMode


@pytest.fixture
def sim_core():
    """Headless sim_core for SDF tests."""
    sim = MultiRobotSimulationCore(SimulationParams(gui=False, monitor=False))
    sim.initialize_simulation()
    yield sim
    try:
        p.disconnect(sim.client)
    except p.error:
        pass


class TestFromSdf:
    """Tests for SimObject.from_sdf()."""

    def test_load_kiva_shelf(self, sim_core):
        """Load pybullet_data kiva_shelf SDF."""
        objects = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core)
        assert len(objects) >= 1
        assert all(isinstance(o, SimObject) for o in objects)
        assert all(o.object_id >= 0 for o in objects)

    def test_returns_list(self, sim_core):
        """Return type is always List[SimObject]."""
        result = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core)
        assert isinstance(result, list)

    def test_objects_registered_in_sim_core(self, sim_core):
        """All loaded objects are registered in sim_core."""
        objects = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core)
        for obj in objects:
            assert obj in sim_core.sim_objects

    def test_objects_have_names(self, sim_core):
        """Each loaded object has a name."""
        objects = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core)
        for obj in objects:
            assert obj.name is not None
            assert len(obj.name) > 0

    def test_global_scaling(self, sim_core):
        """global_scaling parameter is accepted."""
        objects = SimObject.from_sdf(
            "kiva_shelf/model.sdf", sim_core=sim_core, global_scaling=2.0
        )
        assert len(objects) >= 1

    def test_collision_mode(self, sim_core):
        """collision_mode is applied to all loaded objects."""
        objects = SimObject.from_sdf(
            "kiva_shelf/model.sdf",
            sim_core=sim_core,
            collision_mode=CollisionMode.DISABLED,
        )
        for obj in objects:
            assert obj.collision_mode == CollisionMode.DISABLED

    def test_name_prefix(self, sim_core):
        """name_prefix overrides default naming."""
        objects = SimObject.from_sdf(
            "kiva_shelf/model.sdf", sim_core=sim_core, name_prefix="shelf"
        )
        # Names should use the model name from SDF or the prefix
        for obj in objects:
            assert obj.name is not None

    def test_use_fixed_base_static(self, sim_core):
        """use_fixed_base=True makes objects static (mass=0)."""
        objects = SimObject.from_sdf(
            "kiva_shelf/model.sdf", sim_core=sim_core, use_fixed_base=True
        )
        for obj in objects:
            assert obj.mass == 0.0

    def test_invalid_path_raises(self, sim_core):
        """Invalid SDF path raises error."""
        with pytest.raises((FileNotFoundError, RuntimeError)):
            SimObject.from_sdf("nonexistent_model.sdf", sim_core=sim_core)

    def test_without_sim_core(self):
        """from_sdf works without sim_core (standalone physics client)."""
        cid = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=cid)
        try:
            objects = SimObject.from_sdf("kiva_shelf/model.sdf")
            assert len(objects) >= 1
        finally:
            p.disconnect(cid)

    def test_object_spawned_event(self, sim_core):
        """object_spawned event fires for each SDF body (EventBus dependency)."""
        spawned = []
        sim_core.events.on("object_spawned", lambda obj: spawned.append(obj))
        objects = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core)
        assert len(spawned) == len(objects)
```

**Step 2: Run test to verify it fails**

```bash
pytest tests/test_sdf_loader.py::TestFromSdf -v
```
Expected: FAIL — `AttributeError: type object 'SimObject' has no attribute 'from_sdf'`

**Step 3: Commit test**

```bash
git add tests/test_sdf_loader.py
git commit -m "test: add SimObject.from_sdf() tests (red)"
```

---

### Task 4: Implement `SimObject.from_sdf()` (SERIAL, depends on Task 3)

**Files:**
- Modify: `pybullet_fleet/sim_object.py` — add `from_sdf()` classmethod after `from_mesh()`

**Step 1: Add from_sdf classmethod**

In `pybullet_fleet/sim_object.py`, add after `from_mesh()` method (around line 660, before `from_params()`):

```python
    @classmethod
    def from_sdf(
        cls,
        sdf_path: str,
        sim_core=None,
        collision_mode: CollisionMode = CollisionMode.NORMAL_3D,
        global_scaling: float = 1.0,
        name_prefix: Optional[str] = None,
        pickable: bool = False,
        use_fixed_base: bool = True,
    ) -> List["SimObject"]:
        """Load an SDF file and wrap each body in a SimObject.

        Uses PyBullet's ``p.loadSDF()`` to load the file and creates a SimObject
        for each body. All objects are auto-registered to ``sim_core`` if provided.

        Note: PyBullet's SDF loader does NOT support ``<world>`` tags,
        ``<include>`` tags, or ``model://`` URIs. Use for individual SDF models
        (e.g., pybullet_data's kiva_shelf, wsg50_gripper).

        For rmf_demos environments (which use ``<world>`` + ``<include>``),
        use :func:`pybullet_fleet.world_loader.load_rmf_world` instead.

        Args:
            sdf_path: Path to SDF file (resolved via resolve_model() if relative)
            sim_core: Simulation core for registration
            collision_mode: Collision detection mode for all loaded objects
            global_scaling: Uniform scale factor
            name_prefix: Prefix for auto-generated names (default: SDF filename stem)
            pickable: Whether objects can be picked up
            use_fixed_base: If True, fix base position (mass=0, kinematic)

        Returns:
            List of SimObject instances (one per body in the SDF)

        Example::

            shelves = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim)
            for shelf in shelves:
                print(f"{shelf.name}: {shelf.get_pose()}")
        """
        from pathlib import Path as PathLib

        from pybullet_fleet.robot_models import resolve_model

        resolved_path = resolve_model(sdf_path)
        pid = sim_core._client if sim_core is not None else 0

        body_ids = p.loadSDF(
            resolved_path,
            globalScaling=global_scaling,
            physicsClientId=pid,
        )

        if not body_ids:
            raise RuntimeError(f"SDF loaded 0 bodies from: {resolved_path}")

        # Determine name prefix from filename if not provided
        if name_prefix is None:
            name_prefix = PathLib(resolved_path).stem

        objects = []
        for i, bid in enumerate(body_ids):
            # Try to get model name from PyBullet
            body_info = p.getBodyInfo(bid, physicsClientId=pid)
            body_name = body_info[1].decode("utf-8") if body_info[1] else f"{name_prefix}_{i}"

            if use_fixed_base:
                # Set mass to 0 for static/kinematic behavior
                p.changeDynamics(bid, -1, mass=0.0, physicsClientId=pid)

            obj = cls(
                body_id=bid,
                sim_core=sim_core,
                pickable=pickable,
                mass=0.0 if use_fixed_base else None,
                collision_mode=collision_mode,
                name=body_name,
            )
            objects.append(obj)

        lazy_logger.info(f"Loaded {len(objects)} objects from SDF: {resolved_path}")
        return objects
```

Also add `List` to the typing imports at the top of sim_object.py if not already present.

**Step 2: Run tests**

```bash
pytest tests/test_sdf_loader.py::TestFromSdf -v
```
Expected: ALL PASS

**Step 3: Run all tests**

```bash
pytest tests/ -x -q
```
Expected: ALL PASS

**Step 4: Commit**

```bash
git add pybullet_fleet/sim_object.py
git commit -m "feat(sdf): add SimObject.from_sdf() returning List[SimObject]"
```

---

### Task 5: Tests for `load_rmf_world()` (SERIAL)

**Files:**
- Modify: `tests/test_sdf_loader.py` — add load_rmf_world tests

**Step 1: Write failing tests**

Append to `tests/test_sdf_loader.py`:

```python
from pybullet_fleet.world_loader import load_rmf_world


class TestLoadRmfWorld:
    """Tests for load_rmf_world() OBJ directory loader."""

    def test_loads_obj_files(self, sim_core, tmp_path):
        """Loads OBJ files from directory."""
        obj_content = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
        (tmp_path / "wall.obj").write_text(obj_content)
        (tmp_path / "floor.obj").write_text(obj_content)

        objects = load_rmf_world(str(tmp_path), sim_core=sim_core)
        assert len(objects) == 2
        assert all(isinstance(o, SimObject) for o in objects)

    def test_objects_are_static(self, sim_core, tmp_path):
        """All loaded objects are static (mass=0)."""
        obj_content = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
        (tmp_path / "wall.obj").write_text(obj_content)

        objects = load_rmf_world(str(tmp_path), sim_core=sim_core)
        for obj in objects:
            assert obj.mass == 0.0

    def test_returns_list(self, sim_core, tmp_path):
        """Return type is always List[SimObject]."""
        obj_content = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
        (tmp_path / "mesh.obj").write_text(obj_content)

        result = load_rmf_world(str(tmp_path), sim_core=sim_core)
        assert isinstance(result, list)

    def test_empty_dir_returns_empty_list(self, sim_core, tmp_path):
        """Empty directory returns empty list."""
        objects = load_rmf_world(str(tmp_path), sim_core=sim_core)
        assert objects == []

    def test_nonexistent_dir_raises(self, sim_core):
        """Non-existent directory raises FileNotFoundError."""
        with pytest.raises(FileNotFoundError):
            load_rmf_world("/nonexistent/path", sim_core=sim_core)

    def test_names_from_filenames(self, sim_core, tmp_path):
        """Object names come from OBJ filenames."""
        obj_content = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
        (tmp_path / "L1_walls.obj").write_text(obj_content)

        objects = load_rmf_world(str(tmp_path), sim_core=sim_core)
        assert objects[0].name == "L1_walls"

    def test_objects_registered_in_sim_core(self, sim_core, tmp_path):
        """Loaded objects are registered in sim_core."""
        obj_content = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
        (tmp_path / "wall.obj").write_text(obj_content)

        objects = load_rmf_world(str(tmp_path), sim_core=sim_core)
        for obj in objects:
            assert obj in sim_core.sim_objects

    def test_collision_mode_parameter(self, sim_core, tmp_path):
        """collision_mode is applied to loaded objects."""
        obj_content = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
        (tmp_path / "wall.obj").write_text(obj_content)

        objects = load_rmf_world(
            str(tmp_path), sim_core=sim_core, collision_mode=CollisionMode.DISABLED
        )
        for obj in objects:
            assert obj.collision_mode == CollisionMode.DISABLED

    def test_custom_pattern(self, sim_core, tmp_path):
        """Custom glob pattern works."""
        (tmp_path / "mesh.obj").write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
        (tmp_path / "mesh.stl").write_bytes(b"")  # Ignored

        objects = load_rmf_world(str(tmp_path), sim_core=sim_core, pattern="*.obj")
        assert len(objects) == 1

    def test_without_sim_core(self, tmp_path):
        """Works without sim_core (standalone)."""
        cid = p.connect(p.DIRECT)
        try:
            obj_content = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
            (tmp_path / "wall.obj").write_text(obj_content)
            objects = load_rmf_world(str(tmp_path))
            assert len(objects) == 1
        finally:
            p.disconnect(cid)
```

**Step 2: Run test to verify it fails**

```bash
pytest tests/test_sdf_loader.py::TestLoadRmfWorld -v
```
Expected: FAIL — `ModuleNotFoundError: No module named 'pybullet_fleet.world_loader'`

**Step 3: Commit test**

```bash
git add tests/test_sdf_loader.py
git commit -m "test: add load_rmf_world tests (red)"
```

---

### Task 6: Implement `load_rmf_world()` (SERIAL, depends on Task 5)

**Files:**
- Create: `pybullet_fleet/world_loader.py`
- Modify: `pybullet_fleet/__init__.py` — add export

**Step 1: Create world_loader.py**

```python
# pybullet_fleet/world_loader.py
"""Environment loading utilities for simulation worlds.

Provides helpers to load pre-built environment assets (OBJ meshes, SDF models)
into a PyBulletFleet simulation. Primary use case: loading rmf_demos environments
generated by rmf_building_map_tools.
"""

import glob
import os
from pathlib import Path
from typing import List, Optional

from pybullet_fleet.geometry import Pose
from pybullet_fleet.logging_utils import get_lazy_logger
from pybullet_fleet.sim_object import SimObject, ShapeParams
from pybullet_fleet.types import CollisionMode

logger = get_lazy_logger(__name__)


def load_rmf_world(
    mesh_dir: str,
    sim_core=None,
    collision_mode: CollisionMode = CollisionMode.NORMAL_3D,
    mesh_scale: Optional[List[float]] = None,
    rgba_color: Optional[List[float]] = None,
    pattern: str = "*.obj",
) -> List[SimObject]:
    """Load OBJ mesh files from a directory as static SimObjects.

    Designed for rmf_demos environments where ``rmf_building_map_tools``
    generates OBJ meshes from building.yaml floor plans.

    ``rmf_building_map_tools`` bakes world coordinates into OBJ vertex data,
    so all objects are placed at the origin — no per-mesh pose offset is needed.

    Workflow::

        # 1. Generate meshes from rmf_demos building map
        ros2 run rmf_building_map_tools building_map_generator \\
            office.building.yaml output_dir/

        # 2. Load into PyBulletFleet
        objects = load_rmf_world("output_dir/meshes/", sim_core=sim)

    Args:
        mesh_dir: Directory containing OBJ mesh files
        sim_core: Simulation core for registration
        collision_mode: Collision mode for environment objects
        mesh_scale: Optional scale override [sx, sy, sz] (default: [1,1,1])
        rgba_color: Optional color override [r,g,b,a] (default: per-mesh)
        pattern: Glob pattern for mesh files (default: ``"*.obj"``)

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
        logger.warning(lambda: f"No {pattern} files found in {mesh_dir}")
        return []

    scale = mesh_scale or [1.0, 1.0, 1.0]
    objects: List[SimObject] = []

    for mesh_path in mesh_files:
        name = Path(mesh_path).stem

        visual = ShapeParams(
            shape_type="mesh",
            mesh_path=mesh_path,
            mesh_scale=scale,
            rgba_color=rgba_color or [0.7, 0.7, 0.7, 1.0],
        )
        collision = ShapeParams(
            shape_type="mesh",
            mesh_path=mesh_path,
            mesh_scale=scale,
        )

        # Origin placement: rmf_building_map_tools bakes world coordinates
        # into OBJ vertex data, so no per-mesh pose offset is needed.
        obj = SimObject.from_mesh(
            visual_shape=visual,
            collision_shape=collision,
            pose=Pose.from_xyz(0, 0, 0),
            mass=0.0,  # static
            sim_core=sim_core,
            collision_mode=collision_mode,
            name=name,
        )
        objects.append(obj)

    logger.info(lambda: f"Loaded {len(objects)} environment objects from {mesh_dir}")
    return objects
```

**Step 2: Add export in `__init__.py`**

Add import:
```python
from pybullet_fleet.world_loader import load_rmf_world
```

Add to `__all__`:
```python
    "load_rmf_world",
```

**Step 3: Run tests**

```bash
pytest tests/test_sdf_loader.py -v
```
Expected: ALL PASS

**Step 4: Run all tests**

```bash
pytest tests/ -x -q
```
Expected: ALL PASS

**Step 5: Commit**

```bash
git add pybullet_fleet/world_loader.py pybullet_fleet/__init__.py tests/test_sdf_loader.py
git commit -m "feat(sdf): add load_rmf_world() OBJ directory loader"
```

---

### Task 7: Example script (SERIAL, depends on Task 4 + Task 6)

**Files:**
- Create: `examples/basics/sdf_demo.py`

**Step 1: Write demo script**

```python
#!/usr/bin/env python3
"""SDF loading demo — load pybullet_data SDF models.

Demonstrates:
- SimObject.from_sdf() for individual SDF models
- resolve_model() for URDF/SDF name resolution
- load_rmf_world() for OBJ mesh directory loading (if available)

Usage:
    python examples/basics/sdf_demo.py
"""

from pybullet_fleet import (
    MultiRobotSimulationCore,
    SimulationParams,
    SimObject,
    Pose,
    Agent,
    AgentSpawnParams,
    load_rmf_world,
)
from pybullet_fleet.robot_models import resolve_model
from pybullet_fleet.types import MotionMode, CollisionMode


def main():
    sim = MultiRobotSimulationCore(
        SimulationParams(gui=True, physics=False, monitor=True, duration=30.0)
    )
    sim.initialize_simulation()

    # Load SDF models from pybullet_data
    shelves = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim)
    print(f"Loaded {len(shelves)} objects from kiva_shelf SDF")

    # Position the shelf
    for shelf in shelves:
        shelf.set_pose(Pose.from_xyz(3.0, 0.0, 0.0))

    # Load a second shelf with scaling
    shelves_small = SimObject.from_sdf(
        "kiva_shelf/model.sdf",
        sim_core=sim,
        global_scaling=0.5,
        name_prefix="small_shelf",
    )
    for shelf in shelves_small:
        shelf.set_pose(Pose.from_xyz(-3.0, 0.0, 0.0))

    # Spawn a mobile robot to navigate between shelves
    agent = Agent.from_params(
        AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0.1),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
            max_linear_vel=2.0,
            collision_mode=CollisionMode.NORMAL_2D,
        ),
        sim_core=sim,
    )

    # Demonstrate resolve_model
    resolved = resolve_model("kiva_shelf")
    print(f"resolve_model('kiva_shelf') → {resolved}")

    # Run simulation
    sim.run_simulation()


if __name__ == "__main__":
    main()
```

**Step 2: Verify it runs (quick test)**

```bash
python -c "
from pybullet_fleet import SimObject, MultiRobotSimulationCore, SimulationParams
import pybullet as p
sim = MultiRobotSimulationCore(SimulationParams(gui=False, monitor=False))
sim.initialize_simulation()
objs = SimObject.from_sdf('kiva_shelf/model.sdf', sim_core=sim)
print(f'Loaded {len(objs)} objects')
p.disconnect(sim.client)
"
```

**Step 3: Commit**

```bash
git add examples/basics/sdf_demo.py
git commit -m "feat(sdf): add SDF loading demo example"
```

---

### Task 8: Final verification (SERIAL, depends on all)

**Step 1: Run full test suite**

```bash
make verify
```
Expected: ALL PASS, lint clean

**Step 2: Check coverage**

```bash
make test
```
Expected: Coverage ≥ 75%

**Step 3: Final commit (if any formatting fixes)**

```bash
make format
git add -A
git commit -m "style: format SDF loader code"
```
