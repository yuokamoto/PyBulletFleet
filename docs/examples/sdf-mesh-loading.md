# Tutorial 7: SDF and Mesh Assets

Load pre-built SDF environments and directories of mesh assets as managed
`SimObject` instances. This is useful when a scenario starts from a warehouse,
shelf layout, or visual asset collection rather than only individually spawned
objects.

**Source file:** [`examples/models/sdf_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/pybullet_fleet/examples/models/sdf_demo.py)

## Run the demo

```bash
python examples/models/sdf_demo.py
```

The demo loads a multi-model warehouse SDF, loads scaled copies of an SDF shelf,
and bulk-loads selected OBJ meshes from a directory.

## Load an SDF asset

`SimObject.from_sdf()` turns every body returned by PyBullet's `loadSDF()` into
a managed `SimObject`. A multi-model SDF therefore returns a list rather than a
single object:

```python
from pybullet_fleet.sim_object import SimObject

warehouse = SimObject.from_sdf("mesh/warehouse_layout.sdf", sim_core=sim)
for obj in warehouse:
    print(obj.name, obj.object_id)
```

Use `global_scaling` and `name_prefix` when loading a reusable model more than
once:

```python
shelves = SimObject.from_sdf(
    "kiva_shelf/model.sdf",
    sim_core=sim,
    global_scaling=0.5,
    name_prefix="small_shelf",
)
```

## Load mesh files in bulk

Use `load_mesh_directory()` for a directory of visual or collision meshes. The
demo loads selected Panda collision meshes as visual-only objects:

```python
import pybullet_data
from pybullet_fleet import load_mesh_directory
from pybullet_fleet.types import CollisionMode

mesh_dir = pybullet_data.getDataPath() + "/franka_panda/meshes/collision"
meshes = load_mesh_directory(
    mesh_dir,
    sim_core=sim,
    pattern="link[0-2].obj",
    collision_mode=CollisionMode.DISABLED,
)
```

For ROS 2 package assets, use `package://` URIs in bridge configuration; see
[Bridge Configuration](../ros2/configuration).

## See also

- [Tutorial 1 — Spawning Objects](spawning-objects): individual mesh, URDF, and `SimObject` creation
- [Tutorial 6 — Robot Models](robot-models): resolving named URDF and SDF assets
- [Bridge Configuration](../ros2/configuration): `package://` asset paths in ROS 2 scenarios
