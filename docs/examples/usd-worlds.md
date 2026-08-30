# Tutorial 10: OpenUSD Static Worlds

**Source file:** [`examples/models/usd_warehouse_demo.py`](https://github.com/yuokamoto/PyBulletFleet/tree/main/pybullet_fleet/examples/models/usd_warehouse_demo.py)

PyBulletFleet can import a local OpenUSD scene as static visual and collision
geometry. Install the optional dependency first:

```bash
pip install 'pybullet-fleet[usd]'
python pybullet_fleet/examples/models/usd_warehouse_demo.py
```

When running from a source checkout, prefer installing that checkout so its
unreleased optional extra is used:

```bash
python -m pip install -e '.[usd]'
```

Until a release containing this feature is published, install `usd-core`
directly when using an older PyPI package.

The bundled demo opens a small Y-up warehouse scene and automatically frames it
in the PyBullet GUI. It is also safe to use in CI or on a server:

```bash
python pybullet_fleet/examples/models/usd_warehouse_demo.py --headless
```

To inspect the PyBullet GUI main light and shadow-map settings with the bundled
warehouse, use:

```bash
python pybullet_fleet/examples/models/usd_warehouse_demo.py \
  --lighting-controls --texture-brightness 1.0
```

The sliders appear in PyBullet's Parameters panel. They control PyBullet's
single GUI main light and shadow map, not USD `Light` prims. Keeping
`texture_brightness` at `1.0` makes this visual comparison independent of the
importer's texture compatibility adjustment.

The demo is limited to OpenUSD import and static-world rendering. The portable
worker behavior-tree example is intentionally separate so it can be reused
without an OpenUSD dependency.

## Isaac Sim asset-pack examples

The NVIDIA Assets Pack is not bundled with PyBulletFleet. Once you have
downloaded and extracted it, start with the smaller composed stages below. The
measurements were taken from Isaac Sim Assets 6.0 and count composed
`UsdGeom.Mesh` prims and their authored triangles; the `.usd` file size alone
does **not** include referenced meshes or textures.

| Try order | Stage below `$ISAAC_ASSETS/Isaac/Environments` | Stage size | Meshes | Triangles | Notes |
| --- | --- | ---: | ---: | ---: | --- |
| 1 | `Grid/default_environment.usd` | 17 KiB | 1 | 2 | Minimal importer smoke test |
| 2 | `Simple_Room/simple_room.usd` | 1.6 MiB | 60 | 20,148 | Recommended first rendered scene; imported as 59 mesh bodies in the current importer |
| 3 | `Simple_Warehouse/warehouse.usd` | 1.5 MiB | 781 | 470,414 | Small warehouse; may still be slow under WSL software rendering |
| 4 | `Simple_Warehouse/warehouse_with_forklifts.usd` | 17 KiB | 792 | 791,162 | Uses references; the small stage file is not a performance indicator |
| 5 | `Simple_Warehouse/warehouse_multiple_shelves.usd` | 2.2 MiB | 1,878 | 1,248,912 | Large test case |
| 6 | `Simple_Warehouse/full_warehouse.usd` | 6.5 MiB | 3,473 | 676,703 | Stress test; not recommended for WSL software rendering |

For example, with the default local extraction location:

```bash
export ISAAC_ASSETS="$HOME/isaac-assets/extracted/Assets/Isaac/6.0"

python pybullet_fleet/examples/models/usd_warehouse_demo.py \
  --usd "$ISAAC_ASSETS/Isaac/Environments/Simple_Room/simple_room.usd"
```

Use `--headless` first if the machine has no hardware-accelerated OpenGL. The
`full_warehouse.usd` scene creates thousands of separate PyBullet bodies in the
current importer and can make WSL unresponsive.

To use a locally prepared OpenUSD stage instead, pass `--usd`:

```bash
python pybullet_fleet/examples/models/usd_warehouse_demo.py \
  --usd /path/to/warehouse.usdz
```

The initial importer supports composed local mesh stages and bounded
`PointInstancer` expansion, and normalizes Y-up or Z-up scene coordinates to
PyBulletFleet's Z-up metre convention. It warns at 1,000 instances and rejects
more than 5,000 by default before creating PyBullet bodies. It does not execute
Isaac Sim physics, NavMesh, animation, sensors, or `omni://` asset resolution.
Localize/package external dependencies before importing an Isaac scene.

For visuals, the importer uses `UsdPreviewSurface` diffuse color and the common
Isaac MDL `ColorAlbedo` / `AlbedoTexture` and `diffuse_texture` inputs when a
mesh has vertex or indexed face-varying `st` / `st_0` UVs. Isaac MDL albedo
maps are combined with meaningful author tint colors; near-black placeholder
tints are ignored. Vertex normals are preserved for smooth shading. This is an
approximation for PyBullet; MDL and MaterialX lighting,
normal maps, and roughness are not reproduced.

Visual meshes are used as collision meshes only within the default budgets of
50,000 triangles per mesh and 500,000 total. Over-budget geometry stays
visible but becomes collision-free and is reported as `collision_mesh_unavailable`.
Set `collision_required=True` to reject such a stage before any body is created.

For Python use, call `load_usd_world()` and inspect the returned report:

```python
from pybullet_fleet import MultiRobotSimulationCore, SimulationParams, load_usd_world

sim = MultiRobotSimulationCore(SimulationParams(gui=True, monitor=False))
report = load_usd_world("warehouse.usdz", sim_core=sim)
print(report.created_object_ids)  # USD prim path -> PyBulletFleet object ID
sim.run_simulation()  # initializes automatically
```

The importer stores the source USD prim path in each object's `user_data` and
in the report. Do not use PyBullet body IDs as a portable scene identifier.
For rigid or uniformly scaled USD transforms, the normalized position and
orientation are also the object's PyBulletFleet `Pose`, recorded as
`usd_normalized_world_pose`. Non-uniform scale, reflection, or shear remains
correctly rendered by baking the transform into the mesh; its exact normalized
4×4 transform is recorded as `usd_normalized_world_transform` instead.
