# OpenUSD Warehouse Import and Worker Behavior Tree MVP

**Date:** 2026-08-17
**Status:** Draft
**Related:** [Roadmap](../../roadmap.md), [Snapshot / Replay](../snapshot-replay/spec.md), [USO](https://github.com/yuokamoto/Unified-Simulation-Orchestrator)

## Goal

Load a self-contained OpenUSD warehouse scene into PyBulletFleet as a static
environment, then run a small number of worker actors from a portable
behavior-tree definition. The resulting example demonstrates a single scenario
that can reuse a USD environment in Isaac Sim and PyBulletFleet while retaining
PyBulletFleet's own execution, collision, action, and state APIs.

The MVP is deliberately an **OpenUSD scene importer**, not an Isaac Sim
runtime importer. It proves scene/asset reuse without promising compatibility
with Isaac-specific physics, animation, sensor, or Omniverse services.

## Context

USO separates reusable assets, robot definitions, behavior, and runtime state:

| Concern | Shared representation | PyBulletFleet responsibility |
|---|---|---|
| Environment assets and scene layout | OpenUSD | Read supported geometry and transforms into static PyBullet objects |
| Robot structure | URDF / SDF | Existing `Agent` and device loading |
| Worker/robot decision logic | Behavior tree | Interpret a small portable tree using PyBulletFleet actions |
| Runtime synchronization and replay | Snapshot + event log | Future work; this MVP only preserves stable source metadata |

Isaac Sim provides `Simple_Warehouse/full_warehouse.usd` and worker examples
for wander, patrol, and box movement. Its current character behavior tree
support is experimental and depends on Omniverse-specific node types, NavMesh,
animation systems, and trigger schemas. Reusing the scene is useful; directly
executing Isaac's behavior-tree JSON in PyBulletFleet is not a compatibility
goal.

## Decision

Add an optional OpenUSD-based static-world import path and a separate,
PyBulletFleet-owned worker behavior-tree runner.

```
local .usd / .usda / .usdz
          |
          v
  OpenUSD composition (pxr.Usd)
          |
          +-- supported Xform / Mesh / PointInstancer --> static SimObjects
          +-- source prim paths + stage metadata        --> import report
          |
          v
  existing PyBulletFleet world, actions, EventBus
          |
          +-- WorkerBehaviorTree --> worker Agent goals / waits
          +-- Future Snapshot/EventLog --> USO state synchronization
```

The importer uses `pxr.Usd.Stage.Open()` and evaluated world transforms. It
does not parse USDA text itself. Inputs must be readable with the standard
OpenUSD resolver: local files, relative local references, and self-contained
USDZ packages are supported. `omni://` URIs and Nucleus access are not.

## Public API

### Optional dependency

Add a lazy optional extra:

```toml
[project.optional-dependencies]
usd = ["usd-core>=<supported-version>"]
```

Importing or using the core package must not require OpenUSD. Calling a USD API
without the extra installed raises an actionable error explaining how to install
it. The exact supported `usd-core` version range is chosen after the packaging
spike; no Isaac Sim or Omniverse package is a dependency.

### Import API

```python
from pybullet_fleet.usd_loader import UsdImportOptions, UsdImportReport, load_usd_world

report: UsdImportReport = load_usd_world(
    "warehouse.usdz",
    sim_core=sim,
    options=UsdImportOptions(
        collision=True,
        collision_required=False,
        include_invisible=False,
        include_point_instances=True,
        point_instance_warning=1_000,
        max_point_instances=5_000,
        max_collision_triangles_per_mesh=50_000,
        max_collision_triangles_total=500_000,
    ),
)
```

`UsdImportReport` is returned even when selected prims are skipped. It includes
the source stage identifier, source `upAxis` and `metersPerUnit`, the complete
source-to-PyBullet normalization transform, the target convention (`Z-up`,
metres), created object IDs mapped to USD prim paths, and structured warnings
for unsupported or unresolved content. Source metadata and normalized output
metadata are separate fields so a report cannot be mistaken for authored USD
values. Prim paths, not PyBullet body IDs, are the stable external source IDs.

All importer diagnostics use a JSON-serializable `UsdImportDiagnostic` shape:
`severity`, stable `code`, `message`, `prim_path`, `schema_type`, and optional
`asset_path`. A future Isaac-BT conversion tool uses the same shape for its
unsupported-node diagnostics. Snapshot/EventLog work may later envelope these
diagnostics in its versioned records, but the importer does not depend on that
work.

Configuration support, if added in the MVP, is limited to a world entry:

```yaml
world:
  usd_path: assets/warehouse.usdz
  usd_collision: true
```

`usd_path` and the existing `world_file` are mutually exclusive.

### Behavior-tree API

The first public surface is file-driven and uses the BehaviorTree.CPP v4 XML
syntax with a strict PyBulletFleet profile:

```python
from pybullet_fleet.behavior_tree import WorkerBehaviorTree

tree = WorkerBehaviorTree.from_file("worker_wander.xml", agent=worker)
sim.register_behavior_tree(tree)
```

`register_behavior_tree()` owns the normal callback registration, prevents
duplicate registration of the same tree, and unregisters trees when their
owning Agent is removed. A tree is ticked only on the simulation thread and
creates commands through public Agent/action APIs.
It never calls PyBullet directly or runs a background control thread. The
initial parser supports only the profile in [Worker Behavior Tree](#worker-behavior-tree);
it must reject unsupported BehaviorTree.CPP tags and attributes rather than
silently claim general BehaviorTree.CPP compatibility.

## USD Import Semantics

### Supported in the MVP

- `.usd`, `.usda`, `.usdc`, and `.usdz` stages that OpenUSD can resolve locally
- Stage metadata: `upAxis` and `metersPerUnit`
- Composed and evaluated transforms for `UsdGeom.Xform` hierarchies
- Visible `UsdGeom.Mesh` prims with vertex positions, face indices, normals
  when usable, display color, vertex or indexed face-varying `primvars:st` /
  `primvars:st_0` UVs, and a bounded material
  approximation: `UsdPreviewSurface` diffuse color plus the common Isaac MDL
  `ColorAlbedo` / `AlbedoTexture` and `diffuse_texture` inputs
- `UsdGeom.PointInstancer` for repeated static geometry, preserving each
  instance's composed transform
- Local relative references, sublayers, and variants as resolved by OpenUSD
- One static PyBullet collision/visual object per supported mesh or instance

PyBulletFleet normalizes every imported pose and mesh scale to its Z-up,
metre-based coordinate convention. The importer must use the evaluated world
transform rather than separately applying only local translate/rotate/scale
attributes. Any axis conversion is applied exactly once to both visual and
collision geometry.

The implementation owns a single `stage_to_pbf_transform` derived from stage
metadata. All mesh vertices and evaluated world poses pass through this one
transform; lower-level visual and collision constructors receive already
normalized data and must not inspect `upAxis` or `metersPerUnit`. Rigid or
uniformly scaled transforms become a `SimObject` pose plus local mesh scale;
non-representable affine transforms are baked into the mesh. The import report
records the resulting pose or exact normalized transform to make this path
debuggable.

### Collision representation policy

Phase 2 must implement this priority order; it is not left to individual
importer call sites:

1. Use a supported, authored collision/proxy mesh when the stage provides one.
2. Otherwise use the visual mesh as a static concave collision mesh only within
   the documented per-mesh and aggregate triangle budgets.
3. If no suitable collision mesh is available within those budgets, import the
   prim as visual-only and emit `collision_mesh_unavailable`. If the caller
   sets `collision_required=True`, fail before creating a partial world.

The initial limits are 50,000 triangles per mesh and 500,000 triangles for the
whole import. They are recorded in `UsdImportOptions` and can be lowered or
raised explicitly after measuring the target scene. This avoids silently
treating an arbitrary high-detail visual mesh as a usable collision
approximation.

### PointInstancer safety

The default `point_instance_warning` is 1,000 and the default
`max_point_instances` is 5,000 across one import. The importer counts the full
expanded set before allocating PyBullet bodies, emits
`point_instance_warning` at the warning threshold, and fails atomically with
`point_instance_limit_exceeded` above the hard limit. Higher limits require an
explicit user setting and are documented as a performance decision. The
warehouse-validation phase revisits these conservative defaults with measured
results; it does not remove the hard limit.

### Explicitly unsupported

- `omni://` and other non-local resolver schemes; users must first localize or
  package the stage and dependencies
- Isaac/Omniverse schemas such as PhysX, Replicator, RTX sensors, and runtime
  behavior/animation graphs
- USD articulation, joints, skeletal animation, deformable meshes, and dynamic
  rigid-body semantics
- MDL/MaterialX rendering fidelity beyond the documented color/albedo texture
  approximation; normal maps, roughness, and other shader inputs are ignored.
  Unsupported material bindings fall back to display color or PyBullet defaults
- Payload streaming, live layers, authored time-sampled animation, and editing
  USD during a simulation run

An unsupported prim must create a report warning containing its prim path and
schema type. It must not silently produce a partial environment. A failed stage
open or unresolved required mesh is an error, not a warning. Diagnostics use
the report shape above, including warnings emitted while a best-effort Isaac-BT
conversion tool rejects unsupported source nodes.

### Asset preparation

Isaac sample environments often use references and assets served through
Nucleus. The documented interchange workflow is:

1. Obtain the Isaac asset locally under its applicable license.
2. Open and validate it in Isaac Sim or OpenUSD tooling.
3. Localize/package the composition and its dependencies into a local directory
   or `.usdz` archive (for example with OpenUSD `usdzip`).
4. Run `load_usd_world()` on that local artifact.

The project must not redistribute NVIDIA/Isaac assets. The example documentation
uses a user-supplied path and contains instructions, not a checked-in copy of
`full_warehouse.usd` or its dependent assets.

## Worker Behavior Tree

### Runner hierarchy

`BehaviorTree` owns XML parsing, control-node ticking, blackboard state, and
Action-ID dispatch; it has no execution target. `AgentBehaviorTree` owns the
target Agent and adds the common `NavigateTo` action for mobile Agents.
`WorkerBehaviorTree` then adds worker-only zone selection and waiting actions.
Future forklift and picker trees are siblings of the worker tree and may add
their own leaf actions. A future device tree may instead own a device adapter.
Pick/drop XML nodes are deferred until a backend-neutral public Agent
manipulation contract is specified; they must not directly expose PyBullet
action or body identifiers.

`NavigateTo` uses the portable `NavigationAdapter` contract rather than
probing Agent attributes. The adapter starts a request from a three-dimensional
goal and reports `NavigationStatus` (`RUNNING`, `SUCCEEDED`, `FAILED`, or
`CANCELLED`). PyBulletFleet supplies an adapter over its existing Agent goal
API; other simulators supply their own implementation.

Documented Action IDs are exposed as profile enums: `AgentActionId` for mobile
Agent actions and `WorkerActionId` for worker-only actions. XML still uses the
corresponding string values, while a future registry may add explicitly named
extension-profile Action IDs.

### Portable subset

Tree definitions use [BehaviorTree.CPP v4 XML](https://www.behaviortree.dev/docs/learn-the-basics/xml_format/).
This format is the project canonical representation because it is familiar in
robotics and supports runtime loading, blackboard ports, subtrees, and
non-blocking actions. The PyBulletFleet profile supports a deliberately small
subset; it does not require the C++ runtime or promise that every
BehaviorTree.CPP tree will execute unchanged.

Isaac's experimental behavior-tree JSON is neither read nor converted at
runtime. Isaac examples are scenario references. A future developer tool may
perform a best-effort conversion of a documented subset into this XML profile,
but it must report every Isaac-specific node (for example NavMesh, animation,
and trigger nodes) that has no semantic equivalent.

```xml
<root BTCPP_format="4" main_tree_to_execute="WorkerWander">
  <BehaviorTree ID="WorkerWander">
    <Repeat num_cycles="-1">
      <Sequence>
        <Action ID="SelectGoalFromWaypointSet" waypoint_set="{worker_waypoint_set}"
                output_goal="{goal}" />
        <Action ID="NavigateTo" goal="{goal}" speed="1.2" />
        <Action ID="WaitRandom" min_seconds="2.0" max_seconds="5.0" />
      </Sequence>
    </Repeat>
  </BehaviorTree>
</root>
```

The currently implemented node set is:

| Node | Semantics | PyBulletFleet mapping |
|---|---|---|
| `Sequence` | Run children in order; fail fast | Standard control node |
| `Fallback` | Run the first successful child | Standard control node |
| `Repeat` | Re-run a child after success | Standard decorator (`num_cycles="-1"` means indefinitely) |
| `Action ID="SelectGoalFromWaypointSet"` | Choose a deterministic seeded point from a named waypoint set | Blackboard output port |
| `Action ID="NavigateTo"` | Travel to the pose supplied by `goal` | Existing kinematic motion/action path |
| `Action ID="WaitRandom"` | Remain idle for a duration range | Tick-based non-blocking action |

`Condition` and `SubTree` are planned profile extensions, not current support.
`SubTree` requires explicit port remapping and scoped blackboard semantics;
unsupported tags are rejected rather than treated as BehaviorTree.CPP-compatible.

Blackboard references use BehaviorTree.CPP port syntax such as `{goal}`. The
profile requires explicit `Action ID="..."` form rather than compact custom
element names, so validators and future tools can distinguish control,
decorator, condition, and action nodes without project-specific tag parsing.

YAML remains appropriate for simulator- or scenario-specific data, including
worker spawn configuration, named waypoint sets, and initial blackboard values.
It does not define the tree topology. Geometric navigation zones are
environment data and should ultimately be authored in portable USD metadata;
the MVP deliberately uses explicit waypoints because no stable,
engine-neutral zone schema has yet been selected. A later migration must map
zones to a documented PyBulletFleet/USO USD namespace, not to an Isaac-specific
attribute.

Each worker receives its own seed. With identical stage, tree, zone definition,
timestep, and seed, selected goal order must be deterministic. Worker-level
lifecycle events with tree ID, node path, agent identifier, and outcome remain
planned work; they are not a substitute for the future versioned event log.

### Navigation model

The first demo does not import or build an Isaac NavMesh. It uses named
PyBulletFleet waypoint sets. `SelectGoalFromWaypointSet` only chooses a point
explicitly listed in its configured set. Before committing a goal, `NavigateTo`
validates the direct path with the existing
collision/query facilities; on failure it returns `failure` so a `Fallback` or
a later path-planning integration can choose an alternative.

This is a direct-line reachability check, not path planning or obstacle
avoidance. In a cluttered warehouse a selected goal may repeatedly fail and a
worker may remain idle when its tree has no alternative branch; that is expected
MVP behavior, not an importer defect. The example reports these failures and
provides waypoint candidates that make the wander scenario demonstrable.

Workers use a capsule-like collision body and a simple visual by default.
Importing an animated Isaac human USD model is out of scope. This keeps the
demo useful in `p.DIRECT`, avoids animation/runtime dependencies, and makes the
worker a meaningful dynamic obstacle rather than visual-only scenery.

## Example: Isaac Warehouse Worker Wander

Add `examples/models/usd_warehouse_worker_demo.py`, `worker_wander.xml`, and
supporting scenario YAML files.
The example:

1. Accepts `--usd PATH` and reports missing optional dependencies clearly.
2. Loads a user-prepared local copy of Isaac's `full_warehouse.usd` or an
   equivalent packaged stage as a static collision environment.
3. Defines named worker waypoint sets and initial blackboard values in PyBulletFleet
   YAML, then loads the reusable `worker_wander.xml` tree.
4. Spawns 1--10 workers, each running the `Repeat → SelectGoal → Navigate →
   Wait` tree above.
5. Spawns existing PyBulletFleet AMRs and demonstrates a documented minimal
   reaction to nearby workers (slow/stop) when enabled.
6. Runs in GUI and headless modes. Headless output reports imported prims,
   skipped prims, worker tree transitions, and collision/reaction statistics.

The example's acceptance scenario is behaviorally comparable to Isaac's worker
wander demo, not a visual-animation or NavMesh-equivalence claim. A later
example may add `patrol` and a pick/place worker tree after the portable action
set is established.

## USO Alignment

- USD prim paths, original asset identifiers, stage metadata, and importer
  transform normalization are retained as source metadata.
- PyBullet body IDs are private implementation details and never appear as the
  stable asset identity in exported data.
- The importer establishes initial scene assets only. Runtime poses, action
  status, worker tree state, and dynamic-object lifecycle belong in the future
  `SimulationSnapshot` / event-log schema.
- The behavior tree is engine-owned execution logic. Its portable data model
  makes later USO sharing possible, but this MVP does not define a USO-wide BT
  standard or Node adapter.
- The XML profile is intended to be reusable by future adapters such as MuJoCo:
  the engine registers implementations for the same leaf-node IDs and supplies
  its own pose, collision-query, and action adapters. A tree is portable only
  when every referenced leaf node and port type is part of the documented
  profile; engine-specific nodes live in a separately named extension profile.

### Infrastructure-device execution boundary

Infrastructure devices use the same portability principle, but are not
themselves required to be behavior trees.  A device owns a backend-neutral
state machine, its request/event contract, and snapshot-visible state.  A
backend adapter owns joint/actuator commands, physical completion observation,
contact queries, and passenger attachment mechanics.  For example,
`ElevatorStateMachine` depends only on the `ElevatorMotionAdapter` contract;
the existing PyBullet `Elevator` supplies the adapter through `JointAction`
and PyBullet constraints.

Elevators expose a request policy for a request received during cabin motion:
`REJECT` (the default), `REPLACE_NEXT` (retain only the most recent following
destination), or `QUEUE` (FIFO). `ElevatorRequestResult` distinguishes an
immediate acceptance, queueing, replacement, and rejection. These policies do
not redirect a cabin that is already moving; a redirected physical transition
will require its own safety and adapter contract.

Behavior trees remain suitable for workflows that use a device, such as
requesting an elevator and waiting for arrival.  They must call the documented
device request/status API, rather than access backend joints or constraints.
Future work is to extract these pure-Python contracts into a shared package or
USO-owned core after at least one additional backend validates the boundary.

## Non-functional Requirements

- `pybullet_fleet` remains importable and all current simulations work without
  the `usd` extra installed.
- No USD stage parsing, mesh conversion, or behavior-tree tick work occurs
  unless the feature is explicitly configured.
- Static meshes are created in a batched spawn/render-disabled section where
  applicable.
- Mesh/instance conversion is bounded and reports counts before allocating a
  pathological number of PyBullet bodies. PointInstancer uses the documented
  warning and hard limits, which are configurable only by explicit opt-in.
- GUI rendering is optional; all core importer and tree tests use `p.DIRECT`.
- User-visible APIs, optional dependency, configuration, and examples require
  a `[Unreleased]` `CHANGELOG.md` entry in the implementation PR.

## Implementation Phases

1. **Packaging and composition spike**
   - Verify supported Python/OpenUSD packages in the CI Python versions.
   - Add a tiny local `.usda` test fixture with hierarchy, units, and a mesh.
   - Establish error behavior when the optional extra is absent.
   - Select and benchmark the static-collision triangle budgets required by the
     collision representation policy; this is a gate for Phase 2.

2. **Static USD importer**
   - Implement stage metadata, evaluated transforms, mesh conversion, import
     reporting, and direct-mode tests.
   - Add limited PointInstancer support and bounded expansion.
   - Document local/USDZ asset preparation and licenses.

3. **Portable worker tree**
   - Implement BehaviorTree.CPP XML-profile parser, validation, deterministic
     tick runner, goal waypoint sets, and direct-mode node tests.
   - Integrate workers through existing Agent/action APIs and emit lifecycle
     events.
   - Document the supported XML tags, node IDs, port types, and unsupported
     Isaac JSON conversion cases.

4. **Warehouse validation**
   - Validate against a user-localized Isaac warehouse stage in GUI and
     headless modes.
   - Add the runnable wander example, benchmark its worker count separately
     from fleet throughput, and document the unsupported Isaac features.

5. **Follow-on work (separate specs)**
   - Robot reaction modes and crowd-throughput benchmark.
   - Patrol and pick/place nodes.
   - Snapshot/Event-log integration and a USO simulation-node adapter.
   - Optional USD export or broader material/physics support.

## Post-MVP: USD Entity Overrides and Agent Promotion

The static importer must not infer that a mesh is movable merely because it is
named `forklift`, has collision enabled, or carries an Isaac semantic label.
Those are useful candidate signals, not control authority. The source USD stage
remains the shared environment/layout representation; PyBulletFleet-specific
runtime decisions are supplied by a colocated sidecar file such as
`warehouse.pbf.yaml`.

### Candidate discovery

Future import reports expose `entity_candidates` derived from portable USD
signals where available: `UsdPhysics.RigidBodyAPI`, articulation/joint schemas,
collision/proxy geometry, and semantic labels. Each candidate records its
stable USD root path, discovered labels/schemas, and descendant visual and
collision prim paths. Candidate discovery is advisory: it never changes an
object's static/movable status by itself.

For example, Isaac's `/World/Forklift` has a descendant with
`semantics:labels:class=[forklift]` and collision schemas, but no rigid-body or
vehicle controller. It is therefore reported as a candidate rather than
silently promoted to an Agent.

### PyBulletFleet sidecar override

The initial engine-specific format is YAML. It is deliberately separate from
the USD stage so the project does not mutate or redistribute third-party
assets, and so controller settings do not masquerade as portable scene data.

```yaml
version: 1
world:
  usd_path: warehouse_with_forklifts.usd

entities:
  - usd_root: /World/Forklift
    expected_semantic_class: forklift
    role: agent
    visual_descendants: true
    collision_proxy: /World/Forklift/S_ForkliftFork/collision_box
    controller:
      type: omni
      max_linear_vel: 1.0
```

`usd_root` is the stable external identity. The importer validates that it
exists, that promoted roots do not overlap, and that an optional expected
semantic class agrees with the composed USD stage. A configured root is removed
from the ordinary static-mesh import path; its visual descendants follow one
logical entity pose, while its configured collision proxy becomes that entity's
collision representation. A missing or invalid override fails before creating
a partial promoted entity and emits structured diagnostics.

Rigid or uniformly scaled descendant transforms can use the retained normalized
poses introduced by the importer. Non-representable descendant transforms stay
locally baked and follow their promoted root through a visual/collision child
frame. PointInstancer instances are not eligible for promotion in the first
version; they require an explicit future instance-addressing convention.

### USO boundary

Environment facts that become stable and portable—such as a logical entity
root or semantic class—may later migrate to a documented USO/USD namespace.
PyBulletFleet controller type, velocity limits, collision implementation, and
other execution details remain sidecar settings. A future MuJoCo adapter can
interpret the same portable facts but provides its own controller override.

### Follow-on checklist

- [ ] Add `entity_candidates` to `UsdImportReport`, including semantic labels,
  applied physics schemas, and stable logical root paths.
- [ ] Specify, parse, and validate versioned `.pbf.yaml` entity overrides.
- [ ] Build one movable logical entity from a configured USD root and its
  descendant visual meshes, preserving normalized child transforms.
- [ ] Support an explicitly selected collision proxy and avoid duplicate static
  collision bodies for promoted descendants.
- [ ] Add a localized Isaac forklift sidecar fixture and direct/headless tests
  for candidate reporting, successful promotion, invalid roots, overlaps, and
  semantic mismatches.
- [ ] Add `usd_warehouse_worker_demo.py` only after the generic behavior-tree
  demo and entity-promotion path are independently validated.
- [ ] Define the portable USO metadata migration only after the sidecar schema
  has proven stable across at least one additional simulation backend.

## Acceptance Criteria

- [ ] A nested local `.usda` fixture imports with correct metre/Z-up world
  poses, static collision geometry, and stable prim-path source IDs.
- [ ] Known Y-up and Z-up fixtures with non-unit `metersPerUnit` numerically
  verify the expected normalized world coordinates, mesh scale, and visual /
  collision alignment. The test detects both omitted and double-applied axis or
  unit conversion.
- [ ] A local `.usdz` package with relative dependencies imports without an
  Omniverse/Nucleus service.
- [ ] PointInstancer imports warn at 1,000 instances and reject an expansion
  above the default 5,000-instance hard limit before creating PyBullet bodies.
- [ ] The collision representation priority is covered with authored proxy,
  bounded visual fallback, visual-only warning, and `collision_required=True`
  failure cases.
- [ ] An unresolved asset, `omni://` reference, or unsupported prim produces a
  clear error or structured warning as specified above.
- [ ] `gui=False` / `p.DIRECT` USD-import tests pass with no GUI calls.
- [ ] A disabled or unavailable USD feature does not affect existing SDF/URDF
  loading paths.
- [ ] The worker tree validates malformed files before simulation starts.
- [ ] The parser accepts the documented BehaviorTree.CPP XML profile, including
  blackboard port remapping and `SubTree`, and rejects unsupported tags with
  their XML location.
- [ ] Isaac behavior-tree JSON is never treated as directly executable; any
  future conversion tool reports unsupported Isaac-specific nodes.
- [ ] With a fixed seed, the worker-wander test produces the same sequence of
  selected goals and node outcomes across runs.
- [ ] Worker movement uses existing simulation-thread action/control paths and
  never starts a background PyBullet caller.
- [ ] The example runs against a user-provided localized Isaac warehouse or
  equivalent local OpenUSD stage, with documented limitations.
- [ ] The implementation PR updates `CHANGELOG.md`, user documentation, and
  the relevant benchmark/test documentation.

## Open Questions

- [ ] Which OpenUSD Python distribution and version range is installable and
  supportable on the CI platforms?
- [ ] Should geometric worker zones be authored in a small sidecar YAML first, or can a
  stable portable USD metadata schema be selected without coupling to Isaac?
- [ ] Which existing action abstraction best owns `navigate_to_selected_goal`:
  `NavigateTo` for a normal Agent, a new pedestrian controller, or a small
  adapter?
- [ ] Should the profile adopt Groot2-compatible node-model metadata in the MVP,
  or is XML validation plus a static profile document sufficient initially?
