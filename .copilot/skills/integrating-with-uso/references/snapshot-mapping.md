# Snapshot mapping: current PBF evidence versus USO design

The implemented PBF v1 contract is documented in `docs/how-to/replay.md`.
It is a restricted initial-state re-execution profile, not the former draft
full/delta playback or arbitrary restore design. No USO-compatible profile has
yet been defined/tested, and PBF does not depend on a USO runtime package.

| Concept | PBF implementation | USO comparison / future work |
| --- | --- | --- |
| Identity | Persisted entity_id, separate unique API name within session | Map explicitly to asset_id; never use body_id/object_id as durable ID |
| Time | Integer transition step, phase/order; state_step for observations | Timestamp alone does not express these execution semantics |
| State | Position, xyzw orientation and controller-reported velocities | Define frame and velocity meaning before claiming compatibility |
| Full | All fields/entities in observation profile; static definition referenced | Full observation is not a self-contained execution checkpoint |
| Connections | Not supported by navigation profile | Current USO connected_to is a list of logical asset IDs; physical attachment needs more data |
| Execution | PBF input journal, controller configuration, profile version | Keep separate from generic world state and rendering reproduction_info |
| Provenance | Source/environment/asset hashes, run/command IDs | Candidate common concept, not a decided shared schema |
| Deltas | Not implemented | `_moved_this_step` is collision bookkeeping, not a complete state dirty tracker |

Earlier examples using `_name_to_object`, `Agent._urdf_path`, or
`p.getBaseVelocity(obj.object_id)` were design sketches, not supported APIs.
PyBullet queries require body_id and physicsClientId; kinematic reported velocity
is not interchangeable with engine velocity. Arbitrary `properties` cannot make
joint/controller/plugin restoration complete.

Use the local USO specifications as design evidence, especially chapters 03, 06,
16–20. After implementation, feed PBF findings back through Human review and test
with another backend before extracting shared packages.
