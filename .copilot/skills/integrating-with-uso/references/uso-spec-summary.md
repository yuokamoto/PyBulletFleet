# USO design summary and implementation status

USO describes engine-independent world assets, initialization/state exchange,
full/delta logging and playback. Its distributed design adds a Simulation Master,
node adapters, synchronization and messaging. Those are design concepts, not
runtime dependencies of the PBF replay implementation.

Consult the actual USO documents rather than assuming old sketches are executable:

- `03_Snapshot_Specification_EN.md`: world.assets, simulation timestamp, pose,
  velocity, logical connected_to lists; joint field shapes remain open.
- `06_Logging_Replay_EN.md`: full/delta state playback and event logging.
- `16_Current_Status_and_Rollout_Approach_EN.md`: bottom-up validation across
  multiple concrete applications before extracting common abstractions.
- `17_Open_Questions_EN.md`: versioning, extension and restoration questions.
- `19_Snapshot_MetaData_and_Reproduction_Info_EN.md`: rendering reproduction
  information versus freeform metadata; not an execution input journal.
- `20_MetaData_Extensibility_Patterns_EN.md`: alternatives, not decided contracts.

PBF implements a PBF-owned `pbf.kinematic_navigation` profile with stable IDs,
step/order input re-execution and full observations. It does not implement a USO
node, delta synchronization, common runtime library, checkpoint/restore, or a
canonical schema. See `docs/design/snapshot-replay/evidence.md` for findings to
validate against USO and another backend. Do not silently promote this one
implementation into a shared cross-simulator contract.
