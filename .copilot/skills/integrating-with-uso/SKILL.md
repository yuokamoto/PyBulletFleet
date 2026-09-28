---
name: integrating-with-uso
description: "Use when implementing USO (Unified Simulation Orchestrator) integration for PyBulletFleet - SimulationNode adapter, snapshot serialization and deserialization, replay functionality, delta snapshot generation, or ZeroMQ messaging"
---

# Integrating with USO

Read `docs/how-to/replay.md` and `docs/design/snapshot-replay/spec.md` for the
implemented PBF navigation replay contract before designing USO integration.

PBF currently owns its versioned replay artifact. Its initial-state/observation
concepts provide evidence for refining USO; they are not a permanent schema fork
or a claim of USO compatibility. Input execution, state exchange, result playback
and checkpoint/resume are different contracts.

## Required investigation

- Read `working-with-pybullet-fleet` for current code boundaries.
- Read the actual USO snapshot/replay/open-question documents when available.
- Compare identity, time/step, units, state completeness, connections, ownership,
  versioning and engine-specific execution requirements.
- Preserve transport independence: effective PBF inputs can originate in Python,
  ROS or RMF, but PBF replay does not reproduce upstream delivery/planning.
- Keep rendering `reproduction_info` distinct from execution provenance.

## Scope and ownership

Do not introduce a Simulation Master, ZeroMQ, Redis, common package, delta
tracker or arbitrary state serialization solely because USO describes it.
Implement only the Human-approved use case and profile. Any canonical schema or
compatibility claim requires an explicit tested profile and ownership decision.
Use multiple concrete backends before extracting shared abstractions.

References:

- [Current mapping](references/snapshot-mapping.md)
- [USO design summary](references/uso-spec-summary.md)
- USO repository: https://github.com/yuokamoto/Unified-Simulation-Orchestrator
