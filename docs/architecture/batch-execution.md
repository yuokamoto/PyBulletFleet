# Batch Execution

Batch execution is the developer-facing design behind the scale-oriented
controllers. It is distinct from the **fleet command interface**: batch
execution determines how movement is computed inside the simulation, while the
fleet API determines how commands enter the simulation.

## Ownership and lifecycle

An `AgentManager` owns an optional shared `BatchKinematicController`. Agents in
that manager register with the controller; the controller holds contiguous NumPy
buffers for their state and advances the registered set during Phase 1.

```text
AgentManager
  -> BatchKinematicController
       -> registered agents and N-row state buffers
       -> batch_advance(dt)
       -> sim_core.set_poses(...), buffered for the Phase 2 flush
```

The built-in implementations are `batch_omni` and `batch_differential`. Each
shared controller serves one motion-model family; use separate managers for
mixed omnidirectional and differential fleets.

## What remains per-agent

Batch mode does not bypass an agent's lifecycle. `Agent.update()` still runs so
that action queues and entity event hooks remain valid. Only the normal
per-agent controller computation is skipped; `batch_advance(dt)` produces the
movement updates for all registered agents together.

This separation is important when writing actions or plugins: use public Agent
methods and do not mutate controller-private NumPy arrays. It keeps the same
action behavior available when a scenario switches between per-agent and batch
controllers.

## Extension contract

Custom batch controllers subclass `BatchKinematicController`, provide a unique
`_registry_name`, and implement `batch_advance(dt)`. They must keep any
per-agent arrays aligned with registration and removal: registration may grow
the arrays and removal compacts a removed row by swapping in the final row.

The controller should write via the supplied buffered pose path, never by
calling PyBullet pose APIs directly. This preserves the all-objects-consistent
world snapshot required by collision processing. See [Two-Phase Step](two-phase-step).

## User-facing configuration

Users normally select a built-in batch controller through `AgentManager` or a
YAML `managers:`/`entities:` definition; they do not construct buffer arrays.
The complete configuration forms, compatibility rules, and performance guidance
are in [Controller Configuration](../how-to/controller-config). The runnable
fleet tutorial explains when to combine `--controller batch` with
`--command-interface fleet`.
