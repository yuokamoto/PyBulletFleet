# PyBulletFleet Documentation

A **kinematics-first** simulation framework for large-scale multi-robot fleets,
built on PyBullet and designed for **fast N× real-time** evaluation.

## What is PyBulletFleet?

Most robot simulators focus on **physics fidelity** — accurate contact dynamics,
sensor modelling, and real-time rendering. This is essential for single-robot
control research, but fleet-level development has a different set of priorities:

- **Speed over fidelity** — Evaluating fleet algorithms (task allocation, traffic
  control, path planning) requires simulating hundreds to thousands of robots
  *much faster* than real time. Physics-accurate stepping is often the bottleneck.
- **System integration over low-level control** — The primary consumers are
  high-level systems such as WMS, task orchestrators, and fleet managers that
  issue goals and monitor progress, not joint-level controllers.
- **Scale over detail** — Testing at 1 000+ robot scale matters more than
  modelling individual link dynamics.

PyBulletFleet is built for this use case. It treats **kinematics as the default
motion mode** — robots are teleported to their next pose each step, bypassing
`stepSimulation()` entirely — so the loop runs at N× real-time even with
thousands of agents. When physics *is* needed (e.g., grasping, conveyor
dynamics), it can be switched on per-scenario without changing the rest of the
stack.

```{toctree}
:maxdepth: 2
:caption: Contents

getting-started/quickstart
architecture/index
how-to/index
configuration/index
benchmarking/index
testing/index
api/index
```

## Key Features

- **N× speed simulation** — Kinematic (teleport-based) control as the primary motion mode
- **Scalability** — Designed for 100–10,000 robot-scale environments
- **Physics as an option** — Full PyBullet physics can be turned on when needed
- **High-level abstractions** — Action system, agent managers, YAML-driven configuration
- **System integration** — Callback-based loop and snapshot API for connecting to external orchestrators

## Indices and tables

- {ref}`genindex`
- {ref}`modindex`
- {ref}`search`
