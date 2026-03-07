# PyBulletFleet Documentation

A **kinematics-first** simulation framework for large-scale multi-robot fleets,
built on PyBullet and designed for **fast N× real-time** evaluation.

## What is PyBulletFleet?

Different simulation goals call for different tools.
Physics-focused simulators (Gazebo, Isaac Sim, MuJoCo, etc.) excel at accurate
contact dynamics, sensor modelling, and single-robot control — but stepping a
full physics engine for every robot becomes the bottleneck when you need to
evaluate **fleet-level** systems at scale.

PyBulletFleet sits in a different part of the design space: it is a
**kinematics-first, fleet-scale** simulation engine whose primary goal is to
enable fast development and testing of the software that *orchestrates* robot
fleets rather than the software that *controls* individual robots.

### Design priorities

- **Speed over fidelity** — Fleet algorithms (task allocation, traffic control,
  path planning) must be tested with hundreds to thousands of robots running
  *much faster* than real time. Kinematics-based stepping — teleporting each
  robot to its next pose without calling `stepSimulation()` — removes the
  physics bottleneck and enables N× real-time execution.
- **System integration over low-level control** — The primary consumers are
  high-level systems: WMS (Warehouse Management Systems), task orchestrators,
  fleet managers, and monitoring dashboards. These systems issue goals, observe
  progress via state snapshots, and react to events — they do not need
  joint-level torque feedback.
- **Scale over detail** — Validating behaviour at 100+ robot scale matters
  more than modelling individual link dynamics or sensor noise.
- **Interoperability** — The simulation is designed around a callback-driven
  step loop and snapshot-friendly state model, so that it can be plugged into
  larger orchestration frameworks, replay pipelines, or external control
  systems (e.g., gRPC / ROS 2) as those interfaces are built out.
- **Physics as an option** — When physical interaction *is* needed (grasping,
  conveyor dynamics, contact verification), full PyBullet physics can be
  switched on per-scenario without changing the rest of the stack.

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

- **N× real-time simulation** — Kinematics-based (teleport) stepping as the default motion mode
- **Scalability** — Tested with 100+ robots; spatial-hash collision keeps per-step cost low as fleet size grows
- **Physics as an option** — Full PyBullet physics can be enabled per-scenario when needed
- **High-level abstractions** — Action system (MoveTo, Pick, Drop, Wait), agent managers, YAML-driven configuration

### Planned

- **Snapshot-based state** — Full and delta snapshot serialization for logging, replay, and external synchronization
- **External integration** — gRPC / ROS 2 interfaces for connecting to orchestrators, WMS, and fleet managers

## Indices and tables

- {ref}`genindex`
- {ref}`modindex`
- {ref}`search`
