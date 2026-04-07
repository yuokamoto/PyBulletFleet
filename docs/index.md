# PyBulletFleet Documentation

<table width="100%">
<tr>
<td align="center"><b>Mixed Fleet Grid</b><br><small>100robots_grid_demo.py</small><br>
<video src="100robots_grid_mixed.mp4" width="100%" autoplay loop muted playsinline></video></td>
<td align="center"><b>Cube Patrol</b><br><small>100robots_cube_patrol_demo.py</small><br>
<video src="100robots_cube_patrol.mp4" width="100%" autoplay loop muted playsinline></video></td>
</tr>
<tr>
<td align="center"><b>Mobile Pick & Drop</b><br><small>pick_drop_mobile_100robots_demo.py</small><br>
<video src="pick_drop_mobile_100robots.mp4" width="100%" autoplay loop muted playsinline></video></td>
<td align="center"><b>Arm Pick & Drop</b><br><small>pick_drop_arm_100robots_demo.py</small><br>
<video src="pick_drop_arm_100robots.mp4" width="100%" autoplay loop muted playsinline></video></td>
</tr>
</table>

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
examples/index
architecture/index
how-to/index
configuration/index
benchmarking/index
testing/index
api/index
roadmap
```

### How to read this documentation

**📘 Simulation Users** — building simulations with PyBulletFleet:

| Section | What you'll find | When to read |
|---------|------------------|--------------|
| [Quickstart](getting-started/quickstart) | Installation and first simulation | First visit |
| [Examples](examples/index) | Step-by-step tutorials: spawn objects, action queue, 100-robot fleet | Writing your first simulation |
| [How-to](how-to/index) › [Collision Config](how-to/collision-config) | Detection method, per-object modes, spatial hash tuning | Configuring collision for your scene |
| [How-to](how-to/index) › [Custom Profiling](how-to/custom-profiling) | Adding your own profiling metrics | Measuring your custom logic |
| [Configuration](configuration/index) | YAML parameter reference | Looking up a specific setting |
| [Architecture](architecture/index) › [Collision Overview](architecture/collision-overview) | Design goals, mode summary, two-phase pipeline | Understanding the collision system |
| [Benchmarking](benchmarking/index) › [Optimization Guide](benchmarking/optimization-guide) | Performance tuning workflow | Improving simulation speed |
| [API Reference](api/index) | Auto-generated module docs | Looking up a specific class or method |

**🔧 PyBulletFleet Developers** — extending or debugging the framework:

| Section | What you'll find | When to read |
|---------|------------------|--------------|
| [Architecture](architecture/index) | Design decisions, collision internals, spatial hash grid | Understanding *why* things work the way they do |
| [How-to](how-to/index) › [Time](how-to/time-profiling) / [Memory](how-to/memory-profiling) Profiling | Internal component profiling | Diagnosing bottlenecks |
| [How-to](how-to/index) › [Logging](how-to/logging) | LazyLogger, performance-safe logging | Adding or reviewing log output |
| [Benchmarking](benchmarking/index) | Benchmark suite, profiling scripts, experiments | Deep performance analysis |
| [Testing](testing/index) | Test strategy, running tests, coverage | Contributing or debugging |

## Key Features

- **N× real-time simulation** — Kinematics-based (teleport) stepping as the default motion mode
- **Scalability** — Tested with 100+ robots; spatial-hash collision keeps per-step cost low as fleet size grows
- **Physics as an option** — Full PyBullet physics can be enabled per-scenario when needed
- **High-level abstractions** — Action system (MoveTo, Pick, Drop, Wait), agent managers, YAML-driven configuration

## Performance at a Glance

> Single test environment (Intel i7-1185G7, 32 GB RAM, Ubuntu 20.04). Results will vary by hardware.

| Agents | Real-Time Factor | Step Time |
|--------|-----------------|-----------|
| 100    | 41× | 2.4 ms  |
| 500    | 6.7×| 15.0 ms |
| 1000   | 2.4×| 42.4 ms |
| 2000   | 1.1×| 88.6 ms |

Kinematics mode (physics OFF), headless. See {doc}`benchmarking/benchmark-suite` for full data, component breakdown, and methodology.

See the {doc}`roadmap` for upcoming features and integrations.

## Indices and tables

- {ref}`genindex`
- {ref}`modindex`
- {ref}`search`
