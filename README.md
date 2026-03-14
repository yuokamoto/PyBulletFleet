# PyBulletFleet

[![Documentation](https://readthedocs.org/projects/pybulletfleet/badge/?version=latest)](https://pybulletfleet.readthedocs.io/en/latest/)

A **kinematics-first** simulation framework for large-scale multi-robot fleets, built on PyBullet and designed for **fast N× real-time** evaluation.

## What is PyBulletFleet?

Different simulation goals call for different tools.
Physics-focused simulators (Gazebo, Isaac Sim, MuJoCo, etc.) excel at accurate contact dynamics, sensor modelling, and single-robot control — but stepping a full physics engine for every robot becomes the bottleneck when you need to evaluate **fleet-level** systems at scale.

PyBulletFleet sits in a different part of the design space: it is a **kinematics-first, fleet-scale** simulation engine whose primary goal is to enable fast development and testing of the software that *orchestrates* robot fleets rather than the software that *controls* individual robots.

### Design Priorities

- **Speed over fidelity** — Fleet algorithms (task allocation, traffic control, path planning) must be tested with hundreds to thousands of robots running *much faster* than real time. Kinematics-based stepping — teleporting each robot to its next pose without calling `stepSimulation()` — removes the physics bottleneck and enables N× real-time execution.
- **System integration over low-level control** — The primary consumers are high-level systems: WMS (Warehouse Management Systems), task orchestrators, fleet managers, and monitoring dashboards. These systems issue goals, observe progress via state snapshots, and react to events — they do not need joint-level torque feedback.
- **Scale over detail** — Validating behaviour at 100+ robot scale matters more than modelling individual link dynamics or sensor noise.
- **Interoperability** — The simulation is designed around a callback-driven step loop and snapshot-friendly state model, so that it can be plugged into larger orchestration frameworks, replay pipelines, or external control systems (e.g., gRPC / ROS 2) as those interfaces are built out.
- **Physics as an option** — When physical interaction *is* needed (grasping, conveyor dynamics, contact verification), full PyBullet physics can be switched on per-scenario without changing the rest of the stack.

### Target Use Cases

| Use Case | Description |
|----------|-------------|
| Fleet algorithm evaluation | Test path planning, task allocation, and traffic control for large robot fleets at N× real-time speed |
| Warehouse simulation | Simulate pick-and-place, patrol, and transport operations with mobile robots and arms |
| Scalability benchmarking | Measure how fleet software scales from tens to thousands of agents |
| Rapid prototyping | Quickly iterate on multi-robot behaviors with minimal boilerplate |

## Quick Start

```bash
# 1. Install package
cd PyBulletFleet
pip install -e .

# 2. Run demo
python examples/100robots_grid_demo.py
```

## Performance

<!-- sync with docs/benchmarking/results.md -->
> Results from a single test environment (Intel i7-1185G7, 32 GB RAM, Ubuntu 20.04). Your numbers will vary depending on hardware.

| Agents | Real-Time Factor | Step Time |
|--------|-----------------|-----------|
| 100    | 48× | 2.1 ms  |
| 500    | 6.8×| 14.7 ms |
| 1000   | 2.4×| 40.9 ms |
| 2000   | 1.1×| 94.8 ms |

Kinematics mode (physics OFF), headless. See [Benchmark Results](benchmark/README.md#benchmark-results) for full data, component breakdown, and methodology.

## Documentation

📖 **Full documentation:** [Read the Docs](https://pybulletfleet.readthedocs.io)

For local builds:
```bash
cd docs && sphinx-build -b html . _build/html
```

## Dependencies

- Python 3.10+
- PyBullet
- NumPy
- PyYAML

## Development Setup

### Install development dependencies

```bash
pip install -e ".[dev]"
```

### Run tests

```bash
pytest
```

### Pre-commit hooks

Install pre-commit hooks for automatic code formatting and linting:

```bash
pip install pre-commit
pre-commit install
```

Run manually on all files:

```bash
pre-commit run --all-files
```

### Code quality tools

**Format code:**
```bash
black pybullet_fleet examples
```

**Lint code:**
```bash
flake8 pybullet_fleet
```

**Type check:**
```bash
pyright pybullet_fleet
```
