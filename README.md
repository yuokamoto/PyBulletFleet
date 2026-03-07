# PyBulletFleet

A PyBullet-based simulation framework for large-scale multi-robot fleets, designed for **fast N× speed simulation**.

## Why PyBulletFleet?？

Evaluating fleet-level logistics (e.g., warehouse AGV/AMR operations) requires simulating **hundreds to thousands of robots** far faster than real time. PyBullet alone provides a physics engine but lacks the abstractions needed to orchestrate large fleets efficiently.

PyBulletFleet fills this gap by providing:

- **N× speed simulation** — Kinematic (teleport-based) control as the primary motion mode, enabling simulation speeds far exceeding real time without being bottlenecked by physics step overhead.
- **Scalability** — Designed for 100–10,000 robot-scale environments. Spatial-hash collision detection and shared-shape caching keep per-step update times low even at large scale (target: ≤ 10 ms per step).
- **Physics as an option** — Full PyBullet physics can be turned on when needed (e.g., grasping, conveyor dynamics) but is off by default so that pure fleet-level evaluation runs as fast as possible.
- **High-level abstractions** — Action system (MoveTo, Pick, Drop, Wait), agent managers with grid spawning, YAML-driven configuration, and a callback-based simulation loop let users focus on fleet logic rather than low-level PyBullet API calls.

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

## Documentation

📖 **Full documentation:** [Read the Docs](https://pybulletfleet.readthedocs.io)

For local builds:
```bash
cd docs && sphinx-build -b html . _build/html
```

## Dependencies

- Python 3.8+
- PyBullet
- NumPy
- PyYAML

## Development Setup

### Install development dependencies

```bash
pip install -e ".[dev]"
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
