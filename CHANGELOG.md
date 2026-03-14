# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

## v0.1.0 (2026-03-14)

Initial public release of PyBulletFleet — a kinematics-first simulation framework for large-scale multi-robot fleets.

### Highlights

- **Kinematics-first simulation** — Teleport-based stepping enables N× real-time execution for fleet-scale evaluation
- **Scalable to 1000+ agents** — Spatial-hash collision detection and shared-shape caching keep step times low
- **Action system** — Built-in MoveTo, Pick, Drop, and Wait actions with queue-based execution
- **Flexible physics** — Full PyBullet physics available per-scenario when needed (grasping, conveyor dynamics)

### Features

- Multi-robot simulation core with configurable physics/kinematics modes
- Agent and AgentManager with grid spawning, YAML-driven configuration
- Spatial-hash collision detection with configurable cell sizes and modes
- Differential drive motion with smooth Slerp rotation
- Action queue system (MoveTo, Pick, Drop, Wait) with status tracking
- SimObject base class for robots, structures, and custom objects
- Callback-driven simulation loop with pause/resume support
- Time and memory profiling built in
- URDF and mesh object support with shared-shape caching

### CI & Release Infrastructure

- GitHub Actions CI: lint, test matrix (Python 3.10/3.11/3.12), docs build, security audit, license check
- Automated PyPI release via tag push (Trusted Publisher / OIDC)
- Pre-release and publish scripts with semver validation
- Releasing Copilot skill for guided release orchestration

### Documentation

- Complete ReadTheDocs documentation: tutorials, API reference, architecture guide, how-to guides
- Zero Sphinx warnings — builds cleanly with `-W` flag
- Benchmark results and profiling guides

### Performance

- Quaternion Slerp replaces scipy dependency (7× rotation speedup)
- 1000 agents at 3.2× real-time, 31 ms/step (Intel i7-1185G7)
