# Tests

PyBulletFleet has separate verification paths for the Python core, packaging,
documentation, and the optional ROS 2 / Open-RMF bridge. The core suite runs
headless with PyBullet `DIRECT`; the bridge and RMF checks require their ROS 2
runtime and are not part of the default pytest command.

## Everyday Checks

```bash
# Full core CI subset: pre-commit lint + pytest tests/ with 75% coverage gate
make verify

# Core tests with coverage only
make test

# Stop at the first failure; disables coverage for a quicker edit/test loop
make test-fast

# Build documentation with warnings treated as errors
make docs

# Build a wheel, install it in a clean virtual environment, and run a smoke test
make test-clean-install
```

`make verify` is the normal pre-push check for Python, tests, examples,
packaging, and documentation changes. It does **not** run the documentation
build, the clean-install check, or any ROS 2 / RMF tests.

## Targeted Core Tests

```bash
# One test module
pytest tests/test_core_simulation.py -q

# One test by node id
pytest tests/test_core_simulation.py::TestMultiRobotSimulationCore::test_step_once -q

# Tests whose names match an expression
pytest tests/ -k "collision and not comprehensive" -q

# Avoid coverage overhead for a focused local run
pytest tests/test_fleet_api.py -q --no-cov
```

The suite's default configuration already targets `tests/`, enables strict
pytest configuration and markers, and collects coverage for
`pybullet_fleet`. Use `make test` when checking the repository's 75% coverage
threshold rather than inferring coverage from a targeted run.

## What the Core Suite Covers

| Change area | Useful core tests |
|---|---|
| Simulation lifecycle, pacing, two-phase step, profiling | `test_core_simulation.py`, `test_run_simulation_diagnostics.py`, `test_two_phase_step.py` |
| Agents, managers, controllers, batch execution | `test_agent_core.py`, `test_agent_manager.py`, `test_controller*.py`, `test_batch_*.py` |
| Collision and geometry | `test_collision_comprehensive.py`, `test_geometry.py` |
| Configuration, models, SDF/mesh loading | `test_config_*.py`, `test_defaults.py`, `test_robot_models.py`, `test_sdf_loader.py`, `test_mesh_loader.py` |
| Actions, fleet API, plugins, events, workcells | `test_action*.py`, `test_fleet_api.py`, `test_sim_plugin.py`, `test_events.py`, `test_workcell_plugin.py` |
| Examples, recording, camera/capture | `test_examples.py`, `test_recorder.py`, `test_capture_screen_demo.py`, `test_camera_controller.py` |
| Benchmark and command-line code | `test_run_benchmark.py`, `test_cli.py` |

The table is a starting point, not a replacement for the complete suite. Run
`make verify` before merging a cross-cutting core change.

## Fixtures and Test Isolation

`tests/conftest.py` provides more than a PyBullet connection fixture. It:

- opens and tears down headless `DIRECT` sessions through `pybullet_env`;
- clears shared PyBullet shape IDs between tests;
- disables DataMonitor GUI creation globally;
- supplies `MockSimCore` for unit tests that do not need a real simulation; and
- parameterizes arm tests across physics and kinematic paths.

Tests should use these fixtures rather than sharing PyBullet client IDs or
depending on test order.

## ROS 2 and Open-RMF Checks

`ros2_bridge/` is intentionally excluded from the core pytest suite and normal
pre-commit hooks because it needs ROS 2 Jazzy, message packages, and RMF
dependencies. When bridge code, Docker files, launch/config files, or RMF
integration changes, run `make verify` **and** an appropriate bridge check.

```bash
# Docker bridge API smoke test
cd docker
docker compose run --rm --no-deps -v "$(pwd):/docker:ro" \
  bridge bash /docker/test_bridge_api.sh

# Native RMF client-mode matrix (after sourcing .ros2_native_env)
cd ..
source .ros2_native_env
make rmf-matrix
```

The CI bridge workflow also runs ROS package tests with colcon, the bridge API
smoke check, and RMF stack/dispatch scenarios. See the [ROS 2 and Open-RMF
documentation](../ros2/index.md) for setup and integration-specific commands.

## Manual GUI Checks

The automated suite does exercise example discovery, capture paths, and camera
logic, but it does not use a GUI window. Run the relevant script from
`examples/` manually when validating visual layout, camera framing, keyboard
interaction, or user-facing recording quality.
