# Tests

All tests run headless (`p.DIRECT`) and require no GUI.

## Running Tests

```bash
# Run all tests
pytest tests/

# Run a specific file
pytest tests/test_core_simulation.py

# Verbose output
pytest tests/ -v

# Stop on first failure
pytest tests/ -x
```

## Test Coverage

| File | Tests | What it covers |
|------|------:|----------------|
| `test_core_simulation.py` | 110 | SimulationParams, init/teardown, step loop, callbacks, timing |
| `test_agent_core.py` | 76 | Agent spawn, movement, drive modes, state management |
| `test_collision_comprehensive.py` | 58 | All collision methods, spatial hashing, margin, multi-cell |
| `test_sim_object.py` | 58 | SimObject spawn, shapes, properties |
| `test_agent_manager.py` | 49 | AgentManager / SimObjectManager lifecycle |
| `test_action.py` | 42 | Action classes (Move, Pick, Drop, Wait) unit tests |
| `test_geometry.py` | 40 | Pose, Path, distance calculations |
| `test_tools.py` | 33 | Utility functions |
| `test_action_integration.py` | 31 | Action execution with real Agent + PyBullet |
| `test_logging_utils.py` | 28 | Lazy-evaluation loggers |
| `test_e2e.py` | 10 | End-to-end simulation workflows |
| `test_memory_profiling.py` | 6 | Memory profiling hooks |

## Shared Fixtures

`conftest.py` provides pytest fixtures shared across all tests (e.g., PyBullet connection setup/teardown).

## Visual Demos

For GUI-based verification, see `examples/` — test files are automated only.
