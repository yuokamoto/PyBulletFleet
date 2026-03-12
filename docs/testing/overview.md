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

## Shared Fixtures

`conftest.py` provides pytest fixtures shared across all tests (e.g., PyBullet connection setup/teardown).

## Visual Demos

For GUI-based verification, see `examples/` — test files are automated only.
