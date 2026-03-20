# Tests

All tests run headless (`p.DIRECT`) and require no GUI.

## Running Tests

```bash
# Lint + test (CI subset)
make verify

# Tests with coverage (75% threshold)
make test

# Quick test (stop on first failure)
make test-fast

# Run a specific file
pytest tests/test_core_simulation.py

# Verbose output
pytest tests/ -v
```

## Shared Fixtures

`conftest.py` provides pytest fixtures shared across all tests (e.g., PyBullet connection setup/teardown).

## Visual Demos

For GUI-based verification, see `examples/` — test files are automated only.
