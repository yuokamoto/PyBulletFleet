# Agent Instructions

This repository is used by automated coding agents and human maintainers. Keep
changes small, verify the same checks that CI will run, and do not push unless
the user explicitly asks for it.

## Before Pushing

Run the core CI subset before pushing any branch that changes Python source,
tests, examples, packaging, or documentation:

```bash
source .venv/bin/activate
make verify
```

`make verify` runs:

- `make lint`: pre-commit hooks, including black, pyright, and flake8.
- `make test`: `pytest tests/` with coverage and the CI coverage threshold.

If the agent sandbox cannot write to `~/.cache/pre-commit`, run lint with a
temporary cache:

```bash
PRE_COMMIT_HOME=/tmp/pbf-pre-commit make lint
```

For the same pytest command through pre-commit's manual hook:

```bash
pre-commit run --hook-stage manual ci-pytest --all-files
```

## ROS 2 / RMF Changes

Core pytest does not exercise the ROS 2 bridge. For changes under
`ros2_bridge/`, `docker/`, launch/config files, or RMF integration code, also run
the relevant Docker or native ROS 2 checks. At minimum, run the bridge/RMF smoke
test that matches the changed surface before pushing.

If GitHub Actions fail after push, first reproduce locally with:

```bash
PRE_COMMIT_HOME=/tmp/pbf-pre-commit pre-commit run --all-files --show-diff-on-failure
pytest tests/ -q --tb=short --cov=pybullet_fleet --cov-report=term-missing --cov-fail-under=75
```

## Environment Notes

The normal CI install is `pip install -e ".[dev]"`. Optional extras such as
`.[models]` can change local test behavior, especially tests around
`robot_descriptions`. If local failures only appear with optional extras
installed, call that out explicitly and verify the CI-equivalent environment
when practical.
