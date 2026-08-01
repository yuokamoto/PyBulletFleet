# Agent Instructions

This repository is used by automated coding agents and human maintainers. Keep
changes small, verify the same checks that CI will run, and do not push unless
the user explicitly asks for it.

## Repo-Local Skills

Repo-local skills live under `.copilot/skills/`. Claude uses the same files via
`.claude/skills -> ../.copilot/skills`. Some agents do not automatically list
repo-local skills in their active tool-provided skill registry, so inspect these
files explicitly when the task matches their scope.

- Release work: `.copilot/skills/releasing/SKILL.md`
- Performance work: `.copilot/skills/pybullet-performance-workflow/SKILL.md`

## Pull Request Changelog

Every pull request MUST assess its user-visible impact and update the
`[Unreleased]` section of `CHANGELOG.md` in the same pull request when it
changes public APIs, behavior, configuration, packaging, supported
environments, performance characteristics, or user documentation.

Purely internal changes (for example, test-only, CI-only, or mechanical
refactors) may omit a changelog entry only when the PR description explicitly
states that there is no user-visible change. Do not create a version heading in
a feature PR; the release workflow promotes the accumulated `[Unreleased]`
entries after the release version is selected.

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

## Pre-Release Performance Refresh

Before a release, refresh performance numbers rather than relying on stale docs:

```bash
make bench-release
```

Also refresh ROS bridge performance when `ros2_bridge/`, `docker/`, RMF client
modes, fleet API, or batch controller behavior changed. Use the Docker scale
checker for at least the release-relevant fleet/per_robot/hybrid cases, for
example:

```bash
cd docker
docker compose run --rm --no-deps -v "$(pwd):/docker:ro" \
  bridge bash /docker/test_fleet_scale.sh --robots 1000 \
  --interface-mode fleet --command-interface fleet \
  --publish-rate 5 --target-rtf 0 --measure-rtf
```

After benchmarking, sync the documented numbers in `docs/benchmarking/results.md`,
the README performance table, `docs/index.md`, and `ros2_bridge/PERFORMANCE.md`
when ROS bridge numbers changed.
