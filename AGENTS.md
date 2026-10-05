# Agent Instructions

This repository is used by automated coding agents and human maintainers. Keep
changes reviewable and oriented toward a useful user operation, verify according
to change risk, and do not push unless the user explicitly asks for it.

For the development process and human decision points, see
`docs/AI_DEVELOPMENT_WORKFLOW.md`.

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

## Change-Based Verification

Activate `.venv` before running local checks. Match verification to the change;
CI continues to run repository-wide lint, tests and documentation builds on PRs.

| Change | Before pushing | Before requesting final review |
| --- | --- | --- |
| Documentation/instructions only | Run pre-commit on changed files; run `PBF_DOCS_OFFLINE=1 make docs` when Sphinx content or links change. No Python test suite is required. | Confirm the relevant checks and review rendered docs where useful. |
| Python source, tests or examples | Run pre-commit on changed files and focused tests plus the relevant suite. | Run `make verify` once on the final source diff; rerun only checks affected by later changes. |
| Core behavior, public API, packaging or other high-risk change | Run `make verify` before the first push and relevant integration/packaging checks. | Repeat affected checks after fixes; do not rerun an unchanged full suite mechanically. |
| ROS 2 / RMF integration | Apply the applicable row above and the bridge/RMF checks below. | Confirm the relevant integration evidence. |

For mixed changes, use the highest applicable row. `make verify` runs
`make lint` (all-file pre-commit, including black, pyright and flake8) and
`make test` (full pytest with the CI coverage threshold). For a changed-file
check, use `pre-commit run --files <changed paths> --show-diff-on-failure`.
The relevant suite means the existing test modules for the affected component
and its integration points; use the [testing guide](docs/testing/overview.md)
to select them. Record commands and any omitted checks in the PR; a passing
focused check is not a claim that the full suite passed. If targeted tests
cannot establish the changed behavior, broaden them before pushing. Do not
claim a source PR is ready for final review until `make verify` and relevant
integration checks pass.

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

If GitHub Actions fail after push, reproduce the affected check locally. For a
core lint or pytest failure, use:

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
