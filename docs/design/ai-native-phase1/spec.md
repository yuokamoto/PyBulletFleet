# AI-Native Developer Experience — Phase 1

**Status:** Validated
**Scope:** `copilot-instructions.md` + root `Makefile`
**Branch:** TBD (from `main` after PR #7 merge)

---

## Problem

PyBulletFleet has excellent domain skills (`.copilot/skills/`) but lacks two foundational pieces that every AI coding tool expects:

1. **No repository-level AI instructions** — Copilot, Claude, Codex, and Cursor all read `.github/copilot-instructions.md` (or equivalent) automatically. Currently, AI assistants start every session with zero context about PyBulletFleet's architecture, conventions, or guard rails. Skills are only loaded when explicitly triggered.

2. **No single-command entry points** — There is no root `Makefile` or `justfile`. AI agents (and humans) must remember the exact `pytest`, `black`, `pyright` invocations. This creates friction and inconsistency between local development, CI, and AI-assisted workflows.

## Decision

Create both artifacts in a single deliverable:

- `.github/copilot-instructions.md` — condensed from `working-with-pybullet-fleet` skill + guard rails
- `Makefile` — mirrors CI jobs as local commands, provides the entry points that hooks and agents will later consume

## Constraints

- `copilot-instructions.md` must stay under ~200 lines — AI tools have context budgets; verbose instructions get truncated or ignored
- Makefile targets must match CI exactly — `make verify` = what CI runs, no surprises
- No new dependencies — only wraps existing tools (`black`, `pyright`, `pytest`, `pre-commit`, `sphinx-build`)
- Benchmark targets must work without GPU — `make bench-smoke` uses `p.DIRECT` mode

## Design

### copilot-instructions.md Structure

| Section | Lines (approx) | Content |
|---------|----------------|---------|
| Project Identity | 10 | Name, purpose, kinematics-first philosophy |
| Architecture | 30 | Component diagram (text), key classes, file locations |
| Code Conventions | 25 | black 127, pyright basic, lazy logging, factory methods, shared shapes |
| Testing Rules | 20 | p.DIRECT always, conftest fixtures, `--cov-fail-under=75`, strict markers |
| Guard Rails (DO NOT) | 20 | No `stepSimulation()` without physics, no direct `_agents` mutation, no GUI in tests, etc. |
| Common Patterns | 30 | Spawn params, action lifecycle, simulation loop (condensed from skill) |
| Entry Points | 15 | `make verify`, `make test`, `make bench-smoke` etc. |

**Source:** Condensed from `working-with-pybullet-fleet` SKILL.md (~210 lines → ~150 lines).
Guard rails extracted from Common Pitfalls table + lessons from PR #4-#7.

### Makefile Targets

| Target | Command | Mirrors CI Job |
|--------|---------|---------------|
| `make lint` | `pre-commit run --all-files` | `lint` job |
| `make format` | `black pybullet_fleet tests examples` | (auto-fix variant) |
| `make typecheck` | `pyright` | Part of `lint` job |
| `make test` | `pytest tests/ -v --tb=short --cov=pybullet_fleet --cov-report=term-missing --cov-fail-under=75` | `test` job |
| `make test-fast` | `pytest tests/ -x -q` | Quick local feedback |
| `make verify` | `make lint && make test` | Full CI equivalent |
| `make docs` | `cd docs && make html SPHINXOPTS=-W` | `docs` job |
| `make bench-smoke` | `python benchmark/run_benchmark.py --duration 10` | N/A (new) |
| `make bench-full` | `python benchmark/run_benchmark.py --sweep` | N/A (new) |
| `make clean` | Remove `__pycache__`, `.pytest_cache`, `build/`, `dist/`, `.coverage` | N/A |

**Design choice:** `make lint` runs full pre-commit (not just black+pyright) to ensure parity with CI. `make format` is the auto-fix variant that modifies files.

### Guard Rails in copilot-instructions.md

These are the "DO NOT" rules that prevent common AI mistakes:

1. **Never call `p.stepSimulation()` when `physics: false`** — kinematic mode teleports objects; physics stepping breaks everything
2. **Never use `p.GUI` in tests** — always `p.DIRECT` for headless CI
3. **Never mutate `_agents` or `_objects` lists directly** — use `add_object()` / `remove_object()`
4. **Never skip `SimObject._shared_shapes` cleanup** — stale IDs crash after `p.disconnect()`
5. **Never import without `TYPE_CHECKING` guard for circular deps** — common in action.py ↔ agent.py
6. **Always use `get_lazy_logger(__name__)`** — never `logging.getLogger()` or `print()`
7. **Always use factory methods** — never call `Agent.__init__()` directly
8. **Always run `make verify` before claiming work is done**

## Success Criteria

- [ ] `.github/copilot-instructions.md` exists, under 200 lines
- [ ] New Copilot/Claude session on this repo shows correct project context without loading any skill
- [ ] `Makefile` has all targets listed above
- [ ] `make verify` passes (equivalent to CI lint + test)
- [ ] `make bench-smoke` completes in <30 seconds
- [ ] No new dependencies added
- [ ] Pre-commit clean on the new files themselves
