# AI-Native Phase 1 — Agent Spec

**Feature:** copilot-instructions.md + Makefile
**Status:** Validated

## Approach

Two independent files, no code changes to existing source:

1. **`.github/copilot-instructions.md`** — Condense `working-with-pybullet-fleet` SKILL.md into ~150 lines. Add guard rails from Common Pitfalls table and lessons from PR #4-#7. Add `make` entry points section. Keep prose minimal—use tables and code blocks for scanability.

2. **`Makefile`** — Standard GNU Make with `.PHONY` declarations. Each target wraps an existing command verbatim from CI. `verify` is the composite target. `help` is the default target (self-documenting).

## Design

### Architecture

No changes to existing architecture. Two new files:

```
.github/
  copilot-instructions.md   ← NEW: AI-readable project context
Makefile                     ← NEW: entry points for dev commands
```

### Key Components

| Component | Responsibility | Location |
|-----------|---------------|----------|
| copilot-instructions.md | Project context for AI tools | `.github/copilot-instructions.md` |
| Makefile | Single-command dev entry points | `Makefile` (repo root) |

### copilot-instructions.md — Section Spec

**Section 1: Project Identity (~10 lines)**
- Name, one-line description
- "Kinematics-first" design philosophy (from README)
- Python ≥3.10, PyBullet backend
- Key performance claim: 100 robots at 48× real-time

**Section 2: Architecture (~30 lines)**
- Component text diagram (from SKILL.md, verbatim)
- Component Quick Reference table (condensed to 6 rows: Core, Agent, SimObject, Actions, Types, Geometry)
- Key file locations

**Section 3: Code Conventions (~25 lines)**
- black (line-length=127), pyright (basic mode), flake8
- Factory methods required (from_params, from_yaml, from_dict)
- Lazy logging: `get_lazy_logger(__name__)` always
- Shared shape cache: class-level `_shared_shapes` dict
- `TYPE_CHECKING` import guard for circular deps

**Section 4: Testing Rules (~20 lines)**
- Always `p.DIRECT`, never `p.GUI`
- `SimulationParams(gui=False, monitor=False)` for headless
- Factory methods (`Agent.from_params()`) for test setup
- Coverage threshold: 75% (`--cov-fail-under=75`)
- Autouse fixtures: `_clear_shared_shapes`, `_disable_monitor_gui`
- Strict markers: `--strict-markers --strict-config`

**Section 5: Guard Rails (~20 lines)**
Format as numbered "DO NOT" list:
1. No `p.stepSimulation()` without `physics: true`
2. No `p.GUI` in tests
3. No direct `_agents`/`_objects` mutation
4. No skipping `_shared_shapes` cleanup
5. No imports without `TYPE_CHECKING` guard (action↔agent)
6. No `logging.getLogger()` — use `get_lazy_logger()`
7. No `__init__()` — use factory methods
8. No claiming done without `make verify`

**Section 6: Common Patterns (~30 lines)**
- Spawn params dataclass example (AgentSpawnParams)
- Action lifecycle: `add_action()` → `update(dt)` → status transitions
- Sim loop: `from_yaml()` → `register_callback()` → `run_simulation()`
- YAML config loading: `SimulationParams.from_config()`

**Section 7: Entry Points (~15 lines)**
- Table of `make` targets with one-line descriptions
- Highlight `make verify` as the "CI equivalent" command

### Makefile — Target Spec

```makefile
.DEFAULT_GOAL := help

.PHONY: help lint format typecheck test test-fast verify docs bench-smoke bench-full clean

help:  ## Show available targets
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}'

lint:  ## Run all pre-commit hooks (black, pyright, flake8)
	pre-commit run --all-files --show-diff-on-failure

format:  ## Auto-format code with black
	black pybullet_fleet tests examples

typecheck:  ## Run pyright type checker
	pyright

test:  ## Run tests with coverage (CI equivalent)
	pytest tests/ -v --tb=short --cov=pybullet_fleet --cov-report=term-missing --cov-fail-under=75

test-fast:  ## Quick test run (stop on first failure)
	pytest tests/ -x -q

verify: lint test  ## Full verification (CI equivalent: lint + test)

docs:  ## Build documentation (warnings as errors)
	cd docs && make html SPHINXOPTS=-W

bench-smoke:  ## Quick benchmark (~10 seconds)
	python benchmark/run_benchmark.py --duration 10

bench-full:  ## Full benchmark sweep
	python benchmark/run_benchmark.py --sweep

clean:  ## Remove build artifacts and caches
	rm -rf __pycache__ .pytest_cache build/ dist/ .coverage htmlcov/
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name "*.pyc" -delete 2>/dev/null || true
```

### Code Patterns

No code changes. The Makefile pattern follows GNU Make self-documenting convention:
```makefile
target:  ## Description
	command
```
The `help` target uses `grep` + `awk` to extract `##` comments into a formatted help menu.

## File References

Files the plan agent MUST read before implementing:

- `.copilot/skills/working-with-pybullet-fleet/SKILL.md` — primary source for copilot-instructions.md content
- `.github/workflows/ci.yml` — exact commands that `make` targets must mirror
- `.pre-commit-config.yaml` — hooks that `make lint` wraps
- `config/config.yaml` — default configuration (for patterns section)
- `pybullet_fleet/types.py` — enum reference (for architecture section)
- `tests/conftest.py` — fixture reference (for testing section)
- `README.md` — design philosophy and performance claims

## Success Criteria

- [ ] `.github/copilot-instructions.md` < 200 lines, covers all 7 sections
- [ ] AI assistant starting a new session shows correct architecture without skill loading
- [ ] Guard rails section includes all 8 "DO NOT" rules
- [ ] `Makefile` has all 10 targets (help, lint, format, typecheck, test, test-fast, verify, docs, bench-smoke, bench-full, clean)
- [ ] `make help` prints self-documenting target list
- [ ] `make verify` passes locally
- [ ] `make bench-smoke` completes in <30 seconds
- [ ] `make test` matches CI test command exactly
- [ ] `make lint` matches CI lint command exactly
- [ ] Pre-commit passes on all new files
