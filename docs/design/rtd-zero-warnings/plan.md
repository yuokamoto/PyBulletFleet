# ReadTheDocs Zero Warnings — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Eliminate all 238 Sphinx warnings so `sphinx-build -W` passes, and ReadTheDocs builds succeed.

**Architecture:** 5-phase sequential fix: symlinks → duplicate autodoc → docstring formatting → misc markdown → verification. Each phase is independently verifiable by running `sphinx-build` and counting remaining warnings.

**Tech Stack:** Sphinx 7.x, myst-parser 3.x, Napoleon (Google-style docstrings), reStructuredText

---

## Baseline

```bash
cd docs && rm -rf _build && python3 -m sphinx -b html . _build/html 2> /dev/null
# → "build succeeded, 238 warnings."
```

**Warning breakdown (238 total):**

| Category | Count | Phase |
|----------|-------|-------|
| Unexpected indentation | 77 | 3 |
| Duplicate object description | 72 | 2 |
| Block quote ends without blank line | 48 | 3 |
| Definition list ends without blank line | 34 | 3 |
| Inline strong start-string (`**kwargs`) | 5 | 3 |
| Cross-reference not found | 2 | 4 |
| Orphan document | 1 | 2 |
| Adjacent transitions | 1 | 4 |
| **Total** | **238** | |

**Non-duplicate warnings per source file:**

| File | Warnings |
|------|----------|
| `pybullet_fleet/sim_object.py` | 44 |
| `pybullet_fleet/agent.py` | 42 |
| `pybullet_fleet/agent_manager.py` | 24 |
| `pybullet_fleet/core_simulation.py` | 22 |
| `pybullet_fleet/geometry.py` | 16 |
| `pybullet_fleet/action.py` | 10 |
| `pybullet_fleet/tools.py` | 2 |
| `pybullet_fleet/logging_utils.py` | 2 |
| `docs/benchmarking/experiments.md` | 2 |
| `docs/architecture/collision-spatial-hash.md` | 1 |
| `docs/api/generated/modules.rst` | 1 |

---

## Task 1: Replace Symlinks with MyST Include — SERIAL

Three symlinks in `docs/benchmarking/` point outside the docs tree. ReadTheDocs clones via git and won't follow these. Replace each symlink with a real `.md` file containing a MyST `{include}` directive.

**Files:**
- Remove+Create: `docs/benchmarking/benchmark-suite.md`
- Remove+Create: `docs/benchmarking/experiments.md`
- Remove+Create: `docs/benchmarking/profiling-guide.md`

### Step 1: Remove symlinks and create include files

```bash
cd /home/rapyuta/rr_sim_evaluation/PyBulletFleet

# Verify current symlinks
ls -la docs/benchmarking/*.md | grep "^l"

# Remove and replace each symlink
rm docs/benchmarking/benchmark-suite.md
rm docs/benchmarking/experiments.md
rm docs/benchmarking/profiling-guide.md
```

Create `docs/benchmarking/benchmark-suite.md`:
````markdown
```{include} ../../benchmark/README.md
```
````

Create `docs/benchmarking/experiments.md`:
````markdown
```{include} ../../benchmark/experiments/README.md
```
````

Create `docs/benchmarking/profiling-guide.md`:
````markdown
```{include} ../../benchmark/profiling/README.md
```
````

### Step 2: Verify symlinks are gone

```bash
ls -la docs/benchmarking/*.md | grep "^l"
# Expected: no output (no symlinks remain)

file docs/benchmarking/benchmark-suite.md
# Expected: "ASCII text" (not "symbolic link")
```

### Step 3: Test build

```bash
cd docs && rm -rf _build && python3 -m sphinx -b html . _build/html 2> /tmp/sw.txt
tail -1 /tmp/sw.txt
# Expected: warning count drops (symlinks no longer break, but cross-ref warnings remain — fixed in Task 5)
```

### Step 4: Commit

```bash
git add docs/benchmarking/benchmark-suite.md docs/benchmarking/experiments.md docs/benchmarking/profiling-guide.md
git commit -m "docs: replace symlinks with MyST include directives for RTD compatibility"
```

---

## Task 2: Fix Duplicate Object Descriptions (72 warnings) — SERIAL

**Root cause:** `docs/api/generated/pybullet_fleet.rst` has `automodule` directives for each submodule AND for the top-level `pybullet_fleet` package. Since `__init__.py` re-exports symbols, both create index entries → 72 "duplicate object description" warnings.

**Fix:** Add `:noindex:` to every submodule `automodule` block. The top-level `pybullet_fleet` module (at the bottom of the file) remains the canonical entry.

**File:** `docs/api/generated/pybullet_fleet.rst`

### Step 1: Add `:noindex:` to all submodule automodule directives

The file has 11 `.. automodule::` blocks for submodules, plus 1 for the top-level package. Add `:noindex:` to the 11 submodule blocks only. The top-level `.. automodule:: pybullet_fleet` at the bottom must NOT get `:noindex:`.

Current pattern (repeated 11 times):
```rst
.. automodule:: pybullet_fleet.action
   :members:
   :undoc-members:
   :show-inheritance:
```

Change each to:
```rst
.. automodule:: pybullet_fleet.action
   :members:
   :undoc-members:
   :show-inheritance:
   :noindex:
```

The 11 submodules are: `action`, `agent`, `agent_manager`, `config_utils`, `core_simulation`, `data_monitor`, `geometry`, `logging_utils`, `sim_object`, `tools`, `types`.

Do NOT add `:noindex:` to the final block:
```rst
.. automodule:: pybullet_fleet
   :members:
   :undoc-members:
   :show-inheritance:
```

### Step 2: Fix orphan document (1 warning)

**File:** `docs/api/generated/modules.rst`

Add `:orphan:` at the very top of the file:

Current:
```rst
pybullet_fleet
==============
```

Change to:
```rst
:orphan:

pybullet_fleet
==============
```

### Step 3: Verify warning reduction

```bash
cd docs && rm -rf _build && python3 -m sphinx -b html . _build/html 2> /tmp/sw.txt
grep -c "duplicate object description" /tmp/sw.txt
# Expected: 0

grep -c "isn't included in any toctree" /tmp/sw.txt
# Expected: 0

tail -1 /tmp/sw.txt
# Expected: ~165 warnings remaining (238 - 72 duplicates - 1 orphan = 165)
```

### Step 4: Commit

```bash
git add docs/api/generated/pybullet_fleet.rst docs/api/generated/modules.rst
git commit -m "docs: add :noindex: to submodule autodoc, fix orphan modules.rst"
```

---

## Task 3: Fix Docstring RST Formatting — PARALLEL (8 files, independent)

All 8 Python files have the **same root cause**: Napoleon-parsed docstrings contain `Example:`, `Usage:`, `Note:`, or `Performance optimization:` sections followed by indented code, but rST doesn't recognize them as code blocks.

### Universal Fix Pattern

**Pattern A — `Example:` followed by indented code (~80% of all warnings):**

The fix is to convert `Example:` to `Example::` and ensure a blank line before the indented code. The `::` tells rST to treat the following indented block as a code literal.

Before (generates 2-4 warnings per occurrence):
```python
        Example:
            params = SimObjectSpawnParams(
                visual_shape=ShapeParams(...),
            )
```

After (zero warnings):
```python
        Example::

            params = SimObjectSpawnParams(
                visual_shape=ShapeParams(...),
            )
```

This applies to ALL of these docstring section headers:
- `Example:` → `Example::`
- `Example (Mesh):` → `Example (Mesh)::`
- `Example (URDF):` → `Example (URDF)::`
- `Example (Virtual Agent - invisible, no collision):` → `Example (Virtual Agent - invisible, no collision)::`
- `Usage:` → `Usage::`

**Pattern B — Bullet list under a definition-like heading:**

Before:
```python
        Comparison with sim_core.register_callback():
        - AgentManager callback: Provides manager reference
        - sim_core callback: Provides sim_core reference
```

After (add blank line to break definition-list interpretation):
```python
        Comparison with sim_core.register_callback():

        - AgentManager callback: Provides manager reference
        - sim_core callback: Provides sim_core reference
```

**Pattern C — Bullet list under a heading with colon:**

Before:
```python
        Performance optimization:
        - Kinematic objects: Always returns cached pose
        - Dynamic objects: Caches within same timestep
```

After:
```python
        Performance optimization:

        - Kinematic objects: Always returns cached pose
        - Dynamic objects: Caches within same timestep
```

**Pattern D — `**kwargs` / `**args` inline strong:**

Before:
```python
    Calls the callback function func(grid_index, world_pos, **args)
```

After:
```python
    Calls the callback function func(grid_index, world_pos, \*\*args)
```

Or:
```python
    Calls the callback function ``func(grid_index, world_pos, **args)``
```

---

### Task 3a: Fix `pybullet_fleet/sim_object.py` (44 warnings) — PARALLEL

**File:** `pybullet_fleet/sim_object.py`

Fix these docstrings (all Pattern A unless noted):

| Class/Method | Docstring line ref | Pattern |
|---|---|---|
| `ShapeParams` class | lines ~25-68 | A: `Example:` → `Example::` + blank line |
| `SimObjectSpawnParams` class | lines ~82-129 | A: `Example:` → `Example::` + blank line |
| `SimObjectSpawnParams.from_dict` | lines ~134-152 | A: `Example:` → `Example::` + blank line |
| `SimObject.create_shared_shapes` | lines ~281-310 | A: `Example:` → `Example::` + blank line |
| `SimObject.from_mesh` | lines ~531-563 | A: `Example:` → `Example::` + blank line |
| `SimObject.from_params` | lines ~615-638 | A: `Example:` → `Example::` + blank line |
| `SimObject.get_pose` | lines ~692-707 | C: `Performance optimization:` → add blank line after |
| `SimObject.attach_object` | lines ~889-917 | A: `Example:` → `Example::` + blank line |

**Step 1:** Search for all `Example:` in sim_object.py docstrings:
```bash
grep -n "Example:" pybullet_fleet/sim_object.py
```

**Step 2:** For each match, change `Example:` to `Example::` and ensure there is a blank line between the `Example::` line and the first indented code line. Do NOT change any code content.

**Step 3:** Find `Performance optimization:` heading and add blank line after it.

**Step 4:** Verify:
```bash
cd docs && rm -rf _build && python3 -m sphinx -b html . _build/html 2> /tmp/sw.txt
grep "sim_object.py" /tmp/sw.txt | grep -v "duplicate" | wc -l
# Expected: 0
```

**Step 5:** Commit:
```bash
git add pybullet_fleet/sim_object.py
git commit -m "docs: fix sim_object.py docstring RST formatting (44 warnings)"
```

---

### Task 3b: Fix `pybullet_fleet/agent.py` (42 warnings) — PARALLEL

**File:** `pybullet_fleet/agent.py`

Fix these docstrings:

| Class/Method | Pattern |
|---|---|
| `AgentSpawnParams.from_dict` | A: `Example:` → `Example::` + blank line |
| `Agent.from_params` | A: `Example (Mesh):` → `Example (Mesh)::` + blank line (3 Example blocks) |
| `Agent.from_mesh` | A: `Example:` → `Example::` + blank line |
| `Agent.set_path` | B: `Note:` sub-line has extra indentation — de-indent to match continuation lines |
| `Agent.add_action_sequence` | A: `Example:` → `Example::` + blank line |
| `Agent.are_joints_at_targets` | A: `Example:` → `Example::` + blank line |

**Step 1:** Search:
```bash
grep -n "Example\|Example (" pybullet_fleet/agent.py
```

**Step 2:** Apply Pattern A to all `Example` lines. For `Agent.set_path`, find the `Note:` sub-line under the `direction:` parameter and de-indent it to align with other continuation text.

**Step 3:** Verify:
```bash
cd docs && rm -rf _build && python3 -m sphinx -b html . _build/html 2> /tmp/sw.txt
grep "agent.py" /tmp/sw.txt | grep -v "duplicate\|agent_manager" | wc -l
# Expected: 0
```

**Step 4:** Commit:
```bash
git add pybullet_fleet/agent.py
git commit -m "docs: fix agent.py docstring RST formatting (42 warnings)"
```

---

### Task 3c: Fix `pybullet_fleet/agent_manager.py` (24 warnings) — PARALLEL

**File:** `pybullet_fleet/agent_manager.py`

Fix these docstrings:

| Class/Method | Pattern |
|---|---|
| `SimObjectManager.set_pose_all` | A: `Example:` → `Example::` + blank line |
| `SimObjectManager.spawn_grid_mixed` | A: `Example:` → `Example::` + blank line |
| `AgentManager.set_goal_pose_all` | A: `Example:` → `Example::` + blank line |
| `AgentManager.set_joints_targets_all` | A: `Example:` → `Example::` + blank line |
| `AgentManager.add_action_sequence_all` | A: `Example:` → `Example::` + blank line |
| `AgentManager.register_callback` | B: `Comparison with sim_core.register_callback():` → add blank line after |
| `AgentManager.setup_camera` | A: `Example:` → `Example::` + blank line |

**Step 1-4:** Same pattern as 3a/3b. Verify:
```bash
grep "agent_manager.py" /tmp/sw.txt | grep -v "duplicate" | wc -l
# Expected: 0
```

**Step 5:** Commit:
```bash
git add pybullet_fleet/agent_manager.py
git commit -m "docs: fix agent_manager.py docstring RST formatting (24 warnings)"
```

---

### Task 3d: Fix `pybullet_fleet/core_simulation.py` (22 warnings) — PARALLEL

**File:** `pybullet_fleet/core_simulation.py`

Search for all `Example:` in docstrings and apply Pattern A. Also look for any headings ending in `:` followed by indented content (Pattern B/C).

```bash
grep -n "Example:" pybullet_fleet/core_simulation.py
```

Verify:
```bash
grep "core_simulation.py" /tmp/sw.txt | grep -v "duplicate" | wc -l
# Expected: 0
```

Commit:
```bash
git add pybullet_fleet/core_simulation.py
git commit -m "docs: fix core_simulation.py docstring RST formatting (22 warnings)"
```

---

### Task 3e: Fix `pybullet_fleet/geometry.py` (16 warnings) — PARALLEL

**File:** `pybullet_fleet/geometry.py`

Same approach — find all `Example:` and apply Pattern A.

```bash
grep -n "Example:" pybullet_fleet/geometry.py
```

Verify:
```bash
grep "geometry.py" /tmp/sw.txt | grep -v "duplicate" | wc -l
# Expected: 0
```

Commit:
```bash
git add pybullet_fleet/geometry.py
git commit -m "docs: fix geometry.py docstring RST formatting (16 warnings)"
```

---

### Task 3f: Fix `pybullet_fleet/action.py` (10 warnings) — PARALLEL

**File:** `pybullet_fleet/action.py`

Same approach — find all `Example:` and apply Pattern A.

Verify + Commit:
```bash
git add pybullet_fleet/action.py
git commit -m "docs: fix action.py docstring RST formatting (10 warnings)"
```

---

### Task 3g: Fix `pybullet_fleet/tools.py` (2 warnings) — PARALLEL

**File:** `pybullet_fleet/tools.py`

Both warnings are Pattern D — `**args` in `grid_execution` docstring.

Find the function `grid_execution` and its docstring. Two occurrences of `**args`:
1. Summary line: `func(grid_index, world_pos, **args)`
2. Args section: `func: Callback function(..., **args)`

Escape both: `**args` → `\*\*args`

Verify:
```bash
grep "tools.py" /tmp/sw.txt | wc -l
# Expected: 0
```

Commit:
```bash
git add pybullet_fleet/tools.py
git commit -m "docs: escape **args in tools.py docstring (2 warnings)"
```

---

### Task 3h: Fix `pybullet_fleet/logging_utils.py` (2 warnings) — PARALLEL

**File:** `pybullet_fleet/logging_utils.py`

Two issues:

1. `LazyLogger` class docstring: `Usage:` → `Usage::` + blank line (Pattern A)
2. `LazyLogger.debug` docstring: `**kwargs` → `\*\*kwargs` (Pattern D)

Verify:
```bash
grep "logging_utils.py" /tmp/sw.txt | wc -l
# Expected: 0
```

Commit:
```bash
git add pybullet_fleet/logging_utils.py
git commit -m "docs: fix logging_utils.py docstring formatting (2 warnings)"
```

---

## Task 4: Fix Adjacent Transitions (1 warning) — SERIAL

**File:** `docs/architecture/collision-spatial-hash.md`

Line 243 has two consecutive `---` horizontal rules with nothing between them. Sphinx/MyST requires at least one body element between transitions.

### Step 1: Find and remove the duplicate

```bash
grep -n "^---$" docs/architecture/collision-spatial-hash.md | tail -5
```

Current text around lines 241-245:
```markdown
---

---

## See Also
```

Remove one of the two `---` lines (delete line 243, the second one).

Result:
```markdown
---

## See Also
```

### Step 2: Verify

```bash
cd docs && rm -rf _build && python3 -m sphinx -b html . _build/html 2> /tmp/sw.txt
grep "collision-spatial-hash" /tmp/sw.txt
# Expected: no output
```

### Step 3: Commit

```bash
git add docs/architecture/collision-spatial-hash.md
git commit -m "docs: remove duplicate transition in collision-spatial-hash.md"
```

---

## Task 5: Fix Broken Cross-References (2 warnings) — SERIAL

**File:** `benchmark/experiments/README.md`

Two relative links break when this file is included via `{include}` from `docs/benchmarking/experiments.md`:
- `[profiling/README.md](../profiling/README.md)` — resolves to `benchmark/profiling/README.md` (correct from benchmark/ but wrong from docs/benchmarking/)
- `[Benchmark Suite README](../README.md)` — resolves to `benchmark/README.md` (same issue)

### Step 1: Fix the two links

The file is on line 5:
```markdown
All scripts live in `benchmark/experiments/`. For profiling tools, see [`profiling/README.md`](../profiling/README.md). For overall benchmark results, see the [Benchmark Suite README](../README.md).
```

Replace with plain text references (no relative links) since the file is read both standalone on GitHub AND included in Sphinx:

```markdown
All scripts live in `benchmark/experiments/`. For profiling tools, see the Profiling Guide (`benchmark/profiling/README.md`). For overall benchmark results, see the Benchmark Suite README (`benchmark/README.md`).
```

Also check the last line which has: `See the [Profiling Guide](profiling-guide.md)` — this reference uses a docs-relative path and may work when included. Verify after build.

### Step 2: Verify

```bash
cd docs && rm -rf _build && python3 -m sphinx -b html . _build/html 2> /tmp/sw.txt
grep "cross-reference" /tmp/sw.txt
# Expected: no output
```

### Step 3: Commit

```bash
git add benchmark/experiments/README.md
git commit -m "docs: fix broken cross-references in experiments README"
```

---

## Task 6: Final Verification — SERIAL (depends on all above)

### Step 1: Clean build with -W flag

```bash
cd /home/rapyuta/rr_sim_evaluation/PyBulletFleet/docs
rm -rf _build
python3 -m sphinx -W -b html . _build/html
```

Expected: Build succeeds with exit code 0 (zero warnings — `-W` makes warnings fatal).

### Step 2: If warnings remain, diagnose

```bash
python3 -m sphinx -b html . _build/html 2> /tmp/sw.txt
grep -E "WARNING|ERROR" /tmp/sw.txt
```

Fix any remaining issues. Common surprises:
- New warnings from files that changed since the initial analysis
- Stale cache — always `rm -rf _build` before testing

### Step 3: Run existing tests to verify no docstring content changed

```bash
cd /home/rapyuta/rr_sim_evaluation/PyBulletFleet
python3 -m pytest tests/ -v --timeout=60
```

Expected: All tests pass (docstring changes are formatting-only, not content).

### Step 4: Spot-check HTML output

Open `docs/_build/html/index.html` in a browser and verify:
- Navigation tree has all sections
- API docs render correctly (autodoc content present)
- Benchmark pages render (included content appears)
- Example code blocks in docstrings render as code (not as block quotes)

### Step 5: Final commit

```bash
git add -A
git status  # Review — should only be docs/ and pybullet_fleet/ .py files
git commit -m "docs: achieve zero Sphinx warnings for RTD build"
```

---

## Task Dependencies

```
Task 1 (symlinks) ──────────┐
Task 2 (duplicates) ────────┤
Task 3a-3h (docstrings) ────┼── all independent ──→ Task 6 (verify)
Task 4 (transitions) ───────┤
Task 5 (cross-refs) ────────┘
      ↑ depends on Task 1 (symlinks must be replaced first for cross-ref context)
```

- **PARALLEL tasks:** Task 1, Task 2, Task 3a-3h, Task 4 (all independent of each other)
- **SERIAL dependency:** Task 5 should run after Task 1 (the cross-ref issue is visible only after symlinks are replaced with includes)
- **SERIAL dependency:** Task 6 must run after ALL other tasks

**Estimated effort:** ~30-45 minutes total

---

## Appendix: RTD Account Setup Guide

After all warnings are fixed and pushed to `main`:

1. Go to [readthedocs.org](https://readthedocs.org/) and sign in with GitHub
2. Click "Import a Project" → select `PyBulletFleet`
3. RTD auto-detects `.readthedocs.yaml` — no additional configuration needed
4. First build should succeed (zero warnings verified locally with `-W`)
5. Default URL will be: `https://pybulletfleet.readthedocs.io/`

The existing `.readthedocs.yaml` is already correctly configured:
```yaml
version: 2
build:
  os: ubuntu-24.04
  tools:
    python: "3.11"
sphinx:
  configuration: docs/conf.py
python:
  install:
    - requirements: docs/requirements-docs.txt
    - method: pip
      path: .
```
