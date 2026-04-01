# Docs ReadTheDocs Migration — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Restructure all PyBulletFleet markdown documentation into a Sphinx + MyST ReadTheDocs-compatible site with proper hierarchy, deduplication, and English-only content.

**Architecture:** Use Sphinx with `myst-parser` to keep all docs as Markdown (no reStructuredText conversion). Existing `.md` files are reorganized into `docs/` subdirectories matching RTD sections. An `index.md` + `conf.py` at `docs/` root drives the build. Archive stale/session docs to `docs/archive/`. Delete empty files.

**Tech Stack:** Sphinx, myst-parser, sphinx-rtd-theme, sphinx-autodoc-typehints, sphinx-copybutton

---

## Current State

**35 markdown files** across 6 locations:
- `./README.md`, `./DESIGN.md` (repo root)
- `./docs/` (12 files — design docs, guides, profiling results)
- `./docs_conversation/` (8 files — session dumps, migration plans)
- `./benchmark/` (10 files — benchmark suite docs)
- `./config/` (1 file)
- `./tests/` (1 file)

**Problems:**
1. No unified TOC or navigation — scattered flat files
2. Duplicate/overlapping content (3 performance reports, 2 profiling guides, 2 config guides)
3. 5 empty placeholder files (0 lines)
4. Mixed Japanese/English with no consistency
5. Session conversation dumps mixed with intentional documentation
6. No build system — can't generate a browsable site

## Target Structure

```
docs/
├── conf.py                        # Sphinx config
├── index.md                       # Landing page (project intro)
├── getting-started/
│   └── quickstart.md              # Installation + first sim
├── architecture/
│   ├── index.md                   # Section index
│   ├── overview.md                # System architecture (from DESIGN.md)
│   ├── collision-detection.md     # Collision subsystem (from COLLISION_DETECTION_DESIGN_v3.md)
│   └── realtime-sync.md           # RT sync design (from REALTIME_SYNC_DESIGN.md)
├── how-to/
│   ├── index.md                   # Section index
│   ├── memory-profiling.md        # Memory profiling guide
│   ├── logging.md                 # LazyLogger usage
│   └── spatial-hash-config.md     # Cell size mode configuration
├── configuration/
│   ├── index.md                   # Section index
│   └── reference.md               # YAML config reference (from config/README.md)
├── benchmarking/
│   ├── index.md                   # Section index
│   ├── optimization-guide.md      # Perf optimization guide
│   ├── results.md                 # Benchmark results (merged)
│   ├── profiling.md               # Profiling tools (merged + translated)
│   ├── configs.md                 # Benchmark config files
│   └── experiments.md             # Experiment scripts
├── testing/
│   ├── index.md                   # Section index
│   └── overview.md                # Test guide (translated)
├── api/
│   └── index.md                   # Autodoc placeholder
├── design/                        # Kept as-is (agent working docs)
│   ├── core-simulation-tests/
│   └── pybullet-fleet-skills/
│   └── docs-readthedocs/          # This plan
└── archive/                       # Not in toctree, kept for reference
    ├── development-history.md
    ├── test-action-plan.md
    ├── collision-test-design.md
    ├── agent-profiling-results.md
    ├── optimization-results.md
    ├── performance-analysis.md
    └── conversation/              # Entire docs_conversation/ content
```

---

## Task 1: Sphinx Bootstrap (SERIAL)

**Files:**
- Create: `docs/conf.py`
- Create: `docs/index.md`
- Create: `docs/requirements-docs.txt`
- Modify: `requirements-dev.txt` (add doc deps)

**Step 1: Create docs requirements file**

```
# docs/requirements-docs.txt
sphinx>=7.0
sphinx-rtd-theme>=2.0
sphinx-autodoc-typehints>=2.0
sphinx-copybutton>=0.5
myst-parser>=3.0
```

**Step 2: Create `docs/conf.py`**

```python
import os
import sys
sys.path.insert(0, os.path.abspath('..'))

project = 'PyBulletFleet'
copyright = '2026, Yu Okamoto'
author = 'Yu Okamoto'
release = '0.1.0'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.intersphinx',
    'sphinx_autodoc_typehints',
    'sphinx_copybutton',
    'myst_parser',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'archive', 'design']

html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'navigation_depth': 3,
    'collapse_navigation': False,
}

myst_enable_extensions = [
    'colon_fence',
    'deflist',
    'tasklist',
]
myst_heading_anchors = 3

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = True

autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'undoc-members': True,
}

intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
    'pybullet': ('https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA', None),
}
```

**Step 3: Create `docs/index.md`**

Root index with toctree pointing to all sections.

**Step 4: Verify Sphinx build**

Run: `cd docs && sphinx-build -b html . _build/html`
Expected: Build succeeds with warnings only (no errors)

**Step 5: Commit**

```bash
git add docs/conf.py docs/index.md docs/requirements-docs.txt
git commit -m "docs: bootstrap Sphinx with MyST for ReadTheDocs"
```

---

## Task 2: Delete Empty & Obsolete Files (PARALLEL)

**Files to delete:**
- `docs/AGENT_PROFILING_RESULTS_EN.md` (0 lines, empty placeholder)
- `docs/OPTIMIZATION_RESULTS_EN.md` (0 lines, empty placeholder)
- `docs_conversation/MEMORY_ANALYSIS_RESULTS.md` (0 lines)
- `docs_conversation/SYSTEM_PERFORMANCE_ANALYSIS.md` (0 lines)
- `docs_conversation/SYSTEM_RESOURCE_EVIDENCE.md` (0 lines)
- `benchmark/archive/CONFIG_GUIDE.md` (superseded by `benchmark/configs/README.md`)
- `benchmark/archive/PERFORMANCE_REPORT_old.md` (superseded by `benchmark/PERFORMANCE_REPORT.md`)

**Step 1: Delete files**

```bash
git rm docs/AGENT_PROFILING_RESULTS_EN.md \
       docs/OPTIMIZATION_RESULTS_EN.md \
       docs_conversation/MEMORY_ANALYSIS_RESULTS.md \
       docs_conversation/SYSTEM_PERFORMANCE_ANALYSIS.md \
       docs_conversation/SYSTEM_RESOURCE_EVIDENCE.md \
       benchmark/archive/CONFIG_GUIDE.md \
       benchmark/archive/PERFORMANCE_REPORT_old.md
```

**Step 2: Commit**

```bash
git commit -m "docs: remove empty placeholders and superseded files"
```

---

## Task 3: Archive Session & Planning Docs (PARALLEL)

Move files that are reference-only (conversation dumps, point-in-time plans) into `docs/archive/`.

**Step 1: Create archive directories**

```bash
mkdir -p docs/archive/conversation
```

**Step 2: Move docs_conversation/ content**

```bash
git mv docs_conversation/COMPREHENSIVE_REVIEW_AND_IMPROVEMENTS.md docs/archive/conversation/
git mv docs_conversation/REFACTORING_SUMMARY.md docs/archive/conversation/
git mv docs_conversation/SPHINX_MIGRATION_PLAN.md docs/archive/conversation/
git mv docs_conversation/MEMORY_PROFILING_IMPROVEMENTS.md docs/archive/conversation/
git mv docs_conversation/README.md docs/archive/conversation/
```

**Step 3: Move planning/historical docs**

```bash
git mv docs/DEVELOPMENT_HISTORY.md docs/archive/development-history.md
git mv docs/TEST_ACTION_PLAN.md docs/archive/test-action-plan.md
git mv docs/COLLISION_TEST_DESIGN.md docs/archive/collision-test-design.md
```

**Step 4: Remove empty docs_conversation/ directory**

```bash
rmdir docs_conversation
```

**Step 5: Commit**

```bash
git commit -m "docs: archive session logs and planning docs"
```

---

## Task 4: Create Getting Started Section (SERIAL, depends on Task 1)

**Files:**
- Create: `docs/getting-started/quickstart.md`
- Modify: `README.md` (slim down, point to docs)

**Step 1: Create quickstart.md**

Extract Quick Start, installation, first simulation, and basic usage from `README.md`. Content should cover:
- Installation (`pip install -e .`)
- Running first example (`python examples/basics/robot_demo.py`)
- YAML config basics
- Link to configuration reference for details

**Step 2: Slim README.md**

Keep README.md as a concise project overview (motivation, features table, quick install command, link to full docs). Remove detailed sections that now live in RTD:
- Remove detailed configuration section (→ `configuration/reference.md`)
- Remove detailed architecture section (→ `architecture/overview.md`)
- Add "📖 **Full Documentation:** https://pybulletfleet.readthedocs.io" link

**Step 3: Verify Sphinx build**

Run: `cd docs && sphinx-build -b html . _build/html`
Expected: PASS

**Step 4: Commit**

```bash
git add docs/getting-started/ README.md
git commit -m "docs: add getting-started section, slim README"
```

---

## Task 5: Create Architecture Section (SERIAL, depends on Task 1)

**Files:**
- Create: `docs/architecture/index.md`
- Move+edit: `DESIGN.md` → `docs/architecture/overview.md`
- Move: `docs/COLLISION_DETECTION_DESIGN_v3.md` → `docs/architecture/collision-detection.md`
- Move: `docs/REALTIME_SYNC_DESIGN.md` → `docs/architecture/realtime-sync.md`

**Step 1: Create section index**

```markdown
# Architecture

Overview of PyBulletFleet's system design.

```{toctree}
:maxdepth: 2
overview
collision-detection
realtime-sync
```
```

**Step 2: Move and adapt DESIGN.md**

```bash
git mv DESIGN.md docs/architecture/overview.md
```

Edit to add MyST-compatible heading as first line. Remove any redundancy with README. Keep ASCII diagrams.

**Step 3: Move collision and RT sync docs**

```bash
git mv docs/COLLISION_DETECTION_DESIGN_v3.md docs/architecture/collision-detection.md
git mv docs/REALTIME_SYNC_DESIGN.md docs/architecture/realtime-sync.md
```

**Step 4: Verify Sphinx build**

Run: `cd docs && sphinx-build -b html . _build/html`
Expected: PASS

**Step 5: Commit**

```bash
git add docs/architecture/
git commit -m "docs: create architecture section with design, collision, and RT sync"
```

---

## Task 6: Create How-to Guides Section (SERIAL, depends on Task 1)

**Files:**
- Create: `docs/how-to/index.md`
- Move: `docs/MEMORY_PROFILING_GUIDE.md` → `docs/how-to/memory-profiling.md`
- Move: `docs/LOGGING_UTILS.md` → `docs/how-to/logging.md`
- Move: `docs/spatial_hash_cell_size_modes.md` → `docs/how-to/spatial-hash-config.md`

**Step 1: Create section index**

```markdown
# How-to Guides

Practical guides for common tasks.

```{toctree}
:maxdepth: 2
memory-profiling
logging
spatial-hash-config
```
```

**Step 2: Move files**

```bash
mkdir -p docs/how-to
git mv docs/MEMORY_PROFILING_GUIDE.md docs/how-to/memory-profiling.md
git mv docs/LOGGING_UTILS.md docs/how-to/logging.md
git mv docs/spatial_hash_cell_size_modes.md docs/how-to/spatial-hash-config.md
```

**Step 3: Verify Sphinx build and commit**

```bash
cd docs && sphinx-build -b html . _build/html
git add docs/how-to/
git commit -m "docs: create how-to section with memory profiling, logging, spatial hash guides"
```

---

## Task 7: Create Configuration Section (SERIAL, depends on Task 1)

**Files:**
- Create: `docs/configuration/index.md`
- Move: `config/README.md` → `docs/configuration/reference.md`

**Step 1: Create section index and move config docs**

```bash
mkdir -p docs/configuration
```

Create `docs/configuration/index.md` with toctree. Copy content from `config/README.md` to `docs/configuration/reference.md` (keep `config/README.md` as short pointer to docs).

**Step 2: Verify and commit**

```bash
git add docs/configuration/
git commit -m "docs: create configuration reference section"
```

---

## Task 8: Create Benchmarking Section (SERIAL, depends on Task 1)

This is the largest section — merge overlapping docs, translate Japanese.

**Files:**
- Create: `docs/benchmarking/index.md`
- Move: `benchmark/PERFORMANCE_OPTIMIZATION_GUIDE.md` → `docs/benchmarking/optimization-guide.md`
- Merge: `benchmark/PERFORMANCE_REPORT.md` + `benchmark/COLLISION_BENCHMARK_RESULTS.md` → `docs/benchmarking/results.md`
- Merge: `benchmark/profiling/README.md` + `benchmark/profiling/PROFILING_GUIDE.md` → `docs/benchmarking/profiling.md` (translate JP→EN)
- Move+translate: `benchmark/configs/README.md` → `docs/benchmarking/configs.md`
- Move: `benchmark/experiments/README.md` → `docs/benchmarking/experiments.md`

**Step 1: Create section index**

**Step 2: Move optimization guide**

```bash
mkdir -p docs/benchmarking
git mv benchmark/PERFORMANCE_OPTIMIZATION_GUIDE.md docs/benchmarking/optimization-guide.md
```

**Step 3: Merge performance results**

Create `docs/benchmarking/results.md` combining:
- Hardware specs + RTF/timing/memory tables from `PERFORMANCE_REPORT.md`
- Collision benchmark (physics ON vs OFF) from `COLLISION_BENCHMARK_RESULTS.md`

**Step 4: Merge profiling guides (translate)**

Create `docs/benchmarking/profiling.md` combining:
- `benchmark/profiling/README.md` (tool descriptions — translate Japanese portions)
- `benchmark/profiling/PROFILING_GUIDE.md` (methodology — translate Japanese portions)

**Step 5: Move+translate benchmark configs**

Copy+translate `benchmark/configs/README.md` → `docs/benchmarking/configs.md`

**Step 6: Move experiments**

```bash
git mv benchmark/experiments/README.md docs/benchmarking/experiments.md
```

**Step 7: Archive merged originals**

Move the source files that were merged (not moved) to `docs/archive/benchmark/` to preserve git history:

```bash
mkdir -p docs/archive/benchmark
git mv benchmark/PERFORMANCE_REPORT.md docs/archive/benchmark/
git mv benchmark/COLLISION_BENCHMARK_RESULTS.md docs/archive/benchmark/
git mv benchmark/profiling/README.md docs/archive/benchmark/profiling-readme.md
git mv benchmark/profiling/PROFILING_GUIDE.md docs/archive/benchmark/
```

**Step 8: Also archive remaining performance analysis docs**

```bash
git mv docs/PERFORMANCE_ANALYSIS.md docs/archive/performance-analysis.md
git mv docs/AGENT_PROFILING_RESULTS.md docs/archive/agent-profiling-results.md
git mv docs/OPTIMIZATION_RESULTS.md docs/archive/optimization-results.md
```

**Step 9: Update benchmark/README.md**

Update `benchmark/README.md` to point to the new `docs/benchmarking/` section for documentation, keeping only script usage as inline reference.

**Step 10: Verify and commit**

```bash
cd docs && sphinx-build -b html . _build/html
git add docs/benchmarking/ docs/archive/ benchmark/
git commit -m "docs: create benchmarking section with merged results and translated profiling guides"
```

---

## Task 9: Create Testing Section (SERIAL, depends on Task 1)

**Files:**
- Create: `docs/testing/index.md`
- Create: `docs/testing/overview.md` (translated from `tests/README.md`)

**Step 1: Create section index**

**Step 2: Translate tests/README.md to English**

Create `docs/testing/overview.md` — English version of test directory guide. Keep `tests/README.md` as short pointer to docs.

**Step 3: Verify and commit**

```bash
git add docs/testing/
git commit -m "docs: create testing section with translated overview"
```

---

## Task 10: Create API Reference Placeholder (SERIAL, depends on Task 1)

**Files:**
- Create: `docs/api/index.md`

**Step 1: Create API index with autodoc stubs**

```markdown
# API Reference

Auto-generated from source code docstrings.

```{toctree}
:maxdepth: 2
```

```{eval-rst}
.. autosummary::
   :toctree: generated
   :recursive:

   pybullet_fleet
```
```

**Step 2: Verify and commit**

```bash
git add docs/api/
git commit -m "docs: add API reference placeholder with autodoc"
```

---

## Task 11: Add .readthedocs.yaml (SERIAL, depends on Task 1)

**Files:**
- Create: `.readthedocs.yaml`
- Create: `docs/Makefile`
- Modify: `.gitignore` (add `docs/_build/`)

**Step 1: Create `.readthedocs.yaml`**

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

**Step 2: Create `docs/Makefile`**

Standard Sphinx Makefile for local builds.

**Step 3: Update .gitignore**

Add `docs/_build/` to `.gitignore`.

**Step 4: Verify full build**

```bash
cd docs && sphinx-build -b html . _build/html -W
```

`-W` treats warnings as errors. Fix any broken cross-references.

**Step 5: Commit**

```bash
git add .readthedocs.yaml docs/Makefile .gitignore
git commit -m "docs: add ReadTheDocs config and Makefile"
```

---

## Task 12: Final Cleanup & Link Verification (SERIAL, depends on Tasks 2-11)

**Step 1: Verify no broken internal links**

```bash
cd docs && sphinx-build -b linkcheck . _build/linkcheck
```

**Step 2: Verify all old docs are accounted for**

Run a script:
```bash
find . -name "*.md" -not -path "./.git/*" -not -path "./.copilot/*" -not -path "./.pytest_cache/*" | sort
```

Every `.md` should be either:
- In `docs/` toctree (published)
- In `docs/archive/` (preserved, excluded from build)
- In `docs/design/` (working docs, excluded from build)
- `README.md` (repo root, slimmed)
- `benchmark/README.md` (kept as script usage reference)
- `config/README.md` (kept as short pointer)
- `tests/README.md` (kept as short pointer)

**Step 3: Open and browse locally**

```bash
python -m http.server 8000 -d docs/_build/html
```

**Step 4: Final commit**

```bash
git add -A
git commit -m "docs: complete ReadTheDocs migration"
```

---

## Task Dependencies

```
Task 1 (Sphinx bootstrap) ←─── all other tasks depend on this
  ├── Task 2 (delete empties)          PARALLEL
  ├── Task 3 (archive sessions)        PARALLEL
  ├── Task 4 (getting-started)         SERIAL after 1
  ├── Task 5 (architecture)            SERIAL after 1
  ├── Task 6 (how-to)                  SERIAL after 1
  ├── Task 7 (configuration)           SERIAL after 1
  ├── Task 8 (benchmarking)            SERIAL after 1 (largest task)
  ├── Task 9 (testing)                 SERIAL after 1
  ├── Task 10 (API ref)               SERIAL after 1
  └── Task 11 (.readthedocs.yaml)     SERIAL after 1
Task 12 (final cleanup) ←─── depends on ALL above
```

**PARALLEL tasks:** 2, 3 (no dependencies on each other)
**PARALLEL tasks after Task 1:** 4, 5, 6, 7, 8, 9, 10, 11 (independent sections)
**SERIAL gate:** Task 12 waits for all

---

## File Disposition Summary

| Action | Files | Count |
|--------|-------|-------|
| **Move to RTD section** | DESIGN.md, COLLISION_DETECTION_DESIGN_v3.md, REALTIME_SYNC_DESIGN.md, MEMORY_PROFILING_GUIDE.md, LOGGING_UTILS.md, spatial_hash_cell_size_modes.md, PERFORMANCE_OPTIMIZATION_GUIDE.md | 7 |
| **Merge into RTD** | PERFORMANCE_REPORT.md + COLLISION_BENCHMARK_RESULTS.md → results.md; profiling/README.md + PROFILING_GUIDE.md → profiling.md | 4 → 2 |
| **Translate + move** | benchmark/configs/README.md, benchmark/profiling/* (JP portions), tests/README.md | 3 |
| **Archive** | DEVELOPMENT_HISTORY.md, TEST_ACTION_PLAN.md, COLLISION_TEST_DESIGN.md, PERFORMANCE_ANALYSIS.md, AGENT_PROFILING_RESULTS.md, OPTIMIZATION_RESULTS.md, entire docs_conversation/ (5 files) | 11 |
| **Delete** | 5 empty files + 2 superseded files | 7 |
| **Keep as-is** | README.md (slimmed), benchmark/README.md, config/README.md, tests/README.md, docs/design/* | 7+ |
| **Create new** | conf.py, index.md, 8 section indexes, quickstart.md, results.md, profiling.md, testing/overview.md, .readthedocs.yaml, Makefile, requirements-docs.txt | ~15 |
