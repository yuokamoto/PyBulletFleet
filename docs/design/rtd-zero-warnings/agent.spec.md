# ReadTheDocs Zero Warnings — Agent Specification

## Requirements

- Eliminate all 238 Sphinx build warnings achieving `sphinx-build -W` exit 0
- Replace 3 symlinks in `docs/benchmarking/` with MyST `{include}` directives
- Fix 72 duplicate object description warnings from autodoc re-exports
- Fix 185 docstring RST formatting warnings across 9 Python source files
- Fix 4 miscellaneous warnings (orphan doc, cross-refs, adjacent transitions, inline strong)
- ReadTheDocs must build successfully with no configuration changes beyond what's in repo

## Constraints

- Docstring changes must only affect RST formatting (blank lines, escaping), never content
- `pybullet_fleet/__init__.py` re-exports must remain unchanged (public API)
- `docs/api/generated/pybullet_fleet.rst` structure must be preserved (just add `:noindex:`)
- `{include}` paths are relative from the file's location in `docs/`
- The `.readthedocs.yaml` is already valid — no changes needed

## Approach

5-phase sequential execution. Each phase is independently verifiable by running `sphinx-build` and counting remaining warnings.

## Design

### Phase 1: Symlinks → MyST Include (3 files)

**What:** Delete symlinks, create real `.md` files with `{include}` directives.

**Files to modify:**
- `docs/benchmarking/benchmark-suite.md` — symlink → `../../benchmark/README.md`
- `docs/benchmarking/experiments.md` — symlink → `../../benchmark/experiments/README.md`
- `docs/benchmarking/profiling-guide.md` — symlink → `../../benchmark/profiling/README.md`

**Pattern:**
```bash
# Remove symlink
rm docs/benchmarking/benchmark-suite.md

# Create real file with include directive
cat > docs/benchmarking/benchmark-suite.md << 'EOF'
```{include} ../../benchmark/README.md
```
EOF
```

Repeat for all 3 files. The `{include}` path is relative to the file's directory.

**Cross-reference fix:** The included `benchmark/experiments/README.md` contains relative links `../profiling/README.md` and `../README.md` that become broken when included from `docs/benchmarking/`. These 2 warnings will be fixed by editing the source files in `benchmark/` to use MyST cross-reference targets instead of relative paths.

### Phase 2: Duplicate Object Descriptions (1 RST file, 72 warnings)

**What:** Add `:noindex:` option to every `automodule` directive in the generated RST.

**File:** `docs/api/generated/pybullet_fleet.rst`

**Root cause:** `__init__.py` re-exports symbols. Autodoc processes both `pybullet_fleet.Symbol` (from `__init__.py`) and `pybullet_fleet.module.Symbol` (from `api/generated/` submodule directives). Both create index entries → "duplicate object description" warning.

**Fix:** Add `:noindex:` to each `.. automodule::` block in `pybullet_fleet.rst`:

```rst
.. automodule:: pybullet_fleet.action
   :members:
   :undoc-members:
   :show-inheritance:
   :noindex:
```

This keeps the submodule docs rendered but prevents duplicate index entries. The top-level `pybullet_fleet` module (from `__init__.py`) remains the canonical index entry.

**Also fix orphan:** `docs/api/generated/modules.rst` — add `:orphan:` directive at top:

```rst
:orphan:

pybullet_fleet
==============
...
```

### Phase 3: Docstring RST Formatting (9 Python files, ~185 warnings)

**Files (by warning count):**

| File | Warnings | Issues |
|------|----------|--------|
| `pybullet_fleet/sim_object.py` | 59 | indentation, block quote, definition list |
| `pybullet_fleet/agent.py` | 42 | indentation, block quote, definition list |
| `pybullet_fleet/agent_manager.py` | 40 | indentation, block quote, definition list |
| `pybullet_fleet/types.py` | 35 | duplicate (handled in Phase 2) |
| `pybullet_fleet/geometry.py` | 22 | indentation, block quote |
| `pybullet_fleet/core_simulation.py` | 22 | indentation, block quote |
| `pybullet_fleet/action.py` | 10 | indentation, block quote |
| `pybullet_fleet/tools.py` | 2 | indentation |
| `pybullet_fleet/logging_utils.py` | 2 | indentation |

**Common patterns to fix:**

1. **Unexpected indentation** — Missing blank line before indented block in docstring:
```python
# BEFORE (warning)
    Args:
        x: The x value
            Must be positive

# AFTER (fixed)
    Args:
        x: The x value.
            Must be positive.
```

2. **Block quote ends without blank line** — Missing blank line after indented block:
```python
# BEFORE (warning)
        Default: 0.0
    Returns:

# AFTER (fixed)
        Default: 0.0

    Returns:
```

3. **Definition list ends without blank line** — Same pattern for definition lists.

4. **Inline strong start-string** (`**kwargs`):
```python
# BEFORE (warning)
    **kwargs: Additional arguments

# AFTER (fixed)
    \*\*kwargs: Additional arguments
```

**Strategy:** Run `sphinx-build 2>&1 | grep "WARNING"` after each file to verify warning count decreases. Fix files in descending warning count order.

### Phase 4: Miscellaneous Fixes (3 files, 4 warnings)

1. **Adjacent transitions** — `docs/architecture/collision-spatial-hash.md` line 243:
   - Two consecutive `---` with nothing between them
   - Fix: Remove the duplicate `---` (delete line 243)

2. **Broken cross-references** — `benchmark/experiments/README.md`:
   - `../profiling/README.md` → Replace with label reference or description text
   - `../README.md` → Replace with label reference or description text
   - These are relative links that break when the file is `{include}`d from `docs/benchmarking/`

### Phase 5: Verification

```bash
cd docs && sphinx-build -W -b html . _build/html 2>&1
```

Must exit with code 0 (zero warnings). `-W` flag turns warnings into errors.

## File References

Files the implementing agent MUST read before starting:

- `docs/conf.py` — Sphinx configuration (extensions, exclude patterns, autodoc settings)
- `.readthedocs.yaml` — RTD build configuration
- `docs/requirements-docs.txt` — Documentation dependencies
- `pybullet_fleet/__init__.py` — Re-exports causing duplicate warnings
- `docs/api/generated/pybullet_fleet.rst` — Autodoc RST with duplicate `automodule` directives
- `docs/api/generated/modules.rst` — Orphaned RST document
- `docs/benchmarking/index.md` — Toctree referencing symlinked files
- `docs/architecture/collision-spatial-hash.md:235-250` — Adjacent transitions area
- `benchmark/experiments/README.md` — Cross-reference source (lines with `../profiling/README.md` and `../README.md`)
- `pybullet_fleet/sim_object.py` — Highest warning count docstrings
- `pybullet_fleet/agent.py` — Second highest warning count docstrings
- `pybullet_fleet/agent_manager.py` — Third highest warning count docstrings

## Code Patterns

### MyST Include (Phase 1)

```markdown
```{include} ../../benchmark/README.md
```
```

### Autodoc noindex (Phase 2)

```rst
.. automodule:: pybullet_fleet.action
   :members:
   :undoc-members:
   :show-inheritance:
   :noindex:
```

### Docstring blank-line fix (Phase 3)

Napoleon-style Google docstrings. Every section (`Args:`, `Returns:`, `Raises:`, `Note:`, `Example:`) needs a blank line before AND after any indented block.

```python
def method(self, param: float = 0.0) -> bool:
    """Brief description.

    Args:
        param: Description of param.
            Extended description on next line.

    Returns:
        True if successful.

    Raises:
        ValueError: If param is negative.
    """
```

## Success Criteria

- [ ] `sphinx-build -W -b html docs docs/_build/html` exits 0 (zero warnings)
- [ ] All 19 documentation pages render correctly (spot-check TOC navigation)
- [ ] `docs/benchmarking/benchmark-suite.md` is a real file (not symlink) with `{include}`
- [ ] `docs/benchmarking/experiments.md` is a real file (not symlink) with `{include}`
- [ ] `docs/benchmarking/profiling-guide.md` is a real file (not symlink) with `{include}`
- [ ] `docs/api/generated/pybullet_fleet.rst` has `:noindex:` on all `automodule` directives
- [ ] `docs/api/generated/modules.rst` has `:orphan:` at top
- [ ] No double `---` in `collision-spatial-hash.md`
- [ ] `benchmark/experiments/README.md` has no broken relative links
