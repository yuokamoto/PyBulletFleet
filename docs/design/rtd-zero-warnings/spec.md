# ReadTheDocs Zero Warnings Specification

**Date:** 2025-07-17
**Status:** Draft

## Context

The PyBulletFleet Sphinx documentation builds successfully but produces **238 warnings**. Three symlinks in `docs/benchmarking/` will break on ReadTheDocs (which clones via git and doesn't follow symlinks outside the repo tree). The RTD project has not yet been connected. Goal: zero warnings, working RTD deployment.

## Decision

Fix all 238 warnings in-place (docstrings + docs config + Markdown files), replace symlinks with MyST `{include}` directives, and provide RTD account setup guidance. No content restructuring — purely a build-health pass.

## Requirements

- Zero Sphinx build warnings (`sphinx-build -W` passes)
- ReadTheDocs builds succeed with identical output to local builds
- All 19 existing documentation pages render correctly
- Symlinked benchmark docs render on both local Sphinx and RTD
- No regression in rendered content or navigation

## Approach — 7 Fix Categories

| # | Category | Count | Fix |
|---|----------|-------|-----|
| 1 | Symlinks break on RTD | 3 | Replace with MyST `{include}` directives |
| 2 | Duplicate object descriptions | 72 | Add `:noindex:` to `api/generated/` RST module directives |
| 3 | Unexpected indentation | 94 | Fix docstring RST formatting (blank lines) |
| 4 | Block quote ends without blank line | 55 | Add trailing blank lines in docstrings |
| 5 | Definition list ends without blank line | 36 | Add trailing blank lines in docstrings |
| 6 | Inline strong start-string | 5 | Escape `**kwargs` → `\*\*kwargs` in docstrings |
| 7 | Misc (orphan, cross-refs, transitions) | 4 | Per-file targeted fixes |

### Execution Order

1. **Symlinks** (3 files) — highest risk, affects RTD fundamentally
2. **Duplicate descriptions** (1 RST file) — bulk warning reduction
3. **Docstrings** (9 Python files, ~185 warnings) — largest volume
4. **Misc** (3-4 files) — cleanup pass
5. **Verify** — `sphinx-build -W` must exit 0

## Constraints

- Docstring fixes must not change rendered API documentation content
- `{include}` paths must be relative from `docs/` root (both local and RTD)
- Cannot remove `__init__.py` re-exports (public API contract)
- Must preserve existing `api/generated/` autodoc structure

## Out of Scope

- Content writing or restructuring
- Adding new documentation pages
- Changing Sphinx theme or extensions
- CI/CD pipeline integration (beyond RTD setup guidance)

## Open Questions

- [ ] None — design fully explored and approved in conversation

## Success Criteria

- [ ] `cd docs && sphinx-build -W -b html . _build/html` exits with 0 warnings
- [ ] All 19 doc pages render correctly in local build
- [ ] RTD build succeeds after repository connection
- [ ] Benchmark docs (3 included files) render identically to current symlink behavior

## References

- [Sphinx MyST include directive](https://myst-parser.readthedocs.io/en/latest/syntax/roles-and-directives.html)
- [ReadTheDocs configuration](https://docs.readthedocs.io/en/stable/config-file/v2.html)
- [Existing RTD config](.readthedocs.yaml)
- [Prior docs migration plan](docs/design/docs-readthedocs/plan.md)
