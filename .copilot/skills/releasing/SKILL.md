---
name: releasing
description: Use when preparing and executing a PyPI release - guides the full release workflow including pre-flight checks, benchmarks, API compatibility review, changelog generation, and post-release verification
---

# Releasing

## Overview

Guide the full release process from pre-flight checks through PyPI publication and post-release verification.

**Core principle:** Automated checks catch known issues; skill handles judgment calls and orchestration.

**Announce at start:** "I'm using the releasing skill to prepare this release."

## The Process

### Phase 1: Pre-flight Checks

```bash
# 1. Verify CI is green on main
# Check GitHub Actions status for the latest commit on main

# 2. Verify git state
git fetch origin
git status  # Must be on main, clean working tree, up to date

# 3. Verify version
grep 'version' pyproject.toml  # Must match intended release version
```

**If any check fails:** Stop and report. Do not proceed.

### Phase 2: Release-Specific Checks

These require judgment — CI can't fully automate them.

**2a. Performance Benchmark**

```bash
# Run benchmark suite
cd benchmark && python run_benchmark.py --config configs/general.yaml

# Compare with previous results in results/
# Look for:
# - >10% regression in any metric → WARN user
# - >25% regression → BLOCK release, require explanation
# - Improvements → note for release notes
```

Record results. Include summary in release notes.

**2b. API Compatibility Check**

```bash
# Compare public API with previous tag
git diff <prev-tag>..HEAD -- pybullet_fleet/__init__.py

# Check __all__ for:
# - Removed exports → BREAKING CHANGE (requires major version bump)
# - Added exports → Minor version feature
# - No changes → Patch version OK
```

Report any breaking changes to user before proceeding.

**2c. Documentation Check**

```bash
# Sphinx build (should already pass from CI, but verify)
cd docs && sphinx-build -W -b html . _build/html

# Check: any new public classes/functions missing docstrings?
# Compare __all__ additions with docstring coverage
```

**2d. README Version Check**

- Verify `pip install` examples show correct version
- Verify badge URLs are correct (if any)
- Verify getting started section is current

### Phase 3: Execute Release

#### Step 1: Run pre-release.sh (checks + draft)

```bash
./pre-release.sh <version>

# The script will:
# 1. Validate preconditions (branch, clean tree, version match, no dup tag)
# 2. Validate semver increment (+1 patch/minor/major from last tag)
# 3. Generate draft release notes from conventional commit subject lines
# 4. Write draft to CHANGELOG.md
# No interactive prompts — exits after writing the draft.
```

#### Step 2: Rewrite CHANGELOG (AI summarization)

The draft from `pre-release.sh` uses raw commit subject lines, which are often verbose.
**You MUST rewrite the CHANGELOG before proceeding.**

Rules for rewriting:
1. **Merge related commits** — Multiple commits for the same feature/fix become one entry
2. **User-facing language** — Rewrite from the user's perspective, not the developer's
3. **One line per change** — Each entry should be a single concise sentence
4. **Drop internal noise** — Remove chore/refactor entries unless they affect users
5. **Highlight what matters** — Breaking changes, new features, and performance improvements first
6. **Add benchmark results** — If Phase 2a found notable changes, add a Performance section

Example transformation:

```
BEFORE (raw commits):
### Documentation
- docs(sphinx): eliminate all 238 Sphinx warnings for RTD zero-warning build
- docs: restructure collision docs, add roadmap, improve cross-references
- docs: polish examples, how-to guides, and sidebar navigation
- docs: add examples tutorial section (spawn, actions, fleet)
- docs: update benchmark/README.md numbers to 2026-03-10 sweep data
- docs: use public pause()/resume() API instead of private _simulation_paused

AFTER (rewritten):
### Documentation
- Complete documentation overhaul: tutorials, how-to guides, API reference
- Zero Sphinx warnings — ReadTheDocs builds cleanly with `-W` flag
```

```
BEFORE (raw commits):
### Bug Fixes
- fix: add setuptools packages.find config and update CI actions
- fix: myst_fence_as_directive set→list, standardize collision method names to lowercase, remove speed fallback
- fix: address PR review comments (Round 2 & 3)
- fix: correct stale scenario name in argparse help text
- fix: suppress pyright false positive on R.random(random_state=)
- fix: resolve pyright CI failures

AFTER (rewritten):
### Bug Fixes
- Fix packaging configuration for pip install compatibility
- Standardize collision method names to lowercase
- Resolve CI and type-checking issues
```

#### Step 3: Present rewritten CHANGELOG to user

Show the rewritten version and ask:
- "Does this accurately capture the release? Any changes?"
- Apply user feedback

#### Step 4: Run publish-release.sh (commit, tag, push)

After user approves the rewritten CHANGELOG:
- Save the final CHANGELOG.md
- Run the publish script:

```bash
./publish-release.sh <version>

# The script will:
# 1. Re-validate preconditions (branch, version, no dup tag)
# 2. Verify only CHANGELOG.md is modified
# 3. Show release notes and ask for confirmation
# 4. Commit CHANGELOG.md, create annotated tag, push to origin
```

### Phase 4: Post-Release Verification

```bash
# 1. Monitor GitHub Actions
# Check: https://github.com/yuokamoto/PyBulletFleet/actions
# Wait for release.yml workflow to complete

# 2. Verify PyPI publication
pip install pybullet-fleet==<version> --dry-run

# 3. Verify GitHub Release
# Check: https://github.com/yuokamoto/PyBulletFleet/releases
# Confirm release notes are correct
```

**Report final status to user.**

## Checklist

Use this checklist for every release:

```
Pre-flight:
- [ ] CI green on main
- [ ] On main branch, clean, up to date
- [ ] pyproject.toml version matches target

Release checks:
- [ ] Benchmark run, no significant regressions
- [ ] API compatibility reviewed (no unintended breaking changes)
- [ ] Sphinx docs build clean
- [ ] README version references current
- [ ] CHANGELOG.md up to date

Execute:
- [ ] pre-release.sh run (checks passed, draft written)
- [ ] CHANGELOG rewritten (merged, simplified, user-facing)
- [ ] Rewritten CHANGELOG approved by user
- [ ] publish-release.sh run (committed, tagged, pushed)

Post-release:
- [ ] GitHub Actions release workflow completed
- [ ] Package available on PyPI
- [ ] GitHub Release created with notes
```

## Common Issues

| Issue | Solution |
|-------|----------|
| CI failing on main | Fix CI first, do not release with failures |
| Benchmark regression | Investigate cause, document if intentional, block if unintentional |
| Breaking API change | Bump major version, document migration in release notes |
| PyPI upload fails | Use workflow_dispatch to re-run: Actions → Release → Run workflow |
| Tag already exists | Cannot reuse tags; bump version and try again |
| Script fails on precondition | Fix the precondition (wrong branch, dirty tree, etc.) |

## Integration

**Called by:** User requesting a release
**Requires:** `pre-release.sh`, `publish-release.sh`, `.github/workflows/release.yml`, `CHANGELOG.md`
**Pairs with:** `verification-before-completion` (for final checks), `finishing-a-development-branch` (if releasing from feature branch)
