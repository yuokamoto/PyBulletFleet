# PyPI Release Process — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Enable `pip install pybullet-fleet` via PyPI with automated CI quality gates and release workflow.

**Architecture:** ci.yml extended with docs/security/license/coverage checks (PR gate), release.yml for tag-triggered PyPI publishing via Trusted Publisher OIDC, release.sh for local release orchestration, and a Copilot releasing skill for guided release execution.

**Tech Stack:** GitHub Actions, PyPI Trusted Publisher (OIDC), bash, setuptools build, twine, pip-audit, pip-licenses, Sphinx

---

### Task 1: Create CHANGELOG.md [PARALLEL]

**Files:**
- Create: `CHANGELOG.md`

**Step 1: Create the changelog template**

```markdown
# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]
```

**Step 2: Verify file is valid**

Run: `head -10 CHANGELOG.md`
Expected: Content as above

**Step 3: Commit**

```bash
git add CHANGELOG.md
git commit -m "chore: add CHANGELOG.md template"
```

---

### Task 2: Extend ci.yml with quality gates [PARALLEL]

**Files:**
- Modify: `.github/workflows/ci.yml`

**Step 1: Add `--cov-fail-under=75` to existing test job**

In the existing `test` job, change the pytest command:

```yaml
    - name: Run tests
      run: |
        pytest tests/ -v --tb=short --cov=pybullet_fleet --cov-report=term-missing --cov-fail-under=75
```

**Step 2: Add `docs` job for Sphinx -W build**

Add after the existing `test` job:

```yaml
  docs:
    name: Documentation Build
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v5

    - name: Set up Python 3.11
      uses: actions/setup-python@v5
      with:
        python-version: '3.11'

    - name: Install dependencies
      run: |
        python -m pip install --upgrade pip
        pip install -e ".[dev]"
        pip install -r docs/requirements-docs.txt

    - name: Build docs with warnings as errors
      run: |
        cd docs && sphinx-build -W -b html . _build/html
```

**Step 3: Add `security` job for secrets scan, pip-audit, license check**

```yaml
  security:
    name: Security and License Check
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v5

    - name: Set up Python 3.11
      uses: actions/setup-python@v5
      with:
        python-version: '3.11'

    - name: Install dependencies
      run: |
        python -m pip install --upgrade pip
        pip install -e .
        pip install pip-audit pip-licenses

    - name: Scan for secrets
      run: |
        FOUND=0
        grep -rn \
          -e 'AKIA[0-9A-Z]\{16\}' \
          -e 'sk-[a-zA-Z0-9]\{20,\}' \
          -e 'password\s*=\s*["\x27][^"\x27]\+["\x27]' \
          -e 'secret\s*=\s*["\x27][^"\x27]\+["\x27]' \
          --include='*.py' --include='*.yaml' --include='*.yml' --include='*.toml' \
          pybullet_fleet/ docs/ .github/ && FOUND=1 || true
        if [ "$FOUND" -eq 1 ]; then
          echo "::error::Potential secrets detected in source files"
          exit 1
        fi

    - name: Security audit
      run: pip-audit

    - name: License compatibility check
      run: |
        pip-licenses --format=csv --with-license-file --no-license-path > licenses.csv
        if grep -iE '(GNU General Public|GPL|AGPL|SSPL|EUPL|CC-BY-NC|CC-BY-ND)' licenses.csv; then
          echo "::error::Incompatible license detected"
          cat licenses.csv
          exit 1
        fi
        echo "All licenses compatible with Apache 2.0"
```

**Step 4: Verify the YAML is valid**

Run: `python -c "import yaml; yaml.safe_load(open('.github/workflows/ci.yml'))"`
Expected: No errors

**Step 5: Commit**

```bash
git add .github/workflows/ci.yml
git commit -m "ci: add docs build, security scan, license check, coverage threshold"
```

---

### Task 3: Create release.yml workflow [PARALLEL]

**Files:**
- Create: `.github/workflows/release.yml`

**Step 1: Create the release workflow**

```yaml
name: Release

on:
  push:
    tags:
      - 'v*'
  workflow_dispatch:
    inputs:
      version:
        description: 'Release version (e.g., 0.1.0)'
        required: true
        type: string

jobs:
  validate:
    name: Validate Release
    runs-on: ubuntu-latest
    outputs:
      version: ${{ steps.version.outputs.version }}
      tag: ${{ steps.version.outputs.tag }}
    steps:
      - uses: actions/checkout@v5
        with:
          fetch-depth: 0
      - name: Resolve version
        id: version
        run: |
          if [ "${{ github.event_name }}" = "workflow_dispatch" ]; then
            TAG="v${{ inputs.version }}"
          else
            TAG="${GITHUB_REF_NAME}"
          fi
          VERSION="${TAG#v}"
          echo "tag=${TAG}" >> "$GITHUB_OUTPUT"
          echo "version=${VERSION}" >> "$GITHUB_OUTPUT"
          if ! git rev-parse "refs/tags/${TAG}" >/dev/null 2>&1; then
            echo "::error::Tag ${TAG} does not exist"
            exit 1
          fi
          echo "Validated release: ${TAG} (version ${VERSION})"

  test:
    name: Test
    needs: validate
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v5
      - name: Set up Python 3.11
        uses: actions/setup-python@v5
        with:
          python-version: '3.11'
      - name: Install dependencies
        run: |
          python -m pip install --upgrade pip
          pip install -e ".[dev]"
      - name: Run tests
        run: pytest tests/ -v --tb=short

  build:
    name: Build and Verify Package
    needs: test
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v5
      - name: Set up Python 3.11
        uses: actions/setup-python@v5
        with:
          python-version: '3.11'
      - name: Install build tools
        run: pip install build twine
      - name: Build sdist and wheel
        run: python -m build
      - name: Verify package metadata
        run: twine check dist/*
      - name: Upload artifacts
        uses: actions/upload-artifact@v4
        with:
          name: dist
          path: dist/

  publish-pypi:
    name: Publish to PyPI
    needs: build
    runs-on: ubuntu-latest
    environment: release
    permissions:
      id-token: write
    steps:
      - name: Download artifacts
        uses: actions/download-artifact@v4
        with:
          name: dist
          path: dist/
      - name: Publish to PyPI
        uses: pypa/gh-action-pypi-publish@release/v1

  github-release:
    name: Create GitHub Release
    needs: [validate, publish-pypi]
    runs-on: ubuntu-latest
    permissions:
      contents: write
    steps:
      - uses: actions/checkout@v5
      - name: Extract changelog for this version
        run: |
          VERSION="${{ needs.validate.outputs.version }}"
          TAG="${{ needs.validate.outputs.tag }}"
          # Extract section between this version header and next version header
          awk "/^## v${VERSION}/,/^## (v[0-9]|\[Unreleased\])/" CHANGELOG.md | sed '$ d' > release_notes.md
          # Fallback if no changelog entry found
          if [ ! -s release_notes.md ]; then
            echo "Release ${TAG}" > release_notes.md
          fi
          echo "--- Release notes ---"
          cat release_notes.md
      - name: Create GitHub Release
        uses: softprops/action-gh-release@v2
        with:
          tag_name: ${{ needs.validate.outputs.tag }}
          body_path: release_notes.md
          generate_release_notes: false
```

**Step 2: Verify YAML is valid**

Run: `python -c "import yaml; yaml.safe_load(open('.github/workflows/release.yml'))"`
Expected: No errors

**Step 3: Commit**

```bash
git add .github/workflows/release.yml
git commit -m "ci: add release workflow with PyPI trusted publisher and GitHub releases"
```

---

### Task 4: Create release.sh helper script [PARALLEL]

**Files:**
- Create: `release.sh`

**Step 1: Create the release script**

```bash
#!/bin/bash
set -euo pipefail

# =============================================================================
# release.sh — PyBulletFleet Release Helper
#
# Usage: ./release.sh <version>
# Example: ./release.sh 0.1.0
#
# This script:
#   1. Validates preconditions (branch, clean tree, version match, no dup tag)
#   2. Generates release notes from conventional commits
#   3. Updates CHANGELOG.md
#   4. Commits, tags, and pushes
# =============================================================================

VERSION="${1:?Usage: ./release.sh <version> (e.g., 0.1.0)}"
TAG="v${VERSION}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }

# ---------------------------------------------------------------------------
# 1. Precondition checks
# ---------------------------------------------------------------------------
info "Checking preconditions..."

# Must be on main branch
BRANCH=$(git branch --show-current)
if [ "$BRANCH" != "main" ]; then
    error "Must be on 'main' branch (currently on '${BRANCH}')"
fi

# Working tree must be clean
if ! git diff --quiet HEAD 2>/dev/null; then
    error "Working tree is not clean. Commit or stash changes first."
fi

# Must be up to date with remote
git fetch origin main --quiet
LOCAL=$(git rev-parse HEAD)
REMOTE=$(git rev-parse origin/main)
if [ "$LOCAL" != "$REMOTE" ]; then
    error "Local main is not up to date with origin/main. Run 'git pull' first."
fi

# Version in pyproject.toml must match argument
TOML_VERSION=$(grep -oP '(?<=^version = ")[^"]+' pyproject.toml)
if [ "$TOML_VERSION" != "$VERSION" ]; then
    error "pyproject.toml version '${TOML_VERSION}' does not match '${VERSION}'. Update pyproject.toml first."
fi

# Tag must not already exist
if git rev-parse "refs/tags/${TAG}" >/dev/null 2>&1; then
    error "Tag '${TAG}' already exists"
fi

info "All preconditions passed"

# ---------------------------------------------------------------------------
# 2. Generate release notes
# ---------------------------------------------------------------------------
info "Generating release notes..."

generate_release_notes() {
    local prev_tag="$1"
    local version="$2"
    local date
    date=$(date +%Y-%m-%d)

    echo "## v${version} (${date})"
    echo ""

    local categories=("feat:Features" "fix:Bug Fixes" "docs:Documentation" "perf:Performance" "refactor:Refactoring" "test:Testing" "chore:Chores")
    local has_content=false

    for entry in "${categories[@]}"; do
        local prefix="${entry%%:*}"
        local title="${entry##*:}"
        local commits
        if [ -n "$prev_tag" ]; then
            commits=$(git log "${prev_tag}..HEAD" --pretty=format:"- %s" --grep="^${prefix}" --extended-regexp 2>/dev/null || true)
        else
            commits=$(git log --pretty=format:"- %s" --grep="^${prefix}" --extended-regexp 2>/dev/null || true)
        fi
        if [ -n "$commits" ]; then
            echo "### ${title}"
            echo ""
            echo "$commits"
            echo ""
            has_content=true
        fi
    done

    # Commits that don't match any conventional prefix
    local all_commits other
    if [ -n "$prev_tag" ]; then
        all_commits=$(git log "${prev_tag}..HEAD" --pretty=format:"%s" 2>/dev/null || true)
    else
        all_commits=$(git log --pretty=format:"%s" 2>/dev/null || true)
    fi
    other=$(echo "$all_commits" | grep -vE "^(feat|fix|docs|perf|refactor|test|chore)" || true)
    if [ -n "$other" ]; then
        echo "### Other Changes"
        echo ""
        echo "$other" | sed 's/^/- /'
        echo ""
        has_content=true
    fi

    if [ "$has_content" = false ]; then
        echo "Initial release."
        echo ""
    fi
}

# Find previous tag (empty string if none)
PREV_TAG=$(git describe --tags --abbrev=0 2>/dev/null || true)

NOTES=$(generate_release_notes "$PREV_TAG" "$VERSION")

# ---------------------------------------------------------------------------
# 3. Update CHANGELOG.md
# ---------------------------------------------------------------------------
info "Updating CHANGELOG.md..."

# Insert release notes after "## [Unreleased]" line
TMPFILE=$(mktemp)
awk -v notes="$NOTES" '
    /^## \[Unreleased\]/ {
        print
        print ""
        print notes
        next
    }
    { print }
' CHANGELOG.md > "$TMPFILE"
mv "$TMPFILE" CHANGELOG.md

# ---------------------------------------------------------------------------
# 4. Show release notes and confirm
# ---------------------------------------------------------------------------
echo ""
echo "=========================================="
echo "  Release Notes for ${TAG}"
echo "=========================================="
echo ""
echo "$NOTES"
echo "=========================================="
echo ""

read -rp "Proceed with release ${TAG}? [y/N] " CONFIRM
if [[ ! "$CONFIRM" =~ ^[yY]$ ]]; then
    # Restore CHANGELOG.md
    git checkout -- CHANGELOG.md
    info "Release cancelled"
    exit 0
fi

# ---------------------------------------------------------------------------
# 5. Commit, tag, and push
# ---------------------------------------------------------------------------
info "Committing and tagging..."

git add CHANGELOG.md
git commit -m "chore(release): v${VERSION}"
git tag -a "${TAG}" -m "Release ${TAG}"

info "Pushing to origin..."
git push origin main --tags

echo ""
info "Release ${TAG} pushed successfully!"
info "GitHub Actions will now build and publish to PyPI."
info "Monitor progress at: https://github.com/yuokamoto/PyBulletFleet/actions"
```

**Step 2: Make executable**

Run: `chmod +x release.sh`

**Step 3: Test precondition failures (dry run from non-main branch)**

Run: `bash release.sh 0.1.0 2>&1 | head -5`
Expected: Error about branch (currently on `readthedocs`, not `main`)

**Step 4: Commit**

```bash
git add release.sh
git commit -m "chore: add release.sh helper script"
```

---

### Task 5: Create releasing skill [PARALLEL]

**Files:**
- Create: `~/.copilot/skills/releasing/SKILL.md`

**Step 1: Create the skill file**

The skill should follow the structure of existing skills (e.g., `verification-before-completion`, `finishing-a-development-branch`). It orchestrates the full release process:

- Phase 1: Pre-flight checks (CI status, git state)
- Phase 2: Release-specific checks (benchmark, API compat, README version, docs)
- Phase 3: Execute release (run release.sh interactively)
- Phase 4: Post-release verification (PyPI, GitHub Release)

Key design points:
- Frontmatter `name: releasing`, `description: "Use when preparing and executing a release..."`
- Reference `benchmark/run_benchmark.py` for performance measurement
- Compare `__init__.py` `__all__` with previous tag for API compat
- Call `release.sh` and monitor output
- Check PyPI using `pip install --dry-run` after publish
- Performance results go into CHANGELOG.md release notes

**Step 2: Verify skill is discoverable**

Run: `ls ~/.copilot/skills/releasing/SKILL.md`
Expected: File exists

---

### Task 6: Verify full pipeline locally [SERIAL, depends on T1-T5]

**Step 1: Verify ci.yml YAML syntax**

Run: `python -c "import yaml; yaml.safe_load(open('.github/workflows/ci.yml'))"`
Expected: No error

**Step 2: Verify release.yml YAML syntax**

Run: `python -c "import yaml; yaml.safe_load(open('.github/workflows/release.yml'))"`
Expected: No error

**Step 3: Run tests to confirm nothing broken**

Run: `pytest tests/ -v --tb=short --cov=pybullet_fleet --cov-fail-under=75`
Expected: 555 passed, coverage >= 75%

**Step 4: Run Sphinx -W build**

Run: `cd docs && sphinx-build -W -b html . _build/html`
Expected: build succeeded, 0 warnings

**Step 5: Test release.sh precondition checks**

Run: `bash release.sh 0.1.0 2>&1` (from non-main branch)
Expected: Error about branch

**Step 6: Test package build locally**

Run: `pip install build twine && python -m build && twine check dist/*`
Expected: sdist and wheel created, twine check PASSED

**Step 7: Run pre-commit**

Run: `pre-commit run --all-files`
Expected: All hooks passed

**Step 8: Commit all remaining changes**

```bash
git add -A
git commit -m "chore: release pipeline setup complete"
```

---

### Task 7: Document manual GitHub/PyPI setup [SERIAL, depends on T6]

**Files:**
- The setup instructions are already in `docs/design/pypi-release/agent.spec.md` under "手動設定手順"
- No new files needed — just print the checklist for the user

**Step 1: Print the manual setup checklist**

After all code is committed and pushed, display this to the user:

```
Manual setup required (one-time):

1. PyPI Trusted Publisher:
   → https://pypi.org/manage/account/publishing/
   → Add pending publisher:
     - Project: pybullet-fleet
     - Owner: yuokamoto
     - Repo: PyBulletFleet
     - Workflow: release.yml
     - Environment: release

2. GitHub Environment:
   → Settings → Environments → New: "release"
   → Required reviewers: add yourself (optional)
   → Deployment branches/tags: Tags only, pattern "v*"

3. GitHub Tag Protection:
   → Settings → Rules → Rulesets → New
   → Target: Tags → "v*"
   → Bypass: Repository admin only
   → Restrict creations: ON

4. Required Status Checks (update):
   → Settings → Branches → main rule
   → Add: "Documentation Build", "Security and License Check"
```

---

## Task Dependencies

```
Task 1 (CHANGELOG.md)      ─┐
Task 2 (ci.yml)             ─┤
Task 3 (release.yml)        ─┼─→ Task 6 (verify) ─→ Task 7 (manual setup docs)
Task 4 (release.sh)         ─┤
Task 5 (releasing skill)    ─┘
```

- **PARALLEL:** Tasks 1, 2, 3, 4, 5 (no dependencies between them)
- **SERIAL:** Task 6 depends on all of 1-5; Task 7 depends on Task 6
