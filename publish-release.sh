#!/bin/bash
set -euo pipefail

# =============================================================================
# publish-release.sh — PyBulletFleet Commit, Tag & Push
#
# Usage: ./publish-release.sh <version>
# Example: ./publish-release.sh 0.2.0
#
# This script:
#   1. Re-validates critical preconditions (branch, version match, no dup tag)
#   2. Shows the release notes from CHANGELOG.md
#   3. Asks for confirmation
#   4. Commits CHANGELOG.md, creates annotated tag, pushes to origin
#
# Run this AFTER pre-release.sh and CHANGELOG review/rewrite.
# =============================================================================

VERSION="${1:-}"
if [ -z "$VERSION" ]; then
    echo "Usage: ./publish-release.sh <version>" >&2
    echo "Example: ./publish-release.sh 0.2.0" >&2
    exit 1
fi

TAG="v${VERSION}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }

# ---------------------------------------------------------------------------
# 1. Re-validate critical preconditions
# ---------------------------------------------------------------------------
info "Validating preconditions..."

# Must be on main branch
BRANCH=$(git branch --show-current)
if [ "$BRANCH" != "main" ]; then
    error "Must be on 'main' branch (currently on '${BRANCH}')"
fi

# Only CHANGELOG.md should be modified (check both staged and unstaged)
DIRTY_UNSTAGED=$(git diff --name-only 2>/dev/null | grep -v '^CHANGELOG.md$' || true)
DIRTY_STAGED=$(git diff --cached --name-only 2>/dev/null | grep -v '^CHANGELOG.md$' || true)
DIRTY_FILES="${DIRTY_UNSTAGED}${DIRTY_STAGED}"
if [ -n "$DIRTY_FILES" ]; then
    error "Unexpected uncommitted changes:\n${DIRTY_FILES}\nOnly CHANGELOG.md should be modified."
fi

# CHANGELOG.md must actually be modified (staged or unstaged)
if git diff --quiet -- CHANGELOG.md 2>/dev/null && git diff --cached --quiet -- CHANGELOG.md 2>/dev/null; then
    error "CHANGELOG.md has no changes. Run pre-release.sh first."
fi

# Version in pyproject.toml must match
TOML_VERSION=$(grep -oP '(?<=^version = ")[^"]+' pyproject.toml)
if [ "$TOML_VERSION" != "$VERSION" ]; then
    error "pyproject.toml version '${TOML_VERSION}' does not match '${VERSION}'."
fi

# Tag must not already exist
if git rev-parse "refs/tags/${TAG}" >/dev/null 2>&1; then
    error "Tag '${TAG}' already exists"
fi

info "All preconditions passed"

# ---------------------------------------------------------------------------
# 2. Show release notes and confirm
# ---------------------------------------------------------------------------
NOTES_DISPLAY=$(awk '/^## v'"${VERSION}"'/,/^## (v[0-9]|\[Unreleased\])/' CHANGELOG.md | sed '$ d')
if [ -z "$NOTES_DISPLAY" ]; then
    NOTES_DISPLAY="(No release notes found for v${VERSION} in CHANGELOG.md)"
fi

echo ""
echo "=========================================="
echo "  Release Notes for ${TAG}"
echo "=========================================="
echo ""
echo "$NOTES_DISPLAY"
echo "=========================================="
echo ""

read -rp "Proceed with release ${TAG}? [y/N] " CONFIRM
if [[ ! "$CONFIRM" =~ ^[yY]$ ]]; then
    info "Release cancelled. CHANGELOG.md changes preserved."
    exit 0
fi

# ---------------------------------------------------------------------------
# 3. Commit, tag, and push
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
