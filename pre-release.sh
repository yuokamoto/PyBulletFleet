#!/bin/bash
set -euo pipefail

# =============================================================================
# pre-release.sh — PyBulletFleet Pre-Release Checks & Draft CHANGELOG
#
# Usage: ./pre-release.sh <version>
# Example: ./pre-release.sh 0.2.0
#
# This script:
#   1. Validates preconditions (branch, clean tree, version match, no dup tag)
#   2. Validates semver increment (patch+1, minor+1, or major+1 from last tag)
#   3. Generates draft release notes from conventional commits
#   4. Updates CHANGELOG.md with the draft
#
# After running this script, review and rewrite CHANGELOG.md, then run
# publish-release.sh to commit, tag, and push.
# =============================================================================

VERSION="${1:-}"
if [ -z "$VERSION" ]; then
    echo "Usage: ./pre-release.sh <version>" >&2
    echo "" >&2

    # Show latest tag and suggest next versions
    LATEST_TAG=$(git tag --sort=-v:refname | head -1 || true)
    if [ -n "$LATEST_TAG" ]; then
        LATEST="${LATEST_TAG#v}"
        IFS='.' read -r M m p <<< "$LATEST"
        echo "Latest tag: ${LATEST_TAG}" >&2
        echo "" >&2
        echo "Next version candidates:" >&2
        echo "  patch:  ${M}.${m}.$((p + 1))" >&2
        echo "  minor:  ${M}.$((m + 1)).0" >&2
        echo "  major:  $((M + 1)).0.0" >&2

        # Show recent tags for context
        RECENT=$(git tag --sort=-v:refname | head -5)
        if [ "$(echo "$RECENT" | wc -l)" -gt 1 ]; then
            echo "" >&2
            echo "Recent tags:" >&2
            echo "$RECENT" | sed 's/^/  /' >&2
        fi
    else
        echo "No existing tags found. This will be the first release." >&2
        echo "Example: ./pre-release.sh 0.1.0" >&2
    fi
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
# 1. Precondition checks
# ---------------------------------------------------------------------------
info "Checking preconditions..."

# Must be on main branch
BRANCH=$(git branch --show-current)
if [ "$BRANCH" != "main" ]; then
    error "Must be on 'main' branch (currently on '${BRANCH}')"
fi

# Working tree must be clean (no modified, staged, or untracked files)
if [ -n "$(git status --porcelain)" ]; then
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
TOML_VERSION=$(awk -F'"' '/^version[[:space:]]*=/ { print $2; exit }' pyproject.toml)
if [ "$TOML_VERSION" != "$VERSION" ]; then
    error "pyproject.toml version '${TOML_VERSION}' does not match '${VERSION}'. Update pyproject.toml first."
fi

# Tag must not already exist
if git rev-parse "refs/tags/${TAG}" >/dev/null 2>&1; then
    error "Tag '${TAG}' already exists"
fi

info "All preconditions passed"

# ---------------------------------------------------------------------------
# 2. Semver validation
# ---------------------------------------------------------------------------
info "Validating semantic version increment..."

PREV_TAG=$(git tag --sort=-v:refname | head -1 || true)

# Validate version format unconditionally (MAJOR.MINOR.PATCH, integers only)
if ! echo "$VERSION" | grep -qE '^[0-9]+\.[0-9]+\.[0-9]+$'; then
    error "Invalid version format '${VERSION}'. Expected MAJOR.MINOR.PATCH (e.g., 0.1.0)"
fi

if [ -n "$PREV_TAG" ]; then
    # Strip leading 'v' from previous tag
    PREV_VERSION="${PREV_TAG#v}"

    # Parse previous version
    IFS='.' read -r PREV_MAJOR PREV_MINOR PREV_PATCH <<< "$PREV_VERSION"
    # Parse new version
    IFS='.' read -r NEW_MAJOR NEW_MINOR NEW_PATCH <<< "$VERSION"

    # Validate format (all components must be non-negative integers)
    for val in "$PREV_MAJOR" "$PREV_MINOR" "$PREV_PATCH" "$NEW_MAJOR" "$NEW_MINOR" "$NEW_PATCH"; do
        if ! [[ "$val" =~ ^[0-9]+$ ]]; then
            error "Invalid version format. Expected MAJOR.MINOR.PATCH (integers only)."
        fi
    done

    # Check valid increment patterns:
    # - Patch bump:  MAJOR same, MINOR same, PATCH +1
    # - Minor bump:  MAJOR same, MINOR +1,   PATCH = 0
    # - Major bump:  MAJOR +1,   MINOR = 0,  PATCH = 0
    VALID=false

    if [ "$NEW_MAJOR" -eq "$PREV_MAJOR" ] && \
       [ "$NEW_MINOR" -eq "$PREV_MINOR" ] && \
       [ "$NEW_PATCH" -eq $((PREV_PATCH + 1)) ]; then
        info "Version bump: patch (${PREV_VERSION} → ${VERSION})"
        VALID=true
    elif [ "$NEW_MAJOR" -eq "$PREV_MAJOR" ] && \
         [ "$NEW_MINOR" -eq $((PREV_MINOR + 1)) ] && \
         [ "$NEW_PATCH" -eq 0 ]; then
        info "Version bump: minor (${PREV_VERSION} → ${VERSION})"
        VALID=true
    elif [ "$NEW_MAJOR" -eq $((PREV_MAJOR + 1)) ] && \
         [ "$NEW_MINOR" -eq 0 ] && \
         [ "$NEW_PATCH" -eq 0 ]; then
        info "Version bump: major (${PREV_VERSION} → ${VERSION})"
        VALID=true
    fi

    if [ "$VALID" = false ]; then
        error "Invalid version increment: ${PREV_VERSION} → ${VERSION}\n" \
              "Allowed increments:\n" \
              "  patch: ${PREV_MAJOR}.${PREV_MINOR}.$((PREV_PATCH + 1))\n" \
              "  minor: ${PREV_MAJOR}.$((PREV_MINOR + 1)).0\n" \
              "  major: $((PREV_MAJOR + 1)).0.0"
    fi
else
    info "No previous tag found — first release (${VERSION})"
fi

# ---------------------------------------------------------------------------
# 3. Generate release notes
# ---------------------------------------------------------------------------
info "Generating draft release notes..."

# Cleanup temp files on error
cleanup() { [ -n "${TMPFILE:-}" ] && rm -f "$TMPFILE"; }
trap cleanup EXIT

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

NOTES=$(generate_release_notes "$PREV_TAG" "$VERSION")

# ---------------------------------------------------------------------------
# 4. Update CHANGELOG.md
# ---------------------------------------------------------------------------
info "Updating CHANGELOG.md..."

# Prevent duplicate release sections for the same version
if grep -q "^## v${VERSION} " CHANGELOG.md; then
    error "CHANGELOG.md already contains release notes for v${VERSION}. Remove the existing section first."
fi

# Verify ## [Unreleased] header exists
if ! grep -q '^## \[Unreleased\]' CHANGELOG.md; then
    error "CHANGELOG.md is missing the '## [Unreleased]' header. Add it before running this script."
fi

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

echo ""
info "Draft release notes written to CHANGELOG.md"
info "Next steps:"
info "  1. Review and rewrite CHANGELOG.md (simplify commit messages)"
info "  2. Run ./publish-release.sh ${VERSION} to commit, tag, and push"
