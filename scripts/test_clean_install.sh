#!/usr/bin/env bash
# Build (or download) pybullet-fleet, install it into a CLEAN virtualenv with no
# repo mount, and run the smoke test from a temp dir. Catches packaging bugs that
# editable/mounted dev installs hide — e.g. bundled robots/config/mesh missing
# from the wheel.
#
# Usage:
#   scripts/test_clean_install.sh              # build the local tree into a wheel and test it
#   scripts/test_clean_install.sh 0.4.1        # test the released version from PyPI instead
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VERSION="${1:-}"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

echo "=== clean-install test (work dir: $WORK) ==="

# 1. Obtain a distribution.
if [ -n "$VERSION" ]; then
    TARGET="pybullet-fleet==${VERSION}"
    echo "--- testing released ${TARGET} from PyPI ---"
else
    echo "--- building wheel from $REPO_ROOT ---"
    python3 -m venv "$WORK/buildenv"
    "$WORK/buildenv/bin/pip" install -q -U pip setuptools wheel
    "$WORK/buildenv/bin/pip" wheel "$REPO_ROOT" --no-deps -w "$WORK/dist"
    TARGET="$(ls "$WORK"/dist/pybullet_fleet-*.whl | head -1)"
    echo "built: $(basename "$TARGET")"
fi

# 2. Fresh venv, install (with deps), no repo on the path.
python3 -m venv "$WORK/cleanenv"
"$WORK/cleanenv/bin/pip" install -q -U pip
"$WORK/cleanenv/bin/pip" install -q "$TARGET"

# 3. Run the smoke test from a non-repo CWD.
cp "$REPO_ROOT/scripts/clean_install_smoke.py" "$WORK/smoke.py"
( cd "$WORK" && "$WORK/cleanenv/bin/python" smoke.py )
echo "=== clean-install test OK ==="
