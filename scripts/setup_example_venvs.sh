#!/usr/bin/env bash
# Set up two side-by-side virtualenvs for running the examples:
#
#   .venvs/example-install  — the released package from PyPI (how end users run).
#                             Examples use it by default (PBF_USE_INSTALLED=1).
#   .venvs/example-mount    — this checkout via `pip install -e .` (for dev).
#                             The editable install *is* the local source, so
#                             examples run against your working tree.
#
# Examples pick their package source from PBF_USE_INSTALLED (default "1"):
#   - unset / "1" → use the installed pybullet_fleet (correct in BOTH venvs).
#   - "0"         → prepend this checkout to sys.path (for a bare clone with no
#                   install). You normally don't need this when using a venv.
#
# Usage:
#   scripts/setup_example_venvs.sh            # install venv tracks latest PyPI
#   scripts/setup_example_venvs.sh 0.4.1      # install venv pins this version
#
# Extras: examples under examples/models/ need extras — re-run pip in the venv
# with e.g. `pip install 'pybullet-fleet[sdf,models]'` (install) or
# `pip install -e '.[sdf,models]'` (mount) if you want those demos.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VERSION="${1:-}"
VENV_DIR="$REPO_ROOT/.venvs"
INSTALL_ENV="$VENV_DIR/example-install"
MOUNT_ENV="$VENV_DIR/example-mount"

if [ -n "$VERSION" ]; then
    INSTALL_TARGET="pybullet-fleet==${VERSION}"
else
    INSTALL_TARGET="pybullet-fleet"
fi

mkdir -p "$VENV_DIR"

echo "=== [1/2] install venv: ${INSTALL_TARGET} (released package) ==="
python3 -m venv --clear "$INSTALL_ENV"   # --clear: reproducible across re-runs (no stale site-packages)
"$INSTALL_ENV/bin/pip" install -q -U pip
"$INSTALL_ENV/bin/pip" install -q "$INSTALL_TARGET"
echo "    installed pybullet-fleet $("$INSTALL_ENV/bin/pip" show pybullet-fleet | awk '/^Version:/{print $2}')"

echo "=== [2/2] mount venv: editable install of this checkout ==="
python3 -m venv --clear "$MOUNT_ENV"   # --clear: reproducible across re-runs
"$MOUNT_ENV/bin/pip" install -q -U pip
"$MOUNT_ENV/bin/pip" install -q -e "$REPO_ROOT"
echo "    installed pybullet-fleet $("$MOUNT_ENV/bin/pip" show pybullet-fleet | awk '/^Version:/{print $2}') (editable: this checkout)"

cat <<EOF

=== done ===
Run an example against the RELEASED package (what users get):
  PBF_USE_INSTALLED=1 $INSTALL_ENV/bin/python examples/mobile/path_following_demo.py

Run the same example against your WORKING TREE:
  PBF_USE_INSTALLED=1 $MOUNT_ENV/bin/python examples/mobile/path_following_demo.py

Both default to the installed package; the explicit PBF_USE_INSTALLED=1 above
keeps that true even if you already exported PBF_USE_INSTALLED=0 in your shell.
The mount venv's editable install points at this checkout, so it runs your local
code. To run a bare checkout (no install) instead, export PBF_USE_INSTALLED=0.
EOF
