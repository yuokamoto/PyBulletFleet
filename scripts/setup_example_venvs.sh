#!/usr/bin/env bash
# Set up two side-by-side virtualenvs for running the bundled examples:
#
#   .venvs/example-install  — the released package from PyPI (how end users run).
#   .venvs/example-mount    — this checkout via `pip install -e .` (for dev;
#                             the editable install runs your working tree).
#
# Examples ship inside the wheel, so each venv exercises its own copy via the
# `pybullet-fleet examples` CLI — no repo paths, no env vars.
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
# Editable installs (PEP 660) need setuptools>=64, so upgrade build tools first.
"$MOUNT_ENV/bin/pip" install -q -U pip setuptools wheel
"$MOUNT_ENV/bin/pip" install -q -e "$REPO_ROOT"
echo "    installed pybullet-fleet $("$MOUNT_ENV/bin/pip" show pybullet-fleet | awk '/^Version:/{print $2}') (editable: this checkout)"

cat <<EOF

=== done ===
List / run examples against the RELEASED package (what users get):
  $INSTALL_ENV/bin/pybullet-fleet examples --list
  $INSTALL_ENV/bin/pybullet-fleet examples --run path_following_demo

Run the same example against your WORKING TREE (editable install):
  $MOUNT_ENV/bin/pybullet-fleet examples --run path_following_demo

Each venv ships its own examples (inside the wheel / editable checkout), so the
install venv exercises the released wheel — including its bundled robots/, config/,
and mesh/ data — while the mount venv runs your local code. Print the install
location with: <venv>/bin/pybullet-fleet examples --path
EOF
