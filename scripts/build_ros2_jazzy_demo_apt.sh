#!/usr/bin/env bash
# Build an amd64-only Jazzy APT supplement repository from an immutable tag.
set -euo pipefail

source_ref="v0.1.1-ros2"
image="pybullet-fleet-rmf:jazzy"
output_dir=""
include_rmf=false

usage() {
  cat <<'EOF'
Usage: scripts/build_ros2_jazzy_demo_apt.sh --output DIR [options]

Build the PyBulletFleet Jazzy Debian packages from an immutable ROS source tag.
DIR must be empty. The result is a flat APT repository. It is suitable for a
time-limited demo when accompanied by checksums; sign the repository metadata
before general-purpose or long-lived use.

Options:
  --output DIR       Empty directory for .deb files and APT metadata (required)
  --source-ref REF   Immutable source ref to build (default: v0.1.1-ros2)
  --image IMAGE      Jazzy build image (default: pybullet-fleet-rmf:jazzy)
  --include-rmf      Also build pybullet_fleet_rmf (requires an RMF demo overlay at runtime)
  -h, --help         Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output)
      output_dir="${2:-}"
      shift 2
      ;;
    --source-ref)
      source_ref="${2:-}"
      shift 2
      ;;
    --image)
      image="${2:-}"
      shift 2
      ;;
    --include-rmf)
      include_rmf=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "error: unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$output_dir" ]]; then
  echo "error: --output is required" >&2
  usage >&2
  exit 2
fi

repo_root="$(git rev-parse --show-toplevel)"
output_dir="$(realpath -m "$output_dir")"
mkdir -p "$output_dir"

if find "$output_dir" -mindepth 1 -maxdepth 1 -print -quit | grep -q .; then
  echo "error: output directory must be empty: $output_dir" >&2
  exit 2
fi

if ! git -C "$repo_root" rev-parse --verify --quiet "${source_ref}^{commit}" >/dev/null; then
  echo "error: source ref does not resolve to a commit: $source_ref" >&2
  exit 2
fi

worktree="$(mktemp -d /tmp/pbf-jazzy-apt-build.XXXXXX)"
cleanup() {
  git -C "$repo_root" worktree remove --force "$worktree" >/dev/null 2>&1 || true
}
trap cleanup EXIT

git -C "$repo_root" worktree add --detach "$worktree" "$source_ref" >/dev/null
source_commit="$(git -C "$worktree" rev-parse HEAD)"

docker run --rm -i \
  -v "$worktree:/workspace" \
  -v "$output_dir:/output" \
  -w /workspace \
  -e "PBF_SOURCE_REF=$source_ref" \
  -e "PBF_SOURCE_COMMIT=$source_commit" \
  -e "PBF_INCLUDE_RMF=$include_rmf" \
  "$image" \
  bash -s <<'CONTAINER_BUILD'
set -eo pipefail

apt-get update -qq
apt-get install -y -qq --no-install-recommends \
  debhelper dh-python dpkg-dev python3-all

source /opt/ros/jazzy/setup.bash

if [ "$PBF_INCLUDE_RMF" = true ]; then
  # The image's source-built RMF overlay satisfies rosdep, but dpkg validates
  # Build-Depends against installed Debian packages. Install the official Jazzy
  # packages so Bloom's RMF control file can be built and validated faithfully.
  apt-get install -y -qq --no-install-recommends \
    ros-jazzy-rmf-fleet-adapter-python \
    ros-jazzy-rmf-fleet-msgs \
    ros-jazzy-rmf-door-msgs \
    ros-jazzy-rmf-lift-msgs \
    ros-jazzy-rmf-dispenser-msgs \
    ros-jazzy-rmf-ingestor-msgs
fi

# The packages are not indexed in rosdistro yet, so Bloom needs this temporary
# mapping to emit the inter-package Debian dependency.
cat >/tmp/pbf-local-rosdep.yaml <<'EOF'
pybullet_fleet_msgs:
  ubuntu:
    '*': [ros-jazzy-pybullet-fleet-msgs]
pybullet_fleet_ros:
  ubuntu:
    '*': [ros-jazzy-pybullet-fleet-ros]
EOF
printf 'yaml file:///tmp/pbf-local-rosdep.yaml\n' \
  >/etc/ros/rosdep/sources.list.d/00-pbf-local-rosdep.list
rosdep update >/dev/null

build_package() {
  local package_dir="$1"
  cd "/workspace/ros2_bridge/${package_dir}"
  rm -rf debian
  bloom-generate rosdebian --ros-distro jazzy
  # This public fallback distributes binary packages directly. Bloom's default
  # copyright file points at the source repository, so append the full Apache
  # 2.0 text to ensure each .deb carries a copy of its distribution license.
  cat /workspace/LICENSE >>debian/copyright
  rosdep install -y --ignore-src --rosdistro jazzy --from-paths .
  dpkg-buildpackage -us -uc -b
}

build_package pybullet_fleet_msgs
dpkg -i /workspace/ros2_bridge/ros-jazzy-pybullet-fleet-msgs_*.deb
build_package pybullet_fleet_ros
dpkg -i /workspace/ros2_bridge/ros-jazzy-pybullet-fleet-ros_*.deb

if [ "$PBF_INCLUDE_RMF" = true ]; then
  build_package pybullet_fleet_rmf
fi

cp /workspace/ros2_bridge/ros-jazzy-pybullet-fleet-msgs_*.deb /output/
cp /workspace/ros2_bridge/ros-jazzy-pybullet-fleet-ros_*.deb /output/
if [ "$PBF_INCLUDE_RMF" = true ]; then
  cp /workspace/ros2_bridge/ros-jazzy-pybullet-fleet-rmf_*.deb /output/
fi
cd /output
dpkg-scanpackages . /dev/null >Packages
gzip -9c Packages >Packages.gz
sha256sum ./*.deb >SHA256SUMS
printf 'source_ref=%s\nsource_commit=%s\narchitecture=amd64\n' \
  "$PBF_SOURCE_REF" "$PBF_SOURCE_COMMIT" >BUILD_INFO.txt
CONTAINER_BUILD

echo "Generated demo APT repository: $output_dir"
find "$output_dir" -maxdepth 1 -type f -printf '%f\n' | sort
