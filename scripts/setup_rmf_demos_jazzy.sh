#!/usr/bin/env bash
# Build the rmf_demos workspace needed by PyBulletFleet's native ROS 2 Jazzy
# workflow. This installs apt dependencies, then builds only the rmf_demos
# packages used by the PyBulletFleet RMF launch files. Pass --workspace to add
# the sources to an existing colcon workspace; the script does not remove its
# build, install, or log directories.
set -euo pipefail

RMF_WS="${RMF_WS:-$HOME/rmf_demos_ws}"
RMF_DEMOS_VERSION="${RMF_DEMOS_VERSION:-2.3.0}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
PREFETCH_ASSETS=""

usage() {
    cat <<EOF
Usage: $0 [--workspace PATH] [--prefetch-rmf-assets]

Build the RMF demos overlay needed by PyBulletFleet's Jazzy RMF examples.

Options:
  -w, --workspace PATH  Colcon workspace to receive src/rmf_demos
                         (default: $RMF_WS)
      --prefetch-rmf-assets
                         Download Gazebo Fuel assets for every supported RMF
                         demo scenario. Requires network access.
      --prefetch-office-assets
                         Download only Office assets (compatibility option).
  -h, --help            Show this help message

The RMF_WS environment variable remains supported for automation. The
--workspace option takes precedence when both are supplied.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -w|--workspace)
            if [[ $# -lt 2 ]]; then
                echo "Missing path after $1" >&2
                exit 2
            fi
            RMF_WS="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --prefetch-office-assets)
            PREFETCH_ASSETS="office"
            shift
            ;;
        --prefetch-rmf-assets)
            PREFETCH_ASSETS="all"
            shift
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [ ! -f "$ROS_SETUP" ]; then
    echo "ROS setup not found: $ROS_SETUP" >&2
    exit 1
fi

sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-vcstool python3-rosdep python3-catkin-pkg python3-empy python3-lark ros-jazzy-rmf-dev ros-jazzy-rmf-demos-assets ros-jazzy-rmf-demos-bridges ros-jazzy-rmf-demos-fleet-adapter ros-jazzy-rmf-demos-tasks ros-jazzy-nav2-msgs ros-jazzy-control-msgs ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs ros-jazzy-trajectory-msgs ros-jazzy-diagnostic-msgs ros-jazzy-simulation-interfaces ros-jazzy-teleop-twist-keyboard ros-jazzy-xacro ros-jazzy-turtlebot3-description ros-jazzy-ur-description python3-tk

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi

rosdep update
mkdir -p "$RMF_WS/src"

if [ ! -d "$RMF_WS/src/rmf_demos/.git" ]; then
    git clone --branch "$RMF_DEMOS_VERSION" --depth 1 https://github.com/open-rmf/rmf_demos.git "$RMF_WS/src/rmf_demos"
else
    git -C "$RMF_WS/src/rmf_demos" fetch --tags --depth 1 origin "$RMF_DEMOS_VERSION"
    git -C "$RMF_WS/src/rmf_demos" checkout "$RMF_DEMOS_VERSION"
fi

set +u
source "$ROS_SETUP"
set -u

rosdep install --from-paths "$RMF_WS/src/rmf_demos/rmf_demos" "$RMF_WS/src/rmf_demos/rmf_demos_assets" "$RMF_WS/src/rmf_demos/rmf_demos_bridges" "$RMF_WS/src/rmf_demos/rmf_demos_fleet_adapter" "$RMF_WS/src/rmf_demos/rmf_demos_maps" "$RMF_WS/src/rmf_demos/rmf_demos_tasks" --ignore-src -r -y
colcon --log-base "$RMF_WS/log" build --base-paths "$RMF_WS/src/rmf_demos" --build-base "$RMF_WS/build" --install-base "$RMF_WS/install" --symlink-install --packages-select rmf_demos rmf_demos_assets rmf_demos_bridges rmf_demos_fleet_adapter rmf_demos_maps rmf_demos_tasks --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3

ls "$RMF_WS/install/setup.bash"

if [ "$PREFETCH_ASSETS" = "all" ]; then
    PREFETCH_SCENARIO="all"
elif [ "$PREFETCH_ASSETS" = "office" ]; then
    PREFETCH_SCENARIO="office"
else
    PREFETCH_SCENARIO=""
fi

if [ -n "$PREFETCH_SCENARIO" ]; then
    if ! command -v gz >/dev/null; then
        echo "gz CLI not found; cannot prefetch RMF demo assets" >&2
        exit 1
    fi
    mapfile -t fuel_uris < <(python3 - "$RMF_WS" "$PREFETCH_SCENARIO" <<'PY'
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import quote

workspace = Path(sys.argv[1])
selection = sys.argv[2]
scenarios = ("office", "hotel", "clinic", "airport_terminal", "campus", "battle_royale")
if selection != "all":
    scenarios = (selection,)

maps_dir = workspace / "install/rmf_demos_maps/share/rmf_demos_maps/maps"
assets_dir = workspace / "install/rmf_demos_assets/share/rmf_demos_assets/models"
env_paths = [Path(path) for path in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(":") if path]
uris = set()

for scenario in scenarios:
    world = maps_dir / scenario / f"{scenario}.world"
    if not world.is_file():
        raise SystemExit(f"RMF world file not found: {world}")
    root = ET.parse(world).getroot()
    resource_paths = [world.parent / "models", assets_dir, *env_paths]
    for element in root.iter():
        if element.tag.rsplit("}", 1)[-1] != "uri" or not element.text:
            continue
        uri = element.text.strip()
        if uri.startswith("https://fuel."):
            uris.add(uri)
        elif uri.startswith("model://"):
            model = uri[len("model://") :]
            if not any((path / model / "model.sdf").is_file() for path in resource_paths):
                uris.add(f"https://fuel.gazebosim.org/1.0/OpenRobotics/models/{quote(model)}")

print(*sorted(uris), sep="\n")
PY
)
    if [ "${#fuel_uris[@]}" -eq 0 ]; then
        echo "No missing Gazebo Fuel assets found." >&2
        exit 1
    fi
    echo "Prefetching ${#fuel_uris[@]} Gazebo Fuel assets..."
    for fuel_uri in "${fuel_uris[@]}"; do
        echo "Downloading: $fuel_uri"
        gz fuel download -u "$fuel_uri"
    done
fi
