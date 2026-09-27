#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Run a PyBulletFleet example inside Linux and view its GUI in a Mac browser.

Usage: scripts/run_linux_gui_macos.sh EXAMPLE [EXAMPLE_ARGS...]

Examples:
  scripts/run_linux_gui_macos.sh 100robots_grid_demo.py --duration 30
  scripts/run_linux_gui_macos.sh pick_drop_arm_100robots_demo.py --robots 10
EOF
}

if (( $# < 1 )) || [[ ${1:-} == --help ]]; then
  usage
  [[ ${1:-} == --help ]] && exit 0 || exit 2
fi

if [[ $(uname -s) != Darwin ]]; then
  echo "This launcher requires macOS." >&2
  exit 1
fi
for command in colima docker curl open; do
  if ! command -v "$command" >/dev/null 2>&1; then
    echo "Missing $command. Install Colima and Docker CLI with: brew install colima docker" >&2
    exit 1
  fi
done

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
profile=pbf-linux
docker_context=colima-$profile
image=pybullet-fleet-linux-gui:local
container=
started_colima=false

cleanup() {
  trap - EXIT INT TERM
  if [[ -n $container ]]; then
    docker --context "$docker_context" rm -f "$container" >/dev/null 2>&1 || true
  fi
  if [[ $started_colima == true ]]; then
    colima stop "$profile" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if ! colima status --profile "$profile" >/dev/null 2>&1; then
  echo "Starting Linux VM ($profile)..."
  (cd / && colima start "$profile" --vm-type vz --cpu 4 --memory 4 --disk 25 --activate=false)
  started_colima=true
fi

echo "Building Linux GUI image (cached after the first run)..."
docker --context "$docker_context" build -f "$repo_root/docker/Dockerfile.mac-gui" -t "$image" "$repo_root"

container=$(docker --context "$docker_context" run --detach \
  --publish 127.0.0.1::6080 "$image" "$@")
host_port=$(docker --context "$docker_context" port "$container" 6080/tcp | sed -n 's/.*://p' | head -n 1)
if [[ -z $host_port ]]; then
  echo "Could not determine the browser port." >&2
  exit 1
fi

url="http://127.0.0.1:$host_port/vnc.html?autoconnect=1&resize=scale"
ready=false
for _ in {1..100}; do
  if curl --fail --silent --output /dev/null "http://127.0.0.1:$host_port/vnc.html"; then
    ready=true
    break
  fi
  if [[ $(docker --context "$docker_context" inspect --format '{{.State.Running}}' "$container") != true ]]; then
    break
  fi
  sleep 0.2
done

if [[ $ready != true ]]; then
  docker --context "$docker_context" logs "$container" >&2
  echo "Linux GUI failed to start." >&2
  exit 1
fi

echo "Linux GUI: $url"
open "$url" || true
echo "Close the example window or press Ctrl+C to stop."
docker --context "$docker_context" logs --follow "$container" &
logs_pid=$!
exit_code=$(docker --context "$docker_context" wait "$container")
wait "$logs_pid" || true
exit "$exit_code"
