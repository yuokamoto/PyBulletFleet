#!/usr/bin/env bash
set -euo pipefail

if (( $# < 1 )); then
  echo "Usage: run_gui_example EXAMPLE [EXAMPLE_ARGS...]" >&2
  exit 2
fi

export DISPLAY=:99
Xvfb "$DISPLAY" -screen 0 1280x800x24 +extension GLX +render -noreset &

for _ in {1..50}; do
  if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    break
  fi
  sleep 0.2
done
xdpyinfo -display "$DISPLAY" >/dev/null

# The VNC socket stays inside the container; only noVNC is published on the
# Mac's loopback interface by the launcher.
x11vnc -display "$DISPLAY" -listen 127.0.0.1 -rfbport 5900 -forever -shared -nopw -bg -quiet
websockify --web=/usr/share/novnc 6080 127.0.0.1:5900 &

exec pybullet-fleet examples --run "$@"
