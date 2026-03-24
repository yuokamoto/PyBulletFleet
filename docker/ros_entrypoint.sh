#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/bridge_ws/install/setup.bash
exec "$@"
