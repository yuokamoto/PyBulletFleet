#!/bin/bash
# docker/test_integration.sh
# Run inside Docker container to verify full stack
set -e

echo "=== Integration Test: PyBulletFleet ROS 2 Bridge ==="

# Source ROS
source /opt/ros/jazzy/setup.bash
source /opt/bridge_ws/install/setup.bash

# URDF paths are relative to pybullet_fleet root
cd /opt/pybullet_fleet

# Start bridge in background
ros2 run pybullet_fleet_ros bridge_node \
    --ros-args -p num_robots:=3 -p gui:=false -p publish_rate:=10.0 &
BRIDGE_PID=$!
sleep 3

echo "--- Checking topics ---"
TOPICS=$(ros2 topic list)
echo "$TOPICS"

for topic in /clock /robot0/odom /robot0/cmd_vel /robot0/joint_states \
             /robot1/odom /robot2/odom /tf; do
    if echo "$TOPICS" | grep -q "$topic"; then
        echo "  ✓ $topic"
    else
        echo "  ✗ $topic MISSING"
        kill $BRIDGE_PID 2>/dev/null
        exit 1
    fi
done

echo "--- Checking /clock publishes ---"
timeout 5 ros2 topic echo /clock --once || { echo "✗ /clock not publishing"; kill $BRIDGE_PID; exit 1; }
echo "  ✓ /clock publishing"

echo "--- Checking odom publishes ---"
timeout 5 ros2 topic echo /robot0/odom --once || { echo "✗ odom not publishing"; kill $BRIDGE_PID; exit 1; }
echo "  ✓ /robot0/odom publishing"

echo "--- Checking services ---"
SERVICES=$(ros2 service list)
for svc in /sim/get_simulator_features /sim/spawn_entity /sim/get_entities \
           /sim/step_simulation /sim/get_simulation_state; do
    if echo "$SERVICES" | grep -q "$svc"; then
        echo "  ✓ $svc"
    else
        echo "  ✗ $svc MISSING"
        kill $BRIDGE_PID 2>/dev/null
        exit 1
    fi
done

echo "--- Testing cmd_vel → OmniVelocityController ---"
ros2 topic pub --once /robot0/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1
echo "  ✓ cmd_vel published"

echo "--- Testing service calls ---"
RESULT=$(timeout 10 ros2 service call /sim/get_entities simulation_interfaces/srv/GetEntities "{}" 2>&1)
if echo "$RESULT" | grep -q "entities="; then
    echo "  ✓ GetEntities returned entities"
else
    echo "  ✗ GetEntities failed: $RESULT"
    kill $BRIDGE_PID 2>/dev/null
    exit 1
fi

RESULT=$(timeout 10 ros2 service call /sim/get_simulator_features simulation_interfaces/srv/GetSimulatorFeatures "{}" 2>&1)
if echo "$RESULT" | grep -q "features=\["; then
    echo "  ✓ GetSimulatorFeatures returned features"
else
    echo "  ✗ GetSimulatorFeatures failed: $RESULT"
    kill $BRIDGE_PID 2>/dev/null
    exit 1
fi

# Cleanup
kill $BRIDGE_PID 2>/dev/null
wait $BRIDGE_PID 2>/dev/null

echo ""
echo "=== All integration tests PASSED ==="
