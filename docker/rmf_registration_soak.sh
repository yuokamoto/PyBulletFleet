#!/bin/bash
# Launch the headless office RMF demo N times and count bridge_node crashes
# while the fleet adapter adds its robots (issue #152).
#
# Usage (inside the bridge image): rmf_registration_soak.sh N
# Each launch ends once both robots are added (and 5 s more pass without a
# crash), when bridge_node dies, or after 60 s. Prints one line per launch and
# a final "RESULT crash=<n> ok=<n> timeout=<n>" line; exits 1 if any launch
# crashed. Logs of crashed and timed-out launches are kept in $SOAK_OUT.
set -u

N=${1:?usage: rmf_registration_soak.sh N}
SOAK_OUT=${SOAK_OUT:-/tmp/rmf_soak}
mkdir -p "$SOAK_OUT"

set +u
source /opt/ros/jazzy/setup.bash
source /rmf_demos_ws/install/setup.bash
set -u
export ROS_LOG_DIR=/tmp/roslogs
export PYTHONFAULTHANDLER=1

crash=0
ok=0
timeout=0
for i in $(seq 1 "$N"); do
    log="$SOAK_OUT/launch_$i.log"
    setsid ros2 launch pybullet_fleet_rmf office_pybullet.launch.py \
        gui:=false headless:=true target_rtf:=1.0 client_mode:=python_fleet \
        > "$log" 2>&1 &
    pid=$!
    result=timeout
    for _ in $(seq 1 120); do
        sleep 0.5
        if grep -q "bridge_node.*process has died" "$log"; then
            result=crash
            break
        fi
        if [ "$(grep -c 'Successfully added robot' "$log")" -ge 2 ]; then
            sleep 5
            if grep -q "bridge_node.*process has died" "$log"; then
                result=crash
            else
                result=ok
            fi
            break
        fi
    done
    kill -INT -"$pid" 2>/dev/null
    sleep 3
    kill -9 -"$pid" 2>/dev/null
    pkill -9 -f rmf_ 2>/dev/null
    pkill -9 -f bridge_node 2>/dev/null
    sleep 1
    case $result in
        crash) crash=$((crash + 1)) ;;
        ok) ok=$((ok + 1)); rm -f "$log" ;;
        *) timeout=$((timeout + 1)) ;;
    esac
    echo "launch $i: $result (crash=$crash ok=$ok timeout=$timeout)"
    if [ "$result" = crash ]; then
        grep -E "malloc|free\(\)|corrupted|Assertion|Fatal Python error|Thread 0x|File \"" "$log" | head -60
    fi
done

echo "RESULT crash=$crash ok=$ok timeout=$timeout"
[ "$crash" -eq 0 ]
