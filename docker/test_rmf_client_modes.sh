#!/bin/bash
# docker/test_rmf_client_modes.sh
#
# Runs the RMF client-mode validation matrix. This script is intentionally
# opt-in because full RMF dispatch tests are slow.
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FULL=false

while [ "$#" -gt 0 ]; do
    case "$1" in
        --full)
            FULL=true
            shift
            ;;
        *)
            echo "usage: test_rmf_client_modes.sh [--full]" >&2
            exit 2
            ;;
    esac
done

run_e2e() {
    local mode="$1"
    local launch="$2"
    shift 2
    bash "${ROOT_DIR}/test_rmf_e2e.sh" --client-mode "$mode" "$launch" "$@"
}

run_ready() {
    local mode="$1"
    local launch="$2"
    bash "${ROOT_DIR}/test_rmf_e2e.sh" --client-mode "$mode" --ready-only "$launch"
}

echo "=== RMF client-mode matrix ==="

# Office is the primary single-fleet coverage target. It supports all three
# client modes, including python_fleet's in-process bridge plugin path.
for mode in per_robot_ros fleet_ros python_fleet; do
    run_e2e "$mode" office_pybullet \
        "patrol:lounge" \
        "delivery:pantry,coke_dispenser,hardware_2,coke_ingestor"
done

# Hotel covers multi-fleet + doors + lifts + clean. In python_fleet mode each
# fleet adapter runs as an in-process bridge plugin over the same sim_core.
for mode in per_robot_ros fleet_ros python_fleet; do
    run_e2e "$mode" hotel_pybullet \
        "patrol:lobby,L2_room1;zrise=2.0" \
        "clean:clean_lobby"
done

if [ "$FULL" = true ]; then
    for launch in clinic_pybullet airport_terminal_pybullet; do
        for mode in per_robot_ros fleet_ros python_fleet; do
            run_ready "$mode" "$launch"
        done
    done

    for launch in campus_pybullet battle_royale_pybullet; do
        for mode in per_robot_ros fleet_ros python_fleet; do
            run_ready "$mode" "$launch"
        done
    done
fi

echo "=== RMF client-mode matrix PASSED ==="
