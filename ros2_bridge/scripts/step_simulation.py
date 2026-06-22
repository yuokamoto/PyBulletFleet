#!/usr/bin/env python3
"""Control simulation stepping via simulation_interfaces services.

Usage:
    python3 scripts/step_simulation.py --steps 100
    python3 scripts/step_simulation.py --pause
    python3 scripts/step_simulation.py --resume
    python3 scripts/step_simulation.py --status
"""

import argparse

import rclpy
from rclpy.node import Node
from simulation_interfaces.msg import SimulationState
from simulation_interfaces.srv import GetSimulationState, SetSimulationState, StepSimulation


def main():
    parser = argparse.ArgumentParser(description="Control simulation stepping")
    parser.add_argument("--steps", type=int, default=0, help="Number of steps to execute")
    parser.add_argument("--pause", action="store_true", help="Pause simulation")
    parser.add_argument("--resume", action="store_true", help="Resume simulation")
    parser.add_argument("--status", action="store_true", help="Get simulation state")
    args = parser.parse_args()

    rclpy.init()
    node = Node("step_simulation_client")

    if args.pause:
        client = node.create_client(SetSimulationState, "/sim/set_simulation_state")
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error("SetSimulationState service not available")
            node.destroy_node()
            rclpy.shutdown()
            return
        request = SetSimulationState.Request()
        request.state.state = SimulationState.STATE_PAUSED
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        node.get_logger().info("Simulation paused")

    elif args.resume:
        client = node.create_client(SetSimulationState, "/sim/set_simulation_state")
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error("SetSimulationState service not available")
            node.destroy_node()
            rclpy.shutdown()
            return
        request = SetSimulationState.Request()
        request.state.state = SimulationState.STATE_PLAYING
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        node.get_logger().info("Simulation resumed")

    elif args.status:
        client = node.create_client(GetSimulationState, "/sim/get_simulation_state")
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error("GetSimulationState service not available")
            node.destroy_node()
            rclpy.shutdown()
            return
        request = GetSimulationState.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()
        state_name = "PAUSED" if result.state.state == SimulationState.STATE_PAUSED else "PLAYING"
        node.get_logger().info(f"Simulation state: {state_name}")

    elif args.steps > 0:
        client = node.create_client(StepSimulation, "/sim/step_simulation")
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error("StepSimulation service not available")
            node.destroy_node()
            rclpy.shutdown()
            return
        request = StepSimulation.Request()
        request.steps = args.steps
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        node.get_logger().info(f"Executed {args.steps} simulation steps")

    else:
        node.get_logger().info("No action specified. Use --steps, --pause, --resume, or --status")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
