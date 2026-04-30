#!/usr/bin/env python3
"""Attach or detach a simulation object to/from a robot.

Calls the ``/{robot}/attach_object`` service (AttachObject) by default.
Use ``--toggle`` mode for the simpler ``/{robot}/toggle_attach`` service
(SetBool, rmf_demos compatible).

Usage:

    # Attach nearest pickable object (search_radius=1.0m)
    python3 attach_object.py --robot tb3_0 --attach

    # Attach specific object by name
    python3 attach_object.py --robot tb3_0 --attach --object box_A

    # Attach to a specific link with offset
    python3 attach_object.py --robot tb3_0 --attach --object box_A \\
        --parent-link sensor_mast --offset 0 0 0.1

    # Attach with larger search radius
    python3 attach_object.py --robot tb3_0 --attach --search-radius 2.0

    # Detach (first attached object)
    python3 attach_object.py --robot tb3_0 --detach

    # Detach specific object
    python3 attach_object.py --robot tb3_0 --detach --object box_A

    # Toggle mode (rmf_demos compat, SetBool service)
    python3 attach_object.py --robot tb3_0 --attach --toggle
    python3 attach_object.py --robot tb3_0 --detach --toggle
"""

import argparse
import sys

import rclpy
from rclpy.node import Node


def _call_attach_object(node, robot, attach, object_name, parent_link, offset, search_radius):
    """Call the AttachObject service."""
    from pybullet_fleet_msgs.srv import AttachObject

    client = node.create_client(AttachObject, f"/{robot}/attach_object")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f"/{robot}/attach_object service not available")
        return False

    request = AttachObject.Request()
    request.attach = attach
    request.object_name = object_name or ""
    request.parent_link = parent_link or ""
    request.search_radius = search_radius

    if offset:
        request.offset.position.x = offset[0]
        request.offset.position.y = offset[1]
        request.offset.position.z = offset[2]
    request.offset.orientation.w = 1.0

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    if result.success:
        node.get_logger().info(f"OK: {result.message} (object: {result.attached_object_name})")
    else:
        node.get_logger().error(f"FAILED: {result.message}")
    return result.success


def _call_toggle_attach(node, robot, attach):
    """Call the toggle_attach SetBool service."""
    from std_srvs.srv import SetBool

    client = node.create_client(SetBool, f"/{robot}/toggle_attach")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f"/{robot}/toggle_attach service not available")
        return False

    request = SetBool.Request()
    request.data = attach

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    if result.success:
        node.get_logger().info(f"OK: {result.message}")
    else:
        node.get_logger().error(f"FAILED: {result.message}")
    return result.success


def main():
    parser = argparse.ArgumentParser(
        description="Attach/detach objects via ROS 2 services",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--robot", required=True, help="Robot name (e.g. tb3_0)")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--attach", action="store_true", help="Attach an object")
    group.add_argument("--detach", action="store_true", help="Detach an object")

    parser.add_argument("--object", dest="object_name", default="", help="Target object name (empty = nearest/first)")
    parser.add_argument("--parent-link", default="", help="Parent link name (empty = base_link)")
    parser.add_argument(
        "--offset",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help="Attachment offset [x y z] in parent link frame",
    )
    parser.add_argument("--search-radius", type=float, default=0.5, help="Search radius (m) for nearest object")
    parser.add_argument("--toggle", action="store_true", help="Use toggle_attach (SetBool) instead of attach_object")

    args = parser.parse_args()

    rclpy.init()
    node = Node("attach_object_client")

    try:
        if args.toggle:
            ok = _call_toggle_attach(node, args.robot, args.attach)
        else:
            ok = _call_attach_object(
                node,
                args.robot,
                attach=args.attach,
                object_name=args.object_name,
                parent_link=args.parent_link,
                offset=args.offset,
                search_radius=args.search_radius,
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
