"""ROS 2 parameter coercion helpers.

``LaunchConfiguration`` values arrive as strings even when declared with
numeric defaults.  These helpers coerce the raw parameter value to the
intended Python type so callers don't need to handle the string case inline.
"""

from rclpy.node import Node


def get_float_param(node: Node, name: str, default: float) -> float:
    """Read a ROS parameter as float, coercing from string if needed."""
    val = node.get_parameter(name).value
    if isinstance(val, (int, float)):
        return float(val)
    if isinstance(val, str):
        try:
            return float(val)
        except ValueError:
            pass
    return default


def get_bool_param(node: Node, name: str, default: bool) -> bool:
    """Read a ROS parameter as bool, coercing from string if needed."""
    val = node.get_parameter(name).value
    if isinstance(val, bool):
        return val
    if isinstance(val, str):
        return val.lower() in ("true", "1", "yes")
    return default
