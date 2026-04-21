"""Handler class resolution for per-robot ROS interface customization.

Uses **additive accumulation** to determine which RobotHandler subclass(es)
to use for each agent.  All matching sources contribute handlers:

    1. ``agent.user_data["handler_class"]`` — per-robot override
       (class object or dotted import string).
    2. ``handler_map`` exact-name match.
    3. ``handler_map`` regex/glob pattern matches (all that match).

If none of the above produce handlers, ``default_classes`` is used.
Duplicates are removed (order preserved).

This module has **no ROS dependencies** so it can be tested without
a ROS workspace.
"""

import fnmatch
import importlib
import logging
import re
from typing import TYPE_CHECKING, Any, Dict, List, Pattern, Union

if TYPE_CHECKING:
    from pybullet_fleet.agent import Agent

logger = logging.getLogger(__name__)

# Type alias for handler_map keys: str (exact) or compiled regex
HandlerMapKey = Union[str, "Pattern[str]"]
# Values: single class or list of classes
HandlerMapValue = Union[type, List[type]]
HandlerMap = Dict[HandlerMapKey, HandlerMapValue]

# Characters that signal a glob/regex pattern (not an exact name)
_GLOB_CHARS = {"*", "?", "[", "]"}


def _normalize_to_list(value: HandlerMapValue) -> List[type]:
    """Ensure *value* is a list of classes."""
    if isinstance(value, list):
        return value
    return [value]


def _import_class(dotted_path: str) -> type:
    """Import a class from a dotted module path.

    Example::

        cls = _import_class("my_pkg.handlers.ArmHandler")
    """
    module_path, _, class_name = dotted_path.rpartition(".")
    if not module_path:
        raise ImportError(f"Invalid dotted path: '{dotted_path}' (no module component)")
    module = importlib.import_module(module_path)
    return getattr(module, class_name)


def resolve_handler_classes(
    agent: "Agent",
    handler_map: HandlerMap,
    default_classes: List[type],
) -> List[type]:
    """Resolve handler class(es) for *agent* using additive accumulation.

    All matching sources contribute handlers (no early return):

        1. ``agent.user_data["handler_class"]`` — class, dotted string,
           or list of either.
        2. ``handler_map`` — exact name match.
        3. ``handler_map`` — all matching regex patterns.

    If none of the above produce any handlers, ``default_classes`` is used
    as a fallback.

    This additive design allows common handlers to be defined via patterns
    (e.g. ``"*": OdomHandler``) while adding specialised handlers per
    robot via exact match or ``user_data``.

    Args:
        agent: The agent to resolve handlers for.
        handler_map: Mapping of name/pattern → handler class(es).
        default_classes: Fallback handler class list (used only when
            no other source matches).

    Returns:
        A list of handler classes (always a list, deduplicated,
        preserving insertion order).

    Raises:
        ImportError: If a dotted path string cannot be resolved.
    """
    collected: List[type] = []

    # 1. Per-robot override via spawn params user_data
    cls_or_path = agent.user_data.get("handler_class")
    if cls_or_path is not None:
        collected.extend(_resolve_user_data_value(cls_or_path))

    # 2. Name-based mapping (exact match + all pattern matches)
    name = agent.name
    if name:
        # 2a. Exact match
        if name in handler_map:
            collected.extend(_normalize_to_list(handler_map[name]))
        # 2b. All regex pattern matches
        for key, value in handler_map.items():
            if isinstance(key, re.Pattern) and key.search(name):
                collected.extend(_normalize_to_list(value))

    # 3. Default — only if nothing was collected
    if not collected:
        return list(default_classes)

    # Deduplicate while preserving order
    return list(dict.fromkeys(collected))


def _resolve_user_data_value(value: Any) -> List[type]:
    """Resolve user_data['handler_class'] which may be class, str, or list."""
    if isinstance(value, list):
        return [_resolve_single(v) for v in value]
    return [_resolve_single(value)]


def _resolve_single(value: Any) -> type:
    """Resolve a single class reference (class object or dotted string)."""
    if isinstance(value, str):
        return _import_class(value)
    return value


def _glob_to_regex(pattern: str) -> "re.Pattern[str]":
    """Convert a glob pattern to a compiled regex using :func:`fnmatch.translate`.

    Example::

        _glob_to_regex("arm_*")  # matches "arm_0", "arm_foo"
    """
    return re.compile(fnmatch.translate(pattern))


def load_handler_map_from_config(config: Dict[str, Any]) -> HandlerMap:
    """Build a :data:`HandlerMap` from a YAML config dictionary.

    Each key is a robot name (exact) or a glob pattern (containing
    ``*`` or ``?``).  Each value is a dotted Python import path **or a
    list** of dotted paths (for multiple handlers per robot).

    Glob patterns are converted to compiled regex keys so that
    :func:`resolve_handler_classes` can match them.

    Args:
        config: ``{"arm_*": "my_pkg.ArmHandler"}`` or
                ``{"arm_*": ["my_pkg.NavHandler", "my_pkg.ArmHandler"]}``.

    Returns:
        A :data:`HandlerMap` ready for :func:`resolve_handler_classes`.

    Raises:
        ValueError: If a value is not a dotted Python path (or list thereof).
        ImportError: If the module cannot be imported.

    Example YAML::

        handler_map:
          arm_*:
            - my_pkg.handlers.NavHandler
            - my_pkg.handlers.ArmHandler
          mobile_*: my_pkg.handlers.MobileHandler
          special_robot: my_pkg.handlers.SpecialHandler
    """
    handler_map: HandlerMap = {}
    for name_or_pattern, value in config.items():
        classes = _resolve_config_value(name_or_pattern, value)

        # If key contains glob chars, convert to regex
        if any(ch in name_or_pattern for ch in _GLOB_CHARS):
            key: HandlerMapKey = _glob_to_regex(name_or_pattern)
        else:
            key = name_or_pattern

        handler_map[key] = classes
        logger.info("Registered handler(s) %r → %s", name_or_pattern, [c.__name__ for c in classes])

    return handler_map


def _validate_dotted_path(name_or_pattern: str, dotted_path: str) -> None:
    """Raise ValueError if *dotted_path* is not a valid dotted Python path."""
    if not isinstance(dotted_path, str) or "." not in dotted_path:
        raise ValueError(
            f"handler_map[{name_or_pattern!r}] must be a dotted Python path " f"'module.ClassName', got: {dotted_path!r}"
        )


def _resolve_config_value(name_or_pattern: str, value: Any) -> List[type]:
    """Resolve a config value (string or list of strings) to a list of classes."""
    if isinstance(value, str):
        _validate_dotted_path(name_or_pattern, value)
        return [_import_class(value)]
    if isinstance(value, list):
        classes = []
        for v in value:
            _validate_dotted_path(name_or_pattern, v)
            classes.append(_import_class(v))
        return classes
    raise ValueError(f"handler_map[{name_or_pattern!r}] must be a dotted Python path " f"or list of paths, got: {value!r}")
