"""Parse ``ExecuteActionGoal`` messages into PyBulletFleet Action objects.

Shared by both the ``/{robot}/execute_action`` topic subscriber
(fire-and-forget) and the ``ExecuteAction`` action server (blocking).

All public ``init=True`` dataclass fields are exposed automatically.
Required fields (no default) must be present in the JSON; optional
fields use their dataclass defaults when omitted.  JSON keys match
dataclass field names exactly.

Type converters are inferred from field annotations:
``float``, ``int``, ``bool``, ``str``, ``list`` → built-in cast;
``Path`` → ``Path.from_positions()``; ``Pose`` → list or dict to Pose.

Examples::

    # Move to (3, 2)
    {"action_type": "move", "action_params_json": "{\\"path\\": [[3,2,0]]}"}

    # Pick nearest within 1m
    {"action_type": "pick", "action_params_json": "{\\"search_radius\\": 1.0}"}

    # Drop at position
    {"action_type": "drop", "action_params_json": "{\\"drop_pose\\": [5,3,0.1]}"}

    # Wait 3s
    {"action_type": "wait", "action_params_json": "{\\"duration\\": 3.0}"}

    # Set joints
    {"action_type": "joint", "action_params_json": "{\\"target_joint_positions\\": [0, 1.57]}"}
"""

import dataclasses
import inspect
import json
import logging
from typing import Callable, Dict, Optional, Type, Union

import pybullet_fleet.action as _action_module
from pybullet_fleet.action import Action
from pybullet_fleet.geometry import Path, Pose

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Converters
# ---------------------------------------------------------------------------

_PASS = None  # Sentinel: no conversion
_Conv = Optional[Callable]


def _to_path(raw) -> Path:
    """Convert waypoints list to Path — raises on empty."""
    if not raw:
        raise ValueError("'path' must be non-empty")
    return Path.from_positions(raw)


def _to_pose(raw) -> Pose:
    """Convert list or dict to Pose."""
    if isinstance(raw, list):
        return Pose.from_xyz(*raw[:3])
    if isinstance(raw, dict):
        return Pose(
            position=raw.get("position", [0, 0, 0]),
            orientation=raw.get("orientation", [0, 0, 0, 1]),
        )
    raise ValueError(f"pose must be a list or dict, got {type(raw)}")


_TYPE_CONV: Dict[type, _Conv] = {
    Path: _to_path,
    Pose: _to_pose,
}


def _infer_conv(annotation) -> _Conv:
    """Infer a converter from a type annotation (Optional, Union aware)."""
    origin = getattr(annotation, "__origin__", None)
    if origin is Union:
        args = [a for a in annotation.__args__ if a is not type(None)]
        if len(args) == 1:
            return _infer_conv(args[0])
        return _PASS
    if isinstance(annotation, type) and annotation in _TYPE_CONV:
        return _TYPE_CONV[annotation]
    return _PASS


# ---------------------------------------------------------------------------
# action_type → Action class (auto-discovered from pybullet_fleet.action)
# ---------------------------------------------------------------------------


def _discover_actions() -> Dict[str, Type[Action]]:
    """Auto-discover concrete Action subclasses from ``pybullet_fleet.action``.

    Naming convention: ``FooAction`` → ``"foo"``.
    """
    registry: Dict[str, Type[Action]] = {}
    for name, cls in inspect.getmembers(_action_module, inspect.isclass):
        if issubclass(cls, Action) and cls is not Action and not inspect.isabstract(cls) and name.endswith("Action"):
            registry[name[: -len("Action")].lower()] = cls
    return registry


_ACTIONS: Dict[str, Type[Action]] = _discover_actions()


# ---------------------------------------------------------------------------
# Generic builder
# ---------------------------------------------------------------------------


def _build_action(cls: Type[Action], params: dict) -> Action:
    """Construct an Action by introspecting its dataclass fields.

    All public ``init=True`` fields are considered.  Fields without a
    default are required; fields with a default are optional and only
    included when present in *params*.
    """
    kwargs: dict = {}

    for f in dataclasses.fields(cls):
        if not f.init or f.name.startswith("_"):
            continue

        conv = _infer_conv(f.type)
        has_default = f.default is not dataclasses.MISSING or f.default_factory is not dataclasses.MISSING

        if has_default:
            if f.name in params:
                raw = params[f.name]
                kwargs[f.name] = conv(raw) if conv is not None else raw
        else:
            raw = params.get(f.name)
            if raw is None:
                raise ValueError(f"'{f.name}' is required")
            kwargs[f.name] = conv(raw) if conv is not None else raw

    return cls(**kwargs)


def parse_action_goal(action_type: str, action_params_json: str) -> Optional[Action]:
    """Parse an ExecuteActionGoal into a PyBulletFleet Action.

    Args:
        action_type: One of "move", "pick", "drop", "wait", "joint".
        action_params_json: JSON-encoded parameters. Keys match dataclass
            field names (see each Action class for available fields).

    Returns:
        Action instance, or None if parsing fails.
    """
    try:
        params = json.loads(action_params_json) if action_params_json.strip() else {}
    except json.JSONDecodeError as e:
        logger.error("Invalid JSON for action_type='%s': %s", action_type, e)
        return None

    cls = _ACTIONS.get(action_type)
    if cls is None:
        logger.error("Unknown action_type='%s'. Valid: %s", action_type, list(_ACTIONS.keys()))
        return None

    try:
        return _build_action(cls, params)
    except Exception as e:
        logger.error("Failed to create %s action: %s", action_type, e)
        return None
