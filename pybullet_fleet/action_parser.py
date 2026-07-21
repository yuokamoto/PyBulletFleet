"""Parse generic action commands into PyBulletFleet Action objects."""

from __future__ import annotations

import dataclasses
import inspect
import json
from typing import Callable, Dict, Optional, Type, Union

import pybullet_fleet.action as _action_module
from pybullet_fleet.action import Action
from pybullet_fleet.geometry import Path, Pose
from pybullet_fleet.logging_utils import get_lazy_logger

logger = get_lazy_logger(__name__)

_PASS = None
_Conv = Optional[Callable]


def _to_path(raw) -> Path:
    if not raw:
        raise ValueError("'path' must be non-empty")
    return Path.from_positions(raw)


def _to_pose(raw) -> Pose:
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
    origin = getattr(annotation, "__origin__", None)
    if origin is Union:
        args = [a for a in annotation.__args__ if a is not type(None)]
        if len(args) == 1:
            return _infer_conv(args[0])
        return _PASS
    if isinstance(annotation, type) and annotation in _TYPE_CONV:
        return _TYPE_CONV[annotation]
    return _PASS


def _discover_actions() -> Dict[str, Type[Action]]:
    registry: Dict[str, Type[Action]] = {}
    for name, cls in inspect.getmembers(_action_module, inspect.isclass):
        if issubclass(cls, Action) and cls is not Action and not inspect.isabstract(cls) and name.endswith("Action"):
            registry[name[: -len("Action")].lower()] = cls
    return registry


_ACTIONS: Dict[str, Type[Action]] = _discover_actions()


def _build_action(cls: Type[Action], params: dict) -> Action:
    kwargs: dict = {}

    for field in dataclasses.fields(cls):
        if not field.init or field.name.startswith("_"):
            continue

        conv = _infer_conv(field.type)
        has_default = field.default is not dataclasses.MISSING or field.default_factory is not dataclasses.MISSING

        if has_default:
            if field.name in params:
                raw = params[field.name]
                kwargs[field.name] = conv(raw) if conv is not None else raw
        else:
            raw = params.get(field.name)
            if raw is None:
                raise ValueError(f"'{field.name}' is required")
            kwargs[field.name] = conv(raw) if conv is not None else raw

    return cls(**kwargs)


def parse_action_goal(action_type: str, action_params_json: str) -> Optional[Action]:
    """Parse an action type and JSON parameter payload."""
    try:
        params = json.loads(action_params_json) if action_params_json.strip() else {}
    except json.JSONDecodeError as exc:
        logger.error("Invalid JSON for action_type='%s': %s", action_type, exc)
        return None

    cls = _ACTIONS.get(action_type)
    if cls is None:
        logger.error("Unknown action_type='%s'. Valid: %s", action_type, list(_ACTIONS.keys()))
        return None

    try:
        return _build_action(cls, params)
    except Exception as exc:  # noqa: B902
        logger.error("Failed to create %s action: %s", action_type, exc)
        return None
