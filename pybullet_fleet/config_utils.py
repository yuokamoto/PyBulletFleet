"""
config_utils.py
Utility functions for loading and merging configuration files.
"""

import dataclasses
import importlib
import inspect
import os
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Type, TypeVar, Union

import yaml

T = TypeVar("T")


def load_yaml_config(config_path: str) -> Dict[str, Any]:
    """Load a YAML config file and return parsed contents.

    General-purpose YAML loader.  Works for any YAML config file
    (simulation, bridge, spawn definitions, etc.).

    Args:
        config_path: Path to YAML config file.

    Returns:
        Parsed config dict.

    Raises:
        FileNotFoundError: If *config_path* does not exist.
    """
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with open(config_path, "r") as f:
        config = yaml.safe_load(f) or {}

    return config


def merge_configs(base_config: Dict[str, Any], override_config: Dict[str, Any]) -> Dict[str, Any]:
    """
    Recursively merge two configuration dictionaries.

    Values in override_config take precedence over base_config.

    Args:
        base_config: Base configuration dictionary
        override_config: Override configuration dictionary

    Returns:
        Merged configuration dictionary
    """
    merged = base_config.copy()

    for key, value in override_config.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
            # Recursively merge nested dictionaries
            merged[key] = merge_configs(merged[key], value)
        else:
            # Override value
            merged[key] = value

    return merged


def load_config(config_paths: Union[str, List[str]]) -> Dict[str, Any]:
    """
    Load and merge configuration from one or more YAML files.

    If multiple paths are provided, later configs override earlier ones.
    Delegates to :func:`load_yaml_config` for each file.

    Args:
        config_paths: Single path string or list of paths to configuration files.
                     Later configs in the list override earlier ones.

    Returns:
        Merged configuration dictionary

    Examples:
        # Single config
        config = load_config('config.yaml')

        # Multiple configs (100robots_config overrides config.yaml)
        config = load_config(['config.yaml', '100robots_config.yaml'])

    """
    # Convert single path to list
    if isinstance(config_paths, str):
        config_paths = [config_paths]

    if not config_paths:
        raise ValueError("At least one config path must be provided")

    merged_config = load_yaml_config(config_paths[0])

    for config_path in config_paths[1:]:
        additional_config = load_yaml_config(config_path)
        merged_config = merge_configs(merged_config, additional_config)

    return merged_config


def resolve_class(dotted_path: str) -> Type:
    """Import and return a class from a dotted Python path.

    Args:
        dotted_path: Fully-qualified class path, e.g.
            ``"pybullet_fleet.plugins.workcell_plugin.WorkcellPlugin"``.

    Returns:
        The resolved class object.

    Raises:
        ValueError: If *dotted_path* is not a valid ``module.ClassName`` string.
        ModuleNotFoundError: If the module cannot be imported.
        AttributeError: If the class is not found in the module.
    """
    if not isinstance(dotted_path, str) or "." not in dotted_path:
        raise ValueError(f"Expected a dotted Python path 'module.ClassName', got: {dotted_path!r}")

    module_path, class_name = dotted_path.rsplit(".", 1)

    try:
        module = importlib.import_module(module_path)
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            f"Could not import module '{module_path}' " f"(from '{dotted_path}'). Is the package installed?"
        ) from exc

    try:
        return getattr(module, class_name)
    except AttributeError:
        available = [n for n in dir(module) if not n.startswith("_")]
        raise AttributeError(
            f"Class '{class_name}' not found in module '{module_path}'. " f"Available names: {available}"
        ) from None


# ---------------------------------------------------------------------------
# Generic dataclass_from_dict — introspective YAML→dataclass builder
# ---------------------------------------------------------------------------


def dataclass_from_dict(
    cls: Type[T],
    config: Dict[str, Any],
    *,
    aliases: Optional[Dict[str, str]] = None,
    converters: Optional[Dict[str, Callable]] = None,
    strict_keys: Optional[List[str]] = None,
) -> T:
    """Construct a dataclass instance from a config dict via field introspection.

    For each ``init=True`` field in *cls*, look up the corresponding config
    key (or its alias) and apply an optional converter.  Fields with defaults
    are optional; fields without defaults are required.

    Enum fields are auto-wrapped: if the config value is a ``str`` and the
    field type is an :class:`~enum.Enum` subclass, the value is converted
    via ``EnumCls(value)``.

    Args:
        cls: Target dataclass type.
        config: Dict of config values (typically from YAML).
        aliases: ``{field_name: config_key}`` — alternative config keys.
            For example ``{"camera_config": "camera"}`` reads the ``"camera"``
            key from *config* into the ``camera_config`` field.
        converters: ``{field_name: callable}`` — per-field converters.
            Called as ``converter(raw_value)`` before assignment.
            A converter returning ``None`` *skips* the field (useful for
            conditional ``None``-passthrough, e.g. ``collision_detection_method``).
        strict_keys: If provided, only these fields are read from *config*.
            All other fields use their dataclass defaults.

    Returns:
        An instance of *cls*.

    Raises:
        ValueError: If a required field (no default) is missing from *config*.

    Example::

        @dataclass
        class MyParams:
            x: float = 0.0
            mode: MyEnum = MyEnum.A

        params = dataclass_from_dict(MyParams, {"x": 1.5, "mode": "b"})
        assert params.x == 1.5
        assert params.mode == MyEnum.B
    """
    aliases = aliases or {}
    converters = converters or {}
    kwargs: Dict[str, Any] = {}

    for f in dataclasses.fields(cls):
        if not f.init or f.name.startswith("_"):
            continue

        if strict_keys is not None and f.name not in strict_keys:
            continue

        config_key = aliases.get(f.name, f.name)
        has_default = f.default is not dataclasses.MISSING or f.default_factory is not dataclasses.MISSING

        # Custom converter takes priority
        if f.name in converters:
            if config_key in config:
                val = converters[f.name](config[config_key])
                if val is not None or not has_default:
                    kwargs[f.name] = val
            elif not has_default:
                raise ValueError(f"Required field '{f.name}' missing from config")
            continue

        if config_key not in config:
            if not has_default:
                raise ValueError(f"Required field '{f.name}' missing from config")
            continue

        raw = config[config_key]

        # Auto-wrap enum fields
        ftype = f.type if isinstance(f.type, type) else None
        if ftype is None:
            # Resolve string annotations (Python 3.8/3.9 compatibility)
            ns = vars(importlib.import_module(cls.__module__)) if hasattr(cls, "__module__") else {}
            try:
                ftype = eval(f.type, ns) if isinstance(f.type, str) else None  # noqa: S307
            except Exception:
                ftype = None
        if isinstance(ftype, type) and issubclass(ftype, Enum) and isinstance(raw, str):
            raw = ftype(raw)

        kwargs[f.name] = raw

    return cls(**kwargs)


def forward_spawn_params(
    factory_method: Callable,
    spawn_params,
    *,
    aliases: Optional[Dict[str, str]] = None,
    extra_kwargs: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Build kwargs for a factory method from a dataclass spawn_params.

    Introspects the *factory_method*'s signature and forwards matching
    fields from *spawn_params*.  Fields named differently can be mapped
    via *aliases* (e.g. ``{"initial_pose": "pose"}``).

    Args:
        factory_method: Target class method (e.g. ``Agent.from_urdf``).
        spawn_params: Dataclass instance with spawn parameters.
        aliases: ``{spawn_field: factory_param}`` — field name remapping.
        extra_kwargs: Additional kwargs to merge (e.g. ``{"sim_core": sc}``).

    Returns:
        Dict of keyword arguments ready for ``factory_method(**result)``.

    Example::

        kwargs = forward_spawn_params(
            Agent.from_urdf, spawn_params,
            aliases={"initial_pose": "pose"},
            extra_kwargs={"sim_core": sim},
        )
        agent = Agent.from_urdf(**kwargs)
    """
    aliases = aliases or {}
    extra_kwargs = extra_kwargs or {}

    sig = inspect.signature(factory_method)
    valid_params = {p for p in sig.parameters if p not in ("cls", "self")}

    kwargs: Dict[str, Any] = {}
    for f in dataclasses.fields(spawn_params):
        target_name = aliases.get(f.name, f.name)
        if target_name in valid_params:
            kwargs[target_name] = getattr(spawn_params, f.name)

    kwargs.update(extra_kwargs)
    return kwargs
