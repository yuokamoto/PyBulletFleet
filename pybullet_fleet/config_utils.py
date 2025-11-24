"""
config_utils.py
Utility functions for loading and merging configuration files.
"""

import os
from typing import Any, Dict, List, Union

import yaml


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

    # Load first config as base
    base_path = config_paths[0]
    if not os.path.exists(base_path):
        raise FileNotFoundError(f"Config file not found: {base_path}")

    with open(base_path, "r") as f:
        merged_config = yaml.safe_load(f)

    # Merge additional configs
    for config_path in config_paths[1:]:
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with open(config_path, "r") as f:
            additional_config = yaml.safe_load(f)

        # Merge (later config overrides earlier)
        merged_config = merge_configs(merged_config, additional_config)

    return merged_config
