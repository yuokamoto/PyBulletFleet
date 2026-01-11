#!/usr/bin/env python3
"""
Update performance_benchmark.py to support config file loading.
This script adds config file support while preserving the existing structure.
"""

import sys

# Read the original file
with open("performance_benchmark.py", "r") as f:
    content = f.read()

# Add yaml import after other imports
if "import yaml" not in content:
    content = content.replace("from typing import Dict, List", "from typing import Dict, List, Optional, Any\nimport yaml")

# Add config path constant
if "DEFAULT_CONFIG_PATH" not in content:
    content = content.replace(
        "DEFAULT_NUM_AGENTS = 1000",
        'DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "benchmark_config.yaml")\nDEFAULT_NUM_AGENTS = 1000',
    )

# Add load_config function after the helpers section header
load_config_func = '''
def load_config(config_path: str = DEFAULT_CONFIG_PATH, scenario: Optional[str] = None) -> Dict[str, Any]:
    """
    Load benchmark configuration from YAML file.

    Args:
        config_path: Path to configuration file
        scenario: Optional scenario name to override default config

    Returns:
        Configuration dictionary
    """
    if not os.path.exists(config_path):
        print(f"Warning: Config file not found: {config_path}")
        print("Using default configuration")
        return {}

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # If scenario specified, merge scenario config with base config
    if scenario and 'scenarios' in config and scenario in config['scenarios']:
        base_config = config.copy()
        scenario_config = config['scenarios'][scenario]

        # Deep merge scenario config into base config
        for key, value in scenario_config.items():
            if isinstance(value, dict) and key in base_config:
                base_config[key].update(value)
            else:
                base_config[key] = value

        print(f"Using scenario: {scenario}")
        return base_config

    return config

'''

if "def load_config(" not in content:
    content = content.replace(
        "# ==================== Helpers ====================\n",
        "# ==================== Helpers ====================\n" + load_config_func,
    )

# Update run_simulation_benchmark signature
old_sig = "def run_simulation_benchmark(num_agents: int, duration: float, gui: bool = False) -> dict:"
new_sig = "def run_simulation_benchmark(config: Dict[str, Any], num_agents: Optional[int] = None, duration: Optional[float] = None, gui: Optional[bool] = None) -> dict:"

if old_sig in content and new_sig not in content:
    content = content.replace(old_sig, new_sig)

print("Modified performance_benchmark.py successfully!")
print("Writing updated file...")

with open("performance_benchmark_updated.py", "w") as f:
    f.write(content)

print("Created: performance_benchmark_updated.py")
print("Please review the changes before replacing the original file.")
