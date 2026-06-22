"""Tests for handler class resolution — no ROS 2 dependency.

Validates the 3-tier fallback:
  1. agent.user_data["handler_class"]  (per-robot, from spawn params)
  2. handler_map (exact name or regex pattern match)
  3. default_class (class-level default)
"""

import re
from unittest.mock import MagicMock

import pytest

from pybullet_fleet_ros.handler_registry import (
    load_handler_map_from_config,
    resolve_handler_classes,
)


# ---- Stubs (no ROS deps) ----


class _DefaultHandler:
    """Stand-in for RobotHandler in tests."""

    def __init__(self, node, agent, **kwargs):
        pass


class _CustomHandler:
    """Custom handler stub."""

    def __init__(self, node, agent, **kwargs):
        pass


class _AnotherHandler:
    """Second custom handler stub."""

    def __init__(self, node, agent, **kwargs):
        pass


class _ThirdHandler:
    """Third custom handler stub."""

    def __init__(self, node, agent, **kwargs):
        pass


def _make_agent(name="robot0", user_data=None):
    agent = MagicMock()
    agent.name = name
    agent.user_data = user_data or {}
    return agent


# ---- Tests ----


class TestResolveHandlerClasses:
    """Tests for resolve_handler_classes — 3-tier fallback, returns list."""

    def test_default_returns_single_default(self):
        """With no overrides, returns [default_class]."""
        result = resolve_handler_classes(_make_agent(), handler_map={}, default_classes=[_DefaultHandler])
        assert result == [_DefaultHandler]

    def test_handler_map_exact_match_single(self):
        """handler_map with single class returns [cls]."""
        result = resolve_handler_classes(
            _make_agent("arm_0"),
            handler_map={"arm_0": _CustomHandler},
            default_classes=[_DefaultHandler],
        )
        assert result == [_CustomHandler]

    def test_handler_map_exact_match_list(self):
        """handler_map with list of classes returns all."""
        result = resolve_handler_classes(
            _make_agent("arm_0"),
            handler_map={"arm_0": [_CustomHandler, _AnotherHandler]},
            default_classes=[_DefaultHandler],
        )
        assert result == [_CustomHandler, _AnotherHandler]

    def test_handler_map_pattern_match(self):
        """handler_map with compiled regex key matches agent name."""
        result = resolve_handler_classes(
            _make_agent("arm_5"),
            handler_map={re.compile(r"^arm_"): [_CustomHandler]},
            default_classes=[_DefaultHandler],
        )
        assert result == [_CustomHandler]

    def test_handler_map_pattern_no_match_falls_back(self):
        """Pattern that doesn't match falls back to default."""
        result = resolve_handler_classes(
            _make_agent("mobile_1"),
            handler_map={re.compile(r"^arm_"): _CustomHandler},
            default_classes=[_DefaultHandler],
        )
        assert result == [_DefaultHandler]

    def test_exact_and_pattern_are_additive(self):
        """Exact match + pattern match accumulate handlers."""
        result = resolve_handler_classes(
            _make_agent("arm_0"),
            handler_map={
                "arm_0": _CustomHandler,
                re.compile(r"^arm_"): _AnotherHandler,
            },
            default_classes=[_DefaultHandler],
        )
        assert _CustomHandler in result
        assert _AnotherHandler in result
        assert len(result) == 2

    def test_user_data_additive_with_handler_map(self):
        """user_data['handler_class'] is combined with handler_map matches."""
        agent = _make_agent("arm_0", user_data={"handler_class": _CustomHandler})

        result = resolve_handler_classes(
            agent,
            handler_map={"arm_0": _AnotherHandler},
            default_classes=[_DefaultHandler],
        )
        assert _CustomHandler in result
        assert _AnotherHandler in result
        assert _DefaultHandler not in result

    def test_user_data_list(self):
        """user_data['handler_class'] can be a list."""
        agent = _make_agent(user_data={"handler_class": [_CustomHandler, _AnotherHandler]})

        result = resolve_handler_classes(agent, handler_map={}, default_classes=[_DefaultHandler])
        assert result == [_CustomHandler, _AnotherHandler]

    def test_user_data_dotted_string_import(self):
        """user_data['handler_class'] as dotted string imports the class."""
        agent = _make_agent(user_data={"handler_class": f"{__name__}._CustomHandler"})

        result = resolve_handler_classes(agent, handler_map={}, default_classes=[_DefaultHandler])
        assert result == [_CustomHandler]

    def test_user_data_list_of_strings(self):
        """user_data['handler_class'] as list of dotted strings."""
        agent = _make_agent(user_data={"handler_class": [f"{__name__}._CustomHandler", f"{__name__}._AnotherHandler"]})

        result = resolve_handler_classes(agent, handler_map={}, default_classes=[_DefaultHandler])
        assert result == [_CustomHandler, _AnotherHandler]

    def test_user_data_invalid_import_raises(self):
        """user_data['handler_class'] with invalid dotted path raises ImportError."""
        agent = _make_agent(user_data={"handler_class": "nonexistent.module.FakeHandler"})

        with pytest.raises((ImportError, ModuleNotFoundError)):
            resolve_handler_classes(agent, handler_map={}, default_classes=[_DefaultHandler])

    def test_agent_without_name_uses_default(self):
        """Agent with name=None falls back to default (no handler_map match)."""
        result = resolve_handler_classes(
            _make_agent(name=None),
            handler_map={"arm_0": _CustomHandler},
            default_classes=[_DefaultHandler],
        )
        assert result == [_DefaultHandler]

    def test_multiple_defaults(self):
        """Multiple default_classes are returned when no override matches."""
        result = resolve_handler_classes(_make_agent(), handler_map={}, default_classes=[_DefaultHandler, _CustomHandler])
        assert result == [_DefaultHandler, _CustomHandler]

    def test_multiple_patterns_accumulate(self):
        """Multiple matching patterns all contribute handlers."""
        result = resolve_handler_classes(
            _make_agent("arm_special"),
            handler_map={
                re.compile(r"^arm_"): _CustomHandler,
                re.compile(r"_special$"): _AnotherHandler,
            },
            default_classes=[_DefaultHandler],
        )
        assert _CustomHandler in result
        assert _AnotherHandler in result
        assert len(result) == 2

    def test_all_three_tiers_accumulate(self):
        """user_data + exact + pattern all accumulate."""
        agent = _make_agent("arm_0", user_data={"handler_class": _ThirdHandler})
        result = resolve_handler_classes(
            agent,
            handler_map={
                "arm_0": _CustomHandler,
                re.compile(r"^arm_"): _AnotherHandler,
            },
            default_classes=[_DefaultHandler],
        )
        assert _ThirdHandler in result
        assert _CustomHandler in result
        assert _AnotherHandler in result
        assert _DefaultHandler not in result
        assert len(result) == 3

    def test_no_duplicates(self):
        """Same class from multiple sources appears only once."""
        agent = _make_agent("arm_0", user_data={"handler_class": _CustomHandler})
        result = resolve_handler_classes(
            agent,
            handler_map={"arm_0": _CustomHandler},
            default_classes=[_DefaultHandler],
        )
        assert result.count(_CustomHandler) == 1

    def test_handler_map_match_suppresses_default(self):
        """Any handler_map match means default is NOT added."""
        result = resolve_handler_classes(
            _make_agent("arm_0"),
            handler_map={re.compile(r"^arm_"): _CustomHandler},
            default_classes=[_DefaultHandler],
        )
        assert result == [_CustomHandler]
        assert _DefaultHandler not in result


class TestLoadHandlerMapFromConfig:
    """Tests for load_handler_map_from_config — YAML-driven handler registration."""

    def test_exact_name_mapping_single(self):
        """Exact name key with single string maps to [class]."""
        config = {"arm_0": f"{__name__}._CustomHandler"}
        result = load_handler_map_from_config(config)
        assert result["arm_0"] == [_CustomHandler]

    def test_exact_name_mapping_list(self):
        """Exact name key with list of strings maps to [class, class]."""
        config = {"arm_0": [f"{__name__}._CustomHandler", f"{__name__}._AnotherHandler"]}
        result = load_handler_map_from_config(config)
        assert result["arm_0"] == [_CustomHandler, _AnotherHandler]

    def test_glob_pattern_converted_to_regex(self):
        """Glob-style key (e.g. 'arm_*') is converted to a compiled regex."""
        config = {"arm_*": f"{__name__}._CustomHandler"}
        result = load_handler_map_from_config(config)

        # Should have one regex key, no plain string "arm_*"
        assert "arm_*" not in result
        regex_keys = [k for k in result if isinstance(k, re.Pattern)]
        assert len(regex_keys) == 1
        assert regex_keys[0].search("arm_5")
        assert not regex_keys[0].search("mobile_1")

    def test_multiple_entries(self):
        """Multiple entries produce correct mapping."""
        config = {
            "arm_0": f"{__name__}._CustomHandler",
            "mobile_*": f"{__name__}._AnotherHandler",
        }
        result = load_handler_map_from_config(config)
        assert result["arm_0"] == [_CustomHandler]
        regex_keys = [k for k in result if isinstance(k, re.Pattern)]
        assert len(regex_keys) == 1
        assert result[regex_keys[0]] == [_AnotherHandler]

    def test_invalid_dotted_path_raises(self):
        """Invalid dotted path raises ImportError."""
        config = {"robot_0": "nonexistent.module.Handler"}
        with pytest.raises((ImportError, ModuleNotFoundError)):
            load_handler_map_from_config(config)

    def test_invalid_path_format_raises(self):
        """Non-dotted string raises ValueError."""
        config = {"robot_0": "NotADottedPath"}
        with pytest.raises(ValueError, match="dotted Python path"):
            load_handler_map_from_config(config)

    def test_empty_config_returns_empty(self):
        """Empty dict returns empty HandlerMap."""
        result = load_handler_map_from_config({})
        assert result == {}

    def test_integrates_with_resolve(self):
        """Config-loaded handler_map works with resolve_handler_classes."""
        config = {"arm_*": f"{__name__}._CustomHandler"}
        handler_map = load_handler_map_from_config(config)

        result = resolve_handler_classes(
            _make_agent("arm_3"),
            handler_map=handler_map,
            default_classes=[_DefaultHandler],
        )
        assert result == [_CustomHandler]

    def test_list_config_integrates_with_resolve(self):
        """Config with list value works with resolve_handler_classes."""
        config = {"arm_*": [f"{__name__}._CustomHandler", f"{__name__}._AnotherHandler"]}
        handler_map = load_handler_map_from_config(config)

        result = resolve_handler_classes(
            _make_agent("arm_3"),
            handler_map=handler_map,
            default_classes=[_DefaultHandler],
        )
        assert result == [_CustomHandler, _AnotherHandler]
