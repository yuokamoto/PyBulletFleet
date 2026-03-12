"""
Tests for logging_utils module (lazy evaluation loggers).

Design intent & usage policy:
- LazyLogger/NamedLazyLogger only perform lazy evaluation if you pass a lambda.
- For lightweight constants or simple strings, pass them directly (no lambda needed).
- For expensive computations or side effects, always wrap in a lambda.
- These tests explicitly check that both lambda and string messages are supported, and that lazy evaluation works as intended.
- In real usage, you only need to use lambda for expensive or side-effecting computations.

This module tests:
- LazyLogger creation and initialization
- Lazy evaluation of log messages
- Log level filtering
- Helpers (get_lazy_logger, wrap_existing_logger)
- Performance benefit of lazy evaluation
"""

import logging
import time

import numpy as np
import pytest

from pybullet_fleet.logging_utils import (
    LazyLogger,
    NamedLazyLogger,
    get_lazy_logger,
    get_named_lazy_logger,
    wrap_existing_logger,
)


class TestLazyLoggerCreation:
    """Test LazyLogger initialization"""

    def test_create_with_name(self):
        """Test creating LazyLogger with name"""
        logger = LazyLogger(name="test_creation")

        assert logger.logger is not None
        assert logger.logger.name == "test_creation"

    def test_create_with_existing_logger(self):
        """Test creating LazyLogger by wrapping existing logger"""
        std_logger = logging.getLogger("test_wrap")
        logger = LazyLogger(logger=std_logger)

        assert logger.logger is std_logger

    def test_create_without_args_raises_error(self):
        """Test that creating LazyLogger without name or logger raises ValueError"""
        with pytest.raises(ValueError, match="Either 'name' or 'logger' must be provided"):
            LazyLogger()

    def test_get_lazy_logger_helper(self):
        """Test get_lazy_logger convenience function"""
        logger = get_lazy_logger("test_helper")

        assert isinstance(logger, LazyLogger)
        assert logger.logger.name == "test_helper"

    def test_wrap_existing_logger_helper(self):
        """Test wrap_existing_logger convenience function"""
        std_logger = logging.getLogger("test_wrap_helper")
        lazy = wrap_existing_logger(std_logger)

        assert isinstance(lazy, LazyLogger)
        assert lazy.logger is std_logger


class TestLazyLoggerOutput:
    """Test that LazyLogger produces correct log output"""

    def test_debug_message(self, caplog):
        """Test debug level lazy message"""
        logger = get_lazy_logger("test_debug")

        with caplog.at_level(logging.DEBUG, logger="test_debug"):
            logger.debug(lambda: "debug message")

        assert "debug message" in caplog.text

    def test_info_message(self, caplog):
        """Test info level lazy message"""
        logger = get_lazy_logger("test_info")

        with caplog.at_level(logging.INFO, logger="test_info"):
            logger.info(lambda: "info message")

        assert "info message" in caplog.text

    def test_warning_message(self, caplog):
        """Test warning level lazy message"""
        logger = get_lazy_logger("test_warning")

        with caplog.at_level(logging.WARNING, logger="test_warning"):
            logger.warning(lambda: "warning message")

        assert "warning message" in caplog.text

    def test_error_message(self, caplog):
        """Test error level lazy message"""
        logger = get_lazy_logger("test_error")

        with caplog.at_level(logging.ERROR, logger="test_error"):
            logger.error(lambda: "error message")

        assert "error message" in caplog.text

    def test_critical_message(self, caplog):
        """Test critical level lazy message"""
        logger = get_lazy_logger("test_critical")

        with caplog.at_level(logging.CRITICAL, logger="test_critical"):
            logger.critical(lambda: "critical message")

        assert "critical message" in caplog.text

    def test_plain_string_message(self, caplog):
        """Test that plain strings (not lambdas) also work"""
        logger = get_lazy_logger("test_plain")

        with caplog.at_level(logging.WARNING, logger="test_plain"):
            logger.warning("plain string warning")

        assert "plain string warning" in caplog.text

    def test_message_with_numpy_array(self, caplog):
        """Test lazy message containing numpy array formatting"""
        logger = get_lazy_logger("test_numpy")
        arr = np.array([1, 2, 3, 4, 5])

        with caplog.at_level(logging.DEBUG, logger="test_numpy"):
            logger.debug(lambda: f"Array: {arr}, mean: {arr.mean()}")

        assert "Array:" in caplog.text
        assert "mean:" in caplog.text


class TestLazyEvaluation:
    """Test that expensive formatting is skipped when level is disabled"""

    def test_lambda_not_called_when_level_disabled(self):
        """Test that lambda is NOT evaluated when log level is higher"""
        logger = get_lazy_logger("test_skip")
        logger.logger.setLevel(logging.INFO)

        call_count = 0

        def expensive_func():
            nonlocal call_count
            call_count += 1
            return f"Called {call_count} times"

        logger.debug(expensive_func)  # DEBUG < INFO, should NOT call

        assert call_count == 0, f"Expected 0 calls when level=INFO, got {call_count}"

    def test_lambda_called_when_level_enabled(self):
        """Test that lambda IS evaluated when log level allows it"""
        logger = get_lazy_logger("test_call")
        logger.logger.setLevel(logging.DEBUG)
        logger.logger.addHandler(logging.NullHandler())

        call_count = 0

        def expensive_func():
            nonlocal call_count
            call_count += 1
            return f"Called {call_count} times"

        logger.debug(expensive_func)

        assert call_count == 1, f"Expected 1 call when level=DEBUG, got {call_count}"

    def test_mixed_level_filtering(self):
        """Test that only enabled levels trigger evaluation"""
        logger = get_lazy_logger("test_mixed")
        logger.logger.setLevel(logging.INFO)
        logger.logger.addHandler(logging.NullHandler())

        call_count = 0

        def counting_func():
            nonlocal call_count
            call_count += 1
            return f"Call #{call_count}"

        logger.debug(counting_func)  # Should NOT call (DEBUG < INFO)
        assert call_count == 0, f"After debug (disabled): expected 0, got {call_count}"

        logger.info(counting_func)  # Should call
        assert call_count == 1, f"After info: expected 1, got {call_count}"

        logger.warning(counting_func)  # Should call
        assert call_count == 2, f"After warning: expected 2, got {call_count}"

        logger.debug(counting_func)  # Should NOT call
        assert call_count == 2, f"After debug (disabled): expected 2, got {call_count}"

    def test_is_enabled_for(self):
        """Test isEnabledFor method"""
        logger = get_lazy_logger("test_enabled")
        logger.logger.setLevel(logging.WARNING)

        assert logger.isEnabledFor(logging.WARNING) is True
        assert logger.isEnabledFor(logging.ERROR) is True
        assert logger.isEnabledFor(logging.DEBUG) is False
        assert logger.isEnabledFor(logging.INFO) is False


class TestNamedLazyLogger:
    """Test NamedLazyLogger with instance-level prefix"""

    def test_create_with_prefix(self):
        """NamedLazyLogger stores initial prefix."""
        log = NamedLazyLogger(name="test_named", prefix="[obj:1] ")
        assert log.prefix == "[obj:1] "

    def test_create_without_prefix(self):
        """Default prefix is empty string."""
        log = NamedLazyLogger(name="test_named_empty")
        assert log.prefix == ""

    def test_set_prefix(self):
        """set_prefix updates the prefix dynamically."""
        log = NamedLazyLogger(name="test_set_prefix")
        assert log.prefix == ""

        log.set_prefix("[Agent:5:robot_A] ")
        assert log.prefix == "[Agent:5:robot_A] "

    def test_prefix_in_output(self, caplog):
        """Prefix is prepended to every log message."""
        log = NamedLazyLogger(name="test_prefix_out", prefix="[Agent:3] ")

        with caplog.at_level(logging.DEBUG, logger="test_prefix_out"):
            log.info("Path complete")

        assert "[Agent:3] Path complete" in caplog.text

    def test_prefix_with_lazy_lambda(self, caplog):
        """Prefix works with lambda messages."""
        log = NamedLazyLogger(name="test_prefix_lazy", prefix="[obj:7] ")

        with caplog.at_level(logging.DEBUG, logger="test_prefix_lazy"):
            log.debug(lambda: "pos=[1.0, 2.0]")

        assert "[obj:7] pos=[1.0, 2.0]" in caplog.text

    def test_prefix_lazy_skipped_when_disabled(self):
        """Lambda is NOT evaluated when level is disabled, even with prefix."""
        log = NamedLazyLogger(name="test_prefix_skip", prefix="[X] ")
        log.logger.setLevel(logging.INFO)

        call_count = 0

        def expensive():
            nonlocal call_count
            call_count += 1
            return "expensive result"

        log.debug(expensive)  # DEBUG < INFO → skipped
        assert call_count == 0

    def test_prefix_updated_after_creation(self, caplog):
        """Prefix change is reflected in subsequent log messages."""
        log = NamedLazyLogger(name="test_prefix_update", prefix="[init] ")

        with caplog.at_level(logging.DEBUG, logger="test_prefix_update"):
            log.info("before")
            log.set_prefix("[Agent:99:picker] ")
            log.info("after")

        assert "[init] before" in caplog.text
        assert "[Agent:99:picker] after" in caplog.text

    def test_empty_prefix_no_extra_characters(self, caplog):
        """Empty prefix does not add spurious characters."""
        log = NamedLazyLogger(name="test_no_prefix")

        with caplog.at_level(logging.DEBUG, logger="test_no_prefix"):
            log.info("clean message")

        assert "clean message" in caplog.text
        # No leading bracket or space
        record = [r for r in caplog.records if r.name == "test_no_prefix"][0]
        assert record.message == "clean message"

    def test_get_named_lazy_logger_helper(self):
        """get_named_lazy_logger convenience function returns NamedLazyLogger."""
        log = get_named_lazy_logger("test_helper_named", prefix="[A:1] ")

        assert isinstance(log, NamedLazyLogger)
        assert log.prefix == "[A:1] "
        assert log.logger.name == "test_helper_named"

    def test_all_levels_include_prefix(self, caplog):
        """All log levels (debug/info/warning/error/critical) include prefix."""
        log = NamedLazyLogger(name="test_all_levels", prefix="[P] ")

        with caplog.at_level(logging.DEBUG, logger="test_all_levels"):
            log.debug("d")
            log.info("i")
            log.warning("w")
            log.error("e")
            log.critical("c")

        records = [r for r in caplog.records if r.name == "test_all_levels"]
        assert len(records) == 5
        for rec in records:
            assert rec.message.startswith("[P] "), f"Missing prefix in: {rec.message}"

    def test_isinstance_of_lazy_logger(self):
        """NamedLazyLogger is a subclass of LazyLogger."""
        log = NamedLazyLogger(name="test_isinstance")
        assert isinstance(log, LazyLogger)


class TestPerformance:
    """Test that lazy evaluation provides measurable performance benefit"""

    def test_lazy_faster_than_standard(self):
        """Test lazy logger is faster than standard logger for disabled levels"""
        num_iterations = 10000
        test_array = np.random.rand(100)

        # Standard logger: f-string always evaluated
        std_logger = logging.getLogger("perf_standard")
        std_logger.setLevel(logging.INFO)

        start = time.perf_counter()
        for i in range(num_iterations):
            std_logger.debug(f"Iteration {i}: array={test_array[:10]}, mean={test_array.mean()}")
        t_standard = time.perf_counter() - start

        # Lazy logger: lambda NOT evaluated when disabled
        lazy_logger = get_lazy_logger("perf_lazy")
        lazy_logger.logger.setLevel(logging.INFO)

        start = time.perf_counter()
        for i in range(num_iterations):
            lazy_logger.debug(lambda: f"Iteration {i}: array={test_array[:10]}, mean={test_array.mean()}")
        t_lazy = time.perf_counter() - start

        # Lazy should be faster (at least 2x speedup)
        speedup = t_standard / t_lazy
        assert speedup >= 2.0, (
            f"Expected at least 2x speedup, got {speedup:.1f}x "
            f"(standard={t_standard*1000:.2f}ms, lazy={t_lazy*1000:.2f}ms)"
        )
