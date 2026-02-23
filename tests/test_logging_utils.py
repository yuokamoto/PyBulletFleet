"""
Tests for logging_utils module (lazy evaluation loggers).

This module tests:
- LazyLogger creation and initialization
- Lazy evaluation of log messages
- Log level filtering
- Migration helpers (get_lazy_logger, wrap_existing_logger, migrate_logger)
- Performance benefit of lazy evaluation
"""

import logging
import time

import numpy as np
import pytest

from pybullet_fleet.logging_utils import (
    LazyLogger,
    get_lazy_logger,
    migrate_logger,
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
        logger.info(counting_func)  # Should call
        logger.warning(counting_func)  # Should call
        logger.debug(counting_func)  # Should NOT call

        assert call_count == 2, f"Expected 2 calls, got {call_count}"

    def test_is_enabled_for(self):
        """Test isEnabledFor method"""
        logger = get_lazy_logger("test_enabled")
        logger.logger.setLevel(logging.WARNING)

        assert logger.isEnabledFor(logging.WARNING) is True
        assert logger.isEnabledFor(logging.ERROR) is True
        assert logger.isEnabledFor(logging.DEBUG) is False
        assert logger.isEnabledFor(logging.INFO) is False


class TestMigrateLogger:
    """Test migrate_logger helper function"""

    def test_migrate_with_name(self):
        """Test migrate_logger creates both standard and lazy loggers"""
        std_logger, lazy_logger = migrate_logger(logger_name="test_migrate")

        assert isinstance(std_logger, logging.Logger)
        assert isinstance(lazy_logger, LazyLogger)
        assert std_logger.name == "test_migrate"

    def test_migrate_with_existing_logger(self):
        """Test migrate_logger wraps an existing logger"""
        existing = logging.getLogger("test_existing")
        std_logger, lazy_logger = migrate_logger(logger_instance=existing)

        assert std_logger is existing
        assert lazy_logger.logger is existing

    def test_migrate_without_args_raises_error(self):
        """Test that migrate_logger without args raises ValueError"""
        with pytest.raises(ValueError):
            migrate_logger()

    def test_migrate_both_loggers_output(self, caplog):
        """Test that both loggers from migrate produce output"""
        std_logger, lazy_logger = migrate_logger(logger_name="test_both")

        with caplog.at_level(logging.DEBUG, logger="test_both"):
            std_logger.info("standard message")
            lazy_logger.debug(lambda: "lazy message")

        assert "standard message" in caplog.text
        assert "lazy message" in caplog.text


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
