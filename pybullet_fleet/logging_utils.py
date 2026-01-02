#!/usr/bin/env python3
"""
Logging utilities with lazy evaluation for expensive operations.

This module provides logging wrappers that prevent expensive string formatting
when the log level is not enabled, similar to the pattern used in action.py.
"""
import logging
from typing import Callable, Union, Tuple


class LazyLogger:
    """
    Logger wrapper with lazy evaluation for expensive formatting operations.

    This prevents expensive string formatting (e.g., NumPy array conversion)
    when the log level is not enabled.

    Usage:
        logger = LazyLogger(__name__)
        logger.debug(lambda: f"Expensive array: {numpy_array}")

    Or with the decorator:
        @lazy_log
        def my_logger():
            return logging.getLogger(__name__)

        logger = my_logger()
        logger.debug(lambda: f"Array: {arr}")
    """

    def __init__(self, name: str = None, logger: logging.Logger = None):
        """
        Initialize LazyLogger.

        Args:
            name: Logger name (passed to logging.getLogger)
            logger: Existing logger instance (alternative to name)
        """
        if logger is not None:
            self._logger = logger
        elif name is not None:
            self._logger = logging.getLogger(name)
        else:
            raise ValueError("Either 'name' or 'logger' must be provided")

    def _log_lazy(self, level: int, msg_func: Callable[[], str], *args, **kwargs):
        """
        Internal method to handle lazy logging.

        Args:
            level: Logging level (e.g., logging.DEBUG)
            msg_func: Callable that returns the log message
            *args, **kwargs: Additional arguments for logger
        """
        if self._logger.isEnabledFor(level):
            # Only evaluate the message if logging is enabled
            message = msg_func() if callable(msg_func) else msg_func
            self._logger.log(level, message, *args, **kwargs)

    def debug(self, msg_func: Union[Callable[[], str], str], *args, **kwargs):
        """
        Log debug message with lazy evaluation.

        Args:
            msg_func: Callable that returns message, or plain string
            *args, **kwargs: Additional arguments for logger
        """
        self._log_lazy(logging.DEBUG, msg_func, *args, **kwargs)

    def info(self, msg_func: Union[Callable[[], str], str], *args, **kwargs):
        """Log info message with lazy evaluation."""
        self._log_lazy(logging.INFO, msg_func, *args, **kwargs)

    def warning(self, msg_func: Union[Callable[[], str], str], *args, **kwargs):
        """Log warning message with lazy evaluation."""
        self._log_lazy(logging.WARNING, msg_func, *args, **kwargs)

    def error(self, msg_func: Union[Callable[[], str], str], *args, **kwargs):
        """Log error message with lazy evaluation."""
        self._log_lazy(logging.ERROR, msg_func, *args, **kwargs)

    def critical(self, msg_func: Union[Callable[[], str], str], *args, **kwargs):
        """Log critical message with lazy evaluation."""
        self._log_lazy(logging.CRITICAL, msg_func, *args, **kwargs)

    @property
    def logger(self) -> logging.Logger:
        """Get underlying logger instance."""
        return self._logger

    def isEnabledFor(self, level: int) -> bool:
        """Check if logging is enabled for the given level."""
        return self._logger.isEnabledFor(level)


def get_lazy_logger(name: str) -> LazyLogger:
    """
    Get a LazyLogger instance for the given name.

    This is the recommended way to create lazy loggers.

    Args:
        name: Logger name (typically __name__)

    Returns:
        LazyLogger instance

    Example:
        logger = get_lazy_logger(__name__)
        logger.debug(lambda: f"Expensive: {numpy_array}")
    """
    return LazyLogger(name=name)


def wrap_existing_logger(logger: logging.Logger) -> LazyLogger:
    """
    Wrap an existing logger with lazy evaluation.

    Args:
        logger: Existing logging.Logger instance

    Returns:
        LazyLogger wrapping the existing logger

    Example:
        standard_logger = logging.getLogger(__name__)
        lazy_logger = wrap_existing_logger(standard_logger)
        lazy_logger.debug(lambda: f"Array: {arr}")
    """
    return LazyLogger(logger=logger)


# Convenience function for quick migration
def migrate_logger(logger_name: str = None, logger_instance: logging.Logger = None) -> Tuple[logging.Logger, LazyLogger]:
    """
    Create both standard and lazy logger for gradual migration.

    This allows existing code to continue using the standard logger
    while new code can use the lazy logger.

    Args:
        logger_name: Logger name (for new logger)
        logger_instance: Existing logger instance

    Returns:
        Tuple of (standard_logger, lazy_logger)

    Example:
        logger, lazy_logger = migrate_logger(__name__)

        # Existing code (unchanged)
        logger.info("Simple message")

        # New code (lazy evaluation)
        lazy_logger.debug(lambda: f"Expensive: {arr}")
    """
    if logger_instance is not None:
        standard = logger_instance
        lazy = wrap_existing_logger(logger_instance)
    elif logger_name is not None:
        standard = logging.getLogger(logger_name)
        lazy = get_lazy_logger(logger_name)
    else:
        raise ValueError("Either logger_name or logger_instance must be provided")

    return standard, lazy
