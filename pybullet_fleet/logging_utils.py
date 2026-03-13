#!/usr/bin/env python3
"""
Logging utilities with lazy evaluation for expensive operations.

Design intent & usage policy:
- The standard Python logger always evaluates f-strings and expensive computations, even if the log level is disabled.
- LazyLogger/NamedLazyLogger allow you to pass a lambda, so the message is only evaluated if the log level is enabled.
- This prevents unnecessary computation and side effects, improving performance.

Usage policy:
- For lightweight constants or simple strings, pass them directly (no lambda needed).
- For expensive computations or side effects, always wrap in a lambda.
- Both are supported for flexibility and easier migration from standard logging.

See test_logging_utils.py for more usage and test examples.
"""
import logging
from typing import Callable, Union


class LazyLogger:
    """
    Logger wrapper with lazy evaluation for expensive formatting operations.

    This prevents expensive string formatting (e.g., NumPy array conversion)
    when the log level is not enabled.

    Usage::

        logger = LazyLogger(__name__)
        logger.debug(lambda: f"Expensive array: {numpy_array}")

    Or with the decorator::

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
        r"""
        Internal method to handle lazy logging.

        Args:
            level: Logging level (e.g., logging.DEBUG)
            msg_func: Callable that returns the log message
            \*args, \*\*kwargs: Additional arguments for logger
        """
        if self._logger.isEnabledFor(level):
            # Only evaluate the message if logging is enabled
            message = msg_func() if callable(msg_func) else msg_func
            self._logger.log(level, message, *args, **kwargs)

    def debug(self, msg_func: Union[Callable[[], str], str], *args, **kwargs):
        r"""
        Log debug message with lazy evaluation.

        Args:
            msg_func: Callable that returns message, or plain string
            \*args, \*\*kwargs: Additional arguments for logger
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


class NamedLazyLogger(LazyLogger):
    """
    LazyLogger with a dynamic prefix for instance-level identification.

    Designed for SimObject/Agent/Action instances where each log line
    should include identifiers like object_id and name, without
    manually embedding them in every log call.

    The prefix can be updated at any time via :meth:`set_prefix` (e.g.
    after ``object_id`` is assigned by sim_core).

    Usage::

        # In SimObject.__init__
        self._log = NamedLazyLogger(__name__)
        self._log.set_prefix(f"[obj:{self.object_id}] ")

        # In Agent.__init__ (after super().__init__)
        self._log.set_prefix(f"[Agent:{self.object_id}:{self.name}] ")

        # Log calls — prefix is prepended automatically
        self._log.info("Path complete")
        # => [Agent:3:robot_A] Path complete

        self._log.debug(lambda: f"pos={self.get_pose().position[:2]}")
        # => [Agent:3:robot_A] pos=[1.0, 2.0]   (only evaluated if DEBUG enabled)

    Note:
        The prefix is stored as a plain string and prepended inside
        ``_log_lazy`` only when the level is enabled, so the cost is
        negligible when the message is filtered out.
    """

    def __init__(self, name: str = None, logger: logging.Logger = None, prefix: str = ""):
        """
        Initialize NamedLazyLogger.

        Args:
            name: Logger name (passed to logging.getLogger)
            logger: Existing logger instance (alternative to name)
            prefix: Initial prefix string (default: "")
        """
        super().__init__(name=name, logger=logger)
        self._prefix = prefix

    def set_prefix(self, prefix: str):
        """
        Update the prefix prepended to every log message.

        Call this whenever the identifying information changes
        (e.g. after object_id is assigned).

        Args:
            prefix: New prefix string (include trailing space if desired)
        """
        self._prefix = prefix

    @property
    def prefix(self) -> str:
        """Get current prefix string."""
        return self._prefix

    def _log_lazy(self, level: int, msg_func: Callable[[], str], *args, **kwargs):
        """Override to prepend prefix to every message."""
        if self._logger.isEnabledFor(level):
            message = msg_func() if callable(msg_func) else msg_func
            self._logger.log(level, f"{self._prefix}{message}", *args, **kwargs)


def get_named_lazy_logger(name: str, prefix: str = "") -> NamedLazyLogger:
    """
    Get a NamedLazyLogger instance.

    This is the recommended way to create instance-level loggers for
    SimObject / Agent / Action classes.

    Args:
        name: Logger name (typically __name__)
        prefix: Initial prefix (default: "")

    Returns:
        NamedLazyLogger instance

    Example::

        # In __init__
        self._log = get_named_lazy_logger(__name__, f"[Agent:{self.object_id}] ")
        self._log.debug(lambda: f"pos={pos}")
    """
    return NamedLazyLogger(name=name, prefix=prefix)


def get_lazy_logger(name: str) -> LazyLogger:
    """
    Get a LazyLogger instance for the given name.

    This is the recommended way to create lazy loggers.

    Args:
        name: Logger name (typically __name__)

    Returns:
        LazyLogger instance

    Example::

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

    Example::

        standard_logger = logging.getLogger(__name__)
        lazy_logger = wrap_existing_logger(standard_logger)
        lazy_logger.debug(lambda: f"Array: {arr}")
    """
    return LazyLogger(logger=logger)
