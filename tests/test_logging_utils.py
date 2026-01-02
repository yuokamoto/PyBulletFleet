#!/usr/bin/env python3
"""
Test logging_utils.py performance and functionality.
"""
import logging
import time
import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pybullet_fleet.logging_utils import get_lazy_logger, migrate_logger


def benchmark_standard_logger(num_iterations=10000):
    """Benchmark standard logger with array formatting."""
    logger = logging.getLogger("standard_benchmark")
    logger.setLevel(logging.INFO)  # DEBUG messages will be skipped

    test_array = np.random.rand(100)

    start = time.perf_counter()
    for i in range(num_iterations):
        # This will evaluate f-string even though DEBUG is disabled
        logger.debug(f"Iteration {i}: array={test_array[:10]}, mean={test_array.mean()}")
    elapsed = time.perf_counter() - start

    return elapsed


def benchmark_standard_with_check(num_iterations=10000):
    """Benchmark standard logger with manual isEnabledFor check."""
    logger = logging.getLogger("standard_check_benchmark")
    logger.setLevel(logging.INFO)  # DEBUG messages will be skipped

    test_array = np.random.rand(100)

    start = time.perf_counter()
    for i in range(num_iterations):
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(f"Iteration {i}: array={test_array[:10]}, mean={test_array.mean()}")
    elapsed = time.perf_counter() - start

    return elapsed


def benchmark_lazy_logger(num_iterations=10000):
    """Benchmark lazy logger with lambda."""
    logger = get_lazy_logger("lazy_benchmark")
    logger.logger.setLevel(logging.INFO)  # DEBUG messages will be skipped

    test_array = np.random.rand(100)

    start = time.perf_counter()
    for i in range(num_iterations):
        # Lambda is not called when DEBUG is disabled
        logger.debug(lambda: f"Iteration {i}: array={test_array[:10]}, mean={test_array.mean()}")
    elapsed = time.perf_counter() - start

    return elapsed


def test_functionality():
    """Test that lazy logger works correctly."""
    print("\n" + "=" * 70)
    print("Testing LazyLogger Functionality")
    print("=" * 70)

    # Setup handler to capture output
    import io

    log_capture = io.StringIO()
    handler = logging.StreamHandler(log_capture)
    handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter("%(levelname)s - %(message)s")
    handler.setFormatter(formatter)

    # Test 1: Basic functionality
    logger = get_lazy_logger("test_basic")
    logger.logger.addHandler(handler)
    logger.logger.setLevel(logging.DEBUG)

    test_array = np.array([1, 2, 3, 4, 5])

    logger.debug(lambda: f"Array: {test_array}")
    logger.info(lambda: f"Mean: {test_array.mean()}")
    logger.warning("Simple warning")

    output = log_capture.getvalue()
    print("\nCaptured log output:")
    print(output)

    assert "Array:" in output
    assert "Mean:" in output
    assert "Simple warning" in output
    print("✅ Basic functionality test passed")

    # Test 2: Level filtering
    log_capture.truncate(0)
    log_capture.seek(0)
    logger.logger.setLevel(logging.INFO)

    call_count = 0

    def expensive_func():
        nonlocal call_count
        call_count += 1
        return f"Called {call_count} times"

    logger.debug(expensive_func)  # Should NOT call expensive_func
    logger.info(expensive_func)  # Should call expensive_func

    assert call_count == 1, f"Expected 1 call, got {call_count}"
    print("✅ Level filtering test passed")

    # Test 3: Migration helper
    print("\n" + "-" * 70)
    print("Testing migration helper")
    print("-" * 70)

    std_logger, lazy_logger = migrate_logger("test_migration")
    std_logger.setLevel(logging.DEBUG)
    std_logger.addHandler(handler)

    log_capture.truncate(0)
    log_capture.seek(0)

    std_logger.info("Standard message")
    lazy_logger.debug(lambda: f"Lazy message: {test_array}")

    output = log_capture.getvalue()
    print(output)

    assert "Standard message" in output
    assert "Lazy message" in output
    print("✅ Migration test passed")


def test_performance():
    """Test performance improvement."""
    print("\n" + "=" * 70)
    print("Performance Benchmark (10,000 iterations)")
    print("=" * 70)

    print("\nSetup: Logger level=INFO, logging DEBUG messages (will be filtered)")
    print("Each DEBUG message formats NumPy array and calculates mean")

    print("\n1. Standard logger (no check)...")
    t_standard = benchmark_standard_logger()
    print(f"   Time: {t_standard*1000:.2f}ms")

    print("\n2. Standard logger (with isEnabledFor check)...")
    t_check = benchmark_standard_with_check()
    print(f"   Time: {t_check*1000:.2f}ms")
    print(f"   Speedup vs standard: {t_standard/t_check:.1f}x")

    print("\n3. Lazy logger (with lambda)...")
    t_lazy = benchmark_lazy_logger()
    print(f"   Time: {t_lazy*1000:.2f}ms")
    print(f"   Speedup vs standard: {t_standard/t_lazy:.1f}x")
    print(f"   Overhead vs check: {t_lazy/t_check:.2f}x")

    print("\n" + "=" * 70)
    print("Summary:")
    print("=" * 70)
    print(f"Standard logger:           {t_standard*1000:.2f}ms (baseline)")
    print(f"Standard with check:       {t_check*1000:.2f}ms ({(1-t_check/t_standard)*100:.1f}% faster)")
    print(f"Lazy logger:               {t_lazy*1000:.2f}ms ({(1-t_lazy/t_standard)*100:.1f}% faster)")
    print(f"\nConclusion: Lazy logger is ~{t_standard/t_lazy:.0f}x faster for disabled log levels")


if __name__ == "__main__":
    # Suppress actual log output during benchmarks
    logging.basicConfig(level=logging.CRITICAL)

    test_functionality()
    test_performance()

    print("\n" + "=" * 70)
    print("All tests passed! ✅")
    print("=" * 70)
