# How-to Guides

Practical guides for common tasks.

## Profiling & Diagnostics Overview

| Guide | Measures | Detects | Tool | Overhead |
|-------|----------|---------|------|----------|
| [Time Profiling](time-profiling) | Execution time (ms) | Performance bottlenecks | `time.perf_counter()` | < 0.1% CPU |
| [Memory Profiling](memory-profiling) | Memory usage (MB) | Memory leaks, high usage | `tracemalloc` (built-in) | ~5–10% memory |
| [Custom Profiling](custom-profiling) | User-defined metrics | Application-specific | Callback-based | Depends on metric |
| [Logging](logging) | Log messages | Configuration & runtime issues | Python `logging` | Negligible |
| [Spatial Hash Config](spatial-hash-config) | Collision cell size | Sub-optimal grid partitioning | Config / API | None |

> **Tip:** Time and memory profiling can be enabled simultaneously for
> comprehensive analysis. Both share the `profiling_interval` setting.

```{toctree}
:maxdepth: 2

time-profiling
memory-profiling
custom-profiling
logging
spatial-hash-config
```
