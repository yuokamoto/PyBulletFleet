# How-to Guides

Practical guides for common tasks.

**📘 For Simulation Users** — building and configuring your own simulation:

| Guide | What It Covers |
|-------|----------------|
| [Collision Configuration](collision-config) | Detection method, per-object modes, margin, cell size, multi-cell threshold |
| [Custom Profiling](custom-profiling) | Adding your own profiling metrics via Agent/SimulationCore subclasses |
| [Capturing Demos](capturing-demos) | Recording demo videos, `demos.yaml` format, centralized defaults (`_defaults.py`) |

**🔧 For PyBulletFleet Developers** — developing and debugging the framework:

| Guide | Measures | Detects | Tool | Overhead |
|-------|----------|---------|------|----------|
| [Time Profiling](time-profiling) | Execution time (ms) | Performance bottlenecks | `time.perf_counter()` | < 0.1% CPU |
| [Memory Profiling](memory-profiling) | Memory usage (MB) | Memory leaks, high usage | `tracemalloc` (built-in) | ~5–10% memory |
| [Logging](logging) | Log messages | Configuration & runtime issues | Python `logging` | Negligible |

> **Tip:** Time and memory profiling can be enabled simultaneously for
> comprehensive analysis. Both share the `profiling_interval` setting.

```{toctree}
:maxdepth: 2
:caption: For Simulation Users

collision-config
custom-profiling
capturing-demos
```

```{toctree}
:maxdepth: 2
:caption: For PyBulletFleet Developers

time-profiling
memory-profiling
logging
```
