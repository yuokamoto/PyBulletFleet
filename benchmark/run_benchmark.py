#!/usr/bin/env python3
"""
run_benchmark.py
Meta-script for running performance benchmarks with process isolation and statistical analysis.

Features:
- Runs benchmark worker in isolated child processes
- Supports both mobile (simple cube) and arm (robot arm) benchmarks
- Collects statistics (median, mean, stdev) from multiple repetitions
- Supports sweep across multiple agent/arm counts
- Supports multiple scenarios comparison for both types
- Clean separation of concerns (no self-recursion)

Usage:
    # Mobile benchmarks (default)
    python benchmark/run_benchmark.py --agents 1000 --duration 10 --repetitions 3
    python benchmark/run_benchmark.py --sweep 100 500 1000 2000
    python benchmark/run_benchmark.py --compare no_collision collision_10hz

    # Arm benchmarks
    python benchmark/run_benchmark.py --type arm --agents 10 --duration 5
    python benchmark/run_benchmark.py --type arm --sweep 1 10 50 100
    python benchmark/run_benchmark.py --type arm --compare physics kinematic
"""
import os
import sys
import json
import subprocess
import argparse
import statistics
from typing import List, Optional

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


# ---------------------------------------------------------------------------
# Worker dispatch
# ---------------------------------------------------------------------------

# Map benchmark type → worker script filename
_WORKER_SCRIPTS = {
    "mobile": "mobile_benchmark.py",
    "arm": "arm_benchmark.py",
}


def _parse_worker_json(stdout: str) -> dict:
    """Extract JSON dict from worker stdout.

    PyBullet's C library may print warnings (b3Warning, build time) to stdout
    before the JSON line.  We find the last ``{...}`` block on a single line.
    """
    # Try the whole thing first (fast path)
    stripped = stdout.strip()
    try:
        return json.loads(stripped)
    except json.JSONDecodeError:
        pass

    # Fallback: find the last line that looks like JSON
    for line in reversed(stripped.splitlines()):
        line = line.strip()
        if line.startswith("{") and line.endswith("}"):
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue

    # Last resort: PyBullet C warnings may be concatenated on the same line
    # as the JSON output (e.g. "...end_effector{"num_arms":...}").
    # Scan backwards for each '{' and try to parse from there.
    search_from = len(stripped)
    while True:
        idx = stripped.rfind("{", 0, search_from)
        if idx < 0:
            break
        candidate = stripped[idx:]
        try:
            return json.loads(candidate)
        except json.JSONDecodeError:
            search_from = idx
            continue

    raise json.JSONDecodeError("No valid JSON found in worker output", stdout, 0)


def run_worker(
    num_agents: int,
    duration: float,
    benchmark_type: str = "mobile",
    *,
    gui: bool = False,
    config_path: Optional[str] = None,
    scenario: Optional[str] = None,
) -> dict:
    """Run benchmark worker in isolated child process.

    Args:
        num_agents: Number of agents/arms.
        duration: Simulation duration.
        benchmark_type: ``"mobile"`` or ``"arm"``.
        gui: Enable GUI (mobile only).
        config_path: Path to config file.
        scenario: Scenario name (e.g., ``"physics"``, ``"kinematic"``).

    Returns:
        Benchmark results dictionary.
    """
    script = _WORKER_SCRIPTS[benchmark_type]
    script_path = os.path.join(os.path.dirname(__file__), script)

    cmd = [
        sys.executable,
        script_path,
        "--agents",
        str(num_agents),
        "--duration",
        str(duration),
    ]
    if gui and benchmark_type == "mobile":
        cmd.append("--gui")

    if config_path:
        cmd.extend(["--config", config_path])
    if scenario:
        cmd.extend(["--scenario", scenario])

    proc = subprocess.run(cmd, capture_output=True, text=True)

    if proc.returncode != 0:
        raise RuntimeError(
            f"Worker failed ({benchmark_type}, n={num_agents}, scenario={scenario})\n"
            f"STDOUT:\n{proc.stdout}\n"
            f"STDERR:\n{proc.stderr}\n"
        )

    try:
        return _parse_worker_json(proc.stdout)
    except json.JSONDecodeError as e:
        raise RuntimeError(f"Failed to parse worker JSON: {e}\n" f"Raw stdout:\n{proc.stdout}\n")


# ---------------------------------------------------------------------------
# Multi-run aggregation
# ---------------------------------------------------------------------------


def run_multiple(
    num_agents: int,
    duration: float,
    num_reps: int,
    benchmark_type: str = "mobile",
    *,
    gui: bool = False,
    config_path: Optional[str] = None,
    scenario: Optional[str] = None,
) -> dict:
    """Run benchmark multiple times and compute statistics."""
    results = []

    label_parts = [f"{num_agents} {'arms' if benchmark_type == 'arm' else 'agents'}", f"{duration}s"]
    if scenario:
        label_parts.append(f"scenario={scenario}")

    print(f"\n{'='*70}")
    print(f"Benchmark: {', '.join(label_parts)}")
    print(f"{'='*70}")

    for i in range(num_reps):
        print(f"  Run {i+1}/{num_reps}...", end="", flush=True)
        r = run_worker(
            num_agents,
            duration,
            benchmark_type,
            gui=gui,
            config_path=config_path,
            scenario=scenario,
        )
        results.append(r)
        print(" done")

    def stats(xs):
        if len(xs) == 0:
            return {"median": 0.0, "mean": 0.0, "stdev": 0.0, "min": 0.0, "max": 0.0}
        return {
            "median": statistics.median(xs),
            "mean": statistics.mean(xs),
            "stdev": statistics.stdev(xs) if len(xs) > 1 else 0.0,
            "min": min(xs),
            "max": max(xs),
        }

    # Aggregate results — use keys that both workers provide
    aggregated = {
        "num_agents": num_agents,
        "benchmark_type": benchmark_type,
        "duration_s": duration,
        "scenario": scenario,
        "num_reps": num_reps,
        "gui": gui,
        "spawn_time_s": stats([r["spawn_time_s"] for r in results]),
        "spawn_cpu_s": stats([r["spawn_cpu_s"] for r in results]),
        "spawn_cpu_percent": stats([r["spawn_cpu_percent"] for r in results]),
        "simulation_wall_s": stats([r["simulation_wall_s"] for r in results]),
        "simulation_cpu_s": stats([r["simulation_cpu_s"] for r in results]),
        "simulation_cpu_percent": stats([r["simulation_cpu_percent"] for r in results]),
        "real_time_factor": stats([r["real_time_factor"] for r in results]),
        "avg_step_time_ms": stats([r["avg_step_time_ms"] for r in results]),
        "system_info": results[0]["system_info"] if results else {},
        "mem_spawn_rss_mb": stats([r["mem_spawn_mb"]["rss_mb"] for r in results]),
        "mem_total_rss_mb": stats([r["mem_total_mb"]["rss_mb"] for r in results]),
    }

    # Mobile-specific: expected_steps
    if "expected_steps" in results[0]:
        aggregated["expected_steps"] = results[0]["expected_steps"]

    # Arm-specific: total_joints, mode (from worker result)
    if benchmark_type == "arm":
        aggregated["mode"] = results[0].get("mode", "N/A") if results else "N/A"
        aggregated["timestep"] = results[0].get("timestep", 0.01) if results else 0.01
        aggregated["total_joints"] = stats([r.get("total_joints", 0) for r in results])

    return aggregated


# ---------------------------------------------------------------------------
# Output formatting
# ---------------------------------------------------------------------------


def print_results(results: dict):
    """Print formatted benchmark results."""
    benchmark_type = results.get("benchmark_type", "mobile")
    entity = "Arms" if benchmark_type == "arm" else "Agents"

    print("\n" + "=" * 70)
    print("Results")
    print("=" * 70)
    print("\nConfiguration:")
    print(f"  {entity}: {results['num_agents']}")
    print(f"  Duration: {results['duration_s']}s")
    print(f"  Scenario: {results.get('scenario') or results.get('mode') or 'default'}")
    print(f"  Repetitions: {results['num_reps']}")

    print("\nPerformance:")
    print(
        f"  Real-Time Factor: {results['real_time_factor']['median']:.2f}x " f"(±{results['real_time_factor']['stdev']:.2f})"
    )
    print(f"  Step Time: {results['avg_step_time_ms']['median']:.2f}ms " f"(±{results['avg_step_time_ms']['stdev']:.2f})")
    print(f"  Spawn Time: {results['spawn_time_s']['median']:.3f}s " f"(±{results['spawn_time_s']['stdev']:.3f})")
    print(f"  Memory: {results['mem_total_rss_mb']['median']:.2f}MB " f"(±{results['mem_total_rss_mb']['stdev']:.2f})")

    # Assessment
    rtf = results["real_time_factor"]["median"]

    if rtf >= 10.0:
        rtf_status = "✅ EXCELLENT"
    elif rtf >= 5.0:
        rtf_status = "✅ GOOD"
    elif rtf >= 1.0:
        rtf_status = "⚠️  ACCEPTABLE"
    else:
        rtf_status = "❌ POOR"

    print("\nAssessment:")
    print(f"  RTF: {rtf_status} ({rtf:.2f}x)")
    print("=" * 70)


def print_sweep_summary(all_results: List[dict]):
    """Print summary table for sweep results."""
    benchmark_type = all_results[0].get("benchmark_type", "mobile") if all_results else "mobile"
    entity = "Arms" if benchmark_type == "arm" else "Agents"

    print("\n" + "=" * 90)
    print("SWEEP SUMMARY")
    print("=" * 90)

    print(f"\n{entity:<10} {'Scenario':<20} {'RTF (x)':<15} {'Step (ms)':<15} {'Memory (MB)':<15}")
    print("-" * 90)

    for r in all_results:
        label = r.get("scenario") or r.get("mode") or "default"

        rtf = r["real_time_factor"]["median"]
        rtf_std = r["real_time_factor"]["stdev"]
        step = r["avg_step_time_ms"]["median"]
        step_std = r["avg_step_time_ms"]["stdev"]
        mem = r["mem_total_rss_mb"]["median"]
        mem_std = r["mem_total_rss_mb"]["stdev"]

        rtf_status = "✅" if rtf >= 5.0 else "⚠️" if rtf >= 1.0 else "❌"

        print(
            f"{r['num_agents']:<10} {label:<20} {rtf_status} {rtf:>5.2f}±{rtf_std:<4.2f}  "
            f"{step:>6.2f}±{step_std:<5.2f}  "
            f"{mem:>7.2f}±{mem_std:<5.2f}"
        )

    print("=" * 90)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="PyBullet Fleet Performance Benchmark Runner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Mobile (default)
  %(prog)s --sweep 100 500 1000 2000
  %(prog)s --compare no_collision collision_10hz

  # Arm
  %(prog)s --type arm --agents 10 --duration 5
  %(prog)s --type arm --sweep 1 10 50 100
  %(prog)s --type arm --compare physics kinematic
  %(prog)s --type arm --agents 50 --scenario kinematic
""",
    )

    # Benchmark type
    parser.add_argument(
        "--type",
        choices=["mobile", "arm"],
        default="mobile",
        help="Benchmark type: mobile agents or arm robots (default: mobile)",
    )

    # Basic options (shared)
    parser.add_argument("--agents", type=int, default=None, help="Number of agents/arms")
    parser.add_argument("--duration", type=float, default=10.0, help="Simulation duration in seconds (default: 10.0)")
    parser.add_argument("--repetitions", type=int, default=3, help="Number of repetitions (default: 3)")
    parser.add_argument("--gui", action="store_true", help="Enable GUI (mobile only)")
    parser.add_argument("--config", type=str, default=None, help="Path to benchmark config file")
    parser.add_argument("--scenario", type=str, default=None, help="Scenario name from config")

    # Sweep / compare
    parser.add_argument("--sweep", type=int, nargs="+", help="Sweep agent/arm counts")
    parser.add_argument(
        "--compare",
        type=str,
        nargs="+",
        help="Compare scenarios (e.g., --compare physics kinematic)",
    )

    return parser.parse_args()


def get_output_dir(config_path: Optional[str]) -> str:
    """Get output directory from config file, falling back to benchmark/results/."""
    if config_path:
        try:
            with open(config_path) as f:
                config = yaml.safe_load(f)
            output_dir = config.get("benchmark", {}).get("output_dir", ".")
            os.makedirs(output_dir, exist_ok=True)
            return output_dir
        except (FileNotFoundError, yaml.YAMLError):
            pass
    # Default: benchmark/results/
    default_dir = os.path.join(os.path.dirname(__file__), "results")
    os.makedirs(default_dir, exist_ok=True)
    return default_dir


# ---------------------------------------------------------------------------
# Arm benchmark logic
# ---------------------------------------------------------------------------


def _run_arm_benchmark(args):
    """Run arm-specific benchmark flows."""
    default_agents = 10
    num_agents = args.agents or default_agents
    output_dir = get_output_dir(args.config)
    all_results = []

    if args.sweep:
        # Sweep mode: iterate arm counts × scenarios
        scenarios = [args.scenario] if args.scenario else ["physics", "kinematic"]
        for n in args.sweep:
            for s in scenarios:
                result = run_multiple(
                    num_agents=n,
                    duration=args.duration,
                    num_reps=args.repetitions,
                    benchmark_type="arm",
                    config_path=args.config,
                    scenario=s,
                )
                all_results.append(result)

        print_sweep_summary(all_results)

        output_file = os.path.join(output_dir, f"arm_sweep_{args.duration}s.json")
        with open(output_file, "w") as f:
            json.dump(all_results, f, indent=2)
        print(f"\nSweep results saved to: {output_file}")

    elif args.compare:
        # Compare scenarios
        for scenario in args.compare:
            result = run_multiple(
                num_agents=num_agents,
                duration=args.duration,
                num_reps=args.repetitions,
                benchmark_type="arm",
                config_path=args.config,
                scenario=scenario,
            )
            all_results.append(result)

        print_sweep_summary(all_results)

        output_file = os.path.join(output_dir, f"arm_compare_{num_agents}arms.json")
        with open(output_file, "w") as f:
            json.dump(all_results, f, indent=2)
        print(f"\nComparison results saved to: {output_file}")

    else:
        # Single scenario
        result = run_multiple(
            num_agents=num_agents,
            duration=args.duration,
            num_reps=args.repetitions,
            benchmark_type="arm",
            config_path=args.config,
            scenario=args.scenario,
        )
        print_results(result)

        scenario_suffix = f"_{args.scenario}" if args.scenario else ""
        output_file = os.path.join(
            output_dir,
            f"arm_results_{num_agents}arms_{args.duration}s{scenario_suffix}.json",
        )
        with open(output_file, "w") as f:
            json.dump(result, f, indent=2)
        print(f"\nResults saved to: {output_file}")


# ---------------------------------------------------------------------------
# Mobile benchmark logic
# ---------------------------------------------------------------------------


def _run_mobile_benchmark(args):
    """Run mobile-specific benchmark flows (original behaviour)."""
    num_agents = args.agents or 1000
    output_dir = get_output_dir(args.config)
    all_results = []

    if args.sweep:
        for n in args.sweep:
            result = run_multiple(
                num_agents=n,
                duration=args.duration,
                num_reps=args.repetitions,
                benchmark_type="mobile",
                gui=args.gui,
                config_path=args.config,
                scenario=args.scenario,
            )
            all_results.append(result)

        print_sweep_summary(all_results)

        output_file = os.path.join(output_dir, f"benchmark_sweep_{args.duration}s.json")
        with open(output_file, "w") as f:
            json.dump(all_results, f, indent=2)
        print(f"\nSweep results saved to: {output_file}")

    elif args.compare:
        for scenario in args.compare:
            result = run_multiple(
                num_agents=num_agents,
                duration=args.duration,
                num_reps=args.repetitions,
                benchmark_type="mobile",
                gui=args.gui,
                config_path=args.config,
                scenario=scenario,
            )
            all_results.append(result)

        print_sweep_summary(all_results)

        output_file = os.path.join(output_dir, f"benchmark_compare_{num_agents}agents.json")
        with open(output_file, "w") as f:
            json.dump(all_results, f, indent=2)
        print(f"\nComparison results saved to: {output_file}")

    else:
        result = run_multiple(
            num_agents=num_agents,
            duration=args.duration,
            num_reps=args.repetitions,
            benchmark_type="mobile",
            gui=args.gui,
            config_path=args.config,
            scenario=args.scenario,
        )
        print_results(result)

        scenario_suffix = f"_{args.scenario}" if args.scenario else ""
        output_file = os.path.join(
            output_dir,
            f"benchmark_results_{num_agents}agents_{args.duration}s{scenario_suffix}.json",
        )
        with open(output_file, "w") as f:
            json.dump(result, f, indent=2)
        print(f"\nResults saved to: {output_file}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    args = parse_args()

    type_label = "Arm Robot" if args.type == "arm" else "Mobile Agent"
    print("=" * 70)
    print(f"PyBullet Fleet - {type_label} Benchmark Runner")
    print("=" * 70)

    if args.type == "arm":
        _run_arm_benchmark(args)
    else:
        _run_mobile_benchmark(args)


if __name__ == "__main__":
    main()
