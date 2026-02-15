#!/usr/bin/env python3
"""
run_benchmark.py
Meta-script for running performance benchmarks with process isolation and statistical analysis.

Features:
- Runs benchmark worker in isolated child processes
- Collects statistics (median, mean, stdev) from multiple repetitions
- Supports sweep across multiple agent counts
- Supports multiple scenarios comparison
- Clean separation of concerns (no self-recursion)

Usage:
    # Single test
    python benchmark/run_benchmark.py --agents 1000 --duration 10 --repetitions 3

    # With scenario
    python benchmark/run_benchmark.py --agents 1000 --scenario no_collision

    # Sweep multiple agent counts
    python benchmark/run_benchmark.py --sweep 100 500 1000 2000

    # Compare scenarios
    python benchmark/run_benchmark.py --compare no_collision collision_2d_10hz collision_3d_full
"""
import os
import sys
import json
import subprocess
import argparse
import statistics
from typing import List, Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def run_worker(
    num_agents: int, duration: float, gui: bool = False, config_path: Optional[str] = None, scenario: Optional[str] = None
) -> dict:
    """
    Run benchmark worker in isolated child process.

    Args:
        num_agents: Number of agents
        duration: Simulation duration
        gui: Enable GUI
        config_path: Path to config file
        scenario: Scenario name

    Returns:
        Benchmark results dictionary
    """
    cmd = [
        sys.executable,
        os.path.join(os.path.dirname(__file__), "performance_benchmark.py"),
        "--agents",
        str(num_agents),
        "--duration",
        str(duration),
    ]

    if gui:
        cmd.append("--gui")
    if config_path:
        cmd.extend(["--config", config_path])
    if scenario:
        cmd.extend(["--scenario", scenario])

    proc = subprocess.run(cmd, capture_output=True, text=True)

    if proc.returncode != 0:
        raise RuntimeError(
            f"Worker failed (agents={num_agents}, scenario={scenario})\n"
            f"STDOUT:\n{proc.stdout}\n"
            f"STDERR:\n{proc.stderr}\n"
        )

    # Parse JSON output
    try:
        return json.loads(proc.stdout.strip())
    except json.JSONDecodeError as e:
        raise RuntimeError(f"Failed to parse worker JSON: {e}\n" f"Raw stdout:\n{proc.stdout}\n")


def run_multiple(
    num_agents: int,
    duration: float,
    num_reps: int,
    gui: bool = False,
    config_path: Optional[str] = None,
    scenario: Optional[str] = None,
) -> dict:
    """
    Run benchmark multiple times and compute statistics.

    Args:
        num_agents: Number of agents
        duration: Simulation duration
        num_reps: Number of repetitions
        gui: Enable GUI
        config_path: Path to config file
        scenario: Scenario name

    Returns:
        Aggregated results with statistics
    """
    results = []

    print(f"\n{'='*70}")
    print(f"Benchmark: {num_agents} agents, {duration}s" + (f", scenario={scenario}" if scenario else ""))
    print(f"{'='*70}")

    for i in range(num_reps):
        print(f"  Run {i+1}/{num_reps}...", end="", flush=True)
        r = run_worker(num_agents, duration, gui, config_path, scenario)
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

    # Aggregate results
    return {
        "num_agents": num_agents,
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
        "expected_steps": results[0]["expected_steps"] if results else 0,
        "system_info": results[0]["system_info"] if results else {},  # Include system info
        "mem_spawn_rss_mb": stats([r["mem_spawn_mb"]["rss_mb"] for r in results]),
        "mem_total_rss_mb": stats([r["mem_total_mb"]["rss_mb"] for r in results]),
    }


def print_results(results: dict):
    """Print formatted benchmark results."""
    print("\n" + "=" * 70)
    print("Results")
    print("=" * 70)
    print("\nConfiguration:")
    print(f"  Agents: {results['num_agents']}")
    print(f"  Duration: {results['duration_s']}s")
    print(f"  Scenario: {results.get('scenario', 'default')}")
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
    step_time = results["avg_step_time_ms"]["median"]

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
    print("\n" + "=" * 90)
    print("SWEEP SUMMARY")
    print("=" * 90)
    print(f"\n{'Agents':<10} {'Scenario':<20} {'RTF (x)':<15} {'Step (ms)':<15} {'Memory (MB)':<15}")
    print("-" * 90)

    for r in all_results:
        scenario = r.get("scenario") or "default"
        rtf = r["real_time_factor"]["median"]
        rtf_std = r["real_time_factor"]["stdev"]
        step = r["avg_step_time_ms"]["median"]
        step_std = r["avg_step_time_ms"]["stdev"]
        mem = r["mem_total_rss_mb"]["median"]
        mem_std = r["mem_total_rss_mb"]["stdev"]

        rtf_status = "✅" if rtf >= 5.0 else "⚠️" if rtf >= 1.0 else "❌"

        print(
            f"{r['num_agents']:<10} {scenario:<20} {rtf_status} {rtf:>5.2f}±{rtf_std:<4.2f}  "
            f"{step:>6.2f}±{step_std:<5.2f}  "
            f"{mem:>7.2f}±{mem_std:<5.2f}"
        )

    print("=" * 90)


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="PyBullet Fleet Performance Benchmark Runner")

    # Basic options
    parser.add_argument("--agents", type=int, default=1000, help="Number of agents (default: 1000)")
    parser.add_argument("--duration", type=float, default=10.0, help="Simulation duration in seconds (default: 10.0)")
    parser.add_argument("--repetitions", type=int, default=3, help="Number of repetitions (default: 3)")
    parser.add_argument("--gui", action="store_true", help="Enable GUI")
    parser.add_argument("--config", type=str, default=None, help="Path to config file")
    parser.add_argument("--scenario", type=str, default=None, help="Scenario name")

    # Sweep options
    parser.add_argument("--sweep", type=int, nargs="+", help="Sweep agent counts (e.g., --sweep 100 500 1000)")
    parser.add_argument(
        "--compare", type=str, nargs="+", help="Compare scenarios (e.g., --compare no_collision collision_2d_10hz)"
    )

    return parser.parse_args()


def main():
    args = parse_args()

    print("=" * 70)
    print("PyBullet Fleet - Performance Benchmark Runner")
    print("=" * 70)

    all_results = []

    # Sweep mode: multiple agent counts
    if args.sweep:
        for num_agents in args.sweep:
            result = run_multiple(
                num_agents=num_agents,
                duration=args.duration,
                num_reps=args.repetitions,
                gui=args.gui,
                config_path=args.config,
                scenario=args.scenario,
            )
            all_results.append(result)

        print_sweep_summary(all_results)

        # Save sweep results
        output_file = f"benchmark_sweep_{args.duration}s.json"
        with open(output_file, "w") as f:
            json.dump(all_results, f, indent=2)
        print(f"\nSweep results saved to: {output_file}")

    # Compare mode: multiple scenarios
    elif args.compare:
        for scenario in args.compare:
            result = run_multiple(
                num_agents=args.agents,
                duration=args.duration,
                num_reps=args.repetitions,
                gui=args.gui,
                config_path=args.config,
                scenario=scenario,
            )
            all_results.append(result)

        print_sweep_summary(all_results)

        # Save comparison results
        output_file = f"benchmark_compare_{args.agents}agents.json"
        with open(output_file, "w") as f:
            json.dump(all_results, f, indent=2)
        print(f"\nComparison results saved to: {output_file}")

    # Single test mode
    else:
        result = run_multiple(
            num_agents=args.agents,
            duration=args.duration,
            num_reps=args.repetitions,
            gui=args.gui,
            config_path=args.config,
            scenario=args.scenario,
        )

        print_results(result)

        # Save single result
        scenario_suffix = f"_{args.scenario}" if args.scenario else ""
        output_file = f"benchmark_results_{args.agents}agents_{args.duration}s{scenario_suffix}.json"
        with open(output_file, "w") as f:
            json.dump(result, f, indent=2)
        print(f"\nResults saved to: {output_file}")


if __name__ == "__main__":
    main()
