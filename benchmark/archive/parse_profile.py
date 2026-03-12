#!/usr/bin/env python3
"""
Parse profiling logs and compute statistics.
"""
import sys
import re
from statistics import mean, median, stdev

# Read from stdin
lines = sys.stdin.readlines()

data = {
    "agent_update": [],
    "callbacks": [],
    "step_simulation": [],
    "collisions": [],
    "monitor": [],
    "total": [],
}

pattern = (
    r"Agent\.update=([\d.]+)ms.*Callbacks=([\d.]+)ms.*stepSimulation=([\d.]+)ms.*"
    r"Collisions=([\d.]+)ms.*Monitor=([\d.]+)ms.*Total=([\d.]+)ms"
)

for line in lines:
    match = re.search(pattern, line)
    if match:
        data["agent_update"].append(float(match.group(1)))
        data["callbacks"].append(float(match.group(2)))
        data["step_simulation"].append(float(match.group(3)))
        data["collisions"].append(float(match.group(4)))
        data["monitor"].append(float(match.group(5)))
        data["total"].append(float(match.group(6)))

if not data["total"]:
    print("No profiling data found")
    sys.exit(1)

print("=" * 70)
print(f"Profiling Statistics ({len(data['total'])} steps)")
print("=" * 70)

for key, values in data.items():
    if not values:
        continue
    avg = mean(values)
    med = median(values)
    std = stdev(values) if len(values) > 1 else 0
    pct = (avg / mean(data["total"]) * 100) if mean(data["total"]) > 0 else 0

    print(f"\n{key.replace('_', ' ').title()}:")
    print(f"  Mean:   {avg:>8.2f}ms ({pct:>5.1f}%)")
    print(f"  Median: {med:>8.2f}ms")
    print(f"  StdDev: {std:>8.2f}ms")
    print(f"  Min:    {min(values):>8.2f}ms")
    print(f"  Max:    {max(values):>8.2f}ms")

print("\n" + "=" * 70)
