# ROS 2 Bridge Performance Notes

This file captures current ROS 2 bridge scale observations before the content is
promoted into the ReadTheDocs ROS 2 Bridge section.

## Interface Modes

- `fleet`: exposes fleet-level ROS endpoints such as `/fleet/states`,
  `/fleet/navigate`, and `/fleet/joint_command`. This is the scalable default
  for large fleets and RMF integration.
- `per_robot`: exposes per-robot ROS interfaces such as `/{robot}/odom`,
  `/{robot}/goal_pose`, services, and action servers. This is useful for
  compatibility and debugging, but endpoint count grows with robot count.
- `hybrid`: enables the fleet API and selected per-robot interfaces at the same
  time. Hybrid should not imply exposing every per-robot interface for every
  robot at large scale; use it selectively.

## Scope

The numbers below were collected with the Docker fleet scale checker. Unless
otherwise stated, scale checks use lightweight `simple_cube` robots as the
large-fleet baseline; detailed mobile or manipulator models should be measured
separately.

```bash
cd docker
docker compose run --rm --no-deps \
  -v "$(pwd):/docker:ro" \
  bridge bash /docker/test_fleet_scale.sh --robots 1000 \
    --interface-mode fleet --command-interface fleet \
    --publish-rate 5 --target-rtf 1.0
```

The checker sends commands and verifies that commanded robots start moving. For
per-robot topic commands this is important: publish-loop time alone is not an
end-to-end performance metric.

`--interface-mode` configures which ROS interfaces the bridge exposes.
`--command-interface` is an advanced checker option that selects which command
path is exercised during the measurement. In normal runs they should match
(`fleet/fleet` or `per_robot/per_robot`). Mismatched combinations are mainly for
hybrid overhead and compatibility debugging.

Use `--target-rtf 0 --measure-rtf` when measuring the maximum observed RTF. The
RTF value is computed from `/clock` simulation time over wall time after the
command check has completed.

`timestep` is a core part of any RTF comparison. The current scale-check
template does not override `simulation.timestep`, so it uses the PyBulletFleet
default `0.1s` timestep. The bridge logs this as `dt=0.1000s`. `publish_rate`
is independent from timestep: it throttles ROS publication (`/clock`,
`/fleet/states`, per-robot state, TF) but does not change how often the
simulation advances. For example, with `timestep=0.1s`, `target_rtf=0`, and
`publish_rate=5Hz`, the simulation advances by 0.1 simulated seconds per step
as fast as possible, while ROS output is published every 0.2 simulated seconds.

## Current Findings

Fleet-only is the scalable baseline. In the current Docker environment, a
1000-robot fleet-only run with `target_rtf=1.0` returned `/fleet/navigate` in
tens of milliseconds and verified motion shortly after.

The fleet-only maximum-RTF result should be compared against PyBulletFleet-only
benchmarks with the same robot count, timestep, controller, physics mode, GUI
setting, and motion workload. If the PyBulletFleet-only baseline is
substantially faster under those same conditions, the difference is attributable
to ROS bridge overhead: ROS node spinning, message construction, serialization,
DDS publication, service handling, and fleet-state snapshot conversion.

Hybrid mode is sensitive to per-robot endpoint count. Exposing per-robot state
publishers, TF, command topics, services, or actions can add seconds of
`/fleet/navigate` service latency at 1000 robots. Lowering `publish_rate` helps
when state publishers and TF are enabled, but it does not remove the endpoint
count cost.

The slowdown is primarily visible before command ack, not after ack. In hybrid
mode the bridge still accepts one fleet-level command, but the same ROS node also
owns many per-robot publishers/subscriptions/timers and may be constructing and
publishing per-robot state and TF messages while servicing `/fleet/navigate`.
Those per-robot ROS entities increase executor and DDS graph overhead, and the
extra publish work competes with service handling. Once the fleet command is
accepted, all robots still start moving quickly in the measured cases.

Per-robot topic commands are not equivalent to fleet service commands. They are
fire-and-forget and can look fast if only publish time is measured. With motion
verification enabled, hundreds of per-robot topic commands did not reliably
apply to every robot in the current bridge setup.

Per-robot action servers are not viable at 1000 robots in the current Docker
environment. Creating the action-server-heavy configuration can abort in the
DDS/RCL layer.

## Representative Results

These are single-run diagnostic numbers, not stable CI thresholds.

| Mode | Robots | Groups | publish_rate | target_rtf | Command ack | All moved after ack (wall) | Result |
|------|--------|--------|--------------|------------|-------------|----------------------------|--------|
| `fleet` | 100 | none | 5 Hz | 0 | 0.048 s | 0.008 s | max RTF 110.20x |
| `fleet` | 500 | none | 5 Hz | 0 | 0.181 s | 0.043 s | max RTF 21.03x |
| `fleet` | 1000 | none | 5 Hz | 1.0 | 0.027 s | 0.173 s | target-rate command check |
| `fleet` | 1000 | none | 5 Hz | 0 | 0.486 s | 0.104 s | max RTF 9.37x |
| `per_robot` | 1000 | `state_publishers,tf,command_topics` | 5 Hz | 0 | publish 0.217 s | 0/1000 in 60 s | max RTF 0.64x |
| `per_robot` | 100 | `state_publishers,tf,command_topics` | 5 Hz | 0 | publish 0.056 s | 15.033 s | 100/100 moved |
| `hybrid` | 1000 | `state_publishers,tf,command_topics` | 5 Hz | 0 | fleet ack 7.998 s; per-robot publish 0.220 s | not verified | max RTF 0.30x |
| `hybrid` | 1000 | `command_topics` | 5 Hz | 1.0 | 0.527 s | 0.112 s | target-rate command check |
| `hybrid` | 1000 | `state_publishers,tf,command_topics` | 1 Hz | 1.0 | 1.878 s | 0.229 s | target-rate command check |
| `hybrid` | 1000 | `state_publishers,tf,command_topics` | 5 Hz | 1.0 | 8.657 s | 0.323 s | target-rate command check |
| `hybrid` | 1000 | `actions` | 5 Hz | 1.0 | not available | not measured | DDS/RCL abort observed |

`Command ack` is the `/fleet/navigate` service response time. `All moved after
ack` or `All moved after command publication` is a separate checker result based
on observed robot state after the command is accepted or published. Ack and
motion-verification latency values are wall time, while max RTF is derived from
`/clock` simulation time over wall time. Use `--no-verify-motion` for RTF-only
measurements, and use motion verification when debugging command delivery
semantics.
Because motion verification is based on observed state updates, this wall-time
latency is affected by `target_rtf`, timestep, and publish timing.
Current checker output reports motion latency after the relevant boundary as
first, p50, p90, p99, and all-moved timings, plus missing robot names and final
observed positions when not all robots move within the checker window.

The latest 1000-robot publish-only check took 0.217 s for per-robot topics and
0.220 s for the hybrid per-robot command phase. Those numbers only measure
topic publication from the checker and should not be compared directly with
acknowledged fleet service calls. At 100 robots, per-robot commands moved all
robots in 15.033 s; at 1000 robots, no robot was observed moving within the
60-second verification window. This is a command-delivery/scale limitation,
not evidence that publish completion implies motion completion.

## Recommendations

- Use `fleet-only` as the default bridge mode for large fleets and RMF
  integration.
- Use hybrid mode only for debug and compatibility, and enable only the
  per-robot groups that are required.
- In hybrid mode, prefer enabling per-robot interfaces for selected robots only.
  Use `per_robot_api.include_robots` for allow-list style debugging, or
  `exclude_robots` when most but not all robots need compatibility endpoints.
  Example:

  ```yaml
  fleet_api:
    enabled: true
    states: true
    navigate: true
    joint_command: true

  per_robot_api:
    enabled: true
    state_publishers: true
    tf: true
    command_topics: true
    services: false
    actions: false
    include_robots: [robot_0, robot_1]
  ```

- Avoid per-robot action servers and per-robot services at 1000 robot scale.
- Keep `publish_rate` low when per-robot state publishers or TF are enabled.
- Treat per-robot command topics as a debug interface unless an acknowledgement
  or verification layer is added.

## Follow-Up

- Add ReadTheDocs pages under a dedicated ROS 2 Bridge section.
- Convert these notes into `docs/ros2-bridge/performance.md`.
- Add startup time, memory, DDS graph size, and RMF adapter latency measurements.
- Keep Docker scale checks optional; they are diagnostic benchmarks, not stable
  unit-test gates.
