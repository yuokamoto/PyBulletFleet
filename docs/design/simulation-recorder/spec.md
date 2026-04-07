# SimulationRecorder Specification

**Date:** 2026-04-02
**Status:** Validated

## Context

PyBulletFleet has no built-in recording capability. The current `scripts/capture_demo.py`
creates a standalone PyBullet session with fake robot movement — it doesn't capture actual demo
behavior. Users need a way to record real simulation runs as GIF/MP4 with zero or minimal code
changes to their demo scripts.

## Decision

Add `SimulationRecorder` as a first-class optional component in `pybullet_fleet`, integrated
into `MultiRobotSimulationCore` via `start_recording()` / `stop_recording()` API.
Follow the `DataMonitor` pattern: optional, gated, clean lifecycle.

Three trigger mechanisms (all opt-in, never records by default):

1. **Python API** — `sim.start_recording("output.gif", duration=4.0)`
2. **Environment variable** — `RECORD=output.gif python demo.py` (zero code changes)
3. **Programmatic** — Direct `SimulationRecorder(sim_core, ...)` instantiation

## Requirements

- Record actual simulation runs (not fake standalone scenes)
- Work in both `p.DIRECT` (headless) and `p.GUI` modes
- Never record unless explicitly triggered (API call, env var, or direct instantiation)
- Support GIF and MP4 output (MP4 requires ffmpeg; falls back to GIF)
- Auto-stop after `duration` seconds, or manual `stop_recording()`
- Camera auto-framing from entity AABBs (reuse `setup_camera` logic)
- < 5 MB per GIF output, configurable resolution/fps
- No new runtime dependencies (imageio already in dev deps)

## Constraints

- `getCameraImage()` + `ER_TINY_RENDERER` for headless capture
- `computeViewMatrix` / `computeProjectionMatrixFOV` for camera control
  (not `resetDebugVisualizerCamera` which is GUI-only)
- Callback frequency determines capture FPS (e.g., `frequency=15` → 15 fps)
- Frame buffer lives in memory; for long recordings (>1000 frames) consider
  streaming to disk (future optimization, not Phase 1)

## Out of Scope

- Audio recording
- Real-time streaming / live preview
- Video editing / annotation
- Automatic YouTube upload
- Recording to formats other than GIF/MP4

## Open Questions

- [ ] Should `RECORD` env var support duration? e.g., `RECORD=output.gif:4.0`
- [ ] MP4 support deferred to Phase 2 if ffmpeg detection is complex

## Success Criteria

- [ ] `sim.start_recording("test.gif", duration=2.0); sim.run_simulation(duration=3.0)` produces valid GIF
- [ ] `RECORD=test.gif python examples/scale/100robots_grid_demo.py` produces valid GIF with zero code changes
- [ ] Works in `p.DIRECT` mode (headless CI)
- [ ] Works in `p.GUI` mode
- [ ] Existing `make verify` passes (no regressions)
- [x] `scripts/capture_demo.py` refactored to use `SimulationRecorder`
- [ ] All 18 existing demos recordable via `RECORD=` env var
