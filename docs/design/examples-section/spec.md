# Examples Section Specification

**Date:** 2026-03-10
**Status:** Draft

## Context

The current docs cover installation (`getting-started/quickstart.md`) and advanced topics
(profiling, benchmarking), but have no tutorial-style pages that show a new user
**how to write a simulation from scratch**. Users arriving at the docs first want to understand
spawn → pose → goal → action → manager in that order.

## Decision

Add `docs/examples/` as a new top-level docs section with three tutorial pages,
each built around an existing runnable example script. Tutorial style: explain
**what and why** for each code block, not just what.

## Pages

| Page | Source script | Key APIs covered |
|------|--------------|-----------------|
| `spawning-objects.md` | `examples/basics/robot_demo.py` | SimObject, Agent spawn variants, Pose, get/set_pose, set_goal_pose, register_callback |
| `action-system.md` | `examples/basics/action_system_demo.py` | MoveAction, PickAction, DropAction, WaitAction, add_action_sequence, get_current_action |
| `multi-robot-fleet.md` | `examples/scale/100robots_cube_patrol_demo.py` | AgentManager, GridSpawnParams, spawn_agents_grid_mixed, set_path, Path, velocity monitoring callback |

## Requirements

- Each page is a self-contained tutorial: copy-paste code must run
- Code is broken into labelled blocks with before/after explanation prose
- API method signatures shown inline (no separate API page link required to understand the example)
- Path to source file linked at top of each page
- Cross-links between pages where APIs build on each other
- Section added to `docs/index.md` toctree and How-to Table

## Constraints

- Read-only: do not modify example Python files
- `path_following_demo.py` is referenced as a "see also" from spawning-objects (set_path with waypoints), not a standalone page
- Each page is ~300–500 lines of markdown (code + prose)

## Out of Scope

- Auto-generated API reference (already in `docs/api/`)
- Example pages for collision_features_demo, mobile_manipulator (separate request)

## Success Criteria

- [ ] `sphinx-build` completes with no warnings on new pages
- [ ] Three pages exist under `docs/examples/`
- [ ] All six API groups covered: spawn, get/set_pose, set_goal/set_path, Actions, Manager, Callback
- [ ] `docs/index.md` updated with examples/ toctree entry
