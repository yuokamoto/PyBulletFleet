# PyBulletFleet Domain Skills Specification

**Date:** 2026-03-06
**Status:** Draft

## Context

PyBulletFleet has grown into a multi-module simulation framework with complex subsystems (collision detection, action system, performance tuning). Development sessions repeatedly require re-discovering codebase patterns, debugging collision issues, and following optimization workflows. Additionally, the roadmap includes USO (Unified Simulation Orchestrator) integration requiring snapshot/replay capabilities. Five domain-specific agent skills will codify this knowledge for consistent, efficient development.

## Decision

Create five skills organized in a dependency hierarchy:

1. **working-with-pybullet-fleet** — Foundation domain reference for all PyBulletFleet work
2. **pybullet-performance-workflow** — Structured benchmark→profile→optimize→verify cycle
3. **adding-sim-entities** — Checklist-driven process for new robots/objects/infrastructure
4. **pybullet-collision-tuning** — Collision detection debugging and parameter tuning guide
5. **integrating-with-uso** — USO SimulationNode adapter, snapshot serialization, and replay

## Requirements

- Each skill must be self-contained with proper YAML frontmatter (name + description only)
- Skills follow progressive disclosure: metadata → SKILL.md body (<500 lines) → bundled references
- Skill ① serves as the base reference; skills ②–⑤ cross-reference ① but do not duplicate content
- All skills use "fetch from source" pattern — guide agent to read current code rather than hardcoding details that may drift
- Skills must be compatible with existing skill ecosystem (brainstorming, TDD, systematic-debugging, etc.)
- **Skills live in the repo** at `.copilot/skills/` — version-controlled, PR-reviewable
- **`setup_skills.sh`** creates symlinks from `~/.copilot/skills/` to repo skills for all platforms
- OSS users can `git clone` + `./setup_skills.sh` to install domain skills

## Constraints

- USO has no implementation code yet (spec-only repository) — skill ⑤ must be based on published specs
- Skills must work in VS Code Copilot with `runSubagent` for parallel exploration
- SKILL.md files must stay under 500 lines per skill-creator guidelines
- Reference files should be structured with TOC for files >100 lines
- `setup_skills.sh` must be idempotent (safe to run multiple times)

## Out of Scope

- Standalone ROS 2 skill (ROS 2 integration flows through USO)
- CI/CD pipeline skill (too early — no pipeline exists yet)
- GUI/visualization skill (minimal decision surface)

## Open Questions

- [ ] Should ⑤ include ZeroMQ messaging details or defer until USO has implementation code?

## Success Criteria

- [ ] All 5 skills installed in `~/.copilot/skills/`
- [ ] Each SKILL.md has proper frontmatter with "Use when..." description
- [ ] Each skill <500 lines
- [ ] Cross-references use skill names only (no @ force-loading)
- [ ] Reference files have TOC where >100 lines

## References

- [USO Snapshot Spec](https://github.com/yuokamoto/Unified-Simulation-Orchestrator/blob/main/03_Snapshot_Specification_EN.md)
- [USO Architecture](https://github.com/yuokamoto/Unified-Simulation-Orchestrator/blob/main/02_Architecture_EN.md)
- [USO Node Design](https://github.com/yuokamoto/Unified-Simulation-Orchestrator/blob/main/05_SimulationMaster_NodeDesign_EN.md)
- [USO Logging/Replay](https://github.com/yuokamoto/Unified-Simulation-Orchestrator/blob/main/06_Logging_Replay_EN.md)
- [USO Interface/Messaging](https://github.com/yuokamoto/Unified-Simulation-Orchestrator/blob/main/07_Interface_Messaging_EN.md)
