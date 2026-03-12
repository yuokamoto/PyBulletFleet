# PyBulletFleet Domain Skills - Agent Specification

## Requirements

### Functional
- 5 skills with proper YAML frontmatter (`name` + `description` only)
- Each skill follows progressive disclosure (metadata → body → references)
- Skill ① is foundation; ②–⑤ reference ① without duplicating content
- "Fetch from source" pattern: guide agent to read live code, not hardcode details

### Non-Functional
- Each SKILL.md <500 lines
- Reference files include TOC when >100 lines
- Cross-references use skill name only (no `@` prefix that force-loads)

## Constraints

- Install path: `/home/rapyuta/.copilot/skills/{skill-name}/`
- Follow existing skill patterns in the same directory
- USO skill based on published specs only (no implementation code exists)

## Approach

Create skills sequentially: ① → ② → ③ → ④ → ⑤, since later skills cross-reference earlier ones.

## Design

### Skill Dependency Graph

```
① working-with-pybullet-fleet  ←── all others reference this
    ├── ② pybullet-performance-workflow
    │       └── cross-refs: ④ (collision perf issues)
    ├── ③ adding-sim-entities
    │       └── cross-refs: ④ (new entity collision setup)
    ├── ④ pybullet-collision-tuning
    │       └── cross-refs: ② (measuring collision perf)
    └── ⑤ integrating-with-uso
            └── cross-refs: ③ (snapshot type mapping for new entities)
```

### Skill ① working-with-pybullet-fleet

**Directory structure:**
```
working-with-pybullet-fleet/
├── SKILL.md
└── references/
    └── code-patterns.md
```

**SKILL.md sections:**
1. Component quick-reference table (file → class → responsibility)
2. "Before Working" checklist — files agent must read first
3. Key patterns — factory methods, dataclass params, lazy logging, config layering, TYPE_CHECKING guard, shared shape cache
4. Testing patterns — fixtures, MockSimCore, assert helpers
5. Common pitfalls — shape cache stale IDs, physics mode mismatch, collision margin, GUI in tests

**references/code-patterns.md:**
- Detailed code examples for each pattern (factory, config, action lifecycle)
- Grep commands to discover current patterns from source

### Skill ② pybullet-performance-workflow

**Directory structure:**
```
pybullet-performance-workflow/
├── SKILL.md
└── references/
    └── benchmark-reference.md
```

**SKILL.md sections:**
1. Iron Law — no optimization without measurement
2. Four-phase workflow: MEASURE → PROFILE → OPTIMIZE → VERIFY
3. Benchmark quick-reference commands
4. Red flags — anti-patterns that abort the workflow

**references/benchmark-reference.md:**
- Performance targets table (100/1000/10000 agents)
- Known bottlenecks table (getClosestPoints, resetBasePositionAndOrientation, etc.)
- Benchmark CLI full reference (--agents, --sweep, --compare, --scenario)
- Predefined scenarios table

### Skill ③ adding-sim-entities

**Directory structure:**
```
adding-sim-entities/
├── SKILL.md
└── references/
    └── urdf-checklist.md
```

**SKILL.md sections:**
1. Decision: SimObject vs Agent (criteria table)
2. Six-step checklist: URDF → SpawnParams → Config → Spawn+Test → Actions → Demo
3. Entity type reference table (planned entities → type/mode/actions)
4. Common pitfalls
5. Cross-references

**references/urdf-checklist.md:**
- URDF structure requirements for PyBulletFleet
- Collision/visual geometry requirements
- Joint configuration patterns
- Existing URDF files as templates

### Skill ④ pybullet-collision-tuning

**Directory structure:**
```
pybullet-collision-tuning/
├── SKILL.md
└── references/
    └── collision-internals.md
```

**SKILL.md sections:**
1. "Before Touching Collision Code" — required reading list
2. Architecture quick reference (broad phase → narrow phase)
3. CollisionMode decision table
4. Tuning parameters table with guidelines
5. Diagnostic flowcharts (not detected / false positive / performance)
6. Cross-references

**references/collision-internals.md:**
- Spatial hash implementation details (cell indexing, multi-cell registration, incremental update)
- Narrow phase method comparison (closest_points vs contact_points vs hybrid)
- Known historical issues from git history

### Skill ⑤ integrating-with-uso

**Directory structure:**
```
integrating-with-uso/
├── SKILL.md
└── references/
    ├── snapshot-mapping.md
    └── uso-spec-summary.md
```

**SKILL.md sections:**
1. USO architecture overview (Simulation Master, Nodes, Layers)
2. SimulationNode interface → PyBulletFleet method mapping table
3. Implementation phases (Single Mode first → Distributed later)
4. Implementation checklist
5. Cross-references

**references/snapshot-mapping.md:**
- Field-by-field mapping: USO snapshot fields → PyBulletFleet data sources
- Delta snapshot strategy using `_moved_this_step`
- Full snapshot → world reconstruction approach
- Action status → USO event mapping

**references/uso-spec-summary.md:**
- Condensed USO spec (snapshot types, message types, node interface, operational modes)
- Links to canonical USO documents for latest versions

## File References

Files the implementing agent MUST read before creating each skill:

**For all skills:**
- `pybullet_fleet/__init__.py` — public API exports
- `DESIGN.md` — architecture overview
- `pybullet_fleet/types.py` — all enum definitions

**For ①:**
- All files in `pybullet_fleet/` — understand every module
- `tests/conftest.py` — test infrastructure
- `config/config.yaml` — default config

**For ②:**
- `benchmark/run_benchmark.py` — benchmark invocation
- `benchmark/configs/general.yaml` — predefined scenarios
- `benchmark/PERFORMANCE_REPORT.md` — known performance numbers
- `docs/PERFORMANCE_ANALYSIS.md` — analysis history

**For ③:**
- `robots/*.urdf` — existing URDF files
- `pybullet_fleet/sim_object.py` — SimObjectSpawnParams
- `pybullet_fleet/agent.py` — AgentSpawnParams, from_params/from_urdf/from_mesh
- `pybullet_fleet/action.py` — Action ABC and subclasses
- `examples/` — demo patterns

**For ④:**
- `docs/COLLISION_DETECTION_DESIGN_v3.md` — collision design doc
- `pybullet_fleet/core_simulation.py` — collision methods (check_collisions, filter_aabb_pairs)
- `benchmark/collision_check_profile.py` — collision profiling
- `docs/spatial_hash_cell_size_modes.md` — cell size documentation

**For ⑤:**
- USO GitHub repository specs (03_Snapshot, 05_NodeDesign, 06_Logging_Replay, 07_Interface)
- `pybullet_fleet/core_simulation.py` — step_once, sim state access
- `pybullet_fleet/sim_object.py` — state serialization points
- `pybullet_fleet/agent.py` — agent state access

## Success Criteria

- [ ] 5 skill directories created in `~/.copilot/skills/`
- [ ] Each SKILL.md <500 lines with proper YAML frontmatter
- [ ] Description starts with "Use when..." and includes triggering conditions
- [ ] Reference files have TOC when >100 lines
- [ ] No `@` prefix in cross-references (use skill name only)
- [ ] Patterns guide agent to read source code rather than hardcoding details
