# Test Action Plan - PyBulletFleet Test Coverage Improvement

**Date**: February 16, 2026
**Current Status**: 39% overall coverage, 48 passed / 2 failed / 1 skipped (51 total tests)

---

## 📊 Current Test Status Analysis

### Test Coverage by Module

| Module | Statements | Miss | Coverage | Priority |
|--------|-----------|------|----------|----------|
| `types.py` | 29 | 0 | **100%** ✅ | - |
| `__init__.py` | 8 | 0 | **100%** ✅ | - |
| `logging_utils.py` | 42 | 9 | **79%** ✅ | LOW |
| `data_monitor.py` | 98 | 43 | **56%** | MEDIUM |
| `core_simulation.py` | 978 | 454 | **54%** | HIGH |
| `sim_object.py` | 358 | 183 | **49%** | HIGH |
| `agent.py` | 611 | 370 | **39%** | **CRITICAL** |
| `geometry.py` | 217 | 154 | **29%** | HIGH |
| `action.py` | 460 | 339 | **26%** | **CRITICAL** |
| `agent_manager.py` | 241 | 195 | **19%** | **CRITICAL** |
| `collision_visualizer.py` | 148 | 128 | **14%** | HIGH |
| `tools.py` | 178 | 156 | **12%** | HIGH |
| `config_utils.py` | 27 | 27 | **0%** | MEDIUM |
| **TOTAL** | **3,395** | **2,058** | **39%** | - |

### Existing Tests

#### ✅ Working Tests (48 passed)
1. **Collision Detection** (37 tests) - `test_collision_comprehensive.py`
   - Collision mode combinations (32 tests)
   - 2D mode z-separation (1 test)
   - Detection methods (4 tests)

2. **Logging Utils** (2 tests) - `test_logging_utils.py`
   - Functionality test
   - Performance test

3. **Memory Profiling** (6 tests) - `test_memory_profiling.py`
   - Disabled by default
   - Can be enabled
   - Get usage returns None when disabled
   - Collects data
   - With time profiling
   - Detects memory growth

4. **Multi-Cell Registration** (4 tests) - `test_multi_cell_registration.py`
   - Single cell registration
   - Multi-cell registration
   - Large object collision detection
   - Threshold configuration

#### ❌ Failing Tests (2 failed)
1. `test_multi_cell_simple.py::test_multi_cell_with_agents` - Missing URDF file
2. `test_multi_cell_simple.py::test_threshold_effect` - Missing URDF file

#### ⏭️ Skipped Tests (1 skipped)
1. `test_collision_comprehensive.py::TestCollisionDetectionMethod::test_contact_points_requires_step_simulation`

#### 📝 Demo/Manual Tests (Not in pytest)
- `test_robot_movement.py` - No automated tests
- `test_path_following.py` - No automated tests
- `test_path_simple.py` - No automated tests
- `test_path_with_simcore.py` - No automated tests

### Critical Gaps (No Tests)

1. **Agent** (39% coverage)
   - ❌ Goal-based navigation
   - ❌ Motion modes (omnidirectional vs differential)
   - ❌ Velocity/acceleration limits
   - ❌ Object pick/drop
   - ❌ Path following

2. **Action System** (26% coverage)
   - ❌ MoveTo action
   - ❌ Pick action
   - ❌ Drop action
   - ❌ Wait action
   - ❌ Action sequencing
   - ❌ Action state machine

3. **AgentManager** (19% coverage)
   - ❌ Grid spawning
   - ❌ Mixed type spawning
   - ❌ Goal update callbacks
   - ❌ Agent queries

4. **Geometry** (29% coverage)
   - ❌ Pose operations
   - ❌ Path creation
   - ❌ Distance calculations

5. **Tools** (12% coverage)
   - ❌ Pose offset calculations
   - ❌ Utility functions

6. **Config Utils** (0% coverage)
   - ❌ YAML loading
   - ❌ Config validation

---

## 🎯 Action Plan

### Phase 1: Quick Wins (Week 1) - Target: 50% Coverage

#### Priority 1: Fix Failing Tests
- [ ] **Fix URDF path issues in `test_multi_cell_simple.py`**
  - Create missing `robots/platform_robot.urdf` or update test to use existing URDFs
  - Estimated time: 30 minutes

#### Priority 2: Geometry Module Tests (29% → 70%)
- [ ] **Create `tests/test_geometry.py`**
  - Pose creation (`from_xyz`, `from_euler`, `from_pybullet`)
  - Pose conversion (`as_euler`, `as_position_orientation`)
  - Distance calculations
  - Path creation and waypoint management
  - Estimated time: 2 hours
  - Expected coverage gain: +40% on geometry.py

#### Priority 3: Tools Module Tests (12% → 60%)
- [ ] **Create `tests/test_tools.py`**
  - `calculate_offset_pose()` with various scenarios
  - Edge cases (zero offset, same position)
  - Estimated time: 1 hour
  - Expected coverage gain: +48% on tools.py

#### Priority 4: Convert Manual Tests to Automated
- [ ] **Convert `test_robot_movement.py` to pytest**
  - Basic movement test (already exists but not discovered by pytest)
  - Update to use proper pytest structure
  - Estimated time: 30 minutes

### Phase 2: Core Functionality (Week 2) - Target: 60% Coverage

#### Priority 5: Agent Module Tests (39% → 65%)
- [ ] **Create `tests/test_agent_core.py`**
  - Agent creation (from_mesh, from_urdf)
  - Goal setting and navigation
  - Motion modes (omnidirectional, differential)
  - Velocity/acceleration limiting
  - Goal reached detection
  - Estimated time: 3 hours
  - Expected coverage gain: +26% on agent.py

#### Priority 6: AgentManager Tests (19% → 60%)
- [ ] **Create `tests/test_agent_manager.py`**
  - Grid spawning (single type)
  - Mixed type spawning with probabilities
  - Grid counts spawning
  - Goal update callbacks
  - Agent queries (moving, stopped)
  - Estimated time: 2 hours
  - Expected coverage gain: +41% on agent_manager.py

#### Priority 7: SimObject Tests (49% → 70%)
- [ ] **Create `tests/test_sim_object.py`**
  - Object creation (from_mesh)
  - Shape parameters
  - Attachment system
  - Pickable/non-pickable objects
  - Shared shape caching
  - Estimated time: 2 hours
  - Expected coverage gain: +21% on sim_object.py

### Phase 3: Advanced Features (Week 3) - Target: 70% Coverage

#### Priority 8: Action System Tests (26% → 65%)
- [ ] **Create `tests/test_action_system.py`**
  - MoveTo action execution
  - Pick action with object attachment
  - Drop action with object detachment
  - Wait action timing
  - Action sequencing
  - Action state transitions
  - Estimated time: 4 hours
  - Expected coverage gain: +39% on action.py

#### Priority 9: CoreSimulation Tests (54% → 70%)
- [ ] **Create `tests/test_core_simulation.py`**
  - Simulation initialization (from_dict, from_yaml)
  - Callback registration and execution
  - Camera setup
  - Visualizer configuration
  - Step execution
  - Estimated time: 3 hours
  - Expected coverage gain: +16% on core_simulation.py

#### Priority 10: Config Utils Tests (0% → 80%)
- [ ] **Create `tests/test_config_utils.py`**
  - YAML loading
  - Config validation
  - Error handling
  - Estimated time: 1 hour
  - Expected coverage gain: +80% on config_utils.py

### Phase 4: Integration & Performance (Week 4) - Target: 75%+ Coverage

#### Priority 11: Integration Tests
- [ ] **Create `tests/test_integration.py`**
  - End-to-end agent navigation
  - Multi-agent coordination
  - Pick and drop workflow
  - Action system with agents
  - Estimated time: 3 hours

#### Priority 12: Performance Tests
- [ ] **Enhance existing performance tests**
  - Add benchmarks for 100/1000 agents
  - Collision detection performance
  - Memory usage validation
  - Estimated time: 2 hours

#### Priority 13: Edge Cases & Error Handling
- [ ] **Create `tests/test_edge_cases.py`**
  - Invalid inputs
  - Error recovery
  - Boundary conditions
  - Estimated time: 2 hours

---

## 📋 Test Implementation Guidelines

### Test Structure Template
```python
"""
Brief description of what this test module covers.
"""
import pytest
from pybullet_fleet import ...

class TestClassName:
    """Group related tests together"""

    def test_descriptive_name(self):
        """Test specific behavior"""
        # Arrange
        ...
        # Act
        ...
        # Assert
        assert expected == actual
```

### Best Practices
1. **Use fixtures** for common setup (simulation, agents, objects)
2. **Parametrize tests** for testing multiple scenarios
3. **Use headless mode** (`gui=False`) for faster execution
4. **Clean up resources** (disconnect PyBullet after each test)
5. **Name tests descriptively** (`test_agent_moves_to_goal_omnidirectional`)
6. **Test one thing** per test function
7. **Use assertions** with clear error messages

### Coverage Goals by Phase
- **Phase 1 (Week 1)**: 39% → 50% (+11%)
- **Phase 2 (Week 2)**: 50% → 60% (+10%)
- **Phase 3 (Week 3)**: 60% → 70% (+10%)
- **Phase 4 (Week 4)**: 70% → 75%+ (+5%+)

---

## 🚀 Quick Start

### 1. Fix Immediate Issues
```bash
# Fix failing tests
pytest tests/test_multi_cell_simple.py -v

# Run all tests
pytest tests/ -v --cov=pybullet_fleet --cov-report=html

# View coverage report
open htmlcov/index.html
```

### 2. Start with Phase 1
```bash
# Create first new test file
touch tests/test_geometry.py

# Run specific test
pytest tests/test_geometry.py -v
```

### 3. Monitor Progress
```bash
# Check coverage regularly
pytest tests/ --cov=pybullet_fleet --cov-report=term-missing
```

---

## 📈 Success Metrics

### Coverage Targets
- **Minimum**: 60% by end of Week 2
- **Goal**: 75% by end of Week 4
- **Stretch**: 80%+ for production readiness

### Quality Metrics
- All tests passing (0 failures)
- No skipped tests (investigate and fix or remove)
- Test execution time < 60 seconds
- Clear test names and documentation

### CI/CD Integration
- Tests run automatically on every commit
- Coverage reports generated
- Failing tests block merges

---

## 💡 Notes

### Why These Priorities?

1. **Geometry & Tools** (Phase 1) - Foundation for other tests, high impact/effort ratio
2. **Agent & AgentManager** (Phase 2) - Core functionality, most used by users
3. **Action System** (Phase 3) - Complex but critical feature
4. **Integration** (Phase 4) - Ensures everything works together

### Files to Update
- `tests/README.md` - Update with pytest information
- `.github/workflows/` - Enable test automation (currently commented out)
- `pyproject.toml` - Configure pytest options

### Tools & Commands
```bash
# Run tests with coverage
pytest --cov=pybullet_fleet --cov-report=html

# Run specific test file
pytest tests/test_geometry.py -v

# Run tests matching pattern
pytest -k "test_pose" -v

# Run with detailed output
pytest -vv

# Run with print statements shown
pytest -s

# Run failed tests only
pytest --lf
```

---

## 🎓 Learning Resources

- [pytest documentation](https://docs.pytest.org/)
- [pytest-cov documentation](https://pytest-cov.readthedocs.io/)
- [Testing Best Practices](https://docs.pytest.org/en/stable/goodpractices.html)

---

**Last Updated**: February 16, 2026
**Status**: Ready to begin Phase 1
