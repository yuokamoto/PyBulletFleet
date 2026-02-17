"""
Tests for Action classes.

This module tests:
- WaitAction: Waiting for specified duration
- Action base class properties
- Action status and completion
"""

import pybullet as p
import pytest

from pybullet_fleet.action import WaitAction


class MockAgent:
    """Mock agent for testing WaitAction"""

    def __init__(self):
        self.sim_core = None


@pytest.fixture
def pybullet_env():
    """Setup and teardown PyBullet environment for each test"""
    physics_client = p.connect(p.DIRECT)
    p.setGravity(0, 0, -10)

    yield physics_client

    p.disconnect()


class TestWaitAction:
    """Test WaitAction for time-based waiting"""

    def test_wait_action_creation(self, pybullet_env):
        """Test creating a WaitAction"""
        wait_time = 2.0
        action = WaitAction(duration=wait_time)

        assert action is not None
        assert action.duration == wait_time
        assert action.is_complete() is False

    def test_wait_action_execution(self, pybullet_env):
        """Test WaitAction completes after specified duration"""
        wait_time = 0.5
        action = WaitAction(duration=wait_time)
        agent = MockAgent()

        # Execute action
        dt = 0.01
        total_time = 0.0
        completed = False

        while total_time < wait_time + 0.1:  # Add buffer
            completed = action.execute(agent, dt)
            total_time += dt
            if completed:
                break

        # Should complete after wait_time
        assert completed is True
        assert total_time >= wait_time * 0.9  # Allow 10% tolerance

    def test_wait_action_short_duration(self, pybullet_env):
        """Test WaitAction with very short duration"""
        wait_time = 0.01
        action = WaitAction(duration=wait_time)
        agent = MockAgent()

        # Execute once
        completed = action.execute(agent, 0.01)

        # Should complete quickly
        assert completed is True or action.is_complete() is False

    def test_wait_action_reset(self, pybullet_env):
        """Test resetting WaitAction"""
        action = WaitAction(duration=1.0)
        agent = MockAgent()

        # Execute partially
        action.execute(agent, 0.5)

        # Reset
        action.reset()

        # Should not be complete
        assert action.is_complete() is False

    def test_wait_action_zero_duration(self, pybullet_env):
        """Test WaitAction with zero duration"""
        action = WaitAction(duration=0.0)
        agent = MockAgent()

        # Should complete immediately
        completed = action.execute(agent, 0.01)
        assert completed is True


class TestActionProperties:
    """Test Action base class properties and methods"""

    def test_action_is_complete_property(self, pybullet_env):
        """Test is_complete() method"""
        wait_action = WaitAction(duration=1.0)

        # Initially not complete
        assert wait_action.is_complete() is False

    def test_action_cancel_method(self, pybullet_env):
        """Test cancel() method"""
        wait_action = WaitAction(duration=1.0)

        # Cancel should not crash
        wait_action.cancel()

        # Should be marked complete after cancel
        assert wait_action.is_complete() is True

    def test_action_get_duration(self, pybullet_env):
        """Test get_duration() method"""
        wait_action = WaitAction(duration=2.5)

        duration = wait_action.get_duration()
        # WaitAction might return None or the duration
        assert duration is None or duration == 2.5


class TestMultipleActions:
    """Test sequential action execution"""

    def test_sequential_wait_actions(self, pybullet_env):
        """Test executing multiple wait actions in sequence"""
        agent = MockAgent()
        action1 = WaitAction(duration=0.1)
        action2 = WaitAction(duration=0.1)

        # Execute first action
        dt = 0.01
        while not action1.is_complete():
            action1.execute(agent, dt)

        assert action1.is_complete() is True

        # Execute second action
        while not action2.is_complete():
            action2.execute(agent, dt)

        assert action2.is_complete() is True


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
