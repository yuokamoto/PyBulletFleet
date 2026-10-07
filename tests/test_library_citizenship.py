"""Importing and constructing the library must not take over the process.

Three things a program embedding PyBulletFleet should be able to rely on:
its own logging configuration survives, a simulation it never ran still
releases its client, and a headless simulation puts nothing on screen.
"""

import logging

import pybullet as p
import pytest

from pybullet_fleet import MultiRobotSimulationCore, SimulationParams

HEADLESS = dict(gui=False, monitor=False, enable_monitor_gui=False)


class TestLoggingStaysOnThePackageLogger:
    def test_constructing_a_simulation_does_not_touch_the_root_logger(self):
        """log_level defaults to "warn", so setting it on the root logger
        turned off every INFO line in the host application."""
        root = logging.getLogger()
        before = root.level
        sim = MultiRobotSimulationCore(SimulationParams(log_level="error", **HEADLESS))
        try:
            assert root.level == before
        finally:
            sim.close()

    def test_it_sets_the_package_logger_instead(self):
        sim = MultiRobotSimulationCore(SimulationParams(log_level="error", **HEADLESS))
        try:
            assert logging.getLogger("pybullet_fleet").level == logging.ERROR
        finally:
            sim.close()

    def test_the_root_logger_has_no_handler_added_by_the_import(self):
        """basicConfig() on import installed one on the root logger, which
        changed the format of the host application's own output."""
        import pybullet_fleet  # noqa: F401

        package = logging.getLogger("pybullet_fleet")
        assert package.handlers, "the package logger carries its own handler"
        assert all(h not in logging.getLogger().handlers for h in package.handlers)


class TestClose:
    def test_close_disconnects_the_client(self):
        """A simulation that is only built, never run, used to leak its
        client for the life of the process -- and PyBullet's module-level
        calls then resolve against a stale default client, so the *next*
        simulation reports bodies and joints "not found"."""
        sim = MultiRobotSimulationCore(SimulationParams(**HEADLESS))
        sim.initialize_simulation()
        client = sim.client
        assert p.isConnected(client)
        sim.close()
        assert not p.isConnected(client)

    def test_close_is_safe_twice(self):
        sim = MultiRobotSimulationCore(SimulationParams(**HEADLESS))
        sim.initialize_simulation()
        sim.close()
        sim.close()

    def test_close_is_safe_on_a_simulation_that_never_ran(self):
        sim = MultiRobotSimulationCore(SimulationParams(**HEADLESS))
        sim.close()

    def test_it_works_as_a_context_manager(self):
        with MultiRobotSimulationCore(SimulationParams(**HEADLESS)) as sim:
            sim.initialize_simulation()
            client = sim.client
            assert p.isConnected(client)
        assert not p.isConnected(client)


class TestMonitorWindowFollowsGui:
    def test_headless_opens_no_monitor_window(self):
        """enable_monitor_gui used to default to True regardless of gui, so a
        program building several simulations got a Tk window per simulation
        on a machine with no viewer in sight."""
        sim = MultiRobotSimulationCore(SimulationParams(gui=False, monitor=True))
        try:
            assert sim._data_monitor is not None, "data is still collected"
            assert sim._data_monitor.enable_gui is False
        finally:
            sim.close()

    def test_an_explicit_true_is_still_honoured(self):
        sim = MultiRobotSimulationCore(SimulationParams(gui=False, monitor=True, enable_monitor_gui=True))
        try:
            assert sim._data_monitor.enable_gui is True
        finally:
            sim.close()

    def test_an_explicit_false_is_still_honoured(self):
        sim = MultiRobotSimulationCore(SimulationParams(gui=False, monitor=True, enable_monitor_gui=False))
        try:
            assert sim._data_monitor.enable_gui is False
        finally:
            sim.close()
