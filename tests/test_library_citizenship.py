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

    def test_the_root_logger_carries_nothing_of_ours(self):
        """basicConfig() on import installed a handler on the root logger,
        which changed the format of the host application's own output.

        Asserted as "root has none of ours", not "the package has one": when
        the test runner or a fixture has already configured logging, the
        package deliberately installs nothing, and asserting otherwise would
        make this fail for the wrong reason.
        """
        import pybullet_fleet  # noqa: F401

        package_handlers = set(map(id, logging.getLogger("pybullet_fleet").handlers))
        root_handlers = set(map(id, logging.getLogger().handlers))
        assert package_handlers & root_handlers == set()


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


class TestCloseIsOneShotAndAlwaysReleases:
    """Cases raised in review on the first version of close()."""

    def test_shutdown_callbacks_run_once(self):
        """`with sim:` around a run_simulation() that already closed would
        otherwise ask every plugin to shut down twice -- and a plugin that
        closes a file or a socket there cannot do it twice."""
        sim = MultiRobotSimulationCore(SimulationParams(**HEADLESS))
        sim.initialize_simulation()
        calls = []
        sim._shutdown_plugins = lambda: calls.append(1)  # type: ignore[method-assign]

        sim.close()
        sim.close()

        assert calls == [1]

    def test_the_client_is_released_even_if_finalising_a_recording_raises(self):
        """stop_recording() can raise while saving -- an MP4 without imageio,
        for one -- and letting that escape before the disconnect would leak
        the client this API exists to release."""

        sim = MultiRobotSimulationCore(SimulationParams(**HEADLESS))
        sim.initialize_simulation()
        client = sim.client

        class Boom:
            pass

        sim._recorder = Boom()
        sim.stop_recording = lambda: (_ for _ in ()).throw(RuntimeError("imageio missing"))  # type: ignore

        with pytest.raises(RuntimeError, match="imageio missing"):
            sim.close()
        assert not p.isConnected(client)

    def test_the_client_id_stays_readable_after_close(self):
        """run_simulation() ends by calling close(), and callers read .client
        afterwards, so clearing it would break every such caller. The _closed
        guard is what stops a second disconnect, not a cleared id."""
        sim = MultiRobotSimulationCore(SimulationParams(**HEADLESS))
        sim.initialize_simulation()
        client = sim.client
        sim.close()
        assert sim.client == client

    def test_a_second_close_cannot_disconnect_someone_else(self):
        first = MultiRobotSimulationCore(SimulationParams(**HEADLESS))
        first.initialize_simulation()
        first.close()

        second = MultiRobotSimulationCore(SimulationParams(**HEADLESS))
        second.initialize_simulation()
        try:
            first.close()
            assert p.isConnected(second.client)
        finally:
            second.close()


class TestNoDuplicateLogRecords:
    """Records propagate to root, so a handler here plus the host's own would
    print every PyBulletFleet line twice."""

    @staticmethod
    def _loggers(name, root_handlers, package_handlers):
        package, root = logging.getLogger(f"t.{name}.pkg"), logging.getLogger(f"t.{name}.root")
        package.handlers.clear()
        root.handlers.clear()
        package.handlers.extend(package_handlers)
        root.handlers.extend(root_handlers)
        return package, root

    def test_no_handler_when_the_host_has_configured_one(self):
        from pybullet_fleet.core_simulation import _install_default_handler

        package, root = self._loggers("a", [logging.NullHandler()], [])
        assert _install_default_handler(package, root) is False
        assert package.handlers == []

    def test_a_handler_when_nothing_else_is_configured(self):
        """A program that configured nothing still gets the output
        basicConfig() used to give it."""
        from pybullet_fleet.core_simulation import _install_default_handler

        package, root = self._loggers("b", [], [])
        assert _install_default_handler(package, root) is True
        assert len(package.handlers) == 1

    def test_it_does_not_stack_handlers(self):
        from pybullet_fleet.core_simulation import _install_default_handler

        package, root = self._loggers("c", [], [logging.NullHandler()])
        assert _install_default_handler(package, root) is False
        assert len(package.handlers) == 1


class TestTeardownStepsAllRun:
    """Raised in review: one failing step must not strand the resources the
    later steps own, and _closed means a retry would be a no-op."""

    def _sim_that_fails_to_finalise(self):
        sim = MultiRobotSimulationCore(SimulationParams(**HEADLESS))
        sim.initialize_simulation()
        sim._recorder = object()
        sim.stop_recording = lambda: (_ for _ in ()).throw(RuntimeError("imageio missing"))  # type: ignore
        return sim

    def test_plugins_still_shut_down(self):
        sim = self._sim_that_fails_to_finalise()
        calls = []
        sim._shutdown_plugins = lambda: calls.append("plugins")  # type: ignore[method-assign]

        with pytest.raises(RuntimeError, match="imageio missing"):
            sim.close()

        assert calls == ["plugins"]

    def test_the_client_is_still_released(self):
        sim = self._sim_that_fails_to_finalise()
        client = sim.client
        with pytest.raises(RuntimeError, match="imageio missing"):
            sim.close()
        assert not p.isConnected(client)

    def test_the_first_failure_is_the_one_raised(self):
        sim = self._sim_that_fails_to_finalise()
        sim._shutdown_plugins = lambda: (_ for _ in ()).throw(ValueError("second"))  # type: ignore

        with pytest.raises(RuntimeError, match="imageio missing"):
            sim.close()


class TestPropagation:
    """Propagation stays on, so records still reach handlers the host adds
    later -- pytest's caplog among them."""

    def test_installing_a_handler_leaves_propagation_alone(self):
        from pybullet_fleet.core_simulation import _install_default_handler

        package, root = logging.getLogger("t.prop.pkg"), logging.getLogger("t.prop.root")
        package.handlers.clear()
        root.handlers.clear()
        package.propagate = True

        assert _install_default_handler(package, root) is True
        assert package.propagate is True

    def test_the_packages_records_reach_the_root_logger(self, caplog):
        """What propagate=False would have cost: this suite captures
        PyBulletFleet's own records this way in 17 places."""
        with caplog.at_level(logging.WARNING, logger="pybullet_fleet"):
            logging.getLogger("pybullet_fleet.core_simulation").warning("audible")
        assert any("audible" in record.message for record in caplog.records)
