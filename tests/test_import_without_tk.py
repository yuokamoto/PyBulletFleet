"""pybullet_fleet imports and runs headless where Tk cannot be loaded.

A slim container image has the ``tkinter`` package but not the Tk shared
library, and then ``import tkinter`` raises a plain ``ImportError`` (for
example ``libtk8.6.so: cannot open shared object file``), not
``ModuleNotFoundError``. Each case runs in a fresh interpreter, because the
package and ``tkinter`` are already imported in this one.
"""

import os
import subprocess
import sys
import textwrap

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Make `import tkinter` fail the given way, before anything else imports it.
_BLOCK_TK = """
import sys

class _NoTk:
    def find_spec(self, name, path=None, target=None):
        if name in ("tkinter", "_tkinter") or name.startswith("tkinter."):
            raise {error}("libtk8.6.so: cannot open shared object file: No such file or directory")
        return None

sys.meta_path.insert(0, _NoTk())
"""


def _run(error: str, body: str) -> subprocess.CompletedProcess:
    script = _BLOCK_TK.format(error=error) + textwrap.dedent(body)
    env = dict(os.environ, PYTHONPATH=REPO_ROOT + os.pathsep + os.environ.get("PYTHONPATH", ""))
    return subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, env=env, timeout=120)


@pytest.mark.parametrize("error", ["ImportError", "ModuleNotFoundError"])
def test_the_package_imports_and_steps_with_the_monitor_off(error):
    result = _run(
        error,
        """
        import pybullet_fleet
        from pybullet_fleet import MultiRobotSimulationCore, SimulationParams

        sim = MultiRobotSimulationCore(SimulationParams(gui=False, monitor=False, log_level="warning"))
        sim.step_once()
        print("ok")
        """,
    )
    assert result.returncode == 0, result.stderr
    assert "ok" in result.stdout.splitlines()  # PyBullet prints its own lines too
    assert "tkinter" not in result.stderr  # nothing to say when the window was never asked for


@pytest.mark.parametrize("error", ["ImportError", "ModuleNotFoundError"])
def test_asking_for_the_window_falls_back_to_the_file_and_says_why(error):
    result = _run(
        error,
        """
        from pybullet_fleet.data_monitor import DataMonitor

        monitor = DataMonitor(enable_gui=True)
        assert monitor.enable_gui is False
        print("ok")
        """,
    )
    assert result.returncode == 0, result.stderr
    assert "ok" in result.stdout.splitlines()  # PyBullet prints its own lines too
    assert "tkinter is unavailable" in result.stderr
