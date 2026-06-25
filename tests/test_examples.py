"""Validity checks for the bundled example scripts.

These verify the example files ship and are well-formed; they do NOT execute the
demos (most open a GUI and run until closed).
"""

import glob
import os
import py_compile

import pytest

import pybullet_fleet

_EX_DIR = os.path.join(os.path.dirname(pybullet_fleet.__file__), "examples")
_EXAMPLE_FILES = sorted(glob.glob(os.path.join(_EX_DIR, "**", "*.py"), recursive=True))


def test_examples_present():
    assert _EXAMPLE_FILES, f"no bundled example scripts found under {_EX_DIR}"


@pytest.mark.parametrize("path", _EXAMPLE_FILES, ids=[os.path.relpath(p, _EX_DIR) for p in _EXAMPLE_FILES])
def test_every_example_compiles(path):
    # Cheap validity check for *all* demos (no GUI / no execution): catches syntax
    # errors and broken edits in any example, which are otherwise untested.
    py_compile.compile(path, doraise=True)
