"""Tests for package-level metadata exposed by pybullet_fleet."""

import re

import pybullet_fleet


def test_version_is_exposed():
    assert isinstance(pybullet_fleet.__version__, str)
    assert pybullet_fleet.__version__


def test_version_looks_like_semver_or_fallback():
    # Either a real installed version (e.g. "0.4.1", possibly with a local/dev
    # suffix) or the not-installed fallback.
    v = pybullet_fleet.__version__
    assert v == "0.0.0+unknown" or re.match(r"^\d+\.\d+\.\d+", v), v


def test_version_in_all():
    assert "__version__" in pybullet_fleet.__all__
