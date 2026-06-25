"""Tests for the pybullet-fleet command-line interface (examples subcommand)."""

import glob
import os
import py_compile
import sys

import pytest

from pybullet_fleet import cli

# All bundled example scripts (computed once, used to parametrize per-file checks).
_EXAMPLE_FILES = sorted(glob.glob(os.path.join(cli._examples_dir(), "**", "*.py"), recursive=True))


def test_examples_dir_exists_and_is_in_package():
    root = cli._examples_dir()
    assert os.path.isdir(root)
    # Lives next to the installed package (so it ships in the wheel).
    import pybullet_fleet

    assert root == os.path.join(os.path.dirname(pybullet_fleet.__file__), "examples")


def test_iter_examples_finds_known_demos():
    root = cli._examples_dir()
    rels = {rel for rel, _full in cli._iter_examples(root)}
    stems = {os.path.splitext(os.path.basename(r))[0] for r in rels}
    assert "path_following_demo" in stems
    assert "robot_demo" in stems
    assert all(r.endswith(".py") for r in rels)


def test_iter_examples_is_complete():
    # _iter_examples must find *every* .py under the examples tree, not a subset
    # (catches missed subdirectories or filtering bugs) — without hardcoding the
    # demo list, so adding a demo doesn't require editing this test.
    root = cli._examples_dir()
    found = {os.path.abspath(full) for _rel, full in cli._iter_examples(root)}
    expected = {os.path.abspath(p) for p in _EXAMPLE_FILES}
    assert found == expected
    assert len(found) > 0


@pytest.mark.parametrize("path", _EXAMPLE_FILES, ids=[os.path.relpath(p, cli._examples_dir()) for p in _EXAMPLE_FILES])
def test_every_example_compiles(path):
    # Cheap validity check for *all* demos (no GUI / no execution): catches syntax
    # errors and broken edits in any example, which are otherwise untested.
    py_compile.compile(path, doraise=True)


def test_find_example_by_stem_and_relpath():
    root = cli._examples_dir()
    by_stem = cli._find_example(root, "path_following_demo")
    assert len(by_stem) == 1
    # .py extension is optional, and a relative path (any separator) also matches.
    assert cli._find_example(root, "path_following_demo.py") == by_stem
    assert cli._find_example(root, "mobile/path_following_demo.py") == by_stem
    assert cli._find_example(root, os.path.join("mobile", "path_following_demo")) == by_stem
    assert cli._find_example(root, "does_not_exist") == []


def test_main_path(capsys):
    rc = cli.main(["examples", "--path"])
    assert rc == 0
    out = capsys.readouterr().out.strip()
    assert out == cli._examples_dir()


def test_main_list(capsys):
    rc = cli.main(["examples", "--list"])
    assert rc == 0
    out = capsys.readouterr().out
    assert "path_following_demo" in out
    assert "[mobile]" in out


def test_main_copy(tmp_path, capsys):
    dest = tmp_path / "copied"
    rc = cli.main(["examples", "--copy", str(dest)])
    assert rc == 0
    assert (dest / "mobile" / "path_following_demo.py").is_file()


def test_main_run_unknown_returns_error(capsys):
    rc = cli.main(["examples", "--run", "no_such_demo"])
    assert rc == 1
    assert "No example matching" in capsys.readouterr().err


def test_main_no_command_prints_help(capsys):
    rc = cli.main([])
    assert rc == 0
    assert "examples" in capsys.readouterr().out


def test_main_run_executes_example_and_restores_argv(tmp_path, monkeypatch, capsys):
    # Drop a throwaway example into a temp tree and point the CLI at it, so we
    # exercise the --run/runpy path without launching a real GUI demo.
    root = tmp_path / "examples" / "basics"
    root.mkdir(parents=True)
    script = root / "tiny_demo.py"
    script.write_text("import sys\nprint('ran', sys.argv[1:])\n")
    monkeypatch.setattr(cli, "_examples_dir", lambda: str(tmp_path / "examples"))
    saved = list(sys.argv)
    rc = cli.main(["examples", "--run", "tiny_demo", "--flag", "x"])
    assert rc == 0
    assert "ran ['--flag', 'x']" in capsys.readouterr().out
    assert sys.argv == saved  # restored even though the example mutated it


def test_main_run_restores_argv_on_error(tmp_path, monkeypatch):
    root = tmp_path / "examples" / "basics"
    root.mkdir(parents=True)
    (root / "boom_demo.py").write_text("raise SystemExit(3)\n")
    monkeypatch.setattr(cli, "_examples_dir", lambda: str(tmp_path / "examples"))
    saved = list(sys.argv)
    with pytest.raises(SystemExit):
        cli.main(["examples", "--run", "boom_demo"])
    assert sys.argv == saved


def test_main_run_ambiguous_name_errors(tmp_path, monkeypatch, capsys):
    # Two examples sharing a stem in different folders -> ambiguous.
    for sub in ("a", "b"):
        d = tmp_path / "examples" / sub
        d.mkdir(parents=True)
        (d / "dup_demo.py").write_text("pass\n")
    monkeypatch.setattr(cli, "_examples_dir", lambda: str(tmp_path / "examples"))
    rc = cli.main(["examples", "--run", "dup_demo"])
    assert rc == 1
    assert "Ambiguous" in capsys.readouterr().err


def test_unknown_arg_rejected_without_run(capsys):
    # Stray flags are a mistake for --list/--path/--copy (only --run forwards them).
    with pytest.raises(SystemExit):
        cli.main(["examples", "--list", "--bogus"])
    assert "unrecognized arguments" in capsys.readouterr().err


def test_examples_missing_dir_errors(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(cli, "_examples_dir", lambda: str(tmp_path / "nope"))
    rc = cli.main(["examples", "--list"])
    assert rc == 1
    assert "not found" in capsys.readouterr().err
