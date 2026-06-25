"""Command-line interface for pybullet_fleet.

Exposes ``pybullet-fleet examples`` so a pip-installed user can locate, list,
copy, or run the bundled example scripts without cloning the repository. The
examples ship inside the wheel (see ``[tool.setuptools.package-data]``), so they
live next to the installed package.
"""

import argparse
import os
import runpy
import shutil
import sys
from typing import List, Optional, Tuple


def _examples_dir() -> str:
    """Absolute path to the bundled examples directory (inside the package)."""
    import pybullet_fleet

    return os.path.join(os.path.dirname(pybullet_fleet.__file__), "examples")


def _iter_examples(root: str):
    """Yield ``(relpath, fullpath)`` for every example ``.py`` under *root*."""
    for dirpath, _dirs, files in os.walk(root):
        for fn in sorted(files):
            if fn.endswith(".py"):
                full = os.path.join(dirpath, fn)
                yield os.path.relpath(full, root), full


def _find_example(root: str, name: str) -> List[Tuple[str, str]]:
    """Return ``(relpath, fullpath)`` matches for *name* (stem or relative path).

    Relative paths are compared with ``/`` separators on every platform, so
    ``--run mobile/path_following_demo`` works on Windows too.
    """
    target = os.path.splitext(name)[0].replace(os.sep, "/")
    matches = []
    for rel, full in _iter_examples(root):
        stem = os.path.splitext(os.path.basename(rel))[0]
        rel_no_ext = os.path.splitext(rel)[0].replace(os.sep, "/")
        if target in (stem, rel_no_ext):
            matches.append((rel, full))
    return matches


def _cmd_examples(args: argparse.Namespace, extra: List[str]) -> int:
    root = _examples_dir()
    if not os.path.isdir(root):
        print(
            f"examples not found at {root} — is pybullet_fleet installed with its bundled data?",
            file=sys.stderr,
        )
        return 1

    if args.path:
        print(root)
        return 0

    if args.copy:
        dest = os.path.abspath(args.copy)
        shutil.copytree(root, dest, dirs_exist_ok=True)
        print(f"Copied examples to {dest}")
        print("Edit and run them, e.g.:")
        print(f"  python {os.path.join(dest, 'mobile', 'path_following_demo.py')}")
        return 0

    if args.run:
        matches = _find_example(root, args.run)
        if not matches:
            print(f"No example matching {args.run!r}. Try `pybullet-fleet examples --list`.", file=sys.stderr)
            return 1
        if len(matches) > 1:
            print(f"Ambiguous name {args.run!r}; matches:", file=sys.stderr)
            for rel, _ in matches:
                print(f"  {os.path.splitext(rel)[0]}", file=sys.stderr)
            return 1
        _rel, full = matches[0]
        # Forward trailing args to the example, mimicking `python <example> ...`.
        # Restore sys.argv afterwards so a raising example (or SystemExit) can't
        # leave it mutated for later calls in the same process.
        saved_argv = sys.argv
        sys.argv = [full] + list(extra)
        try:
            runpy.run_path(full, run_name="__main__")
        finally:
            sys.argv = saved_argv
        return 0

    # Default action: list.
    print(f"Bundled examples (installed at {root}):\n")
    current = None
    for rel, _full in _iter_examples(root):
        category = os.path.dirname(rel) or "."
        if category != current:
            current = category
            print(f"  [{category}]")
        print(f"    {os.path.splitext(os.path.basename(rel))[0]}")
    print("\n  pybullet-fleet examples --run <name>      run one (GUI)")
    print("  pybullet-fleet examples --copy ./examples  copy them out to edit")
    print("  pybullet-fleet examples --path             print the install directory")
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="pybullet-fleet", description="PyBulletFleet command-line tools")
    sub = parser.add_subparsers(dest="command")

    ex = sub.add_parser("examples", help="list, locate, copy, or run the bundled examples")
    group = ex.add_mutually_exclusive_group()
    group.add_argument("--list", action="store_true", help="list available examples (default action)")
    group.add_argument("--path", action="store_true", help="print the directory where the examples are installed")
    group.add_argument("--copy", metavar="DEST", help="copy the examples into DEST so you can edit them")
    group.add_argument("--run", metavar="NAME", help="run an example by name (e.g. path_following_demo)")

    # parse_known_args so trailing flags after `--run NAME` (e.g. --duration 5)
    # are forwarded to the example instead of rejected by this parser.
    args, extra = parser.parse_known_args(argv)
    if args.command == "examples":
        # Only --run forwards extra args; for the other actions, unknown args are
        # a user mistake (e.g. a typo) and should be reported, not silently dropped.
        if extra and not args.run:
            parser.error(f"unrecognized arguments: {' '.join(extra)}")
        return _cmd_examples(args, extra)
    parser.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
