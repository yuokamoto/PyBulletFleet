# Examples

Runnable demos for PyBulletFleet, grouped by area:

| Folder | What it shows |
|--------|---------------|
| `basics/` | Spawning, the action system, the event bus |
| `mobile/` | Path following and mobile-robot navigation |
| `arm/` | Arm pick/drop, end-effector (IK) control, mobile manipulators |
| `models/` | Bundled model names, `robot_descriptions`, SDF worlds |
| `scale/` | 100–1000 robot fleets |

Most demos open a PyBullet GUI window and run until you close it — they are meant
to be watched, not run in CI.

## Which `pybullet_fleet` does an example use?

Each example reads its package source from the `PBF_USE_INSTALLED` environment
variable (default `"1"`):

- **unset / `1` (default)** — use the **installed** `pybullet_fleet` package.
  This is how end users run, and it works whether the package was installed from
  PyPI or with `pip install -e .`.
- **`0`** — prepend this checkout to `sys.path` so a **bare clone** (no install)
  runs against the source tree. You normally don't need this inside a venv.

```bash
pip install pybullet-fleet
python examples/mobile/path_following_demo.py          # uses the installed package

PBF_USE_INSTALLED=0 python examples/mobile/path_following_demo.py  # bare checkout, no install
```

Some `models/` demos need extras: `pip install 'pybullet-fleet[sdf,models]'`.

## Testing released vs local code side by side

`scripts/setup_example_venvs.sh` builds two virtualenvs so you can compare the
released package against your working tree:

```bash
scripts/setup_example_venvs.sh            # install venv tracks latest PyPI
scripts/setup_example_venvs.sh 0.4.1      # or pin a version

# released package (what users get):
.venvs/example-install/bin/python examples/mobile/path_following_demo.py

# your working tree (editable install):
.venvs/example-mount/bin/python examples/mobile/path_following_demo.py
```

Both venvs install the package, so each runs the right code with the default
`PBF_USE_INSTALLED=1`: `example-install` runs the released wheel, `example-mount`
runs this checkout (editable install). This is the no-mount way to confirm a
release's bundled `robots/`, `config/`, and `mesh/` data resolve for real users —
complementary to `scripts/test_clean_install.sh <version>`, which smoke-tests the
same packaging non-interactively.
