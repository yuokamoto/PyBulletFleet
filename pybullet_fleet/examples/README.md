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
to be watched, not run in CI. Some `models/` demos need extras:
`pip install 'pybullet-fleet[sdf,models]'`.

## Running them after `pip install` (no clone needed)

The examples ship **inside the wheel**, so a pip-installed package can list,
locate, copy, or run them via the `pybullet-fleet` CLI:

```bash
pip install pybullet-fleet

pybullet-fleet examples --list                 # what's available
pybullet-fleet examples --path                 # where they're installed
pybullet-fleet examples --run path_following_demo   # launch one (GUI)
pybullet-fleet examples --copy ./my-examples    # copy them out to read/edit
```

`--run` forwards trailing flags to the example
(`pybullet-fleet examples --run path_following_demo --duration 5`). If an example
flag clashes with a CLI option name (e.g. some demos have their own `--list`),
use `--copy` and run the copied file directly instead.

## Where are they installed?

`pybullet-fleet examples --path` prints the directory. It is alongside the
installed package:

- **pip install** → `…/site-packages/pybullet_fleet/examples/`
- **editable (`pip install -e .`)** → `…/<repo>/pybullet_fleet/examples/` (your
  working tree — edit these directly)

To read or tweak a demo from a normal install, copy it out first
(`pybullet-fleet examples --copy ./my-examples`) and edit the copy; the originals
in `site-packages` are part of the installed package.

## Testing released vs local code side by side

`scripts/setup_example_venvs.sh` builds two virtualenvs so you can compare the
released package against your working tree:

```bash
scripts/setup_example_venvs.sh            # install venv tracks latest PyPI
scripts/setup_example_venvs.sh 0.4.1      # or pin a version

# released package (what users get):
.venvs/example-install/bin/pybullet-fleet examples --run path_following_demo

# your working tree (editable install):
.venvs/example-mount/bin/pybullet-fleet examples --run path_following_demo
```

Both venvs install the package, so each runs its own bundled examples and library:
`example-install` exercises the released wheel (including its bundled `robots/`,
`config/`, and `mesh/` data), `example-mount` runs this checkout. Complementary to
`scripts/test_clean_install.sh <version>`, which smoke-tests the same packaging
non-interactively.
