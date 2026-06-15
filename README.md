# Robotics Maze G1

Milestone-based MuJoCo + Unitree G1 maze-navigation assignment repo.

This repository currently implements **Milestone 3**: project scaffold, reproducible Python environment, configuration loading, MuJoCo installation, Unitree G1 model bring-up, deterministic maze generation/validation, maze-to-MuJoCo world generation, visible command artifacts, tests, and engineering worklog. Navigation, data logging, metrics, and demo behavior are intentionally left for later milestones.

## Repository Layout

```text
configs/        YAML configuration files
assets/         third-party MuJoCo model assets via Git submodules
sim/            simulation integration and shared setup helpers
maze/           deterministic maze generation, validation, and visualization
nav/            future navigation, planning, localization, and control code
data/           future run logging and data schema code
eval/           future KPI and report code
demo/           future live demo entrypoints
scripts/        runnable command-line scripts
tests/          automated tests
docs/           worklog and architecture decision records
runs/           generated run artifacts, ignored by Git
```

## Environment

The project is intended to use Python 3.11 from pyenv and a repo-local virtual environment:

```bash
cd /home/gary/dev_workspaces/robotics-maze-g1
git submodule update --init --recursive
make setup
```

If Python 3.11.15 is missing, install it first:

```bash
env \
  CPPFLAGS=-I/home/gary/.local/openssl/include \
  LDFLAGS=-L/home/gary/.local/openssl/lib \
  PKG_CONFIG_PATH=/home/gary/.local/openssl/lib/pkgconfig \
  LD_LIBRARY_PATH=/home/gary/.local/openssl/lib \
  CONFIGURE_OPTS=--with-openssl=/home/gary/.local/openssl \
  /home/gary/.pyenv/bin/pyenv install 3.11.15
```

On this Ubuntu 20.04 environment, Python 3.11.15 was built against a user-local OpenSSL at `/home/gary/.local/openssl` because interactive `sudo apt` was not available for installing `libssl-dev`. The Makefile sets `LD_LIBRARY_PATH` for repo commands so the venv can use that OpenSSL runtime.

## Milestone Commands

```bash
make smoke
make view-smoke
make test
make view-test
make maze SEED=123
make view-maze SEED=123 MAZE_CELL_PX=48
make world SEED=123
make view-world SEED=123
make run SEED=123 DURATION=3
make view-run SEED=123 DURATION=3
make view SEED=123 VIEW_DURATION=30
```

`make smoke` loads `configs/default.yaml`, prints a short environment/config summary, and writes `runs/visual/smoke_latest.html`. `make test` validates config loading, package imports, runner error handling, maze determinism/solvability, and generated-world XML while writing `runs/visual/test_latest.txt` and `runs/visual/test_latest.html`. `make maze` prints a seeded ASCII maze with the BFS validation path overlaid and writes SVG/ASCII/PGM artifacts. `make world` builds a MuJoCo maze world XML with the G1 placed at the start cell. `make run` opens a side-by-side visual dashboard first, then launches the live MuJoCo passive viewer in the generated maze world. `make view-run` runs headlessly and opens the same side-by-side dashboard with the final render.

## How To See Things

All user-facing commands leave inspectable artifacts under `runs/visual/`.

```bash
make view-smoke
make view-test
make view-maze SEED=123 MAZE_CELL_PX=48
make view-world SEED=123
make run SEED=123 DURATION=30
make view-run SEED=123 DURATION=3
```

The `view-*` commands run the underlying command and open the latest artifact with `xdg-open` when `DISPLAY` is available. If no desktop opener is available, the command prints the exact artifact path.

Expected artifacts include:

```text
runs/visual/smoke_latest.html
runs/visual/test_latest.html
runs/visual/test_latest.txt
runs/visual/maze_seed-123.svg
runs/visual/maze_seed-123.txt
runs/visual/maze_seed-123.pgm
runs/visual/world_seed-123.xml
runs/visual/world_seed-123_summary.json
runs/visual/world_seed-123_topdown.svg
runs/visual/run_seed-123_dashboard.html
runs/visual/run_seed-123_preview.png
runs/visual/run_seed-123_summary.json
runs/visual/run_seed-123_final.png
```

To generate and validate an ASCII maze:

```bash
make maze SEED=123
```

To save and view simple visual artifacts:

```bash
make view-maze SEED=123
```

This writes `runs/visual/maze_seed-123.svg`, `runs/visual/maze_seed-123.txt`, and `runs/visual/maze_seed-123.pgm`. If your terminal has `DISPLAY` and `xdg-open`, it also opens the SVG image. Increase `MAZE_CELL_PX` for a larger rendering:

```bash
make view-maze SEED=123 MAZE_CELL_PX=48
```

To try the MuJoCo passive viewer:

```bash
make run SEED=123 DURATION=30
```

`make run` opens `runs/visual/run_seed-123_dashboard.html` first. The dashboard shows the 2D maze representation next to a MuJoCo render of the generated maze world. Then the live MuJoCo viewer opens so you can inspect/travel through the scene with the viewer camera controls. It does not yet drive the robot through the maze.

For a non-realtime inspection run:

```bash
make view-run SEED=123 DURATION=3
```

To build only the generated MuJoCo maze world:

```bash
make world SEED=123
make view-world SEED=123
```

The generated world uses the maze center as the MuJoCo origin:

```text
x = (col - (width_cells - 1) / 2) * cell_size_m
y = ((height_cells - 1) / 2 - row) * cell_size_m
z = 0 is floor height
```

The default `make run` now uses `WORLD=maze`. To run the original empty Unitree scene from Milestone 1:

```bash
make run WORLD=empty SEED=1 DURATION=3
make view WORLD=empty SEED=1 VIEW_DURATION=30
```

## Model Assets

The Unitree G1 model is loaded from MuJoCo Menagerie:

```text
assets/mujoco_menagerie/unitree_g1/scene.xml
assets/mujoco_menagerie/unitree_g1/g1.xml
```

The generated maze world is written under `runs/visual/` and uses `g1.xml` as its base model. The model is used through a Git submodule so upstream license files and model history stay intact.

## Next Milestone

Milestone 4 should add an explicit oracle planner baseline over the generated maze grid. Keep the boundary clear: simulator ground truth may be used for debugging and evaluation, but final navigation should not secretly depend on privileged simulator state.
