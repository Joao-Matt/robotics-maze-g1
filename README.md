# Robotics Maze G1

Milestone-based MuJoCo + Unitree G1 maze-navigation assignment repo.

This repository currently implements **Milestone 1**: project scaffold, reproducible Python environment, configuration loading, MuJoCo installation, Unitree G1 model bring-up, smoke command, tests, and engineering worklog. Maze generation, navigation, data logging, metrics, and demo behavior are intentionally left for later milestones.

## Repository Layout

```text
configs/        YAML configuration files
assets/         third-party MuJoCo model assets via Git submodules
sim/            simulation integration and shared setup helpers
maze/           future maze generation and validation code
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
make test
make run SEED=1 DURATION=3
make view SEED=1 VIEW_DURATION=30
```

`make smoke` loads `configs/default.yaml` and prints a short environment/config summary. `make test` validates config loading, package imports, and runner error handling. `make run` loads the configured Unitree G1 MuJoCo scene, steps simulation, and prints a state summary. `make view` launches the MuJoCo passive viewer if your terminal has display/OpenGL access.

To try the MuJoCo passive viewer:

```bash
make view SEED=1 VIEW_DURATION=30
```

## Model Assets

The Unitree G1 model is loaded from MuJoCo Menagerie:

```text
assets/mujoco_menagerie/unitree_g1/scene.xml
```

The model is used through a Git submodule so upstream license files and model history stay intact.

## Next Milestone

Milestone 2 should add deterministic maze generation and validation only. Keep the boundary clear: simulator ground truth may be used for debugging and evaluation, but final navigation should not secretly depend on privileged simulator state.
