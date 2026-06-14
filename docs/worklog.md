# Worklog

## 2026-06-14 - Milestone 0: project scaffold and reproducible setup

### Goal
Create a clean, Git-ready repository skeleton with reproducible Python setup, basic configuration conventions, smoke validation, and initial tests. Do not implement maze generation, MuJoCo simulation, robot control, metrics, or demo behavior yet.

### Changes made
- Created the repository at `/home/gary/dev_workspaces/robotics-maze-g1`.
- Added package folders for `sim`, `maze`, `nav`, `data`, `eval`, and `demo`.
- Added `configs/default.yaml` with initial project, simulation, maze, robot, and logging settings.
- Added `sim/config.py` for YAML loading and top-level config validation.
- Added `scripts/smoke_test.py`, pytest tests, `Makefile`, `requirements.txt`, `.gitignore`, `.python-version`, `README.md`, and ADR template.
- Installed pyenv at `/home/gary/.pyenv`, built OpenSSL 3.0.16 at `/home/gary/.local/openssl`, and installed Python 3.11.15 with SSL support.

### Key decisions
- [REPRODUCIBILITY] Decision: use pyenv-managed Python 3.11 plus a repo-local `.venv`.
  - Why: the system Python is 3.8.10, while later robotics dependencies are more likely to support a newer Python cleanly.
  - Alternatives considered: use system Python 3.8, install Python through apt/PPA.
  - Tradeoff accepted: pyenv adds a build step, but avoids changing system Python.
- [TRADEOFF] Decision: keep Milestone 0 dependency-light and exclude MuJoCo until Milestone 1.
  - Why: the first milestone should prove repository shape and commands without mixing in simulator bring-up risk.
  - Alternatives considered: add MuJoCo immediately.
  - Tradeoff accepted: smoke tests do not yet prove simulator readiness.
- [REPRODUCIBILITY] Decision: keep generated run artifacts out of Git except for `runs/.gitkeep`.
  - Why: future runs may contain large logs, frames, and generated plots.
  - Alternatives considered: track sample run outputs immediately.
  - Tradeoff accepted: sample artifacts can be added later if useful.

### Validation performed
- Commands run:
  - `make setup`
  - `make smoke`
  - `make test`
- Tests passed/failed:
  - `make setup` passed and installed PyYAML 6.0.3 and pytest 9.1.0.
  - `make smoke` passed using Python 3.11.15.
  - `make test` passed: 4 tests passed.
- Visual checks/screenshots/log files:
  - Not applicable for Milestone 0.

### Problems encountered
- Problem: system Python was 3.8.10 and pyenv was not installed.
  - Symptom: `command -v pyenv` returned no result.
  - Suspected cause: fresh Ubuntu 20.04 environment.
  - Fix or mitigation: installed pyenv under `/home/gary/.pyenv` and targeted Python 3.11.15.
  - Remaining risk: this Python was built without optional modules that require unavailable system headers: `bz2`, `curses`, `readline`, `sqlite3`, `tkinter`, and `lzma`.
- Problem: the first Python 3.11.15 build lacked `_ssl`.
  - Symptom: pyenv reported `ERROR: The Python ssl extension was not compiled`.
  - Suspected cause: `libssl-dev` was missing and `sudo apt` required an interactive password.
  - Fix or mitigation: built OpenSSL 3.0.16 under `/home/gary/.local/openssl`, rebuilt Python with `CONFIGURE_OPTS=--with-openssl=/home/gary/.local/openssl`, and verified `ssl.OPENSSL_VERSION`.
  - Remaining risk: future Python rebuilds on this machine should use the same OpenSSL environment variables unless system development packages are installed.

### Result
The repository scaffold is in place with clear Milestone 0 boundaries. The config loader, smoke command, tests, and documentation are validated, and the repo is ready for the first commit.

### Next actions
Start Milestone 1 by adding MuJoCo and Unitree G1 bring-up in an empty world. Keep simulator ground truth explicitly separated from final autonomy code.

## 2026-06-14 - Milestone 1: MuJoCo and Unitree G1 bring-up

### Goal
Load the Unitree G1 model in MuJoCo, step a short empty-world simulation, and print enough simulator state to prove the model, dependency, and runner are usable before adding maze logic.

### Changes made
- Added MuJoCo 3.9.0 to `requirements.txt`.
- Added MuJoCo Menagerie as a Git submodule at `assets/mujoco_menagerie`.
- Added `robot.model_xml_path`, `robot.initial_keyframe`, and `sim.default_duration_s` to `configs/default.yaml`.
- Added `sim/mujoco_runner.py`, `sim/robot_interface.py`, and `scripts/run_episode.py`.
- Added `make run SEED=1 DURATION=3` and documented the submodule/model workflow in `README.md`.
- Added tests for the configured G1 model path and MuJoCo runner error handling.

### Key decisions
- [REPRODUCIBILITY] Decision: use MuJoCo Menagerie as a Git submodule.
  - Why: it preserves upstream model layout, mesh paths, license files, and update history.
  - Alternatives considered: vendoring only `unitree_g1`, requiring an external local path.
  - Tradeoff accepted: cloning the repo is heavier, but model provenance is clearer.
- [LOCOMOTION][DEMO_RISK] Decision: load `unitree_g1/scene.xml` and reset to the `stand` keyframe.
  - Why: `scene.xml` includes a floor/light scene, and `stand` avoids raw zero-state bring-up.
  - Alternatives considered: load `g1.xml` directly or use no keyframe.
  - Tradeoff accepted: this is still passive simulation, not locomotion control.
- [GROUND_TRUTH_BOUNDARY] Decision: allow direct MuJoCo state access inside `sim/` for this milestone.
  - Why: Milestone 1 is simulator bring-up, not final autonomy.
  - Alternatives considered: introduce sensor/pose abstractions immediately.
  - Tradeoff accepted: later milestones must keep navigation from depending on privileged state except in explicit debug/oracle modes.

### Validation performed
- Commands run:
  - `git submodule add https://github.com/google-deepmind/mujoco_menagerie.git assets/mujoco_menagerie`
  - `make setup`
  - `make run SEED=1 DURATION=3`
  - `make smoke`
  - `make test`
- Tests passed/failed:
  - `make setup` passed and installed MuJoCo 3.9.0.
  - `make run SEED=1 DURATION=3` passed, loaded G1, reset to `stand`, stepped 1500 steps, and ended at sim time 3.0 seconds.
  - `make smoke` passed.
  - `make test` passed: 8 tests passed.
- Visual checks/screenshots/log files:
  - No viewer check was performed in this milestone validation; headless stepping was validated.

### Problems encountered
- Problem: no Unitree G1 XML was present before Milestone 1.
  - Symptom: local XML search found no model files.
  - Suspected cause: Milestone 0 intentionally excluded simulator assets.
  - Fix or mitigation: added MuJoCo Menagerie as a submodule and configured `assets/mujoco_menagerie/unitree_g1/scene.xml`.
  - Remaining risk: future clones must run `git submodule update --init --recursive`.
- Problem: viewer support is environment-dependent.
  - Symptom: not validated during the headless command-line run.
  - Suspected cause: passive viewer may need display/OpenGL support.
  - Fix or mitigation: keep `--viewer` optional and make non-viewer stepping the required Milestone 1 validation.
  - Remaining risk: live demo milestones may need additional display/OpenGL setup.

### Result
MuJoCo and the Unitree G1 model are usable from the repo. A short passive simulation loads the model, steps cleanly, prints joint/base state, and reports no fall at the end of the 3 second bring-up run.

### Next actions
Start Milestone 2 by implementing deterministic maze generation and validation as pure logic, without integrating MuJoCo or navigation yet.
