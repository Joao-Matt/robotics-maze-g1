# Worklog

## 2026-06-14 - Milestone 0: project scaffold and reproducible setup

### Goal
Create a clean, Git-ready repository skeleton with reproducible Python setup, basic configuration conventions, smoke validation, and initial tests. Do not implement maze generation, MuJoCo simulation, robot control, metrics, or demo behavior yet.

### Changes made
- Created the repository in the user's development workspace.
- Added package folders for `sim`, `maze`, `nav`, `data`, `eval`, and `demo`.
- Added `configs/default.yaml` with initial project, simulation, maze, robot, and logging settings.
- Added `sim/config.py` for YAML loading and top-level config validation.
- Added `scripts/smoke_test.py`, pytest tests, `Makefile`, `requirements.txt`, `.gitignore`, `.python-version`, `README.md`, and ADR template.
- Installed pyenv under `$HOME/.pyenv`, built OpenSSL 3.0.16 under `$HOME/.local/openssl`, and installed Python 3.11.15 with SSL support.

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
  - Fix or mitigation: installed pyenv under `$HOME/.pyenv` and targeted Python 3.11.15.
  - Remaining risk: this Python was built without optional modules that require unavailable system headers: `bz2`, `curses`, `readline`, `sqlite3`, `tkinter`, and `lzma`.
- Problem: the first Python 3.11.15 build lacked `_ssl`.
  - Symptom: pyenv reported `ERROR: The Python ssl extension was not compiled`.
  - Suspected cause: `libssl-dev` was missing and `sudo apt` required an interactive password.
  - Fix or mitigation: built OpenSSL 3.0.16 under `$HOME/.local/openssl`, rebuilt Python with `CONFIGURE_OPTS=--with-openssl=$HOME/.local/openssl`, and verified `ssl.OPENSSL_VERSION`.
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

## 2026-06-15 - Milestone 2: seeded maze generator and validator

### Goal
Generate deterministic occupancy-grid mazes from integer seeds, validate start/goal/path correctness, and provide a simple visual output without integrating MuJoCo or robot control.

### Changes made
- Added `maze/grid.py`, `maze/generator.py`, `maze/validator.py`, and `maze/visualization.py`.
- Added `scripts/generate_maze.py` for seeded ASCII maze generation and optional ASCII/PGM artifact saving.
- Added `make maze SEED=...` to print a validated maze with path overlay.
- Added `make view-maze SEED=...` to save human-inspectable maze artifacts and open the image when display support is available.
- Added maze tests for determinism, seed variation, endpoint validity, occupancy values, solvability across 20 seeds, blocked-goal failure, odd-dimension validation, and ASCII rendering.
- Updated `README.md` and the milestone plan with explicit visual-inspection requirements.

### Key decisions
- [MAZE_VALIDITY][REPRODUCIBILITY] Decision: use randomized DFS on an odd-sized grid with a local `random.Random(seed)`.
  - Why: it creates deterministic perfect mazes with guaranteed connectivity and no global random side effects.
  - Alternatives considered: Prim-style generation, obstacle-field generation with post-hoc BFS repair.
  - Tradeoff accepted: perfect mazes are simple and reliable but less visually varied than arbitrary obstacle fields.
- [DEMO_RISK][TRADEOFF] Decision: keep corridors one grid cell wide and validate `cell_size_m` against configured corridor/safety width.
  - Why: config currently uses 1.0 m cells and a 0.45 m safety radius, which is a conservative first physical check.
  - Alternatives considered: implement obstacle inflation now.
  - Tradeoff accepted: true clearance/inflation is deferred to planner/world-building milestones.
- [REPRODUCIBILITY] Decision: save dependency-free SVG/PGM images instead of adding matplotlib now.
  - Why: Milestone 2 should stay pure logic and lightweight, while SVG gives a readable scaled view.
  - Alternatives considered: add matplotlib for PNG output.
  - Tradeoff accepted: SVG/PGM are simple artifacts rather than polished plots.

### Validation performed
- Commands run:
  - `make maze SEED=123`
  - `make view-maze SEED=123`
  - `.venv/bin/python scripts/generate_maze.py --seed 123 --show-path --save-ascii runs/maze_seed-123.txt --save-pgm runs/maze_seed-123.pgm --save-svg runs/maze_seed-123.svg`
  - `make test`
  - `make smoke`
- Tests passed/failed:
  - `make maze SEED=123` passed and printed a 15x15 maze with a 61-cell BFS path.
  - `make view-maze SEED=123` passed and wrote viewable artifacts.
  - Saved ASCII, PGM, and readable SVG maze artifacts under `runs/`.
  - `make test` passed: 16 tests passed.
  - `make smoke` passed.
- Visual checks/screenshots/log files:
  - `runs/maze_seed-123.txt`
  - `runs/maze_seed-123.pgm`
  - `runs/maze_seed-123.svg`

### Problems encountered
- Problem: no issue yet.
  - Symptom: Milestone wording originally allowed "it works" without requiring a human-viewable command for each visible state.
  - Suspected cause: acceptance criteria emphasized validation before explicit inspection.
  - Fix or mitigation: updated the milestone plan and README to require visual/inspect commands and artifacts; added `make view-maze`.
  - Remaining risk: Maze-to-world coordinate conventions are not defined until Milestone 3.

### Result
Seeded maze generation and validation work as pure logic. The same seed reproduces the same maze, different seeds produce different layouts, generated mazes validate across at least 20 seeds, and a maze can be printed or saved for inspection.

### Next actions
Start Milestone 3 by converting validated maze grids into MuJoCo wall geometry and documenting the cell-to-world coordinate convention.

## 2026-06-15 - Visible command contract

### Goal
Make every user-facing command produce something inspectable, not only console output. The command surface should support both automated validation and human visual checks for smoke reports, test reports, maze output, MuJoCo still frames, and live viewer runs.

### Changes made
- Added `runs/visual/` as the shared artifact convention for visible command outputs.
- Extended `scripts/smoke_test.py` with `--save-html`.
- Extended `scripts/run_episode.py` with `--save-summary-json`, `--save-render`, `--render-width`, and `--render-height`.
- Added `scripts/run_tests_report.py` to preserve pytest exit codes while writing text and HTML reports.
- Added `scripts/open_artifact.py` to open artifacts with `xdg-open` when `DISPLAY` is available and otherwise print paths.
- Updated `Makefile` with `view-smoke`, `view-test`, and `view-run`, and changed `smoke`, `test`, `maze`, and `run` to write visible artifacts by default.
- Updated README and the milestone plan so future milestones must include visible commands and artifacts.
- Added tests for smoke HTML report creation and readable SVG maze output.

### Key decisions
- [VISIBILITY] Decision: every user-facing command writes an artifact under `runs/visual/`.
  - Why: the project should be inspectable at each stage, especially for visual robotics behavior.
  - Alternatives considered: keep console-only commands and add occasional viewer commands.
  - Tradeoff accepted: commands write a few small generated files, but they stay ignored by Git.
- [DEPENDENCY_CONTROL] Decision: use dependency-free HTML, SVG, text, PGM, and PPM artifacts.
  - Why: this keeps the environment light while still making outputs openable.
  - Alternatives considered: add reporting/plotting/image libraries.
  - Tradeoff accepted: reports are simple rather than polished.
- [DEMO_RISK] Decision: default MuJoCo still renders to 640x480.
  - Why: MuJoCo's default offscreen framebuffer rejected a 960 pixel wide render.
  - Alternatives considered: edit the model XML to increase framebuffer size.
  - Tradeoff accepted: use a conservative render size now and revisit richer visuals when maze-world integration begins.

### Validation performed
- Commands run:
  - `make smoke`
  - `make test`
  - `make maze SEED=123 MAZE_CELL_PX=48`
  - `make run SEED=1 DURATION=3`
  - `make view-smoke`
  - `make view-test`
  - `make view-maze SEED=123 MAZE_CELL_PX=48`
  - `make view-run SEED=1 DURATION=3`
  - `make view SEED=1 VIEW_DURATION=1`
- Tests passed/failed:
  - `make test` passed: 18 tests passed.
  - All `view-*` commands exited successfully and reported `opened_artifact`.
  - `make view SEED=1 VIEW_DURATION=1` opened the live MuJoCo passive viewer path and exited successfully after the short run.
- Visual checks/screenshots/log files:
  - `runs/visual/smoke_latest.html`
  - `runs/visual/test_latest.html`
  - `runs/visual/test_latest.txt`
  - `runs/visual/maze_seed-123.svg`
  - `runs/visual/maze_seed-123.txt`
  - `runs/visual/maze_seed-123.pgm`
  - `runs/visual/run_seed-1_summary.json`
  - `runs/visual/run_seed-1_final.ppm`

### Problems encountered
- Problem: the first MuJoCo still render attempt used 960x720.
  - Symptom: MuJoCo reported that image width 960 exceeded the default 640 offscreen framebuffer width.
  - Suspected cause: the Unitree scene does not define a larger offscreen framebuffer.
  - Fix or mitigation: changed the default still render size to 640x480 in the runner, CLI, and Makefile.
  - Remaining risk: larger final visuals will need either model XML visual globals or a rendering setup decision.
- Problem: the live passive viewer path segfaulted during Python/OpenGL teardown after completing a one-second run.
  - Symptom: the summary printed successfully, then the process exited with signal 139.
  - Suspected cause: environment-specific cleanup behavior in the MuJoCo viewer/OpenGL stack.
  - Fix or mitigation: for viewer runs only, flush output and exit immediately after the viewer completes to avoid late teardown finalizers.
  - Remaining risk: future richer viewer features should be checked on the demo machine.

### Result
The repo now has a visible command contract. Smoke, tests, maze generation, MuJoCo state runs, and view commands all create inspectable artifacts under `runs/visual/`, print paths, and open those artifacts when desktop support is available.

### Next actions
Continue to Milestone 3 with the same contract: maze-to-MuJoCo world building should include a generated world artifact, a still render, and a live viewer command.

## 2026-06-15 - Milestone 3: Maze-to-MuJoCo world builder

### Goal
Convert deterministic maze grids into MuJoCo world geometry, place the Unitree G1 at the generated maze start cell, and provide visible/debuggable artifacts without implementing navigation or control.

### Changes made
- Added `sim/world_builder.py` to generate validated maze worlds from a seed.
- Added `scripts/generate_world.py`.
- Added `robot.base_model_xml_path` and `robot.initial_base_height_m` to `configs/default.yaml`.
- Added `make world SEED=...` and `make view-world SEED=...`.
- Updated `scripts/run_episode.py` so `--world maze` is the default and `--world empty` remains available.
- Updated `make run`, `make view-run`, and `make view` to use generated maze worlds by default.
- Added generated-world tests for coordinate conversion, wall count, visual-only markers, keyframe placement, and MuJoCo XML loadability.
- Updated README and the milestone plan with Milestone 3 commands and the coordinate convention.

### Key decisions
- [WORLD_BUILDING][MAZE_VALIDITY] Decision: represent each wall cell as a full-cell MuJoCo box.
  - Why: this matches the occupancy grid directly and makes wall count, collision, and visual debugging straightforward.
  - Alternatives considered: thin wall strips, merged wall runs.
  - Tradeoff accepted: more geoms and blockier visuals now, but fewer conversion mistakes before navigation exists.
- [COORDINATES] Decision: use the maze center as the MuJoCo world origin.
  - Why: this keeps generated worlds centered around `(0, 0)` and makes default camera/statistic framing easier.
  - Alternatives considered: lower-left origin.
  - Tradeoff accepted: row-to-y conversion needs one explicit sign flip.
- [REPRODUCIBILITY] Decision: generate a full XML derived from `g1.xml` with an absolute mesh directory instead of a wrapper include.
  - Why: a generated wrapper under `runs/visual/` could not reliably resolve Unitree mesh paths.
  - Alternatives considered: save generated XML inside the Unitree submodule or use include wrappers with relative paths.
  - Tradeoff accepted: generated XML is larger, but remains self-contained enough to load from `runs/visual/`.
- [VISIBILITY] Decision: `make run` now defaults to `WORLD=maze`.
  - Why: the main command should show the current milestone behavior.
  - Alternatives considered: keep empty-world as default and add only `make world`.
  - Tradeoff accepted: Milestone 1 behavior moves to explicit `WORLD=empty`.

### Validation performed
- Commands run:
  - `make test`
  - `make world SEED=123`
  - `make view-world SEED=123`
  - `make run SEED=123 DURATION=3`
  - `make view-run SEED=123 DURATION=3`
  - `make view SEED=123 VIEW_DURATION=1`
  - `make run WORLD=empty SEED=1 DURATION=1`
- Tests passed/failed:
  - `make test` passed: 24 tests passed.
  - `make world SEED=123` generated a 128-wall MuJoCo world.
  - `make run SEED=123 DURATION=3` loaded `runs/visual/world_seed-123.xml`, stepped 1500 steps, and ended with no fall detected.
  - `make run WORLD=empty SEED=1 DURATION=1` preserved the original empty-scene fallback.
- Visual checks/screenshots/log files:
  - `runs/visual/world_seed-123.xml`
  - `runs/visual/world_seed-123_summary.json`
  - `runs/visual/world_seed-123_topdown.svg`
  - `runs/visual/run_seed-123_summary.json`
  - `runs/visual/run_seed-123_final.png`

### Problems encountered
- Problem: generated wrapper XML under `runs/visual/` could not resolve Unitree mesh paths when including `g1.xml`.
  - Symptom: MuJoCo reported missing STL files such as `pelvis.STL` or `left_hip_pitch_link.STL`.
  - Suspected cause: included XML mesh paths were resolved relative to the generated wrapper location or nested compiler context.
  - Fix or mitigation: parse `g1.xml`, set `compiler.meshdir` to the absolute Unitree asset directory, append maze geoms, and write a full generated XML.
  - Remaining risk: generated XML is larger than a wrapper file.
- Problem: default renderer framing could focus too tightly on the robot instead of the maze.
  - Symptom: the base G1 XML does not provide the broader scene framing from Menagerie's `scene.xml`.
  - Suspected cause: Milestone 3 builds from `g1.xml` to reposition the keyframe and append world geoms.
  - Fix or mitigation: add generated `statistic` and `visual/global` defaults to the world XML.
  - Remaining risk: final demo camera controls will need more deliberate design.

### Result
The repo can generate a seeded MuJoCo maze world, place the G1 at the start cell, save XML/SVG/JSON debug artifacts, load the generated world in MuJoCo, render a final scene frame, and open the artifacts through `view-*` commands. The robot still stands passively; navigation remains out of scope.

### Next actions
Start Milestone 4 by adding an explicit oracle planner over the generated maze grid, converting the path to world waypoints using the same coordinate convention, and visualizing the planned path.

## 2026-06-15 - Realtime run visibility update

### Goal
Make `make run` show the generated maze world in realtime, and make the first visual artifact a side-by-side view of the 2D maze representation next to the generated MuJoCo maze render.

### Changes made
- Added `scripts/write_run_dashboard.py` to create `runs/visual/run_seed-<seed>_dashboard.html`.
- Updated MuJoCo rendering to support dependency-free PNG output so browser dashboards can display rendered scenes directly.
- Changed `make run` to:
  - generate a short preview render,
  - write and open the side-by-side dashboard first,
  - launch the live MuJoCo passive viewer for realtime inspection.
- Changed `make view-run` to stay headless and open the side-by-side dashboard with the final rendered scene.
- Added tests for PNG writing and dashboard generation.

### Key decisions
- [VISIBILITY] Decision: `make run` is now the realtime viewing command.
  - Why: the primary run command should show the maze in MuJoCo, not only prove it by saved files.
  - Alternatives considered: keep realtime behavior only under `make view`.
  - Tradeoff accepted: `make run` is now interactive and blocks until the viewer closes or the requested duration ends.
- [VISIBILITY] Decision: use HTML dashboard plus PNG render for side-by-side inspection.
  - Why: browsers display SVG and PNG reliably, while PPM is not browser-friendly.
  - Alternatives considered: keep opening separate SVG and PPM windows.
  - Tradeoff accepted: PNG writing needs a small stdlib encoder, but avoids a plotting/image dependency.

### Validation performed
- Commands run:
  - `make test`
  - `make view-run SEED=123 DURATION=3`
  - `make run SEED=123 DURATION=1`
- Visual checks/screenshots/log files:
  - `runs/visual/run_seed-123_dashboard.html`
  - `runs/visual/run_seed-123_preview.png`
  - `runs/visual/run_seed-123_final.png`
  - `runs/visual/world_seed-123_topdown.svg`

### Problems encountered
- Problem: the existing PPM render was not suitable for an HTML side-by-side dashboard.
  - Symptom: browser dashboards cannot reliably display PPM images inline.
  - Suspected cause: PPM is useful for dependency-free file output but not web display.
  - Fix or mitigation: added dependency-free PNG writing for rendered MuJoCo frames.
  - Remaining risk: the live viewer is still for camera/world inspection, not robot teleoperation or autonomous navigation.

### Result
`make run` now opens the visual dashboard first and then launches the live MuJoCo viewer in the generated maze world. `make view-run` provides a non-realtime side-by-side dashboard for inspection.

### Next actions
Milestone 4 should add oracle path planning and overlay the planned path in both the 2D dashboard and MuJoCo world.

## 2026-06-16 - Sidequest: G1 locomotion policy sandbox

### Goal
Create a flat-ground visual Unitree G1 locomotion policy sandbox for teleop-driven walking-policy candidates, separate from the maze navigation milestones.

### Changes made
- Added `sim/locomotion_policy_adapter.py` with `placeholder`, `onnx`, and `external_python` adapter modes.
- Added `sim/locomotion_sandbox.py` with teleop command clipping, key mapping, command timeout, fall status checks, CSV logging, frame recording paths, dashboard writing, and final rendering helpers.
- Added `scripts/g1_loco_sandbox.py` as the live/headless sandbox entry point.
- Added `make g1-loco-sandbox POLICY=...` for the live MuJoCo viewer and `make g1-loco-view POLICY=...` for headless artifact/dashboard generation.
- Added alias targets `make locomotion-sandbox` and `make view-locomotion-sandbox`.
- Added `locomotion_sandbox` settings to `configs/default.yaml`.
- Added non-visual tests for command clipping, key mapping, command timeout, placeholder compatibility, report writing, dashboard references, and recording path creation.
- Updated README with command names, controls, policy modes, and artifact paths.

### Key decisions
- [SCOPE] Decision: keep the sandbox on the existing flat G1 Menagerie `scene.xml`.
  - Why: policy bring-up needs a safe flat world with floor, lighting, camera defaults, and G1, not the generated maze world.
  - Tradeoff accepted: maze start placement, planner state, and waypoint following stay out of this branch.
- [POLICY_BOUNDARY] Decision: placeholder mode holds the stand keyframe control target and marks `real_locomotion=false`.
  - Why: it validates viewer, teleop, logging, recording, and artifacts without pretending the robot can walk.
  - Tradeoff accepted: placeholder teleop changes desired commands only; it does not produce gait motion.
- [TELEOP] Decision: reserve `S` and `T` for stop-recording and use `Z` as fallback backward.
  - Why: this removes the ambiguous `S` conflict while keeping a keyboard-only fallback.
- [ONNX_SAFETY] Decision: ONNX policies require explicit metadata and strict dimension/name checks.
  - Why: applying a humanoid policy with guessed joint order or action scaling is unsafe and misleading.
  - Tradeoff accepted: many raw ONNX files will fail compatibility until a policy-specific metadata file is supplied.

### Validation performed
- Commands run:
  - `.venv/bin/python -m pytest tests/test_locomotion_sandbox.py`
  - `make g1-loco-view POLICY=placeholder G1_LOCO_DURATION=0.2`
  - `make g1-loco-sandbox POLICY=placeholder G1_LOCO_DURATION=1`
  - `make test`
- Tests passed/failed:
  - Targeted sandbox tests passed: 7 passed.
  - Full project tests passed: 33 passed.
  - The placeholder compatibility report marks `real_locomotion=false`.
  - Recording path creation and R/S key request handling are covered by non-visual tests; no real live recording was captured during this pass.
- Visual checks/screenshots/log files:
  - Live MuJoCo viewer opened for `make g1-loco-sandbox POLICY=placeholder G1_LOCO_DURATION=1`.
  - Latest artifacts created:
    - `runs/visual/g1_loco_latest_dashboard.html`
    - `runs/visual/g1_loco_latest_summary.json`
    - `runs/visual/g1_loco_latest_commands.csv`
    - `runs/visual/g1_loco_latest_state.csv`
    - `runs/visual/g1_loco_latest_final_render.png`
    - `runs/visual/g1_loco_latest_policy_compatibility.json`
  - Final live summary reported `viewer_opened=true`, `final_status=standing`, `fallen=false`, and `recording_used=false`.

### Problems encountered
- Problem: no real walking policy is included with the Unitree G1 MuJoCo model.
  - Fix or mitigation: added a clear placeholder mode and strict compatibility reports for future policy candidates.
  - Remaining risk: locomotion backend integration still depends on finding or adapting a real G1 walking policy.

### Result
The branch now has a dedicated locomotion-policy sandbox path that can validate policy loading, teleop commands, status display, logs, dashboards, final renders, and optional recording frames without modifying the maze pipeline.

### Next actions
Manually exercise live keypress teleop/recording in a longer viewer session, then test a real ONNX candidate with a metadata file that declares observation/action dimensions, actuator names, control rate, and action scaling.

## 2026-06-16 - Sidequest update: Lucky Robots walker policy integration

### Goal
Use the pretrained G1 walking policy from `luckyrobots/g1-manipulation-challenge` as the first real locomotion backend in the visual sandbox.

### Changes made
- Added `POLICY=lucky_walker` as a Lucky-specific adapter mode.
- Added `make fetch-lucky-g1-policy` to clone `https://github.com/luckyrobots/g1-manipulation-challenge.git` into ignored `third_party/g1-manipulation-challenge/`.
- Added `onnxruntime` to `requirements.txt` for real ONNX policy inference.
- Added `.gitignore` coverage for `third_party/` so upstream policy weights are not vendored into this repo.
- The Lucky adapter generates `third_party/g1-manipulation-challenge/flat_scene_locomotion_sandbox.xml` from the upstream G1 model, then runs only the 29-actuator body walker on flat ground.
- Added tests for the Lucky adapter alias, flat wrapper generation, and actionable missing-asset reports.
- Updated README with the Lucky walker fetch/run commands.

### Key decisions
- [POLICY_ASSETS] Decision: fetch the Lucky repo locally instead of committing `walker.onnx` and mesh assets.
  - Why: the upstream repo contains policy weights and assets but no obvious license file in the fetched checkout.
  - Tradeoff accepted: users must run `make fetch-lucky-g1-policy` before `POLICY=lucky_walker`.
- [MODEL_COMPATIBILITY] Decision: run the Lucky walker with its bundled G1 model on a generated flat scene.
  - Why: the policy was trained with a 0.005 s timestep, specific actuator gains, armature values, default pose, and action scales from `model_config.json`.
  - Tradeoff accepted: this sandbox backend uses the Lucky G1 XML rather than the Menagerie G1 XML, while still keeping the world flat and separate from maze navigation.
- [CONTROL_BOUNDARY] Decision: the Lucky walker adapter controls only the first 29 named body actuators.
  - Why: Lucky's full model includes additional hand actuators, but the walker output is 29D.
  - Tradeoff accepted: hands are present in the Lucky model but not controlled by the walking adapter.

### Validation performed
- Commands run:
  - `LD_LIBRARY_PATH="$HOME/.local/openssl/lib:$LD_LIBRARY_PATH" .venv/bin/python -m pip install 'onnxruntime>=1.17'`
  - `make fetch-lucky-g1-policy`
  - `make g1-loco-view POLICY=lucky_walker G1_LOCO_DURATION=0.2`
  - scripted 3 s rollout with `vx=0.25`
  - `make g1-loco-sandbox POLICY=lucky_walker G1_LOCO_DURATION=1`
- Tests passed/failed:
  - Lucky adapter tests passed: 3 passed.
  - Sandbox tests passed: 10 passed across Lucky and generic sandbox tests.
- Visual checks/screenshots/log files:
  - Live MuJoCo viewer opened for `POLICY=lucky_walker`.
  - Final live summary reported `viewer_opened=true`, `real_locomotion=true`, `final_status=standing`, `fallen=false`.
  - Scripted forward command moved the base about 0.057 m over 3 s and stayed upright.
  - Latest artifacts updated under `runs/visual/g1_loco_latest_*`.

### Problems encountered
- Problem: the initial flat-scene wrapper under `runs/visual/` caused MuJoCo mesh path resolution failures.
  - Symptom: MuJoCo looked for OBJ meshes beside the Lucky repo root instead of under `assets/`.
  - Fix or mitigation: generate the flat wrapper inside the Lucky repo root so its original `meshdir="assets"` resolves as upstream intended.
- Problem: `pip install onnxruntime` initially failed because the venv needs the local OpenSSL library path.
  - Fix or mitigation: install with `LD_LIBRARY_PATH="$HOME/.local/openssl/lib:$LD_LIBRARY_PATH"`, matching the Makefile command environment.

### Result
The sandbox now has a real walking policy path. `make g1-loco-sandbox POLICY=lucky_walker` opens a live viewer with the Lucky walker loaded, and teleop velocity commands are converted into G1 joint targets through the upstream 99D observation to 29D action pipeline.

### Next actions
Run a longer manual session with `POLICY=lucky_walker`, press/hold `W` or the up arrow to command forward velocity, test yaw keys, and record a short walking clip with `R` then `S` or `T`.

## 2026-06-16 - Sidequest update: Lucky walker teleop tuning

### Goal
Make live teleop commands large and persistent enough to produce visible walking with `POLICY=lucky_walker`.

### Changes made
- Added `command_step_fraction` to `locomotion_sandbox` config.
- Added Lucky-specific teleop defaults in `scripts/g1_loco_sandbox.py`.
- For `POLICY=lucky_walker`, command limits now rise to at least:
  - `max_forward_speed_mps=1.0`
  - `max_lateral_speed_mps=0.6`
  - `max_yaw_rate_radps=1.0`
  - `command_timeout_s=2.0`
  - `command_step_fraction=0.2`
- Updated README to tell users to tap `W` or up arrow several times to ramp visible speed.

### Problem encountered
- Problem: the viewer received keys, but each press only produced `vx=0.05` and the command timed out after `0.5s`.
  - Symptom: terminal showed brief `walking` status, but robot movement was not visually obvious.
  - Fix or mitigation: Lucky walker mode now uses upstream-sized velocity increments (`0.2` per tap) and a longer safety timeout (`2s`).

### Validation performed
- Commands run:
  - `.venv/bin/python -m pytest tests/test_locomotion_sandbox.py tests/test_lucky_walker_adapter.py`
  - `make test`
- Tests passed/failed:
  - Focused sandbox tests passed: 10 passed.
  - Full project tests passed: 36 passed.

### Result
Live Lucky walker teleop should now show visible motion after a few `W` or up-arrow taps, while `Space`/`X` still zeroes the command immediately and the timeout still returns to zero if input stops.

## 2026-06-16 - Sidequest update: Unitree RL Gym policy integration

### Goal
Add a walking-policy backend from the official `unitreerobotics/unitree_rl_gym` repo, prioritizing the regular Menagerie G1 model while keeping an official-native comparison path.

### Changes made
- Added `make fetch-unitree-rl-gym-policy` to clone `https://github.com/unitreerobotics/unitree_rl_gym.git` into ignored `third_party/unitree_rl_gym/`.
- Added `make install-torch-cpu` to install `torch==2.12.0+cpu` from the PyTorch CPU wheel index.
- Added `POLICY=unitree_rl_gym_g1`, an experimental adapter that loads Unitree's `deploy/pre_train/g1/motion.pt` and maps its 12 leg actions onto the regular Menagerie G1 model.
- Added `POLICY=unitree_rl_gym_native`, a comparison adapter that loads Unitree's own 12-DoF G1 Mujoco XML.
- Added Torch `LD_PRELOAD` handling for `libgomp.so.1` and `libc10.so` in the locomotion Make targets to avoid static TLS load errors on this platform.
- Added tests for Unitree RL Gym adapter routing and missing-asset reporting.
- Updated README with the Unitree commands and caveats.

### Key decisions
- [POLICY_SOURCE] Decision: keep Unitree RL Gym as an ignored third-party checkout.
  - Why: it is BSD-3 licensed, but keeping fetched policy/model assets out of this repo avoids unrelated vendoring churn.
- [MODEL_COMPATIBILITY] Decision: expose both regular-model and native-model modes.
  - Why: the user's main goal is the regular Menagerie G1, but Unitree's policy was trained/deployed with a 12-DoF torque XML.
  - Tradeoff accepted: `unitree_rl_gym_g1` is explicitly experimental; `unitree_rl_gym_native` is the closer official reference.
- [TORCH_INSTALL] Decision: do not add generic `torch` to `requirements.txt`.
  - Why: plain PyPI Torch attempted to install a large CUDA-enabled dependency set on this aarch64 system.
  - Fix or mitigation: add `make install-torch-cpu` with the CPU wheel index.

### Validation performed
- Commands run:
  - `make fetch-unitree-rl-gym-policy`
  - `make install-torch-cpu`
  - `make g1-loco-view POLICY=unitree_rl_gym_g1 G1_LOCO_DURATION=0.2`
  - `make g1-loco-view POLICY=unitree_rl_gym_native G1_LOCO_DURATION=0.2`
  - scripted 3 s forward rollouts with `vx=0.2` and `vx=0.5`
  - `make g1-loco-sandbox POLICY=unitree_rl_gym_g1 G1_LOCO_DURATION=1`
  - focused adapter/sandbox pytest suite
- Tests passed/failed:
  - Focused adapter/sandbox tests passed: 13 passed.
- Visual checks/screenshots/log files:
  - Live MuJoCo viewer opened for `POLICY=unitree_rl_gym_g1`.
  - Latest artifacts updated under `runs/visual/g1_loco_latest_*`.
  - Scripted regular Menagerie bridge rollouts stayed upright for 3 s:
    - `vx=0.2`: delta about `[0.55, 0.47, 0.02]`
    - `vx=0.5`: delta about `[0.85, -0.02, 0.02]`
  - Scripted native Unitree XML rollouts stayed upright for 3 s:
    - `vx=0.2`: delta about `[0.54, -0.07, -0.01]`
    - `vx=0.5`: delta about `[1.27, -0.07, -0.02]`

### Problems encountered
- Problem: installing `torch>=2.2` from the default PyPI index began pulling a large CUDA-enabled wheel stack.
  - Fix or mitigation: cancelled it and installed `torch==2.12.0+cpu` from `https://download.pytorch.org/whl/cpu`.
- Problem: Torch initially failed to load after MuJoCo with static TLS errors for `libgomp.so.1` and `libc10.so`.
  - Fix or mitigation: preload those Torch shared libraries in the g1-loco Make targets when they are present.
- Problem: the first regular-model scripted rollout fell when PD control was only refreshed at the outer 50 Hz loop.
  - Fix or mitigation: mark Unitree adapters as requiring substep control so PD targets/torques update every MuJoCo step while policy actions update every 10 substeps.

### Result
The sandbox now supports an official Unitree RL Gym policy path. Use `POLICY=unitree_rl_gym_g1` for the regular Menagerie G1 bridge and `POLICY=unitree_rl_gym_native` for Unitree's own 12-DoF XML reference.

### Next actions
Run a longer manual viewer test with `POLICY=unitree_rl_gym_g1`, compare it against `POLICY=unitree_rl_gym_native`, and decide whether the regular-model bridge is stable enough for future maze locomotion or whether maze integration should target a Unitree-derived G1 XML.

## 2026-06-17 - Milestone 4: Lucky G1 oracle walking

### Goal
Make Lucky Robot's G1 model and `walker.onnx` policy the default branch robot stack, then replace the orange waypoint proxy with a physically simulated Lucky G1 walker in explicit oracle mode.

### Changes made
- Switched `configs/default.yaml` to default to `third_party/g1-manipulation-challenge/scene.xml` and `g1.xml`, while keeping the Menagerie paths as legacy fallback references.
- Added `nav/planner.py` for oracle BFS planning, obstacle-inflation handling, waypoint conversion, and path simplification.
- Added `nav/controller.py` with a conservative rotate-then-walk waypoint follower that outputs `VelocityCommand`.
- Extended the Lucky walker adapter with `reset_at_pose(x, y, yaw)` so maze runs can initialize the policy at the maze start instead of the flat-sandbox origin.
- Added `scripts/run_milestone_4.py` to build a Lucky-based maze world, add oracle path markers, load `walker.onnx`, follow waypoints with MuJoCo ground-truth pose, and write trajectory/final-render/dashboard artifacts.
- Added `make milestone_4` for live viewer runs and `make view-milestone_4` for headless dashboard runs.
- Made default world/run/test targets fetch or refresh the ignored Lucky checkout before loading the default model.

### Key decisions
- [LOCOMOTION][MODEL_COMPATIBILITY] Decision: Lucky is the default runtime model/policy pair for this branch.
  - Why: the Lucky walker policy and Lucky XML match each other and produce visible walking, unlike the experimental Unitree policy bridge on Menagerie.
  - Tradeoff accepted: the branch default model changes, but Menagerie remains available as an explicit legacy path.
- [GROUND_TRUTH_BOUNDARY][PLANNING] Decision: Milestone 4 uses oracle grid planning and MuJoCo ground-truth base pose.
  - Why: this isolates locomotion/path-following before ROS 2, Nav2, SLAM, and sensor estimation are introduced.
  - Tradeoff accepted: this is not sensor-based autonomy and must be labeled oracle/debug.
- [DEMO_RISK] Decision: do not silently fall back to a proxy.
  - Why: the milestone should prove whether the Lucky model is actually walking.
  - Tradeoff accepted: long runs may honestly report `FALL_DETECTED`, `STUCK`, or `TIMEOUT` instead of always showing a successful animation.

### Validation performed
- Commands run:
  - `make view-milestone_4 SEED=123 MILESTONE_4_DURATION=1`
  - `rg -n "proxy_waypoint_body|mocap=\"true\"|orange" runs/visual/milestone_4_seed-123_world.xml runs/visual/milestone_4_seed-123_summary.json || true`
  - `make test`
- Tests passed/failed:
  - Full project tests passed: 48 passed.
- Visual/artifact checks:
  - `make view-milestone_4 SEED=123 MILESTONE_4_DURATION=1` wrote the milestone dashboard, world XML, path SVG, trajectory CSV, final render, and compatibility report.
  - The short run moved the Lucky G1 from the maze start, reached waypoint index 1/2 during the first second, stayed upright, and reported `final_status=RUNNING`.
  - The generated Milestone 4 world XML does not contain `proxy_waypoint_body`, `mocap="true"`, or `orange`.

### Current limitations
- The short validation proves integration and initial walking, not full maze completion.
- The controller is intentionally conservative and may time out or report stuck/fallen on longer seeds if the walker cannot negotiate turns/corridors reliably.
- Oracle mode still uses generated-grid planning and MuJoCo truth pose; ROS 2/Nav2/SLAM integration remains future work.

### Next actions
Run a longer live `make milestone_4 SEED=123` inspection, tune waypoint tolerances and speed/yaw limits if needed, then start the sensor/timebase milestone for ROS-facing navigation work.

## 2026-06-17 - Milestone 4 fix: visible Lucky oracle walking speed

### Goal
Fix the first Milestone 4 live/headless runs where the controller sent commands but the Lucky G1 barely moved and then reported `STUCK`.

### Problem encountered
- Problem: `make milestone_4 SEED=123` and `make view-milestone_4 SEED=123` appeared not to move the robot.
  - Evidence: `runs/visual/milestone_4_seed-123_trajectory.csv` showed `vx=0.25`, but the base only moved about 5.5 cm before `final_status=STUCK`.
  - Suspected cause: the oracle controller reused the older conservative Menagerie/proxy speed. Lucky's policy needs a larger command; the upstream controller allows linear commands up to `2.0`.

### Fix
- Increased `oracle.forward_speed_mps` from `0.25` to `0.8`.
- Added a config regression test to keep the default Lucky oracle speed in the visible-walking range.

### Validation performed
- Commands run:
  - Diagnostic Lucky command sweep at the maze start with `vx` values from `0.25` to `1.5`.
  - `make view-milestone_4 SEED=123 MILESTONE_4_DURATION=20`
  - `make view-milestone_4 SEED=123 MILESTONE_4_DURATION=60`
  - `make test`
- Result:
  - `vx=0.25` moved only about `0.057 m` in 4 simulated seconds.
  - `vx=0.8` moved about `2.37 m` in 4 simulated seconds while staying upright.
  - The 20-second Milestone 4 run advanced to waypoint 5, stayed upright, and ended with `final_status=RUNNING`.
  - A later 30-second run showed a false `STUCK` at waypoint index 4 because the stuck timer counted time spent intentionally rotating in place.
  - Fixed the stuck detector to reset its timer while `vx=0`; after that, the 60-second run reached waypoint index 5/6 and kept `final_status=RUNNING`.

## 2026-06-17 - Milestone 4 collision diagnostics and wide maze option

### Goal
Check whether the Lucky G1 is getting close enough to maze walls to collide during oracle walking, and provide an easier/wider maze option for locomotion debugging.

### Changes made
- Added MuJoCo contact diagnostics to `scripts/run_milestone_4.py`.
- Milestone 4 trajectory CSV now includes:
  - `contact_count`
  - `wall_contact_count`
  - `wall_contact_pairs`
- Milestone 4 summary JSON now includes `contact_summary` with first/last wall contact time, wall/robot geom pairs, max contact counts, and sampled steps with wall contacts.
- Added `configs/lucky_wide_maze.yaml`, which keeps the same 15x15 maze topology but increases `maze.cell_size_m` from `1.0` to `1.5`.
- Added `make milestone_4-wide` and `make view-milestone_4-wide`; wide artifacts use the `milestone_4_wide_seed-*` prefix so default and wide runs can be compared side by side.

### Findings
- Default/narrow run:
  - Command: `make view-milestone_4 SEED=123 MILESTONE_4_DURATION=35`
  - Result: `steps_with_wall_contacts=6`
  - First wall contact: `t=26.92s`
  - Contact pair: `maze_wall_0_3<->geom_221:left_hand_index_1_link`
- Wide run:
  - Command: `make view-milestone_4-wide SEED=123 MILESTONE_4_DURATION=20`
  - Result: `steps_with_wall_contacts=0`

### Conclusion
The user hypothesis was correct for the default 1 m corridor maze: the humanoid can brush maze walls, specifically with an arm/hand link while navigating a tight turn near the top row. The wide 1.5 m corridor config avoids wall contact in the checked run and is a better default for locomotion tuning.

## 2026-06-17 - Milestone 4 arc turns and corridor-width command control

### Goal
Fix the Lucky walker getting stuck on pure rotate-in-place turns, and expose corridor width as a per-command tuning knob.

### Changes made
- Changed the oracle waypoint follower from stop-and-turn to arc-turn behavior.
  - When heading error is large, the controller now commands a forward crawl plus yaw instead of `vx=0`.
  - Added `oracle.arc_turn_speed_mps`, defaulting to `0.4`.
- Changed the default maze corridor width from `1.0 m` to `1.6 m`.
- Changed the wide maze config and wide Make targets to use `2.0 m`.
- Added `CORRIDOR_WIDTH_M=1.0..2.0` for `make milestone_4` and `make view-milestone_4`.
  - Example narrow run: `make view-milestone_4 SEED=123 CORRIDOR_WIDTH_M=1.0 MILESTONE_4_LABEL=narrow`
  - Example default run: `make view-milestone_4 SEED=123 CORRIDOR_WIDTH_M=1.6`
  - Example wide run: `make view-milestone_4 SEED=123 CORRIDOR_WIDTH_M=2.0 MILESTONE_4_LABEL=wide`
- Updated stuck detection to track route progress by waypoint index/current-waypoint distance instead of straight-line distance to the final goal.

### Why
Diagnostic sweeps showed the Lucky policy barely changes yaw with a pure turn command, especially on the right-turn segment after the second turn. The same policy yaws much more effectively when given a small forward velocity during the turn, so the controller now follows that policy preference instead of fighting it.

Straight-line goal distance is also a poor stuck signal inside a maze: a valid route can temporarily move away from the final goal. The detector now measures local waypoint-route progress.

## 2026-06-17 - Turn-aware G1 oracle follower

### Goal
Add a visual oracle-following runner that starts turns before corners, commands arc turns with forward velocity, and logs controller state/recovery behavior for the Lucky G1 walking policy.

### Changes made
- Added heading-aware A* support to the oracle planner with configurable turn penalty.
- Added a turn-aware path postprocessor and controller state machine:
  - `FOLLOW_STRAIGHT`
  - `PRE_TURN_SLOWDOWN`
  - `ARC_TURN`
  - `POST_TURN_REALIGN`
  - `RECOVERY`
  - `GOAL_REACHED`
  - `FAILED`
- Added conservative stuck recovery with bounded attempts.
- Added `scripts/run_g1_oracle_follow.py`.
- Added Make targets:
  - `make g1-oracle-follow SEED=123`
  - `make view-g1-oracle-follow SEED=123`
- Added required artifacts:
  - dashboard HTML
  - final MuJoCo render
  - topdown path overlay SVG
  - trajectory CSV
  - commands CSV
  - events JSONL
  - summary JSON
- Added oracle config defaults for turn start distance, arc turn speed/yaw, post-turn heading tolerance, stuck detection, recovery bounds, and turn penalty.

### Validation performed
- Commands run:
  - `.venv/bin/python -m py_compile nav/planner.py nav/oracle_follow.py scripts/run_g1_oracle_follow.py`
  - `.venv/bin/python -m pytest tests/test_oracle_follow.py tests/test_planner.py tests/test_config.py`
  - `make view-g1-oracle-follow SEED=123 ORACLE_FOLLOW_DURATION=5 ORACLE_FOLLOW_LABEL=smoke`
  - `make test`
  - `make view-g1-oracle-follow SEED=5 ORACLE_FOLLOW_DURATION=18 CORRIDOR_WIDTH_M=2.0 ORACLE_FOLLOW_LABEL=rightturn`
- Results:
  - Full project tests passed: 58 passed.
  - The smoke run wrote all required artifacts under `runs/visual/g1_oracle_follow_smoke_seed-123_*`.
  - The event log showed `FOLLOW_STRAIGHT`, `PRE_TURN_SLOWDOWN`, and `ARC_TURN`, including a logged turn-start event.
  - The short run timed out by duration, as expected, and reported no wall contacts.
  - Seed 5 reached a known right turn; the event log recorded `turn_direction=right` and the command CSV showed `vx=0.4`, `yaw_rate=-0.8` during `ARC_TURN`.
  - The seed 5 right-turn run reached `POST_TURN_REALIGN`, timed out only because of the 18 second cap, and reported `steps_with_wall_contacts=0`.

## 2026-06-17 - Docker development environment for ROS 2 Humble

### Goal
Create a portable Docker development environment that keeps ROS 2 Humble, Nav2, SLAM, RViz, and MuJoCo system dependencies inside the image instead of requiring ROS on the host.

### Changes made
- Added `docker/Dockerfile` based on the multi-architecture official ROS Humble Jammy base image, with desktop/navigation packages installed explicitly.
- Added Docker runtime scripts:
  - `docker/entrypoint.sh`
  - `docker/run.sh`
  - `docker/run_gui.sh`
  - `docker/build_multiarch.sh`
- Added `.dockerignore` to keep generated artifacts and caches out of Docker build context while preserving `third_party/` and `assets/` model/policy assets.
- Added `scripts/check_ros_docker_env.sh` to verify ROS 2 Humble, Nav2, SLAM, depth conversion, localization, RViz, and workspace availability inside Docker.
- Added Make targets:
  - `make docker-build`
  - `make docker-run`
  - `make docker-run-gui`
  - `make docker-test`
  - `make docker-smoke`
  - `make docker-check-ros`
  - `make docker-milestone_4`
  - `make docker-milestone_5`
  - `make docker-build-multiarch`
- Added README Docker quick-start documentation.
- Added lightweight tests for Docker support files, executable bits, `.dockerignore`, README docs, and Make targets.
- Added `VENV_PYTHON` Makefile support so normal host commands still use `.venv/bin/python`, while Docker wrappers can export `VENV=/usr` and run the same Make targets with `/usr/bin/python3`.
- Pinned `pytest>=8.0,<9.0` because ROS 2 Humble's `launch_testing` pytest plugin is not compatible with pytest 9.

### Key decisions
- [REPRODUCIBILITY][SENSORS] Decision: use `ros:humble-ros-base-jammy` plus explicit ROS desktop/navigation packages.
  - Why: `osrf/ros:humble-desktop` failed on the ARM64 Docker host with `exec /bin/bash: exec format error`; manifest inspection showed it was not multi-architecture here, while `ros:humble-ros-base-jammy` supports `linux/amd64` and `linux/arm64/v8`.
  - Alternatives considered: native host ROS install, plain Ubuntu base image, or forcing amd64 emulation.
  - Tradeoff accepted: slightly more explicit apt package list, but the image can build on x86 and ARM/Jetson-class machines.
- [REPRODUCIBILITY][TRADEOFF] Decision: bind-mount the repo into `/workspace` for normal development.
  - Why: source edits should appear inside Docker immediately without rebuilding.
  - Tradeoff accepted: image copies the repo for reproducible builds, but runtime development uses the host checkout.
- [DEMO_RISK] Decision: split headless and GUI run scripts.
  - Why: CI/headless development should not depend on X11, while RViz/MuJoCo viewer work still needs a GUI path.

### Validation performed
- Commands run:
  - `make docker-build`
  - `make docker-check-ros`
  - `make docker-smoke`
  - `make docker-test`
  - `make test`
- Result:
  - First build attempt using `osrf/ros:humble-desktop` failed on this ARM64 Docker host with `exec /bin/bash: exec format error`.
  - Patched the Dockerfile to use the multi-architecture official ROS Humble Jammy base image and install desktop/navigation packages explicitly.
  - Rebuilt image successfully as `robotics-maze-g1:humble`.
  - `make docker-check-ros` passed inside Docker and verified ROS 2 Humble, `ros2`, `rviz2`, `colcon`, Nav2, SLAM Toolbox, depth image conversion, robot localization, image transport, CV bridge, vision messages, and `/workspace`.
  - `make docker-smoke` passed inside Docker and wrote `runs/visual/smoke_latest.html`.
  - `make docker-test` passed inside Docker: 63 tests passed with Python 3.10.12 and pytest 8.4.2.
  - `make test` passed on the host: 63 tests passed with the repo `.venv`.
  - Intermediate Docker test run failed with pytest 9/plugin incompatibility, which is why `requirements.txt` now pins pytest below 9.
- Tests/checks added:
  - `tests/test_docker_support.py`
- Current limitations:
  - GPU acceleration is not part of the base image.
  - GUI support uses Linux X11 forwarding and depends on the host display/session.
  - Multi-architecture manifests normally require `PUSH=1` and a registry image.

### Next actions
If a desktop display is available, run `make docker-run-gui` and test `rviz2` or `make milestone_5 SEED=123` from inside the container.

## 2026-06-19 - Separate Milestone 4 planning from Milestone 5 execution

### Goal
Keep the milestone commands aligned with the assignment plan: Milestone 4 produces an oracle path, while Milestone 5 executes that path with the robot.

### Changes made
- Added `scripts/run_milestone_4_planner.py` for planner-only SVG, ASCII, and JSON artifacts.
- Changed `make milestone_4` and its view/wide variants so they never launch MuJoCo robot execution.
- Added `make milestone_5`, `make view-milestone_5`, and `make docker-milestone_5` for the turn-aware Lucky G1 oracle follower.
- Preserved `g1-oracle-follow` commands as compatibility aliases for Milestone 5.
- Updated README and milestone-plan language, commands, and completion checklist.
- Added regression tests for the M4/M5 command boundary.

### Decision
- [REPRODUCIBILITY][DEMO_RISK] Milestone 5 accepts explicit oracle/debug execution as its completion baseline.
  - Milestone 4 uses the generated maze grid to plan and convert waypoints only.
  - Milestone 5 uses those oracle concepts plus MuJoCo ground-truth pose to command the real Lucky G1 policy.
  - This remains clearly distinct from later sensor-based autonomous navigation.

### Validation performed
- `make milestone_4 SEED=123` produced a 61-cell path without loading robot execution.
- The short `view-milestone_5` command check loaded the Lucky G1 policy, executed simulation commands, wrote all expected artifacts, and reported `milestone: 5` plus `mode: milestone_5_oracle_path_execution`.
- Focused milestone and Docker-support tests passed.

### Viewer command correction
- A user run showed `view-milestone_5` and `view-g1-oracle-follow` reporting `viewer_requested: false`; only the HTML dashboard opened.
- Changed both `view-*` commands to launch the live MuJoCo simulator.
- Moved the previous headless/dashboard behavior to `report-milestone_5` and `report-g1-oracle-follow` so the command names state what will appear.

## 2026-06-19 - External-storage guardrails and migration helper

### Goal
Prevent Docker 29/containerd and generated project data from silently filling the small host root partition.

### Changes made
- Added an ignored `.env.storage` policy with a portable committed example.
- Added `make storage-check` before large-output, dependency-fetch, test, Docker-build, and milestone targets.
- Added a container-aware check that requires `/workspace` to be bind-mounted and a host check that validates the configured mount, UUID, Docker root, and containerd root.
- Added a privileged migration helper that backs up system configuration, removes only conservative disposable data, quarantines old Docker/containerd stores externally, configures persistent storage, and verifies the restarted services.

### Current system finding
- Docker `data-root` already points externally, but Docker Engine 29 uses the system containerd snapshotter and `/etc/containerd/config.toml` does not set an external root.
- The external filesystem is mounted now, but its fstab entry is commented out.
- The administrative migration requires the user's sudo authentication and therefore cannot be run non-interactively by Codex.

## 2026-06-19 - Milestone 5 waypoint capture and recovery correction

### Evidence
- Seed 1 passed within `0.702 m` of the first arc waypoint before wall contact, but the old `0.35 m` capture radius kept commanding the left turn.
- Seed 123 passed within `0.04 m` of its post-turn waypoint, but the old completion rule rejected it because heading error was still above `0.3 rad`; wall contact began later.
- Both runs entered recovery twice within `0.02 s` because the next controller update overwrote `RECOVERY` with the segment state before executing the timed recovery phases.

### Changes made
- Kept turn-entry approach capture precise at `0.35 m`, while increasing arc-exit and post-turn capture to `0.75 m` for default and wide configurations.
- Kept final-goal capture separate and strict at `0.5 m`.
- Removed the heading gate from intermediate point capture so the following segment can continue correcting orientation.
- Removed projection-only segment completion, which could accept a robot that was far laterally from the point.
- Fixed recovery-state ordering so stop, reverse, and recovery-arc phases run for their configured durations.
- Reset the bounded recovery budget after segment completion so unrelated later obstacles receive their own recovery attempts.
- Added regression tests for broad intermediate capture, strict final capture, and persistent timed recovery.
