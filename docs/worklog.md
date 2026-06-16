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

## 2026-06-16 - Milestone 4: Oracle planner baseline

### Goal
Add an explicit oracle/debug planner that uses the generated maze grid to compute a path and world-coordinate waypoints. This is not sensor-based autonomy and does not command the robot.

### Changes made
- Added `nav/planner.py` with 4-connected A* using Manhattan distance.
- Added obstacle inflation based on robot safety radius and maze cell size.
- Added waypoint conversion using the same cell-to-world convention as `sim/world_builder.py`.
- Added `scripts/plan_path.py`.
- Added `make plan SEED=...`, `make view-plan SEED=...`, and `make milestone_4 SEED=...`.
- Added tests for A*, blocked grids, inflation behavior, generated-maze planning, waypoint conversion, and wall-cell rejection.
- Updated README and the milestone plan with Milestone 4 commands and oracle/debug boundaries.

### Key decisions
- [PLANNING][GROUND_TRUTH_BOUNDARY] Decision: label the planner as `mode=oracle`.
  - Why: it uses the generated maze grid directly and should not be mistaken for sensor-based autonomy.
  - Alternatives considered: expose it as the main navigation planner.
  - Tradeoff accepted: it is useful for debugging and future control work, but not a final autonomy claim.
- [PLANNING] Decision: use A* with Manhattan distance and 4-connected motion.
  - Why: it matches grid corridors, avoids diagonal corner cutting, and is easy to explain.
  - Alternatives considered: BFS, diagonal A*, continuous planning.
  - Tradeoff accepted: paths are grid-aligned rather than smooth.
- [MAZE_VALIDITY] Decision: inflate obstacles by continuous clearance from wall-cell boxes rather than simply blocking adjacent cells.
  - Why: with 1.0 m cells and a 0.45 m robot safety radius, naive one-cell inflation would erase valid corridors.
  - Alternatives considered: integer-cell dilation.
  - Tradeoff accepted: the inflation model is still grid-level and conservative, but it respects the configured corridor width.

### Validation performed
- Commands run:
  - `make test`
  - `make view-plan SEED=123 MAZE_CELL_PX=48`
- Tests passed/failed:
  - `make test` passed: 33 tests passed.
  - `make view-plan SEED=123 MAZE_CELL_PX=48` produced a 61-cell oracle path and opened the SVG artifact.
- Visual checks/screenshots/log files:
  - `runs/visual/plan_seed-123_oracle.svg`
  - `runs/visual/plan_seed-123_oracle.json`
  - `runs/visual/test_latest.html`

### Problems encountered
- Problem: obstacle inflation could easily over-block the generated perfect mazes.
  - Symptom: a simple cell-radius inflation would block one-cell corridors even though `cell_size_m=1.0` and `safety_radius_m=0.45` leaves 0.5 m from corridor center to wall-cell boundary.
  - Suspected cause: integer grid dilation ignores physical cell dimensions and wall extents.
  - Fix or mitigation: compute clearance from each free cell center to wall-cell boxes and block only when clearance is less than the configured safety radius.
  - Remaining risk: this is still a grid approximation; continuous robot body geometry and real gait clearance are future controller/world-validation concerns.

### Result
The repo now has an oracle/debug planner that finds A* paths on validated generated mazes, converts path cells to MuJoCo world-coordinate waypoints, saves JSON/SVG plan artifacts, and opens the path visualization through `make view-plan`.

### Next actions
Start Milestone 5 by adding a conservative waypoint follower and robot command interface. Keep oracle waypoints as a debug input, not a final autonomy claim.

## 2026-06-16 - Milestone 5: Waypoint follower and robot command interface

### Goal
Add conservative waypoint-following control math and high-level velocity-command plumbing without pretending the Unitree G1 can walk yet. Validate the follower in a clearly named point-robot debug mode.

### Changes made
- Added `nav/controller.py` with `Pose2D`, `VelocityCommand`, `WaypointFollowerConfig`, `ControlOutput`, and `WaypointFollower`.
- Implemented rotate-before-walk behavior, heading wrapping, waypoint switching, command clipping, goal tolerance, and point-robot integration.
- Updated `sim/robot_interface.py` to accept high-level velocity commands and report the current `unitree_g1_velocity_placeholder` adapter result.
- Added `GroundTruthPoseProvider`, restricted to `MODE=oracle` debugging.
- Added `scripts/follow_waypoints.py` to simulate the waypoint follower with a point-robot proxy over oracle waypoints.
- Added `make follow`, `make view-follow`, and `make milestone_5`.
- Added controller and robot-interface tests for heading error, waypoint switching, command clipping, goal reached, point integration, velocity-command acceptance, and oracle-only ground-truth pose access.
- Updated README and the milestone plan with Milestone 5 commands, artifacts, and limitations.

### Key decisions
- [LOCOMOTION][DEMO_RISK] Decision: do not claim G1 walking yet.
  - Why: the current Unitree model exposes joint actuators, not a ready high-level walking velocity API.
  - Alternatives considered: map velocity directly to joint controls or fake base motion in MuJoCo.
  - Tradeoff accepted: the controller can be validated now, while real humanoid locomotion remains a separate adapter problem.
- [CONTROL] Decision: use rotate-before-walk with clipped `vx`, optional `vy`, and `wz`.
  - Why: this is conservative, explainable, and reduces corner-cutting risk before physical walking exists.
  - Alternatives considered: pure pursuit or continuous holonomic tracking.
  - Tradeoff accepted: slower and less smooth motion, but easier to debug.
- [GROUND_TRUTH_BOUNDARY] Decision: add `GroundTruthPoseProvider` only for `MODE=oracle`.
  - Why: controller math needs a pose source for debugging, but autonomous mode must later use estimated pose.
  - Alternatives considered: expose ground truth through the main robot interface.
  - Tradeoff accepted: extra boundary code now prevents accidental autonomy claims later.

### Validation performed
- Commands run:
  - `make test`
  - `make view-follow SEED=123 MAZE_CELL_PX=48`
  - `make milestone_5 SEED=123 MAZE_CELL_PX=48`
- Tests passed/failed:
  - `make test` passed: 41 tests passed.
  - `make view-follow SEED=123 MAZE_CELL_PX=48` reached the goal region in point-robot debug mode.
  - `make milestone_5 SEED=123 MAZE_CELL_PX=48` passed tests and opened the follow SVG artifact.
- Visual checks/screenshots/log files:
  - `runs/visual/follow_seed-123_point.svg`
  - `runs/visual/follow_seed-123_point.json`
  - `runs/visual/test_latest.html`

### Problems encountered
- Problem: direct humanoid velocity control is not available yet.
  - Symptom: `RobotInterface` can read MuJoCo state, but no tested walking adapter maps `vx/vy/wz` to stable G1 joint actuation.
  - Suspected cause: Unitree G1 Menagerie provides a model and actuators, not a high-level locomotion policy.
  - Fix or mitigation: implemented explicit high-level command plumbing plus a point-robot debug adapter for controller validation.
  - Remaining risk: Milestone 6 or a dedicated locomotion milestone must identify or implement a stable G1 walking controller before the robot can physically follow waypoints.
- Problem: default speed is conservative and generated path length is long.
  - Symptom: seed 123 point-robot follow took about 262 simulated seconds to reach the goal region.
  - Suspected cause: `max_forward_speed_mps=0.25` and a 61-waypoint grid path.
  - Fix or mitigation: keep conservative defaults for safety and record elapsed time in the follow artifact.
  - Remaining risk: future waypoint smoothing or controller tuning may be needed for demo timing.

### Result
The repo now has a conservative waypoint follower, high-level velocity command interface, oracle-only ground-truth pose provider, and a visible point-robot waypoint-following artifact. The G1 still does not walk under these commands.

### Next actions
Start the next milestone by adding sensor/timebase infrastructure, or insert a dedicated locomotion investigation milestone to find a stable G1 velocity-control adapter before attempting physical waypoint following in MuJoCo.

## 2026-06-16 - Milestone 5.1: Visual MuJoCo waypoint follower inspection

### Goal
Correct Milestone 5 so waypoint following is visible inside MuJoCo, not only as an SVG trajectory. Use a moving proxy body because real G1 locomotion is not implemented.

### Changes made
- Added `sim/proxy_robot.py` to augment generated MuJoCo worlds with a visible mocap-controlled proxy body and oracle path markers.
- Added `scripts/sim_follow_waypoints.py` to generate the maze/world/path, move the proxy through MuJoCo using the waypoint follower, save artifacts, and optionally open the live viewer.
- Added `make sim-follow SEED=...` for live MuJoCo proxy-follow inspection.
- Added `make view-sim-follow SEED=...` for headless proxy-follow rendering and dashboard inspection.
- Added tests for proxy XML generation, yaw quaternion conversion, and sim-follow dashboard content.
- Updated README and the milestone plan with the Milestone 5.1 correction and required artifacts.

### Key decisions
- [LOCOMOTION][VISIBILITY] Decision: use a MuJoCo mocap proxy body as the debug locomotion backend.
  - Why: the G1 model does not currently have a tested high-level walking controller, but the waypoint follower must be visible in MuJoCo.
  - Alternatives considered: continue with SVG-only point robot, fake moving the G1 base, or attempt unvalidated joint control.
  - Tradeoff accepted: the moving body is a proxy, not the humanoid, but it validates the maze/world/planner/controller/visual pipeline honestly.
- [GROUND_TRUTH_BOUNDARY] Decision: keep the mode named `proxy_waypoint_follow`.
  - Why: this prevents confusion with autonomous navigation or real G1 locomotion.
  - Alternatives considered: call it waypoint following generically.
  - Tradeoff accepted: more explicit wording, fewer accidental claims.
- [VISIBILITY] Decision: save a dashboard plus MuJoCo final render and trajectory CSV.
  - Why: the user needs to inspect what happened even if the live viewer or display stack is unavailable.
  - Alternatives considered: live viewer only.
  - Tradeoff accepted: more artifacts, but the run is auditable.

### Validation performed
- Commands run:
  - `make test`
  - `make view-sim-follow SEED=123 MAZE_CELL_PX=48`
  - `make sim-follow SEED=123 MAZE_CELL_PX=48 SIM_FOLLOW_TIME_SCALE=256 SIM_FOLLOW_FRAME_STRIDE=128 SIM_FOLLOW_HOLD_S=0`
- Tests passed/failed:
  - `make test` passed after adding proxy/dashboard tests.
  - `make view-sim-follow SEED=123 MAZE_CELL_PX=48` completed with `final_status=GOAL_REACHED` and opened the dashboard.
  - `make sim-follow ...` opened the live MuJoCo viewer path and completed with `final_status=GOAL_REACHED`.
- Visual checks/screenshots/log files:
  - `runs/visual/sim_follow_seed-123_world.xml`
  - `runs/visual/sim_follow_seed-123_topdown.svg`
  - `runs/visual/sim_follow_seed-123_path.svg`
  - `runs/visual/sim_follow_seed-123_final.png`
  - `runs/visual/sim_follow_seed-123_dashboard.html`
  - `runs/visual/sim_follow_seed-123_summary.json`
  - `runs/visual/sim_follow_seed-123_trajectory.csv`

### Problems encountered
- Problem: syncing the live MuJoCo viewer every controller tick was too slow.
  - Symptom: the first live validation run took too long even though the proxy eventually reached the goal.
  - Suspected cause: `viewer.sync()` is expensive across roughly 13k controller steps.
  - Fix or mitigation: added `SIM_FOLLOW_FRAME_STRIDE` so simulation still updates every control tick but viewer syncs at display intervals.
  - Remaining risk: playback speed is a visualization setting, not physical time.
- Problem: the live viewer handle can close before completion if the user closes it or the display stack ends the window.
  - Symptom: an early strict failure reported that the viewer closed before the proxy finished.
  - Suspected cause: live viewer lifecycle is partly user/environment controlled.
  - Fix or mitigation: if the viewer launches successfully, the script records `viewer_closed_before_completion` and continues headlessly; if the viewer cannot launch at all, live mode exits nonzero.
  - Remaining risk: GUI behavior still depends on the machine's display/OpenGL stack.

### Result
`make sim-follow SEED=123` provides a MuJoCo viewer path with a visible orange proxy moving through the generated maze using the waypoint follower. `make view-sim-follow SEED=123` creates and opens a dashboard with the top-down maze, planned path, final MuJoCo render, trajectory CSV, and summary. G1 walking is still not implemented.

### Next actions
Investigate a real G1 locomotion adapter or policy before replacing the proxy with humanoid walking. Keep sensor/timebase work separate unless locomotion is deferred intentionally.
