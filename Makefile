PYTHON_VERSION ?= 3.11.15
PYENV_ROOT ?= $(HOME)/.pyenv
PYENV ?= $(PYENV_ROOT)/bin/pyenv
PYTHON ?= $(PYENV_ROOT)/versions/$(PYTHON_VERSION)/bin/python
VENV ?= .venv
OPENSSL_ROOT ?= $(HOME)/.local/openssl
SEED ?= 1
DURATION ?= 3
VIEW_DURATION ?= 30
CONFIG ?= configs/default.yaml
VISUAL_DIR ?= runs/visual
WORLD ?= maze
MAZE_CELL_PX ?= 36
PREVIEW_DURATION ?= 0.02
RUN_RENDER_WIDTH ?= 640
RUN_RENDER_HEIGHT ?= 480
SIM_FOLLOW_TIME_SCALE ?= 64
SIM_FOLLOW_FRAME_STRIDE ?= 64
SIM_FOLLOW_HOLD_S ?= 2

.PHONY: setup smoke view-smoke maze view-maze world view-world plan view-plan follow view-follow sim-follow view-sim-follow run view-run view test view-test milestone_3 milestone_4 milestone_5 clean

setup:
	@test -x "$(PYTHON)" || (echo "Python $(PYTHON_VERSION) was not found at $(PYTHON). Install it with: $(PYENV) install $(PYTHON_VERSION)" && exit 1)
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(PYTHON)" -m venv "$(VENV)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" -m pip install --upgrade pip
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" -m pip install -r requirements.txt

smoke:
	@test -x "$(VENV)/bin/python" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/smoke_test.py --config "$(CONFIG)" --save-html "$(VISUAL_DIR)/smoke_latest.html"

view-smoke:
	@$(MAKE) smoke CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)"; status=$$?; if [ $$status -eq 0 ]; then LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/open_artifact.py "$(VISUAL_DIR)/smoke_latest.html"; fi; exit $$status

maze:
	@test -x "$(VENV)/bin/python" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/generate_maze.py --seed "$(SEED)" --config "$(CONFIG)" --show-path --save-ascii "$(VISUAL_DIR)/maze_seed-$(SEED).txt" --save-pgm "$(VISUAL_DIR)/maze_seed-$(SEED).pgm" --save-svg "$(VISUAL_DIR)/maze_seed-$(SEED).svg" --cell-px "$(MAZE_CELL_PX)"

view-maze:
	@$(MAKE) maze SEED="$(SEED)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" MAZE_CELL_PX="$(MAZE_CELL_PX)"; status=$$?; if [ $$status -eq 0 ]; then LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/open_artifact.py "$(VISUAL_DIR)/maze_seed-$(SEED).svg"; fi; exit $$status

world:
	@test -x "$(VENV)/bin/python" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/generate_world.py --seed "$(SEED)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)"

view-world:
	@$(MAKE) world SEED="$(SEED)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)"; status=$$?; if [ $$status -eq 0 ]; then LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/open_artifact.py "$(VISUAL_DIR)/world_seed-$(SEED)_topdown.svg"; fi; exit $$status

plan:
	@test -x "$(VENV)/bin/python" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/plan_path.py --seed "$(SEED)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)" --cell-px "$(MAZE_CELL_PX)"

view-plan:
	@$(MAKE) plan SEED="$(SEED)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" MAZE_CELL_PX="$(MAZE_CELL_PX)"; status=$$?; if [ $$status -eq 0 ]; then LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/open_artifact.py "$(VISUAL_DIR)/plan_seed-$(SEED)_oracle.svg"; fi; exit $$status

follow:
	@test -x "$(VENV)/bin/python" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/follow_waypoints.py --seed "$(SEED)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)" --cell-px "$(MAZE_CELL_PX)"

view-follow:
	@$(MAKE) follow SEED="$(SEED)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" MAZE_CELL_PX="$(MAZE_CELL_PX)"; status=$$?; if [ $$status -eq 0 ]; then LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/open_artifact.py "$(VISUAL_DIR)/follow_seed-$(SEED)_point.svg"; fi; exit $$status

sim-follow:
	@test -x "$(VENV)/bin/python" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/sim_follow_waypoints.py --seed "$(SEED)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)" --cell-px "$(MAZE_CELL_PX)" --render-width "$(RUN_RENDER_WIDTH)" --render-height "$(RUN_RENDER_HEIGHT)" --time-scale "$(SIM_FOLLOW_TIME_SCALE)" --viewer-frame-stride "$(SIM_FOLLOW_FRAME_STRIDE)" --hold-s "$(SIM_FOLLOW_HOLD_S)" --viewer

view-sim-follow:
	@test -x "$(VENV)/bin/python" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/sim_follow_waypoints.py --seed "$(SEED)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)" --cell-px "$(MAZE_CELL_PX)" --render-width "$(RUN_RENDER_WIDTH)" --render-height "$(RUN_RENDER_HEIGHT)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/open_artifact.py "$(VISUAL_DIR)/sim_follow_seed-$(SEED)_dashboard.html"

run:
	@test -x "$(VENV)/bin/python" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/run_episode.py --seed "$(SEED)" --duration "$(PREVIEW_DURATION)" --config "$(CONFIG)" --world "$(WORLD)" --world-output-dir "$(VISUAL_DIR)" --save-summary-json "$(VISUAL_DIR)/run_seed-$(SEED)_summary.json" --save-render "$(VISUAL_DIR)/run_seed-$(SEED)_preview.png" --render-width "$(RUN_RENDER_WIDTH)" --render-height "$(RUN_RENDER_HEIGHT)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/write_run_dashboard.py --seed "$(SEED)" --mode "$(WORLD)" --html "$(VISUAL_DIR)/run_seed-$(SEED)_dashboard.html" --topdown-svg "$(VISUAL_DIR)/world_seed-$(SEED)_topdown.svg" --render-image "$(VISUAL_DIR)/run_seed-$(SEED)_preview.png" --world-xml "$(VISUAL_DIR)/world_seed-$(SEED).xml" --world-summary "$(VISUAL_DIR)/world_seed-$(SEED)_summary.json" --run-summary "$(VISUAL_DIR)/run_seed-$(SEED)_summary.json"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/open_artifact.py "$(VISUAL_DIR)/run_seed-$(SEED)_dashboard.html"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/run_episode.py --seed "$(SEED)" --duration "$(DURATION)" --config "$(CONFIG)" --world "$(WORLD)" --world-output-dir "$(VISUAL_DIR)" --viewer

view-run:
	@test -x "$(VENV)/bin/python" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/run_episode.py --seed "$(SEED)" --duration "$(DURATION)" --config "$(CONFIG)" --world "$(WORLD)" --world-output-dir "$(VISUAL_DIR)" --save-summary-json "$(VISUAL_DIR)/run_seed-$(SEED)_summary.json" --save-render "$(VISUAL_DIR)/run_seed-$(SEED)_final.png" --render-width "$(RUN_RENDER_WIDTH)" --render-height "$(RUN_RENDER_HEIGHT)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/write_run_dashboard.py --seed "$(SEED)" --mode "$(WORLD)" --html "$(VISUAL_DIR)/run_seed-$(SEED)_dashboard.html" --topdown-svg "$(VISUAL_DIR)/world_seed-$(SEED)_topdown.svg" --render-image "$(VISUAL_DIR)/run_seed-$(SEED)_final.png" --world-xml "$(VISUAL_DIR)/world_seed-$(SEED).xml" --world-summary "$(VISUAL_DIR)/world_seed-$(SEED)_summary.json" --run-summary "$(VISUAL_DIR)/run_seed-$(SEED)_summary.json"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/open_artifact.py "$(VISUAL_DIR)/run_seed-$(SEED)_dashboard.html"

view:
	@test -x "$(VENV)/bin/python" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/run_episode.py --seed "$(SEED)" --duration "$(VIEW_DURATION)" --config "$(CONFIG)" --world "$(WORLD)" --world-output-dir "$(VISUAL_DIR)" --viewer

test:
	@test -x "$(VENV)/bin/python" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/run_tests_report.py --text "$(VISUAL_DIR)/test_latest.txt" --html "$(VISUAL_DIR)/test_latest.html" tests

view-test:
	@$(MAKE) test VISUAL_DIR="$(VISUAL_DIR)"; status=$$?; LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/open_artifact.py "$(VISUAL_DIR)/test_latest.html" || true; exit $$status

milestone_3:
	@echo "Milestone 3 acceptance run: seed=$(SEED), duration=$(DURATION), world=$(WORLD)"
	@$(MAKE) test VISUAL_DIR="$(VISUAL_DIR)"
	@$(MAKE) view-world SEED="$(SEED)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)"
	@$(MAKE) view-run SEED="$(SEED)" DURATION="$(DURATION)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" WORLD="$(WORLD)" RUN_RENDER_WIDTH="$(RUN_RENDER_WIDTH)" RUN_RENDER_HEIGHT="$(RUN_RENDER_HEIGHT)"
	@$(MAKE) run SEED="$(SEED)" DURATION="$(DURATION)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" WORLD="$(WORLD)" PREVIEW_DURATION="$(PREVIEW_DURATION)" RUN_RENDER_WIDTH="$(RUN_RENDER_WIDTH)" RUN_RENDER_HEIGHT="$(RUN_RENDER_HEIGHT)"

milestone_4:
	@echo "Milestone 4 acceptance run: seed=$(SEED), world=$(WORLD)"
	@$(MAKE) test VISUAL_DIR="$(VISUAL_DIR)"
	@$(MAKE) view-plan SEED="$(SEED)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" MAZE_CELL_PX="$(MAZE_CELL_PX)"

milestone_5:
	@echo "Milestone 5 acceptance run: seed=$(SEED)"
	@$(MAKE) test VISUAL_DIR="$(VISUAL_DIR)"
	@$(MAKE) view-follow SEED="$(SEED)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" MAZE_CELL_PX="$(MAZE_CELL_PX)"

clean:
	rm -rf .pytest_cache
