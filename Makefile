PYTHON_VERSION ?= 3.11.15
PYENV_ROOT ?= $(HOME)/.pyenv
PYENV ?= $(PYENV_ROOT)/bin/pyenv
PYTHON ?= $(PYENV_ROOT)/versions/$(PYTHON_VERSION)/bin/python
VENV ?= .venv
ifeq ($(VENV),/usr)
VENV_PYTHON ?= /usr/bin/python3
else
VENV_PYTHON ?= $(VENV)/bin/python
endif
OPENSSL_ROOT ?= $(HOME)/.local/openssl
SEED ?= 1
DURATION ?= 3
VIEW_DURATION ?= 30
CONFIG ?= configs/default.yaml
WIDE_CONFIG ?= configs/lucky_wide_maze.yaml
VISUAL_DIR ?= runs/visual
WORLD ?= maze
MAZE_CELL_PX ?= 36
PREVIEW_DURATION ?= 0.02
RUN_RENDER_WIDTH ?= 640
RUN_RENDER_HEIGHT ?= 480
POLICY ?= placeholder
G1_LOCO_DURATION ?= 3
MILESTONE_4_DURATION ?= 300
ORACLE_FOLLOW_DURATION ?= 300
MILESTONE_5_DURATION ?= $(ORACLE_FOLLOW_DURATION)
CORRIDOR_WIDTH_M ?= 1.6
WIDE_CORRIDOR_WIDTH_M ?= 2.0
MILESTONE_4_LABEL ?=
ORACLE_FOLLOW_LABEL ?=
MILESTONE_5_LABEL ?= $(ORACLE_FOLLOW_LABEL)
THIRD_PARTY_DIR ?= third_party
LUCKY_G1_REPO ?= $(THIRD_PARTY_DIR)/g1-manipulation-challenge
UNITREE_RL_GYM_REPO ?= $(THIRD_PARTY_DIR)/unitree_rl_gym
TORCH_CPU_INDEX ?= https://download.pytorch.org/whl/cpu
DOCKER_IMAGE ?= robotics-maze-g1:humble
DOCKER_PLATFORMS ?= linux/amd64,linux/arm64
ROS_DOMAIN_ID ?= 0
PROJECT_TMP ?= .tmp
export TMPDIR := $(abspath $(PROJECT_TMP))

-include .env.storage
export REQUIRED_STORAGE_MOUNT
export EXPECTED_STORAGE_UUID

.PHONY: storage-check setup install-torch-cpu smoke view-smoke maze view-maze world view-world run view-run view fetch-lucky-g1-policy fetch-unitree-rl-gym-policy g1-loco-sandbox g1-loco-view locomotion-sandbox view-locomotion-sandbox milestone_4 view-milestone_4 milestone_4-wide view-milestone_4-wide milestone_5 view-milestone_5 report-milestone_5 g1-oracle-follow view-g1-oracle-follow report-g1-oracle-follow docker-build docker-run docker-run-gui docker-test docker-smoke docker-check-ros docker-milestone_4 docker-milestone_5 docker-build-multiarch test view-test clean

storage-check:
	scripts/check_storage_layout.sh

setup: storage-check
	@test -x "$(PYTHON)" || (echo "Python $(PYTHON_VERSION) was not found at $(PYTHON). Install it with: $(PYENV) install $(PYTHON_VERSION)" && exit 1)
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(PYTHON)" -m venv "$(VENV)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" -m pip install --upgrade pip
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" -m pip install -r requirements.txt

install-torch-cpu:
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" -m pip install "torch==2.12.0+cpu" --index-url "$(TORCH_CPU_INDEX)"

smoke: storage-check
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/smoke_test.py --config "$(CONFIG)" --save-html "$(VISUAL_DIR)/smoke_latest.html"

view-smoke:
	@$(MAKE) smoke CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)"; status=$$?; if [ $$status -eq 0 ]; then LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/smoke_latest.html"; fi; exit $$status

maze: storage-check
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/generate_maze.py --seed "$(SEED)" --config "$(CONFIG)" --show-path --save-ascii "$(VISUAL_DIR)/maze_seed-$(SEED).txt" --save-pgm "$(VISUAL_DIR)/maze_seed-$(SEED).pgm" --save-svg "$(VISUAL_DIR)/maze_seed-$(SEED).svg" --cell-px "$(MAZE_CELL_PX)"

view-maze:
	@$(MAKE) maze SEED="$(SEED)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" MAZE_CELL_PX="$(MAZE_CELL_PX)"; status=$$?; if [ $$status -eq 0 ]; then LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/maze_seed-$(SEED).svg"; fi; exit $$status

world: storage-check fetch-lucky-g1-policy
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/generate_world.py --seed "$(SEED)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)"

view-world:
	@$(MAKE) world SEED="$(SEED)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)"; status=$$?; if [ $$status -eq 0 ]; then LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/world_seed-$(SEED)_topdown.svg"; fi; exit $$status

run: storage-check fetch-lucky-g1-policy
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_episode.py --seed "$(SEED)" --duration "$(PREVIEW_DURATION)" --config "$(CONFIG)" --world "$(WORLD)" --world-output-dir "$(VISUAL_DIR)" --save-summary-json "$(VISUAL_DIR)/run_seed-$(SEED)_summary.json" --save-render "$(VISUAL_DIR)/run_seed-$(SEED)_preview.png" --render-width "$(RUN_RENDER_WIDTH)" --render-height "$(RUN_RENDER_HEIGHT)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/write_run_dashboard.py --seed "$(SEED)" --mode "$(WORLD)" --html "$(VISUAL_DIR)/run_seed-$(SEED)_dashboard.html" --topdown-svg "$(VISUAL_DIR)/world_seed-$(SEED)_topdown.svg" --render-image "$(VISUAL_DIR)/run_seed-$(SEED)_preview.png" --world-xml "$(VISUAL_DIR)/world_seed-$(SEED).xml" --world-summary "$(VISUAL_DIR)/world_seed-$(SEED)_summary.json" --run-summary "$(VISUAL_DIR)/run_seed-$(SEED)_summary.json"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/run_seed-$(SEED)_dashboard.html"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_episode.py --seed "$(SEED)" --duration "$(DURATION)" --config "$(CONFIG)" --world "$(WORLD)" --world-output-dir "$(VISUAL_DIR)" --viewer

view-run: fetch-lucky-g1-policy
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_episode.py --seed "$(SEED)" --duration "$(DURATION)" --config "$(CONFIG)" --world "$(WORLD)" --world-output-dir "$(VISUAL_DIR)" --save-summary-json "$(VISUAL_DIR)/run_seed-$(SEED)_summary.json" --save-render "$(VISUAL_DIR)/run_seed-$(SEED)_final.png" --render-width "$(RUN_RENDER_WIDTH)" --render-height "$(RUN_RENDER_HEIGHT)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/write_run_dashboard.py --seed "$(SEED)" --mode "$(WORLD)" --html "$(VISUAL_DIR)/run_seed-$(SEED)_dashboard.html" --topdown-svg "$(VISUAL_DIR)/world_seed-$(SEED)_topdown.svg" --render-image "$(VISUAL_DIR)/run_seed-$(SEED)_final.png" --world-xml "$(VISUAL_DIR)/world_seed-$(SEED).xml" --world-summary "$(VISUAL_DIR)/world_seed-$(SEED)_summary.json" --run-summary "$(VISUAL_DIR)/run_seed-$(SEED)_summary.json"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/run_seed-$(SEED)_dashboard.html"

view: fetch-lucky-g1-policy
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_episode.py --seed "$(SEED)" --duration "$(VIEW_DURATION)" --config "$(CONFIG)" --world "$(WORLD)" --world-output-dir "$(VISUAL_DIR)" --viewer

fetch-lucky-g1-policy: storage-check
	@mkdir -p "$(THIRD_PARTY_DIR)"
	@if [ -d "$(LUCKY_G1_REPO)/.git" ]; then \
		git -C "$(LUCKY_G1_REPO)" pull --ff-only; \
	else \
		git clone --depth 1 https://github.com/luckyrobots/g1-manipulation-challenge.git "$(LUCKY_G1_REPO)"; \
	fi

fetch-unitree-rl-gym-policy: storage-check
	@mkdir -p "$(THIRD_PARTY_DIR)"
	@if [ -d "$(UNITREE_RL_GYM_REPO)/.git" ]; then \
		git -C "$(UNITREE_RL_GYM_REPO)" pull --ff-only; \
	else \
		git clone --depth 1 https://github.com/unitreerobotics/unitree_rl_gym.git "$(UNITREE_RL_GYM_REPO)"; \
	fi

g1-loco-sandbox: storage-check
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	@torch_dir=$$(dirname "$$(ls "$(VENV)"/lib/python*/site-packages/torch/lib/libgomp.so.1 2>/dev/null | head -1)"); \
	torch_preload=""; if [ -f "$$torch_dir/libgomp.so.1" ] && [ -f "$$torch_dir/libc10.so" ]; then torch_preload="$$torch_dir/libgomp.so.1:$$torch_dir/libc10.so"; fi; \
	LD_PRELOAD="$${torch_preload}$${LD_PRELOAD:+:$$LD_PRELOAD}" LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/g1_loco_sandbox.py --policy "$(POLICY)" --duration "$(G1_LOCO_DURATION)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)" --lucky-g1-repo "$(LUCKY_G1_REPO)" --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)" --viewer

g1-loco-view:
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	@torch_dir=$$(dirname "$$(ls "$(VENV)"/lib/python*/site-packages/torch/lib/libgomp.so.1 2>/dev/null | head -1)"); \
	torch_preload=""; if [ -f "$$torch_dir/libgomp.so.1" ] && [ -f "$$torch_dir/libc10.so" ]; then torch_preload="$$torch_dir/libgomp.so.1:$$torch_dir/libc10.so"; fi; \
	LD_PRELOAD="$${torch_preload}$${LD_PRELOAD:+:$$LD_PRELOAD}" LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/g1_loco_sandbox.py --policy "$(POLICY)" --duration "$(G1_LOCO_DURATION)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)" --lucky-g1-repo "$(LUCKY_G1_REPO)" --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/g1_loco_latest_dashboard.html"

locomotion-sandbox: g1-loco-sandbox

view-locomotion-sandbox: g1-loco-view

milestone_4: storage-check
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_milestone_4_planner.py --seed "$(SEED)" --corridor-width-m "$(CORRIDOR_WIDTH_M)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)" --label "$(MILESTONE_4_LABEL)"

view-milestone_4:
	@$(MAKE) milestone_4 SEED="$(SEED)" CORRIDOR_WIDTH_M="$(CORRIDOR_WIDTH_M)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" MILESTONE_4_LABEL="$(MILESTONE_4_LABEL)"
	@label_suffix=""; if [ -n "$(MILESTONE_4_LABEL)" ]; then label_suffix="_$(MILESTONE_4_LABEL)"; fi; LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/milestone_4$${label_suffix}_seed-$(SEED)_path.svg"

milestone_4-wide: storage-check
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_milestone_4_planner.py --seed "$(SEED)" --corridor-width-m "$(WIDE_CORRIDOR_WIDTH_M)" --config "$(WIDE_CONFIG)" --output-dir "$(VISUAL_DIR)" --label wide

view-milestone_4-wide:
	@$(MAKE) milestone_4-wide SEED="$(SEED)" WIDE_CORRIDOR_WIDTH_M="$(WIDE_CORRIDOR_WIDTH_M)" WIDE_CONFIG="$(WIDE_CONFIG)" VISUAL_DIR="$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/milestone_4_wide_seed-$(SEED)_path.svg"

milestone_5: storage-check fetch-lucky-g1-policy
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_g1_oracle_follow.py --seed "$(SEED)" --duration "$(MILESTONE_5_DURATION)" --corridor-width-m "$(CORRIDOR_WIDTH_M)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)" --lucky-g1-repo "$(LUCKY_G1_REPO)" --label "$(MILESTONE_5_LABEL)" --viewer

view-milestone_5:
	@$(MAKE) milestone_5 SEED="$(SEED)" MILESTONE_5_DURATION="$(MILESTONE_5_DURATION)" CORRIDOR_WIDTH_M="$(CORRIDOR_WIDTH_M)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" LUCKY_G1_REPO="$(LUCKY_G1_REPO)" MILESTONE_5_LABEL="$(MILESTONE_5_LABEL)"

report-milestone_5: storage-check fetch-lucky-g1-policy
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_g1_oracle_follow.py --seed "$(SEED)" --duration "$(MILESTONE_5_DURATION)" --corridor-width-m "$(CORRIDOR_WIDTH_M)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)" --lucky-g1-repo "$(LUCKY_G1_REPO)" --label "$(MILESTONE_5_LABEL)"
	@label_suffix=""; if [ -n "$(MILESTONE_5_LABEL)" ]; then label_suffix="_$(MILESTONE_5_LABEL)"; fi; LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/g1_oracle_follow$${label_suffix}_seed-$(SEED)_dashboard.html"

g1-oracle-follow:
	@$(MAKE) milestone_5 SEED="$(SEED)" MILESTONE_5_DURATION="$(ORACLE_FOLLOW_DURATION)" CORRIDOR_WIDTH_M="$(CORRIDOR_WIDTH_M)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" LUCKY_G1_REPO="$(LUCKY_G1_REPO)" MILESTONE_5_LABEL="$(ORACLE_FOLLOW_LABEL)"

view-g1-oracle-follow:
	@$(MAKE) view-milestone_5 SEED="$(SEED)" MILESTONE_5_DURATION="$(ORACLE_FOLLOW_DURATION)" CORRIDOR_WIDTH_M="$(CORRIDOR_WIDTH_M)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" LUCKY_G1_REPO="$(LUCKY_G1_REPO)" MILESTONE_5_LABEL="$(ORACLE_FOLLOW_LABEL)"

report-g1-oracle-follow:
	@$(MAKE) report-milestone_5 SEED="$(SEED)" MILESTONE_5_DURATION="$(ORACLE_FOLLOW_DURATION)" CORRIDOR_WIDTH_M="$(CORRIDOR_WIDTH_M)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" LUCKY_G1_REPO="$(LUCKY_G1_REPO)" MILESTONE_5_LABEL="$(ORACLE_FOLLOW_LABEL)"

docker-build: storage-check
	docker build -t "$(DOCKER_IMAGE)" -f docker/Dockerfile .

docker-run: storage-check
	DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh

docker-run-gui: storage-check
	DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run_gui.sh

docker-test: storage-check
	DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make test

docker-smoke: storage-check
	DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make smoke

docker-check-ros: storage-check
	DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh scripts/check_ros_docker_env.sh

docker-milestone_4: storage-check
	DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make milestone_4 SEED="$(SEED)" CORRIDOR_WIDTH_M="$(CORRIDOR_WIDTH_M)"

docker-milestone_5: storage-check
	@if [ -n "$$DISPLAY" ]; then \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run_gui.sh make milestone_5 SEED="$(SEED)" MILESTONE_5_DURATION="$(MILESTONE_5_DURATION)" CORRIDOR_WIDTH_M="$(CORRIDOR_WIDTH_M)"; \
	else \
		echo "DISPLAY is not set; running the Milestone 5 oracle follower headlessly."; \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make report-milestone_5 SEED="$(SEED)" MILESTONE_5_DURATION="$(MILESTONE_5_DURATION)" CORRIDOR_WIDTH_M="$(CORRIDOR_WIDTH_M)"; \
	fi

docker-build-multiarch: storage-check
	DOCKER_IMAGE="$(DOCKER_IMAGE)" DOCKER_PLATFORMS="$(DOCKER_PLATFORMS)" docker/build_multiarch.sh

test: storage-check fetch-lucky-g1-policy
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_tests_report.py --text "$(VISUAL_DIR)/test_latest.txt" --html "$(VISUAL_DIR)/test_latest.html" tests

view-test:
	@$(MAKE) test VISUAL_DIR="$(VISUAL_DIR)"; status=$$?; LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/test_latest.html" || true; exit $$status

clean:
	rm -rf .pytest_cache
