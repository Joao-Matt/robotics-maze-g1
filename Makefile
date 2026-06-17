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
CORRIDOR_WIDTH_M ?= 1.6
WIDE_CORRIDOR_WIDTH_M ?= 2.0
MILESTONE_4_LABEL ?=
ORACLE_FOLLOW_LABEL ?=
THIRD_PARTY_DIR ?= third_party
LUCKY_G1_REPO ?= $(THIRD_PARTY_DIR)/g1-manipulation-challenge
UNITREE_RL_GYM_REPO ?= $(THIRD_PARTY_DIR)/unitree_rl_gym
TORCH_CPU_INDEX ?= https://download.pytorch.org/whl/cpu
DOCKER_IMAGE ?= robotics-maze-g1:humble
DOCKER_PLATFORMS ?= linux/amd64,linux/arm64
ROS_DOMAIN_ID ?= 0

.PHONY: setup install-torch-cpu smoke view-smoke maze view-maze world view-world run view-run view fetch-lucky-g1-policy fetch-unitree-rl-gym-policy g1-loco-sandbox g1-loco-view locomotion-sandbox view-locomotion-sandbox milestone_4 view-milestone_4 milestone_4-wide view-milestone_4-wide g1-oracle-follow view-g1-oracle-follow docker-build docker-run docker-run-gui docker-test docker-smoke docker-check-ros docker-milestone_4 docker-build-multiarch test view-test clean

setup:
	@test -x "$(PYTHON)" || (echo "Python $(PYTHON_VERSION) was not found at $(PYTHON). Install it with: $(PYENV) install $(PYTHON_VERSION)" && exit 1)
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(PYTHON)" -m venv "$(VENV)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" -m pip install --upgrade pip
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" -m pip install -r requirements.txt

install-torch-cpu:
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" -m pip install "torch==2.12.0+cpu" --index-url "$(TORCH_CPU_INDEX)"

smoke:
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/smoke_test.py --config "$(CONFIG)" --save-html "$(VISUAL_DIR)/smoke_latest.html"

view-smoke:
	@$(MAKE) smoke CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)"; status=$$?; if [ $$status -eq 0 ]; then LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/smoke_latest.html"; fi; exit $$status

maze:
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/generate_maze.py --seed "$(SEED)" --config "$(CONFIG)" --show-path --save-ascii "$(VISUAL_DIR)/maze_seed-$(SEED).txt" --save-pgm "$(VISUAL_DIR)/maze_seed-$(SEED).pgm" --save-svg "$(VISUAL_DIR)/maze_seed-$(SEED).svg" --cell-px "$(MAZE_CELL_PX)"

view-maze:
	@$(MAKE) maze SEED="$(SEED)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" MAZE_CELL_PX="$(MAZE_CELL_PX)"; status=$$?; if [ $$status -eq 0 ]; then LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/maze_seed-$(SEED).svg"; fi; exit $$status

world: fetch-lucky-g1-policy
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/generate_world.py --seed "$(SEED)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)"

view-world:
	@$(MAKE) world SEED="$(SEED)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)"; status=$$?; if [ $$status -eq 0 ]; then LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/world_seed-$(SEED)_topdown.svg"; fi; exit $$status

run: fetch-lucky-g1-policy
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

fetch-lucky-g1-policy:
	@mkdir -p "$(THIRD_PARTY_DIR)"
	@if [ -d "$(LUCKY_G1_REPO)/.git" ]; then \
		git -C "$(LUCKY_G1_REPO)" pull --ff-only; \
	else \
		git clone --depth 1 https://github.com/luckyrobots/g1-manipulation-challenge.git "$(LUCKY_G1_REPO)"; \
	fi

fetch-unitree-rl-gym-policy:
	@mkdir -p "$(THIRD_PARTY_DIR)"
	@if [ -d "$(UNITREE_RL_GYM_REPO)/.git" ]; then \
		git -C "$(UNITREE_RL_GYM_REPO)" pull --ff-only; \
	else \
		git clone --depth 1 https://github.com/unitreerobotics/unitree_rl_gym.git "$(UNITREE_RL_GYM_REPO)"; \
	fi

g1-loco-sandbox:
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

milestone_4: fetch-lucky-g1-policy
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_milestone_4.py --seed "$(SEED)" --duration "$(MILESTONE_4_DURATION)" --corridor-width-m "$(CORRIDOR_WIDTH_M)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)" --lucky-g1-repo "$(LUCKY_G1_REPO)" --label "$(MILESTONE_4_LABEL)" --viewer

view-milestone_4: fetch-lucky-g1-policy
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_milestone_4.py --seed "$(SEED)" --duration "$(MILESTONE_4_DURATION)" --corridor-width-m "$(CORRIDOR_WIDTH_M)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)" --lucky-g1-repo "$(LUCKY_G1_REPO)" --label "$(MILESTONE_4_LABEL)"
	@label_suffix=""; if [ -n "$(MILESTONE_4_LABEL)" ]; then label_suffix="_$(MILESTONE_4_LABEL)"; fi; LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/milestone_4$${label_suffix}_seed-$(SEED)_dashboard.html"

milestone_4-wide:
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	@$(MAKE) fetch-lucky-g1-policy THIRD_PARTY_DIR="$(THIRD_PARTY_DIR)" LUCKY_G1_REPO="$(LUCKY_G1_REPO)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_milestone_4.py --seed "$(SEED)" --duration "$(MILESTONE_4_DURATION)" --corridor-width-m "$(WIDE_CORRIDOR_WIDTH_M)" --config "$(WIDE_CONFIG)" --output-dir "$(VISUAL_DIR)" --lucky-g1-repo "$(LUCKY_G1_REPO)" --label wide --viewer

view-milestone_4-wide:
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	@$(MAKE) fetch-lucky-g1-policy THIRD_PARTY_DIR="$(THIRD_PARTY_DIR)" LUCKY_G1_REPO="$(LUCKY_G1_REPO)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_milestone_4.py --seed "$(SEED)" --duration "$(MILESTONE_4_DURATION)" --corridor-width-m "$(WIDE_CORRIDOR_WIDTH_M)" --config "$(WIDE_CONFIG)" --output-dir "$(VISUAL_DIR)" --lucky-g1-repo "$(LUCKY_G1_REPO)" --label wide
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/milestone_4_wide_seed-$(SEED)_dashboard.html"

g1-oracle-follow: fetch-lucky-g1-policy
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_g1_oracle_follow.py --seed "$(SEED)" --duration "$(ORACLE_FOLLOW_DURATION)" --corridor-width-m "$(CORRIDOR_WIDTH_M)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)" --lucky-g1-repo "$(LUCKY_G1_REPO)" --label "$(ORACLE_FOLLOW_LABEL)" --viewer

view-g1-oracle-follow: fetch-lucky-g1-policy
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_g1_oracle_follow.py --seed "$(SEED)" --duration "$(ORACLE_FOLLOW_DURATION)" --corridor-width-m "$(CORRIDOR_WIDTH_M)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)" --lucky-g1-repo "$(LUCKY_G1_REPO)" --label "$(ORACLE_FOLLOW_LABEL)"
	@label_suffix=""; if [ -n "$(ORACLE_FOLLOW_LABEL)" ]; then label_suffix="_$(ORACLE_FOLLOW_LABEL)"; fi; LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/g1_oracle_follow$${label_suffix}_seed-$(SEED)_dashboard.html"

docker-build:
	docker build -t "$(DOCKER_IMAGE)" -f docker/Dockerfile .

docker-run:
	DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh

docker-run-gui:
	DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run_gui.sh

docker-test:
	DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make test

docker-smoke:
	DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make smoke

docker-check-ros:
	DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh scripts/check_ros_docker_env.sh

docker-milestone_4:
	@if [ -n "$$DISPLAY" ]; then \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run_gui.sh make milestone_4 SEED="$(SEED)" MILESTONE_4_DURATION="$(MILESTONE_4_DURATION)" CORRIDOR_WIDTH_M="$(CORRIDOR_WIDTH_M)"; \
	else \
		echo "DISPLAY is not set; running headless dashboard mode with view-milestone_4 instead of live MuJoCo viewer."; \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make view-milestone_4 SEED="$(SEED)" MILESTONE_4_DURATION="$(MILESTONE_4_DURATION)" CORRIDOR_WIDTH_M="$(CORRIDOR_WIDTH_M)"; \
	fi

docker-build-multiarch:
	DOCKER_IMAGE="$(DOCKER_IMAGE)" DOCKER_PLATFORMS="$(DOCKER_PLATFORMS)" docker/build_multiarch.sh

test: fetch-lucky-g1-policy
	@test -x "$(VENV_PYTHON)" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	@mkdir -p "$(VISUAL_DIR)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/run_tests_report.py --text "$(VISUAL_DIR)/test_latest.txt" --html "$(VISUAL_DIR)/test_latest.html" tests

view-test:
	@$(MAKE) test VISUAL_DIR="$(VISUAL_DIR)"; status=$$?; LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV_PYTHON)" scripts/open_artifact.py "$(VISUAL_DIR)/test_latest.html" || true; exit $$status

clean:
	rm -rf .pytest_cache
