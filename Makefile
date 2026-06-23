SHELL := /bin/bash

CONFIG ?= configs/default.yaml
SEED ?= 123
CELL_SIZE_M ?= $(if $(CORRIDOR_WIDTH),$(CORRIDOR_WIDTH),2.0)
CELL_SIZE_MIN ?= 1.0
CELL_SIZE_MAX ?= 4.0
DEMO_CELL_SIZE_M ?= $(CELL_SIZE_M)
RUN_ROOT ?= runs
VISUAL_DIR ?= runs/visual
ORACLE_DURATION ?= 300
ORACLE_LABEL ?=
SLAM_DURATION ?= 300
NAVIGATE_DURATION ?= 1200
ROS_BRIDGE_PORT ?= 8765
DASHBOARD_PORT ?= 8765
ROS_DOMAIN_ID ?= 0
DOCKER_IMAGE ?= robotics-maze-g1:production
DOCKER_PLATFORMS ?= linux/amd64,linux/arm64
VENV ?= .venv
ifeq ($(VENV),/usr)
VENV_PYTHON ?= /usr/bin/python3
else
VENV_PYTHON ?= $(VENV)/bin/python
endif
PYTHON_PACKAGE_DIR ?= $(PROJECT_TMP)/python-packages
PROJECT_TMP ?= .tmp
THIRD_PARTY_DIR ?= third_party
UNITREE_RL_GYM_REPO ?= $(THIRD_PARTY_DIR)/unitree_rl_gym
M_EXPLORE_REPO ?= $(THIRD_PARTY_DIR)/m-explore-ros2
M_EXPLORE_URL ?= https://github.com/robo-friends/m-explore-ros2.git
M_EXPLORE_COMMIT ?= 326cf8a0b487c34246bb8f3326afbcd69576dc60
TORCH_CPU_INDEX ?= https://download.pytorch.org/whl/cpu
TORCH_CPU_PACKAGE ?= torch==2.5.1+cpu
NAVIGATE_SKIP_BUILD ?= auto
NAVIGATE_WITH_RVIZ ?= false
NAVIGATE_WITH_MUJOCO ?= false
NAVIGATE_DASHBOARD ?= true
DASHBOARD_AUTO_OPEN ?= true
NAVIGATE_LAUNCH_ARGS ?=
NAVIGATE_COMMAND ?= navigate
NAVIGATE_CAPTURE_SCHEMA ?=
NAVIGATE_CAPTURE_LAUNCH_ARGS ?=
NAV_RECORD_SCHEMA ?= configs/navigation_capture_topics.yaml
NAV_RECORD_SPLIT_BYTES ?= 536870912
NAV_RECORD_STORAGE ?= sqlite3
NAV_RECORD_RGBD_RATE_HZ ?= 3.0
RUN_DIR ?=
ODOM_TUNE_RUN_ROOT ?= runs/odom_tuning
ODOM_TUNE_DURATION ?= 240
ODOM_TUNE_SEEDS ?= 123 81
ODOM_TUNE_SKIP_PREBUILD ?= false
ROS_PREBUILD_STAMP ?= $(PROJECT_TMP)/ros-prebuild.stamp
ROS_PREBUILD_CHECK_SOURCES ?= false
ROS_FORCE_PREBUILD ?= false
CALIBRATION_CONFIG ?= configs/g1_locomotion_calibration.yaml
CALIBRATION_RUN_ROOT ?= runs/calibration
CALIBRATION_BATCH_COUNT ?= 100
CALIBRATION_BATCH_SEEDS ?=
CALIBRATION_BATCH_PROFILE ?= balanced
CALIBRATION_BATCH_FRICTION_MIN ?= 0.75
CALIBRATION_BATCH_FRICTION_MAX ?= 1.15
CALIBRATION_BATCH_MIN_STABLE_RATE ?= 0.70
CALIBRATION_BATCH_MAX_FALL_COUNT ?= 0
CALIBRATION_BATCH_MAX_STUCK_COUNT ?= 0
CALIBRATION_BATCH_MAX_NON_FLOOR_CONTACT_COUNT ?= 5
CALIBRATION_BATCH_MIN_SAFE_VX ?= 0.40
CALIBRATION_BATCH_MIN_SAFE_WZ ?= 0.40
LOCOMOTION_CALIBRATION ?=
NAV2_LIMIT_MODE ?= $(if $(LOCOMOTION_CALIBRATION),use-calibration,cap)
CALIBRATED_ORACLE_SPEED_MODE ?= cap
RL_CONFIG ?= configs/rl_velocity_controller.yaml
RL_RUN_ROOT ?= runs/rl_velocity
RL_TIMESTEPS ?=
RL_NUM_ENVS ?= 1
RL_STAGE ?=
RL_EVAL_EPISODES ?=
RL_EVAL_SUITE ?=
CHECKPOINT ?=
VEC_NORMALIZE ?=
HELDOUT_RUN_ROOT ?= runs/heldout-20
HELDOUT_LABEL ?= held_out
HELDOUT_JOBS ?= 1
HELDOUT_BASE_ROS_DOMAIN_ID ?= 40
HELDOUT_SEEDS ?= 1126362096 1979650228 1206536813 795378426 711116612 1738064285 971229033 329188623 894390399 32784551 170038683 1285796317 299369674 380511688 1312910544 1648306726 1106423062 1945965940 916063502 781626286
SEEN_NAV_ROOT ?=
SLAM_WITH_RVIZ ?= false

export TMPDIR := $(abspath $(PROJECT_TMP))

-include .env.storage
export REQUIRED_STORAGE_MOUNT
export EXPECTED_STORAGE_UUID

.PHONY: help storage-check setup install-torch-cpu install-rl-deps docker-build docker-run docker-run-gui docker-check-ros docker-build-multiarch
.PHONY: fetch-unitree-rl-gym-policy fetch-m-explore prebuild prebuild-inner maze world oracle oracle-view oracle-calibrated oracle-calibrated-view oracle-inner slam slam-view slam-inner navigate navigate-record navigate-view navigate-full-view demo navigate-inner heldout-navigate heldout-report bag-info repair-run clean
.PHONY: odom-tune locomotion-calibrate locomotion-calibrate-smoke locomotion-calibrate-batch locomotion-calibrate-batch-smoke rl-train rl-eval rl-eval-corridor-sweep rl-replay

help:
	@printf '%s\n' \
		'Production targets:' \
		'  make docker-build' \
		'  make docker-run                      # headless shell with ROS Humble' \
		'  make docker-run-gui                  # GUI shell for RViz/MuJoCo viewer' \
		'  make prebuild                        # fetch Unitree RL Gym + m-explore and build ROS workspace' \
		'  make maze CELL_SIZE_M=2.0 SEED=123   # generate/validate square grid, 1-4 m cells' \
		'  make world CELL_SIZE_M=2.0 SEED=123  # generate MuJoCo world with G1 + D435i + laser source' \
		'  make oracle SEED=123                 # Unitree RL Gym native oracle path following' \
		'  make oracle-view SEED=123            # oracle path following with MuJoCo viewer' \
		'  make oracle-calibrated LOCOMOTION_CALIBRATION=... # oracle maze using calibrated command limits' \
			'  make slam SEED=123                   # oracle-driven SLAM with rosbag' \
			'  make slam-view SEED=123              # SLAM with RViz' \
			'  make navigate SEED=123               # SLAM + m-explore + Nav2 + rosbag' \
			'  make navigate-record SEED=123        # dataset capture: RGB-D + schema allowlist + split rosbag' \
			'  make navigate LOCOMOTION_CALIBRATION=... # Nav2 uses calibrated cmd_vel envelope' \
		'  make odom-tune                       # sweep scan-odom params against offline ground-truth metrics' \
		'  make locomotion-calibrate            # direct MuJoCo G1 walking policy command sweep' \
		'  make locomotion-calibrate-smoke      # tiny G1 walking calibration smoke run' \
		'  make locomotion-calibrate-batch      # 100-seed G1 walking calibration safety batch' \
		'  make locomotion-calibrate-batch-smoke # tiny multi-seed calibration batch smoke run' \
		'  make navigate-view SEED=123          # navigation with RViz' \
		'  make navigate-full-view SEED=123     # navigation with RViz + MuJoCo viewer' \
		'  make demo SEED=123 DEMO_CELL_SIZE_M=2.0 # interview demo with full-view navigation + live KPI dashboard' \
		'  make rl-train                        # direct MuJoCo PPO velocity-controller training' \
		'  make rl-eval CHECKPOINT=...          # evaluate a trained PPO checkpoint' \
		'  make rl-eval-corridor-sweep CHECKPOINT=... # 100 random mazes across 2-4 m corridors' \
		'  make rl-replay CHECKPOINT=... SEED=123 # replay one trained PPO episode' \
		'  make heldout-navigate                # run the fixed 20-seed headless held-out batch' \
		'  make heldout-report                  # aggregate held-out solve rate with 95% CI' \
		'  make repair-run RUN_DIR=runs/navigate-record/... # reindex/validate a crashed dataset run'

storage-check:
	scripts/check_storage_layout.sh

setup: storage-check
	python3 -m venv "$(VENV)"
	"$(VENV_PYTHON)" -m pip install --upgrade pip
	"$(VENV_PYTHON)" -m pip install -r requirements.txt

install-torch-cpu:
	@if PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$$PYTHONPATH" "$(VENV_PYTHON)" -c "import torch" >/dev/null 2>&1; then \
		echo "torch dependency ok"; \
	elif [ "$(VENV)" = "/usr" ]; then \
		mkdir -p "$(PYTHON_PACKAGE_DIR)"; \
		"$(VENV_PYTHON)" -m pip install --target "$(PYTHON_PACKAGE_DIR)" "$(TORCH_CPU_PACKAGE)" --index-url "$(TORCH_CPU_INDEX)"; \
	else \
		"$(VENV_PYTHON)" -m pip install "$(TORCH_CPU_PACKAGE)" --index-url "$(TORCH_CPU_INDEX)"; \
	fi

install-rl-deps: install-torch-cpu
	@if PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$$PYTHONPATH" "$(VENV_PYTHON)" -c "import gymnasium, stable_baselines3, tensorboard, mujoco, yaml, numpy" >/dev/null 2>&1; then \
		echo "RL dependencies ok"; \
	elif [ "$(VENV)" = "/usr" ]; then \
		mkdir -p "$(PYTHON_PACKAGE_DIR)"; \
		PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$$PYTHONPATH" "$(VENV_PYTHON)" -m pip install --target "$(PYTHON_PACKAGE_DIR)" -r requirements.txt -r requirements-rl.txt; \
	else \
		PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$$PYTHONPATH" "$(VENV_PYTHON)" -m pip install -r requirements.txt -r requirements-rl.txt; \
	fi

docker-build: storage-check
	docker build -t "$(DOCKER_IMAGE)" -f docker/Dockerfile .

docker-run: storage-check
	DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh

docker-run-gui: storage-check
	DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run_gui.sh

docker-check-ros: storage-check
	DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh scripts/check_ros_docker_env.sh

docker-build-multiarch: storage-check
	DOCKER_IMAGE="$(DOCKER_IMAGE)" DOCKER_PLATFORMS="$(DOCKER_PLATFORMS)" docker/build_multiarch.sh

fetch-unitree-rl-gym-policy: storage-check
	@mkdir -p "$(THIRD_PARTY_DIR)"
	@if [ -d "$(UNITREE_RL_GYM_REPO)/.git" ]; then \
		git -C "$(UNITREE_RL_GYM_REPO)" pull --ff-only; \
	else \
		git clone --depth 1 https://github.com/unitreerobotics/unitree_rl_gym.git "$(UNITREE_RL_GYM_REPO)"; \
	fi

fetch-m-explore: storage-check
	@mkdir -p "$(THIRD_PARTY_DIR)"
	@if [ ! -d "$(M_EXPLORE_REPO)/.git" ]; then git clone --no-checkout "$(M_EXPLORE_URL)" "$(M_EXPLORE_REPO)"; fi
	@if [ "$$(git -C "$(M_EXPLORE_REPO)" rev-parse HEAD 2>/dev/null || true)" != "$(M_EXPLORE_COMMIT)" ]; then \
		git -C "$(M_EXPLORE_REPO)" fetch --depth 1 origin "$(M_EXPLORE_COMMIT)"; \
		git -C "$(M_EXPLORE_REPO)" checkout --detach "$(M_EXPLORE_COMMIT)"; \
	fi
	@if grep -q 'rclcpp::Time(0)' "$(M_EXPLORE_REPO)/explore/src/costmap_client.cpp" && \
		grep -q 'prefer_minimum_turn' "$(M_EXPLORE_REPO)/explore/src/explore.cpp" && \
		grep -q 'max_frontier_goal_distance' "$(M_EXPLORE_REPO)/explore/src/explore.cpp"; then \
		echo "m-explore Humble TF/frontier patch already present"; \
	elif git -C "$(M_EXPLORE_REPO)" apply --reverse --check "$(abspath patches/m-explore-ros2-humble-latest-tf.patch)" >/dev/null 2>&1; then :; else \
		git -C "$(M_EXPLORE_REPO)" apply "$(abspath patches/m-explore-ros2-humble-latest-tf.patch)"; \
	fi

prebuild: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
		$(MAKE) prebuild-inner ROS_PREBUILD_CHECK_SOURCES=true; \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make prebuild-inner ROS_PREBUILD_CHECK_SOURCES=true; \
	fi

prebuild-inner: storage-check install-torch-cpu fetch-unitree-rl-gym-policy fetch-m-explore
	@test "$${ROS_DISTRO:-}" = "humble" || (echo "prebuild-inner requires ROS 2 Humble" && exit 1)
	@if ROS_PREBUILD_CHECK_SOURCES="$(ROS_PREBUILD_CHECK_SOURCES)" ROS_FORCE_PREBUILD="$(ROS_FORCE_PREBUILD)" bash scripts/ros_prebuild_needed.sh "$(ROS_PREBUILD_STAMP)"; then \
		rm -rf ros_ws/build/explore_lite_msgs ros_ws/build/explore_lite ros_ws/build/g1_mujoco_bridge ros_ws/build/g1_nav_bringup \
			ros_ws/install/explore_lite_msgs ros_ws/install/explore_lite ros_ws/install/g1_mujoco_bridge ros_ws/install/g1_nav_bringup; \
		PYTHONPATH="$(CURDIR):$$PYTHONPATH" colcon --log-base ros_ws/log build --base-paths ros_ws/src "$(M_EXPLORE_REPO)/explore" "$(M_EXPLORE_REPO)/explore_lite_msgs" --build-base ros_ws/build --install-base ros_ws/install --symlink-install --packages-select explore_lite_msgs explore_lite g1_mujoco_bridge g1_nav_bringup; \
		mkdir -p "$$(dirname "$(ROS_PREBUILD_STAMP)")"; touch "$(ROS_PREBUILD_STAMP)"; \
	else \
		echo "ROS workspace already built; skipping colcon build"; \
	fi

maze: storage-check
	@mkdir -p "$(VISUAL_DIR)"
	"$(VENV_PYTHON)" scripts/generate_maze.py --seed "$(SEED)" --config "$(CONFIG)" --cell-size-m "$(CELL_SIZE_M)" --show-path --save-ascii "$(VISUAL_DIR)/maze_seed-$(SEED).txt" --save-pgm "$(VISUAL_DIR)/maze_seed-$(SEED).pgm" --save-svg "$(VISUAL_DIR)/maze_seed-$(SEED).svg"

world: storage-check fetch-unitree-rl-gym-policy
	@mkdir -p "$(VISUAL_DIR)"
	"$(VENV_PYTHON)" scripts/generate_world.py --seed "$(SEED)" --config "$(CONFIG)" --cell-size-m "$(CELL_SIZE_M)" --output-dir "$(VISUAL_DIR)"

oracle: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ] || [ "$(VENV)" != ".venv" ]; then \
		$(MAKE) oracle-inner ORACLE_VIEWER=false LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)" CALIBRATED_ORACLE_SPEED_MODE="$(CALIBRATED_ORACLE_SPEED_MODE)" ORACLE_LABEL="$(ORACLE_LABEL)"; \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make oracle-inner ORACLE_VIEWER=false SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" ORACLE_DURATION="$(ORACLE_DURATION)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)" CALIBRATED_ORACLE_SPEED_MODE="$(CALIBRATED_ORACLE_SPEED_MODE)" ORACLE_LABEL="$(ORACLE_LABEL)"; \
	fi

oracle-view: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ] || [ "$(VENV)" != ".venv" ]; then \
		$(MAKE) oracle-inner ORACLE_VIEWER=true LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)" CALIBRATED_ORACLE_SPEED_MODE="$(CALIBRATED_ORACLE_SPEED_MODE)" ORACLE_LABEL="$(ORACLE_LABEL)"; \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run_gui.sh make oracle-inner ORACLE_VIEWER=true SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" ORACLE_DURATION="$(ORACLE_DURATION)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)" CALIBRATED_ORACLE_SPEED_MODE="$(CALIBRATED_ORACLE_SPEED_MODE)" ORACLE_LABEL="$(ORACLE_LABEL)"; \
	fi

oracle-calibrated:
	@test -n "$(LOCOMOTION_CALIBRATION)" || (echo "Usage: make oracle-calibrated LOCOMOTION_CALIBRATION=runs/calibration/.../locomotion_calibration.json" && exit 1)
	@$(MAKE) oracle LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)" CALIBRATED_ORACLE_SPEED_MODE="$(CALIBRATED_ORACLE_SPEED_MODE)" ORACLE_LABEL="$(if $(ORACLE_LABEL),$(ORACLE_LABEL),calibrated-$(CALIBRATED_ORACLE_SPEED_MODE))"

oracle-calibrated-view:
	@test -n "$(LOCOMOTION_CALIBRATION)" || (echo "Usage: make oracle-calibrated-view LOCOMOTION_CALIBRATION=runs/calibration/.../locomotion_calibration.json" && exit 1)
	@$(MAKE) oracle-view LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)" CALIBRATED_ORACLE_SPEED_MODE="$(CALIBRATED_ORACLE_SPEED_MODE)" ORACLE_LABEL="$(if $(ORACLE_LABEL),$(ORACLE_LABEL),calibrated-$(CALIBRATED_ORACLE_SPEED_MODE))"

oracle-inner: storage-check install-torch-cpu fetch-unitree-rl-gym-policy
	@mkdir -p "$(VISUAL_DIR)"
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/run_g1_oracle_follow.py --seed "$(SEED)" --duration "$(ORACLE_DURATION)" --corridor-width-m "$(CELL_SIZE_M)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)" --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)" --locomotion-policy unitree_rl_gym_native $(if $(ORACLE_LABEL),--label "$(ORACLE_LABEL)",) $(if $(LOCOMOTION_CALIBRATION),--locomotion-calibration "$(LOCOMOTION_CALIBRATION)" --calibrated-speed-mode "$(CALIBRATED_ORACLE_SPEED_MODE)",) $(if $(filter true,$(ORACLE_VIEWER)),--viewer,)

slam: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
		$(MAKE) slam-inner SLAM_WITH_RVIZ=false; \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make slam-inner SLAM_WITH_RVIZ=false SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" SLAM_DURATION="$(SLAM_DURATION)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)"; \
	fi

slam-view: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
		$(MAKE) slam-inner SLAM_WITH_RVIZ=true; \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run_gui.sh make slam-inner SLAM_WITH_RVIZ=true SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" SLAM_DURATION="$(SLAM_DURATION)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)" ROS_BRIDGE_PORT="$(ROS_BRIDGE_PORT)"; \
	fi

slam-inner: storage-check install-torch-cpu fetch-unitree-rl-gym-policy
	@test "$${ROS_DISTRO:-}" = "humble" || (echo "slam-inner requires ROS 2 Humble" && exit 1)
	@rm -rf ros_ws/build/g1_mujoco_bridge ros_ws/install/g1_mujoco_bridge
	PYTHONPATH="$(CURDIR):$$PYTHONPATH" colcon --log-base ros_ws/log build --base-paths ros_ws/src --build-base ros_ws/build --install-base ros_ws/install --symlink-install --packages-select g1_mujoco_bridge
	@rm -rf "$(VISUAL_DIR)/slam_seed-$(SEED)_bag"; rm -f "$(VISUAL_DIR)/slam_seed-$(SEED)_summary.json"
	@. ros_ws/install/setup.sh; PYTHONPATH="$(CURDIR):$$PYTHONPATH" ros2 launch g1_mujoco_bridge slam_map.launch.py seed:="$(SEED)" config_path:="$(abspath $(CONFIG))" output_dir:="$(abspath $(VISUAL_DIR))" duration_s:="$(SLAM_DURATION)" corridor_width_m:="$(CELL_SIZE_M)" unitree_rl_gym_repo:="$(abspath $(UNITREE_RL_GYM_REPO))" locomotion_policy:=unitree_rl_gym_native bag_path:="$(abspath $(VISUAL_DIR))/slam_seed-$(SEED)_bag" port:="$(ROS_BRIDGE_PORT)" with_rviz:="$(SLAM_WITH_RVIZ)"

navigate: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
			$(MAKE) navigate-inner NAVIGATE_WITH_RVIZ=false NAVIGATE_WITH_MUJOCO=false NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD="$(NAVIGATE_DASHBOARD)" DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)" NAVIGATE_LAUNCH_ARGS="$(NAVIGATE_LAUNCH_ARGS)" LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)" NAV2_LIMIT_MODE="$(NAV2_LIMIT_MODE)"; \
		else \
			DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make navigate-inner NAVIGATE_WITH_RVIZ=false NAVIGATE_WITH_MUJOCO=false SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" NAVIGATE_DURATION="$(NAVIGATE_DURATION)" CONFIG="$(CONFIG)" RUN_ROOT="$(RUN_ROOT)" NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD="$(NAVIGATE_DASHBOARD)" DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)" NAVIGATE_LAUNCH_ARGS="$(NAVIGATE_LAUNCH_ARGS)" LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)" NAV2_LIMIT_MODE="$(NAV2_LIMIT_MODE)"; \
		fi

navigate-record: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
			$(MAKE) navigate-inner NAVIGATE_COMMAND=navigate-record NAVIGATE_WITH_RVIZ=false NAVIGATE_WITH_MUJOCO=false NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD="$(NAVIGATE_DASHBOARD)" DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)" NAVIGATE_LAUNCH_ARGS="$(NAVIGATE_LAUNCH_ARGS)" NAVIGATE_CAPTURE_SCHEMA="$(NAV_RECORD_SCHEMA)" NAVIGATE_CAPTURE_LAUNCH_ARGS="camera_enabled:=true camera_rate_hz:=$(NAV_RECORD_RGBD_RATE_HZ) depth_only:=false" NAV_RECORD_STORAGE="$(NAV_RECORD_STORAGE)" NAV_RECORD_SPLIT_BYTES="$(NAV_RECORD_SPLIT_BYTES)" NAV_RECORD_RGBD_RATE_HZ="$(NAV_RECORD_RGBD_RATE_HZ)" LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)" NAV2_LIMIT_MODE="$(NAV2_LIMIT_MODE)"; \
		else \
			DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make navigate-inner NAVIGATE_COMMAND=navigate-record NAVIGATE_WITH_RVIZ=false NAVIGATE_WITH_MUJOCO=false SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" NAVIGATE_DURATION="$(NAVIGATE_DURATION)" CONFIG="$(CONFIG)" RUN_ROOT="$(RUN_ROOT)" NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD="$(NAVIGATE_DASHBOARD)" DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)" NAVIGATE_LAUNCH_ARGS="$(NAVIGATE_LAUNCH_ARGS)" NAVIGATE_CAPTURE_SCHEMA="$(NAV_RECORD_SCHEMA)" NAVIGATE_CAPTURE_LAUNCH_ARGS="camera_enabled:=true camera_rate_hz:=$(NAV_RECORD_RGBD_RATE_HZ) depth_only:=false" NAV_RECORD_STORAGE="$(NAV_RECORD_STORAGE)" NAV_RECORD_SPLIT_BYTES="$(NAV_RECORD_SPLIT_BYTES)" NAV_RECORD_RGBD_RATE_HZ="$(NAV_RECORD_RGBD_RATE_HZ)" LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)" NAV2_LIMIT_MODE="$(NAV2_LIMIT_MODE)"; \
		fi

navigate-view: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
			$(MAKE) navigate-inner NAVIGATE_WITH_RVIZ=true NAVIGATE_WITH_MUJOCO=false NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD="$(NAVIGATE_DASHBOARD)" DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)" NAVIGATE_LAUNCH_ARGS="$(NAVIGATE_LAUNCH_ARGS)" LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)" NAV2_LIMIT_MODE="$(NAV2_LIMIT_MODE)"; \
		else \
			DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run_gui.sh make navigate-inner NAVIGATE_WITH_RVIZ=true NAVIGATE_WITH_MUJOCO=false SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" NAVIGATE_DURATION="$(NAVIGATE_DURATION)" CONFIG="$(CONFIG)" RUN_ROOT="$(RUN_ROOT)" NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD="$(NAVIGATE_DASHBOARD)" DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)" NAVIGATE_LAUNCH_ARGS="$(NAVIGATE_LAUNCH_ARGS)" LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)" NAV2_LIMIT_MODE="$(NAV2_LIMIT_MODE)"; \
		fi

navigate-full-view: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
			$(MAKE) navigate-inner NAVIGATE_WITH_RVIZ=true NAVIGATE_WITH_MUJOCO=true NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD="$(NAVIGATE_DASHBOARD)" DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)" NAVIGATE_LAUNCH_ARGS="$(NAVIGATE_LAUNCH_ARGS)" LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)" NAV2_LIMIT_MODE="$(NAV2_LIMIT_MODE)"; \
		else \
			DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run_gui.sh make navigate-inner NAVIGATE_WITH_RVIZ=true NAVIGATE_WITH_MUJOCO=true SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" NAVIGATE_DURATION="$(NAVIGATE_DURATION)" CONFIG="$(CONFIG)" RUN_ROOT="$(RUN_ROOT)" NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD="$(NAVIGATE_DASHBOARD)" DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)" NAVIGATE_LAUNCH_ARGS="$(NAVIGATE_LAUNCH_ARGS)" LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)" NAV2_LIMIT_MODE="$(NAV2_LIMIT_MODE)"; \
		fi

demo: storage-check
	@$(MAKE) navigate-full-view SEED="$(SEED)" CELL_SIZE_M="$(DEMO_CELL_SIZE_M)" NAVIGATE_DURATION="$(NAVIGATE_DURATION)" CONFIG="$(CONFIG)" RUN_ROOT="$(RUN_ROOT)" NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD=true DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)" LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)" NAV2_LIMIT_MODE="$(NAV2_LIMIT_MODE)"

navigate-inner: storage-check
	@test "$${ROS_DISTRO:-}" = "humble" || (echo "navigate-inner requires ROS 2 Humble" && exit 1)
	@run_dir=$$(python3 scripts/create_run_context.py --command "$(NAVIGATE_COMMAND)" --seed "$(SEED)" --root "$(RUN_ROOT)" --config "$(CONFIG)" --parameter cell_size="$(CELL_SIZE_M)m" --parameter duration="$(NAVIGATE_DURATION)s"); \
	echo "Run directory: $$run_dir"; \
	if [ "$(NAVIGATE_DASHBOARD)" = "true" ]; then echo "Live KPI dashboard requested; the URL prints after the monitor HTTP server binds."; fi; \
	if [ "$(NAVIGATE_SKIP_BUILD)" = "true" ]; then echo "Skipping prebuild; using existing ros_ws/install."; elif [ "$(NAVIGATE_SKIP_BUILD)" = "auto" ] && ! ROS_PREBUILD_CHECK_SOURCES=false bash scripts/ros_prebuild_needed.sh "$(ROS_PREBUILD_STAMP)"; then echo "ROS workspace install is present; skipping prebuild."; else $(MAKE) prebuild-inner; fi; \
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/characterize_nav_locomotion.py --output-dir "$$run_dir" --config "$(CONFIG)" --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)" $(if $(LOCOMOTION_CALIBRATION),--calibration "$(LOCOMOTION_CALIBRATION)",); \
	PYTHONPATH="$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/render_navigation_config.py --config "$(CONFIG)" --nav2-template ros_ws/src/g1_nav_bringup/config/nav2_exploration_params.yaml --calibration "$$run_dir/locomotion_calibration.json" --output-dir "$$run_dir" --cell-size-m "$(CELL_SIZE_M)" --limit-mode "$(NAV2_LIMIT_MODE)"; \
	. ros_ws/install/setup.sh; \
	torch_dir=$$(dirname "$$(ls "$(VENV)"/lib/python*/site-packages/torch/lib/libgomp.so.1 2>/dev/null | head -1)"); \
	torch_preload=""; if [ -f "$$torch_dir/libgomp.so.1" ] && [ -f "$$torch_dir/libc10.so" ]; then torch_preload="$$torch_dir/libgomp.so.1:$$torch_dir/libc10.so"; fi; \
	bag_path="$(abspath .)/$$run_dir/rosbag"; bag_log="$(abspath .)/$$run_dir/rosbag-record.log"; \
	if [ -n "$(NAVIGATE_CAPTURE_SCHEMA)" ]; then \
		schema_path="$(abspath $(NAVIGATE_CAPTURE_SCHEMA))"; \
		PYTHONPATH="$(CURDIR):$$PYTHONPATH" python3 scripts/navigation_capture_artifacts.py prepare --run-dir "$$run_dir" --schema "$$schema_path" --bag-path "$$bag_path" --storage "$(NAV_RECORD_STORAGE)" --split-size-bytes "$(NAV_RECORD_SPLIT_BYTES)" --rgbd-rate-hz "$(NAV_RECORD_RGBD_RATE_HZ)"; \
		capture_topics=$$(PYTHONPATH="$(CURDIR):$$PYTHONPATH" python3 scripts/navigation_capture_artifacts.py topics --schema "$$schema_path"); \
		test -n "$$capture_topics" || (echo "capture schema produced no topics" && exit 1); \
		echo "Recording dataset ROS topics: $$bag_path"; \
		ros2 bag record -s "$(NAV_RECORD_STORAGE)" -b "$(NAV_RECORD_SPLIT_BYTES)" --output "$$bag_path" $$capture_topics >"$$bag_log" 2>&1 & bag_pid=$$!; \
	else \
		echo "Recording all ROS topics: $$bag_path"; \
		ros2 bag record --all --include-hidden-topics --output "$$bag_path" >"$$bag_log" 2>&1 & bag_pid=$$!; \
	fi; \
	cleanup_bag() { if [ -n "$$bag_pid" ] && kill -0 "$$bag_pid" 2>/dev/null; then kill -INT "$$bag_pid"; wait "$$bag_pid" || true; fi; bag_pid=""; }; \
	trap cleanup_bag EXIT INT TERM; \
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" LD_PRELOAD="$${torch_preload}$${LD_PRELOAD:+:$$LD_PRELOAD}" ros2 launch g1_nav_bringup navigate_d435i.launch.py seed:="$(SEED)" duration_s:="$(NAVIGATE_DURATION)" output_dir:="$(abspath .)/$$run_dir" config_path:="$(abspath .)/$$run_dir/resolved_config.yaml" nav2_params_file:="$(abspath .)/$$run_dir/resolved_nav2_params.yaml" corridor_width_m:="$(CELL_SIZE_M)" unitree_rl_gym_repo:="$(abspath $(UNITREE_RL_GYM_REPO))" locomotion_policy:=unitree_rl_gym_native with_rviz:="$(NAVIGATE_WITH_RVIZ)" mujoco_viewer:="$(NAVIGATE_WITH_MUJOCO)" dashboard:="$(NAVIGATE_DASHBOARD)" dashboard_port:="$(DASHBOARD_PORT)" dashboard_auto_open:="$(DASHBOARD_AUTO_OPEN)" $(NAVIGATE_LAUNCH_ARGS) $(NAVIGATE_CAPTURE_LAUNCH_ARGS); \
	status=$$?; cleanup_bag; trap - EXIT INT TERM; if [ -n "$(NAVIGATE_CAPTURE_SCHEMA)" ]; then PYTHONPATH="$(CURDIR):$$PYTHONPATH" python3 scripts/navigation_capture_artifacts.py finalize --run-dir "$$run_dir" --exit-code "$$status" || echo "capture finalization failed for $$run_dir"; fi; python3 scripts/finalize_run_context.py "$$run_dir"; echo "Report: $$run_dir/dashboard.html"; exit $$status

odom-tune: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
		python3 scripts/tune_navigation_odometry.py --run-root "$(ODOM_TUNE_RUN_ROOT)" --cell-size-m "$(CELL_SIZE_M)" --duration "$(ODOM_TUNE_DURATION)" --config "$(CONFIG)" --seeds $(ODOM_TUNE_SEEDS) $(if $(filter true,$(ODOM_TUNE_SKIP_PREBUILD)),--skip-prebuild,); \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make odom-tune ODOM_TUNE_RUN_ROOT="$(ODOM_TUNE_RUN_ROOT)" CELL_SIZE_M="$(CELL_SIZE_M)" ODOM_TUNE_DURATION="$(ODOM_TUNE_DURATION)" CONFIG="$(CONFIG)" ODOM_TUNE_SEEDS="$(ODOM_TUNE_SEEDS)" ODOM_TUNE_SKIP_PREBUILD="$(ODOM_TUNE_SKIP_PREBUILD)"; \
	fi

locomotion-calibrate: storage-check fetch-unitree-rl-gym-policy install-torch-cpu
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/run_g1_locomotion_calibration.py --seed "$(SEED)" --config "$(CONFIG)" --calibration-config "$(CALIBRATION_CONFIG)" --run-root "$(CALIBRATION_RUN_ROOT)" --profile balanced --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)"

locomotion-calibrate-smoke: storage-check fetch-unitree-rl-gym-policy install-torch-cpu
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/run_g1_locomotion_calibration.py --seed "$(SEED)" --config "$(CONFIG)" --calibration-config "$(CALIBRATION_CONFIG)" --run-root "$(CALIBRATION_RUN_ROOT)" --profile smoke --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)"

locomotion-calibrate-batch: storage-check fetch-unitree-rl-gym-policy install-torch-cpu
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/run_g1_locomotion_calibration_seed_batch.py --seed "$(SEED)" $(if $(CALIBRATION_BATCH_SEEDS),--seeds $(CALIBRATION_BATCH_SEEDS),--count "$(CALIBRATION_BATCH_COUNT)") --config "$(CONFIG)" --calibration-config "$(CALIBRATION_CONFIG)" --run-root "$(CALIBRATION_RUN_ROOT)" --profile "$(CALIBRATION_BATCH_PROFILE)" --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)" --friction-scale-min "$(CALIBRATION_BATCH_FRICTION_MIN)" --friction-scale-max "$(CALIBRATION_BATCH_FRICTION_MAX)" --min-stable-rate "$(CALIBRATION_BATCH_MIN_STABLE_RATE)" --max-fall-count "$(CALIBRATION_BATCH_MAX_FALL_COUNT)" --max-stuck-count "$(CALIBRATION_BATCH_MAX_STUCK_COUNT)" --max-non-floor-contact-count "$(CALIBRATION_BATCH_MAX_NON_FLOOR_CONTACT_COUNT)" --min-safe-vx "$(CALIBRATION_BATCH_MIN_SAFE_VX)" --min-safe-wz "$(CALIBRATION_BATCH_MIN_SAFE_WZ)"

locomotion-calibrate-batch-smoke: storage-check fetch-unitree-rl-gym-policy install-torch-cpu
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/run_g1_locomotion_calibration_seed_batch.py --seed "$(SEED)" --count 3 --config "$(CONFIG)" --calibration-config "$(CALIBRATION_CONFIG)" --run-root "$(CALIBRATION_RUN_ROOT)" --profile smoke --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)" --friction-scale-min 1.0 --friction-scale-max 1.0 --min-stable-rate 0.0 --max-fall-count 999 --max-stuck-count 999 --max-non-floor-contact-count 999 --min-safe-vx 0.0 --min-safe-wz 0.0

rl-train: storage-check fetch-unitree-rl-gym-policy install-rl-deps
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/train_maze_velocity_policy.py --config "$(CONFIG)" --rl-config "$(RL_CONFIG)" --run-root "$(RL_RUN_ROOT)" --seed "$(SEED)" --num-envs "$(RL_NUM_ENVS)" --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)" $(if $(RL_TIMESTEPS),--total-timesteps "$(RL_TIMESTEPS)",) $(if $(RL_STAGE),--stage "$(RL_STAGE)",)

rl-eval: storage-check fetch-unitree-rl-gym-policy install-rl-deps
	@test -n "$(CHECKPOINT)" || (echo "Usage: make rl-eval CHECKPOINT=runs/rl_velocity/train/.../final_model.zip" && exit 1)
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/evaluate_maze_velocity_policy.py --checkpoint "$(CHECKPOINT)" --config "$(CONFIG)" --rl-config "$(RL_CONFIG)" --run-root "$(RL_RUN_ROOT)" --seed "$(SEED)" --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)" $(if $(RL_EVAL_EPISODES),--episodes "$(RL_EVAL_EPISODES)",) $(if $(RL_EVAL_SUITE),--episode-suite "$(RL_EVAL_SUITE)",) $(if $(VEC_NORMALIZE),--vec-normalize "$(VEC_NORMALIZE)",) $(if $(RL_STAGE),--stage "$(RL_STAGE)",) $(if $(LOCOMOTION_CALIBRATION),--locomotion-calibration "$(LOCOMOTION_CALIBRATION)",)

rl-eval-corridor-sweep:
	@$(MAKE) rl-eval CHECKPOINT="$(CHECKPOINT)" VEC_NORMALIZE="$(VEC_NORMALIZE)" RL_EVAL_SUITE="configs/rl_velocity_eval_corridor_sweep_100.yaml" RL_EVAL_EPISODES=100 SEED="$(SEED)" CONFIG="$(CONFIG)" RL_CONFIG="$(RL_CONFIG)" RL_RUN_ROOT="$(RL_RUN_ROOT)" UNITREE_RL_GYM_REPO="$(UNITREE_RL_GYM_REPO)" LOCOMOTION_CALIBRATION="$(LOCOMOTION_CALIBRATION)"

rl-replay: storage-check fetch-unitree-rl-gym-policy install-rl-deps
	@test -n "$(CHECKPOINT)" || (echo "Usage: make rl-replay CHECKPOINT=runs/rl_velocity/train/.../final_model.zip SEED=123" && exit 1)
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/replay_maze_velocity_policy.py --checkpoint "$(CHECKPOINT)" --config "$(CONFIG)" --rl-config "$(RL_CONFIG)" --run-root "$(RL_RUN_ROOT)" --seed "$(SEED)" --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)" $(if $(VEC_NORMALIZE),--vec-normalize "$(VEC_NORMALIZE)",) $(if $(RL_STAGE),--stage "$(RL_STAGE)",) $(if $(LOCOMOTION_CALIBRATION),--locomotion-calibration "$(LOCOMOTION_CALIBRATION)",)

heldout-navigate: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
		"$(VENV_PYTHON)" scripts/run_navigation_seed_batch.py --run-root "$(HELDOUT_RUN_ROOT)" --label "$(HELDOUT_LABEL)" --cell-size-m "$(CELL_SIZE_M)" --duration "$(NAVIGATE_DURATION)" --config "$(CONFIG)" --jobs "$(HELDOUT_JOBS)" --base-ros-domain-id "$(HELDOUT_BASE_ROS_DOMAIN_ID)" --seeds $(HELDOUT_SEEDS); \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(HELDOUT_BASE_ROS_DOMAIN_ID)" docker/run.sh python3 scripts/run_navigation_seed_batch.py --run-root "$(HELDOUT_RUN_ROOT)" --label "$(HELDOUT_LABEL)" --cell-size-m "$(CELL_SIZE_M)" --duration "$(NAVIGATE_DURATION)" --config "$(CONFIG)" --jobs "$(HELDOUT_JOBS)" --base-ros-domain-id "$(HELDOUT_BASE_ROS_DOMAIN_ID)" --seeds $(HELDOUT_SEEDS); \
	fi

heldout-report:
	python3 scripts/aggregate_navigation_seeds.py --root "$(HELDOUT_RUN_ROOT)/navigate" --label "$(HELDOUT_LABEL)" --seeds $(HELDOUT_SEEDS) --output-json "$(HELDOUT_RUN_ROOT)/heldout_summary.json" --output-csv "$(HELDOUT_RUN_ROOT)/heldout_summary.csv" --output-html "$(HELDOUT_RUN_ROOT)/heldout_summary.html" $(if $(SEEN_NAV_ROOT),--compare-root "$(SEEN_NAV_ROOT)" --compare-label seen,)

bag-info:
	@test -n "$(BAG)" || (echo "Usage: make bag-info BAG=runs/.../rosbag" && exit 1)
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then ros2 bag info "$(BAG)"; else DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh ros2 bag info "$(BAG)"; fi

repair-run: storage-check
	@test -n "$(RUN_DIR)" || (echo "Usage: make repair-run RUN_DIR=runs/navigate-record/seed-.../..." && exit 1)
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
		PYTHONPATH="$(CURDIR):$$PYTHONPATH" python3 scripts/navigation_capture_artifacts.py repair --run-dir "$(RUN_DIR)"; \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh python3 scripts/navigation_capture_artifacts.py repair --run-dir "$(RUN_DIR)"; \
	fi

clean:
	rm -rf ros_ws/build ros_ws/install ros_ws/log
