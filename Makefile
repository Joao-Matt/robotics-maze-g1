SHELL := /bin/bash

CONFIG ?= configs/default.yaml
SEED ?= 123
CELL_SIZE_M ?= $(if $(CORRIDOR_WIDTH),$(CORRIDOR_WIDTH),4.0)
CELL_SIZE_MIN ?= 1.0
CELL_SIZE_MAX ?= 4.0
RUN_ROOT ?= runs
VISUAL_DIR ?= runs/visual
ORACLE_DURATION ?= 300
SLAM_DURATION ?= 300
NAVIGATE_DURATION ?= 600
ROS_BRIDGE_PORT ?= 8765
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
NAVIGATE_SKIP_BUILD ?= false
NAVIGATE_WITH_RVIZ ?= false
NAVIGATE_WITH_MUJOCO ?= false
SLAM_WITH_RVIZ ?= false

export TMPDIR := $(abspath $(PROJECT_TMP))

-include .env.storage
export REQUIRED_STORAGE_MOUNT
export EXPECTED_STORAGE_UUID

.PHONY: help storage-check setup install-torch-cpu docker-build docker-run docker-run-gui docker-check-ros docker-build-multiarch
.PHONY: fetch-unitree-rl-gym-policy fetch-m-explore prebuild prebuild-inner maze world oracle oracle-view oracle-inner slam slam-view slam-inner navigate navigate-view navigate-full-view navigate-inner bag-info clean

help:
	@printf '%s\n' \
		'Production targets:' \
		'  make docker-build' \
		'  make docker-run                      # headless shell with ROS Humble' \
		'  make docker-run-gui                  # GUI shell for RViz/MuJoCo viewer' \
		'  make prebuild                        # fetch Unitree RL Gym + m-explore and build ROS workspace' \
		'  make maze CELL_SIZE_M=4.0 SEED=123   # generate/validate square grid, 1-4 m cells' \
		'  make world CELL_SIZE_M=4.0 SEED=123  # generate MuJoCo world with G1 + D435i + laser source' \
		'  make oracle SEED=123                 # Unitree RL Gym native oracle path following' \
		'  make oracle-view SEED=123            # oracle path following with MuJoCo viewer' \
		'  make slam SEED=123                   # oracle-driven SLAM with rosbag' \
		'  make slam-view SEED=123              # SLAM with RViz' \
		'  make navigate SEED=123               # SLAM + m-explore + Nav2 + rosbag' \
		'  make navigate-view SEED=123          # navigation with RViz' \
		'  make navigate-full-view SEED=123     # navigation with RViz + MuJoCo viewer'

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
	@if git -C "$(M_EXPLORE_REPO)" apply --reverse --check "$(abspath patches/m-explore-ros2-humble-latest-tf.patch)" >/dev/null 2>&1; then :; else \
		git -C "$(M_EXPLORE_REPO)" apply "$(abspath patches/m-explore-ros2-humble-latest-tf.patch)"; \
	fi

prebuild: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
		$(MAKE) prebuild-inner; \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make prebuild-inner; \
	fi

prebuild-inner: storage-check install-torch-cpu fetch-unitree-rl-gym-policy fetch-m-explore
	@test "$${ROS_DISTRO:-}" = "humble" || (echo "prebuild-inner requires ROS 2 Humble" && exit 1)
	@rm -rf ros_ws/build/explore_lite_msgs ros_ws/build/explore_lite ros_ws/build/g1_mujoco_bridge ros_ws/build/g1_nav_bringup \
		ros_ws/install/explore_lite_msgs ros_ws/install/explore_lite ros_ws/install/g1_mujoco_bridge ros_ws/install/g1_nav_bringup
	PYTHONPATH="$(CURDIR):$$PYTHONPATH" colcon --log-base ros_ws/log build --base-paths ros_ws/src "$(M_EXPLORE_REPO)/explore" "$(M_EXPLORE_REPO)/explore_lite_msgs" --build-base ros_ws/build --install-base ros_ws/install --symlink-install --packages-select explore_lite_msgs explore_lite g1_mujoco_bridge g1_nav_bringup

maze: storage-check
	@mkdir -p "$(VISUAL_DIR)"
	"$(VENV_PYTHON)" scripts/generate_maze.py --seed "$(SEED)" --config "$(CONFIG)" --cell-size-m "$(CELL_SIZE_M)" --show-path --save-ascii "$(VISUAL_DIR)/maze_seed-$(SEED).txt" --save-pgm "$(VISUAL_DIR)/maze_seed-$(SEED).pgm" --save-svg "$(VISUAL_DIR)/maze_seed-$(SEED).svg"

world: storage-check fetch-unitree-rl-gym-policy
	@mkdir -p "$(VISUAL_DIR)"
	"$(VENV_PYTHON)" scripts/generate_world.py --seed "$(SEED)" --config "$(CONFIG)" --cell-size-m "$(CELL_SIZE_M)" --output-dir "$(VISUAL_DIR)"

oracle: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ] || [ "$(VENV)" != ".venv" ]; then \
		$(MAKE) oracle-inner ORACLE_VIEWER=false; \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make oracle-inner ORACLE_VIEWER=false SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" ORACLE_DURATION="$(ORACLE_DURATION)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)"; \
	fi

oracle-view: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ] || [ "$(VENV)" != ".venv" ]; then \
		$(MAKE) oracle-inner ORACLE_VIEWER=true; \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run_gui.sh make oracle-inner ORACLE_VIEWER=true SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" ORACLE_DURATION="$(ORACLE_DURATION)" CONFIG="$(CONFIG)" VISUAL_DIR="$(VISUAL_DIR)"; \
	fi

oracle-inner: storage-check install-torch-cpu fetch-unitree-rl-gym-policy
	@mkdir -p "$(VISUAL_DIR)"
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/run_g1_oracle_follow.py --seed "$(SEED)" --duration "$(ORACLE_DURATION)" --corridor-width-m "$(CELL_SIZE_M)" --config "$(CONFIG)" --output-dir "$(VISUAL_DIR)" --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)" --locomotion-policy unitree_rl_gym_native $(if $(filter true,$(ORACLE_VIEWER)),--viewer,)

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
		$(MAKE) navigate-inner NAVIGATE_WITH_RVIZ=false NAVIGATE_WITH_MUJOCO=false; \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make navigate-inner NAVIGATE_WITH_RVIZ=false NAVIGATE_WITH_MUJOCO=false SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" NAVIGATE_DURATION="$(NAVIGATE_DURATION)" CONFIG="$(CONFIG)" RUN_ROOT="$(RUN_ROOT)"; \
	fi

navigate-view: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
		$(MAKE) navigate-inner NAVIGATE_WITH_RVIZ=true NAVIGATE_WITH_MUJOCO=false; \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run_gui.sh make navigate-inner NAVIGATE_WITH_RVIZ=true NAVIGATE_WITH_MUJOCO=false SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" NAVIGATE_DURATION="$(NAVIGATE_DURATION)" CONFIG="$(CONFIG)" RUN_ROOT="$(RUN_ROOT)"; \
	fi

navigate-full-view: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
		$(MAKE) navigate-inner NAVIGATE_WITH_RVIZ=true NAVIGATE_WITH_MUJOCO=true; \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run_gui.sh make navigate-inner NAVIGATE_WITH_RVIZ=true NAVIGATE_WITH_MUJOCO=true SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" NAVIGATE_DURATION="$(NAVIGATE_DURATION)" CONFIG="$(CONFIG)" RUN_ROOT="$(RUN_ROOT)"; \
	fi

navigate-inner: storage-check
	@test "$${ROS_DISTRO:-}" = "humble" || (echo "navigate-inner requires ROS 2 Humble" && exit 1)
	@run_dir=$$(python3 scripts/create_run_context.py --command navigate --seed "$(SEED)" --root "$(RUN_ROOT)" --config "$(CONFIG)" --parameter cell_size="$(CELL_SIZE_M)m" --parameter duration="$(NAVIGATE_DURATION)s"); \
	echo "Run directory: $$run_dir"; \
	$(MAKE) prebuild-inner NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)"; \
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/characterize_nav_locomotion.py --output-dir "$$run_dir" --config "$(CONFIG)" --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)"; \
	PYTHONPATH="$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/render_navigation_config.py --config "$(CONFIG)" --nav2-template ros_ws/src/g1_nav_bringup/config/nav2_exploration_params.yaml --calibration "$$run_dir/locomotion_calibration.json" --output-dir "$$run_dir"; \
	. ros_ws/install/setup.sh; \
	torch_dir=$$(dirname "$$(ls "$(VENV)"/lib/python*/site-packages/torch/lib/libgomp.so.1 2>/dev/null | head -1)"); \
	torch_preload=""; if [ -f "$$torch_dir/libgomp.so.1" ] && [ -f "$$torch_dir/libc10.so" ]; then torch_preload="$$torch_dir/libgomp.so.1:$$torch_dir/libc10.so"; fi; \
	bag_path="$(abspath .)/$$run_dir/rosbag"; bag_log="$(abspath .)/$$run_dir/rosbag-record.log"; \
	echo "Recording all ROS topics: $$bag_path"; \
	ros2 bag record --all --include-hidden-topics --output "$$bag_path" >"$$bag_log" 2>&1 & bag_pid=$$!; \
	cleanup_bag() { if [ -n "$$bag_pid" ] && kill -0 "$$bag_pid" 2>/dev/null; then kill -INT "$$bag_pid"; wait "$$bag_pid" || true; fi; bag_pid=""; }; \
	trap cleanup_bag EXIT INT TERM; \
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" LD_PRELOAD="$${torch_preload}$${LD_PRELOAD:+:$$LD_PRELOAD}" ros2 launch g1_nav_bringup navigate_d435i.launch.py seed:="$(SEED)" duration_s:="$(NAVIGATE_DURATION)" output_dir:="$(abspath .)/$$run_dir" config_path:="$(abspath .)/$$run_dir/resolved_config.yaml" nav2_params_file:="$(abspath .)/$$run_dir/resolved_nav2_params.yaml" corridor_width_m:="$(CELL_SIZE_M)" unitree_rl_gym_repo:="$(abspath $(UNITREE_RL_GYM_REPO))" locomotion_policy:=unitree_rl_gym_native with_rviz:="$(NAVIGATE_WITH_RVIZ)" mujoco_viewer:="$(NAVIGATE_WITH_MUJOCO)"; \
	status=$$?; cleanup_bag; trap - EXIT INT TERM; python3 scripts/finalize_run_context.py "$$run_dir"; echo "Report: $$run_dir/dashboard.html"; exit $$status

bag-info:
	@test -n "$(BAG)" || (echo "Usage: make bag-info BAG=runs/.../rosbag" && exit 1)
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then ros2 bag info "$(BAG)"; else DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh ros2 bag info "$(BAG)"; fi

clean:
	rm -rf ros_ws/build ros_ws/install ros_ws/log
