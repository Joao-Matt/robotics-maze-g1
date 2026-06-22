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
SLAM_DURATION ?= 300
NAVIGATE_DURATION ?= 600
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
NAVIGATE_SKIP_BUILD ?= false
NAVIGATE_WITH_RVIZ ?= false
NAVIGATE_WITH_MUJOCO ?= false
NAVIGATE_DASHBOARD ?= true
DASHBOARD_AUTO_OPEN ?= true
NAVIGATE_LAUNCH_ARGS ?=
ODOM_TUNE_RUN_ROOT ?= runs/odom_tuning
ODOM_TUNE_DURATION ?= 240
ODOM_TUNE_SEEDS ?= 123 81
ODOM_TUNE_SKIP_PREBUILD ?= false
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
.PHONY: fetch-unitree-rl-gym-policy fetch-m-explore prebuild prebuild-inner maze world oracle oracle-view oracle-inner slam slam-view slam-inner navigate navigate-view navigate-full-view demo navigate-inner heldout-navigate heldout-report bag-info clean
.PHONY: odom-tune rl-train rl-eval rl-eval-corridor-sweep rl-replay

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
		'  make slam SEED=123                   # oracle-driven SLAM with rosbag' \
		'  make slam-view SEED=123              # SLAM with RViz' \
		'  make navigate SEED=123               # SLAM + m-explore + Nav2 + rosbag' \
		'  make odom-tune                       # sweep scan-odom params against offline ground-truth metrics' \
		'  make navigate-view SEED=123          # navigation with RViz' \
		'  make navigate-full-view SEED=123     # navigation with RViz + MuJoCo viewer' \
		'  make demo SEED=123 DEMO_CELL_SIZE_M=2.0 # interview demo with full-view navigation + live KPI dashboard' \
		'  make rl-train                        # direct MuJoCo PPO velocity-controller training' \
		'  make rl-eval CHECKPOINT=...          # evaluate a trained PPO checkpoint' \
		'  make rl-eval-corridor-sweep CHECKPOINT=... # 100 random mazes across 2-4 m corridors' \
		'  make rl-replay CHECKPOINT=... SEED=123 # replay one trained PPO episode' \
		'  make heldout-navigate                # run the fixed 20-seed headless held-out batch' \
		'  make heldout-report                  # aggregate held-out solve rate with 95% CI'

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
		$(MAKE) navigate-inner NAVIGATE_WITH_RVIZ=false NAVIGATE_WITH_MUJOCO=false NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD="$(NAVIGATE_DASHBOARD)" DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)" NAVIGATE_LAUNCH_ARGS="$(NAVIGATE_LAUNCH_ARGS)"; \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make navigate-inner NAVIGATE_WITH_RVIZ=false NAVIGATE_WITH_MUJOCO=false SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" NAVIGATE_DURATION="$(NAVIGATE_DURATION)" CONFIG="$(CONFIG)" RUN_ROOT="$(RUN_ROOT)" NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD="$(NAVIGATE_DASHBOARD)" DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)" NAVIGATE_LAUNCH_ARGS="$(NAVIGATE_LAUNCH_ARGS)"; \
	fi

navigate-view: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
		$(MAKE) navigate-inner NAVIGATE_WITH_RVIZ=true NAVIGATE_WITH_MUJOCO=false NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD="$(NAVIGATE_DASHBOARD)" DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)" NAVIGATE_LAUNCH_ARGS="$(NAVIGATE_LAUNCH_ARGS)"; \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run_gui.sh make navigate-inner NAVIGATE_WITH_RVIZ=true NAVIGATE_WITH_MUJOCO=false SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" NAVIGATE_DURATION="$(NAVIGATE_DURATION)" CONFIG="$(CONFIG)" RUN_ROOT="$(RUN_ROOT)" NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD="$(NAVIGATE_DASHBOARD)" DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)" NAVIGATE_LAUNCH_ARGS="$(NAVIGATE_LAUNCH_ARGS)"; \
	fi

navigate-full-view: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
		$(MAKE) navigate-inner NAVIGATE_WITH_RVIZ=true NAVIGATE_WITH_MUJOCO=true NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD="$(NAVIGATE_DASHBOARD)" DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)" NAVIGATE_LAUNCH_ARGS="$(NAVIGATE_LAUNCH_ARGS)"; \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run_gui.sh make navigate-inner NAVIGATE_WITH_RVIZ=true NAVIGATE_WITH_MUJOCO=true SEED="$(SEED)" CELL_SIZE_M="$(CELL_SIZE_M)" NAVIGATE_DURATION="$(NAVIGATE_DURATION)" CONFIG="$(CONFIG)" RUN_ROOT="$(RUN_ROOT)" NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD="$(NAVIGATE_DASHBOARD)" DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)" NAVIGATE_LAUNCH_ARGS="$(NAVIGATE_LAUNCH_ARGS)"; \
	fi

demo: storage-check
	@$(MAKE) navigate-full-view SEED="$(SEED)" CELL_SIZE_M="$(DEMO_CELL_SIZE_M)" NAVIGATE_DURATION="$(NAVIGATE_DURATION)" CONFIG="$(CONFIG)" RUN_ROOT="$(RUN_ROOT)" NAVIGATE_SKIP_BUILD="$(NAVIGATE_SKIP_BUILD)" NAVIGATE_DASHBOARD=true DASHBOARD_PORT="$(DASHBOARD_PORT)" DASHBOARD_AUTO_OPEN="$(DASHBOARD_AUTO_OPEN)"

navigate-inner: storage-check
	@test "$${ROS_DISTRO:-}" = "humble" || (echo "navigate-inner requires ROS 2 Humble" && exit 1)
	@run_dir=$$(python3 scripts/create_run_context.py --command navigate --seed "$(SEED)" --root "$(RUN_ROOT)" --config "$(CONFIG)" --parameter cell_size="$(CELL_SIZE_M)m" --parameter duration="$(NAVIGATE_DURATION)s"); \
	echo "Run directory: $$run_dir"; \
	if [ "$(NAVIGATE_DASHBOARD)" = "true" ]; then echo "Live KPI dashboard requested; the URL prints after the monitor HTTP server binds."; fi; \
	if [ "$(NAVIGATE_SKIP_BUILD)" = "true" ]; then echo "Skipping prebuild; using existing ros_ws/install."; else $(MAKE) prebuild-inner; fi; \
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/characterize_nav_locomotion.py --output-dir "$$run_dir" --config "$(CONFIG)" --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)"; \
	PYTHONPATH="$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/render_navigation_config.py --config "$(CONFIG)" --nav2-template ros_ws/src/g1_nav_bringup/config/nav2_exploration_params.yaml --calibration "$$run_dir/locomotion_calibration.json" --output-dir "$$run_dir" --cell-size-m "$(CELL_SIZE_M)"; \
	. ros_ws/install/setup.sh; \
	torch_dir=$$(dirname "$$(ls "$(VENV)"/lib/python*/site-packages/torch/lib/libgomp.so.1 2>/dev/null | head -1)"); \
	torch_preload=""; if [ -f "$$torch_dir/libgomp.so.1" ] && [ -f "$$torch_dir/libc10.so" ]; then torch_preload="$$torch_dir/libgomp.so.1:$$torch_dir/libc10.so"; fi; \
	bag_path="$(abspath .)/$$run_dir/rosbag"; bag_log="$(abspath .)/$$run_dir/rosbag-record.log"; \
	echo "Recording all ROS topics: $$bag_path"; \
	ros2 bag record --all --include-hidden-topics --output "$$bag_path" >"$$bag_log" 2>&1 & bag_pid=$$!; \
	cleanup_bag() { if [ -n "$$bag_pid" ] && kill -0 "$$bag_pid" 2>/dev/null; then kill -INT "$$bag_pid"; wait "$$bag_pid" || true; fi; bag_pid=""; }; \
	trap cleanup_bag EXIT INT TERM; \
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" LD_PRELOAD="$${torch_preload}$${LD_PRELOAD:+:$$LD_PRELOAD}" ros2 launch g1_nav_bringup navigate_d435i.launch.py seed:="$(SEED)" duration_s:="$(NAVIGATE_DURATION)" output_dir:="$(abspath .)/$$run_dir" config_path:="$(abspath .)/$$run_dir/resolved_config.yaml" nav2_params_file:="$(abspath .)/$$run_dir/resolved_nav2_params.yaml" corridor_width_m:="$(CELL_SIZE_M)" unitree_rl_gym_repo:="$(abspath $(UNITREE_RL_GYM_REPO))" locomotion_policy:=unitree_rl_gym_native with_rviz:="$(NAVIGATE_WITH_RVIZ)" mujoco_viewer:="$(NAVIGATE_WITH_MUJOCO)" dashboard:="$(NAVIGATE_DASHBOARD)" dashboard_port:="$(DASHBOARD_PORT)" dashboard_auto_open:="$(DASHBOARD_AUTO_OPEN)" $(NAVIGATE_LAUNCH_ARGS); \
	status=$$?; cleanup_bag; trap - EXIT INT TERM; python3 scripts/finalize_run_context.py "$$run_dir"; echo "Report: $$run_dir/dashboard.html"; exit $$status

odom-tune: storage-check
	@if [ "$${ROS_DISTRO:-}" = "humble" ]; then \
		python3 scripts/tune_navigation_odometry.py --run-root "$(ODOM_TUNE_RUN_ROOT)" --cell-size-m "$(CELL_SIZE_M)" --duration "$(ODOM_TUNE_DURATION)" --config "$(CONFIG)" --seeds $(ODOM_TUNE_SEEDS) $(if $(filter true,$(ODOM_TUNE_SKIP_PREBUILD)),--skip-prebuild,); \
	else \
		DOCKER_IMAGE="$(DOCKER_IMAGE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" docker/run.sh make odom-tune ODOM_TUNE_RUN_ROOT="$(ODOM_TUNE_RUN_ROOT)" CELL_SIZE_M="$(CELL_SIZE_M)" ODOM_TUNE_DURATION="$(ODOM_TUNE_DURATION)" CONFIG="$(CONFIG)" ODOM_TUNE_SEEDS="$(ODOM_TUNE_SEEDS)" ODOM_TUNE_SKIP_PREBUILD="$(ODOM_TUNE_SKIP_PREBUILD)"; \
	fi

rl-train: storage-check fetch-unitree-rl-gym-policy install-rl-deps
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/train_maze_velocity_policy.py --config "$(CONFIG)" --rl-config "$(RL_CONFIG)" --run-root "$(RL_RUN_ROOT)" --seed "$(SEED)" --num-envs "$(RL_NUM_ENVS)" --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)" $(if $(RL_TIMESTEPS),--total-timesteps "$(RL_TIMESTEPS)",) $(if $(RL_STAGE),--stage "$(RL_STAGE)",)

rl-eval: storage-check fetch-unitree-rl-gym-policy install-rl-deps
	@test -n "$(CHECKPOINT)" || (echo "Usage: make rl-eval CHECKPOINT=runs/rl_velocity/train/.../final_model.zip" && exit 1)
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/evaluate_maze_velocity_policy.py --checkpoint "$(CHECKPOINT)" --config "$(CONFIG)" --rl-config "$(RL_CONFIG)" --run-root "$(RL_RUN_ROOT)" --seed "$(SEED)" --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)" $(if $(RL_EVAL_EPISODES),--episodes "$(RL_EVAL_EPISODES)",) $(if $(RL_EVAL_SUITE),--episode-suite "$(RL_EVAL_SUITE)",) $(if $(VEC_NORMALIZE),--vec-normalize "$(VEC_NORMALIZE)",) $(if $(RL_STAGE),--stage "$(RL_STAGE)",)

rl-eval-corridor-sweep:
	@$(MAKE) rl-eval CHECKPOINT="$(CHECKPOINT)" VEC_NORMALIZE="$(VEC_NORMALIZE)" RL_EVAL_SUITE="configs/rl_velocity_eval_corridor_sweep_100.yaml" RL_EVAL_EPISODES=100 SEED="$(SEED)" CONFIG="$(CONFIG)" RL_CONFIG="$(RL_CONFIG)" RL_RUN_ROOT="$(RL_RUN_ROOT)" UNITREE_RL_GYM_REPO="$(UNITREE_RL_GYM_REPO)"

rl-replay: storage-check fetch-unitree-rl-gym-policy install-rl-deps
	@test -n "$(CHECKPOINT)" || (echo "Usage: make rl-replay CHECKPOINT=runs/rl_velocity/train/.../final_model.zip SEED=123" && exit 1)
	PYTHONPATH="$(abspath $(PYTHON_PACKAGE_DIR)):$(CURDIR):$$PYTHONPATH" "$(VENV_PYTHON)" scripts/replay_maze_velocity_policy.py --checkpoint "$(CHECKPOINT)" --config "$(CONFIG)" --rl-config "$(RL_CONFIG)" --run-root "$(RL_RUN_ROOT)" --seed "$(SEED)" --unitree-rl-gym-repo "$(UNITREE_RL_GYM_REPO)" $(if $(VEC_NORMALIZE),--vec-normalize "$(VEC_NORMALIZE)",) $(if $(RL_STAGE),--stage "$(RL_STAGE)",)

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

clean:
	rm -rf ros_ws/build ros_ws/install ros_ws/log
