ARG ROS_BASE_IMAGE=ros:humble-ros-base-jammy
FROM ${ROS_BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_NO_CACHE_DIR=1
ENV PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    curl \
    ffmpeg \
    git \
    libgl1 \
    libgl1-mesa-dri \
    libglew-dev \
    libglfw3 \
    libglfw3-dev \
    libosmesa6 \
    libosmesa6-dev \
    mesa-utils \
    nano \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-venv \
    tmux \
    vim \
    wget \
    xdg-utils \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-depthimage-to-laserscan \
    ros-humble-robot-localization \
    ros-humble-tf-transformations \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-rtabmap-ros \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY requirements.txt /tmp/robotics-maze-g1-requirements.txt
RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install -r /tmp/robotics-maze-g1-requirements.txt

COPY . /workspace

RUN chmod +x /workspace/docker/entrypoint.sh \
    /workspace/docker/run.sh \
    /workspace/docker/run_gui.sh \
    /workspace/docker/build_multiarch.sh \
    /workspace/scripts/check_ros_docker_env.sh

ENTRYPOINT ["/workspace/docker/entrypoint.sh"]
CMD ["bash"]
