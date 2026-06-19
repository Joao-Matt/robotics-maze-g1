#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-${DOCKER_IMAGE:-robotics-maze-g1:humble}}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
DISPLAY_VALUE="${DISPLAY:-}"
CONTAINER_HOME="${CONTAINER_HOME:-/tmp}"
CONTAINER_VENV="${CONTAINER_VENV:-/usr}"
MUJOCO_GL_VALUE="${MUJOCO_GL:-glfw}"
DOCKER_TTY_ARGS=()

mkdir -p "${REPO_ROOT}/.tmp"

if [ -t 0 ] && [ -t 1 ]; then
  DOCKER_TTY_ARGS=(-it)
fi

if [ "$#" -eq 0 ]; then
  set -- bash
fi

if [ -z "${DISPLAY_VALUE}" ]; then
  echo "DISPLAY is not set; GUI Docker runs require an active X11 display. Use docker/run.sh for headless work." >&2
  exit 1
fi

if [ ! -d /tmp/.X11-unix ]; then
  echo "/tmp/.X11-unix is unavailable; GUI Docker runs require an active X11 socket. Use docker/run.sh for headless work." >&2
  exit 1
fi

if command -v xhost >/dev/null 2>&1; then
  xhost +local:docker >/dev/null 2>&1 || true
fi

DOCKER_ARGS=(
  --rm
  "${DOCKER_TTY_ARGS[@]}"
  --network host
  --user "$(id -u):$(id -g)"
  -e "HOME=${CONTAINER_HOME}"
  -e "USER=${USER:-developer}"
  -e "VENV=${CONTAINER_VENV}"
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  -e "MUJOCO_GL=${MUJOCO_GL_VALUE}"
  -e TMPDIR=/workspace/.tmp
  -e PYTHONUNBUFFERED=1
  -e "DISPLAY=${DISPLAY_VALUE}"
  -e QT_X11_NO_MITSHM=1
  -v /etc/passwd:/etc/passwd:ro
  -v /etc/group:/etc/group:ro
  -v "${REPO_ROOT}:/workspace"
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  -w /workspace
)

if [ -d /dev/dri ]; then
  DOCKER_ARGS+=(--device /dev/dri)
fi

docker run "${DOCKER_ARGS[@]}" "${IMAGE_NAME}" "$@"
