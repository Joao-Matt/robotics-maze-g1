#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-${DOCKER_IMAGE:-robotics-maze-g1:humble}}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
CONTAINER_HOME="${CONTAINER_HOME:-/tmp}"
CONTAINER_VENV="${CONTAINER_VENV:-/usr}"
MUJOCO_GL_VALUE="${MUJOCO_GL:-osmesa}"
DOCKER_TTY_ARGS=()

if [ -t 0 ] && [ -t 1 ]; then
  DOCKER_TTY_ARGS=(-it)
fi

if [ "$#" -eq 0 ]; then
  set -- bash
fi

docker run --rm "${DOCKER_TTY_ARGS[@]}" \
  --network host \
  --user "$(id -u):$(id -g)" \
  -e HOME="${CONTAINER_HOME}" \
  -e USER="${USER:-developer}" \
  -e VENV="${CONTAINER_VENV}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -e MUJOCO_GL="${MUJOCO_GL_VALUE}" \
  -e PYTHONUNBUFFERED=1 \
  -v /etc/passwd:/etc/passwd:ro \
  -v /etc/group:/etc/group:ro \
  -v "${REPO_ROOT}:/workspace" \
  -w /workspace \
  "${IMAGE_NAME}" \
  "$@"
