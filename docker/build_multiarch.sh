#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-${DOCKER_IMAGE:-robotics-maze-g1:humble}}"
PLATFORMS="${PLATFORMS:-${DOCKER_PLATFORMS:-linux/amd64,linux/arm64}}"
PUSH="${PUSH:-0}"

if ! docker buildx version >/dev/null 2>&1; then
  echo "Docker Buildx is required for multi-architecture builds." >&2
  exit 1
fi

OUTPUT_ARGS=()
if [ "${PUSH}" = "1" ]; then
  OUTPUT_ARGS+=(--push)
else
  if [[ "${PLATFORMS}" == *,* ]]; then
    echo "Note: Docker cannot --load a true multi-platform manifest into the local image store." >&2
    echo "For linux/amd64,linux/arm64 together, use PUSH=1 IMAGE_NAME=<registry/image:tag> ${BASH_SOURCE[0]}." >&2
    echo "For a local test build, set PLATFORMS to a single platform such as linux/amd64." >&2
    echo "Building without --push/--load; results remain in the Buildx cache." >&2
  else
    OUTPUT_ARGS+=(--load)
  fi
fi

docker buildx build \
  --platform "${PLATFORMS}" \
  -t "${IMAGE_NAME}" \
  -f "${REPO_ROOT}/docker/Dockerfile" \
  "${OUTPUT_ARGS[@]}" \
  "${REPO_ROOT}"
