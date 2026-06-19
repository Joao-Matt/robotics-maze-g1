#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [ -f "${REPO_ROOT}/.env.storage" ]; then
  set -a
  # shellcheck source=/dev/null
  source "${REPO_ROOT}/.env.storage"
  set +a
fi
RUNS_DIR="${RUNS_DIR:-${REPO_ROOT}/runs}"
TEMP_DIR="${TMPDIR:-${REPO_ROOT}/.tmp}"
REQUIRED_MOUNT="${REQUIRED_STORAGE_MOUNT:-}"
EXPECTED_UUID="${EXPECTED_STORAGE_UUID:-}"

fail() {
  echo "storage-check failed: $*" >&2
  exit 1
}

mkdir -p "${RUNS_DIR}" "${TEMP_DIR}"

repo_device="$(stat -c '%d' "${REPO_ROOT}")"
runs_device="$(stat -c '%d' "${RUNS_DIR}")"
root_device="$(stat -c '%d' /)"
temp_device="$(stat -c '%d' "${TEMP_DIR}")"

if [ "${repo_device}" != "${runs_device}" ]; then
  fail "runs directory is not on the repository filesystem (${RUNS_DIR})"
fi
if [ "${repo_device}" != "${temp_device}" ]; then
  fail "temporary directory is not on the repository filesystem (${TEMP_DIR})"
fi

if [ -f /.dockerenv ]; then
  if [ "${repo_device}" = "${root_device}" ]; then
    fail "repository is on the container writable layer; start it with the repository bind-mounted"
  fi
  echo "storage-check ok: container repository, runs, and temporary files use the bind-mounted filesystem"
  exit 0
fi

if [ -z "${REQUIRED_MOUNT}" ]; then
  echo "storage-check ok: repository and runs share one filesystem"
  echo "storage-check note: set REQUIRED_STORAGE_MOUNT in .env.storage for strict host validation"
  exit 0
fi

mountpoint -q "${REQUIRED_MOUNT}" || fail "required storage is not mounted: ${REQUIRED_MOUNT}"
mount_root="$(realpath "${REQUIRED_MOUNT}")"
repo_real="$(realpath "${REPO_ROOT}")"
case "${repo_real}/" in
  "${mount_root}/"*) ;;
  *) fail "repository is outside required storage (${repo_real} is not under ${mount_root})" ;;
esac

if [ -n "${EXPECTED_UUID}" ]; then
  actual_uuid="$(findmnt -T "${mount_root}" -no UUID 2>/dev/null || true)"
  [ "${actual_uuid}" = "${EXPECTED_UUID}" ] || fail \
    "storage UUID mismatch: expected ${EXPECTED_UUID}, got ${actual_uuid:-unknown}"
fi

if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
  docker_root="$(docker info --format '{{.DockerRootDir}}')"
  case "$(realpath -m "${docker_root}")/" in
    "${mount_root}/"*) ;;
    *) fail "Docker root is outside required storage: ${docker_root}" ;;
  esac
fi

if [ -r /etc/containerd/config.toml ]; then
  containerd_root="$(sed -nE 's/^[[:space:]]*root[[:space:]]*=[[:space:]]*"([^"]+)".*/\1/p' /etc/containerd/config.toml | head -1)"
  [ -n "${containerd_root}" ] || fail "containerd root is not explicitly configured"
  case "$(realpath -m "${containerd_root}")/" in
    "${mount_root}/"*) ;;
    *) fail "containerd root is outside required storage: ${containerd_root}" ;;
  esac
fi

echo "storage-check ok: repository, runs, Docker, and containerd use ${mount_root}"
