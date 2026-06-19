#!/usr/bin/env bash
set -euo pipefail

if [ "${EUID}" -ne 0 ]; then
  echo "Run this script with sudo from a host terminal." >&2
  exit 1
fi

if [ "$#" -ne 2 ]; then
  echo "Usage: sudo $0 <external-mount> <filesystem-uuid>" >&2
  exit 1
fi

EXTERNAL_MOUNT="$(realpath "$1")"
EXPECTED_UUID="$2"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BACKUP_DIR="${EXTERNAL_MOUNT}/system-storage-migration-${TIMESTAMP}"
DOCKER_ROOT="${EXTERNAL_MOUNT}/docker-data"
CONTAINERD_ROOT="${EXTERNAL_MOUNT}/containerd-data"
FSTAB_LINE="UUID=${EXPECTED_UUID} ${EXTERNAL_MOUNT} ext4 defaults,nofail,x-systemd.device-timeout=10 0 2"
MOUNT_UNIT="$(systemd-escape --path --suffix=mount "${EXTERNAL_MOUNT}")"

fail() {
  echo "migration failed: $*" >&2
  echo "backups/quarantines (if created): ${BACKUP_DIR}" >&2
  exit 1
}

mountpoint -q "${EXTERNAL_MOUNT}" || fail "external storage is not mounted: ${EXTERNAL_MOUNT}"
actual_uuid="$(findmnt -T "${EXTERNAL_MOUNT}" -no UUID 2>/dev/null || true)"
[ "${actual_uuid}" = "${EXPECTED_UUID}" ] || fail \
  "UUID mismatch: expected ${EXPECTED_UUID}, got ${actual_uuid:-unknown}"
[ "$(findmnt -T "${EXTERNAL_MOUNT}" -no FSTYPE)" = "ext4" ] || fail "external filesystem is not ext4"

install -d -m 0700 "${BACKUP_DIR}"
cp -a /etc/fstab "${BACKUP_DIR}/fstab.before"
cp -a /etc/docker/daemon.json "${BACKUP_DIR}/daemon.json.before"
cp -a /etc/containerd/config.toml "${BACKUP_DIR}/containerd-config.toml.before"
docker info >"${BACKUP_DIR}/docker-info.before.txt" 2>&1 || true
docker ps -a --no-trunc >"${BACKUP_DIR}/docker-ps.before.txt" 2>&1 || true
df -hT / "${EXTERNAL_MOUNT}" >"${BACKUP_DIR}/df.before.txt"

echo "Reclaiming conservative emergency space..."
find /var/lib/apport/coredump -xdev -type f -delete 2>/dev/null || true
find /var/crash -xdev -type f -delete 2>/dev/null || true
while read -r snap_name snap_revision; do
  [ -n "${snap_name}" ] || continue
  snap remove "${snap_name}" --revision="${snap_revision}"
done < <(snap list --all | awk '$NF ~ /disabled/ {print $1, $3}')
if ! apt-get clean; then
  echo "warning: APT is busy; skipping package-cache cleanup and continuing" >&2
fi

echo "Stopping running containers and storage services..."
mapfile -t running_containers < <(docker ps -q)
if [ "${#running_containers[@]}" -gt 0 ]; then
  docker stop "${running_containers[@]}"
fi
systemctl stop docker.service docker.socket
systemctl stop containerd.service

echo "Configuring persistent mount and service guards..."
if ! grep -Eq "^[^#].*[[:space:]]${EXTERNAL_MOUNT//\//\\/}[[:space:]]" /etc/fstab; then
  printf '\n%s\n' "${FSTAB_LINE}" >>/etc/fstab
fi
mount -a
mountpoint -q "${EXTERNAL_MOUNT}" || fail "mount verification failed after updating fstab"

install -d -m 0755 /etc/systemd/system/docker.service.d /etc/systemd/system/containerd.service.d
for service in docker containerd; do
  cat >"/etc/systemd/system/${service}.service.d/external-storage.conf" <<EOF
[Unit]
RequiresMountsFor=${EXTERNAL_MOUNT}
After=${MOUNT_UNIT}
ConditionPathIsMountPoint=${EXTERNAL_MOUNT}
EOF
done

cat >/etc/containerd/config.toml <<EOF
version = 3
root = "${CONTAINERD_ROOT}"
state = "/run/containerd"
disabled_plugins = []
EOF

python3 - "${DOCKER_ROOT}" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path("/etc/docker/daemon.json")
config = json.loads(path.read_text(encoding="utf-8"))
config["data-root"] = sys.argv[1]
path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
PY

echo "Quarantining old Docker and containerd stores on external storage..."
if [ -e "${DOCKER_ROOT}" ]; then
  mv "${DOCKER_ROOT}" "${BACKUP_DIR}/docker-data.old"
fi
if [ -e /var/lib/containerd ]; then
  mv /var/lib/containerd "${BACKUP_DIR}/containerd-data.old"
fi
install -d -m 0711 "${DOCKER_ROOT}" "${CONTAINERD_ROOT}"

systemctl daemon-reload
systemctl start containerd.service
systemctl start docker.socket docker.service

[ "$(docker info --format '{{.DockerRootDir}}')" = "${DOCKER_ROOT}" ] || fail "Docker root verification failed"
configured_containerd_root="$(sed -nE 's/^root[[:space:]]*=[[:space:]]*"([^"]+)"/\1/p' /etc/containerd/config.toml)"
[ "${configured_containerd_root}" = "${CONTAINERD_ROOT}" ] || fail "containerd root verification failed"
docker system df >"${BACKUP_DIR}/docker-system-df.after.txt"
df -hT / "${EXTERNAL_MOUNT}" | tee "${BACKUP_DIR}/df.after.txt"

root_available_bytes="$(df --output=avail -B1 / | tail -1 | tr -d ' ')"
minimum_free_bytes=$((8 * 1024 * 1024 * 1024))
if [ "${root_available_bytes}" -lt "${minimum_free_bytes}" ]; then
  echo "warning: root filesystem has less than the 8 GiB acceptance target free" >&2
fi

echo "System storage migration completed."
echo "Quarantine retained at: ${BACKUP_DIR}"
echo "Next: rebuild the project Docker image and run its validation targets."
