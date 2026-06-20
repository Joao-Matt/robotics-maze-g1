"""Shared, collision-safe artifact directory allocation."""

from __future__ import annotations

from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess


def _safe(value: object) -> str:
    return str(value).strip().replace("/", "-").replace(" ", "-").replace(":", "")


def allocate_run(root: Path, command: str, seed: int | None, parameters: dict[str, object], now: datetime | None = None) -> Path:
    now = now or datetime.now().astimezone()
    seed_name = f"seed-{seed}" if seed is not None else "seed-none"
    stamp = now.strftime("%Y%m%dT%H%M%S.%f")[:-3] + now.strftime("%z")
    suffix = "".join(f"__{_safe(key)}-{_safe(value)}" for key, value in sorted(parameters.items()) if value not in (None, ""))
    parent = root / _safe(command) / seed_name
    candidate = parent / f"{stamp}{suffix}"
    counter = 1
    while candidate.exists():
        candidate = parent / f"{stamp}{suffix}__{counter:02d}"
        counter += 1
    candidate.mkdir(parents=True)
    latest = parent / "latest"
    temporary = parent / ".latest.tmp"
    temporary.unlink(missing_ok=True)
    temporary.symlink_to(candidate.name, target_is_directory=True)
    temporary.replace(latest)
    return candidate


def git_metadata(project_root: Path) -> dict[str, object]:
    def run(*args: str) -> str:
        result = subprocess.run(args, cwd=project_root, capture_output=True, text=True, check=False)
        return result.stdout.strip()
    return {"revision": run("git", "rev-parse", "HEAD"), "dirty": bool(run("git", "status", "--porcelain"))}


def file_sha256(path: Path) -> str | None:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def write_manifest(run_dir: Path, *, command: str, seed: int | None, parameters: dict[str, object], project_root: Path, config_path: Path | None = None) -> Path:
    manifest = {
        "schema_version": 1,
        "command": command,
        "seed": seed,
        "run_directory": str(run_dir.resolve()),
        "started_at": datetime.now().astimezone().isoformat(),
        "timezone": str(datetime.now().astimezone().tzinfo),
        "parameters": parameters,
        "config_path": str(config_path) if config_path else None,
        "config_sha256": file_sha256(config_path) if config_path else None,
        "config_contents": config_path.read_text(encoding="utf-8") if config_path and config_path.is_file() else None,
        "git": git_metadata(project_root),
        "docker_image": os.environ.get("DOCKER_IMAGE", "robotics-maze-g1:production"),
        "final_status": "RUNNING",
    }
    path = run_dir / "run_manifest.json"
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def finalize_manifest(run_dir: Path, final_status: str, summary_path: Path | None = None) -> None:
    path = run_dir / "run_manifest.json"
    values = json.loads(path.read_text(encoding="utf-8"))
    started=datetime.fromisoformat(values["started_at"])
    values["ended_at"] = datetime.now().astimezone(started.tzinfo).isoformat()
    values["final_status"] = final_status
    values["summary"] = str(summary_path) if summary_path else None
    path.write_text(json.dumps(values, indent=2, sort_keys=True) + "\n", encoding="utf-8")
