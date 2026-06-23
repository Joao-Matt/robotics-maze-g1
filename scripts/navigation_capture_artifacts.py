#!/usr/bin/env python3
"""Crash-tolerant sidecar artifacts for navigation dataset captures."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys
from typing import Any

import yaml


DEFAULT_SCHEMA = Path("configs/navigation_capture_topics.yaml")


def now_iso() -> str:
    return datetime.now().astimezone().isoformat()


def atomic_write_bytes(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    with temporary.open("wb") as output:
        output.write(data)
        output.flush()
        os.fsync(output.fileno())
    os.replace(temporary, path)
    try:
        directory = os.open(path.parent, os.O_DIRECTORY)
    except OSError:
        return
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def atomic_write_text(path: Path, text: str) -> None:
    atomic_write_bytes(path, text.encode("utf-8"))


def atomic_write_json(path: Path, values: dict[str, Any]) -> None:
    atomic_write_text(path, json.dumps(values, indent=2, sort_keys=True) + "\n")


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as input_file:
        values = yaml.safe_load(input_file) or {}
    if not isinstance(values, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return values


def load_schema(path: Path) -> dict[str, Any]:
    schema = load_yaml(path)
    errors = validate_schema(schema)
    if errors:
        raise ValueError("invalid capture schema:\n" + "\n".join(f"- {error}" for error in errors))
    return schema


def schema_topics(schema: dict[str, Any]) -> list[dict[str, Any]]:
    topics = schema.get("topics", [])
    if not isinstance(topics, list):
        return []
    return [topic for topic in topics if isinstance(topic, dict)]


def topic_names(schema: dict[str, Any]) -> list[str]:
    return [str(topic["name"]) for topic in schema_topics(schema)]


def validate_schema(schema: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    required_schema_fields = {"schema_version", "schema_name", "timestamp_semantics", "topics"}
    missing_schema_fields = sorted(required_schema_fields - set(schema))
    for field in missing_schema_fields:
        errors.append(f"schema missing {field}")

    seen: set[str] = set()
    required_topic_fields = {"name", "type", "producer", "timestamp_policy", "expected_rate_hz", "required"}
    for index, topic in enumerate(schema_topics(schema)):
        missing = sorted(required_topic_fields - set(topic))
        name = str(topic.get("name", f"<topic {index}>"))
        for field in missing:
            errors.append(f"{name} missing {field}")
        if name in seen:
            errors.append(f"duplicate topic {name}")
        seen.add(name)
        if name.startswith("/ground_truth/") and topic.get("evaluation_only") is not True:
            errors.append(f"{name} must be marked evaluation_only")

    policy = schema.get("ground_truth_policy", {})
    if isinstance(policy, dict):
        text = json.dumps(policy, sort_keys=True).lower()
        if "evaluation" not in text or "navigation" not in text:
            errors.append("ground_truth_policy must document evaluation-only and navigation-forbidden usage")
    else:
        errors.append("ground_truth_policy must be a mapping")

    for group in schema.get("groups", []) or []:
        if not isinstance(group, dict):
            errors.append("group entries must be mappings")
            continue
        for field in ("name", "min_present", "topics"):
            if field not in group:
                errors.append(f"group {group.get('name', '<unnamed>')} missing {field}")
        for topic_name in group.get("topics", []) or []:
            if str(topic_name) not in seen:
                errors.append(f"group {group.get('name', '<unnamed>')} references unknown topic {topic_name}")
    return errors


def read_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        values = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return values if isinstance(values, dict) else {}


def update_json(path: Path, updates: dict[str, Any]) -> dict[str, Any]:
    values = read_json(path)
    values.update(updates)
    values["updated_at"] = now_iso()
    atomic_write_json(path, values)
    return values


def prepare_capture(
    run_dir: Path,
    schema_path: Path,
    *,
    bag_path: Path,
    storage: str,
    split_size_bytes: int,
    rgbd_rate_hz: float,
) -> dict[str, Any]:
    schema = load_schema(schema_path)
    run_dir.mkdir(parents=True, exist_ok=True)
    capture_schema = run_dir / "capture_schema.yaml"
    atomic_write_bytes(capture_schema, schema_path.read_bytes())
    manifest = {
        "schema_version": 1,
        "status": "RECORDING",
        "started_at": now_iso(),
        "updated_at": now_iso(),
        "run_directory": str(run_dir),
        "schema_path": str(capture_schema),
        "source_schema_path": str(schema_path),
        "rosbag_path": str(bag_path),
        "storage_identifier": storage,
        "split_size_bytes": int(split_size_bytes),
        "rgbd_rate_hz": float(rgbd_rate_hz),
        "topics": topic_names(schema),
        "timestamp_semantics": schema.get("timestamp_semantics", {}),
        "ground_truth_policy": schema.get("ground_truth_policy", {}),
    }
    atomic_write_json(run_dir / "capture_manifest.json", manifest)
    return manifest


def metadata_path(run_dir: Path) -> Path:
    return run_dir / "rosbag" / "metadata.yaml"


def load_bag_metadata(run_dir: Path) -> dict[str, Any]:
    path = metadata_path(run_dir)
    if not path.is_file():
        return {}
    try:
        values = load_yaml(path)
    except Exception:
        return {}
    info = values.get("rosbag2_bagfile_information", {})
    return info if isinstance(info, dict) else {}


def metadata_topics(metadata: dict[str, Any]) -> dict[str, dict[str, Any]]:
    topics: dict[str, dict[str, Any]] = {}
    for row in metadata.get("topics_with_message_count", []) or []:
        if not isinstance(row, dict):
            continue
        topic_metadata = row.get("topic_metadata", {})
        if not isinstance(topic_metadata, dict):
            continue
        name = str(topic_metadata.get("name", ""))
        if not name:
            continue
        topics[name] = {
            "type": topic_metadata.get("type"),
            "message_count": int(row.get("message_count") or 0),
        }
    return topics


def sqlite_integrity_checks(bag_dir: Path) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    for db_path in sorted(bag_dir.glob("*.db3")):
        row: dict[str, Any] = {"path": str(db_path)}
        try:
            connection = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
            try:
                result = connection.execute("PRAGMA integrity_check").fetchone()
                row["result"] = result[0] if result else "no_result"
                row["ok"] = row["result"] == "ok"
            finally:
                connection.close()
        except Exception as exc:
            row["ok"] = False
            row["error"] = str(exc)
        checks.append(row)
    return checks


def try_ros_deserializers():
    try:
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except Exception:
        return None, None
    return deserialize_message, get_message


def message_header_values(serialized: bytes, type_name: str, cache: dict[str, Any]) -> tuple[int | None, str | None]:
    deserialize_message, get_message = cache.get("__ros_helpers__", (None, None))
    if deserialize_message is None or get_message is None:
        return None, None
    try:
        message_type = cache.get(type_name)
        if message_type is None:
            message_type = get_message(type_name)
            cache[type_name] = message_type
        message = deserialize_message(serialized, message_type)
        header = getattr(message, "header", None)
        if header is None:
            return None, None
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return None, getattr(header, "frame_id", None)
        sec = int(getattr(stamp, "sec", 0))
        nanosec = int(getattr(stamp, "nanosec", 0))
        return sec * 1_000_000_000 + nanosec, str(getattr(header, "frame_id", ""))
    except Exception:
        return None, None


def atomic_csv_path(path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    return path.with_name(f".{path.name}.tmp-{os.getpid()}")


def finish_atomic_file(temporary: Path, final: Path, file_handle) -> None:
    file_handle.flush()
    os.fsync(file_handle.fileno())
    os.replace(temporary, final)
    try:
        directory = os.open(final.parent, os.O_DIRECTORY)
    except OSError:
        return
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def generate_sample_index(run_dir: Path) -> dict[str, Any]:
    bag_dir = run_dir / "rosbag"
    output_path = run_dir / "capture_samples.csv"
    temporary = atomic_csv_path(output_path)
    deserialize_message, get_message = try_ros_deserializers()
    cache: dict[str, Any] = {"__ros_helpers__": (deserialize_message, get_message)}
    total = 0
    db_count = 0
    with temporary.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(["topic", "bag_time_ns", "header_stamp_ns", "frame_id", "message_type", "bag_file"])
        for db_path in sorted(bag_dir.glob("*.db3")):
            db_count += 1
            connection = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
            try:
                topic_rows = connection.execute("SELECT id, name, type FROM topics").fetchall()
                topics = {int(topic_id): (str(name), str(type_name)) for topic_id, name, type_name in topic_rows}
                query = "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
                for topic_id, timestamp, data in connection.execute(query):
                    topic_name, type_name = topics.get(int(topic_id), ("", ""))
                    header_stamp, frame_id = message_header_values(bytes(data), type_name, cache)
                    writer.writerow([
                        topic_name,
                        int(timestamp),
                        "" if header_stamp is None else int(header_stamp),
                        "" if frame_id is None else frame_id,
                        type_name,
                        db_path.name,
                    ])
                    total += 1
            finally:
                connection.close()
        finish_atomic_file(temporary, output_path, output)
    return {
        "path": str(output_path),
        "rows": total,
        "bag_files": db_count,
        "header_extraction": "ros_deserialization" if deserialize_message is not None else "unavailable",
    }


def read_ndjson_lenient(path: Path) -> dict[str, int]:
    if not path.is_file():
        return {"rows": 0, "ignored_malformed_rows": 0}
    rows = 0
    ignored = 0
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    for index, line in enumerate(lines):
        if not line.strip():
            continue
        try:
            json.loads(line)
            rows += 1
        except json.JSONDecodeError:
            if index == len(lines) - 1:
                ignored += 1
            else:
                ignored += 1
    return {"rows": rows, "ignored_malformed_rows": ignored}


def read_csv_lenient(path: Path) -> dict[str, int]:
    if not path.is_file():
        return {"rows": 0, "ignored_malformed_rows": 0}
    text = path.read_text(encoding="utf-8", errors="replace")
    lines = text.splitlines()
    if not lines:
        return {"rows": 0, "ignored_malformed_rows": 0}
    header = next(csv.reader([lines[0]]), [])
    rows = 0
    ignored = 0
    expected_columns = len(header)
    for index, line in enumerate(lines[1:], start=1):
        try:
            parsed = next(csv.reader([line], strict=True))
        except csv.Error:
            if index == len(lines) - 1:
                ignored += 1
            else:
                ignored += 1
            continue
        if expected_columns and len(parsed) != expected_columns:
            if index == len(lines) - 1:
                ignored += 1
            else:
                ignored += 1
            continue
        rows += 1
    return {"rows": rows, "ignored_malformed_rows": ignored}


def validate_live_dashboard(run_dir: Path) -> dict[str, Any]:
    live_dir = run_dir / "live_dashboard"
    return {
        "kpi_stream": read_ndjson_lenient(live_dir / "kpi_stream.ndjson"),
        "events": read_ndjson_lenient(live_dir / "events.ndjson"),
        "timeseries": read_csv_lenient(live_dir / "timeseries_downsampled.csv"),
    }


def validate_capture(run_dir: Path, *, generate_samples: bool = True) -> dict[str, Any]:
    schema_path = run_dir / "capture_schema.yaml"
    if not schema_path.is_file():
        schema_path = DEFAULT_SCHEMA
    schema = load_schema(schema_path)
    metadata = load_bag_metadata(run_dir)
    actual = metadata_topics(metadata)
    errors: list[str] = []
    warnings: list[str] = []
    topic_results: dict[str, dict[str, Any]] = {}

    for topic in schema_topics(schema):
        name = str(topic["name"])
        required = bool(topic.get("required"))
        expected_type = str(topic["type"])
        found = actual.get(name)
        count = int((found or {}).get("message_count") or 0)
        actual_type = (found or {}).get("type")
        result = {
            "required": required,
            "expected_type": expected_type,
            "actual_type": actual_type,
            "message_count": count,
            "present": found is not None,
        }
        if found is None:
            (errors if required else warnings).append(f"{name} missing from rosbag metadata")
        elif actual_type != expected_type:
            errors.append(f"{name} type mismatch: expected {expected_type}, got {actual_type}")
        elif required and count <= 0:
            errors.append(f"{name} has zero messages")
        topic_results[name] = result

    group_results: dict[str, dict[str, Any]] = {}
    for group in schema.get("groups", []) or []:
        if not isinstance(group, dict):
            continue
        names = [str(value) for value in group.get("topics", []) or []]
        present = [name for name in names if int(topic_results.get(name, {}).get("message_count") or 0) > 0]
        minimum = int(group.get("min_present") or 0)
        group_results[str(group.get("name"))] = {"min_present": minimum, "present": present, "topics": names}
        if len(present) < minimum:
            errors.append(f"group {group.get('name')} needs {minimum} populated topics, got {len(present)}")

    integrity = sqlite_integrity_checks(run_dir / "rosbag")
    if not integrity:
        errors.append("no sqlite .db3 bag files found")
    for row in integrity:
        if not row.get("ok"):
            errors.append(f"sqlite integrity failed for {row.get('path')}: {row.get('result') or row.get('error')}")

    sample_index = None
    if generate_samples:
        try:
            sample_index = generate_sample_index(run_dir)
        except Exception as exc:
            errors.append(f"could not generate capture_samples.csv: {exc}")
            sample_index = {"error": str(exc)}

    validation = {
        "schema_version": 1,
        "generated_at": now_iso(),
        "run_directory": str(run_dir),
        "metadata_path": str(metadata_path(run_dir)),
        "ok": not errors,
        "errors": errors,
        "warnings": warnings,
        "topic_results": topic_results,
        "group_results": group_results,
        "sqlite_integrity": integrity,
        "sample_index": sample_index,
        "live_dashboard": validate_live_dashboard(run_dir),
        "message_count": int(metadata.get("message_count") or 0),
        "storage_identifier": metadata.get("storage_identifier"),
    }
    atomic_write_json(run_dir / "capture_validation.json", validation)
    return validation


def finalize_capture(run_dir: Path, exit_code: int | None) -> dict[str, Any]:
    validation = validate_capture(run_dir, generate_samples=True)
    status = "FINALIZED" if validation.get("ok") else "FINALIZED_WITH_ERRORS"
    manifest = update_json(
        run_dir / "capture_manifest.json",
        {
            "status": status,
            "ended_at": now_iso(),
            "process_exit_code": exit_code,
            "validation_path": str(run_dir / "capture_validation.json"),
            "sample_index_path": str(run_dir / "capture_samples.csv"),
        },
    )
    return {"manifest": manifest, "validation": validation}


def run_reindex_if_needed(run_dir: Path, *, force: bool = False) -> dict[str, Any]:
    bag_dir = run_dir / "rosbag"
    if not force and metadata_path(run_dir).is_file():
        return {"attempted": False, "reason": "metadata_exists"}
    ros2 = shutil.which("ros2")
    if ros2 is None:
        return {"attempted": False, "reason": "ros2_not_found"}
    result = subprocess.run([ros2, "bag", "reindex", str(bag_dir)], capture_output=True, text=True, check=False)
    return {
        "attempted": True,
        "returncode": result.returncode,
        "stdout": result.stdout,
        "stderr": result.stderr,
    }


def mark_run_manifest_crashed(run_dir: Path) -> dict[str, Any] | None:
    path = run_dir / "run_manifest.json"
    values = read_json(path)
    if not values or values.get("final_status") != "RUNNING":
        return None
    values["final_status"] = "CRASHED_OR_INTERRUPTED"
    values["ended_at"] = now_iso()
    values["summary"] = str(run_dir / "summary.json") if (run_dir / "summary.json").is_file() else None
    atomic_write_json(path, values)
    return values


def repair_capture(run_dir: Path, *, force_reindex: bool = False) -> dict[str, Any]:
    reindex = run_reindex_if_needed(run_dir, force=force_reindex)
    validation = validate_capture(run_dir, generate_samples=True)
    manifest_path = run_dir / "capture_manifest.json"
    current = read_json(manifest_path)
    status = "CRASHED_OR_INTERRUPTED" if current.get("status") == "RECORDING" else current.get("status", "REPAIRED")
    manifest = update_json(
        manifest_path,
        {
            "status": status,
            "repaired_at": now_iso(),
            "reindex": reindex,
            "validation_path": str(run_dir / "capture_validation.json"),
            "sample_index_path": str(run_dir / "capture_samples.csv"),
        },
    )
    run_manifest = mark_run_manifest_crashed(run_dir)
    report = {"manifest": manifest, "run_manifest": run_manifest, "reindex": reindex, "validation": validation}
    atomic_write_json(run_dir / "capture_repair.json", report)
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    topics_parser = subparsers.add_parser("topics")
    topics_parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA)

    prepare_parser = subparsers.add_parser("prepare")
    prepare_parser.add_argument("--run-dir", type=Path, required=True)
    prepare_parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA)
    prepare_parser.add_argument("--bag-path", type=Path, required=True)
    prepare_parser.add_argument("--storage", default="sqlite3")
    prepare_parser.add_argument("--split-size-bytes", type=int, default=536870912)
    prepare_parser.add_argument("--rgbd-rate-hz", type=float, default=3.0)

    finalize_parser = subparsers.add_parser("finalize")
    finalize_parser.add_argument("--run-dir", type=Path, required=True)
    finalize_parser.add_argument("--exit-code", type=int)

    repair_parser = subparsers.add_parser("repair")
    repair_parser.add_argument("--run-dir", type=Path, required=True)
    repair_parser.add_argument("--force-reindex", action="store_true")

    validate_parser = subparsers.add_parser("validate")
    validate_parser.add_argument("--run-dir", type=Path, required=True)
    validate_parser.add_argument("--no-sample-index", action="store_true")

    args = parser.parse_args()
    if args.command == "topics":
        print(" ".join(topic_names(load_schema(args.schema))))
        return 0
    if args.command == "prepare":
        manifest = prepare_capture(
            args.run_dir,
            args.schema,
            bag_path=args.bag_path,
            storage=args.storage,
            split_size_bytes=args.split_size_bytes,
            rgbd_rate_hz=args.rgbd_rate_hz,
        )
        print(json.dumps({"capture_manifest": str(args.run_dir / "capture_manifest.json"), "topics": manifest["topics"]}))
        return 0
    if args.command == "finalize":
        print(json.dumps(finalize_capture(args.run_dir, args.exit_code), indent=2, sort_keys=True))
        return 0
    if args.command == "repair":
        print(json.dumps(repair_capture(args.run_dir, force_reindex=args.force_reindex), indent=2, sort_keys=True))
        return 0
    if args.command == "validate":
        print(json.dumps(validate_capture(args.run_dir, generate_samples=not args.no_sample_index), indent=2, sort_keys=True))
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
