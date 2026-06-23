from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = ROOT / "configs" / "navigation_capture_topics.yaml"


def load_module():
    path = ROOT / "scripts" / "navigation_capture_artifacts.py"
    spec = importlib.util.spec_from_file_location("navigation_capture_artifacts", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    module.DEFAULT_SCHEMA = SCHEMA_PATH
    return module


def fake_metadata(schema: dict) -> dict:
    rows = []
    for topic in schema["topics"]:
        rows.append(
            {
                "topic_metadata": {
                    "name": topic["name"],
                    "type": topic["type"],
                    "serialization_format": "cdr",
                    "offered_qos_profiles": "",
                },
                "message_count": 1,
            }
        )
    return {
        "rosbag2_bagfile_information": {
            "version": 5,
            "storage_identifier": "sqlite3",
            "message_count": len(rows),
            "topics_with_message_count": rows,
        }
    }


def write_fake_bag(run_dir: Path, schema: dict) -> None:
    bag_dir = run_dir / "rosbag"
    bag_dir.mkdir(parents=True)
    (bag_dir / "metadata.yaml").write_text(yaml.safe_dump(fake_metadata(schema)), encoding="utf-8")
    db = sqlite3.connect(bag_dir / "rosbag_0.db3")
    try:
        db.execute(
            "CREATE TABLE topics("
            "id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, "
            "serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL)"
        )
        db.execute(
            "CREATE TABLE messages("
            "id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL, data BLOB NOT NULL)"
        )
        for index, topic in enumerate(schema["topics"][:3], start=1):
            db.execute(
                "INSERT INTO topics(id, name, type, serialization_format, offered_qos_profiles) VALUES(?, ?, ?, 'cdr', '')",
                (index, topic["name"], topic["type"]),
            )
            db.execute(
                "INSERT INTO messages(topic_id, timestamp, data) VALUES(?, ?, ?)",
                (index, 1_000_000_000 + index, b""),
            )
        db.commit()
    finally:
        db.close()


class NavigationCaptureArtifactsTest(unittest.TestCase):
    def test_schema_is_documented_and_valid(self) -> None:
        module = load_module()
        schema = module.load_schema(SCHEMA_PATH)

        self.assertEqual(module.validate_schema(schema), [])
        self.assertEqual(len(module.topic_names(schema)), len(set(module.topic_names(schema))))
        for topic in schema["topics"]:
            for field in ("name", "type", "producer", "timestamp_policy", "expected_rate_hz", "required"):
                self.assertIn(field, topic)
        ground_truth = [topic for topic in schema["topics"] if topic["name"].startswith("/ground_truth/")]
        self.assertTrue(ground_truth)
        self.assertTrue(all(topic.get("evaluation_only") is True for topic in ground_truth))

    def test_prepare_validate_and_sample_index(self) -> None:
        module = load_module()
        schema = module.load_schema(SCHEMA_PATH)
        with tempfile.TemporaryDirectory() as temporary:
            run_dir = Path(temporary) / "run"
            manifest = module.prepare_capture(
                run_dir,
                SCHEMA_PATH,
                bag_path=run_dir / "rosbag",
                storage="sqlite3",
                split_size_bytes=536870912,
                rgbd_rate_hz=3.0,
            )
            write_fake_bag(run_dir, schema)
            validation = module.validate_capture(run_dir, generate_samples=True)

            self.assertEqual(manifest["status"], "RECORDING")
            self.assertTrue((run_dir / "capture_schema.yaml").is_file())
            self.assertTrue(validation["ok"], validation["errors"])
            sample_text = (run_dir / "capture_samples.csv").read_text(encoding="utf-8")
            self.assertIn("topic,bag_time_ns,header_stamp_ns,frame_id,message_type,bag_file", sample_text)
            self.assertIn("/camera/color/image_raw", sample_text)

    def test_repair_marks_stale_recording_run_crashed(self) -> None:
        module = load_module()
        schema = module.load_schema(SCHEMA_PATH)
        with tempfile.TemporaryDirectory() as temporary:
            run_dir = Path(temporary) / "run"
            run_dir.mkdir()
            module.prepare_capture(
                run_dir,
                SCHEMA_PATH,
                bag_path=run_dir / "rosbag",
                storage="sqlite3",
                split_size_bytes=536870912,
                rgbd_rate_hz=3.0,
            )
            (run_dir / "run_manifest.json").write_text(
                json.dumps({"final_status": "RUNNING", "started_at": "2026-01-01T00:00:00+00:00"}) + "\n",
                encoding="utf-8",
            )
            write_fake_bag(run_dir, schema)

            report = module.repair_capture(run_dir)

            self.assertEqual(report["manifest"]["status"], "CRASHED_OR_INTERRUPTED")
            repaired_run_manifest = json.loads((run_dir / "run_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(repaired_run_manifest["final_status"], "CRASHED_OR_INTERRUPTED")
            self.assertTrue((run_dir / "capture_repair.json").is_file())

    def test_lenient_readers_ignore_malformed_tail(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            ndjson = root / "events.ndjson"
            ndjson.write_text('{"ok": true}\n{"partial": ', encoding="utf-8")
            csv_path = root / "timeseries.csv"
            csv_path.write_text("a,b\n1,2\nbroken\n", encoding="utf-8")

            self.assertEqual(module.read_ndjson_lenient(ndjson), {"rows": 1, "ignored_malformed_rows": 1})
            self.assertEqual(module.read_csv_lenient(csv_path), {"rows": 1, "ignored_malformed_rows": 1})


if __name__ == "__main__":
    unittest.main()
