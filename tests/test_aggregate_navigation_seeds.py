from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


def load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "aggregate_navigation_seeds.py"
    spec = importlib.util.spec_from_file_location("aggregate_navigation_seeds", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def write_summary(root: Path, seed: int, stamp: str, *, reached: bool, status: str) -> Path:
    run_dir = root / f"seed-{seed}" / stamp
    run_dir.mkdir(parents=True)
    path = run_dir / "summary.json"
    path.write_text(
        json.dumps(
            {
                "maze_goal_reached": reached,
                "final_status": status,
                "physical_goal_error_m": 0.25 if reached else 12.0,
                "mapping": {"coverage_fraction": 0.5},
                "motion": {"contact_counts": {"wall": 1}},
            }
        )
        + "\n",
        encoding="utf-8",
    )
    return path


class AggregateNavigationSeedsTest(unittest.TestCase):
    def test_summarize_uses_latest_run_and_reports_missing_seed(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "navigate"
            write_summary(root, 101, "20260101T000000.000+0000__duration-600s", reached=False, status="TIMEOUT")
            latest = write_summary(
                root,
                101,
                "20260102T000000.000+0000__duration-600s",
                reached=True,
                status="GOAL_REACHED",
            )
            write_summary(
                root,
                202,
                "20260101T000000.000+0000__duration-600s",
                reached=False,
                status="COLLISION_ABORT",
            )

            report = module.summarize(root, label="held_out", seeds=[101, 202, 303])

        self.assertEqual(report["count"], 2)
        self.assertEqual(report["goal_reached_count"], 1)
        self.assertEqual(report["missing_seeds"], [303])
        self.assertEqual(report["results"][0]["summary_path"], str(latest))
        self.assertTrue(report["results"][0]["success"])
        self.assertIsNotNone(report["wilson_95_ci"]["low"])

    def test_parse_seed_values_accepts_space_and_comma_lists(self) -> None:
        module = load_module()

        self.assertEqual(module.parse_seed_values(["1", "2,3", "4 5"]), [1, 2, 3, 4, 5])

    def test_write_report_files_emits_html_verdict_and_plots(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "navigate"
            write_summary(root, 101, "20260101T000000.000+0000__duration-600s", reached=True, status="GOAL_REACHED")
            write_summary(root, 202, "20260101T000000.000+0000__duration-600s", reached=False, status="TIMEOUT")
            report = module.summarize(root, label="held_out", seeds=[101, 202])
            html_path = Path(temporary) / "heldout_summary.html"

            module.write_report_files(report, None, None, html_path)

            html = html_path.read_text(encoding="utf-8")
        self.assertIn("Navigation KPI Report", html)
        self.assertIn("One-to-two-page analysis writeup", html)
        self.assertIn("Dominant failure modes", html)
        self.assertIn("Verdict", html)
        self.assertIn("Per-seed path completion", html)


if __name__ == "__main__":
    unittest.main()
