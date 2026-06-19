from pathlib import Path
import json
import subprocess
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_milestone_4_runner_plans_without_robot_execution(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            "scripts/run_milestone_4_planner.py",
            "--seed",
            "123",
            "--output-dir",
            str(tmp_path),
        ],
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    summary_path = tmp_path / "milestone_4_seed-123_planner_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["milestone"] == 4
    assert summary["mode"] == "oracle_planner_only"
    assert summary["robot_execution"] is False
    assert summary["path_cell_count"] > 1
    assert summary["next_command"] == "make milestone_5 SEED=123"
    assert (tmp_path / "milestone_4_seed-123_path.svg").exists()


def test_makefile_assigns_robot_execution_to_milestone_5():
    makefile = (PROJECT_ROOT / "Makefile").read_text(encoding="utf-8")
    milestone_4_block = makefile.split("\nmilestone_4:", 1)[1].split("\nview-milestone_4:", 1)[0]
    milestone_5_block = makefile.split("\nmilestone_5:", 1)[1].split(
        "\nview-milestone_5:", 1
    )[0]

    assert "run_milestone_4_planner.py" in milestone_4_block
    assert "run_g1_oracle_follow.py" not in milestone_4_block
    assert "run_g1_oracle_follow.py" in milestone_5_block
    assert "--viewer" in milestone_5_block


def test_view_milestone_5_is_live_and_report_is_headless():
    makefile = (PROJECT_ROOT / "Makefile").read_text(encoding="utf-8")
    view_block = makefile.split("\nview-milestone_5:", 1)[1].split("\nreport-milestone_5:", 1)[0]
    report_block = makefile.split("\nreport-milestone_5:", 1)[1].split(
        "\ng1-oracle-follow:", 1
    )[0]

    assert "$(MAKE) milestone_5" in view_block
    assert "run_g1_oracle_follow.py" in report_block
    assert "--viewer" not in report_block
