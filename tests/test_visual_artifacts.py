from pathlib import Path

from scripts.smoke_test import write_html_report
from maze.generator import generate_maze
from maze.validator import validate_maze
from maze.visualization import save_svg
from sim.mujoco_runner import _write_png

import numpy as np
import subprocess
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_smoke_html_report_is_written(tmp_path):
    path = tmp_path / "smoke_latest.html"

    write_html_report(path, ["status: ok", "project: robotics-maze-g1"])

    rendered = path.read_text(encoding="utf-8")
    assert "Robotics Maze G1 Smoke Report" in rendered
    assert "status: ok" in rendered


def test_maze_svg_artifact_is_readable_at_requested_cell_size(tmp_path):
    maze = generate_maze(seed=123)
    result = validate_maze(maze)
    path = tmp_path / "maze_seed-123.svg"

    save_svg(maze, path, result.path, cell_px=48)

    rendered = path.read_text(encoding="utf-8")
    assert "<svg" in rendered
    assert 'width="720"' in rendered
    assert "Maze seed 123" in rendered


def test_png_writer_creates_browser_readable_png(tmp_path):
    path = tmp_path / "render.png"
    pixels = np.zeros((2, 3, 3), dtype=np.uint8)
    pixels[:, :, 0] = 255

    _write_png(path, pixels)

    assert path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def test_run_dashboard_is_written_with_side_by_side_artifacts(tmp_path):
    topdown = tmp_path / "world_seed-123_topdown.svg"
    render = tmp_path / "run_seed-123_final.png"
    world_xml = tmp_path / "world_seed-123.xml"
    world_summary = tmp_path / "world_seed-123_summary.json"
    run_summary = tmp_path / "run_seed-123_summary.json"
    dashboard = tmp_path / "run_seed-123_dashboard.html"
    topdown.write_text("<svg></svg>\n", encoding="utf-8")
    render.write_bytes(b"\x89PNG\r\n\x1a\n")
    world_xml.write_text("<mujoco />\n", encoding="utf-8")
    world_summary.write_text("{}\n", encoding="utf-8")
    run_summary.write_text("{}\n", encoding="utf-8")

    completed = subprocess.run(
        [
            sys.executable,
            "scripts/write_run_dashboard.py",
            "--seed",
            "123",
            "--mode",
            "maze",
            "--html",
            str(dashboard),
            "--topdown-svg",
            str(topdown),
            "--render-image",
            str(render),
            "--world-xml",
            str(world_xml),
            "--world-summary",
            str(world_summary),
            "--run-summary",
            str(run_summary),
        ],
        cwd=PROJECT_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )

    assert completed.returncode == 0, completed.stdout
    rendered = dashboard.read_text(encoding="utf-8")
    assert "2D Maze Representation" in rendered
    assert "Generated MuJoCo Maze World" in rendered
