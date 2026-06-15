"""Milestone 0 smoke test entrypoint."""

from pathlib import Path
import argparse
import html
import platform
import sys
from datetime import datetime, timezone

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sim.config import ConfigError, load_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a lightweight config/environment smoke check.")
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "default.yaml",
        help="Path to YAML config.",
    )
    parser.add_argument("--save-html", type=Path, default=None, help="Optional HTML report path.")
    return parser.parse_args()


def summary_lines(config: dict, config_path: Path) -> list[str]:
    return [
        "Robotics Maze G1 smoke test",
        f"project: {config['project']['name']}",
        f"python: {platform.python_version()}",
        f"config: {config_path}",
        f"sim timestep: {config['sim']['timestep']}",
        f"sim control_dt: {config['sim']['control_dt']}",
        f"sim default_duration_s: {config['sim']['default_duration_s']}",
        f"robot model_xml_path: {config['robot']['model_xml_path']}",
        (
            "maze: "
            f"{config['maze']['width_cells']}x{config['maze']['height_cells']} cells, "
            f"cell_size_m={config['maze']['cell_size_m']}"
        ),
        f"logging output_root: {config['logging']['output_root']}",
        "status: ok",
    ]


def write_html_report(path: Path, lines: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    generated = datetime.now(timezone.utc).isoformat(timespec="seconds")
    body = "\n".join(html.escape(line) for line in lines)
    document = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Robotics Maze G1 Smoke Report</title>
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 2rem; color: #111827; }}
    pre {{ background: #0f172a; color: #e5e7eb; padding: 1rem; overflow: auto; }}
    .status {{ color: #166534; font-weight: 700; }}
  </style>
</head>
<body>
  <h1>Robotics Maze G1 Smoke Report</h1>
  <p>Status: <span class="status">OK</span></p>
  <p>Generated: {html.escape(generated)}</p>
  <pre>{body}</pre>
</body>
</html>
"""
    path.write_text(document, encoding="utf-8")


def main() -> int:
    args = parse_args()
    config_path = args.config

    try:
        config = load_config(config_path)
    except ConfigError as exc:
        print(f"Smoke test failed: {exc}", file=sys.stderr)
        return 1

    lines = summary_lines(config, config_path)
    print("\n".join(lines))
    if args.save_html:
        write_html_report(args.save_html, lines)
        print(f"smoke_html_artifact: {args.save_html}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
