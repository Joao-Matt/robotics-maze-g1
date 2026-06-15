"""Run pytest and write inspectable text/HTML reports."""

from __future__ import annotations

from pathlib import Path
import argparse
import html
import platform
import subprocess
import sys
from datetime import datetime, timezone


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run pytest and save visible reports.")
    parser.add_argument("--text", type=Path, default=PROJECT_ROOT / "runs" / "visual" / "test_latest.txt")
    parser.add_argument("--html", type=Path, default=PROJECT_ROOT / "runs" / "visual" / "test_latest.html")
    parser.add_argument("pytest_args", nargs="*", default=["tests"])
    return parser.parse_args()


def write_html_report(path: Path, command: list[str], output: str, returncode: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    status = "PASS" if returncode == 0 else "FAIL"
    color = "#166534" if returncode == 0 else "#991b1b"
    generated = datetime.now(timezone.utc).isoformat(timespec="seconds")
    document = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Robotics Maze G1 Test Report</title>
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 2rem; color: #111827; }}
    code, pre {{ font-family: ui-monospace, SFMono-Regular, Consolas, monospace; }}
    .status {{ color: {color}; font-weight: 700; }}
    pre {{ background: #0f172a; color: #e5e7eb; padding: 1rem; overflow: auto; }}
  </style>
</head>
<body>
  <h1>Robotics Maze G1 Test Report</h1>
  <p>Status: <span class="status">{status}</span></p>
  <p>Generated: {html.escape(generated)}</p>
  <p>Python: {html.escape(platform.python_version())}</p>
  <p>Command: <code>{html.escape(" ".join(command))}</code></p>
  <pre>{html.escape(output)}</pre>
</body>
</html>
"""
    path.write_text(document, encoding="utf-8")


def main() -> int:
    args = parse_args()
    command = [sys.executable, "-m", "pytest", *args.pytest_args]
    completed = subprocess.run(
        command,
        cwd=PROJECT_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    output = completed.stdout

    print(output, end="")
    args.text.parent.mkdir(parents=True, exist_ok=True)
    args.text.write_text(output, encoding="utf-8")
    write_html_report(args.html, command, output, completed.returncode)
    print(f"test_text_artifact: {args.text}")
    print(f"test_html_artifact: {args.html}")
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
