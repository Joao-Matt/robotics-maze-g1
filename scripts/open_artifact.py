"""Open a generated artifact when desktop support is available."""

from __future__ import annotations

from pathlib import Path
import argparse
import os
import shutil
import subprocess
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Open an artifact with xdg-open when available.")
    parser.add_argument("path", type=Path, help="Artifact path to open.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    path = args.path
    if not path.exists():
        print(f"Artifact does not exist yet: {path}", file=sys.stderr)
        return 1

    opener = shutil.which("xdg-open")
    if opener and os.environ.get("DISPLAY"):
        subprocess.Popen(
            [opener, str(path)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        print(f"opened_artifact: {path}")
        return 0

    print(f"artifact: {path}")
    print("No DISPLAY/xdg-open viewer detected; open this file from a desktop session.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
