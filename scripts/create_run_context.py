#!/usr/bin/env python3
"""Allocate one structured run directory and print only its path."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim.run_context import allocate_run, write_manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--command", required=True)
    parser.add_argument("--seed", default="none")
    parser.add_argument("--root", type=Path, default=Path("runs"))
    parser.add_argument("--config", type=Path)
    parser.add_argument("--parameter", action="append", default=[])
    args = parser.parse_args()
    parameters = dict(item.split("=", 1) for item in args.parameter)
    seed = None if args.seed == "none" else int(args.seed)
    project_root = Path(__file__).resolve().parents[1]
    run_dir = allocate_run(args.root, args.command, seed, parameters)
    write_manifest(run_dir, command=args.command, seed=seed, parameters=parameters, project_root=project_root, config_path=args.config)
    print(run_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
