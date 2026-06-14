"""Milestone 0 smoke test entrypoint."""

from pathlib import Path
import platform
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sim.config import ConfigError, load_config


def main() -> int:
    config_path = PROJECT_ROOT / "configs" / "default.yaml"

    try:
        config = load_config(config_path)
    except ConfigError as exc:
        print(f"Smoke test failed: {exc}", file=sys.stderr)
        return 1

    print("Robotics Maze G1 smoke test")
    print(f"project: {config['project']['name']}")
    print(f"python: {platform.python_version()}")
    print(f"config: {config_path}")
    print(f"sim timestep: {config['sim']['timestep']}")
    print(f"sim control_dt: {config['sim']['control_dt']}")
    print(f"sim default_duration_s: {config['sim']['default_duration_s']}")
    print(f"robot model_xml_path: {config['robot']['model_xml_path']}")
    print(
        "maze: "
        f"{config['maze']['width_cells']}x{config['maze']['height_cells']} cells, "
        f"cell_size_m={config['maze']['cell_size_m']}"
    )
    print(f"logging output_root: {config['logging']['output_root']}")
    print("status: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
