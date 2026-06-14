from pathlib import Path

import pytest

from sim.mujoco_runner import MuJoCoModelError, MuJoCoRunner, resolve_project_path


def test_resolve_project_path_handles_relative_paths():
    root = Path("/tmp/example-root")

    assert resolve_project_path("assets/model.xml", root) == root / "assets" / "model.xml"


def test_resolve_project_path_preserves_absolute_paths():
    path = Path("/tmp/model.xml")

    assert resolve_project_path(path, Path("/tmp/example-root")) == path


def test_missing_model_path_raises_actionable_error():
    config = {
        "sim": {"timestep": 0.002, "default_duration_s": 0.01},
        "robot": {"model_xml_path": "assets/missing/scene.xml"},
    }

    with pytest.raises(MuJoCoModelError, match="git submodule update --init --recursive"):
        MuJoCoRunner(config, project_root=Path("/tmp/example-root")).run()
