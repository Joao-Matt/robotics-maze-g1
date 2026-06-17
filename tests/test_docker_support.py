from pathlib import Path
import os


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_docker_support_files_exist_and_scripts_are_executable():
    required_files = [
        PROJECT_ROOT / "docker" / "Dockerfile",
        PROJECT_ROOT / "docker" / "entrypoint.sh",
        PROJECT_ROOT / "docker" / "run.sh",
        PROJECT_ROOT / "docker" / "run_gui.sh",
        PROJECT_ROOT / "docker" / "build_multiarch.sh",
        PROJECT_ROOT / ".dockerignore",
        PROJECT_ROOT / "scripts" / "check_ros_docker_env.sh",
    ]

    for path in required_files:
        assert path.exists(), f"missing Docker support file: {path}"

    for path in required_files:
        if path.suffix == ".sh":
            assert os.access(path, os.X_OK), f"Docker script is not executable: {path}"


def test_dockerignore_keeps_runtime_assets_available():
    ignored = (PROJECT_ROOT / ".dockerignore").read_text(encoding="utf-8")

    assert "runs/" in ignored
    assert ".venv" in ignored
    assert "third_party/" not in ignored
    assert "assets/" not in ignored
    assert "walker.onnx" not in ignored


def test_readme_documents_docker_quick_start():
    readme = (PROJECT_ROOT / "README.md").read_text(encoding="utf-8")

    assert "## Docker Quick Start" in readme
    assert "make docker-build" in readme
    assert "make docker-run" in readme
    assert "make docker-run-gui" in readme
    assert "make docker-check-ros" in readme


def test_makefile_exposes_docker_targets():
    makefile = (PROJECT_ROOT / "Makefile").read_text(encoding="utf-8")

    for target in (
        "docker-build:",
        "docker-run:",
        "docker-run-gui:",
        "docker-test:",
        "docker-smoke:",
        "docker-check-ros:",
        "docker-milestone_4:",
        "docker-build-multiarch:",
    ):
        assert target in makefile


def test_pytest_pin_stays_ros_humble_plugin_compatible():
    requirements = (PROJECT_ROOT / "requirements.txt").read_text(encoding="utf-8")

    assert "pytest>=8.0,<9.0" in requirements


def test_headless_docker_defaults_to_osmesa_rendering():
    run_script = (PROJECT_ROOT / "docker" / "run.sh").read_text(encoding="utf-8")
    gui_script = (PROJECT_ROOT / "docker" / "run_gui.sh").read_text(encoding="utf-8")

    assert 'MUJOCO_GL_VALUE="${MUJOCO_GL:-osmesa}"' in run_script
    assert 'MUJOCO_GL_VALUE="${MUJOCO_GL:-glfw}"' in gui_script
