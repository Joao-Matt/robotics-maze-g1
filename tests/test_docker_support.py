from pathlib import Path
import os
import subprocess


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
        PROJECT_ROOT / "scripts" / "check_storage_layout.sh",
        PROJECT_ROOT / "scripts" / "migrate_system_storage.sh",
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
        "docker-milestone_5:",
        "docker-build-multiarch:",
        "storage-check:",
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


def test_gui_docker_fails_clearly_without_display():
    env = os.environ.copy()
    env.pop("DISPLAY", None)

    result = subprocess.run(
        [str(PROJECT_ROOT / "docker" / "run_gui.sh")],
        cwd=PROJECT_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode != 0
    assert "DISPLAY is not set" in result.stderr
    assert "docker/run.sh" in result.stderr


def test_storage_support_is_portable_and_machine_config_is_ignored():
    gitignore = (PROJECT_ROOT / ".gitignore").read_text(encoding="utf-8")
    example = (PROJECT_ROOT / ".env.storage.example").read_text(encoding="utf-8")
    scripts = "\n".join(
        (PROJECT_ROOT / "scripts" / name).read_text(encoding="utf-8")
        for name in ("check_storage_layout.sh", "migrate_system_storage.sh")
    )

    assert ".env.storage" in gitignore
    assert "REQUIRED_STORAGE_MOUNT" in example
    assert "EXPECTED_STORAGE_UUID" in example
    for machine_path in ("/mnt/robotics_ssd", "/media/robojoe", "/home/robojoe"):
        assert machine_path not in scripts
