from pathlib import Path
import json

from sim.locomotion_policy_adapter import LuckyWalkerPolicyAdapter, create_policy_adapter


JOINT_NAMES = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]


def test_lucky_walker_alias_creates_adapter(tmp_path):
    adapter = create_policy_adapter("lucky_walker", lucky_g1_repo=tmp_path)

    assert isinstance(adapter, LuckyWalkerPolicyAdapter)
    assert adapter.adapter_name == "lucky_walker"
    assert adapter.real_locomotion is True


def test_lucky_walker_prepare_model_xml_writes_flat_wrapper(tmp_path):
    repo = tmp_path / "lucky"
    repo.mkdir()
    (repo / "g1.xml").write_text("<mujoco model=\"g1\"><worldbody/></mujoco>\n", encoding="utf-8")
    adapter = LuckyWalkerPolicyAdapter(repo_root=repo)

    scene_path = adapter.prepare_model_xml(tmp_path / "visual")

    assert scene_path == repo / "flat_scene_locomotion_sandbox.xml"
    rendered = scene_path.read_text(encoding="utf-8")
    assert '<include file="g1.xml"/>' in rendered
    assert 'name="floor"' in rendered


def test_missing_lucky_assets_report_is_actionable(tmp_path):
    adapter = LuckyWalkerPolicyAdapter(repo_root=tmp_path / "missing")

    report = adapter.compatibility_report(FakeModel(), Path("flat.xml"))

    assert report.compatible is False
    assert any("make fetch-lucky-g1-policy" in error for error in report.errors)


class FakeModel:
    nu = 29
