from pathlib import Path

from sim.locomotion_policy_adapter import UnitreeRLGymG1PolicyAdapter, create_policy_adapter


def test_unitree_rl_gym_alias_creates_adapter(tmp_path):
    adapter = create_policy_adapter("unitree_rl_gym_g1", unitree_rl_gym_repo=tmp_path)

    assert isinstance(adapter, UnitreeRLGymG1PolicyAdapter)
    assert adapter.adapter_name == "unitree_rl_gym_g1"
    assert adapter.real_locomotion is True


def test_unitree_rl_gym_native_alias_creates_native_adapter(tmp_path):
    adapter = create_policy_adapter("unitree_rl_gym_native", unitree_rl_gym_repo=tmp_path)

    assert isinstance(adapter, UnitreeRLGymG1PolicyAdapter)
    assert adapter.adapter_name == "unitree_rl_gym_native"
    assert adapter.native_model is True


def test_missing_unitree_rl_gym_assets_report_is_actionable(tmp_path):
    adapter = UnitreeRLGymG1PolicyAdapter(repo_root=tmp_path / "missing")

    report = adapter.compatibility_report(FakeModel(), Path("scene.xml"))

    assert report.compatible is False
    assert any("make fetch-unitree-rl-gym-policy" in error for error in report.errors)


def test_unitree_rl_gym_leg_joint_order_is_declared():
    assert UnitreeRLGymG1PolicyAdapter.leg_joint_names[:2] == [
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
    ]
    assert len(UnitreeRLGymG1PolicyAdapter.leg_joint_names) == 12


class FakeModel:
    nu = 29
