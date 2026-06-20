from pathlib import Path

from sim.oracle_motion_session import oracle_stop_decision
from nav.oracle_follow import GOAL_REACHED


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_oracle_motion_session_is_shared_ros_capability():
    session = (PROJECT_ROOT / "sim" / "oracle_motion_session.py").read_text(encoding="utf-8")
    bridge = (
        PROJECT_ROOT / "ros_ws" / "src" / "g1_mujoco_bridge" / "g1_mujoco_bridge" / "bridge_node.py"
    ).read_text(encoding="utf-8")

    assert "class OracleMotionSession" in session
    assert "TurnAwareOracleFollower" in session
    assert "lucky_walker" in session
    assert "OracleMotionSession" in bridge


def test_oracle_stop_decision_goal_precedes_zero_and_duration_limits():
    assert oracle_stop_decision(
        follower_state=GOAL_REACHED, fallen=False, zero_duration_s=20.0,
        zero_timeout_s=20.0, timed_out=True,
    ) == (GOAL_REACHED, "goal_reached")


def test_oracle_stop_decision_detects_zero_command_timeout():
    assert oracle_stop_decision(
        follower_state="FOLLOWING", fallen=False, zero_duration_s=20.0,
        zero_timeout_s=20.0, timed_out=False,
    ) == ("ZERO_COMMAND_TIMEOUT", "zero_oracle_command_timeout")
