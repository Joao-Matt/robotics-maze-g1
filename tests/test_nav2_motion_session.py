from __future__ import annotations

import importlib.util
import importlib.machinery
from pathlib import Path
import sys
import types
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
_NUMPY_STUB_INSTALLED = False
if importlib.util.find_spec("numpy") is None:
    class _Array:
        def __init__(self, values):
            self.values = list(values)

        def reshape(self, height, width):
            rows = [self.values[index * width : (index + 1) * width] for index in range(height)]
            return _Grid(rows)

    class _Grid:
        def __init__(self, rows):
            self.rows = rows

        def __getitem__(self, key):
            if isinstance(key, tuple):
                row, col = key
                return self.rows[row][col]
            return self.rows[key]

    numpy_stub = types.ModuleType("numpy")
    numpy_stub.__spec__ = importlib.machinery.ModuleSpec("numpy", loader=None)
    numpy_stub.array = lambda values, dtype=None: list(values)
    numpy_stub.asarray = lambda values, dtype=None: _Array(values)
    numpy_stub.float32 = float
    numpy_stub.float64 = float
    numpy_stub.int16 = int
    numpy_stub.ndarray = object
    numpy_stub.zeros = lambda size, dtype=None: [0.0] * int(size)
    sys.modules.setdefault("numpy", numpy_stub)
    _NUMPY_STUB_INSTALLED = True

from sim.locomotion_policy_adapter import VelocityCommand  # noqa: E402
from sim.nav2_motion_session import NavigationLimits, Nav2MotionSession  # noqa: E402

if _NUMPY_STUB_INSTALLED:
    sys.modules.pop("numpy", None)


class Nav2MotionSessionLimiterTest(unittest.TestCase):
    def make_session(self) -> Nav2MotionSession:
        session = object.__new__(Nav2MotionSession)
        session.limits = NavigationLimits(
            max_forward_mps=0.45,
            min_forward_mps=0.12,
            max_reverse_mps=-0.30,
            max_yaw_rate_radps=1.20,
            max_linear_accel_mps2=2.00,
            max_linear_decel_mps2=2.00,
            max_yaw_accel_radps2=1.20,
            max_yaw_decel_radps2=1.50,
            turn_slowdown_start_radps=0.45,
            turn_slowdown_full_radps=1.10,
            turn_slowdown_min_forward_mps=0.12,
        )
        session.control_dt = 0.02
        session.last_command = VelocityCommand()
        return session

    def test_sharp_turn_keeps_forward_arc_for_native_policy(self) -> None:
        limited = self.make_session()._limited_command(VelocityCommand(vx=0.0, yaw_rate=1.20))

        self.assertAlmostEqual(limited.vx, 0.12)
        self.assertAlmostEqual(limited.yaw_rate, 1.20)

    def test_sharp_turn_clamps_forward_speed_to_coupled_limit(self) -> None:
        limited = self.make_session()._limited_command(VelocityCommand(vx=0.45, yaw_rate=1.20))

        self.assertAlmostEqual(limited.vx, 0.12)
        self.assertAlmostEqual(limited.yaw_rate, 1.20)

    def test_straight_motion_preserves_requested_forward_speed(self) -> None:
        limited = self.make_session()._limited_command(VelocityCommand(vx=0.30, yaw_rate=0.0))

        self.assertAlmostEqual(limited.vx, 0.30)
        self.assertAlmostEqual(limited.yaw_rate, 0.0)

    def test_reverse_motion_stays_available_for_recovery(self) -> None:
        limited = self.make_session()._limited_command(VelocityCommand(vx=-0.20, yaw_rate=0.0))

        self.assertAlmostEqual(limited.vx, -0.20)
        self.assertAlmostEqual(limited.yaw_rate, 0.0)

    def test_reverse_motion_is_clamped_to_safe_limit(self) -> None:
        limited = self.make_session()._limited_command(VelocityCommand(vx=-0.80, yaw_rate=0.0))

        self.assertAlmostEqual(limited.vx, -0.30)
        self.assertAlmostEqual(limited.yaw_rate, 0.0)

    def test_slew_limiter_ramps_forward_and_yaw_commands(self) -> None:
        limited = self.make_session()._slewed_command(VelocityCommand(vx=0.45, yaw_rate=1.20))

        self.assertAlmostEqual(limited.vx, 0.04)
        self.assertAlmostEqual(limited.yaw_rate, 0.024)

    def test_slew_limiter_decelerates_before_reversing(self) -> None:
        session = self.make_session()
        session.last_command = VelocityCommand(vx=0.45, yaw_rate=0.90)

        limited = session._slewed_command(VelocityCommand(vx=-0.20, yaw_rate=-0.90))

        self.assertAlmostEqual(limited.vx, 0.41)
        self.assertAlmostEqual(limited.yaw_rate, 0.87)


class Nav2MotionSessionCommandedStuckTest(unittest.TestCase):
    def make_session(self) -> Nav2MotionSession:
        session = object.__new__(Nav2MotionSession)
        session.limits = NavigationLimits(
            commanded_stuck_timeout_s=30.0,
            commanded_stuck_min_translation_m=0.08,
            commanded_stuck_min_yaw_rad=0.10,
        )
        session.data = types.SimpleNamespace(
            qpos=[0.0, 0.0, 0.80, 1.0, 0.0, 0.0, 0.0],
            qvel=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        session.last_actual_xy = (0.0, 0.0)
        session.distance_traveled_m = 0.0
        session.progress_anchor_xy = (0.0, 0.0)
        session.progress_anchor_time = 0.0
        session.progress_watch_active = False
        session.commanded_motion_anchor_xy = (0.0, 0.0)
        session.commanded_motion_anchor_yaw = 0.0
        session.commanded_motion_anchor_time = 0.0
        session.commanded_motion_watch_active = False
        session.recovery_events = []
        session.raw_command = VelocityCommand(vx=0.20, yaw_rate=0.0)
        session.last_command = VelocityCommand(vx=0.20, yaw_rate=0.0)
        session.status = "RUNNING"
        session.stop_reason = "navigation_active"
        return session

    def test_nonzero_command_without_ground_truth_motion_stops_as_stuck(self) -> None:
        session = self.make_session()

        session._update_motion(0.0)
        session._update_motion(29.9)
        self.assertEqual(session.status, "RUNNING")

        session._update_motion(30.0)

        self.assertEqual(session.status, "STUCK")
        self.assertEqual(session.stop_reason, "commanded_no_ground_truth_motion")
        self.assertEqual(session.recovery_events[-1]["event"], "commanded_stuck")

    def test_zero_command_does_not_trigger_commanded_stuck(self) -> None:
        session = self.make_session()
        session.raw_command = VelocityCommand()
        session.last_command = VelocityCommand()

        session._update_motion(0.0)
        session._update_motion(31.0)

        self.assertEqual(session.status, "RUNNING")

    def test_ground_truth_motion_resets_commanded_stuck_timer(self) -> None:
        session = self.make_session()

        session._update_motion(0.0)
        session.data.qpos[0] = 0.10
        session._update_motion(10.0)
        session._update_motion(39.0)

        self.assertEqual(session.status, "RUNNING")


if __name__ == "__main__":
    unittest.main()
