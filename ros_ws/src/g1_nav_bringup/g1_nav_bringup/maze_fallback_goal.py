"""Rolling unknown-edge fallback while m-explore is idle."""

from __future__ import annotations

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from explore_lite_msgs.msg import ExploreStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from g1_nav_bringup.corridor_recenter import select_corridor_center_goal
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String


ACTIVE_EXPLORE_STATUSES = {
    ExploreStatus.EXPLORATION_STARTED,
    ExploreStatus.EXPLORATION_IN_PROGRESS,
    ExploreStatus.RETURNING_TO_ORIGIN,
}


class MazeFallbackGoal(Node):
    def __init__(self):
        super().__init__("maze_fallback_goal")
        for name, default in (
            ("activation_delay_s", 2.0),
            ("retry_delay_s", 2.0),
            ("goal_refresh_s", 4.0),
            ("m_explore_max_active_s", 45.0),
            ("min_goal_distance_m", 3.2),
            ("goal_lookahead_m", 3.8),
            ("min_goal_clearance_m", 1.05),
            ("clearance_check_radius_m", 1.6),
            ("clearance_sample_step_m", 0.15),
            ("unknown_extension_m", 0.6),
            ("ray_step_m", 0.2),
            ("heading_samples", 3),
            ("max_heading_offset_deg", 10.0),
            ("wide_heading_offsets_deg", "0,30,-30,60,-60,90,-90,135,-135,180"),
            ("start_away_bias_weight", 1.5),
            ("positive_x_bias_weight", 0.8),
            ("dead_end_memory_s", 90.0),
            ("dead_end_radius_m", 2.0),
            ("dead_end_heading_deg", 50.0),
            ("dead_end_penalty", 12.0),
            ("plan_heading_max_age_s", 8.0),
            ("motion_heading_window_s", 4.0),
            ("min_motion_heading_m", 0.25),
            ("resume_m_explore_after_fallback", False),
            ("m_explore_reset_on_complete", True),
            ("recenter_on_explore_complete", True),
            ("recenter_search_radius_m", 4.0),
            ("recenter_candidate_step_m", 0.20),
            ("recenter_min_clearance_m", 0.75),
            ("recenter_min_step_m", 0.20),
            ("recenter_goal_timeout_s", 12.0),
        ):
            self.declare_parameter(name, default)
        self.map_pose = None
        self.start_xy = None
        self.pose_history = []
        self.map = None
        self.last_plan = []
        self.last_plan_wall = 0.0
        self.last_explore_status = None
        self.last_explore_wall = 0.0
        self.explore_active_since_wall = None
        self.last_goal_end_wall = 0.0
        self.last_goal_sent_wall = 0.0
        self.current_goal = None
        self.current_cell = None
        self.current_goal_kind = None
        self.current_heading_source = None
        self.current_goal_clearance = None
        self.goal_handle = None
        self.goal_active = False
        self.recovery_pending = False
        self.recovery_attempts = 0
        self.recovery_note = None
        self.dead_end_memory = []
        self.events = []
        self.navigate = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.resume_pub = self.create_publisher(Bool, "/explore/resume", 10)
        self.status_pub = self.create_publisher(String, "/exploration/status", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/exploration/fallback_goal", 10)
        status_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ExploreStatus, "/explore/status", self._explore_status, status_qos)
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self._slam_pose, 20)
        self.create_subscription(NavPath, "/plan", self._plan, 10)
        self.create_subscription(OccupancyGrid, "/map", lambda msg: setattr(self, "map", msg), 10)
        self.create_timer(1.0, self._tick)

    def _explore_status(self, msg):
        status = str(msg.status)
        now = time.monotonic()
        self.last_explore_status = status
        self.last_explore_wall = now
        if status in ACTIVE_EXPLORE_STATUSES and self.explore_active_since_wall is None:
            self.explore_active_since_wall = now
        elif status not in ACTIVE_EXPLORE_STATUSES:
            self.explore_active_since_wall = None
        if status == ExploreStatus.EXPLORATION_COMPLETE:
            self._schedule_explore_complete_recovery(now)
        if (
            status in ACTIVE_EXPLORE_STATUSES
            and not self._m_explore_timed_out(now)
            and self.goal_active
            and self.goal_handle is not None
        ):
            self.events.append({"event": "cancel_for_m_explore", "goal": self.current_goal, "explore_status": status})
            self.goal_handle.cancel_goal_async()
            self.goal_active = False

    def _tick(self):
        now = time.monotonic()
        explore_active = self.last_explore_status in ACTIVE_EXPLORE_STATUSES
        explore_timed_out = self._m_explore_timed_out(now)
        if explore_active and explore_timed_out and not self.goal_active:
            self.events.append({"event": "m_explore_watchdog_takeover", "status": self.last_explore_status})
        status = "FALLBACK_OVERRIDING_M_EXPLORE" if explore_timed_out else "FALLBACK_WAITING_M_EXPLORE" if explore_active else "FALLBACK_NAVIGATING" if self.goal_active else "FALLBACK_WAITING"
        self._publish_status(status)
        if self.goal_active and self.current_goal_kind == "explore_complete_recenter":
            timeout = float(self.get_parameter("recenter_goal_timeout_s").value)
            if timeout > 0.0 and now - self.last_goal_sent_wall >= timeout:
                self.events.append({"event": "explore_complete_recenter_timeout", "goal": self.current_goal})
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                self.goal_active = False
                self.goal_handle = None
                self.last_goal_end_wall = now
                self._finish_explore_complete_recovery("recenter_timeout")
            return
        if self.recovery_pending:
            if self.goal_active:
                return
            if now - self.last_goal_end_wall < float(self.get_parameter("retry_delay_s").value):
                return
            self._send_explore_complete_recenter_goal()
            return
        if (explore_active and not explore_timed_out) or self.goal_active:
            if (
                self.goal_active
                and (not explore_active or explore_timed_out)
                and self.goal_handle is not None
                and now - self.last_goal_sent_wall >= self._current_goal_refresh_s()
            ):
                self.events.append({"event": "refresh_cancel", "goal": self.current_goal, "kind": self.current_goal_kind})
                self.goal_handle.cancel_goal_async()
                self.goal_active = False
                self.goal_handle = None
            return
        if self.last_explore_status is None:
            return
        if now - self.last_explore_wall < float(self.get_parameter("activation_delay_s").value):
            return
        if now - self.last_goal_end_wall < float(self.get_parameter("retry_delay_s").value):
            return
        self._send_next_goal()

    def _send_next_goal(self):
        if not self.navigate.wait_for_server(timeout_sec=0.2):
            return False
        goal_xy, cell, kind = self._next_goal()
        if goal_xy is None:
            return False
        return self._send_navigation_goal(goal_xy, cell, kind)

    def _send_navigation_goal(self, goal_xy, cell, kind, yaw=None):
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(*goal_xy, yaw=yaw)
        self.current_goal = goal_xy
        self.current_cell = cell
        self.current_goal_kind = kind
        self.current_goal_clearance = self._occupied_clearance(goal_xy)
        self.goal_active = True
        self.last_goal_sent_wall = time.monotonic()
        self.goal_pub.publish(goal.pose)
        self.events.append({
            "event": "sent",
            "cell": cell,
            "goal": goal_xy,
            "kind": kind,
            "heading": self.current_heading_source,
            "clearance_m": self.current_goal_clearance,
        })
        self.navigate.send_goal_async(goal).add_done_callback(self._goal_response)
        return True

    def _schedule_explore_complete_recovery(self, now):
        if not (
            bool(self.get_parameter("m_explore_reset_on_complete").value)
            or bool(self.get_parameter("recenter_on_explore_complete").value)
        ):
            return
        if self.recovery_pending or (self.goal_active and self.current_goal_kind == "explore_complete_recenter"):
            return
        self.recovery_pending = True
        self.recovery_attempts += 1
        self.recovery_note = "If reset/recenter still cannot find new frontiers, try respawning only explore_node."
        self.events.append({
            "event": "m_explore_complete_recovery_scheduled",
            "attempt": self.recovery_attempts,
            "reminder": self.recovery_note,
        })
        if self.goal_active and self.goal_handle is not None:
            self.events.append({"event": "cancel_for_m_explore_recovery", "goal": self.current_goal, "kind": self.current_goal_kind})
            self.goal_handle.cancel_goal_async()
            self.goal_active = False
            self.goal_handle = None
            self.last_goal_end_wall = now

    def _send_explore_complete_recenter_goal(self):
        if not bool(self.get_parameter("recenter_on_explore_complete").value):
            self._finish_explore_complete_recovery("recenter_disabled")
            return True
        if self.map is None or self.map_pose is None:
            return False
        if not self.navigate.wait_for_server(timeout_sec=0.2):
            return False
        current = self._current_xy()
        goal = self._recenter_goal(current)
        if goal is None:
            self.events.append({"event": "explore_complete_recenter_unavailable"})
            self._finish_explore_complete_recovery("no_recenter_candidate")
            return True
        yaw = math.atan2(goal.y - current[1], goal.x - current[0])
        self.current_heading_source = "slam_corridor_center"
        self.current_goal_clearance = goal.clearance_m
        self.events.append({
            "event": "explore_complete_recenter_selected",
            "goal": [goal.x, goal.y],
            "clearance_m": goal.clearance_m,
            "distance_m": goal.distance_m,
        })
        return self._send_navigation_goal((goal.x, goal.y), None, "explore_complete_recenter", yaw=yaw)

    def _finish_explore_complete_recovery(self, reason):
        self.recovery_pending = False
        self.events.append({
            "event": "m_explore_reset_resume",
            "reason": reason,
            "reminder": self.recovery_note,
        })
        if bool(self.get_parameter("m_explore_reset_on_complete").value):
            self.resume_pub.publish(Bool(data=True))

    def _m_explore_timed_out(self, now=None):
        limit = float(self.get_parameter("m_explore_max_active_s").value)
        if limit <= 0.0 or self.explore_active_since_wall is None:
            return False
        return (time.monotonic() if now is None else now) - self.explore_active_since_wall >= limit

    def _goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self._finish_goal("response_error", {"error": str(exc)})
            return
        self.goal_handle = handle
        if not handle.accepted:
            self._finish_goal("rejected", {})
            return
        handle.get_result_async().add_done_callback(self._goal_result)

    def _goal_result(self, future):
        try:
            status = int(future.result().status)
        except Exception as exc:
            self._finish_goal("result_error", {"error": str(exc)})
            return
        label = "succeeded" if status == GoalStatus.STATUS_SUCCEEDED else "ended"
        kind = self.current_goal_kind
        self._finish_goal(label, {"action_status": status})
        if kind == "explore_complete_recenter":
            self._finish_explore_complete_recovery(label)
        elif bool(self.get_parameter("resume_m_explore_after_fallback").value):
            self.resume_pub.publish(Bool(data=True))

    def _finish_goal(self, event, extra):
        details = {
            "event": event,
            "cell": self.current_cell,
            "goal": self.current_goal,
            "kind": self.current_goal_kind,
            "heading": self.current_heading_source,
            "clearance_m": self.current_goal_clearance,
        }
        details.update(extra)
        self.events.append(details)
        if event != "succeeded" and self.current_goal_kind in {"unknown_edge_probe", "unknown_edge_alternate"}:
            self._remember_dead_end(event)
        self.goal_active = False
        self.goal_handle = None
        self.last_goal_end_wall = time.monotonic()

    def _current_goal_refresh_s(self):
        return float(self.get_parameter("goal_refresh_s").value)

    def _next_goal(self):
        current = self._current_xy()
        if self.map is not None and current is not None:
            goal = self._next_unknown_edge_goal(current)
            if goal[0] is not None:
                return goal
        return None, None, None

    def _recenter_goal(self, current):
        if current is None or self.map is None:
            return None
        info = self.map.info
        return select_corridor_center_goal(
            self.map.data,
            int(info.width),
            int(info.height),
            float(info.resolution),
            float(info.origin.position.x),
            float(info.origin.position.y),
            current,
            search_radius_m=float(self.get_parameter("recenter_search_radius_m").value),
            candidate_step_m=float(self.get_parameter("recenter_candidate_step_m").value),
            clearance_radius_m=float(self.get_parameter("clearance_check_radius_m").value),
            clearance_sample_step_m=float(self.get_parameter("clearance_sample_step_m").value),
            min_clearance_m=float(self.get_parameter("recenter_min_clearance_m").value),
            min_step_m=float(self.get_parameter("recenter_min_step_m").value),
        )

    def _next_unknown_edge_goal(self, current):
        yaw, heading_source = self._preferred_yaw(current)
        if yaw is None:
            return None, None, None
        self.current_heading_source = heading_source
        minimum = float(self.get_parameter("min_goal_distance_m").value)
        lookahead = float(self.get_parameter("goal_lookahead_m").value)
        extension = float(self.get_parameter("unknown_extension_m").value)
        step = float(self.get_parameter("ray_step_m").value)
        samples = max(1, int(self.get_parameter("heading_samples").value))
        span = math.radians(float(self.get_parameter("max_heading_offset_deg").value))
        offsets = self._narrow_heading_offsets(samples, span)
        best, best_relaxed = self._scan_unknown_edge_candidates(
            current, yaw, offsets, minimum, lookahead, extension, step, "unknown_edge_probe"
        )
        wide_offsets = self._wide_heading_offsets(offsets)
        wide_best, wide_relaxed = self._scan_unknown_edge_candidates(
            current, yaw, wide_offsets, minimum, lookahead, extension, step, "unknown_edge_alternate"
        )
        if wide_best is not None and (best is None or wide_best[0] > best[0]):
            best = wide_best
        if wide_relaxed is not None and (
            best_relaxed is None
            or wide_relaxed[3] > best_relaxed[3]
            or (wide_relaxed[3] == best_relaxed[3] and wide_relaxed[0] > best_relaxed[0])
        ):
            best_relaxed = wide_relaxed

        if best is None:
            best = best_relaxed
        if best is None:
            return None, None, None
        point = self._clear_goal_toward(current, best[1])
        if point is None:
            return None, None, None
        return point, None, best[2]

    @staticmethod
    def _narrow_heading_offsets(samples, span):
        if samples == 1:
            return [0.0]
        offsets = [-span + (2.0 * span * index / (samples - 1)) for index in range(samples)]
        offsets.sort(key=abs)
        return offsets

    def _wide_heading_offsets(self, narrow_offsets):
        raw = str(self.get_parameter("wide_heading_offsets_deg").value)
        offsets = []
        for part in raw.replace(";", ",").split(","):
            try:
                offsets.append(math.radians(float(part.strip())))
            except ValueError:
                pass
        if not offsets:
            offsets = [math.radians(value) for value in (0, 30, -30, 60, -60, 90, -90, 135, -135, 180)]
        seen = {round(value, 6) for value in narrow_offsets}
        wide = []
        for offset in offsets:
            key = round(offset, 6)
            if key not in seen:
                wide.append(offset)
                seen.add(key)
        wide.sort(key=abs)
        return wide

    def _scan_unknown_edge_candidates(self, current, yaw, offsets, minimum, lookahead, extension, step, kind):
        best = None
        best_relaxed = None
        for offset in offsets:
            angle = yaw + offset
            direction = math.cos(angle), math.sin(angle)
            first_unknown = None
            distance = max(step, 0.05)
            while distance <= lookahead:
                value = self._map_value(current[0] + direction[0] * distance, current[1] + direction[1] * distance)
                if value >= 65:
                    break
                if value < 0:
                    first_unknown = distance
                    break
                distance += step

            if first_unknown is None:
                continue
            target_distance = min(lookahead, max(minimum, first_unknown + extension))
            point = (current[0] + direction[0] * target_distance, current[1] + direction[1] * target_distance)
            clearance = self._occupied_clearance(point)
            score = target_distance + 2.0 * clearance - 1.0 * abs(offset)
            score += self._direction_priority(current, point, direction)
            score -= self._dead_end_penalty(point, angle)
            candidate = (score, point, kind, clearance, angle)
            if best_relaxed is None or candidate[3] > best_relaxed[3] or (
                candidate[3] == best_relaxed[3] and candidate[0] > best_relaxed[0]
            ):
                best_relaxed = candidate
            if clearance >= self._min_goal_clearance() and (best is None or candidate[0] > best[0]):
                best = candidate
        return best, best_relaxed

    def _direction_priority(self, current, point, direction):
        score = float(self.get_parameter("positive_x_bias_weight").value) * float(direction[0])
        if self.start_xy is not None:
            before = math.hypot(current[0] - self.start_xy[0], current[1] - self.start_xy[1])
            after = math.hypot(point[0] - self.start_xy[0], point[1] - self.start_xy[1])
            score += float(self.get_parameter("start_away_bias_weight").value) * (after - before)
        return score

    def _remember_dead_end(self, reason):
        if self.current_goal is None:
            return
        current = self._current_xy()
        if current is None:
            return
        dx, dy = self.current_goal[0] - current[0], self.current_goal[1] - current[1]
        heading = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else self._current_yaw()
        self.dead_end_memory.append({
            "time": time.monotonic(),
            "goal": self.current_goal,
            "heading": heading,
            "reason": reason,
        })
        self._prune_dead_ends()

    def _prune_dead_ends(self):
        limit = float(self.get_parameter("dead_end_memory_s").value)
        if limit <= 0.0:
            self.dead_end_memory = []
            return
        cutoff = time.monotonic() - limit
        self.dead_end_memory = [entry for entry in self.dead_end_memory if entry["time"] >= cutoff]

    def _dead_end_penalty(self, point, angle):
        self._prune_dead_ends()
        radius = float(self.get_parameter("dead_end_radius_m").value)
        heading_limit = math.radians(float(self.get_parameter("dead_end_heading_deg").value))
        penalty = float(self.get_parameter("dead_end_penalty").value)
        total = 0.0
        for entry in self.dead_end_memory:
            goal = entry["goal"]
            heading = entry.get("heading")
            if math.hypot(point[0] - goal[0], point[1] - goal[1]) > radius:
                continue
            if heading is not None and abs(_angle_diff(angle, heading)) > heading_limit:
                continue
            total += penalty
        return total

    def _preferred_yaw(self, current):
        plan_age = time.monotonic() - self.last_plan_wall
        if self.last_plan and plan_age <= float(self.get_parameter("plan_heading_max_age_s").value):
            endpoint = self.last_plan[-1]
            dx, dy = endpoint[0] - current[0], endpoint[1] - current[1]
            if math.hypot(dx, dy) >= float(self.get_parameter("min_motion_heading_m").value):
                return math.atan2(dy, dx), "last_plan_tail"

        window = float(self.get_parameter("motion_heading_window_s").value)
        if len(self.pose_history) >= 2:
            latest = self.pose_history[-1]
            for sample in reversed(self.pose_history):
                if latest[0] - sample[0] >= window:
                    break
            dx, dy = latest[1] - sample[1], latest[2] - sample[2]
            if math.hypot(dx, dy) >= float(self.get_parameter("min_motion_heading_m").value):
                return math.atan2(dy, dx), "recent_motion"

        yaw = self._current_yaw()
        if yaw is not None:
            return yaw, "current_pose_yaw"

        return None, None

    def _map_value(self, x, y):
        info = self.map.info
        resolution = float(info.resolution)
        if resolution <= 0.0:
            return -1
        col = math.floor((x - float(info.origin.position.x)) / resolution)
        row = math.floor((y - float(info.origin.position.y)) / resolution)
        if col < 0 or row < 0 or col >= int(info.width) or row >= int(info.height):
            return -1
        return int(self.map.data[int(row) * int(info.width) + int(col)])

    def _min_goal_clearance(self):
        return float(self.get_parameter("min_goal_clearance_m").value)

    def _occupied_clearance(self, point, max_distance=None):
        if self.map is None:
            return float(max_distance if max_distance is not None else self.get_parameter("clearance_check_radius_m").value)
        limit = float(max_distance if max_distance is not None else self.get_parameter("clearance_check_radius_m").value)
        step = max(
            float(self.get_parameter("clearance_sample_step_m").value),
            float(self.map.info.resolution),
            0.05,
        )
        if self._map_value(point[0], point[1]) >= 65:
            return 0.0
        rings = max(1, int(math.ceil(limit / step)))
        for ring in range(1, rings + 1):
            radius = min(limit, ring * step)
            samples = max(8, int(math.ceil(2.0 * math.pi * radius / step)))
            for index in range(samples):
                angle = 2.0 * math.pi * index / samples
                value = self._map_value(point[0] + math.cos(angle) * radius, point[1] + math.sin(angle) * radius)
                if value >= 65:
                    return radius
        return limit

    def _clear_goal_toward(self, current, target):
        clearance = self._occupied_clearance(target)
        minimum = self._min_goal_clearance()
        if clearance >= minimum:
            return float(target[0]), float(target[1])

        dx, dy = target[0] - current[0], target[1] - current[1]
        distance = math.hypot(dx, dy)
        if distance <= 0.0:
            return None
        direction = dx / distance, dy / distance
        min_distance = min(distance, float(self.get_parameter("min_goal_distance_m").value))
        step = max(0.25, float(self.get_parameter("clearance_sample_step_m").value))
        best_point = None
        best_clearance = -1.0
        samples = max(1, int(math.ceil((distance - min_distance) / step)))
        for index in range(samples + 1):
            candidate_distance = max(min_distance, distance - index * step)
            point = (
                current[0] + direction[0] * candidate_distance,
                current[1] + direction[1] * candidate_distance,
            )
            candidate_clearance = self._occupied_clearance(point)
            if candidate_clearance > best_clearance:
                best_clearance = candidate_clearance
                best_point = point
            if candidate_clearance >= minimum:
                return float(point[0]), float(point[1])
        if best_point is not None and best_clearance >= 0.75 * minimum:
            return float(best_point[0]), float(best_point[1])
        return None

    def _current_xy(self):
        if self.map_pose is None:
            return None
        p = self.map_pose.pose.pose.position
        return float(p.x), float(p.y)

    def _slam_pose(self, msg):
        self.map_pose = msg
        p = msg.pose.pose.position
        now = time.monotonic()
        if self.start_xy is None:
            self.start_xy = (float(p.x), float(p.y))
        self.pose_history.append((now, float(p.x), float(p.y)))
        cutoff = now - max(8.0, float(self.get_parameter("motion_heading_window_s").value) + 1.0)
        self.pose_history = [sample for sample in self.pose_history if sample[0] >= cutoff]

    def _plan(self, msg):
        if self.last_explore_status not in ACTIVE_EXPLORE_STATUSES:
            return
        points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
        if points:
            self.last_plan = points
            self.last_plan_wall = time.monotonic()

    def _current_yaw(self):
        if self.map_pose is None:
            return None
        q = self.map_pose.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _pose(self, x, y, yaw=None):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        if yaw is None:
            pose.pose.orientation.w = 1.0
        else:
            pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
            pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return pose

    def _publish_status(self, status):
        self.status_pub.publish(String(data=json.dumps({
            "status": status,
            "algorithm": "m-explore-rolling-unknown-fallback",
            "m_explore_status": self.last_explore_status,
            "current_goal": self.current_goal,
            "current_cell": self.current_cell,
            "current_goal_kind": self.current_goal_kind,
            "current_heading_source": self.current_heading_source,
            "current_goal_clearance_m": self.current_goal_clearance,
            "current_pose_source": "slam_toolbox_pose" if self.map_pose is not None else None,
            "oracle_runtime_dependency": False,
            "m_explore_recovery_pending": self.recovery_pending,
            "m_explore_recovery_attempts": self.recovery_attempts,
            "m_explore_recovery_note": self.recovery_note,
            "start_xy": self.start_xy,
            "dead_end_memory_count": len(self.dead_end_memory),
            "events": self.events[-20:],
        }, sort_keys=True)))


def _angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def main():
    rclpy.init()
    node = MazeFallbackGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
