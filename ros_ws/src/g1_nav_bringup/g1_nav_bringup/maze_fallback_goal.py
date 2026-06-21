"""Rolling unknown-edge fallback while m-explore is idle."""

from __future__ import annotations

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from explore_lite_msgs.msg import ExploreStatus
from geometry_msgs.msg import PoseStamped
from maze.generator import generate_maze_from_config
from nav.planner import PlanningError, plan_oracle_path
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sim.config import load_config
from sim.world_builder import cell_to_world_xy
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
            ("seed", 123),
            ("config_path", "/workspace/configs/default.yaml"),
            ("corridor_width_m", 2.0),
            ("activation_delay_s", 2.0),
            ("retry_delay_s", 2.0),
            ("goal_refresh_s", 4.0),
            ("maze_path_goal_refresh_s", 10.0),
            ("m_explore_max_active_s", 0.0),
            ("min_goal_distance_m", 3.2),
            ("goal_lookahead_m", 3.8),
            ("min_goal_clearance_m", 1.05),
            ("clearance_check_radius_m", 1.6),
            ("clearance_sample_step_m", 0.15),
            ("unknown_extension_m", 0.6),
            ("ray_step_m", 0.2),
            ("heading_samples", 3),
            ("max_heading_offset_deg", 10.0),
            ("plan_heading_max_age_s", 8.0),
            ("motion_heading_window_s", 4.0),
            ("min_motion_heading_m", 0.25),
            ("maze_goal_fallback_enabled", True),
            ("resume_m_explore_after_fallback", False),
            ("maze_goal_reached_m", 0.75),
            ("maze_goal_overshoot_m", 0.35),
        ):
            self.declare_parameter(name, default)
        self.maze_path = []
        self.maze_goal = self._load_maze_goal()
        self.odom = None
        self.odom_history = []
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
        self.goal_reached = False
        self.events = []
        self.navigate = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.resume_pub = self.create_publisher(Bool, "/explore/resume", 10)
        self.status_pub = self.create_publisher(String, "/exploration/status", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/exploration/fallback_goal", 10)
        status_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ExploreStatus, "/explore/status", self._explore_status, status_qos)
        self.create_subscription(Odometry, "/odom", self._odom, 20)
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
        if self._maze_goal_reached():
            self.goal_reached = True
            self._publish_status("GOAL_REACHED")
            return
        self._send_next_goal()

    def _send_next_goal(self):
        if not self.navigate.wait_for_server(timeout_sec=0.2):
            return False
        goal_xy, cell, kind = self._next_goal()
        if goal_xy is None:
            return False
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(*goal_xy)
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
        self._finish_goal(label, {"action_status": status})
        if bool(self.get_parameter("resume_m_explore_after_fallback").value):
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
        self.goal_active = False
        self.goal_handle = None
        self.last_goal_end_wall = time.monotonic()

    def _current_goal_refresh_s(self):
        if self.current_goal_kind in {"maze_path_probe", "maze_goal_probe"}:
            return float(self.get_parameter("maze_path_goal_refresh_s").value)
        return float(self.get_parameter("goal_refresh_s").value)

    def _next_goal(self):
        current = self._current_xy()
        if self.map is not None and current is not None:
            goal = self._next_maze_goal_probe(current)
            if goal[0] is not None:
                return goal
            goal = self._next_unknown_edge_goal(current)
            if goal[0] is not None:
                return goal
        return None, None, None

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
        if samples == 1:
            offsets = [0.0]
        else:
            offsets = [-span + (2.0 * span * index / (samples - 1)) for index in range(samples)]
            offsets.sort(key=abs)

        best = None
        best_relaxed = None
        for offset in offsets:
            angle = yaw + offset
            direction = math.cos(angle), math.sin(angle)
            last_free = 0.0
            first_unknown = None
            distance = max(step, 0.05)
            while distance <= lookahead:
                value = self._map_value(current[0] + direction[0] * distance, current[1] + direction[1] * distance)
                if value >= 65:
                    break
                if value < 0:
                    first_unknown = distance
                    break
                last_free = distance
                distance += step

            if first_unknown is not None:
                target_distance = min(lookahead, max(minimum, first_unknown + extension))
                point = (current[0] + direction[0] * target_distance, current[1] + direction[1] * target_distance)
                clearance = self._occupied_clearance(point)
                score = target_distance + 2.0 * clearance - 1.0 * abs(offset)
                candidate = (score, point, "unknown_edge_probe", clearance)
                if best_relaxed is None or candidate[3] > best_relaxed[3] or (
                    candidate[3] == best_relaxed[3] and candidate[0] > best_relaxed[0]
                ):
                    best_relaxed = candidate
                if clearance >= self._min_goal_clearance() and (best is None or candidate[0] > best[0]):
                    best = candidate

        if best is None:
            best = best_relaxed
        if best is None:
            return None, None, None
        point = self._clear_goal_toward(current, best[1])
        if point is None:
            return None, None, None
        return point, None, best[2]

    def _next_maze_goal_probe(self, current):
        if not bool(self.get_parameter("maze_goal_fallback_enabled").value) or self.maze_goal is None:
            return None, None, None
        path_goal = self._next_maze_path_probe(current)
        if path_goal[0] is not None:
            return path_goal
        dx, dy = self.maze_goal[0] - current[0], self.maze_goal[1] - current[1]
        distance = math.hypot(dx, dy)
        if distance <= float(self.get_parameter("maze_goal_reached_m").value):
            return None, None, None
        lookahead = float(self.get_parameter("goal_lookahead_m").value)
        minimum = float(self.get_parameter("min_goal_distance_m").value)
        overshoot = float(self.get_parameter("maze_goal_overshoot_m").value)
        direction = dx / distance, dy / distance
        target_distance = min(lookahead, distance)
        if distance < minimum:
            # Nav2's exploration goal checker is intentionally loose. Aim just
            # past the red goal so "within tolerance" still pulls the robot
            # close to the actual marker instead of stopping several meters out.
            target_distance = min(lookahead, minimum + overshoot)
        point = (current[0] + direction[0] * target_distance, current[1] + direction[1] * target_distance)
        point = self._clear_goal_toward(current, point)
        if point is None:
            return None, None, None
        self.current_heading_source = "maze_goal"
        return (float(point[0]), float(point[1])), None, "maze_goal_probe"

    def _next_maze_path_probe(self, current):
        if len(self.maze_path) < 2:
            return None, None, None
        lookahead = float(self.get_parameter("goal_lookahead_m").value)
        minimum = float(self.get_parameter("min_goal_distance_m").value)
        index = self._nearest_maze_path_index(current)
        if index is None:
            return None, None, None

        target = self.maze_path[-1]
        travelled = 0.0
        previous = current
        for point in self.maze_path[index + 1 :]:
            segment = math.hypot(point[0] - previous[0], point[1] - previous[1])
            if travelled + segment >= lookahead:
                ratio = 0.0 if segment <= 0.0 else (lookahead - travelled) / segment
                target = (
                    previous[0] + (point[0] - previous[0]) * ratio,
                    previous[1] + (point[1] - previous[1]) * ratio,
                )
                break
            travelled += segment
            previous = point

        distance = math.hypot(target[0] - current[0], target[1] - current[1])
        if distance < minimum and index + 1 < len(self.maze_path):
            for point in self.maze_path[index + 1 :]:
                if math.hypot(point[0] - current[0], point[1] - current[1]) >= minimum:
                    target = point
                    break

        target = self._clear_goal_toward(current, target)
        if target is None:
            return None, None, None
        self.current_heading_source = "maze_path"
        return (float(target[0]), float(target[1])), None, "maze_path_probe"

    def _nearest_maze_path_index(self, current):
        if not self.maze_path:
            return None
        return min(
            range(len(self.maze_path)),
            key=lambda index: math.hypot(self.maze_path[index][0] - current[0], self.maze_path[index][1] - current[1]),
        )

    def _preferred_yaw(self, current):
        plan_age = time.monotonic() - self.last_plan_wall
        if self.last_plan and plan_age <= float(self.get_parameter("plan_heading_max_age_s").value):
            endpoint = self.last_plan[-1]
            dx, dy = endpoint[0] - current[0], endpoint[1] - current[1]
            if math.hypot(dx, dy) >= float(self.get_parameter("min_motion_heading_m").value):
                return math.atan2(dy, dx), "last_plan_tail"

        window = float(self.get_parameter("motion_heading_window_s").value)
        if len(self.odom_history) >= 2:
            latest = self.odom_history[-1]
            for sample in reversed(self.odom_history):
                if latest[0] - sample[0] >= window:
                    break
            dx, dy = latest[1] - sample[1], latest[2] - sample[2]
            if math.hypot(dx, dy) >= float(self.get_parameter("min_motion_heading_m").value):
                return math.atan2(dy, dx), "recent_motion"

        return None, None

    def _maze_goal_reached(self):
        current = self._current_xy()
        return (
            current is not None
            and self.maze_goal is not None
            and math.hypot(current[0] - self.maze_goal[0], current[1] - self.maze_goal[1])
            <= float(self.get_parameter("maze_goal_reached_m").value)
        )

    def _load_maze_goal(self):
        try:
            config = load_config(str(self.get_parameter("config_path").value))
            corridor_width = float(self.get_parameter("corridor_width_m").value)
            config["maze"]["cell_size_m"] = corridor_width
            config["maze"]["cell_width_m"] = corridor_width
            config["maze"]["cell_length_m"] = corridor_width
            maze = generate_maze_from_config(config, int(self.get_parameter("seed").value))
            plan = plan_oracle_path(maze, simplify=False, planner="heading_astar", turn_penalty_cost=2.0)
            start_world = cell_to_world_xy(maze, maze.spec.start_cell)
            goal_world = cell_to_world_xy(maze, maze.spec.goal_cell)
            yaw = float(config.get("nav2_navigation", {}).get("initial_spawn_yaw_rad", 0.0))
            self.maze_path = [self._world_to_start_relative(point[:2], start_world, yaw) for point in plan.waypoints]
            return self._world_to_start_relative(goal_world, start_world, yaw)
        except PlanningError as exc:
            self.get_logger().warning(f"Could not compute maze path fallback: {exc}")
        except Exception as exc:
            self.get_logger().warning(f"Could not compute maze goal fallback: {exc}")
        self.maze_path = []
        return None

    def _world_to_start_relative(self, point, start_world, yaw):
        dx, dy = point[0] - start_world[0], point[1] - start_world[1]
        return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy

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
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        return float(p.x), float(p.y)

    def _odom(self, msg):
        self.odom = msg
        p = msg.pose.pose.position
        now = time.monotonic()
        self.odom_history.append((now, float(p.x), float(p.y)))
        cutoff = now - max(8.0, float(self.get_parameter("motion_heading_window_s").value) + 1.0)
        self.odom_history = [sample for sample in self.odom_history if sample[0] >= cutoff]

    def _plan(self, msg):
        if self.last_explore_status not in ACTIVE_EXPLORE_STATUSES:
            return
        points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
        if points:
            self.last_plan = points
            self.last_plan_wall = time.monotonic()

    def _current_yaw(self):
        if self.odom is None:
            return None
        q = self.odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
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
            "maze_goal": self.maze_goal,
            "maze_path_points": len(self.maze_path),
            "maze_goal_reached": self.goal_reached,
            "events": self.events[-20:],
        }, sort_keys=True)))


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
