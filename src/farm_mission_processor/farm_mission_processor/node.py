"""ROS action servers for farm task planning and route execution."""

from __future__ import annotations

import math
import threading
import uuid
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from tf2_ros import Buffer, TransformListener, TransformException

from farm_mission_interfaces.action import ExecuteMission, PlanMission
from route_planning import (FarmTask, ImplementSide, PlannerConfig,
                            Pose, load_farm_map, plan_route)
from route_planning.model import DepotParkingState
from route_planning.diagnostic import load_route_json
from .route_progress import AnnotatedRoute, RouteProgressTracker


def to_ros_path(node: Node, poses: list[Pose]) -> RosPath:
    path = RosPath()
    path.header.frame_id = "map"
    path.header.stamp = node.get_clock().now().to_msg()
    for pose in poses:
        stamped = PoseStamped()
        stamped.header = path.header
        stamped.pose.position.x = pose.x
        stamped.pose.position.y = pose.y
        stamped.pose.orientation.z = math.sin(pose.yaw / 2)
        stamped.pose.orientation.w = math.cos(pose.yaw / 2)
        path.poses.append(stamped)
    return path


def task_from_goal(goal) -> FarmTask:
    def endpoint(prefix):
        kind = getattr(goal, f"{prefix}_type") or "pose"
        if kind == "depot":
            return DepotParkingState(getattr(goal, f"{prefix}_depot_spot"),
                                     getattr(goal, f"{prefix}_depot_direction") or "in")
        if kind == "pose":
            return Pose(*(getattr(goal, f"{prefix}_{axis}") for axis in ("x", "y", "yaw")))
        raise ValueError(f"Invalid {prefix}_type {kind}")
    side = ImplementSide(goal.implement_side or "both")
    implement_range = goal.implement_range_m or None
    return FarmTask(tuple(goal.field_ids), side, endpoint("start"), endpoint("end"),
                    implement_range)


class FarmMissionProcessor(Node):
    def __init__(self):
        super().__init__("farm_mission_processor")
        self.declare_parameter("farm", "debug_out/farms/farm_seed_1_120m2_depot")
        self.declare_parameter("mission_segmentation_strategy", "by_length")
        self.declare_parameter("by_length_m", 5.0)
        self.declare_parameter("overlap_distance_m", 1.0)
        self.declare_parameter("max_nav_pose_spacing_m", 0.2)
        self.declare_parameter("max_forward_search_m", 10.0)
        self.declare_parameter("solver_time_s", 60.0)
        self.declare_parameter("tractor_config_path", "configs/tractor_default.yaml")
        self.declare_parameter("injected_route_path", "")
        self._farm_path = Path(self.get_parameter("farm").value).expanduser().resolve()
        self._farm = None
        self._route = None
        self._mission_id = ""
        self._lock = threading.Lock()
        self._planning = False
        self._executing = False
        self._speed = math.inf
        self._group = ReentrantCallbackGroup()
        self._nav = ActionClient(self, FollowPath, "/follow_path", callback_group=self._group)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self, spin_thread=False)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10, callback_group=self._group)
        self._planner = ActionServer(
            self, PlanMission, "mission_planner", self._plan,
            goal_callback=self._plan_goal, cancel_callback=self._cancel,
            callback_group=self._group)
        self._executor = ActionServer(
            self, ExecuteMission, "mission_executor", self._execute,
            goal_callback=self._execute_goal, cancel_callback=self._cancel,
            callback_group=self._group)
        injected_path = self.get_parameter("injected_route_path").value
        if injected_path:
            self.inject_route(load_route_json(injected_path))

    def inject_route(self, route, mission_id: str = "injected"):
        """Test hook; only call while both action servers are idle."""
        with self._lock:
            if self._planning or self._executing:
                raise RuntimeError("Cannot inject a route during a mission")
            self._route, self._mission_id = route, mission_id

    def _on_odom(self, message):
        self._speed = abs(message.twist.twist.linear.x)

    def _plan_goal(self, request):
        with self._lock:
            if self._planning or self._executing:
                return GoalResponse.REJECT
            try:
                task_from_goal(request)
            except (ValueError, TypeError) as exc:
                self.get_logger().warning(f"Rejecting invalid task: {exc}")
                return GoalResponse.REJECT
            self._planning = True
            return GoalResponse.ACCEPT

    def _execute_goal(self, request):
        with self._lock:
            if self._executing or self._planning:
                return GoalResponse.REJECT
            if self._route is None or (request.mission_id and request.mission_id != self._mission_id):
                return GoalResponse.REJECT
            self._executing = True
            return GoalResponse.ACCEPT

    @staticmethod
    def _cancel(_goal):
        return CancelResponse.ACCEPT

    async def _background(self, function, *args):
        future = Future()
        def work():
            try:
                future.set_result(function(*args))
            except Exception as exc:
                future.set_exception(exc)
        threading.Thread(target=work, daemon=True).start()
        return await future

    async def _pause(self, seconds=0.05):
        future = Future()
        timer = self.create_timer(seconds, lambda: future.set_result(None)
                                  if not future.done() else None,
                                  callback_group=self._group)
        try:
            await future
        finally:
            self.destroy_timer(timer)

    async def _plan(self, goal_handle):
        result = PlanMission.Result()
        try:
            goal_handle.publish_feedback(PlanMission.Feedback(stage="loading_map"))
            if self._farm is None:
                self._farm = await self._background(load_farm_map, self._farm_path)
            if goal_handle.is_cancel_requested:
                raise InterruptedError("Planning canceled")
            goal_handle.publish_feedback(PlanMission.Feedback(stage="planning"))
            config = PlannerConfig(
                solver_time_s=self.get_parameter("solver_time_s").value,
                tractor_config_path=self.get_parameter("tractor_config_path").value)
            route = await self._background(plan_route, self._farm, task_from_goal(goal_handle.request),
                                           config, lambda: goal_handle.is_cancel_requested)
            if goal_handle.is_cancel_requested:
                raise InterruptedError("Planning canceled")
            mission_id = str(uuid.uuid4())
            with self._lock:
                self._route, self._mission_id = route, mission_id
            result.success = True
            result.message = route.optimality_status
            result.mission_id = mission_id
            result.total_distance_m = route.total_distance_m
            result.path = to_ros_path(self, route.continuous_path())
            goal_handle.succeed()
        except InterruptedError as exc:
            result.message = str(exc)
            goal_handle.canceled()
        except Exception as exc:
            result.message = str(exc)
            self.get_logger().error(f"Planning failed: {exc}")
            goal_handle.abort()
        finally:
            with self._lock:
                self._planning = False
        return result

    def _progress(self, tracker):
        try:
            transform = self._tf.lookup_transform("map", "base_link", rclpy.time.Time())
        except TransformException:
            return tracker.station_m
        p = transform.transform.translation
        return tracker.update(p.x, p.y)

    async def _send(self, path, direction):
        goal = FollowPath.Goal()
        goal.path = to_ros_path(self, path)
        goal.controller_id = "FollowPath" if direction > 0 else "FollowPathReverse"
        goal.goal_checker_id = "goal_checker"
        handle = await self._nav.send_goal_async(goal)
        if not handle.accepted:
            raise RuntimeError("Nav2 rejected FollowPath goal")
        return handle

    async def _execute(self, goal_handle):
        result = ExecuteMission.Result()
        nav_handle = None
        try:
            with self._lock:
                route = self._route
            annotated = AnnotatedRoute(route,
                self.get_parameter("mission_segmentation_strategy").value,
                self.get_parameter("by_length_m").value)
            overlap = self.get_parameter("overlap_distance_m").value
            if overlap < 0:
                raise ValueError("overlap_distance_m must be nonnegative")
            tracker = RouteProgressTracker(annotated, self.get_parameter("max_forward_search_m").value)
            while not self._nav.wait_for_server(timeout_sec=0.2):
                if goal_handle.is_cancel_requested:
                    raise InterruptedError("Execution canceled")
            sections = annotated.sections
            for index, section in enumerate(sections):
                start = section.start_m
                if index and section.direction == sections[index - 1].direction:
                    start = max(sections[index - 1].start_m, start - overlap)
                path = annotated.slice(start, section.end_m,
                                       self.get_parameter("max_nav_pose_spacing_m").value)
                nav_handle = await self._send(path, section.direction)
                result_future = nav_handle.get_result_async()
                next_section = sections[index + 1] if index + 1 < len(sections) else None
                seamless = next_section is not None and next_section.direction == section.direction
                trigger = section.end_m - min(overlap, (section.end_m - section.start_m) / 2)
                while True:
                    if goal_handle.is_cancel_requested:
                        raise InterruptedError("Execution canceled")
                    progress = self._progress(tracker)
                    goal_handle.publish_feedback(ExecuteMission.Feedback(
                        distance_traveled_m=progress,
                        distance_remaining_m=max(0.0, annotated.length_m - progress),
                        section_index=index))
                    if seamless and progress >= trigger:
                        break
                    if result_future.done():
                        outcome = result_future.result()
                        if outcome.status != GoalStatus.STATUS_SUCCEEDED:
                            raise RuntimeError(f"Nav2 FollowPath ended with status {outcome.status}")
                        break
                    await self._pause()
                if seamless:
                    # Nav2 accepts the next path as a preemption. Its overlap keeps
                    # the robot on the reference path while the controller switches.
                    continue
                if not result_future.done():
                    while not result_future.done():
                        if goal_handle.is_cancel_requested:
                            raise InterruptedError("Execution canceled")
                        await self._pause()
                    outcome = result_future.result()
                    if outcome.status != GoalStatus.STATUS_SUCCEEDED:
                        raise RuntimeError(f"Nav2 FollowPath ended with status {outcome.status}")
                if next_section and next_section.direction != section.direction:
                    while self._speed > 0.05:
                        if goal_handle.is_cancel_requested:
                            raise InterruptedError("Execution canceled")
                        await self._pause()
            result.success = True
            result.message = "Mission completed"
            goal_handle.succeed()
        except InterruptedError as exc:
            if nav_handle is not None:
                await nav_handle.cancel_goal_async()
            result.message = str(exc)
            goal_handle.canceled()
        except Exception as exc:
            if nav_handle is not None:
                await nav_handle.cancel_goal_async()
            result.message = str(exc)
            self.get_logger().error(f"Execution failed: {exc}")
            goal_handle.abort()
        finally:
            with self._lock:
                self._executing = False
        return result


def main():
    rclpy.init()
    node = FarmMissionProcessor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
