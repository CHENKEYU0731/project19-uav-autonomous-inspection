#!/usr/bin/env python3

# Copyright 2026 Project19 contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Insert the project-owned M3 blocker after the real stack has progressed."""

import json
import math
import subprocess
import threading
import time
from typing import Optional

import rclpy
from drone_interfaces.msg import PlannedTrajectory, PlannerStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, String


ACTIVE_INSERTION_TIMEOUT_S = 6.0
HOLD_SETTLE_SPEED_MPS = 0.1
HOLD_SETTLE_TIME_S = 0.5
HOLD_SETTLE_TIMEOUT_S = 5.0


def removal_is_ready(
    remove_after_progress: float,
    progress_m: Optional[float],
    hold_active: bool,
    inserted: bool,
    replan_confirmed: bool,
    removed: bool,
    removal_in_progress: bool,
) -> bool:
    return bool(
        remove_after_progress > 0.0
        and progress_m is not None
        and math.isfinite(progress_m)
        and progress_m >= remove_after_progress
        and not hold_active
        and inserted
        and replan_confirmed
        and not removed
        and not removal_in_progress
    )


class DynamicBlocker:
    def __init__(self) -> None:
        self.node = rclpy.create_node("m3_dynamic_blocker")
        declare = self.node.declare_parameter
        self.name = str(declare("blocker_name", "m3_dynamic_blocker").value)
        self.initial_x = float(declare("initial_x_m", 0.0).value)
        self.initial_y = float(declare("initial_y_m", -3.0).value)
        self.active_x = float(declare("active_x_m", 0.0).value)
        self.active_y = float(declare("active_y_m", 1.5).value)
        self.height = float(declare("height_m", 1.5).value)
        self.trigger_progress = float(declare("trigger_progress_m", 0.5).value)
        self.required_maps = int(declare("required_map_observations", 1).value)
        self.status_timeout_s = float(declare("status_timeout_s", 1.0).value)
        self.minimum_lead = float(declare("minimum_insertion_lead_m", 0.6).value)
        self.remove_after_progress = float(
            declare("remove_after_progress_m", -1.0).value
        )
        self.service_name = str(
            declare("set_pose_service", "/world/inspection/set_pose").value
        )
        if (
            self.required_maps < 1
            or self.trigger_progress <= 0.0
            or self.status_timeout_s <= 0.0
            or self.minimum_lead <= 0.0
            or self.active_y - self.trigger_progress <= self.minimum_lead
            or (
                self.remove_after_progress > 0.0
                and self.remove_after_progress <= self.active_y + self.minimum_lead
            )
        ):
            raise ValueError("dynamic blocker trigger window is invalid")

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pose_publisher = self.node.create_publisher(
            PoseStamped, "/drone_m3/dynamic_blocker_pose", latched
        )
        self.event_publisher = self.node.create_publisher(
            String, "/drone_m3/dynamic_blocker_event", latched
        )
        self.hold_publisher = self.node.create_publisher(
            Bool, "/drone_m3/insertion_hold", latched
        )
        self.node.create_subscription(
            PlannedTrajectory,
            "/drone_planner/trajectory",
            self._handle_trajectory,
            10,
        )
        self.node.create_subscription(
            PlannerStatus, "/drone_planner/status", self._handle_status, 10
        )
        self.node.create_subscription(
            OccupancyGrid, "/local_occupancy_grid", self._handle_map, 10
        )
        self.node.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._handle_position,
            qos_profile_sensor_data,
        )
        self.latest_trajectory_id = 0
        self.latest_trajectory_received_at: Optional[float] = None
        self.safe_trajectory_id = 0
        self.safe_status_received_at: Optional[float] = None
        self.progress_m: Optional[float] = None
        self.forward_speed_mps: Optional[float] = None
        self.progress_threshold_reached = False
        self.map_count_after_progress = 0
        self.inserted = False
        self.replan_confirmed = False
        self.insertion_trajectory_id = 0
        self.insertion_confirmed_at: Optional[float] = None
        self.hold_active = False
        self.failed = False
        self.exit_code = 0
        self.insertion_thread: Optional[threading.Thread] = None
        self.hold_requested_at: Optional[float] = None
        self.hold_settled_at: Optional[float] = None
        self.insertion_started_at: Optional[float] = None
        self.insertion_result: Optional[bool] = None
        self.removed = False
        self.removal_thread: Optional[threading.Thread] = None
        self.removal_started_at: Optional[float] = None
        self.removal_result: Optional[bool] = None
        if not self._set_pose_with_retry(self.initial_x, self.initial_y):
            raise RuntimeError(
                "failed to place the dynamic blocker at its initial pose"
            )
        self._publish_pose(self.initial_x, self.initial_y)
        self._publish_event("blocker_initialized", self.initial_x, self.initial_y)
        self._publish_hold(False)
        self.timer = self.node.create_timer(0.1, self._tick)

    def _handle_trajectory(self, message: PlannedTrajectory) -> None:
        if message.trajectory_id > 0:
            self.latest_trajectory_id = message.trajectory_id
            self.latest_trajectory_received_at = time.monotonic()

    def _handle_status(self, message: PlannerStatus) -> None:
        self.safe_trajectory_id = 0
        self.safe_status_received_at = None
        if message.trajectory_id == 0:
            return
        if message.state in (PlannerStatus.READY, PlannerStatus.GOAL_REACHED):
            if message.map_fresh and message.trajectory_valid:
                self.safe_trajectory_id = message.trajectory_id
                self.safe_status_received_at = time.monotonic()

    def _handle_map(self, _message: OccupancyGrid) -> None:
        if self.progress_threshold_reached and not self.inserted:
            self.map_count_after_progress += 1

    def _handle_position(self, message: VehicleLocalPosition) -> None:
        if not message.xy_valid or not message.z_valid:
            return
        # PX4 local x is north; the M3 map route advances along map +y.
        self.progress_m = float(message.x)
        self.forward_speed_mps = (
            float(message.vx)
            if message.v_xy_valid and math.isfinite(message.vx)
            else None
        )
        if (
            not self.progress_threshold_reached
            and self.progress_m >= self.trigger_progress
        ):
            self.progress_threshold_reached = True
            self.map_count_after_progress = 0

    def _tick(self) -> None:
        if self.insertion_thread is not None:
            if self.insertion_thread.is_alive():
                if (
                    self.insertion_started_at is not None
                    and time.monotonic() - self.insertion_started_at
                    > ACTIVE_INSERTION_TIMEOUT_S
                ):
                    self._fail_insertion("set_pose response timed out")
                return
            self.insertion_thread = None
            self.insertion_started_at = None
            if self.insertion_result is True and self._insertion_is_ahead():
                self.inserted = True
                self.insertion_confirmed_at = time.monotonic()
                self._publish_pose(self.active_x, self.active_y)
                self._publish_event(
                    "blocker_inserted",
                    self.active_x,
                    self.active_y,
                    trajectory_id=self.insertion_trajectory_id,
                    safe_trajectory_id=self.insertion_trajectory_id,
                )
            else:
                reason = (
                    "set_pose rejected"
                    if self.insertion_result is not True
                    else "insertion window missed"
                )
                self._fail_insertion(reason)
            return
        if self.removal_thread is not None:
            if self.removal_thread.is_alive():
                if (
                    self.removal_started_at is not None
                    and time.monotonic() - self.removal_started_at
                    > ACTIVE_INSERTION_TIMEOUT_S
                ):
                    self._fail_removal("set_pose response timed out")
                return
            self.removal_thread = None
            self.removal_started_at = None
            if self.removal_result is True:
                self.removed = True
                self._publish_pose(self.initial_x, self.initial_y)
                self._publish_event("blocker_removed", self.initial_x, self.initial_y)
            else:
                self._fail_removal("set_pose rejected")
            return
        if self.removed:
            self._publish_pose(self.initial_x, self.initial_y)
            return
        if (
            not self.inserted
            and self.hold_requested_at is None
            and self._trigger_ready()
        ):
            self._publish_hold(True)
            self.insertion_trajectory_id = self.latest_trajectory_id
            self.hold_requested_at = time.monotonic()
            self._publish_event("blocker_hold_started", self.active_x, self.active_y)
            return
        if not self.inserted and self.hold_requested_at is not None:
            self._advance_insertion_hold()
            return
        if not self.inserted and self._insertion_window_missed():
            self._fail_insertion("insertion window missed")
            return
        if self.inserted:
            self._publish_pose(self.active_x, self.active_y)
            if (
                self.hold_active
                and self.latest_trajectory_id != self.insertion_trajectory_id
                and self.safe_trajectory_id == self.latest_trajectory_id
                and self.insertion_confirmed_at is not None
                and self.latest_trajectory_received_at is not None
                and self.latest_trajectory_received_at >= self.insertion_confirmed_at
                and self.safe_status_received_at is not None
                and self.safe_status_received_at >= self.insertion_confirmed_at
                and time.monotonic() - self.safe_status_received_at
                <= self.status_timeout_s
            ):
                self._publish_event(
                    "blocker_replan_confirmed", self.active_x, self.active_y
                )
                self.replan_confirmed = True
                self._publish_hold(False)
            if removal_is_ready(
                self.remove_after_progress,
                self.progress_m,
                self.hold_active,
                self.inserted,
                self.replan_confirmed,
                self.removed,
                self.removal_thread is not None,
            ):
                self._publish_event(
                    "blocker_removal_started", self.active_x, self.active_y
                )
                self.removal_result = None
                self.removal_started_at = time.monotonic()
                self.removal_thread = threading.Thread(
                    target=self._move_removed_blocker,
                    daemon=True,
                )
                self.removal_thread.start()

    def _advance_insertion_hold(self) -> None:
        now = time.monotonic()
        if now - self.hold_requested_at > HOLD_SETTLE_TIMEOUT_S:
            self._fail_insertion("vehicle did not settle before insertion")
            return
        if self.latest_trajectory_id != self.insertion_trajectory_id:
            self._fail_insertion("trajectory changed while waiting for insertion hold")
            return
        status_is_current = bool(
            self.safe_trajectory_id == self.insertion_trajectory_id
            and self.safe_status_received_at is not None
            and now - self.safe_status_received_at <= self.status_timeout_s
        )
        vehicle_is_settled = bool(
            self.forward_speed_mps is not None
            and abs(self.forward_speed_mps) <= HOLD_SETTLE_SPEED_MPS
            and self._insertion_is_ahead()
        )
        if not status_is_current or not vehicle_is_settled:
            self.hold_settled_at = None
            return
        if self.hold_settled_at is None:
            self.hold_settled_at = now
            return
        if now - self.hold_settled_at < HOLD_SETTLE_TIME_S:
            return

        self._publish_event("blocker_insertion_started", self.active_x, self.active_y)
        self.insertion_result = None
        self.insertion_started_at = now
        self.insertion_thread = threading.Thread(
            target=self._move_active_blocker,
            daemon=True,
        )
        self.insertion_thread.start()

    def _trigger_ready(self) -> bool:
        return bool(
            self.latest_trajectory_id > 0
            and self.safe_trajectory_id == self.latest_trajectory_id
            and self.progress_threshold_reached
            and self.map_count_after_progress >= self.required_maps
            and self.safe_status_received_at is not None
            and time.monotonic() - self.safe_status_received_at <= self.status_timeout_s
            and self._insertion_is_ahead()
        )

    def _insertion_is_ahead(self) -> bool:
        return bool(
            self.progress_m is not None
            and self.active_y - self.progress_m >= self.minimum_lead
        )

    def _insertion_window_missed(self) -> bool:
        return bool(
            self.progress_threshold_reached
            and self.progress_m is not None
            and self.active_y - self.progress_m < self.minimum_lead
        )

    def _fail_insertion(self, reason: str) -> None:
        self.failed = True
        self.exit_code = 1
        self._publish_event("blocker_insertion_failed", self.active_x, self.active_y)
        self.node.get_logger().error(f"Gazebo blocker insertion failed: {reason}")
        rclpy.shutdown()

    def _fail_removal(self, reason: str) -> None:
        self.failed = True
        self.exit_code = 1
        self._publish_event("blocker_removal_failed", self.active_x, self.active_y)
        self.node.get_logger().error(f"Gazebo blocker removal failed: {reason}")
        rclpy.shutdown()

    def _publish_hold(self, active: bool) -> None:
        self.hold_active = active
        self.hold_publisher.publish(Bool(data=active))

    def _set_pose_with_retry(self, x: float, y: float) -> bool:
        for attempt in range(10):
            if self._move_blocker(x, y):
                return True
            if attempt < 9:
                time.sleep(0.5)
        return False

    def _move_active_blocker(self) -> None:
        self.insertion_result = self._move_blocker(self.active_x, self.active_y)

    def _move_removed_blocker(self) -> None:
        self.removal_result = self._move_blocker(self.initial_x, self.initial_y)

    def _move_blocker(self, x: float, y: float) -> bool:
        request = (
            f'name: "{self.name}", '
            f"position: {{x: {x}, y: {y}, z: {self.height}}}, "
            "orientation: {w: 1.0}"
        )
        command = [
            "gz",
            "service",
            "-s",
            self.service_name,
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            request,
        ]
        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=5.0,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            self.node.get_logger().error(f"set_pose invocation failed: {error}")
            return False
        output = f"{result.stdout}\n{result.stderr}".lower()
        self.node.get_logger().info(f"set_pose response: {output.strip()}")
        return result.returncode == 0 and "true" in output

    def _publish_pose(self, x: float, y: float) -> None:
        message = PoseStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = self.height
        message.pose.orientation.w = 1.0
        self.pose_publisher.publish(message)

    def _publish_event(
        self,
        event: str,
        x: float,
        y: float,
        trajectory_id: Optional[int] = None,
        safe_trajectory_id: Optional[int] = None,
    ) -> None:
        payload = {
            "event": event,
            "blocker_name": self.name,
            "blocker_xy_m": [x, y],
            "trajectory_id": (
                self.latest_trajectory_id if trajectory_id is None else trajectory_id
            ),
            "safe_trajectory_id": (
                self.safe_trajectory_id
                if safe_trajectory_id is None
                else safe_trajectory_id
            ),
            "progress_m": self.progress_m,
            "map_count_after_progress": self.map_count_after_progress,
            "minimum_insertion_lead_m": self.minimum_lead,
            "set_pose_service": self.service_name,
        }
        self.event_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))


def main() -> int:
    rclpy.init()
    blocker: Optional[DynamicBlocker] = None
    try:
        blocker = DynamicBlocker()
        rclpy.spin(blocker.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as error:  # pragma: no cover - launch-side fatal path
        if blocker is not None:
            blocker.node.get_logger().error(f"dynamic blocker fatal error: {error}")
            blocker.exit_code = 1
        else:
            print(f"dynamic blocker fatal error: {error}")
        return 1
    finally:
        if blocker is not None:
            blocker.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return blocker.exit_code if blocker is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
