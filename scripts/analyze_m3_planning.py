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

"""Independently validate an M3 dynamic replanning flight bag."""

import argparse
from bisect import bisect_right
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path


TRAJECTORY_TOPIC = "/drone_planner/trajectory"
STATUS_TOPIC = "/drone_planner/status"
EVENT_TOPIC = "/drone_m3/dynamic_blocker_event"
TF_TOPIC = "/tf"
SETPOINT_TOPIC = "/fmu/in/trajectory_setpoint"
VEHICLE_STATUS_TOPIC = "/fmu/out/vehicle_status_v1"
LAND_TOPIC = "/fmu/out/vehicle_land_detected"
CONTACTS_TOPIC = "/world/inspection/contacts"

READY = 5
GOAL_REACHED = 7
ARMING_STATE_DISARMED = 1
ARMING_STATE_ARMED = 2
NAVIGATION_STATE_OFFBOARD = 14
VEHICLE_RADIUS_M = 0.35
BLOCKER_RADIUS_M = 0.22
REQUIRED_CENTER_CLEARANCE_M = VEHICLE_RADIUS_M + BLOCKER_RADIUS_M
MAX_STATUS_AGE_NS = 600_000_000
MAX_REPLAN_LATENCY_NS = 3_000_000_000
GOAL_X_M = 0.0
GOAL_Y_M = 3.0
GOAL_Z_M = 2.5
GOAL_TOLERANCE_M = 0.30
ALTITUDE_TOLERANCE_M = 0.35
MOTION_VELOCITY_THRESHOLD_M_S = 0.02
MOTION_POSITION_THRESHOLD_M = 0.02

RUNTIME_SOURCE_PATHS = (
    "scripts/analyze_m3_planning.py",
    "scripts/m3_dynamic_blocker.py",
    "scripts/run_m3_randomized_evaluation.py",
    "scripts/run-px4-sitl.sh",
    "scripts/px4-headless-rcS",
    "src/drone_bringup/config/m3_mapping.yaml",
    "src/drone_bringup/config/m3_mission.yaml",
    "src/drone_bringup/config/m3_planner.yaml",
    "src/drone_bringup/launch/local_mapping.launch.py",
    "src/drone_bringup/launch/m3_autonomy.launch.py",
    "src/drone_controller/src/waypoint_controller_node.cpp",
    "src/drone_perception/src/depth_grid_node.cpp",
    "src/drone_perception/src/px4_tf_broadcaster_node.cpp",
    "src/drone_planner/src/a_star.cpp",
    "src/drone_planner/src/grid_map.cpp",
    "src/drone_planner/src/planner_node.cpp",
    "src/drone_planner/src/trajectory.cpp",
    "src/drone_sim/worlds/inspection.sdf",
)


@dataclass
class TrajectorySample:
    timestamp_ns: int
    trajectory_id: int
    points: tuple


@dataclass
class StatusSample:
    timestamp_ns: int
    trajectory_id: int
    state: int
    map_fresh: bool
    trajectory_valid: bool


@dataclass
class BlockerEvent:
    timestamp_ns: int
    event: str
    x: float
    y: float
    trajectory_id: int = 0
    safe_trajectory_id: int = 0


@dataclass
class PoseSample:
    timestamp_ns: int
    x: float
    y: float
    z: float


@dataclass
class SetpointSample:
    timestamp_ns: int
    position_ned: tuple
    velocity_ned: tuple


@dataclass
class VehicleStatusSample:
    timestamp_ns: int
    arming_state: int
    nav_state: int
    failsafe: bool


@dataclass
class LandSample:
    timestamp_ns: int
    landed: bool


@dataclass
class PlanningEvidence:
    trajectories: list
    statuses: list
    events: list
    poses: list
    setpoints: list
    vehicle_statuses: list
    land_samples: list
    collision_pairs: list
    contacts_topic_present: bool


def parse_args():
    parser = argparse.ArgumentParser(
        description="Validate an M3 dynamic replanning ROS 2 bag."
    )
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory")
    parser.add_argument("--metrics", required=True, type=Path)
    parser.add_argument("--launch-exit-code", required=True, type=int)
    parser.add_argument("--expected-blocker-x-m", type=float)
    parser.add_argument("--expected-blocker-y-m", type=float)
    return parser.parse_args()


def finite_tuple(values):
    return all(math.isfinite(value) for value in values)


def normalized_frame(frame_id):
    return frame_id.lstrip("/")


def read_bag(bag_path):
    if not bag_path.is_dir() or not (bag_path / "metadata.yaml").is_file():
        raise RuntimeError(f"not a ROS 2 bag directory: {bag_path}")

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {
        topic.name: get_message(topic.type)
        for topic in reader.get_all_topics_and_types()
    }
    required = {
        TRAJECTORY_TOPIC,
        STATUS_TOPIC,
        EVENT_TOPIC,
        TF_TOPIC,
        SETPOINT_TOPIC,
        VEHICLE_STATUS_TOPIC,
        LAND_TOPIC,
        CONTACTS_TOPIC,
    }
    missing = sorted(required - topic_types.keys())
    if missing:
        raise RuntimeError("bag is missing required M3 topics: " + ", ".join(missing))

    evidence = PlanningEvidence([], [], [], [], [], [], [], [], True)
    while reader.has_next():
        topic, serialized, received_ns = reader.read_next()
        if topic not in required:
            continue
        message = deserialize_message(serialized, topic_types[topic])
        if topic == TRAJECTORY_TOPIC:
            points = tuple(
                (
                    float(point.transforms[0].translation.x),
                    float(point.transforms[0].translation.y),
                    float(point.transforms[0].translation.z),
                )
                for point in message.trajectory.points
                if len(point.transforms) == 1
            )
            evidence.trajectories.append(
                TrajectorySample(received_ns, int(message.trajectory_id), points)
            )
        elif topic == STATUS_TOPIC:
            evidence.statuses.append(
                StatusSample(
                    received_ns,
                    int(message.trajectory_id),
                    int(message.state),
                    bool(message.map_fresh),
                    bool(message.trajectory_valid),
                )
            )
        elif topic == EVENT_TOPIC:
            try:
                payload = json.loads(message.data)
                x, y = payload["blocker_xy_m"]
                evidence.events.append(
                    BlockerEvent(
                        received_ns,
                        str(payload["event"]),
                        float(x),
                        float(y),
                        int(payload.get("trajectory_id", 0)),
                        int(payload.get("safe_trajectory_id", 0)),
                    )
                )
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
                raise RuntimeError("dynamic blocker event is malformed") from error
        elif topic == TF_TOPIC:
            for transform in message.transforms:
                if (
                    normalized_frame(transform.header.frame_id),
                    normalized_frame(transform.child_frame_id),
                ) != ("map", "base_link"):
                    continue
                translation = transform.transform.translation
                evidence.poses.append(
                    PoseSample(
                        received_ns,
                        float(translation.x),
                        float(translation.y),
                        float(translation.z),
                    )
                )
        elif topic == SETPOINT_TOPIC:
            evidence.setpoints.append(
                SetpointSample(
                    received_ns,
                    tuple(float(value) for value in message.position),
                    tuple(float(value) for value in message.velocity),
                )
            )
        elif topic == VEHICLE_STATUS_TOPIC:
            evidence.vehicle_statuses.append(
                VehicleStatusSample(
                    received_ns,
                    int(message.arming_state),
                    int(message.nav_state),
                    bool(message.failsafe),
                )
            )
        elif topic == LAND_TOPIC:
            evidence.land_samples.append(LandSample(received_ns, bool(message.landed)))
        else:
            for contact in message.contacts:
                evidence.collision_pairs.append(
                    (str(contact.collision1.name), str(contact.collision2.name))
                )
    return evidence


def point_segment_distance(point, start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 0.0:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    fraction = (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / length_squared
    fraction = min(1.0, max(0.0, fraction))
    closest = (start[0] + fraction * dx, start[1] + fraction * dy)
    return math.hypot(point[0] - closest[0], point[1] - closest[1])


def path_center_clearance(points, blocker):
    if not points:
        raise RuntimeError("trajectory has no valid transform points")
    if not all(finite_tuple(point) for point in points):
        raise RuntimeError("trajectory contains non-finite points")
    if len(points) == 1:
        return math.hypot(points[0][0] - blocker[0], points[0][1] - blocker[1])
    return min(
        point_segment_distance(blocker, start, end)
        for start, end in zip(points, points[1:])
    )


def safe_status(status):
    return (
        status.state in (READY, GOAL_REACHED)
        and status.map_fresh
        and status.trajectory_valid
        and status.trajectory_id > 0
    )


def latest_at_or_before(samples, timestamps, timestamp_ns):
    index = bisect_right(timestamps, timestamp_ns) - 1
    return samples[index] if index >= 0 else None


def validate_flight(evidence, insertion_ns):
    statuses = sorted(evidence.vehicle_statuses, key=lambda sample: sample.timestamp_ns)
    lands = sorted(evidence.land_samples, key=lambda sample: sample.timestamp_ns)
    if not statuses or not lands:
        raise RuntimeError("flight evidence is missing")
    if any(sample.failsafe for sample in statuses):
        raise RuntimeError("flight entered failsafe")
    armed = next(
        (
            sample
            for sample in statuses
            if sample.arming_state == ARMING_STATE_ARMED
            and sample.nav_state == NAVIGATION_STATE_OFFBOARD
        ),
        None,
    )
    airborne = next(
        (
            sample
            for sample in lands
            if not sample.landed and armed and sample.timestamp_ns > armed.timestamp_ns
        ),
        None,
    )
    landed = next(
        (
            sample
            for sample in lands
            if sample.landed
            and airborne
            and sample.timestamp_ns > airborne.timestamp_ns
        ),
        None,
    )
    disarmed = next(
        (
            sample
            for sample in statuses
            if sample.arming_state == ARMING_STATE_DISARMED
            and landed
            and sample.timestamp_ns > landed.timestamp_ns
        ),
        None,
    )
    if armed is None or airborne is None or landed is None or disarmed is None:
        raise RuntimeError("flight did not complete airborne -> landed -> disarmed")
    if landed.timestamp_ns <= insertion_ns:
        raise RuntimeError("flight landed before dynamic insertion")
    return armed, landed, disarmed


def validate_dynamic_replanning_authorization(
    evidence,
    insertion_start,
    insertion,
    confirmation,
    blocker,
    motion_end_ns=None,
):
    if not finite_tuple(blocker):
        raise RuntimeError("blocker position is non-finite")
    trajectories = sorted(evidence.trajectories, key=lambda sample: sample.timestamp_ns)
    if (
        insertion_start.trajectory_id <= 0
        or insertion_start.safe_trajectory_id != insertion_start.trajectory_id
        or insertion.trajectory_id != insertion_start.trajectory_id
        or insertion.safe_trajectory_id != insertion_start.trajectory_id
    ):
        raise RuntimeError("blocker insertion lacks a matching safe trajectory ID")
    initial = next(
        (
            sample
            for sample in reversed(trajectories)
            if sample.timestamp_ns < insertion_start.timestamp_ns
            and sample.trajectory_id == insertion_start.trajectory_id
        ),
        None,
    )
    if initial is None:
        raise RuntimeError("blocker must be inserted after the initial trajectory")
    statuses = sorted(evidence.statuses, key=lambda sample: sample.timestamp_ns)
    pre_status = next(
        (
            status
            for status in reversed(statuses)
            if status.timestamp_ns <= insertion_start.timestamp_ns
        ),
        None,
    )
    if (
        pre_status is None
        or not safe_status(pre_status)
        or pre_status.trajectory_id != initial.trajectory_id
        or insertion_start.timestamp_ns - pre_status.timestamp_ns > MAX_STATUS_AGE_NS
    ):
        raise RuntimeError("blocker insertion lacks a fresh matching planner status")
    initial_clearance = path_center_clearance(initial.points, blocker)
    if initial_clearance > REQUIRED_CENTER_CLEARANCE_M:
        raise RuntimeError("initial path does not cross the future blocker envelope")

    if (
        confirmation.trajectory_id <= 0
        or confirmation.trajectory_id == initial.trajectory_id
        or confirmation.safe_trajectory_id != confirmation.trajectory_id
    ):
        raise RuntimeError("blocker replan confirmation lacks a new safe trajectory ID")
    replanned = next(
        (
            sample
            for sample in trajectories
            if sample.timestamp_ns > insertion_start.timestamp_ns
            and sample.trajectory_id == confirmation.trajectory_id
        ),
        None,
    )
    if replanned is None:
        raise RuntimeError("dynamic insertion produced no new trajectory ID")
    if replanned.timestamp_ns > confirmation.timestamp_ns:
        raise RuntimeError("replan confirmation preceded the confirmed trajectory")
    confirmation_status = latest_at_or_before(
        statuses,
        [sample.timestamp_ns for sample in statuses],
        confirmation.timestamp_ns,
    )
    if (
        confirmation_status is None
        or not safe_status(confirmation_status)
        or confirmation_status.trajectory_id != replanned.trajectory_id
        or confirmation.timestamp_ns - confirmation_status.timestamp_ns
        > MAX_STATUS_AGE_NS
    ):
        raise RuntimeError("replan confirmation lacks a fresh matching planner status")
    replan_latency_ns = confirmation.timestamp_ns - insertion.timestamp_ns
    if replan_latency_ns > MAX_REPLAN_LATENCY_NS:
        raise RuntimeError("dynamic replanning exceeded the latency limit")
    replanned_clearance = path_center_clearance(replanned.points, blocker)
    if replanned_clearance <= REQUIRED_CENTER_CLEARANCE_M:
        raise RuntimeError("replanned path clearance is insufficient")

    status_times = [sample.timestamp_ns for sample in statuses]
    trajectory_times = [sample.timestamp_ns for sample in trajectories]
    moving_setpoints = 0
    previous_nonmoving_position = None
    previous_velocity_is_finite = None
    for setpoint in sorted(evidence.setpoints, key=lambda sample: sample.timestamp_ns):
        if motion_end_ns is not None and setpoint.timestamp_ns > motion_end_ns:
            break
        velocity = setpoint.velocity_ned
        velocity_is_finite = finite_tuple(velocity)
        velocity_implies_motion = velocity_is_finite and (
            math.sqrt(sum(value * value for value in velocity))
            > MOTION_VELOCITY_THRESHOLD_M_S
        )
        position = setpoint.position_ned
        position_changed = False
        if not velocity_implies_motion and finite_tuple(position):
            if (
                previous_nonmoving_position is not None
                and previous_velocity_is_finite == velocity_is_finite
            ):
                position_changed = (
                    math.dist(position, previous_nonmoving_position)
                    > MOTION_POSITION_THRESHOLD_M
                )
            previous_nonmoving_position = position
            previous_velocity_is_finite = velocity_is_finite
        else:
            previous_nonmoving_position = None
            previous_velocity_is_finite = None
        if setpoint.timestamp_ns < replanned.timestamp_ns:
            continue
        if not position_changed and not velocity_implies_motion:
            continue
        moving_setpoints += 1
        status = latest_at_or_before(statuses, status_times, setpoint.timestamp_ns)
        trajectory = latest_at_or_before(
            trajectories, trajectory_times, setpoint.timestamp_ns
        )
        if (
            status is None
            or trajectory is None
            or not safe_status(status)
            or status.trajectory_id != trajectory.trajectory_id
            or setpoint.timestamp_ns - status.timestamp_ns > MAX_STATUS_AGE_NS
        ):
            raise RuntimeError("motion lacked a fresh matching planner status")
    if moving_setpoints == 0:
        raise RuntimeError("no post-replan motion command was recorded")
    return (
        initial,
        replanned,
        initial_clearance,
        replanned_clearance,
        moving_setpoints,
        replan_latency_ns,
    )


def validate_evidence(evidence, launch_exit_code, expected_blocker=None):
    if launch_exit_code != 0:
        raise RuntimeError(f"top-level launch exit code was {launch_exit_code}")
    if not evidence.contacts_topic_present:
        raise RuntimeError("collision evidence topic is missing")

    insertions = [
        event for event in evidence.events if event.event == "blocker_inserted"
    ]
    if len(insertions) != 1:
        raise RuntimeError("bag must contain exactly one blocker inserted event")
    insertion = insertions[0]
    insertion_starts = [
        event for event in evidence.events if event.event == "blocker_insertion_started"
    ]
    if len(insertion_starts) != 1:
        raise RuntimeError(
            "bag must contain exactly one blocker insertion started event"
        )
    insertion_start = insertion_starts[0]
    confirmations = [
        event for event in evidence.events if event.event == "blocker_replan_confirmed"
    ]
    if len(confirmations) != 1:
        raise RuntimeError(
            "bag must contain exactly one blocker replan confirmed event"
        )
    confirmation = confirmations[0]
    blocker = (insertion.x, insertion.y)
    if not finite_tuple(blocker):
        raise RuntimeError("blocker insertion position is non-finite")
    if expected_blocker is not None:
        if not finite_tuple(expected_blocker) or any(
            not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-6)
            for actual, expected in zip(blocker, expected_blocker)
        ):
            raise RuntimeError("recorded blocker position does not match the scenario")
    if (
        (insertion_start.x, insertion_start.y) != blocker
        or (confirmation.x, confirmation.y) != blocker
        or not insertion_start.timestamp_ns < insertion.timestamp_ns
        or not insertion.timestamp_ns < confirmation.timestamp_ns
    ):
        raise RuntimeError("blocker event sequence or position is inconsistent")

    (
        initial,
        replanned,
        initial_clearance,
        replanned_clearance,
        moving_setpoints,
        replan_latency_ns,
    ) = validate_dynamic_replanning_authorization(
        evidence, insertion_start, insertion, confirmation, blocker
    )

    armed, landed, disarmed = validate_flight(evidence, insertion.timestamp_ns)
    poses = sorted(evidence.poses, key=lambda sample: sample.timestamp_ns)
    airborne_poses = [
        pose
        for pose in poses
        if insertion.timestamp_ns <= pose.timestamp_ns <= landed.timestamp_ns
    ]
    if not airborne_poses:
        raise RuntimeError("no vehicle poses were recorded after insertion")
    if not all(finite_tuple((pose.x, pose.y, pose.z)) for pose in airborne_poses):
        raise RuntimeError("vehicle pose contains non-finite values")
    pose_points = [(pose.x, pose.y) for pose in airborne_poses]
    if len(pose_points) == 1:
        minimum_center_distance = math.hypot(
            pose_points[0][0] - blocker[0], pose_points[0][1] - blocker[1]
        )
    else:
        minimum_center_distance = min(
            point_segment_distance(blocker, start, end)
            for start, end in zip(pose_points, pose_points[1:])
        )
    minimum_actual_clearance = minimum_center_distance - REQUIRED_CENTER_CLEARANCE_M
    if minimum_actual_clearance <= 0.0:
        raise RuntimeError("actual vehicle clearance is not positive")

    goal_altitude_poses = [
        pose
        for pose in airborne_poses
        if abs(pose.z - GOAL_Z_M) <= ALTITUDE_TOLERANCE_M
    ]
    if not goal_altitude_poses:
        raise RuntimeError("vehicle did not reach the M3 goal altitude before landing")
    goal_pose = min(
        goal_altitude_poses,
        key=lambda pose: math.hypot(pose.x - GOAL_X_M, pose.y - GOAL_Y_M),
    )
    goal_error = math.hypot(goal_pose.x - GOAL_X_M, goal_pose.y - GOAL_Y_M)
    if goal_error > GOAL_TOLERANCE_M:
        raise RuntimeError("vehicle did not reach the M3 goal before landing")

    blocker_collisions = [
        pair
        for pair in evidence.collision_pairs
        if any("m3_dynamic_blocker" in name for name in pair)
        and any("x500" in name for name in pair)
    ]
    if blocker_collisions:
        raise RuntimeError("x500 collision with the dynamic blocker was recorded")

    return {
        "accepted": True,
        "replanning": {
            "blocker_x_m": blocker[0],
            "blocker_y_m": blocker[1],
            "initial_trajectory_id": initial.trajectory_id,
            "replanned_trajectory_id": replanned.trajectory_id,
            "insertion_started_timestamp_ns": insertion_start.timestamp_ns,
            "insertion_timestamp_ns": insertion.timestamp_ns,
            "replan_confirmation_timestamp_ns": confirmation.timestamp_ns,
            "first_replanned_trajectory_timestamp_ns": replanned.timestamp_ns,
            "replan_latency_s": replan_latency_ns / 1e9,
            "initial_path_center_clearance_m": initial_clearance,
            "replanned_path_center_clearance_m": replanned_clearance,
            "post_replan_motion_setpoint_count": moving_setpoints,
        },
        "clearance": {
            "vehicle_radius_m": VEHICLE_RADIUS_M,
            "blocker_radius_m": BLOCKER_RADIUS_M,
            "minimum_center_distance_m": minimum_center_distance,
            "minimum_actual_clearance_m": minimum_actual_clearance,
            "blocker_collision_count": len(blocker_collisions),
        },
        "flight": {
            "armed_offboard_timestamp_ns": armed.timestamp_ns,
            "goal_error_m": goal_error,
            "landed_timestamp_ns": landed.timestamp_ns,
            "disarmed_timestamp_ns": disarmed.timestamp_ns,
            "landed_and_disarmed": True,
            "failsafe_observed": False,
        },
    }


def validate_randomized_results(results):
    if len(results) != 10:
        raise RuntimeError("randomized evaluation requires exactly 10 runs")
    success_count = sum(bool(result.get("accepted")) for result in results)
    if success_count < 8:
        raise RuntimeError("randomized evaluation requires at least 8 successful runs")
    return {
        "run_count": 10,
        "success_count": success_count,
        "failure_count": 10 - success_count,
        "success_rate": success_count / 10.0,
    }


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_directory(path):
    digest = hashlib.sha256()
    bag_files = [path / "metadata.yaml"] + sorted(path.glob("*.db3"))
    if not bag_files[0].is_file() or len(bag_files) < 2:
        raise RuntimeError("bag provenance files are incomplete")
    for file_path in bag_files:
        digest.update(file_path.relative_to(path).as_posix().encode("utf-8"))
        digest.update(bytes.fromhex(sha256_file(file_path)))
    return digest.hexdigest()


def add_provenance(metrics, bag_path):
    project_root = Path(__file__).resolve().parents[1]
    source_hashes = {}
    for relative_path in RUNTIME_SOURCE_PATHS:
        path = project_root / relative_path
        if not path.is_file():
            raise RuntimeError(f"runtime source is missing: {relative_path}")
        source_hashes[relative_path] = sha256_file(path)
    metrics["provenance"] = {
        "bag_path": str(bag_path.resolve()),
        "bag_sha256": sha256_directory(bag_path),
        "runtime_source_sha256": source_hashes,
    }


def write_metrics(metrics, output_path):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_name(output_path.name + ".tmp")
    temporary_path.write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary_path.replace(output_path)


def analyze_and_write(bag_path, output_path, launch_exit_code, expected_blocker=None):
    write_metrics(
        {"accepted": False, "rejection_reason": "analysis did not complete"},
        output_path,
    )
    try:
        evidence = read_bag(bag_path)
        metrics = validate_evidence(evidence, launch_exit_code, expected_blocker)
        add_provenance(metrics, bag_path)
    except RuntimeError as error:
        rejected = {"accepted": False, "rejection_reason": str(error)}
        try:
            add_provenance(rejected, bag_path)
        except RuntimeError as provenance_error:
            rejected["provenance_error"] = str(provenance_error)
        write_metrics(rejected, output_path)
        raise
    write_metrics(metrics, output_path)
    return metrics


def main():
    args = parse_args()
    if (args.expected_blocker_x_m is None) != (args.expected_blocker_y_m is None):
        raise SystemExit("both expected blocker coordinates must be provided together")
    expected_blocker = None
    if args.expected_blocker_x_m is not None:
        expected_blocker = (
            args.expected_blocker_x_m,
            args.expected_blocker_y_m,
        )
    try:
        metrics = analyze_and_write(
            args.bag,
            args.metrics,
            args.launch_exit_code,
            expected_blocker,
        )
    except RuntimeError as error:
        raise SystemExit(f"M3 evidence rejected: {error}") from error
    print(
        "M3 dynamic evidence accepted: "
        f"trajectory {metrics['replanning']['initial_trajectory_id']} -> "
        f"{metrics['replanning']['replanned_trajectory_id']}, "
        f"clearance {metrics['clearance']['minimum_actual_clearance_m']:.3f} m"
    )
    print(f"wrote {args.metrics}")


if __name__ == "__main__":
    main()
