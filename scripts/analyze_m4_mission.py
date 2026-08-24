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

"""Independently validate an M4 inspection mission bag."""

import argparse
from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import sys
import tempfile

import yaml


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
if str(SCRIPT_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIRECTORY))

import analyze_m3_planning as m3  # noqa: E402


MISSION_EVENT_TOPIC = "/drone_mission/event"
MISSION_COMMAND_TOPIC = "/drone_mission/command"
PLANNER_GOAL_TOPIC = "/drone_planner/goal"
LAND_COMMAND = 1
MAX_EVENT_POSE_AGE_NS = 500_000_000
GOAL_RECEIVE_REORDER_TOLERANCE_NS = 1_000_000
REACHED_HORIZONTAL_TOLERANCE_M = 0.30
REACHED_ALTITUDE_TOLERANCE_M = 0.35

STANDBY = 0
TAKEOFF = 1
INSPECTING = 2
HANDLING_EXCEPTION = 3
RETURNING_HOME = 4
LANDING = 5
COMPLETE = 6

RUNTIME_SOURCE_PATHS = (
    "scripts/analyze_m3_planning.py",
    "scripts/analyze_m4_mission.py",
    "scripts/m3_dynamic_blocker.py",
    "src/drone_bringup/config/m3_mapping.yaml",
    "src/drone_bringup/config/m3_planner.yaml",
    "src/drone_bringup/config/m4_controller.yaml",
    "src/drone_bringup/config/m4_mission.yaml",
    "src/drone_bringup/launch/m4_inspection.launch.py",
    "src/drone_controller/src/waypoint_controller_node.cpp",
    "src/drone_mission/src/mission_fsm.cpp",
    "src/drone_mission/src/mission_node.cpp",
    "src/drone_perception/src/depth_grid_node.cpp",
    "src/drone_planner/src/planner_node.cpp",
    "src/drone_sim/worlds/inspection.sdf",
)


@dataclass
class MissionEventSample:
    timestamp_ns: int
    sequence: int
    from_state: int
    to_state: int
    waypoint_index: int
    event: str
    reason: str


@dataclass
class MissionCommandSample:
    timestamp_ns: int
    command: int


@dataclass
class PlannerGoalSample:
    timestamp_ns: int
    x: float
    y: float
    z: float
    frame_id: str


@dataclass
class MissionEvidence:
    planning: m3.PlanningEvidence
    mission_events: list
    mission_commands: list
    planner_goals: list = field(default_factory=list)


def parse_args():
    parser = argparse.ArgumentParser(description="Validate an M4 mission ROS 2 bag.")
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory")
    parser.add_argument("--metrics", required=True, type=Path)
    parser.add_argument("--launch-exit-code", required=True, type=int)
    return parser.parse_args()


def read_bag(bag_path):
    planning = m3.read_bag(bag_path)

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
    required = {MISSION_EVENT_TOPIC, MISSION_COMMAND_TOPIC, PLANNER_GOAL_TOPIC}
    missing = sorted(required - topic_types.keys())
    if missing:
        raise RuntimeError("bag is missing required M4 topics: " + ", ".join(missing))

    events = []
    commands = []
    planner_goals = []
    while reader.has_next():
        topic, serialized, received_ns = reader.read_next()
        if topic not in required:
            continue
        message = deserialize_message(serialized, topic_types[topic])
        if topic == MISSION_EVENT_TOPIC:
            events.append(
                MissionEventSample(
                    received_ns,
                    int(message.sequence),
                    int(message.from_state),
                    int(message.to_state),
                    int(message.waypoint_index),
                    str(message.event),
                    str(message.reason),
                )
            )
        elif topic == MISSION_COMMAND_TOPIC:
            commands.append(
                MissionCommandSample(received_ns, int(message.command))
            )
        else:
            position = message.pose.position
            planner_goals.append(
                PlannerGoalSample(
                    received_ns,
                    float(position.x),
                    float(position.y),
                    float(position.z),
                    str(message.header.frame_id),
                )
            )
    return MissionEvidence(planning, events, commands, planner_goals)


def exactly_one(samples, event_name):
    matches = [sample for sample in samples if sample.event == event_name]
    if len(matches) != 1:
        raise RuntimeError(f"mission requires exactly one {event_name} event")
    return matches[0]


def load_mission_contract():
    config_path = (
        Path(__file__).resolve().parents[1] / "src/drone_bringup/config/m4_mission.yaml"
    )
    try:
        document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        parameters = document["mission_node"]["ros__parameters"]
        altitude = float(parameters["goal_altitude_m"])
        values = [float(value) for value in parameters["inspection_waypoints_xy"]]
        home = (
            float(parameters["home_x_m"]),
            float(parameters["home_y_m"]),
            altitude,
        )
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        raise RuntimeError(f"invalid M4 mission contract: {error}") from error
    if not values or len(values) % 2 != 0 or not m3.finite_tuple((*values, *home)):
        raise RuntimeError("M4 mission contract contains invalid coordinates")
    waypoints = [
        (values[index], values[index + 1], altitude)
        for index in range(0, len(values), 2)
    ]
    if len(set(waypoints)) != len(waypoints):
        raise RuntimeError("M4 mission contract contains duplicate waypoints")
    return waypoints, home


def require_reached_pose(poses, event, expected, label):
    candidates = [pose for pose in poses if pose.timestamp_ns <= event.timestamp_ns]
    if not candidates:
        raise RuntimeError(f"{label} has no preceding vehicle pose")
    pose = candidates[-1]
    if event.timestamp_ns - pose.timestamp_ns > MAX_EVENT_POSE_AGE_NS:
        raise RuntimeError(f"{label} vehicle pose is stale")
    if not m3.finite_tuple((pose.x, pose.y, pose.z)):
        raise RuntimeError(f"{label} vehicle pose is non-finite")
    if (
        math.hypot(pose.x - expected[0], pose.y - expected[1])
        > REACHED_HORIZONTAL_TOLERANCE_M
        or abs(pose.z - expected[2]) > REACHED_ALTITUDE_TOLERANCE_M
    ):
        raise RuntimeError(f"{label} vehicle pose does not match the configured target")


def validate_mission_events(events):
    events = sorted(events, key=lambda sample: sample.timestamp_ns)
    if not events:
        raise RuntimeError("mission event log is empty")
    if [event.sequence for event in events] != list(range(1, len(events) + 1)):
        raise RuntimeError("mission event sequence is not contiguous from one")
    if any(
        later.timestamp_ns <= earlier.timestamp_ns
        for earlier, later in zip(events, events[1:])
    ):
        raise RuntimeError("mission event timestamps are not strictly increasing")
    if any(event.event == "inspection_completed" for event in events):
        raise RuntimeError(
            "low-battery mission cannot also complete inspection normally"
        )

    allowed = {
        "mission_started": (STANDBY, TAKEOFF),
        "takeoff_completed": (TAKEOFF, INSPECTING),
        "waypoint_reached": (INSPECTING, INSPECTING),
        "waypoint_unreachable": (INSPECTING, HANDLING_EXCEPTION),
        "waypoint_skipped": None,
        "low_battery": (INSPECTING, HANDLING_EXCEPTION),
        "inspection_interrupted": (HANDLING_EXCEPTION, RETURNING_HOME),
        "inspection_completed": (INSPECTING, RETURNING_HOME),
        "return_waypoint_reached": (RETURNING_HOME, RETURNING_HOME),
        "home_reached": (RETURNING_HOME, LANDING),
        "landing_completed": (LANDING, COMPLETE),
    }
    for event in events:
        if event.event not in allowed:
            raise RuntimeError(f"unknown mission event: {event.event}")
        expected = allowed[event.event]
        if event.event == "waypoint_skipped":
            if event.from_state != HANDLING_EXCEPTION or event.to_state not in (
                INSPECTING,
                RETURNING_HOME,
            ):
                raise RuntimeError("waypoint_skipped has an invalid state transition")
        elif (event.from_state, event.to_state) != expected:
            raise RuntimeError(f"{event.event} has an invalid state transition")

    started = exactly_one(events, "mission_started")
    takeoff = exactly_one(events, "takeoff_completed")
    unreachable = exactly_one(events, "waypoint_unreachable")
    skipped = exactly_one(events, "waypoint_skipped")
    low_battery = exactly_one(events, "low_battery")
    interrupted = exactly_one(events, "inspection_interrupted")
    home = exactly_one(events, "home_reached")
    completed = exactly_one(events, "landing_completed")
    if any(
        previous.to_state != current.from_state
        for previous, current in zip(events, events[1:])
    ):
        raise RuntimeError("mission event state chain is discontinuous")

    reached = [event for event in events if event.event == "waypoint_reached"]
    if len(reached) < 2:
        raise RuntimeError("mission reached fewer than two inspection waypoints")
    reached_indexes = [event.waypoint_index for event in reached]
    if any(index < 1 for index in reached_indexes) or any(
        later <= earlier for earlier, later in zip(reached_indexes, reached_indexes[1:])
    ):
        raise RuntimeError(
            "reached inspection waypoint indexes are not strictly increasing"
        )
    if takeoff.waypoint_index != 0:
        raise RuntimeError("takeoff completion did not select the first waypoint")
    if (
        skipped.to_state != INSPECTING
        or skipped.waypoint_index != unreachable.waypoint_index + 1
    ):
        raise RuntimeError(
            "unreachable waypoint skip did not advance to the next waypoint"
        )

    active_waypoint_index = takeoff.waypoint_index
    for event in events:
        if (
            event.timestamp_ns <= takeoff.timestamp_ns
            or event.timestamp_ns >= low_battery.timestamp_ns
        ):
            continue
        if event.event == "waypoint_reached":
            expected_index = active_waypoint_index + 1
            active_waypoint_index = event.waypoint_index
        elif event.event == "waypoint_unreachable":
            expected_index = active_waypoint_index
        elif event.event == "waypoint_skipped":
            expected_index = active_waypoint_index + 1
            active_waypoint_index = event.waypoint_index
        else:
            continue
        if event.waypoint_index != expected_index:
            raise RuntimeError("inspection waypoint progression is not contiguous")
    if any(
        event.waypoint_index != -1
        for event in (started, low_battery, interrupted, home, completed)
    ):
        raise RuntimeError("non-waypoint mission event carries a waypoint index")
    if not (
        started.timestamp_ns
        < takeoff.timestamp_ns
        < reached[0].timestamp_ns
        < unreachable.timestamp_ns
        < skipped.timestamp_ns
        < reached[-1].timestamp_ns
        < low_battery.timestamp_ns
        < interrupted.timestamp_ns
        < home.timestamp_ns
        < completed.timestamp_ns
    ):
        raise RuntimeError("required M4 mission events are out of order")
    if skipped.sequence != unreachable.sequence + 1:
        raise RuntimeError("unreachable waypoint was not immediately skipped")
    if interrupted.sequence != low_battery.sequence + 1:
        raise RuntimeError("low battery did not immediately interrupt inspection")
    return (
        {
            "event_count": len(events),
            "reached_waypoint_count": len(reached),
            "unreachable_timestamp_ns": unreachable.timestamp_ns,
            "low_battery_timestamp_ns": low_battery.timestamp_ns,
            "home_reached_timestamp_ns": home.timestamp_ns,
            "landing_completed_timestamp_ns": completed.timestamp_ns,
        },
        home,
        completed,
    )


def validate_dynamic_avoidance(planning, motion_end_ns=None):
    events = sorted(planning.events, key=lambda sample: sample.timestamp_ns)
    by_name = {}
    for name in (
        "blocker_insertion_started",
        "blocker_inserted",
        "blocker_replan_confirmed",
        "blocker_removal_started",
        "blocker_removed",
    ):
        matches = [event for event in events if event.event == name]
        if len(matches) != 1:
            raise RuntimeError(f"dynamic avoidance requires exactly one {name} event")
        by_name[name] = matches[0]
    start = by_name["blocker_insertion_started"]
    inserted = by_name["blocker_inserted"]
    confirmed = by_name["blocker_replan_confirmed"]
    removal_start = by_name["blocker_removal_started"]
    removed = by_name["blocker_removed"]
    if not (
        start.timestamp_ns
        < inserted.timestamp_ns
        < confirmed.timestamp_ns
        < removal_start.timestamp_ns
        < removed.timestamp_ns
    ):
        raise RuntimeError("dynamic blocker events are out of order")
    blocker = (inserted.x, inserted.y)
    if not m3.finite_tuple(blocker):
        raise RuntimeError("dynamic blocker active position is non-finite")
    if (start.x, start.y) != blocker or (confirmed.x, confirmed.y) != blocker:
        raise RuntimeError("dynamic blocker active positions are inconsistent")
    if (removal_start.x, removal_start.y) != blocker:
        raise RuntimeError("dynamic blocker removal started from the wrong position")
    if not m3.finite_tuple((removed.x, removed.y)) or not (
        math.isclose(removed.x, 0.0, rel_tol=0.0, abs_tol=1e-6)
        and math.isclose(removed.y, -3.0, rel_tol=0.0, abs_tol=1e-6)
    ):
        raise RuntimeError("dynamic blocker was not removed to its expected safe pose")

    (
        initial,
        replanned,
        initial_clearance,
        replanned_clearance,
        moving_setpoints,
        replan_latency_ns,
    ) = m3.validate_dynamic_replanning_authorization(
        planning,
        start,
        inserted,
        confirmed,
        blocker,
        motion_end_ns=motion_end_ns,
    )

    poses = [
        pose
        for pose in sorted(planning.poses, key=lambda sample: sample.timestamp_ns)
        if inserted.timestamp_ns <= pose.timestamp_ns <= removal_start.timestamp_ns
    ]
    if len(poses) < 2 or not all(
        m3.finite_tuple((pose.x, pose.y, pose.z)) for pose in poses
    ):
        raise RuntimeError("dynamic avoidance lacks finite vehicle poses")
    minimum_center_distance = min(
        m3.point_segment_distance(
            blocker,
            (first.x, first.y),
            (second.x, second.y),
        )
        for first, second in zip(poses, poses[1:])
    )
    minimum_actual_clearance = minimum_center_distance - m3.REQUIRED_CENTER_CLEARANCE_M
    if minimum_actual_clearance <= 0.0:
        raise RuntimeError("vehicle did not safely clear the blocker before removal")
    final_pose = poses[-1]
    if final_pose.y - blocker[1] <= m3.REQUIRED_CENTER_CLEARANCE_M:
        raise RuntimeError("blocker was removed before the vehicle passed it")

    replan_latency_s = replan_latency_ns / 1e9
    return {
        "initial_trajectory_id": initial.trajectory_id,
        "replanned_trajectory_id": replanned.trajectory_id,
        "replan_latency_s": replan_latency_s,
        "initial_path_center_clearance_m": initial_clearance,
        "replanned_path_center_clearance_m": replanned_clearance,
        "post_replan_motion_setpoint_count": moving_setpoints,
        "minimum_actual_clearance_m": minimum_actual_clearance,
        "removed_timestamp_ns": removed.timestamp_ns,
    }, inserted


def validate_planner_goals(goals, mission_events, poses):
    events = sorted(mission_events, key=lambda sample: sample.timestamp_ns)
    poses = sorted(poses, key=lambda sample: sample.timestamp_ns)
    expected_waypoints, expected_home = load_mission_contract()
    low_battery = exactly_one(events, "low_battery")
    interrupted = exactly_one(events, "inspection_interrupted")
    home = exactly_one(events, "home_reached")
    goals = sorted(goals, key=lambda sample: sample.timestamp_ns)
    if not goals:
        raise RuntimeError("bag contains no planner goals")
    if any(
        not m3.finite_tuple((goal.x, goal.y, goal.z)) or goal.frame_id != "map"
        for goal in goals
    ):
        raise RuntimeError("planner goal is non-finite or not in the map frame")

    waypoint_selectors = [
        event
        for event in events
        if event.timestamp_ns < low_battery.timestamp_ns
        and (
            event.event in {"takeoff_completed", "waypoint_reached"}
            or (event.event == "waypoint_skipped" and event.to_state == INSPECTING)
        )
    ]
    selected_goals = []
    used_goal_indexes = set()
    for selector in waypoint_selectors:
        if not 0 <= selector.waypoint_index < len(expected_waypoints):
            raise RuntimeError("planner goal selector has an invalid waypoint index")
        expected = expected_waypoints[selector.waypoint_index]
        next_event = next(
            event for event in events if event.timestamp_ns > selector.timestamp_ns
        )
        matching_by_time = [
            (index, goal)
            for index, goal in enumerate(goals)
            if index not in used_goal_indexes
            if selector.timestamp_ns - GOAL_RECEIVE_REORDER_TOLERANCE_NS
            <= goal.timestamp_ns < next_event.timestamp_ns
        ]
        if not matching_by_time:
            raise RuntimeError(f"{selector.event} was not followed by a planner goal")
        matching = [
            (index, goal)
            for index, goal in matching_by_time
            if all(
                math.isclose(actual, configured, rel_tol=0.0, abs_tol=1e-6)
                for actual, configured in zip((goal.x, goal.y, goal.z), expected)
            )
        ]
        if not matching:
            raise RuntimeError("planner goal does not match the configured waypoint")
        goal_index, goal = matching[0]
        used_goal_indexes.add(goal_index)
        selected_goals.append(goal)

    unique_positions = {
        (round(goal.x, 6), round(goal.y, 6), round(goal.z, 6))
        for goal in selected_goals
    }
    if len(unique_positions) != len(waypoint_selectors):
        raise RuntimeError("inspection planner goals are not distinct")
    reached_events = [event for event in events if event.event == "waypoint_reached"]
    return_events = [
        event for event in events if event.event == "return_waypoint_reached"
    ]
    reached_completed_indexes = [event.waypoint_index - 1 for event in reached_events]
    expected_return_indexes = list(reversed(reached_completed_indexes[:-1]))
    if [event.waypoint_index for event in return_events] != expected_return_indexes:
        raise RuntimeError("return waypoints do not reverse the verified inspection route")

    return_selectors = [interrupted, *return_events]
    return_targets = [
        expected_waypoints[event.waypoint_index] for event in return_events
    ] + [expected_home]
    return_goals = []
    for target_index, (selector, expected) in enumerate(
        zip(return_selectors, return_targets)
    ):
        is_home_target = target_index + 1 == len(return_targets)
        next_event = next(
            event for event in events if event.timestamp_ns > selector.timestamp_ns
        )
        matching_by_time = [
            (index, goal)
            for index, goal in enumerate(goals)
            if index not in used_goal_indexes
            if selector.timestamp_ns - GOAL_RECEIVE_REORDER_TOLERANCE_NS
            <= goal.timestamp_ns < next_event.timestamp_ns
        ]
        if not matching_by_time:
            if is_home_target:
                raise RuntimeError("return route was not followed by a home goal")
            raise RuntimeError(f"{selector.event} was not followed by a return goal")
        matching = [
            (index, goal)
            for index, goal in matching_by_time
            if all(
                math.isclose(actual, configured, rel_tol=0.0, abs_tol=1e-6)
                for actual, configured in zip((goal.x, goal.y, goal.z), expected)
            )
        ]
        if not matching:
            if is_home_target:
                raise RuntimeError("home planner goal does not match the configured home")
            raise RuntimeError("return planner goal does not match the verified route")
        goal_index, goal = matching[0]
        used_goal_indexes.add(goal_index)
        if any(
            not math.isclose(actual, configured, rel_tol=0.0, abs_tol=1e-6)
            for actual, configured in zip((goal.x, goal.y, goal.z), expected)
        ):
            if is_home_target:
                raise RuntimeError("home planner goal does not match the configured home")
            raise RuntimeError("return planner goal does not match the verified route")
        return_goals.append(goal)

    home_goals = [return_goals[-1]]
    if any(
        not math.isclose(actual, configured, rel_tol=0.0, abs_tol=1e-6)
        for actual, configured in zip(
            (home_goals[0].x, home_goals[0].y, home_goals[0].z), expected_home
        )
    ):
        raise RuntimeError("home planner goal does not match the configured home")

    for event in reached_events:
        reached_index = event.waypoint_index - 1
        if not 0 <= reached_index < len(expected_waypoints):
            raise RuntimeError(
                "waypoint_reached refers to an invalid completed waypoint"
            )
        require_reached_pose(
            poses,
            event,
            expected_waypoints[reached_index],
            "waypoint_reached",
        )
    for event in return_events:
        require_reached_pose(
            poses,
            event,
            expected_waypoints[event.waypoint_index],
            "return_waypoint_reached",
        )
    require_reached_pose(poses, home, expected_home, "home_reached")
    return {
        "goal_count": len(selected_goals),
        "distinct_goal_count": len(unique_positions),
        "return_waypoint_count": len(return_events),
        "home_goal_timestamp_ns": home_goals[0].timestamp_ns,
    }


def validate_evidence(evidence, launch_exit_code):
    if launch_exit_code != 0:
        raise RuntimeError(f"top-level launch exit code was {launch_exit_code}")
    if not evidence.planning.contacts_topic_present:
        raise RuntimeError("collision evidence topic is missing")
    mission, home, completed = validate_mission_events(evidence.mission_events)
    goals = validate_planner_goals(
        evidence.planner_goals,
        evidence.mission_events,
        evidence.planning.poses,
    )
    insertions = [
        event for event in evidence.planning.events if event.event == "blocker_inserted"
    ]
    if len(insertions) != 1:
        raise RuntimeError(
            "dynamic avoidance requires exactly one blocker_inserted event"
        )
    insertion = insertions[0]
    armed, landed, disarmed = m3.validate_flight(
        evidence.planning, insertion.timestamp_ns
    )
    avoidance, insertion = validate_dynamic_avoidance(
        evidence.planning, motion_end_ns=landed.timestamp_ns
    )

    land_commands = sorted(
        (
            command
            for command in evidence.mission_commands
            if command.command == LAND_COMMAND
        ),
        key=lambda sample: sample.timestamp_ns,
    )
    if not land_commands:
        raise RuntimeError("mission issued no LAND command")
    if any(command.timestamp_ns <= home.timestamp_ns for command in land_commands):
        raise RuntimeError("LAND was commanded before home was reached")
    if not (
        home.timestamp_ns
        < land_commands[0].timestamp_ns
        < landed.timestamp_ns
        < disarmed.timestamp_ns
        <= completed.timestamp_ns
    ):
        raise RuntimeError(
            "required home -> LAND -> landed -> disarmed -> complete order is invalid"
        )
    vehicle_collisions = [
        pair
        for pair in evidence.planning.collision_pairs
        if any("x500" in name for name in pair)
    ]
    if vehicle_collisions:
        raise RuntimeError("x500 collision was recorded during the mission")
    return {
        "accepted": True,
        "mission": mission,
        "planner_goals": goals,
        "dynamic_avoidance": avoidance,
        "landing": {
            "first_land_command_timestamp_ns": land_commands[0].timestamp_ns,
            "land_command_count": len(land_commands),
            "landed_timestamp_ns": landed.timestamp_ns,
            "disarmed_timestamp_ns": disarmed.timestamp_ns,
            "landed_and_disarmed": True,
        },
        "safety": {
            "failsafe_observed": False,
            "vehicle_collision_count": len(vehicle_collisions),
        },
        "flight": {"armed_offboard_timestamp_ns": armed.timestamp_ns},
    }


def add_provenance(metrics, bag_path):
    project_root = Path(__file__).resolve().parents[1]
    source_hashes = {}
    for relative_path in RUNTIME_SOURCE_PATHS:
        path = project_root / relative_path
        if not path.is_file():
            raise RuntimeError(f"runtime source is missing: {relative_path}")
        source_hashes[relative_path] = m3.sha256_file(path)
    metrics["provenance"] = {
        "bag_path": str(bag_path.resolve()),
        "bag_sha256": m3.sha256_directory(bag_path),
        "runtime_source_sha256": source_hashes,
    }


def write_metrics(metrics, output_path):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output_path.name}-", dir=output_path.parent
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as file:
            json.dump(metrics, file, indent=2, sort_keys=True)
            file.write("\n")
        os.replace(temporary_path, output_path)
    finally:
        temporary_path.unlink(missing_ok=True)


def analyze_and_write(bag_path, output_path, launch_exit_code):
    write_metrics(
        {"accepted": False, "rejection_reason": "analysis did not complete"},
        output_path,
    )
    try:
        evidence = read_bag(bag_path)
        metrics = validate_evidence(evidence, launch_exit_code)
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
    try:
        metrics = analyze_and_write(args.bag, args.metrics, args.launch_exit_code)
    except RuntimeError as error:
        raise SystemExit(f"M4 evidence rejected: {error}") from error
    print(
        "M4 evidence accepted: "
        f"{metrics['mission']['reached_waypoint_count']} waypoints reached, "
        f"replan {metrics['dynamic_avoidance']['replan_latency_s']:.3f} s, "
        "landed and disarmed"
    )
    print(f"wrote {args.metrics}")


if __name__ == "__main__":
    main()
