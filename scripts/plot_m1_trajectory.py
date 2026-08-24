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

import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path

from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TARGET_TOPIC = "/fmu/in/trajectory_setpoint"
POSITION_TOPIC = "/fmu/out/vehicle_local_position_v1"
STATUS_TOPIC = "/fmu/out/vehicle_status_v1"
LAND_TOPIC = "/fmu/out/vehicle_land_detected"
EXPECTED_TARGET_SEGMENTS = 6
STEADY_WINDOW_NS = int(1e9)
WINDOW_COVERAGE_TOLERANCE_NS = int(0.1e9)
MIN_STEADY_SAMPLES_PER_SEGMENT = 10


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Plot M1 desired and actual NED trajectories from a ROS 2 bag."
        )
    )
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory")
    parser.add_argument("output", type=Path, help="Output PNG path")
    parser.add_argument(
        "--animation",
        type=Path,
        help="Optional output GIF path for an actual-trajectory replay",
    )
    parser.add_argument(
        "--metrics",
        type=Path,
        help=(
            "Optional JSON output for full-mission and steady-state errors"
        ),
    )
    return parser.parse_args()


def read_trajectory(bag_path):
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

    target_samples = []
    position_samples = []
    status_samples = []
    land_samples = []
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic == TARGET_TOPIC:
            message = deserialize_message(data, topic_types[topic])
            target_samples.append((timestamp_ns, *message.position))
        elif topic == POSITION_TOPIC:
            message = deserialize_message(data, topic_types[topic])
            if message.xy_valid and message.z_valid:
                position_samples.append(
                    (timestamp_ns, message.x, message.y, message.z)
                )
        elif topic == STATUS_TOPIC:
            message = deserialize_message(data, topic_types[topic])
            status_samples.append(
                (
                    timestamp_ns,
                    message.arming_state == message.ARMING_STATE_ARMED,
                    message.arming_state == message.ARMING_STATE_DISARMED,
                    message.nav_state == message.NAVIGATION_STATE_OFFBOARD,
                    message.failsafe,
                )
            )
        elif topic == LAND_TOPIC:
            message = deserialize_message(data, topic_types[topic])
            land_samples.append((timestamp_ns, message.landed))

    missing_topics = [
        topic
        for topic, samples in (
            (TARGET_TOPIC, target_samples),
            (POSITION_TOPIC, position_samples),
            (STATUS_TOPIC, status_samples),
            (LAND_TOPIC, land_samples),
        )
        if not samples
    ]
    if missing_topics:
        raise RuntimeError(
            "bag must contain non-empty mission evidence topics: "
            + ", ".join(missing_topics)
        )
    return target_samples, position_samples, status_samples, land_samples


def relative_seconds(samples, start_ns):
    return [(sample[0] - start_ns) / 1e9 for sample in samples]


def summarize_errors(errors):
    if not errors:
        raise RuntimeError("cannot summarize an empty error sample set")
    return {
        "sample_count": len(errors),
        "mean_m": sum(errors) / len(errors),
        "rmse_m": math.sqrt(
            sum(error * error for error in errors) / len(errors)
        ),
        "max_m": max(errors),
    }


def validate_finite_positions(samples, label):
    for sample in samples:
        if not all(math.isfinite(value) for value in sample[1:4]):
            raise RuntimeError(
                f"{label} contains a non-finite position sample"
            )


def target_segments(target_samples):
    if not target_samples:
        raise RuntimeError("target trajectory is empty")
    validate_finite_positions(target_samples, "target trajectory")

    segments = []
    segment_start_ns = target_samples[0][0]
    segment_target = target_samples[0][1:4]
    previous_timestamp_ns = segment_start_ns
    for sample in target_samples[1:]:
        if sample[0] <= previous_timestamp_ns:
            raise RuntimeError("target timestamps must be strictly increasing")
        previous_timestamp_ns = sample[0]
        if sample[1:4] != segment_target:
            segments.append((segment_start_ns, sample[0], segment_target))
            segment_start_ns = sample[0]
            segment_target = sample[1:4]
    segments.append((segment_start_ns, target_samples[-1][0], segment_target))

    if len(segments) != EXPECTED_TARGET_SEGMENTS:
        raise RuntimeError(
            "expected "
            f"{EXPECTED_TARGET_SEGMENTS} complete target segments, "
            f"found {len(segments)}"
        )
    for start_ns, end_ns, _target in segments:
        if end_ns - start_ns < STEADY_WINDOW_NS:
            raise RuntimeError(
                "every target segment must contain a complete 1.0 s "
                "steady-state window"
            )
    return segments


def validate_mission_evidence(
    target_samples, position_samples, status_samples, land_samples
):
    segments = target_segments(target_samples)
    validate_finite_positions(position_samples, "position trajectory")
    mission_start_ns = target_samples[0][0]
    mission_end_ns = target_samples[-1][0]

    mission_status = [
        sample
        for sample in status_samples
        if mission_start_ns <= sample[0]
    ]
    if not any(sample[1] and sample[3] for sample in mission_status):
        raise RuntimeError("bag never confirms armed Offboard flight")
    if any(sample[4] for sample in mission_status):
        raise RuntimeError("bag contains a PX4 failsafe during the mission")

    post_mission_status = [
        sample for sample in status_samples if sample[0] >= mission_end_ns
    ]
    post_mission_land = [
        sample for sample in land_samples if sample[0] >= mission_end_ns
    ]
    if not post_mission_status or not post_mission_status[-1][2]:
        raise RuntimeError("bag does not end with a disarmed vehicle status")
    if not post_mission_land or not post_mission_land[-1][1]:
        raise RuntimeError("bag does not end with a landed vehicle status")
    return segments


def calculate_tracking_metrics(target_samples, position_samples):
    segments = target_segments(target_samples)
    validate_finite_positions(position_samples, "position trajectory")
    target_times = [sample[0] for sample in target_samples]
    mission_end_ns = target_samples[-1][0]
    full_mission_errors = []

    for timestamp_ns, north, east, down in position_samples:
        if timestamp_ns < target_times[0] or timestamp_ns > mission_end_ns:
            continue
        target_index = bisect_right(target_times, timestamp_ns) - 1
        target = target_samples[target_index]
        error = math.dist((north, east, down), target[1:4])
        full_mission_errors.append(error)

    steady_state_errors = []
    for start_ns, end_ns, target in segments:
        window_start_ns = end_ns - STEADY_WINDOW_NS
        segment_positions = [
            sample
            for sample in position_samples
            if window_start_ns <= sample[0] <= end_ns
        ]
        if len(segment_positions) < MIN_STEADY_SAMPLES_PER_SEGMENT:
            raise RuntimeError(
                "steady-state window has too few position samples"
            )
        if (
            segment_positions[0][0]
            > window_start_ns + WINDOW_COVERAGE_TOLERANCE_NS
            or segment_positions[-1][0]
            < end_ns - WINDOW_COVERAGE_TOLERANCE_NS
        ):
            raise RuntimeError(
                "position samples do not cover a full steady window"
            )
        steady_state_errors.extend(
            math.dist((north, east, down), target)
            for _timestamp_ns, north, east, down in segment_positions
        )

    return {
        "coordinate_frame": "NED",
        "full_mission_setpoint_error": summarize_errors(full_mission_errors),
        "steady_state_error": {
            "definition": (
                "last 1.0 s of each of "
                f"{len(segments)} target segments"
            ),
            "segment_count": len(segments),
            **summarize_errors(steady_state_errors),
        },
    }


def write_metrics(target_samples, position_samples, output_path):
    metrics = calculate_tracking_metrics(target_samples, position_samples)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="\n") as output_file:
        json.dump(metrics, output_file, indent=2)
        output_file.write("\n")
    return metrics


def plot_trajectory(target_samples, position_samples, output_path):
    start_ns = min(target_samples[0][0], position_samples[0][0])
    target_time = relative_seconds(target_samples, start_ns)
    position_time = relative_seconds(position_samples, start_ns)

    target_north = [sample[1] for sample in target_samples]
    target_east = [sample[2] for sample in target_samples]
    target_up = [-sample[3] for sample in target_samples]
    actual_north = [sample[1] for sample in position_samples]
    actual_east = [sample[2] for sample in position_samples]
    actual_up = [-sample[3] for sample in position_samples]

    figure = Figure(figsize=(12, 8), constrained_layout=True)
    FigureCanvasAgg(figure)
    axes = figure.subplots(2, 2)
    desired_color = "#c2410c"
    actual_color = "#155e75"

    axes[0, 0].plot(
        target_east,
        target_north,
        color=desired_color,
        linewidth=2.0,
        drawstyle="steps-post",
        label="Desired",
    )
    axes[0, 0].plot(
        actual_east,
        actual_north,
        color=actual_color,
        linewidth=1.5,
        label="Actual",
    )
    axes[0, 0].set_title("Horizontal trajectory")
    axes[0, 0].set_xlabel("East (m)")
    axes[0, 0].set_ylabel("North (m)")
    axes[0, 0].axis("equal")
    axes[0, 0].legend()

    series = (
        (
            axes[0, 1],
            target_north,
            actual_north,
            "North position",
            "North (m)",
        ),
        (axes[1, 0], target_east, actual_east, "East position", "East (m)"),
        (
            axes[1, 1],
            target_up,
            actual_up,
            "Altitude above NED origin",
            "Up (m)",
        ),
    )
    for axis, desired, actual, title, ylabel in series:
        axis.step(
            target_time,
            desired,
            where="post",
            color=desired_color,
            linewidth=1.8,
            label="Desired",
        )
        axis.plot(
            position_time,
            actual,
            color=actual_color,
            linewidth=1.2,
            label="Actual",
        )
        axis.set_title(title)
        axis.set_xlabel("Mission time (s)")
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.25)
        axis.legend()

    figure.suptitle("M1 PX4 SITL waypoint mission: desired vs actual")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=160)


def animate_trajectory(target_samples, position_samples, output_path):
    start_ns = position_samples[0][0]
    mission_time = relative_seconds(position_samples, start_ns)
    target_north = [sample[1] for sample in target_samples]
    target_east = [sample[2] for sample in target_samples]
    actual_north = [sample[1] for sample in position_samples]
    actual_east = [sample[2] for sample in position_samples]

    figure = Figure(figsize=(6.4, 6.4), constrained_layout=True)
    FigureCanvasAgg(figure)
    axis = figure.subplots()
    axis.plot(
        target_east,
        target_north,
        color="#c2410c",
        linewidth=2.0,
        drawstyle="steps-post",
        label="Desired",
    )
    actual_line, = axis.plot(
        [], [], color="#155e75", linewidth=2.0, label="Actual"
    )
    vehicle_marker, = axis.plot(
        [], [], "o", color="#0f766e", markersize=8, label="Vehicle"
    )
    time_label = axis.text(
        0.02,
        0.98,
        "",
        transform=axis.transAxes,
        horizontalalignment="left",
        verticalalignment="top",
    )
    axis.set_title("M1 PX4 SITL waypoint mission")
    axis.set_xlabel("East (m)")
    axis.set_ylabel("North (m)")
    axis.axis("equal")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="lower right")

    frame_step = max(1, len(position_samples) // 120)
    frame_indices = list(range(0, len(position_samples), frame_step))
    if frame_indices[-1] != len(position_samples) - 1:
        frame_indices.append(len(position_samples) - 1)

    def update(frame_index):
        actual_line.set_data(
            actual_east[: frame_index + 1],
            actual_north[: frame_index + 1],
        )
        vehicle_marker.set_data(
            [actual_east[frame_index]], [actual_north[frame_index]]
        )
        time_label.set_text(f"Mission time: {mission_time[frame_index]:.1f} s")
        return actual_line, vehicle_marker, time_label

    animation = FuncAnimation(
        figure,
        update,
        frames=frame_indices,
        interval=100,
        blit=True,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    animation.save(output_path, writer=PillowWriter(fps=10), dpi=110)


def main():
    args = parse_args()
    (
        target_samples,
        position_samples,
        status_samples,
        land_samples,
    ) = read_trajectory(args.bag)
    validate_mission_evidence(
        target_samples, position_samples, status_samples, land_samples
    )
    plot_trajectory(target_samples, position_samples, args.output)
    if args.animation:
        animate_trajectory(target_samples, position_samples, args.animation)
    if args.metrics:
        metrics = write_metrics(target_samples, position_samples, args.metrics)
    print(
        f"wrote {args.output} from "
        f"{len(target_samples)} target and "
        f"{len(position_samples)} position samples"
    )
    if args.animation:
        print(f"wrote {args.animation}")
    if args.metrics:
        print(f"wrote {args.metrics}: {json.dumps(metrics, sort_keys=True)}")


if __name__ == "__main__":
    main()
