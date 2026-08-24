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
from array import array
from bisect import bisect_left
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
from statistics import median
import tempfile


GRID_TOPIC = "/local_occupancy_grid"
DIAGNOSTIC_TOPIC = "/drone_perception/diagnostics"
DEPTH_TOPIC = "/camera/depth/image_raw"
CAMERA_INFO_TOPIC = "/camera/depth/camera_info"
VEHICLE_STATUS_TOPIC = "/fmu/out/vehicle_status_v1"
LAND_TOPIC = "/fmu/out/vehicle_land_detected"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
MIN_GRID_SAMPLES = 10
MIN_WINDOW_SAMPLES = 3
MIN_RATE_HZ = 5.0
MAX_GRID_GAP_S = 0.5
MAX_P95_INTERVAL_S = 1.0 / MIN_RATE_HZ
MAX_TF_OFFSET_NS = 250_000_000
MIN_PLANAR_MOVEMENT_M = 1.0
MIN_AIRBORNE_ALTITUDE_M = 2.0
MIN_HOVER_ALTITUDE_M = 2.3
HOVER_RADIUS_M = 0.3
MIN_HOVER_DURATION_S = 1.0
MAX_HOVER_AVERAGE_SPEED_M_S = 0.2
MAX_HOVER_VERTICAL_RANGE_M = 0.05
ALIGNMENT_TOLERANCE_M = 0.25
MIN_ALIGNMENT_RATIO = 0.5
ARMING_STATE_DISARMED = 1
ARMING_STATE_ARMED = 2
NAVIGATION_STATE_OFFBOARD = 14
EXPECTED_CAMERA_TRANSLATION = (0.13233, 0.0, 0.26078)
EXPECTED_CAMERA_QUATERNION = (-0.5, 0.5, -0.5, 0.5)
TRANSFORM_TOLERANCE = 1e-5
MAX_ANIMATION_FRAMES = 60

RUNTIME_SOURCE_PATHS = (
    "scripts/run-px4-sitl.sh",
    "scripts/px4-headless-rcS",
    "src/drone_bringup/config/local_mapping.yaml",
    "src/drone_bringup/config/mapping_mission.yaml",
    "src/drone_bringup/launch/local_mapping.launch.py",
    "src/drone_controller/include/drone_controller/waypoint_tracker.hpp",
    "src/drone_controller/src/waypoint_controller_node.cpp",
    "src/drone_controller/src/waypoint_tracker.cpp",
    "src/drone_perception/include/drone_perception/depth_grid_mapper.hpp",
    "src/drone_perception/include/drone_perception/frame_conversions.hpp",
    "src/drone_perception/src/depth_grid_mapper.cpp",
    "src/drone_perception/src/depth_grid_node.cpp",
    "src/drone_perception/src/frame_conversions.cpp",
    "src/drone_perception/src/px4_tf_broadcaster_node.cpp",
    "src/drone_sim/models/x500_depth_project/model.sdf",
    "src/drone_sim/worlds/inspection.sdf",
)

OBSTACLE_SEGMENTS = (
    ("left_wall", -1.0, -4.0, 11.0, -4.0),
    ("right_wall", -1.0, 4.0, 11.0, 4.0),
    ("doorway", 5.0, -4.0, 5.0, 4.0),
)
OBSTACLE_CIRCLES = (
    ("column_left", 3.0, -2.0, 0.35),
    ("column_right", 3.0, 2.0, 0.35),
)


@dataclass
class GridSample:
    timestamp_ns: int
    frame_id: str
    resolution_m: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    cells: object


@dataclass
class PoseSample:
    timestamp_ns: int
    x: float
    y: float
    z: float


@dataclass
class DepthSample:
    timestamp_ns: int
    frame_id: str
    encoding: str
    width: int
    height: int
    step: int
    data_size: int


@dataclass
class CameraInfoSample:
    timestamp_ns: int
    frame_id: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class TransformSample:
    timestamp_ns: int
    parent: str
    child: str
    is_static: bool
    tx: float
    ty: float
    tz: float
    qx: float
    qy: float
    qz: float
    qw: float


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
class DiagnosticSample:
    timestamp_ns: int
    processing_latency_ms: float
    output_rate_hz: float
    used_depth_count: int
    occupied_cell_count: int
    mapper_parameters: dict | None = None


@dataclass
class MappingEvidence:
    grids: list
    poses: list
    diagnostics: list
    depth_samples: list
    camera_info_samples: list
    transforms: list
    vehicle_status_samples: list
    land_samples: list


def parse_args():
    parser = argparse.ArgumentParser(
        description="Validate an M2 local-mapping ROS 2 bag."
    )
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory")
    parser.add_argument(
        "--metrics",
        required=True,
        type=Path,
        help="Output JSON metrics path",
    )
    parser.add_argument(
        "--plot",
        required=True,
        type=Path,
        help="Output PNG evidence path",
    )
    parser.add_argument(
        "--animation",
        type=Path,
        help="Optional output GIF showing the rolling grid over time",
    )
    return parser.parse_args()


def normalized_frame(frame_id):
    return frame_id.lstrip("/")


def stamp_ns(stamp, fallback_ns):
    timestamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return timestamp_ns if timestamp_ns > 0 else fallback_ns


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
    required_topics = {
        GRID_TOPIC,
        DIAGNOSTIC_TOPIC,
        DEPTH_TOPIC,
        CAMERA_INFO_TOPIC,
        VEHICLE_STATUS_TOPIC,
        LAND_TOPIC,
        TF_TOPIC,
        TF_STATIC_TOPIC,
    }
    missing_topics = sorted(required_topics - topic_types.keys())
    if missing_topics:
        raise RuntimeError(
            "bag is missing required M2 topics: " + ", ".join(missing_topics)
        )

    grids = []
    poses = []
    diagnostics = []
    depth_samples = []
    camera_info_samples = []
    transforms = []
    vehicle_status_samples = []
    land_samples = []
    while reader.has_next():
        topic, serialized, received_ns = reader.read_next()
        if topic not in required_topics:
            continue
        message = deserialize_message(serialized, topic_types[topic])
        if topic == GRID_TOPIC:
            grids.append(
                GridSample(
                    timestamp_ns=stamp_ns(
                        message.header.stamp, received_ns
                    ),
                    frame_id=normalized_frame(message.header.frame_id),
                    resolution_m=float(message.info.resolution),
                    width=int(message.info.width),
                    height=int(message.info.height),
                    origin_x=float(message.info.origin.position.x),
                    origin_y=float(message.info.origin.position.y),
                    cells=array("b", message.data),
                )
            )
        elif topic == DIAGNOSTIC_TOPIC:
            append_diagnostics(diagnostics, message, received_ns)
        elif topic == DEPTH_TOPIC:
            depth_samples.append(
                DepthSample(
                    timestamp_ns=stamp_ns(message.header.stamp, received_ns),
                    frame_id=normalized_frame(message.header.frame_id),
                    encoding=message.encoding,
                    width=int(message.width),
                    height=int(message.height),
                    step=int(message.step),
                    data_size=len(message.data),
                )
            )
        elif topic == CAMERA_INFO_TOPIC:
            camera_info_samples.append(
                CameraInfoSample(
                    timestamp_ns=stamp_ns(message.header.stamp, received_ns),
                    frame_id=normalized_frame(message.header.frame_id),
                    width=int(message.width),
                    height=int(message.height),
                    fx=float(message.k[0]),
                    fy=float(message.k[4]),
                    cx=float(message.k[2]),
                    cy=float(message.k[5]),
                )
            )
        elif topic == VEHICLE_STATUS_TOPIC:
            vehicle_status_samples.append(
                VehicleStatusSample(
                    timestamp_ns=received_ns,
                    arming_state=int(message.arming_state),
                    nav_state=int(message.nav_state),
                    failsafe=bool(message.failsafe),
                )
            )
        elif topic == LAND_TOPIC:
            land_samples.append(
                LandSample(
                    timestamp_ns=received_ns,
                    landed=bool(message.landed),
                )
            )
        else:
            append_transforms(
                poses,
                transforms,
                message,
                received_ns,
                topic == TF_STATIC_TOPIC,
            )

    return MappingEvidence(
        grids=grids,
        poses=poses,
        diagnostics=diagnostics,
        depth_samples=depth_samples,
        camera_info_samples=camera_info_samples,
        transforms=transforms,
        vehicle_status_samples=vehicle_status_samples,
        land_samples=land_samples,
    )


def append_diagnostics(output, message, received_ns):
    for status in message.status:
        if status.name != "drone_perception/local_grid":
            continue
        values = {item.key: item.value for item in status.values}
        required = {
            "processing_latency_ms",
            "output_rate_hz",
            "used_depth_count",
            "occupied_cell_count",
        }
        if not required <= values.keys():
            raise RuntimeError("mapping diagnostics omit required values")
        mapper_parameter_names = {
            "map_frame",
            "base_frame",
            "tf_timeout_s",
            "resolution_m",
            "width_m",
            "height_m",
            "min_depth_m",
            "max_depth_m",
            "min_relative_height_m",
            "max_relative_height_m",
            "pixel_stride",
        }
        present_mapper_parameters = mapper_parameter_names & values.keys()
        if present_mapper_parameters and present_mapper_parameters != mapper_parameter_names:
            raise RuntimeError("mapping diagnostics omit runtime mapper parameters")
        mapper_parameters = (
            {name: values[name] for name in mapper_parameter_names}
            if present_mapper_parameters
            else None
        )
        try:
            output.append(
                DiagnosticSample(
                    timestamp_ns=stamp_ns(
                        message.header.stamp, received_ns
                    ),
                    processing_latency_ms=float(
                        values["processing_latency_ms"]
                    ),
                    output_rate_hz=float(values["output_rate_hz"]),
                    used_depth_count=int(values["used_depth_count"]),
                    occupied_cell_count=int(
                        values["occupied_cell_count"]
                    ),
                    mapper_parameters=mapper_parameters,
                )
            )
        except ValueError as error:
            raise RuntimeError(
                "mapping diagnostics contain non-numeric values"
            ) from error


def append_transforms(poses, transforms, message, received_ns, is_static):
    for transform in message.transforms:
        parent = normalized_frame(transform.header.frame_id)
        child = normalized_frame(transform.child_frame_id)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        timestamp_ns = stamp_ns(transform.header.stamp, received_ns)
        transforms.append(
            TransformSample(
                timestamp_ns=timestamp_ns,
                parent=parent,
                child=child,
                is_static=is_static,
                tx=float(translation.x),
                ty=float(translation.y),
                tz=float(translation.z),
                qx=float(rotation.x),
                qy=float(rotation.y),
                qz=float(rotation.z),
                qw=float(rotation.w),
            )
        )
        if (parent, child) != ("map", "base_link"):
            continue
        poses.append(
            PoseSample(
                timestamp_ns=timestamp_ns,
                x=float(translation.x),
                y=float(translation.y),
                z=float(translation.z),
            )
        )


def require_finite(values, label):
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"{label} contains non-finite values")


def percentile(values, probability):
    ordered = sorted(values)
    if not ordered:
        raise RuntimeError("cannot calculate a percentile of empty data")
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def nearest_pose(poses, pose_times, timestamp_ns):
    index = bisect_left(pose_times, timestamp_ns)
    candidates = []
    if index < len(poses):
        candidates.append(poses[index])
    if index > 0:
        candidates.append(poses[index - 1])
    pose = min(
        candidates,
        key=lambda sample: abs(sample.timestamp_ns - timestamp_ns),
    )
    offset_ns = abs(pose.timestamp_ns - timestamp_ns)
    if offset_ns > MAX_TF_OFFSET_NS:
        raise RuntimeError(
            "grid has no map -> base_link TF within 250 ms"
        )
    return pose, offset_ns


def validate_transforms(evidence):
    if not evidence.transforms:
        raise RuntimeError("bag has no TF samples")
    links = set()
    child_parents = {}
    for transform in evidence.transforms:
        values = (
            transform.tx,
            transform.ty,
            transform.tz,
            transform.qx,
            transform.qy,
            transform.qz,
            transform.qw,
        )
        require_finite(values, "TF transform")
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
        if abs(quaternion_norm - 1.0) > 1e-3:
            raise RuntimeError("TF transform has a non-unit quaternion")
        links.add((transform.parent, transform.child))
        child_parents.setdefault(transform.child, set()).add(
            transform.parent
        )

    required_links = {
        ("map", "base_link"),
        ("base_link", "camera_optical_frame"),
    }
    missing_links = sorted(required_links - links)
    if missing_links:
        raise RuntimeError(f"missing required TF link: {missing_links[0]}")
    if "map" in child_parents:
        raise RuntimeError("map is not a fixed root frame")
    if child_parents.get("base_link") != {"map"}:
        raise RuntimeError("base_link must have map as its unique TF parent")
    if child_parents.get("camera_optical_frame") != {"base_link"}:
        raise RuntimeError(
            "camera_optical_frame must have base_link as its unique TF parent"
        )

    base_transforms = [
        transform
        for transform in evidence.transforms
        if (transform.parent, transform.child) == ("map", "base_link")
    ]
    if any(transform.is_static for transform in base_transforms):
        raise RuntimeError("map -> base_link must be a dynamic transform")
    camera_transforms = [
        transform
        for transform in evidence.transforms
        if (transform.parent, transform.child)
        == ("base_link", "camera_optical_frame")
    ]
    if not camera_transforms or any(
        not transform.is_static for transform in camera_transforms
    ):
        raise RuntimeError("required static camera transform is missing")
    camera = camera_transforms[0]
    translation = (camera.tx, camera.ty, camera.tz)
    if any(
        abs(actual - expected) > TRANSFORM_TOLERANCE
        for actual, expected in zip(
            translation, EXPECTED_CAMERA_TRANSLATION
        )
    ):
        raise RuntimeError("camera transform translation is incorrect")
    quaternion = (camera.qx, camera.qy, camera.qz, camera.qw)
    dot = sum(
        actual * expected
        for actual, expected in zip(
            quaternion, EXPECTED_CAMERA_QUATERNION
        )
    )
    if abs(abs(dot) - 1.0) > TRANSFORM_TOLERANCE:
        raise RuntimeError("camera transform rotation is incorrect")
    return {
        "map_is_fixed": True,
        "required_tf_links": [
            "map -> base_link",
            "base_link -> camera_optical_frame",
        ],
        "observed_tf_links": [
            f"{parent} -> {child}" for parent, child in sorted(links)
        ],
        "camera_transform_is_static": True,
        "camera_translation_m": list(translation),
        "camera_quaternion_xyzw": list(quaternion),
    }


def validate_flight_sequence(evidence):
    statuses = sorted(
        evidence.vehicle_status_samples,
        key=lambda sample: sample.timestamp_ns,
    )
    land_samples = sorted(
        evidence.land_samples,
        key=lambda sample: sample.timestamp_ns,
    )
    if not statuses or not land_samples:
        raise RuntimeError("flight sequence evidence is missing")
    if any(sample.failsafe for sample in statuses):
        raise RuntimeError("flight sequence entered failsafe")

    armed = next(
        (
            sample
            for sample in statuses
            if sample.arming_state == ARMING_STATE_ARMED
            and sample.nav_state == NAVIGATION_STATE_OFFBOARD
        ),
        None,
    )
    if armed is None or not any(
        sample.arming_state == ARMING_STATE_DISARMED
        and sample.timestamp_ns < armed.timestamp_ns
        for sample in statuses
    ):
        raise RuntimeError("flight sequence lacks disarmed -> armed Offboard")
    airborne = next(
        (
            sample
            for sample in land_samples
            if not sample.landed and sample.timestamp_ns > armed.timestamp_ns
        ),
        None,
    )
    landed = next(
        (
            sample
            for sample in land_samples
            if sample.landed
            and airborne is not None
            and sample.timestamp_ns > airborne.timestamp_ns
        ),
        None,
    )
    disarmed = next(
        (
            sample
            for sample in statuses
            if sample.arming_state == ARMING_STATE_DISARMED
            and landed is not None
            and sample.timestamp_ns > landed.timestamp_ns
        ),
        None,
    )
    if airborne is None or landed is None or disarmed is None:
        raise RuntimeError(
            "flight sequence lacks airborne -> landed -> disarmed completion"
        )
    return {
        "status_sample_count": len(statuses),
        "land_sample_count": len(land_samples),
        "armed_offboard": True,
        "airborne": True,
        "landed_and_disarmed": True,
        "failsafe_observed": False,
    }


def validate_sensor_inputs(evidence, grids, occupied_counts):
    if not evidence.depth_samples:
        raise RuntimeError("bag has no depth input samples")
    if not evidence.camera_info_samples:
        raise RuntimeError("bag has no camera-info samples")
    for sample in evidence.depth_samples:
        if (
            sample.frame_id != "camera_optical_frame"
            or sample.encoding != "32FC1"
            or sample.width <= 0
            or sample.height <= 0
            or sample.step < sample.width * 4
            or sample.data_size < sample.height * sample.step
        ):
            raise RuntimeError("depth input contract is invalid")
    for sample in evidence.camera_info_samples:
        intrinsics = (sample.fx, sample.fy, sample.cx, sample.cy)
        if (
            sample.frame_id != "camera_optical_frame"
            or sample.width <= 0
            or sample.height <= 0
            or not all(math.isfinite(value) for value in intrinsics)
            or sample.fx <= 0.0
            or sample.fy <= 0.0
        ):
            raise RuntimeError("camera-info contract is invalid")

    depth_timestamps = {sample.timestamp_ns for sample in evidence.depth_samples}
    if any(grid.timestamp_ns not in depth_timestamps for grid in grids):
        raise RuntimeError("a grid has no depth sample with the same timestamp")
    diagnostics_by_time = {
        sample.timestamp_ns: sample for sample in evidence.diagnostics
    }
    if len(diagnostics_by_time) != len(evidence.diagnostics):
        raise RuntimeError("mapping diagnostics contain duplicate timestamps")
    for grid, occupied_count in zip(grids, occupied_counts):
        diagnostic = diagnostics_by_time.get(grid.timestamp_ns)
        if (
            diagnostic is None
            or diagnostic.used_depth_count <= 0
            or diagnostic.occupied_cell_count != occupied_count
        ):
            raise RuntimeError("grid and diagnostic evidence are inconsistent")
    return {
        "depth_sample_count": len(evidence.depth_samples),
        "camera_info_sample_count": len(evidence.camera_info_samples),
        "validated_grid_depth_pairs": len(grids),
    }


def find_continuous_hover_window(paired):
    maximum_interval_s = 1.0 / MIN_RATE_HZ
    for start_index, (start_grid, reference) in enumerate(paired):
        if reference.z < MIN_HOVER_ALTITUDE_M:
            continue
        window = [(start_grid, reference)]
        previous_grid = start_grid
        previous_pose = reference
        path_length_m = 0.0
        for grid, pose in paired[start_index + 1:]:
            interval_s = (grid.timestamp_ns - previous_grid.timestamp_ns) / 1e9
            distance_from_reference = math.sqrt(
                (pose.x - reference.x) ** 2
                + (pose.y - reference.y) ** 2
                + (pose.z - reference.z) ** 2
            )
            step_distance = math.sqrt(
                (pose.x - previous_pose.x) ** 2
                + (pose.y - previous_pose.y) ** 2
                + (pose.z - previous_pose.z) ** 2
            )
            if (
                interval_s > maximum_interval_s
                or pose.z < MIN_HOVER_ALTITUDE_M
                or distance_from_reference > HOVER_RADIUS_M
            ):
                break
            window.append((grid, pose))
            path_length_m += step_distance
            previous_grid = grid
            previous_pose = pose
            duration_s = (
                window[-1][0].timestamp_ns - window[0][0].timestamp_ns
            ) / 1e9
            if duration_s >= MIN_HOVER_DURATION_S:
                heights = [sample_pose.z for _sample_grid, sample_pose in window]
                vertical_range_m = max(heights) - min(heights)
                average_speed_m_s = path_length_m / duration_s
                if vertical_range_m > MAX_HOVER_VERTICAL_RANGE_M:
                    break
                if average_speed_m_s <= MAX_HOVER_AVERAGE_SPEED_M_S:
                    return (
                        window,
                        reference,
                        duration_s,
                        average_speed_m_s,
                        vertical_range_m,
                    )
    raise RuntimeError("bag lacks a continuous hover window")


def find_motion_window(paired, reference):
    candidates = [
        (grid, pose)
        for grid, pose in paired
        if pose.z >= MIN_AIRBORNE_ALTITUDE_M
        and math.hypot(pose.x - reference.x, pose.y - reference.y)
        >= MIN_PLANAR_MOVEMENT_M
    ]
    if len(candidates) < MIN_WINDOW_SAMPLES:
        raise RuntimeError("bag lacks a mapping window after 1.0 m motion")
    return candidates


def point_segment_distance(x, y, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.hypot(x - x1, y - y1)
    projection = ((x - x1) * dx + (y - y1) * dy) / length_squared
    projection = max(0.0, min(1.0, projection))
    nearest_x = x1 + projection * dx
    nearest_y = y1 + projection * dy
    return math.hypot(x - nearest_x, y - nearest_y)


def nearest_obstacle(x, y):
    distances = [
        (
            point_segment_distance(x, y, x1, y1, x2, y2),
            name,
        )
        for name, x1, y1, x2, y2 in OBSTACLE_SEGMENTS
    ]
    distances.extend(
        (abs(math.hypot(x - cx, y - cy) - radius), name)
        for name, cx, cy, radius in OBSTACLE_CIRCLES
    )
    return min(distances)


def occupied_points(grid):
    points = []
    for index, value in enumerate(grid.cells):
        if value != 100:
            continue
        row, column = divmod(index, grid.width)
        points.append(
            (
                grid.origin_x + (column + 0.5) * grid.resolution_m,
                grid.origin_y + (row + 0.5) * grid.resolution_m,
            )
        )
    return points


def obstacle_alignment(window, label):
    occupied_count = 0
    aligned_count = 0
    matched_obstacles = set()
    snapshots = []
    for grid, _pose in window:
        points = occupied_points(grid)
        occupied_count += len(points)
        snapshot_aligned = 0
        for x, y in points:
            distance, obstacle = nearest_obstacle(x, y)
            tolerance = max(
                ALIGNMENT_TOLERANCE_M, 2.0 * grid.resolution_m
            )
            if distance <= tolerance:
                aligned_count += 1
                snapshot_aligned += 1
                matched_obstacles.add(obstacle)
        snapshots.append(
            (
                grid,
                snapshot_aligned / len(points) if points else 0.0,
                len(points),
            )
        )

    if occupied_count == 0:
        raise RuntimeError(f"{label} window has an empty occupied grid")
    alignment_ratio = aligned_count / occupied_count
    if (
        aligned_count < MIN_WINDOW_SAMPLES
        or alignment_ratio < MIN_ALIGNMENT_RATIO
        or not matched_obstacles
    ):
        raise RuntimeError(
            f"{label} occupied cells do not align with a known obstacle"
        )
    representative, representative_ratio, _count = min(
        snapshots,
        key=lambda item: (
            abs(item[1] - alignment_ratio),
            -item[2],
        ),
    )
    return {
        "snapshot_count": len(window),
        "occupied_cell_count": occupied_count,
        "aligned_cell_count": aligned_count,
        "alignment_ratio": alignment_ratio,
        "matched_obstacles": sorted(matched_obstacles),
        "representative_timestamp_ns": representative.timestamp_ns,
        "representative_alignment_ratio": representative_ratio,
    }


def validate_evidence(evidence):
    if len(evidence.grids) < MIN_GRID_SAMPLES:
        raise RuntimeError(
            f"need at least {MIN_GRID_SAMPLES} occupancy-grid samples"
        )
    if not evidence.poses:
        raise RuntimeError("bag has no map -> base_link pose samples")
    if len(evidence.diagnostics) < MIN_WINDOW_SAMPLES:
        raise RuntimeError("bag has too few mapping diagnostics")

    coordinate_frames = validate_transforms(evidence)
    flight_sequence = validate_flight_sequence(evidence)

    poses = sorted(evidence.poses, key=lambda pose: pose.timestamp_ns)
    require_finite(
        [value for pose in poses for value in (pose.x, pose.y, pose.z)],
        "TF pose",
    )
    origin_pose = poses[0]
    planar_displacement_m = max(
        math.hypot(pose.x - origin_pose.x, pose.y - origin_pose.y)
        for pose in poses
    )
    if planar_displacement_m < MIN_PLANAR_MOVEMENT_M:
        raise RuntimeError("base_link must move at least 1.0 m")

    grid_timestamps = [grid.timestamp_ns for grid in evidence.grids]
    if any(
        current <= previous
        for previous, current in zip(
            grid_timestamps, grid_timestamps[1:]
        )
    ):
        raise RuntimeError("grid timestamps must be strictly increasing")
    intervals_s = [
        (current - previous) / 1e9
        for previous, current in zip(
            grid_timestamps, grid_timestamps[1:]
        )
    ]
    require_finite(intervals_s, "grid timing")
    median_rate_hz = 1.0 / median(intervals_s)
    duration_s = sum(intervals_s)
    average_rate_hz = (len(evidence.grids) - 1) / duration_s
    p95_interval_s = percentile(intervals_s, 0.95)
    max_gap_s = max(intervals_s)
    if median_rate_hz < MIN_RATE_HZ:
        raise RuntimeError("mapping output must run at least 5 Hz")
    if average_rate_hz < MIN_RATE_HZ:
        raise RuntimeError("mapping average rate must be at least 5 Hz")
    if p95_interval_s > MAX_P95_INTERVAL_S:
        raise RuntimeError("mapping P95 interval exceeds the 5 Hz period")
    if max_gap_s > MAX_GRID_GAP_S:
        raise RuntimeError("mapping output contains a gap longer than 0.5 s")

    pose_times = [pose.timestamp_ns for pose in poses]
    paired = []
    center_errors = []
    tf_offsets_ms = []
    for grid in evidence.grids:
        require_finite(
            [grid.resolution_m, grid.origin_x, grid.origin_y],
            "occupancy grid",
        )
        if grid.frame_id != "map":
            raise RuntimeError("every occupancy grid must use frame map")
        if grid.resolution_m <= 0.0 or grid.width <= 0 or grid.height <= 0:
            raise RuntimeError("occupancy grid geometry must be positive")
        if len(grid.cells) != grid.width * grid.height:
            raise RuntimeError("occupancy grid data length is invalid")
        if any(value not in (-1, 0, 100) for value in grid.cells):
            raise RuntimeError("occupancy grid contains unsupported values")
        pose, offset_ns = nearest_pose(
            poses, pose_times, grid.timestamp_ns
        )
        center_x = (
            grid.origin_x + grid.width * grid.resolution_m / 2.0
        )
        center_y = (
            grid.origin_y + grid.height * grid.resolution_m / 2.0
        )
        center_error = math.hypot(center_x - pose.x, center_y - pose.y)
        center_errors.append(center_error)
        tf_offsets_ms.append(offset_ns / 1e6)
        paired.append((grid, pose))

    maximum_center_error = max(center_errors)
    maximum_allowed_error = max(
        0.15,
        1.5 * max(grid.resolution_m for grid in evidence.grids),
    )
    if maximum_center_error > maximum_allowed_error:
        raise RuntimeError("rolling grid center does not follow base_link")

    occupied_counts = [
        sum(value == 100 for value in grid.cells)
        for grid in evidence.grids
    ]
    if max(occupied_counts) == 0:
        raise RuntimeError("all occupancy grids have an empty occupied grid")
    sensor_inputs = validate_sensor_inputs(
        evidence, evidence.grids, occupied_counts
    )

    latency_ms = [
        sample.processing_latency_ms for sample in evidence.diagnostics
    ]
    diagnostic_rates = [
        sample.output_rate_hz for sample in evidence.diagnostics
    ]
    if not all(
        math.isfinite(value) and value >= 0.0 for value in latency_ms
    ):
        raise RuntimeError("mapping diagnostics contain non-finite latency")
    if not all(
        math.isfinite(value) and value >= 0.0
        for value in diagnostic_rates
    ):
        raise RuntimeError("mapping diagnostics contain invalid output rate")
    if median(diagnostic_rates) < MIN_RATE_HZ:
        raise RuntimeError("diagnostic output rate must be at least 5 Hz")

    (
        hover_window,
        hover_reference,
        hover_duration_s,
        hover_average_speed_m_s,
        hover_vertical_range_m,
    ) = (
        find_continuous_hover_window(paired)
    )
    motion_window = find_motion_window(paired, hover_reference)

    hover_alignment = obstacle_alignment(hover_window, "hover")
    hover_alignment["continuous_duration_s"] = hover_duration_s
    hover_alignment["minimum_altitude_m"] = MIN_HOVER_ALTITUDE_M
    hover_alignment["average_speed_m_s"] = hover_average_speed_m_s
    hover_alignment["vertical_range_m"] = hover_vertical_range_m
    motion_alignment = obstacle_alignment(motion_window, "motion")

    return {
        "verdict": "accepted",
        "coordinate_frames": coordinate_frames,
        "flight_sequence": flight_sequence,
        "sensor_inputs": sensor_inputs,
        "motion": {
            "planar_displacement_m": planar_displacement_m,
            "required_m": MIN_PLANAR_MOVEMENT_M,
            "pose_sample_count": len(poses),
        },
        "rolling_grid": {
            "sample_count": len(evidence.grids),
            "median_center_error_m": median(center_errors),
            "max_center_error_m": maximum_center_error,
            "max_tf_offset_ms": max(tf_offsets_ms),
            "occupied_cells_min": min(occupied_counts),
            "occupied_cells_median": median(occupied_counts),
            "occupied_cells_max": max(occupied_counts),
        },
        "mapping_rate": {
            "median_hz": median_rate_hz,
            "average_hz": average_rate_hz,
            "p95_interval_s": p95_interval_s,
            "maximum_gap_s": max_gap_s,
            "required_hz": MIN_RATE_HZ,
        },
        "processing_latency_ms": {
            "sample_count": len(latency_ms),
            "median": median(latency_ms),
            "p95": percentile(latency_ms, 0.95),
            "maximum": max(latency_ms),
        },
        "diagnostic_output_rate_hz": {
            "median": median(diagnostic_rates),
            "minimum": min(diagnostic_rates),
            "maximum": max(diagnostic_rates),
        },
        "hover_obstacle_alignment": hover_alignment,
        "motion_obstacle_alignment": motion_alignment,
    }


def draw_known_obstacles(axis):
    from matplotlib.patches import Circle

    for name, x1, y1, x2, y2 in OBSTACLE_SEGMENTS:
        axis.plot(
            [x1, x2],
            [y1, y2],
            color="#b91c1c",
            linewidth=2.0,
            label="Known world geometry" if name == "left_wall" else None,
        )
    for _name, cx, cy, radius in OBSTACLE_CIRCLES:
        axis.add_patch(
            Circle(
                (cx, cy),
                radius,
                fill=False,
                edgecolor="#b91c1c",
                linewidth=2.0,
            )
        )


def plot_evidence(evidence, metrics, output_path):
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    pose_times = [pose.timestamp_ns for pose in evidence.poses]
    figure = Figure(figsize=(12, 6), constrained_layout=True)
    FigureCanvasAgg(figure)
    axes = figure.subplots(1, 2)
    windows = (
        ("Hover mapping", metrics["hover_obstacle_alignment"]),
        ("Mapping after motion", metrics["motion_obstacle_alignment"]),
    )
    for axis, (title, alignment) in zip(axes, windows):
        timestamp_ns = alignment["representative_timestamp_ns"]
        grid = next(
            sample
            for sample in evidence.grids
            if sample.timestamp_ns == timestamp_ns
        )
        pose, _offset_ns = nearest_pose(
            evidence.poses, pose_times, timestamp_ns
        )
        points = occupied_points(grid)
        axis.scatter(
            [point[0] for point in points],
            [point[1] for point in points],
            s=8,
            color="#155e75",
            label="Occupied cells",
        )
        draw_known_obstacles(axis)
        axis.plot(
            [sample.x for sample in evidence.poses],
            [sample.y for sample in evidence.poses],
            color="#64748b",
            linewidth=1.0,
            label="base_link path",
        )
        axis.scatter(
            [pose.x],
            [pose.y],
            s=45,
            color="#0f766e",
            label="base_link",
            zorder=5,
        )
        axis.set_title(
            f"{title}: window {alignment['alignment_ratio']:.1%}; "
            f"shown frame "
            f"{alignment['representative_alignment_ratio']:.1%}"
        )
        axis.set_xlabel("East / map x (m)")
        axis.set_ylabel("North / map y (m)")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")
    figure.suptitle("M2 local occupancy grid vs project world geometry")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=160)


def animation_grids(grids, max_frames=MAX_ANIMATION_FRAMES):
    if max_frames < 2:
        raise ValueError("animation requires at least two output frames")
    if len(grids) <= max_frames:
        return list(grids)
    return [
        grids[round(index * (len(grids) - 1) / (max_frames - 1))]
        for index in range(max_frames)
    ]


def animate_evidence(evidence, output_path, max_frames=MAX_ANIMATION_FRAMES):
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    grids = animation_grids(evidence.grids, max_frames)
    if len(grids) < 2:
        raise RuntimeError("need at least two grids for animation")

    poses = sorted(evidence.poses, key=lambda pose: pose.timestamp_ns)
    pose_times = [pose.timestamp_ns for pose in poses]
    figure = Figure(figsize=(9, 6), constrained_layout=True)
    FigureCanvasAgg(figure)
    axis = figure.subplots()
    draw_known_obstacles(axis)
    path_line, = axis.plot(
        [], [], color="#64748b", linewidth=1.2, label="base_link path"
    )
    occupied = axis.scatter(
        [], [], s=10, color="#155e75", label="Occupied cells"
    )
    vehicle = axis.scatter(
        [], [], s=55, color="#0f766e", label="base_link", zorder=5
    )
    title = axis.set_title("")

    minimum_x = min(
        [grid.origin_x for grid in grids]
        + [segment[1] for segment in OBSTACLE_SEGMENTS]
        + [circle[1] - circle[3] for circle in OBSTACLE_CIRCLES]
    )
    maximum_x = max(
        [grid.origin_x + grid.width * grid.resolution_m for grid in grids]
        + [segment[3] for segment in OBSTACLE_SEGMENTS]
        + [circle[1] + circle[3] for circle in OBSTACLE_CIRCLES]
    )
    minimum_y = min(
        [grid.origin_y for grid in grids]
        + [segment[2] for segment in OBSTACLE_SEGMENTS]
        + [circle[2] - circle[3] for circle in OBSTACLE_CIRCLES]
    )
    maximum_y = max(
        [grid.origin_y + grid.height * grid.resolution_m for grid in grids]
        + [segment[4] for segment in OBSTACLE_SEGMENTS]
        + [circle[2] + circle[3] for circle in OBSTACLE_CIRCLES]
    )
    axis.set_xlim(minimum_x - 0.5, maximum_x + 0.5)
    axis.set_ylim(minimum_y - 0.5, maximum_y + 0.5)
    axis.set_xlabel("East / map x (m)")
    axis.set_ylabel("North / map y (m)")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best")

    start_ns = grids[0].timestamp_ns

    def update(grid):
        points = occupied_points(grid)
        occupied.set_offsets(points or [(math.nan, math.nan)])
        pose, _offset_ns = nearest_pose(poses, pose_times, grid.timestamp_ns)
        pose_index = bisect_left(pose_times, pose.timestamp_ns)
        path = poses[: pose_index + 1]
        path_line.set_data(
            [sample.x for sample in path],
            [sample.y for sample in path],
        )
        vehicle.set_offsets([(pose.x, pose.y)])
        elapsed_s = (grid.timestamp_ns - start_ns) / 1e9
        title.set_text(
            f"M2 rolling local grid: t={elapsed_s:.1f} s, "
            f"occupied={len(points)}"
        )
        return occupied, path_line, vehicle, title

    animation = FuncAnimation(
        figure,
        update,
        frames=grids,
        interval=100,
        blit=False,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    animation.save(output_path, writer=PillowWriter(fps=10), dpi=100)


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_bag(bag_path):
    digest = hashlib.sha256()
    files = sorted(
        path
        for path in bag_path.iterdir()
        if path.is_file()
    )
    for path in files:
        digest.update(path.name.encode("utf-8"))
        digest.update(bytes.fromhex(sha256_file(path)))
    return digest.hexdigest(), [path.name for path in files]


def add_provenance(metrics, bag_path):
    project_root = Path(__file__).resolve().parent.parent
    world_path = project_root / "src" / "drone_sim" / "worlds" / "inspection.sdf"
    if not world_path.is_file():
        raise RuntimeError(f"project world not found: {world_path}")
    bag_sha256, bag_files = sha256_bag(bag_path)
    metrics["provenance"] = {
        "bag_sha256": bag_sha256,
        "bag_files": bag_files,
        "analysis_script_sha256": sha256_file(Path(__file__).resolve()),
        "world_sha256": sha256_file(world_path),
        "runtime_source_sha256": {
            relative_path: sha256_file(project_root / relative_path)
            for relative_path in RUNTIME_SOURCE_PATHS
        },
    }


def write_metrics(metrics, output_path, bag_path):
    output = {
        "bag_path": str(bag_path),
        **metrics,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="\n") as file:
        json.dump(output, file, indent=2, sort_keys=True)
        file.write("\n")


def temporary_path_for(output_path):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(
        prefix=f".{output_path.stem}-",
        suffix=output_path.suffix,
        dir=output_path.parent,
    )
    os.close(descriptor)
    return Path(name)


def main():
    args = parse_args()
    try:
        evidence = read_bag(args.bag)
        metrics = validate_evidence(evidence)
        add_provenance(metrics, args.bag)
        temporary_plot = temporary_path_for(args.plot)
        temporary_metrics = temporary_path_for(args.metrics)
        temporary_animation = (
            temporary_path_for(args.animation) if args.animation else None
        )
        try:
            plot_evidence(evidence, metrics, temporary_plot)
            if temporary_animation:
                animate_evidence(evidence, temporary_animation)
            write_metrics(metrics, temporary_metrics, args.bag)
            os.replace(temporary_plot, args.plot)
            os.replace(temporary_metrics, args.metrics)
            if temporary_animation:
                os.replace(temporary_animation, args.animation)
        finally:
            temporary_plot.unlink(missing_ok=True)
            temporary_metrics.unlink(missing_ok=True)
            if temporary_animation:
                temporary_animation.unlink(missing_ok=True)
    except RuntimeError as error:
        raise SystemExit(f"M2 evidence rejected: {error}") from error
    print(
        "M2 evidence accepted: "
        f"{metrics['rolling_grid']['sample_count']} grids, "
        f"{metrics['mapping_rate']['median_hz']:.2f} Hz median, "
        f"{metrics['motion']['planar_displacement_m']:.2f} m motion"
    )
    print(f"wrote {args.metrics}")
    print(f"wrote {args.plot}")
    if args.animation:
        print(f"wrote {args.animation}")


if __name__ == "__main__":
    main()
