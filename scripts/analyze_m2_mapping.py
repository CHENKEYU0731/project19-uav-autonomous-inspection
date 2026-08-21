#!/usr/bin/env python3

import argparse
from array import array
from bisect import bisect_left
from dataclasses import dataclass
import json
import math
from pathlib import Path
from statistics import median


GRID_TOPIC = "/local_occupancy_grid"
DIAGNOSTIC_TOPIC = "/drone_perception/diagnostics"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
MIN_GRID_SAMPLES = 10
MIN_WINDOW_SAMPLES = 3
MIN_RATE_HZ = 5.0
MAX_GRID_GAP_S = 0.5
MAX_TF_OFFSET_NS = 250_000_000
MIN_PLANAR_MOVEMENT_M = 1.0
MIN_AIRBORNE_ALTITUDE_M = 2.0
HOVER_RADIUS_M = 0.3
ALIGNMENT_TOLERANCE_M = 0.25
MIN_ALIGNMENT_RATIO = 0.5

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
class DiagnosticSample:
    timestamp_ns: int
    processing_latency_ms: float
    output_rate_hz: float
    used_depth_count: int
    occupied_cell_count: int


@dataclass
class MappingEvidence:
    grids: list
    poses: list
    diagnostics: list
    tf_links: set
    child_frames: set


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
    tf_links = set()
    child_frames = set()
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
        else:
            append_transforms(
                poses,
                tf_links,
                child_frames,
                message,
                received_ns,
            )

    return MappingEvidence(
        grids=grids,
        poses=poses,
        diagnostics=diagnostics,
        tf_links=tf_links,
        child_frames=child_frames,
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
                )
            )
        except ValueError as error:
            raise RuntimeError(
                "mapping diagnostics contain non-numeric values"
            ) from error


def append_transforms(
    poses, tf_links, child_frames, message, received_ns
):
    for transform in message.transforms:
        parent = normalized_frame(transform.header.frame_id)
        child = normalized_frame(transform.child_frame_id)
        tf_links.add((parent, child))
        child_frames.add(child)
        if (parent, child) != ("map", "base_link"):
            continue
        translation = transform.transform.translation
        poses.append(
            PoseSample(
                timestamp_ns=stamp_ns(
                    transform.header.stamp, received_ns
                ),
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
    representative = None
    best_aligned_count = -1
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
        if snapshot_aligned > best_aligned_count:
            best_aligned_count = snapshot_aligned
            representative = grid

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
    return {
        "snapshot_count": len(window),
        "occupied_cell_count": occupied_count,
        "aligned_cell_count": aligned_count,
        "alignment_ratio": alignment_ratio,
        "matched_obstacles": sorted(matched_obstacles),
        "representative_timestamp_ns": representative.timestamp_ns,
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

    required_links = {
        ("map", "base_link"),
        ("base_link", "camera_optical_frame"),
    }
    missing_links = sorted(required_links - evidence.tf_links)
    if missing_links:
        raise RuntimeError(f"missing required TF link: {missing_links[0]}")
    if "map" in evidence.child_frames:
        raise RuntimeError("map is not a fixed root frame")

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
    max_gap_s = max(intervals_s)
    if median_rate_hz < MIN_RATE_HZ:
        raise RuntimeError("mapping output must run at least 5 Hz")
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

    airborne = [
        pose for pose in poses if pose.z >= MIN_AIRBORNE_ALTITUDE_M
    ]
    if not airborne:
        raise RuntimeError("bag has no airborne base_link poses")
    hover_reference = airborne[0]
    hover_window = []
    motion_window = []
    for grid, pose in paired:
        if pose.z < MIN_AIRBORNE_ALTITUDE_M:
            continue
        displacement = math.hypot(
            pose.x - hover_reference.x,
            pose.y - hover_reference.y,
        )
        if displacement <= HOVER_RADIUS_M:
            hover_window.append((grid, pose))
        if displacement >= MIN_PLANAR_MOVEMENT_M:
            motion_window.append((grid, pose))
    if len(hover_window) < MIN_WINDOW_SAMPLES:
        raise RuntimeError("bag lacks a stable airborne hover window")
    if len(motion_window) < MIN_WINDOW_SAMPLES:
        raise RuntimeError("bag lacks a mapping window after 1.0 m motion")

    hover_alignment = obstacle_alignment(hover_window, "hover")
    motion_alignment = obstacle_alignment(motion_window, "motion")
    occupied_counts = [
        sum(value == 100 for value in grid.cells)
        for grid in evidence.grids
    ]
    if max(occupied_counts) == 0:
        raise RuntimeError("all occupancy grids are empty")

    return {
        "verdict": "accepted",
        "coordinate_frames": {
            "map_is_fixed": True,
            "required_tf_links": [
                "map -> base_link",
                "base_link -> camera_optical_frame",
            ],
            "observed_tf_links": [
                f"{parent} -> {child}"
                for parent, child in sorted(evidence.tf_links)
            ],
        },
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
            f"{title}: {alignment['alignment_ratio']:.1%} aligned"
        )
        axis.set_xlabel("East / map x (m)")
        axis.set_ylabel("North / map y (m)")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")
    figure.suptitle("M2 local occupancy grid vs project world geometry")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=160)


def write_metrics(metrics, output_path, bag_path):
    output = {
        "bag_path": str(bag_path),
        **metrics,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="\n") as file:
        json.dump(output, file, indent=2, sort_keys=True)
        file.write("\n")


def main():
    args = parse_args()
    try:
        evidence = read_bag(args.bag)
        metrics = validate_evidence(evidence)
        plot_evidence(evidence, metrics, args.plot)
        write_metrics(metrics, args.metrics, args.bag)
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


if __name__ == "__main__":
    main()
