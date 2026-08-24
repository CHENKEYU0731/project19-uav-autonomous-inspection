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
import hashlib
import heapq
import json
import math
from pathlib import Path
import sys

import yaml


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
if str(SCRIPT_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIRECTORY))

import analyze_m2_mapping as m2  # noqa: E402


UNKNOWN = -1
FREE = 0
OCCUPIED = 100
SQRT_TWO = math.sqrt(2.0)
RESOLUTION_RELATIVE_TOLERANCE = 1e-12
RUNTIME_MAPPING_PARAMETERS = (
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
)
FLOAT_MAPPING_PARAMETERS = RUNTIME_MAPPING_PARAMETERS[2:-1]


class FusedGrid:
    def __init__(self, resolution_m, width, height, origin_x, origin_y, cells):
        self.resolution_m = float(resolution_m)
        self.width = int(width)
        self.height = int(height)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.cells = list(cells)
        self._validate()

    @classmethod
    def from_sample(cls, sample):
        return cls(
            sample.resolution_m,
            sample.width,
            sample.height,
            sample.origin_x,
            sample.origin_y,
            sample.cells,
        )

    def _validate(self):
        if (
            not math.isfinite(self.resolution_m)
            or self.resolution_m <= 0.0
            or self.width <= 0
            or self.height <= 0
            or len(self.cells) != self.width * self.height
            or not math.isfinite(self.origin_x)
            or not math.isfinite(self.origin_y)
        ):
            raise ValueError("invalid fused-grid geometry")
        if any(value not in (UNKNOWN, FREE, OCCUPIED) for value in self.cells):
            raise ValueError("grid cells must be unknown, free, or occupied")

    def copy(self):
        return FusedGrid(
            self.resolution_m,
            self.width,
            self.height,
            self.origin_x,
            self.origin_y,
            self.cells,
        )

    def index(self, column, row):
        return row * self.width + column

    def cell_center(self, column, row):
        return (
            self.origin_x + (column + 0.5) * self.resolution_m,
            self.origin_y + (row + 0.5) * self.resolution_m,
        )

    def world_to_cell(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        column = math.floor((x - self.origin_x) / self.resolution_m)
        row = math.floor((y - self.origin_y) / self.resolution_m)
        if column < 0 or row < 0 or column >= self.width or row >= self.height:
            return None
        return int(column), int(row)

    def integrate(self, sample):
        observed_resolution = float(sample.resolution_m)
        observed_width = int(sample.width)
        observed_height = int(sample.height)
        observed_origin_x = float(sample.origin_x)
        observed_origin_y = float(sample.origin_y)
        if (
            not math.isfinite(observed_resolution)
            or abs(observed_resolution - self.resolution_m)
            > RESOLUTION_RELATIVE_TOLERANCE
            * max(observed_resolution, self.resolution_m)
            or observed_width <= 0
            or observed_height <= 0
            or len(sample.cells) != observed_width * observed_height
            or not math.isfinite(observed_origin_x)
            or not math.isfinite(observed_origin_y)
            or any(value not in (UNKNOWN, FREE, OCCUPIED) for value in sample.cells)
        ):
            raise ValueError("incompatible rolling grid")

        observed_maximum_x = observed_origin_x + observed_width * self.resolution_m
        observed_maximum_y = observed_origin_y + observed_height * self.resolution_m
        if not math.isfinite(observed_maximum_x) or not math.isfinite(observed_maximum_y):
            raise ValueError("incompatible rolling grid")

        for target_row in range(self.height):
            target_minimum_y = self.origin_y + target_row * self.resolution_m
            target_maximum_y = target_minimum_y + self.resolution_m
            for target_column in range(self.width):
                target_minimum_x = self.origin_x + target_column * self.resolution_m
                target_maximum_x = target_minimum_x + self.resolution_m
                if (
                    target_maximum_x <= observed_origin_x
                    or target_minimum_x >= observed_maximum_x
                    or target_maximum_y <= observed_origin_y
                    or target_minimum_y >= observed_maximum_y
                ):
                    continue

                first_column = max(
                    0,
                    math.floor(
                        (target_minimum_x - observed_origin_x)
                        / self.resolution_m
                    ) - 1,
                )
                last_column = min(
                    observed_width - 1,
                    math.ceil(
                        (target_maximum_x - observed_origin_x)
                        / self.resolution_m
                    ) + 1,
                )
                first_row = max(
                    0,
                    math.floor(
                        (target_minimum_y - observed_origin_y)
                        / self.resolution_m
                    ) - 1,
                )
                last_row = min(
                    observed_height - 1,
                    math.ceil(
                        (target_maximum_y - observed_origin_y)
                        / self.resolution_m
                    ) + 1,
                )

                has_overlap = False
                has_occupied_overlap = False
                all_overlapping_cells_are_free = True
                for observed_row in range(first_row, last_row + 1):
                    cell_minimum_y = observed_origin_y + observed_row * self.resolution_m
                    cell_maximum_y = cell_minimum_y + self.resolution_m
                    for observed_column in range(first_column, last_column + 1):
                        cell_minimum_x = (
                            observed_origin_x
                            + observed_column * self.resolution_m
                        )
                        cell_maximum_x = cell_minimum_x + self.resolution_m
                        overlaps = (
                            max(target_minimum_x, cell_minimum_x)
                            < min(target_maximum_x, cell_maximum_x)
                            and max(target_minimum_y, cell_minimum_y)
                            < min(target_maximum_y, cell_maximum_y)
                        )
                        if not overlaps:
                            continue
                        has_overlap = True
                        value = sample.cells[
                            observed_row * observed_width + observed_column
                        ]
                        has_occupied_overlap = (
                            has_occupied_overlap or value == OCCUPIED
                        )
                        all_overlapping_cells_are_free = (
                            all_overlapping_cells_are_free and value == FREE
                        )

                target_index = self.index(target_column, target_row)
                if has_occupied_overlap:
                    self.cells[target_index] = OCCUPIED
                elif (
                    has_overlap
                    and all_overlapping_cells_are_free
                    and target_minimum_x >= observed_origin_x
                    and target_maximum_x <= observed_maximum_x
                    and target_minimum_y >= observed_origin_y
                    and target_maximum_y <= observed_maximum_y
                ):
                    self.cells[target_index] = FREE

    def _disk_candidates(self, x, y, radius_m):
        if (
            not all(math.isfinite(value) for value in (x, y, radius_m))
            or radius_m < 0.0
        ):
            raise ValueError("disk center and radius must be finite")
        min_column = max(
            0, math.floor((x - radius_m - self.origin_x) / self.resolution_m)
        )
        min_row = max(
            0, math.floor((y - radius_m - self.origin_y) / self.resolution_m)
        )
        max_column = min(
            self.width - 1,
            math.floor((x + radius_m - self.origin_x) / self.resolution_m),
        )
        max_row = min(
            self.height - 1,
            math.floor((y + radius_m - self.origin_y) / self.resolution_m),
        )
        for row in range(min_row, max_row + 1):
            for column in range(min_column, max_column + 1):
                yield column, row

    def clear_disk(self, x, y, radius_m):
        if radius_m > math.hypot(
            self.width * self.resolution_m,
            self.height * self.resolution_m,
        ):
            raise ValueError("footprint radius exceeds the grid diagonal")
        for column, row in self._disk_candidates(x, y, radius_m):
            minimum_x = self.origin_x + column * self.resolution_m
            maximum_x = minimum_x + self.resolution_m
            minimum_y = self.origin_y + row * self.resolution_m
            maximum_y = minimum_y + self.resolution_m
            farthest_x = max(abs(minimum_x - x), abs(maximum_x - x))
            farthest_y = max(abs(minimum_y - y), abs(maximum_y - y))
            if math.hypot(farthest_x, farthest_y) <= radius_m:
                self.cells[self.index(column, row)] = FREE

    def add_disk_obstacle(self, x, y, radius_m):
        for column, row in self._disk_candidates(x, y, radius_m):
            minimum_x = self.origin_x + column * self.resolution_m
            maximum_x = minimum_x + self.resolution_m
            minimum_y = self.origin_y + row * self.resolution_m
            maximum_y = minimum_y + self.resolution_m
            closest_x = min(max(x, minimum_x), maximum_x)
            closest_y = min(max(y, minimum_y), maximum_y)
            if math.hypot(closest_x - x, closest_y - y) <= radius_m:
                self.cells[self.index(column, row)] = OCCUPIED


def blocked_mask(grid, inflation_radius_m):
    if not math.isfinite(inflation_radius_m) or inflation_radius_m < 0.0:
        raise ValueError("inflation radius must be finite and non-negative")
    blocked = [value != FREE for value in grid.cells]
    occupied = [
        (column, row)
        for row in range(grid.height)
        for column in range(grid.width)
        if grid.cells[grid.index(column, row)] == OCCUPIED
    ]
    if inflation_radius_m == 0.0:
        return blocked

    cell_margin = grid.resolution_m * SQRT_TWO / 2.0
    cell_radius = math.ceil(
        (inflation_radius_m + cell_margin) / grid.resolution_m
    )
    maximum_distance = inflation_radius_m + cell_margin
    for obstacle_column, obstacle_row in occupied:
        for delta_row in range(-cell_radius, cell_radius + 1):
            for delta_column in range(-cell_radius, cell_radius + 1):
                column = obstacle_column + delta_column
                row = obstacle_row + delta_row
                if not (0 <= column < grid.width and 0 <= row < grid.height):
                    continue
                if math.hypot(delta_column, delta_row) * grid.resolution_m <= maximum_distance:
                    blocked[grid.index(column, row)] = True
    return blocked


def _octile_distance(first, second):
    delta_column = abs(first[0] - second[0])
    delta_row = abs(first[1] - second[1])
    return (
        delta_column + delta_row
        + (SQRT_TWO - 2.0) * min(delta_column, delta_row)
    )


def _reconstruct_path(parent, goal):
    path = [goal]
    while path[-1] in parent:
        path.append(parent[path[-1]])
    path.reverse()
    return path


def find_path(grid, start_xy, goal_xy, inflation_radius_m):
    start = grid.world_to_cell(*start_xy)
    goal = grid.world_to_cell(*goal_xy)
    if start is None or goal is None:
        return []
    blocked = blocked_mask(grid, inflation_radius_m)
    if blocked[grid.index(*start)] or blocked[grid.index(*goal)]:
        return []

    queue = []
    start_h = _octile_distance(start, goal)
    heapq.heappush(queue, (start_h, start_h, 0.0, start[1], start[0]))
    best_cost = {start: 0.0}
    parent = {}
    closed = set()
    neighbours = (
        (-1, -1, SQRT_TWO), (0, -1, 1.0), (1, -1, SQRT_TWO),
        (-1, 0, 1.0), (1, 0, 1.0),
        (-1, 1, SQRT_TWO), (0, 1, 1.0), (1, 1, SQRT_TWO),
    )

    while queue:
        _f_score, _h_score, cost, row, column = heapq.heappop(queue)
        current = (column, row)
        if current in closed or cost > best_cost.get(current, math.inf) + 1e-12:
            continue
        if current == goal:
            return _reconstruct_path(parent, goal)
        closed.add(current)

        for delta_column, delta_row, move_cost in neighbours:
            next_column = column + delta_column
            next_row = row + delta_row
            if not (0 <= next_column < grid.width and 0 <= next_row < grid.height):
                continue
            if blocked[grid.index(next_column, next_row)]:
                continue
            if delta_column != 0 and delta_row != 0:
                if (
                    blocked[grid.index(column + delta_column, row)]
                    or blocked[grid.index(column, row + delta_row)]
                ):
                    continue
            neighbour = (next_column, next_row)
            candidate_cost = cost + move_cost
            if candidate_cost + 1e-12 >= best_cost.get(neighbour, math.inf):
                continue
            best_cost[neighbour] = candidate_cost
            parent[neighbour] = current
            heuristic = _octile_distance(neighbour, goal)
            heapq.heappush(
                queue,
                (
                    candidate_cost + heuristic,
                    heuristic,
                    candidate_cost,
                    next_row,
                    next_column,
                ),
            )
    return []


def parse_args():
    parser = argparse.ArgumentParser(
        description="Probe conservative M3 route connectivity in a mapping bag."
    )
    parser.add_argument("bag", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--plot", type=Path)
    parser.add_argument("--goal-x", type=float, default=0.0)
    parser.add_argument("--goal-y", type=float, default=3.0)
    parser.add_argument("--blocker-x", type=float, default=0.0)
    parser.add_argument("--blocker-y", type=float, default=1.5)
    parser.add_argument("--blocker-radius", type=float, default=0.35)
    parser.add_argument("--footprint-radius", type=float, default=0.35)
    parser.add_argument("--inflation-radius", type=float, default=0.50)
    parser.add_argument("--minimum-altitude", type=float, default=2.3)
    return parser.parse_args()


def validate_runtime_mapping_contract(evidence, configuration):
    try:
        configured = configuration["depth_grid_node"]["ros__parameters"]
        expected = {name: configured[name] for name in RUNTIME_MAPPING_PARAMETERS}
    except (KeyError, TypeError) as error:
        raise RuntimeError("mapping configuration omits required parameters") from error

    if not evidence.diagnostics or any(
        getattr(sample, "mapper_parameters", None) is None
        for sample in evidence.diagnostics
    ):
        raise RuntimeError("bag diagnostics omit runtime mapper parameters")

    diagnostic_timestamps = [
        sample.timestamp_ns for sample in evidence.diagnostics
    ]
    grid_timestamps = [grid.timestamp_ns for grid in evidence.grids]
    if (
        len(set(diagnostic_timestamps)) != len(diagnostic_timestamps)
        or set(diagnostic_timestamps) != set(grid_timestamps)
    ):
        raise RuntimeError(
            "runtime mapper diagnostic timestamps do not match recorded grids"
        )

    for sample in evidence.diagnostics:
        actual = sample.mapper_parameters
        try:
            if actual["map_frame"] != str(expected["map_frame"]):
                raise RuntimeError("runtime mapper parameters do not match configuration")
            if actual["base_frame"] != str(expected["base_frame"]):
                raise RuntimeError("runtime mapper parameters do not match configuration")
            if int(actual["pixel_stride"]) != int(expected["pixel_stride"]):
                raise RuntimeError("runtime mapper parameters do not match configuration")
            for name in FLOAT_MAPPING_PARAMETERS:
                runtime_value = float(actual[name])
                configured_value = float(expected[name])
                if (
                    not math.isfinite(runtime_value)
                    or not math.isfinite(configured_value)
                    or not math.isclose(
                        runtime_value,
                        configured_value,
                        rel_tol=RESOLUTION_RELATIVE_TOLERANCE,
                        abs_tol=0.0,
                    )
                ):
                    raise RuntimeError(
                        "runtime mapper parameters do not match configuration"
                    )
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError("invalid runtime mapper parameters") from error

    resolution_m = float(expected["resolution_m"])
    width = math.ceil(float(expected["width_m"]) / resolution_m)
    height = math.ceil(float(expected["height_m"]) / resolution_m)
    if not evidence.grids:
        raise RuntimeError("bag has no occupancy grids")
    for grid in evidence.grids:
        if (
            grid.frame_id != str(expected["map_frame"])
            or not math.isclose(
                float(grid.resolution_m),
                resolution_m,
                rel_tol=1e-6,
                abs_tol=0.0,
            )
            or int(grid.width) != width
            or int(grid.height) != height
        ):
            raise RuntimeError("recorded grid does not match runtime mapper parameters")

    return {
        **expected,
        "tf_timeout_s": float(expected["tf_timeout_s"]),
        "resolution_m": resolution_m,
        "width_m": float(expected["width_m"]),
        "height_m": float(expected["height_m"]),
        "min_depth_m": float(expected["min_depth_m"]),
        "max_depth_m": float(expected["max_depth_m"]),
        "min_relative_height_m": float(expected["min_relative_height_m"]),
        "max_relative_height_m": float(expected["max_relative_height_m"]),
        "pixel_stride": int(expected["pixel_stride"]),
        "diagnostic_sample_count": len(evidence.diagnostics),
    }


def _path_world_points(grid, path):
    return [grid.cell_center(column, row) for column, row in path]


def _path_length(points):
    return sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(points, points[1:])
    )


def _point_segment_distance(point, first, second):
    delta_x = second[0] - first[0]
    delta_y = second[1] - first[1]
    denominator = delta_x * delta_x + delta_y * delta_y
    if denominator <= 0.0:
        return math.hypot(point[0] - first[0], point[1] - first[1])
    fraction = (
        (point[0] - first[0]) * delta_x
        + (point[1] - first[1]) * delta_y
    ) / denominator
    fraction = max(0.0, min(1.0, fraction))
    projected = (first[0] + fraction * delta_x, first[1] + fraction * delta_y)
    return math.hypot(point[0] - projected[0], point[1] - projected[1])


def _minimum_polyline_clearance(points, center, radius_m):
    if len(points) == 1:
        return math.hypot(points[0][0] - center[0], points[0][1] - center[1]) - radius_m
    return min(
        _point_segment_distance(center, first, second) - radius_m
        for first, second in zip(points, points[1:])
    )


def _maximum_direct_path_deviation(points, start, goal):
    return max(_point_segment_distance(point, start, goal) for point in points)


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _provenance(bag_path, config_path):
    bag_files = sorted(path for path in bag_path.iterdir() if path.is_file())
    return {
        "bag_files_sha256": {
            path.name: _sha256(path) for path in bag_files
            if path.suffix in (".db3", ".yaml")
        },
        "config_path": str(config_path),
        "config_sha256": _sha256(config_path),
        "probe_sha256": _sha256(Path(__file__).resolve()),
    }


def evaluate_connectivity(
    evidence,
    goal,
    blocker,
    blocker_radius_m,
    footprint_radius_m,
    inflation_radius_m,
    minimum_altitude_m,
):
    grids = sorted(evidence.grids, key=lambda grid: grid.timestamp_ns)
    poses = sorted(evidence.poses, key=lambda pose: pose.timestamp_ns)
    if not grids or not poses:
        raise RuntimeError("bag must contain occupancy grids and map-frame poses")
    fused = FusedGrid.from_sample(grids[0])
    pose_times = [pose.timestamp_ns for pose in poses]
    first_connected = None
    connected_grid = None
    connected_path = None
    aligned_poses = [
        m2.nearest_pose(poses, pose_times, grid.timestamp_ns)
        for grid in grids
    ]
    maximum_pose_offset_ns = max(offset for _pose, offset in aligned_poses)

    for grid_sample, (pose, pose_offset_ns) in zip(grids, aligned_poses):
        fused.integrate(grid_sample)
        if pose.z < minimum_altitude_m:
            continue
        candidate = fused.copy()
        candidate.clear_disk(pose.x, pose.y, footprint_radius_m)
        candidate.add_disk_obstacle(blocker[0], blocker[1], blocker_radius_m)
        path = find_path(
            candidate,
            (pose.x, pose.y),
            goal,
            inflation_radius_m,
        )
        if not path:
            continue
        world_points = _path_world_points(candidate, path)
        first_connected = {
            "map_timestamp_ns": grid_sample.timestamp_ns,
            "pose_offset_ms": pose_offset_ns / 1_000_000.0,
            "vehicle_position_m": [pose.x, pose.y, pose.z],
            "path_cell_count": len(path),
            "path_length_m": _path_length(world_points),
            "maximum_direct_path_deviation_m": _maximum_direct_path_deviation(
                world_points, (pose.x, pose.y), goal
            ),
            "minimum_blocker_clearance_m": _minimum_polyline_clearance(
                world_points, blocker, blocker_radius_m
            ),
            "path_world_xy": [[x, y] for x, y in world_points],
        }
        connected_grid = candidate
        connected_path = path
        break

    known_free = sum(value == FREE for value in fused.cells)
    occupied = sum(value == OCCUPIED for value in fused.cells)
    result = {
        "verdict": "accepted" if first_connected else "rejected",
        "policy": {
            "unknown_is_traversable": False,
            "unknown_is_inflated": False,
            "occupied_applies_to_every_positive_overlap": True,
            "free_requires_complete_target_cell_coverage": True,
            "footprint_clears_only_fully_contained_cells": True,
            "footprint_radius_m": footprint_radius_m,
            "inflation_radius_m": inflation_radius_m,
            "blocker_radius_m": blocker_radius_m,
        },
        "goal_xy_m": list(goal),
        "blocker_xy_m": list(blocker),
        "input_grid_count": len(grids),
        "pose_alignment": {
            "maximum_offset_ms": maximum_pose_offset_ns / 1_000_000.0,
            "required_maximum_offset_ms": m2.MAX_TF_OFFSET_NS / 1_000_000.0,
        },
        "fused_grid": {
            "resolution_m": fused.resolution_m,
            "width": fused.width,
            "height": fused.height,
            "origin_xy_m": [fused.origin_x, fused.origin_y],
            "known_free_cell_count": known_free,
            "occupied_cell_count": occupied,
            "unknown_cell_count": len(fused.cells) - known_free - occupied,
        },
        "first_connected": first_connected,
    }
    return result, connected_grid, connected_path


def _plot_result(grid, path, goal, blocker, blocker_radius_m, inflation_radius_m, output):
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    from matplotlib.patches import Circle

    blocked = blocked_mask(grid, inflation_radius_m)
    image = []
    for row in range(grid.height):
        image_row = []
        for column in range(grid.width):
            index = grid.index(column, row)
            if grid.cells[index] == UNKNOWN:
                image_row.append(0.35)
            elif grid.cells[index] == OCCUPIED:
                image_row.append(1.0)
            elif blocked[index]:
                image_row.append(0.75)
            else:
                image_row.append(0.0)
        image.append(image_row)

    figure = Figure(figsize=(8, 8), constrained_layout=True)
    FigureCanvasAgg(figure)
    axis = figure.add_subplot(1, 1, 1)
    extent = (
        grid.origin_x,
        grid.origin_x + grid.width * grid.resolution_m,
        grid.origin_y,
        grid.origin_y + grid.height * grid.resolution_m,
    )
    axis.imshow(image, origin="lower", extent=extent, cmap="Reds", vmin=0.0, vmax=1.0)
    points = _path_world_points(grid, path)
    axis.plot([point[0] for point in points], [point[1] for point in points], "b-", linewidth=2)
    axis.plot(points[0][0], points[0][1], "bo", label="first connected start")
    axis.plot(goal[0], goal[1], "g*", markersize=12, label="goal")
    axis.add_patch(Circle(blocker, blocker_radius_m, fill=False, color="black", linewidth=2))
    axis.set_title("M3 conservative known-free connectivity")
    axis.set_xlabel("map x / east (m)")
    axis.set_ylabel("map y / north (m)")
    axis.set_aspect("equal")
    axis.legend(loc="best")
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=160)


def main():
    args = parse_args()
    configuration = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    stride = configuration["depth_grid_node"]["ros__parameters"]["pixel_stride"]
    if stride != 1:
        raise RuntimeError("connectivity evidence requires pixel_stride: 1")

    evidence = m2.read_bag(args.bag)
    runtime_mapping_contract = validate_runtime_mapping_contract(
        evidence, configuration
    )
    result, connected_grid, connected_path = evaluate_connectivity(
        evidence,
        (args.goal_x, args.goal_y),
        (args.blocker_x, args.blocker_y),
        args.blocker_radius,
        args.footprint_radius,
        args.inflation_radius,
        args.minimum_altitude,
    )
    result["runtime_mapping_contract"] = runtime_mapping_contract
    result["provenance"] = _provenance(args.bag, args.config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary_output = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary_output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary_output.replace(args.output)

    if args.plot and connected_grid is not None and connected_path is not None:
        _plot_result(
            connected_grid,
            connected_path,
            (args.goal_x, args.goal_y),
            (args.blocker_x, args.blocker_y),
            args.blocker_radius,
            args.inflation_radius,
            args.plot,
        )
    if result["verdict"] != "accepted":
        raise RuntimeError("no conservative inflated route became connected")
    connected = result["first_connected"]
    print(
        "M3 connectivity accepted: "
        f"{connected['path_cell_count']} cells, "
        f"{connected['minimum_blocker_clearance_m']:.3f} m blocker clearance"
    )
    print(f"wrote {args.output}")
    if args.plot:
        print(f"wrote {args.plot}")


if __name__ == "__main__":
    main()
