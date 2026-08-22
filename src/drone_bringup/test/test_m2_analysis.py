from importlib.util import module_from_spec, spec_from_file_location
import math
from pathlib import Path

import pytest
from PIL import Image


PROJECT_ROOT = Path(__file__).resolve().parents[3]
ANALYZER_PATH = PROJECT_ROOT / "scripts" / "analyze_m2_mapping.py"


def load_module(name, path):
    spec = spec_from_file_location(name, path)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_grid(module, timestamp_ns, base_x, base_y, obstacle=(5.0, 0.0)):
    resolution = 0.1
    width = 120
    height = 120
    origin_x = base_x - width * resolution / 2.0
    origin_y = base_y - height * resolution / 2.0
    cells = [-1] * (width * height)
    obstacle_x, obstacle_y = obstacle
    column = int((obstacle_x - origin_x) / resolution)
    row = int((obstacle_y - origin_y) / resolution)
    if 0 <= column < width and 0 <= row < height:
        cells[row * width + column] = 100
    return module.GridSample(
        timestamp_ns=timestamp_ns,
        frame_id="map",
        resolution_m=resolution,
        width=width,
        height=height,
        origin_x=origin_x,
        origin_y=origin_y,
        cells=tuple(cells),
    )


def complete_evidence(module):
    poses = []
    grids = []
    diagnostics = []
    for index in range(31):
        timestamp_ns = index * 100_000_000
        if index < 10:
            x = 0.0
        elif index < 20:
            x = (index - 9) * 0.2
        else:
            x = 2.0
        poses.append(
            module.PoseSample(timestamp_ns, x, 0.0, 2.5)
        )
        grids.append(make_grid(module, timestamp_ns, x, 0.0))
        diagnostics.append(
            module.DiagnosticSample(
                timestamp_ns=timestamp_ns,
                processing_latency_ms=1.5,
                output_rate_hz=10.0,
                used_depth_count=100,
                occupied_cell_count=1,
            )
        )
    return module.MappingEvidence(
        grids=grids,
        poses=poses,
        diagnostics=diagnostics,
        depth_samples=[
            module.DepthSample(
                grid.timestamp_ns,
                "camera_optical_frame",
                "32FC1",
                160,
                120,
                640,
                76800,
            )
            for grid in grids
        ],
        camera_info_samples=[
            module.CameraInfoSample(
                0,
                "camera_optical_frame",
                160,
                120,
                100.0,
                100.0,
                80.0,
                60.0,
            )
        ],
        transforms=[
            *[
                module.TransformSample(
                    pose.timestamp_ns,
                    "map",
                    "base_link",
                    False,
                    pose.x,
                    pose.y,
                    pose.z,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                )
                for pose in poses
            ],
            module.TransformSample(
                0,
                "base_link",
                "camera_optical_frame",
                True,
                0.13233,
                0.0,
                0.26078,
                -0.5,
                0.5,
                -0.5,
                0.5,
            ),
        ],
        vehicle_status_samples=[
            module.VehicleStatusSample(0, 1, 4, False),
            module.VehicleStatusSample(500_000_000, 2, 14, False),
            module.VehicleStatusSample(3_200_000_000, 1, 14, False),
        ],
        land_samples=[
            module.LandSample(0, True),
            module.LandSample(600_000_000, False),
            module.LandSample(3_100_000_000, True),
        ],
    )


def test_complete_mapping_evidence_passes_all_gates():
    module = load_module("analyze_m2_mapping", ANALYZER_PATH)
    evidence = complete_evidence(module)

    metrics = module.validate_evidence(evidence)

    assert metrics["coordinate_frames"]["map_is_fixed"] is True
    assert metrics["motion"]["planar_displacement_m"] >= 2.0
    assert metrics["rolling_grid"]["max_center_error_m"] < 1e-9
    assert metrics["mapping_rate"]["median_hz"] == pytest.approx(10.0)
    assert metrics["hover_obstacle_alignment"]["aligned_cell_count"] >= 1
    assert metrics["motion_obstacle_alignment"]["aligned_cell_count"] >= 1


@pytest.mark.parametrize(
    "mutation, expected_message",
    [
        (
            lambda module, evidence: setattr(
                evidence,
                "poses",
                [module.PoseSample(p.timestamp_ns, 0.0, 0.0, p.z)
                 for p in evidence.poses],
            ),
            "at least 1.0 m",
        ),
        (
            lambda _module, evidence: setattr(
                evidence,
                "transforms",
                [
                    transform
                    for transform in evidence.transforms
                    if transform.child != "camera_optical_frame"
                ],
            ),
            "required TF link",
        ),
        (
            lambda module, evidence: evidence.grids.__setitem__(
                5,
                module.GridSample(
                    **{
                        **evidence.grids[5].__dict__,
                        "origin_x": evidence.grids[5].origin_x + 1.0,
                    }
                ),
            ),
            "grid center",
        ),
        (
            lambda module, evidence: setattr(
                evidence,
                "grids",
                [
                    make_grid(
                        module,
                        grid.timestamp_ns,
                        pose.x,
                        pose.y,
                        obstacle=(1.1, 1.1),
                    )
                    for grid, pose in zip(evidence.grids, evidence.poses)
                ],
            ),
            "known obstacle",
        ),
        (
            lambda module, evidence: setattr(
                evidence,
                "diagnostics",
                [
                    module.DiagnosticSample(
                        diagnostic.timestamp_ns,
                        math.nan,
                        diagnostic.output_rate_hz,
                        diagnostic.used_depth_count,
                        diagnostic.occupied_cell_count,
                    )
                    for diagnostic in evidence.diagnostics
                ],
            ),
            "non-finite latency",
        ),
    ],
)
def test_incomplete_or_invalid_evidence_is_rejected(
    mutation, expected_message
):
    module = load_module("analyze_m2_mapping", ANALYZER_PATH)
    evidence = complete_evidence(module)
    mutation(module, evidence)

    with pytest.raises(RuntimeError, match=expected_message):
        module.validate_evidence(evidence)


def test_low_output_rate_is_rejected():
    module = load_module("analyze_m2_mapping", ANALYZER_PATH)
    evidence = complete_evidence(module)
    evidence.grids = [
        module.GridSample(
            **{
                **grid.__dict__,
                "timestamp_ns": index * 300_000_000,
            }
        )
        for index, grid in enumerate(evidence.grids)
    ]

    with pytest.raises(RuntimeError, match="at least 5 Hz"):
        module.validate_evidence(evidence)


def test_mixed_intervals_below_five_hz_are_rejected():
    module = load_module("analyze_m2_mapping", ANALYZER_PATH)
    evidence = complete_evidence(module)
    intervals_ns = [100_000_000] * 16 + [490_000_000] * 14
    timestamps = [0]
    for interval in intervals_ns:
        timestamps.append(timestamps[-1] + interval)
    evidence.grids = [
        module.GridSample(
            **{**grid.__dict__, "timestamp_ns": timestamps[index]}
        )
        for index, grid in enumerate(evidence.grids)
    ]
    evidence.depth_samples = [
        module.DepthSample(
            **{**sample.__dict__, "timestamp_ns": timestamps[index]}
        )
        for index, sample in enumerate(evidence.depth_samples)
    ]
    evidence.diagnostics = [
        module.DiagnosticSample(
            **{**sample.__dict__, "timestamp_ns": timestamps[index]}
        )
        for index, sample in enumerate(evidence.diagnostics)
    ]

    with pytest.raises(RuntimeError, match="average rate"):
        module.validate_evidence(evidence)


@pytest.mark.parametrize(
    "mutation, expected_message",
    [
        (
            lambda _module, evidence: setattr(
                evidence, "depth_samples", []
            ),
            "depth",
        ),
        (
            lambda _module, evidence: setattr(
                evidence, "vehicle_status_samples", []
            ),
            "flight sequence",
        ),
        (
            lambda module, evidence: setattr(
                evidence,
                "transforms",
                [
                    module.TransformSample(
                        **{
                            **transform.__dict__,
                            "is_static": False,
                        }
                    )
                    if transform.child == "camera_optical_frame"
                    else transform
                    for transform in evidence.transforms
                ],
            ),
            "static camera transform",
        ),
        (
            lambda module, evidence: setattr(
                evidence,
                "transforms",
                [
                    module.TransformSample(
                        **{**transform.__dict__, "tx": 0.5}
                    )
                    if transform.child == "camera_optical_frame"
                    else transform
                    for transform in evidence.transforms
                ],
            ),
            "camera transform",
        ),
        (
            lambda module, evidence: setattr(
                evidence,
                "poses",
                [
                    module.PoseSample(
                        pose.timestamp_ns + 300_000_000,
                        pose.x,
                        pose.y,
                        pose.z,
                    )
                    for pose in evidence.poses
                ],
            ),
            "within 250 ms",
        ),
        (
            lambda module, evidence: setattr(
                evidence,
                "grids",
                [
                    module.GridSample(
                        **{
                            **grid.__dict__,
                            "cells": (-1,) * (grid.width * grid.height),
                        }
                    )
                    for grid in evidence.grids
                ],
            ),
            "empty occupied grid",
        ),
        (
            lambda module, evidence: evidence.diagnostics.__setitem__(
                5,
                module.DiagnosticSample(
                    **{
                        **evidence.diagnostics[5].__dict__,
                        "occupied_cell_count": 9,
                    }
                ),
            ),
            "diagnostic",
        ),
    ],
)
def test_false_acceptance_paths_are_rejected(mutation, expected_message):
    module = load_module("analyze_m2_mapping", ANALYZER_PATH)
    evidence = complete_evidence(module)
    mutation(module, evidence)

    with pytest.raises(RuntimeError, match=expected_message):
        module.validate_evidence(evidence)


def test_non_contiguous_flyby_cannot_count_as_hover():
    module = load_module("analyze_m2_mapping", ANALYZER_PATH)
    evidence = complete_evidence(module)
    evidence.poses = [
        module.PoseSample(
            pose.timestamp_ns,
            0.0 if index % 10 == 0 else 2.0,
            pose.y,
            pose.z,
        )
        for index, pose in enumerate(evidence.poses)
    ]
    evidence.grids = [
        make_grid(
            module,
            grid.timestamp_ns,
            evidence.poses[index].x,
            evidence.poses[index].y,
        )
        for index, grid in enumerate(evidence.grids)
    ]

    with pytest.raises(RuntimeError, match="continuous hover"):
        module.validate_evidence(evidence)


def test_low_altitude_slow_climb_cannot_count_as_hover():
    module = load_module("analyze_m2_mapping_hover_altitude", ANALYZER_PATH)
    paired = []
    for index in range(25):
        timestamp_ns = index * 100_000_000
        if index < 12:
            x = 0.0
            z = 2.3 + index * 0.01
        else:
            x = 2.0
            z = 2.5
        paired.append(
            (
                make_grid(module, timestamp_ns, x, 0.0),
                module.PoseSample(timestamp_ns, x, 0.0, z),
            )
        )

    (
        _window,
        reference,
        duration_s,
        average_speed_m_s,
        vertical_range_m,
    ) = module.find_continuous_hover_window(paired)

    assert reference.x == 2.0
    assert reference.z >= module.MIN_HOVER_ALTITUDE_M
    assert duration_s >= module.MIN_HOVER_DURATION_S
    assert average_speed_m_s <= module.MAX_HOVER_AVERAGE_SPEED_M_S
    assert vertical_range_m <= module.MAX_HOVER_VERTICAL_RANGE_M


def test_animation_renders_multiple_frames(tmp_path):
    module = load_module("analyze_m2_mapping_animation", ANALYZER_PATH)
    evidence = complete_evidence(module)
    evidence.grids = evidence.grids[::15]
    output_path = tmp_path / "mapping.gif"

    module.animate_evidence(evidence, output_path, max_frames=3)

    with Image.open(output_path) as animation:
        assert animation.format == "GIF"
        assert animation.n_frames == 3
        assert animation.width > 0
        assert animation.height > 0


def test_provenance_hashes_bag_and_runtime_sources(tmp_path):
    module = load_module("analyze_m2_mapping_provenance", ANALYZER_PATH)
    bag_path = tmp_path / "mapping_bag"
    bag_path.mkdir()
    bag_path.joinpath("metadata.yaml").write_text(
        "version: 5\n", encoding="utf-8"
    )
    bag_path.joinpath("mapping.db3").write_bytes(b"bag evidence")
    metrics = {}

    module.add_provenance(metrics, bag_path)

    provenance = metrics["provenance"]
    assert len(provenance["bag_sha256"]) == 64
    assert set(provenance["runtime_source_sha256"]) == set(
        module.RUNTIME_SOURCE_PATHS
    )
    assert all(
        len(digest) == 64
        for digest in provenance["runtime_source_sha256"].values()
    )
