from importlib.util import module_from_spec, spec_from_file_location
import math
from pathlib import Path

import pytest


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
        tf_links={
            ("map", "base_link"),
            ("base_link", "camera_optical_frame"),
        },
        child_frames={"base_link", "camera_optical_frame"},
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
            lambda _module, evidence: evidence.tf_links.remove(
                ("base_link", "camera_optical_frame")
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
