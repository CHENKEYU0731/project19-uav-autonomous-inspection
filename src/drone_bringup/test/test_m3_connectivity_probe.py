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

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[3]
PROBE_PATH = PROJECT_ROOT / "scripts" / "probe_m3_connectivity.py"


def load_probe():
    spec = spec_from_file_location("probe_m3_connectivity", PROBE_PATH)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sample(
    width,
    height,
    cells,
    origin_x=0.0,
    origin_y=0.0,
    resolution_m=1.0,
    frame_id="map",
    timestamp_ns=1,
):
    return SimpleNamespace(
        resolution_m=resolution_m,
        width=width,
        height=height,
        origin_x=origin_x,
        origin_y=origin_y,
        cells=tuple(cells),
        frame_id=frame_id,
        timestamp_ns=timestamp_ns,
    )


def test_fusion_preserves_unknown_and_accepts_new_known_evidence():
    module = load_probe()
    initial = sample(3, 1, [module.FREE, module.OCCUPIED, module.UNKNOWN])
    fused = module.FusedGrid.from_sample(initial)

    fused.integrate(sample(3, 1, [module.UNKNOWN, module.FREE, module.OCCUPIED]))

    assert fused.cells == [module.FREE, module.FREE, module.OCCUPIED]


def test_fusion_maps_occupied_to_all_overlaps_and_requires_full_free_coverage():
    module = load_probe()
    occupied = module.FusedGrid.from_sample(
        sample(2, 1, [module.FREE, module.FREE])
    )
    occupied.integrate(
        sample(1, 1, [module.OCCUPIED], origin_x=0.5)
    )
    assert occupied.cells == [module.OCCUPIED, module.OCCUPIED]

    partial_free = module.FusedGrid.from_sample(
        sample(2, 1, [module.OCCUPIED, module.OCCUPIED])
    )
    partial_free.integrate(sample(1, 1, [module.FREE], origin_x=0.5))
    assert partial_free.cells == [module.OCCUPIED, module.OCCUPIED]

    complete_free = module.FusedGrid.from_sample(
        sample(3, 1, [module.OCCUPIED] * 3)
    )
    complete_free.integrate(
        sample(2, 1, [module.FREE, module.FREE], origin_x=0.5)
    )
    assert complete_free.cells == [module.OCCUPIED, module.FREE, module.OCCUPIED]


def test_current_footprint_clearing_is_bounded_to_the_requested_disk():
    module = load_probe()
    fused = module.FusedGrid.from_sample(sample(5, 5, [module.OCCUPIED] * 25))

    fused.clear_disk(2.5, 2.5, 1.0)

    assert fused.cells[2 * 5 + 2] == module.FREE
    assert fused.cells[0] == module.OCCUPIED


def test_current_footprint_clearing_preserves_partial_cell_overlap():
    module = load_probe()
    fused = module.FusedGrid.from_sample(
        sample(3, 1, [module.OCCUPIED] * 3)
    )

    fused.clear_disk(0.5, 0.5, 0.75)

    assert fused.cells[fused.index(0, 0)] == module.FREE
    assert fused.cells[fused.index(1, 0)] == module.OCCUPIED


def mapper_parameters(pixel_stride=1):
    return {
        "map_frame": "map",
        "base_frame": "base_link",
        "tf_timeout_s": "0.1",
        "resolution_m": "0.1",
        "width_m": "12.0",
        "height_m": "12.0",
        "min_depth_m": "0.2",
        "max_depth_m": "10.0",
        "min_relative_height_m": "-0.5",
        "max_relative_height_m": "0.5",
        "pixel_stride": str(pixel_stride),
    }


def test_runtime_mapping_contract_must_be_present_and_match_configuration():
    module = load_probe()
    configuration = {
        "depth_grid_node": {
            "ros__parameters": {
                "map_frame": "map",
                "base_frame": "base_link",
                "tf_timeout_s": 0.1,
                "resolution_m": 0.1,
                "width_m": 12.0,
                "height_m": 12.0,
                "min_depth_m": 0.2,
                "max_depth_m": 10.0,
                "min_relative_height_m": -0.5,
                "max_relative_height_m": 0.5,
                "pixel_stride": 1,
            }
        }
    }
    grid = sample(
        120,
        120,
        [module.UNKNOWN] * (120 * 120),
        resolution_m=0.1,
    )
    evidence = SimpleNamespace(
        grids=[grid],
        diagnostics=[
            SimpleNamespace(
                timestamp_ns=grid.timestamp_ns,
                mapper_parameters=mapper_parameters(),
            )
        ],
    )

    contract = module.validate_runtime_mapping_contract(
        evidence, configuration
    )
    assert contract["pixel_stride"] == 1
    assert contract["diagnostic_sample_count"] == 1

    evidence.diagnostics[0].mapper_parameters = None
    with pytest.raises(RuntimeError, match="runtime mapper parameters"):
        module.validate_runtime_mapping_contract(evidence, configuration)

    evidence.diagnostics[0].mapper_parameters = mapper_parameters(2)
    with pytest.raises(RuntimeError, match="do not match"):
        module.validate_runtime_mapping_contract(evidence, configuration)

    evidence.diagnostics[0].mapper_parameters = mapper_parameters()
    evidence.diagnostics[0].timestamp_ns += 1
    with pytest.raises(RuntimeError, match="timestamps"):
        module.validate_runtime_mapping_contract(evidence, configuration)


def test_unknown_cells_are_blocked_but_do_not_inflate_into_known_free_space():
    module = load_probe()
    cells = [module.UNKNOWN] * 15
    cells[5:10] = [module.FREE] * 5
    fused = module.FusedGrid.from_sample(sample(5, 3, cells))

    path = module.find_path(fused, (0.5, 1.5), (4.5, 1.5), 1.0)

    assert path
    assert all(row == 1 for _column, row in path)


def test_astar_rejects_diagonal_corner_cutting():
    module = load_probe()
    fused = module.FusedGrid.from_sample(
        sample(
            2,
            2,
            [
                module.FREE,
                module.OCCUPIED,
                module.OCCUPIED,
                module.FREE,
            ],
        )
    )

    assert module.find_path(fused, (0.5, 0.5), (1.5, 1.5), 0.0) == []


def test_disk_obstacle_is_inflated_and_route_is_deterministic():
    module = load_probe()
    fused = module.FusedGrid.from_sample(sample(9, 7, [module.FREE] * 63))
    fused.add_disk_obstacle(4.5, 3.5, 0.4)

    first = module.find_path(fused, (0.5, 3.5), (8.5, 3.5), 1.0)
    second = module.find_path(fused, (0.5, 3.5), (8.5, 3.5), 1.0)

    assert first == second
    assert first
    assert all(fused.cells[row * fused.width + column] != module.OCCUPIED
               for column, row in first)
    assert any(row != 3 for _column, row in first)
