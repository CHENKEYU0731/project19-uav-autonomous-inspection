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

import ast
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[3]
M2_CONFIG_PATH = (
    PROJECT_ROOT / "src" / "drone_bringup" / "config" / "local_mapping.yaml"
)
M3_CONFIG_PATH = (
    PROJECT_ROOT / "src" / "drone_bringup" / "config" / "m3_mapping.yaml"
)
LAUNCH_PATH = (
    PROJECT_ROOT
    / "src"
    / "drone_bringup"
    / "launch"
    / "local_mapping.launch.py"
)
MAPPER_NODE_PATH = (
    PROJECT_ROOT
    / "src"
    / "drone_perception"
    / "src"
    / "depth_grid_node.cpp"
)


def string_literals(path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }


def test_m3_mapping_uses_stride_one_without_changing_m2_geometry():
    m2 = yaml.safe_load(M2_CONFIG_PATH.read_text(encoding="utf-8"))
    m3 = yaml.safe_load(M3_CONFIG_PATH.read_text(encoding="utf-8"))

    assert m3["px4_tf_broadcaster"] == m2["px4_tf_broadcaster"]
    m2_mapper = m2["depth_grid_node"]["ros__parameters"]
    m3_mapper = m3["depth_grid_node"]["ros__parameters"]
    assert m3_mapper["pixel_stride"] == 1
    assert {
        key: value
        for key, value in m3_mapper.items()
        if key != "pixel_stride"
    } == {
        key: value
        for key, value in m2_mapper.items()
        if key != "pixel_stride"
    }


def test_m2_launch_accepts_mapping_config_override_and_keeps_default():
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    literals = string_literals(LAUNCH_PATH)

    assert "mapping_config" in literals
    assert "local_mapping.yaml" in literals
    assert 'mapping_config = LaunchConfiguration("mapping_config")' in source
    assert "parameters=[mapping_config]" in source
    assert 'DeclareLaunchArgument(\n                "mapping_config"' in source


def test_mapper_diagnostics_record_every_runtime_mapping_parameter():
    source = MAPPER_NODE_PATH.read_text(encoding="utf-8")
    for parameter in (
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
    ):
        assert f'"{parameter}"' in source
    assert source.count("diagnostic_value(") >= 11
