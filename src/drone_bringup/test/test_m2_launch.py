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
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, OpaqueFunction
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[3]
LAUNCH_PATH = (
    PROJECT_ROOT
    / "src"
    / "drone_bringup"
    / "launch"
    / "local_mapping.launch.py"
)
PX4_WRAPPER_PATH = PROJECT_ROOT / "scripts" / "run-px4-sitl.sh"
PX4_HEADLESS_RCS_PATH = PROJECT_ROOT / "scripts" / "px4-headless-rcS"


def load_module(name, path):
    spec = spec_from_file_location(name, path)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def string_literals(path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }


def test_launch_owns_m2_processes_topics_and_local_evidence():
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    literals = string_literals(LAUNCH_PATH)

    required_literals = {
        "inspection.sdf",
        "MicroXRCEAgent",
        "run-px4-sitl.sh",
        "px4-headless-rcS",
        "PX4_GZ_STANDALONE",
        "PX4_GZ_WORLD",
        "PX4_SIM_MODEL",
        "PX4_SYS_AUTOSTART",
        "PX4_GZ_MODELS",
        "4001",
        "gz_x500_depth_project",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        "/camera/depth/camera_info"
        "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "px4_tf_broadcaster",
        "depth_grid_node",
        "waypoint_controller",
        "local_mapping.rviz",
        "/local_occupancy_grid",
        "/drone_perception/diagnostics",
        "/tf",
        "/tf_static",
    }
    assert required_literals <= literals
    assert '"log" / "m2"' in source
    assert "mapping_%Y%m%d_%H%M%S" in source
    assert 'DeclareLaunchArgument(\n                "use_rviz"' in source
    assert 'DeclareLaunchArgument(\n                "run_mission"' in source
    assert 'DeclareLaunchArgument(\n                "record_bag"' in source

    for action_name in (
        "gazebo_server",
        "dds_agent",
        "px4_sitl",
        "bridge",
        "tf_broadcaster",
        "mapper",
        "rviz",
        "bag_recorder",
    ):
        assert f"target_action={action_name}" in source


def test_headless_px4_override_runs_after_airframe_configuration():
    wrapper = PX4_WRAPPER_PATH.read_text(encoding="utf-8")
    startup = PX4_HEADLESS_RCS_PATH.read_text(encoding="utf-8")

    assert "px4-headless-rcS" in LAUNCH_PATH.read_text(encoding="utf-8")
    assert 'px4_startup_script="${2:-}"' in wrapper
    assert '"${px4_binary}" -d -s "${px4_startup_script}"' in wrapper

    original_startup = ". etc/init.d-posix/rcS"
    headless_override = "param set-default NAV_DLL_ACT 0"
    assert original_startup in startup
    assert headless_override in startup
    assert startup.index(original_startup) < startup.index(headless_override)


def test_m2_configs_define_valid_grid_safe_route_and_rviz_view():
    share = Path(get_package_share_directory("drone_bringup"))
    mapping_config = share / "config" / "local_mapping.yaml"
    mission_config = share / "config" / "mapping_mission.yaml"
    rviz_config = share / "config" / "local_mapping.rviz"

    mapping = yaml.safe_load(mapping_config.read_text(encoding="utf-8"))
    mapper = mapping["depth_grid_node"]["ros__parameters"]
    tf_parameters = mapping["px4_tf_broadcaster"]["ros__parameters"]
    assert mapper["map_frame"] == "map"
    assert mapper["base_frame"] == "base_link"
    assert mapper["resolution_m"] > 0.0
    assert mapper["width_m"] >= 10.0
    assert mapper["height_m"] >= 10.0
    assert 0.0 < mapper["min_depth_m"] < mapper["max_depth_m"]
    assert mapper["min_relative_height_m"] < 0.0
    assert mapper["max_relative_height_m"] > 0.0
    assert mapper["pixel_stride"] >= 1
    assert tf_parameters["map_frame"] == "map"
    assert tf_parameters["base_frame"] == "base_link"
    assert tf_parameters["camera_frame"] == "camera_optical_frame"

    mission = yaml.safe_load(mission_config.read_text(encoding="utf-8"))
    controller = mission["waypoint_controller"]["ros__parameters"]
    offsets = controller["waypoint_offsets_xy"]
    assert 2.0 <= controller["takeoff_altitude_m"] <= 2.5
    assert len(offsets) >= 4 and len(offsets) % 2 == 0
    route = list(zip(offsets[::2], offsets[1::2]))
    assert max(math.hypot(x, y) for x, y in route) >= 1.0
    for x, y in route:
        assert 0.5 <= x <= 2.0
        assert -1.0 <= y <= 1.0
        assert math.hypot(x - 3.0, y - 2.0) > 0.8
        assert math.hypot(x - 3.0, y + 2.0) > 0.8

    rviz = rviz_config.read_text(encoding="utf-8")
    assert "Fixed Frame: map" in rviz
    assert "rviz_default_plugins/Map" in rviz
    assert "Value: /local_occupancy_grid" in rviz
    assert "rviz_default_plugins/TF" in rviz


def test_package_metadata_declares_m2_runtime_dependencies():
    share = Path(get_package_share_directory("drone_bringup"))
    package_root = ET.parse(share / "package.xml").getroot()
    dependencies = {
        element.text
        for element in package_root.findall("exec_depend")
    }
    assert {
        "drone_controller",
        "drone_perception",
        "drone_sim",
        "ros_gz_bridge",
        "ros_gz_sim",
        "rosbag2_transport",
        "rviz2",
    } <= dependencies


def test_mission_and_required_process_exits_propagate():
    module = load_module("local_mapping_launch", LAUNCH_PATH)
    active_context = SimpleNamespace(is_shutdown=False)
    shutdown_context = SimpleNamespace(is_shutdown=True)

    success_actions = module.mission_exit_actions(
        SimpleNamespace(returncode=0), active_context
    )
    failure_actions = module.mission_exit_actions(
        SimpleNamespace(returncode=9), active_context
    )
    assert isinstance(success_actions[0], EmitEvent)
    assert isinstance(failure_actions[0], OpaqueFunction)
    assert module.mission_exit_actions(
        SimpleNamespace(returncode=-2), shutdown_context
    ) == []

    required_handler = module.required_process_exit_actions("Gazebo server")
    assert isinstance(
        required_handler(
            SimpleNamespace(returncode=3), active_context
        )[0],
        OpaqueFunction,
    )
    assert required_handler(
        SimpleNamespace(returncode=-2), shutdown_context
    ) == []
