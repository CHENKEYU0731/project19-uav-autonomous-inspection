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

from pathlib import Path
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[3]
SIM_PACKAGE = PROJECT_ROOT / "src" / "drone_sim"
MODEL_SDF = SIM_PACKAGE / "models" / "x500_depth_project" / "model.sdf"
WORLD_SDF = SIM_PACKAGE / "worlds" / "inspection.sdf"


def required_text(element, path):
    child = element.find(path)
    assert child is not None, f"missing XML element: {path}"
    assert child.text is not None
    return child.text.strip()


def test_depth_camera_contract_is_explicit_and_lightweight():
    root = ET.parse(MODEL_SDF).getroot()
    model = root.find("model")
    assert model is not None
    assert model.attrib["name"] == "x500_depth_project"
    assert required_text(model, "include/uri") == "x500"

    sensor = model.find(".//sensor[@type='depth_camera']")
    assert sensor is not None
    assert required_text(sensor, "gz_frame_id") == "camera_optical_frame"
    assert required_text(sensor, "topic") == "/camera/depth/image_raw"
    assert required_text(sensor, "update_rate") == "10"
    assert required_text(sensor, "camera/camera_info_topic") == (
        "/camera/depth/camera_info"
    )
    assert required_text(sensor, "camera/image/width") == "160"
    assert required_text(sensor, "camera/image/height") == "120"
    assert required_text(sensor, "camera/image/format") == "R_FLOAT32"
    assert float(required_text(sensor, "camera/clip/near")) == 0.2
    assert float(required_text(sensor, "camera/clip/far")) == 12.0


def test_inspection_world_has_collision_matched_obstacles():
    root = ET.parse(WORLD_SDF).getroot()
    world = root.find("world")
    assert world is not None
    assert world.attrib["name"] == "inspection"

    plugin_filenames = {
        plugin.attrib["filename"] for plugin in world.findall("plugin")
    }
    assert {
        "gz-sim-physics-system",
        "gz-sim-user-commands-system",
        "gz-sim-scene-broadcaster-system",
        "gz-sim-contact-system",
        "gz-sim-imu-system",
        "gz-sim-air-pressure-system",
        "gz-sim-air-speed-system",
        "gz-sim-apply-link-wrench-system",
        "gz-sim-navsat-system",
        "gz-sim-magnetometer-system",
        "gz-sim-sensors-system",
    } <= plugin_filenames

    required_obstacles = {
        "left_wall",
        "right_wall",
        "door_left",
        "door_right",
        "door_top",
        "column_left",
        "column_right",
    }
    models = {model.attrib["name"]: model for model in world.findall("model")}
    assert required_obstacles <= models.keys()

    for name in required_obstacles:
        link = models[name].find("link")
        assert link is not None, f"{name} has no link"
        collision = link.find("collision/geometry")
        visual = link.find("visual/geometry")
        assert collision is not None, f"{name} has no collision geometry"
        assert visual is not None, f"{name} has no visual geometry"
        assert ET.tostring(collision) == ET.tostring(visual)


def test_sim_package_installs_models_and_worlds():
    cmake = (SIM_PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "DIRECTORY models worlds" in cmake
    assert "DESTINATION share/${PROJECT_NAME}" in cmake


def test_px4_wrapper_allows_model_override():
    wrapper = (PROJECT_ROOT / "scripts" / "run-px4-sitl.sh").read_text(
        encoding="utf-8"
    )
    assert 'PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500}"' in wrapper
    assert 'GZ_IP="${GZ_IP:-127.0.0.1}"' in wrapper
