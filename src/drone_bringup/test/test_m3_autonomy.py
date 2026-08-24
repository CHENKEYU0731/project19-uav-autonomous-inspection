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
LAUNCH_PATH = PROJECT_ROOT / "src" / "drone_bringup" / "launch" / "m3_autonomy.launch.py"
PLANNER_CONFIG = PROJECT_ROOT / "src" / "drone_bringup" / "config" / "m3_planner.yaml"
MISSION_CONFIG = PROJECT_ROOT / "src" / "drone_bringup" / "config" / "m3_mission.yaml"
PLANNER_STATUS = PROJECT_ROOT / "src" / "drone_interfaces" / "msg" / "PlannerStatus.msg"
PLANNER_NODE = PROJECT_ROOT / "src" / "drone_planner" / "src" / "planner_node.cpp"
CONTROLLER_NODE = (
    PROJECT_ROOT
    / "src"
    / "drone_controller"
    / "src"
    / "waypoint_controller_node.cpp"
)
BLOCKER_SCRIPT = PROJECT_ROOT / "scripts" / "m3_dynamic_blocker.py"
WORLD_PATH = PROJECT_ROOT / "src" / "drone_sim" / "worlds" / "inspection.sdf"


def string_literals(path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }


def test_m3_configs_define_planner_and_controller_contracts():
    planner = yaml.safe_load(PLANNER_CONFIG.read_text(encoding="utf-8"))
    controller = yaml.safe_load(MISSION_CONFIG.read_text(encoding="utf-8"))
    planner_params = planner["planner_node"]["ros__parameters"]
    controller_params = controller["waypoint_controller"]["ros__parameters"]

    assert planner_params["map_frame"] == controller_params["map_frame"] == "map"
    assert planner_params["base_frame"] == "base_link"
    assert controller_params["use_planned_trajectory"] is True
    assert controller_params["planned_trajectory_topic"] == "/drone_planner/trajectory"
    assert 0.0 < controller_params["planner_stale_timeout_s"] <= 10.0
    assert planner_params["maximum_map_age_s"] <= controller_params["planner_stale_timeout_s"]
    assert planner_params["minimum_altitude_m"] <= (
        controller_params["takeoff_altitude_m"]
        - controller_params["acceptance_radius_m"]
    )


def test_m3_launch_starts_planner_controller_and_records_safety_topics():
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    literals = string_literals(LAUNCH_PATH)
    assert "local_mapping.launch.py" in literals
    assert "m3_mapping.yaml" in literals
    assert "m3_planner.yaml" in literals
    assert "m3_mission.yaml" in literals
    assert 'package="drone_planner"' in source
    assert 'package="drone_controller"' in source
    assert "m3_dynamic_blocker.py" in literals
    assert 'LaunchConfiguration("bag_directory")' in source
    assert "str(bag_directory)" not in source
    assert 'DeclareLaunchArgument(\n                "bag_directory"' in source
    for topic in (
        "/camera/depth/image_raw",
        "/camera/depth/camera_info",
        "/local_occupancy_grid",
        "/drone_perception/diagnostics",
        "/drone_planner/trajectory",
        "/drone_planner/status",
        "/fmu/out/vehicle_odometry",
        "/fmu/in/trajectory_setpoint",
        "/tf",
        "/tf_static",
        "/drone_m3/dynamic_blocker_pose",
        "/drone_m3/dynamic_blocker_event",
        "/drone_m3/insertion_hold",
        "/world/inspection/contacts",
    ):
        assert topic in literals
    assert '"run_mission": "false"' in source


def test_m3_world_and_blocker_script_define_runtime_insertion():
    world = WORLD_PATH.read_text(encoding="utf-8")
    blocker = BLOCKER_SCRIPT.read_text(encoding="utf-8")

    assert '<model name="m3_dynamic_blocker">' in world
    assert "<radius>0.22</radius>" in world
    assert "/world/inspection/set_pose" in blocker
    assert "gz.msgs.Pose" in blocker
    assert "subprocess.run" in blocker
    assert "threading.Thread" in blocker
    assert "_set_pose_with_retry(self.initial_x, self.initial_y)" in blocker
    assert "failed to place the dynamic blocker at its initial pose" in blocker
    assert "blocker_inserted" in blocker
    assert "safe_trajectory_id == self.latest_trajectory_id" in blocker
    assert "self.safe_trajectory_id = 0" in blocker
    assert "status_timeout_s" in blocker
    assert "qos_profile_sensor_data" in blocker
    assert "self._insertion_is_ahead()" in blocker
    assert "_insertion_window_missed" in blocker
    assert "self._publish_hold(True)" in blocker
    assert "self._publish_hold(False)" in blocker
    assert "blocker_replan_confirmed" in blocker
    assert "self.insertion_trajectory_id = self.latest_trajectory_id" in blocker
    assert "self.latest_trajectory_received_at >= self.insertion_confirmed_at" in blocker
    assert "blocker_hold_started" in blocker
    assert "HOLD_SETTLE_SPEED_MPS" in blocker
    assert "HOLD_SETTLE_TIME_S" in blocker
    assert "vehicle did not settle before insertion" in blocker
    assert "trajectory changed while waiting for insertion hold" in blocker


def test_controller_holds_a_fixed_target_during_dynamic_insertion():
    controller = CONTROLLER_NODE.read_text(encoding="utf-8")

    assert '"/drone_m3/insertion_hold"' in controller
    assert "insertion_hold_target_ = current_position_" in controller
    assert "publish_position_setpoint(insertion_hold_target_.value())" in controller
    assert "insertion_hold_target_.reset()" in controller
    assert "dynamic blocker insertion hold timed out" in controller


def test_planner_uses_relative_takeoff_altitude_and_fixed_unsafe_hold():
    planner = PLANNER_NODE.read_text(encoding="utf-8")
    controller = CONTROLLER_NODE.read_text(encoding="utf-8")

    assert "initial_altitude_m_ = current_altitude_m" in planner
    assert (
        "current_altitude_m - initial_altitude_m_.value() "
        "< minimum_altitude_m_"
    ) in planner
    assert "minimum relative altitude" in planner
    assert "planner_hold_target_ = current_position_" in controller
    assert (
        "publish_position_setpoint(planner_hold_target_.value())"
        in controller
    )
    assert controller.count("planner_hold_target_.reset()") >= 2


def test_goal_reached_is_a_valid_planner_controller_contract():
    status = PLANNER_STATUS.read_text(encoding="utf-8")
    planner = PLANNER_NODE.read_text(encoding="utf-8")
    controller = CONTROLLER_NODE.read_text(encoding="utf-8")

    assert "uint8 GOAL_REACHED=7" in status
    assert "parameterize_hold" in planner
    assert "PlannerStatus::GOAL_REACHED" in planner
    assert controller.count("PlannerStatus::GOAL_REACHED") >= 2
    assert "publish_position_setpoint(planner_hold_target_.value())" in controller
    assert "planner_hold_active_" in controller
    assert "planner trajectory or map remained unsafe during hold" in controller


def test_planner_reuses_safe_active_trajectory_and_replans_blocked_remainder():
    planner = PLANNER_NODE.read_text(encoding="utf-8")
    assert "remaining_path_is_safe" in planner
    assert "active_grid_path_" in planner
    assert "active_trajectory_" in planner
    assert "active trajectory remains collision-free" in planner
    assert "++trajectory_id_" in planner
