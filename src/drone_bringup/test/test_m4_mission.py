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
LAUNCH_PATH = PROJECT_ROOT / "src/drone_bringup/launch/m4_inspection.launch.py"
MISSION_CONFIG = PROJECT_ROOT / "src/drone_bringup/config/m4_mission.yaml"
CONTROLLER_CONFIG = PROJECT_ROOT / "src/drone_bringup/config/m4_controller.yaml"
MISSION_NODE = PROJECT_ROOT / "src/drone_mission/src/mission_node.cpp"
CONTROLLER_NODE = PROJECT_ROOT / "src/drone_controller/src/waypoint_controller_node.cpp"
INTERFACES_CMAKE = PROJECT_ROOT / "src/drone_interfaces/CMakeLists.txt"
WORLD_PATH = PROJECT_ROOT / "src/drone_sim/worlds/inspection.sdf"
BLOCKER_SCRIPT = PROJECT_ROOT / "scripts/m3_dynamic_blocker.py"


def string_literals(path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }


def test_m4_configs_define_mission_and_default_preserving_controller_contract():
    mission = yaml.safe_load(MISSION_CONFIG.read_text(encoding="utf-8"))
    controller = yaml.safe_load(CONTROLLER_CONFIG.read_text(encoding="utf-8"))
    mission_params = mission["mission_node"]["ros__parameters"]
    controller_params = controller["waypoint_controller"]["ros__parameters"]

    assert controller_params["use_planned_trajectory"] is True
    assert controller_params["mission_managed_landing"] is True
    assert mission_params["map_frame"] == controller_params["map_frame"] == "map"
    assert (
        mission_params["planner_status_timeout_s"]
        <= mission_params["unreachable_timeout_s"]
    )
    assert len(mission_params["inspection_waypoints_xy"]) >= 8
    assert len(mission_params["inspection_waypoints_xy"]) % 2 == 0
    assert mission_params["simulate_low_battery_after_reached_waypoints"] > 0
    controller_source = CONTROLLER_NODE.read_text(encoding="utf-8")
    assert (
        'declare_parameter<bool>("mission_managed_landing", false)' in controller_source
    )


def test_m4_interfaces_and_node_enforce_replayable_safety_contract():
    interfaces = INTERFACES_CMAKE.read_text(encoding="utf-8")
    node = MISSION_NODE.read_text(encoding="utf-8")
    controller = CONTROLLER_NODE.read_text(encoding="utf-8")

    assert '"msg/MissionCommand.msg"' in interfaces
    assert '"msg/MissionEvent.msg"' in interfaces
    assert "stamp < active_goal_stamp_" in node
    assert "unreachable_timeout_s_" in node
    assert "fsm_.goal_unreachable" in node
    assert "fsm_.low_battery" in node
    assert '"/drone_mission/event"' in node
    assert '"/drone_mission/command"' in controller
    assert "Rejecting stale or invalid mission LAND command" in controller
    assert "MissionCommand::SET_YAW" in node
    assert "MissionCommand::SET_YAW" in controller
    assert "setpoint.yaw = commanded_yaw_rad_" in controller
    assert "holding for the mission manager" in controller


def test_m4_launch_is_one_command_and_records_direct_evidence():
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    literals = string_literals(LAUNCH_PATH)
    assert "local_mapping.launch.py" in literals
    assert "m4_controller.yaml" in literals
    assert "m4_mission.yaml" in literals
    assert 'package="drone_mission"' in source
    assert 'package="drone_planner"' in source
    assert 'package="drone_controller"' in source
    assert '["gz", "sim", "-g"]' in source
    assert 'DeclareLaunchArgument("use_rviz", default_value="true")' in source
    assert 'DeclareLaunchArgument("use_gazebo_gui", default_value="true")' in source
    assert '"remove_after_progress_m:=2.3"' in source
    for topic in (
        "/drone_planner/goal",
        "/drone_planner/trajectory",
        "/drone_planner/status",
        "/drone_mission/command",
        "/drone_mission/event",
        "/fmu/out/vehicle_local_position_v1",
        "/fmu/out/vehicle_status_v1",
        "/fmu/out/vehicle_land_detected",
        "/fmu/in/trajectory_setpoint",
        "/drone_m3/dynamic_blocker_event",
        "/world/inspection/contacts",
    ):
        assert topic in literals


def test_m4_launch_can_delay_only_controller_and_mission_for_recording_setup():
    source = LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("demo_start_delay_s", default_value="0.0")' in source
    assert "delayed_mission_start = TimerAction(" in source
    assert "period=demo_start_delay" in source
    assert "actions=[controller, mission]" in source
    assert "delayed_mission_start," in source


def test_m4_removes_dynamic_blocker_only_after_the_vehicle_has_passed_it():
    blocker = BLOCKER_SCRIPT.read_text(encoding="utf-8")
    assert 'declare("remove_after_progress_m", -1.0)' in blocker
    assert "self.remove_after_progress <= self.active_y + self.minimum_lead" in blocker
    assert '"blocker_removal_started"' in blocker
    assert '"blocker_removed"' in blocker


def test_final_world_contains_inspection_equipment_outside_center_route():
    world = WORLD_PATH.read_text(encoding="utf-8")
    assert '<model name="warehouse_rack_left">' in world
    assert '<model name="warehouse_rack_right">' in world
    assert '<model name="substation_cabinet">' in world
