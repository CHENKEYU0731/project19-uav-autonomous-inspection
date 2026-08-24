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

from datetime import datetime
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def fail_launch(message):
    def raise_failure(_context):
        raise RuntimeError(message)

    return OpaqueFunction(function=raise_failure)


def controller_exit_actions(event, context):
    if context.is_shutdown:
        return []
    if event.returncode == 0:
        return [EmitEvent(event=Shutdown(reason="M3 autonomy completed"))]
    return [
        fail_launch(
            "M3 controller failed with exit code " f"{event.returncode}"
        )
    ]


def required_process_exit_actions(process_name):
    def handle_exit(event, context):
        if context.is_shutdown:
            return []
        return [
            fail_launch(
                f"required M3 process {process_name} exited unexpectedly "
                f"with code {event.returncode}"
            )
        ]

    return handle_exit


def generate_launch_description():
    project_root = Path(os.environ.get("PROJECT_ROOT", "/opt/project19"))
    bringup_share = Path(get_package_share_directory("drone_bringup"))
    local_mapping_launch = bringup_share / "launch" / "local_mapping.launch.py"
    mapping_config = bringup_share / "config" / "m3_mapping.yaml"
    planner_config = bringup_share / "config" / "m3_planner.yaml"
    mission_config = bringup_share / "config" / "m3_mission.yaml"
    blocker_x = LaunchConfiguration("blocker_x_m")
    blocker_y = LaunchConfiguration("blocker_y_m")
    blocker_initial_x = LaunchConfiguration("blocker_initial_x_m")
    blocker_initial_y = LaunchConfiguration("blocker_initial_y_m")
    trigger_progress = LaunchConfiguration("trigger_progress_m")
    required_maps = LaunchConfiguration("required_map_observations")
    minimum_lead = LaunchConfiguration("minimum_insertion_lead_m")
    default_bag_directory = (
        project_root
        / "log"
        / "m3"
        / f"planner_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    default_bag_directory.parent.mkdir(parents=True, exist_ok=True)
    bag_directory = LaunchConfiguration("bag_directory")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(local_mapping_launch)),
        launch_arguments={
            "mapping_config": str(mapping_config),
            "run_mission": "false",
            "record_bag": "false",
            "use_rviz": "false",
        }.items(),
    )
    planner = Node(
        package="drone_planner",
        executable="planner_node",
        name="planner_node",
        output="screen",
        parameters=[str(planner_config)],
    )
    controller = Node(
        package="drone_controller",
        executable="waypoint_controller",
        name="waypoint_controller",
        output="screen",
        parameters=[str(mission_config)],
    )
    bag_recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            bag_directory,
            "/clock",
            "/camera/depth/image_raw",
            "/camera/depth/camera_info",
            "/local_occupancy_grid",
            "/drone_perception/diagnostics",
            "/drone_planner/trajectory",
            "/drone_planner/status",
            "/fmu/out/vehicle_odometry",
            "/fmu/out/vehicle_local_position_v1",
            "/fmu/out/vehicle_status_v1",
            "/fmu/out/vehicle_land_detected",
            "/fmu/in/trajectory_setpoint",
            "/tf",
            "/tf_static",
            "/drone_m3/dynamic_blocker_pose",
            "/drone_m3/dynamic_blocker_event",
            "/drone_m3/insertion_hold",
            "/world/inspection/contacts",
        ],
        output="screen",
    )
    dynamic_blocker = ExecuteProcess(
        cmd=[
            "python3",
            str(project_root / "scripts" / "m3_dynamic_blocker.py"),
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            ["active_x_m:=", blocker_x],
            "-p",
            ["active_y_m:=", blocker_y],
            "-p",
            ["initial_x_m:=", blocker_initial_x],
            "-p",
            ["initial_y_m:=", blocker_initial_y],
            "-p",
            ["trigger_progress_m:=", trigger_progress],
            "-p",
            ["required_map_observations:=", required_maps],
            "-p",
            ["minimum_insertion_lead_m:=", minimum_lead],
        ],
        output="screen",
        emulate_tty=False,
    )

    return LaunchDescription(
        [
            RegisterEventHandler(
                OnProcessExit(
                    target_action=controller,
                    on_exit=controller_exit_actions,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=planner,
                    on_exit=required_process_exit_actions("planner"),
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=bag_recorder,
                    on_exit=required_process_exit_actions("rosbag recorder"),
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=dynamic_blocker,
                    on_exit=required_process_exit_actions("dynamic blocker"),
                )
            ),
            DeclareLaunchArgument("blocker_x_m", default_value="0.0"),
            DeclareLaunchArgument("blocker_y_m", default_value="1.5"),
            DeclareLaunchArgument("blocker_initial_x_m", default_value="0.0"),
            DeclareLaunchArgument("blocker_initial_y_m", default_value="-3.0"),
            DeclareLaunchArgument("trigger_progress_m", default_value="0.5"),
            DeclareLaunchArgument("required_map_observations", default_value="1"),
            DeclareLaunchArgument("minimum_insertion_lead_m", default_value="0.6"),
            DeclareLaunchArgument(
                "bag_directory", default_value=str(default_bag_directory)
            ),
            simulation,
            planner,
            bag_recorder,
            dynamic_blocker,
            controller,
        ]
    )
