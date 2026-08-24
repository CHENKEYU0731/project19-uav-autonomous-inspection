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
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def fail_launch(message):
    def raise_failure(_context):
        raise RuntimeError(message)

    return OpaqueFunction(function=raise_failure)


def mission_exit_actions(event, context):
    if context.is_shutdown:
        return []
    if event.returncode == 0:
        return [
            EmitEvent(
                event=Shutdown(reason="M2 mapping mission completed")
            )
        ]
    return [
        fail_launch(
            "M2 mapping mission failed with exit code "
            f"{event.returncode}"
        )
    ]


def required_process_exit_actions(process_name):
    def handle_exit(event, context):
        if context.is_shutdown:
            return []
        return [
            fail_launch(
                f"required process {process_name} exited unexpectedly "
                f"with code {event.returncode}"
            )
        ]

    return handle_exit


def validate_simulation_dependencies(
    _context,
    agent_binary,
    px4_directory,
    px4_wrapper,
    px4_startup_script,
    world_path,
    project_model_path,
):
    missing = []
    if not agent_binary.is_file() or not os.access(agent_binary, os.X_OK):
        missing.append(str(agent_binary))
    if not (px4_directory / "Makefile").is_file():
        missing.append(str(px4_directory))
    if not px4_wrapper.is_file():
        missing.append(str(px4_wrapper))
    if not px4_startup_script.is_file():
        missing.append(str(px4_startup_script))
    if not world_path.is_file():
        missing.append(str(world_path))
    if not project_model_path.is_file():
        missing.append(str(project_model_path))
    if missing:
        raise RuntimeError(
            "missing required M2 dependency: " + ", ".join(missing)
        )
    return []


def resource_path(*paths):
    return os.pathsep.join(
        str(path) for path in paths if path and str(path)
    )


def generate_launch_description():
    project_root = Path(os.environ.get("PROJECT_ROOT", "/opt/project19"))
    px4_directory = project_root / "external" / "PX4-Autopilot"
    px4_wrapper = project_root / "scripts" / "run-px4-sitl.sh"
    px4_startup_script = project_root / "scripts" / "px4-headless-rcS"
    agent_binary = (
        project_root
        / ".local"
        / "micro-xrce-dds-agent"
        / "bin"
        / "MicroXRCEAgent"
    )

    bringup_share = Path(get_package_share_directory("drone_bringup"))
    simulation_share = Path(get_package_share_directory("drone_sim"))
    project_models = simulation_share / "models"
    world_path = simulation_share / "worlds" / "inspection.sdf"
    project_model_path = (
        project_models / "x500_depth_project" / "model.sdf"
    )
    px4_models = px4_directory / "Tools" / "simulation" / "gz" / "models"
    px4_worlds = px4_directory / "Tools" / "simulation" / "gz" / "worlds"
    px4_plugins = (
        px4_directory
        / "build"
        / "px4_sitl_default"
        / "src"
        / "modules"
        / "simulation"
        / "gz_plugins"
    )
    gz_resource_path = resource_path(
        project_models,
        px4_models,
        px4_worlds,
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    )
    gz_plugin_path = resource_path(
        px4_plugins,
        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
    )

    default_mapping_config = (
        bringup_share / "config" / "local_mapping.yaml"
    )
    mission_config = bringup_share / "config" / "mapping_mission.yaml"
    rviz_config = bringup_share / "config" / "local_mapping.rviz"
    bag_directory = (
        project_root
        / "log" / "m2"
        / datetime.now().strftime("mapping_%Y%m%d_%H%M%S")
    )
    bag_directory.parent.mkdir(parents=True, exist_ok=True)

    use_rviz = LaunchConfiguration("use_rviz")
    run_mission = LaunchConfiguration("run_mission")
    record_bag = LaunchConfiguration("record_bag")
    mapping_config = LaunchConfiguration("mapping_config")

    simulation_preflight = OpaqueFunction(
        function=validate_simulation_dependencies,
        args=[
            agent_binary,
            px4_directory,
            px4_wrapper,
            px4_startup_script,
            world_path,
            project_model_path,
        ],
    )
    gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", str(world_path)],
        output="screen",
        emulate_tty=False,
        additional_env={
            "GZ_SIM_RESOURCE_PATH": gz_resource_path,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
        },
    )
    dds_agent = ExecuteProcess(
        cmd=[str(agent_binary), "udp4", "-p", "8888"],
        output="screen",
    )
    px4_sitl = ExecuteProcess(
        cmd=[
            "bash",
            str(px4_wrapper),
            str(px4_directory),
            str(px4_startup_script),
        ],
        output="screen",
        emulate_tty=False,
        additional_env={
            "HEADLESS": "1",
            "PX4_GZ_STANDALONE": "1",
            "PX4_GZ_WORLD": "inspection",
            "PX4_SIM_MODEL": "gz_x500_depth_project",
            "PX4_SYS_AUTOSTART": "4001",
            "PX4_GZ_MODELS": str(project_models),
            "GZ_SIM_RESOURCE_PATH": gz_resource_path,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
        },
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="m2_gazebo_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth/camera_info"
            "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/world/inspection/contacts"
            "@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
        ],
        parameters=[{"use_sim_time": True}],
    )
    tf_broadcaster = Node(
        package="drone_perception",
        executable="px4_tf_broadcaster",
        name="px4_tf_broadcaster",
        output="screen",
        parameters=[mapping_config],
    )
    mapper = Node(
        package="drone_perception",
        executable="depth_grid_node",
        name="depth_grid_node",
        output="screen",
        parameters=[mapping_config],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="m2_rviz",
        output="screen",
        arguments=["-d", str(rviz_config)],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )
    mission_controller = Node(
        package="drone_controller",
        executable="waypoint_controller",
        name="waypoint_controller",
        output="screen",
        parameters=[str(mission_config)],
        condition=IfCondition(run_mission),
    )
    bag_recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            str(bag_directory),
            "/clock",
            "/camera/depth/image_raw",
            "/camera/depth/camera_info",
            "/fmu/out/vehicle_odometry",
            "/fmu/out/vehicle_local_position_v1",
            "/fmu/out/vehicle_status_v1",
            "/fmu/out/vehicle_land_detected",
            "/local_occupancy_grid",
            "/drone_perception/diagnostics",
            "/tf",
            "/tf_static",
        ],
        output="screen",
        condition=IfCondition(record_bag),
    )

    process_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=mission_controller,
                on_exit=mission_exit_actions,
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo_server,
                on_exit=required_process_exit_actions("Gazebo server"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=dds_agent,
                on_exit=required_process_exit_actions(
                    "Micro XRCE-DDS Agent"
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=px4_sitl,
                on_exit=required_process_exit_actions("PX4 SITL"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=bridge,
                on_exit=required_process_exit_actions("Gazebo ROS bridge"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=tf_broadcaster,
                on_exit=required_process_exit_actions("PX4 TF broadcaster"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=mapper,
                on_exit=required_process_exit_actions("depth grid mapper"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=rviz,
                on_exit=required_process_exit_actions("RViz2"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=bag_recorder,
                on_exit=required_process_exit_actions("rosbag recorder"),
            )
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Open RViz2 with the project M2 mapping view",
            ),
            DeclareLaunchArgument(
                "run_mission",
                default_value="true",
                description="Fly the safe M2 mapping route and then exit",
            ),
            DeclareLaunchArgument(
                "record_bag",
                default_value="true",
                description="Record M2 evidence under project-local log/m2",
            ),
            DeclareLaunchArgument(
                "mapping_config",
                default_value=str(default_mapping_config),
                description="ROS parameter file for TF and local mapping",
            ),
            *process_handlers,
            simulation_preflight,
            gazebo_server,
            dds_agent,
            px4_sitl,
            bridge,
            tf_broadcaster,
            mapper,
            rviz,
            bag_recorder,
            mission_controller,
        ]
    )
