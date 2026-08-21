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


def controller_exit_actions(event, context):
    if context.is_shutdown:
        return []
    if event.returncode == 0:
        return [
            EmitEvent(
                event=Shutdown(
                    reason="waypoint mission controller completed"
                )
            )
        ]
    return [
        fail_launch(
            "waypoint mission controller failed with exit code "
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
    _context, agent_binary, px4_directory, px4_wrapper
):
    missing = []
    if not agent_binary.is_file() or not os.access(agent_binary, os.X_OK):
        missing.append(str(agent_binary))
    if not (px4_directory / "Makefile").is_file():
        missing.append(str(px4_directory))
    if not px4_wrapper.is_file():
        missing.append(str(px4_wrapper))
    if missing:
        raise RuntimeError(
            "missing required simulation dependency: " + ", ".join(missing)
        )
    return []


def generate_launch_description():
    project_root = Path(os.environ.get("PROJECT_ROOT", "/opt/project19"))
    px4_directory = project_root / "external" / "PX4-Autopilot"
    px4_wrapper = project_root / "scripts" / "run-px4-sitl.sh"
    agent_binary = (
        project_root
        / ".local"
        / "micro-xrce-dds-agent"
        / "bin"
        / "MicroXRCEAgent"
    )
    config_path = (
        Path(get_package_share_directory("drone_bringup"))
        / "config"
        / "waypoint_mission.yaml"
    )
    bag_directory = (
        project_root
        / "log"
        / "m1"
        / f"trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    bag_directory.parent.mkdir(parents=True, exist_ok=True)

    start_simulation = LaunchConfiguration("start_simulation")
    record_bag = LaunchConfiguration("record_bag")

    simulation_preflight = OpaqueFunction(
        function=validate_simulation_dependencies,
        args=[agent_binary, px4_directory, px4_wrapper],
        condition=IfCondition(start_simulation),
    )
    dds_agent = ExecuteProcess(
        cmd=[str(agent_binary), "udp4", "-p", "8888"],
        output="screen",
        condition=IfCondition(start_simulation),
    )
    px4_sitl = ExecuteProcess(
        cmd=[
            "bash",
            str(px4_wrapper),
            str(px4_directory),
        ],
        output="screen",
        # The wrapper uses PX4 daemon mode so no interactive shell is started.
        emulate_tty=False,
        additional_env={
            "HEADLESS": "1",
            "PX4_PARAM_NAV_DLL_ACT": "0",
        },
        condition=IfCondition(start_simulation),
    )
    bag_recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            str(bag_directory),
            "/fmu/in/trajectory_setpoint",
            "/fmu/out/vehicle_local_position_v1",
            "/fmu/out/vehicle_status_v1",
            "/fmu/out/vehicle_land_detected",
            "/fmu/out/vehicle_command_ack",
        ],
        output="screen",
        condition=IfCondition(record_bag),
    )
    controller = Node(
        package="drone_controller",
        executable="waypoint_controller",
        name="waypoint_controller",
        output="screen",
        parameters=[str(config_path)],
    )

    handle_controller_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=controller,
            on_exit=controller_exit_actions,
        )
    )
    handle_agent_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=dds_agent,
            on_exit=required_process_exit_actions("Micro XRCE-DDS Agent"),
        )
    )
    handle_px4_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=px4_sitl,
            on_exit=required_process_exit_actions("PX4 SITL"),
        )
    )
    handle_bag_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_recorder,
            on_exit=required_process_exit_actions("rosbag recorder"),
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_simulation",
                default_value="true",
                description="Start Micro XRCE-DDS Agent and PX4 gz_x500 SITL",
            ),
            DeclareLaunchArgument(
                "record_bag",
                default_value="true",
                description=(
                    "Record desired and actual trajectories under log/m1"
                ),
            ),
            handle_controller_exit,
            handle_agent_exit,
            handle_px4_exit,
            handle_bag_exit,
            simulation_preflight,
            dds_agent,
            px4_sitl,
            bag_recorder,
            controller,
        ]
    )
