from importlib.util import module_from_spec, spec_from_file_location
import asyncio
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from types import SimpleNamespace

from launch import LaunchDescription, LaunchService
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


PROJECT_ROOT = Path(__file__).resolve().parents[3]


def load_module(name, path):
    spec = spec_from_file_location(name, path)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


plot_m1 = load_module(
    "plot_m1_trajectory",
    PROJECT_ROOT / "scripts" / "plot_m1_trajectory.py",
)
mission_launch = load_module(
    "waypoint_mission_launch",
    PROJECT_ROOT
    / "src"
    / "drone_bringup"
    / "launch"
    / "waypoint_mission.launch.py",
)


def complete_trajectory():
    targets = (
        (0.0, 0.0, -2.5),
        (2.0, 0.0, -2.5),
        (2.0, 2.0, -2.5),
        (-2.0, 2.0, -2.5),
        (-2.0, 0.0, -2.5),
        (0.0, 0.0, -2.5),
    )
    target_samples = []
    for segment_index, target in enumerate(targets):
        for sample_index in range(20):
            timestamp_ns = int(
                (segment_index * 2.0 + sample_index * 0.1) * 1e9
            )
            target_samples.append((timestamp_ns, *target))

    position_samples = []
    for sample_index in range(239):
        timestamp_s = sample_index * 0.05
        segment_index = min(int(timestamp_s // 2.0), len(targets) - 1)
        position_samples.append(
            (int(timestamp_s * 1e9), *targets[segment_index])
        )
    return target_samples, position_samples


def test_metrics_require_six_complete_segments():
    target_samples, position_samples = complete_trajectory()

    metrics = plot_m1.calculate_tracking_metrics(
        target_samples, position_samples
    )

    assert metrics["steady_state_error"]["segment_count"] == 6
    assert "6 target segments" in metrics["steady_state_error"]["definition"]
    assert math.isfinite(metrics["steady_state_error"]["mean_m"])


def test_metrics_reject_truncated_or_non_finite_evidence():
    target_samples, position_samples = complete_trajectory()

    try:
        plot_m1.calculate_tracking_metrics(
            target_samples[:20], position_samples
        )
        assert False, "single-segment evidence must be rejected"
    except RuntimeError as error:
        assert "expected 6" in str(error)

    invalid_positions = list(position_samples)
    invalid_positions[0] = (0, math.nan, 0.0, -2.5)
    try:
        plot_m1.calculate_tracking_metrics(
            target_samples, invalid_positions
        )
        assert False, "non-finite evidence must be rejected"
    except RuntimeError as error:
        assert "non-finite" in str(error)


def test_mission_evidence_requires_safe_terminal_state():
    target_samples, position_samples = complete_trajectory()
    mission_end_ns = target_samples[-1][0]
    status_samples = [
        (0, False, True, False, False),
        (int(1e9), True, False, True, False),
        (mission_end_ns + int(1e8), False, True, False, False),
    ]
    land_samples = [
        (0, True),
        (mission_end_ns + int(1e8), True),
    ]

    segments = plot_m1.validate_mission_evidence(
        target_samples,
        position_samples,
        status_samples,
        land_samples,
    )
    assert len(segments) == 6

    unsafe_status = list(status_samples)
    unsafe_status[-1] = (
        unsafe_status[-1][0],
        True,
        False,
        True,
        False,
    )
    try:
        plot_m1.validate_mission_evidence(
            target_samples,
            position_samples,
            unsafe_status,
            land_samples,
        )
        assert False, "armed terminal state must be rejected"
    except RuntimeError as error:
        assert "disarmed" in str(error)


def test_controller_exit_code_is_propagated():
    active_context = SimpleNamespace(is_shutdown=False)
    success_actions = mission_launch.controller_exit_actions(
        SimpleNamespace(returncode=0), active_context
    )
    failure_actions = mission_launch.controller_exit_actions(
        SimpleNamespace(returncode=7), active_context
    )

    assert isinstance(success_actions[0], EmitEvent)
    assert isinstance(failure_actions[0], OpaqueFunction)
    assert mission_launch.controller_exit_actions(
        SimpleNamespace(returncode=-2),
        SimpleNamespace(is_shutdown=True),
    ) == []


def test_required_process_exit_is_ignored_only_during_shutdown():
    handler = mission_launch.required_process_exit_actions("PX4 SITL")
    event = SimpleNamespace(returncode=2)

    active_actions = handler(event, SimpleNamespace(is_shutdown=False))
    assert isinstance(active_actions[0], OpaqueFunction)
    assert handler(event, SimpleNamespace(is_shutdown=True)) == []


def test_simulation_preflight_requires_project_local_dependencies(tmp_path):
    agent_binary = tmp_path / "MicroXRCEAgent"
    px4_directory = tmp_path / "PX4-Autopilot"
    px4_wrapper = tmp_path / "run-px4-sitl.sh"

    try:
        mission_launch.validate_simulation_dependencies(
            None, agent_binary, px4_directory, px4_wrapper
        )
        assert False, "missing simulation dependencies must be rejected"
    except RuntimeError as error:
        assert "missing required simulation dependency" in str(error)

    agent_binary.write_text("#!/bin/true\n", encoding="utf-8")
    agent_binary.chmod(0o755)
    px4_directory.mkdir()
    (px4_directory / "Makefile").write_text("all:\n", encoding="utf-8")
    px4_wrapper.write_text("#!/usr/bin/env bash\n", encoding="utf-8")

    assert mission_launch.validate_simulation_dependencies(
        None, agent_binary, px4_directory, px4_wrapper
    ) == []


def test_successful_controller_exit_shuts_down_launch_dependencies():
    async def run_launch():
        dependencies = [
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    "trap 'exit 0' INT TERM; while true; do sleep 1; done",
                ],
            )
            for _ in range(3)
        ]
        controller = Node(
            package="drone_controller",
            executable="waypoint_controller",
            prefix="/bin/true",
        )
        launch_description = LaunchDescription(
            [
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=controller,
                        on_exit=mission_launch.controller_exit_actions,
                    )
                ),
                *[
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=dependency,
                            on_exit=(
                                mission_launch.required_process_exit_actions(
                                    f"dependency {index}"
                                )
                            ),
                        )
                    )
                    for index, dependency in enumerate(dependencies)
                ],
                *dependencies,
                controller,
            ]
        )
        launch_service = LaunchService()
        launch_service.include_launch_description(launch_description)
        try:
            return await asyncio.wait_for(
                launch_service.run_async(), timeout=15.0
            )
        except asyncio.TimeoutError:
            launch_service.shutdown()
            raise AssertionError(
                "launch did not stop after the controller exited successfully"
            )

    assert asyncio.run(run_launch()) == 0


def test_failed_controller_exit_fails_launch_and_cleans_dependencies():
    async def run_launch():
        dependency = ExecuteProcess(
            cmd=[
                "bash",
                "-c",
                "trap 'exit 0' INT TERM; while true; do sleep 1; done",
            ],
        )
        controller = Node(
            package="drone_controller",
            executable="waypoint_controller",
            prefix="/bin/false",
        )
        launch_description = LaunchDescription(
            [
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=controller,
                        on_exit=mission_launch.controller_exit_actions,
                    )
                ),
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=dependency,
                        on_exit=mission_launch.required_process_exit_actions(
                            "dependency"
                        ),
                    )
                ),
                dependency,
                controller,
            ]
        )
        launch_service = LaunchService()
        launch_service.include_launch_description(launch_description)
        try:
            return await asyncio.wait_for(
                launch_service.run_async(), timeout=5.0
            )
        except asyncio.TimeoutError:
            launch_service.shutdown()
            raise AssertionError(
                "launch did not stop after the controller failed"
            )

    assert asyncio.run(run_launch()) != 0


def test_px4_wrapper_starts_daemon_and_cleans_process_group(tmp_path):
    px4_directory = tmp_path / "PX4-Autopilot"
    px4_binary = (
        px4_directory / "build" / "px4_sitl_default" / "bin" / "px4"
    )
    rootfs_directory = px4_binary.parents[1] / "rootfs"
    capture_path = tmp_path / "px4-invocation.txt"
    px4_binary.parent.mkdir(parents=True)
    rootfs_directory.mkdir()
    (px4_directory / "Makefile").write_text(
        "px4_sitl:\n\t@true\n", encoding="utf-8"
    )
    px4_binary.write_text(
        "#!/usr/bin/env python3\n"
        "import os\n"
        "from pathlib import Path\n"
        "import signal\n"
        "import sys\n"
        "capture = Path(os.environ['PX4_CAPTURE'])\n"
        "capture.write_text('\\n'.join([sys.argv[1], "
        "os.environ['PX4_SIM_MODEL'], os.environ['GZ_IP'], os.getcwd(), "
        "str(os.getpid())]) + '\\n', encoding='utf-8')\n"
        "def stop(_signum, _frame):\n"
        "    raise SystemExit(0)\n"
        "signal.signal(signal.SIGINT, stop)\n"
        "signal.signal(signal.SIGTERM, stop)\n"
        "while True:\n"
        "    signal.pause()\n",
        encoding="utf-8",
    )
    px4_binary.chmod(0o755)

    environment = os.environ.copy()
    environment["PX4_CAPTURE"] = str(capture_path)
    process = subprocess.Popen(
        [
            "bash",
            str(PROJECT_ROOT / "scripts" / "run-px4-sitl.sh"),
            str(px4_directory),
        ],
        env=environment,
    )
    try:
        deadline = time.monotonic() + 15.0
        while not capture_path.exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert capture_path.exists(), "PX4 wrapper did not start the binary"

        invocation = capture_path.read_text(encoding="utf-8").splitlines()
        assert invocation[:4] == [
            "-d",
            "gz_x500",
            "127.0.0.1",
            str(rootfs_directory),
        ]
        px4_pid = int(invocation[4])

        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=5.0) == 143
        try:
            os.kill(px4_pid, 0)
            assert False, "PX4 process survived wrapper shutdown"
        except ProcessLookupError:
            pass
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
            try:
                process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)
