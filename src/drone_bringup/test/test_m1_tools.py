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
import asyncio
import json
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


def wait_for_pid(path, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            value = int(path.read_text(encoding="utf-8").strip())
            if value > 0:
                return value
        except (FileNotFoundError, ValueError):
            pass
        time.sleep(0.05)
    raise AssertionError(f"timed out waiting for a complete PID in {path}")


def wait_for_file(path, timeout=2.0):
    deadline = time.monotonic() + timeout
    while not path.exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    assert path.exists(), f"timed out waiting for {path}"


def stop_wrapper(process):
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGTERM)
    try:
        process.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)


def kill_process_group(process_group_id):
    if process_group_id is None:
        return
    try:
        os.killpg(process_group_id, signal.SIGKILL)
    except ProcessLookupError:
        pass


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
        "capture.write_text('\\n'.join([sys.argv[1], sys.argv[2], "
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
        assert invocation[:5] == [
            "-d",
            str(px4_directory / "ROMFS" / "px4fmu_common"),
            "gz_x500",
            "127.0.0.1",
            str(rootfs_directory),
        ]
        px4_pid = int(invocation[5])

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


def test_px4_wrapper_cleans_build_process_group_when_interrupted(tmp_path):
    px4_directory = tmp_path / "PX4-Autopilot"
    px4_binary = (
        px4_directory / "build" / "px4_sitl_default" / "bin" / "px4"
    )
    capture_path = tmp_path / "make-pid.txt"
    px4_binary.parent.mkdir(parents=True)
    px4_binary.parents[1].joinpath("rootfs").mkdir()
    px4_binary.write_text("#!/bin/true\n", encoding="utf-8")
    px4_binary.chmod(0o755)
    (px4_directory / "Makefile").write_text(
        "px4_sitl:\n"
        f"\t@printf '%s\\n' \"$$PPID\" > '{capture_path}'\n"
        "\t@sleep 30\n",
        encoding="utf-8",
    )

    process = subprocess.Popen(
        [
            "bash",
            str(PROJECT_ROOT / "scripts" / "run-px4-sitl.sh"),
            str(px4_directory),
        ]
    )
    make_pid = None
    try:
        make_pid = wait_for_pid(capture_path)

        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=10.0) == 143
        try:
            os.killpg(make_pid, 0)
            assert False, "make process group survived wrapper shutdown"
        except ProcessLookupError:
            pass
    finally:
        stop_wrapper(process)
        kill_process_group(make_pid)


def test_px4_wrapper_finishes_cleanup_after_escalated_signal(tmp_path):
    px4_directory = tmp_path / "PX4-Autopilot"
    px4_binary = (
        px4_directory / "build" / "px4_sitl_default" / "bin" / "px4"
    )
    rootfs_directory = px4_binary.parents[1] / "rootfs"
    capture_path = tmp_path / "px4-pid.txt"
    int_path = tmp_path / "px4-int.txt"
    term_path = tmp_path / "px4-term.txt"
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
        "capture = Path(os.environ['PX4_CAPTURE'])\n"
        "int_path = Path(os.environ['PX4_INT_CAPTURE'])\n"
        "term_path = Path(os.environ['PX4_TERM_CAPTURE'])\n"
        "signal.signal(signal.SIGINT, lambda *_: int_path.write_text('INT'))\n"
        "signal.signal(signal.SIGTERM, lambda *_: term_path.write_text('TERM'))\n"
        "capture.write_text(str(os.getpid()), encoding='utf-8')\n"
        "while True:\n"
        "    signal.pause()\n",
        encoding="utf-8",
    )
    px4_binary.chmod(0o755)

    environment = os.environ.copy()
    environment["PX4_CAPTURE"] = str(capture_path)
    environment["PX4_INT_CAPTURE"] = str(int_path)
    environment["PX4_TERM_CAPTURE"] = str(term_path)
    process = subprocess.Popen(
        [
            "bash",
            str(PROJECT_ROOT / "scripts" / "run-px4-sitl.sh"),
            str(px4_directory),
        ],
        env=environment,
    )
    px4_pid = None
    try:
        px4_pid = wait_for_pid(capture_path)

        process.send_signal(signal.SIGINT)
        wait_for_file(int_path)
        process.send_signal(signal.SIGTERM)
        wait_for_file(term_path, timeout=0.5)

        assert process.wait(timeout=8.0) == 130
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                os.killpg(px4_pid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.05)
        else:
            assert False, "PX4 process group survived escalated wrapper shutdown"
    finally:
        stop_wrapper(process)
        kill_process_group(px4_pid)


def test_px4_wrapper_passes_startup_script_with_spaces(tmp_path):
    px4_directory = tmp_path / "PX4-Autopilot"
    px4_binary = (
        px4_directory / "build" / "px4_sitl_default" / "bin" / "px4"
    )
    rootfs_directory = px4_binary.parents[1] / "rootfs"
    startup_script = tmp_path / "startup scripts" / "headless rcS"
    capture_path = tmp_path / "startup-arguments.json"
    px4_binary.parent.mkdir(parents=True)
    rootfs_directory.mkdir()
    startup_script.parent.mkdir()
    startup_script.write_text("# test startup\n", encoding="utf-8")
    (px4_directory / "Makefile").write_text(
        "px4_sitl:\n\t@true\n", encoding="utf-8"
    )
    px4_binary.write_text(
        "#!/usr/bin/env python3\n"
        "import json\n"
        "import os\n"
        "from pathlib import Path\n"
        "import sys\n"
        "Path(os.environ['PX4_CAPTURE']).write_text(\n"
        "    json.dumps(sys.argv[1:]), encoding='utf-8')\n",
        encoding="utf-8",
    )
    px4_binary.chmod(0o755)

    environment = os.environ.copy()
    environment["PX4_CAPTURE"] = str(capture_path)
    result = subprocess.run(
        [
            "bash",
            str(PROJECT_ROOT / "scripts" / "run-px4-sitl.sh"),
            str(px4_directory),
            str(startup_script),
        ],
        env=environment,
        timeout=10.0,
        check=False,
    )

    assert result.returncode == 0
    assert json.loads(capture_path.read_text(encoding="utf-8")) == [
        "-d",
        "-s",
        str(startup_script),
        str(px4_directory / "ROMFS" / "px4fmu_common"),
    ]


def test_px4_wrapper_rejects_missing_startup_script_and_invalid_arity(tmp_path):
    px4_directory = tmp_path / "PX4-Autopilot"
    px4_binary = (
        px4_directory / "build" / "px4_sitl_default" / "bin" / "px4"
    )
    rootfs_directory = px4_binary.parents[1] / "rootfs"
    marker_path = tmp_path / "px4-started.txt"
    px4_binary.parent.mkdir(parents=True)
    rootfs_directory.mkdir()
    (px4_directory / "Makefile").write_text(
        "px4_sitl:\n\t@true\n", encoding="utf-8"
    )
    px4_binary.write_text(
        f"#!/usr/bin/env bash\nprintf started > '{marker_path}'\n",
        encoding="utf-8",
    )
    px4_binary.chmod(0o755)

    missing_result = subprocess.run(
        [
            "bash",
            str(PROJECT_ROOT / "scripts" / "run-px4-sitl.sh"),
            str(px4_directory),
            str(tmp_path / "missing rcS"),
        ],
        timeout=10.0,
        check=False,
    )
    no_argument_result = subprocess.run(
        ["bash", str(PROJECT_ROOT / "scripts" / "run-px4-sitl.sh")],
        timeout=5.0,
        check=False,
    )
    too_many_arguments_result = subprocess.run(
        [
            "bash",
            str(PROJECT_ROOT / "scripts" / "run-px4-sitl.sh"),
            "one",
            "two",
            "three",
        ],
        timeout=5.0,
        check=False,
    )

    assert missing_result.returncode == 1
    assert not marker_path.exists()
    assert no_argument_result.returncode == 2
    assert too_many_arguments_result.returncode == 2
