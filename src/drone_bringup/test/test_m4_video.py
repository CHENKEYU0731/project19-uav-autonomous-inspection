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
from pathlib import Path

from PIL import Image, ImageDraw
import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[3]
VERIFIER_PATH = PROJECT_ROOT / "scripts" / "verify_m4_video.py"
RECORDER_PATH = PROJECT_ROOT / "scripts" / "record_m4_demo.sh"
OPENBOX_CONFIG_PATH = PROJECT_ROOT / "scripts" / "m4-openbox.xml"


def load_module():
    spec = spec_from_file_location("verify_m4_video", VERIFIER_PATH)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def detailed_frame(left_offset=0, right_offset=0):
    image = Image.new("RGB", (1280, 720), (25, 25, 25))
    draw = ImageDraw.Draw(image)
    for x in range(0, 640, 40):
        draw.rectangle((x + left_offset, 0, x + left_offset + 18, 719), fill=(180, 90, 40))
    for y in range(0, 720, 40):
        draw.rectangle((640, y + right_offset, 1279, y + right_offset + 18), fill=(40, 170, 190))
    return image


def test_two_visible_dynamic_halves_are_accepted():
    module = load_module()
    metrics = module.analyze_frames(
        [detailed_frame(0, 0), detailed_frame(5, 7), detailed_frame(10, 14)]
    )

    assert max(metrics["left_motion_deltas"]) > module.MIN_MOTION_DELTA
    assert max(metrics["right_motion_deltas"]) > module.MIN_MOTION_DELTA


def test_black_half_is_rejected():
    module = load_module()
    frames = [detailed_frame(0, 0), detailed_frame(5, 7), detailed_frame(10, 14)]
    for frame in frames:
        frame.paste((0, 0, 0), (640, 0, 1280, 720))

    with pytest.raises(RuntimeError, match="right video half is effectively black"):
        module.analyze_frames(frames)


def test_static_half_is_rejected():
    module = load_module()
    static = detailed_frame(0, 0)

    with pytest.raises(RuntimeError, match="left video half is static"):
        module.analyze_frames([static.copy(), static.copy(), static.copy()])


def test_contact_sheet_supports_ubuntu_pillow(tmp_path):
    module = load_module()
    output = tmp_path / "contact-sheet.png"

    module.write_contact_sheet(
        [detailed_frame(0, 0), detailed_frame(5, 7), detailed_frame(10, 14)],
        output,
    )

    assert output.is_file()
    assert Image.open(output).size == (1920, 1080)


def test_recorder_uses_virtual_display_and_full_m4_launch_lifecycle():
    source = RECORDER_PATH.read_text(encoding="utf-8")

    assert 'repo_root="$(cd ' in source
    assert "project_root=" not in source
    assert 'source "${repo_root}/scripts/project-env.sh"' in source
    assert 'set +u\nsource "${repo_root}/scripts/project-env.sh"\nset -u' in source
    assert source.index('source "${repo_root}/scripts/project-env.sh"') < source.index(
        "for command in Xvfb"
    )
    assert "Xvfb" in source
    assert "wmctrl -i -r" in source
    assert "wmctrl -lx 2>/dev/null || true" in source
    assert "gazebo_x + gazebo_width > rviz_x" in source
    assert "pkill -KILL -f \"^Xvfb ${display} \"" in source
    assert "screen_width=2400" in source
    assert "kill -KILL" in source
    assert "m4_inspection.launch.py" in source
    assert "use_rviz:=true" in source
    assert "use_gazebo_gui:=true" in source
    assert "wait \"${launch_pid}\"" in source
    assert "analyze_m4_mission.py" in source
    assert "verify_m4_video.py" in source
    assert "M4_RECORDING_CHECK_ONLY" in source
    assert "M4_RECORDING_LAYOUT_ONLY" in source
    assert "ros2 topic echo --once /fmu/out/vehicle_odometry" in source
    assert source.index("ros2 topic echo --once /fmu/out/vehicle_odometry") < source.index(
        "ffmpeg -hide_banner"
    )

    openbox_config = OPENBOX_CONFIG_PATH.read_text(encoding="utf-8")
    assert '<application name="gz-sim-gui" class="Gazebo GUI">' in openbox_config
    assert '<application name="rviz2" class="rviz2">' in openbox_config
    assert openbox_config.count("<width>1200</width>") == 2
