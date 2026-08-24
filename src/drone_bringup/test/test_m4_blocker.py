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
import time

import pytest


pytest.importorskip("rclpy")

PROJECT_ROOT = Path(__file__).resolve().parents[3]
BLOCKER_PATH = PROJECT_ROOT / "scripts" / "m3_dynamic_blocker.py"


def load_module():
    spec = spec_from_file_location("m3_dynamic_blocker_m4_test", BLOCKER_PATH)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    "remove_after, progress, hold, inserted, confirmed, removed, in_progress, expected",
    [
        (-1.0, 3.0, False, True, True, False, False, False),
        (2.3, 3.0, True, True, True, False, False, False),
        (2.3, 2.29, False, True, True, False, False, False),
        (2.3, 2.3, False, False, True, False, False, False),
        (2.3, 2.3, False, True, False, False, False, False),
        (2.3, 2.3, False, True, True, True, False, False),
        (2.3, 2.3, False, True, True, False, True, False),
        (2.3, 2.3, False, True, True, False, False, True),
    ],
)
def test_removal_readiness_requires_a_confirmed_safe_passage(
    remove_after, progress, hold, inserted, confirmed, removed, in_progress, expected
):
    module = load_module()
    assert (
        module.removal_is_ready(
            remove_after,
            progress,
            hold,
            inserted,
            confirmed,
            removed,
            in_progress,
        )
        is expected
    )


class AliveThread:
    def __init__(self, target=None, daemon=None):
        self.target = target
        self.daemon = daemon
        self.started = False

    def start(self):
        self.started = True

    def is_alive(self):
        return True


class FinishedThread:
    def is_alive(self):
        return False


def base_blocker(module):
    blocker = object.__new__(module.DynamicBlocker)
    blocker.insertion_thread = None
    blocker.removal_thread = None
    blocker.removal_started_at = None
    blocker.removal_result = None
    blocker.removed = False
    blocker.inserted = True
    blocker.replan_confirmed = True
    blocker.hold_active = False
    blocker.remove_after_progress = 2.3
    blocker.progress_m = 2.3
    blocker.initial_x = 0.0
    blocker.initial_y = -3.0
    blocker.active_x = 0.0
    blocker.active_y = 1.5
    blocker.published_events = []
    blocker.published_poses = []
    blocker._publish_event = (
        lambda event, x, y, **_kwargs: blocker.published_events.append((event, x, y))
    )
    blocker._publish_pose = lambda x, y: blocker.published_poses.append((x, y))
    return blocker


def test_removal_starts_once_after_safe_passage(monkeypatch):
    module = load_module()
    blocker = base_blocker(module)
    monkeypatch.setattr(module.threading, "Thread", AliveThread)
    blocker._tick()
    thread = blocker.removal_thread
    blocker._tick()

    assert isinstance(thread, AliveThread)
    assert thread.started is True
    assert blocker.removal_thread is thread
    assert [event[0] for event in blocker.published_events] == [
        "blocker_removal_started"
    ]


def test_successful_removal_publishes_safe_pose_and_terminal_event():
    module = load_module()
    blocker = base_blocker(module)
    blocker.removal_thread = FinishedThread()
    blocker.removal_result = True

    blocker._tick()

    assert blocker.removed is True
    assert blocker.removal_thread is None
    assert blocker.published_poses == [(0.0, -3.0)]
    assert blocker.published_events == [("blocker_removed", 0.0, -3.0)]


@pytest.mark.parametrize("timed_out", [False, True])
def test_removal_rejection_or_timeout_fails_the_process(timed_out):
    module = load_module()
    blocker = base_blocker(module)
    failures = []
    blocker._fail_removal = failures.append
    if timed_out:
        blocker.removal_thread = AliveThread()
        blocker.removal_started_at = (
            time.monotonic() - module.ACTIVE_INSERTION_TIMEOUT_S - 1.0
        )
    else:
        blocker.removal_thread = FinishedThread()
        blocker.removal_result = False

    blocker._tick()

    assert failures == [
        "set_pose response timed out" if timed_out else "set_pose rejected"
    ]
