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
import json
import math
from pathlib import Path

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[3]
ANALYZER_PATH = PROJECT_ROOT / "scripts" / "analyze_m3_planning.py"


def load_module():
    spec = spec_from_file_location("analyze_m3_planning", ANALYZER_PATH)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def complete_evidence(module):
    second = 1_000_000_000
    pre_points = ((0.0, 0.0, 2.5), (0.0, 3.0, 2.5))
    post_points = (
        (0.0, 0.8, 2.5),
        (0.8, 0.8, 2.5),
        (0.8, 2.2, 2.5),
        (0.0, 3.0, 2.5),
    )
    poses = [
        module.PoseSample(0, 0.0, 0.0, 0.0),
        module.PoseSample(2 * second, 0.0, 0.0, 2.5),
        module.PoseSample(4 * second, 0.0, 0.8, 2.5),
        module.PoseSample(4_500_000_000, 0.9, 0.8, 2.5),
        module.PoseSample(5_500_000_000, 0.9, 2.2, 2.5),
        module.PoseSample(7 * second, 0.0, 3.0, 2.5),
    ]
    return module.PlanningEvidence(
        trajectories=[
            module.TrajectorySample(2 * second, 1, pre_points),
            module.TrajectorySample(4_200_000_000, 2, post_points),
        ],
        statuses=[
            module.StatusSample(2_050_000_000, 1, 5, True, True),
            module.StatusSample(3_900_000_000, 1, 5, True, True),
            module.StatusSample(4_250_000_000, 2, 5, True, True),
            module.StatusSample(5 * second, 2, 5, True, True),
            module.StatusSample(6 * second, 2, 5, True, True),
        ],
        events=[
            module.BlockerEvent(1 * second, "blocker_initialized", 0.0, -3.0),
            module.BlockerEvent(
                3_950_000_000,
                "blocker_insertion_started",
                0.0,
                1.5,
                1,
                1,
            ),
            module.BlockerEvent(4 * second, "blocker_inserted", 0.0, 1.5, 1, 1),
            module.BlockerEvent(
                4_300_000_000,
                "blocker_replan_confirmed",
                0.0,
                1.5,
                2,
                2,
            ),
        ],
        poses=poses,
        setpoints=[
            module.SetpointSample(4_300_000_000, (0.1, 0.8, -2.5), (0.5, 0.5, 0.0)),
            module.SetpointSample(5_100_000_000, (1.5, 0.8, -2.5), (-0.5, 0.5, 0.0)),
        ],
        vehicle_statuses=[
            module.VehicleStatusSample(0, 1, 4, False),
            module.VehicleStatusSample(1 * second, 2, 14, False),
            module.VehicleStatusSample(9 * second, 1, 4, False),
        ],
        land_samples=[
            module.LandSample(0, True),
            module.LandSample(1_100_000_000, False),
            module.LandSample(8 * second, True),
        ],
        collision_pairs=[],
        contacts_topic_present=True,
    )


def invalidate_latest_pre_insertion_status(evidence):
    insertion = next(
        event for event in evidence.events if event.event == "blocker_insertion_started"
    )
    latest = max(
        (
            status
            for status in evidence.statuses
            if status.timestamp_ns <= insertion.timestamp_ns
        ),
        key=lambda status: status.timestamp_ns,
    )
    latest.map_fresh = False


def move_replan_status_after_confirmation(evidence):
    evidence.statuses[2].timestamp_ns = 4_310_000_000


def remove_event(evidence, event_name):
    evidence.events = [event for event in evidence.events if event.event != event_name]


def move_pose_to_blocker(evidence):
    evidence.poses[3].x = 0.0
    evidence.poses[3].y = 1.5


def test_complete_dynamic_replanning_evidence_passes():
    module = load_module()
    metrics = module.validate_evidence(complete_evidence(module), launch_exit_code=0)

    assert metrics["accepted"] is True
    assert metrics["replanning"]["initial_trajectory_id"] == 1
    assert metrics["replanning"]["replanned_trajectory_id"] == 2
    assert metrics["flight"]["landed_and_disarmed"] is True
    assert metrics["clearance"]["minimum_actual_clearance_m"] > 0.0


def test_replan_may_be_recorded_before_insertion_completion_event():
    module = load_module()
    evidence = complete_evidence(module)
    evidence.trajectories[1].timestamp_ns = 3_980_000_000

    metrics = module.validate_evidence(evidence, launch_exit_code=0)

    assert metrics["replanning"]["replan_latency_s"] == pytest.approx(0.3)


def test_replan_latency_is_measured_after_gazebo_insertion_completes():
    module = load_module()
    evidence = complete_evidence(module)
    evidence.trajectories[0].timestamp_ns = 100_000_000
    evidence.statuses[0].timestamp_ns = 400_000_000
    evidence.events[0].timestamp_ns = 0
    insertion_start = next(
        event for event in evidence.events if event.event == "blocker_insertion_started"
    )
    insertion_start.timestamp_ns = 500_000_000

    metrics = module.validate_evidence(evidence, launch_exit_code=0)

    assert metrics["replanning"]["replan_latency_s"] == pytest.approx(0.3)


def test_recorded_blocker_must_match_expected_random_scenario():
    module = load_module()
    evidence = complete_evidence(module)

    with pytest.raises(RuntimeError, match="does not match the scenario"):
        module.validate_evidence(
            evidence,
            launch_exit_code=0,
            expected_blocker=(0.1, 1.5),
        )


def test_goal_check_ignores_horizontally_closer_landing_descent_pose():
    module = load_module()
    evidence = complete_evidence(module)
    evidence.poses[-1].x = 0.1
    evidence.poses.append(module.PoseSample(7_500_000_000, 0.0, 3.0, 0.2))

    metrics = module.validate_evidence(evidence, launch_exit_code=0)

    assert metrics["flight"]["goal_error_m"] == pytest.approx(0.1)


def test_clearance_check_rejects_collision_between_pose_samples():
    module = load_module()
    evidence = complete_evidence(module)
    evidence.poses = [
        module.PoseSample(0, 0.0, 0.0, 0.0),
        module.PoseSample(2_000_000_000, 0.0, 0.0, 2.5),
        module.PoseSample(4_000_000_000, 0.0, 0.8, 2.5),
        module.PoseSample(5_000_000_000, 0.0, 2.2, 2.5),
        module.PoseSample(7_000_000_000, 0.0, 3.0, 2.5),
    ]

    with pytest.raises(RuntimeError, match="actual vehicle clearance"):
        module.validate_evidence(evidence, launch_exit_code=0)


def test_position_change_is_treated_as_motion_when_velocity_is_unused():
    module = load_module()
    evidence = complete_evidence(module)
    unused_velocity = (math.nan, math.nan, math.nan)
    evidence.setpoints = [
        module.SetpointSample(4_300_000_000, (0.1, 0.8, -2.5), unused_velocity),
        module.SetpointSample(5_100_000_000, (1.5, 0.8, -2.5), unused_velocity),
    ]

    metrics = module.validate_evidence(evidence, launch_exit_code=0)

    assert metrics["replanning"]["post_replan_motion_setpoint_count"] == 1


def test_position_only_motion_requires_fresh_matching_status():
    module = load_module()
    evidence = complete_evidence(module)
    unused_velocity = (math.nan, math.nan, math.nan)
    evidence.setpoints = [
        module.SetpointSample(4_300_000_000, (0.1, 0.8, -2.5), unused_velocity),
        module.SetpointSample(5_800_000_000, (1.5, 0.8, -2.5), unused_velocity),
    ]

    with pytest.raises(RuntimeError, match="fresh matching planner status"):
        module.validate_evidence(evidence, launch_exit_code=0)


def test_switching_from_velocity_motion_to_fixed_hold_is_not_false_motion():
    module = load_module()
    evidence = complete_evidence(module)
    unused_velocity = (math.nan, math.nan, math.nan)
    evidence.statuses.append(module.StatusSample(4_350_000_000, 2, 4, True, False))
    evidence.setpoints = [
        module.SetpointSample(4_300_000_000, (0.1, 0.8, -2.5), (0.5, 0.5, 0.0)),
        module.SetpointSample(4_400_000_000, (0.6, 0.3, -2.5), unused_velocity),
        module.SetpointSample(4_500_000_000, (0.6, 0.3, -2.5), unused_velocity),
    ]

    metrics = module.validate_evidence(evidence, launch_exit_code=0)

    assert metrics["replanning"]["post_replan_motion_setpoint_count"] == 1


@pytest.mark.parametrize(
    "mutation, message",
    [
        (
            lambda evidence: remove_event(evidence, "blocker_inserted"),
            "inserted event",
        ),
        (
            lambda evidence: remove_event(evidence, "blocker_insertion_started"),
            "insertion started event",
        ),
        (
            lambda evidence: remove_event(evidence, "blocker_replan_confirmed"),
            "replan confirmed event",
        ),
        (
            lambda evidence: setattr(evidence.events[1], "timestamp_ns", 1_500_000_000),
            "after the initial trajectory",
        ),
        (
            lambda evidence: setattr(evidence.trajectories[1], "trajectory_id", 1),
            "new trajectory ID",
        ),
        (
            lambda evidence: setattr(
                evidence.trajectories[0],
                "points",
                (
                    (0.0, 0.0, 2.5),
                    (0.8, 0.8, 2.5),
                    (0.8, 2.2, 2.5),
                    (0.0, 3.0, 2.5),
                ),
            ),
            "initial path does not cross",
        ),
        (
            lambda evidence: setattr(
                evidence.trajectories[1],
                "points",
                ((0.0, 0.8, 2.5), (0.0, 3.0, 2.5)),
            ),
            "replanned path clearance",
        ),
        (
            invalidate_latest_pre_insertion_status,
            "fresh matching planner status",
        ),
        (
            move_replan_status_after_confirmation,
            "replan confirmation lacks a fresh matching planner status",
        ),
        (
            lambda evidence: evidence.collision_pairs.append(
                ("x500_depth_project_0::base_link", "m3_dynamic_blocker::link")
            ),
            "collision",
        ),
        (move_pose_to_blocker, "actual vehicle clearance"),
        (
            lambda evidence: setattr(
                evidence, "land_samples", evidence.land_samples[:2]
            ),
            "landed",
        ),
    ],
)
def test_false_dynamic_success_is_rejected(mutation, message):
    module = load_module()
    evidence = complete_evidence(module)
    mutation(evidence)

    with pytest.raises(RuntimeError, match=message):
        module.validate_evidence(evidence, launch_exit_code=0)


def test_nonzero_launch_exit_is_rejected():
    module = load_module()
    with pytest.raises(RuntimeError, match="launch exit"):
        module.validate_evidence(complete_evidence(module), launch_exit_code=1)


def test_randomized_summary_requires_ten_runs_and_eight_successes():
    module = load_module()
    with pytest.raises(RuntimeError, match="exactly 10"):
        module.validate_randomized_results([{"accepted": True}] * 9)
    with pytest.raises(RuntimeError, match="at least 8"):
        module.validate_randomized_results(
            [{"accepted": index < 7} for index in range(10)]
        )

    summary = module.validate_randomized_results(
        [{"accepted": index < 8} for index in range(10)]
    )
    assert summary["success_count"] == 8
    assert summary["success_rate"] == pytest.approx(0.8)


def test_provenance_hashes_bag_and_runtime_sources(tmp_path):
    module = load_module()
    assert "scripts/analyze_m3_planning.py" in module.RUNTIME_SOURCE_PATHS

    bag = tmp_path / "bag"
    bag.mkdir()
    bag.joinpath("metadata.yaml").write_text("version: 5\n", encoding="utf-8")
    bag.joinpath("data.db3").write_bytes(b"m3 evidence")
    metrics = {}

    module.add_provenance(metrics, bag)

    assert len(metrics["provenance"]["bag_sha256"]) == 64
    assert set(metrics["provenance"]["runtime_source_sha256"]) == set(
        module.RUNTIME_SOURCE_PATHS
    )


def test_rejected_analysis_replaces_stale_success_metrics(tmp_path, monkeypatch):
    module = load_module()
    bag = tmp_path / "bag"
    bag.mkdir()
    bag.joinpath("metadata.yaml").write_text("version: 5\n", encoding="utf-8")
    bag.joinpath("data.db3").write_bytes(b"m3 evidence")
    output = tmp_path / "metrics.json"
    output.write_text('{"accepted": true}\n', encoding="utf-8")
    monkeypatch.setattr(module, "read_bag", lambda _path: complete_evidence(module))

    with pytest.raises(RuntimeError, match="launch exit"):
        module.analyze_and_write(bag, output, launch_exit_code=1)

    metrics = json.loads(output.read_text(encoding="utf-8"))
    assert metrics["accepted"] is False
    assert "launch exit" in metrics["rejection_reason"]
    assert not output.with_name(output.name + ".tmp").exists()


def test_rejected_analysis_records_provenance_failure(tmp_path, monkeypatch):
    module = load_module()
    bag = tmp_path / "bag"
    bag.mkdir()
    output = tmp_path / "metrics.json"
    monkeypatch.setattr(module, "read_bag", lambda _path: complete_evidence(module))
    monkeypatch.setattr(
        module,
        "add_provenance",
        lambda _metrics, _path: (_ for _ in ()).throw(
            RuntimeError("injected provenance failure")
        ),
    )

    with pytest.raises(RuntimeError, match="launch exit"):
        module.analyze_and_write(bag, output, launch_exit_code=1)

    metrics = json.loads(output.read_text(encoding="utf-8"))
    assert metrics["accepted"] is False
    assert "launch exit" in metrics["rejection_reason"]
    assert metrics["provenance_error"] == "injected provenance failure"
    assert not output.with_name(output.name + ".tmp").exists()
