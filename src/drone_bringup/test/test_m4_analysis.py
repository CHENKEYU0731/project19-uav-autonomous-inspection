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
from pathlib import Path

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[3]
ANALYZER_PATH = PROJECT_ROOT / "scripts" / "analyze_m4_mission.py"


def load_module():
    spec = spec_from_file_location("analyze_m4_mission", ANALYZER_PATH)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def complete_evidence(module):
    m3 = module.m3
    blocker_events = [
        m3.BlockerEvent(100, "blocker_insertion_started", 0.0, 1.5, 1, 1),
        m3.BlockerEvent(110, "blocker_inserted", 0.0, 1.5, 1, 1),
        m3.BlockerEvent(120, "blocker_replan_confirmed", 0.0, 1.5, 2, 2),
        m3.BlockerEvent(300, "blocker_removal_started", 0.0, 1.5, 2, 2),
        m3.BlockerEvent(320, "blocker_removed", 0.0, -3.0, 2, 2),
    ]
    planning = m3.PlanningEvidence(
        trajectories=[
            m3.TrajectorySample(90, 1, ((0.0, 0.0, 2.5), (0.0, 3.0, 2.5))),
            m3.TrajectorySample(
                115,
                2,
                ((0.0, 0.0, 2.5), (1.0, 1.5, 2.5), (0.0, 3.0, 2.5)),
            ),
        ],
        statuses=[
            m3.StatusSample(95, 1, m3.READY, True, True),
            m3.StatusSample(119, 2, m3.READY, True, True),
        ],
        events=blocker_events,
        poses=[
            m3.PoseSample(110, 0.0, 0.5, 2.5),
            m3.PoseSample(200, 1.0, 1.5, 2.5),
            m3.PoseSample(300, 0.8, 2.3, 2.5),
            m3.PoseSample(390, 0.0, 3.0, 2.5),
            m3.PoseSample(490, 1.0, 2.5, 2.5),
            m3.PoseSample(590, 0.0, 3.0, 2.5),
            m3.PoseSample(690, 0.0, 0.0, 2.5),
        ],
        setpoints=[m3.SetpointSample(130, (0.0, 0.0, -2.5), (0.1, 0.0, 0.0))],
        vehicle_statuses=[
            m3.VehicleStatusSample(
                50, m3.ARMING_STATE_ARMED, m3.NAVIGATION_STATE_OFFBOARD, False
            ),
            m3.VehicleStatusSample(950, m3.ARMING_STATE_DISARMED, 0, False),
        ],
        land_samples=[m3.LandSample(60, False), m3.LandSample(900, True)],
        collision_pairs=[],
        contacts_topic_present=True,
    )
    event_specs = [
        (10, "mission_started", module.STANDBY, module.TAKEOFF, -1),
        (70, "takeoff_completed", module.TAKEOFF, module.INSPECTING, 0),
        (400, "waypoint_reached", module.INSPECTING, module.INSPECTING, 1),
        (420, "waypoint_unreachable", module.INSPECTING, module.HANDLING_EXCEPTION, 1),
        (430, "waypoint_skipped", module.HANDLING_EXCEPTION, module.INSPECTING, 2),
        (500, "waypoint_reached", module.INSPECTING, module.INSPECTING, 3),
        (510, "low_battery", module.INSPECTING, module.HANDLING_EXCEPTION, -1),
        (
            511,
            "inspection_interrupted",
            module.HANDLING_EXCEPTION,
            module.RETURNING_HOME,
            -1,
        ),
        (
            600,
            "return_waypoint_reached",
            module.RETURNING_HOME,
            module.RETURNING_HOME,
            0,
        ),
        (700, "home_reached", module.RETURNING_HOME, module.LANDING, -1),
        (1000, "landing_completed", module.LANDING, module.COMPLETE, -1),
    ]
    mission_events = [
        module.MissionEventSample(
            timestamp, sequence, source, target, waypoint, name, "reason"
        )
        for sequence, (timestamp, name, source, target, waypoint) in enumerate(
            event_specs, 1
        )
    ]
    return module.MissionEvidence(
        planning,
        mission_events,
        [module.MissionCommandSample(710, module.LAND_COMMAND)],
        [
            module.PlannerGoalSample(71, 0.0, 3.0, 2.5, "map"),
            module.PlannerGoalSample(401, 0.0, 6.0, 2.5, "map"),
            module.PlannerGoalSample(431, 1.0, 2.5, 2.5, "map"),
            module.PlannerGoalSample(501, 1.0, 1.0, 2.5, "map"),
            module.PlannerGoalSample(512, 0.0, 3.0, 2.5, "map"),
            module.PlannerGoalSample(601, 0.0, 0.0, 2.5, "map"),
        ],
    )


def remove_mission_event(evidence, event_name):
    evidence.mission_events = [
        event for event in evidence.mission_events if event.event != event_name
    ]
    for sequence, event in enumerate(evidence.mission_events, 1):
        event.sequence = sequence


def insert_normal_completion(module, evidence):
    evidence.mission_events.append(
        module.MissionEventSample(
            505,
            0,
            module.INSPECTING,
            module.RETURNING_HOME,
            -1,
            "inspection_completed",
            "reason",
        )
    )
    evidence.mission_events.sort(key=lambda event: event.timestamp_ns)
    for sequence, event in enumerate(evidence.mission_events, 1):
        event.sequence = sequence


def rewind_unreachable_progress(_module, evidence):
    evidence.mission_events[3].waypoint_index = 0
    evidence.mission_events[4].waypoint_index = 1


def test_complete_m4_evidence_passes_all_gates():
    module = load_module()
    metrics = module.validate_evidence(complete_evidence(module), launch_exit_code=0)

    assert metrics["accepted"] is True
    assert metrics["mission"]["reached_waypoint_count"] == 2
    assert metrics["planner_goals"]["return_waypoint_count"] == 1
    assert metrics["dynamic_avoidance"]["minimum_actual_clearance_m"] > 0.0
    assert metrics["landing"]["landed_and_disarmed"] is True
    assert metrics["safety"]["vehicle_collision_count"] == 0


def test_cross_topic_receive_order_may_invert_within_one_millisecond():
    module = load_module()
    evidence = complete_evidence(module)
    evidence.planner_goals[1].timestamp_ns = evidence.mission_events[2].timestamp_ns - 1

    metrics = module.validate_evidence(evidence, launch_exit_code=0)

    assert metrics["accepted"] is True


def test_cross_topic_receive_inversion_beyond_tolerance_is_rejected():
    module = load_module()
    evidence = complete_evidence(module)
    evidence.planner_goals[1].timestamp_ns = (
        evidence.mission_events[2].timestamp_ns
        - module.GOAL_RECEIVE_REORDER_TOLERANCE_NS
        - 1
    )

    with pytest.raises(RuntimeError, match="planner goal"):
        module.validate_evidence(evidence, launch_exit_code=0)


@pytest.mark.parametrize(
    "mutation, expected",
    [
        (lambda _m, e: setattr(e.mission_events[1], "sequence", 9), "contiguous"),
        (
            lambda _m, e: remove_mission_event(e, "waypoint_unreachable"),
            "waypoint_unreachable",
        ),
        (
            lambda _m, e: remove_mission_event(e, "low_battery"),
            "low_battery",
        ),
        (
            lambda _m, e: remove_mission_event(e, "waypoint_reached"),
            "fewer than two",
        ),
        (
            lambda _m, e: setattr(e.mission_events[5], "waypoint_index", 1),
            "strictly increasing",
        ),
        (
            rewind_unreachable_progress,
            "progression is not contiguous",
        ),
        (
            insert_normal_completion,
            "cannot also complete inspection",
        ),
        (
            lambda _m, e: e.planner_goals.clear(),
            "no planner goals",
        ),
        (
            lambda _m, e: setattr(e.planner_goals[0], "frame_id", "odom"),
            "map frame",
        ),
        (
            lambda _m, e: setattr(e.planner_goals[1], "x", 9.0),
            "configured waypoint",
        ),
        (
            lambda _m, e: e.planner_goals.pop(2),
            "waypoint_skipped was not followed",
        ),
        (
            lambda _m, e: e.planner_goals.pop(),
            "not followed by a home goal",
        ),
        (
            lambda _m, e: e.planner_goals.pop(-2),
            "inspection_interrupted was not followed by a return goal",
        ),
        (
            lambda _m, e: setattr(e.planner_goals[-2], "x", 9.0),
            "return planner goal does not match",
        ),
        (
            lambda _m, e: setattr(e.planner_goals[-1], "x", 9.0),
            "configured home",
        ),
        (
            lambda _m, e: setattr(e.planning.poses[3], "x", 9.0),
            "waypoint_reached vehicle pose",
        ),
        (
            lambda _m, e: setattr(e.planning.poses[3], "x", float("nan")),
            "waypoint_reached vehicle pose is non-finite",
        ),
        (
            lambda _m, e: setattr(e.planning.poses[-1], "x", 9.0),
            "home_reached vehicle pose",
        ),
        (
            lambda _m, e: setattr(e.planning.poses[-1], "z", float("nan")),
            "home_reached vehicle pose is non-finite",
        ),
        (
            lambda _m, e: setattr(e.mission_commands[0], "timestamp_ns", 699),
            "before home",
        ),
        (
            lambda _m, e: e.planning.events.pop(4),
            "blocker_removed",
        ),
        (
            lambda _m, e: e.planning.collision_pairs.append(
                ("x500_depth_project_0::base_link", "warehouse_rack::link")
            ),
            "collision",
        ),
        (
            lambda _m, e: setattr(e.planning, "contacts_topic_present", False),
            "collision evidence topic",
        ),
        (
            lambda m, e: setattr(
                e.planning.vehicle_statuses[-1],
                "arming_state",
                m.m3.ARMING_STATE_ARMED,
            ),
            "landed -> disarmed",
        ),
        (
            lambda _m, e: e.planning.statuses.clear(),
            "fresh matching planner status",
        ),
        (
            lambda _m, e: e.planning.setpoints.clear(),
            "no post-replan motion",
        ),
        (
            lambda _m, e: setattr(e.planning.events[1], "trajectory_id", 99),
            "matching safe trajectory ID",
        ),
        (
            lambda _m, e: setattr(e.planning.trajectories[1], "timestamp_ns", 121),
            "confirmation preceded",
        ),
        (
            lambda _m, e: setattr(e.planning.statuses[1], "timestamp_ns", 121),
            "replan confirmation lacks a fresh matching planner status",
        ),
        (
            lambda _m, e: setattr(e.mission_commands[0], "timestamp_ns", 920),
            "required home",
        ),
        (
            lambda _m, e: setattr(e.planning.land_samples[1], "timestamp_ns", 650),
            "required home",
        ),
    ],
)
def test_missing_or_invalid_m4_evidence_is_rejected(mutation, expected):
    module = load_module()
    evidence = complete_evidence(module)
    mutation(module, evidence)

    with pytest.raises(RuntimeError, match=expected):
        module.validate_evidence(evidence, launch_exit_code=0)


def test_nonzero_launch_exit_is_rejected():
    module = load_module()
    with pytest.raises(RuntimeError, match="launch exit"):
        module.validate_evidence(complete_evidence(module), launch_exit_code=1)


@pytest.mark.parametrize(
    "removed", [(0.0, 1.500001), (0.0, 2.0), (float("nan"), float("nan"))]
)
def test_blocker_must_be_removed_to_the_recorded_safe_pose(removed):
    module = load_module()
    evidence = complete_evidence(module)
    evidence.planning.events[-1].x, evidence.planning.events[-1].y = removed

    with pytest.raises(RuntimeError, match="expected safe pose"):
        module.validate_evidence(evidence, launch_exit_code=0)


def test_provenance_hashes_every_runtime_dependency(tmp_path):
    module = load_module()
    bag = tmp_path / "bag"
    bag.mkdir()
    bag.joinpath("metadata.yaml").write_text("version: 5\n", encoding="utf-8")
    bag.joinpath("data.db3").write_bytes(b"m4 evidence")
    metrics = {}

    module.add_provenance(metrics, bag)

    assert "scripts/analyze_m3_planning.py" in module.RUNTIME_SOURCE_PATHS
    assert set(metrics["provenance"]["runtime_source_sha256"]) == set(
        module.RUNTIME_SOURCE_PATHS
    )


def test_rejected_analysis_records_provenance_failure(tmp_path, monkeypatch):
    module = load_module()
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
        module.analyze_and_write(tmp_path / "bag", output, launch_exit_code=1)

    metrics = json.loads(output.read_text(encoding="utf-8"))
    assert metrics["accepted"] is False
    assert "launch exit" in metrics["rejection_reason"]
    assert metrics["provenance_error"] == "injected provenance failure"
    assert not list(tmp_path.glob(f".{output.name}-*"))
