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
RUNNER_PATH = PROJECT_ROOT / "scripts" / "run_m3_randomized_evaluation.py"


def load_module():
    spec = spec_from_file_location("run_m3_randomized_evaluation", RUNNER_PATH)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def accepted_result(index):
    return {
        "index": index,
        "seed": index + 100,
        "bag_sha256": f"hash-{index}",
        "accepted": True,
    }


def create_existing_batch(module, output_root, master_seed=20260822):
    scenarios = module.generate_scenarios(master_seed)
    output_root.mkdir()
    module.atomic_write_json(
        output_root / "manifest.json",
        module.build_manifest(master_seed, scenarios),
    )
    module.atomic_write_json(output_root / "summary.json", {"accepted": False})
    module.atomic_write_json(output_root / "results.json", [])
    (output_root / "results.csv").write_text("index,accepted\n", encoding="utf-8")
    for scenario in scenarios:
        run_directory = output_root / f"run_{scenario.index:02d}_seed_{scenario.seed}"
        bag_path = run_directory / "bag"
        bag_path.mkdir(parents=True)
        module.atomic_write_json(run_directory / "scenario.json", vars(scenario))
        (run_directory / "launch.exitcode").write_text(
            "1\n" if scenario.index == 7 else "0\n", encoding="ascii"
        )
        (bag_path / "metadata.yaml").write_text(
            "rosbag2_bagfile_information:\n"
            "  duration:\n"
            "    nanoseconds: 1000000000\n",
            encoding="utf-8",
        )
        (bag_path / "data.db3").write_bytes(f"bag-{scenario.index}".encode())
        module.atomic_write_json(
            run_directory / "m3_metrics.json",
            {
                "accepted": False,
                "provenance": {
                    "bag_sha256": module.analyzer.sha256_directory(bag_path)
                },
            },
        )
        module.atomic_write_json(run_directory / "result.json", {"accepted": False})
    return scenarios


def test_scenarios_are_ten_deterministic_bounded_and_unique():
    module = load_module()

    first = module.generate_scenarios(20260822)
    second = module.generate_scenarios(20260822)

    assert first == second
    assert len(first) == module.RUN_COUNT == 10
    assert len({scenario.seed for scenario in first}) == 10
    assert (
        len({(scenario.blocker_x_m, scenario.blocker_y_m) for scenario in first}) == 10
    )
    for scenario in first:
        assert (
            module.BLOCKER_X_RANGE_M[0]
            <= scenario.blocker_x_m
            <= module.BLOCKER_X_RANGE_M[1]
        )
        assert (
            module.BLOCKER_Y_RANGE_M[0]
            <= scenario.blocker_y_m
            <= module.BLOCKER_Y_RANGE_M[1]
        )
        assert (
            abs(scenario.blocker_x_m)
            < module.VEHICLE_RADIUS_M + module.BLOCKER_RADIUS_M
        )
        assert (
            scenario.blocker_y_m - scenario.trigger_progress_m
            > module.MINIMUM_INSERTION_LEAD_M
        )


def test_launch_command_binds_pose_trigger_and_exact_bag_path(tmp_path):
    module = load_module()
    scenario = module.generate_scenarios(20260822)[0]
    bag_path = tmp_path / "run" / "bag"

    command = module.launch_command(scenario, bag_path)

    assert command[:4] == ["ros2", "launch", "drone_bringup", "m3_autonomy.launch.py"]
    assert f"blocker_x_m:={scenario.blocker_x_m:.3f}" in command
    assert f"blocker_y_m:={scenario.blocker_y_m:.3f}" in command
    assert f"trigger_progress_m:={scenario.trigger_progress_m:.3f}" in command
    assert f"bag_directory:={bag_path.resolve()}" in command


def test_formal_summary_requires_exactly_ten_runs_and_eight_successes():
    module = load_module()
    with pytest.raises(RuntimeError, match="exactly 10"):
        module.build_summary([accepted_result(index) for index in range(1, 10)])
    with pytest.raises(RuntimeError, match="at least 8"):
        module.build_summary(
            [
                {**accepted_result(index), "accepted": index <= 7}
                for index in range(1, 11)
            ]
        )

    summary = module.build_summary(
        [{**accepted_result(index), "accepted": index <= 8} for index in range(1, 11)]
    )
    assert summary["accepted"] is True
    assert summary["run_count"] == 10
    assert summary["success_count"] == 8


def test_atomic_json_replaces_stale_content(tmp_path):
    module = load_module()
    output = tmp_path / "result.json"
    output.write_text('{"accepted": true}\n', encoding="utf-8")

    module.atomic_write_json(output, {"accepted": False})

    assert json.loads(output.read_text(encoding="utf-8")) == {"accepted": False}
    assert not output.with_name(output.name + ".tmp").exists()


def test_incomplete_bag_is_recorded_without_reading_missing_metadata(tmp_path):
    module = load_module()
    scenario = module.generate_scenarios(20260822)[0]
    incomplete_bag = tmp_path / "bag"
    incomplete_bag.mkdir()

    result = module.result_from_metrics(
        scenario,
        incomplete_bag,
        launch_exit_code=1,
        metrics={"accepted": False, "rejection_reason": "incomplete bag"},
        timed_out=False,
    )

    assert result["accepted"] is False
    assert result["bag_duration_s"] is None
    assert result["failure_reason"] == "incomplete bag"


def test_nonzero_analysis_exit_cannot_be_accepted(tmp_path):
    module = load_module()
    scenario = module.generate_scenarios(20260822)[0]
    bag_path = tmp_path / "bag"
    bag_path.mkdir()

    result = module.result_from_metrics(
        scenario,
        bag_path,
        launch_exit_code=0,
        metrics={"accepted": True},
        timed_out=False,
        analysis_exit_code=2,
    )

    assert result["accepted"] is False
    assert result["failure_reason"] == "analysis exited with code 2"


def test_analyze_run_rewrites_accepted_metrics_after_nonzero_exit(
    tmp_path, monkeypatch
):
    module = load_module()
    scenario = module.generate_scenarios(20260822)[0]
    metrics_path = tmp_path / "metrics.json"
    module.atomic_write_json(metrics_path, {"accepted": True})
    monkeypatch.setattr(module, "run_process", lambda *_args: (2, False))

    return_code, metrics = module.analyze_run(
        tmp_path,
        tmp_path / "bag",
        metrics_path,
        launch_exit_code=0,
        log_path=tmp_path / "analysis.log",
        scenario=scenario,
    )

    assert return_code == 2
    assert metrics["accepted"] is False
    assert metrics["rejection_reason"] == "analysis exited with code 2"
    assert json.loads(metrics_path.read_text(encoding="utf-8")) == metrics


def test_reanalysis_processes_all_ten_runs_and_preserves_real_failure(
    tmp_path, monkeypatch
):
    module = load_module()
    output_root = tmp_path / "batch"
    scenarios = create_existing_batch(module, output_root)
    analyzed = []

    def fake_analyze(
        _project_root,
        bag_path,
        metrics_path,
        launch_exit_code,
        _log_path,
        _scenario,
    ):
        analyzed.append(bag_path.parent.name)
        accepted = launch_exit_code == 0
        metrics = {
            "accepted": accepted,
            "rejection_reason": None
            if accepted
            else "top-level launch exit code was 1",
            "provenance": {"bag_sha256": module.analyzer.sha256_directory(bag_path)},
        }
        module.atomic_write_json(metrics_path, metrics)
        return (0 if accepted else 1), metrics

    monkeypatch.setattr(module, "analyze_run", fake_analyze)
    summary = module.reanalyze_existing_runs(
        tmp_path, output_root, scenarios, master_seed=20260822
    )

    assert len(analyzed) == module.RUN_COUNT
    assert analyzed == [
        f"run_{scenario.index:02d}_seed_{scenario.seed}" for scenario in scenarios
    ]
    assert summary["accepted"] is True
    assert summary["success_count"] == 9
    assert summary["failure_count"] == 1
    pointer = json.loads(
        (output_root / "current_reanalysis.json").read_text(encoding="utf-8")
    )
    analysis_root = output_root / pointer["relative_analysis_root"]
    assert (
        json.loads(
            (analysis_root / "run_07_seed_309474163" / "result.json").read_text()
        )["accepted"]
        is False
    )
    assert json.loads((output_root / "summary.json").read_text())["accepted"] is False
    assert (analysis_root / "summary.json").is_file()


def test_reanalysis_rejects_manifest_mismatch_before_analyzing(tmp_path, monkeypatch):
    module = load_module()
    output_root = tmp_path / "batch"
    scenarios = create_existing_batch(module, output_root)
    manifest_path = output_root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["scenarios"][0]["seed"] += 1
    module.atomic_write_json(manifest_path, manifest)
    called = False

    def fake_analyze(*_args):
        nonlocal called
        called = True

    monkeypatch.setattr(module, "analyze_run", fake_analyze)
    with pytest.raises(RuntimeError, match="manifest"):
        module.reanalyze_existing_runs(
            tmp_path, output_root, scenarios, master_seed=20260822
        )
    assert called is False


def test_reanalysis_rejects_changed_bag_before_analyzing(tmp_path, monkeypatch):
    module = load_module()
    output_root = tmp_path / "batch"
    scenarios = create_existing_batch(module, output_root)
    first_bag = output_root / f"run_01_seed_{scenarios[0].seed}" / "bag" / "data.db3"
    first_bag.write_bytes(b"changed")
    called = False

    def fake_analyze(*_args):
        nonlocal called
        called = True

    monkeypatch.setattr(module, "analyze_run", fake_analyze)
    with pytest.raises(RuntimeError, match="bag hash changed"):
        module.reanalyze_existing_runs(
            tmp_path, output_root, scenarios, master_seed=20260822
        )
    assert called is False


@pytest.mark.parametrize("missing", ["run", "bag", "exitcode"])
def test_reanalysis_rejects_incomplete_run_set(tmp_path, missing):
    module = load_module()
    output_root = tmp_path / "batch"
    scenarios = create_existing_batch(module, output_root)
    first = output_root / f"run_01_seed_{scenarios[0].seed}"
    if missing == "run":
        first.rename(output_root / "run_11_seed_999")
    elif missing == "bag":
        (first / "bag" / "data.db3").unlink()
    else:
        (first / "launch.exitcode").unlink()

    with pytest.raises(RuntimeError):
        module.reanalyze_existing_runs(
            tmp_path, output_root, scenarios, master_seed=20260822
        )


def test_reanalysis_rejects_an_eleventh_run(tmp_path):
    module = load_module()
    output_root = tmp_path / "batch"
    scenarios = create_existing_batch(module, output_root)
    (output_root / "run_11_seed_999").mkdir()

    with pytest.raises(RuntimeError, match="exactly the expected 10"):
        module.reanalyze_existing_runs(
            tmp_path, output_root, scenarios, master_seed=20260822
        )


def test_failed_reanalysis_publish_leaves_canonical_evidence_and_allows_retry(
    tmp_path, monkeypatch
):
    module = load_module()
    output_root = tmp_path / "batch"
    scenarios = create_existing_batch(module, output_root)

    def fake_analyze(
        _project_root,
        bag_path,
        metrics_path,
        _launch_exit_code,
        _log_path,
        _scenario,
    ):
        metrics = {
            "accepted": True,
            "provenance": {"bag_sha256": module.analyzer.sha256_directory(bag_path)},
        }
        module.atomic_write_json(metrics_path, metrics)
        return 0, metrics

    monkeypatch.setattr(module, "analyze_run", fake_analyze)
    original_writer = module.write_results_csv
    monkeypatch.setattr(
        module,
        "write_results_csv",
        lambda *_args: (_ for _ in ()).throw(OSError("injected write failure")),
    )
    with pytest.raises(OSError, match="injected write failure"):
        module.reanalyze_existing_runs(
            tmp_path, output_root, scenarios, master_seed=20260822
        )
    assert json.loads((output_root / "summary.json").read_text()) == {"accepted": False}
    assert not (output_root / "current_reanalysis.json").exists()

    monkeypatch.setattr(module, "write_results_csv", original_writer)
    summary = module.reanalyze_existing_runs(
        tmp_path, output_root, scenarios, master_seed=20260822
    )
    assert summary["accepted"] is True
