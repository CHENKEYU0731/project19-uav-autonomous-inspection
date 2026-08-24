#!/usr/bin/env python3

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

"""Run and independently verify ten reproducible M3 SITL scenarios."""

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import sys
import time

import yaml


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
if str(SCRIPT_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIRECTORY))

import analyze_m3_planning as analyzer  # noqa: E402
import probe_m3_connectivity as connectivity  # noqa: E402


RUN_COUNT = 10
MINIMUM_SUCCESS_COUNT = 8
DEFAULT_MASTER_SEED = 20260822
BLOCKER_X_RANGE_M = (-0.18, 0.18)
BLOCKER_Y_RANGE_M = (1.45, 1.75)
TRIGGER_PROGRESS_RANGE_M = (0.45, 0.65)
MINIMUM_INSERTION_LEAD_M = 0.60
BLOCKER_RADIUS_M = 0.22
VEHICLE_RADIUS_M = 0.35
INFLATION_RADIUS_M = 0.50
GOAL_X_M = 0.0
GOAL_Y_M = 3.0
MINIMUM_ALTITUDE_M = 2.3


@dataclass(frozen=True)
class Scenario:
    index: int
    seed: int
    blocker_x_m: float
    blocker_y_m: float
    trigger_progress_m: float


def parse_args():
    default_output = (
        Path("log") / "m3" / f"randomized_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    parser = argparse.ArgumentParser(
        description="Run the formal ten-scenario M3 randomized evaluation."
    )
    parser.add_argument("--output-root", type=Path, default=default_output)
    parser.add_argument("--master-seed", type=int, default=DEFAULT_MASTER_SEED)
    parser.add_argument("--topology-bag", type=Path)
    parser.add_argument(
        "--topology-config",
        type=Path,
        default=Path("src/drone_bringup/config/m3_mapping.yaml"),
    )
    parser.add_argument("--launch-timeout-s", type=float, default=240.0)
    parser.add_argument("--cooldown-s", type=float, default=3.0)
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="Generate and topology-check all scenarios without launching SITL.",
    )
    parser.add_argument(
        "--reanalyze-only",
        action="store_true",
        help="Uniformly reanalyze an existing complete ten-run batch without flying.",
    )
    return parser.parse_args()


def generate_scenarios(master_seed):
    master = random.Random(master_seed)
    seeds = master.sample(range(1, 2**31), RUN_COUNT)
    scenarios = []
    for index, seed in enumerate(seeds, start=1):
        scenario_random = random.Random(seed)
        scenario = Scenario(
            index=index,
            seed=seed,
            blocker_x_m=round(scenario_random.uniform(*BLOCKER_X_RANGE_M), 3),
            blocker_y_m=round(scenario_random.uniform(*BLOCKER_Y_RANGE_M), 3),
            trigger_progress_m=round(
                scenario_random.uniform(*TRIGGER_PROGRESS_RANGE_M), 3
            ),
        )
        validate_scenario_geometry(scenario)
        scenarios.append(scenario)
    return scenarios


def validate_scenario_geometry(scenario):
    values = (
        scenario.blocker_x_m,
        scenario.blocker_y_m,
        scenario.trigger_progress_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("scenario geometry contains non-finite values")
    if not BLOCKER_X_RANGE_M[0] <= scenario.blocker_x_m <= BLOCKER_X_RANGE_M[1]:
        raise RuntimeError("scenario blocker x lies outside the bounded range")
    if not BLOCKER_Y_RANGE_M[0] <= scenario.blocker_y_m <= BLOCKER_Y_RANGE_M[1]:
        raise RuntimeError("scenario blocker y lies outside the bounded range")
    if not (
        TRIGGER_PROGRESS_RANGE_M[0]
        <= scenario.trigger_progress_m
        <= TRIGGER_PROGRESS_RANGE_M[1]
    ):
        raise RuntimeError("scenario trigger progress lies outside the bounded range")
    if abs(scenario.blocker_x_m) >= VEHICLE_RADIUS_M + BLOCKER_RADIUS_M:
        raise RuntimeError(
            "scenario blocker does not intersect the nominal initial path"
        )
    if scenario.blocker_y_m - scenario.trigger_progress_m <= MINIMUM_INSERTION_LEAD_M:
        raise RuntimeError("scenario has no safe dynamic-insertion window")


def atomic_write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def build_manifest(master_seed, scenarios):
    return {
        "formal_run_count": RUN_COUNT,
        "master_seed": master_seed,
        "minimum_success_count": MINIMUM_SUCCESS_COUNT,
        "bounds": {
            "blocker_x_range_m": list(BLOCKER_X_RANGE_M),
            "blocker_y_range_m": list(BLOCKER_Y_RANGE_M),
            "trigger_progress_range_m": list(TRIGGER_PROGRESS_RANGE_M),
            "minimum_insertion_lead_m": MINIMUM_INSERTION_LEAD_M,
        },
        "scenarios": [asdict(scenario) for scenario in scenarios],
    }


def topology_preflight(scenarios, bag_path, config_path):
    if not bag_path.is_dir() or not config_path.is_file():
        raise RuntimeError("topology preflight inputs are missing")
    configuration = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    evidence = connectivity.m2.read_bag(bag_path)
    runtime_contract = connectivity.validate_runtime_mapping_contract(
        evidence, configuration
    )
    grids = sorted(evidence.grids, key=lambda grid: grid.timestamp_ns)
    poses = sorted(evidence.poses, key=lambda pose: pose.timestamp_ns)
    if not grids or not poses:
        raise RuntimeError("topology bag contains no grids or poses")
    pose_times = [pose.timestamp_ns for pose in poses]
    aligned_poses = [
        connectivity.m2.nearest_pose(poses, pose_times, grid.timestamp_ns)
        for grid in grids
    ]
    maximum_pose_offset_ns = max(offset for _pose, offset in aligned_poses)
    if maximum_pose_offset_ns > connectivity.m2.MAX_TF_OFFSET_NS:
        raise RuntimeError("topology bag exceeds the TF alignment limit")

    fused = connectivity.FusedGrid.from_sample(grids[0])
    pending = {scenario.index: scenario for scenario in scenarios}
    connected_results = {}
    for grid_sample, (pose, pose_offset_ns) in zip(grids, aligned_poses):
        fused.integrate(grid_sample)
        if pose.z < MINIMUM_ALTITUDE_M:
            continue
        for scenario_index, scenario in list(pending.items()):
            candidate = fused.copy()
            candidate.clear_disk(pose.x, pose.y, VEHICLE_RADIUS_M)
            candidate.add_disk_obstacle(
                scenario.blocker_x_m,
                scenario.blocker_y_m,
                BLOCKER_RADIUS_M,
            )
            path = connectivity.find_path(
                candidate,
                (pose.x, pose.y),
                (GOAL_X_M, GOAL_Y_M),
                INFLATION_RADIUS_M,
            )
            if not path:
                continue
            world_points = connectivity._path_world_points(candidate, path)
            clearance = connectivity._minimum_polyline_clearance(
                world_points,
                (scenario.blocker_x_m, scenario.blocker_y_m),
                BLOCKER_RADIUS_M,
            )
            if clearance <= VEHICLE_RADIUS_M:
                continue
            connected_results[scenario_index] = {
                "index": scenario.index,
                "seed": scenario.seed,
                "path_cell_count": len(path),
                "path_length_m": connectivity._path_length(world_points),
                "minimum_blocker_clearance_m": clearance,
                "map_timestamp_ns": int(grid_sample.timestamp_ns),
                "pose_offset_ms": pose_offset_ns / 1_000_000.0,
            }
            del pending[scenario_index]
        if not pending:
            break
    if pending:
        missing = ", ".join(str(index) for index in sorted(pending))
        raise RuntimeError(f"scenarios have no conservative topology detour: {missing}")
    results = [connected_results[index] for index in sorted(connected_results)]
    return {
        "bag_path": str(bag_path.resolve()),
        "bag_sha256": analyzer.sha256_directory(bag_path),
        "maximum_pose_offset_ms": maximum_pose_offset_ns / 1_000_000.0,
        "runtime_mapping_contract": runtime_contract,
        "scenarios": results,
    }


def launch_command(scenario, bag_path):
    return [
        "ros2",
        "launch",
        "drone_bringup",
        "m3_autonomy.launch.py",
        f"blocker_x_m:={scenario.blocker_x_m:.3f}",
        f"blocker_y_m:={scenario.blocker_y_m:.3f}",
        f"trigger_progress_m:={scenario.trigger_progress_m:.3f}",
        f"minimum_insertion_lead_m:={MINIMUM_INSERTION_LEAD_M:.3f}",
        f"bag_directory:={bag_path.resolve()}",
    ]


def run_process(command, log_path, timeout_s):
    log_path.parent.mkdir(parents=True, exist_ok=True)
    timed_out = False
    with log_path.open("w", encoding="utf-8") as log_file:
        process = subprocess.Popen(
            command,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        try:
            return_code = process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            timed_out = True
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=10.0)
            return_code = 124
    return return_code, timed_out


def wait_for_bag(bag_path, timeout_s=15.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if (bag_path / "metadata.yaml").is_file() and any(bag_path.glob("*.db3")):
            return True
        time.sleep(0.25)
    return False


def read_bag_duration_s(bag_path):
    metadata = yaml.safe_load((bag_path / "metadata.yaml").read_text(encoding="utf-8"))
    duration_ns = metadata["rosbag2_bagfile_information"]["duration"]["nanoseconds"]
    return int(duration_ns) / 1e9


def analyze_run(
    project_root,
    bag_path,
    metrics_path,
    launch_exit_code,
    log_path,
    scenario,
):
    command = [
        sys.executable,
        str(project_root / "scripts" / "analyze_m3_planning.py"),
        str(bag_path),
        "--metrics",
        str(metrics_path),
        "--launch-exit-code",
        str(launch_exit_code),
        "--expected-blocker-x-m",
        str(scenario.blocker_x_m),
        "--expected-blocker-y-m",
        str(scenario.blocker_y_m),
    ]
    return_code, timed_out = run_process(command, log_path, 180.0)
    if timed_out:
        return return_code, {
            "accepted": False,
            "rejection_reason": "analysis timed out",
        }
    if not metrics_path.is_file():
        return return_code, {
            "accepted": False,
            "rejection_reason": "analysis produced no metrics file",
        }
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    if return_code != 0 and metrics.get("accepted"):
        metrics["accepted"] = False
        metrics["rejection_reason"] = f"analysis exited with code {return_code}"
        atomic_write_json(metrics_path, metrics)
    return return_code, metrics


def result_from_metrics(
    scenario,
    bag_path,
    launch_exit_code,
    metrics,
    timed_out,
    analysis_exit_code=0,
):
    replanning = metrics.get("replanning", {})
    clearance = metrics.get("clearance", {})
    flight = metrics.get("flight", {})
    accepted = (
        bool(metrics.get("accepted"))
        and launch_exit_code == 0
        and analysis_exit_code == 0
        and not timed_out
    )
    if not accepted and metrics.get("accepted") and analysis_exit_code != 0:
        failure_reason = f"analysis exited with code {analysis_exit_code}"
    else:
        failure_reason = metrics.get(
            "rejection_reason",
            "launch timed out" if timed_out else "run was not accepted",
        )
    return {
        **asdict(scenario),
        "accepted": accepted,
        "failure_reason": None if accepted else failure_reason,
        "launch_exit_code": launch_exit_code,
        "launch_timed_out": timed_out,
        "analysis_exit_code": analysis_exit_code,
        "bag_path": str(bag_path.resolve()),
        "bag_duration_s": read_bag_duration_s(bag_path)
        if (bag_path / "metadata.yaml").is_file()
        else None,
        "initial_trajectory_id": replanning.get("initial_trajectory_id"),
        "replanned_trajectory_id": replanning.get("replanned_trajectory_id"),
        "replan_latency_s": replanning.get("replan_latency_s"),
        "minimum_actual_clearance_m": clearance.get("minimum_actual_clearance_m"),
        "goal_error_m": flight.get("goal_error_m"),
        "landed_and_disarmed": flight.get("landed_and_disarmed", False),
        "failsafe_observed": flight.get("failsafe_observed"),
        "bag_sha256": metrics.get("provenance", {}).get("bag_sha256"),
    }


def build_summary(results):
    if len(results) != RUN_COUNT:
        raise RuntimeError("randomized evaluation requires exactly 10 runs")
    if [result.get("index") for result in results] != list(range(1, RUN_COUNT + 1)):
        raise RuntimeError(
            "randomized evaluation requires ordered run indexes 1 through 10"
        )
    seeds = [result.get("seed") for result in results]
    if None in seeds or len(set(seeds)) != RUN_COUNT:
        raise RuntimeError("randomized evaluation requires 10 unique scenario seeds")
    bag_hashes = [result.get("bag_sha256") for result in results]
    if None in bag_hashes or len(set(bag_hashes)) != RUN_COUNT:
        raise RuntimeError("randomized evaluation requires 10 unique bag hashes")
    randomized = analyzer.validate_randomized_results(results)
    return {
        "accepted": True,
        **randomized,
        "required_success_count": MINIMUM_SUCCESS_COUNT,
    }


def write_results_csv(path, results):
    fields = [
        "index",
        "seed",
        "blocker_x_m",
        "blocker_y_m",
        "trigger_progress_m",
        "accepted",
        "failure_reason",
        "launch_exit_code",
        "analysis_exit_code",
        "bag_duration_s",
        "initial_trajectory_id",
        "replanned_trajectory_id",
        "replan_latency_s",
        "minimum_actual_clearance_m",
        "goal_error_m",
        "landed_and_disarmed",
        "failsafe_observed",
        "bag_sha256",
        "bag_path",
    ]
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(results)
    temporary.replace(path)


def summarize_results(results, master_seed, output_root):
    try:
        summary = build_summary(results)
    except RuntimeError as error:
        summary = {
            "accepted": False,
            "run_count": len(results),
            "success_count": sum(bool(result.get("accepted")) for result in results),
            "failure_count": sum(
                not bool(result.get("accepted")) for result in results
            ),
            "required_success_count": MINIMUM_SUCCESS_COUNT,
            "rejection_reason": str(error),
        }
    summary["master_seed"] = master_seed
    summary["output_root"] = str(output_root)
    return summary


def validate_reanalysis_inputs(output_root, manifest, scenarios, master_seed):
    if len(scenarios) != RUN_COUNT:
        raise RuntimeError("reanalysis requires exactly 10 generated scenarios")
    expected_manifest = build_manifest(master_seed, scenarios)
    if manifest != expected_manifest:
        raise RuntimeError(
            "existing manifest does not match the complete fixed scenario set"
        )

    expected_directories = {
        f"run_{scenario.index:02d}_seed_{scenario.seed}" for scenario in scenarios
    }
    actual_directories = {
        path.name for path in output_root.glob("run_*") if path.is_dir()
    }
    if actual_directories != expected_directories:
        raise RuntimeError(
            "existing batch does not contain exactly the expected 10 run directories"
        )

    inputs = []
    bag_hashes = set()
    for scenario in scenarios:
        run_directory = output_root / f"run_{scenario.index:02d}_seed_{scenario.seed}"
        scenario_path = run_directory / "scenario.json"
        exit_code_path = run_directory / "launch.exitcode"
        bag_path = run_directory / "bag"
        if not scenario_path.is_file():
            raise RuntimeError(f"run {scenario.index:02d} is missing scenario.json")
        recorded_scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
        for key, value in asdict(scenario).items():
            if recorded_scenario.get(key) != value:
                raise RuntimeError(
                    f"run {scenario.index:02d} scenario does not match manifest"
                )
        if not exit_code_path.is_file():
            raise RuntimeError(f"run {scenario.index:02d} is missing launch.exitcode")
        try:
            launch_exit_code = int(exit_code_path.read_text(encoding="ascii").strip())
        except ValueError as error:
            raise RuntimeError(
                f"run {scenario.index:02d} has an invalid launch.exitcode"
            ) from error
        if not (bag_path / "metadata.yaml").is_file() or not any(
            bag_path.glob("*.db3")
        ):
            raise RuntimeError(f"run {scenario.index:02d} has no complete bag")
        metrics_path = run_directory / "m3_metrics.json"
        if not metrics_path.is_file():
            raise RuntimeError(f"run {scenario.index:02d} is missing original metrics")
        original_metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        original_bag_hash = original_metrics.get("provenance", {}).get("bag_sha256")
        current_bag_hash = analyzer.sha256_directory(bag_path)
        if original_bag_hash != current_bag_hash:
            raise RuntimeError(
                f"run {scenario.index:02d} bag hash changed after capture"
            )
        if current_bag_hash in bag_hashes:
            raise RuntimeError("existing batch contains duplicate bag evidence")
        bag_hashes.add(current_bag_hash)
        inputs.append((scenario, run_directory, bag_path, launch_exit_code))
    return inputs


def reanalyze_existing_runs(project_root, output_root, scenarios, master_seed):
    if not output_root.is_dir():
        raise RuntimeError("reanalysis output root does not exist")
    manifest_path = output_root / "manifest.json"
    if not manifest_path.is_file():
        raise RuntimeError("existing batch is missing manifest.json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    inputs = validate_reanalysis_inputs(output_root, manifest, scenarios, master_seed)

    version = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    staging_root = output_root / f".reanalysis_stage_{version}"
    final_root = output_root / f"reanalysis_{version}"
    staging_root.mkdir(exist_ok=False)
    staged = []
    for scenario, run_directory, bag_path, launch_exit_code in inputs:
        analysis_directory = staging_root / run_directory.name
        analysis_directory.mkdir()
        metrics_path = analysis_directory / "m3_metrics.json"
        analysis_code, metrics = analyze_run(
            project_root,
            bag_path,
            metrics_path,
            launch_exit_code,
            analysis_directory / "analysis.log",
            scenario,
        )
        if not metrics_path.is_file():
            atomic_write_json(metrics_path, metrics)
        if analysis_code == 0 and not metrics.get("accepted"):
            metrics = {
                "accepted": False,
                "rejection_reason": "analysis exited successfully without accepting evidence",
            }
            atomic_write_json(metrics_path, metrics)
        result = result_from_metrics(
            scenario,
            bag_path,
            launch_exit_code,
            metrics,
            timed_out=launch_exit_code == 124,
            analysis_exit_code=analysis_code,
        )
        atomic_write_json(analysis_directory / "result.json", result)
        staged.append(result)
        print(
            f"[{scenario.index:02d}/{RUN_COUNT}] reanalyzed: "
            f"{'accepted' if result['accepted'] else 'rejected'}",
            flush=True,
        )

    results = staged
    write_results_csv(staging_root / "results.csv", results)
    atomic_write_json(staging_root / "results.json", results)
    atomic_write_json(staging_root / "manifest.json", manifest)
    summary = summarize_results(results, master_seed, final_root)
    summary["reanalyzed"] = True
    summary["source_output_root"] = str(output_root)
    atomic_write_json(staging_root / "summary.json", summary)
    staging_root.replace(final_root)
    atomic_write_json(
        output_root / "current_reanalysis.json",
        {
            "analysis_root": str(final_root),
            "relative_analysis_root": final_root.name,
            "summary_sha256": analyzer.sha256_file(final_root / "summary.json"),
        },
    )
    return summary


def main():
    args = parse_args()
    if args.preflight_only and args.reanalyze_only:
        raise SystemExit("--preflight-only and --reanalyze-only are mutually exclusive")
    if (
        not math.isfinite(args.launch_timeout_s)
        or args.launch_timeout_s <= 0.0
        or not math.isfinite(args.cooldown_s)
        or args.cooldown_s < 0.0
    ):
        raise SystemExit("timeouts must be finite and valid")

    project_root = Path(__file__).resolve().parents[1]
    output_root = args.output_root.resolve()
    scenarios = generate_scenarios(args.master_seed)
    if args.reanalyze_only:
        try:
            summary = reanalyze_existing_runs(
                project_root, output_root, scenarios, args.master_seed
            )
        except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as error:
            raise SystemExit(f"reanalysis rejected: {error}") from error
        print(
            f"reanalyzed formal result: {summary['success_count']}/{RUN_COUNT} accepted; "
            f"output={output_root}",
            flush=True,
        )
        return 0 if summary["accepted"] else 1

    if args.topology_bag is None:
        raise SystemExit("--topology-bag is required unless --reanalyze-only is used")
    output_root.mkdir(parents=True, exist_ok=False)
    manifest = build_manifest(args.master_seed, scenarios)
    atomic_write_json(output_root / "manifest.json", manifest)
    print(f"preflighting {RUN_COUNT} scenarios", flush=True)
    preflight = topology_preflight(
        scenarios, args.topology_bag.resolve(), args.topology_config.resolve()
    )
    atomic_write_json(output_root / "topology_preflight.json", preflight)
    print("all scenarios passed conservative topology preflight", flush=True)
    if args.preflight_only:
        print(f"preflight-only output: {output_root}", flush=True)
        return 0

    results = []
    for scenario in scenarios:
        run_directory = output_root / f"run_{scenario.index:02d}_seed_{scenario.seed}"
        run_directory.mkdir(parents=True, exist_ok=False)
        bag_path = run_directory / "bag"
        command = launch_command(scenario, bag_path)
        atomic_write_json(
            run_directory / "scenario.json",
            {**asdict(scenario), "command": command},
        )
        print(
            f"[{scenario.index:02d}/{RUN_COUNT}] seed={scenario.seed} "
            f"blocker=({scenario.blocker_x_m:.3f}, {scenario.blocker_y_m:.3f})",
            flush=True,
        )
        launch_exit_code, timed_out = run_process(
            command, run_directory / "launch.log", args.launch_timeout_s
        )
        (run_directory / "launch.exitcode").write_text(
            f"{launch_exit_code}\n", encoding="ascii"
        )
        bag_complete = wait_for_bag(bag_path)
        if not bag_complete:
            metrics = {
                "accepted": False,
                "rejection_reason": "launch produced no complete bag",
            }
            atomic_write_json(run_directory / "m3_metrics.json", metrics)
            analysis_code = None
        else:
            analysis_code, metrics = analyze_run(
                project_root,
                bag_path,
                run_directory / "m3_metrics.json",
                launch_exit_code,
                run_directory / "analysis.log",
                scenario,
            )
        result = result_from_metrics(
            scenario,
            bag_path,
            launch_exit_code,
            metrics,
            timed_out,
            analysis_exit_code=analysis_code,
        )
        atomic_write_json(run_directory / "result.json", result)
        results.append(result)
        atomic_write_json(output_root / "results.partial.json", results)
        print(
            f"[{scenario.index:02d}/{RUN_COUNT}] "
            f"{'accepted' if result['accepted'] else 'rejected'}: "
            f"{result['failure_reason'] or 'verified'}",
            flush=True,
        )
        if scenario.index < RUN_COUNT and args.cooldown_s > 0.0:
            time.sleep(args.cooldown_s)

    write_results_csv(output_root / "results.csv", results)
    atomic_write_json(output_root / "results.json", results)
    summary = summarize_results(results, args.master_seed, output_root)
    atomic_write_json(output_root / "summary.json", summary)
    print(
        f"formal result: {summary['success_count']}/{RUN_COUNT} accepted; "
        f"output={output_root}",
        flush=True,
    )
    return 0 if summary["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
