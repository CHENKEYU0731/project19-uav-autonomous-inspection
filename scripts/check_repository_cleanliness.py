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

"""Fail when versioned or working-tree residue violates repository policy."""

import argparse
from pathlib import Path, PurePosixPath
import subprocess


DEFAULT_MAX_TRACKED_BYTES = 10 * 1024 * 1024
PROHIBITED_PARTS = {
    ".cache",
    ".mypy_cache",
    ".nox",
    ".local",
    ".pytest_cache",
    ".ruff_cache",
    ".simulation-gazebo",
    ".tmp",
    ".tox",
    ".venv",
    ".wsl",
    "build",
    "downloads",
    "htmlcov",
    "install",
    "log",
    "venv",
    "__pycache__",
}
PROHIBITED_ROOT_DIRECTORY_PREFIXES = ("build-", "install-")
PROHIBITED_PATH_PREFIXES = (
    PurePosixPath("external/PX4-Autopilot"),
    PurePosixPath("external/Micro-XRCE-DDS-Agent"),
    PurePosixPath("src/px4_msgs"),
    PurePosixPath("src/px4_ros_com"),
)
PROHIBITED_SUFFIXES = (
    ".avi",
    ".bak",
    ".db3",
    ".log",
    ".mcap",
    ".mkv",
    ".mov",
    ".mp4",
    ".pyc",
    ".pyo",
    ".swo",
    ".swp",
    ".tmp",
    ".ulg",
    ".webm",
)
PROHIBITED_NAMES = {".coverage"}
PROHIBITED_ROOT_NAMES = {"findings.md", "progress.md", "task_plan.md"}


def run_git(repo, *arguments):
    result = subprocess.run(
        ["git", "-C", str(repo), *arguments],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout


def candidate_paths(repo):
    output = run_git(
        repo, "ls-files", "-z", "--cached", "--others", "--exclude-standard"
    )
    return [PurePosixPath(item) for item in output.split("\0") if item]


def is_prohibited(path):
    if len(path.parts) == 1 and path.name.casefold() in PROHIBITED_ROOT_NAMES:
        return True
    if path.name in PROHIBITED_NAMES or path.name.endswith("~"):
        return True
    if path.suffix.lower() in PROHIBITED_SUFFIXES:
        return True
    if any(path == prefix or prefix in path.parents for prefix in PROHIBITED_PATH_PREFIXES):
        return True
    if len(path.parts) > 1 and path.parts[0].startswith(
        PROHIBITED_ROOT_DIRECTORY_PREFIXES
    ):
        return True
    for part in path.parts:
        if part in PROHIBITED_PARTS:
            return True
    return False


def audit_candidate_files(repo, max_tracked_bytes=DEFAULT_MAX_TRACKED_BYTES):
    violations = []
    for relative_path in candidate_paths(repo):
        if is_prohibited(relative_path):
            violations.append(f"prohibited candidate path: {relative_path}")
            continue
        file_path = repo / Path(*relative_path.parts)
        if file_path.exists() and file_path.lstat().st_size > max_tracked_bytes:
            violations.append(
                f"candidate file exceeds {max_tracked_bytes} bytes: {relative_path}"
            )
    return violations


def dirty_entries(repo):
    output = run_git(repo, "status", "--short", "--untracked-files=all")
    return [line for line in output.splitlines() if line]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument(
        "--max-tracked-bytes", type=int, default=DEFAULT_MAX_TRACKED_BYTES
    )
    parser.add_argument(
        "--allow-dirty",
        action="store_true",
        help="Audit candidate paths and sizes without requiring a clean worktree.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    repo = args.repo.resolve()
    violations = audit_candidate_files(repo, args.max_tracked_bytes)
    dirty = [] if args.allow_dirty else dirty_entries(repo)
    if dirty:
        violations.append(f"worktree has {len(dirty)} changed or untracked path(s)")
    if violations:
        for violation in violations:
            print(f"repository cleanliness rejected: {violation}")
        raise SystemExit(1)
    worktree_status = "worktree check skipped" if args.allow_dirty else "clean worktree"
    print(
        "repository cleanliness accepted: "
        f"{len(candidate_paths(repo))} candidate paths, {worktree_status}"
    )


if __name__ == "__main__":
    main()
