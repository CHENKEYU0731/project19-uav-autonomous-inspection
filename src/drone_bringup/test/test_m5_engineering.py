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

import configparser
import importlib.util
import os
from pathlib import Path
import subprocess
import textwrap

import pytest
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[3]


def write_executable(path, content):
    path.write_text(textwrap.dedent(content))
    path.chmod(0o755)


def run_fake_reproduction(
    tmp_path,
    *,
    rehearsal=False,
    compose_exit=0,
    analyzer_exit=0,
    analyzer_accepted=True,
    tee_fail_call=0,
    clock=(1000, 1100, 1101),
    preexisting_bags=(),
    origin="https://example.invalid/project19.git",
):
    repo = tmp_path / "repo"
    scripts = repo / "scripts"
    fake_bin = tmp_path / "bin"
    scripts.mkdir(parents=True)
    fake_bin.mkdir()
    runner = scripts / "run_m5_reproduction.sh"
    runner.write_text((PROJECT_ROOT / "scripts" / runner.name).read_text())
    runner.chmod(0o755)
    for bag_name in preexisting_bags:
        (repo / "log" / "m4" / bag_name).mkdir(parents=True)

    write_executable(
        fake_bin / "date",
        """\
        #!/usr/bin/env bash
        if [[ "${1:-}" != "+%s" ]]; then
          echo 20260824_000000
          exit 0
        fi
        count=0
        [[ -f "${FAKE_DATE_STATE}" ]] && count="$(<"${FAKE_DATE_STATE}")"
        IFS=, read -r -a values <<<"${FAKE_CLOCK}"
        echo "${values[${count}]}"
        echo "$((count + 1))" >"${FAKE_DATE_STATE}"
        """,
    )
    write_executable(
        fake_bin / "git",
        """\
        #!/usr/bin/env bash
        case "${1:-} ${2:-}" in
          "status --porcelain") exit 0 ;;
          "rev-parse HEAD") echo 0123456789abcdef ;;
          "remote get-url")
            [[ -n "${FAKE_ORIGIN}" ]] || exit 2
            echo "${FAKE_ORIGIN}"
            ;;
          *) exit 1 ;;
        esac
        """,
    )
    write_executable(
        fake_bin / "docker",
        """\
        #!/usr/bin/env bash
        if [[ "${1:-}" == "info" ]]; then exit 0; fi
        if [[ "${1:-} ${2:-}" == "image inspect" ]]; then
          [[ " $* " == *" --format "* ]] && echo sha256:fake-image && exit 0
          exit 1
        fi
        if [[ "${1:-}" == "version" ]]; then echo 29.1.3; exit 0; fi
        if [[ "${1:-} ${2:-}" == "compose version" ]]; then
          echo 2.40.3
          exit 0
        fi
        if [[ "${1:-} ${2:-} ${3:-}" == "compose ps -aq" ]]; then exit 0; fi
        if [[ "${1:-} ${2:-}" == "compose down" ]]; then exit 0; fi
        if [[ "${1:-} ${2:-}" == "compose up" ]]; then
          mkdir -p "log/m4/${FAKE_BAG_NAME}"
          echo "fake compose output"
          exit "${FAKE_COMPOSE_EXIT}"
        fi
        if [[ "${1:-} ${2:-}" == "compose run" ]]; then
          metrics=""
          for ((index = 1; index <= $#; ++index)); do
            if [[ "${!index}" == "--metrics" ]]; then
              next=$((index + 1))
              metrics="${!next#/opt/project19/}"
            fi
          done
          if [[ -n "${metrics}" && "${FAKE_ANALYZER_EXIT}" == "0" ]]; then
            mkdir -p "$(dirname "${metrics}")"
            printf '{"accepted": %s, ' "${FAKE_ANALYZER_ACCEPTED}" >"${metrics}"
            printf '%s\n' '"provenance": {"bag_sha256": "fake-bag-sha256"}}' >>"${metrics}"
          fi
          echo "fake analyzer output"
          exit "${FAKE_ANALYZER_EXIT}"
        fi
        exit 1
        """,
    )
    write_executable(
        fake_bin / "tee",
        """\
        #!/usr/bin/env bash
        /usr/bin/tee "$@"
        count=0
        [[ -f "${FAKE_TEE_STATE}" ]] && count="$(<"${FAKE_TEE_STATE}")"
        count=$((count + 1))
        echo "${count}" >"${FAKE_TEE_STATE}"
        if [[ "${FAKE_TEE_FAIL_CALL}" != "0" && "${count}" == "${FAKE_TEE_FAIL_CALL}" ]]; then
          exit 28
        fi
        """,
    )

    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}{os.pathsep}{environment['PATH']}",
            "M5_REHEARSAL": "1" if rehearsal else "0",
            "FAKE_COMPOSE_EXIT": str(compose_exit),
            "FAKE_ANALYZER_EXIT": str(analyzer_exit),
            "FAKE_ANALYZER_ACCEPTED": str(analyzer_accepted).lower(),
            "FAKE_TEE_FAIL_CALL": str(tee_fail_call),
            "FAKE_BAG_NAME": "inspection_fake_run",
            "FAKE_CLOCK": ",".join(str(value) for value in clock),
            "FAKE_DATE_STATE": str(tmp_path / "date-state"),
            "FAKE_TEE_STATE": str(tmp_path / "tee-state"),
            "FAKE_ORIGIN": origin,
        }
    )
    result = subprocess.run(
        ["bash", str(runner), "1000"],
        cwd=repo,
        env=environment,
        capture_output=True,
        text=True,
    )
    evidence_paths = list((repo / "log" / "m5").glob("reproduction_*/evidence.env"))
    evidence = {}
    if evidence_paths:
        evidence = dict(
            line.split("=", 1)
            for line in evidence_paths[0].read_text().splitlines()
            if "=" in line
        )
    return result, evidence


def load_cleanliness_module():
    script = PROJECT_ROOT / "scripts" / "check_repository_cleanliness.py"
    spec = importlib.util.spec_from_file_location("check_repository_cleanliness", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def git(repo, *arguments):
    subprocess.run(
        ["git", "-C", str(repo), *arguments],
        check=True,
        capture_output=True,
        text=True,
    )


def test_compose_default_runs_complete_headless_m4():
    compose = yaml.safe_load((PROJECT_ROOT / "compose.yaml").read_text())
    service = compose["services"]["inspection"]
    command = service["command"]
    assert "m4_inspection.launch.py" in command
    assert "use_rviz:=false" in command
    assert "use_gazebo_gui:=false" in command
    assert service["network_mode"] == "host"
    assert "./log:/opt/project19/log" in service["volumes"]


def test_container_uses_pinned_project_dependencies_and_entrypoint():
    dockerfile = (PROJECT_ROOT / "Dockerfile").read_text()
    entrypoint = (PROJECT_ROOT / "docker" / "entrypoint.sh").read_text()
    license_text = (PROJECT_ROOT / "LICENSE").read_text()
    assert "FROM ubuntu:22.04" in dockerfile
    assert "vcs import --shallow --retry 5 . < dependencies.repos" in dockerfile
    assert "make -C external/PX4-Autopilot px4_sitl" in dockerfile
    assert "colcon build --symlink-install" in dockerfile
    assert "rosdep init" in dockerfile
    assert "rosdep init 2>/dev/null || true" not in dockerfile
    assert "gosu" in dockerfile
    assert 'exec gosu simulator "$0" "$@"' in entrypoint
    assert 'ENTRYPOINT ["/opt/project19/docker/entrypoint.sh"]' in dockerfile
    assert "Apache License" in license_text
    assert "Version 2.0, January 2004" in license_text


def test_dockerfile_applies_optional_github_mirror_to_bootstrap_download():
    dockerfile = (PROJECT_ROOT / "Dockerfile").read_text()

    argument = 'ARG GITHUB_MIRROR_PREFIX=""'
    bootstrap = 'RUN ros_apt_source_url="https://github.com/ros-infrastructure/'
    assert dockerfile.index(argument) < dockerfile.index(bootstrap)
    assert (
        'ros_apt_source_url="${GITHUB_MIRROR_PREFIX}${ros_apt_source_url}"'
        in dockerfile
    )
    assert '"${ros_apt_source_url}"' in dockerfile


def test_docker_context_and_bringup_install_exclude_generated_python_bytecode():
    dockerignore = (PROJECT_ROOT / ".dockerignore").read_text()
    cmake = (PROJECT_ROOT / "src" / "drone_bringup" / "CMakeLists.txt").read_text()

    assert "**/__pycache__/" in dockerignore.splitlines()
    assert "**/*.pyc" in dockerignore.splitlines()
    assert "**/*.pyo" in dockerignore.splitlines()

    install_rule = cmake[cmake.index("install(\n  DIRECTORY config launch"):]
    install_rule = install_rule[: install_rule.index(")") + 1]
    assert 'PATTERN "__pycache__" EXCLUDE' in install_rule
    assert 'PATTERN "*.pyc" EXCLUDE' in install_rule
    assert 'PATTERN "*.pyo" EXCLUDE' in install_rule


def test_bringup_install_tree_contains_no_generated_python_bytecode():
    install_root = (
        PROJECT_ROOT / "install" / "drone_bringup" / "share" / "drone_bringup"
    )
    if not install_root.is_dir():
        pytest.skip("drone_bringup must be installed before checking install output")

    generated = sorted(
        path.relative_to(install_root).as_posix()
        for path in install_root.rglob("*")
        if path.name == "__pycache__" or path.suffix in {".pyc", ".pyo"}
    )
    assert generated == []


def test_ci_runs_build_project_tests_ament_lint_and_cleanliness_gate():
    workflow = (PROJECT_ROOT / ".github" / "workflows" / "ci.yml").read_text()
    ci_script = (PROJECT_ROOT / "scripts" / "ci.sh").read_text()
    flake8_config = (PROJECT_ROOT / ".github" / "ament_flake8.ini").read_text()
    assert "actions/checkout@v5" in workflow
    assert "bash scripts/ci.sh" in workflow
    assert "colcon build --symlink-install" in ci_script
    assert "colcon test" in ci_script
    workspace_setup = 'source "${PROJECT_ROOT}/install/setup.bash"'
    assert ci_script.index("colcon build --symlink-install") < ci_script.index(
        workspace_setup
    )
    assert ci_script.index(workspace_setup) < ci_script.index("colcon test")
    assert "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1" in ci_script
    assert "ament_cppcheck src/drone_*/include src/drone_*/src" in ci_script
    assert "--filters=-legal/copyright,-build/include_order" in ci_script
    assert '--config "${PROJECT_ROOT}/.github/ament_flake8.ini"' in ci_script
    assert "import-order-style = google" in flake8_config
    assert "max-line-length = 99" in flake8_config
    parsed_flake8_config = configparser.ConfigParser()
    parsed_flake8_config.read_string(flake8_config)
    ignored_rules = parsed_flake8_config["flake8"]["extend-ignore"].split(",")
    for rule in ("Q000", "I100", "I101", "CNL100"):
        assert rule in ignored_rules
    for linter in (
        "ament_copyright",
        "ament_cppcheck",
        "ament_cpplint",
        "ament_uncrustify",
        "ament_flake8",
        "ament_pep257",
        "ament_lint_cmake",
        "ament_xmllint",
    ):
        assert linter in ci_script
    assert "check_repository_cleanliness.py" in ci_script


def test_harmonic_install_skips_conflicting_fortress_rosdep_keys():
    dockerfile = (PROJECT_ROOT / "Dockerfile").read_text()
    workflow = (PROJECT_ROOT / ".github" / "workflows" / "ci.yml").read_text()
    skip_keys = '--skip-keys "ros_gz_bridge ros_gz_interfaces ros_gz_sim"'

    assert skip_keys in dockerfile
    assert skip_keys in workflow


def test_reproduction_protocol_times_from_clone_and_accepts_only_a_new_bag():
    guide = (PROJECT_ROOT / "docs" / "reproduction-guide.md").read_text()
    runner = (PROJECT_ROOT / "scripts" / "run_m5_reproduction.sh").read_text()

    assert guide.index('start_epoch="$(date +%s)"') < guide.index("git clone")
    assert 'clone_start_epoch="${1:-}"' in runner
    assert "mission_elapsed_s=$((mission_end_epoch - clone_start_epoch))" in runner
    assert "docker compose up --build --abort-on-container-exit" in runner
    assert 'comm -13 "${before_bags}" "${after_bags}"' in runner
    assert "expected exactly one new M4 bag" in runner
    assert "scripts/analyze_m4_mission.py" in runner
    assert "M5_REHEARSAL" in runner
    assert "acceptance_candidate=false" in runner
    assert 'if [[ "${rehearsal}" == "0" ]]; then\n  acceptance_candidate=true' in runner
    assert "rehearsal passed (not an acceptance candidate)" in runner
    assert "existing_project_image_before" in runner
    assert "within_time_limit" in runner


def test_reproduction_records_new_bag_when_compose_fails(tmp_path):
    result, evidence = run_fake_reproduction(tmp_path, compose_exit=7)

    assert result.returncode == 7
    assert evidence["acceptance_candidate"] == "false"
    assert evidence["compose_exit_code"] == "7"
    assert evidence["new_m4_bag_count"] == "1"
    assert evidence["new_m4_bags"] == "inspection_fake_run"
    assert evidence["bag_path"] == "log/m4/inspection_fake_run"


def test_acceptance_run_rejects_preexisting_bag_before_start(tmp_path):
    result, evidence = run_fake_reproduction(
        tmp_path,
        preexisting_bags=("inspection_old",),
    )

    assert result.returncode == 2
    assert "requires no pre-existing M4 inspection bags" in result.stderr
    assert evidence == {}


def test_acceptance_run_requires_origin_from_fresh_clone(tmp_path):
    result, evidence = run_fake_reproduction(tmp_path, origin="")

    assert result.returncode == 2
    assert "requires a Git origin from the fresh clone" in result.stderr
    assert evidence == {}


def test_reproduction_sanitizes_credentials_and_url_metadata_from_origin(tmp_path):
    result, evidence = run_fake_reproduction(
        tmp_path,
        origin="https://user:secret@example.invalid/project19.git?token=value#part",
    )

    assert result.returncode == 0
    assert evidence["repository_origin"] == "https://example.invalid/project19.git"


@pytest.mark.parametrize(
    "line_break",
    ("\n", "\r", "\v", "\f", "\x1c", "\x1d", "\x1e", "\x85", "\u2028", "\u2029"),
)
def test_reproduction_rejects_origin_with_evidence_line_break(tmp_path, line_break):
    result, evidence = run_fake_reproduction(
        tmp_path,
        origin=(
            "https://example.invalid/project19.git"
            f"{line_break}acceptance_candidate=true"
        ),
    )

    assert result.returncode == 2
    assert "origin cannot be recorded safely" in result.stderr
    assert evidence == {}


def test_rehearsal_selects_only_new_bag_among_preexisting_bags(tmp_path):
    result, evidence = run_fake_reproduction(
        tmp_path,
        rehearsal=True,
        preexisting_bags=("inspection_old_a", "inspection_old_b"),
    )

    assert result.returncode == 0
    assert evidence["preexisting_m4_bags"] == "2"
    assert evidence["new_m4_bag_count"] == "1"
    assert evidence["new_m4_bags"] == "inspection_fake_run"
    assert evidence["bag_path"] == "log/m4/inspection_fake_run"


def test_reproduction_rejects_compose_log_capture_failure(tmp_path):
    result, evidence = run_fake_reproduction(tmp_path, tee_fail_call=1)

    assert result.returncode == 28
    assert "Compose log capture failed" in result.stderr
    assert evidence["acceptance_candidate"] == "false"
    assert evidence["compose_exit_code"] == "0"
    assert evidence["compose_tee_exit_code"] == "28"
    assert evidence["analyzer_exit_code"] == "-1"


def test_reproduction_rejects_analyzer_log_capture_failure(tmp_path):
    result, evidence = run_fake_reproduction(tmp_path, tee_fail_call=2)

    assert result.returncode == 28
    assert "analyzer log capture failed" in result.stderr
    assert evidence["acceptance_candidate"] == "false"
    assert evidence["analyzer_exit_code"] == "0"
    assert evidence["analyzer_tee_exit_code"] == "28"


def test_reproduction_records_analyzer_failure_without_candidate(tmp_path):
    result, evidence = run_fake_reproduction(tmp_path, analyzer_exit=9)

    assert result.returncode == 9
    assert evidence["acceptance_candidate"] == "false"
    assert evidence["bag_path"] == "log/m4/inspection_fake_run"
    assert evidence["analyzer_exit_code"] == "9"
    assert evidence["analyzer_tee_exit_code"] == "0"


def test_reproduction_rejects_unaccepted_analyzer_metrics(tmp_path):
    result, evidence = run_fake_reproduction(tmp_path, analyzer_accepted=False)

    assert result.returncode == 1
    assert "analyzer did not accept" in result.stderr
    assert evidence["acceptance_candidate"] == "false"
    assert evidence["analyzer_exit_code"] == "0"
    assert evidence["analyzer_accepted"] == "false"


def test_reproduction_rehearsal_can_pass_but_never_become_candidate(tmp_path):
    result, evidence = run_fake_reproduction(tmp_path, rehearsal=True)

    assert result.returncode == 0
    assert "rehearsal passed (not an acceptance candidate)" in result.stdout
    assert "candidate accepted" not in result.stdout
    assert evidence["acceptance_candidate"] == "false"
    assert evidence["rehearsal"] == "1"
    assert evidence["analyzer_accepted"] == "true"


@pytest.mark.parametrize(
    ("mission_end", "expected_returncode", "expected_candidate"),
    ((2800, 0, "true"), (2801, 1, "false")),
)
def test_reproduction_enforces_mission_time_boundary(
    tmp_path, mission_end, expected_returncode, expected_candidate
):
    result, evidence = run_fake_reproduction(
        tmp_path,
        clock=(1000, mission_end, mission_end + 1),
    )

    assert result.returncode == expected_returncode
    assert evidence["mission_elapsed_s"] == str(mission_end - 1000)
    assert evidence["acceptance_candidate"] == expected_candidate


def test_cleanliness_audit_rejects_generated_and_large_candidate_files(tmp_path):
    module = load_cleanliness_module()
    git(tmp_path, "init", "-q")
    (tmp_path / "build-sanitized").mkdir()
    (tmp_path / "build-sanitized" / "artifact.txt").write_text("generated")
    (tmp_path / "large.bin").write_bytes(b"12345")
    violations = module.audit_candidate_files(tmp_path, max_tracked_bytes=4)

    assert any("prohibited candidate path" in item for item in violations)
    assert any("candidate file exceeds" in item for item in violations)


@pytest.mark.parametrize(
    "relative_path",
    (
        "external/PX4-Autopilot/README.md",
        "external/Micro-XRCE-DDS-Agent/README.md",
        "src/px4_msgs/README.md",
        "src/px4_ros_com/README.md",
    ),
)
def test_cleanliness_audit_rejects_force_tracked_external_source(
    tmp_path, relative_path
):
    module = load_cleanliness_module()
    git(tmp_path, "init", "-q")
    (tmp_path / ".gitignore").write_text("external/\nsrc/px4_*/\n")
    dependency_file = tmp_path / relative_path
    dependency_file.parent.mkdir(parents=True)
    dependency_file.write_text("third-party source")
    git(tmp_path, "add", ".gitignore")
    git(tmp_path, "add", "-f", relative_path)

    violations = module.audit_candidate_files(tmp_path)

    assert any(relative_path in item for item in violations)


def test_cleanliness_audit_allows_build_and_install_document_names(tmp_path):
    module = load_cleanliness_module()
    git(tmp_path, "init", "-q")
    docs = tmp_path / "docs"
    docs.mkdir()
    (docs / "build-notes.md").write_text("documentation")
    (docs / "install-guide.md").write_text("documentation")
    git(tmp_path, "add", "docs")

    assert module.audit_candidate_files(tmp_path) == []


@pytest.mark.parametrize(
    "name",
    ("findings.md", "Progress.md", "TASK_PLAN.MD"),
)
def test_cleanliness_audit_rejects_root_agent_working_notes(tmp_path, name):
    module = load_cleanliness_module()
    git(tmp_path, "init", "-q")
    (tmp_path / name).write_text("stale agent working note")

    violations = module.audit_candidate_files(tmp_path)

    assert violations == [f"prohibited candidate path: {name}"]


def test_cleanliness_audit_allows_same_names_below_docs(tmp_path):
    module = load_cleanliness_module()
    git(tmp_path, "init", "-q")
    docs = tmp_path / "docs"
    docs.mkdir()
    for name in module.PROHIBITED_ROOT_NAMES:
        (docs / name).write_text("intentional project documentation")

    assert module.audit_candidate_files(tmp_path) == []


@pytest.mark.parametrize(
    "relative_path",
    (
        "flight.mcap",
        "flight.ulg",
        "run.log",
        "demo.webm",
        "scratch.tmp",
        ".coverage",
        "src/main.cpp~",
        ".venv/pyvenv.cfg",
    ),
)
def test_cleanliness_audit_rejects_project_residue_formats(tmp_path, relative_path):
    module = load_cleanliness_module()
    git(tmp_path, "init", "-q")
    residue = tmp_path / relative_path
    residue.parent.mkdir(parents=True, exist_ok=True)
    residue.write_text("generated")

    violations = module.audit_candidate_files(tmp_path)

    assert any("prohibited candidate path" in item for item in violations)


def test_cleanliness_audit_accepts_small_source_file(tmp_path):
    module = load_cleanliness_module()
    git(tmp_path, "init", "-q")
    (tmp_path / "src").mkdir()
    (tmp_path / "src" / "main.cpp").write_text("int main() { return 0; }\n")
    git(tmp_path, "add", "src/main.cpp")

    assert module.audit_candidate_files(tmp_path) == []
