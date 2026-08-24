#!/usr/bin/env bash
set -Eeuo pipefail

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

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
clone_start_epoch="${1:-}"
time_limit_s=1800
rehearsal="${M5_REHEARSAL:-0}"

if [[ ! "${clone_start_epoch}" =~ ^[0-9]+$ ]]; then
  echo "usage: bash scripts/run_m5_reproduction.sh <clone-start-epoch>" >&2
  exit 2
fi
if [[ "${rehearsal}" != "0" && "${rehearsal}" != "1" ]]; then
  echo "M5_REHEARSAL must be 0 or 1" >&2
  exit 2
fi

script_start_epoch="$(date +%s)"
if ((clone_start_epoch > script_start_epoch)); then
  echo "clone start epoch is in the future" >&2
  exit 2
fi

cd "${project_root}"
for command in docker git python3 sha256sum; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "required command is unavailable: ${command}" >&2
    exit 2
  fi
done
docker compose version >/dev/null
if ! docker info >/dev/null 2>&1; then
  echo "current user cannot access the Docker daemon; 'docker info' must succeed" >&2
  exit 2
fi

repository_status="$(git status --porcelain --untracked-files=all)"
repository_clean=true
if [[ -n "${repository_status}" ]]; then
  repository_clean=false
  if [[ "${rehearsal}" == "0" ]]; then
    echo "acceptance run requires a clean repository" >&2
    exit 2
  fi
fi
repository_origin_raw="$(git remote get-url origin 2>/dev/null || true)"
if [[ "${rehearsal}" == "0" && -z "${repository_origin_raw}" ]]; then
  echo "acceptance run requires a Git origin from the fresh clone" >&2
  exit 2
fi
repository_origin=""
if [[ -n "${repository_origin_raw}" ]]; then
  if ! repository_origin="$(python3 - "${repository_origin_raw}" <<'PY'
from urllib.parse import urlsplit, urlunsplit
import re
import sys

raw = sys.argv[1]
if "\0" in raw or raw.splitlines() != [raw]:
    raise SystemExit(1)
if "://" in raw:
    parsed = urlsplit(raw)
    if not parsed.scheme or not parsed.hostname:
        raise SystemExit(1)
    host = parsed.hostname
    if ":" in host:
        host = f"[{host}]"
    try:
        port = parsed.port
    except ValueError:
        raise SystemExit(1)
    netloc = f"{host}:{port}" if port is not None else host
    print(urlunsplit((parsed.scheme, netloc, parsed.path, "", "")))
else:
    scp_origin = re.fullmatch(r"(?:[^@/]+@)?([^:/]+):(.+)", raw)
    print(f"{scp_origin.group(1)}:{scp_origin.group(2)}" if scp_origin else raw)
PY
  )"; then
    echo "Git origin cannot be recorded safely" >&2
    exit 2
  fi
fi

existing_project_image=false
if docker image inspect project19-inspection:local >/dev/null 2>&1; then
  existing_project_image=true
fi
existing_container_ids="$(docker compose ps -aq)"
existing_container_count=0
if [[ -n "${existing_container_ids}" ]]; then
  existing_container_count="$(wc -l <<<"${existing_container_ids}")"
fi

bag_root="${project_root}/log/m4"
preexisting_bag_count=0
if [[ -d "${bag_root}" ]]; then
  preexisting_bag_count="$(
    find "${bag_root}" -mindepth 1 -maxdepth 1 -type d \
      -name 'inspection_*' -printf '.' | wc -c
  )"
fi

if [[ "${rehearsal}" == "0" ]]; then
  if [[ "${existing_project_image}" == "true" ]]; then
    echo "acceptance run requires no pre-existing project image" >&2
    exit 2
  fi
  if ((existing_container_count > 0)); then
    echo "acceptance run requires no pre-existing project containers" >&2
    exit 2
  fi
  if ((preexisting_bag_count > 0)); then
    echo "acceptance run requires no pre-existing M4 inspection bags" >&2
    exit 2
  fi
else
  docker compose down >/dev/null 2>&1 || true
fi

run_id="$(date +%Y%m%d_%H%M%S)"
output_directory="${project_root}/log/m5/reproduction_${run_id}"
mkdir -p "${output_directory}" "${bag_root}"
before_bags="${output_directory}/bags-before.txt"
after_bags="${output_directory}/bags-after.txt"
find "${bag_root}" -mindepth 1 -maxdepth 1 -type d \
  -name 'inspection_*' -printf '%f\n' | sort >"${before_bags}"

compose_exit_code=-1
analyzer_exit_code=-1
compose_tee_exit_code=-1
analyzer_tee_exit_code=-1
mission_end_epoch=-1
verification_end_epoch=-1
mission_elapsed_s=-1
verified_elapsed_s=-1
bag_directory=""
new_bag_count=-1
new_bags_csv=""
bag_sha256=""
analyzer_accepted=false
within_time_limit=false
acceptance_candidate=false

write_evidence() {
  local image_id=""
  image_id="$(
    docker image inspect project19-inspection:local --format '{{.Id}}' \
      2>/dev/null || true
  )"
  {
    printf 'schema_version=2\n'
    printf 'acceptance_candidate=%s\n' "${acceptance_candidate}"
    printf 'rehearsal=%s\n' "${rehearsal}"
    printf 'clone_start_epoch=%s\n' "${clone_start_epoch}"
    printf 'script_start_epoch=%s\n' "${script_start_epoch}"
    printf 'mission_end_epoch=%s\n' "${mission_end_epoch}"
    printf 'verification_end_epoch=%s\n' "${verification_end_epoch}"
    printf 'mission_elapsed_s=%s\n' "${mission_elapsed_s}"
    printf 'verified_elapsed_s=%s\n' "${verified_elapsed_s}"
    printf 'time_limit_s=%s\n' "${time_limit_s}"
    printf 'within_time_limit=%s\n' "${within_time_limit}"
    printf 'compose_exit_code=%s\n' "${compose_exit_code}"
    printf 'compose_tee_exit_code=%s\n' "${compose_tee_exit_code}"
    printf 'analyzer_exit_code=%s\n' "${analyzer_exit_code}"
    printf 'analyzer_tee_exit_code=%s\n' "${analyzer_tee_exit_code}"
    printf 'analyzer_accepted=%s\n' "${analyzer_accepted}"
    printf 'repository_clean=%s\n' "${repository_clean}"
    printf 'repository_commit=%s\n' "$(git rev-parse HEAD)"
    printf 'repository_origin=%s\n' "${repository_origin}"
    printf 'existing_project_image_before=%s\n' "${existing_project_image}"
    printf 'existing_project_containers_before=%s\n' "${existing_container_count}"
    printf 'preexisting_m4_bags=%s\n' "${preexisting_bag_count}"
    printf 'new_m4_bag_count=%s\n' "${new_bag_count}"
    printf 'new_m4_bags=%s\n' "${new_bags_csv}"
    printf 'image_id=%s\n' "${image_id}"
    printf 'bag_path=%s\n' "${bag_directory}"
    printf 'bag_sha256=%s\n' "${bag_sha256}"
    printf 'docker_engine=%s\n' "$(docker version --format '{{.Server.Version}}')"
    printf 'docker_compose=%s\n' "$(docker compose version --short)"
  } >"${output_directory}/evidence.env"
}

cleanup() {
  local status=$?
  trap - EXIT
  write_evidence || true
  docker compose down >/dev/null 2>&1 || true
  echo "M5 reproduction evidence: ${output_directory}/evidence.env"
  exit "${status}"
}
trap cleanup EXIT

set +e
docker compose up --build --abort-on-container-exit \
  --exit-code-from inspection inspection \
  2>&1 | tee "${output_directory}/compose.log"
compose_pipeline_status=("${PIPESTATUS[@]}")
set -e
compose_exit_code=${compose_pipeline_status[0]}
compose_tee_exit_code=${compose_pipeline_status[1]}
mission_end_epoch="$(date +%s)"
mission_elapsed_s=$((mission_end_epoch - clone_start_epoch))

find "${bag_root}" -mindepth 1 -maxdepth 1 -type d \
  -name 'inspection_*' -printf '%f\n' | sort >"${after_bags}"
mapfile -t new_bags < <(comm -13 "${before_bags}" "${after_bags}")
new_bag_count=${#new_bags[@]}
new_bags_csv="$(IFS=,; printf '%s' "${new_bags[*]}")"
if ((new_bag_count == 1)); then
  bag_directory="log/m4/${new_bags[0]}"
fi
if ((compose_exit_code != 0)); then
  echo "Compose failed with exit code ${compose_exit_code}" >&2
  exit "${compose_exit_code}"
fi
if ((compose_tee_exit_code != 0)); then
  echo "Compose log capture failed with exit code ${compose_tee_exit_code}" >&2
  exit "${compose_tee_exit_code}"
fi
if ((new_bag_count != 1)); then
  echo "expected exactly one new M4 bag, found ${new_bag_count}" >&2
  exit 1
fi

set +e
docker compose run --rm --no-deps inspection \
  python3 scripts/analyze_m4_mission.py "/opt/project19/${bag_directory}" \
  --metrics "/opt/project19/${bag_directory}/m4_metrics.json" \
  --launch-exit-code "${compose_exit_code}" \
  2>&1 | tee "${output_directory}/analyzer.log"
analyzer_pipeline_status=("${PIPESTATUS[@]}")
set -e
analyzer_exit_code=${analyzer_pipeline_status[0]}
analyzer_tee_exit_code=${analyzer_pipeline_status[1]}
verification_end_epoch="$(date +%s)"
verified_elapsed_s=$((verification_end_epoch - clone_start_epoch))
if ((analyzer_exit_code != 0)); then
  echo "M4 analyzer failed with exit code ${analyzer_exit_code}" >&2
  exit "${analyzer_exit_code}"
fi
if ((analyzer_tee_exit_code != 0)); then
  echo "M4 analyzer log capture failed with exit code ${analyzer_tee_exit_code}" >&2
  exit "${analyzer_tee_exit_code}"
fi

read -r analyzer_accepted bag_sha256 < <(
  python3 - "${bag_directory}/m4_metrics.json" <<'PY'
import json
from pathlib import Path
import sys

metrics = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
print(str(metrics.get("accepted", False)).lower(), metrics["provenance"]["bag_sha256"])
PY
)
if [[ "${analyzer_accepted}" != "true" ]]; then
  echo "M4 analyzer did not accept the new bag" >&2
  exit 1
fi
if ((mission_elapsed_s <= time_limit_s)); then
  within_time_limit=true
else
  echo "M5 reproduction exceeded ${time_limit_s} seconds: ${mission_elapsed_s}" >&2
  exit 1
fi

if [[ "${rehearsal}" == "0" ]]; then
  acceptance_candidate=true
fi
write_evidence
if [[ "${rehearsal}" == "1" ]]; then
  echo "M5 reproduction rehearsal passed (not an acceptance candidate): mission_elapsed_s=${mission_elapsed_s}"
else
  echo "M5 reproduction candidate accepted: mission_elapsed_s=${mission_elapsed_s}"
fi
