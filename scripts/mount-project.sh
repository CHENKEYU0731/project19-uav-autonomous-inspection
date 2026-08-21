#!/usr/bin/env bash
set -euo pipefail

mount_point="/opt/project19"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/.." && pwd -P)"

sudo mkdir -p "${mount_point}"

if mountpoint -q "${mount_point}"; then
  if [[ ! "${project_root}/AGENTS.md" -ef "${mount_point}/AGENTS.md" ]]; then
    echo "error: ${mount_point} is already mounted from another location" >&2
    exit 1
  fi
else
  sudo mount --bind "${project_root}" "${mount_point}"
fi

echo "Project mounted at ${mount_point}"
