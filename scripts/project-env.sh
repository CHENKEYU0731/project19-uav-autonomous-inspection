#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "error: source this file instead of executing it" >&2
  exit 1
fi

project_root="/opt/project19"

if ! mountpoint -q "${project_root}"; then
  echo "error: run 'bash scripts/mount-project.sh' first" >&2
  return 1
fi

export PROJECT_ROOT="${project_root}"
export XDG_CACHE_HOME="${PROJECT_ROOT}/.cache"
export PIP_CACHE_DIR="${XDG_CACHE_HOME}/pip"
export COLCON_HOME="${XDG_CACHE_HOME}/colcon"
export TMPDIR="${XDG_CACHE_HOME}/tmp"
export ROS_LOG_DIR="${PROJECT_ROOT}/log/ros"
export PYTHONUSERBASE="${PROJECT_ROOT}/.local/python"
export MICRO_XRCE_DDS_AGENT_PREFIX="${PROJECT_ROOT}/.local/micro-xrce-dds-agent"

mkdir -p \
  "${PIP_CACHE_DIR}" \
  "${COLCON_HOME}" \
  "${TMPDIR}" \
  "${ROS_LOG_DIR}" \
  "${PYTHONUSERBASE}" \
  "${MICRO_XRCE_DDS_AGENT_PREFIX}"

# WSL appends the Windows PATH by default. Keep Linux builds isolated from
# host tools such as Anaconda, whose CMake packages are not ABI-compatible.
linux_system_path="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export PATH="${PYTHONUSERBASE}/bin:${MICRO_XRCE_DDS_AGENT_PREFIX}/bin:${HOME}/.local/bin:${linux_system_path}"
export LD_LIBRARY_PATH="${MICRO_XRCE_DDS_AGENT_PREFIX}/lib:${LD_LIBRARY_PATH:-}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "error: ROS 2 Humble is not installed" >&2
  return 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

if [[ -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/install/setup.bash"
fi

cd "${PROJECT_ROOT}" || return 1
unset linux_system_path project_root
