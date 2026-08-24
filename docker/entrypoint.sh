#!/usr/bin/env bash
set -euo pipefail

export PROJECT_ROOT="/opt/project19"
export XDG_CACHE_HOME="${PROJECT_ROOT}/.cache"
export PIP_CACHE_DIR="${XDG_CACHE_HOME}/pip"
export COLCON_HOME="${XDG_CACHE_HOME}/colcon"
export TMPDIR="${XDG_CACHE_HOME}/tmp"
export ROS_LOG_DIR="${PROJECT_ROOT}/log/ros"
export PYTHONUSERBASE="${PROJECT_ROOT}/.local/python"
export MICRO_XRCE_DDS_AGENT_PREFIX="${PROJECT_ROOT}/.local/micro-xrce-dds-agent"
export PATH="${PYTHONUSERBASE}/bin:${MICRO_XRCE_DDS_AGENT_PREFIX}/bin:${PATH}"
export LD_LIBRARY_PATH="${MICRO_XRCE_DDS_AGENT_PREFIX}/lib:${LD_LIBRARY_PATH:-}"

if [[ "$(id -u)" == "0" ]]; then
  mkdir -p "${PROJECT_ROOT}/log"
  chown -R simulator:simulator "${PROJECT_ROOT}/log"
  exec gosu simulator "$0" "$@"
fi

mkdir -p "${TMPDIR}" "${ROS_LOG_DIR}"

# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${PROJECT_ROOT}/install/setup.bash"
set -u

cd "${PROJECT_ROOT}"
exec "$@"
