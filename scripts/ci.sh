#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
project_packages=(
  drone_interfaces
  drone_sim
  drone_perception
  drone_planner
  drone_controller
  drone_mission
  drone_bringup
)

export PROJECT_ROOT="${project_root}"
export XDG_CACHE_HOME="${PROJECT_ROOT}/.cache"
export PIP_CACHE_DIR="${XDG_CACHE_HOME}/pip"
export COLCON_HOME="${XDG_CACHE_HOME}/colcon"
export TMPDIR="${XDG_CACHE_HOME}/tmp"
export ROS_LOG_DIR="${PROJECT_ROOT}/log/ros"
export PYTHONNOUSERSITE=1
export AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1
mkdir -p "${TMPDIR}" "${ROS_LOG_DIR}"

# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
set -u
cd "${PROJECT_ROOT}"

colcon build --symlink-install --event-handlers console_direct+
set +u
source "${PROJECT_ROOT}/install/setup.bash"
set -u
colcon test \
  --packages-select "${project_packages[@]}" \
  --event-handlers console_direct+

for package in "${project_packages[@]}"; do
  colcon test-result --test-result-base "build/${package}" --verbose
done

ament_copyright src/drone_* scripts
ament_cppcheck src/drone_*/include src/drone_*/src
ament_cpplint --filters=-legal/copyright,-build/include_order src/drone_*
ament_uncrustify src/drone_*
ament_flake8 src/drone_* scripts
ament_pep257 src/drone_* scripts
ament_lint_cmake src/drone_*
ament_xmllint src/drone_* scripts/m4-openbox.xml

cleanliness_arguments=()
if [[ "${CI_ALLOW_DIRTY:-0}" == "1" ]]; then
  cleanliness_arguments+=(--allow-dirty)
fi
python3 scripts/check_repository_cleanliness.py "${cleanliness_arguments[@]}"
