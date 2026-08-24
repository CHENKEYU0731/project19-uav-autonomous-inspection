#!/usr/bin/env bash
set -Eeuo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_video="${1:-${repo_root}/docs/assets/m4-inspection-demo.mp4}"
timestamp="$(date +%Y%m%d_%H%M%S)"
bag_directory="${2:-${repo_root}/log/m4/inspection_video_${timestamp}}"
work_directory="${repo_root}/.tmp/m4-video/recording_${timestamp}"
raw_video="${work_directory}/m4-inspection-demo-raw.mp4"
display_number="${M4_VIRTUAL_DISPLAY:-98}"
display=":${display_number}"
screen_width=2400
screen_height=1080
half_width=$((screen_width / 2))

set +u
source "${repo_root}/scripts/project-env.sh"
set -u

for command in Xvfb openbox wmctrl ffmpeg ffprobe ros2; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "required recording command is unavailable: ${command}" >&2
    exit 1
  fi
done
if [[ "${M4_RECORDING_CHECK_ONLY:-0}" == "1" ]]; then
  echo "M4 recording prerequisites are available"
  exit 0
fi

if [[ -e "${output_video}" ]]; then
  echo "refusing to overwrite existing video: ${output_video}" >&2
  exit 1
fi
if [[ -e "${bag_directory}" ]]; then
  echo "refusing to overwrite existing bag: ${bag_directory}" >&2
  exit 1
fi

mkdir -p "${work_directory}" "$(dirname "${output_video}")" "$(dirname "${bag_directory}")"

export DISPLAY="${display}"
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe

xvfb_pid=""
openbox_pid=""
launch_pid=""
ffmpeg_pid=""

stop_process() {
  local signal="$1"
  local pid="$2"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal}" "${pid}" 2>/dev/null || true
    for _ in $(seq 1 50); do
      if ! kill -0 "${pid}" 2>/dev/null; then
        wait "${pid}" 2>/dev/null || true
        return
      fi
      sleep 0.1
    done
    kill -KILL "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
}

stop_process_group() {
  local signal="$1"
  local pid="$2"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal}" -- "-${pid}" 2>/dev/null || true
    for _ in $(seq 1 100); do
      if ! kill -0 "${pid}" 2>/dev/null; then
        wait "${pid}" 2>/dev/null || true
        return
      fi
      sleep 0.1
    done
    kill -KILL -- "-${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  stop_process INT "${ffmpeg_pid}"
  stop_process_group INT "${launch_pid}"
  stop_process TERM "${openbox_pid}"
  stop_process_group TERM "${xvfb_pid}"
  pkill -KILL -f "^Xvfb ${display} " 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT INT TERM

if xdpyinfo -display "${display}" >/dev/null 2>&1; then
  echo "virtual display is already in use: ${display}" >&2
  exit 1
fi

setsid Xvfb "${display}" \
  -screen 0 "${screen_width}x${screen_height}x24" \
  +extension GLX +render -noreset \
  >"${work_directory}/xvfb.log" 2>&1 &
xvfb_pid=$!

for _ in $(seq 1 50); do
  if xdpyinfo -display "${display}" >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done
if ! xdpyinfo -display "${display}" >/dev/null 2>&1; then
  echo "Xvfb did not become ready on ${display}" >&2
  exit 1
fi

openbox --config-file "${repo_root}/scripts/m4-openbox.xml" \
  >"${work_directory}/openbox.log" 2>&1 &
openbox_pid=$!
for _ in $(seq 1 50); do
  if wmctrl -m >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done
if ! wmctrl -m >/dev/null 2>&1; then
  echo "Openbox did not become ready on ${display}" >&2
  exit 1
fi

setsid ros2 launch drone_bringup m4_inspection.launch.py \
  "bag_directory:=${bag_directory}" \
  use_rviz:=true \
  use_gazebo_gui:=true \
  demo_start_delay_s:=75 \
  >"${work_directory}/launch.log" 2>&1 &
launch_pid=$!

gazebo_window=""
rviz_window=""
for _ in $(seq 1 180); do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "M4 launch exited before both display windows became ready" >&2
    wait "${launch_pid}" || true
    exit 1
  fi
  window_list="$(wmctrl -lx 2>/dev/null || true)"
  gazebo_window="$(awk '$3 == "gz-sim-gui.Gazebo" {print $1; exit}' <<<"${window_list}")"
  rviz_window="$(awk '$3 == "rviz2.rviz2" {print $1; exit}' <<<"${window_list}")"
  if [[ -n "${gazebo_window}" && -n "${rviz_window}" ]]; then
    break
  fi
  sleep 0.5
done
if [[ -z "${gazebo_window}" || -z "${rviz_window}" ]]; then
  echo "Gazebo and RViz2 windows were not both ready within 90 seconds" >&2
  exit 1
fi

for window in "${gazebo_window}" "${rviz_window}"; do
  wmctrl -i -r "${window}" -b remove,maximized_vert,maximized_horz || true
done
wmctrl -i -r "${gazebo_window}" -e "0,0,0,${half_width},${screen_height}"
wmctrl -i -r "${rviz_window}" -e "0,${half_width},0,${half_width},${screen_height}"
sleep 3
wmctrl -lx >"${work_directory}/windows.txt"
window_geometry="$(wmctrl -lGx)"
gazebo_geometry="$(awk '$0 ~ /gz-sim-gui\.Gazebo/ {print $3, $4, $5, $6; exit}' <<<"${window_geometry}")"
rviz_geometry="$(awk '$0 ~ /rviz2\.rviz2/ {print $3, $4, $5, $6; exit}' <<<"${window_geometry}")"
read -r gazebo_x gazebo_y gazebo_width gazebo_height <<<"${gazebo_geometry}"
read -r rviz_x rviz_y rviz_width rviz_height <<<"${rviz_geometry}"
if [[ -z "${gazebo_width:-}" || -z "${rviz_width:-}" ]] ||
  ((gazebo_x > 5 || gazebo_y > 5 || gazebo_width < 900 || gazebo_height < 720 ||
    gazebo_x + gazebo_width > half_width + 5 ||
    rviz_x < half_width - 5 || rviz_y > 5 || rviz_width < 1100 ||
    rviz_height < 720 || rviz_x + rviz_width > screen_width + 5 ||
    gazebo_x + gazebo_width > rviz_x)); then
  echo "Gazebo and RViz2 did not form a non-overlapping side-by-side layout" >&2
  echo "Gazebo geometry: ${gazebo_geometry:-missing}" >&2
  echo "RViz2 geometry: ${rviz_geometry:-missing}" >&2
  exit 1
fi
printf '%s\n' "${window_geometry}" >"${work_directory}/window-geometry.txt"
if [[ "${M4_RECORDING_LAYOUT_ONLY:-0}" == "1" ]]; then
  echo "M4 side-by-side layout is ready"
  exit 0
fi

if ! timeout 150s ros2 topic echo --once /fmu/out/vehicle_odometry \
  >"${work_directory}/odometry-ready.txt" 2>&1; then
  echo "PX4 odometry did not become ready before video recording" >&2
  exit 1
fi

ffmpeg -hide_banner -loglevel warning \
  -f x11grab -framerate 12 \
  -video_size "${screen_width}x${screen_height}" -i "${display}" \
  -an -c:v libx264 -preset ultrafast -crf 24 -pix_fmt yuv420p \
  -movflags +faststart -y "${raw_video}" \
  >"${work_directory}/ffmpeg.log" 2>&1 &
ffmpeg_pid=$!

set +e
wait "${launch_pid}"
launch_exit_code=$?
set -e
launch_pid=""
stop_process INT "${ffmpeg_pid}"
ffmpeg_pid=""

if [[ ${launch_exit_code} -ne 0 ]]; then
  echo "M4 launch failed with exit code ${launch_exit_code}" >&2
  exit "${launch_exit_code}"
fi
if [[ ! -s "${raw_video}" ]]; then
  echo "video recorder produced no data" >&2
  exit 1
fi

duration="$(ffprobe -v error -show_entries format=duration \
  -of default=noprint_wrappers=1:nokey=1 "${raw_video}")"
if awk -v duration="${duration}" 'BEGIN {exit !(duration > 178.0)}'; then
  speed_factor="$(awk -v duration="${duration}" 'BEGIN {printf "%.9f", 175.0 / duration}')"
  ffmpeg -hide_banner -loglevel warning -i "${raw_video}" \
    -an -vf "setpts=${speed_factor}*PTS" \
    -c:v libx264 -preset medium -crf 22 -pix_fmt yuv420p \
    -movflags +faststart -y "${output_video}"
else
  ffmpeg -hide_banner -loglevel warning -i "${raw_video}" \
    -map 0:v:0 -an -c copy -movflags +faststart -y "${output_video}"
fi

python3 "${repo_root}/scripts/analyze_m4_mission.py" \
  "${bag_directory}" \
  --metrics "${bag_directory}/m4_metrics.json" \
  --launch-exit-code "${launch_exit_code}"
python3 "${repo_root}/scripts/verify_m4_video.py" \
  "${output_video}" \
  --metrics "${output_video%.mp4}-metrics.json" \
  --contact-sheet "${output_video%.mp4}-contact-sheet.png" \
  --work-dir "${work_directory}/verification"

echo "M4 demo video: ${output_video}"
echo "M4 evidence bag: ${bag_directory}"
echo "recording work directory: ${work_directory}"
