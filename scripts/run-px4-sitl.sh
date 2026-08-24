#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -lt 1 || "$#" -gt 2 ]]; then
  echo "usage: $0 <px4-directory> [px4-startup-script]" >&2
  exit 2
fi

px4_directory="$1"
px4_startup_script="${2:-}"
build_directory="${px4_directory}/build/px4_sitl_default"
px4_binary="${build_directory}/bin/px4"
rootfs_directory="${build_directory}/rootfs"
data_directory="${px4_directory}/ROMFS/px4fmu_common"
px4_pid=""

stop_px4_process_group()
{
  local signal_name="$1"

  if [[ -n "${px4_pid}" ]] && kill -0 -- "-${px4_pid}" 2>/dev/null; then
    kill "-${signal_name}" -- "-${px4_pid}" 2>/dev/null || true
  fi
}

reap_exited_group_leader()
{
  local process_state

  [[ -n "${px4_pid}" ]] || return 0
  if [[ -r "/proc/${px4_pid}/stat" ]]; then
    read -r _ _ process_state _ < "/proc/${px4_pid}/stat" || process_state=""
  else
    process_state=""
  fi
  if [[ -z "${process_state}" || "${process_state}" == Z ]]; then
    wait "${px4_pid}" 2>/dev/null || true
  fi
}

wait_for_px4_process_group()
{
  local deadline=$((SECONDS + 2))

  while ((SECONDS < deadline)); do
    reap_exited_group_leader
    if ! kill -0 -- "-${px4_pid}" 2>/dev/null; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

cleanup()
{
  local exit_status="$?"
  local initial_signal="INT"

  trap - EXIT
  trap 'stop_px4_process_group INT' INT
  trap 'stop_px4_process_group TERM' TERM
  if [[ "${exit_status}" -eq 143 ]]; then
    initial_signal="TERM"
  fi

  stop_px4_process_group "${initial_signal}"
  if wait_for_px4_process_group; then
    exit "${exit_status}"
  fi

  if [[ "${initial_signal}" != "TERM" ]]; then
    stop_px4_process_group TERM
    if wait_for_px4_process_group; then
      exit "${exit_status}"
    fi
  fi

  stop_px4_process_group KILL
  wait "${px4_pid}" 2>/dev/null || true
  exit "${exit_status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid make -C "${px4_directory}" px4_sitl &
px4_pid="$!"
wait "${px4_pid}"
px4_pid=""

if [[ ! -x "${px4_binary}" ]]; then
  echo "PX4 SITL binary not found after build: ${px4_binary}" >&2
  exit 1
fi

if [[ -n "${px4_startup_script}" && ! -f "${px4_startup_script}" ]]; then
  echo "PX4 startup script not found: ${px4_startup_script}" >&2
  exit 1
fi

cd "${rootfs_directory}"
if [[ -n "${px4_startup_script}" ]]; then
  PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500}" GZ_IP="${GZ_IP:-127.0.0.1}" \
    setsid "${px4_binary}" -d -s "${px4_startup_script}" \
      "${data_directory}" &
else
  PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500}" GZ_IP="${GZ_IP:-127.0.0.1}" \
    setsid "${px4_binary}" -d "${data_directory}" &
fi
px4_pid="$!"
wait "${px4_pid}"
