#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -ne 1 ]]; then
  echo "usage: $0 <px4-directory>" >&2
  exit 2
fi

px4_directory="$1"
build_directory="${px4_directory}/build/px4_sitl_default"
px4_binary="${build_directory}/bin/px4"
rootfs_directory="${build_directory}/rootfs"
px4_pid=""

stop_px4_process_group()
{
  local signal_name="$1"

  if [[ -n "${px4_pid}" ]] && kill -0 -- "-${px4_pid}" 2>/dev/null; then
    kill "-${signal_name}" -- "-${px4_pid}" 2>/dev/null || true
  fi
}

cleanup()
{
  local exit_status="$?"
  local attempt

  trap - EXIT INT TERM
  stop_px4_process_group INT
  for ((attempt = 0; attempt < 50; ++attempt)); do
    if ! kill -0 -- "-${px4_pid}" 2>/dev/null; then
      wait "${px4_pid}" 2>/dev/null || true
      exit "${exit_status}"
    fi
    sleep 0.1
  done

  stop_px4_process_group TERM
  for ((attempt = 0; attempt < 50; ++attempt)); do
    if ! kill -0 -- "-${px4_pid}" 2>/dev/null; then
      wait "${px4_pid}" 2>/dev/null || true
      exit "${exit_status}"
    fi
    sleep 0.1
  done

  stop_px4_process_group KILL
  wait "${px4_pid}" 2>/dev/null || true
  exit "${exit_status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

make -C "${px4_directory}" px4_sitl

if [[ ! -x "${px4_binary}" ]]; then
  echo "PX4 SITL binary not found after build: ${px4_binary}" >&2
  exit 1
fi

cd "${rootfs_directory}"
PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500}" GZ_IP="${GZ_IP:-127.0.0.1}" \
  setsid "${px4_binary}" -d &
px4_pid="$!"
wait "${px4_pid}"
