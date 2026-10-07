#!/usr/bin/env bash

LIVO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export LIVO_ROOT

site_file="${LIVO_SITE_CONFIG:-${LIVO_ROOT}/Scripts/livo-site.sh}"
if [[ -f "${site_file}" ]]; then
  # shellcheck disable=SC1090
  source "${site_file}"
fi

export LIVO_BUILD_DIR="${LIVO_BUILD_DIR:-${LIVO_ROOT}/build}"
export LIVO_CONFIG_DIR="${LIVO_CONFIG_DIR:-${LIVO_ROOT}/Multiview/config}"
export LIVO_SERVER_HOST="${LIVO_SERVER_HOST:-127.0.0.1}"
export LIVO_CLIENT_HOST="${LIVO_CLIENT_HOST:-127.0.0.1}"
export LIVO_MAHIMAHI_HOST="${LIVO_MAHIMAHI_HOST:-100.64.0.2}"
export LIVO_DATA_ROOT="${LIVO_DATA_ROOT:-/path/to/panoptic_data}"
export LIVO_USER_TRACE_ROOT="${LIVO_USER_TRACE_ROOT:-${LIVO_ROOT}/data}"
export LIVO_OUTPUT_ROOT="${LIVO_OUTPUT_ROOT:-${LIVO_ROOT}/output}"
export LIVO_TRACE_ROOT="${LIVO_TRACE_ROOT:-/path/to/mahimahi-traces}"
export LIVO_SERVER_CPUS="${LIVO_SERVER_CPUS:-0-11}"
export LIVO_CLIENT_CPUS="${LIVO_CLIENT_CPUS:-0-19}"
export LIVO_DRACO_CPUS="${LIVO_DRACO_CPUS:-0-3}"

# Activate private dependencies when present; otherwise leave system resolution
# available.
# shellcheck disable=SC1091
source "${LIVO_ROOT}/install/livo-env.sh"

livo_require_path() {
  local name="$1"
  local value="$2"
  if [[ "${value}" == /path/to/* || ! -e "${value}" ]]; then
    echo "[LiVo] ERROR: ${name} is not configured: ${value}" >&2
    echo "[LiVo] Copy Scripts/livo-site.example.sh to Scripts/livo-site.sh and edit it." >&2
    return 1
  fi
}
