#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/livo-env.sh"
cd "${SCRIPT_DIR}"
livo_require_path LIVO_TRACE_ROOT "${LIVO_TRACE_ROOT}"

rm -f mm_time
up_file="${LIVO_TRACE_ROOT}/mh-120-240"
down_file="${LIVO_TRACE_ROOT}/mh-120-240"
date +%s%N > mm_time
echo "${up_file}" >> mm_time
echo "${down_file}" >> mm_time
mm-link "${up_file}" "${down_file}" --meter-uplink --meter-downlink