#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/livo-env.sh"
cd "${SCRIPT_DIR}"
livo_require_path LIVO_TRACE_ROOT "${LIVO_TRACE_ROOT}"
rm -f mm_time

trace_path="${LIVO_TRACE_ROOT}"

# tracep1-scaled10.0
up_file=$trace_path/net-trace/mahimahi/laia-dst/tracep1-scaled10.0.txt
down_file=$trace_path/net-trace/mahimahi/laia-dst/tracep1-scaled10.0.txt

## wifi-<bw>.down
# up_file=$trace_path/net-trace/mahimahi/network/wifi-25-scaled15.0.down
# down_file=$trace_path/net-trace/mahimahi/network/wifi-25-scaled15.0.down

# up_file=$trace_path/mh-120-240-new
# down_file=$trace_path/mh-120-240-new

date +%s%N > mm_time
echo "$up_file" >> mm_time
echo "$down_file" >> mm_time
mm-link "$up_file" "$down_file" --meter-uplink --meter-downlink
