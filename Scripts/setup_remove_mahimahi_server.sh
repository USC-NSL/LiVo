#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/livo-env.sh"

if sudo iptables -C FORWARD -s "${LIVO_CLIENT_HOST}" -d "${LIVO_MAHIMAHI_HOST}" -j ACCEPT 2>/dev/null; then
    sudo iptables -D FORWARD -s "${LIVO_CLIENT_HOST}" -d "${LIVO_MAHIMAHI_HOST}" -j ACCEPT
fi
if sudo iptables -t nat -C PREROUTING -s "${LIVO_CLIENT_HOST}" -j DNAT --to-destination "${LIVO_MAHIMAHI_HOST}" 2>/dev/null; then
    sudo iptables -t nat -D PREROUTING -s "${LIVO_CLIENT_HOST}" -j DNAT --to-destination "${LIVO_MAHIMAHI_HOST}"
fi