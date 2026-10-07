#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/livo-env.sh"

for port in 8080 8081 8082 8083 5252 5253; do
    sudo ufw allow "${port}/tcp"
done
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables --table filter --policy FORWARD ACCEPT
sudo iptables -t nat -C PREROUTING -s "${LIVO_CLIENT_HOST}" -j DNAT --to-destination "${LIVO_MAHIMAHI_HOST}" 2>/dev/null ||
    sudo iptables -t nat -A PREROUTING -s "${LIVO_CLIENT_HOST}" -j DNAT --to-destination "${LIVO_MAHIMAHI_HOST}"
sudo iptables -C FORWARD -s "${LIVO_CLIENT_HOST}" -d "${LIVO_MAHIMAHI_HOST}" -j ACCEPT 2>/dev/null ||
    sudo iptables -A FORWARD -s "${LIVO_CLIENT_HOST}" -d "${LIVO_MAHIMAHI_HOST}" -j ACCEPT