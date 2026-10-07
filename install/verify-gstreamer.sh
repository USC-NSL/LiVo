#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/install/livo-env.sh"

command -v gst-inspect-1.0 >/dev/null ||
  { echo "[LiVo] ERROR: gst-inspect-1.0 is unavailable" >&2; exit 1; }

echo "[LiVo] $(gst-inspect-1.0 --version | sed -n '1p')"
elements=(
  appsrc appsink queue videoconvert
  webrtcbin nicesrc nicesink dtlsenc dtlsdec srtpenc srtpdec
  rtpbin rtph265pay rtph265depay h265parse nvh265enc nvh265dec
)

failed=0
for element in "${elements[@]}"; do
  if gst-inspect-1.0 "${element}" >/dev/null 2>&1; then
    echo "[LiVo] GStreamer element ready: ${element}"
  else
    echo "[LiVo] ERROR: missing GStreamer element ${element}" >&2
    failed=1
  fi
done

for package in gstreamer-1.0 gstreamer-app-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0; do
  pkg-config --exists "${package}" ||
    { echo "[LiVo] ERROR: pkg-config cannot resolve ${package}" >&2; failed=1; continue; }
  echo "[LiVo] ${package} $(pkg-config --modversion "${package}")"
done

if [[ "${LIVO_GSTREAMER_MODE}" == "private" ]]; then
  gst_prefix="${ROOT_DIR}/Multiview/lib/gstreamer/install"
  for plugin in webrtc nvcodec; do
    plugin_info="$(gst-inspect-1.0 "${plugin}" 2>/dev/null)"
    filename="$(awk '$1 == "Filename" {print $2; exit}' <<<"${plugin_info}")"
    if [[ "${filename}" != "${gst_prefix}"/* ]]; then
      echo "[LiVo] ERROR: ${plugin} loaded outside private prefix: ${filename}" >&2
      failed=1
    fi
  done
fi

encoder_info="$(gst-inspect-1.0 nvh265enc 2>/dev/null || true)"
for capability in Y444_16LE bitrate rc-mode zerolatency; do
  if ! grep -q "${capability}" <<<"${encoder_info}"; then
    echo "[LiVo] ERROR: nvh265enc does not advertise ${capability}" >&2
    failed=1
  fi
done

gst-inspect-1.0 -b || true
exit "${failed}"
