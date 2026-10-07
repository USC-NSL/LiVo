#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" && "${1:-}" != "--" ]]; then
  echo "Source this file, or run: install/livo-env.sh -- <command> [args...]" >&2
  exit 2
fi

LIVO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export LIVO_ROOT
export LIVO_DEPS_ROOT="${LIVO_ROOT}/Multiview/lib"

prepend_path() {
  local variable="$1"
  local value="$2"
  [[ -d "${value}" ]] || return 0
  local current="${!variable:-}"
  if [[ -n "${current}" ]]; then
    printf -v "${variable}" '%s:%s' "${value}" "${current}"
  else
    printf -v "${variable}" '%s' "${value}"
  fi
  export "${variable}"
}

for prefix in "${LIVO_DEPS_ROOT}"/*/install; do
  [[ -d "${prefix}" ]] || continue
  prepend_path PATH "${prefix}/bin"
  prepend_path CMAKE_PREFIX_PATH "${prefix}"
  prepend_path LD_LIBRARY_PATH "${prefix}/lib"
  prepend_path LD_LIBRARY_PATH "${prefix}/lib64"
  prepend_path PKG_CONFIG_PATH "${prefix}/lib/pkgconfig"
  prepend_path PKG_CONFIG_PATH "${prefix}/lib64/pkgconfig"
  prepend_path PKG_CONFIG_PATH "${prefix}/share/pkgconfig"
done

gst_prefix="${LIVO_DEPS_ROOT}/gstreamer/install"
if [[ -x "${gst_prefix}/bin/gst-inspect-1.0" ]]; then
  export LIVO_GSTREAMER_MODE=private
  export GST_PLUGIN_SYSTEM_PATH_1_0=
  export GST_PLUGIN_PATH_1_0="${gst_prefix}/lib/gstreamer-1.0"
  export GST_PLUGIN_PATH="${GST_PLUGIN_PATH_1_0}"
  export GST_PLUGIN_SCANNER_1_0="${gst_prefix}/libexec/gstreamer-1.0/gst-plugin-scanner"
  registry_dir="${LIVO_ROOT}/install/cache/gstreamer"
  mkdir -p "${registry_dir}"
  export GST_REGISTRY_1_0="${registry_dir}/registry-1.24.13-$(uname -m).bin"
else
  export LIVO_GSTREAMER_MODE=system
fi

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  shift
  exec "$@"
fi
