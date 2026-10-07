#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-}"

case "${MODE}" in
  ""|--resume) child_mode="" ;;
  --rebuild|--clean) child_mode="--rebuild" ;;
  *)
    printf 'Usage: install/build-all.sh [--resume|--rebuild]\n' >&2
    exit 2
    ;;
esac

dependencies=(
  eigen
  boost
  zstd
  opencv
  pcl
  open3d
  draco
  flatbuffers
  gflags
  abseil
  json
  azure-kinect
  usrsctp
  gstreamer
)

"${ROOT_DIR}/install/fetch.sh"
for dependency in "${dependencies[@]}"; do
  prefix_name="${dependency}"
  [[ "${dependency}" == "abseil" ]] && prefix_name="abseil-cpp"
  prefix="${ROOT_DIR}/Multiview/lib/${prefix_name}/install"
  if [[ -z "${child_mode}" && -d "${prefix}" && -n "$(find "${prefix}" -mindepth 1 -print -quit)" ]]; then
    printf '[LiVo] Reusing installed dependency: %s\n' "${dependency}"
    continue
  fi
  "${ROOT_DIR}/install/build-one.sh" "${dependency}" ${child_mode:+"${child_mode}"}
done

"${ROOT_DIR}/install/verify.sh" --strict-local
