#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${LIVO_BUILD_DIR:-${ROOT_DIR}/build}"
JOBS="${LIVO_BUILD_JOBS:-$(nproc)}"

# shellcheck disable=SC1091
source "${ROOT_DIR}/install/livo-env.sh"

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -GNinja \
  -DCMAKE_BUILD_TYPE=Release \
  -DLIVO_DEPS_ROOT="${ROOT_DIR}/Multiview/lib" \
  "$@"
cmake --build "${BUILD_DIR}" --parallel "${JOBS}"

echo "[LiVo] LiVo sources built under ${BUILD_DIR}; no dependency build is part of this target graph."
