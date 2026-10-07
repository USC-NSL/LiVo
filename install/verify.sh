#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
STRICT_LOCAL=0
[[ "${1:-}" == "--strict-local" ]] && STRICT_LOCAL=1

required=(opencv open3d pcl gstreamer usrsctp azure-kinect eigen boost flatbuffers zstd draco abseil-cpp gflags json)
if [[ "${STRICT_LOCAL}" == "1" ]]; then
  for dependency in "${required[@]}"; do
    prefix="${ROOT_DIR}/Multiview/lib/${dependency}/install"
    [[ -d "${prefix}" && -n "$(find "${prefix}" -mindepth 1 -print -quit)" ]] ||
      { echo "[LiVo] ERROR: local installation is missing: ${prefix}" >&2; exit 1; }
  done
fi

# shellcheck disable=SC1091
source "${ROOT_DIR}/install/livo-env.sh"

tmp_dir="$(mktemp -d)"
trap 'rm -rf "${tmp_dir}"' EXIT
cat > "${tmp_dir}/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.20)
project(LiVoDependencyProbe LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost 1.80 REQUIRED COMPONENTS program_options filesystem date_time chrono serialization system json)
find_package(OpenCV REQUIRED CONFIG)
find_package(PCL 1.12 REQUIRED)
find_package(Open3D REQUIRED CONFIG)
find_package(draco REQUIRED CONFIG)
find_package(flatbuffers REQUIRED CONFIG)
find_package(gflags REQUIRED CONFIG)
find_package(absl REQUIRED CONFIG)
find_package(nlohmann_json REQUIRED CONFIG)
message(STATUS "LiVo probe Eigen3=${Eigen3_VERSION}")
message(STATUS "LiVo probe Boost=${Boost_VERSION}")
message(STATUS "LiVo probe OpenCV=${OpenCV_VERSION}")
message(STATUS "LiVo probe PCL=${PCL_VERSION}")
message(STATUS "LiVo probe Open3D=${Open3D_VERSION}")
EOF

cmake -S "${tmp_dir}" -B "${tmp_dir}/build" -GNinja
pkg-config --exists libzstd
pkg-config --exists usrsctp

k4a_prefix="${ROOT_DIR}/Multiview/lib/azure-kinect/install"
if [[ -d "${k4a_prefix}" ]]; then
  [[ -f "${k4a_prefix}/include/k4a/k4a.h" ]] ||
    { echo "[LiVo] ERROR: Azure Kinect headers are missing" >&2; exit 1; }
  find "${k4a_prefix}/lib" -name 'libk4a.so*' -print -quit | grep -q . ||
    { echo "[LiVo] ERROR: libk4a is missing" >&2; exit 1; }
  [[ -f "${k4a_prefix}/lib/libdepthengine.so.2.0" ]] ||
    { echo "[LiVo] ERROR: libdepthengine.so.2.0 is missing" >&2; exit 1; }
fi

"${ROOT_DIR}/install/verify-gstreamer.sh"

manifest="${ROOT_DIR}/install/dependencies.ready"
{
  echo "verified_at=$(date --iso-8601=seconds)"
  echo "cmake=$(cmake --version | awk 'NR==1 {print $3}')"
  echo "compiler=$(c++ --version | sed -n '1p')"
  echo "cuda=$(nvcc --version 2>/dev/null | sed -n 's/.*release \([^,]*\).*/\1/p' || true)"
  git -C "${ROOT_DIR}" submodule status
} > "${manifest}"
echo "[LiVo] Dependency verification passed; wrote ${manifest}"
