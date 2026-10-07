#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
LIB_DIR="${ROOT_DIR}/Multiview/lib"
TOOLS_DIR="${ROOT_DIR}/install/tools"
JOBS="${LIVO_BUILD_JOBS:-$(nproc)}"
BUILD_TYPE="${LIVO_BUILD_TYPE:-Release}"

log() { printf '[LiVo] %s\n' "$*"; }
die() { printf '[LiVo] ERROR: %s\n' "$*" >&2; exit 1; }

usage() {
  cat <<'EOF'
Usage: install/build-one.sh <dependency> [--clean|--rebuild]

Dependencies:
  eigen boost zstd opencv pcl open3d draco flatbuffers gflags usrsctp
  abseil json azure-kinect gstreamer
EOF
}

[[ $# -ge 1 ]] || { usage; exit 2; }
name="$1"
mode="${2:-}"
case "${name}" in
  abseil) source_dir="${LIB_DIR}/abseil-cpp" ;;
  *) source_dir="${LIB_DIR}/${name}" ;;
esac
build_dir="${source_dir}/build-livo"
prefix="${source_dir}/install"

[[ -f "${source_dir}/.git" || -d "${source_dir}/.git" ]] ||
  die "Source submodule is missing: ${source_dir}. Run install/fetch.sh first."

if [[ "${mode}" == "--clean" || "${mode}" == "--rebuild" ]]; then
  rm -rf "${build_dir}" "${prefix}"
fi
mkdir -p "${build_dir}" "${prefix}" "${ROOT_DIR}/install/logs"
log_file="${ROOT_DIR}/install/logs/build-${name}.log"

prefixes=()
for candidate in "${LIB_DIR}"/*/install; do
  [[ -d "${candidate}" && "${candidate}" != "${prefix}" ]] && prefixes+=("${candidate}")
done
cmake_prefix=""
if ((${#prefixes[@]})); then
  cmake_prefix="$(IFS=';'; printf '%s' "${prefixes[*]}")"
fi
pkgconfig_paths=()
for dependency_prefix in "${prefixes[@]}"; do
  for pkgdir in lib/pkgconfig lib64/pkgconfig share/pkgconfig; do
    [[ -d "${dependency_prefix}/${pkgdir}" ]] &&
      pkgconfig_paths+=("${dependency_prefix}/${pkgdir}")
  done
done
if ((${#pkgconfig_paths[@]})); then
  pkgconfig_prefix="$(IFS=:; printf '%s' "${pkgconfig_paths[*]}")"
  export PKG_CONFIG_PATH="${pkgconfig_prefix}${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"
fi

cmake_configure() {
  cmake -S "$1" -B "${build_dir}" -GNinja \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX="${prefix}" \
    -DCMAKE_PREFIX_PATH="${cmake_prefix}" \
    "${@:2}"
}

cmake_install() {
  cmake --build "${build_dir}" --parallel "${JOBS}"
  cmake --install "${build_dir}"
}

build_dependency() {
  case "${name}" in
    eigen)
      cmake_configure "${source_dir}" \
        -DBUILD_TESTING=OFF -DEIGEN_BUILD_DOC=OFF -DEIGEN_BUILD_PKGCONFIG=ON
      cmake_install
      ;;
    boost)
      (
        cd "${source_dir}"
        ./bootstrap.sh --prefix="${prefix}" \
          --with-libraries=program_options,filesystem,date_time,chrono,serialization,system,json,iostreams,regex,thread
        ./b2 -j"${JOBS}" variant=release link=shared runtime-link=shared \
          cxxflags="-std=c++17" install
      )
      ;;
    zstd)
      cmake_configure "${source_dir}/build/cmake" \
        -DZSTD_BUILD_PROGRAMS=OFF -DZSTD_BUILD_TESTS=OFF \
        -DZSTD_BUILD_SHARED=ON -DZSTD_BUILD_STATIC=ON
      cmake_install
      ;;
    opencv)
      contrib="${LIB_DIR}/opencv_contrib/modules"
      [[ -d "${contrib}" ]] || die "opencv_contrib submodule is missing"
      cmake_configure "${source_dir}" \
        -DOPENCV_EXTRA_MODULES_PATH="${contrib}" \
        -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF \
        -DBUILD_opencv_apps=OFF -DBUILD_JAVA=OFF -DBUILD_opencv_python=OFF \
        -DWITH_CUDA=ON -DWITH_OPENMP=ON -DWITH_OPENGL=ON \
        -DWITH_FFMPEG=OFF -DOPENCV_GENERATE_PKGCONFIG=ON \
        -DBUILD_JPEG=ON -DBUILD_PNG=ON
      cmake_install
      ;;
    pcl)
      cmake_configure "${source_dir}" \
        -DCMAKE_CXX_STANDARD=17 \
        -DBUILD_SHARED_LIBS=ON -DBUILD_TESTS=OFF -DBUILD_examples=OFF \
        -DBUILD_CUDA=ON -DBUILD_GPU=ON -DWITH_CUDA=ON \
        -DBUILD_common=ON -DBUILD_octree=ON -DBUILD_filters=ON \
        -DBUILD_geometry=ON -DBUILD_io=ON -DBUILD_segmentation=ON \
        -DBUILD_visualization=ON -DBUILD_registration=ON -DBUILD_apps=ON \
        -DBUILD_gpu_kinfu_tools=OFF \
        -DBUILD_gpu_kinfu_large_scale_tools=OFF
      cmake_install
      ;;
    open3d)
      cmake_configure "${source_dir}" \
        -DBUILD_SHARED_LIBS=ON -DBUILD_PYTHON_MODULE=OFF \
        -DBUILD_EXAMPLES=OFF -DBUILD_UNIT_TESTS=OFF \
        -DBUILD_GUI=ON -DBUILD_CUDA_MODULE=ON \
        -DBUILD_WEBRTC=OFF \
        -DGLIBCXX_USE_CXX11_ABI=ON
      # Materialize external archives whose Open3D 0.18 Ninja byproduct
      # declarations are missing or use a different filename.
      cmake --build "${build_dir}" --parallel "${JOBS}" \
        --target ext_curl ext_vtk ext_uvatlas
      cmake_install
      ;;
    draco)
      cmake_configure "${source_dir}" \
        -DBUILD_SHARED_LIBS=ON -DDRACO_TESTS=OFF -DDRACO_EXAMPLES=OFF
      cmake_install
      ;;
    flatbuffers)
      cmake_configure "${source_dir}" \
        -DFLATBUFFERS_BUILD_TESTS=OFF -DFLATBUFFERS_BUILD_GRPCTEST=OFF \
        -DFLATBUFFERS_BUILD_SHAREDLIB=ON -DFLATBUFFERS_BUILD_STATICLIB=ON
      cmake_install
      ;;
    gflags)
      cmake_configure "${source_dir}" \
        -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF \
        -DGFLAGS_BUILD_TESTING=OFF -DGFLAGS_BUILD_gflags_LIB=ON \
        -DGFLAGS_BUILD_gflags_nothreads_LIB=OFF
      cmake_install
      ;;
    abseil)
      cmake_configure "${source_dir}" \
        -DCMAKE_CXX_STANDARD=17 -DBUILD_SHARED_LIBS=ON \
        -DABSL_PROPAGATE_CXX_STD=ON -DABSL_ENABLE_INSTALL=ON \
        -DABSL_BUILD_TESTING=OFF
      cmake_install
      ;;
    json)
      cmake_configure "${source_dir}" \
        -DJSON_BuildTests=OFF -DJSON_Install=ON
      cmake_install
      ;;
    usrsctp)
      cmake_configure "${source_dir}" \
        -Dsctp_build_programs=OFF -Dsctp_build_shared_lib=ON \
        -Dsctp_werror=OFF -Dsctp_inet=ON -Dsctp_inet6=ON
      cmake_install
      ;;
    azure-kinect)
      cmake_configure "${source_dir}" \
        -DK4A_BUILD_TESTS=OFF -DK4A_BUILD_EXAMPLES=OFF \
        -DK4A_BUILD_TOOLS=ON
      cmake --build "${build_dir}" --parallel "${JOBS}"
      if cmake --install "${build_dir}"; then
        :
      else
        log "Azure Kinect has no complete install target; normalizing build outputs"
      fi
      mkdir -p "${prefix}/include" "${prefix}/lib"
      cp -a "${source_dir}/include/." "${prefix}/include/"
      shopt -s nullglob
      k4a_libraries=(
        "${build_dir}"/bin/libk4a.so*
        "${build_dir}"/bin/libk4arecord.so*
        "${build_dir}"/lib/libk4a.so*
        "${build_dir}"/lib/libk4arecord.so*
      )
      ((${#k4a_libraries[@]})) || die "Azure Kinect libraries were not produced"
      cp -a "${k4a_libraries[@]}" "${prefix}/lib/"
      shopt -u nullglob
      depthengine="$(find "${source_dir}" -type f -name 'libdepthengine.so.2.0' -print -quit)"
      [[ -n "${depthengine}" ]] || die "Azure Kinect libdepthengine.so.2.0 was not found"
      cp -a "${depthengine}" "${prefix}/lib/"
      ;;
    gstreamer)
      venv="${TOOLS_DIR}/venv"
      if [[ -x "${venv}/bin/meson" ]]; then
        export PATH="${venv}/bin:${PATH}"
      elif command -v meson >/dev/null 2>&1; then
        meson_version="$(meson --version)"
        [[ "$(printf '%s\n%s\n' "1.1.0" "${meson_version}" | sort -V | head -n1)" == "1.1.0" ]] ||
          die "Meson 1.1+ is required; found ${meson_version}"
        log "Private Meson is unavailable; using $(command -v meson) ${meson_version}"
      else
        die "Meson 1.1+ is required. Run install/install-deps.sh first."
      fi
      # nvcodec uses std::thread directly; older Ubuntu toolchains do not
      # always propagate the pthread link flag through Meson's CUDA checks.
      export CFLAGS="${CFLAGS:-} -pthread"
      export CXXFLAGS="${CXXFLAGS:-} -pthread"
      export LDFLAGS="${LDFLAGS:-} -pthread"
      gst_options=(
        --prefix="${prefix}" --libdir=lib --buildtype=release
        --default-library=shared --wrap-mode=default
        --force-fallback-for=glib,libnice,orc
        -Dauto_features=disabled
        -Dtools=enabled
        -Dtests=disabled -Dexamples=disabled -Ddoc=disabled
        -Dintrospection=disabled -Dnls=disabled
        -Dbase=enabled -Dgood=enabled -Dbad=enabled
        -Dugly=disabled -Dlibav=disabled -Drs=disabled
        -Dpython=disabled -Ddevtools=disabled
        -Dlibnice=enabled -Dwebrtc=enabled
        -Dlibnice:gstreamer=enabled
        -Dgst-plugins-base:app=enabled
        -Dgst-plugins-base:videoconvertscale=enabled
        -Dgst-plugins-base:videotestsrc=enabled
        -Dgst-plugins-base:audiotestsrc=enabled
        -Dgst-plugins-base:opus=enabled
        -Dgst-plugins-good:rtp=enabled
        -Dgst-plugins-good:rtpmanager=enabled
        -Dgst-plugins-good:vpx=enabled
        -Dgst-plugins-bad:videoparsers=enabled
        -Dgst-plugins-bad:webrtc=enabled
        -Dgst-plugins-bad:dtls=enabled
        -Dgst-plugins-bad:sctp=enabled
        -Dgst-plugins-bad:srtp=enabled
        -Dgst-plugins-bad:nvcodec=enabled
      )
      if [[ -f "${build_dir}/build.ninja" ]]; then
        meson setup --reconfigure "${build_dir}" "${source_dir}" "${gst_options[@]}"
      else
        meson setup "${build_dir}" "${source_dir}" "${gst_options[@]}"
      fi
      meson compile -C "${build_dir}" -j "${JOBS}"
      meson install -C "${build_dir}"
      ;;
    *)
      usage
      die "Unknown dependency: ${name}"
      ;;
  esac
}

log "Building ${name}; log: ${log_file}"
build_dependency 2>&1 | tee "${log_file}"
log "Installed ${name} under ${prefix}"
