#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TOOLS_DIR="${ROOT_DIR}/install/tools"
CMAKE_VERSION="3.31.12"
MIN_CMAKE_VERSION="3.20.0"
NONINTERACTIVE="${LIVO_NONINTERACTIVE:-0}"
CMAKE_CHOICE="${LIVO_INSTALL_CMAKE:-ask}"

log() { printf '[LiVo] %s\n' "$*"; }
die() { printf '[LiVo] ERROR: %s\n' "$*" >&2; exit 1; }

[[ -r /etc/os-release ]] || die "Cannot identify Ubuntu: /etc/os-release is missing"
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == "ubuntu" ]] || die "Supported systems are Ubuntu 18.04, 20.04, and 22.04"
case "${VERSION_ID:-}" in
  18.04|20.04|22.04) ;;
  *) die "Unsupported Ubuntu release ${VERSION_ID:-unknown}; expected 18.04, 20.04, or 22.04" ;;
esac

COMMON_PACKAGES=(
  build-essential git git-lfs ca-certificates curl wget xz-utils unzip
  pkg-config autoconf automake libtool m4 perl flex bison gettext
  ninja-build python3 python3-dev python3-pip python3-venv
  libssl-dev libsrtp2-dev libopus-dev libvpx-dev libffi-dev
  zlib1g-dev libpcre2-dev libmount-dev libselinux1-dev
  libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
  libgl1-mesa-dev libglu1-mesa-dev xorg-dev libglew-dev libglfw3-dev
  libgtk-3-dev qtbase5-dev
  libudev-dev libusb-1.0-0-dev libturbojpeg0-dev libflann-dev libqhull-dev
  libjpeg-dev libpng-dev libtiff-dev libtbb-dev libopenni2-dev libpcap-dev
  libasound2-dev libsdl2-dev libsoundio-dev
  libavcodec-dev libavformat-dev libavutil-dev libavfilter-dev
  libavdevice-dev libswscale-dev libswresample-dev
  libbz2-dev libreadline-dev libsqlite3-dev libncursesw5-dev tk-dev
  libxml2-dev libxmlsec1-dev liblzma-dev
  libomp-dev chrony iperf3 ufw net-tools mahimahi
)

case "${VERSION_ID}" in
  18.04|20.04)
    VERSION_PACKAGES=(libvtk7-dev libvtk7-qt-dev)
    ;;
  22.04)
    VERSION_PACKAGES=(libvtk9-dev libvtk9-qt-dev gcc-10 g++-10)
    ;;
esac

log "Installing Ubuntu ${VERSION_ID} system prerequisites"
sudo apt-get update

AVAILABLE_PACKAGES=()
for package in "${COMMON_PACKAGES[@]}" "${VERSION_PACKAGES[@]}"; do
  if apt-cache show "$package" >/dev/null 2>&1; then
    AVAILABLE_PACKAGES+=("$package")
  else
    log "Package is unavailable on Ubuntu ${VERSION_ID}; skipping: ${package}"
  fi
done
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "${AVAILABLE_PACKAGES[@]}"

version_ge() {
  [[ "$(printf '%s\n%s\n' "$2" "$1" | sort -V | head -n1)" == "$2" ]]
}

existing_cmake=""
existing_cmake_version=""
if command -v cmake >/dev/null 2>&1; then
  existing_cmake="$(command -v cmake)"
  existing_cmake_version="$(cmake --version | awk 'NR==1 {print $3}')"
  log "Found CMake ${existing_cmake_version} at ${existing_cmake}"
else
  log "CMake is not currently available"
fi

install_cmake=0
case "${CMAKE_CHOICE}" in
  yes|1|true) install_cmake=1 ;;
  no|0|false) install_cmake=0 ;;
  auto)
    if [[ -z "${existing_cmake_version}" ]] || ! version_ge "${existing_cmake_version}" "${MIN_CMAKE_VERSION}"; then
      install_cmake=1
    fi
    ;;
  ask)
    if [[ "${NONINTERACTIVE}" == "1" ]]; then
      if [[ -z "${existing_cmake_version}" ]] || ! version_ge "${existing_cmake_version}" "${MIN_CMAKE_VERSION}"; then
        install_cmake=1
      fi
    else
      prompt="Install CMake ${CMAKE_VERSION} under ${HOME}/.local?"
      if [[ -n "${existing_cmake_version}" ]]; then
        prompt+=" Existing version: ${existing_cmake_version}."
      fi
      read -r -p "${prompt} [y/N] " answer
      [[ "${answer}" =~ ^[Yy]$ ]] && install_cmake=1
    fi
    ;;
  *) die "LIVO_INSTALL_CMAKE must be ask, auto, yes, or no" ;;
esac

if [[ "${install_cmake}" == "1" ]]; then
  case "$(uname -m)" in
    x86_64) cmake_arch="x86_64" ;;
    aarch64|arm64) cmake_arch="aarch64" ;;
    *) die "No official CMake installer configured for architecture $(uname -m)" ;;
  esac
  mkdir -p "${TOOLS_DIR}/downloads" "${HOME}/.local"
  cmake_file="cmake-${CMAKE_VERSION}-linux-${cmake_arch}.sh"
  cmake_url="https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${cmake_file}"
  sums_url="https://cmake.org/files/v3.31/cmake-${CMAKE_VERSION}-SHA-256.txt"
  curl -fL --retry 3 -o "${TOOLS_DIR}/downloads/${cmake_file}" "${cmake_url}"
  curl -fL --retry 3 -o "${TOOLS_DIR}/downloads/cmake-SHA-256.txt" "${sums_url}"
  (
    cd "${TOOLS_DIR}/downloads"
    expected="$(awk -v file="${cmake_file}" '$2 == file {print $1}' cmake-SHA-256.txt)"
    [[ -n "${expected}" ]] || die "CMake checksum is missing for ${cmake_file}"
    printf '%s  %s\n' "${expected}" "${cmake_file}" | sha256sum -c -
  )
  sh "${TOOLS_DIR}/downloads/${cmake_file}" --skip-license --exclude-subdir --prefix="${HOME}/.local"
  export PATH="${HOME}/.local/bin:${PATH}"
  log "Installed $(cmake --version | sed -n '1p') at $(command -v cmake)"
fi

if ! command -v cmake >/dev/null 2>&1; then
  die "CMake is required. Re-run with LIVO_INSTALL_CMAKE=yes"
fi
if ! version_ge "$(cmake --version | awk 'NR==1 {print $3}')" "${MIN_CMAKE_VERSION}"; then
  die "CMake ${MIN_CMAKE_VERSION}+ is required"
fi

python_cmd="$(command -v python3)"
python_version="$("${python_cmd}" -c 'import sys; print(".".join(map(str, sys.version_info[:3])))')"
if ! version_ge "${python_version}" "3.8.0"; then
  die "Python 3.8+ is required to build GStreamer 1.24.13. Install Python 3.8+ and rerun."
fi

venv="${TOOLS_DIR}/venv"
"${python_cmd}" -m venv "${venv}"
"${venv}/bin/pip" install --upgrade pip
"${venv}/bin/pip" install "meson==1.4.2" "ninja==1.11.1.1" "tomli==2.0.1"

if [[ ":${PATH}:" != *":${HOME}/.local/bin:"* ]]; then
  log "Add this to your shell profile before building LiVo:"
  printf 'export PATH="%s/.local/bin:$PATH"\n' "${HOME}"
fi

log "System prerequisites and private build tools are ready"
