#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
LOCK_FILE="${ROOT_DIR}/install/versions.lock"

log() { printf '[LiVo] %s\n' "$*"; }
die() { printf '[LiVo] ERROR: %s\n' "$*" >&2; exit 1; }

[[ -f "${LOCK_FILE}" ]] || die "Missing ${LOCK_FILE}"
git -C "${ROOT_DIR}" submodule sync --recursive
git -C "${ROOT_DIR}" submodule update --init --recursive

while IFS='|' read -r name path ref expected_sha url; do
  [[ -z "${name}" || "${name}" == \#* || "${path}" == "-" ]] && continue
  source_dir="${ROOT_DIR}/${path}"
  [[ -d "${source_dir}" ]] || die "Submodule directory is missing: ${path}"
  actual_sha="$(git -C "${source_dir}" rev-parse HEAD)"
  if [[ "${actual_sha}" != "${expected_sha}" ]]; then
    die "${name} is at ${actual_sha}, expected ${expected_sha} (${ref}). Run git submodule update --init --recursive"
  fi
  actual_url="$(git -C "${source_dir}" remote get-url origin)"
  log "${name}: ${ref} (${actual_sha:0:12}) from ${actual_url}"
done < "${LOCK_FILE}"

log "All dependency submodules match install/versions.lock"
