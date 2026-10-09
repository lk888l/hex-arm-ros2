#!/usr/bin/env bash
# Build locally by default; releases require the checked-in immutable ABI review.
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_dir}"
release_build=0
release_args=()
case "${1:-}" in
  --release)
    release_build=1
    release_args=(--release --review-file docker/release-base.json)
    if [[ -n "$(git status --porcelain)" ]]; then
      echo "error: release builds require a clean, committed source tree" >&2
      exit 2
    fi
    default_build_image="$(python3 -c 'import json; print(json.load(open("docker/release-base.json"))["build_image"])')"
    ;;
  "") default_build_image=hex-arm-jazzy:local ;;
  *) echo "usage: $0 [--release]" >&2; exit 2 ;;
esac
build_image="${HEX_ARM_BUILD_IMAGE:-${default_build_image}}"
source_revision="$(git rev-parse HEAD)"
python3 scripts/runtime-manifest.py validate-base --build-image "${build_image}" "${release_args[@]}"
docker build --file docker/Dockerfile.runtime \
  --build-arg "BUILD_IMAGE=${build_image}" --build-arg "RELEASE_BUILD=${release_build}" \
  --build-arg "SOURCE_REVISION=${source_revision}" \
  --tag "${HEX_ARM_RUNTIME_TAG:-hex-arm-runtime:local}" .
docker image inspect "${HEX_ARM_RUNTIME_TAG:-hex-arm-runtime:local}" --format '{{.Id}} {{json .RepoDigests}}'
