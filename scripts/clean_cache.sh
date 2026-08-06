#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APPLY=0
INCLUDE_BUILD=0

usage() {
  cat <<'EOF'
Usage: scripts/clean_cache.sh [--apply] [--include-build]

By default, this command only previews safe cache targets.

Options:
  --apply          Delete the listed targets.
  --include-build  Also include local CMake build trees and root CMake cache.
  -h, --help       Show this help.

Release archives, experiment results, figures, videos, meshes, and source files
are never selected by this script.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --apply) APPLY=1 ;;
    --include-build) INCLUDE_BUILD=1 ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

declare -a TARGETS=()

add_target() {
  local target="$1"
  [[ -e "$target" || -L "$target" ]] || return 0

  case "$target" in
    "$ROOT"/*) ;;
    *)
      echo "Refusing target outside repository: $target" >&2
      exit 2
      ;;
  esac

  local existing
  for existing in "${TARGETS[@]:-}"; do
    [[ "$existing" == "$target" ]] && return 0
  done
  TARGETS+=("$target")
}

while IFS= read -r -d '' path; do
  add_target "$path"
done < <(
  find "$ROOT" \
    -path "$ROOT/.git" -prune -o \
    -type d \( \
      -name __pycache__ -o \
      -name .pytest_cache -o \
      -name .mypy_cache -o \
      -name .ruff_cache \
    \) -prune -print0
)

while IFS= read -r -d '' path; do
  add_target "$path"
done < <(
  find "$ROOT" \
    -path "$ROOT/.git" -prune -o \
    -type f \( -name .DS_Store -o -name MUJOCO_LOG.TXT \) -print0
)

if [[ "$INCLUDE_BUILD" -eq 1 ]]; then
  add_target "$ROOT/build"
  while IFS= read -r -d '' path; do
    add_target "$path"
  done < <(find "$ROOT" -maxdepth 1 -type d -name 'cmake-build-*' -print0)

  for filename in CMakeCache.txt cmake_install.cmake compile_commands.json Makefile; do
    add_target "$ROOT/$filename"
  done
  add_target "$ROOT/CMakeFiles"
fi

if [[ "${#TARGETS[@]}" -eq 0 ]]; then
  echo "No cache targets found."
  exit 0
fi

total_kib=0
echo "Repository: $ROOT"
echo "Selected cache targets:"
for target in "${TARGETS[@]}"; do
  size_kib="$(du -sk "$target" 2>/dev/null | awk '{print $1}')"
  size_kib="${size_kib:-0}"
  total_kib=$((total_kib + size_kib))
  printf '  %8s KiB  %s\n' "$size_kib" "${target#$ROOT/}"
done
printf 'Total reclaimable: %.2f MiB\n' "$(awk -v kib="$total_kib" 'BEGIN {print kib / 1024}')"

if [[ "$APPLY" -ne 1 ]]; then
  echo "Dry run only. Re-run with --apply to delete these targets."
  exit 0
fi

for target in "${TARGETS[@]}"; do
  rm -rf -- "$target"
done

echo "Cache cleanup complete."
