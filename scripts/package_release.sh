#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLUGIN="${1:-}"
OUTPUT="${2:-$ROOT/dist}"

if [[ -z "$PLUGIN" || ! -f "$PLUGIN" ]]; then
  echo "Usage: $0 <plugin-library> [output-directory]" >&2
  exit 2
fi

VERSION="$(tr -d '[:space:]' < "$ROOT/VERSION")"
SYSTEM="$(uname -s | tr '[:upper:]' '[:lower:]')"
ARCH="$(uname -m)"
NAME="mujoco-cable-dynamics-v${VERSION}-${SYSTEM}-${ARCH}"
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

mkdir -p "$STAGE/$NAME/lib" "$STAGE/$NAME/scripts"
cp "$PLUGIN" "$STAGE/$NAME/lib/"
cp "$ROOT"/{README.md,README_zh.md,LICENSE,THIRD_PARTY_NOTICES.md,VERSION} "$STAGE/$NAME/"
cp "$ROOT/scripts"/{run_demo.sh,view_cpp_plugin_demo.py,smoke_cpp_plugin.py} "$STAGE/$NAME/scripts/"
cp -R "$ROOT/cable_plugin_demos" "$STAGE/$NAME/"

mkdir -p "$OUTPUT"
LC_ALL=C tar -czf "$OUTPUT/$NAME.tar.gz" -C "$STAGE" "$NAME"
if command -v shasum >/dev/null 2>&1; then
  (cd "$OUTPUT" && LC_ALL=C shasum -a 256 "$NAME.tar.gz" > "$NAME.tar.gz.sha256")
else
  (cd "$OUTPUT" && LC_ALL=C sha256sum "$NAME.tar.gz" > "$NAME.tar.gz.sha256")
fi
echo "$OUTPUT/$NAME.tar.gz"
