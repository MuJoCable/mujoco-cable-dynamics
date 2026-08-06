#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
INPUT="${1:-$ROOT/docs/plugin_method_strategy_report.md}"
OUTPUT="${2:-$ROOT/docs/plugin_method_strategy_report.pdf}"
TITLE="${3:-}"

if [[ -z "$TITLE" ]]; then
  if [[ "$(basename "$INPUT")" == *_en.md ]]; then
    TITLE="MuJoCable Method and Implementation Report"
  else
    TITLE="MuJoCo 绳索插件方法与实现策略报告"
  fi
fi

if ! command -v pandoc >/dev/null 2>&1; then
  echo "pandoc is required to build the method report PDF" >&2
  exit 1
fi
if ! command -v xelatex >/dev/null 2>&1; then
  echo "xelatex is required to build the method report PDF" >&2
  exit 1
fi

PANDOC_INPUT="$(mktemp "${TMPDIR:-/tmp}/cable-method-report.XXXXXX.md")"
trap 'rm -f "$PANDOC_INPUT"' EXIT
# The PDF metadata already renders the title; omit the Markdown H1 to avoid a
# duplicate title below the table of contents.
sed '1{/^# /d;}' "$INPUT" > "$PANDOC_INPUT"

PANDOC_ARGS=(
  "$PANDOC_INPUT"
  --from=markdown+tex_math_dollars+tex_math_single_backslash \
  --pdf-engine=xelatex \
  --resource-path="$(dirname "$INPUT"):$ROOT" \
  --toc \
  --metadata "title=$TITLE" \
  --metadata author="MuJoCable" \
  -V mainfont="Times New Roman" \
  -V monofont="Menlo" \
  -V fontsize=10pt \
  -V geometry:margin=18mm \
  -V colorlinks=true \
  -V linkcolor=blue \
  -V urlcolor=blue \
  -V linestretch=1.15
)
if [[ "$(basename "$INPUT")" != *_en.md ]]; then
  PANDOC_ARGS+=( -V CJKmainfont="Songti SC" )
fi
pandoc "${PANDOC_ARGS[@]}" -o "$OUTPUT"

echo "Wrote $OUTPUT"
