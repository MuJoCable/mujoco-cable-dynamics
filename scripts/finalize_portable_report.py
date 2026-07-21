#!/usr/bin/env python3
"""Apply the repository's narrow-screen fallback to a portable HTML report."""

from __future__ import annotations

import argparse
from pathlib import Path


STYLE_MARKER = "data-cable-report-mobile-fallback"
MOBILE_STYLE = f"""\
<style {STYLE_MARKER}="true">
@media screen and (max-width:600px) {{
  #data-analytics-portable-reader {{ display:none !important; }}
  .portable-fallback.portable-enhanced-hidden {{ display:block !important; }}
  .portable-page-header h1,
  .portable-markdown {{ overflow-wrap:anywhere; word-break:break-word; }}
}}
</style>
"""


def finalize(path: Path) -> bool:
    html = path.read_text(encoding="utf-8")
    updated = html.replace('<html lang="en"', '<html lang="zh-CN"', 1)
    if STYLE_MARKER not in updated:
        if "</head>" not in updated:
            raise ValueError(f"HTML report has no closing head element: {path}")
        updated = updated.replace("</head>", MOBILE_STYLE + "</head>", 1)
    if updated == html:
        return False
    path.write_text(updated, encoding="utf-8")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    changed = finalize(args.report)
    print(f"{'updated' if changed else 'already finalized'}: {args.report.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
