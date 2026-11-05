"""Check local HTML targets, downloads and publication privacy."""
import argparse
from html.parser import HTMLParser
from pathlib import Path
import re
from urllib.parse import unquote, urlsplit
import zipfile


class Links(HTMLParser):
    def __init__(self):
        super().__init__()
        self.targets = []

    def handle_starttag(self, tag, attrs):
        for key, value in attrs:
            if key in {"href", "src", "poster"} and value:
                self.targets.append(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("site", type=Path)
    root = parser.parse_args().site.resolve()
    failures = []
    pages = list(root.rglob("*.html"))
    checked = 0
    for path in pages:
        content = path.read_text()
        private_patterns = ("/" + "Users/", "/" + "private/tmp/", r"/home/[^/]+/", "cod" + "ex")
        if re.search("|".join(private_patterns), content, re.I):
            failures.append(f"Private path/tool name in {path.relative_to(root)}")
        links = Links()
        links.feed(content)
        for value in links.targets:
            url = urlsplit(value)
            if url.scheme or url.netloc or not url.path or url.path.startswith("/"):
                continue
            target = (path.parent / unquote(url.path)).resolve()
            checked += 1
            if not target.is_relative_to(root) or not target.exists():
                failures.append(f"Broken local link: {path.relative_to(root)} -> {value}")
    packages = list((root / "downloads").glob("*.zip"))
    for path in packages:
        with zipfile.ZipFile(path) as package:
            if package.testzip():
                failures.append(f"Corrupt ZIP: {path.name}")
            for item in package.namelist():
                if item.startswith("/") or ".." in Path(item).parts:
                    failures.append(f"Unsafe ZIP path: {path.name}")
    if failures:
        raise SystemExit("\n".join(failures))
    print(f"Checked {len(pages)} HTML pages, {checked} local links, {len(packages)} ZIP packages; no private paths in HTML")


if __name__ == "__main__":
    main()
