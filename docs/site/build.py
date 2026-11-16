"""Build a self-contained static manual from a MuJoCable checkout."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

from localize import prepare_english

SITE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=SITE.parents[1])
    parser.add_argument("--output", type=Path, default=SITE / "_build/html")
    parser.add_argument("--skip-assets", action="store_true", help="Reuse generated shared resources")
    args = parser.parse_args()
    output = args.output.resolve()
    if not args.skip_assets:
        subprocess.run([sys.executable, str(SITE / "build_assets.py"), "--repo", str(args.repo.resolve())], check=True)
    zh_env = dict(os.environ, MUJOCABLE_DOCS_LANGUAGE="zh_CN")
    subprocess.run([sys.executable, "-m", "sphinx", "-b", "html", "-d", str(SITE / "_build/doctrees"), "-W", "--keep-going", str(SITE), str(output)], check=True, env=zh_env)
    shutil.copytree(SITE / "downloads", output / "downloads", dirs_exist_ok=True)
    english_source = SITE / "_build/en-source"
    prepare_english(english_source, args.repo.resolve())
    en_env = dict(os.environ, MUJOCABLE_DOCS_LANGUAGE="en")
    subprocess.run([sys.executable, "-m", "sphinx", "-b", "html", "-d", str(SITE / "_build/doctrees-en"), "-W", "--keep-going", str(english_source), str(output / "en")], check=True, env=en_env)
    (output / ".nojekyll").touch()
    subprocess.run([sys.executable, str(SITE / "check_site.py"), str(output)], check=True)
    print(f"Static manual ready: {output}")


if __name__ == "__main__":
    main()
