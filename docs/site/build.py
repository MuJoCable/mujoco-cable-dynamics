"""Build a self-contained static manual from a MuJoCable checkout."""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys

SITE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=SITE.parents[1])
    parser.add_argument("--output", type=Path, default=SITE / "_build/html")
    args = parser.parse_args()
    output = args.output.resolve()
    subprocess.run([sys.executable, str(SITE / "build_assets.py"), "--repo", str(args.repo.resolve())], check=True)
    subprocess.run([sys.executable, "-m", "sphinx", "-b", "html", "-d", str(SITE / "_build/doctrees"), "-W", "--keep-going", str(SITE), str(output)], check=True)
    shutil.copytree(SITE / "downloads", output / "downloads", dirs_exist_ok=True)
    (output / ".nojekyll").touch()
    subprocess.run([sys.executable, str(SITE / "check_site.py"), str(output)], check=True)
    print(f"Static manual ready: {output}")


if __name__ == "__main__":
    main()
