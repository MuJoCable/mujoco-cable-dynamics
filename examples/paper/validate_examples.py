#!/usr/bin/env python3
"""Validate model loading, short stepping, media, paths and archive checksums."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess

import mujoco
import numpy as np


HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", required=True, type=Path)
    parser.add_argument("--steps", type=int, default=10)
    parser.add_argument("--check-videos", action="store_true")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if args.steps < 0:
        parser.error("--steps must be nonnegative")
    mujoco.mj_loadPluginLibrary(str(args.plugin.resolve()))
    report = {"scope": "load/short-step check, not a full motion or hardware validation", "models": [], "videos": [], "errors": []}
    for path in sorted(HERE.rglob("*.xml")):
        warnings = Counter()
        mujoco.set_mju_user_warning(lambda message: warnings.update([message]))
        item = {"model": path.relative_to(HERE).as_posix()}
        try:
            model = mujoco.MjModel.from_xml_path(str(path))
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            for _ in range(args.steps):
                mujoco.mj_step(model, data)
            if not (np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.sensordata))):
                raise ValueError("nonfinite state")
            statuses = [float(data.sensordata[model.sensor_adr[i] + 8]) for i in range(model.nsensor)
                        if model.sensor_type[i] == mujoco.mjtSensor.mjSENS_PLUGIN and model.sensor_dim[i] == 12]
            item.update(nq=int(model.nq), nv=int(model.nv), nu=int(model.nu), steps=args.steps,
                        max_route_status=max(statuses, default=0), warning_count=sum(warnings.values()))
            if item["max_route_status"] > 1:
                raise ValueError("invalid surface route")
        except Exception as exc:
            item["error"] = str(exc).replace(str(HERE), "examples/paper")
            report["errors"].append(item)
        report["models"].append(item)
    mujoco.set_mju_user_warning(None)
    if args.check_videos:
        for path in sorted(HERE.rglob("*.mp4")):
            process = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration:stream=width,height", "-of", "json", str(path)], capture_output=True, text=True)
            if process.returncode:
                report["errors"].append({"video": path.relative_to(HERE).as_posix(), "error": process.stderr})
            else:
                report["videos"].append({"video": path.relative_to(HERE).as_posix(), **json.loads(process.stdout)})
    manifest = HERE / "ASSET_MANIFEST.json"
    if manifest.is_file():
        for relative, expected in json.loads(manifest.read_text()).items():
            path = HERE / relative
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                report["errors"].append({"file": relative, "error": "checksum mismatch"})
    report["passed"] = not report["errors"]
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"passed": report["passed"], "models": len(report["models"]), "videos": len(report["videos"]), "errors": report["errors"]}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
