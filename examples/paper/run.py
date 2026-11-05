#!/usr/bin/env python3
"""List, inspect, or reproduce the paper cases without overwriting the archive."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
MODELS = {
    "fixed": "result_1_1/models/result_1_1.xml",
    "fixed_friction": "result_1_1/models/result_1_1_mu_0p15.xml",
    "free_sheave": "result_1_1_b/models/result_1_1_b.xml",
    "winch": "result_1_1_c/models/result_1_1_c.xml",
    "moving_pulley": "result_1_2/models/result_1_2.xml",
    "atwood": "result_1_3/models/result_1_3.xml",
    "rolling_native": "result_2_1/models/result_2_1a_ideal_virtual_hinge.xml",
    "rolling_mesh": "result_2_1/models/result_2_1b_mujocable_surface.xml",
    "spirobs": "result_2_2/models/result_2_2a_mujocable_mu_0p15.xml",
    "spirobs_native": "result_2_2/models/result_2_2b_native_tendon.xml",
    "finger": "result_2_2/demo42_single_finger/models/result_2_2c_demo42_single_finger.xml",
    "spirobs_calibrated": "result_3_2/models/result_3_2.xml",
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def plugin_path(value):
    if value:
        result = Path(value).expanduser().resolve()
    elif os.environ.get("CABLE_PLUGIN_LIBRARY"):
        result = Path(os.environ["CABLE_PLUGIN_LIBRARY"]).expanduser().resolve()
    else:
        suffix = "dylib" if sys.platform == "darwin" else "so"
        result = REPO / f"build/plugin/libcable_unilateral.{suffix}"
    if not result.is_file():
        raise FileNotFoundError("Build the current plugin or pass --plugin / set CABLE_PLUGIN_LIBRARY")
    return result


def load_module(name):
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def copy_inputs(output):
    if output == HERE or HERE in output.parents:
        raise ValueError("Choose an output outside examples/paper to preserve the historical archive")
    output.mkdir(parents=True, exist_ok=True)
    for model in HERE.glob("result_*/**/*.xml"):
        target = output / model.relative_to(HERE)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(model, target)
    shutil.copytree(HERE / "assets", output / "assets", dirs_exist_ok=True)


def record_run(output, action, library):
    metadata = {
        "action": action, "plugin_sha256": digest(library),
        "archived_plugin_sha256": json.loads((HERE / "PROVENANCE.json").read_text())["historical_plugin_sha256"],
        "historical_data_overwritten": False,
        "source_sha256": {str(p.relative_to(REPO)): digest(p) for p in (REPO / "plugin").iterdir() if p.suffix in {".cc", ".h"}},
    }
    (output / f"{action}_reproduction.json").write_text(json.dumps(metadata, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest="action", required=True)
    for name in ("list", "view", "audit", "result1", "result2", "benchmark", "cycle"):
        action = actions.add_parser(name)
        action.add_argument("--plugin")
        action.add_argument("--output", type=Path, default=REPO / "outputs/paper_latest")
        if name == "view":
            action.add_argument("case", choices=tuple(MODELS))
    args, forwarded = parser.parse_known_args()
    if args.action == "list":
        for key, value in MODELS.items():
            print(f"{key:22s} {value}")
        return 0
    library = plugin_path(args.plugin)
    if args.action == "view":
        executable = shutil.which("mjpython") if sys.platform == "darwin" else sys.executable
        if executable is None:
            raise RuntimeError("On macOS install MuJoCo's mjpython launcher")
        command = [executable, str(REPO / "scripts/view_cpp_plugin_demo.py"),
                   "--plugin", str(library), "--model", str(HERE / MODELS[args.case]),
                   "--no-auto-control", "--duration", "120"]
        return subprocess.call(command + forwarded)
    if args.action == "audit":
        return subprocess.call([sys.executable, str(HERE / "validate_examples.py"),
                                "--plugin", str(library)] + forwarded)

    output = args.output.resolve()
    if args.action == "cycle":
        spec = importlib.util.spec_from_file_location("paper_bending_cycle", HERE / "scripts/run_tpu95hf_bidirectional_bending.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        module.DEFAULT_MODELS = (("demo49_tpu95hf", "Calibrated SpiRobs", HERE / MODELS["spirobs_calibrated"]),)
        module.DEFAULT_OUTPUT = output
        if output == HERE or HERE in output.parents:
            raise ValueError("Choose an output outside the historical archive")
        old_argv = sys.argv
        sys.argv = ["cycle", "--plugin", str(library), "--output", str(output)] + forwarded
        try:
            status = module.main()
        finally:
            sys.argv = old_argv
        record_run(output, args.action, library)
        return status
    if args.action == "benchmark":
        name = "benchmark_result2_spirobs_runtime"
    else:
        name = "run_result1_experiments" if args.action == "result1" else "run_result2_experiments"
    copy_inputs(output)
    module = load_module(name)
    module.HERE = output
    if hasattr(module, "REPO"):
        module.REPO = REPO
    command = [name, "--plugin", str(library)]
    if args.action in {"result2", "benchmark"}:
        sys.path.insert(0, str(HERE))
        command += ["--research-root", str(HERE)]
    if args.action == "result2":
        command += ["--release-root", str(REPO)]
    if args.action == "benchmark":
        # The historical benchmark module stores output paths at import time.
        module.RAW_OUTPUT = output / "result_2_2/data/result_2_2_runtime_samples.csv"
        module.SUMMARY_OUTPUT = output / "result_2_2/data/result_2_2_runtime_summary.csv"
        module.RAW_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        module.result2.HERE = output
        module.PLUGIN_MODEL = output / MODELS["spirobs"]
        module.NATIVE_MODEL = output / MODELS["spirobs_native"]
    old_argv = sys.argv
    sys.argv = command + forwarded
    try:
        status = module.main()
    finally:
        sys.argv = old_argv
    record_run(output, args.action, library)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
