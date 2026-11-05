#!/usr/bin/env python3
"""Headless route-state checks using the showcase recording controls."""

import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path
import sys

import mujoco
import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", required=True, type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    path = HERE / "scripts/record_showcases.py"
    spec = importlib.util.spec_from_file_location("showcase_recorder", path)
    recorder = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = recorder
    spec.loader.exec_module(recorder)
    mujoco.mj_loadPluginLibrary(str(args.plugin.resolve()))
    rows = []
    for case in recorder.SPECS:
        warnings = Counter()
        mujoco.set_mju_user_warning(lambda message: warnings.update([message]))
        model = mujoco.MjModel.from_xml_path(str(HERE / "models" / case.model))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        actuator = model.actuator(case.actuator).id
        sensors = [int(model.sensor_adr[i]) for i in range(model.nsensor)
                   if model.sensor_type[i] == mujoco.mjtSensor.mjSENS_PLUGIN and model.sensor_dim[i] == 12]
        max_status = 0
        max_tension = 0.
        while data.time < case.duration - .5 * model.opt.timestep:
            data.ctrl[actuator] = recorder.smooth_pulse(data.time, case.duration, case.command)
            mujoco.mj_step(model, data)
            for address in sensors:
                max_status = max(max_status, int(round(data.sensordata[address + 8])))
                max_tension = max(max_tension, float(data.sensordata[address + 5]))
        rows.append({"case": case.name, "duration_s": case.duration,
                     "maximum_route_status": max_status, "maximum_tension_N": max_tension,
                     "finite": bool(np.all(np.isfinite(data.qpos))), "warnings": sum(warnings.values())})
    mujoco.set_mju_user_warning(None)
    result = {"scope": "headless route/finite-state check; not new rendered videos or formal accuracy validation",
              "cases": rows, "passed": all(row["maximum_route_status"] <= 1 and row["finite"] for row in rows)}
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
