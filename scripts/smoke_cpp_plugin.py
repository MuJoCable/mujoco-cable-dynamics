#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def resolve_model_path(model: str) -> Path:
    model_path = Path(model)
    if not model_path.is_absolute():
        model_path = ROOT / model_path
    return model_path.resolve()


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke-test the C++ unilateral cable plugin")
    parser.add_argument("--plugin", required=True, help="Path to libcable_unilateral shared library")
    parser.add_argument(
        "--model",
        default="cable_plugin_demos/09_cpp_plugin_free_hanging_single_pulley.xml",
        help="Path to the plugin-backed MJCF model",
    )
    parser.add_argument("--ctrl", type=float, default=0.8, help="Target control value for the plugin actuator")
    parser.add_argument("--steps", type=int, default=100, help="Number of simulation steps")
    args = parser.parse_args()

    plugin_path = Path(args.plugin).resolve()
    model_path = resolve_model_path(args.model)
    mujoco.mj_loadPluginLibrary(str(plugin_path))

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    initial_qpos = np.array(data.qpos, copy=True)
    initial_sensor = np.array(data.sensordata, copy=True)
    tensions = [float(data.sensordata[5])] if model.nsensordata >= 6 else []

    data.ctrl[0] = args.ctrl
    for _ in range(args.steps):
        mujoco.mj_step(model, data)
        if model.nsensordata >= 6:
            tensions.append(float(data.sensordata[5]))

    summary = {
        "plugin": str(plugin_path),
        "model": str(model_path),
        "nplugin": int(model.nplugin),
        "nu": int(model.nu),
        "nsensor": int(model.nsensor),
        "sensor_dim": [int(x) for x in model.sensor_dim],
        "initial_qpos": initial_qpos.tolist(),
        "final_qpos": np.array(data.qpos, copy=True).tolist(),
        "initial_sensor": initial_sensor.tolist(),
        "final_sensor": np.array(data.sensordata, copy=True).tolist(),
        "max_tension": max(tensions) if tensions else None,
    }
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
