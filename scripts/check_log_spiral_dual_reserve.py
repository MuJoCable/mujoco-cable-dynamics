#!/usr/bin/env python3
"""Validate self-collision and coordinated payout in the dual-reserve model."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def _sensor(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    sensor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    address = model.sensor_adr[sensor]
    return np.asarray(
        data.sensordata[address : address + model.sensor_dim[sensor]], dtype=float
    ).copy()


def _minimum_contact_distance(data: mujoco.MjData) -> float:
    if data.ncon == 0:
        return math.inf
    return min(float(data.contact[index].dist) for index in range(data.ncon))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", type=Path, required=True)
    parser.add_argument(
        "--model",
        type=Path,
        default=ROOT / "cable_plugin_demos/33_cpp_plugin_log_spiral_dual_reserve.xml",
    )
    parser.add_argument("--reserve", type=float, default=0.025)
    parser.add_argument("--contraction", type=float, default=0.050)
    parser.add_argument("--ramp-time", type=float, default=4.0)
    parser.add_argument("--hold-time", type=float, default=4.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "docs/results/demo33_log_spiral/dual_reserve_collision.json",
    )
    args = parser.parse_args()

    mujoco.mj_loadPluginLibrary(str(args.plugin.resolve()))
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    initial_contacts = int(data.ncon)
    initial_minimum_distance = _minimum_contact_distance(data)
    maximum_contacts = initial_contacts
    minimum_distance = initial_minimum_distance
    duration = 2.0 * args.ramp_time + args.hold_time
    steps = math.ceil(duration / model.opt.timestep)

    for _ in range(steps):
        if data.time < args.ramp_time:
            takeup = data.time / args.ramp_time
            data.ctrl[:] = args.reserve * takeup
        elif data.time < 2.0 * args.ramp_time:
            bend = (data.time - args.ramp_time) / args.ramp_time
            data.ctrl[0] = args.reserve + (args.contraction - args.reserve) * bend
            data.ctrl[1] = args.reserve * (1.0 - bend)
        else:
            data.ctrl[0] = args.contraction
            data.ctrl[1] = 0.0
        mujoco.mj_step(model, data)
        maximum_contacts = max(maximum_contacts, int(data.ncon))
        minimum_distance = min(minimum_distance, _minimum_contact_distance(data))

    positive = _sensor(model, data, "positive_state")
    negative = _sensor(model, data, "negative_state")
    report = {
        "pass": bool(
            np.isfinite(data.qpos).all()
            and maximum_contacts > 0
            and float(negative[5]) < 1e-6
        ),
        "model": args.model.name,
        "initial_contacts": initial_contacts,
        "initial_minimum_contact_distance_m": initial_minimum_distance,
        "maximum_contacts": maximum_contacts,
        "minimum_contact_distance_m": minimum_distance,
        "final_positive_control_m": float(data.ctrl[0]),
        "final_negative_control_m": float(data.ctrl[1]),
        "final_positive_tension_N": float(positive[5]),
        "final_negative_tension_N": float(negative[5]),
        "final_net_bend_deg": abs(math.degrees(float(np.sum(data.qpos)))),
        "maximum_joint_angle_deg": math.degrees(float(np.max(np.abs(data.qpos)))),
        "finite_state": bool(np.isfinite(data.qpos).all()),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
