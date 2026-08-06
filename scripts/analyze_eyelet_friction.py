#!/usr/bin/env python3
"""Validate rigid-flex aperture geometry and analytic eyelet friction."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
FLEX_MODEL = ROOT / "cable_plugin_demos/31_rigid_flex_through_hole.xml"
FRICTION_MODEL = ROOT / "cable_plugin_demos/32_cpp_plugin_eyelet_friction.xml"
DEFAULT_PLUGIN = ROOT / "build/plugin/libcable_unilateral.dylib"
DEFAULT_OUTPUT = ROOT / "docs/results/demo31_32_eyelet"


def _name2id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    identifier = mujoco.mj_name2id(model, kind, name)
    if identifier < 0:
        raise ValueError(f"model has no {kind.name} named {name!r}")
    return identifier


def _sensor_scalar(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> float:
    sensor = _name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    address = int(model.sensor_adr[sensor])
    return float(data.sensordata[address])


def _plugin_tension(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> float:
    sensor = _name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    return float(data.sensordata[int(model.sensor_adr[sensor]) + 5])


def _guide_turn_angle(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    anchor_name: str,
    guide_name: str,
    load_name: str,
) -> float:
    ids = [
        _name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
        for name in (anchor_name, guide_name, load_name)
    ]
    anchor, guide, load = [
        np.array(data.site_xpos[identifier], dtype=float) for identifier in ids
    ]
    incoming = guide - anchor
    outgoing = load - guide
    incoming /= np.linalg.norm(incoming)
    outgoing /= np.linalg.norm(outgoing)
    return math.acos(float(np.clip(np.dot(incoming, outgoing), -1.0, 1.0)))


def simulate_flex_aperture(duration: float = 1.5) -> dict[str, float | bool]:
    model = mujoco.MjModel.from_xml_path(str(FLEX_MODEL))
    data = mujoco.MjData(model)
    while data.time < duration:
        mujoco.mj_step(model, data)
    convex_displacement = _sensor_scalar(model, data, "convex_hull_ball_z")
    flex_displacement = _sensor_scalar(model, data, "rigid_flex_ball_z")
    summary: dict[str, float | bool] = {
        "duration_s": float(data.time),
        "convex_hull_ball_displacement_m": convex_displacement,
        "rigid_flex_ball_displacement_m": flex_displacement,
        "convex_hull_blocks_aperture": convex_displacement > -0.16,
        "rigid_flex_preserves_aperture": flex_displacement < -0.25,
    }
    summary["pass"] = bool(
        summary["convex_hull_blocks_aperture"]
        and summary["rigid_flex_preserves_aperture"]
    )
    return summary


def simulate_eyelet_friction(
    plugin: Path,
    duration: float = 2.0,
    sample_stride: int = 10,
) -> tuple[list[dict[str, float]], dict[str, float | bool]]:
    mujoco.mj_loadPluginLibrary(str(plugin.resolve()))
    model = mujoco.MjModel.from_xml_path(str(FRICTION_MODEL))
    data = mujoco.MjData(model)
    records: list[dict[str, float]] = []
    step = 0
    while data.time < duration:
        mujoco.mj_step(model, data)
        if step % sample_stride == 0:
            smooth_upstream = _plugin_tension(
                model, data, "smooth_eyelet_state"
            )
            rough_upstream = _plugin_tension(
                model, data, "rough_eyelet_state"
            )
            rough_turn = _guide_turn_angle(
                model,
                data,
                "rough_anchor",
                "rough_guide",
                "rough_load_end",
            )
            records.append(
                {
                    "time_s": float(data.time),
                    "smooth_displacement_m": _sensor_scalar(
                        model, data, "smooth_payload_displacement"
                    ),
                    "rough_displacement_m": _sensor_scalar(
                        model, data, "rough_payload_displacement"
                    ),
                    "smooth_upstream_tension_N": smooth_upstream,
                    "smooth_downstream_tension_N": smooth_upstream,
                    "rough_upstream_tension_N": rough_upstream,
                    "rough_downstream_tension_N": rough_upstream
                    * math.exp(-0.45 * rough_turn),
                    "rough_turn_angle_rad": rough_turn,
                }
            )
        step += 1

    mujoco.mj_forward(model, data)
    smooth_upstream = _plugin_tension(model, data, "smooth_eyelet_state")
    rough_upstream = _plugin_tension(model, data, "rough_eyelet_state")
    rough_turn = _guide_turn_angle(
        model, data, "rough_anchor", "rough_guide", "rough_load_end"
    )
    expected_ratio = math.exp(0.45 * rough_turn)
    rough_downstream = rough_upstream / expected_ratio
    smooth_displacement = _sensor_scalar(
        model, data, "smooth_payload_displacement"
    )
    rough_displacement = _sensor_scalar(
        model, data, "rough_payload_displacement"
    )
    payload_mass = float(
        model.body_mass[
            _name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rough_payload")
        ]
    )
    weight = payload_mass * abs(float(model.opt.gravity[2]))
    summary: dict[str, float | bool] = {
        "duration_s": float(data.time),
        "guide_friction_mu": 0.45,
        "turn_angle_rad": rough_turn,
        "turn_angle_deg": math.degrees(rough_turn),
        "expected_tension_ratio": expected_ratio,
        "smooth_upstream_tension_N": smooth_upstream,
        "smooth_downstream_tension_N": smooth_upstream,
        "rough_upstream_tension_N": rough_upstream,
        "rough_downstream_tension_N": rough_downstream,
        "measured_rough_tension_ratio": rough_upstream / rough_downstream,
        "payload_weight_N": weight,
        "smooth_displacement_m": smooth_displacement,
        "rough_displacement_m": rough_displacement,
        "smooth_force_balance_error_N": abs(smooth_upstream - weight),
        "rough_force_balance_error_N": abs(rough_downstream - weight),
    }
    summary["pass"] = bool(
        smooth_displacement > 0.01
        and rough_displacement < -0.004
        and summary["smooth_force_balance_error_N"] < 0.01
        and summary["rough_force_balance_error_N"] < 0.01
        and abs(summary["measured_rough_tension_ratio"] - expected_ratio)
        < 1e-12
    )
    return records, summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    if not args.plugin.exists():
        raise FileNotFoundError(f"compiled plugin not found: {args.plugin}")

    flex_summary = simulate_flex_aperture()
    records, friction_summary = simulate_eyelet_friction(args.plugin)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    with (output / "eyelet_friction_timeseries.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    combined = {
        "rigid_flex_aperture": flex_summary,
        "analytic_eyelet_friction": friction_summary,
        "pass": bool(flex_summary["pass"] and friction_summary["pass"]),
    }
    with (output / "eyelet_validation_summary.json").open(
        "w", encoding="utf-8"
    ) as stream:
        json.dump(combined, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(combined, indent=2, sort_keys=True))
    if args.strict and not combined["pass"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
