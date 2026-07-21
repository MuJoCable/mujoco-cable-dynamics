#!/usr/bin/env python3
"""Validate mixed stiffness, elastic strain, and slack behavior in Demo 19."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "cable_plugin_demos" / "19_cpp_plugin_mixed_stiffness_tensegrity.xml"
DEFAULT_PLUGIN = ROOT / "build/cable_surface/plugin/libcable_unilateral.dylib"
CABLE_NAMES = (
    "bottom_01", "bottom_12", "bottom_20",
    "top_01", "top_12", "top_20",
    "cross_00", "cross_11", "cross_22",
)


def _sensor_values(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    rows = []
    for name in CABLE_NAMES:
        sensor_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SENSOR, f"state_{name}"
        )
        address = int(model.sensor_adr[sensor_id])
        rows.append(np.asarray(data.sensordata[address : address + 8], dtype=float))
    return np.asarray(rows)


def analyze(
    plugin_path: Path = DEFAULT_PLUGIN,
    model_path: Path = DEFAULT_MODEL,
    *,
    settle_steps: int = 12000,
    ramp_steps: int = 4000,
    hold_steps: int = 4000,
    release_steps: int = 8000,
    contraction_m: float = 0.06,
) -> dict[str, object]:
    mujoco.mj_loadPluginLibrary(str(plugin_path.resolve()))
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    controlled = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_cross_00"
    )

    negative_data = mujoco.MjData(model)
    negative_data.ctrl[controlled] = -0.01
    mujoco.mj_forward(model, negative_data)
    negative_state = _sensor_values(model, negative_data)[6]

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    initial = _sensor_values(model, data)
    for _ in range(settle_steps):
        data.ctrl[:] = 0
        mujoco.mj_step(model, data)

    maximum_strain = np.zeros(9)
    maximum_tension = np.zeros(9)
    minimum_tension = np.full(9, np.inf)
    slack_steps = np.zeros(9, dtype=int)
    maximum_shape_change = 0.0
    site_ids = np.asarray([
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
        for name in ("node_b0", "node_b1", "node_b2", "node_t0", "node_t1", "node_t2")
    ])
    settled_nodes = np.asarray(data.site_xpos)[site_ids].copy()
    settled_distances = np.linalg.norm(
        settled_nodes[:, None, :] - settled_nodes[None, :, :], axis=2
    )

    total_steps = ramp_steps + hold_steps + release_steps
    for step in range(total_steps):
        data.ctrl[:] = 0
        if step < ramp_steps:
            data.ctrl[controlled] = contraction_m * step / ramp_steps
        elif step < ramp_steps + hold_steps:
            data.ctrl[controlled] = contraction_m
        elif step < ramp_steps + hold_steps + ramp_steps:
            fraction = (step - ramp_steps - hold_steps) / ramp_steps
            data.ctrl[controlled] = contraction_m * (1.0 - fraction)
        mujoco.mj_step(model, data)
        values = _sensor_values(model, data)
        strain = np.maximum(values[:, 4], 0.0) / np.maximum(values[:, 2], 1e-12)
        maximum_strain = np.maximum(maximum_strain, strain)
        maximum_tension = np.maximum(maximum_tension, values[:, 5])
        minimum_tension = np.minimum(minimum_tension, values[:, 5])
        slack_steps += values[:, 6] < 0.5
        nodes = np.asarray(data.site_xpos)[site_ids]
        distances = np.linalg.norm(nodes[:, None, :] - nodes[None, :, :], axis=2)
        maximum_shape_change = max(
            maximum_shape_change,
            float(np.max(np.abs(distances - settled_distances))),
        )

    final = _sensor_values(model, data)
    checks = {
        "initial_self_stress": bool(
            np.allclose(initial[:6, 5], 1.45215, atol=1e-5)
            and np.allclose(initial[6:, 5], 3.0, atol=1e-5)
        ),
        "negative_command_slackens_controlled_cross": bool(
            negative_state[4] < 0
            and negative_state[5] == 0
            and negative_state[6] == 0
        ),
        "stiff_cross_has_low_strain": bool(np.max(maximum_strain[6:]) < 0.01),
        "ring_is_more_compliant": bool(
            np.mean(maximum_strain[:6]) > 2.0 * np.max(maximum_strain[6:])
        ),
        "visible_shape_response": maximum_shape_change > 0.03,
        "no_saturation": bool(np.all(maximum_tension[:6] < 30.0) and np.all(maximum_tension[6:] < 120.0)),
        "finite": bool(np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))),
    }
    return {
        "pass": all(checks.values()),
        "checks": checks,
        "controlled_cable": "cross_00",
        "commanded_contraction_m": contraction_m,
        "initial_tension_n": dict(zip(CABLE_NAMES, initial[:, 5].tolist(), strict=True)),
        "maximum_elastic_strain": dict(zip(CABLE_NAMES, maximum_strain.tolist(), strict=True)),
        "minimum_tension_n": dict(zip(CABLE_NAMES, minimum_tension.tolist(), strict=True)),
        "maximum_tension_n": dict(zip(CABLE_NAMES, maximum_tension.tolist(), strict=True)),
        "slack_steps_during_positive_cycle": dict(zip(CABLE_NAMES, slack_steps.tolist(), strict=True)),
        "negative_command_state": {
            "command_m": -0.01,
            "length_m": float(negative_state[0]),
            "free_length_m": float(negative_state[2]),
            "extension_m": float(negative_state[4]),
            "tension_n": float(negative_state[5]),
            "taut": bool(negative_state[6]),
        },
        "maximum_pairwise_shape_change_m": maximum_shape_change,
        "final_tension_n": dict(zip(CABLE_NAMES, final[:, 5].tolist(), strict=True)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    report = analyze(args.plugin, args.model)
    payload = json.dumps(report, indent=2)
    print(payload)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(payload + "\n", encoding="utf-8")
    return 1 if args.strict and not report["pass"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
