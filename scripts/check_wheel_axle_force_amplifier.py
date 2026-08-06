#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = (
    ROOT
    / "cable_plugin_demos"
    / "21_cpp_plugin_wheel_axle_force_amplifier.xml"
)
DEFAULT_PLUGIN = ROOT / "build/plugin/libcable_unilateral.dylib"
SENSOR_FIELDS = (
    "length_m",
    "velocity_mps",
    "free_length_m",
    "contraction_m",
    "extension_m",
    "tension_N",
    "taut",
    "saturated",
)


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    value = mujoco.mj_name2id(model, kind, name)
    if value < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return value


def sensor_values(
    model: mujoco.MjModel, data: mujoco.MjData, name: str
) -> np.ndarray:
    sensor_id = object_id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    return np.asarray(
        data.sensordata[address : address + dimension], dtype=float
    ).copy()


def settle(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    steps: int,
    input_pull: float = 0.0,
    rows: list[dict[str, float]] | None = None,
) -> None:
    input_pull_id = object_id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "input_pull"
    )
    shaft_joint = object_id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "compound_drum_hinge"
    )
    input_joint = object_id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "input_slide"
    )
    output_joint = object_id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "output_slide"
    )
    shaft_qpos = int(model.jnt_qposadr[shaft_joint])
    input_qpos = int(model.jnt_qposadr[input_joint])
    output_qpos = int(model.jnt_qposadr[output_joint])
    for step in range(steps):
        data.ctrl[input_pull_id] = input_pull
        mujoco.mj_step(model, data)
        if rows is not None and (step % 10 == 0 or step + 1 == steps):
            input_state = sensor_values(
                model, data, "large_drum_input_state"
            )
            output_state = sensor_values(
                model, data, "small_drum_output_state"
            )
            rows.append(
                {
                    "time_s": float(data.time),
                    "input_pull_N": input_pull,
                    "shaft_angle_rad": float(data.qpos[shaft_qpos]),
                    "input_position_m": float(data.qpos[input_qpos]),
                    "output_position_m": float(data.qpos[output_qpos]),
                    "input_tension_N": float(input_state[5]),
                    "output_tension_N": float(output_state[5]),
                }
            )


def analyze(
    plugin: Path,
    model_path: Path,
    output: Path,
    settle_steps: int,
    drive_force: float,
    drive_steps: int,
) -> dict[str, Any]:
    mujoco.mj_loadPluginLibrary(str(plugin))
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    if model.nu != 3:
        raise RuntimeError(f"Demo 21 expected nu=3, got {model.nu}")

    input_radius = float(
        model.geom_size[
            object_id(
                model, mujoco.mjtObj.mjOBJ_GEOM, "large_input_drum"
            ),
            0,
        ]
    )
    output_radius = float(
        model.geom_size[
            object_id(
                model, mujoco.mjtObj.mjOBJ_GEOM, "small_output_drum"
            ),
            0,
        ]
    )
    theory_ratio = input_radius / output_radius
    rows: list[dict[str, float]] = []
    settle(model, data, settle_steps, rows=rows)

    input_state = sensor_values(model, data, "large_drum_input_state")
    output_state = sensor_values(model, data, "small_drum_output_state")
    static_input_tension = float(input_state[5])
    static_output_tension = float(output_state[5])
    measured_ratio = static_output_tension / static_input_tension
    input_torque = static_input_tension * input_radius
    output_torque = static_output_tension * output_radius

    shaft_joint = object_id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "compound_drum_hinge"
    )
    input_joint = object_id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "input_slide"
    )
    output_joint = object_id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "output_slide"
    )
    shaft_qpos = int(model.jnt_qposadr[shaft_joint])
    input_qpos = int(model.jnt_qposadr[input_joint])
    output_qpos = int(model.jnt_qposadr[output_joint])
    before_drive = np.array(
        [
            data.qpos[shaft_qpos],
            data.qpos[input_qpos],
            data.qpos[output_qpos],
        ],
        dtype=float,
    )
    settle(
        model,
        data,
        drive_steps,
        input_pull=drive_force,
        rows=rows,
    )
    after_drive = np.array(
        [
            data.qpos[shaft_qpos],
            data.qpos[input_qpos],
            data.qpos[output_qpos],
        ],
        dtype=float,
    )
    displacement = after_drive - before_drive
    displacement_ratio = (
        abs(float(displacement[1] / displacement[2]))
        if abs(float(displacement[2])) > 1e-9
        else float("inf")
    )

    torque_scale = max(abs(input_torque), abs(output_torque), 1e-12)
    checks = {
        "theory_ratio_is_three": abs(theory_ratio - 3.0) <= 1e-12,
        "static_tension_ratio_within_2pct": (
            abs(measured_ratio / theory_ratio - 1.0) <= 0.02
        ),
        "static_torque_balance_within_2pct": (
            abs(input_torque - output_torque) / torque_scale <= 0.02
        ),
        "input_cable_is_taut": bool(round(float(input_state[6]))),
        "output_cable_is_taut": bool(round(float(output_state[6]))),
        "input_pull_moves_input_down": float(displacement[1]) < -0.001,
        "input_pull_moves_output_up": float(displacement[2]) > 0.0003,
        "opposite_spool_motion": float(displacement[0]) < 0,
        "displacement_ratio_matches_radius_ratio": (
            abs(displacement_ratio / theory_ratio - 1.0) <= 0.20
        ),
    }
    report = {
        "pass": all(checks.values()),
        "model": str(model_path),
        "plugin": str(plugin),
        "radii_m": {
            "input": input_radius,
            "output": output_radius,
        },
        "theory": {
            "force_ratio_output_over_input": theory_ratio,
            "displacement_ratio_input_over_output": theory_ratio,
            "torque_balance": "T_input * R_input = T_output * R_output",
        },
        "static": {
            "input_tension_N": static_input_tension,
            "output_tension_N": static_output_tension,
            "measured_force_ratio": measured_ratio,
            "input_torque_Nm": input_torque,
            "output_torque_Nm": output_torque,
            "torque_residual_Nm": input_torque - output_torque,
        },
        "driven": {
            "input_pull_N": drive_force,
            "shaft_rotation_rad": float(displacement[0]),
            "input_displacement_m": float(displacement[1]),
            "output_displacement_m": float(displacement[2]),
            "measured_displacement_ratio": displacement_ratio,
        },
        "checks": checks,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "summary.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    with (output / "timeseries.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument(
        "--out",
        type=Path,
        default=ROOT / "outputs" / "demo21_wheel_axle_force_amplifier",
    )
    parser.add_argument("--settle-steps", type=int, default=3000)
    parser.add_argument("--drive-force", type=float, default=0.12)
    parser.add_argument("--drive-steps", type=int, default=900)
    args = parser.parse_args()
    report = analyze(
        args.plugin.resolve(),
        args.model.resolve(),
        args.out.resolve(),
        args.settle_steps,
        args.drive_force,
        args.drive_steps,
    )
    print(json.dumps(report, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
