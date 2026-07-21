#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import time
from typing import Any

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "cable_plugin_demos" / "20_cpp_plugin_controlled_saddle_joint.xml"
DEFAULT_PLUGIN = ROOT / "build/cable_surface/plugin/libcable_unilateral.dylib"
DEFAULT_OUTPUT = ROOT / "outputs" / "demo20_controlled_saddle_joint"

SENSOR_FIELDS = (
    "length_m",
    "velocity_mps",
    "free_length_m",
    "contraction_m",
    "extension_m",
    "tension_N",
    "taut",
    "saturated",
    "route_status",
    "tangent_residual",
    "surface_residual_m",
    "solver_iterations",
)
CABLE_SENSORS = (
    ("passive_upper", "passive_figure_eight_upper_state"),
    ("passive_lower", "passive_figure_eight_lower_state"),
    ("control_upper", "upper_control_state"),
    ("control_lower", "lower_control_state"),
)


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    value = mujoco.mj_name2id(model, kind, name)
    if value < 0:
        raise ValueError(f"MuJoCo object {name!r} was not found")
    return value


def sensor_values(
    model: mujoco.MjModel, data: mujoco.MjData, name: str
) -> np.ndarray:
    sensor_id = object_id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    if dimension != len(SENSOR_FIELDS):
        raise ValueError(f"sensor {name!r} has dimension {dimension}, expected 12")
    return np.asarray(
        data.sensordata[address : address + dimension], dtype=float
    ).copy()


def rotation_vector(current: np.ndarray, initial: np.ndarray) -> np.ndarray:
    difference = np.zeros(3)
    mujoco.mju_subQuat(difference, current, initial)
    return difference


def sample_row(
    case: str,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_id: int,
    initial_position: np.ndarray,
    initial_quaternion: np.ndarray,
) -> dict[str, Any]:
    displacement = np.asarray(data.xpos[body_id], dtype=float) - initial_position
    rotation = rotation_vector(
        np.asarray(data.xquat[body_id], dtype=float), initial_quaternion
    )
    row: dict[str, Any] = {
        "case": case,
        "time_s": float(data.time),
        "upper_control_m": float(data.ctrl[0]),
        "lower_control_m": float(data.ctrl[1]),
        "contact_count": int(data.ncon),
        "displacement_x_mm": 1000.0 * float(displacement[0]),
        "displacement_y_mm": 1000.0 * float(displacement[1]),
        "displacement_z_mm": 1000.0 * float(displacement[2]),
        "displacement_norm_mm": 1000.0 * float(np.linalg.norm(displacement)),
        "rotation_x_deg": math.degrees(float(rotation[0])),
        "rotation_y_deg": math.degrees(float(rotation[1])),
        "rotation_z_deg": math.degrees(float(rotation[2])),
        "rotation_norm_deg": math.degrees(float(np.linalg.norm(rotation))),
    }
    for label, sensor_name in CABLE_SENSORS:
        values = sensor_values(model, data, sensor_name)
        for field, value in zip(SENSOR_FIELDS, values, strict=True):
            row[f"{label}_{field}"] = float(value)
    return row


def run_case(
    model_path: Path,
    selected_actuator: str,
    contraction: float,
    release_ratio: float,
    duration: float,
    ramp_start: float,
    ramp_end: float,
    stride: int,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    body_id = object_id(
        model, mujoco.mjtObj.mjOBJ_BODY, "distal_metacarpal"
    )
    selected_id = object_id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, selected_actuator
    )
    other_id = 1 - selected_id
    mujoco.mj_forward(model, data)
    initial_position = np.asarray(data.xpos[body_id], dtype=float).copy()
    initial_quaternion = np.asarray(data.xquat[body_id], dtype=float).copy()
    steps = max(1, int(round(duration / float(model.opt.timestep))))
    rows: list[dict[str, Any]] = []
    step_times_ms: list[float] = []
    contact_steps = 0
    maximum_status = 0
    maximum_tangent_residual = 0.0
    maximum_surface_residual = 0.0
    maximum_tension = 0.0
    any_saturated = False

    for step in range(steps):
        if data.time <= ramp_start:
            fraction = 0.0
        elif data.time >= ramp_end:
            fraction = 1.0
        else:
            fraction = (data.time - ramp_start) / (ramp_end - ramp_start)
        data.ctrl[selected_id] = fraction * contraction
        data.ctrl[other_id] = -fraction * release_ratio * contraction
        started = time.perf_counter()
        mujoco.mj_step(model, data)
        step_times_ms.append(1000.0 * (time.perf_counter() - started))
        contact_steps += int(data.ncon > 0)
        for _, sensor_name in CABLE_SENSORS:
            values = sensor_values(model, data, sensor_name)
            maximum_status = max(maximum_status, int(round(float(values[8]))))
            maximum_tangent_residual = max(
                maximum_tangent_residual, float(values[9])
            )
            maximum_surface_residual = max(
                maximum_surface_residual, float(values[10])
            )
            maximum_tension = max(maximum_tension, float(values[5]))
            any_saturated = any_saturated or bool(round(float(values[7])))
        if step % stride == 0 or step + 1 == steps:
            rows.append(
                sample_row(
                    selected_actuator,
                    model,
                    data,
                    body_id,
                    initial_position,
                    initial_quaternion,
                )
            )

    final = rows[-1]
    summary = {
        "actuator": selected_actuator,
        "final_rotation_x_deg": float(final["rotation_x_deg"]),
        "final_rotation_y_deg": float(final["rotation_y_deg"]),
        "final_rotation_z_deg": float(final["rotation_z_deg"]),
        "final_rotation_norm_deg": float(final["rotation_norm_deg"]),
        "final_displacement_mm": float(final["displacement_norm_mm"]),
        "contact_fraction": contact_steps / steps,
        "maximum_route_status": maximum_status,
        "maximum_tangent_residual": maximum_tangent_residual,
        "maximum_surface_residual_m": maximum_surface_residual,
        "maximum_tension_N": maximum_tension,
        "any_saturated": any_saturated,
        "median_step_time_ms": float(np.median(step_times_ms)),
        "p95_step_time_ms": float(np.percentile(step_times_ms, 95)),
        "maximum_step_time_ms": float(np.max(step_times_ms)),
    }
    return rows, summary


def acceptance(cases: list[dict[str, Any]]) -> dict[str, Any]:
    upper, lower = cases
    upper_angle = float(upper["final_rotation_z_deg"])
    lower_angle = float(lower["final_rotation_z_deg"])
    checks = {
        "opposite_primary_rotation": upper_angle * lower_angle < 0,
        "primary_rotation_8_to_30deg": all(
            8.0 <= abs(angle) <= 30.0 for angle in (upper_angle, lower_angle)
        ),
        "contact_fraction_at_least_95pct": all(
            float(case["contact_fraction"]) >= 0.95 for case in cases
        ),
        "routes_valid_or_degraded": all(
            int(case["maximum_route_status"]) <= 1 for case in cases
        ),
        "tangent_residual_below_1e_5": all(
            float(case["maximum_tangent_residual"]) < 1e-5 for case in cases
        ),
        "surface_residual_below_1e_7m": all(
            float(case["maximum_surface_residual_m"]) < 1e-7 for case in cases
        ),
        "control_tension_not_saturated": all(
            not bool(case["any_saturated"]) and float(case["maximum_tension_N"]) < 15
            for case in cases
        ),
        "p95_step_time_below_5ms": all(
            float(case["p95_step_time_ms"]) < 5.0 for case in cases
        ),
    }
    return {"checks": checks, "passes": all(checks.values())}


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def analyze(
    plugin: Path,
    model_path: Path,
    output: Path,
    contraction: float,
    release_ratio: float,
    duration: float,
    ramp_start: float,
    ramp_end: float,
    stride: int,
) -> dict[str, Any]:
    mujoco.mj_loadPluginLibrary(str(plugin))
    model = mujoco.MjModel.from_xml_path(str(model_path))
    if model.nu != 2 or model.neq != 0:
        raise RuntimeError(
            f"Demo 20 must have nu=2 and neq=0, got nu={model.nu}, neq={model.neq}"
        )
    output.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    summaries: list[dict[str, Any]] = []
    for actuator in ("saddle_upper_control", "saddle_lower_control"):
        case_rows, summary = run_case(
            model_path,
            actuator,
            contraction,
            release_ratio,
            duration,
            ramp_start,
            ramp_end,
            stride,
        )
        rows.extend(case_rows)
        summaries.append(summary)
    write_csv(output / "controlled_saddle_timeseries.csv", rows)
    result = {
        "model": str(model_path),
        "plugin": str(plugin),
        "settings": {
            "contraction_m": contraction,
            "release_ratio": release_ratio,
            "duration_s": duration,
            "ramp_start_s": ramp_start,
            "ramp_end_s": ramp_end,
            "stride": stride,
        },
        "cases": summaries,
        "acceptance": acceptance(summaries),
    }
    (output / "controlled_saddle_summary.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare upper and lower cable actuation in saddle-joint Demo 20"
    )
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--contraction", type=float, default=0.0051)
    parser.add_argument("--release-ratio", type=float, default=0.25)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--ramp-start", type=float, default=0.1)
    parser.add_argument("--ramp-end", type=float, default=0.9)
    parser.add_argument("--stride", type=int, default=20)
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    if args.contraction <= 0:
        parser.error("--contraction must be positive")
    if args.release_ratio < 0:
        parser.error("--release-ratio must be nonnegative")
    if not 0 <= args.ramp_start < args.ramp_end <= args.duration:
        parser.error("require 0 <= ramp-start < ramp-end <= duration")
    result = analyze(
        args.plugin.resolve(),
        args.model.resolve(),
        args.output.resolve(),
        args.contraction,
        args.release_ratio,
        args.duration,
        args.ramp_start,
        args.ramp_end,
        max(1, args.stride),
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return int(args.strict and not result["acceptance"]["passes"])


if __name__ == "__main__":
    raise SystemExit(main())
