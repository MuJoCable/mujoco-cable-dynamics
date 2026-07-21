#!/usr/bin/env python3
"""Compare the Faive virtual-hinge PIP model with the cable/contact model."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import time
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BASELINE = (
    ROOT / "cable_plugin_demos/24_faive_index_pip_virtual_hinge_baseline.xml"
)
DEFAULT_SURFACE = ROOT / "cable_plugin_demos/25_faive_index_pip_surface_cable.xml"
JOINT_AXIS = np.array([-0.990425994580321, -0.104241812967506, -0.090498583905110])


def object_id(model: mujoco.MjModel, object_type: mujoco.mjtObj, name: str) -> int:
    object_id_value = mujoco.mj_name2id(model, object_type, name)
    if object_id_value < 0:
        raise ValueError(f"missing MuJoCo object: {name}")
    return object_id_value


def smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def signed_quaternion_angle(quaternion: np.ndarray, axis: np.ndarray) -> float:
    quaternion = quaternion.copy()
    if quaternion[0] < 0:
        quaternion *= -1
    vector_norm = float(np.linalg.norm(quaternion[1:]))
    if vector_norm < 1e-14:
        return 0.0
    angle = 2.0 * math.atan2(vector_norm, float(quaternion[0]))
    return math.copysign(angle, float(np.dot(quaternion[1:], axis)))


def cable_state(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[float, int]:
    maximum_tension = 0.0
    maximum_status = 0
    for sensor in range(model.nsensor):
        if model.sensor_type[sensor] != mujoco.mjtSensor.mjSENS_PLUGIN:
            continue
        address = model.sensor_adr[sensor]
        maximum_tension = max(maximum_tension, float(data.sensordata[address + 5]))
        if model.sensor_dim[sensor] >= 12:
            maximum_status = max(
                maximum_status, int(round(data.sensordata[address + 8]))
            )
    return maximum_tension, maximum_status


def sample_row(
    label: str,
    protocol: str,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    tip_site: int,
    distal_body: int,
    step_ms: float,
    control_command: float,
    control_command_kind: str,
    actual_contraction: float,
) -> dict[str, float | int | str]:
    if label == "virtual_hinge":
        angle = float(data.qpos[0] + data.qpos[1])
        maximum_tension = math.nan
        route_status = -1
    else:
        angle = signed_quaternion_angle(data.qpos[3:7], JOINT_AXIS)
        maximum_tension, route_status = cable_state(model, data)
    return {
        "model": label,
        "protocol": protocol,
        "time_s": float(data.time),
        "angle_deg": math.degrees(angle),
        "tip_x_m": float(data.site_xpos[tip_site, 0]),
        "tip_y_m": float(data.site_xpos[tip_site, 1]),
        "tip_z_m": float(data.site_xpos[tip_site, 2]),
        "com_x_m": float(data.xipos[distal_body, 0]),
        "com_y_m": float(data.xipos[distal_body, 1]),
        "com_z_m": float(data.xipos[distal_body, 2]),
        "maximum_tension_N": maximum_tension,
        "route_status": route_status,
        "contacts": int(data.ncon),
        "step_ms": step_ms,
        "control_command": control_command,
        "control_command_kind": control_command_kind,
        "actual_contraction_m": actual_contraction,
    }


def check_state(model: mujoco.MjModel, data: mujoco.MjData, step: int) -> None:
    if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
        raise RuntimeError(f"non-finite state at step {step}")
    if float(np.max(np.abs(data.qvel))) > 200.0:
        raise RuntimeError(
            f"unbounded velocity at step {step}: {np.max(np.abs(data.qvel))}"
        )


def run_actuation(
    label: str,
    model_path: Path,
    duration: float,
    baseline_target: float,
    surface_contraction: float,
    ramp_start: float,
    ramp_duration: float,
) -> list[dict[str, float | int | str]]:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    tip_site = object_id(model, mujoco.mjtObj.mjOBJ_SITE, "distal_tip")
    distal_body = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "faive_index_mp")
    stride = max(1, round(0.002 / model.opt.timestep))
    steps = round(duration / model.opt.timestep)
    rows = []
    maximum_route_status_seen = -1
    flexor_sensor_address = -1
    if label != "virtual_hinge":
        sensor = object_id(
            model, mujoco.mjtObj.mjOBJ_SENSOR, "pip_flexor_state"
        )
        flexor_sensor_address = int(model.sensor_adr[sensor])
    for step in range(steps):
        command = smoothstep((data.time - ramp_start) / ramp_duration)
        if label == "virtual_hinge":
            commanded_value = baseline_target * command
            data.ctrl[0] = commanded_value
            command_kind = "hinge_target_rad"
            actual_contraction = math.nan
        else:
            commanded_value = surface_contraction * command
            data.ctrl[0] = commanded_value
            data.ctrl[1] = 0.0
            command_kind = "cable_contraction_m"
        start = time.perf_counter()
        mujoco.mj_step(model, data)
        elapsed_ms = 1000.0 * (time.perf_counter() - start)
        if label != "virtual_hinge":
            actual_contraction = float(
                data.sensordata[flexor_sensor_address + 3]
            )
        check_state(model, data, step)
        if label != "virtual_hinge":
            _, route_status = cable_state(model, data)
            maximum_route_status_seen = max(maximum_route_status_seen, route_status)
        if step % stride == 0:
            row = sample_row(
                label,
                "actuation",
                model,
                data,
                tip_site,
                distal_body,
                elapsed_ms,
                commanded_value,
                command_kind,
                actual_contraction,
            )
            row["route_status_peak"] = maximum_route_status_seen
            rows.append(row)
    return rows


def run_lateral_load(
    label: str,
    model_path: Path,
    duration: float,
    force: float,
) -> list[dict[str, float | int | str]]:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    tip_site = object_id(model, mujoco.mjtObj.mjOBJ_SITE, "distal_tip")
    distal_body = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "faive_index_mp")
    stride = max(1, round(0.002 / model.opt.timestep))
    steps = round(duration / model.opt.timestep)
    rows = []
    maximum_route_status_seen = -1
    for step in range(steps):
        if 0.35 <= data.time < 0.55:
            data.xfrc_applied[distal_body, :3] = force * JOINT_AXIS
        else:
            data.xfrc_applied[distal_body, :3] = 0.0
        start = time.perf_counter()
        mujoco.mj_step(model, data)
        elapsed_ms = 1000.0 * (time.perf_counter() - start)
        check_state(model, data, step)
        if label != "virtual_hinge":
            _, route_status = cable_state(model, data)
            maximum_route_status_seen = max(maximum_route_status_seen, route_status)
        if step % stride == 0:
            row = sample_row(
                label,
                "lateral_load",
                model,
                data,
                tip_site,
                distal_body,
                elapsed_ms,
                0.0,
                "none",
                0.0 if label != "virtual_hinge" else math.nan,
            )
            row["route_status_peak"] = maximum_route_status_seen
            rows.append(row)
    return rows


def summarize(
    rows: list[dict[str, float | int | str]],
    baseline_target: float,
    surface_contraction: float,
    output_matched: bool,
) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    for model_name in ("virtual_hinge", "surface_cable"):
        for protocol in ("actuation", "lateral_load"):
            subset = [
                row
                for row in rows
                if row["model"] == model_name and row["protocol"] == protocol
            ]
            if not subset:
                continue
            initial_com = np.array(
                [subset[0]["com_x_m"], subset[0]["com_y_m"], subset[0]["com_z_m"]],
                dtype=float,
            )
            axial_displacement = [
                float(
                    np.dot(
                        np.array(
                            [row["com_x_m"], row["com_y_m"], row["com_z_m"]],
                            dtype=float,
                        )
                        - initial_com,
                        JOINT_AXIS,
                    )
                )
                for row in subset
            ]
            finite_tensions = [
                float(row["maximum_tension_N"])
                for row in subset
                if math.isfinite(float(row["maximum_tension_N"]))
            ]
            final_angle = float(subset[-1]["angle_deg"])
            rise_threshold = 0.9 * abs(final_angle)
            rise_time = next(
                (
                    float(row["time_s"])
                    for row in subset
                    if abs(float(row["angle_deg"])) >= rise_threshold
                ),
                math.nan,
            )
            tail = subset[-min(25, len(subset)) :]
            tail_dt = float(tail[-1]["time_s"]) - float(tail[0]["time_s"])
            final_rate = (
                (float(tail[-1]["angle_deg"]) - float(tail[0]["angle_deg"]))
                / tail_dt
                if tail_dt > 0
                else math.nan
            )
            summary[f"{model_name}:{protocol}"] = {
                "final_angle_deg": final_angle,
                "peak_abs_angle_deg": max(
                    abs(float(row["angle_deg"])) for row in subset
                ),
                "rise_time_to_90pct_final_s": rise_time,
                "final_angular_rate_deg_s": final_rate,
                "final_control_command": float(subset[-1]["control_command"]),
                "final_actual_contraction_mm": (
                    1000.0 * float(subset[-1]["actual_contraction_m"])
                    if math.isfinite(float(subset[-1]["actual_contraction_m"]))
                    else None
                ),
                "peak_axial_compliance_mm": 1000.0
                * max(abs(value) for value in axial_displacement),
                "residual_axial_displacement_mm": 1000.0 * abs(axial_displacement[-1]),
                "contact_fraction": sum(int(row["contacts"]) > 0 for row in subset)
                / len(subset),
                "maximum_tension_N": max(finite_tensions) if finite_tensions else None,
                "maximum_route_status": max(
                    int(row.get("route_status_peak", row["route_status"]))
                    for row in subset
                ),
                "p95_step_ms": float(
                    np.percentile([float(row["step_ms"]) for row in subset], 95)
                ),
            }
    baseline = summary.get("virtual_hinge:actuation")
    surface = summary.get("surface_cable:actuation")
    comparison: dict[str, Any] = {
        "protocol": "output_matched" if output_matched else "independent_inputs",
        "virtual_hinge_command_per_joint_rad": baseline_target,
        "virtual_hinge_nominal_total_angle_deg": math.degrees(2.0 * baseline_target),
        "surface_contraction_command_mm": 1000.0 * surface_contraction,
        "interpretation": (
            "The virtual-hinge input prescribes joint angle; the surface-cable "
            "input shortens a unilateral cable and the output emerges from contact dynamics."
        ),
    }
    if baseline and surface:
        comparison["final_angle_difference_deg"] = (
            float(surface["final_angle_deg"]) - float(baseline["final_angle_deg"])
        )
        comparison["surface_to_baseline_angle_ratio"] = (
            abs(float(surface["final_angle_deg"]))
            / max(abs(float(baseline["final_angle_deg"])), 1e-12)
        )
    summary["_comparison"] = comparison
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--plugin",
        default=os.environ.get("CABLE_PLUGIN_LIBRARY"),
        help="Path to libcable_unilateral; defaults to CABLE_PLUGIN_LIBRARY",
    )
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument("--surface", type=Path, default=DEFAULT_SURFACE)
    parser.add_argument(
        "--output-dir", type=Path, default=ROOT / "outputs/faive_index_pip_comparison"
    )
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--baseline-target", type=float, default=0.45)
    parser.add_argument("--surface-contraction", type=float, default=0.006)
    parser.add_argument("--ramp-start", type=float, default=0.10)
    parser.add_argument("--ramp-duration", type=float, default=0.35)
    parser.add_argument("--lateral-force", type=float, default=0.005)
    parser.add_argument(
        "--models",
        choices=("both", "virtual_hinge", "surface_cable"),
        default="both",
        help="Run both models or only one side of the comparison",
    )
    parser.add_argument(
        "--skip-lateral-load",
        action="store_true",
        help="Skip the compliance protocol when calibrating actuation",
    )
    parser.add_argument(
        "--output-matched",
        action="store_true",
        help=(
            "Run the surface cable first, then command each virtual hinge to half "
            "of its final angle so the two models are compared at matched output."
        ),
    )
    args = parser.parse_args()
    if not args.plugin:
        parser.error("--plugin or CABLE_PLUGIN_LIBRARY is required")
    if args.output_matched and args.models != "both":
        parser.error("--output-matched requires --models both")

    mujoco.set_mju_user_warning(lambda message: print(f"MuJoCo warning: {message}"))
    mujoco.mj_loadPluginLibrary(str(Path(args.plugin).resolve()))

    rows = []
    effective_baseline_target = args.baseline_target
    if args.output_matched:
        surface_rows = run_actuation(
            "surface_cable",
            args.surface.resolve(),
            args.duration,
            effective_baseline_target,
            args.surface_contraction,
            args.ramp_start,
            args.ramp_duration,
        )
        effective_baseline_target = 0.5 * math.radians(
            abs(float(surface_rows[-1]["angle_deg"]))
        )
        baseline_rows = run_actuation(
            "virtual_hinge",
            args.baseline.resolve(),
            args.duration,
            effective_baseline_target,
            args.surface_contraction,
            args.ramp_start,
            args.ramp_duration,
        )
        rows.extend(baseline_rows)
        rows.extend(surface_rows)
        if not args.skip_lateral_load:
            rows.extend(
                run_lateral_load(
                    "virtual_hinge",
                    args.baseline.resolve(),
                    args.duration,
                    args.lateral_force,
                )
            )
            rows.extend(
                run_lateral_load(
                    "surface_cable",
                    args.surface.resolve(),
                    args.duration,
                    args.lateral_force,
                )
            )
    else:
        model_paths = (
            ("virtual_hinge", args.baseline),
            ("surface_cable", args.surface),
        )
        for label, path in model_paths:
            if args.models != "both" and label != args.models:
                continue
            rows.extend(
                run_actuation(
                    label,
                    path.resolve(),
                    args.duration,
                    effective_baseline_target,
                    args.surface_contraction,
                    args.ramp_start,
                    args.ramp_duration,
                )
            )
            if not args.skip_lateral_load:
                rows.extend(
                    run_lateral_load(
                        label, path.resolve(), args.duration, args.lateral_force
                    )
                )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / "faive_index_pip_comparison.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = summarize(
        rows,
        effective_baseline_target,
        args.surface_contraction,
        args.output_matched,
    )
    json_path = args.output_dir / "faive_index_pip_summary.json"
    json_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f"wrote {csv_path}")
    print(f"wrote {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
