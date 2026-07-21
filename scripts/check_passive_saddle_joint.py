#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

try:
    from run_ieee_force_report import (
        FigureStyle,
        draw_png,
        draw_svg,
        draw_vector_page,
    )
except ModuleNotFoundError:
    from scripts.run_ieee_force_report import (
        FigureStyle,
        draw_png,
        draw_svg,
        draw_vector_page,
    )


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "cable_plugin_demos" / "16_cpp_plugin_passive_saddle_joint.xml"
DEFAULT_PLUGIN = ROOT / "build/cable_surface/plugin/libcable_unilateral.dylib"
DEFAULT_OUTPUT = ROOT / "outputs" / "demo16_passive_saddle_joint"

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
LIGAMENTS = (
    ("figure_eight_upper", "passive_figure_eight_upper_state"),
    ("figure_eight_lower", "passive_figure_eight_lower_state"),
)


@dataclass(frozen=True)
class LoadCase:
    name: str
    force: tuple[float, float, float]
    torque: tuple[float, float, float]
    target_axis: int | None


CASES = (
    LoadCase("flexion_positive", (0, 0, 0), (0, 0.002, 0), 1),
    LoadCase("flexion_negative", (0, 0, 0), (0, -0.002, 0), 1),
    LoadCase("abduction_positive", (0, 0, 0), (0, 0, 0.002), 2),
    LoadCase("abduction_negative", (0, 0, 0), (0, 0, -0.002), 2),
    LoadCase("axial_twist", (0, 0, 0), (0.001, 0, 0), 0),
    LoadCase("separation", (0.05, 0, 0), (0, 0, 0), None),
)


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    value = mujoco.mj_name2id(model, kind, name)
    if value < 0:
        raise ValueError(f"MuJoCo object {name!r} was not found")
    return value


def sensor_values(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    sensor_id = object_id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    if dimension != len(SENSOR_FIELDS):
        raise ValueError(f"sensor {name!r} has dimension {dimension}, expected 12")
    return np.asarray(data.sensordata[address : address + dimension], dtype=float).copy()


def quaternion_multiply(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    return np.array(
        [
            first[0] * second[0] - np.dot(first[1:], second[1:]),
            first[0] * second[1] + second[0] * first[1] + first[2] * second[3] - first[3] * second[2],
            first[0] * second[2] + second[0] * first[2] + first[3] * second[1] - first[1] * second[3],
            first[0] * second[3] + second[0] * first[3] + first[1] * second[2] - first[2] * second[1],
        ]
    )


def rotation_vector(current: np.ndarray, initial: np.ndarray) -> np.ndarray:
    relative = quaternion_multiply(current, np.r_[initial[0], -initial[1:]])
    if relative[0] < 0:
        relative = -relative
    vector_norm = float(np.linalg.norm(relative[1:]))
    if vector_norm < 1e-12:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vector_norm, float(np.clip(relative[0], -1.0, 1.0)))
    return relative[1:] * (angle / vector_norm)


def contact_force(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[float, float]:
    resultant = np.zeros(3)
    normal_sum = 0.0
    local = np.zeros(6)
    for contact_id in range(data.ncon):
        mujoco.mj_contactForce(model, data, contact_id, local)
        frame = np.asarray(data.contact[contact_id].frame, dtype=float).reshape(3, 3)
        resultant += frame.T @ local[:3]
        normal_sum += abs(float(local[0]))
    return float(np.linalg.norm(resultant)), normal_sum


def route_diagnostics(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[int, float, float]:
    status = 0
    tangent = 0.0
    surface = 0.0
    for _, sensor_name in LIGAMENTS:
        values = sensor_values(model, data, sensor_name)
        status = max(status, int(round(float(values[8]))))
        tangent = max(tangent, float(values[9]))
        surface = max(surface, float(values[10]))
    return status, tangent, surface


def model_checks(model: mujoco.MjModel) -> dict[str, Any]:
    joint_types = [int(value) for value in model.jnt_type]
    return {
        "nu": int(model.nu),
        "neq": int(model.neq),
        "nq": int(model.nq),
        "nv": int(model.nv),
        "free_joint_count": joint_types.count(int(mujoco.mjtJoint.mjJNT_FREE)),
        "hinge_joint_count": joint_types.count(int(mujoco.mjtJoint.mjJNT_HINGE)),
        "slide_joint_count": joint_types.count(int(mujoco.mjtJoint.mjJNT_SLIDE)),
        "passes": bool(
            model.nu == 0
            and model.neq == 0
            and joint_types.count(int(mujoco.mjtJoint.mjJNT_FREE)) == 1
            and joint_types.count(int(mujoco.mjtJoint.mjJNT_HINGE)) == 0
            and joint_types.count(int(mujoco.mjtJoint.mjJNT_SLIDE)) == 0
        ),
    }


def sample_row(
    case: str,
    phase: str,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_id: int,
    initial_position: np.ndarray,
    initial_quaternion: np.ndarray,
) -> dict[str, Any]:
    position = np.asarray(data.xpos[body_id], dtype=float)
    quaternion = np.asarray(data.xquat[body_id], dtype=float)
    displacement = position - initial_position
    rotation = rotation_vector(quaternion, initial_quaternion)
    resultant_contact, normal_contact = contact_force(model, data)
    row: dict[str, Any] = {
        "case": case,
        "phase": phase,
        "time_s": float(data.time),
        "contact_count": int(data.ncon),
        "contact_resultant_N": resultant_contact,
        "contact_normal_sum_N": normal_contact,
        "displacement_x_mm": 1000.0 * float(displacement[0]),
        "displacement_y_mm": 1000.0 * float(displacement[1]),
        "displacement_z_mm": 1000.0 * float(displacement[2]),
        "displacement_norm_mm": 1000.0 * float(np.linalg.norm(displacement)),
        "rotation_x_deg": math.degrees(float(rotation[0])),
        "rotation_y_deg": math.degrees(float(rotation[1])),
        "rotation_z_deg": math.degrees(float(rotation[2])),
        "rotation_norm_deg": math.degrees(float(np.linalg.norm(rotation))),
    }
    for label, sensor_name in LIGAMENTS:
        values = sensor_values(model, data, sensor_name)
        for field, value in zip(SENSOR_FIELDS, values, strict=True):
            row[f"{label}_{field}"] = float(value)
    return row


def run_case(
    model_path: Path,
    load_case: LoadCase,
    load_time: float,
    release_time: float,
    stride: int,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    body_id = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "distal_metacarpal")
    mujoco.mj_forward(model, data)
    initial_position = np.asarray(data.xpos[body_id], dtype=float).copy()
    initial_quaternion = np.asarray(data.xquat[body_id], dtype=float).copy()
    timestep = float(model.opt.timestep)
    load_steps = max(1, int(round(load_time / timestep)))
    release_steps = max(1, int(round(release_time / timestep)))
    rows: list[dict[str, Any]] = []
    contact_load_steps = 0
    max_contacts = int(data.ncon)
    max_status, max_tangent, max_surface = route_diagnostics(model, data)

    for step in range(load_steps + release_steps + 1):
        applying_load = step < load_steps
        data.xfrc_applied[body_id, :] = 0.0
        if applying_load:
            data.xfrc_applied[body_id, :3] = load_case.force
            data.xfrc_applied[body_id, 3:] = load_case.torque
        phase = "load" if step <= load_steps else "release"
        if step % stride == 0 or step in (load_steps, load_steps + release_steps):
            rows.append(
                sample_row(
                    load_case.name,
                    phase,
                    model,
                    data,
                    body_id,
                    initial_position,
                    initial_quaternion,
                )
            )
        if applying_load and data.ncon > 0:
            contact_load_steps += 1
        max_contacts = max(max_contacts, int(data.ncon))
        status, tangent, surface = route_diagnostics(model, data)
        max_status = max(max_status, status)
        max_tangent = max(max_tangent, tangent)
        max_surface = max(max_surface, surface)
        if step < load_steps + release_steps:
            mujoco.mj_step(model, data)

    load_rows = [row for row in rows if row["phase"] == "load"]
    release_rows = [row for row in rows if row["phase"] == "release"]
    loaded = load_rows[-1]
    released = release_rows[-1]
    axis_angle = (
        abs(float(loaded[("rotation_x_deg", "rotation_y_deg", "rotation_z_deg")[load_case.target_axis]]))
        if load_case.target_axis is not None
        else 0.0
    )
    summary = {
        "case": load_case.name,
        "loaded_axis_rotation_deg": axis_angle,
        "loaded_rotation_norm_deg": float(loaded["rotation_norm_deg"]),
        "loaded_displacement_mm": float(loaded["displacement_norm_mm"]),
        "released_rotation_norm_deg": float(released["rotation_norm_deg"]),
        "released_displacement_mm": float(released["displacement_norm_mm"]),
        "contact_fraction_during_load": contact_load_steps / load_steps,
        "maximum_contact_count": max_contacts,
        "maximum_route_status": max_status,
        "maximum_tangent_residual": max_tangent,
        "maximum_surface_residual_m": max_surface,
    }
    return rows, summary


def run_static(model_path: Path, duration: float, stride: int) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    body_id = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "distal_metacarpal")
    mujoco.mj_forward(model, data)
    initial_position = np.asarray(data.xpos[body_id], dtype=float).copy()
    initial_quaternion = np.asarray(data.xquat[body_id], dtype=float).copy()
    steps = max(1, int(round(duration / float(model.opt.timestep))))
    rows: list[dict[str, Any]] = []
    maximum_contacts = int(data.ncon)
    minimum_contacts = int(data.ncon)
    for step in range(steps + 1):
        if step % stride == 0 or step == steps:
            rows.append(
                sample_row(
                    "static",
                    "static",
                    model,
                    data,
                    body_id,
                    initial_position,
                    initial_quaternion,
                )
            )
        maximum_contacts = max(maximum_contacts, int(data.ncon))
        minimum_contacts = min(minimum_contacts, int(data.ncon))
        if step < steps:
            mujoco.mj_step(model, data)
    final = rows[-1]
    return rows, {
        "duration_s": duration,
        "final_displacement_mm": float(final["displacement_norm_mm"]),
        "final_rotation_deg": float(final["rotation_norm_deg"]),
        "maximum_contact_count": maximum_contacts,
        "minimum_contact_count": minimum_contacts,
    }


def acceptance(static: dict[str, Any], cases: list[dict[str, Any]]) -> dict[str, Any]:
    by_name = {case["case"]: case for case in cases}
    principal = [
        by_name[name]["loaded_axis_rotation_deg"]
        for name in (
            "flexion_positive",
            "flexion_negative",
            "abduction_positive",
            "abduction_negative",
        )
    ]
    axial = by_name["axial_twist"]["loaded_rotation_norm_deg"]
    separation = by_name["separation"]["loaded_displacement_mm"]
    checks = {
        "static_translation_below_0p5mm": static["final_displacement_mm"] < 0.5,
        "static_rotation_below_3deg": static["final_rotation_deg"] < 3.0,
        "principal_rotations_8_to_35deg": all(8.0 <= value <= 35.0 for value in principal),
        "load_contact_fraction_at_least_95pct": all(
            case["contact_fraction_during_load"] >= 0.95 for case in cases[:4]
        ),
        "return_within_3deg": all(case["released_rotation_norm_deg"] <= 3.0 for case in cases[:4]),
        "return_within_1mm": all(case["released_displacement_mm"] <= 1.0 for case in cases[:4]),
        "axial_rotation_below_principal_mean": axial < float(np.mean(principal)),
        "separation_displacement_below_1mm": separation < 1.0,
        "selective_directions_not_hard_locked": axial > 0.01 and separation > 1e-4,
        "static_contact_count_2_to_32": (
            static["minimum_contact_count"] >= 2 and static["maximum_contact_count"] <= 32
        ),
        "maximum_contacts_below_128": all(case["maximum_contact_count"] <= 128 for case in cases),
        "routes_valid_or_degraded": all(case["maximum_route_status"] <= 1 for case in cases),
        "tangent_residual_below_1e_5": all(
            case["maximum_tangent_residual"] < 1e-5 for case in cases
        ),
        "surface_residual_below_1e_7m": all(
            case["maximum_surface_residual_m"] < 1e-7 for case in cases
        ),
    }
    return {"checks": checks, "passes": all(checks.values()), "principal_rotations_deg": principal}


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def draw_figure(output: Path, rows: list[dict[str, Any]]) -> dict[str, Any]:
    colors = {
        "positive": "#C84B31",
        "negative": "#2369A8",
    }
    plotted: list[dict[str, float | str]] = []
    for row in rows:
        case = str(row["case"])
        if case == "static":
            continue
        plotted_row: dict[str, float | str] = {"time_s": float(row["time_s"])}
        if case.startswith("flexion_"):
            response_rotation = float(row["rotation_y_deg"])
        elif case.startswith("abduction_"):
            response_rotation = float(row["rotation_z_deg"])
        else:
            response_rotation = float(row["rotation_norm_deg"])
        plotted_row[f"{case}_rotation_deg"] = response_rotation
        plotted_row[f"{case}_translation_mm"] = float(row["displacement_norm_mm"])
        plotted_row[f"{case}_contacts"] = float(row["contact_count"])
        plotted_row[f"{case}_tension_N"] = max(
            float(row[f"{label}_tension_N"]) for label, _ in LIGAMENTS
        )
        plotted_row[f"{case}_tangent_residual"] = max(
            float(row[f"{label}_tangent_residual"]) for label, _ in LIGAMENTS
        )
        plotted.append(plotted_row)

    panels = [
        (
            "Flexion response (deg)",
            "",
            [
                ("flexion_positive_rotation_deg", colors["positive"], "+"),
                ("flexion_negative_rotation_deg", colors["negative"], "-"),
            ],
        ),
        (
            "Abduction response (deg)",
            "",
            [
                ("abduction_positive_rotation_deg", colors["positive"], "+"),
                ("abduction_negative_rotation_deg", colors["negative"], "-"),
            ],
        ),
        (
            "Translation (mm)",
            "",
            [
                ("flexion_positive_translation_mm", "#C84B31", "F"),
                ("abduction_positive_translation_mm", "#2369A8", "A"),
            ],
        ),
        (
            "Contacts",
            "",
            [
                ("flexion_positive_contacts", "#C84B31", "F"),
                ("abduction_positive_contacts", "#2369A8", "A"),
            ],
        ),
        (
            "Peak ligament tension (N)",
            "",
            [
                ("flexion_positive_tension_N", "#C84B31", "F"),
                ("abduction_positive_tension_N", "#2369A8", "A"),
            ],
        ),
        (
            "Tangent residual",
            "",
            [
                ("flexion_positive_tangent_residual", "#C84B31", "F"),
                ("abduction_positive_tangent_residual", "#2369A8", "A"),
            ],
        ),
    ]
    style = FigureStyle(width_in=7.16, height_in=6.8, dpi=600)
    stem = output / "ieee_passive_saddle_response"
    files = {
        "pdf": str(stem.with_suffix(".pdf")),
        "eps": str(stem.with_suffix(".eps")),
        "svg": str(stem.with_suffix(".svg")),
        "png": str(stem.with_suffix(".png")),
    }
    title = "Passive saddle-joint load and release response"
    draw_svg(Path(files["svg"]), plotted, title, style, panels)
    draw_vector_page(Path(files["pdf"]), plotted, title, style, "pdf", panels)
    draw_vector_page(Path(files["eps"]), plotted, title, style, "eps", panels)
    draw_png(Path(files["png"]), plotted, title, style, panels)
    return {
        **files,
        "width_in": style.width_in,
        "height_in": style.height_in,
        "dpi": style.dpi,
        "font_family": style.font_family,
        "minimum_font_pt": min(style.title_pt, style.text_pt, style.tick_pt),
    }


def analyze(
    plugin: Path,
    model_path: Path,
    output: Path,
    static_time: float,
    load_time: float,
    release_time: float,
    stride: int,
) -> dict[str, Any]:
    mujoco.mj_loadPluginLibrary(str(plugin))
    structure_model = mujoco.MjModel.from_xml_path(str(model_path))
    structure = model_checks(structure_model)
    if not structure["passes"]:
        raise RuntimeError(f"Demo 16 structure check failed: {structure}")
    output.mkdir(parents=True, exist_ok=True)
    static_rows, static_summary = run_static(model_path, static_time, stride)
    all_rows = static_rows.copy()
    case_summaries: list[dict[str, Any]] = []
    for load_case in CASES:
        rows, summary = run_case(model_path, load_case, load_time, release_time, stride)
        all_rows.extend(rows)
        case_summaries.append(summary)
    write_csv(output / "passive_saddle_timeseries.csv", all_rows)
    result = {
        "model": str(model_path),
        "plugin": str(plugin),
        "structure": structure,
        "timing": {
            "static_time_s": static_time,
            "load_time_s": load_time,
            "release_time_s": release_time,
            "stride": stride,
        },
        "static": static_summary,
        "cases": case_summaries,
    }
    result["acceptance"] = acceptance(static_summary, case_summaries)
    result["figures"] = draw_figure(output, all_rows)
    (output / "passive_saddle_summary.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate passive saddle-joint Demo 16")
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--static-time", type=float, default=3.0)
    parser.add_argument("--load-time", type=float, default=0.5)
    parser.add_argument("--release-time", type=float, default=1.5)
    parser.add_argument("--stride", type=int, default=20)
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    result = analyze(
        args.plugin.resolve(),
        args.model.resolve(),
        args.output.resolve(),
        args.static_time,
        args.load_time,
        args.release_time,
        max(1, args.stride),
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return int(args.strict and not result["acceptance"]["passes"])


if __name__ == "__main__":
    raise SystemExit(main())
