#!/usr/bin/env python3
"""Run taut-cable right/neutral/left/neutral bending comparisons."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLUGIN = ROOT / "build/plugin/libcable_unilateral.dylib"
DEFAULT_OUTPUT = ROOT / "outputs/tpu95hf_full_bending"
DEFAULT_MODELS = (
    (
        "demo43_cubic",
        "Demo 43: cubic similarity",
        ROOT / "cable_plugin_demos/43_cpp_plugin_solidworks_spirobot.xml",
    ),
    (
        "demo44_empirical",
        "Demo 44: empirical exponent",
        ROOT / "cable_plugin_demos/44_cpp_plugin_solidworks_spirobot_bending.xml",
    ),
    (
        "demo49_tpu95hf",
        "TPU95HF connector calibration",
        ROOT
        / "cable_plugin_demos/49_cpp_plugin_solidworks_spirobot_tpu95hf_calibrated.xml",
    ),
)
SPOOL_RADIUS_M = 0.010
SEGMENT_COUNT = 18
COLORS = {"right": "#0072B2", "left": "#D55E00", "robot": "#202124"}
MODEL_STYLES = (
    ("#202124", "-", "o"),
    ("#0072B2", "--", "s"),
    ("#D55E00", "-.", "^"),
)
MODEL_FULL_BEND_SETTINGS = {
    "demo43_cubic": {"target_bend_deg": 360.0, "drive_extension_m": 0.0050},
    "demo44_empirical": {"target_bend_deg": 530.0, "drive_extension_m": 0.0150},
    "demo49_tpu95hf": {"target_bend_deg": 530.0, "drive_extension_m": 0.0150},
}


@dataclass
class Result:
    key: str
    label: str
    model_path: Path
    rows: list[dict[str, float | int | str]]
    summary: dict[str, float | int | str | bool]
    stiffness: np.ndarray


def smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def bend_profile(time_s: float, target_bend_deg: float) -> tuple[float, str]:
    """Target bend: negative is toward the right-side cable."""
    if time_s < 1.0:
        return 0.0, "neutral_start"
    if time_s < 5.0:
        return -target_bend_deg * smoothstep((time_s - 1.0) / 4.0), "bend_right"
    if time_s < 6.0:
        return -target_bend_deg, "hold_right"
    if time_s < 10.0:
        return (
            -target_bend_deg * (1.0 - smoothstep((time_s - 6.0) / 4.0)),
            "return_from_right",
        )
    if time_s < 11.0:
        return 0.0, "neutral_middle"
    if time_s < 15.0:
        return target_bend_deg * smoothstep((time_s - 11.0) / 4.0), "bend_left"
    if time_s < 16.0:
        return target_bend_deg, "hold_left"
    if time_s < 20.0:
        return (
            target_bend_deg * (1.0 - smoothstep((time_s - 16.0) / 4.0)),
            "return_from_left",
        )
    return 0.0, "neutral_end"


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    identifier = mujoco.mj_name2id(model, kind, name)
    if identifier < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return identifier


def sensor(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    identifier = object_id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    address = int(model.sensor_adr[identifier])
    dimension = int(model.sensor_dim[identifier])
    return np.asarray(
        data.sensordata[address : address + dimension], dtype=float
    ).copy()


def states(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
    return sensor(model, data, "positive_state"), sensor(
        model, data, "negative_state"
    )


def segment_index(name: str | None) -> int | None:
    if name == "solidworks_base_collision":
        return 0
    prefix = "solidworks_unit_"
    suffix = "_collision"
    if name is None or not name.startswith(prefix) or not name.endswith(suffix):
        return None
    try:
        return int(name[len(prefix) : -len(suffix)])
    except ValueError:
        return None


def segment_contact_metrics(
    model: mujoco.MjModel, data: mujoco.MjData
) -> dict[str, float | int]:
    """Count force-carrying contacts between the robot's solid segments."""
    adjacent_pairs: set[tuple[int, int]] = set()
    positive_pairs: set[tuple[int, int]] = set()
    negative_pairs: set[tuple[int, int]] = set()
    nonadjacent_pairs: set[tuple[int, int]] = set()
    total_normal_force = 0.0
    maximum_penetration = 0.0
    side_tolerance = 1.0e-5

    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        geom1 = int(contact.geom1)
        geom2 = int(contact.geom2)
        name1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1)
        name2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2)
        index1 = segment_index(name1)
        index2 = segment_index(name2)
        if index1 is None or index2 is None or index1 == index2:
            continue

        force = np.zeros(6, dtype=float)
        mujoco.mj_contactForce(model, data, contact_index, force)
        normal_force = max(0.0, float(force[0]))
        if normal_force <= 1.0e-6:
            continue

        total_normal_force += normal_force
        maximum_penetration = max(maximum_penetration, max(0.0, -float(contact.dist)))
        lower_index, upper_index = sorted((index1, index2))
        pair = (lower_index, upper_index)
        if upper_index - lower_index != 1:
            nonadjacent_pairs.add(pair)
            continue

        adjacent_pairs.add(pair)
        proximal_geom = geom1 if index1 == lower_index else geom2
        proximal_body = int(model.geom_bodyid[proximal_geom])
        rotation = np.asarray(data.xmat[proximal_body], dtype=float).reshape(3, 3)
        relative_position = (
            np.asarray(contact.pos, dtype=float)
            - np.asarray(data.xpos[proximal_body], dtype=float)
        )
        local_x = float(rotation[:, 0] @ relative_position)
        if local_x > side_tolerance:
            positive_pairs.add(pair)
        elif local_x < -side_tolerance:
            negative_pairs.add(pair)

    return {
        "segment_contact_pair_count": len(adjacent_pairs) + len(nonadjacent_pairs),
        "adjacent_contact_pair_count": len(adjacent_pairs),
        "positive_side_adjacent_contact_pair_count": len(positive_pairs),
        "negative_side_adjacent_contact_pair_count": len(negative_pairs),
        "nonadjacent_contact_pair_count": len(nonadjacent_pairs),
        "segment_contact_normal_force_N": total_normal_force,
        "maximum_segment_penetration_mm": 1000.0 * maximum_penetration,
    }


def set_config(instance: ET.Element, key: str, value: str) -> None:
    config = next(
        (item for item in instance.findall("config") if item.attrib.get("key") == key),
        None,
    )
    if config is None:
        raise ValueError(f"plugin instance {instance.attrib.get('name')} lacks {key}")
    config.set("value", value)


def common_control_variant(
    source: Path,
    output: Path,
    reserve_m: float,
    control_max_m: float,
    guide_friction_mu: float | None = None,
) -> Path:
    root = ET.parse(source).getroot()
    root.set("model", f"{root.attrib.get('model', 'spirobot')}_taut_comparison")
    compiler = root.find("compiler")
    if compiler is None or "meshdir" not in compiler.attrib:
        raise ValueError(f"model has no meshdir: {source}")
    asset_directory = (source.parent / compiler.attrib["meshdir"]).resolve()
    compiler.set("meshdir", os.path.relpath(asset_directory, output.parent))
    option = root.find("option")
    if option is not None:
        option.set("iterations", "100")
        option.set("ls_iterations", "40")
    default_geom = root.find("./default/geom")
    if default_geom is not None:
        default_geom.set("solref", "0.0015 1")
        default_geom.set("solimp", "0.98 0.999 0.0002")
    for pair in root.findall("./contact/pair"):
        pair.set("solref", "0.0015 1")
        pair.set("solimp", "0.98 0.999 0.0002")
    for instance in root.findall("./extension/plugin/instance"):
        set_config(instance, "pretension_offset", f"{-reserve_m:.12g}")
        if guide_friction_mu is not None:
            set_config(instance, "guide_friction_mu", f"{guide_friction_mu:.12g}")
    for actuator in root.findall("./actuator/plugin"):
        actuator.set("ctrlrange", f"0 {control_max_m:.12g}")
    ET.indent(root, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output, encoding="unicode")
    return output


def series_stiffness(stiffness: np.ndarray) -> float:
    return float(1.0 / np.sum(1.0 / stiffness))


def route_lengths_at_q(
    model: mujoco.MjModel, data: mujoco.MjData, qpos: np.ndarray
) -> np.ndarray:
    data.qpos[:] = qpos
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    right, left = states(model, data)
    return np.asarray((right[0], left[0]), dtype=float)


def route_jacobian(model: mujoco.MjModel, qpos: np.ndarray) -> np.ndarray:
    probe = mujoco.MjData(model)
    epsilon = 1.0e-6
    jacobian = np.zeros((2, model.nq), dtype=float)
    for index in range(model.nq):
        plus = qpos.copy()
        minus = qpos.copy()
        plus[index] += epsilon
        minus[index] -= epsilon
        positive = route_lengths_at_q(model, probe, plus)
        negative = route_lengths_at_q(model, probe, minus)
        jacobian[:, index] = (positive - negative) / (2.0 * epsilon)
    return jacobian


def target_commands(
    route_lengths: np.ndarray,
    home_lengths: np.ndarray,
    reserve_m: float,
    slack_m: float,
    preload_extension_m: float,
    drive_extension_m: float,
    signed_level: float,
    control_range: np.ndarray,
) -> np.ndarray:
    target_extensions = np.full(2, preload_extension_m, dtype=float)
    if signed_level >= 0.0:
        target_extensions[0] += drive_extension_m * signed_level
    else:
        target_extensions[1] -= drive_extension_m * signed_level
    commands = (
        reserve_m
        + slack_m
        + target_extensions
        - (route_lengths - home_lengths)
    )
    return np.clip(commands, control_range[:, 0], control_range[:, 1])


def prestress(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    reserve_m: float,
    slack_m: float,
    preload_extension_m: float,
    settle_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    mujoco.mj_forward(model, data)
    initial_states = states(model, data)
    home_lengths = np.asarray((initial_states[0][0], initial_states[1][0]))
    while data.time < settle_s:
        current_states = states(model, data)
        route_lengths = np.asarray((current_states[0][0], current_states[1][0]))
        data.ctrl[:] = target_commands(
            route_lengths,
            home_lengths,
            reserve_m,
            slack_m,
            preload_extension_m,
            0.0,
            0.0,
            model.actuator_ctrlrange,
        )
        mujoco.mj_step(model, data)
    data.time = 0.0
    mujoco.mj_forward(model, data)
    settled_states = states(model, data)
    reference_lengths = np.asarray((settled_states[0][0], settled_states[1][0]))
    reference_contractions = np.asarray(
        (settled_states[0][3], settled_states[1][3])
    )
    return home_lengths, reference_lengths, reference_contractions


def linear_fit(x: np.ndarray, y: np.ndarray) -> tuple[float, float, float]:
    if x.size < 2 or float(np.ptp(x)) < 1.0e-12:
        return math.nan, math.nan, math.nan
    slope, intercept = np.polyfit(x, y, 1)
    prediction = slope * x + intercept
    residual = float(np.sum((y - prediction) ** 2))
    total = float(np.sum((y - np.mean(y)) ** 2))
    r_squared = 1.0 - residual / total if total > 1.0e-20 else 1.0
    return float(slope), float(intercept), r_squared


def simulation_summary(
    key: str,
    label: str,
    model_path: Path,
    rows: list[dict[str, float | int | str]],
    stiffness: np.ndarray,
) -> dict[str, float | int | str | bool]:
    right_path = np.asarray(
        [float(row["right_route_change_position_mm"]) for row in rows]
    )
    left_path = np.asarray(
        [float(row["left_route_change_position_mm"]) for row in rows]
    )
    right_theory = np.asarray(
        [float(row["right_route_change_linear_theory_mm"]) for row in rows]
    )
    left_theory = np.asarray(
        [float(row["left_route_change_linear_theory_mm"]) for row in rows]
    )
    bend = np.asarray([float(row["robot_bend_angle_position_deg"]) for row in rows])
    time = np.asarray([float(row["time_s"]) for row in rows])
    active = np.abs(bend) >= 5.0
    coupling = linear_fit(right_path[active], left_path[active])
    right_bend = np.asarray([str(row["phase"]) == "bend_right" for row in rows])
    left_bend = np.asarray([str(row["phase"]) == "bend_left" for row in rows])
    right_time_fit = linear_fit(time[right_bend], right_path[right_bend])
    left_time_fit = linear_fit(time[left_bend], left_path[left_bend])
    right_theory_error = right_path - right_theory
    left_theory_error = left_path - left_theory
    right_taut = np.asarray([int(row["right_taut"]) for row in rows])
    left_taut = np.asarray([int(row["left_taut"]) for row in rows])
    right_hold = np.asarray([str(row["phase"]) == "hold_right" for row in rows])
    left_hold = np.asarray([str(row["phase"]) == "hold_left" for row in rows])
    neutral_end = np.asarray([str(row["phase"]) == "neutral_end" for row in rows])
    positive_contacts = np.asarray(
        [int(row["positive_side_adjacent_contact_pair_count"]) for row in rows]
    )
    negative_contacts = np.asarray(
        [int(row["negative_side_adjacent_contact_pair_count"]) for row in rows]
    )
    adjacent_contacts = np.asarray(
        [int(row["adjacent_contact_pair_count"]) for row in rows]
    )
    nonadjacent_contacts = np.asarray(
        [int(row["nonadjacent_contact_pair_count"]) for row in rows]
    )
    penetrations = np.asarray(
        [float(row["maximum_segment_penetration_mm"]) for row in rows]
    )
    target = np.asarray(
        [float(row["robot_bend_target_deg"]) for row in rows], dtype=float
    )
    return {
        "model_key": key,
        "model_label": label,
        "model_file": model_path.name,
        "series_stiffness_Nm_per_rad": series_stiffness(stiffness),
        "minimum_joint_stiffness_Nm_per_rad": float(np.min(stiffness)),
        "maximum_joint_stiffness_Nm_per_rad": float(np.max(stiffness)),
        "right_hold_bend_deg": float(np.mean(bend[right_hold])),
        "left_hold_bend_deg": float(np.mean(bend[left_hold])),
        "right_hold_bending_side_contact_pairs": float(
            np.mean(positive_contacts[right_hold])
        ),
        "left_hold_bending_side_contact_pairs": float(
            np.mean(negative_contacts[left_hold])
        ),
        "right_hold_bending_side_contact_fraction": float(
            np.mean(positive_contacts[right_hold]) / SEGMENT_COUNT
        ),
        "left_hold_bending_side_contact_fraction": float(
            np.mean(negative_contacts[left_hold]) / SEGMENT_COUNT
        ),
        "right_hold_adjacent_contact_pairs": float(
            np.mean(adjacent_contacts[right_hold])
        ),
        "left_hold_adjacent_contact_pairs": float(
            np.mean(adjacent_contacts[left_hold])
        ),
        "right_hold_all_adjacent_pairs_in_contact": bool(
            np.all(adjacent_contacts[right_hold] == SEGMENT_COUNT)
        ),
        "left_hold_all_adjacent_pairs_in_contact": bool(
            np.all(adjacent_contacts[left_hold] == SEGMENT_COUNT)
        ),
        "maximum_nonadjacent_contact_pairs": int(np.max(nonadjacent_contacts)),
        "maximum_segment_penetration_mm": float(np.max(penetrations)),
        "final_return_error_deg": float(np.mean(np.abs(bend[neutral_end]))),
        "maximum_abs_bend_deg": float(np.max(np.abs(bend))),
        "bend_tracking_rmse_deg": float(np.sqrt(np.mean((bend - target) ** 2))),
        "minimum_right_tension_N": min(float(row["right_tension_N"]) for row in rows),
        "minimum_left_tension_N": min(float(row["left_tension_N"]) for row in rows),
        "maximum_tension_N": max(
            max(float(row["right_tension_N"]) for row in rows),
            max(float(row["left_tension_N"]) for row in rows),
        ),
        "right_taut_fraction": float(np.mean(right_taut)),
        "left_taut_fraction": float(np.mean(left_taut)),
        "both_taut_all_samples": bool(np.all((right_taut > 0) & (left_taut > 0))),
        "left_vs_right_path_slope": coupling[0],
        "left_vs_right_path_intercept_mm": coupling[1],
        "left_vs_right_path_r_squared": coupling[2],
        "right_path_linear_theory_rmse_mm": float(
            np.sqrt(np.mean(right_theory_error**2))
        ),
        "left_path_linear_theory_rmse_mm": float(
            np.sqrt(np.mean(left_theory_error**2))
        ),
        "right_bend_path_vs_time_slope_mm_s": right_time_fit[0],
        "right_bend_path_vs_time_r_squared": right_time_fit[2],
        "left_bend_path_vs_time_slope_mm_s": left_time_fit[0],
        "left_bend_path_vs_time_r_squared": left_time_fit[2],
        "maximum_command_m": max(
            max(float(row["right_command_m"]) for row in rows),
            max(float(row["left_command_m"]) for row in rows),
        ),
        "finite": bool(
            np.isfinite(right_path).all()
            and np.isfinite(left_path).all()
            and np.isfinite(bend).all()
        ),
    }


def simulate(
    key: str,
    label: str,
    model_path: Path,
    reserve_m: float,
    slack_m: float,
    preload_extension_m: float,
    drive_extension_m: float,
    target_bend_deg: float,
    angle_error_full_scale_deg: float,
    bend_rate_full_scale_deg_s: float,
    settle_s: float,
    duration_s: float,
    sample_period_s: float,
) -> Result:
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    data = mujoco.MjData(model)
    home_lengths, reference_lengths, reference_contractions = prestress(
        model,
        data,
        reserve_m,
        slack_m,
        preload_extension_m,
        settle_s,
    )
    reference_qpos = np.asarray(data.qpos, dtype=float).copy()
    jacobian = route_jacobian(model, reference_qpos)
    stiffness = np.asarray(model.jnt_stiffness, dtype=float).copy()
    rows: list[dict[str, float | int | str]] = []
    next_sample = 0.0
    previous_path_change = np.zeros(2, dtype=float)
    previous_spool_change = np.zeros(2, dtype=float)
    previous_spool_angle = np.zeros(2, dtype=float)
    previous_bend_deg = 0.0

    while data.time < duration_s - 0.5 * model.opt.timestep:
        current = states(model, data)
        route_lengths = np.asarray((current[0][0], current[1][0]))
        bend_deg_before_step = float(
            np.degrees(np.sum(np.asarray(data.qpos, dtype=float) - reference_qpos))
        )
        bend_rate_deg_s = float(np.degrees(np.sum(np.asarray(data.qvel, dtype=float))))
        target_bend, phase = bend_profile(float(data.time), target_bend_deg)
        bend_error = target_bend - bend_deg_before_step
        # Positive signed_level tensions the right cable and produces a negative
        # bend angle. The derivative term damps the measured bend rate.
        signed_level = float(
            np.clip(
                -bend_error / angle_error_full_scale_deg
                + bend_rate_deg_s / bend_rate_full_scale_deg_s,
                -1.0,
                1.0,
            )
        )
        data.ctrl[:] = target_commands(
            route_lengths,
            home_lengths,
            reserve_m,
            slack_m,
            preload_extension_m,
            drive_extension_m,
            signed_level,
            model.actuator_ctrlrange,
        )
        mujoco.mj_step(model, data)
        if data.time + 1.0e-12 < next_sample:
            continue

        right, left = states(model, data)
        contacts = segment_contact_metrics(model, data)
        qpos_delta = np.asarray(data.qpos, dtype=float) - reference_qpos
        route = np.asarray((right[0], left[0]))
        contractions = np.asarray((right[3], left[3]))
        path_change_mm = 1000.0 * (route - reference_lengths)
        spool_change_mm = 1000.0 * (contractions - reference_contractions)
        spool_angle_rad = (contractions - reference_contractions) / SPOOL_RADIUS_M
        theory_mm = 1000.0 * (jacobian @ qpos_delta)
        bend_deg = float(np.degrees(np.sum(qpos_delta)))
        row: dict[str, float | int | str] = {
            "time_s": float(data.time),
            "phase": phase,
            "right_route_length_mm": 1000.0 * float(right[0]),
            "right_route_change_position_mm": float(path_change_mm[0]),
            "right_route_change_increment_mm": float(
                path_change_mm[0] - previous_path_change[0]
            ),
            "left_route_length_mm": 1000.0 * float(left[0]),
            "left_route_change_position_mm": float(path_change_mm[1]),
            "left_route_change_increment_mm": float(
                path_change_mm[1] - previous_path_change[1]
            ),
            "right_spool_cable_position_mm": float(spool_change_mm[0]),
            "right_spool_cable_increment_mm": float(
                spool_change_mm[0] - previous_spool_change[0]
            ),
            "left_spool_cable_position_mm": float(spool_change_mm[1]),
            "left_spool_cable_increment_mm": float(
                spool_change_mm[1] - previous_spool_change[1]
            ),
            "right_spool_angle_position_rad": float(spool_angle_rad[0]),
            "right_spool_angle_increment_rad": float(
                spool_angle_rad[0] - previous_spool_angle[0]
            ),
            "left_spool_angle_position_rad": float(spool_angle_rad[1]),
            "left_spool_angle_increment_rad": float(
                spool_angle_rad[1] - previous_spool_angle[1]
            ),
            "robot_bend_angle_position_deg": bend_deg,
            "robot_bend_angle_increment_deg": bend_deg - previous_bend_deg,
            "robot_bend_target_deg": target_bend,
            "differential_drive_level": signed_level,
            "right_route_change_linear_theory_mm": float(theory_mm[0]),
            "left_route_change_linear_theory_mm": float(theory_mm[1]),
            "right_command_m": float(data.ctrl[0]),
            "left_command_m": float(data.ctrl[1]),
            "right_tension_N": float(right[5]),
            "left_tension_N": float(left[5]),
            "right_extension_mm": 1000.0 * float(right[4]),
            "left_extension_mm": 1000.0 * float(left[4]),
            "right_taut": int(right[6] > 0.5),
            "left_taut": int(left[6] > 0.5),
        }
        row.update(contacts)
        for index, angle in enumerate(np.degrees(qpos_delta), start=1):
            row[f"joint_{index:02d}_angle_deg"] = float(angle)
        rows.append(row)
        previous_path_change = path_change_mm
        previous_spool_change = spool_change_mm
        previous_spool_angle = spool_angle_rad
        previous_bend_deg = bend_deg
        next_sample += sample_period_s

    rebase_cycle_rows(rows)
    summary = simulation_summary(key, label, model_path, rows, stiffness)
    summary["target_bend_deg"] = target_bend_deg
    summary["drive_extension_m"] = drive_extension_m
    return Result(key, label, model_path, rows, summary, stiffness)


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def rebase_cycle_rows(rows: list[dict[str, float | int | str]]) -> None:
    """Make all reported displacement channels start exactly at zero."""
    channels = (
        ("right_route_change_position_mm", "right_route_change_increment_mm"),
        ("left_route_change_position_mm", "left_route_change_increment_mm"),
        ("right_spool_cable_position_mm", "right_spool_cable_increment_mm"),
        ("left_spool_cable_position_mm", "left_spool_cable_increment_mm"),
        ("right_spool_angle_position_rad", "right_spool_angle_increment_rad"),
        ("left_spool_angle_position_rad", "left_spool_angle_increment_rad"),
        ("robot_bend_angle_position_deg", "robot_bend_angle_increment_deg"),
    )
    for position_key, increment_key in channels:
        baseline = float(rows[0][position_key])
        previous = 0.0
        for row in rows:
            position = float(row[position_key]) - baseline
            row[position_key] = position
            row[increment_key] = position - previous
            previous = position


def servo_trajectory_rows(
    rows: list[dict[str, float | int | str]],
) -> list[dict[str, float | int | str]]:
    compact: list[dict[str, float | int | str]] = []
    for row in rows:
        compact.append(
            {
                "time_s": row["time_s"],
                "phase": row["phase"],
                "right_route_length_change_absolute_mm": row[
                    "right_route_change_position_mm"
                ],
                "right_route_length_change_relative_mm": row[
                    "right_route_change_increment_mm"
                ],
                "right_spool_takeup_absolute_mm": row[
                    "right_spool_cable_position_mm"
                ],
                "right_spool_takeup_relative_mm": row[
                    "right_spool_cable_increment_mm"
                ],
                "right_servo_angle_absolute_rad": row[
                    "right_spool_angle_position_rad"
                ],
                "right_servo_angle_relative_rad": row[
                    "right_spool_angle_increment_rad"
                ],
                "right_tension_N": row["right_tension_N"],
                "right_taut": row["right_taut"],
                "left_route_length_change_absolute_mm": row[
                    "left_route_change_position_mm"
                ],
                "left_route_length_change_relative_mm": row[
                    "left_route_change_increment_mm"
                ],
                "left_spool_takeup_absolute_mm": row[
                    "left_spool_cable_position_mm"
                ],
                "left_spool_takeup_relative_mm": row[
                    "left_spool_cable_increment_mm"
                ],
                "left_servo_angle_absolute_rad": row[
                    "left_spool_angle_position_rad"
                ],
                "left_servo_angle_relative_rad": row[
                    "left_spool_angle_increment_rad"
                ],
                "left_tension_N": row["left_tension_N"],
                "left_taut": row["left_taut"],
                "robot_bend_angle_absolute_deg": row[
                    "robot_bend_angle_position_deg"
                ],
                "robot_bend_angle_relative_deg": row[
                    "robot_bend_angle_increment_deg"
                ],
                "positive_side_adjacent_contact_pair_count": row[
                    "positive_side_adjacent_contact_pair_count"
                ],
                "negative_side_adjacent_contact_pair_count": row[
                    "negative_side_adjacent_contact_pair_count"
                ],
            }
        )
    return compact


def configure_plotting() -> None:
    mpl.rcParams.update(
        {
            "font.family": "Times New Roman",
            "font.size": 10,
            "axes.labelsize": 10,
            "axes.titlesize": 10,
            "xtick.labelsize": 10,
            "ytick.labelsize": 10,
            "legend.fontsize": 9,
            "axes.linewidth": 0.8,
            "lines.linewidth": 1.3,
            "lines.markersize": 4.0,
            "grid.linewidth": 0.45,
            "grid.color": "#C8CDD2",
            "grid.alpha": 0.65,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "none",
            "savefig.facecolor": "white",
        }
    )


def style_axis(axis: plt.Axes) -> None:
    axis.grid(True)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)
    axis.tick_params(direction="out", length=3, width=0.7)


def export_figure(figure: plt.Figure, stem: Path) -> None:
    stem.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(stem.with_suffix(".pdf"), bbox_inches="tight")
    figure.savefig(stem.with_suffix(".svg"), bbox_inches="tight")
    figure.savefig(stem.with_suffix(".png"), dpi=600, bbox_inches="tight")
    plt.close(figure)


def plot_result(result: Result, output: Path) -> None:
    time = np.asarray([float(row["time_s"]) for row in result.rows])
    right = np.asarray(
        [float(row["right_route_change_position_mm"]) for row in result.rows]
    )
    left = np.asarray(
        [float(row["left_route_change_position_mm"]) for row in result.rows]
    )
    right_theory = np.asarray(
        [float(row["right_route_change_linear_theory_mm"]) for row in result.rows]
    )
    left_theory = np.asarray(
        [float(row["left_route_change_linear_theory_mm"]) for row in result.rows]
    )
    right_angle = np.asarray(
        [float(row["right_spool_angle_position_rad"]) for row in result.rows]
    )
    left_angle = np.asarray(
        [float(row["left_spool_angle_position_rad"]) for row in result.rows]
    )
    bend = np.asarray(
        [float(row["robot_bend_angle_position_deg"]) for row in result.rows]
    )
    markevery = max(1, len(time) // 20)
    figure, axes = plt.subplots(
        2, 1, figsize=(7.16, 4.8), sharex=True, constrained_layout=True
    )
    axes[0].plot(
        time,
        right,
        color=COLORS["right"],
        linestyle="-",
        marker="o",
        markevery=markevery,
        label="Right cable: simulated",
    )
    axes[0].plot(
        time,
        right_theory,
        color=COLORS["right"],
        linestyle="--",
        label="Right cable: linear theory",
    )
    axes[0].plot(
        time,
        left,
        color=COLORS["left"],
        linestyle="-.",
        marker="s",
        markevery=markevery,
        label="Left cable: simulated",
    )
    axes[0].plot(
        time,
        left_theory,
        color=COLORS["left"],
        linestyle=":",
        label="Left cable: linear theory",
    )
    axes[0].set_ylabel("Route-length change (mm)")
    axes[0].set_title(result.label)
    axes[0].legend(ncol=2, loc="best")
    style_axis(axes[0])

    axes[1].plot(
        time,
        right_angle,
        color=COLORS["right"],
        linestyle="-",
        marker="o",
        markevery=markevery,
        label="Right spool",
    )
    axes[1].plot(
        time,
        left_angle,
        color=COLORS["left"],
        linestyle="--",
        marker="s",
        markevery=markevery,
        label="Left spool",
    )
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Spool angle (rad)")
    style_axis(axes[1])
    angle_axis = axes[1].twinx()
    angle_axis.plot(time, bend, color=COLORS["robot"], linestyle=":", label="Robot bend")
    angle_axis.set_ylabel("Robot bend (deg)")
    angle_axis.spines["top"].set_visible(False)
    handles_a, labels_a = axes[1].get_legend_handles_labels()
    handles_b, labels_b = angle_axis.get_legend_handles_labels()
    axes[1].legend(handles_a + handles_b, labels_a + labels_b, ncol=3, loc="best")
    export_figure(figure, output / result.key / "bending_cycle")


def plot_cable_length_time(result: Result, output: Path) -> None:
    time = np.asarray([float(row["time_s"]) for row in result.rows])
    right = np.asarray(
        [float(row["right_route_change_position_mm"]) for row in result.rows]
    )
    left = np.asarray(
        [float(row["left_route_change_position_mm"]) for row in result.rows]
    )
    markevery = max(1, len(time) // 24)
    figure, axis = plt.subplots(figsize=(7.16, 2.8), constrained_layout=True)
    axis.plot(
        time,
        right,
        color=COLORS["right"],
        linestyle="-",
        marker="o",
        markevery=markevery,
        label="Right cable",
    )
    axis.plot(
        time,
        left,
        color=COLORS["left"],
        linestyle="--",
        marker="s",
        markevery=markevery,
        label="Left cable",
    )
    axis.axhline(0.0, color="#777777", linewidth=0.7, linestyle=":")
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Route-length change (mm)")
    axis.set_title(result.label)
    axis.legend(ncol=2, loc="best")
    style_axis(axis)
    export_figure(figure, output / result.key / "cable_length_change_vs_time")


def plot_comparison(results: list[Result], output: Path) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(7.16, 3.0), constrained_layout=True)
    for result, (color, line, marker) in zip(results, MODEL_STYLES):
        time = np.asarray([float(row["time_s"]) for row in result.rows])
        bend = np.asarray(
            [float(row["robot_bend_angle_position_deg"]) for row in result.rows]
        )
        right = np.asarray(
            [float(row["right_route_change_position_mm"]) for row in result.rows]
        )
        left = np.asarray(
            [float(row["left_route_change_position_mm"]) for row in result.rows]
        )
        markevery = max(1, len(time) // 16)
        axes[0].plot(
            time,
            bend,
            color=color,
            linestyle=line,
            marker=marker,
            markevery=markevery,
            label=result.label,
        )
        axes[1].plot(
            right,
            left,
            color=color,
            linestyle=line,
            marker=marker,
            markevery=markevery,
            label=result.label,
        )
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Total bend (deg)")
    axes[0].legend(loc="best")
    style_axis(axes[0])
    limits = axes[1].get_xlim()
    maximum = max(abs(limits[0]), abs(limits[1]), 1.0)
    axes[1].plot(
        (-maximum, maximum),
        (maximum, -maximum),
        color="#777777",
        linestyle=":",
        label=r"Ideal $\Delta L_L=-\Delta L_R$",
    )
    axes[1].set_xlabel("Right route-length change (mm)")
    axes[1].set_ylabel("Left route-length change (mm)")
    axes[1].legend(loc="best")
    style_axis(axes[1])
    export_figure(figure, output / "model_comparison")


def write_report(output: Path, results: list[Result], arguments: argparse.Namespace) -> None:
    lines = [
        "# TPU95HF Bidirectional Taut-Cable Bending Comparison",
        "",
        "The cycle is right bend, return to neutral, left bend, and return to neutral. "
        "A geometry-following antagonist controller pays out each cable while retaining "
        "a positive target extension, so both unilateral cables remain in tension.",
        "",
        "For a 20 mm diameter spool, the reported servo angle is",
        "",
        r"$$\phi_i(t)=\frac{u_i(t)-u_i(0)}{r_s},\qquad "
        r"r_s=10\ \mathrm{mm}.$$",
        "The exact massless-cable kinematics are",
        "",
        r"$$\dot L_i=J_i(q)\dot q,\qquad "
        r"\Delta L_i=\int_{q_0}^{q}J_i(\xi)\,\mathrm d\xi.$$",
        "For planar bending with a constant routing offset $d$, this reduces to "
        r"$\Delta L_R=-d\Theta$ and $\Delta L_L=d\Theta$. The first-order "
        "route prediction freezes the neutral cable Jacobian:",
        "",
        r"$$\Delta L_i^{\mathrm{lin}}(t)=\sum_j "
        r"\left.\frac{\partial L_i}{\partial q_j}\right|_{q_0} "
        r"[q_j(t)-q_j(0)].$$",
        "",
        "| Model | Target (deg) | Series stiffness (N m/rad) | Right hold (deg) | Left hold (deg) | "
        "Both taut | Adjacent contacts R/L | Max penetration (mm) | "
        "Coupling $R^2$ | Linear-theory RMSE (R/L, mm) |",
        "|---|---:|---:|---:|---:|:---:|---:|---:|---:|---:|",
    ]
    for result in results:
        item = result.summary
        lines.append(
            f"| {result.label} | {float(item['target_bend_deg']):.0f} | "
            f"{float(item['series_stiffness_Nm_per_rad']):.3e} | "
            f"{float(item['right_hold_bend_deg']):.1f} | "
            f"{float(item['left_hold_bend_deg']):.1f} | "
            f"{'yes' if item['both_taut_all_samples'] else 'no'} | "
            f"{float(item['right_hold_adjacent_contact_pairs']):.1f} / "
            f"{float(item['left_hold_adjacent_contact_pairs']):.1f} | "
            f"{float(item['maximum_segment_penetration_mm']):.4f} | "
            f"{float(item['left_vs_right_path_r_squared']):.4f} | "
            f"{float(item['right_path_linear_theory_rmse_mm']):.3f} / "
            f"{float(item['left_path_linear_theory_rmse_mm']):.3f} |"
        )
    lines.extend(
        [
            "",
            "The path changes are not expected to be linear in time because the imposed "
            "trajectory uses a smoothstep ramp and the robot is dynamic. Approximate "
            "linearity should instead be assessed between cable length and joint motion, "
            "or between the antagonistic cable changes. Deviations from the neutral "
            "Jacobian quantify finite-rotation geometry and contact effects. Once segment "
            "contact or a joint limit becomes active, $J_i(q)$ changes branch, so the "
            "length-angle law is only piecewise smooth. Damping and guide friction make "
            "the return path dynamically hysteretic even when both cables remain taut.",
            "",
            f"Controller settings: preload extension = "
            f"{1000.0 * arguments.preload_extension:.3f} mm, reserve = "
            f"{1000.0 * arguments.reserve:.1f} mm, spool diameter = 20 mm. "
            "Each model uses the stable full-bend target and drive extension listed "
            "in summary.json unless a command-line override is supplied. "
            "The damping values remain provisional because the material datasheet does "
            "not provide a printed-part loss model.",
            "",
        ]
    )
    (output / "README.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--models",
        nargs="+",
        choices=tuple(item[0] for item in DEFAULT_MODELS),
        default=[item[0] for item in DEFAULT_MODELS],
    )
    parser.add_argument("--reserve", type=float, default=0.090)
    parser.add_argument("--control-max", type=float, default=0.200)
    parser.add_argument("--slack", type=float, default=0.00015)
    parser.add_argument("--preload-extension", type=float, default=0.0010)
    parser.add_argument("--drive-extension", type=float)
    parser.add_argument("--target-bend-deg", type=float)
    parser.add_argument(
        "--guide-friction-mu",
        type=float,
        help="override guide_friction_mu for both antagonistic cables",
    )
    parser.add_argument("--angle-error-full-scale-deg", type=float, default=30.0)
    parser.add_argument(
        "--bend-rate-full-scale-deg-s", type=float, default=180.0
    )
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=24.0)
    parser.add_argument("--sample-period", type=float, default=0.02)
    args = parser.parse_args()
    if args.duration < 24.0:
        parser.error("duration must be at least 24 s for the full cycle")
    if args.reserve <= 0.0 or args.control_max <= args.reserve:
        parser.error("control-max must exceed a positive reserve")
    if args.preload_extension <= 0.0 or (
        args.drive_extension is not None and args.drive_extension <= 0.0
    ):
        parser.error("preload and drive extensions must be positive")
    if args.guide_friction_mu is not None and args.guide_friction_mu < 0.0:
        parser.error("guide friction must be nonnegative")
    if (
        (args.target_bend_deg is not None and args.target_bend_deg <= 0.0)
        or args.angle_error_full_scale_deg <= 0.0
        or args.bend_rate_full_scale_deg_s <= 0.0
    ):
        parser.error("bend target and controller scales must be positive")
    if not args.plugin.is_file():
        parser.error(f"plugin library not found: {args.plugin}")

    calibrated_model = DEFAULT_MODELS[-1][2]
    if not calibrated_model.is_file():
        raise FileNotFoundError(
            f"generate {calibrated_model.name} with "
            "scripts/prepare_tpu95hf_calibrated_spirobot.py first"
        )

    output = args.output.resolve()
    model_directory = output / "models"
    mujoco.mj_loadPluginLibrary(str(args.plugin.resolve()))
    results: list[Result] = []
    selected = set(args.models)
    for key, label, source in DEFAULT_MODELS:
        if key not in selected:
            continue
        settings = MODEL_FULL_BEND_SETTINGS[key]
        target_bend_deg = (
            args.target_bend_deg
            if args.target_bend_deg is not None
            else settings["target_bend_deg"]
        )
        drive_extension_m = (
            args.drive_extension
            if args.drive_extension is not None
            else settings["drive_extension_m"]
        )
        variant = common_control_variant(
            source,
            model_directory / f"{key}_taut_control.xml",
            args.reserve,
            args.control_max,
            args.guide_friction_mu,
        )
        result = simulate(
            key,
            label,
            variant,
            args.reserve,
            args.slack,
            args.preload_extension,
            drive_extension_m,
            target_bend_deg,
            args.angle_error_full_scale_deg,
            args.bend_rate_full_scale_deg_s,
            args.settle,
            args.duration,
            args.sample_period,
        )
        results.append(result)
        write_csv(output / key / "bending_cycle_timeseries.csv", result.rows)
        write_csv(
            output / key / "servo_trajectory.csv",
            servo_trajectory_rows(result.rows),
        )
        (output / key / "summary.json").write_text(
            json.dumps(result.summary, indent=2) + "\n", encoding="utf-8"
        )

    write_csv(output / "comparison_summary.csv", [item.summary for item in results])
    stiffness_rows: list[dict[str, object]] = []
    for result in results:
        for index, stiffness in enumerate(result.stiffness, start=1):
            stiffness_rows.append(
                {
                    "model_key": result.key,
                    "model_label": result.label,
                    "joint": index,
                    "stiffness_Nm_per_rad": float(stiffness),
                }
            )
    write_csv(output / "joint_stiffness_comparison.csv", stiffness_rows)
    configure_plotting()
    for result in results:
        plot_result(result, output)
        plot_cable_length_time(result, output)
    plot_comparison(results, output)
    write_report(output, results, args)
    (output / "summary.json").write_text(
        json.dumps(
            {
                "spool_radius_m": SPOOL_RADIUS_M,
                "reserve_m": args.reserve,
                "preload_extension_m": args.preload_extension,
                "drive_extension_m": args.drive_extension,
                "guide_friction_mu": args.guide_friction_mu,
                "model_full_bend_settings": {
                    result.key: {
                        "target_bend_deg": result.summary["target_bend_deg"],
                        "drive_extension_m": result.summary["drive_extension_m"],
                    }
                    for result in results
                },
                "duration_s": args.duration,
                "sample_period_s": args.sample_period,
                "models": [result.summary for result in results],
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(json.dumps([result.summary for result in results], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
