#!/usr/bin/env python3
"""Validate the velocity-directed Capstan model on a free inertial pulley."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = (
    ROOT / "cable_plugin_demos/29_cpp_plugin_free_rotating_pulley.xml"
)
DEFAULT_OUTPUT = ROOT / "docs/results/demo29_free_rotating_pulley"


def default_plugin() -> Path:
    configured = os.environ.get("CABLE_PLUGIN_LIBRARY")
    if configured:
        return Path(configured)
    candidates = (
        ROOT / "build/plugin/libcable_unilateral.dylib",
        ROOT / "build/plugin/libcable_unilateral.so",
        ROOT / "build/plugin/libcable_unilateral.dylib",
        ROOT / "build/plugin/libcable_unilateral.so",
    )
    return next((path for path in candidates if path.exists()), candidates[0])


def name2id(model: mujoco.MjModel, objtype: int, name: str) -> int:
    obj_id = mujoco.mj_name2id(model, objtype, name)
    if obj_id < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return obj_id


def plugin_sensor(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    sensor_id = name2id(
        model, mujoco.mjtObj.mjOBJ_SENSOR, "free_pulley_state"
    )
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    if dimension != 12:
        raise ValueError(f"surface cable sensor has dimension {dimension}, expected 12")
    return np.asarray(
        data.sensordata[address : address + dimension], dtype=float
    ).copy()


def percentile(values: list[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=float), q)) if values else 0.0


def portable_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(ROOT))
    except ValueError:
        return resolved.name


def simulate(
    plugin_path: Path,
    model_path: Path = DEFAULT_MODEL,
    duration: float = 0.34,
    sample_stride: int = 5,
) -> tuple[list[dict[str, float]], dict[str, Any]]:
    mujoco.mj_loadPluginLibrary(str(plugin_path.resolve()))
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    data = mujoco.MjData(model)

    joint_names = ("pulley_hinge", "left_slide", "right_slide")
    joint_ids = [
        name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        for name in joint_names
    ]
    pulley_dof, left_dof, right_dof = [
        int(model.jnt_dofadr[joint_id]) for joint_id in joint_ids
    ]
    pulley_geom = name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "free_pulley_wrap"
    )
    radius = float(model.geom_size[pulley_geom, 0])
    stiffness = 15000.0
    mu = 0.12
    velocity_scale = 0.020
    wrap_angle = math.pi
    timestep = float(model.opt.timestep)
    steps = int(math.ceil(duration / timestep))

    mujoco.mj_forward(model, data)
    mass_matrix = np.zeros((model.nv, model.nv), dtype=float)
    mujoco.mj_fullM(model, mass_matrix, data.qM)
    inertia = float(mass_matrix[pulley_dof, pulley_dof])
    initial_sensor = plugin_sensor(model, data)
    initial_energy = float(np.sum(data.energy)) + 0.5 * stiffness * max(
        0.0, float(initial_sensor[4])
    ) ** 2
    cumulative_dissipation = 0.0
    rows: list[dict[str, float]] = []

    for step in range(steps):
        mujoco.mj_step(model, data)
        sensor = plugin_sensor(model, data)

        left_tension = max(0.0, float(data.qfrc_passive[left_dof]))
        right_tension = max(0.0, float(data.qfrc_passive[right_dof]))
        pulley_torque = float(data.qfrc_passive[pulley_dof])
        omega = float(data.qvel[pulley_dof])
        left_velocity = float(data.qvel[left_dof])
        right_velocity = float(data.qvel[right_dof])
        material_speed = 0.5 * (left_velocity - right_velocity)
        rim_speed = radius * omega
        slip_speed = material_speed - rim_speed

        contact_power = (
            (left_tension - right_tension) * material_speed
            + pulley_torque * omega
        )
        dissipated_power = -contact_power
        cumulative_dissipation += dissipated_power * timestep

        effective_exponent = (
            mu * wrap_angle * math.tanh(slip_speed / velocity_scale)
        )
        expected_ratio = math.exp(effective_exponent)
        measured_ratio = (
            right_tension / left_tension if left_tension > 1e-12 else 1.0
        )
        expected_torque = radius * (right_tension - left_tension)
        expected_acceleration = pulley_torque / inertia
        total_energy = float(np.sum(data.energy)) + 0.5 * stiffness * max(
            0.0, float(sensor[4])
        ) ** 2

        if step % sample_stride == 0:
            rows.append(
                {
                    "time_s": float(data.time),
                    "left_tension_N": left_tension,
                    "right_tension_N": right_tension,
                    "tension_difference_N": right_tension - left_tension,
                    "measured_tension_ratio": measured_ratio,
                    "expected_tension_ratio": expected_ratio,
                    "pulley_angle_rad": float(data.qpos[joint_ids[0]]),
                    "pulley_angular_velocity_rad_s": omega,
                    "pulley_angular_acceleration_rad_s2": float(
                        data.qacc[pulley_dof]
                    ),
                    "expected_angular_acceleration_rad_s2": expected_acceleration,
                    "pulley_torque_Nm": pulley_torque,
                    "expected_pulley_torque_Nm": expected_torque,
                    "rope_material_speed_m_s": material_speed,
                    "pulley_rim_speed_m_s": rim_speed,
                    "slip_speed_m_s": slip_speed,
                    "dissipated_power_W": dissipated_power,
                    "cumulative_dissipation_J": cumulative_dissipation,
                    "mechanical_plus_cable_energy_J": total_energy,
                    "energy_change_J": total_energy - initial_energy,
                    "energy_balance_J": total_energy + cumulative_dissipation,
                    "route_length_m": float(sensor[0]),
                    "route_velocity_m_s": float(sensor[1]),
                    "extension_m": float(sensor[4]),
                    "route_tension_N": float(sensor[5]),
                    "route_status": float(sensor[8]),
                    "tangent_residual": float(sensor[9]),
                    "surface_residual_m": float(sensor[10]),
                    "left_displacement_m": float(data.qpos[joint_ids[1]]),
                    "right_displacement_m": float(data.qpos[joint_ids[2]]),
                }
            )

    active = [
        row
        for row in rows
        if row["time_s"] >= 0.03
        and abs(row["rope_material_speed_m_s"]) >= 0.02
        and abs(row["left_displacement_m"]) < 0.095
        and abs(row["right_displacement_m"]) < 0.095
    ]
    ratio_errors = [
        abs(row["measured_tension_ratio"] - row["expected_tension_ratio"])
        / max(abs(row["expected_tension_ratio"]), 1e-12)
        for row in active
    ]
    torque_errors = [
        abs(row["pulley_torque_Nm"] - row["expected_pulley_torque_Nm"])
        / max(abs(row["expected_pulley_torque_Nm"]), 1e-8)
        for row in active
        if abs(row["expected_pulley_torque_Nm"]) > 1e-7
    ]
    acceleration_errors = [
        abs(
            row["pulley_angular_acceleration_rad_s2"]
            - row["expected_angular_acceleration_rad_s2"]
        )
        / max(abs(row["expected_angular_acceleration_rad_s2"]), 1e-8)
        for row in active
        if abs(row["expected_angular_acceleration_rad_s2"]) > 1e-5
    ]
    energy_balance = [row["energy_balance_J"] for row in rows]
    energy_residual = max(
        abs(value - initial_energy) for value in energy_balance
    )
    dissipation_scale = max(abs(cumulative_dissipation), 1e-9)
    relative_energy_residual = energy_residual / dissipation_scale
    negative_power = [max(0.0, -row["dissipated_power_W"]) for row in active]

    checks = {
        "route_remains_valid": all(int(round(row["route_status"])) <= 1 for row in rows),
        "surface_residual_below_1e-7_m": max(
            row["surface_residual_m"] for row in rows
        ) <= 1e-7,
        "pulley_rotates": max(
            abs(row["pulley_angular_velocity_rad_s"]) for row in rows
        ) >= 1.0,
        "tension_difference_is_resolved": max(
            abs(row["tension_difference_N"]) for row in active
        ) >= 0.01,
        "capstan_ratio_matches": percentile(ratio_errors, 95) <= 0.01,
        "pulley_torque_matches_tension_difference": percentile(
            torque_errors, 95
        ) <= 0.01,
        "angular_acceleration_matches_inertia": percentile(
            acceleration_errors, 95
        ) <= 0.03,
        "friction_is_dissipative": percentile(negative_power, 99) <= 2e-5,
        "rim_speed_tracks_rope_speed": percentile(
            [abs(row["slip_speed_m_s"]) for row in active], 95
        ) <= 0.01,
        "energy_balance_closes": relative_energy_residual <= 0.15,
    }
    summary: dict[str, Any] = {
        "pass": all(checks.values()),
        "model": portable_path(model_path),
        "plugin": plugin_path.name,
        "duration_s": duration,
        "timestep_s": timestep,
        "pulley_radius_m": radius,
        "pulley_inertia_kg_m2": inertia,
        "capstan_mu": mu,
        "capstan_velocity_scale_m_s": velocity_scale,
        "wrap_angle_rad": wrap_angle,
        "samples": len(rows),
        "active_samples": len(active),
        "peak_tension_difference_N": max(
            abs(row["tension_difference_N"]) for row in active
        ),
        "peak_pulley_angular_velocity_rad_s": max(
            abs(row["pulley_angular_velocity_rad_s"]) for row in rows
        ),
        "p95_absolute_slip_speed_m_s": percentile(
            [abs(row["slip_speed_m_s"]) for row in active], 95
        ),
        "p95_capstan_ratio_relative_error": percentile(ratio_errors, 95),
        "p95_torque_relative_error": percentile(torque_errors, 95),
        "p95_angular_acceleration_relative_error": percentile(
            acceleration_errors, 95
        ),
        "minimum_dissipated_power_W": min(
            row["dissipated_power_W"] for row in active
        ),
        "cumulative_dissipation_J": cumulative_dissipation,
        "energy_balance_residual_J": energy_residual,
        "energy_balance_residual_relative_to_dissipation": (
            relative_energy_residual
        ),
        "checks": checks,
    }
    return rows, summary


def write_results(
    rows: list[dict[str, float]], summary: dict[str, Any], output_dir: Path
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "free_rotating_pulley_timeseries.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output_dir / "free_rotating_pulley_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )


def plot_results(rows: list[dict[str, float]], output_dir: Path) -> None:
    os.environ.setdefault(
        "MPLCONFIGDIR",
        str(Path(tempfile.gettempdir()) / "mujocable_matplotlib_cache"),
    )
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
            "font.size": 10,
            "axes.labelsize": 10,
            "legend.fontsize": 9,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "lines.linewidth": 1.3,
        }
    )
    values = {key: np.asarray([row[key] for row in rows]) for key in rows[0]}
    time = values["time_s"]
    fig, axes = plt.subplots(2, 2, figsize=(7.16, 5.2), constrained_layout=True)

    axes[0, 0].plot(time, values["left_tension_N"], "-", label=r"$T_1$")
    axes[0, 0].plot(
        time, values["right_tension_N"], "--", label=r"$T_2$"
    )
    axes[0, 0].set_ylabel("Tension (N)")
    axes[0, 0].legend(frameon=False)

    axes[0, 1].plot(
        time, values["rope_material_speed_m_s"], "-", label=r"$u$"
    )
    axes[0, 1].plot(
        time, values["pulley_rim_speed_m_s"], "--", label=r"$R\omega$"
    )
    axes[0, 1].set_ylabel("Speed (m/s)")
    axes[0, 1].legend(frameon=False)

    axes[1, 0].plot(
        time, 1000 * values["slip_speed_m_s"], "-", label="Slip speed"
    )
    axes[1, 0].plot(
        time,
        1000 * values["dissipated_power_W"],
        "--",
        label="Dissipated power",
    )
    axes[1, 0].set_ylabel("Slip (mm/s), power (mW)")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].legend(frameon=False)

    energy_change = 1000 * values["energy_change_J"]
    axes[1, 1].plot(time, energy_change, "-", label=r"$E-E_0$")
    axes[1, 1].plot(
        time,
        -1000 * values["cumulative_dissipation_J"],
        "--",
        label=r"$-\int P_{\rm diss}\,dt$",
    )
    axes[1, 1].set_ylabel("Energy change (mJ)")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].legend(frameon=False)

    for axis in axes.flat:
        axis.grid(True, color="0.86", linewidth=0.6)
        axis.tick_params(direction="in")
    figure_base = output_dir / "free_rotating_pulley_validation"
    fig.savefig(figure_base.with_suffix(".pdf"))
    fig.savefig(figure_base.with_suffix(".svg"))
    fig.savefig(figure_base.with_suffix(".png"), dpi=600)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", type=Path, default=default_plugin())
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--duration", type=float, default=0.34)
    parser.add_argument("--sample-stride", type=int, default=5)
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()

    rows, summary = simulate(
        args.plugin, args.model, args.duration, args.sample_stride
    )
    write_results(rows, summary, args.output_dir)
    if not args.no_plot:
        plot_results(rows, args.output_dir)
    print(json.dumps(summary, indent=2))
    return 1 if args.strict and not summary["pass"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
