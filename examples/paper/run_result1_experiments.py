#!/usr/bin/env python3
"""Run the frozen Result 1 models and build their paper evidence bundle."""

from __future__ import annotations

import argparse
import csv
import ctypes
import json
import math
import shutil
import subprocess
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib as mpl

mpl.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


HERE = Path(__file__).resolve().parent
FONT_PATHS = (
    Path("/System/Library/Fonts/Supplemental/Times New Roman.ttf"),
    Path("/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf"),
)
PLUGIN_FIELDS = (
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
THEORY_COLOR = "#666666"
SIM_COLORS = ("#0072B2", "#D55E00", "#009E73", "#CC79A7")


@dataclass(frozen=True)
class Case:
    key: str
    result_id: str
    model: str
    kind: str
    duration_s: float
    ramp_s: float = 0.0
    command_final: float = 0.0
    title: str = ""


CASES = (
    Case(
        "result_1_1_mu_0",
        "result_1_1",
        "result_1_1/models/result_1_1.xml",
        "fixed",
        3.0,
        1.8,
        0.025,
        "Fixed pulley, frictionless",
    ),
    Case(
        "result_1_1_mu_0p15",
        "result_1_1",
        "result_1_1/models/result_1_1_mu_0p15.xml",
        "fixed",
        3.0,
        1.8,
        0.025,
        "Fixed pulley, mu = 0.15",
    ),
    Case(
        "result_1_1_b_mu_0",
        "result_1_1_b",
        "result_1_1_b/models/result_1_1_b_mu_0.xml",
        "rotating",
        3.0,
        1.8,
        0.025,
        "Free pulley, frictionless",
    ),
    Case(
        "result_1_1_b_mu_0p15",
        "result_1_1_b",
        "result_1_1_b/models/result_1_1_b.xml",
        "rotating",
        3.0,
        1.8,
        0.025,
        "Free pulley, mu = 0.15",
    ),
    Case(
        "result_1_1_c_winch",
        "result_1_1_c",
        "result_1_1_c/models/result_1_1_c.xml",
        "winch",
        3.5,
        2.2,
        1.6,
        "Physical winch and ideal radius transmission",
    ),
    Case(
        "result_1_2_moving_pulley",
        "result_1_2",
        "result_1_2/models/result_1_2.xml",
        "moving",
        3.5,
        2.2,
        0.020,
        "Single moving pulley",
    ),
    Case(
        "result_1_3_atwood_landing",
        "result_1_3",
        "result_1_3/models/result_1_3.xml",
        "atwood",
        1.25,
        0.0,
        0.0,
        "Inertial Atwood pulley with landing",
    ),
)


def load_plugin(path: Path) -> None:
    ctypes.CDLL(str(path.resolve()), mode=ctypes.RTLD_GLOBAL)


def configure_plotting() -> None:
    font = next((path for path in FONT_PATHS if path.is_file()), None)
    if font is not None:
        mpl.font_manager.fontManager.addfont(str(font))
    mpl.rcParams.update(
        {
            "font.family": "Times New Roman" if font else "serif",
            "font.size": 10,
            "axes.labelsize": 10,
            "axes.titlesize": 10,
            "legend.fontsize": 9,
            "xtick.labelsize": 10,
            "ytick.labelsize": 10,
            "axes.linewidth": 0.8,
            "lines.linewidth": 1.5,
            "lines.markersize": 4.0,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "none",
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
        }
    )


def object_id(model: mujoco.MjModel, kind, name: str) -> int:
    value = mujoco.mj_name2id(model, kind, name)
    if value < 0:
        raise ValueError(f"object not found: {name}")
    return value


def optional_id(model: mujoco.MjModel, kind, name: str) -> int:
    return mujoco.mj_name2id(model, kind, name)


def numeric(model: mujoco.MjModel, name: str, default: float = math.nan) -> float:
    index = optional_id(model, mujoco.mjtObj.mjOBJ_NUMERIC, name)
    if index < 0:
        return default
    return float(model.numeric_data[model.numeric_adr[index]])


def sensor(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    index = object_id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    address = int(model.sensor_adr[index])
    dimension = int(model.sensor_dim[index])
    return np.asarray(data.sensordata[address : address + dimension], dtype=float).copy()


def plugin_state(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
    for index in range(model.nsensor):
        if model.sensor_type[index] != mujoco.mjtSensor.mjSENS_PLUGIN:
            continue
        address = int(model.sensor_adr[index])
        dimension = int(model.sensor_dim[index])
        values = data.sensordata[address : address + dimension]
        return {
            name: float(values[i]) if i < dimension else math.nan
            for i, name in enumerate(PLUGIN_FIELDS)
        }
    raise ValueError("Result 1 model does not contain a plugin sensor")


def body_mass(model: mujoco.MjModel, name: str) -> float:
    return float(model.body_mass[object_id(model, mujoco.mjtObj.mjOBJ_BODY, name)])


def body_position(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    return np.asarray(
        data.xpos[object_id(model, mujoco.mjtObj.mjOBJ_BODY, name)], dtype=float
    ).copy()


def joint_values(
    model: mujoco.MjModel, data: mujoco.MjData, name: str
) -> tuple[float, float, int]:
    joint_id = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    qadr = int(model.jnt_qposadr[joint_id])
    dadr = int(model.jnt_dofadr[joint_id])
    return float(data.qpos[qadr]), float(data.qvel[dadr]), dadr


def freejoint_force(
    model: mujoco.MjModel, data: mujoco.MjData, name: str
) -> np.ndarray:
    joint_id = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    address = int(model.jnt_dofadr[joint_id])
    return np.asarray(data.qfrc_passive[address : address + 3], dtype=float).copy()


def full_mass_diagonal(model: mujoco.MjModel, data: mujoco.MjData, dof: int) -> float:
    matrix = np.empty((model.nv, model.nv), dtype=float)
    mujoco.mj_fullM(model, matrix, data.qM)
    return float(matrix[dof, dof])


def command(case: Case, elapsed_s: float) -> tuple[float, float]:
    if case.ramp_s <= 0 or case.command_final == 0:
        return 0.0, 0.0
    phase = min(max(elapsed_s / case.ramp_s, 0.0), 1.0)
    # A cubic smoothstep gives zero endpoint acceleration and a known rate.
    value = case.command_final * (3 * phase * phase - 2 * phase * phase * phase)
    rate = (
        case.command_final * 6 * phase * (1 - phase) / case.ramp_s
        if elapsed_s < case.ramp_s
        else 0.0
    )
    return value, rate


def wrap_angle_from_length(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    route_length: float,
    endpoint_a: str,
    endpoint_b: str,
    geom_name: str,
    radius: float,
) -> float:
    geom = object_id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    site_a = object_id(model, mujoco.mjtObj.mjOBJ_SITE, endpoint_a)
    site_b = object_id(model, mujoco.mjtObj.mjOBJ_SITE, endpoint_b)
    center = np.asarray(data.geom_xpos[geom], dtype=float)
    axis = np.asarray(data.geom_xmat[geom], dtype=float).reshape(3, 3)[:, 2]

    def radial_distance(point: np.ndarray) -> float:
        delta = point - center
        radial = delta - np.dot(delta, axis) * axis
        return float(np.linalg.norm(radial))

    d_a = radial_distance(np.asarray(data.site_xpos[site_a], dtype=float))
    d_b = radial_distance(np.asarray(data.site_xpos[site_b], dtype=float))
    tangent_a = math.sqrt(max(d_a * d_a - radius * radius, 0.0))
    tangent_b = math.sqrt(max(d_b * d_b - radius * radius, 0.0))
    return max(0.0, (route_length - tangent_a - tangent_b) / radius)


def right_body_contact(model: mujoco.MjModel, data: mujoco.MjData) -> bool:
    body_id = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "right_mass")
    for index in range(data.ncon):
        contact = data.contact[index]
        if model.geom_bodyid[contact.geom1] == body_id:
            return True
        if model.geom_bodyid[contact.geom2] == body_id:
            return True
    return False


def collect_row(
    case: Case,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    command_value: float,
    command_rate: float,
    step_ms: float,
) -> dict[str, float]:
    row: dict[str, float] = {
        "time_s": float(data.time),
        "command": command_value,
        "command_rate_per_s": command_rate,
        "step_ms": step_ms,
    }
    row.update(plugin_state(model, data))
    row["kinetic_energy_J"] = float(data.energy[1])
    row["potential_energy_J"] = float(data.energy[0])

    if case.kind in {"fixed", "rotating", "winch"}:
        pos = body_position(model, data, "free_payload")
        vel = sensor(model, data, "payload_velocity")
        force = freejoint_force(model, data, "payload_free")
        row.update(
            {
                "payload_x_m": pos[0],
                "payload_y_m": pos[1],
                "payload_z_m": pos[2],
                "payload_vx_mps": vel[0],
                "payload_vy_mps": vel[1],
                "payload_vz_mps": vel[2],
                "payload_endpoint_tension_N": float(np.linalg.norm(force)),
            }
        )
    if case.kind in {"fixed", "rotating"}:
        radius = numeric(model, "pulley_radius_m")
        row["wrap_angle_rad"] = wrap_angle_from_length(
            model,
            data,
            row["length_m"],
            "red_fixed_end",
            "payload_cable_end",
            "fixed_pulley_wrap",
            radius,
        )
    if case.kind == "rotating":
        theta, omega, dof = joint_values(model, data, "pulley_hinge")
        bearing = numeric(model, "bearing_damping_Nms_per_rad")
        row.update(
            {
                "pulley_angle_rad": theta,
                "pulley_omega_radps": omega,
                "pulley_effective_inertia_kgm2": full_mass_diagonal(model, data, dof),
                "pulley_net_passive_torque_Nm": float(data.qfrc_passive[dof]),
                "pulley_cable_torque_Nm": float(data.qfrc_passive[dof] + bearing * omega),
            }
        )
    elif case.kind == "winch":
        theta_w, omega_w, _ = joint_values(model, data, "winch_hinge")
        theta_p, omega_p, _ = joint_values(model, data, "pulley_hinge")
        row.update(
            {
                "winch_angle_rad": theta_w,
                "winch_omega_radps": omega_w,
                "pulley_angle_rad": theta_p,
                "pulley_omega_radps": omega_p,
            }
        )
    elif case.kind == "moving":
        moving_pos = body_position(model, data, "moving_pulley")
        load_pos = body_position(model, data, "moving_load")
        theta_u, omega_u, _ = joint_values(model, data, "upper_pulley_hinge")
        theta_m, omega_m, _ = joint_values(model, data, "moving_pulley_hinge")
        row.update(
            {
                "moving_pulley_x_m": moving_pos[0],
                "moving_pulley_z_m": moving_pos[2],
                "moving_load_x_m": load_pos[0],
                "moving_load_z_m": load_pos[2],
                "upper_pulley_angle_rad": theta_u,
                "upper_pulley_omega_radps": omega_u,
                "moving_pulley_angle_rad": theta_m,
                "moving_pulley_omega_radps": omega_m,
                "tether_length_m": float(sensor(model, data, "pulley_load_tether_length")[0]),
                "tether_velocity_mps": float(sensor(model, data, "pulley_load_tether_velocity")[0]),
            }
        )
    elif case.kind == "atwood":
        left_pos = body_position(model, data, "left_mass")
        right_pos = body_position(model, data, "right_mass")
        left_vel = sensor(model, data, "left_velocity")
        right_vel = sensor(model, data, "right_velocity")
        theta, omega, dof = joint_values(model, data, "pulley_hinge")
        left_force = freejoint_force(model, data, "left_free")
        right_force = freejoint_force(model, data, "right_free")
        right_joint = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, "right_free")
        right_dof = int(model.jnt_dofadr[right_joint])
        row.update(
            {
                "left_z_m": left_pos[2],
                "right_z_m": right_pos[2],
                "left_vz_mps": left_vel[2],
                "right_vz_mps": right_vel[2],
                "left_endpoint_tension_N": float(np.linalg.norm(left_force)),
                "right_endpoint_tension_N": float(np.linalg.norm(right_force)),
                "pulley_angle_rad": theta,
                "pulley_omega_radps": omega,
                "pulley_effective_inertia_kgm2": full_mass_diagonal(model, data, dof),
                "pulley_net_passive_torque_Nm": float(data.qfrc_passive[dof]),
                "right_contact": float(right_body_contact(model, data)),
                "right_constraint_force_z_N": float(data.qfrc_constraint[right_dof + 2]),
            }
        )
    return row


def apply_theory(case: Case, model: mujoco.MjModel, rows: list[dict[str, float]]) -> dict:
    gravity = abs(float(model.opt.gravity[2]))
    times = np.asarray([row["time_s"] for row in rows])
    commands = np.asarray([row["command"] for row in rows])
    rates = np.asarray([row["command_rate_per_s"] for row in rows])
    result: dict[str, object] = {
        "case": case.key,
        "model": case.model,
        "duration_s": case.duration_s,
        "sample_count": len(rows),
        "equations": [],
        "parameters": {},
        "metrics": {},
    }
    if case.kind in {"fixed", "rotating"}:
        mass = body_mass(model, "free_payload")
        radius = numeric(model, "pulley_radius_m")
        mu = numeric(model, "capstan_mu", 0.0)
        scale = 0.01
        z0 = rows[0]["payload_z_m"]
        omega_theory = 0.0
        previous_time = times[0]
        for row in rows:
            row["payload_lift_m"] = row["payload_z_m"] - z0
            row["theory_payload_lift_m"] = row["command"]
            row["theory_payload_tension_N"] = mass * gravity
            # For this route orientation, winding moves material opposite the
            # seed direction.  The endpoint contribution is the mean of the
            # two endpoint speeds, matching the plugin's material-speed
            # estimator.  A free pulley reduces slip as it spins up.
            surface_speed = -radius * omega_theory if case.kind == "rotating" else 0.0
            slip_speed = -1.5 * row["command_rate_per_s"] - surface_speed
            regularized_ratio = math.exp(
                -mu
                * row["wrap_angle_rad"]
                * math.tanh(slip_speed / scale)
            )
            row["theory_relative_slip_mps"] = slip_speed
            row["theory_capstan_ratio"] = regularized_ratio
            row["theory_capstan_limit_ratio"] = math.exp(mu * row["wrap_angle_rad"])
            row["measured_capstan_ratio"] = row["tension_N"] / max(
                row["payload_endpoint_tension_N"], 1e-12
            )
            row["theory_source_tension_N"] = mass * gravity * regularized_ratio
            if case.kind == "rotating":
                dt = max(row["time_s"] - previous_time, 0.0)
                inertia = row["pulley_effective_inertia_kgm2"]
                bearing = numeric(model, "bearing_damping_Nms_per_rad")
                torque = radius * (
                    row["theory_source_tension_N"] - row["theory_payload_tension_N"]
                )
                if inertia > 0 and dt > 0:
                    omega_theory += dt * (torque - bearing * omega_theory) / inertia
                row["theory_pulley_omega_abs_radps"] = abs(omega_theory)
                row["theory_capstan_torque_abs_Nm"] = abs(torque)
                row["measured_pulley_omega_abs_radps"] = abs(row["pulley_omega_radps"])
                row["measured_cable_torque_abs_Nm"] = abs(row["pulley_cable_torque_Nm"])
                previous_time = row["time_s"]
        result["equations"] = [
            "T_load = m g",
            "h = c",
            "T_source/T_load = exp(mu beta tanh(|c_dot|/v_s))",
        ]
        if case.kind == "rotating":
            result["equations"].append("I_eff omega_dot + b omega = R (T_source - T_load)")
        result["parameters"] = {
            "payload_mass_kg": mass,
            "gravity_mps2": gravity,
            "pulley_radius_m": radius,
            "capstan_mu": mu,
            "capstan_velocity_scale_mps": scale,
            "bearing_damping_Nms_per_rad": numeric(
                model, "bearing_damping_Nms_per_rad", 0.0
            ),
        }
        steady = times >= max(case.ramp_s + 0.35, 0.7 * case.duration_s)
        sim_lift = np.asarray([row["payload_lift_m"] for row in rows])
        theory_lift = commands
        sim_source = np.asarray([row["tension_N"] for row in rows])
        theory_source = np.asarray([row["theory_source_tension_N"] for row in rows])
        result["metrics"] = {
            "payload_lift_rmse_mm": 1000 * rmse(sim_lift, theory_lift),
            "source_tension_rmse_N": rmse(sim_source, theory_source),
            "steady_payload_tension_mean_N": float(
                np.mean([rows[i]["payload_endpoint_tension_N"] for i in np.flatnonzero(steady)])
            ),
            "theory_payload_weight_N": mass * gravity,
            "peak_tangent_residual": max(row["tangent_residual"] for row in rows),
            "peak_surface_residual_m": max(row["surface_residual_m"] for row in rows),
            "maximum_route_status": max(row["route_status"] for row in rows),
            "mean_wrap_angle_deg": math.degrees(
                float(np.mean([row["wrap_angle_rad"] for row in rows]))
            ),
            "peak_theory_capstan_ratio": max(
                row["theory_capstan_ratio"] for row in rows
            ),
            "peak_measured_capstan_ratio": max(
                row["measured_capstan_ratio"] for row in rows
            ),
            "peak_source_tension_N": max(row["tension_N"] for row in rows),
        }
        if case.kind == "rotating":
            result["metrics"].update(
                {
                    "peak_pulley_omega_radps": max(
                        abs(row["pulley_omega_radps"]) for row in rows
                    ),
                    "capstan_torque_balance_rmse_Nm": rmse(
                        np.asarray([row["measured_cable_torque_abs_Nm"] for row in rows]),
                        np.asarray([row["theory_capstan_torque_abs_Nm"] for row in rows]),
                    ),
                }
            )
    elif case.kind == "winch":
        radius_w = numeric(model, "winch_radius_m")
        radius_p = numeric(model, "pulley_radius_m")
        z0 = rows[0]["payload_z_m"]
        for row in rows:
            row["payload_lift_m"] = row["payload_z_m"] - z0
            row["theory_pulley_angle_rad"] = -radius_w / radius_p * row["winch_angle_rad"]
            row["theory_takeup_m"] = radius_w * row["winch_angle_rad"]
            row["winch_rim_travel_m"] = radius_w * row["winch_angle_rad"]
            row["pulley_rim_travel_m"] = -radius_p * row["pulley_angle_rad"]
            row["theory_payload_lift_m"] = row["theory_takeup_m"]
        result["equations"] = [
            "s = r_w theta_w = -r_p theta_p",
            "theta_p/theta_w = -r_w/r_p",
            "h approximately equals s for an inextensible cable",
        ]
        result["parameters"] = {
            "winch_radius_m": radius_w,
            "pulley_radius_m": radius_p,
            "ideal_angle_ratio": -radius_w / radius_p,
            "payload_mass_kg": body_mass(model, "free_payload"),
        }
        result["metrics"] = {
            "angle_constraint_rmse_rad": rmse(
                np.asarray([row["pulley_angle_rad"] for row in rows]),
                np.asarray([row["theory_pulley_angle_rad"] for row in rows]),
            ),
            "rim_travel_rmse_mm": 1000
            * rmse(
                np.asarray([row["winch_rim_travel_m"] for row in rows]),
                np.asarray([row["pulley_rim_travel_m"] for row in rows]),
            ),
            "payload_lift_rmse_mm": 1000
            * rmse(
                np.asarray([row["payload_lift_m"] for row in rows]),
                np.asarray([row["theory_payload_lift_m"] for row in rows]),
            ),
            "maximum_route_status": max(row["route_status"] for row in rows),
            "final_payload_lift_mm": 1000 * rows[-1]["payload_lift_m"],
            "final_theory_takeup_mm": 1000 * rows[-1]["theory_takeup_m"],
        }
    elif case.kind == "moving":
        radius = numeric(model, "pulley_radius_m")
        total_mass = body_mass(model, "moving_pulley") + body_mass(model, "moving_load")
        tension_theory = total_mass * gravity / 2
        z0_p = rows[0]["moving_pulley_z_m"]
        z0_l = rows[0]["moving_load_z_m"]
        tether0 = rows[0]["tether_length_m"]
        for row in rows:
            row["moving_pulley_lift_m"] = row["moving_pulley_z_m"] - z0_p
            row["moving_load_lift_m"] = row["moving_load_z_m"] - z0_l
            row["theory_lift_m"] = 0.5 * row["command"]
            row["theory_tension_N"] = tension_theory
            row["theory_upper_angle_abs_rad"] = row["command"] / radius
            row["theory_moving_angle_abs_rad"] = row["command"] / (2 * radius)
            row["upper_angle_abs_rad"] = abs(row["upper_pulley_angle_rad"])
            row["moving_angle_abs_rad"] = abs(row["moving_pulley_angle_rad"])
            row["tether_drift_m"] = row["tether_length_m"] - tether0
        result["equations"] = [
            "c = 2 h",
            "2 T = (m_p + m_l) g",
            "|theta_fixed/theta_moving| = 2 only under ideal no-slip transport",
        ]
        result["parameters"] = {
            "pulley_radius_m": radius,
            "moving_pulley_mass_kg": body_mass(model, "moving_pulley"),
            "payload_mass_kg": body_mass(model, "moving_load"),
            "total_suspended_mass_kg": total_mass,
            "ideal_tension_N": tension_theory,
            "capstan_mu": numeric(model, "capstan_mu"),
        }
        ramp_index = int(np.argmin(np.abs(times - case.ramp_s)))
        moving_angle = abs(rows[ramp_index]["moving_pulley_angle_rad"])
        result["metrics"] = {
            "moving_pulley_lift_rmse_mm": 1000
            * rmse(
                np.asarray([row["moving_pulley_lift_m"] for row in rows]),
                0.5 * commands,
            ),
            "moving_load_lift_rmse_mm": 1000
            * rmse(
                np.asarray([row["moving_load_lift_m"] for row in rows]),
                0.5 * commands,
            ),
            "tension_rmse_N": rmse(
                np.asarray([row["tension_N"] for row in rows]),
                np.full(len(rows), tension_theory),
            ),
            "maximum_tether_drift_um": 1e6
            * max(abs(row["tether_drift_m"]) for row in rows),
            "ramp_end_fixed_to_moving_angle_ratio": (
                abs(rows[ramp_index]["upper_pulley_angle_rad"]) / moving_angle
                if moving_angle > 1e-9
                else math.nan
            ),
            "maximum_route_status": max(row["route_status"] for row in rows),
            "final_moving_pulley_lift_mm": 1000
            * rows[-1]["moving_pulley_lift_m"],
            "final_load_lift_mm": 1000 * rows[-1]["moving_load_lift_m"],
            "final_theory_lift_mm": 1000 * rows[-1]["theory_lift_m"],
        }
    elif case.kind == "atwood":
        m_left = body_mass(model, "left_mass")
        m_right = body_mass(model, "right_mass")
        radius = numeric(model, "pulley_radius_m")
        bearing = numeric(model, "bearing_damping_Nms_per_rad")
        inertia = rows[0]["pulley_effective_inertia_kgm2"]
        effective_mass = m_left + m_right + inertia / (radius * radius)
        gamma = bearing / (radius * radius)
        drive = (m_right - m_left) * gravity
        if gamma > 0:
            tau = effective_mass / gamma
            terminal_speed = drive / gamma
        else:
            tau = math.inf
            terminal_speed = math.inf
        right0 = rows[0]["right_z_m"]
        left0 = rows[0]["left_z_m"]
        right_half_size = numeric(model, "right_box_half_size_m")
        floor_height = 0.08
        travel_to_floor = right0 - right_half_size - floor_height

        def ideal_state(t: float) -> tuple[float, float, float]:
            if gamma > 0:
                velocity = terminal_speed * (1 - math.exp(-t / tau))
                displacement = terminal_speed * (
                    t - tau * (1 - math.exp(-t / tau))
                )
                acceleration = drive / effective_mass * math.exp(-t / tau)
            else:
                acceleration = drive / effective_mass
                velocity = acceleration * t
                displacement = 0.5 * acceleration * t * t
            return displacement, velocity, acceleration

        theory_landing = solve_time_for_displacement(ideal_state, travel_to_floor, case.duration_s)
        for row in rows:
            valid = row["time_s"] <= theory_landing
            displacement, velocity, acceleration = ideal_state(min(row["time_s"], theory_landing))
            row["theory_valid_before_landing"] = float(valid)
            row["theory_left_z_m"] = left0 + displacement if valid else math.nan
            row["theory_right_z_m"] = right0 - displacement if valid else math.nan
            row["theory_speed_mps"] = velocity if valid else math.nan
            row["theory_pulley_omega_abs_radps"] = velocity / radius if valid else math.nan
            row["theory_left_tension_N"] = (
                m_left * (gravity + acceleration) if valid else math.nan
            )
            row["theory_right_tension_N"] = (
                m_right * (gravity - acceleration) if valid else math.nan
            )
            row["left_speed_up_mps"] = row["left_vz_mps"]
            row["right_speed_down_mps"] = -row["right_vz_mps"]
            row["pulley_omega_abs_radps"] = abs(row["pulley_omega_radps"])
        contact_rows = [row for row in rows if row["right_contact"] > 0.5]
        measured_landing = contact_rows[0]["time_s"] if contact_rows else math.nan
        pre = np.asarray([row["time_s"] < min(theory_landing, measured_landing) for row in rows])
        if not np.any(pre):
            pre = np.asarray([row["time_s"] < theory_landing for row in rows])
        result["equations"] = [
            "M_eff = m_L + m_R + I_eff/R^2",
            "M_eff v_dot + (b/R^2) v = (m_R-m_L) g",
            "T_L = m_L (g + v_dot), T_R = m_R (g - v_dot)",
            "omega = v/R before landing under ideal no-slip transport",
        ]
        result["parameters"] = {
            "left_mass_kg": m_left,
            "right_mass_kg": m_right,
            "pulley_radius_m": radius,
            "effective_pulley_inertia_kgm2": inertia,
            "bearing_damping_Nms_per_rad": bearing,
            "effective_translational_mass_kg": effective_mass,
            "travel_to_floor_m": travel_to_floor,
        }
        theory_speed = np.asarray([row["theory_speed_mps"] for row in rows])
        sim_speed = np.asarray([row["right_speed_down_mps"] for row in rows])
        result["metrics"] = {
            "theory_landing_time_s": theory_landing,
            "measured_landing_time_s": measured_landing,
            "prelanding_right_speed_rmse_mps": rmse(sim_speed[pre], theory_speed[pre]),
            "prelanding_pulley_omega_rmse_radps": rmse(
                np.asarray([row["pulley_omega_abs_radps"] for row in rows])[pre],
                np.asarray([row["theory_pulley_omega_abs_radps"] for row in rows])[pre],
            ),
            "peak_postlanding_contact_force_N": max(
                [abs(row["right_constraint_force_z_N"]) for row in contact_rows] or [0.0]
            ),
            "maximum_route_status": max(row["route_status"] for row in rows),
            "landing_time_relative_error_percent": 100
            * (measured_landing - theory_landing)
            / theory_landing,
        }
    result["metrics"]["p95_step_time_ms"] = float(
        np.percentile([row["step_ms"] for row in rows[1:]], 95)
    )
    result["metrics"]["taut_fraction"] = float(np.mean([row["taut"] for row in rows]))
    result["metrics"]["saturation_fraction"] = float(
        np.mean([row["saturated"] for row in rows])
    )
    return result


def solve_time_for_displacement(function, target: float, upper: float) -> float:
    lo, hi = 0.0, upper
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if function(mid)[0] < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def rmse(a: np.ndarray, b: np.ndarray) -> float:
    mask = np.isfinite(a) & np.isfinite(b)
    if not np.any(mask):
        return math.nan
    return float(np.sqrt(np.mean(np.square(a[mask] - b[mask]))))


def write_csv(path: Path, rows: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: list[str] = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def style_axis(axis: plt.Axes) -> None:
    axis.grid(True, color="#D1D5D8", linewidth=0.5)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)
    axis.tick_params(direction="out", length=3, width=0.7)


def export_figure(figure: plt.Figure, stem: Path) -> None:
    stem.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(stem.with_suffix(".pdf"), bbox_inches="tight")
    figure.savefig(stem.with_suffix(".svg"), bbox_inches="tight")
    figure.savefig(stem.with_suffix(".png"), dpi=600, bbox_inches="tight")
    plt.close(figure)


def plot_case(case: Case, rows: list[dict[str, float]], output: Path) -> None:
    t = np.asarray([row["time_s"] for row in rows])
    markevery = max(1, len(rows) // 12)
    figure, axes = plt.subplots(1, 3, figsize=(7.16, 2.55), constrained_layout=True)
    if case.kind == "fixed":
        axes[0].plot(t, [row["theory_source_tension_N"] for row in rows], color=THEORY_COLOR, label="Theory")
        axes[0].plot(t, [row["tension_N"] for row in rows], color=SIM_COLORS[0], linestyle="--", marker="o", markevery=markevery, zorder=3, label="Simulation")
        axes[0].set_ylabel("Source tension (N)")
        axes[1].plot(t, 1000 * np.asarray([row["theory_payload_lift_m"] for row in rows]), color=THEORY_COLOR, label="Theory")
        axes[1].plot(t, 1000 * np.asarray([row["payload_lift_m"] for row in rows]), color=SIM_COLORS[1], linestyle="--", marker="s", markevery=markevery, zorder=3, label="Simulation")
        axes[1].set_ylabel("Payload lift (mm)")
        axes[2].plot(t, [row["theory_capstan_ratio"] for row in rows], color=THEORY_COLOR, label="Theory")
        axes[2].plot(t, [row["measured_capstan_ratio"] for row in rows], color=SIM_COLORS[2], linestyle="--", marker="^", markevery=markevery, zorder=3, label="Simulation")
        axes[2].set_ylabel("Tension ratio")
    elif case.kind == "rotating":
        axes[0].plot(t, [row["theory_source_tension_N"] for row in rows], color=THEORY_COLOR, label="Theory")
        axes[0].plot(t, [row["tension_N"] for row in rows], color=SIM_COLORS[0], linestyle="--", marker="o", markevery=markevery, zorder=3, label="Simulation")
        axes[0].set_ylabel("Source tension (N)")
        axes[1].plot(t, [row["theory_capstan_torque_abs_Nm"] for row in rows], color=THEORY_COLOR, label="Theory")
        axes[1].plot(t, [row["measured_cable_torque_abs_Nm"] for row in rows], color=SIM_COLORS[1], linestyle="--", marker="s", markevery=markevery, zorder=3, label="Simulation")
        axes[1].set_ylabel("Pulley torque magnitude (N m)")
        axes[2].plot(t, [row["theory_pulley_omega_abs_radps"] for row in rows], color=THEORY_COLOR, label="Theory")
        axes[2].plot(t, [row["measured_pulley_omega_abs_radps"] for row in rows], color=SIM_COLORS[2], linestyle="--", marker="^", markevery=markevery, zorder=3, label="Simulation")
        axes[2].set_ylabel("Pulley speed magnitude (rad/s)")
    elif case.kind == "winch":
        axes[0].plot(t, [row["theory_pulley_angle_rad"] for row in rows], color=THEORY_COLOR, label="Theory")
        axes[0].plot(t, [row["pulley_angle_rad"] for row in rows], color=SIM_COLORS[0], linestyle="--", marker="o", markevery=markevery, zorder=3, label="Simulation")
        axes[0].set_ylabel("Pulley angle (rad)")
        axes[1].plot(t, 1000 * np.asarray([row["winch_rim_travel_m"] for row in rows]), color=THEORY_COLOR, label="Winch theory")
        axes[1].plot(t, 1000 * np.asarray([row["pulley_rim_travel_m"] for row in rows]), color=SIM_COLORS[1], linestyle="--", marker="s", markevery=markevery, zorder=3, label="Pulley simulation")
        axes[1].set_ylabel("Rim travel (mm)")
        axes[2].plot(t, 1000 * np.asarray([row["theory_payload_lift_m"] for row in rows]), color=THEORY_COLOR, label="Theory")
        axes[2].plot(t, 1000 * np.asarray([row["payload_lift_m"] for row in rows]), color=SIM_COLORS[2], linestyle="--", marker="^", markevery=markevery, zorder=3, label="Simulation")
        axes[2].set_ylabel("Payload lift (mm)")
    elif case.kind == "moving":
        axes[0].plot(t, 1000 * np.asarray([row["theory_lift_m"] for row in rows]), color=THEORY_COLOR, label="Theory")
        axes[0].plot(t, 1000 * np.asarray([row["moving_pulley_lift_m"] for row in rows]), color=SIM_COLORS[0], linestyle="--", marker="o", markevery=markevery, zorder=3, label="Moving pulley")
        axes[0].plot(t, 1000 * np.asarray([row["moving_load_lift_m"] for row in rows]), color=SIM_COLORS[1], linestyle="--", marker="s", markevery=markevery, zorder=4, label="Load")
        axes[0].set_ylabel("Lift (mm)")
        axes[1].plot(t, [row["theory_tension_N"] for row in rows], color=THEORY_COLOR, label="Theory")
        axes[1].plot(t, [row["tension_N"] for row in rows], color=SIM_COLORS[1], linestyle="--", marker="s", markevery=markevery, zorder=3, label="Simulation")
        axes[1].set_ylabel("Cable tension (N)")
        axes[2].plot(t, [row["theory_upper_angle_abs_rad"] for row in rows], color=THEORY_COLOR, label="Fixed theory")
        axes[2].plot(t, [row["upper_angle_abs_rad"] for row in rows], color=SIM_COLORS[0], linestyle="--", marker="o", markevery=markevery, zorder=3, label="Fixed simulation")
        axes[2].plot(t, [row["theory_moving_angle_abs_rad"] for row in rows], color="#999999", linestyle="-", label="Moving theory")
        axes[2].plot(t, [row["moving_angle_abs_rad"] for row in rows], color=SIM_COLORS[2], linestyle="--", marker="^", markevery=markevery, zorder=4, label="Moving simulation")
        axes[2].set_ylabel("Pulley angle magnitude (rad)")
    elif case.kind == "atwood":
        axes[0].plot(t, [row["theory_speed_mps"] for row in rows], color=THEORY_COLOR, label="Theory")
        axes[0].plot(t, [row["right_speed_down_mps"] for row in rows], color=SIM_COLORS[0], linestyle="--", marker="o", markevery=markevery, zorder=3, label="Right mass")
        axes[0].plot(t, [row["left_speed_up_mps"] for row in rows], color=SIM_COLORS[1], linestyle="--", marker="s", markevery=markevery, zorder=4, label="Left mass")
        axes[0].set_ylabel("Signed travel speed (m/s)")
        axes[1].plot(t, [row["theory_pulley_omega_abs_radps"] for row in rows], color=THEORY_COLOR, label="Theory")
        axes[1].plot(t, [row["pulley_omega_abs_radps"] for row in rows], color=SIM_COLORS[2], linestyle="--", marker="^", markevery=markevery, zorder=3, label="Simulation")
        axes[1].set_ylabel("Pulley speed magnitude (rad/s)")
        axes[2].plot(t, [row["theory_left_tension_N"] for row in rows], color=THEORY_COLOR, label="Left theory")
        axes[2].plot(t, [row["left_endpoint_tension_N"] for row in rows], color=SIM_COLORS[0], linestyle="--", marker="o", markevery=markevery, zorder=3, label="Left simulation")
        axes[2].plot(t, [row["theory_right_tension_N"] for row in rows], color="#999999", linestyle="-", label="Right theory")
        axes[2].plot(t, [row["right_endpoint_tension_N"] for row in rows], color=SIM_COLORS[1], linestyle="--", marker="s", markevery=markevery, zorder=4, label="Right simulation")
        axes[2].set_ylabel("Endpoint tension (N)")
    for index, axis in enumerate(axes):
        axis.set_xlabel("Time (s)")
        style_axis(axis)
        axis.legend(frameon=False, loc="best")
        axis.text(-0.18, 1.04, f"({chr(ord('a') + index)})", transform=axis.transAxes, fontweight="bold")
    export_figure(figure, output / "figures" / f"{case.key}_theory_vs_simulation")


def plot_overview(all_rows: dict[str, list[dict[str, float]]]) -> None:
    figure, axes = plt.subplots(2, 3, figsize=(7.16, 4.75), constrained_layout=True)
    panels = axes.ravel()

    def values(key: str, field: str) -> np.ndarray:
        return np.asarray([row[field] for row in all_rows[key]], dtype=float)

    def timeline(key: str) -> np.ndarray:
        return values(key, "time_s")

    def marker_interval(key: str) -> int:
        return max(1, len(all_rows[key]) // 10)

    key = "result_1_1_mu_0"
    panels[0].plot(timeline(key), 1000 * values(key, "theory_payload_lift_m"), color=THEORY_COLOR, label="Theory")
    panels[0].plot(timeline(key), 1000 * values(key, "payload_lift_m"), color=SIM_COLORS[0], linestyle="--", marker="o", markevery=marker_interval(key), zorder=3, label="Simulation")
    panels[0].set_ylabel("Payload lift (mm)")
    panels[0].set_title("Fixed pulley, $\\mu=0$")

    key = "result_1_1_mu_0p15"
    panels[1].plot(timeline(key), values(key, "theory_capstan_ratio"), color=THEORY_COLOR, label="Theory")
    panels[1].plot(timeline(key), values(key, "measured_capstan_ratio"), color=SIM_COLORS[1], linestyle="--", marker="s", markevery=marker_interval(key), zorder=3, label="Simulation")
    panels[1].set_ylabel("Tension ratio")
    panels[1].set_title("Fixed pulley, $\\mu=0.15$")

    key = "result_1_1_b_mu_0p15"
    panels[2].plot(timeline(key), values(key, "theory_pulley_omega_abs_radps"), color=THEORY_COLOR, label="Theory")
    panels[2].plot(timeline(key), values(key, "measured_pulley_omega_abs_radps"), color=SIM_COLORS[2], linestyle="--", marker="^", markevery=marker_interval(key), zorder=3, label="Simulation")
    panels[2].set_ylabel("Pulley speed (rad/s)")
    panels[2].set_title("Passive inertial pulley")

    key = "result_1_1_c_winch"
    panels[3].plot(timeline(key), 1000 * values(key, "winch_rim_travel_m"), color=THEORY_COLOR, label="Winch theory")
    panels[3].plot(timeline(key), 1000 * values(key, "pulley_rim_travel_m"), color=SIM_COLORS[0], linestyle="--", marker="o", markevery=marker_interval(key), zorder=3, label="Pulley simulation")
    panels[3].set_ylabel("Rim travel (mm)")
    panels[3].set_title("25/50 mm radius ratio")

    key = "result_1_2_moving_pulley"
    panels[4].plot(timeline(key), 1000 * values(key, "theory_lift_m"), color=THEORY_COLOR, label="Theory")
    panels[4].plot(timeline(key), 1000 * values(key, "moving_pulley_lift_m"), color=SIM_COLORS[0], linestyle="--", marker="o", markevery=marker_interval(key), zorder=3, label="Pulley")
    panels[4].plot(timeline(key), 1000 * values(key, "moving_load_lift_m"), color=SIM_COLORS[1], linestyle="--", marker="s", markevery=marker_interval(key), zorder=4, label="Load")
    panels[4].set_ylabel("Lift (mm)")
    panels[4].set_title("Moving pulley, $n=2$")

    key = "result_1_3_atwood_landing"
    panels[5].plot(timeline(key), values(key, "theory_speed_mps"), color=THEORY_COLOR, label="Theory")
    panels[5].plot(timeline(key), values(key, "right_speed_down_mps"), color=SIM_COLORS[0], linestyle="--", marker="o", markevery=marker_interval(key), zorder=3, label="Right mass")
    panels[5].plot(timeline(key), values(key, "left_speed_up_mps"), color=SIM_COLORS[1], linestyle="--", marker="s", markevery=marker_interval(key), zorder=4, label="Left mass")
    contact_times = [row["time_s"] for row in all_rows[key] if row["right_contact"] > 0.5]
    if contact_times:
        panels[5].axvline(contact_times[0], color="#888888", linestyle=":", linewidth=1.0, label="First contact")
    panels[5].set_ylabel("Travel speed (m/s)")
    panels[5].set_title("Atwood pulley and landing")

    for index, axis in enumerate(panels):
        axis.set_xlabel("Time (s)")
        style_axis(axis)
        axis.legend(frameon=False, loc="best", fontsize=8)
        axis.text(-0.18, 1.08, f"({chr(ord('a') + index)})", transform=axis.transAxes, fontweight="bold")
    export_figure(figure, HERE / "figures" / "figure3_result1_pulley_validation")


def font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    candidates = []
    for base in FONT_PATHS:
        if bold:
            candidates.append(base.with_name("Times New Roman Bold.ttf"))
        candidates.append(base)
    for candidate in candidates:
        if candidate.is_file():
            return ImageFont.truetype(str(candidate), size=size)
    return ImageFont.load_default()


def annotate(frame: np.ndarray, case: Case, row: dict[str, float]) -> Image.Image:
    image = Image.fromarray(frame).convert("RGBA")
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    top = image.height - 128
    draw.rounded_rectangle((18, top, 500, image.height - 18), radius=4, fill=(7, 11, 15, 190), outline=(120, 130, 138, 210))
    draw.text((32, top + 11), case.title, fill=(248, 248, 248, 255), font=font(22, True))
    unit = "rad" if case.kind == "winch" else "m"
    draw.text((32, top + 43), f"t = {row['time_s']:.2f} s   command = {row['command']:.4f} {unit}", fill=(225, 230, 234, 255), font=font(17))
    draw.text((32, top + 71), f"L = {1000*row['length_m']:.1f} mm   T = {row['tension_N']:.2f} N   route = {int(round(row['route_status']))}", fill=(225, 230, 234, 255), font=font(17))
    draw.text((image.width - 150, 22), "MuJoCable", fill=(226, 231, 234, 235), font=font(17, True))
    return Image.alpha_composite(image, overlay).convert("RGB")


def encode_video(frames: Path, output: Path, fps: int) -> None:
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise RuntimeError("ffmpeg is required to encode Result 1 videos")
    output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            ffmpeg,
            "-y",
            "-loglevel",
            "error",
            "-framerate",
            str(fps),
            "-i",
            str(frames / "frame_%05d.png"),
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "22",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(output),
        ],
        check=True,
    )


def run_case(
    case: Case,
    fps: int,
    width: int,
    height: int,
    make_video: bool,
) -> tuple[list[dict[str, float]], dict]:
    model_path = HERE / case.model
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), width)
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), height)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    actuator = 0 if model.nu else -1
    sample_period = 0.002
    next_sample = 0.0
    next_frame = 0.0
    rows: list[dict[str, float]] = []
    result_dir = HERE / case.result_id
    raw_dir = Path(tempfile.mkdtemp(prefix=f"{case.key}_raw_")) if make_video else None
    annotated_dir = Path(tempfile.mkdtemp(prefix=f"{case.key}_ann_")) if make_video else None
    renderer = mujoco.Renderer(model, height=height, width=width, max_geom=20000) if make_video else None
    frame_index = 0
    step_ms = 0.0
    try:
        while data.time <= case.duration_s + 1e-12:
            value, rate = command(case, float(data.time))
            if actuator >= 0:
                data.ctrl[actuator] = value
            if data.time + 1e-12 >= next_sample:
                mujoco.mj_forward(model, data)
                rows.append(collect_row(case, model, data, value, rate, step_ms))
                next_sample += sample_period
            if renderer is not None and data.time + 1e-12 >= next_frame:
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera="overview")
                pixels = renderer.render()
                Image.fromarray(pixels).save(raw_dir / f"frame_{frame_index:05d}.png")
                row = collect_row(case, model, data, value, rate, step_ms)
                annotate(pixels, case, row).save(annotated_dir / f"frame_{frame_index:05d}.png")
                frame_index += 1
                next_frame += 1.0 / fps
            start = time.perf_counter()
            mujoco.mj_step(model, data)
            step_ms = 1000.0 * (time.perf_counter() - start)
    finally:
        if renderer is not None:
            renderer.close()
    summary = apply_theory(case, model, rows)
    data_dir = result_dir / "data"
    write_csv(data_dir / f"{case.key}_timeseries.csv", rows)
    (data_dir / f"{case.key}_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    plot_case(case, rows, result_dir)
    if make_video and raw_dir is not None and annotated_dir is not None:
        videos = result_dir / "videos"
        encode_video(raw_dir, videos / f"{case.key}_raw.mp4", fps)
        encode_video(annotated_dir, videos / f"{case.key}_annotated.mp4", fps)
        poster = sorted(annotated_dir.glob("*.png"))[max(0, frame_index // 2)]
        shutil.copy2(poster, videos / f"{case.key}.png")
        shutil.rmtree(raw_dir)
        shutil.rmtree(annotated_dir)
    return rows, summary


def write_summary(summaries: list[dict]) -> None:
    rows = []
    for summary in summaries:
        base = {"case": summary["case"], "model": summary["model"]}
        base.update(summary["metrics"])
        rows.append(base)
    fields = ["case", "model"]
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    with (HERE / "result_1_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    (HERE / "result_1_theory_and_statistics.json").write_text(
        json.dumps(summaries, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    write_report(summaries)


def write_report(summaries: list[dict]) -> None:
    values = {summary["case"]: summary for summary in summaries}
    f0 = values["result_1_1_mu_0"]
    f15 = values["result_1_1_mu_0p15"]
    r0 = values["result_1_1_b_mu_0"]
    r15 = values["result_1_1_b_mu_0p15"]
    winch = values["result_1_1_c_winch"]
    moving = values["result_1_2_moving_pulley"]
    atwood = values["result_1_3_atwood_landing"]
    mp = moving["parameters"]
    ap = atwood["parameters"]
    am = atwood["metrics"]
    a0 = (
        (ap["right_mass_kg"] - ap["left_mass_kg"]) * 9.81
        / ap["effective_translational_mass_kg"]
    )
    t_left0 = ap["left_mass_kg"] * (9.81 + a0)
    t_right0 = ap["right_mass_kg"] * (9.81 - a0)
    text = rf"""# Result 1：滑轮与绳索动力学的理论—仿真验证

## 1. 验证边界

Result 1 使用已经人工确认的七个冻结 MJCF 工况，不再调用历史 Demo 15、21 或
29。验证对象是 MuJoCable 的质量为零、只受拉、可伸长绳索，以及圆柱表面包络、
速度正则化 Capstan 摩擦、自由滑轮惯量、卷轴和落地碰撞。它不是有质量绳、绳弯曲
刚度或严格静摩擦粘着模型。

| 工况 | 主要变量 | 理论基线 | 输出目录 |
|---|---|---|---|
| `result_1_1_mu_0` | 固定滑轮，`mu=0` | 等张力、1:1 位移 | `result_1_1/` |
| `result_1_1_mu_0p15` | 固定滑轮，`mu=0.15` | Euler–Eytelwein | `result_1_1/` |
| `result_1_1_b_mu_0` | 自由滑轮，`mu=0` | 零切向驱动转矩 | `result_1_1_b/` |
| `result_1_1_b_mu_0p15` | 自由滑轮，`mu=0.15` | Capstan 张力差与转动方程 | `result_1_1_b/` |
| `result_1_1_c_winch` | 25 mm 卷轴和 50 mm 滑轮 | 圆周位移守恒 | `result_1_1_c/` |
| `result_1_2_moving_pulley` | 单动滑轮，承载段数 `n=2` | `c=2h`、`2T=G` | `result_1_2/` |
| `result_1_3_atwood_landing` | 不等质量、惯性滑轮、落地 | 带阻尼 Atwood 方程 | `result_1_3/` |

每个工况均保存逐时刻 CSV、参数和统计 JSON、PDF/SVG/600 dpi PNG 曲线，以及
960 x 540、20 fps 的 raw/annotated 视频。曲线中理论为灰色实线，仿真为彩色虚线
并带标记；仿真层的 `zorder` 高于理论层。

## 2. 公共绳索模型和可测量量

实时包络路线给出长度 `L(q)` 和速度 `L_dot`。自由长度、有效伸长和单边张力为

$$
L_f=L_0-c-c_0,\qquad
e=L-L_f-s,\qquad
T=\max(0,k e+d\dot L),
$$

其中 `c` 为控制收缩量，`c_0` 为预张紧偏置，`s` 为松弛阈值。该 `max`
保证绳索不会产生压力。包络节点将相邻绳段的矢量合力通过 `mj_applyFT` 写入
`qfrc_passive`；插件传感器报告 12 个量：`L`、`L_dot`、`L_f`、`c`、`e`、
`T`、taut、saturated、route status、切向残差、表面残差和迭代次数。

速度方向 Capstan 模型为

$$
\frac{{T_{{i+1}}}}{{T_i}}
=\exp\left[\mu\beta_i\tanh\left(\frac{{v_{{\mathrm{{rel}},i}}}}{{v_s}}\right)\right].
$$

`v_s=0.01 m/s` 是固定/自由单滑轮工况的正则化尺度。它使零相对滑速时
张力比连续回到 1，而 `exp(mu beta)` 只是充分滑移时的上界。CSV 中
`tension_N` 是 seed 起点侧张力；自由负载平动自由度上的 `qfrc_passive` 模长
用于重建负载侧端点张力。

## 3. Result 1.1：固定滑轮

载荷 `m=0.2 kg`，所以无摩擦准静态理论为

$$T=mg=0.2\times9.81={f0['metrics']['theory_payload_weight_N']:.3f}\ \mathrm{{N}},
\qquad h=c.$$

解析包角均值为 `{f15['metrics']['mean_wrap_angle_deg']:.3f} deg`，即约 `2.094 rad`。
当 `mu=0.15` 时，充分滑移上界为

$$\exp(\mu\beta)=\exp(0.15\times2.094)\approx1.369.$$

`mu=0` 时，25 mm 收绳的位移 RMSE 为
`{f0['metrics']['payload_lift_rmse_mm']:.3f} mm`，源端张力 RMSE 为
`{f0['metrics']['source_tension_rmse_N']:.4f} N`，稳态负载端张力为
`{f0['metrics']['steady_payload_tension_mean_N']:.6f} N`。`mu=0.15` 时，速度
正则化公式与仿真源端张力的 RMSE 为
`{f15['metrics']['source_tension_rmse_N']:.4f} N`；理论/仿真峰值张力比分别为
`{f15['metrics']['peak_theory_capstan_ratio']:.4f}` 和
`{f15['metrics']['peak_measured_capstan_ratio']:.4f}`。这说明摩擦改变收绳过程中的
瞬态源端张力，但停止收绳且滑轮固定后，正则化模型回到等张力稳态。

[无摩擦曲线](result_1_1/figures/result_1_1_mu_0_theory_vs_simulation.pdf)；
[mu=0.15 曲线](result_1_1/figures/result_1_1_mu_0p15_theory_vs_simulation.pdf)。

## 4. Result 1.1b：带惯量的自由滑轮

自由滑轮满足

$$I_{{eff}}\dot\omega+b\omega=R(T_{{source}}-T_{{load}}),$$

其中 `R=0.05 m`、`b=0.005 N m s/rad`。`mu=0` 时两侧张力相等，峰值角速度仅
`{r0['metrics']['peak_pulley_omega_radps']:.3e} rad/s`，说明轴不会无输入自转。
`mu=0.15` 时，滑轮由张力差被动带动，峰值角速度为
`{r15['metrics']['peak_pulley_omega_radps']:.3f} rad/s`；Capstan 预测与从
`qfrc_passive+b omega` 恢复的绳索转矩 RMSE 为
`{r15['metrics']['capstan_torque_balance_rmse_Nm']:.3e} N m`。自由滑轮转动后会
降低绳—轮相对滑速，因此其峰值张力比显著小于固定滑轮，符合速度正则化模型。

[自由滑轮 mu=0.15 曲线](result_1_1_b/figures/result_1_1_b_mu_0p15_theory_vs_simulation.pdf)。

## 5. Result 1.1c：卷轴—滑轮半径传动

卷轴半径 `r_w=0.025 m`、滑轮半径 `r_p=0.050 m`。理想无滑移关系为

$$s=r_w\theta_w=-r_p\theta_p,
\qquad \frac{{\theta_p}}{{\theta_w}}=-\frac{{r_w}}{{r_p}}=-0.5.$$

该工况只有 `winch_motor` 一个控制，joint equality 只用于构造理想半径比基线。
角度约束 RMSE 为 `{winch['metrics']['angle_constraint_rmse_rad']:.3e} rad`，两轮
圆周位移 RMSE 为 `{winch['metrics']['rim_travel_rmse_mm']:.6f} mm`。最终理论卷绕
`{winch['metrics']['final_theory_takeup_mm']:.3f} mm`，负载实际上升
`{winch['metrics']['final_payload_lift_mm']:.3f} mm`，全程位移 RMSE 为
`{winch['metrics']['payload_lift_rmse_mm']:.3f} mm`。

[卷轴曲线](result_1_1_c/figures/result_1_1_c_winch_theory_vs_simulation.pdf)。

## 6. Result 1.2：单动滑轮

动滑轮和负载总质量为
`m_p+m_l={mp['moving_pulley_mass_kg']:.2f}+{mp['payload_mass_kg']:.2f}={mp['total_suspended_mass_kg']:.2f} kg`。
理想准静态关系为

$$c=2h,\qquad
T=\frac{{(m_p+m_l)g}}2
=\frac{{0.28\times9.81}}2={mp['ideal_tension_N']:.4f}\ \mathrm{{N}}.$$

20 mm 收绳对应理论上升 10 mm；仿真最终动滑轮和负载分别上升
`{moving['metrics']['final_moving_pulley_lift_mm']:.3f} mm` 和
`{moving['metrics']['final_load_lift_mm']:.3f} mm`，位移 RMSE 分别为
`{moving['metrics']['moving_pulley_lift_rmse_mm']:.3f} mm`、
`{moving['metrics']['moving_load_lift_rmse_mm']:.3f} mm`。黄色定长连接索最大漂移
`{moving['metrics']['maximum_tether_drift_um']:.3f} um`。

斜坡末端的两滑轮角度比为
`{moving['metrics']['ramp_end_fixed_to_moving_angle_ratio']:.3f}`，不是理想无滑移的
2。这个差异不能用于否定 `c=2h`：当前 `target_contraction` 缩短整根自由长度，
并未定义材料从某个自由端进入的速度，也没有施加 `v_cable=R omega` 的静摩擦
粘着约束。该工况可验证承载绳段、位移和张力趋势，但滑轮角度比只是滑移诊断量。

[动滑轮曲线](result_1_2/figures/result_1_2_moving_pulley_theory_vs_simulation.pdf)。

## 7. Result 1.3：带阻尼惯性滑轮和落地事件

模型实际质量为 `m_L={ap['left_mass_kg']:.7f} kg`、
`m_R={ap['right_mass_kg']:.7f} kg`。含 armature 的有效转动惯量为
`I_eff={ap['effective_pulley_inertia_kgm2']:.7f} kg m^2`，因此

$$M_{{eff}}=m_L+m_R+\frac{{I_{{eff}}}}{{R^2}}
={ap['effective_translational_mass_kg']:.7f}\ \mathrm{{kg}},$$

$$M_{{eff}}\dot v+\frac b{{R^2}}v=(m_R-m_L)g.$$

初始理想加速度为 `{a0:.4f} m/s^2`，相应两侧张力为
`T_L={t_left0:.4f} N`、`T_R={t_right0:.4f} N`。右块底面距地面
`{1000*ap['travel_to_floor_m']:.1f} mm`；带轴承阻尼的理论落地时间为
`{am['theory_landing_time_s']:.4f} s`，仿真首次接触为
`{am['measured_landing_time_s']:.4f} s`，相对差
`{am['landing_time_relative_error_percent']:.2f}%`。落地前右块速度 RMSE 为
`{am['prelanding_right_speed_rmse_mps']:.4f} m/s`，滑轮角速度 RMSE 为
`{am['prelanding_pulley_omega_rmse_radps']:.4f} rad/s`。

理论曲线在落地时终止；之后是接触冲击、绳索弹性振荡和单边松弛的分段动力学，
不能继续用无碰撞 Atwood 闭式解解释。仿真峰值地面约束力为
`{am['peak_postlanding_contact_force_N']:.3f} N`，全程 taut 比例为
`{am['taut_fraction']:.3f}`，这正是落地后绳索会周期性松弛的数值证据。

[Atwood 落地曲线](result_1_3/figures/result_1_3_atwood_landing_theory_vs_simulation.pdf)。

## 8. 数值质量和结论边界

- 七个工况的最大 `route_status` 均为 0，未出现 degraded/invalid 路线；所有工况
  饱和比例均为 0。
- 单圆柱切向残差峰值约 `7.2e-8`，表面残差不超过浮点舍入量级。
- 当前 CSV 是确定性单次轨迹；RMSE 是时序理论—仿真误差，不是重复试验置信区间。
- `p95_step_time_ms` 包含 Python 调用和当前机器负载，仅作回归诊断，不作为跨平台
  C++ 插件性能结论。
- 最强证据是固定滑轮、自由滑轮转矩平衡、卷轴半径比和动滑轮位移关系。Atwood
  落地实验用于展示摩擦、滑移、冲击和松弛带来的理想模型偏差，论文中应单独报告
  落地前和落地后两个阶段。

## 9. 复现

```bash
cd "[your path]/mujoco-cable-dynamics/paper/icra2027_outline/experiment0814"
conda activate rope_plugin

python run_result1_experiments.py \\
  --plugin "[your path]/mujoco-cable-dynamics/build/plugin/libcable_unilateral.dylib"
```

只重算 CSV/JSON/曲线时添加 `--skip-videos`。结构和短时动力学回归使用：

```bash
python validate_result1_models.py
```
"""
    (HERE / "RESULT1_THEORY_AND_RESULTS_ZH.md").write_text(text, encoding="utf-8")


def write_video_inventory() -> None:
    rows = []
    for video in sorted(HERE.glob("result_1*/videos/*.mp4")):
        probe = subprocess.run(
            [
                "ffprobe",
                "-v",
                "error",
                "-select_streams",
                "v:0",
                "-show_entries",
                "stream=width,height,avg_frame_rate,nb_frames",
                "-show_entries",
                "format=duration,size",
                "-of",
                "json",
                str(video),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        metadata = json.loads(probe.stdout)
        stream = metadata["streams"][0]
        rows.append(
            {
                "case": video.stem.removesuffix("_raw").removesuffix("_annotated"),
                "variant": "annotated" if video.stem.endswith("_annotated") else "raw",
                "path": video.relative_to(HERE).as_posix(),
                "duration_s": float(metadata["format"]["duration"]),
                "fps": stream["avg_frame_rate"],
                "frame_count": int(stream["nb_frames"]),
                "width_px": int(stream["width"]),
                "height_px": int(stream["height"]),
                "size_bytes": int(metadata["format"]["size"]),
            }
        )
    if not rows:
        return
    with (HERE / "RESULT1_VIDEO_INVENTORY.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", type=Path, required=True)
    parser.add_argument("--case", action="append", choices=[case.key for case in CASES])
    parser.add_argument("--skip-videos", action="store_true")
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    args = parser.parse_args()
    if not args.plugin.is_file():
        parser.error(f"plugin not found: {args.plugin}")
    load_plugin(args.plugin)
    configure_plotting()
    selected = [case for case in CASES if not args.case or case.key in args.case]
    summaries = []
    all_rows: dict[str, list[dict[str, float]]] = {}
    for case in selected:
        print(f"[Result 1] {case.key}", flush=True)
        rows, summary = run_case(
            case, args.fps, args.width, args.height, not args.skip_videos
        )
        all_rows[case.key] = rows
        summaries.append(summary)
    if len(selected) == len(CASES):
        plot_overview(all_rows)
        write_summary(summaries)
        if not args.skip_videos:
            write_video_inventory()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
