#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "cable_plugin_demos" / "15_cpp_plugin_surface_single_pulley.xml"
DEFAULT_PLUGIN = ROOT / "build/plugin/libcable_unilateral.dylib"


def unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    return vector / norm if norm > 1e-12 else np.zeros(3)


def name2id(model: mujoco.MjModel, objtype: int, name: str) -> int:
    obj_id = mujoco.mj_name2id(model, objtype, name)
    if obj_id < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return obj_id


def surface_sensor(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    sensor_id = name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "surface_pulley_state")
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    if dimension != 12:
        raise ValueError(f"surface pulley sensor dimension is {dimension}, expected 12")
    return np.asarray(data.sensordata[address : address + dimension], dtype=float).copy()


def scene_route_points(
    model: mujoco.MjModel, data: mujoco.MjData, tendon_id: int
) -> list[np.ndarray]:
    option = mujoco.MjvOption()
    perturb = mujoco.MjvPerturb()
    camera = mujoco.MjvCamera()
    scene = mujoco.MjvScene(model, maxgeom=10000)
    mujoco.mjv_updateScene(
        model,
        data,
        option,
        perturb,
        camera,
        mujoco.mjtCatBit.mjCAT_ALL,
        scene,
    )
    segments: list[tuple[np.ndarray, np.ndarray]] = []
    for geom in scene.geoms[: scene.ngeom]:
        if (
            int(geom.type) != int(mujoco.mjtGeom.mjGEOM_LINE)
            or int(geom.objtype) != int(mujoco.mjtObj.mjOBJ_UNKNOWN)
            or int(geom.objid) != tendon_id
        ):
            continue
        start = np.asarray(geom.pos, dtype=float).copy()
        direction = np.asarray(geom.mat, dtype=float).reshape(3, 3)[:, 2]
        end = start + direction * float(geom.size[2])
        segments.append((start, end))
    if not segments:
        raise RuntimeError("surface-route visualize callback produced no cable segments")
    points = [segments[0][0]]
    for start, end in segments:
        if np.linalg.norm(points[-1] - start) > 2e-6:
            raise RuntimeError("surface-route visualization segments are discontinuous")
        points.append(end)
    return points


def cylinder_coordinates(
    model: mujoco.MjModel, data: mujoco.MjData, geom_id: int, point: np.ndarray
) -> tuple[float, float, np.ndarray]:
    center = np.asarray(data.geom_xpos[geom_id], dtype=float)
    rotation = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
    axis = unit(rotation[:, 2])
    delta = point - center
    axial = float(np.dot(delta, axis))
    radial = delta - axial * axis
    return float(np.linalg.norm(radial)), axial, radial


def route_forces(points: list[np.ndarray], tension: float) -> list[np.ndarray]:
    directions = [unit(points[index + 1] - points[index]) for index in range(len(points) - 1)]
    forces: list[np.ndarray] = []
    for index in range(len(points)):
        force = np.zeros(3)
        if index > 0:
            force -= tension * directions[index - 1]
        if index + 1 < len(points):
            force += tension * directions[index]
        forces.append(force)
    return forces


def seed_roles(model: mujoco.MjModel, tendon_id: int) -> list[int]:
    users = np.asarray(model.site_user, dtype=float).reshape(model.nsite, model.nuser_site)
    roles: list[int] = []
    address = int(model.tendon_adr[tendon_id])
    count = int(model.tendon_num[tendon_id])
    for wrap_index in range(address, address + count):
        if int(model.wrap_type[wrap_index]) != int(mujoco.mjtWrap.mjWRAP_SITE):
            raise ValueError("surface pulley seed must contain sites only")
        roles.append(int(round(float(users[int(model.wrap_objid[wrap_index]), 0]))))
    return roles


def route_geometry(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    tendon_id: int,
    pulley_geom_id: int,
) -> dict[str, Any]:
    points = scene_route_points(model, data, tendon_id)
    radius = float(model.geom_size[pulley_geom_id, 0])
    half_length = float(model.geom_size[pulley_geom_id, 1])
    contact_indices: list[int] = []
    surface_errors: list[float] = []
    for index, point in enumerate(points):
        radial, axial, _ = cylinder_coordinates(model, data, pulley_geom_id, point)
        if abs(radial - radius) <= 2e-6 and abs(axial) <= half_length + 2e-6:
            contact_indices.append(index)
            surface_errors.append(abs(radial - radius))
    if len(contact_indices) < 2:
        raise RuntimeError("runtime route does not contain a pulley surface envelope")
    first = min(contact_indices)
    last = max(contact_indices)
    _, _, radial_in = cylinder_coordinates(model, data, pulley_geom_id, points[first])
    _, _, radial_out = cylinder_coordinates(model, data, pulley_geom_id, points[last])
    tangent_in = abs(float(np.dot(unit(radial_in), unit(points[first] - points[first - 1]))))
    tangent_out = abs(float(np.dot(unit(radial_out), unit(points[last + 1] - points[last]))))
    center_z = float(data.geom_xpos[pulley_geom_id, 2])
    return {
        "points": points,
        "point_count": len(points),
        "surface_point_count": len(contact_indices),
        "max_surface_error_m": max(surface_errors),
        "tangent_residual_in": tangent_in,
        "tangent_residual_out": tangent_out,
        "maximum_route_z": max(float(point[2]) for point in points),
        "pulley_center_z": center_z,
        "pulley_radius": radius,
        "uses_upper_side": max(float(point[2]) for point in points) > center_z + 0.9 * radius,
    }


def analyze_demo(
    plugin_path: Path = DEFAULT_PLUGIN,
    model_path: Path = DEFAULT_MODEL,
    ctrl: float = 0.060,
    steps: int = 2500,
    ramp_steps: int = 1000,
) -> dict[str, Any]:
    mujoco.mj_loadPluginLibrary(str(plugin_path))
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    tendon_id = name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "surface_pulley_seed")
    pulley_geom_id = name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "fixed_pulley_wrap")
    payload_body_id = name2id(model, mujoco.mjtObj.mjOBJ_BODY, "free_payload")
    hint_site_id = name2id(model, mujoco.mjtObj.mjOBJ_SITE, "pulley_upper_hint")

    for _ in range(4):
        mujoco.mj_forward(model, data)
    initial_sensor = surface_sensor(model, data)
    initial_geometry = route_geometry(model, data, tendon_id, pulley_geom_id)
    initial_points = initial_geometry.pop("points")
    initial_tension = float(initial_sensor[5])
    initial_forces = route_forces(initial_points, initial_tension)
    expected_payload_force = initial_forces[-1]
    applied_payload_force = np.asarray(data.qfrc_passive[:3], dtype=float).copy()
    pulley_force = np.sum(initial_forces[1:-1], axis=0)
    endpoint_force_sum = initial_forces[0] + initial_forces[-1]
    total_force = np.sum(initial_forces, axis=0)
    total_moment = np.sum(
        [np.cross(point, force) for point, force in zip(initial_points, initial_forces, strict=True)],
        axis=0,
    )

    initial_length = float(initial_sensor[0])
    saved_hint = np.asarray(model.site_pos[hint_site_id], dtype=float).copy()
    model.site_pos[hint_site_id] = saved_hint + np.array([0.09, 0.07, -0.11])
    for _ in range(4):
        mujoco.mj_forward(model, data)
    hint_shifted_length = float(surface_sensor(model, data)[0])
    model.site_pos[hint_site_id] = saved_hint
    mujoco.mj_forward(model, data)

    initial_payload_pos = np.asarray(data.xpos[payload_body_id], dtype=float).copy()
    maximum_tangent_residual = 0.0
    maximum_surface_residual = 0.0
    statuses: set[int] = set()
    maximum_tension = 0.0
    for step in range(steps):
        data.ctrl[0] = ctrl * min(1.0, step / max(ramp_steps, 1))
        mujoco.mj_step(model, data)
        values = surface_sensor(model, data)
        statuses.add(int(round(float(values[8]))))
        maximum_tangent_residual = max(maximum_tangent_residual, float(values[9]))
        maximum_surface_residual = max(maximum_surface_residual, float(values[10]))
        maximum_tension = max(maximum_tension, float(values[5]))

    for _ in range(4):
        mujoco.mj_forward(model, data)
    final_sensor = surface_sensor(model, data)
    final_geometry = route_geometry(model, data, tendon_id, pulley_geom_id)
    final_geometry.pop("points")
    final_payload_pos = np.asarray(data.xpos[payload_body_id], dtype=float).copy()
    displacement = final_payload_pos - initial_payload_pos
    mass = float(model.body_mass[payload_body_id])
    weight = mass * abs(float(model.opt.gravity[2]))

    xml = ET.parse(model_path).getroot()
    instance = xml.find("./extension/plugin/instance[@name='surface_pulley_cable']")
    configs = {
        node.attrib["key"]: node.attrib["value"]
        for node in instance.findall("config")
    } if instance is not None else {}
    gear = np.asarray(model.actuator_gear[0], dtype=float)

    checks = {
        "surface_configuration": configs.get("route_mode") == "surface"
        and configs.get("route_tendon") == "surface_pulley_seed"
        and configs.get("wrap_geoms") == "fixed_pulley_wrap",
        "seed_roles_are_endpoint_hint_endpoint": seed_roles(model, tendon_id) == [1, 2, 1],
        "surface_actuator_gear_is_zero": float(np.max(np.abs(gear))) <= 1e-12,
        "upper_surface_envelope": bool(initial_geometry["uses_upper_side"]),
        "surface_error_below_1e-7_m": max(
            float(initial_geometry["max_surface_error_m"]),
            float(final_geometry["max_surface_error_m"]),
            maximum_surface_residual,
        ) <= 1e-7,
        "tangent_residual_below_1e-5": max(
            float(initial_geometry["tangent_residual_in"]),
            float(initial_geometry["tangent_residual_out"]),
            float(final_geometry["tangent_residual_in"]),
            float(final_geometry["tangent_residual_out"]),
            maximum_tangent_residual,
        ) <= 1e-5,
        "hint_is_initialization_only": abs(hint_shifted_length - initial_length) <= 1e-9,
        "payload_force_matches_route": float(np.linalg.norm(applied_payload_force - expected_payload_force)) <= 1e-8,
        "cable_force_balance": float(np.linalg.norm(total_force)) <= 1e-8,
        "cable_moment_balance": float(np.linalg.norm(total_moment)) <= 1e-8,
        "pulley_resultant_balances_endpoints": float(np.linalg.norm(pulley_force + endpoint_force_sum)) <= 1e-8,
        "route_remained_valid": statuses.issubset({0, 1}),
        "payload_lift_is_visible": float(displacement[2]) >= 0.045,
        "payload_stays_in_pulley_plane": float(np.linalg.norm(displacement[:2])) <= 0.005,
        "final_tension_supports_payload": abs(float(final_sensor[5]) - weight) <= 0.08,
        "lift_matches_commanded_contraction": abs(float(displacement[2]) - ctrl) <= 0.004,
    }

    return {
        "pass": all(checks.values()),
        "model": str(model_path),
        "plugin": str(plugin_path),
        "control_contraction_m": ctrl,
        "steps": steps,
        "checks": checks,
        "seed_roles": seed_roles(model, tendon_id),
        "initial": {
            "path_length_m": initial_length,
            "tension_N": initial_tension,
            "payload_position_m": initial_payload_pos.tolist(),
            "geometry": initial_geometry,
        },
        "final": {
            "path_length_m": float(final_sensor[0]),
            "free_length_m": float(final_sensor[2]),
            "tension_N": float(final_sensor[5]),
            "payload_position_m": final_payload_pos.tolist(),
            "payload_displacement_m": displacement.tolist(),
            "geometry": final_geometry,
        },
        "force_check": {
            "equal_side_tension_N": initial_tension,
            "payload_force_expected_N": expected_payload_force.tolist(),
            "payload_force_applied_qfrc_N": applied_payload_force.tolist(),
            "pulley_cable_resultant_N": pulley_force.tolist(),
            "support_reaction_N": (-pulley_force).tolist(),
            "total_force_residual_N": float(np.linalg.norm(total_force)),
            "total_moment_residual_Nm": float(np.linalg.norm(total_moment)),
        },
        "runtime": {
            "route_status_values": sorted(statuses),
            "maximum_tension_N": maximum_tension,
            "maximum_tangent_residual": maximum_tangent_residual,
            "maximum_surface_residual_m": maximum_surface_residual,
            "hint_shift_length_change_m": hint_shifted_length - initial_length,
        },
        "model_limit": (
            "This demo is an ideal frictionless fixed guide pulley. The current cable model computes the "
            "surface envelope and cable forces, but it does not impose rope/sheave no-slip rotation."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate surface-envelope fixed-pulley Demo 15")
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--ctrl", type=float, default=0.060)
    parser.add_argument("--steps", type=int, default=2500)
    parser.add_argument("--ramp-steps", type=int, default=1000)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs" / "demo15_surface_pulley_check.json")
    args = parser.parse_args()
    report = analyze_demo(args.plugin.resolve(), args.model.resolve(), args.ctrl, args.steps, args.ramp_steps)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
