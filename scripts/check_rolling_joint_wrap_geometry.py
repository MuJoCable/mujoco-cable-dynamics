#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "cable_plugin_demos" / "13_cpp_plugin_rolling_joint_figure_eight.xml"
DEFAULT_PLUGIN = ROOT / "build/plugin/libcable_unilateral.dylib"


REQUIRED_WRAP_GEOMS: dict[str, tuple[str, ...]] = {
    "upper_strap_seed": ("proximal_roller", "distal_roller"),
    "lower_strap_seed": ("proximal_roller", "distal_roller"),
    "upper_drive_seed": ("proximal_roller", "distal_roller"),
    "lower_drive_seed": ("proximal_roller", "distal_roller"),
    "left_ligament_a_seed": ("proximal_side_left", "distal_side_left"),
    "left_ligament_b_seed": ("proximal_side_left", "distal_side_left"),
    "right_ligament_a_seed": ("proximal_side_right", "distal_side_right"),
    "right_ligament_b_seed": ("proximal_side_right", "distal_side_right"),
}


def load_plugin(plugin: Path | None) -> None:
    if plugin is not None and plugin.exists():
        mujoco.mj_loadPluginLibrary(str(plugin))


def name2id(model: mujoco.MjModel, objtype: int, name: str) -> int:
    obj_id = mujoco.mj_name2id(model, objtype, name)
    if obj_id < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return obj_id


def unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    return vector / norm if norm > 1e-12 else np.zeros(3)


def cylinder_coordinates(
    model: mujoco.MjModel, data: mujoco.MjData, geom_id: int, point: np.ndarray
) -> tuple[float, float, np.ndarray]:
    center = np.asarray(data.geom_xpos, dtype=float)[geom_id]
    axis = unit(np.asarray(data.geom_xmat, dtype=float).reshape(-1, 3, 3)[geom_id][:, 2])
    delta = point - center
    axial = float(np.dot(delta, axis))
    radial = delta - axial * axis
    return float(np.linalg.norm(radial)), axial, radial


def scene_route_points(model: mujoco.MjModel, data: mujoco.MjData) -> dict[int, list[np.ndarray]]:
    option = mujoco.MjvOption()
    perturb = mujoco.MjvPerturb()
    camera = mujoco.MjvCamera()
    scene = mujoco.MjvScene(model, maxgeom=max(10000, 128 * model.ntendon))
    mujoco.mjv_updateScene(
        model,
        data,
        option,
        perturb,
        camera,
        mujoco.mjtCatBit.mjCAT_ALL,
        scene,
    )
    segments: dict[int, list[tuple[np.ndarray, np.ndarray]]] = defaultdict(list)
    for geom in scene.geoms[: scene.ngeom]:
        tendon_id = int(geom.objid)
        if (
            int(geom.type) != int(mujoco.mjtGeom.mjGEOM_LINE)
            or int(geom.objtype) != int(mujoco.mjtObj.mjOBJ_UNKNOWN)
            or tendon_id < 0
            or tendon_id >= model.ntendon
            or float(model.tendon_width[tendon_id]) > 1e-8
        ):
            continue
        start = np.asarray(geom.pos, dtype=float).copy()
        direction = np.asarray(geom.mat, dtype=float).reshape(3, 3)[:, 2]
        end = start + direction * float(geom.size[2])
        segments[tendon_id].append((start, end))

    routes: dict[int, list[np.ndarray]] = {}
    for tendon_id, tendon_segments in segments.items():
        points = [tendon_segments[0][0]]
        for start, end in tendon_segments:
            if np.linalg.norm(points[-1] - start) > 1e-6:
                raise ValueError(f"visual route for tendon {tendon_id} is discontinuous")
            points.append(end)
        routes[tendon_id] = points
    return routes


def route_length(points: list[np.ndarray]) -> float:
    return float(
        sum(np.linalg.norm(points[index + 1] - points[index]) for index in range(len(points) - 1))
    )


def seed_site_roles(model: mujoco.MjModel, tendon_id: int) -> list[int]:
    roles: list[int] = []
    address = int(model.tendon_adr[tendon_id])
    count = int(model.tendon_num[tendon_id])
    users = np.asarray(model.site_user, dtype=float).reshape(model.nsite, model.nuser_site)
    for wrap_index in range(address, address + count):
        if int(model.wrap_type[wrap_index]) != int(mujoco.mjtWrap.mjWRAP_SITE):
            raise ValueError("surface route seed tendon may contain only site elements")
        site_id = int(model.wrap_objid[wrap_index])
        roles.append(int(round(float(users[site_id, 0]))))
    return roles


def plugin_configs(model_path: Path) -> dict[str, dict[str, str]]:
    root = ET.parse(model_path).getroot()
    configs: dict[str, dict[str, str]] = {}
    for instance in root.findall("./extension/plugin/instance"):
        values = {item.attrib["key"]: item.attrib["value"] for item in instance.findall("config")}
        if values.get("route_mode", "native") == "surface":
            configs[instance.attrib["name"]] = values
    return configs


def sensor_diagnostics(model: mujoco.MjModel, data: mujoco.MjData) -> list[dict[str, Any]]:
    diagnostics: list[dict[str, Any]] = []
    for sensor_id in range(model.nsensor):
        if (
            int(model.sensor_type[sensor_id]) != int(mujoco.mjtSensor.mjSENS_PLUGIN)
            or int(model.sensor_dim[sensor_id]) != 12
        ):
            continue
        address = int(model.sensor_adr[sensor_id])
        values = np.asarray(data.sensordata[address : address + 12], dtype=float)
        diagnostics.append(
            {
                "sensor": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_id),
                "length": float(values[0]),
                "route_status": int(round(float(values[8]))),
                "tangent_residual": float(values[9]),
                "surface_residual": float(values[10]),
                "solver_iterations": int(round(float(values[11]))),
            }
        )
    return diagnostics


def analyze_model(
    model_path: Path = DEFAULT_MODEL,
    plugin_path: Path | None = DEFAULT_PLUGIN,
) -> dict[str, Any]:
    load_plugin(plugin_path)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    # Let the coupled multi-surface warm start reach its fixed point before
    # measuring hint independence.
    for _ in range(8):
        mujoco.mj_forward(model, data)

    failures: list[str] = []
    configs = plugin_configs(model_path)
    route_points = scene_route_points(model, data)
    route_lengths_before = {tendon: route_length(points) for tendon, points in route_points.items()}
    max_surface_error = 0.0
    max_tangent_error = 0.0
    tendon_reports: list[dict[str, Any]] = []

    configured_routes = {
        values.get("route_tendon", ""): tuple(values.get("wrap_geoms", "").split())
        for values in configs.values()
    }
    if configured_routes != REQUIRED_WRAP_GEOMS:
        failures.append(f"surface route configs differ from expected routes: {configured_routes}")
    if model.nuser_site < 1:
        failures.append("nuser_site must be at least one")
    if model.neq:
        failures.append("rolling-joint demo must not use equality constraints")
    nonfree_joints = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        for joint_id in range(model.njnt)
        if int(model.jnt_type[joint_id]) != int(mujoco.mjtJoint.mjJNT_FREE)
    ]
    if nonfree_joints:
        failures.append(f"rolling-joint demo contains non-free joints: {nonfree_joints}")
    for actuator_id in range(model.nu):
        if np.max(np.abs(np.asarray(model.actuator_gear[actuator_id], dtype=float))) > 1e-12:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            failures.append(f"surface actuator {name} must use gear=0")

    for tendon_name, geom_names in REQUIRED_WRAP_GEOMS.items():
        tendon_id = name2id(model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name)
        roles = seed_site_roles(model, tendon_id)
        if roles != [1, 2, 2, 1]:
            failures.append(f"{tendon_name} has site roles {roles}, expected [1, 2, 2, 1]")
        points = route_points.get(tendon_id, [])
        if len(points) < 4:
            failures.append(f"{tendon_name} has no plugin-visualized surface route")
            continue
        geom_reports: list[dict[str, Any]] = []
        for geom_name in geom_names:
            geom_id = name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            if int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_CYLINDER):
                failures.append(f"{geom_name} is not a cylinder")
                continue
            radius = float(model.geom_size[geom_id, 0])
            half_length = float(model.geom_size[geom_id, 1])
            contacts: list[int] = []
            errors: list[float] = []
            for point_index, point in enumerate(points):
                radial, axial, _ = cylinder_coordinates(model, data, geom_id, point)
                error = abs(radial - radius)
                if error <= 2e-6 and abs(axial) <= half_length + 2e-6:
                    contacts.append(point_index)
                    errors.append(error)
            if len(contacts) < 2:
                failures.append(f"{tendon_name} does not envelope {geom_name}")
                continue
            first, last = min(contacts), max(contacts)
            local_tangent_errors: list[float] = []
            if first > 0:
                _, _, radial = cylinder_coordinates(model, data, geom_id, points[first])
                local_tangent_errors.append(
                    abs(float(np.dot(unit(radial), unit(points[first] - points[first - 1]))))
                )
            if last + 1 < len(points):
                _, _, radial = cylinder_coordinates(model, data, geom_id, points[last])
                local_tangent_errors.append(
                    abs(float(np.dot(unit(radial), unit(points[last + 1] - points[last]))))
                )
            max_surface_error = max(max_surface_error, max(errors, default=0.0))
            max_tangent_error = max(max_tangent_error, max(local_tangent_errors, default=0.0))
            geom_reports.append(
                {
                    "geom": geom_name,
                    "contact_point_count": len(contacts),
                    "surface_error": max(errors, default=0.0),
                    "tangent_errors": local_tangent_errors,
                }
            )
        tendon_reports.append(
            {
                "tendon": tendon_name,
                "site_roles": roles,
                "length": route_length(points),
                "required_wrap_geoms": list(geom_names),
                "contacts": geom_reports,
            }
        )

    diagnostics_before = sensor_diagnostics(model, data)
    for item in diagnostics_before:
        if item["route_status"] not in (0, 1):
            failures.append(f"invalid route status in {item['sensor']}: {item['route_status']}")
        max_tangent_error = max(max_tangent_error, float(item["tangent_residual"]))
        max_surface_error = max(max_surface_error, float(item["surface_residual"]))

    users = np.asarray(model.site_user, dtype=float).reshape(model.nsite, model.nuser_site)
    hint_site_ids = np.flatnonzero(np.rint(users[:, 0]).astype(int) == 2)
    original_hint_positions = np.asarray(model.site_pos, dtype=float)[hint_site_ids].copy()
    model.site_pos[hint_site_ids] += np.array([0.071, -0.043, 0.029])
    mujoco.mj_forward(model, data)
    route_points_after = scene_route_points(model, data)
    hint_length_changes: dict[str, float] = {}
    for tendon_name in REQUIRED_WRAP_GEOMS:
        tendon_id = name2id(model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name)
        change = abs(route_length(route_points_after[tendon_id]) - route_lengths_before[tendon_id])
        hint_length_changes[tendon_name] = change
        if change > 1e-10:
            failures.append(f"{tendon_name} changed by {change:g} after moving initialization hints")
    model.site_pos[hint_site_ids] = original_hint_positions

    return {
        "model": str(model_path),
        "plugin": str(plugin_path) if plugin_path is not None else None,
        "pass": not failures and max_surface_error <= 1e-7 and max_tangent_error <= 1e-6,
        "max_surface_error": max_surface_error,
        "max_tangent_error": max_tangent_error,
        "missing_required": failures,
        "hint_length_changes": hint_length_changes,
        "surface_sensors": diagnostics_before,
        "tendons": tendon_reports,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Check surface-envelope routes in rolling-joint Demo 13")
    parser.add_argument("--model", default=str(DEFAULT_MODEL), help="Path to the MJCF model")
    parser.add_argument("--plugin", default=str(DEFAULT_PLUGIN), help="Path to the compiled C++ plugin")
    args = parser.parse_args()

    report = analyze_model(Path(args.model).resolve(), Path(args.plugin).resolve())
    print(json.dumps(report, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
