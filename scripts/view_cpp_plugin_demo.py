#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SLACK_RGBA = np.array([0.48, 0.51, 0.55, 0.55], dtype=np.float64)
INVALID_RGBA = np.array([0.95, 0.08, 0.08, 1.0], dtype=np.float64)


@dataclass(frozen=True)
class CableSensorBinding:
    name: str
    sensor_id: int
    tendon_id: int
    sensor_address: int
    sensor_dimension: int


@dataclass(frozen=True)
class CableVisualState:
    binding: CableSensorBinding
    length: float
    free_length: float
    extension: float
    tension: float
    taut: bool
    route_status: int


@dataclass(frozen=True)
class SpoolDebugSpec:
    label: str
    instance_name: str
    geom_id: int
    fixed_site_id: int
    exit_site_id: int
    tendon_id: int
    joint_id: int
    qpos_address: int
    qpos0: float
    spool_radius: float
    reserve_length: float
    reserve_sign: float


@dataclass(frozen=True)
class SpoolDebugGeometry:
    center: np.ndarray
    cylinder_axis: np.ndarray
    joint_axis: np.ndarray
    axis_alignment: float
    fixed: np.ndarray
    exit_point: np.ndarray
    fixed_axial: float
    fixed_direction: np.ndarray
    signed_wound_angle: float
    arc_angle: float
    expected_exit: np.ndarray


def _as_vec3(values) -> np.ndarray:
    return np.asarray(values, dtype=np.float64).reshape(3)


def _unit(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-12:
        return np.zeros(3, dtype=np.float64)
    return vec / norm


def _cylinder_axis(data: mujoco.MjData, geom_id: int) -> np.ndarray:
    xmat = np.asarray(data.geom_xmat, dtype=np.float64).reshape(-1, 3, 3)[geom_id]
    return _unit(xmat[:, 2])


def _surface_offset_point(model: mujoco.MjModel, data: mujoco.MjData, point: np.ndarray, geom_id: int, offset: float) -> np.ndarray:
    if geom_id < 0:
        return point
    center = np.asarray(data.geom_xpos, dtype=np.float64).reshape(-1, 3)[geom_id]
    if int(model.geom_type[geom_id]) == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
        axis = _cylinder_axis(data, geom_id)
        radial = point - center
        radial = radial - axis * float(np.dot(radial, axis))
        direction = _unit(radial)
        return point + offset * direction if np.linalg.norm(direction) > 0 else point
    direction = _unit(point - center)
    return point + offset * direction if np.linalg.norm(direction) > 0 else point


def _short_cylinder_arc_points(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    geom_id: int,
    start: np.ndarray,
    end: np.ndarray,
    segments: int,
    offset: float,
) -> list[np.ndarray]:
    center = np.asarray(data.geom_xpos, dtype=np.float64).reshape(-1, 3)[geom_id]
    axis = _cylinder_axis(data, geom_id)
    axial = 0.5 * (float(np.dot(start - center, axis)) + float(np.dot(end - center, axis)))
    arc_center = center + axial * axis

    r0 = start - arc_center
    r1 = end - arc_center
    r0 = r0 - axis * float(np.dot(r0, axis))
    r1 = r1 - axis * float(np.dot(r1, axis))
    if np.linalg.norm(r0) <= 1e-12 or np.linalg.norm(r1) <= 1e-12:
        return [start, end]

    e0 = _unit(r0)
    e1 = _unit(np.cross(axis, e0))
    signed_angle = float(np.arctan2(np.dot(r1, e1), np.dot(r1, e0)))
    if signed_angle > np.pi:
        signed_angle -= 2.0 * np.pi
    elif signed_angle < -np.pi:
        signed_angle += 2.0 * np.pi

    radius = float(model.geom_size[geom_id, 0]) + offset
    count = max(2, int(segments) + 1)
    return [
        arc_center + radius * (np.cos(theta) * e0 + np.sin(theta) * e1)
        for theta in np.linspace(0.0, signed_angle, count)
    ]


def _project_to_cylinder_surface(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    geom_id: int,
    point: np.ndarray,
    offset: float,
) -> np.ndarray:
    center = np.asarray(data.geom_xpos, dtype=np.float64).reshape(-1, 3)[geom_id]
    axis = _cylinder_axis(data, geom_id)
    axial = float(np.dot(point - center, axis))
    arc_center = center + axial * axis
    radial = point - arc_center
    direction = _unit(radial)
    if np.linalg.norm(direction) <= 0:
        return point
    return arc_center + (float(model.geom_size[geom_id, 0]) + offset) * direction


def _cylinder_tangent_from_external_point(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    geom_id: int,
    start: np.ndarray,
    target: np.ndarray,
    offset: float,
) -> np.ndarray | None:
    center = np.asarray(data.geom_xpos, dtype=np.float64).reshape(-1, 3)[geom_id]
    axis = _cylinder_axis(data, geom_id)
    axial = float(np.dot(start - center, axis))
    arc_center = center + axial * axis
    radius = float(model.geom_size[geom_id, 0]) + offset

    target_in_plane = target - axis * float(np.dot(target - arc_center, axis))
    to_target = target_in_plane - arc_center
    distance = float(np.linalg.norm(to_target))
    if distance <= radius + 1e-9:
        return None

    e = _unit(to_target)
    side = _unit(np.cross(axis, e))
    along = radius * radius / distance
    lateral = radius * np.sqrt(max(distance * distance - radius * radius, 0.0)) / distance
    candidates = [
        arc_center + along * e + lateral * side,
        arc_center + along * e - lateral * side,
    ]

    start_surface = _project_to_cylinder_surface(model, data, geom_id, start, offset)

    def arc_size(candidate: np.ndarray) -> float:
        arc = _short_cylinder_arc_points(model, data, geom_id, start_surface, candidate, 4, offset)
        return sum(float(np.linalg.norm(p1 - p0)) for p0, p1 in zip(arc[:-1], arc[1:], strict=False))

    return min(candidates, key=arc_size)


def _connector(scene: mujoco.MjvScene, geom_type: int, p0: np.ndarray, p1: np.ndarray, width: float, rgba: np.ndarray) -> None:
    if scene.ngeom >= len(scene.geoms):
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        geom_type,
        np.zeros(3, dtype=np.float64),
        np.zeros(3, dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(9),
        rgba.astype(np.float32),
    )
    mujoco.mjv_connector(geom, geom_type, width, _as_vec3(p0), _as_vec3(p1))
    scene.ngeom += 1


def _sphere(
    scene: mujoco.MjvScene,
    position: np.ndarray,
    radius: float,
    rgba: np.ndarray,
) -> None:
    if scene.ngeom >= len(scene.geoms):
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.full(3, radius, dtype=np.float64),
        _as_vec3(position),
        np.eye(3, dtype=np.float64).reshape(9),
        rgba.astype(np.float32),
    )
    scene.ngeom += 1


def _label(scene: mujoco.MjvScene, position: np.ndarray, text: str, rgba: np.ndarray) -> None:
    if scene.ngeom >= len(scene.geoms):
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_LABEL,
        np.zeros(3, dtype=np.float64),
        np.zeros(3, dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(9),
        rgba.astype(np.float32),
    )
    geom.pos[:] = _as_vec3(position)
    geom.label = text
    scene.ngeom += 1


def _surface_route_tendons(model_path: Path) -> dict[str, str]:
    root = ET.parse(model_path).getroot()
    route_by_instance = {}
    for instance in root.findall("./extension/plugin/instance"):
        config = {
            item.attrib["key"]: item.attrib["value"]
            for item in instance.findall("config")
        }
        route = config.get("route_tendon")
        if route:
            route_by_instance[instance.attrib["name"]] = route
    return {
        sensor.attrib["name"]: route_by_instance[sensor.attrib["instance"]]
        for sensor in root.findall("./sensor/plugin")
        if sensor.attrib.get("instance") in route_by_instance
    }


def cable_sensor_bindings(model: mujoco.MjModel, model_path: Path) -> list[CableSensorBinding]:
    xml_routes = _surface_route_tendons(model_path)
    actuator_plugins = np.asarray(model.actuator_plugin, dtype=np.int32)
    actuator_tendons = np.asarray(model.actuator_trnid, dtype=np.int32).reshape(-1, 2)
    bindings = []
    for sensor_id in range(model.nsensor):
        dimension = int(model.sensor_dim[sensor_id])
        instance = int(model.sensor_plugin[sensor_id])
        if dimension < 8 or instance < 0:
            continue
        sensor_name = (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_id)
            or f"sensor_{sensor_id}"
        )
        tendon_id = -1
        actuator_matches = np.flatnonzero(actuator_plugins == instance)
        if actuator_matches.size:
            tendon_id = int(actuator_tendons[int(actuator_matches[0]), 0])
        if tendon_id < 0 and sensor_name in xml_routes:
            tendon_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_TENDON, xml_routes[sensor_name]
            )
        if tendon_id < 0:
            stem = sensor_name.removeprefix("state_").removesuffix("_state")
            for candidate in (stem, f"{stem}_seed", stem.removeprefix("passive_")):
                tendon_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_TENDON, candidate
                )
                if tendon_id >= 0:
                    break
        if tendon_id < 0:
            continue
        tendon_name = (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TENDON, tendon_id)
            or f"tendon_{tendon_id}"
        )
        bindings.append(
            CableSensorBinding(
                name=tendon_name.removesuffix("_seed"),
                sensor_id=sensor_id,
                tendon_id=tendon_id,
                sensor_address=int(model.sensor_adr[sensor_id]),
                sensor_dimension=dimension,
            )
        )
    return bindings


def read_cable_visual_states(
    data: mujoco.MjData, bindings: list[CableSensorBinding]
) -> list[CableVisualState]:
    states = []
    for binding in bindings:
        values = np.asarray(
            data.sensordata[
                binding.sensor_address : binding.sensor_address
                + binding.sensor_dimension
            ],
            dtype=np.float64,
        )
        states.append(
            CableVisualState(
                binding=binding,
                length=float(values[0]),
                free_length=float(values[2]),
                extension=float(values[4]),
                tension=float(values[5]),
                taut=bool(values[6] > 0.5),
                route_status=(int(round(values[8])) if len(values) >= 12 else 0),
            )
        )
    return states


def update_cable_state_colors(
    model: mujoco.MjModel,
    states: list[CableVisualState],
    base_rgba: np.ndarray,
) -> None:
    for state in states:
        tendon_id = state.binding.tendon_id
        model.tendon_rgba[tendon_id] = (
            base_rgba[tendon_id] if state.taut else SLACK_RGBA
        )


def _tendon_path_midpoint(
    data: mujoco.MjData, tendon_id: int
) -> np.ndarray | None:
    address = int(data.ten_wrapadr[tendon_id])
    count = int(data.ten_wrapnum[tendon_id])
    if count < 2:
        return None
    points = np.asarray(data.wrap_xpos, dtype=np.float64).reshape(-1, 3)[
        address : address + count
    ]
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = float(np.sum(segment_lengths))
    if total <= 1e-12:
        return np.mean(points, axis=0)
    target = 0.5 * total
    accumulated = 0.0
    for index, length in enumerate(segment_lengths):
        if accumulated + length >= target:
            fraction = (target - accumulated) / max(float(length), 1e-12)
            return points[index] + fraction * (points[index + 1] - points[index])
        accumulated += float(length)
    return points[-1].copy()


def draw_cable_state_labels(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    scene: mujoco.MjvScene | None,
    states: list[CableVisualState],
) -> None:
    if scene is None:
        return
    for state in states:
        midpoint = _tendon_path_midpoint(data, state.binding.tendon_id)
        if midpoint is None:
            continue
        midpoint = midpoint + np.array([0.0, 0.0, 0.012], dtype=np.float64)
        strain = max(state.extension, 0.0) / max(state.free_length, 1e-12)
        if state.route_status >= 2:
            status = "INVALID"
        elif state.route_status == 1:
            status = "DEGRADED"
        else:
            status = "TAUT" if state.taut else "SLACK"
        if state.route_status >= 2:
            text = f"{state.binding.name}  ROUTE INVALID - FORCE DISABLED"
        else:
            text = (
                f"{state.binding.name}  L={1000.0 * state.length:.1f}/"
                f"{1000.0 * state.free_length:.1f}mm  "
                f"eps={100.0 * strain:.2f}%  "
                f"T={state.tension:.2f}N  {status}"
            )
        rgba = np.asarray(model.tendon_rgba[state.binding.tendon_id], dtype=np.float64)
        if state.route_status >= 2:
            rgba = INVALID_RGBA
        _label(scene, midpoint, text, rgba)


def _tendon_style(model: mujoco.MjModel, tendon_id: int, base_width: float) -> tuple[float, np.ndarray]:
    rgba = np.asarray(model.tendon_rgba, dtype=np.float64).reshape(-1, 4)[tendon_id].copy()
    if float(np.max(rgba[:3])) <= 0:
        rgba[:3] = np.array([1.0, 0.25, 0.05], dtype=np.float64)
    rgba[3] = max(float(rgba[3]), 0.82)
    tendon_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TENDON, tendon_id) or ""
    if "drive" in tendon_name:
        width = base_width * 1.12
    elif "strap" in tendon_name:
        width = base_width * 0.86
    else:
        width = base_width * 0.72
    return width, rgba


def _draw_tension_arrow(scene: mujoco.MjvScene, p0: np.ndarray, p1: np.ndarray, width: float, rgba: np.ndarray) -> None:
    direction = p1 - p0
    length = float(np.linalg.norm(direction))
    if length <= 1e-9:
        return
    direction /= length
    arrow_length = min(0.045, 0.35 * length)
    mid = 0.5 * (p0 + p1)
    start = mid - 0.5 * arrow_length * direction
    end = mid + 0.5 * arrow_length * direction
    _connector(scene, mujoco.mjtGeom.mjGEOM_ARROW, start, end, width, rgba)


def _draw_segment(scene: mujoco.MjvScene, p0: np.ndarray, p1: np.ndarray, width: float, rgba: np.ndarray) -> None:
    _connector(scene, mujoco.mjtGeom.mjGEOM_CAPSULE, p0, p1, width, rgba)


def _draw_arc(
    scene: mujoco.MjvScene,
    points: list[np.ndarray],
    line_width: float,
    rgba: np.ndarray,
) -> None:
    for p0, p1 in zip(points[:-1], points[1:], strict=False):
        _connector(scene, mujoco.mjtGeom.mjGEOM_LINE, p0, p1, line_width, rgba)


def draw_wrapped_tendon_visuals(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    scene: mujoco.MjvScene | None,
    *,
    tendon_ids: list[int],
    cable_width: float,
    arc_line_width: float,
    arc_segments: int,
    show_tension_arrows: bool,
    show_spool_wrap: bool,
    spool_geom_name: str,
    spool_anchor_site: str,
    clear_scene: bool = True,
) -> None:
    if scene is None:
        return
    if clear_scene:
        scene.ngeom = 0
    arrow_rgba = np.array([1.0, 0.95, 0.1, 0.88], dtype=np.float32)
    spool_geom_id = (
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, spool_geom_name)
        if show_spool_wrap
        else -1
    )
    spool_site_id = (
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, spool_anchor_site)
        if show_spool_wrap
        else -1
    )

    for tendon_id in tendon_ids:
        width, rgba = _tendon_style(model, tendon_id, cable_width)
        surface_offset = 0.7 * width
        adr = int(data.ten_wrapadr[tendon_id])
        num = int(data.ten_wrapnum[tendon_id])
        if num < 2:
            continue
        points = np.asarray(data.wrap_xpos, dtype=np.float64).reshape(-1, 3)[adr : adr + num]
        objs = np.asarray(data.wrap_obj, dtype=np.int32).reshape(-1)[adr : adr + num]
        first_point_override = None
        if spool_geom_id >= 0 and spool_site_id >= 0 and num >= 2:
            anchor = np.asarray(data.site_xpos, dtype=np.float64).reshape(-1, 3)[spool_site_id]
            if float(np.linalg.norm(points[0] - anchor)) < 1e-5:
                tangent = _cylinder_tangent_from_external_point(
                    model, data, spool_geom_id, anchor, points[1], surface_offset
                )
                if tangent is not None:
                    anchor_surface = _project_to_cylinder_surface(
                        model, data, spool_geom_id, anchor, surface_offset
                    )
                    spool_arc = _short_cylinder_arc_points(
                        model, data, spool_geom_id, anchor_surface, tangent, arc_segments, surface_offset
                    )
                    _draw_arc(scene, spool_arc, arc_line_width, rgba)
                    first_point_override = tangent

        for i in range(num - 1):
            geom0 = int(objs[i])
            geom1 = int(objs[i + 1])
            if (
                geom0 >= 0
                and geom0 == geom1
                and int(model.geom_type[geom0]) == int(mujoco.mjtGeom.mjGEOM_CYLINDER)
            ):
                arc = _short_cylinder_arc_points(
                    model, data, geom0, points[i], points[i + 1], arc_segments, surface_offset
                )
                _draw_arc(scene, arc, arc_line_width, rgba)
                if show_tension_arrows and len(arc) >= 3:
                    _draw_tension_arrow(scene, arc[len(arc) // 3], arc[min(len(arc) - 1, len(arc) // 3 + 1)], 0.45 * width, arrow_rgba)
                continue

            p0 = _surface_offset_point(model, data, points[i], geom0, surface_offset)
            if i == 0 and first_point_override is not None:
                p0 = first_point_override
            p1 = _surface_offset_point(model, data, points[i + 1], geom1, surface_offset)
            _draw_segment(scene, p0, p1, width, rgba)
            if show_tension_arrows:
                _draw_tension_arrow(scene, p0, p1, 0.45 * width, arrow_rgba)


def _spool_debug_specs(
    model: mujoco.MjModel, model_path: Path
) -> list[SpoolDebugSpec]:
    root = ET.parse(model_path).getroot()
    instance_configs = {
        instance.attrib["name"]: {
            config.attrib["key"]: config.attrib["value"]
            for config in instance.findall("config")
        }
        for instance in root.findall("./extension/plugin/instance")
    }
    specs: list[SpoolDebugSpec] = []
    for item in root.findall("./custom/text"):
        name = item.attrib.get("name", "")
        if not name.startswith("spool_debug_"):
            continue
        fields = item.attrib.get("data", "").split()
        if len(fields) != 5:
            raise ValueError(
                f"custom text {name!r} must contain "
                "'instance geom fixed_site exit_site tendon'"
            )
        instance_name, geom_name, fixed_name, exit_name, tendon_name = fields
        config = instance_configs.get(instance_name)
        if config is None:
            raise ValueError(
                f"spool debug instance not found: {instance_name}"
            )
        joint_name = config.get("spool_joint", "")
        geom_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, geom_name
        )
        fixed_site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, fixed_name
        )
        exit_site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, exit_name
        )
        tendon_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name
        )
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
        )
        if min(geom_id, fixed_site_id, exit_site_id, tendon_id, joint_id) < 0:
            raise ValueError(f"invalid spool debug object in {name!r}")
        qpos_address = int(model.jnt_qposadr[joint_id])
        specs.append(
            SpoolDebugSpec(
                label=name.removeprefix("spool_debug_"),
                instance_name=instance_name,
                geom_id=geom_id,
                fixed_site_id=fixed_site_id,
                exit_site_id=exit_site_id,
                tendon_id=tendon_id,
                joint_id=joint_id,
                qpos_address=qpos_address,
                qpos0=float(model.qpos0[qpos_address]),
                spool_radius=float(config["spool_radius"]),
                reserve_length=float(
                    config.get("spool_reserve_length", "0")
                ),
                reserve_sign=(
                    -1.0
                    if config.get("spool_reserve_direction", "positive")
                    == "negative"
                    else 1.0
                ),
            )
        )
    return specs


def _rotate_vector(
    vector: np.ndarray, axis: np.ndarray, angle: float
) -> np.ndarray:
    axis = _unit(axis)
    return (
        np.cos(angle) * vector
        + np.sin(angle) * np.cross(axis, vector)
        + (1.0 - np.cos(angle)) * axis * float(np.dot(axis, vector))
    )


def _spool_debug_geometry(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    spec: SpoolDebugSpec,
) -> SpoolDebugGeometry:
    geom_positions = np.asarray(data.geom_xpos, dtype=np.float64).reshape(-1, 3)
    geom_rotations = np.asarray(data.geom_xmat, dtype=np.float64).reshape(
        -1, 3, 3
    )
    site_positions = np.asarray(data.site_xpos, dtype=np.float64).reshape(-1, 3)
    joint_axes = np.asarray(data.xaxis, dtype=np.float64).reshape(-1, 3)

    center = geom_positions[spec.geom_id]
    cylinder_axis = _unit(geom_rotations[spec.geom_id][:, 2])
    joint_axis = _unit(joint_axes[spec.joint_id])
    axis_dot = float(np.dot(cylinder_axis, joint_axis))
    if abs(axis_dot) < 0.999:
        geom_name = (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, spec.geom_id)
            or str(spec.geom_id)
        )
        joint_name = (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, spec.joint_id)
            or str(spec.joint_id)
        )
        raise ValueError(
            f"spool debug geom {geom_name!r} axis must be parallel to "
            f"joint {joint_name!r}; abs(dot)={abs(axis_dot):.6f}"
        )
    axis_alignment = 1.0 if axis_dot >= 0 else -1.0

    fixed = site_positions[spec.fixed_site_id]
    exit_point = site_positions[spec.exit_site_id]
    fixed_delta = fixed - center
    fixed_axial = float(np.dot(fixed_delta, cylinder_axis))
    fixed_radial = fixed_delta - fixed_axial * cylinder_axis
    fixed_direction = _unit(fixed_radial)
    if float(np.linalg.norm(fixed_direction)) <= 0:
        raise ValueError(f"spool debug fixed site for {spec.label!r} lies on axis")

    qdelta = float(data.qpos[spec.qpos_address]) - spec.qpos0
    signed_wound_angle = (
        spec.reserve_sign * spec.reserve_length / spec.spool_radius + qdelta
    )
    # Positive spool angle is defined about the joint axis. The material fixed
    # point already rotates with the body, so the contact arc traverses the
    # opposite signed angle to reach the stationary free-cable exit.
    arc_angle = -axis_alignment * signed_wound_angle
    expected_exit = (
        center
        + fixed_axial * cylinder_axis
        + spec.spool_radius
        * _rotate_vector(fixed_direction, cylinder_axis, arc_angle)
    )
    return SpoolDebugGeometry(
        center=center,
        cylinder_axis=cylinder_axis,
        joint_axis=joint_axis,
        axis_alignment=axis_alignment,
        fixed=fixed,
        exit_point=exit_point,
        fixed_axial=fixed_axial,
        fixed_direction=fixed_direction,
        signed_wound_angle=signed_wound_angle,
        arc_angle=arc_angle,
        expected_exit=expected_exit,
    )


def draw_spool_debug_visuals(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    scene: mujoco.MjvScene | None,
    specs: list[SpoolDebugSpec],
    *,
    segments_per_turn: int,
) -> None:
    if scene is None:
        return
    fixed_rgba = np.array([0.20, 1.0, 0.35, 1.0], dtype=np.float64)
    exit_rgba = np.array([1.0, 0.82, 0.05, 1.0], dtype=np.float64)
    expected_rgba = np.array([0.0, 0.88, 0.95, 1.0], dtype=np.float64)
    residual_rgba = np.array([1.0, 0.1, 0.75, 0.95], dtype=np.float64)
    axis_rgba = np.array([0.75, 0.78, 0.82, 0.9], dtype=np.float64)
    tendon_rgba = np.asarray(model.tendon_rgba, dtype=np.float64).reshape(-1, 4)

    drawn_axes: set[int] = set()
    for spec in specs:
        geometry = _spool_debug_geometry(model, data, spec)
        visual_radius = spec.spool_radius + 0.0005
        point_count = max(
            9,
            int(
                np.ceil(
                    abs(geometry.arc_angle)
                    / (2.0 * np.pi)
                    * max(segments_per_turn, 8)
                )
            )
            + 1,
        )
        arc = [
            geometry.center
            + geometry.fixed_axial * geometry.cylinder_axis
            + visual_radius
            * _rotate_vector(
                geometry.fixed_direction,
                geometry.cylinder_axis,
                geometry.arc_angle * fraction,
            )
            for fraction in np.linspace(0.0, 1.0, point_count)
        ]
        color = tendon_rgba[spec.tendon_id].copy()
        color[3] = 1.0
        _draw_arc(scene, arc, 4.0, color)

        _sphere(scene, geometry.fixed, 0.0055, fixed_rgba)
        _sphere(scene, geometry.exit_point, 0.0050, exit_rgba)
        _sphere(scene, geometry.expected_exit, 0.0035, expected_rgba)
        _connector(
            scene,
            mujoco.mjtGeom.mjGEOM_LINE,
            geometry.expected_exit,
            geometry.exit_point,
            3.0,
            residual_rgba,
        )
        _connector(
            scene,
            mujoco.mjtGeom.mjGEOM_LINE,
            geometry.center
            + geometry.fixed_axial * geometry.cylinder_axis,
            geometry.fixed,
            2.0,
            fixed_rgba,
        )
        phase_error_mm = 1000.0 * float(
            np.linalg.norm(geometry.expected_exit - geometry.exit_point)
        )
        turns = geometry.signed_wound_angle / (2.0 * np.pi)
        label_offset = (
            0.012 * geometry.cylinder_axis + np.array([0.0, 0.0, 0.010])
        )
        _label(
            scene,
            geometry.fixed + label_offset,
            f"{spec.label} FIXED",
            fixed_rgba,
        )
        _label(
            scene,
            geometry.exit_point + label_offset,
            f"{spec.label} EXIT",
            exit_rgba,
        )
        _label(
            scene,
            geometry.center
            - 0.015 * geometry.cylinder_axis
            + np.array([0.0, 0.0, -0.020]),
            (
                f"{spec.label}: turns={turns:+.3f} "
                f"phase_error={phase_error_mm:.3f}mm"
            ),
            color,
        )

        if spec.geom_id not in drawn_axes:
            _connector(
                scene,
                mujoco.mjtGeom.mjGEOM_ARROW,
                geometry.center - 0.075 * geometry.joint_axis,
                geometry.center + 0.075 * geometry.joint_axis,
                0.004,
                axis_rgba,
            )
            drawn_axes.add(spec.geom_id)


def ramp(time_s: float, start: float, end: float, final: float) -> float:
    if time_s <= start:
        return 0.0
    if time_s >= end:
        return final
    phase = (time_s - start) / (end - start)
    smooth_phase = phase**3 * (10.0 + phase * (-15.0 + 6.0 * phase))
    return final * smooth_phase


def resolve_model_path(model: str) -> Path:
    model_path = Path(model)
    if not model_path.is_absolute():
        model_path = ROOT / model_path
    return model_path.resolve()


def _surface_wrap_geom_names(model_path: Path) -> set[str]:
    root = ET.parse(model_path).getroot()
    names: set[str] = set()
    for instance in root.findall("./extension/plugin/instance"):
        config = {
            item.attrib["key"]: item.attrib["value"]
            for item in instance.findall("config")
        }
        if config.get("route_mode") == "surface":
            names.update(config.get("wrap_geoms", "").split())
    return names


def enable_route_debug_visuals(model: mujoco.MjModel, model_path: Path) -> None:
    if model.nuser_site:
        for site_id in range(model.nsite):
            role = int(round(float(model.site_user[site_id, 0])))
            if role == 1:
                model.site_size[site_id, 0] = max(float(model.site_size[site_id, 0]), 0.003)
                model.site_rgba[site_id, 3] = 1.0
            elif role == 2:
                model.site_size[site_id, 0] = 0.0035
                model.site_rgba[site_id] = (1.0, 0.82, 0.05, 1.0)
            elif role == 3:
                model.site_size[site_id, 0] = 0.0035
                model.site_rgba[site_id] = (0.0, 0.82, 0.86, 1.0)
    wrap_geom_names = _surface_wrap_geom_names(model_path)
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if "route_proxy" in name or name in wrap_geom_names:
            model.geom_rgba[geom_id] = (0.12, 0.72, 0.34, 0.22)


def main() -> int:
    parser = argparse.ArgumentParser(description="Open an interactive viewer for the C++ cable plugin demo")
    parser.add_argument(
        "--plugin",
        required=True,
        help="Path to the standalone libcable_unilateral shared library",
    )
    parser.add_argument(
        "--model",
        default="cable_plugin_demos/09_cpp_plugin_free_hanging_single_pulley.xml",
        help="Path to the MJCF model",
    )
    parser.add_argument("--ctrl-final", type=float, default=0.8, help="Final plugin actuator control value")
    parser.add_argument(
        "--ctrl-ramp-start",
        type=float,
        default=0.5,
        help="Simulation time at which automatic control starts (default: 0.5 s)",
    )
    parser.add_argument(
        "--ctrl-ramp-end",
        type=float,
        default=3.0,
        help="Simulation time at which automatic control reaches --ctrl-final (default: 3.0 s)",
    )
    parser.add_argument(
        "--auto-actuator",
        help="Name of the actuator ramped automatically; defaults to actuator 0",
    )
    parser.add_argument(
        "--no-auto-control",
        action="store_true",
        help="Keep every actuator control at its model default instead of ramping actuator 0",
    )
    parser.add_argument(
        "--release-other-actuators",
        action="store_true",
        help="Release every non-selected actuator by a fraction of the selected contraction",
    )
    parser.add_argument(
        "--release-other-ratio",
        type=float,
        default=0.25,
        help="Release ratio used with --release-other-actuators (default: 0.25)",
    )
    parser.add_argument("--duration", type=float, default=12.0, help="Seconds before auto-resetting the demo loop")
    parser.add_argument(
        "--no-wrap-visual",
        action="store_true",
        help="Disable the custom wrapped-cable visual overlay and show MuJoCo's native tendon rendering",
    )
    parser.add_argument(
        "--show-native-tendons",
        action="store_true",
        help="Keep MuJoCo's native tendon rendering visible underneath the wrapped-cable overlay",
    )
    parser.add_argument("--visual-cable-width", type=float, default=0.0025, help="Base capsule radius for straight cable spans")
    parser.add_argument("--arc-line-width", type=float, default=3.0, help="Pixel width for smooth wrapped-cylinder arc overlays")
    parser.add_argument("--arc-segments", type=int, default=32, help="Number of visual line segments for each wrapped cylinder arc")
    parser.add_argument(
        "--show-tension-arrows",
        action="store_true",
        help="Show small force-direction arrows on the custom cable overlay",
    )
    parser.add_argument(
        "--show-cable-state",
        action="store_true",
        help="Draw live L/L_free/strain/tension/taut labels beside every plugin cable",
    )
    parser.add_argument(
        "--no-slack-color",
        action="store_true",
        help="Keep each tendon's XML color when slack instead of changing it to gray",
    )
    parser.add_argument(
        "--show-route-debug",
        action="store_true",
        help="Show role-1 endpoints, yellow role-2 hints, cyan role-3 guides, and green route proxies",
    )
    parser.add_argument(
        "--show-contact-debug",
        action="store_true",
        help="Show MuJoCo contact points and contact-force arrows",
    )
    parser.add_argument(
        "--show-spool-debug",
        action="store_true",
        help=(
            "Draw XML-defined drum FIXED/EXIT markers, live wound arcs, "
            "right-hand axes, and phase-error labels"
        ),
    )
    parser.add_argument(
        "--spool-debug-segments-per-turn",
        type=int,
        default=64,
        help="Line segments per complete debug winding turn (default: 64)",
    )
    parser.add_argument(
        "--no-spool-wrap-visual",
        action="store_true",
        help="Disable the visual surface-contact arc on the winch spool",
    )
    args = parser.parse_args()
    if args.release_other_ratio < 0:
        parser.error("--release-other-ratio must be nonnegative")
    if args.ctrl_ramp_start < 0:
        parser.error("--ctrl-ramp-start must be nonnegative")
    if args.ctrl_ramp_end <= args.ctrl_ramp_start:
        parser.error("--ctrl-ramp-end must be greater than --ctrl-ramp-start")
    if args.spool_debug_segments_per_turn < 8:
        parser.error("--spool-debug-segments-per-turn must be at least 8")

    plugin_path = Path(args.plugin).resolve()
    model_path = resolve_model_path(args.model)
    mujoco.mj_loadPluginLibrary(str(plugin_path))
    try:
        model = mujoco.MjModel.from_xml_path(str(model_path))
    except ValueError as exc:
        message = str(exc)
        if "plugin mujoco.cable.unilateral not found" in message:
            print(
                "Could not register mujoco.cable.unilateral in the Python MuJoCo runtime.\n"
                f"Plugin attempted: {plugin_path}\n"
                "\n"
                "For this Python viewer, build and pass the standalone plugin, for example:\n"
                "  MUJOCO_PYTHON_PACKAGE_DIR=\"$(conda run -n rope_plugin python -c "
                "'from pathlib import Path; import mujoco; print(Path(mujoco.__file__).resolve().parent)')\"\n"
                "  conda run -n rope_plugin cmake -S . -B build/cable_surface "
                "-DMUJOCO_PYTHON_PACKAGE_DIR=\"$MUJOCO_PYTHON_PACKAGE_DIR\"\n"
                "  conda run -n rope_plugin cmake --build build/cable_surface\n"
                "  conda run -n rope_plugin mjpython scripts/view_cpp_plugin_demo.py "
                "--plugin build/cable_surface/plugin/libcable_unilateral.dylib\n"
                "\n"
                "If you want to view the MuJoCo source-tree plugin "
                "(build/mujoco-source-tree/lib/libcable.dylib), use:\n"
                "  scripts/view_source_tree_simulate.sh\n",
                file=sys.stderr,
            )
        raise
    if args.show_route_debug:
        enable_route_debug_visuals(model, model_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    cable_bindings = cable_sensor_bindings(model, model_path)
    spool_debug_specs = _spool_debug_specs(model, model_path)
    if args.show_spool_debug and not spool_debug_specs:
        raise ValueError(
            "model has no custom/text entries named spool_debug_*"
        )
    base_tendon_rgba = np.asarray(model.tendon_rgba, dtype=np.float64).copy()
    auto_actuator_id = 0
    if args.auto_actuator:
        auto_actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, args.auto_actuator
        )
        if auto_actuator_id < 0:
            raise ValueError(f"actuator not found: {args.auto_actuator}")
    # Surface-route seed tendons use a near-zero legal width. Their runtime
    # envelope is drawn by the plugin visualize callback, not from wrap_xpos.
    visual_tendon_ids = [
        tendon_id
        for tendon_id in range(model.ntendon)
        if float(model.tendon_width[tendon_id]) > 1e-8
    ]
    wrap_visual = not args.no_wrap_visual and bool(visual_tendon_ids)
    if wrap_visual and not args.show_native_tendons:
        for tendon_id in visual_tendon_ids:
            model.tendon_width[tendon_id] = 0.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        if args.show_contact_debug:
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        while viewer.is_running():
            if data.time >= args.duration:
                mujoco.mj_resetData(model, data)
            if model.nu and not args.no_auto_control:
                commanded = ramp(
                    data.time,
                    start=args.ctrl_ramp_start,
                    end=args.ctrl_ramp_end,
                    final=args.ctrl_final,
                )
                data.ctrl[auto_actuator_id] = commanded
                if args.release_other_actuators:
                    for actuator_id in range(model.nu):
                        if actuator_id != auto_actuator_id:
                            data.ctrl[actuator_id] = (
                                -args.release_other_ratio * commanded
                            )
            step_start = time.time()
            mujoco.mj_step(model, data)
            cable_states = read_cable_visual_states(data, cable_bindings)
            if not args.no_slack_color:
                update_cable_state_colors(model, cable_states, base_tendon_rgba)
            if wrap_visual:
                draw_wrapped_tendon_visuals(
                    model,
                    data,
                    viewer.user_scn,
                    tendon_ids=visual_tendon_ids,
                    cable_width=args.visual_cable_width,
                    arc_line_width=args.arc_line_width,
                    arc_segments=args.arc_segments,
                    show_tension_arrows=args.show_tension_arrows,
                    show_spool_wrap=not args.no_spool_wrap_visual,
                    spool_geom_name="winch_drum_visual",
                    spool_anchor_site="site_spool_anchor",
                )
            elif (
                args.show_cable_state or args.show_spool_debug
            ) and viewer.user_scn is not None:
                viewer.user_scn.ngeom = 0
            if args.show_spool_debug:
                draw_spool_debug_visuals(
                    model,
                    data,
                    viewer.user_scn,
                    spool_debug_specs,
                    segments_per_turn=args.spool_debug_segments_per_turn,
                )
            if args.show_cable_state:
                draw_cable_state_labels(
                    model, data, viewer.user_scn, cable_states
                )
            viewer.sync()
            sleep_time = model.opt.timestep - (time.time() - step_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
