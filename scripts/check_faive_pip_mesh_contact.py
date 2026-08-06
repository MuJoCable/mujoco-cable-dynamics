#!/usr/bin/env python3
"""Audit Demo 25 shell contact, cable penetration, and endpoint placement."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

try:
    from check_saddle_route_surface_contact import (
        mesh_triangles_world,
        point_inside_mesh,
        point_triangle_distance,
        sample_segment,
    )
    from prepare_faive_pip_outer_surfaces import triangles_intersect
    from render_cpp_demo_screenshots import surface_line_segments
except ModuleNotFoundError:
    from scripts.check_saddle_route_surface_contact import (
        mesh_triangles_world,
        point_inside_mesh,
        point_triangle_distance,
        sample_segment,
    )
    from scripts.prepare_faive_pip_outer_surfaces import triangles_intersect
    from scripts.render_cpp_demo_screenshots import surface_line_segments


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLUGIN = ROOT / "build/plugin/libcable_unilateral.dylib"
DEFAULT_MODEL = ROOT / "cable_plugin_demos/25_faive_index_pip_surface_cable.xml"
ROUTE_GEOMS = ("proximal_route_surface", "distal_route_surface")
VISUAL_GEOMS = ("proximal_visual_mesh", "distal_visual_mesh")
LIGAMENT_TENDONS = (
    "ligament_right_upper_seed",
    "ligament_right_lower_seed",
)
PENETRATION_CLEARANCE_M = 1e-7


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    value = mujoco.mj_name2id(model, kind, name)
    if value < 0:
        raise ValueError(f"MuJoCo object {name!r} was not found")
    return value


def cross_mesh_intersections(
    first: np.ndarray, second: np.ndarray, tolerance: float = 1e-10
) -> list[tuple[int, int]]:
    """Return exact triangle intersections after an AABB broad phase."""
    first_lower = first.min(axis=1)
    first_upper = first.max(axis=1)
    second_lower = second.min(axis=1)
    second_upper = second.max(axis=1)
    centers = np.concatenate(
        (0.5 * (first_lower + first_upper), 0.5 * (second_lower + second_upper)),
        axis=0,
    )
    sweep_axis = int(np.argmax(np.ptp(centers, axis=0)))
    second_order = np.argsort(second_lower[:, sweep_axis])
    sorted_second_lower = second_lower[second_order, sweep_axis]
    faces = np.asarray(((0, 1, 2), (3, 4, 5)), dtype=np.int64)
    intersections: list[tuple[int, int]] = []
    for first_index in range(len(first)):
        stop = int(
            np.searchsorted(
                sorted_second_lower,
                first_upper[first_index, sweep_axis] + tolerance,
                side="right",
            )
        )
        candidates = second_order[:stop]
        candidates = candidates[
            second_upper[candidates, sweep_axis]
            >= first_lower[first_index, sweep_axis] - tolerance
        ]
        if len(candidates) == 0:
            continue
        overlap = np.logical_and(
            first_lower[first_index] <= second_upper[candidates] + tolerance,
            second_lower[candidates] <= first_upper[first_index] + tolerance,
        ).all(axis=1)
        for second_index in candidates[overlap]:
            vertices = np.concatenate(
                (first[first_index], second[int(second_index)]), axis=0
            )
            if triangles_intersect(vertices, faces[0], faces[1], tolerance):
                intersections.append((first_index, int(second_index)))
    return intersections


def plugin_route_status(model: mujoco.MjModel, data: mujoco.MjData) -> int:
    status = 0
    for sensor in range(model.nsensor):
        if (
            model.sensor_type[sensor] == mujoco.mjtSensor.mjSENS_PLUGIN
            and model.sensor_dim[sensor] >= 12
        ):
            address = int(model.sensor_adr[sensor])
            status = max(status, int(round(data.sensordata[address + 8])))
    return status


def endpoint_rows(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    triangles: dict[str, np.ndarray],
    geom_ids: dict[str, int],
) -> list[dict[str, Any]]:
    body_surface = {
        int(model.geom_bodyid[geom_id]): name for name, geom_id in geom_ids.items()
    }
    rows = []
    for site_id in range(model.nsite):
        if model.nuser_site < 1 or int(round(model.site_user[site_id, 0])) != 1:
            continue
        body_id = int(model.site_bodyid[site_id])
        surface = body_surface.get(body_id)
        if surface is None:
            continue
        point = np.asarray(data.site_xpos[site_id], dtype=float)
        mesh = triangles[surface]
        rows.append(
            {
                "site": mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_SITE, site_id
                ),
                "body_id": body_id,
                "surface": surface,
                "inside": point_inside_mesh(point, mesh),
                "surface_distance_mm": 1000.0
                * point_triangle_distance(point, mesh),
                "world_position_m": point.tolist(),
            }
        )
    return rows


def route_penetration_rows(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    triangles: dict[str, np.ndarray],
    samples: int,
) -> list[dict[str, Any]]:
    rows = []
    for segment, (tendon_id, first, second, _) in enumerate(
        surface_line_segments(model, data)
    ):
        points = sample_segment(first, second, samples)
        penetrating_surfaces = []
        for name, mesh in triangles.items():
            penetrating = 0
            for point in points:
                distance = point_triangle_distance(point, mesh)
                if distance > PENETRATION_CLEARANCE_M and point_inside_mesh(
                    point, mesh
                ):
                    penetrating += 1
            if penetrating:
                penetrating_surfaces.append(
                    {"surface": name, "penetrating_samples": penetrating}
                )
        if penetrating_surfaces:
            rows.append(
                {
                    "segment": segment,
                    "tendon": mujoco.mj_id2name(
                        model, mujoco.mjtObj.mjOBJ_TENDON, tendon_id
                    ),
                    "length_mm": 1000.0 * float(np.linalg.norm(second - first)),
                    "surfaces": penetrating_surfaces,
                }
            )
    return rows


def interface_route_rows(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    triangles: dict[str, np.ndarray],
) -> list[dict[str, Any]]:
    """Measure the free span and tangent continuity at both surfaces."""
    segments_by_tendon: dict[int, list[tuple[np.ndarray, np.ndarray]]] = {}
    for tendon_id, first, second, _ in surface_line_segments(model, data):
        segments_by_tendon.setdefault(tendon_id, []).append((first, second))

    proximal = triangles[ROUTE_GEOMS[0]]
    distal = triangles[ROUTE_GEOMS[1]]
    rows = []
    for tendon_name in LIGAMENT_TENDONS:
        tendon_id = object_id(model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name)
        tendon_segments = segments_by_tendon.get(tendon_id, [])
        candidates = []
        for segment_index, (first, second) in enumerate(tendon_segments):
            direct = point_triangle_distance(
                first, proximal
            ) + point_triangle_distance(second, distal)
            reverse = point_triangle_distance(
                first, distal
            ) + point_triangle_distance(second, proximal)
            candidates.append(
                (
                    min(direct, reverse),
                    float(np.linalg.norm(second - first)),
                    segment_index,
                )
            )
        if not candidates:
            continue
        residual, gap, bridge_index = min(candidates)
        bridge_first, bridge_second = tendon_segments[bridge_index]

        def surface_tangent_point(before_bridge: bool) -> np.ndarray | None:
            remaining = 0.0005
            cursor = bridge_first.copy() if before_bridge else bridge_second.copy()
            indices = (
                range(bridge_index - 1, -1, -1)
                if before_bridge
                else range(bridge_index + 1, len(tendon_segments))
            )
            for index in indices:
                first, second = tendon_segments[index]
                target = first if before_bridge else second
                if min(
                    float(np.linalg.norm(first - cursor)),
                    float(np.linalg.norm(second - cursor)),
                ) > 1e-6:
                    return None
                available = float(np.linalg.norm(target - cursor))
                if available >= remaining and available > 1e-16:
                    return cursor + (target - cursor) * remaining / available
                remaining -= available
                cursor = target
            return cursor

        def tangent_angle(first: np.ndarray, vertex: np.ndarray, second: np.ndarray):
            incoming = vertex - first
            outgoing = second - vertex
            denominator = float(np.linalg.norm(incoming) * np.linalg.norm(outgoing))
            if denominator <= 1e-16:
                return None
            cosine = float(np.clip(np.dot(incoming, outgoing) / denominator, -1, 1))
            return float(np.degrees(np.arccos(cosine)))

        first_continuation = surface_tangent_point(True)
        second_continuation = surface_tangent_point(False)
        first_angle = (
            tangent_angle(first_continuation, bridge_first, bridge_second)
            if first_continuation is not None
            else None
        )
        second_angle = (
            tangent_angle(bridge_first, bridge_second, second_continuation)
            if second_continuation is not None
            else None
        )
        rows.append(
            {
                "tendon": tendon_name,
                "interface_gap_mm": 1000.0 * gap,
                "endpoint_surface_residual_mm": 1000.0 * residual,
                "departure_tangent_angle_deg": first_angle,
                "arrival_tangent_angle_deg": second_angle,
            }
        )
    return rows


def analyze(
    plugin: Path,
    model_path: Path,
    steps: int,
    flexor: float,
    extensor: float,
    route_samples: int,
    shell_samples: int,
) -> dict[str, Any]:
    mujoco.mj_loadPluginLibrary(str(plugin))
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    geom_ids = {
        name: object_id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in ROUTE_GEOMS
    }
    visual_geom_ids = {
        name: object_id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in VISUAL_GEOMS
    }
    flexor_id = object_id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "pip_flexor_command"
    )
    extensor_id = object_id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "pip_extensor_command"
    )
    # The audit intentionally permits candidate strokes beyond the XML's
    # validated operating range.  This does not change the demo defaults; it
    # lets the checker identify the first geometric or route failure before a
    # larger range is accepted in MJCF.
    model.actuator_ctrlrange[flexor_id, 1] = max(
        float(model.actuator_ctrlrange[flexor_id, 1]), flexor
    )
    model.actuator_ctrlrange[extensor_id, 1] = max(
        float(model.actuator_ctrlrange[extensor_id, 1]), extensor
    )
    sample_steps = set(
        int(value)
        for value in np.linspace(0, steps, max(2, shell_samples), dtype=int)
    )
    snapshots = []
    minimum_contact_distance = 0.0
    maximum_contacts = 0
    maximum_route_status = 0
    maximum_route_status_by_sensor: dict[str, int] = {}
    first_invalid_route_by_sensor: dict[str, dict[str, float | int]] = {}

    for step in range(steps + 1):
        if step:
            data.ctrl[flexor_id] = flexor
            data.ctrl[extensor_id] = extensor
            mujoco.mj_step(model, data)
        else:
            mujoco.mj_forward(model, data)
        maximum_contacts = max(maximum_contacts, int(data.ncon))
        maximum_route_status = max(
            maximum_route_status, plugin_route_status(model, data)
        )
        for sensor in range(model.nsensor):
            if model.sensor_dim[sensor] < 12:
                continue
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor)
            address = int(model.sensor_adr[sensor])
            status = int(round(data.sensordata[address + 8]))
            maximum_route_status_by_sensor[name] = max(
                maximum_route_status_by_sensor.get(name, 0), status
            )
            if status >= 2 and name not in first_invalid_route_by_sensor:
                first_invalid_route_by_sensor[name] = {
                    "step": step,
                    "time_s": float(data.time),
                    "length_m": float(data.sensordata[address]),
                    "tangent_residual": float(data.sensordata[address + 9]),
                }
        if data.ncon:
            minimum_contact_distance = min(
                minimum_contact_distance,
                min(float(data.contact[index].dist) for index in range(data.ncon)),
            )
        if step in sample_steps:
            triangles = {
                name: mesh_triangles_world(model, data, geom_id)
                for name, geom_id in geom_ids.items()
            }
            intersections = cross_mesh_intersections(
                triangles[ROUTE_GEOMS[0]], triangles[ROUTE_GEOMS[1]]
            )
            visual_triangles = {
                name: mesh_triangles_world(model, data, geom_id)
                for name, geom_id in visual_geom_ids.items()
            }
            visual_intersections = cross_mesh_intersections(
                visual_triangles[VISUAL_GEOMS[0]],
                visual_triangles[VISUAL_GEOMS[1]],
            )
            interface_routes = interface_route_rows(model, data, triangles)
            snapshots.append(
                {
                    "step": step,
                    "time_s": float(data.time),
                    "contacts": int(data.ncon),
                    "triangle_intersection_count": len(intersections),
                    "first_intersection_pairs": intersections[:8],
                    "visual_triangle_intersection_count": len(
                        visual_intersections
                    ),
                    "first_visual_intersection_pairs": visual_intersections[:8],
                    "interface_routes": interface_routes,
                }
            )

    triangles = {
        name: mesh_triangles_world(model, data, geom_id)
        for name, geom_id in geom_ids.items()
    }
    endpoints = endpoint_rows(model, data, triangles, geom_ids)
    penetrating_routes = route_penetration_rows(
        model, data, triangles, route_samples
    )
    def portable_path(path: Path) -> str:
        try:
            return str(path.resolve().relative_to(ROOT.resolve()))
        except ValueError:
            return path.name

    return {
        "model": portable_path(model_path),
        "plugin": portable_path(plugin),
        "steps": steps,
        "controls_m": {"flexor": flexor, "extensor": extensor},
        "snapshots": snapshots,
        "endpoints": endpoints,
        "penetrating_route_segments": penetrating_routes,
        "summary": {
            "maximum_shell_intersections": max(
                row["triangle_intersection_count"] for row in snapshots
            ),
            "maximum_visual_mesh_intersections": max(
                row["visual_triangle_intersection_count"] for row in snapshots
            ),
            "maximum_interface_gap_mm": max(
                route["interface_gap_mm"]
                for row in snapshots
                for route in row["interface_routes"]
            ),
            "maximum_interface_surface_residual_mm": max(
                route["endpoint_surface_residual_mm"]
                for row in snapshots
                for route in row["interface_routes"]
            ),
            "maximum_interface_tangent_angle_deg": max(
                angle
                for row in snapshots
                for route in row["interface_routes"]
                for angle in (
                    route["departure_tangent_angle_deg"],
                    route["arrival_tangent_angle_deg"],
                )
                if angle is not None
            ),
            "minimum_contact_distance_mm": 1000.0 * minimum_contact_distance,
            "maximum_contacts": maximum_contacts,
            "maximum_route_status": maximum_route_status,
            "maximum_route_status_by_sensor": maximum_route_status_by_sensor,
            "first_invalid_route_by_sensor": first_invalid_route_by_sensor,
            "inside_endpoint_count": sum(row["inside"] for row in endpoints),
            "penetrating_route_segment_count": len(penetrating_routes),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--steps", type=int, default=7500)
    parser.add_argument("--flexor", type=float, default=0.003)
    parser.add_argument("--extensor", type=float, default=0.0027)
    parser.add_argument("--route-samples", type=int, default=41)
    parser.add_argument("--shell-samples", type=int, default=7)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    report = analyze(
        args.plugin.resolve(),
        args.model.resolve(),
        max(0, args.steps),
        args.flexor,
        args.extensor,
        max(3, args.route_samples),
        max(2, args.shell_samples),
    )
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.out:
        args.out.resolve().write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
