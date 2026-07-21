#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

try:
    from render_cpp_demo_screenshots import surface_line_segments
except ModuleNotFoundError:
    from scripts.render_cpp_demo_screenshots import surface_line_segments


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "cable_plugin_demos/20_cpp_plugin_controlled_saddle_joint.xml"
DEFAULT_PLUGIN = ROOT / "build/cable_surface/plugin/libcable_unilateral.dylib"
WRAP_GEOMS = ("proximal_saddle_visual", "distal_saddle_visual")
PENETRATION_CLEARANCE_M = 1e-7


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    value = mujoco.mj_name2id(model, kind, name)
    if value < 0:
        raise ValueError(f"MuJoCo object {name!r} was not found")
    return value


def mesh_triangles_world(
    model: mujoco.MjModel, data: mujoco.MjData, geom_id: int
) -> np.ndarray:
    mesh_id = int(model.geom_dataid[geom_id])
    vertex_address = int(model.mesh_vertadr[mesh_id])
    vertex_count = int(model.mesh_vertnum[mesh_id])
    face_address = int(model.mesh_faceadr[mesh_id])
    face_count = int(model.mesh_facenum[mesh_id])
    vertices = np.asarray(
        model.mesh_vert[vertex_address : vertex_address + vertex_count],
        dtype=float,
    )
    faces = np.asarray(
        model.mesh_face[face_address : face_address + face_count], dtype=int
    )
    rotation = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
    translation = np.asarray(data.geom_xpos[geom_id], dtype=float)
    world_vertices = vertices @ rotation.T + translation
    return world_vertices[faces]


def point_segment_distance(
    points: np.ndarray, first: np.ndarray, second: np.ndarray
) -> np.ndarray:
    delta = second - first
    denominator = np.einsum("ij,ij->i", delta, delta)
    parameter = np.divide(
        np.einsum("ij,ij->i", points - first, delta),
        denominator,
        out=np.zeros_like(denominator),
        where=denominator > 1e-24,
    )
    parameter = np.clip(parameter, 0.0, 1.0)
    closest = first + parameter[:, None] * delta
    return np.linalg.norm(points - closest, axis=1)


def point_triangle_distance(point: np.ndarray, triangles: np.ndarray) -> float:
    first = triangles[:, 0]
    second = triangles[:, 1]
    third = triangles[:, 2]
    edge0 = second - first
    edge1 = third - first
    normal = np.cross(edge0, edge1)
    normal_norm = np.linalg.norm(normal, axis=1)
    unit_normal = np.divide(
        normal,
        normal_norm[:, None],
        out=np.zeros_like(normal),
        where=normal_norm[:, None] > 1e-24,
    )
    relative = point - first
    plane_signed = np.einsum("ij,ij->i", relative, unit_normal)
    projection = point - plane_signed[:, None] * unit_normal

    dot00 = np.einsum("ij,ij->i", edge0, edge0)
    dot01 = np.einsum("ij,ij->i", edge0, edge1)
    dot11 = np.einsum("ij,ij->i", edge1, edge1)
    projected = projection - first
    dot20 = np.einsum("ij,ij->i", projected, edge0)
    dot21 = np.einsum("ij,ij->i", projected, edge1)
    denominator = dot00 * dot11 - dot01 * dot01
    barycentric1 = np.divide(
        dot11 * dot20 - dot01 * dot21,
        denominator,
        out=np.full_like(denominator, np.inf),
        where=np.abs(denominator) > 1e-24,
    )
    barycentric2 = np.divide(
        dot00 * dot21 - dot01 * dot20,
        denominator,
        out=np.full_like(denominator, np.inf),
        where=np.abs(denominator) > 1e-24,
    )
    inside = (
        (barycentric1 >= -1e-10)
        & (barycentric2 >= -1e-10)
        & (barycentric1 + barycentric2 <= 1.0 + 1e-10)
    )
    plane_distance = np.where(inside, np.abs(plane_signed), np.inf)
    repeated = np.broadcast_to(point, first.shape)
    edge_distance = np.minimum.reduce(
        (
            point_segment_distance(repeated, first, second),
            point_segment_distance(repeated, second, third),
            point_segment_distance(repeated, third, first),
        )
    )
    return float(np.min(np.minimum(plane_distance, edge_distance)))


def point_inside_mesh(point: np.ndarray, triangles: np.ndarray) -> bool:
    relative = triangles - point[None, None, :]
    first = relative[:, 0]
    second = relative[:, 1]
    third = relative[:, 2]
    numerator = np.einsum("ij,ij->i", first, np.cross(second, third))
    first_norm = np.linalg.norm(first, axis=1)
    second_norm = np.linalg.norm(second, axis=1)
    third_norm = np.linalg.norm(third, axis=1)
    denominator = (
        first_norm * second_norm * third_norm
        + np.einsum("ij,ij->i", first, second) * third_norm
        + np.einsum("ij,ij->i", second, third) * first_norm
        + np.einsum("ij,ij->i", third, first) * second_norm
    )
    winding = float(np.sum(2.0 * np.arctan2(numerator, denominator)))
    return abs(winding) > 2.0 * np.pi


def sample_segment(first: np.ndarray, second: np.ndarray, samples: int) -> np.ndarray:
    fractions = np.linspace(0.0, 1.0, samples)
    return first[None, :] + fractions[:, None] * (second - first)[None, :]


def analyze(
    plugin: Path,
    model_path: Path,
    steps: int,
    contraction: float,
    release_ratio: float,
    samples: int,
) -> dict[str, Any]:
    mujoco.mj_loadPluginLibrary(str(plugin))
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    for _ in range(steps):
        if model.nu:
            data.ctrl[0] = contraction
            for actuator_id in range(1, model.nu):
                data.ctrl[actuator_id] = -release_ratio * contraction
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    geom_ids = {
        name: object_id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in WRAP_GEOMS
    }
    triangles = {
        name: mesh_triangles_world(model, data, geom_id)
        for name, geom_id in geom_ids.items()
    }
    segments = surface_line_segments(model, data)
    rows: list[dict[str, Any]] = []
    for segment_index, (tendon_id, first, second, _) in enumerate(segments):
        tendon_name = (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TENDON, tendon_id)
            or f"tendon_{tendon_id}"
        )
        points = sample_segment(first, second, samples)
        distances = {
            name: np.asarray(
                [point_triangle_distance(point, mesh) for point in points]
            )
            for name, mesh in triangles.items()
        }
        inside = {
            name: np.asarray(
                [point_inside_mesh(point, mesh) for point in points], dtype=bool
            )
            for name, mesh in triangles.items()
        }
        endpoint_surface = min(
            WRAP_GEOMS,
            key=lambda name: max(float(distances[name][0]), float(distances[name][-1])),
        )
        endpoint_max = max(
            float(distances[endpoint_surface][0]),
            float(distances[endpoint_surface][-1]),
        )
        row = {
            "segment": segment_index,
            "tendon": tendon_name,
            "length_mm": 1000.0 * float(np.linalg.norm(second - first)),
            "nearest_surface_for_endpoints": endpoint_surface,
            "endpoint_max_distance_mm": 1000.0 * endpoint_max,
            "sample_max_distance_mm": 1000.0
            * float(np.max(distances[endpoint_surface])),
            "sample_min_distance_mm": 1000.0
            * float(np.min(distances[endpoint_surface])),
            "inside_sample_count": int(np.count_nonzero(inside[endpoint_surface])),
            "penetrating_sample_count": int(
                np.count_nonzero(
                    inside[endpoint_surface]
                    & (distances[endpoint_surface] > PENETRATION_CLEARANCE_M)
                )
            ),
            "starts_and_ends_on_same_surface": endpoint_max < 1e-5,
        }
        rows.append(row)

    surface_rows = [row for row in rows if row["starts_and_ends_on_same_surface"]]
    return {
        "model": str(model_path),
        "plugin": str(plugin),
        "steps": steps,
        "control": {
            "contraction_m": contraction,
            "release_ratio": release_ratio,
        },
        "segments": rows,
        "summary": {
            "segment_count": len(rows),
            "surface_segment_count": len(surface_rows),
            "maximum_surface_segment_gap_mm": max(
                (float(row["sample_max_distance_mm"]) for row in surface_rows),
                default=0.0,
            ),
            "penetrating_surface_segment_count": sum(
                int(row["penetrating_sample_count"] > 0) for row in surface_rows
            ),
            "penetration_clearance_m": PENETRATION_CLEARANCE_M,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Measure Demo 16/20 rendered cable segments against the actual STL meshes"
    )
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--steps", type=int, default=0)
    parser.add_argument("--contraction", type=float, default=0.0)
    parser.add_argument("--release-ratio", type=float, default=0.25)
    parser.add_argument("--samples", type=int, default=9)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    result = analyze(
        args.plugin.resolve(),
        args.model.resolve(),
        max(0, args.steps),
        args.contraction,
        args.release_ratio,
        max(3, args.samples),
    )
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.out:
        args.out.resolve().write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
