#!/usr/bin/env python3
"""Extract closed outer shells from the nonmanifold Faive PIP STL assets.

The source files contain duplicated faces and several edges shared by three or
four triangles.  The extractor removes exact duplicate faces, cuts the triangle
soup at nonmanifold edges, and enumerates combinations of the resulting
manifold patches.  It selects the connected, closed combination with the
largest enclosed volume and preserves its original triangle coordinates.

Two assets are emitted per source:

* ``*_outer.obj`` is the complete closed shell used by taut-obstacle routing.
* ``*_contact.obj`` is an open patch cut from that shell near the joint and is
  used by MuJoCo rigid-flex triangle contact.

Only NumPy is required.  All internal coordinates are metres; the source STL
files use millimetres.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict, deque
from dataclasses import dataclass
import json
from pathlib import Path
import struct

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ASSET_DIR = ROOT / "cable_plugin_demos/assets/faive_index_pip"
SOURCE_SCALE = 0.001
MERGE_TOLERANCE_NATIVE = 1e-6


@dataclass(frozen=True)
class SourceSpec:
    source: str
    outer: str
    contact: str
    joint_side: str


SOURCES = (
    SourceSpec("index_pp.stl", "index_pp_outer.obj", "index_pp_contact.obj", "max_y"),
    SourceSpec("index_mp.stl", "index_mp_outer.obj", "index_mp_contact.obj", "min_y"),
)


def read_binary_stl(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError(f"STL is too short: {path}")
    face_count = struct.unpack_from("<I", raw, 80)[0]
    expected = 84 + 50 * face_count
    if len(raw) != expected:
        raise ValueError(
            f"only binary STL is supported: expected {expected} bytes, found {len(raw)}"
        )
    records = np.frombuffer(
        raw,
        dtype=np.dtype(
            [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
        ),
        count=face_count,
        offset=84,
    )
    return np.asarray(records["vertices"], dtype=np.float64)


def indexed_unique_faces(
    triangles: np.ndarray, tolerance: float
) -> tuple[np.ndarray, np.ndarray, int]:
    flat = triangles.reshape(-1, 3)
    keys = np.rint(flat / tolerance).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    vertices = flat[first]
    faces = inverse.reshape(-1, 3)
    valid = np.logical_and.reduce(
        (faces[:, 0] != faces[:, 1], faces[:, 1] != faces[:, 2], faces[:, 2] != faces[:, 0])
    )
    faces = faces[valid]
    canonical = np.sort(faces, axis=1)
    _, first_face = np.unique(canonical, axis=0, return_index=True)
    unique_faces = faces[np.sort(first_face)]
    return vertices, unique_faces, int(len(faces) - len(unique_faces))


def edge_faces(faces: np.ndarray) -> dict[tuple[int, int], list[int]]:
    result: dict[tuple[int, int], list[int]] = defaultdict(list)
    for face_id, face in enumerate(faces):
        for first, second in (
            (face[0], face[1]),
            (face[1], face[2]),
            (face[2], face[0]),
        ):
            result[tuple(sorted((int(first), int(second))))].append(face_id)
    return dict(result)


class DisjointSet:
    def __init__(self, size: int):
        self.parent = np.arange(size, dtype=np.int64)

    def find(self, index: int) -> int:
        while self.parent[index] != index:
            self.parent[index] = self.parent[self.parent[index]]
            index = int(self.parent[index])
        return index

    def union(self, first: int, second: int) -> None:
        first_root = self.find(first)
        second_root = self.find(second)
        if first_root != second_root:
            self.parent[second_root] = first_root


def manifold_patches(
    faces: np.ndarray, adjacency: dict[tuple[int, int], list[int]]
) -> tuple[np.ndarray, list[int]]:
    sets = DisjointSet(len(faces))
    for adjacent in adjacency.values():
        if len(adjacent) == 2:
            sets.union(adjacent[0], adjacent[1])
    roots = [sets.find(face) for face in range(len(faces))]
    unique_roots = sorted(set(roots))
    root_to_patch = {root: patch for patch, root in enumerate(unique_roots)}
    face_patch = np.asarray([root_to_patch[root] for root in roots], dtype=np.int64)
    counts = np.bincount(face_patch, minlength=len(unique_roots))
    return face_patch, [int(count) for count in counts]


def orient_closed_faces(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    adjacency = edge_faces(faces)
    if any(len(adjacent) != 2 for adjacent in adjacency.values()):
        raise ValueError("selected shell is not closed and two-manifold")

    edge_signs: dict[tuple[int, int], list[tuple[int, int]]] = defaultdict(list)
    for face_id, face in enumerate(faces):
        for first, second in (
            (int(face[0]), int(face[1])),
            (int(face[1]), int(face[2])),
            (int(face[2]), int(face[0])),
        ):
            edge = tuple(sorted((first, second)))
            edge_signs[edge].append((face_id, 1 if (first, second) == edge else -1))

    orientation = np.zeros(len(faces), dtype=np.int8)
    orientation[0] = 1
    queue: deque[int] = deque([0])
    face_neighbors: dict[int, list[tuple[int, int, int]]] = defaultdict(list)
    for adjacent in edge_signs.values():
        (first_face, first_sign), (second_face, second_sign) = adjacent
        face_neighbors[first_face].append((second_face, first_sign, second_sign))
        face_neighbors[second_face].append((first_face, second_sign, first_sign))
    while queue:
        face = queue.popleft()
        for neighbor, face_sign, neighbor_sign in face_neighbors[face]:
            required = -orientation[face] * face_sign * neighbor_sign
            if orientation[neighbor] == 0:
                orientation[neighbor] = required
                queue.append(neighbor)
            elif orientation[neighbor] != required:
                raise ValueError("selected shell is non-orientable")
    if np.any(orientation == 0):
        raise ValueError("selected shell has more than one connected component")

    oriented = faces.copy()
    flipped = orientation < 0
    oriented[flipped, 1], oriented[flipped, 2] = (
        oriented[flipped, 2].copy(),
        oriented[flipped, 1].copy(),
    )
    volume = signed_volume(vertices, oriented)
    if volume < 0:
        oriented[:, [1, 2]] = oriented[:, [2, 1]]
    return oriented


def signed_volume(vertices: np.ndarray, faces: np.ndarray) -> float:
    triangles = vertices[faces]
    return float(
        np.einsum(
            "ij,ij->i", triangles[:, 0], np.cross(triangles[:, 1], triangles[:, 2])
        ).sum()
        / 6.0
    )


def connected_face_count(faces: np.ndarray) -> int:
    adjacency = edge_faces(faces)
    sets = DisjointSet(len(faces))
    for adjacent in adjacency.values():
        for face in adjacent[1:]:
            sets.union(adjacent[0], face)
    return len({sets.find(face) for face in range(len(faces))})


def closest_point_triangle(point: np.ndarray, triangle: np.ndarray) -> np.ndarray:
    a, b, c = triangle
    ab = b - a
    ac = c - a
    ap = point - a
    d1 = float(np.dot(ab, ap))
    d2 = float(np.dot(ac, ap))
    if d1 <= 0 and d2 <= 0:
        return a
    bp = point - b
    d3 = float(np.dot(ab, bp))
    d4 = float(np.dot(ac, bp))
    if d3 >= 0 and d4 <= d3:
        return b
    vc = d1 * d4 - d3 * d2
    if vc <= 0 and d1 >= 0 and d3 <= 0:
        return a + (d1 / (d1 - d3)) * ab
    cp = point - c
    d5 = float(np.dot(ab, cp))
    d6 = float(np.dot(ac, cp))
    if d6 >= 0 and d5 <= d6:
        return c
    vb = d5 * d2 - d1 * d6
    if vb <= 0 and d2 >= 0 and d6 <= 0:
        return a + (d2 / (d2 - d6)) * ac
    va = d3 * d6 - d5 * d4
    if va <= 0 and d4 - d3 >= 0 and d5 - d6 >= 0:
        return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b)
    denominator = 1.0 / (va + vb + vc)
    return a + vb * denominator * ab + vc * denominator * ac


def segment_triangle_intersection(
    start: np.ndarray,
    end: np.ndarray,
    triangle: np.ndarray,
    tolerance: float,
) -> np.ndarray | None:
    a, b, c = triangle
    normal = np.cross(b - a, c - a)
    normal_norm = float(np.linalg.norm(normal))
    if normal_norm <= np.finfo(float).tiny:
        return None
    plane_tolerance = tolerance * normal_norm
    start_distance = float(np.dot(normal, start - a))
    end_distance = float(np.dot(normal, end - a))
    if (start_distance > plane_tolerance and end_distance > plane_tolerance) or (
        start_distance < -plane_tolerance and end_distance < -plane_tolerance
    ):
        return None
    denominator = start_distance - end_distance
    if abs(denominator) <= plane_tolerance:
        return None
    parameter = start_distance / denominator
    if parameter < -tolerance or parameter > 1 + tolerance:
        return None
    point = start + np.clip(parameter, 0, 1) * (end - start)
    if np.linalg.norm(point - closest_point_triangle(point, triangle)) > tolerance:
        return None
    return point


def cross2(first: np.ndarray, second: np.ndarray, point: np.ndarray) -> float:
    return float(
        (second[0] - first[0]) * (point[1] - first[1])
        - (second[1] - first[1]) * (point[0] - first[0])
    )


def point_in_triangle_2d(
    point: np.ndarray, triangle: np.ndarray, tolerance: float
) -> bool:
    values = [
        cross2(triangle[0], triangle[1], point),
        cross2(triangle[1], triangle[2], point),
        cross2(triangle[2], triangle[0], point),
    ]
    return not (any(value < -tolerance for value in values) and any(value > tolerance for value in values))


def proper_segments_intersect_2d(
    a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray, tolerance: float
) -> bool:
    first = cross2(a, b, c)
    second = cross2(a, b, d)
    third = cross2(c, d, a)
    fourth = cross2(c, d, b)
    return (
        ((first > tolerance and second < -tolerance) or (first < -tolerance and second > tolerance))
        and ((third > tolerance and fourth < -tolerance) or (third < -tolerance and fourth > tolerance))
    )


def projected_triangle(triangle: np.ndarray, normal: np.ndarray) -> np.ndarray:
    omitted_axis = int(np.argmax(np.abs(normal)))
    return np.delete(triangle, omitted_axis, axis=1)


def coplanar_triangles_intersect(
    first: np.ndarray,
    second: np.ndarray,
    normal: np.ndarray,
    tolerance: float,
    first_shared_vertex: int,
    second_shared_vertex: int,
) -> bool:
    first_2d = projected_triangle(first, normal)
    second_2d = projected_triangle(second, normal)
    edge_lengths = [
        np.linalg.norm(triangle[(edge + 1) % 3] - triangle[edge])
        for triangle in (first, second)
        for edge in range(3)
    ]
    area_tolerance = tolerance * max(edge_lengths)
    for vertex in range(3):
        if vertex != first_shared_vertex and point_in_triangle_2d(
            first_2d[vertex], second_2d, area_tolerance
        ):
            return True
        if vertex != second_shared_vertex and point_in_triangle_2d(
            second_2d[vertex], first_2d, area_tolerance
        ):
            return True
    return any(
        proper_segments_intersect_2d(
            first_2d[first_edge],
            first_2d[(first_edge + 1) % 3],
            second_2d[second_edge],
            second_2d[(second_edge + 1) % 3],
            area_tolerance,
        )
        for first_edge in range(3)
        for second_edge in range(3)
    )


def triangles_intersect(
    vertices: np.ndarray,
    first_indices: np.ndarray,
    second_indices: np.ndarray,
    tolerance: float,
) -> bool:
    shared_pairs = [
        (first_vertex, second_vertex)
        for first_vertex in range(3)
        for second_vertex in range(3)
        if first_indices[first_vertex] == second_indices[second_vertex]
    ]
    shared_count = len(shared_pairs)
    first_shared_vertex, second_shared_vertex = shared_pairs[-1] if shared_pairs else (-1, -1)
    first = vertices[first_indices]
    second = vertices[second_indices]
    first_normal = np.cross(first[1] - first[0], first[2] - first[0])
    second_normal = np.cross(second[1] - second[0], second[2] - second[0])
    first_normal_norm = float(np.linalg.norm(first_normal))
    second_normal_norm = float(np.linalg.norm(second_normal))
    if first_normal_norm <= np.finfo(float).tiny or second_normal_norm <= np.finfo(float).tiny:
        return False
    parallel_measure = float(np.linalg.norm(np.cross(first_normal, second_normal)))
    coplanar = parallel_measure <= tolerance * first_normal_norm * second_normal_norm
    if coplanar:
        plane_tolerance = tolerance * first_normal_norm
        coplanar = all(
            abs(float(np.dot(first_normal, point - first[0]))) <= plane_tolerance
            for point in second
        )
    if shared_count == 3:
        return True
    if shared_count == 2:
        if not coplanar:
            return False
        shared_vertex_ids = set(int(index) for index in first_indices) & set(
            int(index) for index in second_indices
        )
        shared_points = vertices[np.asarray(sorted(shared_vertex_ids), dtype=np.int64)]
        first_unshared = next(
            vertex for vertex in range(3) if int(first_indices[vertex]) not in shared_vertex_ids
        )
        second_unshared = next(
            vertex for vertex in range(3) if int(second_indices[vertex]) not in shared_vertex_ids
        )
        omitted_axis = int(np.argmax(np.abs(first_normal)))
        project = lambda point: np.delete(point, omitted_axis)
        edge_first, edge_second = (project(point) for point in shared_points)
        first_side = cross2(edge_first, edge_second, project(first[first_unshared]))
        second_side = cross2(edge_first, edge_second, project(second[second_unshared]))
        side_tolerance = tolerance * float(np.linalg.norm(shared_points[0] - shared_points[1]))
        return (first_side > side_tolerance and second_side > side_tolerance) or (
            first_side < -side_tolerance and second_side < -side_tolerance
        )
    if coplanar:
        return coplanar_triangles_intersect(
            first,
            second,
            first_normal,
            tolerance,
            first_shared_vertex if shared_count == 1 else -1,
            second_shared_vertex if shared_count == 1 else -1,
        )
    shared_point = first[first_shared_vertex] if shared_count == 1 else None
    for edge in range(3):
        intersection = segment_triangle_intersection(
            first[edge], first[(edge + 1) % 3], second, tolerance
        )
        if intersection is not None and (
            shared_point is None or np.linalg.norm(intersection - shared_point) > 4 * tolerance
        ):
            return True
        intersection = segment_triangle_intersection(
            second[edge], second[(edge + 1) % 3], first, tolerance
        )
        if intersection is not None and (
            shared_point is None or np.linalg.norm(intersection - shared_point) > 4 * tolerance
        ):
            return True
    return False


def find_self_intersection(
    vertices: np.ndarray, faces: np.ndarray, tolerance: float
) -> tuple[int, int] | None:
    triangles = vertices[faces]
    lower = triangles.min(axis=1)
    upper = triangles.max(axis=1)
    for first in range(len(faces)):
        candidates = np.flatnonzero(
            np.logical_and.reduce(
                (
                    np.arange(len(faces)) > first,
                    np.all(upper[first] >= lower - tolerance, axis=1),
                    np.all(upper >= lower[first] - tolerance, axis=1),
                )
            )
        )
        for second in candidates:
            if triangles_intersect(vertices, faces[first], faces[second], tolerance):
                return first, int(second)
    return None


def choose_outer_shell(
    vertices: np.ndarray, faces: np.ndarray
) -> tuple[np.ndarray, dict[str, object]]:
    adjacency = edge_faces(faces)
    face_patch, patch_sizes = manifold_patches(faces, adjacency)
    patch_count = len(patch_sizes)
    if patch_count > 22:
        raise RuntimeError(
            f"{patch_count} manifold patches require an exponential search; expected at most 22"
        )
    edge_patch_incidence = [
        Counter(int(face_patch[face]) for face in adjacent)
        for adjacent in adjacency.values()
    ]

    candidates: list[tuple[float, int, int, np.ndarray]] = []
    for mask in range(1, 1 << patch_count):
        invalid = False
        for incidence in edge_patch_incidence:
            selected_incidence = sum(
                count for patch, count in incidence.items() if mask & (1 << patch)
            )
            if selected_incidence not in (0, 2):
                invalid = True
                break
        if invalid:
            continue
        selected = np.asarray(
            [bool(mask & (1 << int(patch))) for patch in face_patch], dtype=bool
        )
        selected_faces = faces[selected]
        if len(selected_faces) < 4 or connected_face_count(selected_faces) != 1:
            continue
        try:
            oriented = orient_closed_faces(vertices, selected_faces)
        except ValueError:
            continue
        volume = signed_volume(vertices, oriented)
        if volume <= 0:
            continue
        candidates.append((volume, len(oriented), mask, oriented))
    if not candidates:
        raise RuntimeError("no connected closed orientable patch combination was found")
    candidates.sort(key=lambda item: (item[0], item[1]), reverse=True)
    scale = float(np.linalg.norm(vertices.max(axis=0) - vertices.min(axis=0)))
    intersection_tolerance = max(1e-11, scale * 1e-9)
    rejected_intersections: list[dict[str, object]] = []
    selected_candidate: tuple[float, int, int, np.ndarray] | None = None
    for candidate in candidates:
        intersection = find_self_intersection(
            vertices, candidate[3], intersection_tolerance
        )
        if intersection is None:
            selected_candidate = candidate
            break
        rejected_intersections.append(
            {
                "faces": candidate[1],
                "enclosed_volume_m3": candidate[0],
                "intersecting_faces": list(intersection),
            }
        )
    if selected_candidate is None:
        raise RuntimeError(
            "all connected closed patch combinations contain triangle self-intersections"
        )
    volume, _, mask, outer_faces = selected_candidate
    selected_patches = [patch for patch in range(patch_count) if mask & (1 << patch)]
    return outer_faces, {
        "manifold_patch_sizes": patch_sizes,
        "candidate_count": len(candidates),
        "selected_patches": selected_patches,
        "selected_patch_sizes": [patch_sizes[patch] for patch in selected_patches],
        "enclosed_volume_m3": volume,
        "self_intersection_tolerance_m": intersection_tolerance,
        "rejected_larger_self_intersecting_candidates": rejected_intersections,
    }


def compact_mesh(vertices: np.ndarray, faces: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    used = np.unique(faces)
    remap = np.full(len(vertices), -1, dtype=np.int64)
    remap[used] = np.arange(len(used), dtype=np.int64)
    return vertices[used], remap[faces]


def subdivide_long_edges(
    vertices: np.ndarray, faces: np.ndarray, max_edge_length: float
) -> tuple[np.ndarray, np.ndarray]:
    """Conformingly split shared edges until every route edge is short enough."""
    if max_edge_length <= 0:
        return vertices.copy(), faces.copy()

    result_vertices = [vertex.copy() for vertex in vertices]
    result_faces = [tuple(int(index) for index in face) for face in faces]
    for _ in range(16):
        marked: set[tuple[int, int]] = set()
        for a, b, c in result_faces:
            for first, second in ((a, b), (b, c), (c, a)):
                edge = tuple(sorted((first, second)))
                if (
                    np.linalg.norm(
                        result_vertices[edge[1]] - result_vertices[edge[0]]
                    )
                    > max_edge_length
                ):
                    marked.add(edge)
        if not marked:
            break

        midpoint: dict[tuple[int, int], int] = {}
        for edge in sorted(marked):
            midpoint[edge] = len(result_vertices)
            result_vertices.append(
                0.5 * (result_vertices[edge[0]] + result_vertices[edge[1]])
            )

        refined: list[tuple[int, int, int]] = []
        for a, b, c in result_faces:
            mab = midpoint.get(tuple(sorted((a, b))))
            mbc = midpoint.get(tuple(sorted((b, c))))
            mca = midpoint.get(tuple(sorted((c, a))))
            count = sum(value is not None for value in (mab, mbc, mca))
            if count == 0:
                refined.append((a, b, c))
            elif count == 1:
                if mab is not None:
                    refined.extend(((a, mab, c), (mab, b, c)))
                elif mbc is not None:
                    refined.extend(((b, mbc, a), (mbc, c, a)))
                else:
                    refined.extend(((c, mca, b), (mca, a, b)))
            elif count == 2:
                if mab is None:
                    refined.extend(((c, mca, mbc), (a, b, mbc), (a, mbc, mca)))
                elif mbc is None:
                    refined.extend(((a, mab, mca), (b, c, mca), (b, mca, mab)))
                else:
                    refined.extend(((b, mbc, mab), (c, a, mab), (c, mab, mbc)))
            else:
                refined.extend(
                    (
                        (a, mab, mca),
                        (mab, b, mbc),
                        (mca, mbc, c),
                        (mab, mbc, mca),
                    )
                )
        result_faces = refined
    else:
        raise RuntimeError("route mesh edge subdivision did not converge")

    return np.asarray(result_vertices), np.asarray(result_faces, dtype=np.int64)


def mesh_metrics(vertices: np.ndarray, faces: np.ndarray) -> dict[str, object]:
    adjacency = edge_faces(faces)
    triangles = vertices[faces]
    doubled_area = np.linalg.norm(
        np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]),
        axis=1,
    )
    return {
        "vertices": int(len(vertices)),
        "faces": int(len(faces)),
        "boundary_edges": sum(len(adjacent) == 1 for adjacent in adjacency.values()),
        "nonmanifold_edges": sum(len(adjacent) > 2 for adjacent in adjacency.values()),
        "degenerate_faces": int(np.count_nonzero(doubled_area <= 1e-14)),
        "connected_components": connected_face_count(faces),
        "signed_volume_m3": signed_volume(vertices, faces),
        "bounds_m": [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()],
    }


def contact_patch(
    vertices: np.ndarray, faces: np.ndarray, joint_side: str, depth: float
) -> tuple[np.ndarray, np.ndarray]:
    centroids = vertices[faces].mean(axis=1)
    if joint_side == "max_y":
        threshold = float(vertices[:, 1].max()) - depth
        selected = centroids[:, 1] >= threshold
    elif joint_side == "min_y":
        threshold = float(vertices[:, 1].min()) + depth
        selected = centroids[:, 1] <= threshold
    else:
        raise ValueError(f"unsupported joint side: {joint_side}")
    if np.count_nonzero(selected) < 8:
        raise RuntimeError("joint contact extraction selected fewer than eight triangles")
    return compact_mesh(vertices, faces[selected])


def write_obj(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    lines = ["# Generated by scripts/prepare_faive_pip_outer_surfaces.py"]
    lines.extend(f"v {x:.10g} {y:.10g} {z:.10g}" for x, y, z in vertices)
    lines.extend(f"f {a + 1} {b + 1} {c + 1}" for a, b, c in faces)
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def prepare(
    asset_dir: Path, contact_depth: float, route_max_edge: float
) -> dict[str, object]:
    reports: dict[str, object] = {}
    for spec in SOURCES:
        source_path = asset_dir / spec.source
        triangles_native = read_binary_stl(source_path)
        vertices_native, faces, removed_duplicates = indexed_unique_faces(
            triangles_native, MERGE_TOLERANCE_NATIVE
        )
        vertices = vertices_native * SOURCE_SCALE
        outer_faces, selection = choose_outer_shell(vertices, faces)
        outer_vertices, outer_faces = compact_mesh(vertices, outer_faces)
        extracted_outer_metrics = mesh_metrics(outer_vertices, outer_faces)
        if (
            extracted_outer_metrics["boundary_edges"]
            or extracted_outer_metrics["nonmanifold_edges"]
            or extracted_outer_metrics["degenerate_faces"]
            or extracted_outer_metrics["connected_components"] != 1
            or extracted_outer_metrics["signed_volume_m3"] <= 0
        ):
            raise RuntimeError(
                f"outer shell validation failed for {spec.source}: "
                f"{extracted_outer_metrics}"
            )

        patch_vertices, patch_faces = contact_patch(
            outer_vertices, outer_faces, spec.joint_side, contact_depth
        )
        patch_metrics = mesh_metrics(patch_vertices, patch_faces)
        route_vertices, route_faces = subdivide_long_edges(
            outer_vertices, outer_faces, route_max_edge
        )
        route_metrics = mesh_metrics(route_vertices, route_faces)
        write_obj(asset_dir / spec.outer, route_vertices, route_faces)
        write_obj(asset_dir / spec.contact, patch_vertices, patch_faces)
        reports[spec.source] = {
            "source_faces": int(len(triangles_native)),
            "exact_duplicate_faces_removed": removed_duplicates,
            "selection": selection,
            "outer_asset": spec.outer,
            "extracted_outer_metrics": extracted_outer_metrics,
            "outer_metrics": route_metrics,
            "route_max_edge_m": route_max_edge,
            "contact_asset": spec.contact,
            "contact_depth_m": contact_depth,
            "contact_metrics": patch_metrics,
        }

    manifest = {
        "generator": "scripts/prepare_faive_pip_outer_surfaces.py",
        "source_units": "mm",
        "output_units": "m",
        "method": (
            "exact duplicate removal, nonmanifold-edge patch cut, exhaustive closed-patch "
            "selection by maximum enclosed volume, consistent outward winding, and "
            "conforming route-edge subdivision"
        ),
        "assets": reports,
    }
    (asset_dir / "outer_surface_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-dir", type=Path, default=DEFAULT_ASSET_DIR)
    parser.add_argument(
        "--contact-depth",
        type=float,
        default=0.006,
        help="Joint-facing shell depth retained for rigid-flex contact, in metres.",
    )
    parser.add_argument(
        "--route-max-edge",
        type=float,
        default=0.002,
        help="Maximum edge length for the closed route shell, in metres.",
    )
    args = parser.parse_args()
    manifest = prepare(
        args.asset_dir.resolve(), args.contact_depth, args.route_max_edge
    )
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
