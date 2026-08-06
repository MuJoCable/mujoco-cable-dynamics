#!/usr/bin/env python3
"""Extract the 100_fingers human hand and generate a MuJoCable MJCF model.

The source ``human_hand_v3.1_nolig.stl`` contains one palm/metacarpal shell
and fifteen disconnected phalanx shells.  This script identifies those
components, places each phalanx in a four-DOF digit tree, and emits the five
functional cable routes described by the source project:

  extensor, abductor, adductor, intermediate flexor, distal flexor.

Only the published design dimensions and CC BY 4.0 mesh asset are used.  The
GPL-licensed OpenSCAD implementation is not copied into the generated model.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ASSET_DIR = ROOT / "cable_plugin_demos/assets/100_fingers_human"
DEFAULT_MODEL = ROOT / "cable_plugin_demos/26_cpp_plugin_100_fingers_human_hand.xml"

DIGIT_ORDER = ("index", "middle", "ring", "little", "thumb")
FINGER_ORDER = ("index", "middle", "ring", "little")

# Values are in millimetres and come from the human preset in parameters.scad.
DIGIT_PARAMETERS = {
    "index": {"lengths": [38.0, 20.0, 20.0], "joint_diameter": 12.5,
              "joint_width": 15.0},
    "middle": {"lengths": [41.0, 20.5, 20.5], "joint_diameter": 14.0,
               "joint_width": 16.0},
    "ring": {"lengths": [38.0, 19.0, 19.0], "joint_diameter": 12.5,
             "joint_width": 15.0},
    "little": {"lengths": [35.0, 17.0, 17.0], "joint_diameter": 11.0,
               "joint_width": 13.0},
    "thumb": {"lengths": [35.0, 25.0, 25.0], "joint_diameter": 16.0,
              "joint_width": 20.0},
}

CABLES = {
    "extensor": {
        "color": "0.10 0.35 0.95 1", "wraps": ("mcp", "pip", "dip"),
        "stiffness": 500.0,
    },
    "abductor": {
        "color": "0.00 0.68 0.72 1", "wraps": ("mcp_lateral",),
        "stiffness": 350.0,
    },
    "adductor": {
        "color": "0.82 0.18 0.66 1", "wraps": ("mcp_lateral",),
        "stiffness": 350.0,
    },
    "flexor_intermediate": {
        "color": "1.00 0.55 0.05 1", "wraps": ("mcp", "pip"),
        "stiffness": 700.0,
    },
    "flexor_distal": {
        "color": "0.92 0.10 0.08 1", "wraps": ("mcp", "pip", "dip"),
        "stiffness": 700.0,
    },
}


def _format(values: np.ndarray | list[float] | tuple[float, ...]) -> str:
    return " ".join(f"{float(value):.9g}" for value in values)


def _load_binary_stl(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError(f"STL is too short: {path}")
    triangle_count = struct.unpack_from("<I", raw, 80)[0]
    expected = 84 + 50 * triangle_count
    if len(raw) != expected:
        raise ValueError(
            f"Expected a binary STL with {triangle_count} triangles; "
            f"file has {len(raw)} bytes instead of {expected}"
        )
    dtype = np.dtype(
        [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)),
         ("attribute", "<u2")]
    )
    return np.frombuffer(raw, dtype=dtype, offset=84, count=triangle_count)[
        "vertices"
    ].astype(np.float64)


def _connected_components(triangles: np.ndarray) -> list[np.ndarray]:
    _, inverse = np.unique(
        triangles.reshape(-1, 3), axis=0, return_inverse=True
    )
    face_vertices = inverse.reshape(-1, 3)
    parent = np.arange(len(triangles), dtype=np.int64)
    rank = np.zeros(len(triangles), dtype=np.int8)

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = int(parent[index])
        return index

    def union(first: int, second: int) -> None:
        first_root, second_root = find(first), find(second)
        if first_root == second_root:
            return
        if rank[first_root] < rank[second_root]:
            first_root, second_root = second_root, first_root
        parent[second_root] = first_root
        if rank[first_root] == rank[second_root]:
            rank[first_root] += 1

    vertex_owner: dict[int, int] = {}
    for face_index, vertex_ids in enumerate(face_vertices):
        for vertex_id in vertex_ids:
            vertex_id = int(vertex_id)
            owner = vertex_owner.setdefault(vertex_id, face_index)
            union(face_index, owner)

    roots = np.asarray([find(index) for index in range(len(triangles))])
    return [triangles[roots == root] for root in np.unique(roots)]


def _component_record(triangles: np.ndarray) -> dict[str, object]:
    vertices = np.unique(triangles.reshape(-1, 3), axis=0)
    return {
        "triangles": triangles,
        "vertices": vertices,
        "centroid": triangles.reshape(-1, 3).mean(axis=0),
        "face_count": len(triangles),
    }


def _classify_components(
    components: list[np.ndarray],
) -> tuple[dict[str, object], dict[str, list[dict[str, object]]]]:
    records = [_component_record(component) for component in components]
    if len(records) != 16:
        raise ValueError(
            "The human no-ligament asset must contain 16 connected components "
            f"(palm plus 15 phalanges); found {len(records)}"
        )
    palm = max(records, key=lambda record: int(record["face_count"]))
    moving = [record for record in records if record is not palm]

    # In the published human preset the thumb is the only three-component
    # chain whose centroids remain below x=65 mm in the print frame.
    thumb = sorted(
        [record for record in moving if record["centroid"][0] < 65.0],
        key=lambda record: float(record["centroid"][0]),
    )
    fingers = sorted(
        [record for record in moving if record["centroid"][0] >= 65.0],
        key=lambda record: float(record["centroid"][2]),
    )
    if len(thumb) != 3 or len(fingers) != 12:
        raise ValueError(
            "Could not identify the human-preset thumb and four finger chains"
        )

    groups: dict[str, list[dict[str, object]]] = {"thumb": thumb}
    for index, name in enumerate(FINGER_ORDER):
        group = fingers[3 * index : 3 * index + 3]
        groups[name] = sorted(
            group, key=lambda record: float(record["centroid"][0])
        )
    return palm, groups


def _closest_interface_center(
    first: dict[str, object], second: dict[str, object]
) -> tuple[float, np.ndarray]:
    first_vertices = np.asarray(first["vertices"])
    second_vertices = np.asarray(second["vertices"])
    best_squared = math.inf
    for start in range(0, len(first_vertices), 300):
        chunk = first_vertices[start : start + 300]
        squared = np.sum(
            (chunk[:, None, :] - second_vertices[None, :, :]) ** 2, axis=2
        )
        best_squared = min(best_squared, float(squared.min()))

    threshold_squared = (math.sqrt(best_squared) + 0.20) ** 2
    midpoints = []
    for start in range(0, len(first_vertices), 300):
        chunk = first_vertices[start : start + 300]
        squared = np.sum(
            (chunk[:, None, :] - second_vertices[None, :, :]) ** 2, axis=2
        )
        first_ids, second_ids = np.where(squared <= threshold_squared)
        if len(first_ids):
            midpoints.append(
                (chunk[first_ids] + second_vertices[second_ids]) * 0.5
            )
    if not midpoints:
        raise RuntimeError("No interface vertices found between hand components")
    return math.sqrt(best_squared), np.median(np.concatenate(midpoints), axis=0)


def _digit_frame(pivots: list[np.ndarray]) -> np.ndarray:
    longitudinal = pivots[-1] - pivots[0]
    longitudinal /= np.linalg.norm(longitudinal)
    palm_normal = np.array([0.0, 1.0, 0.0])
    lateral = np.cross(longitudinal, palm_normal)
    lateral /= np.linalg.norm(lateral)
    normal = np.cross(lateral, longitudinal)
    normal /= np.linalg.norm(normal)
    return np.column_stack((longitudinal, normal, lateral))


def _rotation_quaternion(matrix: np.ndarray) -> np.ndarray:
    trace = float(np.trace(matrix))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quat = np.array(
            [0.25 * scale,
             (matrix[2, 1] - matrix[1, 2]) / scale,
             (matrix[0, 2] - matrix[2, 0]) / scale,
             (matrix[1, 0] - matrix[0, 1]) / scale]
        )
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            quat = np.array(
                [(matrix[2, 1] - matrix[1, 2]) / scale, 0.25 * scale,
                 (matrix[0, 1] + matrix[1, 0]) / scale,
                 (matrix[0, 2] + matrix[2, 0]) / scale]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            quat = np.array(
                [(matrix[0, 2] - matrix[2, 0]) / scale,
                 (matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                 (matrix[1, 2] + matrix[2, 1]) / scale]
            )
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            quat = np.array(
                [(matrix[1, 0] - matrix[0, 1]) / scale,
                 (matrix[0, 2] + matrix[2, 0]) / scale,
                 (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale]
            )
    return quat / np.linalg.norm(quat)


def _write_binary_stl(path: Path, triangles: np.ndarray) -> None:
    edges_a = triangles[:, 1] - triangles[:, 0]
    edges_b = triangles[:, 2] - triangles[:, 0]
    normals = np.cross(edges_a, edges_b)
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > 1e-12
    normals[valid] /= lengths[valid, None]
    normals[~valid] = 0.0

    dtype = np.dtype(
        [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)),
         ("attribute", "<u2")]
    )
    records = np.zeros(len(triangles), dtype=dtype)
    records["normal"] = normals.astype(np.float32)
    records["vertices"] = triangles.astype(np.float32)
    header = b"100_fingers CC BY 4.0 derivative for MuJoCable".ljust(80, b" ")
    path.write_bytes(header + struct.pack("<I", len(records)) + records.tobytes())


def _add_config(instance: ET.Element, key: str, value: object) -> None:
    ET.SubElement(instance, "config", key=key, value=str(value))


def _add_plugin_instances(extension: ET.Element) -> None:
    plugin = ET.SubElement(extension, "plugin", plugin="mujoco.cable.unilateral")
    for digit in DIGIT_ORDER:
        for cable_name, cable in CABLES.items():
            instance = ET.SubElement(
                plugin, "instance", name=f"{digit}_{cable_name}_cable"
            )
            _add_config(instance, "route_mode", "surface")
            _add_config(instance, "route_tendon", f"{digit}_{cable_name}_seed")
            wraps = " ".join(f"{digit}_{name}_wrap" for name in cable["wraps"])
            _add_config(instance, "wrap_geoms", wraps)
            _add_config(instance, "home_length", "auto_initial")
            _add_config(instance, "stiffness", cable["stiffness"])
            _add_config(instance, "damping", 1.2)
            _add_config(instance, "slack", 0.00005)
            _add_config(instance, "pretension_offset", 0.00010)
            _add_config(instance, "max_tension", 5.0)
            _add_config(instance, "ctrl_mode", "target_contraction")
            _add_config(instance, "control_timeconstant", 0.04)
            _add_config(instance, "max_contraction_rate", 0.04)
            _add_config(instance, "taut_transition", 0.00008)
            _add_config(instance, "taut_hysteresis", 0.00002)
            _add_config(instance, "route_hysteresis", 0.00001)
            _add_config(instance, "visual_width", 1.6)
            _add_config(instance, "visual_smoothing_timeconstant", 0.02)


def _site(
    body: ET.Element, name: str, pos_mm: tuple[float, float, float] | np.ndarray,
    *, role: int, color: str = "0 0 0 0", size: float = 0.00015,
) -> None:
    ET.SubElement(
        body, "site", name=name, pos=_format(np.asarray(pos_mm) * 0.001),
        user=str(role), size=f"{size:.8g}", rgba=color,
    )


def _build_digit(
    frame: ET.Element, digit: str, records: list[dict[str, object]],
    pivots: list[np.ndarray], basis: np.ndarray,
) -> dict[str, object]:
    params = DIGIT_PARAMETERS[digit]
    base_diameter = float(params["joint_diameter"])
    base_width = float(params["joint_width"])
    # The source dimensions describe the printed joint.  A small centerline
    # clearance keeps the rendered cable outside the visual STL rather than
    # z-fighting with the groove surface.
    cable_clearance_mm = 0.7
    radii_mm = [
        0.5 * base_diameter + cable_clearance_mm,
        0.3 * base_diameter + cable_clearance_mm,
        0.25 * base_diameter + cable_clearance_mm,
    ]
    widths_mm = [0.5 * base_width, 0.4 * base_width, 0.35 * base_width]
    quat = _rotation_quaternion(basis)

    mount = ET.SubElement(
        frame, "body", name=f"{digit}_mount",
        pos=_format(pivots[0] * 0.001), quat=_format(quat),
    )
    start_x = -max(12.0, radii_mm[0] + 5.0)
    _site(mount, f"{digit}_extensor_start", (start_x, -1.15 * radii_mm[0], 0),
          role=1, color=CABLES["extensor"]["color"], size=0.00065)
    _site(mount, f"{digit}_abductor_start", (start_x, 0, 1.15 * radii_mm[0]),
          role=1, color=CABLES["abductor"]["color"], size=0.00065)
    _site(mount, f"{digit}_adductor_start", (start_x, 0, -1.15 * radii_mm[0]),
          role=1, color=CABLES["adductor"]["color"], size=0.00065)
    _site(mount, f"{digit}_flexor_intermediate_start",
          (start_x, 1.05 * radii_mm[0], -0.8), role=1,
          color=CABLES["flexor_intermediate"]["color"], size=0.00065)
    _site(mount, f"{digit}_flexor_distal_start",
          (start_x, 1.20 * radii_mm[0], 0.8), role=1,
          color=CABLES["flexor_distal"]["color"], size=0.00065)

    parent = mount
    segment_bodies = []
    segment_names = ("proximal", "intermediate", "distal")
    for segment_index, (segment_name, record) in enumerate(
        zip(segment_names, records)
    ):
        if segment_index == 0:
            body_pos = np.zeros(3)
        else:
            body_pos = (pivots[segment_index] - pivots[segment_index - 1]) @ basis
        body = ET.SubElement(
            parent, "body", name=f"{digit}_{segment_name}",
            pos=_format(body_pos * 0.001),
        )
        local_vertices = (
            np.asarray(record["vertices"]) - pivots[segment_index]
        ) @ basis
        center = local_vertices.mean(axis=0) * 0.001
        mass = (0.010, 0.006, 0.004)[segment_index]
        inertia = (1.2e-6, 7.0e-7, 5.0e-7)[segment_index]
        ET.SubElement(
            body, "inertial", pos=_format(center), mass=f"{mass:.6g}",
            diaginertia=_format((inertia, inertia, inertia)),
        )
        if segment_index == 0:
            ET.SubElement(
                body, "joint", name=f"{digit}_mcp_abduction", type="hinge",
                axis="0 1 0", range="-0.45 0.45", damping="0.02",
            )
            ET.SubElement(
                body, "joint", name=f"{digit}_mcp_flexion", type="hinge",
                axis="0 0 1", range="-0.15 1.30", damping="0.02",
            )
        else:
            joint_name = "pip_flexion" if segment_index == 1 else "dip_flexion"
            joint_range = "0 1.75" if segment_index == 1 else "0 1.45"
            ET.SubElement(
                body, "joint", name=f"{digit}_{joint_name}", type="hinge",
                axis="0 0 1", range=joint_range, damping="0.018",
            )
        ET.SubElement(
            body, "geom", name=f"{digit}_{segment_name}_visual", type="mesh",
            mesh=f"{digit}_{segment_name}_mesh", material="bone_material",
            density="0", contype="0", conaffinity="0",
        )

        joint_key = ("mcp", "pip", "dip")[segment_index]
        ET.SubElement(
            body, "geom", name=f"{digit}_{joint_key}_wrap", type="cylinder",
            size=_format((radii_mm[segment_index] * 0.001,
                          widths_mm[segment_index] * 0.001)),
            density="0", contype="0", conaffinity="0", rgba="0 0 0 0",
        )
        if segment_index == 0:
            ET.SubElement(
                body, "geom", name=f"{digit}_mcp_lateral_wrap",
                type="cylinder",
                size=_format((radii_mm[0] * 0.001, widths_mm[0] * 0.001)),
                quat="0.707106781 -0.707106781 0 0", density="0",
                contype="0", conaffinity="0", rgba="0 0 0 0",
            )

        _site(body, f"{digit}_{joint_key}_flexor_intermediate_hint",
              (0, 1.05 * radii_mm[segment_index], -0.8), role=2)
        _site(body, f"{digit}_{joint_key}_flexor_distal_hint",
              (0, 1.20 * radii_mm[segment_index], 0.8), role=2)
        _site(body, f"{digit}_{joint_key}_extensor_hint",
              (0, -1.15 * radii_mm[segment_index], 0), role=2)
        if segment_index == 0:
            _site(body, f"{digit}_mcp_abductor_hint",
                  (0, 0, 1.15 * radii_mm[0]), role=2)
            _site(body, f"{digit}_mcp_adductor_hint",
                  (0, 0, -1.15 * radii_mm[0]), role=2)
            proximal_x = float(local_vertices[:, 0].max()) * 0.55
            _site(body, f"{digit}_abductor_end",
                  (proximal_x, 0, 0.70 * radii_mm[0]), role=1,
                  color=CABLES["abductor"]["color"], size=0.00065)
            _site(body, f"{digit}_adductor_end",
                  (proximal_x, 0, -0.70 * radii_mm[0]), role=1,
                  color=CABLES["adductor"]["color"], size=0.00065)
        if segment_index < 2:
            guide_x = 0.52 * float(local_vertices[:, 0].max())
            _site(body, f"{digit}_{segment_name}_flexor_guide",
                  (guide_x, float(local_vertices[:, 1].max()) + 0.8, 0),
                  role=3, color="1 0.72 0.08 0.75", size=0.00045)
            _site(body, f"{digit}_{segment_name}_extensor_guide",
                  (guide_x, float(local_vertices[:, 1].min()) - 0.8, 0),
                  role=3, color=CABLES["extensor"]["color"], size=0.00045)
        if segment_index == 1:
            intermediate_x = float(local_vertices[:, 0].max()) - 2.0
            near_tip = local_vertices[local_vertices[:, 0] > intermediate_x - 3.0]
            _site(body, f"{digit}_flexor_intermediate_end",
                  (intermediate_x, float(near_tip[:, 1].max()) + 0.8, -0.8), role=1,
                  color=CABLES["flexor_intermediate"]["color"], size=0.00065)
        if segment_index == 2:
            tip_x = float(local_vertices[:, 0].max()) - 1.5
            near_tip = local_vertices[local_vertices[:, 0] > tip_x - 3.0]
            _site(body, f"{digit}_flexor_distal_end",
                  (tip_x, float(near_tip[:, 1].max()) + 0.8, 0.8), role=1,
                  color=CABLES["flexor_distal"]["color"], size=0.00065)
            _site(body, f"{digit}_extensor_end",
                  (tip_x, float(near_tip[:, 1].min()) - 0.8, 0), role=1,
                  color=CABLES["extensor"]["color"], size=0.00065)
            ET.SubElement(
                body, "site", name=f"{digit}_tip", pos=_format((tip_x * 0.001, 0, 0)),
                size="0.0012", rgba="0.95 0.75 0.12 1",
            )
        segment_bodies.append(body)
        parent = body

    return {
        "basis": basis.tolist(),
        "pivots_mm": [pivot.tolist() for pivot in pivots],
        "gaps_mm": [],
    }


def _add_tendon_route(
    tendon_root: ET.Element, digit: str, cable_name: str,
) -> None:
    cable = CABLES[cable_name]
    spatial = ET.SubElement(
        tendon_root, "spatial", name=f"{digit}_{cable_name}_seed",
        width="0.000000001", rgba=str(cable["color"]), limited="false",
    )
    ET.SubElement(spatial, "site", site=f"{digit}_{cable_name}_start")
    if cable_name == "abductor":
        ET.SubElement(spatial, "site", site=f"{digit}_mcp_abductor_hint")
        ET.SubElement(spatial, "site", site=f"{digit}_abductor_end")
    elif cable_name == "adductor":
        ET.SubElement(spatial, "site", site=f"{digit}_mcp_adductor_hint")
        ET.SubElement(spatial, "site", site=f"{digit}_adductor_end")
    elif cable_name == "flexor_intermediate":
        ET.SubElement(spatial, "site", site=f"{digit}_mcp_flexor_intermediate_hint")
        ET.SubElement(spatial, "site", site=f"{digit}_proximal_flexor_guide")
        ET.SubElement(spatial, "site", site=f"{digit}_pip_flexor_intermediate_hint")
        ET.SubElement(spatial, "site", site=f"{digit}_flexor_intermediate_end")
    elif cable_name == "flexor_distal":
        ET.SubElement(spatial, "site", site=f"{digit}_mcp_flexor_distal_hint")
        ET.SubElement(spatial, "site", site=f"{digit}_proximal_flexor_guide")
        ET.SubElement(spatial, "site", site=f"{digit}_pip_flexor_distal_hint")
        ET.SubElement(spatial, "site", site=f"{digit}_intermediate_flexor_guide")
        ET.SubElement(spatial, "site", site=f"{digit}_dip_flexor_distal_hint")
        ET.SubElement(spatial, "site", site=f"{digit}_flexor_distal_end")
    elif cable_name == "extensor":
        ET.SubElement(spatial, "site", site=f"{digit}_mcp_extensor_hint")
        ET.SubElement(spatial, "site", site=f"{digit}_proximal_extensor_guide")
        ET.SubElement(spatial, "site", site=f"{digit}_pip_extensor_hint")
        ET.SubElement(spatial, "site", site=f"{digit}_intermediate_extensor_guide")
        ET.SubElement(spatial, "site", site=f"{digit}_dip_extensor_hint")
        ET.SubElement(spatial, "site", site=f"{digit}_extensor_end")


def _generate_model(
    asset_dir: Path, model_path: Path,
    palm: dict[str, object], groups: dict[str, list[dict[str, object]]],
    component_metadata: dict[str, object],
) -> None:
    root = ET.Element("mujoco", model="100_fingers_human_mujocable")
    ET.SubElement(
        root, "compiler", angle="radian", autolimits="true",
        meshdir="assets/100_fingers_human",
    )
    ET.SubElement(root, "size", nuser_site="1")
    ET.SubElement(
        root, "option", timestep="0.0005", gravity="0 0 0",
        integrator="implicitfast",
    )
    extension = ET.SubElement(root, "extension")
    _add_plugin_instances(extension)

    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker",
                  rgb1="0.17 0.18 0.19", rgb2="0.24 0.25 0.26",
                  width="512", height="512")
    ET.SubElement(asset, "material", name="floor_material", texture="grid",
                  texrepeat="8 8", reflectance="0.08")
    ET.SubElement(asset, "material", name="bone_material",
                  rgba="0.82 0.73 0.42 1", specular="0.25", shininess="0.3")
    ET.SubElement(asset, "material", name="palm_material",
                  rgba="0.68 0.58 0.28 1", specular="0.20", shininess="0.25")
    ET.SubElement(asset, "mesh", name="human_palm_mesh", file="human_palm.stl",
                  scale="0.001 0.001 0.001")
    for digit in DIGIT_ORDER:
        for segment in ("proximal", "intermediate", "distal"):
            ET.SubElement(
                asset, "mesh", name=f"{digit}_{segment}_mesh",
                file=f"human_{digit}_{segment}.stl", scale="0.001 0.001 0.001",
            )

    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "headlight", diffuse="0.75 0.75 0.75",
                  ambient="0.25 0.25 0.25")
    ET.SubElement(visual, "global", azimuth="-90", elevation="-52")

    default = ET.SubElement(root, "default")
    ET.SubElement(default, "geom", friction="0.9 0.01 0.001",
                  solref="0.008 1", solimp="0.92 0.99 0.002")
    ET.SubElement(default, "joint", armature="0.00002", frictionloss="0.001",
                  solreflimit="0.003 1", solimplimit="0.95 0.999 0.001")

    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", name="key", pos="0 -0.25 0.55",
                  dir="0 0.25 -1")
    ET.SubElement(world, "geom", name="floor", type="plane", pos="0 0 0",
                  size="0.40 0.40 0.01", material="floor_material")
    ET.SubElement(world, "camera", name="overview", pos="0.02 -0.34 0.34",
                  xyaxes="1 0 0 0 0.447213595 0.894427191")
    ET.SubElement(world, "camera", name="dorsal", pos="0.02 0 0.48",
                  xyaxes="1 0 0 0 1 0")
    frame = ET.SubElement(
        world, "body", name="hand_frame", pos="-0.055 0 0.205",
        euler="1.57079632679 0 0",
    )
    ET.SubElement(
        frame, "geom", name="human_palm_visual", type="mesh",
        mesh="human_palm_mesh", material="palm_material", density="0",
        contype="0", conaffinity="0",
    )

    for digit in DIGIT_ORDER:
        records = groups[digit]
        gaps: list[float] = []
        pivots: list[np.ndarray] = []
        previous = palm
        for record in records:
            gap, pivot = _closest_interface_center(previous, record)
            gaps.append(gap)
            pivots.append(pivot)
            previous = record
        basis = _digit_frame(pivots)
        digit_meta = _build_digit(frame, digit, records, pivots, basis)
        digit_meta["gaps_mm"] = gaps
        component_metadata["digits"][digit] = digit_meta

    tendon = ET.SubElement(root, "tendon")
    for digit in DIGIT_ORDER:
        for cable_name in CABLES:
            _add_tendon_route(tendon, digit, cable_name)

    actuator = ET.SubElement(root, "actuator")
    for digit in DIGIT_ORDER:
        for cable_name in CABLES:
            ET.SubElement(
                actuator, "plugin", name=f"{digit}_{cable_name}_command",
                tendon=f"{digit}_{cable_name}_seed", gear="0",
                instance=f"{digit}_{cable_name}_cable", ctrllimited="true",
                ctrlrange="0 0.018",
            )

    sensor = ET.SubElement(root, "sensor")
    for digit in DIGIT_ORDER:
        for cable_name in CABLES:
            ET.SubElement(
                sensor, "plugin", name=f"{digit}_{cable_name}_state",
                instance=f"{digit}_{cable_name}_cable",
            )
        ET.SubElement(
            sensor, "framepos", name=f"{digit}_tip_position",
            objtype="site", objname=f"{digit}_tip",
        )

    ET.indent(root, space="  ")
    model_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(
        model_path, encoding="unicode", xml_declaration=False,
        short_empty_elements=True,
    )
    with model_path.open("a", encoding="utf-8") as stream:
        stream.write("\n")


def prepare(source_stl: Path, asset_dir: Path, model_path: Path) -> None:
    triangles = _load_binary_stl(source_stl)
    palm, groups = _classify_components(_connected_components(triangles))
    asset_dir.mkdir(parents=True, exist_ok=True)
    _write_binary_stl(asset_dir / "human_palm.stl", np.asarray(palm["triangles"]))

    metadata: dict[str, object] = {
        "source": "100_fingers human_hand_v3.1_nolig.stl",
        "source_sha256": hashlib.sha256(source_stl.read_bytes()).hexdigest(),
        "source_license": "CC BY 4.0",
        "source_project": "https://github.com/kg398/100_fingers",
        "component_count": 16,
        "digits": {},
    }
    # Compute and write the local-frame moving meshes before generating MJCF.
    for digit in DIGIT_ORDER:
        records = groups[digit]
        pivots = []
        previous = palm
        for record in records:
            _, pivot = _closest_interface_center(previous, record)
            pivots.append(pivot)
            previous = record
        basis = _digit_frame(pivots)
        for index, (segment, record) in enumerate(
            zip(("proximal", "intermediate", "distal"), records)
        ):
            local = (np.asarray(record["triangles"]) - pivots[index]) @ basis
            _write_binary_stl(asset_dir / f"human_{digit}_{segment}.stl", local)

    _generate_model(asset_dir, model_path, palm, groups, metadata)
    (asset_dir / "asset_manifest.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-stl", type=Path, required=True,
        help="Path to 100_fingers STLs/hands/human_hand_v3.1_nolig.stl",
    )
    parser.add_argument("--asset-dir", type=Path, default=DEFAULT_ASSET_DIR)
    parser.add_argument("--model-output", type=Path, default=DEFAULT_MODEL)
    args = parser.parse_args()
    prepare(args.source_stl.resolve(), args.asset_dir.resolve(), args.model_output.resolve())
    print(f"Wrote assets to {args.asset_dir}")
    print(f"Wrote model to {args.model_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
