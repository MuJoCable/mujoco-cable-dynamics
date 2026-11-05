#!/usr/bin/env python3
"""Generate watertight showcase meshes, textures, and MJCF models."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image


SHOWCASE = Path(__file__).resolve().parents[1]
ASSETS = SHOWCASE / "assets"
MODELS = SHOWCASE / "models"


def signed_area(points: np.ndarray) -> float:
    shifted = np.roll(points, -1, axis=0)
    return 0.5 * float(
        np.sum(points[:, 0] * shifted[:, 1] - shifted[:, 0] * points[:, 1])
    )


def point_in_triangle(point: np.ndarray, triangle: np.ndarray) -> bool:
    a, b, c = triangle
    cross = lambda u, v: float(u[0] * v[1] - u[1] * v[0])
    ab = cross(b - a, point - a)
    bc = cross(c - b, point - b)
    ca = cross(a - c, point - c)
    return ab >= -1e-12 and bc >= -1e-12 and ca >= -1e-12


def triangulate_polygon(points: np.ndarray) -> list[tuple[int, int, int]]:
    if len(points) < 3:
        raise ValueError("polygon must have at least three vertices")
    if signed_area(points) < 0:
        points[:] = points[::-1]
    remaining = list(range(len(points)))
    triangles: list[tuple[int, int, int]] = []
    guard = 0
    while len(remaining) > 3:
        ear_found = False
        for cursor, current in enumerate(remaining):
            previous = remaining[(cursor - 1) % len(remaining)]
            following = remaining[(cursor + 1) % len(remaining)]
            a, b, c = points[[previous, current, following]]
            first = b - a
            second = c - b
            cross = float(first[0] * second[1] - first[1] * second[0])
            if cross <= 1e-12:
                continue
            triangle = np.asarray([a, b, c])
            if any(
                point_in_triangle(points[index], triangle)
                for index in remaining
                if index not in {previous, current, following}
            ):
                continue
            triangles.append((previous, current, following))
            del remaining[cursor]
            ear_found = True
            break
        guard += 1
        if not ear_found or guard > len(points) * len(points):
            raise RuntimeError("failed to triangulate a simple polygon")
    triangles.append(tuple(remaining))
    return triangles


def extruded_polygon(
    points: list[tuple[float, float]], half_depth: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    polygon = np.asarray(points, dtype=float)
    if signed_area(polygon) < 0:
        polygon = polygon[::-1].copy()
    front = np.column_stack(
        (polygon[:, 0], np.full(len(polygon), -half_depth), polygon[:, 1])
    )
    back = np.column_stack(
        (polygon[:, 0], np.full(len(polygon), half_depth), polygon[:, 1])
    )
    vertices = np.vstack((front, back))
    planar = triangulate_polygon(polygon.copy())
    count = len(polygon)
    faces: list[tuple[int, int, int]] = []
    faces.extend(planar)
    faces.extend((a + count, c + count, b + count) for a, b, c in planar)
    for index in range(count):
        following = (index + 1) % count
        faces.append((index, index + count, following + count))
        faces.append((index, following + count, following))
    minimum = polygon.min(axis=0)
    span = np.maximum(polygon.max(axis=0) - minimum, 1e-9)
    uv = (polygon - minimum) / span
    texcoords = np.vstack((uv, uv))
    return vertices, np.asarray(faces, dtype=int), texcoords


def write_obj(
    path: Path, vertices: np.ndarray, faces: np.ndarray, texcoords: np.ndarray
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as stream:
        stream.write("# MuJoCable deterministic watertight extrusion\n")
        for vertex in vertices:
            stream.write(f"v {vertex[0]:.9f} {vertex[1]:.9f} {vertex[2]:.9f}\n")
        for uv in texcoords:
            stream.write(f"vt {uv[0]:.9f} {uv[1]:.9f}\n")
        for face in faces:
            tokens = [f"{index + 1}/{index + 1}" for index in face]
            stream.write("f " + " ".join(tokens) + "\n")


def edge_statistics(faces: np.ndarray) -> tuple[int, int]:
    counts: dict[tuple[int, int], int] = {}
    for face in faces:
        for index in range(3):
            edge = tuple(sorted((int(face[index]), int(face[(index + 1) % 3]))))
            counts[edge] = counts.get(edge, 0) + 1
    return len(counts), sum(count != 2 for count in counts.values())


def texture(path: Path, rgb: tuple[int, int, int]) -> None:
    image = Image.new("RGB", (256, 256), rgb)
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)


def pulley_polygons() -> dict[str, list[tuple[float, float]]]:
    ellipse = [
        (0.060 * math.cos(theta), 0.040 * math.sin(theta))
        for theta in np.linspace(0, 2 * math.pi, 64, endpoint=False)
    ]
    cam = []
    for theta in np.linspace(0, 2 * math.pi, 72, endpoint=False):
        radius = 0.046 * (
            1.0 + 0.13 * math.cos(theta) + 0.055 * math.cos(2 * theta - 0.4)
        )
        cam.append((radius * math.cos(theta), radius * math.sin(theta)))
    box = []
    exponent = 8.0
    for theta in np.linspace(0, 2 * math.pi, 72, endpoint=False):
        cosine = math.cos(theta)
        sine = math.sin(theta)
        box.append(
            (
                0.064 * math.copysign(abs(cosine) ** (2.0 / exponent), cosine),
                0.034 * math.copysign(abs(sine) ** (2.0 / exponent), sine),
            )
        )
    return {"ellipse": ellipse, "cam": cam, "box": box}


def letter_polygons() -> dict[str, list[tuple[float, float]]]:
    # Silhouettes follow the supplied ICRA mark: a narrow I, circular C,
    # rounded-bowl R, and an open-counter stylized A.
    i_points = [(-0.012, -0.075), (0.012, -0.075), (0.012, 0.075), (-0.012, 0.075)]

    outer = [
        (0.071 * math.cos(theta), 0.075 * math.sin(theta))
        for theta in np.linspace(math.radians(38), math.radians(322), 52)
    ]
    inner = [
        (0.045 * math.cos(theta), 0.050 * math.sin(theta))
        for theta in np.linspace(math.radians(322), math.radians(38), 42)
    ]
    c_points = outer + inner

    r_outer = [
        (0.006 + 0.054 * math.cos(theta), 0.035 + 0.040 * math.sin(theta))
        for theta in np.linspace(math.pi / 2, -math.pi / 2, 18)
    ]
    r_inner = [
        (0.004 + 0.030 * math.cos(theta), 0.035 + 0.020 * math.sin(theta))
        for theta in np.linspace(-math.pi / 2, math.pi / 2, 14)
    ]
    r_points = [
        (-0.045, -0.075),
        (-0.045, 0.075),
        *r_outer,
        (0.058, -0.075),
        (0.027, -0.075),
        (-0.004, -0.018),
        *r_inner,
        (-0.018, 0.055),
        (-0.018, -0.075),
    ]

    a_points = [
        (-0.065, -0.075),
        (-0.009, 0.075),
        (0.011, 0.075),
        (0.060, -0.075),
        (0.030, -0.075),
        (0.014, -0.022),
        (-0.006, -0.022),
        (-0.010, -0.006),
        (0.009, -0.006),
        (0.000, 0.030),
        (-0.038, -0.075),
    ]
    return {"I": i_points, "C": c_points, "R": r_points, "A": a_points}


def pulley_model(profile: str) -> str:
    display = {"ellipse": "elliptic", "cam": "free-form cam", "box": "rectangular"}[profile]
    return f'''<mujoco model="showcase_{profile}_pulley">
  <compiler autolimits="true" angle="radian" meshdir="../assets/irregular_pulleys" />
  <size nuser_site="1" />
  <option timestep="0.0002" gravity="0 0 -9.81" integrator="implicitfast"><flag energy="enable" /></option>
  <extension>
    <plugin plugin="mujoco.cable.unilateral">
      <instance name="irregular_cable">
        <config key="route_mode" value="surface" />
        <config key="mesh_route_mode" value="guided_surface" />
        <config key="route_tendon" value="irregular_seed" />
        <config key="wrap_geoms" value="irregular_pulley_visual" />
        <config key="mesh_guide_axis" value="0 1 0" />
        <config key="mesh_guide_weight" value="500" />
        <config key="home_length" value="auto_initial" />
        <config key="stiffness" value="15000" />
        <config key="damping" value="2" />
        <config key="pretension_offset" value="0.00015" />
        <config key="max_tension" value="100" />
        <config key="capstan_mu" value="0.12" />
        <config key="capstan_direction" value="forward" />
        <config key="route_hysteresis" value="0.00002" />
        <config key="visual_width" value="3.4" />
        <config key="visual_smoothing_timeconstant" value="0.020" />
      </instance>
    </plugin>
  </extension>
  <asset>
    <mesh name="irregular_profile" file="{profile}.obj" />
    <material name="frame" rgba="0.52 0.60 0.66 1" />
    <material name="pulley" rgba="0.76 0.78 0.80 0.46" />
    <material name="left" rgba="0.12 0.52 0.88 1" />
    <material name="right" rgba="0.92 0.35 0.10 1" />
  </asset>
  <visual><global azimuth="90" elevation="-8" /><headlight ambient="0.28 0.28 0.28" diffuse="0.75 0.75 0.75" /></visual>
  <default><site size="0.004" /><default class="hint"><site user="2" size="0.0001" rgba="0 0 0 0" /></default></default>
  <worldbody>
    <camera name="overview" pos="0 -0.72 0.225" xyaxes="1 0 0 0 0 1" fovy="42" />
    <light name="key" pos="-0.25 -0.80 0.75" dir="0.2 0.8 -0.7" />
    <geom name="floor" type="plane" pos="0 0 0.06" size="0.34 0.24 0.01" rgba="0.13 0.15 0.17 1" friction="0.8 0.04 0.005" />
    <geom type="capsule" fromto="0 0.04 0.06 0 0.04 0.415" size="0.007" material="frame" contype="0" conaffinity="0" />
    <geom type="capsule" fromto="-0.20 0.04 0.415 0.20 0.04 0.415" size="0.007" material="frame" contype="0" conaffinity="0" />
    <body name="irregular_pulley" pos="0 0 0.315">
      <joint name="pulley_hinge" type="hinge" axis="0 1 0" damping="0.080"
             armature="0.00080" limited="true" range="-0.32 0.32" />
      <geom name="irregular_pulley_visual" type="mesh" mesh="irregular_profile" mass="0.080" material="pulley" contype="0" conaffinity="0" />
      <geom type="cylinder" euler="1.57079632679 0 0" size="0.009 0.020" density="0" rgba="0.42 0.46 0.50 1" contype="0" conaffinity="0" />
      <geom name="radius_marker" type="capsule" fromto="0 -0.019 0 0.038 -0.019 0" size="0.0022" density="0" rgba="0.08 0.35 1 1" contype="0" conaffinity="0" />
    </body>
    <site name="upper_hint" class="hint" pos="0 0 0.377" />
    <body name="left_mass" pos="-0.074 0 0.145">
      <freejoint name="left_free" />
      <geom type="box" size="0.023 0.023 0.023" density="1200" material="left" />
      <site name="left_end" pos="0 0 0.029" user="1" rgba="0.12 0.75 1 1" />
    </body>
    <body name="right_mass" pos="0.074 0 0.145">
      <freejoint name="right_free" />
      <geom type="box" size="0.029 0.029 0.029" density="1200" material="right" />
      <site name="right_end" pos="0 0 0.035" user="1" rgba="1 0.35 0.08 1" />
    </body>
  </worldbody>
  <tendon>
    <spatial name="irregular_seed" width="0.000000001" rgba="1 0.25 0.04 1">
      <site site="left_end" /><site site="upper_hint" /><site site="right_end" />
    </spatial>
  </tendon>
  <actuator><motor name="pulley_drive" joint="pulley_hinge" gear="1" ctrllimited="true" ctrlrange="-0.002 0.002" /></actuator>
  <sensor>
    <plugin name="irregular_cable_state" instance="irregular_cable" />
    <jointpos name="pulley_angle" joint="pulley_hinge" />
    <jointvel name="pulley_speed" joint="pulley_hinge" />
  </sensor>
  <!-- {display} profile: the motor is optional; u=0 recovers the passive Result 1.3 topology. -->
</mujoco>
'''


def icra_model() -> str:
    return '''<mujoco model="icra_surface_cable_logo">
  <compiler autolimits="true" angle="radian" meshdir="../assets/icra" texturedir="../assets/icra" />
  <size nuser_site="1" memory="96M" />
  <option timestep="0.0005" gravity="0 0 0" integrator="implicitfast" />
  <extension>
    <plugin plugin="mujoco.cable.unilateral">
      <instance name="red_logo_cable">
        <config key="route_mode" value="surface" /><config key="mesh_route_mode" value="guided_surface" />
        <config key="route_tendon" value="red_seed" /><config key="wrap_geoms" value="C_wrap R_wrap" />
        <config key="mesh_guide_axis" value="0 1 0" /><config key="mesh_guide_weight" value="2000" />
        <config key="home_length" value="auto_initial" /><config key="stiffness" value="300" />
        <config key="damping" value="1.5" /><config key="pretension_offset" value="0.010" />
        <config key="max_tension" value="10" /><config key="route_hysteresis" value="0.00005" />
        <config key="visual_width" value="2.7" /><config key="visual_smoothing_timeconstant" value="0.025" />
      </instance>
      <instance name="blue_logo_cable">
        <config key="route_mode" value="surface" /><config key="mesh_route_mode" value="guided_surface" />
        <config key="route_tendon" value="blue_seed" /><config key="wrap_geoms" value="C_wrap" />
        <config key="mesh_guide_axis" value="0 1 0" /><config key="mesh_guide_weight" value="2000" />
        <config key="home_length" value="auto_initial" /><config key="stiffness" value="300" />
        <config key="damping" value="1.5" /><config key="pretension_offset" value="0.010" />
        <config key="max_tension" value="10" /><config key="route_hysteresis" value="0.00005" />
        <config key="visual_width" value="2.7" /><config key="visual_smoothing_timeconstant" value="0.025" />
      </instance>
      <instance name="gold_logo_cable">
        <config key="route_mode" value="surface" /><config key="mesh_route_mode" value="guided_surface" />
        <config key="route_tendon" value="gold_seed" /><config key="wrap_geoms" value="A_wrap" />
        <config key="mesh_guide_axis" value="0 1 0" /><config key="mesh_guide_weight" value="2000" />
        <config key="home_length" value="auto_initial" /><config key="stiffness" value="250" />
        <config key="damping" value="1.2" /><config key="pretension_offset" value="0.006" />
        <config key="max_tension" value="8" /><config key="route_hysteresis" value="0.00005" />
        <config key="visual_width" value="2.5" /><config key="visual_smoothing_timeconstant" value="0.025" />
      </instance>
    </plugin>
  </extension>
  <asset>
    <mesh name="I_mesh" file="I.obj" /><mesh name="C_mesh" file="C.obj" />
    <mesh name="R_mesh" file="R.obj" /><mesh name="A_mesh" file="A.obj" />
    <texture name="purple_texture" type="2d" file="purple.png" />
    <texture name="orange_texture" type="2d" file="orange.png" />
    <material name="purple" texture="purple_texture" texuniform="true" texrepeat="2 2" rgba="1 1 1 1" />
    <material name="orange" texture="orange_texture" texuniform="true" texrepeat="2 2" rgba="1 1 1 1" />
  </asset>
  <visual><global azimuth="90" elevation="0" /><headlight ambient="0.35 0.35 0.35" diffuse="0.78 0.78 0.78" /></visual>
  <default>
    <site size="0.005" />
    <default class="hint"><site user="2" size="0.0001" rgba="0 0 0 0" /></default>
    <default class="letter"><joint type="hinge" axis="0 1 0" damping="0.012" armature="0.00008" /></default>
  </default>
  <worldbody>
    <camera name="overview" pos="0 -1.00 0.31" xyaxes="1 0 0 0 0 1" fovy="36" />
    <light name="key" pos="-0.2 -0.8 0.7" dir="0.2 0.9 -0.5" />
    <geom type="plane" pos="0 0 0.10" size="0.55 0.24 0.01" rgba="0.96 0.96 0.96 1" contype="0" conaffinity="0" />
    <body name="letter_I" pos="-0.300 0 0.300">
      <joint name="I_hinge" class="letter" damping="0.020" />
      <geom name="I_visual" type="mesh" mesh="I_mesh" material="purple" mass="0.035" contype="0" conaffinity="0" />
      <!-- All cable endpoints and hints use y=0, the extrusion mid-plane. -->
      <site name="I_red_start" pos="0 0 0.075" user="1" rgba="1 0.20 0.25 1" />
      <site name="I_blue_start" pos="0 0 -0.075" user="1" rgba="0.10 0.55 1 1" />
      <geom type="sphere" size="0.007" pos="0 -0.016 0" rgba="0.96 0.96 0.98 1" density="0" contype="0" conaffinity="0" />
    </body>
    <body name="letter_C" pos="-0.120 0 0.300">
      <joint name="C_hinge" class="letter" />
      <geom name="C_wrap" type="mesh" mesh="C_mesh" material="orange" mass="0.060" contype="0" conaffinity="0" />
      <site name="C_red_hint" class="hint" pos="0 -0.000  -0.074" />
      <site name="C_blue_hint" class="hint" pos="0 0 0.074" />
    </body>
    <body name="letter_R" pos="0.075 0 0.300">
      <joint name="R_hinge" class="letter" />
      <geom name="R_wrap" type="mesh" mesh="R_mesh" material="purple" mass="0.065" contype="0" conaffinity="0" />
      <site name="R_red_hint" class="hint" pos="0 0 0.074" />
      <site name="R_blue_end" pos="-0.043 0 0.060" user="1" rgba="0.10 0.55 1 1" />
      <site name="R_gold_start" pos="0.052 0 -0.064" user="1" rgba="1 0.72 0.10 1" />
    </body>
    <body name="letter_A" pos="0.280 0 0.300">
      <joint name="A_hinge" class="letter" />
      <geom name="A_wrap" type="mesh" mesh="A_mesh" material="purple" mass="0.060" contype="0" conaffinity="0" />
      <site name="A_gold_hint" class="hint" pos="-0.040 0 -0.052" />
      <site name="A_red_end" pos="-0.040 0 -0.010" user="1" rgba="1 0.20 0.25 1" />
    </body>
    <site name="gold_world_end" pos="0.430 0 0.285" user="1" rgba="1 0.72 0.10 1" />
  </worldbody>
  <tendon>
    <spatial name="red_seed" width="0.000000001" rgba="1 0.18 0.22 1">
      <site site="I_red_start" /><site site="C_red_hint" /><site site="R_red_hint" /><site site="A_red_end" />
    </spatial>
    <spatial name="blue_seed" width="0.000000001" rgba="0.08 0.55 1 1">
      <site site="I_blue_start" /><site site="C_blue_hint" /><site site="R_blue_end" />
    </spatial>
    <spatial name="gold_seed" width="0.000000001" rgba="1 0.70 0.08 1">
      <site site="R_gold_start" /><site site="A_gold_hint" /><site site="gold_world_end" />
    </spatial>
  </tendon>
  <actuator><motor name="I_drive" joint="I_hinge" ctrllimited="true" ctrlrange="-0.030 0.030" gear="1" /></actuator>
  <sensor>
    <plugin name="red_state" instance="red_logo_cable" /><plugin name="blue_state" instance="blue_logo_cable" />
    <plugin name="gold_state" instance="gold_logo_cable" />
    <jointpos name="I_angle" joint="I_hinge" /><jointpos name="C_angle" joint="C_hinge" />
    <jointpos name="R_angle" joint="R_hinge" /><jointpos name="A_angle" joint="A_hinge" />
  </sensor>
</mujoco>
'''


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=SHOWCASE / "assets/manifest.json")
    args = parser.parse_args()
    manifest: dict[str, object] = {"meshes": {}}

    for name, polygon in pulley_polygons().items():
        vertices, faces, uv = extruded_polygon(polygon, 0.010)
        path = ASSETS / "irregular_pulleys" / f"{name}.obj"
        write_obj(path, vertices, faces, uv)
        edge_count, bad_edges = edge_statistics(faces)
        manifest["meshes"][str(path.relative_to(SHOWCASE))] = {
            "vertices": len(vertices), "triangles": len(faces),
            "edges": edge_count, "nonmanifold_edges": bad_edges,
        }
    letter_dir = ASSETS / "icra"
    for name, polygon in letter_polygons().items():
        vertices, faces, uv = extruded_polygon(polygon, 0.012)
        path = letter_dir / f"{name}.obj"
        write_obj(path, vertices, faces, uv)
        edge_count, bad_edges = edge_statistics(faces)
        manifest["meshes"][str(path.relative_to(SHOWCASE))] = {
            "vertices": len(vertices), "triangles": len(faces),
            "edges": edge_count, "nonmanifold_edges": bad_edges,
        }
    texture(letter_dir / "purple.png", (112, 17, 137))
    texture(letter_dir / "orange.png", (245, 159, 0))

    MODELS.mkdir(parents=True, exist_ok=True)
    for profile in pulley_polygons():
        (MODELS / f"irregular_pulley_{profile}.xml").write_text(
            pulley_model(profile), encoding="utf-8"
        )
    (MODELS / "icra_surface_cables.xml").write_text(icra_model(), encoding="utf-8")
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    if any(item["nonmanifold_edges"] for item in manifest["meshes"].values()):
        raise RuntimeError("generated mesh is not closed and manifold")
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
