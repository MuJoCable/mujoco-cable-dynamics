#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLUGIN = ROOT / "build/plugin/libcable_unilateral.dylib"
PLUGIN_COMMAND_PATH = Path("build/plugin/libcable_unilateral.dylib")
DEFAULT_OUT = ROOT / "cable_plugin_demos/screenshots"
TIMES_NEW_ROMAN = Path("/System/Library/Fonts/Supplemental/Times New Roman.ttf")


@dataclass(frozen=True)
class DemoShotSpec:
    label: str
    model: Path
    ctrl: float
    steps: int = 900
    release_other_ratio: float = 0.0
    actuator_name: str | None = None


DEFAULT_DEMOS = [
    DemoShotSpec("demo09_free_hanging_single_pulley", ROOT / "cable_plugin_demos/09_cpp_plugin_free_hanging_single_pulley.xml", 0.2),
    DemoShotSpec("demo10_dual_pulley_free_payload", ROOT / "cable_plugin_demos/10_cpp_plugin_dual_pulley_free_payload.xml", 0.2),
    DemoShotSpec("demo11_reverse_reserve_release", ROOT / "cable_plugin_demos/11_cpp_plugin_reverse_reserve_release.xml", 0.2),
    DemoShotSpec("demo12_frictional_pulley_free_payload", ROOT / "cable_plugin_demos/12_cpp_plugin_frictional_pulley_free_payload.xml", 0.018),
    DemoShotSpec("demo13_rolling_joint_figure_eight", ROOT / "cable_plugin_demos/13_cpp_plugin_rolling_joint_figure_eight.xml", 0.04, 2500),
    DemoShotSpec("demo14_convex_mesh_rolling_joint", ROOT / "cable_plugin_demos/14_cpp_plugin_convex_mesh_rolling_joint.xml", 0.012, 300),
    DemoShotSpec("demo15_surface_single_pulley", ROOT / "cable_plugin_demos/15_cpp_plugin_surface_single_pulley.xml", 0.06, 2500),
    DemoShotSpec("demo16_passive_saddle_joint", ROOT / "cable_plugin_demos/16_cpp_plugin_passive_saddle_joint.xml", 0.0, 100),
    DemoShotSpec("demo17_three_strut_nine_cable", ROOT / "cable_plugin_demos/17_cpp_plugin_three_strut_nine_cable.xml", 0.0, 6000),
    DemoShotSpec(
        "demo20_controlled_saddle_joint",
        ROOT / "cable_plugin_demos/20_cpp_plugin_controlled_saddle_joint.xml",
        0.0051,
        4000,
        0.25,
        "saddle_upper_control",
    ),
    DemoShotSpec(
        "demo21_wheel_axle_force_amplifier",
        ROOT / "cable_plugin_demos/21_cpp_plugin_wheel_axle_force_amplifier.xml",
        0.12,
        3900,
        0.0,
        "input_pull",
    ),
]


def resolve_path(path: str | Path) -> Path:
    path = Path(path)
    return path if path.is_absolute() else (ROOT / path).resolve()


def parse_demo_spec(value: str) -> DemoShotSpec:
    # Format: label=path:ctrl[:steps[:release_ratio]]. Split from the right so
    # absolute paths are safe.
    label, right = value.split("=", 1)
    parts = right.rsplit(":", 3)
    if len(parts) == 2:
        model_text, ctrl_text = parts
        steps = 900
        release_other_ratio = 0.0
    elif len(parts) == 3:
        model_text, ctrl_text, steps_text = parts
        steps = int(steps_text)
        release_other_ratio = 0.0
    elif len(parts) == 4:
        model_text, ctrl_text, steps_text, release_text = parts
        steps = int(steps_text)
        release_other_ratio = float(release_text)
    else:
        raise ValueError(
            "demo format must be label=path:ctrl[:steps[:release_ratio]]"
        )
    return DemoShotSpec(
        label=label,
        model=resolve_path(model_text),
        ctrl=float(ctrl_text),
        steps=steps,
        release_other_ratio=release_other_ratio,
    )


def font(size: int) -> ImageFont.ImageFont:
    if TIMES_NEW_ROMAN.exists():
        return ImageFont.truetype(str(TIMES_NEW_ROMAN), size)
    return ImageFont.load_default()


def unit(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-12:
        return np.zeros(3)
    return vec / norm


def primary_actuator(model: mujoco.MjModel) -> int:
    for actuator_id in range(model.nu):
        if int(model.actuator_trntype[actuator_id]) == int(mujoco.mjtTrn.mjTRN_TENDON):
            return actuator_id
    return 0 if model.nu else -1


def active_tendons(model: mujoco.MjModel) -> list[int]:
    tendons: list[int] = []
    for actuator_id in range(model.nu):
        if int(model.actuator_trntype[actuator_id]) == int(mujoco.mjtTrn.mjTRN_TENDON):
            tendon_id = int(model.actuator_trnid[actuator_id, 0])
            if tendon_id not in tendons and float(model.tendon_width[tendon_id]) > 1e-8:
                tendons.append(tendon_id)
    widths = np.asarray(model.tendon_width, dtype=float)
    for tendon_id in range(model.ntendon):
        if tendon_id not in tendons and widths[tendon_id] > 1e-8:
            tendons.append(tendon_id)
    return tendons


def surface_line_segments(
    model: mujoco.MjModel, data: mujoco.MjData
) -> list[tuple[int, np.ndarray, np.ndarray, np.ndarray]]:
    """Extract lines added by the cable plugin's standard visualize callback."""
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
    segments: list[tuple[int, np.ndarray, np.ndarray, np.ndarray]] = []
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
        segments.append(
            (tendon_id, start, end, np.asarray(geom.rgba, dtype=float).copy())
        )
    return segments


def tendon_points(data: mujoco.MjData, tendon_id: int) -> tuple[np.ndarray, np.ndarray]:
    adr = int(data.ten_wrapadr[tendon_id])
    num = int(data.ten_wrapnum[tendon_id])
    points = np.asarray(data.wrap_xpos, dtype=float).reshape(-1, 3)[adr : adr + num]
    objs = np.asarray(data.wrap_obj, dtype=int).reshape(-1)[adr : adr + num]
    return points, objs


def projection_coordinates(pos: np.ndarray, projection: str) -> tuple[float, float]:
    if projection == "isometric":
        x, y, z = (float(value) for value in pos)
        horizontal = 0.8660254 * x + 0.5 * y
        depth = -0.5 * x + 0.8660254 * y
        return horizontal, z + 0.34 * depth
    return float(pos[0]), float(pos[2])


class Projector:
    def __init__(self, bounds: tuple[float, float, float, float], width: int, height: int, projection: str = "xz"):
        self.xmin, self.xmax, self.zmin, self.zmax = bounds
        self.width = width
        self.height = height
        self.left = 125
        self.right = 80
        self.top = 110
        self.bottom = 90
        self.projection = projection

    def point(self, pos: np.ndarray | tuple[float, float]) -> tuple[int, int]:
        if isinstance(pos, tuple):
            x, z = pos
        else:
            x, z = projection_coordinates(pos, self.projection)
        plot_w = self.width - self.left - self.right
        plot_h = self.height - self.top - self.bottom
        sx = self.left + (x - self.xmin) / max(self.xmax - self.xmin, 1e-9) * plot_w
        sy = self.top + plot_h - (z - self.zmin) / max(self.zmax - self.zmin, 1e-9) * plot_h
        return int(round(sx)), int(round(sy))

    def length_px(self, length: float) -> int:
        plot_w = self.width - self.left - self.right
        return max(1, int(round(abs(length) / max(self.xmax - self.xmin, 1e-9) * plot_w)))


def collect_bounds_points(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    tendon_ids: list[int],
    surface_segments: list[tuple[int, np.ndarray, np.ndarray, np.ndarray]],
    projection: str,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for pos in np.asarray(data.geom_xpos):
        points.append(projection_coordinates(pos, projection))
    for pos in np.asarray(data.site_xpos):
        points.append(projection_coordinates(pos, projection))
    for tendon_id in tendon_ids:
        wrap_points, _ = tendon_points(data, tendon_id)
        for point in wrap_points:
            points.append(projection_coordinates(point, projection))
    for _, start, end, _ in surface_segments:
        points.append(projection_coordinates(start, projection))
        points.append(projection_coordinates(end, projection))
    if projection == "isometric":
        for geom_id in range(model.ngeom):
            if int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_PLANE):
                continue
            center = np.asarray(data.geom_xpos)[geom_id]
            for x in (-0.42, 0.42):
                for y in (-0.42, 0.42):
                    points.append(projection_coordinates(center + np.array([x, y, 0.0]), projection))
    return points


def scene_bounds(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    tendon_ids: list[int],
    surface_segments: list[tuple[int, np.ndarray, np.ndarray, np.ndarray]],
    projection: str = "xz",
) -> tuple[float, float, float, float]:
    points = collect_bounds_points(model, data, tendon_ids, surface_segments, projection)
    xs = [p[0] for p in points]
    zs = [p[1] for p in points]
    xmin, xmax = min(xs), max(xs)
    zmin, zmax = min(zs), max(zs)
    xpad = max(0.08, 0.16 * max(xmax - xmin, 1e-6))
    zpad = max(0.08, 0.16 * max(zmax - zmin, 1e-6))
    return xmin - xpad, xmax + xpad, zmin - zpad, zmax + zpad


def draw_capsule(draw: ImageDraw.ImageDraw, model: mujoco.MjModel, data: mujoco.MjData, geom_id: int, projector: Projector, rgba: np.ndarray) -> None:
    center = np.asarray(data.geom_xpos)[geom_id]
    xmat = np.asarray(data.geom_xmat).reshape(-1, 3, 3)[geom_id]
    axis = unit(xmat[:, 2])
    half_length = float(model.geom_size[geom_id, 1])
    radius = projector.length_px(float(model.geom_size[geom_id, 0]))
    p0 = center - half_length * axis
    p1 = center + half_length * axis
    a = projector.point(p0)
    b = projector.point(p1)
    color = rgba_color(rgba, (173, 196, 204))
    draw.line([a, b], fill=color, width=max(2 * radius, 5))
    draw.ellipse([a[0] - radius, a[1] - radius, a[0] + radius, a[1] + radius], fill=color)
    draw.ellipse([b[0] - radius, b[1] - radius, b[0] + radius, b[1] + radius], fill=color)


def rgba_color(rgba: np.ndarray, fallback: tuple[int, int, int]) -> tuple[int, int, int]:
    if float(np.max(rgba[:3])) <= 0:
        return fallback
    return tuple(int(np.clip(value, 0, 1) * 255) for value in rgba[:3])


def convex_hull(points: list[tuple[int, int]]) -> list[tuple[int, int]]:
    unique = sorted(set(points))
    if len(unique) <= 2:
        return unique

    def cross(origin: tuple[int, int], first: tuple[int, int], second: tuple[int, int]) -> int:
        return (first[0] - origin[0]) * (second[1] - origin[1]) - (
            first[1] - origin[1]
        ) * (second[0] - origin[0])

    lower: list[tuple[int, int]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[int, int]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def mesh_world_vertices(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int) -> np.ndarray:
    mesh_id = int(model.geom_dataid[geom_id])
    address = int(model.mesh_vertadr[mesh_id])
    count = int(model.mesh_vertnum[mesh_id])
    local = np.asarray(model.mesh_vert, dtype=float).reshape(-1, 3)[address : address + count]
    rotation = np.asarray(data.geom_xmat, dtype=float).reshape(-1, 3, 3)[geom_id]
    return local @ rotation.T + np.asarray(data.geom_xpos, dtype=float)[geom_id]


def draw_geoms(draw: ImageDraw.ImageDraw, model: mujoco.MjModel, data: mujoco.MjData, projector: Projector) -> None:
    geom_rgba = np.asarray(model.geom_rgba, dtype=float).reshape(-1, 4)
    material_rgba = np.asarray(model.mat_rgba, dtype=float).reshape(-1, 4)
    for geom_id in range(model.ngeom):
        material_id = int(model.geom_matid[geom_id])
        rgba = material_rgba[material_id] if material_id >= 0 else geom_rgba[geom_id]
        if float(rgba[3]) < 0.2:
            continue
        geom_type = int(model.geom_type[geom_id])
        pos = np.asarray(data.geom_xpos)[geom_id]
        size = np.asarray(model.geom_size)[geom_id]
        sx, sy = projector.point(pos)
        if geom_type == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
            draw_capsule(draw, model, data, geom_id, projector, rgba)
        elif geom_type == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
            radius = projector.length_px(float(size[0]))
            fill = rgba_color(rgba, (28, 28, 28))
            draw.ellipse([sx - radius, sy - radius, sx + radius, sy + radius], fill=fill, outline=(0, 0, 0), width=2)
        elif geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
            rx = projector.length_px(float(size[0]))
            rz = projector.length_px(float(size[2] if size.size >= 3 else size[0]))
            fill = rgba_color(rgba, (18, 83, 180))
            outline = tuple(max(0, int(0.55 * value)) for value in fill)
            draw.rectangle([sx - rx, sy - rz, sx + rx, sy + rz], fill=fill, outline=outline, width=2)
        elif geom_type == int(mujoco.mjtGeom.mjGEOM_SPHERE):
            radius = max(4, projector.length_px(float(size[0])))
            fill = rgba_color(rgba, (80, 80, 80))
            draw.ellipse([sx - radius, sy - radius, sx + radius, sy + radius], fill=fill, outline=(0, 0, 0), width=1)
        elif geom_type == int(mujoco.mjtGeom.mjGEOM_MESH):
            projected = [projector.point(vertex) for vertex in mesh_world_vertices(model, data, geom_id)]
            hull = convex_hull(projected)
            if len(hull) >= 3:
                fill = rgba_color(rgba, (28, 28, 28))
                draw.polygon(hull, fill=fill, outline=(0, 0, 0))


def draw_ground_grid(draw: ImageDraw.ImageDraw, model: mujoco.MjModel, data: mujoco.MjData, projector: Projector) -> None:
    if projector.projection != "isometric":
        return
    for geom_id in range(model.ngeom):
        if int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_PLANE):
            continue
        center = np.asarray(data.geom_xpos)[geom_id]
        half = 0.42
        corners = [
            center + np.array([-half, -half, 0.0]),
            center + np.array([half, -half, 0.0]),
            center + np.array([half, half, 0.0]),
            center + np.array([-half, half, 0.0]),
        ]
        draw.polygon([projector.point(point) for point in corners], fill=(235, 237, 238), outline=(95, 99, 102))
        grid_color = (180, 184, 187)
        for value in np.linspace(-half, half, 13):
            draw.line(
                [
                    projector.point(center + np.array([value, -half, 0.0])),
                    projector.point(center + np.array([value, half, 0.0])),
                ],
                fill=grid_color,
                width=1,
            )
            draw.line(
                [
                    projector.point(center + np.array([-half, value, 0.0])),
                    projector.point(center + np.array([half, value, 0.0])),
                ],
                fill=grid_color,
                width=1,
            )


def draw_sites(draw: ImageDraw.ImageDraw, model: mujoco.MjModel, data: mujoco.MjData, projector: Projector) -> None:
    rgba_values = np.asarray(model.site_rgba).reshape(-1, 4)
    for site_id, pos in enumerate(np.asarray(data.site_xpos)):
        rgba = rgba_values[site_id]
        if float(rgba[3]) < 0.2:
            continue
        sx, sy = projector.point(pos)
        radius = 6
        fill = tuple(int(np.clip(v, 0, 1) * 255) for v in rgba[:3])
        outline = tuple(max(0, int(0.55 * c)) for c in fill)
        draw.ellipse([sx - radius, sy - radius, sx + radius, sy + radius], fill=fill, outline=outline, width=1)


def arc_points(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int, p0: np.ndarray, p1: np.ndarray, samples: int = 72) -> list[np.ndarray]:
    center = np.asarray(data.geom_xpos)[geom_id]
    xmat = np.asarray(data.geom_xmat).reshape(-1, 3, 3)[geom_id]
    axis = unit(xmat[:, 2])
    axial = 0.5 * (float(np.dot(p0 - center, axis)) + float(np.dot(p1 - center, axis)))
    arc_center = center + axial * axis
    radius = float(model.geom_size[geom_id, 0])
    r0 = p0 - arc_center
    r1 = p1 - arc_center
    r0 = r0 - axis * float(np.dot(r0, axis))
    r1 = r1 - axis * float(np.dot(r1, axis))
    if np.linalg.norm(r0) <= 1e-12 or np.linalg.norm(r1) <= 1e-12:
        return [p0, p1]
    e0 = unit(r0)
    e1 = unit(np.cross(axis, e0))
    signed = float(np.arctan2(np.dot(r1, e1), np.dot(r1, e0)))
    signed = (signed + math.pi) % (2.0 * math.pi) - math.pi
    return [arc_center + radius * (math.cos(t) * e0 + math.sin(t) * e1) for t in np.linspace(0.0, signed, samples)]


def draw_tendon(draw: ImageDraw.ImageDraw, model: mujoco.MjModel, data: mujoco.MjData, tendon_id: int, projector: Projector) -> None:
    if tendon_id < 0:
        return
    points, objs = tendon_points(data, tendon_id)
    if len(points) < 2:
        return
    rgba = np.asarray(model.tendon_rgba).reshape(-1, 4)[tendon_id]
    if float(np.max(rgba[:3])) <= 0:
        cable = (224, 52, 22)
    else:
        cable = tuple(int(np.clip(v, 0, 1) * 255) for v in rgba[:3])
    highlight = tuple(min(255, int(0.55 * c + 110)) for c in cable)
    for i in range(len(points) - 1):
        geom_id = int(objs[i])
        if geom_id >= 0 and geom_id == int(objs[i + 1]) and int(model.geom_type[geom_id]) == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
            arc = [projector.point(point) for point in arc_points(model, data, geom_id, points[i], points[i + 1])]
            if len(arc) >= 2:
                draw.line(arc, fill=cable, width=9, joint="curve")
                draw.line(arc, fill=highlight, width=3, joint="curve")
        else:
            a = projector.point(points[i])
            b = projector.point(points[i + 1])
            draw.line([a, b], fill=cable, width=9)
            draw.line([a, b], fill=highlight, width=3)


def draw_surface_segments(
    draw: ImageDraw.ImageDraw,
    segments: list[tuple[int, np.ndarray, np.ndarray, np.ndarray]],
    projector: Projector,
) -> None:
    for _, start, end, rgba in segments:
        cable = tuple(int(np.clip(value, 0, 1) * 255) for value in rgba[:3])
        a = projector.point(start)
        b = projector.point(end)
        draw.line([a, b], fill=cable, width=6)


def render_screenshot(model: mujoco.MjModel, data: mujoco.MjData, tendon_ids: list[int], label: str, ctrl: float, width: int, height: int, projection: str = "xz") -> Image.Image:
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    surface_segments = surface_line_segments(model, data)
    bounds = scene_bounds(model, data, tendon_ids, surface_segments, projection)
    projector = Projector(bounds, width, height, projection)
    draw_ground_grid(draw, model, data, projector)
    draw_geoms(draw, model, data, projector)
    for tendon_id in tendon_ids:
        draw_tendon(draw, model, data, tendon_id, projector)
    draw_surface_segments(draw, surface_segments, projector)
    draw_sites(draw, model, data, projector)
    draw.text((42, 34), label.replace("_", " "), fill=(0, 0, 0), font=font(34))
    subtitle = f"ctrl = {ctrl:g}" if model.nu else "passive, no actuator"
    draw.text((42, 76), subtitle, fill=(55, 55, 55), font=font(24))
    return image


def run_one(spec: DemoShotSpec, plugin: Path, out_dir: Path, width: int, height: int, dpi: int) -> dict[str, str | float | int]:
    mujoco.mj_loadPluginLibrary(str(plugin))
    model = mujoco.MjModel.from_xml_path(str(spec.model))
    data = mujoco.MjData(model)
    actuator_id = (
        mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, spec.actuator_name
        )
        if spec.actuator_name
        else primary_actuator(model)
    )
    if spec.actuator_name and actuator_id < 0:
        raise ValueError(f"actuator not found: {spec.actuator_name}")
    tendon_ids = active_tendons(model)
    for _ in range(spec.steps):
        data.ctrl[:] = 0
        if actuator_id >= 0:
            data.ctrl[actuator_id] = spec.ctrl
            if spec.release_other_ratio:
                for other_id in range(model.nu):
                    if other_id != actuator_id:
                        data.ctrl[other_id] = -spec.release_other_ratio * spec.ctrl
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    out_path = out_dir / f"{spec.label}.png"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    projection = (
        "isometric"
        if spec.model.name.startswith(("17_", "21_", "25_"))
        else "xz"
    )
    image = render_screenshot(model, data, tendon_ids, spec.label, spec.ctrl, width, height, projection)
    image.save(out_path, dpi=(dpi, dpi))
    try:
        model_label = str(spec.model.relative_to(ROOT))
    except ValueError:
        model_label = str(spec.model)
    try:
        screenshot_label = str(out_path.relative_to(ROOT))
    except ValueError:
        screenshot_label = str(out_path)
    passive_flag = " --no-auto-control" if model.nu and abs(spec.ctrl) <= 1e-12 else ""
    antagonist_flags = (
        " --release-other-actuators"
        f" --release-other-ratio {spec.release_other_ratio:g}"
        if spec.release_other_ratio
        else ""
    )
    actuator_flag = (
        f" --auto-actuator {spec.actuator_name}"
        if spec.actuator_name
        else ""
    )
    return {
        "label": spec.label,
        "model": model_label,
        "ctrl": spec.ctrl,
        "steps": spec.steps,
        "screenshot": screenshot_label,
        "open_command": (
            "conda run -n rope_plugin mjpython scripts/view_cpp_plugin_demo.py "
            f"--plugin {PLUGIN_COMMAND_PATH} --model {spec.model.relative_to(ROOT)} "
            f"--ctrl-final {spec.ctrl:g}{actuator_flag}{passive_flag}"
            f"{antagonist_flags}"
        ),
    }


def write_readme(out_dir: Path, summaries: list[dict[str, str | float | int]]) -> None:
    lines = [
        "# C++ Plugin Demo Screenshots",
        "",
        "These deterministic screenshots use compiled MuJoCo tendon wrap points for native routes and line segments emitted by the plugin's standard `visualize` callback for surface routes. The interactive viewer remains the authoritative visual check.",
        "",
        "Regenerate with:",
        "",
        "```bash",
        "conda run -n rope_plugin python scripts/render_cpp_demo_screenshots.py \\",
        "  --plugin build/plugin/libcable_unilateral.dylib \\",
        "  --out cable_plugin_demos/screenshots",
        "```",
        "",
    ]
    for item in summaries:
        image = Path(str(item["screenshot"])).name
        lines.extend(
            [
                f"## {item['label']}",
                "",
                f"![{item['label']}]({image})",
                "",
                "Open in MuJoCo viewer:",
                "",
                "```bash",
                str(item["open_command"]),
                "```",
                "",
            ]
        )
    (out_dir / "README.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Render deterministic schematic screenshots for C++ cable plugin demos")
    parser.add_argument("--plugin", default=str(DEFAULT_PLUGIN), help="Path to the standalone C++ cable plugin")
    parser.add_argument("--out", default=str(DEFAULT_OUT), help="Output screenshot directory")
    parser.add_argument("--width", type=int, default=1800)
    parser.add_argument("--height", type=int, default=1100)
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument(
        "--demo",
        action="append",
        help=(
            "Optional demo override, format "
            "label=path:ctrl[:steps[:release_ratio]]. Can be repeated."
        ),
    )
    args = parser.parse_args()

    plugin = resolve_path(args.plugin)
    if not plugin.exists():
        raise FileNotFoundError(f"Plugin not found: {plugin}")
    specs = [parse_demo_spec(item) for item in args.demo] if args.demo else DEFAULT_DEMOS
    out_dir = resolve_path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    summaries = [run_one(spec, plugin, out_dir, args.width, args.height, args.dpi) for spec in specs]
    write_readme(out_dir, summaries)
    (out_dir / "summary.json").write_text(
        json.dumps({"plugin": str(PLUGIN_COMMAND_PATH), "demos": summaries}, indent=2),
        encoding="utf-8",
    )
    print(json.dumps({"plugin": str(plugin), "out": str(out_dir), "demos": summaries}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
