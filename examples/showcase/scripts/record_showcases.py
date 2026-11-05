#!/usr/bin/env python3
"""Record raw and annotated videos for the standalone showcase models."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[3]
SHOWCASE = Path(__file__).resolve().parents[1]

try:
    from scripts.view_cpp_plugin_demo import (
        cable_sensor_bindings,
        read_cable_visual_states,
        update_cable_state_colors,
    )
except ModuleNotFoundError:
    import sys

    sys.path.insert(0, str(ROOT))
    from scripts.view_cpp_plugin_demo import (
        cable_sensor_bindings,
        read_cable_visual_states,
        update_cable_state_colors,
    )


@dataclass(frozen=True)
class Spec:
    name: str
    title: str
    model: str
    actuator: str
    command: float
    duration: float
    tracked_joints: tuple[str, ...]
    preserve_cable_colors: bool = False


SPECS = (
    Spec("irregular_pulley_ellipse", "Elliptic pulley", "irregular_pulley_ellipse.xml", "pulley_drive", 0.0015, 6.0, ("pulley_hinge",)),
    Spec("irregular_pulley_cam", "Free-form cam pulley", "irregular_pulley_cam.xml", "pulley_drive", 0.0020, 6.0, ("pulley_hinge",)),
    Spec("irregular_pulley_box", "Rounded rectangular pulley", "irregular_pulley_box.xml", "pulley_drive", 0.0010, 6.0, ("pulley_hinge",)),
    Spec("icra_surface_cables", "ICRA multi-surface cable sculpture", "icra_surface_cables.xml", "I_drive", 0.008, 7.0, ("I_hinge", "C_hinge", "R_hinge", "A_hinge"), True),
)


def resolve(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    object_id = mujoco.mj_name2id(model, kind, name)
    if object_id < 0:
        raise ValueError(f"missing MuJoCo object: {name}")
    return object_id


def smooth_pulse(time: float, duration: float, amplitude: float) -> float:
    phase = time / max(duration, 1e-9)
    if phase < 0.15:
        value = 0.5 - 0.5 * np.cos(np.pi * phase / 0.15)
    elif phase < 0.50:
        value = 1.0
    elif phase < 0.80:
        value = np.cos(np.pi * (phase - 0.50) / 0.30)
    elif phase < 0.85:
        value = -1.0
    else:
        value = -0.5 - 0.5 * np.cos(np.pi * (phase - 0.85) / 0.15)
    return float(amplitude * value)


def font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    path = Path("/System/Library/Fonts/Supplemental/Times New Roman Bold.ttf" if bold else "/System/Library/Fonts/Supplemental/Times New Roman.ttf")
    if path.exists():
        return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def annotate(image: Image.Image, spec: Spec, time: float, command: float, angles: list[float], states) -> Image.Image:
    result = image.convert("RGBA")
    layer = Image.new("RGBA", result.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    lines = [f"u = {command:+.3f} N m"]
    if len(angles) == 1:
        lines.append(f"pulley angle = {np.degrees(angles[0]):+.1f} deg")
    else:
        lines.append("letter angles = " + ", ".join(f"{name}:{np.degrees(angle):+.1f}" for name, angle in zip("ICRA", angles)))
    if states:
        lines.append(
            "cable tension = "
            + ", ".join(
                f"{state.tension:.2f} N ({'TAUT' if state.taut else 'SLACK'})"
                for state in states[:3]
            )
        )
    height = 72 + 24 * len(lines)
    draw.rectangle((20, 20, 735, 20 + height), fill=(8, 12, 17, 198), outline=(110, 125, 136, 220), width=1)
    draw.text((34, 31), spec.title, fill=(248, 249, 250, 255), font=font(26, True))
    draw.text((35, 62), f"MuJoCable surface route | t = {time:4.2f} s", fill=(180, 198, 208, 255), font=font(17))
    for index, line in enumerate(lines):
        draw.text((35, 91 + 24 * index), line, fill=(238, 241, 243, 255), font=font(17))
    draw.text((result.width - 145, 22), "MuJoCable", fill=(190, 202, 210, 235), font=font(17, True))
    return Image.alpha_composite(result, layer).convert("RGB")


def encode(directory: Path, target: Path, fps: int) -> None:
    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i", str(directory / "frame_%05d.png"), "-c:v", "libx264", "-crf", "22", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(target)],
        check=True,
    )


def record(spec: Spec, output: Path, fps: int, width: int, height: int) -> dict[str, object]:
    model_path = SHOWCASE / "models" / spec.model
    model = mujoco.MjModel.from_xml_path(str(model_path))
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, height)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    actuator = resolve(model, mujoco.mjtObj.mjOBJ_ACTUATOR, spec.actuator)
    joints = [resolve(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in spec.tracked_joints]
    bindings = cable_sensor_bindings(model, model_path)
    base_rgba = np.asarray(model.tendon_rgba, dtype=float).copy()
    renderer = mujoco.Renderer(model, width=width, height=height, max_geom=30000)
    raw_dir = Path(tempfile.mkdtemp(prefix=f"{spec.name}_raw_"))
    annotated_dir = Path(tempfile.mkdtemp(prefix=f"{spec.name}_annotated_"))
    output.mkdir(parents=True, exist_ok=True)
    next_frame = 0.0
    frame_index = 0
    max_tension = 0.0
    max_status = 0
    try:
        while data.time < spec.duration:
            command = smooth_pulse(float(data.time), spec.duration, spec.command)
            data.ctrl[:] = 0
            data.ctrl[actuator] = command
            mujoco.mj_step(model, data)
            if data.time + 1e-12 < next_frame:
                continue
            states = read_cable_visual_states(data, bindings)
            if not spec.preserve_cable_colors:
                update_cable_state_colors(model, states, base_rgba)
            renderer.update_scene(data, camera="overview")
            pixels = renderer.render()
            raw = Image.fromarray(pixels)
            raw.save(raw_dir / f"frame_{frame_index:05d}.png")
            angles = [float(data.qpos[model.jnt_qposadr[joint]]) for joint in joints]
            annotated = annotate(raw, spec, float(data.time), command, angles, states)
            annotated.save(annotated_dir / f"frame_{frame_index:05d}.png")
            if states:
                max_tension = max(max_tension, max(state.tension for state in states))
                for binding in bindings:
                    if binding.sensor_dimension >= 12:
                        max_status = max(max_status, int(round(data.sensordata[binding.sensor_address + 8])))
            if frame_index == 0 or abs(float(data.time) - 0.48 * spec.duration) < 0.5 / fps:
                annotated.save(output / f"{spec.name}_poster.png")
            frame_index += 1
            next_frame += 1.0 / fps
    finally:
        renderer.close()
    raw_target = output / f"{spec.name}_raw.mp4"
    annotated_target = output / f"{spec.name}_annotated.mp4"
    encode(raw_dir, raw_target, fps)
    encode(annotated_dir, annotated_target, fps)
    shutil.rmtree(raw_dir)
    shutil.rmtree(annotated_dir)
    return {"name": spec.name, "frames": frame_index, "duration_s": spec.duration, "max_tension_N": max_tension, "max_route_status": max_status, "raw": raw_target.name, "annotated": annotated_target.name}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=SHOWCASE / "videos")
    parser.add_argument("--showcase", action="append", choices=[spec.name for spec in SPECS])
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=1120)
    parser.add_argument("--height", type=int, default=630)
    args = parser.parse_args()
    mujoco.mj_loadPluginLibrary(str(args.plugin.resolve()))
    selected = [spec for spec in SPECS if not args.showcase or spec.name in args.showcase]
    summaries = [record(spec, args.output.resolve(), args.fps, args.width, args.height) for spec in selected]
    manifest_path = args.output.resolve() / "manifest.json"
    existing: dict[str, object] = {}
    if manifest_path.exists():
        existing = json.loads(manifest_path.read_text(encoding="utf-8"))
    videos = {
        item["name"]: item
        for item in existing.get("videos", [])
        if isinstance(item, dict) and "name" in item
    }
    videos.update({item["name"]: item for item in summaries})
    ordered = [videos[spec.name] for spec in SPECS if spec.name in videos]
    manifest = {"plugin": args.plugin.name, "videos": ordered}
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
