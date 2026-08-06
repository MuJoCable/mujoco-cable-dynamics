#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

try:
    from view_cpp_plugin_demo import (
        _spool_debug_specs,
        cable_sensor_bindings,
        draw_spool_debug_visuals,
        draw_wrapped_tendon_visuals,
        read_cable_visual_states,
        update_cable_state_colors,
    )
except ModuleNotFoundError:
    from scripts.view_cpp_plugin_demo import (
        _spool_debug_specs,
        cable_sensor_bindings,
        draw_spool_debug_visuals,
        draw_wrapped_tendon_visuals,
        read_cable_visual_states,
        update_cable_state_colors,
    )


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLUGIN = ROOT / "build/plugin/libcable_unilateral.dylib"
DEFAULT_OUTPUT = ROOT / "website/public/media/videos"
TIMES_NEW_ROMAN = Path(
    "/System/Library/Fonts/Supplemental/Times New Roman.ttf"
)


@dataclass(frozen=True)
class DemoVideoSpec:
    label: str
    title: str
    model: str
    profile: str
    duration_s: float
    primary_actuator: str | None = None
    secondary_actuator: str | None = None
    primary_command: float = 0.0
    secondary_command: float = 0.0
    tracked_body: str | None = None
    external_body: str | None = None
    camera: str | None = "overview"
    description: str = ""
    method_tag: str = "native route"
    warmup_s: float = 0.0
    display_cables: tuple[str, ...] = ()
    primary_tendon: str | None = None
    tracked_axis_local: tuple[float, float, float] | None = None
    native_cable_width: float = 0.0022
    camera_distance_scale: float = 1.0
    fixed_camera_fovy: float | None = None
    overlay_position: str = "bottom"
    secondary_camera: str | None = None
    secondary_camera_fovy: float | None = None
    split_view_labels: tuple[str, str] = ("Oblique view", "Normal to route plane")


DEMO_SPECS = (
    DemoVideoSpec(
        "demo09_free_pulley",
        "Free payload over one pulley",
        "cable_plugin_demos/09_cpp_plugin_free_hanging_single_pulley.xml",
        "single_pulse",
        5.0,
        "winch_motor",
        primary_command=0.20,
        tracked_body="load",
        description="A physical spool joint and fixed pulley both participate in the solved surface envelope.",
        method_tag="surface route",
        fixed_camera_fovy=42.0,
    ),
    DemoVideoSpec(
        "demo10_dual_pulley",
        "Dual-pulley routed lift",
        "cable_plugin_demos/10_cpp_plugin_dual_pulley_free_payload.xml",
        "single_pulse",
        5.0,
        "winch_motor",
        primary_command=0.20,
        tracked_body="load",
        description="A surface route wraps the drum and two pulleys without a fixed bridge waypoint.",
        method_tag="surface route",
        fixed_camera_fovy=42.0,
    ),
    DemoVideoSpec(
        "demo11_reverse_reserve",
        "Reverse reserve release",
        "cable_plugin_demos/11_cpp_plugin_reverse_reserve_release.xml",
        "single_pulse",
        5.0,
        "winch_motor",
        primary_command=0.20,
        description="Signed reserve drives payout and rewinding while the route wraps both drum and pulley.",
        method_tag="surface route",
        fixed_camera_fovy=42.0,
    ),
    DemoVideoSpec(
        "demo12_frictional_pulley",
        "Frictional pulley lift",
        "cable_plugin_demos/12_cpp_plugin_frictional_pulley_free_payload.xml",
        "single_pulse",
        4.5,
        "frictional_pulley_cable_actuator",
        primary_command=0.018,
        tracked_body="load",
        description="Pulley wrapping and directional tension propagation act together.",
        fixed_camera_fovy=42.0,
    ),
    DemoVideoSpec(
        "demo13_cylinder_rolling",
        "Cylinder rolling-joint surrogate",
        "cable_plugin_demos/13_cpp_plugin_rolling_joint_figure_eight.xml",
        "ramp_hold",
        2.25,
        "upper_drive_contraction",
        "lower_drive_contraction",
        0.055,
        -0.006,
        "distal_link",
        description="Eight colored cables wrap two cylinders without a hinge or equality constraint.",
        method_tag="surface route",
        tracked_axis_local=(1.0, 0.0, 0.0),
    ),
    DemoVideoSpec(
        "demo14_mesh_rolling",
        "Convex-mesh rolling surrogate",
        "cable_plugin_demos/14_cpp_plugin_convex_mesh_rolling_joint.xml",
        "ramp_hold",
        2.5,
        "mesh_upper_drive",
        "mesh_lower_drive",
        0.020,
        -0.004,
        "mesh_distal_link",
        description="A fixed-side surface route follows closed convex mesh proxies.",
        method_tag="surface route",
        tracked_axis_local=(1.0, 0.0, 0.0),
    ),
    DemoVideoSpec(
        "demo15_surface_pulley",
        "Runtime surface-envelope pulley",
        "cable_plugin_demos/15_cpp_plugin_surface_single_pulley.xml",
        "single_pulse",
        4.5,
        "cable_contraction",
        primary_command=0.060,
        tracked_body="free_payload",
        description="The runtime cable route is tangent to the upper cylinder surface.",
        method_tag="surface route",
        fixed_camera_fovy=42.0,
    ),
    DemoVideoSpec(
        "demo16_passive_saddle",
        "Passive compliant saddle joint",
        "cable_plugin_demos/16_cpp_plugin_passive_saddle_joint.xml",
        "passive_torque",
        0.6,
        tracked_body="distal_metacarpal",
        external_body="distal_metacarpal",
        description="Two passive unilateral ligaments and nonconvex contact permit compliant motion.",
        method_tag="nonconvex mesh",
        display_cables=("figure_eight_upper", "figure_eight_lower"),
        tracked_axis_local=(1.0, 0.0, 0.0),
    ),
    DemoVideoSpec(
        "demo17_tensegrity",
        "Three-strut nine-cable tensegrity",
        "cable_plugin_demos/17_cpp_plugin_three_strut_nine_cable.xml",
        "single_pulse",
        4.5,
        "act_cross_00",
        primary_command=0.060,
        tracked_body="strut_0",
        description="Nine tension-only cables maintain and reshape a free three-strut structure.",
    ),
    DemoVideoSpec(
        "demo18_native_tensegrity",
        "Matched native-tendon tensegrity",
        "cable_plugin_demos/18_native_tendon_three_strut_nine_cable.xml",
        "native_rest_cycle",
        4.5,
        primary_command=0.060,
        tracked_body="strut_0",
        description="The matched MuJoCo tendon baseline changes cross_00 spring length.",
        method_tag="MuJoCo native tendon",
        primary_tendon="cross_00",
    ),
    DemoVideoSpec(
        "demo19_mixed_stiffness",
        "Mixed stiffness and slack",
        "cable_plugin_demos/19_cpp_plugin_mixed_stiffness_tensegrity.xml",
        "shorten_release",
        6.0,
        "act_cross_00",
        primary_command=0.055,
        secondary_command=-0.030,
        tracked_body="strut_0",
        description="A stiff controlled cross cable shortens, releases, and becomes slack.",
    ),
    DemoVideoSpec(
        "demo20_controlled_saddle",
        "Controlled nonconvex saddle joint",
        "cable_plugin_demos/20_cpp_plugin_controlled_saddle_joint.xml",
        "ramp_hold",
        3.0,
        "saddle_upper_control",
        "saddle_lower_control",
        0.006,
        -0.003,
        "distal_metacarpal",
        description="Passive figure-eight ligaments coexist with antagonistic control cables.",
        method_tag="nonconvex mesh",
        warmup_s=0.3,
        display_cables=("upper_control", "lower_control"),
        tracked_axis_local=(1.0, 0.0, 0.0),
    ),
    DemoVideoSpec(
        "demo21_wheel_axle",
        "Wheel-and-axle force amplifier",
        "cable_plugin_demos/21_cpp_plugin_wheel_axle_force_amplifier.xml",
        "single_pulse",
        5.0,
        "input_pull",
        primary_command=0.16,
        tracked_body="output_load",
        description="Opposite winding on a shared 60/20 mm shaft produces a 3:1 force ratio.",
        method_tag="joint spool",
    ),
    DemoVideoSpec(
        "demo24_faive_baseline",
        "Faive PIP virtual-hinge baseline",
        "cable_plugin_demos/24_faive_index_pip_virtual_hinge_baseline.xml",
        "ramp_hold",
        2.5,
        "pip_joint_command",
        primary_command=0.45,
        tracked_body="faive_index_mp",
        description="The published two-virtual-hinge approximation provides the kinematic baseline.",
        method_tag="MuJoCo joint baseline",
        fixed_camera_fovy=34.0,
        overlay_position="top",
    ),
    DemoVideoSpec(
        "demo25_faive_surface_cable",
        "Faive PIP surface-cable joint",
        "cable_plugin_demos/25_faive_index_pip_surface_cable.xml",
        "ramp_hold",
        2.5,
        "pip_flexor_command",
        primary_command=0.006,
        tracked_body="faive_index_mp",
        camera="split_oblique",
        description="Two right-side ligaments wrap the repaired nonconvex meshes while a thin flexor drives contact-guided rolling.",
        method_tag="guided nonconvex surface",
        warmup_s=0.1,
        display_cables=("pip_flexor", "ligament_right_upper"),
        native_cable_width=0.00032,
        fixed_camera_fovy=24.0,
        overlay_position="top",
        secondary_camera="route_normal",
        secondary_camera_fovy=60.0,
    ),
)


def smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def pulse_envelope(time_s: float, duration_s: float) -> float:
    ramp_start = 0.35
    ramp_end = min(1.45, 0.42 * duration_s)
    release_start = max(ramp_end + 0.45, duration_s - 0.85)
    if time_s <= ramp_start:
        return 0.0
    if time_s < ramp_end:
        return smoothstep((time_s - ramp_start) / (ramp_end - ramp_start))
    if time_s < release_start:
        return 1.0
    return 1.0 - smoothstep(
        (time_s - release_start) / max(duration_s - release_start, 1e-9)
    )


def resolve_id(
    model: mujoco.MjModel, kind: mujoco.mjtObj, name: str | None
) -> int:
    if not name:
        return -1
    object_id = mujoco.mj_name2id(model, kind, name)
    if object_id < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return object_id


def apply_profile(
    spec: DemoVideoSpec,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    primary_id: int,
    secondary_id: int,
    external_body_id: int,
    elapsed_s: float,
    primary_tendon_id: int = -1,
    native_rest_length: float = 0.0,
) -> dict[str, float]:
    data.ctrl[:] = 0
    data.xfrc_applied[:] = 0
    envelope = pulse_envelope(elapsed_s, spec.duration_s)
    primary = 0.0
    secondary = 0.0
    external_torque = 0.0
    if spec.profile == "single_pulse":
        primary = envelope * spec.primary_command
        secondary = envelope * spec.secondary_command
    elif spec.profile == "ramp_hold":
        ramp_end = min(1.0, 0.55 * spec.duration_s)
        fraction = smoothstep(
            (elapsed_s - 0.10) / max(ramp_end - 0.10, 1e-9)
        )
        primary = fraction * spec.primary_command
        secondary = fraction * spec.secondary_command
    elif spec.profile == "shorten_release":
        split = 0.53 * spec.duration_s
        if elapsed_s <= split:
            primary = pulse_envelope(
                elapsed_s, split
            ) * spec.primary_command
        else:
            local_time = elapsed_s - split
            local_duration = spec.duration_s - split
            primary = pulse_envelope(
                local_time, local_duration
            ) * spec.secondary_command
    elif spec.profile == "passive_torque":
        if 0.10 <= elapsed_s < spec.duration_s:
            external_torque = 0.002
        if external_body_id >= 0:
            data.xfrc_applied[external_body_id, 4] = external_torque
    elif spec.profile == "native_rest_cycle":
        primary = envelope * spec.primary_command
        if primary_tendon_id < 0:
            raise ValueError("native_rest_cycle requires primary_tendon")
        current_rest = native_rest_length - primary
        model.tendon_lengthspring[primary_tendon_id, 0] = current_rest
        model.tendon_lengthspring[primary_tendon_id, 1] = current_rest
    else:
        raise ValueError(f"Unknown video profile: {spec.profile}")

    if primary_id >= 0:
        data.ctrl[primary_id] = primary
    if secondary_id >= 0:
        data.ctrl[secondary_id] = secondary
    return {
        "primary": primary,
        "secondary": secondary,
        "external_torque": external_torque,
        "native_tension": 0.0,
    }


def font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    candidates = (
        Path(
            "/System/Library/Fonts/Supplemental/Times New Roman Bold.ttf"
            if bold
            else TIMES_NEW_ROMAN
        ),
        Path("/System/Library/Fonts/Supplemental/Arial.ttf"),
    )
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size)
    return ImageFont.load_default()


def rotation_degrees(
    data: mujoco.MjData,
    body_id: int,
    initial_quaternion: np.ndarray | None,
    tracked_axis_local: tuple[float, float, float] | None = None,
    initial_axis_world: np.ndarray | None = None,
) -> float:
    if body_id < 0 or initial_quaternion is None:
        return 0.0
    if tracked_axis_local is not None and initial_axis_world is not None:
        rotation = np.asarray(data.xmat[body_id], dtype=float).reshape(3, 3)
        current_axis = rotation @ np.asarray(tracked_axis_local, dtype=float)
        current_axis /= max(float(np.linalg.norm(current_axis)), 1e-12)
        cosine_angle = float(
            np.clip(np.dot(current_axis, initial_axis_world), -1.0, 1.0)
        )
        return math.degrees(math.acos(cosine_angle))
    current = np.asarray(data.xquat[body_id], dtype=float)
    cosine_half_angle = float(
        np.clip(abs(np.dot(current, initial_quaternion)), 0.0, 1.0)
    )
    return math.degrees(2.0 * math.acos(cosine_half_angle))


def annotate_frame(
    pixels: np.ndarray,
    spec: DemoVideoSpec,
    data: mujoco.MjData,
    controls: dict[str, float],
    cable_states,
    rotation_deg: float,
    elapsed_s: float,
) -> Image.Image:
    image = Image.fromarray(pixels)
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    width, height = image.size

    selected_states = cable_states
    if spec.display_cables:
        selected_states = [
            state
            for name in spec.display_cables
            for state in cable_states
            if state.binding.name == name
        ]
    state_lines = []
    for state in selected_states[:2]:
        status = "TAUT" if state.taut else "SLACK"
        state_lines.append(
            f"{state.binding.name}: L={1000.0 * state.length:.1f} mm, "
            f"T={state.tension:.2f} N, {status}"
        )
    control_text = f"u1={controls['primary']:+.4f}"
    if abs(controls["secondary"]) > 1e-10:
        control_text += f", u2={controls['secondary']:+.4f}"
    if abs(controls["external_torque"]) > 1e-10:
        control_text = f"external torque={1000.0 * controls['external_torque']:+.2f} mN m"
    if controls["native_tension"] > 0:
        control_text += f"  |  T={controls['native_tension']:.2f} N"
    if spec.tracked_body:
        motion_label = (
            "axis angle" if spec.tracked_axis_local is not None else "rotation"
        )
        control_text += f"  |  {motion_label}={rotation_deg:.1f} deg"
    state_lines.insert(0, control_text)

    box_height = 74 + 24 * len(state_lines)
    panel_width = width // 2 if spec.secondary_camera else width
    box_width = min(panel_width - 40, 650)
    top = 20 if spec.overlay_position == "top" else height - box_height - 20
    draw.rectangle(
        (20, top, 20 + box_width, top + box_height),
        fill=(8, 12, 17, 196),
        outline=(80, 97, 108, 220),
        width=1,
    )
    draw.text(
        (34, top + 10),
        spec.title,
        fill=(245, 247, 249, 255),
        font=font(25, True),
    )
    draw.text(
        (35, top + 39),
        f"{spec.method_tag}  |  t = {elapsed_s:4.2f} s",
        fill=(170, 190, 202, 255),
        font=font(16),
    )
    for index, line in enumerate(state_lines):
        draw.text(
            (34, top + 66 + 24 * index),
            line,
            fill=(235, 239, 242, 255),
            font=font(16),
        )
    draw.text(
        (width - 245, 20),
        "MuJoCo Cable Dynamics",
        fill=(186, 198, 205, 230),
        font=font(15),
    )
    if spec.secondary_camera:
        divider_x = width // 2
        draw.line((divider_x, 0, divider_x, height), fill=(210, 220, 226, 210), width=2)
        for index, label in enumerate(spec.split_view_labels):
            label_x = 20 + index * divider_x
            label_y = height - 39
            label_box = draw.textbbox((label_x, label_y), label, font=font(16, True))
            draw.rectangle(
                (label_box[0] - 8, label_box[1] - 5, label_box[2] + 8, label_box[3] + 5),
                fill=(8, 12, 17, 190),
            )
            draw.text(
                (label_x, label_y),
                label,
                fill=(240, 244, 247, 255),
                font=font(16, True),
            )
    return Image.alpha_composite(image.convert("RGBA"), overlay).convert("RGB")


def free_camera(
    model: mujoco.MjModel, distance_scale: float = 1.0
) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.lookat[:] = np.asarray(model.stat.center, dtype=float)
    camera.distance = 1.35 * distance_scale * float(model.stat.extent)
    camera.azimuth = 90
    camera.elevation = -18
    return camera


def encode_video(frames_dir: Path, output: Path, fps: int) -> None:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg not found")
    output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            ffmpeg,
            "-y",
            "-loglevel",
            "error",
            "-framerate",
            str(fps),
            "-i",
            str(frames_dir / "frame_%05d.png"),
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "24",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(output),
        ],
        check=True,
    )


def record_one(
    spec: DemoVideoSpec,
    plugin: Path,
    output_dir: Path,
    width: int,
    height: int,
    fps: int,
) -> dict[str, object]:
    model_path = ROOT / spec.model
    model = mujoco.MjModel.from_xml_path(str(model_path))
    if spec.camera and spec.fixed_camera_fovy is not None:
        camera_id = resolve_id(model, mujoco.mjtObj.mjOBJ_CAMERA, spec.camera)
        model.cam_fovy[camera_id] = spec.fixed_camera_fovy
    if spec.secondary_camera and spec.secondary_camera_fovy is not None:
        camera_id = resolve_id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, spec.secondary_camera
        )
        model.cam_fovy[camera_id] = spec.secondary_camera_fovy
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), width)
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), height)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    bindings = cable_sensor_bindings(model, model_path)
    base_tendon_rgba = np.asarray(model.tendon_rgba, dtype=float).copy()
    primary_id = resolve_id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, spec.primary_actuator
    )
    secondary_id = resolve_id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, spec.secondary_actuator
    )
    tracked_body_id = resolve_id(
        model, mujoco.mjtObj.mjOBJ_BODY, spec.tracked_body
    )
    external_body_id = resolve_id(
        model, mujoco.mjtObj.mjOBJ_BODY, spec.external_body
    )
    primary_tendon_id = resolve_id(
        model, mujoco.mjtObj.mjOBJ_TENDON, spec.primary_tendon
    )
    native_rest_length = (
        float(model.tendon_lengthspring[primary_tendon_id, 0])
        if primary_tendon_id >= 0
        else 0.0
    )
    if spec.warmup_s > 0:
        warmup_end = float(data.time) + spec.warmup_s
        while data.time < warmup_end:
            data.ctrl[:] = 0
            data.xfrc_applied[:] = 0
            mujoco.mj_step(model, data)
    initial_quaternion = (
        np.asarray(data.xquat[tracked_body_id], dtype=float).copy()
        if tracked_body_id >= 0
        else None
    )
    initial_position = (
        np.asarray(data.xpos[tracked_body_id], dtype=float).copy()
        if tracked_body_id >= 0
        else None
    )
    initial_axis_world = None
    if tracked_body_id >= 0 and spec.tracked_axis_local is not None:
        initial_rotation = np.asarray(
            data.xmat[tracked_body_id], dtype=float
        ).reshape(3, 3)
        initial_axis_world = initial_rotation @ np.asarray(
            spec.tracked_axis_local, dtype=float
        )
        initial_axis_world /= max(
            float(np.linalg.norm(initial_axis_world)), 1e-12
        )
    recording_start = float(data.time)

    visual_tendon_ids = [
        tendon_id
        for tendon_id in range(model.ntendon)
        if float(model.tendon_width[tendon_id]) > 1e-8
    ]
    native_visual = bool(visual_tendon_ids)
    if native_visual:
        for tendon_id in visual_tendon_ids:
            model.tendon_width[tendon_id] = 0
    spool_specs = _spool_debug_specs(model, model_path)

    if spec.secondary_camera and width % 2:
        raise ValueError("split-view recordings require an even output width")
    render_width = width // 2 if spec.secondary_camera else width
    renderer = mujoco.Renderer(
        model, height=height, width=render_width, max_geom=20000
    )
    secondary_renderer = (
        mujoco.Renderer(
            model, height=height, width=render_width, max_geom=20000
        )
        if spec.secondary_camera
        else None
    )
    camera = (
        spec.camera
        if spec.camera and model.ncam
        else free_camera(model, spec.camera_distance_scale)
    )
    secondary_camera = spec.secondary_camera

    def render_view(view_renderer: mujoco.Renderer, view_camera) -> np.ndarray:
        view_renderer.update_scene(data, camera=view_camera)
        if native_visual:
            draw_wrapped_tendon_visuals(
                model,
                data,
                view_renderer.scene,
                tendon_ids=visual_tendon_ids,
                cable_width=spec.native_cable_width,
                arc_line_width=4.0,
                arc_segments=42,
                show_tension_arrows=False,
                show_spool_wrap=False,
                spool_geom_name="",
                spool_anchor_site="",
                clear_scene=False,
            )
        if spool_specs:
            draw_spool_debug_visuals(
                model,
                data,
                view_renderer.scene,
                spool_specs,
                segments_per_turn=72,
            )
        return view_renderer.render()
    frame_period = 1.0 / fps
    next_frame_time = 0.0
    frame_index = 0
    maximum_rotation = 0.0
    maximum_displacement = 0.0
    maximum_tension = 0.0
    maximum_route_status = 0
    contact_frames = 0
    sampled_frames = 0
    poster: Image.Image | None = None
    poster_score = -1.0
    current_displacement = 0.0
    frames_dir = Path(tempfile.mkdtemp(prefix=f"{spec.label}_frames_"))
    try:
        while float(data.time) - recording_start < spec.duration_s:
            elapsed_s = float(data.time) - recording_start
            controls = apply_profile(
                spec,
                model,
                data,
                primary_id,
                secondary_id,
                external_body_id,
                elapsed_s,
                primary_tendon_id,
                native_rest_length,
            )
            mujoco.mj_step(model, data)
            if primary_tendon_id >= 0:
                native_tension = (
                    float(model.tendon_stiffness[primary_tendon_id])
                    * (
                        float(data.ten_length[primary_tendon_id])
                        - float(model.tendon_lengthspring[primary_tendon_id, 0])
                    )
                    + float(model.tendon_damping[primary_tendon_id])
                    * float(data.ten_velocity[primary_tendon_id])
                )
                controls["native_tension"] = native_tension
            states = read_cable_visual_states(data, bindings)
            update_cable_state_colors(model, states, base_tendon_rgba)
            rotation = rotation_degrees(
                data,
                tracked_body_id,
                initial_quaternion,
                spec.tracked_axis_local,
                initial_axis_world,
            )
            maximum_rotation = max(maximum_rotation, rotation)
            if tracked_body_id >= 0 and initial_position is not None:
                current_displacement = float(
                    np.linalg.norm(
                        np.asarray(data.xpos[tracked_body_id], dtype=float)
                        - initial_position
                    )
                )
                maximum_displacement = max(
                    maximum_displacement, current_displacement
                )
            if states:
                maximum_tension = max(
                    maximum_tension, max(state.tension for state in states)
                )
                for binding in bindings:
                    if binding.sensor_dimension >= 12:
                        maximum_route_status = max(
                            maximum_route_status,
                            int(
                                round(
                                    data.sensordata[
                                        binding.sensor_address + 8
                                    ]
                                )
                            ),
                        )
            if primary_tendon_id >= 0:
                maximum_tension = max(
                    maximum_tension, max(0.0, controls["native_tension"])
                )

            if data.time + 1e-12 < next_frame_time:
                continue
            pixels = render_view(renderer, camera)
            if secondary_renderer is not None:
                secondary_pixels = render_view(
                    secondary_renderer, secondary_camera
                )
                pixels = np.concatenate((pixels, secondary_pixels), axis=1)
            frame = annotate_frame(
                pixels,
                spec,
                data,
                controls,
                states,
                rotation,
                float(data.time) - recording_start,
            )
            frame.save(frames_dir / f"frame_{frame_index:05d}.png")
            score = max(
                rotation,
                1000.0 * current_displacement,
                120.0 * abs(controls["primary"]),
                120.0 * abs(controls["external_torque"]),
            )
            if score > poster_score:
                poster = frame.copy()
                poster_score = score
            frame_index += 1
            sampled_frames += 1
            contact_frames += int(data.ncon > 0)
            next_frame_time += frame_period
    finally:
        renderer.close()
        if secondary_renderer is not None:
            secondary_renderer.close()

    output_path = output_dir / f"{spec.label}.mp4"
    poster_path = output_dir / f"{spec.label}.jpg"
    encode_video(frames_dir, output_path, fps)
    if poster is not None:
        poster.save(poster_path, quality=91, optimize=True)
    shutil.rmtree(frames_dir)
    summary = {
        **asdict(spec),
        "video": f"/media/videos/{output_path.name}",
        "poster": f"/media/videos/{poster_path.name}",
        "fps": fps,
        "frames": frame_index,
        "resolution": [width, height],
        "view_layout": "split" if spec.secondary_camera else "single",
        "maximum_rotation_deg": maximum_rotation,
        "rotation_metric": (
            "body_axis_change"
            if spec.tracked_axis_local is not None
            else "quaternion_distance"
        ),
        "maximum_displacement_mm": 1000.0 * maximum_displacement,
        "maximum_tension_N": maximum_tension,
        "maximum_route_status": maximum_route_status,
        "contact_frame_fraction": (
            contact_frames / sampled_frames if sampled_frames else 0.0
        ),
        "video_size_bytes": output_path.stat().st_size,
    }
    print(json.dumps(summary, indent=2))
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Record MuJoCo-rendered videos for the C++ cable plugin demos"
    )
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--demo",
        action="append",
        help="Record only the named demo label; may be repeated",
    )
    args = parser.parse_args()
    plugin = args.plugin.resolve()
    if not plugin.exists():
        raise FileNotFoundError(f"Plugin not found: {plugin}")
    if args.width < 320 or args.height < 240 or args.fps < 1:
        parser.error("invalid video dimensions or frame rate")
    selected = [
        spec
        for spec in DEMO_SPECS
        if not args.demo or spec.label in set(args.demo)
    ]
    if args.demo and len(selected) != len(set(args.demo)):
        known = ", ".join(spec.label for spec in DEMO_SPECS)
        parser.error(f"unknown demo label; known labels: {known}")

    mujoco.mj_loadPluginLibrary(str(plugin))
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    summaries = [
        record_one(spec, plugin, output, args.width, args.height, args.fps)
        for spec in selected
    ]
    manifest_path = output.parent / "demo_manifest.json"
    previous: dict[str, object] = {}
    if args.demo and manifest_path.exists():
        existing = json.loads(manifest_path.read_text(encoding="utf-8"))
        previous = {
            str(item["label"]): item for item in existing.get("videos", [])
        }
    previous.update({str(item["label"]): item for item in summaries})
    ordered_summaries = [
        previous[spec.label] for spec in DEMO_SPECS if spec.label in previous
    ]
    manifest = {
        "plugin": args.plugin.name,
        "video_count": len(ordered_summaries),
        "videos": ordered_summaries,
    }
    manifest_path.write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({"manifest": str(manifest_path), "count": len(summaries)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
