#!/usr/bin/env python3
"""Run the frozen Result 2 comparisons, friction sweep, figures, and videos."""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import matplotlib as mpl

mpl.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
FONT = Path("/System/Library/Fonts/Supplemental/Times New Roman.ttf")
PAPER_TARGET_DEG = 530.0
PAPER_JOINT_LIMIT_DEG = 30.0
FRICTIONS = (0.0, 0.15, 0.30, 0.45, 0.60)
# Sequential blue-green colors for ascending friction.  Luminance decreases
# monotonically with mu, preserving the ordering in grayscale reproduction.
FRICTION_COLORS = ("#D9F0D3", "#8FD4C1", "#41B6C4", "#225EA8", "#084081")
SAMPLE_PERIOD = 0.02


@dataclass
class Simulation:
    name: str
    rows: list[dict[str, float | int | str]]
    summary: dict[str, float | int | str]
    final_qpos: np.ndarray | None = None
    final_ctrl: np.ndarray | None = None


def configure_plotting() -> None:
    if FONT.exists():
        mpl.font_manager.fontManager.addfont(str(FONT))
    mpl.rcParams.update(
        {
            "font.family": "Times New Roman" if FONT.exists() else "serif",
            "mathtext.fontset": "custom",
            "mathtext.rm": "Times New Roman",
            "mathtext.it": "Times New Roman:italic",
            "mathtext.bf": "Times New Roman:bold",
            "font.size": 10,
            "axes.labelsize": 10,
            "axes.titlesize": 10,
            "legend.fontsize": 9,
            "xtick.labelsize": 10,
            "ytick.labelsize": 10,
            "axes.linewidth": 0.8,
            "lines.linewidth": 1.5,
            "lines.markersize": 4.5,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "none",
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
        }
    )


def style_axis(axis: plt.Axes) -> None:
    axis.grid(True, color="#CDD2D6", linewidth=0.5)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)
    axis.tick_params(direction="out", length=3, width=0.7)


def export_figure(figure: plt.Figure, stem: Path) -> None:
    stem.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(stem.with_suffix(".pdf"), bbox_inches="tight")
    figure.savefig(stem.with_suffix(".svg"), bbox_inches="tight")
    figure.savefig(stem.with_suffix(".png"), dpi=600, bbox_inches="tight")
    plt.close(figure)


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def one_side_target(time_s: float) -> tuple[float, str]:
    if time_s < 1.0:
        return 0.0, "neutral"
    if time_s < 5.0:
        return -PAPER_TARGET_DEG * smoothstep((time_s - 1.0) / 4.0), "bend_right"
    return -PAPER_TARGET_DEG, "hold_right"


def sensor_value(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    if sensor_id < 0:
        raise ValueError(f"sensor not found: {name}")
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    return np.asarray(data.sensordata[address : address + dimension], dtype=float).copy()


def summarize_spirobot(rows: list[dict[str, float | int | str]]) -> dict[str, float | int | str]:
    hold = [row for row in rows if row["phase"] == "hold_right"]
    if not hold:
        hold = rows[-20:]
    bend = np.asarray([float(row["bend_deg"]) for row in rows])
    target = np.asarray([float(row["paper_target_deg"]) for row in rows])
    return {
        "final_bend_deg": float(bend[-1]),
        "hold_bend_deg": float(np.mean([float(row["bend_deg"]) for row in hold])),
        "tracking_rmse_deg": float(np.sqrt(np.mean((bend - target) ** 2))),
        "maximum_active_tension_N": float(max(float(row["active_tension_N"]) for row in rows)),
        "maximum_active_cable_takeup_mm": float(max(float(row["active_cable_takeup_mm"]) for row in rows)),
        "minimum_active_cable_takeup_mm": float(min(float(row["active_cable_takeup_mm"]) for row in rows)),
        "maximum_route_status": int(max(int(row["route_status"]) for row in rows)),
        "mean_hold_joint_angle_deg": float(
            np.mean(
                [
                    float(row[f"joint_{joint:02d}_angle_deg"])
                    for row in hold
                    for joint in range(1, 19)
                ]
            )
        ),
    }


class VideoFrames:
    def __init__(self, output: Path, width: int, height: int, fps: int, title: str):
        self.output = output
        self.width = width
        self.height = height
        self.fps = fps
        self.title = title
        self.temporary = Path(tempfile.mkdtemp(prefix="result2_video_"))
        self.raw = self.temporary / "raw"
        self.annotated = self.temporary / "annotated"
        self.raw.mkdir()
        self.annotated.mkdir()
        self.index = 0

    @staticmethod
    def font(size: int, bold: bool = False) -> ImageFont.ImageFont:
        candidates = (
            Path("/System/Library/Fonts/Supplemental/Times New Roman Bold.ttf")
            if bold
            else FONT,
            Path("/System/Library/Fonts/Supplemental/Arial Bold.ttf")
            if bold
            else Path("/System/Library/Fonts/Supplemental/Arial.ttf"),
        )
        for candidate in candidates:
            if candidate.exists():
                return ImageFont.truetype(str(candidate), size)
        return ImageFont.load_default()

    def add(
        self,
        pixels: np.ndarray,
        time_s: float,
        bend_deg: float,
        target_deg: float,
        cable_value: float,
        tension_N: float,
    ) -> None:
        raw = Image.fromarray(pixels).convert("RGB")
        raw.save(self.raw / f"frame_{self.index:05d}.png", compress_level=2)
        overlay = Image.new("RGBA", raw.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        draw.rectangle((20, self.height - 124, 600, self.height - 20), fill=(8, 12, 17, 196))
        draw.text((34, self.height - 116), self.title, fill="white", font=self.font(23, True))
        draw.text(
            (34, self.height - 82),
            f"t={time_s:4.2f} s | bend={bend_deg:6.1f} deg | target={target_deg:6.1f} deg",
            fill=(225, 232, 237),
            font=self.font(16),
        )
        draw.text(
            (34, self.height - 55),
            f"cable={cable_value:6.1f} mm | force={tension_N:5.2f} N",
            fill=(225, 232, 237),
            font=self.font(16),
        )
        draw.text(
            (self.width - 170, 20),
            "MuJoCable",
            fill=(210, 218, 223),
            font=self.font(16, True),
        )
        annotated = Image.alpha_composite(raw.convert("RGBA"), overlay).convert("RGB")
        annotated.save(self.annotated / f"frame_{self.index:05d}.png", compress_level=2)
        self.index += 1

    def close(self) -> tuple[Path, Path]:
        self.output.mkdir(parents=True, exist_ok=True)
        raw_video = self.output / f"{self.title.lower().replace(' ', '_')}_raw.mp4"
        annotated_video = self.output / f"{self.title.lower().replace(' ', '_')}_annotated.mp4"
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            raise RuntimeError("ffmpeg is required for Result 2 videos")
        for source, destination in ((self.raw, raw_video), (self.annotated, annotated_video)):
            subprocess.run(
                [
                    ffmpeg,
                    "-y",
                    "-loglevel",
                    "error",
                    "-framerate",
                    str(self.fps),
                    "-i",
                    str(source / "frame_%05d.png"),
                    "-c:v",
                    "libx264",
                    "-preset",
                    "medium",
                    "-crf",
                    "22",
                    "-pix_fmt",
                    "yuv420p",
                    "-movflags",
                    "+faststart",
                    str(destination),
                ],
                check=True,
            )
        shutil.rmtree(self.temporary)
        return raw_video, annotated_video


class AdaptivePlanarCamera:
    """Keep all robot geoms visible while using the available image area."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, width: int, height: int):
        self.model = model
        self.camera_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, "bending_plane"
        )
        if self.camera_id < 0:
            raise ValueError("bending_plane camera not found")
        self.aspect = width / height
        model.cam_fovy[self.camera_id] = 45.0
        self.center, self.distance = self.target(data)
        self.apply()

    def target(self, data: mujoco.MjData) -> tuple[np.ndarray, float]:
        radii = np.asarray(self.model.geom_rbound, dtype=float)
        centers = np.asarray(data.geom_xpos, dtype=float)
        valid = np.isfinite(radii) & (radii > 0.0)
        centers = centers[valid]
        radii = radii[valid]
        lower = np.min(centers - radii[:, None], axis=0)
        upper = np.max(centers + radii[:, None], axis=0)
        center = 0.5 * (lower + upper)
        half_x = max(0.5 * (upper[0] - lower[0]), 0.035)
        half_y = max(0.5 * (upper[1] - lower[1]), 0.035)
        tangent = math.tan(math.radians(float(self.model.cam_fovy[self.camera_id])) / 2.0)
        fill = 0.78
        distance = max(
            half_y / (fill * tangent),
            half_x / (fill * tangent * self.aspect),
            0.09,
        )
        return center, distance

    def update(self, data: mujoco.MjData) -> None:
        target_center, target_distance = self.target(data)
        self.center = 0.82 * self.center + 0.18 * target_center
        if target_distance > self.distance:
            self.distance = target_distance
        else:
            self.distance = 0.90 * self.distance + 0.10 * target_distance
        self.apply()

    def apply(self) -> None:
        self.model.cam_pos[self.camera_id] = self.center
        self.model.cam_pos[self.camera_id, 2] += self.distance


def simulate_plugin_spirobot(
    cycle,
    model_path: Path,
    name: str,
    duration: float,
    sample_period: float,
    video: VideoFrames | None = None,
) -> Simulation:
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    data = mujoco.MjData(model)
    reserve = 0.090
    slack = 0.00015
    preload = 0.0010
    drive = 0.015
    home, reference_lengths, reference_contractions = cycle.prestress(
        model, data, reserve, slack, preload, 2.0
    )
    reference_qpos = np.asarray(data.qpos, dtype=float).copy()
    rows: list[dict[str, float | int | str]] = []
    next_sample = 0.0
    next_frame = 0.0
    renderer = None
    camera = None
    if video is not None:
        renderer = mujoco.Renderer(model, height=video.height, width=video.width, max_geom=20000)
        camera = AdaptivePlanarCamera(model, data, video.width, video.height)
    try:
        while data.time < duration - 0.5 * model.opt.timestep:
            right, left = cycle.states(model, data)
            route_lengths = np.asarray((right[0], left[0]))
            bend_before = float(np.degrees(np.sum(np.asarray(data.qpos) - reference_qpos)))
            bend_rate = float(np.degrees(np.sum(np.asarray(data.qvel))))
            target, phase = one_side_target(float(data.time))
            error = target - bend_before
            signed_level = float(np.clip(-error / 30.0 + bend_rate / 180.0, -1.0, 1.0))
            data.ctrl[:] = cycle.target_commands(
                route_lengths,
                home,
                reserve,
                slack,
                preload,
                drive,
                signed_level,
                model.actuator_ctrlrange,
            )
            start = time.perf_counter()
            mujoco.mj_step(model, data)
            step_ms = 1000.0 * (time.perf_counter() - start)
            if data.time + 1.0e-12 >= next_sample:
                right, left = cycle.states(model, data)
                qpos_delta = np.asarray(data.qpos) - reference_qpos
                row: dict[str, float | int | str] = {
                    "model": name,
                    "time_s": float(data.time),
                    "phase": phase,
                    "paper_target_deg": target,
                    "bend_deg": float(np.degrees(np.sum(qpos_delta))),
                    "active_cable_takeup_mm": 1000.0 * float(right[3] - reference_contractions[0]),
                    "antagonist_cable_takeup_mm": 1000.0 * float(left[3] - reference_contractions[1]),
                    "active_route_change_mm": 1000.0 * float(right[0] - reference_lengths[0]),
                    "active_tension_N": float(right[5]),
                    "antagonist_tension_N": float(left[5]),
                    "route_status": int(max(round(right[8]), round(left[8]))),
                    "step_ms": step_ms,
                }
                for joint, angle in enumerate(np.degrees(qpos_delta), start=1):
                    row[f"joint_{joint:02d}_angle_deg"] = float(angle)
                rows.append(row)
                next_sample += sample_period
            if renderer is not None and video is not None and data.time + 1.0e-12 >= next_frame:
                right, _ = cycle.states(model, data)
                bend = float(np.degrees(np.sum(np.asarray(data.qpos) - reference_qpos)))
                camera.update(data)
                renderer.update_scene(data, camera="bending_plane")
                video.add(
                    renderer.render(),
                    float(data.time),
                    bend,
                    target,
                    1000.0 * float(right[3] - reference_contractions[0]),
                    float(right[5]),
                )
                next_frame += 1.0 / video.fps
    finally:
        if renderer is not None:
            renderer.close()
    return Simulation(
        name,
        rows,
        summarize_spirobot(rows),
        np.asarray(data.qpos, dtype=float).copy(),
        np.asarray(data.ctrl, dtype=float).copy(),
    )


def simulate_native_spirobot(
    model_path: Path,
    name: str,
    duration: float,
    sample_period: float,
    video: VideoFrames | None = None,
) -> Simulation:
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    positive_tendon = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "positive_seed")
    negative_tendon = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "negative_seed")
    reference_lengths = np.asarray(
        (data.ten_length[positive_tendon], data.ten_length[negative_tendon]), dtype=float
    )
    reference_qpos = np.asarray(data.qpos, dtype=float).copy()
    rows: list[dict[str, float | int | str]] = []
    next_sample = 0.0
    next_frame = 0.0
    renderer = None
    camera = None
    if video is not None:
        renderer = mujoco.Renderer(model, height=video.height, width=video.width, max_geom=20000)
        camera = AdaptivePlanarCamera(model, data, video.width, video.height)
    try:
        while data.time < duration - 0.5 * model.opt.timestep:
            bend_before = float(np.degrees(np.sum(np.asarray(data.qpos) - reference_qpos)))
            bend_rate = float(np.degrees(np.sum(np.asarray(data.qvel))))
            target, phase = one_side_target(float(data.time))
            error = target - bend_before
            signed_level = float(np.clip(-error / 45.0 + bend_rate / 240.0, -1.0, 1.0))
            data.ctrl[0] = max(0.0, signed_level)
            data.ctrl[1] = max(0.0, -signed_level)
            start = time.perf_counter()
            mujoco.mj_step(model, data)
            step_ms = 1000.0 * (time.perf_counter() - start)
            if data.time + 1.0e-12 >= next_sample:
                qpos_delta = np.asarray(data.qpos) - reference_qpos
                row: dict[str, float | int | str] = {
                    "model": name,
                    "time_s": float(data.time),
                    "phase": phase,
                    "paper_target_deg": target,
                    "bend_deg": float(np.degrees(np.sum(qpos_delta))),
                    "active_cable_takeup_mm": -1000.0 * float(
                        data.ten_length[positive_tendon] - reference_lengths[0]
                    ),
                    "antagonist_cable_takeup_mm": -1000.0 * float(
                        data.ten_length[negative_tendon] - reference_lengths[1]
                    ),
                    "active_route_change_mm": 1000.0 * float(
                        data.ten_length[positive_tendon] - reference_lengths[0]
                    ),
                    "active_tension_N": abs(float(data.actuator_force[0])),
                    "antagonist_tension_N": abs(float(data.actuator_force[1])),
                    "route_status": -1,
                    "step_ms": step_ms,
                }
                for joint, angle in enumerate(np.degrees(qpos_delta), start=1):
                    row[f"joint_{joint:02d}_angle_deg"] = float(angle)
                rows.append(row)
                next_sample += sample_period
            if renderer is not None and video is not None and data.time + 1.0e-12 >= next_frame:
                bend = float(np.degrees(np.sum(np.asarray(data.qpos) - reference_qpos)))
                camera.update(data)
                renderer.update_scene(data, camera="bending_plane")
                video.add(
                    renderer.render(),
                    float(data.time),
                    bend,
                    target,
                    -1000.0 * float(data.ten_length[positive_tendon] - reference_lengths[0]),
                    abs(float(data.actuator_force[0])),
                )
                next_frame += 1.0 / video.fps
    finally:
        if renderer is not None:
            renderer.close()
    return Simulation(
        name,
        rows,
        summarize_spirobot(rows),
        np.asarray(data.qpos, dtype=float).copy(),
        np.asarray(data.ctrl, dtype=float).copy(),
    )


def run_faive(compare_module, output: Path) -> tuple[list[dict[str, object]], dict[str, object]]:
    baseline = output / "models/result_2_1a_ideal_virtual_hinge.xml"
    surface = output / "models/result_2_1b_mujocable_surface.xml"
    baseline_rows = compare_module.run_actuation(
        "virtual_hinge", baseline, 2.5, math.radians(25.0), 0.010, 0.10, 0.80
    )
    surface_rows = compare_module.run_actuation(
        "surface_cable", surface, 2.5, math.radians(25.0), 0.010, 0.10, 0.80
    )
    rows = baseline_rows + surface_rows
    summary = compare_module.summarize(rows, math.radians(25.0), 0.010, False)
    write_csv(output / "data/result_2_1_faive_comparison.csv", rows)
    (output / "data/result_2_1_faive_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    figure, axis = plt.subplots(figsize=(3.5, 2.65), constrained_layout=True)
    for label, color, line, marker, legend in (
        ("virtual_hinge", "#666666", "-", None, "Virtual-hinge reference"),
        ("surface_cable", "#0072B2", "--", "o", "MuJoCable"),
    ):
        selected = [row for row in rows if row["model"] == label]
        axis.plot(
            [float(row["time_s"]) for row in selected],
            [float(row["angle_deg"]) for row in selected],
            color=color,
            linestyle=line,
            marker=marker,
            markevery=max(1, len(selected) // 12),
            label=legend,
            zorder=3 if label == "surface_cable" else 2,
        )
    axis.axhline(50.0, color="#999999", linestyle=":", linewidth=0.9, label="50 deg target")
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("PIP rotation (deg)")
    axis.legend(frameon=False)
    style_axis(axis)
    export_figure(figure, output / "figures/result_2_1_faive_50deg_comparison")
    return rows, summary


def plot_result_2_2(plugin: Simulation, native: Simulation, output: Path) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(7.16, 2.75), constrained_layout=True)
    for simulation, color, line, marker, label, zorder in (
        (native, "#D55E00", "-.", "s", "Native tendon + muscle", 3),
        (plugin, "#0072B2", "--", "o", "MuJoCable, mu=0.15", 4),
    ):
        time_s = np.asarray([float(row["time_s"]) for row in simulation.rows])
        bend = np.asarray([float(row["bend_deg"]) for row in simulation.rows])
        axes[0].plot(
            time_s,
            bend,
            color=color,
            linestyle=line,
            marker=marker,
            markevery=max(1, len(time_s) // 14),
            label=label,
            zorder=zorder,
        )
    time_s = np.asarray([float(row["time_s"]) for row in plugin.rows])
    reference = np.asarray([float(row["paper_target_deg"]) for row in plugin.rows])
    axes[0].plot(time_s, reference, color="#666666", linestyle="-", label="Paper target", zorder=2)
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Total bending angle (deg)")
    axes[0].legend(frameon=False)

    joints = np.arange(1, 19)
    axes[1].plot(joints, -PAPER_JOINT_LIMIT_DEG * np.ones(18), color="#666666", label="Paper packed limit")
    for simulation, color, line, marker, label, zorder in (
        (native, "#D55E00", "-.", "s", "Native tendon + muscle", 3),
        (plugin, "#0072B2", "--", "o", "MuJoCable, mu=0.15", 4),
    ):
        hold = [row for row in simulation.rows if row["phase"] == "hold_right"]
        values = [
            np.mean([float(row[f"joint_{joint:02d}_angle_deg"]) for row in hold])
            for joint in joints
        ]
        axes[1].plot(
            joints,
            values,
            color=color,
            linestyle=line,
            marker=marker,
            label=label,
            zorder=zorder,
        )
    axes[1].set_xlabel("Joint index from base")
    axes[1].set_ylabel("Packed-state joint angle (deg)")
    axes[1].set_xticks((1, 4, 7, 10, 13, 16, 18))
    axes[1].legend(frameon=False)
    for label, axis in zip(("(a)", "(b)"), axes):
        style_axis(axis)
        axis.text(-0.17, 1.04, label, transform=axis.transAxes, fontweight="bold")
    export_figure(figure, output / "figures/result_2_2_native_vs_mujocable")


def plot_result_2_3(simulations: list[Simulation], output: Path) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(7.16, 2.8), constrained_layout=True)
    long_rows: list[dict[str, object]] = []
    for index, (mu, simulation) in enumerate(zip(FRICTIONS, simulations)):
        selected = [row for row in simulation.rows if row["phase"] == "bend_right"]
        takeup = np.asarray([float(row["active_cable_takeup_mm"]) for row in selected])
        tension = np.asarray([float(row["active_tension_N"]) for row in selected])
        bend = np.asarray([float(row["bend_deg"]) for row in selected])
        marker = ("o", "s", "^", "D", "v")[index]
        markevery = max(1, len(selected) // 12)
        axes[0].plot(
            takeup,
            bend,
            color=FRICTION_COLORS[index],
            linestyle="--",
            marker=marker,
            markevery=markevery,
            label=rf"$\mu={mu:g}$",
            zorder=3 + index,
        )
        axes[1].plot(
            tension,
            bend,
            color=FRICTION_COLORS[index],
            linestyle="--",
            marker=marker,
            markevery=markevery,
            label=rf"$\mu={mu:g}$",
            zorder=3 + index,
        )
        for row in selected:
            for joint in range(1, 19):
                long_rows.append(
                    {
                        "guide_friction_mu": mu,
                        "time_s": row["time_s"],
                        "joint_index": joint,
                        "joint_angle_deg": row[f"joint_{joint:02d}_angle_deg"],
                        "active_cable_takeup_mm": row["active_cable_takeup_mm"],
                        "active_tension_N": row["active_tension_N"],
                        "total_bend_deg": row["bend_deg"],
                    }
                )
    axes[0].set_xlabel("Active-cable take-up (mm)")
    axes[0].set_ylabel("Total bending angle (deg)")
    axes[1].set_xlabel("Active-cable tension (N)")
    axes[1].set_ylabel("Total bending angle (deg)")
    for label, axis in zip(("(a)", "(b)"), axes):
        style_axis(axis)
        axis.text(-0.17, 1.04, label, transform=axis.transAxes, fontweight="bold")
        axis.legend(frameon=False)
    export_figure(figure, output / "figures/result_2_3_friction_summary")
    write_csv(output / "data/result_2_3_joint_response_long.csv", long_rows)

    for x_key, x_label, filename in (
        ("active_cable_takeup_mm", "Active-cable take-up (mm)", "result_2_3_extension_vs_each_joint"),
        ("active_tension_N", "Active-cable tension (N)", "result_2_3_tension_vs_each_joint"),
    ):
        detailed, detailed_axes = plt.subplots(3, 2, figsize=(7.16, 7.4), constrained_layout=True)
        for axis, mu, simulation in zip(detailed_axes.flat, FRICTIONS, simulations):
            selected = [row for row in simulation.rows if row["phase"] == "bend_right"]
            x = np.asarray([float(row[x_key]) for row in selected])
            for joint in range(1, 19):
                y = np.asarray([float(row[f"joint_{joint:02d}_angle_deg"]) for row in selected])
                axis.plot(
                    x,
                    y,
                    color=plt.cm.viridis((joint - 1) / 17),
                    linestyle="--",
                    marker="o" if joint in (1, 6, 12, 18) else None,
                    markevery=max(1, len(x) // 9),
                    linewidth=1.0,
                    label=f"J{joint}" if joint in (1, 6, 12, 18) else None,
                )
            axis.set_title(rf"$\mu={mu:g}$")
            axis.set_xlabel(x_label)
            axis.set_ylabel("Joint angle (deg)")
            style_axis(axis)
        detailed_axes.flat[0].legend(ncol=2, frameon=False)
        detailed_axes.flat[-1].axis("off")
        export_figure(detailed, output / f"figures/{filename}")

    plot_result_2_3_bending_states(simulations, output)


def spirobot_planar_geometry(model_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return segment lengths and approximate start/end half-widths from MJCF."""
    root = ET.parse(model_path).getroot()
    lengths: list[float] = []
    start_widths: list[float] = []
    end_widths: list[float] = []
    for joint in range(1, 19):
        if joint < 18:
            next_body = root.find(f".//body[@name='solidworks_unit_{joint + 1:02d}_body']")
            if next_body is None:
                raise ValueError(f"missing body for segment {joint + 1}")
            length = float(next_body.attrib["pos"].split()[1])
        else:
            tip = root.find(".//site[@name='tip_marker']")
            if tip is None:
                raise ValueError("missing tip_marker")
            length = float(tip.attrib["pos"].split()[1])
        entry = root.find(f".//site[@name='positive_{joint:02d}_entry']")
        exit_site = root.find(f".//site[@name='positive_{joint:02d}_exit']")
        if entry is None or exit_site is None:
            raise ValueError(f"missing routing sites for segment {joint}")
        lengths.append(length)
        start_widths.append(abs(float(entry.attrib["pos"].split()[0])))
        end_widths.append(abs(float(exit_site.attrib["pos"].split()[0])))
    return np.asarray(lengths), np.asarray(start_widths), np.asarray(end_widths)


def planar_chain_shape(
    angles_rad: np.ndarray,
    lengths: np.ndarray,
    start_widths: np.ndarray,
    end_widths: np.ndarray,
) -> tuple[np.ndarray, list[np.ndarray]]:
    """Construct the bending-plane centerline and tapered segment outlines."""
    points = [np.zeros(2)]
    polygons: list[np.ndarray] = []
    heading = 0.0
    for angle, length, start_width, end_width in zip(
        angles_rad, lengths, start_widths, end_widths
    ):
        heading += float(angle)
        direction = np.asarray((-math.sin(heading), math.cos(heading)))
        normal = np.asarray((direction[1], -direction[0]))
        start = points[-1]
        end = start + float(length) * direction
        polygons.append(
            np.asarray(
                (
                    start + float(start_width) * normal,
                    end + float(end_width) * normal,
                    end - float(end_width) * normal,
                    start - float(start_width) * normal,
                )
            )
        )
        points.append(end)
    return 1000.0 * np.asarray(points), [1000.0 * polygon for polygon in polygons]


def plot_result_2_3_bending_states(simulations: list[Simulation], output: Path) -> None:
    """Overlay packed centerlines using one physical scale for both axes."""
    model_path = output / "models/result_2_3_mu_0p00.xml"
    lengths, start_widths, end_widths = spirobot_planar_geometry(model_path)
    states: list[tuple[float, Simulation, np.ndarray, list[np.ndarray]]] = []
    coordinate_rows: list[dict[str, object]] = []
    for mu, simulation in zip(FRICTIONS, simulations):
        hold = [row for row in simulation.rows if row["phase"] == "hold_right"]
        if not hold:
            hold = simulation.rows[-20:]
        angles = np.radians(
            [
                np.mean([float(row[f"joint_{joint:02d}_angle_deg"]) for row in hold])
                for joint in range(1, 19)
            ]
        )
        points, polygons = planar_chain_shape(angles, lengths, start_widths, end_widths)
        states.append((mu, simulation, points, polygons))
        for point_index, point in enumerate(points):
            coordinate_rows.append(
                {
                    "guide_friction_mu": mu,
                    "point_index": point_index,
                    "x_mm": float(point[0]),
                    "y_mm": float(point[1]),
                    "hold_total_bend_deg": float(simulation.summary["hold_bend_deg"]),
                    "peak_tension_N": float(simulation.summary["maximum_active_tension_N"]),
                    "peak_takeup_mm": float(
                        simulation.summary["maximum_active_cable_takeup_mm"]
                    ),
                }
            )

    all_points = np.concatenate([points for _, _, points, _ in states], axis=0)
    lower = np.min(all_points, axis=0)
    upper = np.max(all_points, axis=0)
    center = 0.5 * (lower + upper)
    common_span = 1.12 * max(float(upper[0] - lower[0]), float(upper[1] - lower[1]), 1.0)
    line_styles = ("-", "--", "-.", ":", (0, (5, 2, 1, 2)))
    markers = ("o", "s", "^", "D", "v")
    figure, overlay = plt.subplots(figsize=(3.5, 3.5), constrained_layout=True)
    # Draw and list high friction first.  Dark high-mu curves stay above the
    # lighter low-mu curves where centerlines overlap.
    for index in reversed(range(len(states))):
        mu, simulation, points, _ = states[index]
        overlay.plot(
            points[:, 0],
            points[:, 1],
            color=FRICTION_COLORS[index],
            linestyle=line_styles[index],
            marker=markers[index],
            markevery=3,
            linewidth=1.3,
            markersize=3.2,
            label=rf"$\mu={mu:g}$",
            zorder=3 + index,
        )
    overlay.scatter(0.0, 0.0, marker="s", color="#252525", s=18, zorder=5)
    overlay.set_xlim(center[0] - 0.5 * common_span, center[0] + 0.5 * common_span)
    overlay.set_ylim(center[1] - 0.5 * common_span, center[1] + 0.5 * common_span)
    overlay.set_aspect("equal", adjustable="box")
    overlay.set_xlabel("Bending-plane x (mm)")
    overlay.set_ylabel("Bending-plane y (mm)")
    overlay.legend(
        frameon=False,
        loc="lower center",
        bbox_to_anchor=(0.5, 1.01),
        ncol=3,
        columnspacing=1.0,
        handlelength=2.2,
    )
    overlay.grid(True, color="#D9DDE0", linewidth=0.45)
    overlay.spines["top"].set_visible(False)
    overlay.spines["right"].set_visible(False)
    overlay.tick_params(direction="out", length=2.5, width=0.6)
    export_figure(figure, output / "figures/result_2_3_friction_bending_states")
    write_csv(output / "data/result_2_3_hold_shape_coordinates.csv", coordinate_rows)


def render_result_2_3_final_states(
    simulations: list[Simulation],
    output: Path,
    width: int = 1280,
    height: int = 720,
) -> None:
    """Render all final states from one fixed bending-plane camera."""
    prepared: list[
        tuple[float, Simulation, mujoco.MjModel, mujoco.MjData, np.ndarray, np.ndarray]
    ] = []
    global_lower = np.full(3, np.inf)
    global_upper = np.full(3, -np.inf)
    for mu, simulation in zip(FRICTIONS, simulations):
        slug = f"{mu:.2f}".replace(".", "p")
        model = mujoco.MjModel.from_xml_path(
            str((output / f"models/result_2_3_mu_{slug}.xml").resolve())
        )
        data = mujoco.MjData(model)
        if simulation.final_qpos is None:
            raise ValueError(f"missing final qpos for {simulation.name}")
        data.qpos[:] = simulation.final_qpos
        if simulation.final_ctrl is not None:
            data.ctrl[:] = simulation.final_ctrl
        mujoco.mj_forward(model, data)
        site_positions = np.asarray(data.site_xpos, dtype=float)
        margin = np.asarray((0.008, 0.025, 0.002))
        state_lower = np.min(site_positions, axis=0) - margin
        state_upper = np.max(site_positions, axis=0) + margin
        global_lower = np.minimum(global_lower, state_lower)
        global_upper = np.maximum(global_upper, state_upper)
        prepared.append((mu, simulation, model, data, state_lower, state_upper))

    fovy = 45.0
    tangent = math.tan(math.radians(fovy) / 2.0)
    fill = 0.86

    def camera_pose(lower: np.ndarray, upper: np.ndarray) -> tuple[np.ndarray, float]:
        center = 0.5 * (lower + upper)
        half_x = max(0.5 * float(upper[0] - lower[0]), 0.035)
        half_y = max(0.5 * float(upper[1] - lower[1]), 0.035)
        distance = max(
            half_y / (fill * tangent),
            half_x / (fill * tangent * width / height),
            0.09,
        )
        return center, distance

    common_center, common_distance = camera_pose(global_lower, global_upper)

    screenshot_dir = output / "screenshots"
    screenshot_dir.mkdir(parents=True, exist_ok=True)
    rendered: list[tuple[float, Simulation, Image.Image]] = []
    for mu, simulation, model, data, state_lower, state_upper in prepared:
        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "bending_plane")
        if camera_id < 0:
            raise ValueError("bending_plane camera not found")
        model.cam_fovy[camera_id] = fovy
        state_center, state_distance = camera_pose(state_lower, state_upper)
        model.cam_pos[camera_id] = state_center
        model.cam_pos[camera_id, 2] += state_distance
        renderer = mujoco.Renderer(model, height=height, width=width, max_geom=20000)
        try:
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera="bending_plane")
            image = Image.fromarray(renderer.render()).convert("RGB")
            model.cam_pos[camera_id] = common_center
            model.cam_pos[camera_id, 2] += common_distance
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera="bending_plane")
            common_image = Image.fromarray(renderer.render()).convert("RGB")
        finally:
            renderer.close()
        slug = f"{mu:.2f}".replace(".", "p")
        image.save(screenshot_dir / f"result_2_3_mu_{slug}_final.png", compress_level=2)
        common_image.save(
            screenshot_dir / f"result_2_3_mu_{slug}_final_common_scale.png",
            compress_level=2,
        )
        rendered.append((mu, simulation, image))

    panel_width = 640
    panel_height = 360
    header = 42
    montage = Image.new("RGB", (3 * panel_width, 2 * (panel_height + header)), "white")
    draw = ImageDraw.Draw(montage)
    font = VideoFrames.font(24, True)
    for index, (mu, simulation, image) in enumerate(rendered):
        column = index % 3
        row = index // 3
        x = column * panel_width
        y = row * (panel_height + header)
        montage.paste(image.resize((panel_width, panel_height), Image.Resampling.LANCZOS), (x, y + header))
        draw.text(
            (x + 14, y + 7),
            f"mu={mu:g} | final bend={float(simulation.summary['final_bend_deg']):.1f} deg",
            fill="#202428",
            font=font,
        )
    montage.save(screenshot_dir / "result_2_3_final_state_montage.png", compress_level=2)


def validate_matched_spirobot_models(plugin_path: Path, native_path: Path) -> None:
    plugin_model = mujoco.MjModel.from_xml_path(str(plugin_path.resolve()))
    native_model = mujoco.MjModel.from_xml_path(str(native_path.resolve()))
    scalar_fields = ("nq", "nv", "nbody", "njnt", "ngeom", "nsite")
    for field in scalar_fields:
        if getattr(plugin_model, field) != getattr(native_model, field):
            raise ValueError(f"Result 2.2 model mismatch in {field}")
    array_fields = (
        "jnt_type",
        "jnt_range",
        "jnt_stiffness",
        "dof_damping",
        "body_mass",
        "body_inertia",
        "geom_type",
        "geom_size",
        "site_pos",
    )
    for field in array_fields:
        if not np.allclose(getattr(plugin_model, field), getattr(native_model, field)):
            raise ValueError(f"Result 2.2 model mismatch in {field}")


def write_joint_parameters(model_path: Path) -> list[dict[str, object]]:
    root = ET.parse(model_path).getroot()
    rows = []
    for index, joint in enumerate(root.findall(".//joint"), start=1):
        rows.append(
            {
                "joint_index": index,
                "joint_name": joint.attrib.get("name", f"joint_{index:02d}"),
                "type": joint.attrib.get("type", "hinge"),
                "range_deg": joint.attrib.get("range", ""),
                "stiffness_Nm_per_rad": joint.attrib.get("stiffness", "0"),
                "damping_Nms_per_rad": joint.attrib.get("damping", "0"),
            }
        )
    write_csv(HERE / "result_2_2/data/result_2_2_joint_parameters.csv", rows)
    return rows


def plot_integrated_result_2(
    faive_rows: list[dict[str, object]],
    plugin: Simulation,
    native: Simulation,
    friction: list[Simulation],
) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(7.16, 5.25), constrained_layout=True)
    for label, color, line, marker, legend, zorder in (
        ("virtual_hinge", "#666666", "-", None, "Ideal virtual hinge", 2),
        ("surface_cable", "#0072B2", "--", "o", "MuJoCable", 4),
    ):
        selected = [row for row in faive_rows if row["model"] == label]
        axes[0, 0].plot(
            [float(row["time_s"]) for row in selected],
            [float(row["angle_deg"]) for row in selected],
            color=color,
            linestyle=line,
            marker=marker,
            markevery=max(1, len(selected) // 10),
            label=legend,
            zorder=zorder,
        )
    axes[0, 0].axhline(50.0, color="#999999", linestyle=":", linewidth=0.9)
    axes[0, 0].set_xlabel("Time (s)")
    axes[0, 0].set_ylabel("PIP rotation (deg)")
    axes[0, 0].legend(frameon=False)

    reference_time = np.asarray([float(row["time_s"]) for row in plugin.rows])
    reference_angle = np.asarray([float(row["paper_target_deg"]) for row in plugin.rows])
    axes[0, 1].plot(
        reference_time,
        reference_angle,
        color="#666666",
        linestyle="-",
        label="Published packed-state target",
        zorder=2,
    )
    for simulation, color, line, marker, label in (
        (native, "#D55E00", "-.", "s", "Native tendon + muscle"),
        (plugin, "#0072B2", "--", "o", r"MuJoCable, $\mu=0.15$"),
    ):
        axes[0, 1].plot(
            [float(row["time_s"]) for row in simulation.rows],
            [float(row["bend_deg"]) for row in simulation.rows],
            color=color,
            linestyle=line,
            marker=marker,
            markevery=max(1, len(simulation.rows) // 12),
            label=label,
            zorder=4,
        )
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].set_ylabel("Total bending angle (deg)")
    axes[0, 1].legend(frameon=False)

    for index, (mu, simulation) in enumerate(zip(FRICTIONS, friction)):
        selected = [row for row in simulation.rows if row["phase"] == "bend_right"]
        marker = ("o", "s", "^", "D", "v")[index]
        markevery = max(1, len(selected) // 9)
        common = {
            "color": FRICTION_COLORS[index],
            "linestyle": "--",
            "marker": marker,
            "markevery": markevery,
            "label": rf"$\mu={mu:g}$",
            "zorder": 3 + index,
        }
        bend = [float(row["bend_deg"]) for row in selected]
        axes[1, 0].plot(
            [float(row["active_cable_takeup_mm"]) for row in selected], bend, **common
        )
        axes[1, 1].plot(
            [float(row["active_tension_N"]) for row in selected], bend, **common
        )
    axes[1, 0].set_xlabel("Active-cable take-up (mm)")
    axes[1, 0].set_ylabel("Total bending angle (deg)")
    axes[1, 1].set_xlabel("Active-cable tension (N)")
    axes[1, 1].set_ylabel("Total bending angle (deg)")
    axes[1, 0].legend(frameon=False, ncol=2)
    axes[1, 1].legend(frameon=False, ncol=2)
    for label, axis in zip(("(a)", "(b)", "(c)", "(d)"), axes.flat):
        style_axis(axis)
        axis.text(-0.17, 1.04, label, transform=axis.transAxes, fontweight="bold")
    export_figure(figure, HERE / "figures/figure4_result2_simulation_comparison")


def write_method_table(
    faive_summary: dict[str, object],
    plugin: Simulation,
    native: Simulation,
    friction: list[Simulation],
    joint_rows: list[dict[str, object]],
) -> None:
    rows = []
    for simulation in (plugin, native, *friction):
        row = {"case": simulation.name}
        row.update(simulation.summary)
        rows.append(row)
    write_csv(HERE / "result_2_summary.csv", rows)

    lines = [
        "# Result 2 模型、理论基线与统计说明",
        "",
        "## 1. Result 2.1：50 deg Faive PIP",
        "",
        "两根主动索均显式列出 `proximal_route_surface distal_route_surface`，因此端点之间的运行时路线由插件在两个真实闭合 mesh 外壳上求解，而不是沿 seed site 直线连接。该 Result 为隔离主动驱动路线，移除了在大角度下丢失同伦走廊的两条被动韧带；这不是完整 Faive 韧带模型。",
        "",
        "## 2. Result 2.2：官方 native 思路与 MuJoCable",
        "",
        "两组模型共享相同的 18 个 hinge、刚度、阻尼、质量、惯量、关节限位、碰撞几何和穿绳 site。原论文 Note S3 将二维机器人表示为串联 1-DOF revolute joints，并使用 MuJoCo tendon-muscle。论文还规定每个关节的极限等于离散步长 30 deg；所有关节同时到达极限时形成紧密对数螺旋。图中的灰色实线是这一公开理论/目标，不是从论文图片反向拟合的数据。",
        "",
        "| 项目 | MuJoCo native baseline | MuJoCable |",
        "|---|---|---|",
        "| 路线 | spatial tendon 依次经过固定 site | guide/孔口路线并由插件计算长度和受力 |",
        "| 驱动 | muscle 激活量，官方生成器默认 `force=5 N` | 目标收缩量加双索预紧闭环 |",
        "| 单向受拉 | 由 muscle/tendon 执行链决定 | 本构显式投影为 `T >= 0` |",
        "| 摩擦 | 不表达沿多 guide 的 Capstan 张力传播 | `guide_friction_mu` 逐段传播张力 |",
        "| 松弛状态 | 无插件 route 状态 | 输出 length、contraction、tension、taut、route status |",
        "| 公平控制变量 | 相同关节、质量、碰撞和穿绳点 | 相同关节、质量、碰撞和穿绳点 |",
        "",
        "两者输入物理量不同：native 输入是 muscle activation，MuJoCable 输入是收缩量。因此本实验比较模型能力和在同一目标轨迹下的响应，不把输入数值直接解释为相同执行器功。",
        "",
        "## 3. Result 2.3：摩擦敏感性",
        "",
        "固定模型、关节柔性、预紧、目标角和最大收缩，只改变 `guide_friction_mu = 0, 0.15, 0.30, 0.45, 0.60`。输出包含总弯曲、18 个关节角、主动索收缩和张力。该实验是参数敏感性分析；在实物辨识前，不能把某个 mu 称为已标定真实摩擦系数。",
        "",
        "## 4. 原论文参考",
        "",
        "Wang, Freris, and Wei, *SpiRobs: Logarithmic spiral-shaped robots for versatile grasping across scales*, Device, 2025, DOI: 10.1016/j.device.2024.100646. 采用其 Note S3 的串联关节、tendon-muscle 与 30 deg 离散关节极限作为公开基线。",
        "",
    ]
    (HERE / "RESULT2_MODEL_AND_METHOD_COMPARISON_ZH.md").write_text(
        "\n".join(lines), encoding="utf-8"
    )
    surface = faive_summary["surface_cable:actuation"]
    theory = [
        "# Result 2 理论、模型匹配与统计结果",
        "",
        "## 1. 统一动力学边界",
        "",
        "对于关节坐标 q，绳路长度为 L(q)，几何力臂和广义力分别为",
        "",
        "```text",
        "r(q) = -dL/dq,                  tau_c = T r(q) = -T dL/dq.",
        "T = clip(k [L-(L0-c)-s]_+ + d Ldot, 0, Tmax).",
        "```",
        "",
        "MuJoCo 负责刚体惯性、关节弹簧阻尼和接触；MuJoCable 负责运行时绳路、单向张力、分段摩擦和由路径 Jacobian 产生的节点力。",
        "",
        "## 2. Result 2.1：Faive PIP 50 deg",
        "",
        "理想基线给两个虚拟铰链各 25 deg 指令，总目标为 50 deg。MuJoCable 使用 10 mm 收缩；flexor 与 extensor 均按 proximal mesh -> distal mesh 的顺序包络两个真实外壳。为隔离主动路线，大角度下会失去有效同伦走廊的两条被动韧带未包含在本项中。",
        "",
        f"- 理想基线最终角：{faive_summary['virtual_hinge:actuation']['final_angle_deg']:.3f} deg；",
        f"- MuJoCable 最终角：{surface['final_angle_deg']:.3f} deg；",
        f"- 峰值张力：{surface['maximum_tension_N']:.3f} N；",
        f"- mesh 接触保持率：{100.0 * surface['contact_fraction']:.2f}%；",
        f"- 最大 route status：{surface['maximum_route_status']}（0 为 valid）。",
        "",
        "这证明当前主动双表面路线可以覆盖 50 deg，但不能被表述为完整 Faive 韧带模型已经在 50 deg 全部有效。",
        "",
        "## 3. Result 2.2：原生 tendon 与 MuJoCable",
        "",
        "原论文 Note S3 使用串联 1-DOF hinge 和 MuJoCo spatial tendon + muscle。这里保留 Demo 49 的 18 个关节、网格、质量、惯量、限位、穿绳点、刚度和阻尼，仅替换绳索执行链。官方生成器默认 muscle force 为 5 N，本基线沿用该值。",
        "",
        "| 指标 | Native tendon + muscle | MuJoCable (mu=0.15) |",
        "|---|---:|---:|",
        f"| 保持段总弯曲 (deg) | {native.summary['hold_bend_deg']:.3f} | {plugin.summary['hold_bend_deg']:.3f} |",
        f"| 跟踪 RMSE (deg) | {native.summary['tracking_rmse_deg']:.3f} | {plugin.summary['tracking_rmse_deg']:.3f} |",
        f"| 峰值主动索力 (N) | {native.summary['maximum_active_tension_N']:.3f} | {plugin.summary['maximum_active_tension_N']:.3f} |",
        f"| 峰值收绳/路线缩短 (mm) | {native.summary['maximum_active_cable_takeup_mm']:.3f} | {plugin.summary['maximum_active_cable_takeup_mm']:.3f} |",
        "",
        "论文给出的离散关节极限为每节 30 deg，18 节的形式上限为 540 deg；实验目标使用 530 deg，为接近紧密对数螺旋且避免长期顶住全部关节限位。灰色实线表示这一公开目标/极限，不是从论文图片数字化得到的实测曲线。",
        "",
        "## 4. Result 2.3：摩擦改变运动分配",
        "",
        "插件在每个 guide 上使用正则化 Capstan 关系",
        "",
        "```text",
        "T_(i+1) / T_i = exp(mu beta_i tanh(v_rel / v_s)),",
        "```",
        "",
        "其中 beta_i 为局部转角。摩擦使远端可用张力降低，因而同一目标需要更大源端张力和行程，并改变 18 个关节的弯曲分配。",
        "",
        "| mu | 保持段总弯曲 (deg) | 峰值张力 (N) | 峰值收绳 (mm) |",
        "|---:|---:|---:|---:|",
    ]
    for mu, simulation in zip(FRICTIONS, friction):
        theory.append(
            f"| {mu:g} | {simulation.summary['hold_bend_deg']:.3f} | "
            f"{simulation.summary['maximum_active_tension_N']:.3f} | "
            f"{simulation.summary['maximum_active_cable_takeup_mm']:.3f} |"
        )
    theory.extend(
        [
            "",
            "`mu=0.60` 是敏感性压力工况，不代表已标定材料。实物辨识前只能得出参数敏感性结论。",
            "",
            "弯曲状态叠加图见 `result_2_3/figures/result_2_3_friction_bending_states`。五条曲线使用保持阶段的逐关节平均角重建平面中心线，横纵坐标采用相同物理尺度。对应关节点坐标保存在 `result_2_3/data/result_2_3_hold_shape_coordinates.csv`；五个工况的 MuJoCo 最终状态截图位于 `result_2_3/screenshots/`。",
            "",
            "## 5. 关节柔性匹配",
            "",
            "完整 18 关节参数见 `result_2_2/data/result_2_2_joint_parameters.csv`。Native 与 MuJoCable 模型在加载后逐数组比较 joint range、stiffness、damping、body mass/inertia、geom 和 site；任一不一致都会使实验脚本终止。代表值如下：",
            "",
            "| Joint | Stiffness (N m/rad) | Damping (N m s/rad) |",
            "|---:|---:|---:|",
        ]
    )
    for index in (0, 5, 11, 17):
        row = joint_rows[index]
        theory.append(
            f"| {row['joint_index']} | {float(row['stiffness_Nm_per_rad']):.8g} | "
            f"{float(row['damping_Nms_per_rad']):.8g} |"
        )
    theory.extend(
        [
            "",
            "## 6. 结论边界",
            "",
            "Result 2.1 证明表面包络主动索能够驱动大角度滚动接触运动；Result 2.2 比较同一柔性结构下两种绳索表示；Result 2.3 证明摩擦参数会改变行程、张力和关节角分配。它们是 simulation-to-simulation 证据，不构成 MuJoCable 更接近实物的充分证明；准确性仍需 Result 3 的同步实物数据。",
            "",
        ]
    )
    (HERE / "RESULT2_THEORY_AND_RESULTS_ZH.md").write_text(
        "\n".join(theory), encoding="utf-8"
    )


def record_faive_videos(record_module, plugin: Path, width: int, height: int, fps: int) -> list[Path]:
    output = HERE / "result_2_1/videos"
    specifications = (
        record_module.DemoVideoSpec(
            label="result_2_1_virtual_hinge",
            title="Faive PIP virtual-hinge reference",
            model=str((HERE / "result_2_1/models/result_2_1a_ideal_virtual_hinge.xml").resolve()),
            profile="ramp_hold",
            duration_s=2.5,
            primary_actuator="pip_joint_command",
            primary_command=math.radians(25.0),
            tracked_body="faive_index_mp",
            camera="bending_plane",
            method_tag="two prescribed virtual hinges",
            fixed_camera_fovy=26.0,
            overlay_position="top",
        ),
        record_module.DemoVideoSpec(
            label="result_2_1_mujocable_50deg",
            title="Faive PIP two-surface cable",
            model=str((HERE / "result_2_1/models/result_2_1b_mujocable_surface.xml").resolve()),
            profile="ramp_hold",
            duration_s=2.5,
            primary_actuator="pip_flexor_command",
            primary_command=0.010,
            tracked_body="faive_index_mp",
            camera="bending_plane",
            description="Two active cables wrap both real mesh shells.",
            method_tag="two antagonistic surface cables",
            display_cables=("pip_flexor", "pip_extensor"),
            fixed_camera_fovy=26.0,
            overlay_position="top",
        ),
    )
    paths = []
    for specification in specifications:
        record_module.record_one(
            specification, plugin, output, width, height, fps, paired_output=True
        )
        paths.extend(
            (
                output / f"{specification.label}_raw.mp4",
                output / f"{specification.label}_annotated.mp4",
            )
        )
    return paths


def video_inventory(paths: list[Path]) -> None:
    rows = []
    for path in paths:
        probe = subprocess.run(
            [
                "ffprobe",
                "-v",
                "error",
                "-select_streams",
                "v:0",
                "-show_entries",
                "stream=width,height,avg_frame_rate,nb_frames",
                "-show_entries",
                "format=duration,size",
                "-of",
                "json",
                str(path),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        metadata = json.loads(probe.stdout)
        stream = metadata["streams"][0]
        rows.append(
            {
                "path": path.relative_to(HERE).as_posix(),
                "variant": "annotated" if "annotated" in path.stem else "raw",
                "duration_s": metadata["format"]["duration"],
                "fps": stream["avg_frame_rate"],
                "frames": stream["nb_frames"],
                "width_px": stream["width"],
                "height_px": stream["height"],
                "size_bytes": metadata["format"]["size"],
            }
        )
    write_csv(HERE / "RESULT2_VIDEO_INVENTORY.csv", rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", type=Path, required=True)
    parser.add_argument("--research-root", type=Path, default=HERE)
    parser.add_argument("--release-root", type=Path, default=REPO)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--skip-videos", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.plugin.is_file():
        raise FileNotFoundError(args.plugin)
    research_scripts = args.research_root.resolve() / "scripts"
    release_scripts = args.release_root.resolve() / "scripts"
    sys.path.insert(0, str(research_scripts))
    import run_tpu95hf_bidirectional_bending as cycle

    sys.path.insert(0, str(REPO / "scripts"))
    import compare_faive_index_pip as compare_module

    sys.path.insert(0, str(release_scripts))
    import record_cpp_plugin_demos as record_module

    configure_plotting()
    mujoco.set_mju_user_warning(lambda message: print(f"MuJoCo warning: {message}"))
    mujoco.mj_loadPluginLibrary(str(args.plugin.resolve()))

    faive_rows, faive_summary = run_faive(compare_module, HERE / "result_2_1")
    plugin_model_path = HERE / "result_2_2/models/result_2_2a_mujocable_mu_0p15.xml"
    native_model_path = HERE / "result_2_2/models/result_2_2b_native_tendon.xml"
    validate_matched_spirobot_models(plugin_model_path, native_model_path)
    joint_rows = write_joint_parameters(plugin_model_path)
    plugin_video = None
    native_video = None
    video_paths: list[Path] = []
    if not args.skip_videos:
        video_paths.extend(
            record_faive_videos(record_module, args.plugin, args.width, args.height, args.fps)
        )
        plugin_video = VideoFrames(
            HERE / "result_2_2/videos", args.width, args.height, args.fps, "Result 2 2 MuJoCable"
        )
        native_video = VideoFrames(
            HERE / "result_2_2/videos", args.width, args.height, args.fps, "Result 2 2 Native tendon"
        )

    plugin = simulate_plugin_spirobot(
        cycle,
        plugin_model_path,
        "mujocable_mu_0.15",
        args.duration,
        SAMPLE_PERIOD,
        plugin_video,
    )
    native = simulate_native_spirobot(
        native_model_path,
        "native_tendon_muscle",
        args.duration,
        SAMPLE_PERIOD,
        native_video,
    )
    write_csv(HERE / "result_2_2/data/result_2_2_comparison.csv", plugin.rows + native.rows)
    (HERE / "result_2_2/data/result_2_2_summary.json").write_text(
        json.dumps({plugin.name: plugin.summary, native.name: native.summary}, indent=2) + "\n",
        encoding="utf-8",
    )
    plot_result_2_2(plugin, native, HERE / "result_2_2")
    if plugin_video is not None and native_video is not None:
        video_paths.extend(plugin_video.close())
        video_paths.extend(native_video.close())

    friction_results = []
    for mu in FRICTIONS:
        slug = f"{mu:.2f}".replace(".", "p")
        video = None
        if not args.skip_videos and mu in (0.0, 0.15, 0.60):
            video = VideoFrames(
                HERE / "result_2_3/videos",
                args.width,
                args.height,
                args.fps,
                f"Result 2 3 mu {slug}",
            )
        simulation = simulate_plugin_spirobot(
            cycle,
            HERE / f"result_2_3/models/result_2_3_mu_{slug}.xml",
            f"mujocable_mu_{mu:g}",
            args.duration,
            SAMPLE_PERIOD,
            video,
        )
        friction_results.append(simulation)
        write_csv(HERE / f"result_2_3/data/result_2_3_mu_{slug}.csv", simulation.rows)
        if video is not None:
            video_paths.extend(video.close())
    (HERE / "result_2_3/data/result_2_3_summary.json").write_text(
        json.dumps({result.name: result.summary for result in friction_results}, indent=2) + "\n",
        encoding="utf-8",
    )
    plot_result_2_3(friction_results, HERE / "result_2_3")
    if not args.skip_videos:
        render_result_2_3_final_states(friction_results, HERE / "result_2_3")
    plot_integrated_result_2(faive_rows, plugin, native, friction_results)
    write_method_table(faive_summary, plugin, native, friction_results, joint_rows)
    if video_paths:
        video_inventory(video_paths)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
