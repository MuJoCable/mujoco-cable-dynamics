#!/usr/bin/env python3
"""View the CPhO 2018 final problem 3 massive-rope benchmark."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

from analyze_cpho_2018_problem3 import (
    DEFAULT_MODEL,
    load_parameters,
    name2id,
    numeric,
    official_coefficients,
    official_sliding_acceleration,
    official_transition_time,
    segment_tensions,
)


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
        np.asarray(position, dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(9),
        np.asarray(rgba, dtype=np.float32),
    )
    scene.ngeom += 1


def _label(
    scene: mujoco.MjvScene,
    position: np.ndarray,
    text: str,
    rgba: np.ndarray,
) -> None:
    if scene.ngeom >= len(scene.geoms):
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_LABEL,
        np.zeros(3, dtype=np.float64),
        np.asarray(position, dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(9),
        np.asarray(rgba, dtype=np.float32),
    )
    geom.label = text
    scene.ngeom += 1


def path_position(
    coordinate: float, radius: float, height: float
) -> np.ndarray:
    total = 2.0 * height + math.pi * radius
    coordinate %= total
    if coordinate < height:
        return np.array([radius, -0.024, coordinate], dtype=float)
    coordinate -= height
    if coordinate < math.pi * radius:
        angle = coordinate / radius
        return np.array(
            [
                radius * math.cos(angle),
                -0.024,
                height + radius * math.sin(angle),
            ],
            dtype=float,
        )
    coordinate -= math.pi * radius
    return np.array([-radius, -0.024, height - coordinate], dtype=float)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument(
        "--case", choices=("sliding", "stick"), default="sliding"
    )
    parser.add_argument("--omega", type=float)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--reset-after", type=float)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    parameters = load_parameters(model)
    coefficients = official_coefficients(parameters)
    omega = args.omega
    if omega is None:
        omega = numeric(
            model,
            "sliding_case_omega" if args.case == "sliding" else "sticking_case_omega",
        )
    if omega <= 0:
        raise ValueError("omega must be positive")

    rope_joint = name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "rope_transport"
    )
    pulley_joint = name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "pulley_hinge")
    rope_dof = int(model.jnt_dofadr[rope_joint])
    pulley_dof = int(model.jnt_dofadr[pulley_joint])
    rim_speed = parameters.radius * omega
    predicted_maximum = min(rim_speed, coefficients.sliding_speed_limit)
    transition = official_transition_time(rim_speed, coefficients)
    reset_after = args.reset_after
    if reset_after is None:
        reset_after = 3.5 if transition is None else max(1.8, transition + 1.0)

    def reset() -> None:
        mujoco.mj_resetData(model, data)
        data.qvel[pulley_dof] = -omega
        mujoco.mj_forward(model, data)

    reset()
    camera_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, "overview"
    )
    started = time.monotonic()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        if camera_id >= 0:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = camera_id
        while viewer.is_running() and time.monotonic() - started < args.duration:
            if data.time >= reset_after:
                reset()

            speed = max(0.0, float(data.qvel[rope_dof]))
            sticking = rim_speed < coefficients.sliding_speed_limit and (
                speed >= rim_speed - 1e-10
            )
            if sticking:
                speed = rim_speed
                data.qvel[rope_dof] = speed
                acceleration = 0.0
                regime = "STICK"
            else:
                acceleration = official_sliding_acceleration(
                    speed, coefficients
                )
                regime = "SLIDING"

            left_tension, right_tension = segment_tensions(
                speed, acceleration, parameters
            )
            data.qfrc_applied[:] = 0.0
            data.qfrc_applied[rope_dof] = acceleration
            data.qvel[pulley_dof] = -omega
            step_start = time.monotonic()
            previous_speed = speed
            mujoco.mj_step(model, data)
            data.qvel[pulley_dof] = -omega
            if (
                transition is not None
                and previous_speed < rim_speed
                and data.qvel[rope_dof] >= rim_speed
            ):
                data.qvel[rope_dof] = rim_speed

            scene = viewer.user_scn
            scene.ngeom = 0
            path_length = 2.0 * parameters.height + math.pi * parameters.radius
            marker_spacing = 0.055
            marker_count = int(math.ceil(path_length / marker_spacing))
            distance = float(data.qpos[int(model.jnt_qposadr[rope_joint])])
            for marker in range(marker_count):
                coordinate = marker * marker_spacing + distance
                _sphere(
                    scene,
                    path_position(
                        coordinate, parameters.radius, parameters.height
                    ),
                    0.006,
                    np.array([1.0, 0.82, 0.15, 1.0]),
                )

            label_color = np.array([0.96, 0.96, 0.96, 1.0])
            _label(
                scene,
                np.array([-0.28, -0.02, 0.61]),
                (
                    f"CPhO 2018 P3 | {regime} | "
                    f"v={speed:.3f} m/s | Rw={rim_speed:.3f} m/s"
                ),
                label_color,
            )
            _label(
                scene,
                np.array([-0.28, -0.02, 0.57]),
                (
                    f"T1={left_tension:.3f} N | T2={right_tension:.3f} N | "
                    f"official vmax={predicted_maximum:.3f} m/s"
                ),
                label_color,
            )
            viewer.sync()
            delay = float(model.opt.timestep) - (time.monotonic() - step_start)
            if delay > 0:
                time.sleep(delay)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
