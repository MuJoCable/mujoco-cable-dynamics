#!/usr/bin/env python3
"""Open the MuJoCable OpenSpiRobs model with an automatic two-cable cycle."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def _sensor(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    start = model.sensor_adr[sensor_id]
    return np.asarray(
        data.sensordata[start : start + model.sensor_dim[sensor_id]], dtype=float
    )


def _label(
    scene: mujoco.MjvScene,
    position: tuple[float, float, float],
    text: str,
    color: tuple[float, float, float, float],
) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_LABEL,
        np.zeros(3),
        np.asarray(position),
        np.eye(3).reshape(9),
        np.asarray(color, dtype=np.float32),
    )
    geom.label = text
    scene.ngeom += 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", type=Path, required=True)
    parser.add_argument(
        "--model",
        type=Path,
        default=ROOT / "cable_plugin_demos/33_cpp_plugin_log_spiral_dual_reserve.xml",
    )
    parser.add_argument(
        "--mode",
        choices=("alternate", "positive", "negative", "differential", "manual"),
        default="alternate"
    )
    parser.add_argument("--contraction", type=float, default=0.025)
    parser.add_argument("--period", type=float, default=7.0)
    parser.add_argument("--ramp-time", type=float, default=1.5)
    parser.add_argument(
        "--reserve",
        type=float,
        default=0.0,
        help="common initial cable reserve used by differential mode",
    )
    parser.add_argument("--duration", type=float, default=120.0)
    args = parser.parse_args()
    minimum_period = (
        2.0 * args.ramp_time
        if args.mode == "differential"
        else args.ramp_time
    )
    if args.ramp_time <= 0.0 or args.period <= minimum_period:
        parser.error("period is too short for the requested ramp sequence")
    if args.reserve < 0.0 or args.reserve > args.contraction:
        parser.error("reserve must lie between zero and the final contraction")

    mujoco.mj_loadPluginLibrary(str(args.plugin.resolve()))
    if args.model.is_absolute():
        model_path = args.model
    else:
        cwd_candidate = args.model.resolve()
        model_path = cwd_candidate if cwd_candidate.exists() else ROOT / args.model
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "overview")
    started = time.monotonic()
    phase_index = -1
    target = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "grasp_target"
    )

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = camera
        while viewer.is_running() and time.monotonic() - started < args.duration:
            if args.mode != "manual":
                elapsed = time.monotonic() - started
                new_phase = int(elapsed // args.period)
                if new_phase != phase_index:
                    phase_index = new_phase
                    mujoco.mj_resetData(model, data)
                if args.mode == "alternate":
                    active = phase_index % 2
                else:
                    active = 0 if args.mode == "positive" else 1
                local_time = elapsed % args.period
                data.ctrl[:] = 0
                if args.mode == "differential":
                    takeup = min(1.0, local_time / args.ramp_time)
                    data.ctrl[:] = args.reserve * takeup
                    if local_time >= args.ramp_time:
                        bend = min(
                            1.0,
                            (local_time - args.ramp_time) / args.ramp_time,
                        )
                        data.ctrl[active] = args.reserve + (
                            args.contraction - args.reserve
                        ) * bend
                        data.ctrl[1 - active] = args.reserve * (1.0 - bend)
                else:
                    ramp = min(1.0, local_time / args.ramp_time)
                    data.ctrl[active] = args.contraction * ramp

            step_started = time.monotonic()
            mujoco.mj_step(model, data)
            positive = _sensor(model, data, "positive_state")
            negative = _sensor(model, data, "negative_state")
            tip = _sensor(model, data, "tip_position")
            scene = viewer.user_scn
            scene.ngeom = 0
            _label(
                scene,
                (-0.075, -0.02, 0.185),
                (
                    f"positive | L={1000*positive[0]:.1f} mm | "
                    f"T={positive[5]:.3f} N | u={1000*data.ctrl[0]:.1f} mm"
                ),
                (0.05, 0.90, 1.0, 1.0),
            )
            _label(
                scene,
                (-0.075, -0.02, 0.174),
                (
                    f"negative | L={1000*negative[0]:.1f} mm | "
                    f"T={negative[5]:.3f} N | u={1000*data.ctrl[1]:.1f} mm"
                ),
                (1.0, 0.30, 0.18, 1.0),
            )
            _label(
                scene,
                (-0.075, -0.02, 0.163),
                f"tip = ({1000*tip[0]:.1f}, {1000*tip[1]:.1f}, {1000*tip[2]:.1f}) mm",
                (0.96, 0.96, 0.96, 1.0),
            )
            if target >= 0:
                target_contacts = sum(
                    target in (data.contact[index].geom1, data.contact[index].geom2)
                    for index in range(data.ncon)
                )
                _label(
                    scene,
                    (-0.075, -0.02, 0.152),
                    f"cylinder contacts = {target_contacts}",
                    (1.0, 0.76, 0.18, 1.0),
                )
            viewer.sync()
            delay = float(model.opt.timestep) - (time.monotonic() - step_started)
            if delay > 0:
                time.sleep(delay)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
