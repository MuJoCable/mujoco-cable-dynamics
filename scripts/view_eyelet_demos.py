#!/usr/bin/env python3
"""View the rigid-flex aperture and analytic eyelet-friction demos."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

from analyze_eyelet_friction import (
    FLEX_MODEL,
    FRICTION_MODEL,
    _guide_turn_angle,
    _plugin_tension,
    _sensor_scalar,
)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLUGIN = ROOT / "build/plugin/libcable_unilateral.dylib"


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
        np.zeros(3, dtype=np.float64),
        np.asarray(position, dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(9),
        np.asarray(color, dtype=np.float32),
    )
    geom.label = text
    scene.ngeom += 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", choices=("flex", "friction"), required=True)
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--reset-after", type=float, default=4.0)
    args = parser.parse_args()

    model_path = FLEX_MODEL
    if args.demo == "friction":
        if not args.plugin.exists():
            raise FileNotFoundError(f"compiled plugin not found: {args.plugin}")
        mujoco.mj_loadPluginLibrary(str(args.plugin.resolve()))
        model_path = FRICTION_MODEL
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    camera = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, "overview"
    )

    def reset() -> None:
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)

    reset()
    started = time.monotonic()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        if camera >= 0:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = camera
        while viewer.is_running() and time.monotonic() - started < args.duration:
            if data.time >= args.reset_after:
                reset()
            step_started = time.monotonic()
            mujoco.mj_step(model, data)
            scene = viewer.user_scn
            scene.ngeom = 0
            white = (0.96, 0.96, 0.96, 1.0)
            if args.demo == "flex":
                blocked = _sensor_scalar(model, data, "convex_hull_ball_z")
                passed = _sensor_scalar(model, data, "rigid_flex_ball_z")
                _label(
                    scene,
                    (-0.24, -0.02, 0.39),
                    f"mesh geom: convex hull | dz={1000*blocked:.1f} mm",
                    white,
                )
                _label(
                    scene,
                    (0.03, -0.02, 0.39),
                    f"rigid flex: true hole | dz={1000*passed:.1f} mm",
                    white,
                )
            else:
                smooth_tension = _plugin_tension(
                    model, data, "smooth_eyelet_state"
                )
                rough_tension = _plugin_tension(
                    model, data, "rough_eyelet_state"
                )
                turn = _guide_turn_angle(
                    model,
                    data,
                    "rough_anchor",
                    "rough_guide",
                    "rough_load_end",
                )
                downstream = rough_tension * math.exp(-0.45 * turn)
                smooth_q = _sensor_scalar(
                    model, data, "smooth_payload_displacement"
                )
                rough_q = _sensor_scalar(
                    model, data, "rough_payload_displacement"
                )
                _label(
                    scene,
                    (-0.28, -0.02, 0.40),
                    (
                        f"smooth guide | T={smooth_tension:.2f} N | "
                        f"dz={1000*smooth_q:.1f} mm"
                    ),
                    white,
                )
                _label(
                    scene,
                    (0.00, -0.02, 0.40),
                    (
                        f"eyelet mu=0.45 | Tin/Tout="
                        f"{rough_tension:.2f}/{downstream:.2f} N | "
                        f"dz={1000*rough_q:.1f} mm"
                    ),
                    white,
                )
            viewer.sync()
            delay = float(model.opt.timestep) - (
                time.monotonic() - step_started
            )
            if delay > 0:
                time.sleep(delay)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
