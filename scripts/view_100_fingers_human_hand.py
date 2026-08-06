#!/usr/bin/env python3
"""View the 100-fingers hand with automatic or manual cable commands."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "cable_plugin_demos/26_cpp_plugin_100_fingers_human_hand.xml"
SITE_ROLE_COLORS = {
    1: np.array([1.0, 0.12, 0.08, 1.0]),
    2: np.array([1.0, 0.78, 0.0, 1.0]),
    3: np.array([0.0, 0.85, 1.0, 1.0]),
}


def _default_plugin() -> Path:
    candidates = [
        ROOT / "build/plugin/libcable_unilateral.dylib",
        ROOT / "build/plugin/libcable_unilateral.so",
        ROOT / "lib/libcable_unilateral.dylib",
        ROOT / "lib/libcable_unilateral.so",
    ]
    configured = os.environ.get("MUJOCABLE_PLUGIN")
    if configured:
        return Path(configured)
    return next((candidate for candidate in candidates if candidate.is_file()), candidates[0])


def smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def set_ctrl_if_present(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    name: str,
    value: float,
) -> None:
    actuator_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
    )
    if actuator_id >= 0:
        data.ctrl[actuator_id] = value


def command_grasp(model: mujoco.MjModel, data: mujoco.MjData, phase: float) -> None:
    """Close, hold, and reopen while preserving tension-only behavior."""
    cycle = phase % 8.0
    if cycle < 1.0:
        close, reopen = 0.0, 0.0
    elif cycle < 3.0:
        close, reopen = smoothstep((cycle - 1.0) / 2.0), 0.0
    elif cycle < 5.0:
        close, reopen = 1.0, 0.0
    elif cycle < 7.0:
        close = 1.0 - smoothstep((cycle - 5.0) / 2.0)
        reopen = smoothstep((cycle - 5.0) / 2.0)
    else:
        close, reopen = 0.0, 1.0 - smoothstep(cycle - 7.0)

    data.ctrl[:] = 0.0
    for digit in ("index", "middle", "ring", "little", "thumb"):
        scale = 0.82 if digit == "thumb" else 1.0
        set_ctrl_if_present(
            model,
            data,
            f"{digit}_flexor_intermediate_command",
            0.0035 * scale * close,
        )
        set_ctrl_if_present(
            model,
            data,
            f"{digit}_flexor_distal_command",
            0.0060 * scale * close,
        )
        set_ctrl_if_present(
            model,
            data,
            f"{digit}_extensor_command",
            0.0030 * scale * reopen,
        )

    # Draw the digits slightly toward the middle finger during closure.
    set_ctrl_if_present(model, data, "index_adductor_command", 0.0007 * close)
    set_ctrl_if_present(model, data, "ring_abductor_command", 0.0005 * close)
    set_ctrl_if_present(model, data, "little_abductor_command", 0.0007 * close)
    set_ctrl_if_present(model, data, "thumb_adductor_command", 0.0008 * close)


def _route_site_ids(model: mujoco.MjModel, tendon_id: int) -> set[int]:
    result: set[int] = set()
    first = int(model.tendon_adr[tendon_id])
    for wrap_id in range(first, first + int(model.tendon_num[tendon_id])):
        if model.wrap_type[wrap_id] == mujoco.mjtWrap.mjWRAP_SITE:
            result.add(int(model.wrap_objid[wrap_id]))
    return result


def configure_route_debug(
    model: mujoco.MjModel,
    cable_name: str | None = None,
    route_json: Path | None = None,
) -> None:
    """Reveal route roles and analytic wrapping geometry for manual tuning."""
    selected_sites: set[int] | None = None
    if cable_name:
        tendon_name = (
            cable_name if cable_name.endswith("_seed") else f"{cable_name}_seed"
        )
        tendon_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name
        )
        if tendon_id < 0:
            raise ValueError(f"Cable seed tendon not found: {tendon_name}")
        selected_sites = _route_site_ids(model, tendon_id)
        for other_id in range(model.ntendon):
            model.tendon_rgba[other_id, 3] = 1.0 if other_id == tendon_id else 0.06
        print(f"Debugging only: {tendon_name}")

    print("Cable route sites: role 1=endpoint, 2=initial hint, 3=physical guide")
    exported: list[dict[str, object]] = []
    for site_id in range(model.nsite):
        role = int(round(model.site_user[site_id, 0])) if model.nuser_site else 0
        if role not in SITE_ROLE_COLORS:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, site_id)
        if not name or not name.startswith("index_"):
            continue
        if selected_sites is not None and site_id not in selected_sites:
            model.site_rgba[site_id, 3] = 0.0
            continue
        body_id = model.site_bodyid[site_id]
        body_name = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY, body_id
        )
        model.site_rgba[site_id] = SITE_ROLE_COLORS[role]
        model.site_size[site_id, 0] = max(model.site_size[site_id, 0], 0.00065)
        local = " ".join(f"{value:.6g}" for value in model.site_pos[site_id])
        print(f"  role={role} body={body_name} site={name} pos=\"{local}\"")
        exported.append(
            {
                "site": name,
                "role": role,
                "body": body_name,
                "local_pos": model.site_pos[site_id].tolist(),
            }
        )

    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        if name and name.startswith("index_") and name.endswith("_wrap"):
            model.geom_rgba[geom_id] = np.array([0.1, 0.9, 0.35, 0.24])
    if route_json is not None:
        route_json.write_text(
            json.dumps(exported, indent=2) + "\n", encoding="utf-8"
        )
        print(f"Wrote route-site coordinates to {route_json}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", type=Path, default=_default_plugin())
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument(
        "--camera",
        default="overview",
        help="Fixed camera name, or 'free' to retain the interactive camera.",
    )
    parser.add_argument(
        "--show-route-debug",
        action="store_true",
        help="Show index endpoints, hints, guides, and analytic wrap geoms.",
    )
    parser.add_argument(
        "--debug-cable",
        help=(
            "Highlight one seed route, for example index_flexor_distal or "
            "index_ligament_pip_left."
        ),
    )
    parser.add_argument(
        "--route-json",
        type=Path,
        help="Write the displayed route sites and local coordinates to JSON.",
    )
    parser.add_argument(
        "--freeze",
        action="store_true",
        help="Keep the neutral pose fixed while inspecting route geometry.",
    )
    parser.add_argument(
        "--manual", action="store_true",
        help="Do not write controls; use the MuJoCo Control panel manually.",
    )
    args = parser.parse_args()

    plugin = args.plugin.expanduser().resolve()
    model_path = args.model.expanduser().resolve()
    if not plugin.is_file():
        raise FileNotFoundError(
            f"Plugin library not found: {plugin}. Build the project or set "
            "MUJOCABLE_PLUGIN."
        )
    mujoco.mj_loadPluginLibrary(str(plugin))
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    if args.show_route_debug or args.debug_cable or args.route_json:
        route_json = args.route_json.expanduser().resolve() if args.route_json else None
        configure_route_debug(model, args.debug_cable, route_json)
    camera_id = -1
    if args.camera != "free":
        camera_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, args.camera
        )
        if camera_id < 0:
            raise ValueError(f"Camera not found: {args.camera}")

    wall_start = time.monotonic()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        if camera_id >= 0:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = camera_id
        while viewer.is_running() and time.monotonic() - wall_start < args.duration:
            step_start = time.monotonic()
            if not args.manual and not args.freeze:
                command_grasp(model, data, data.time)
            if args.freeze:
                mujoco.mj_forward(model, data)
            else:
                mujoco.mj_step(model, data)
            viewer.sync()
            remaining = model.opt.timestep - (time.monotonic() - step_start)
            if remaining > 0:
                time.sleep(remaining)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
