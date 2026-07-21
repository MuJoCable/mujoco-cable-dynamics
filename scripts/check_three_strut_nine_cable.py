#!/usr/bin/env python3
"""Validate equilibrium and compliance of Demo 17's tensegrity prism."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "cable_plugin_demos" / "17_cpp_plugin_three_strut_nine_cable.xml"
DEFAULT_PLUGIN = ROOT / "build/cable_surface/plugin/libcable_unilateral.dylib"
STRUT_NAMES = ("strut_0", "strut_1", "strut_2")
NODE_PAIRS = (("node_b0", "node_t1"), ("node_b1", "node_t2"), ("node_b2", "node_t0"))


def _body_ids(model: mujoco.MjModel) -> np.ndarray:
    return np.asarray(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in STRUT_NAMES],
        dtype=np.int32,
    )


def _site_ids(model: mujoco.MjModel) -> np.ndarray:
    names = ("node_b0", "node_b1", "node_b2", "node_t0", "node_t1", "node_t2")
    return np.asarray(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name) for name in names],
        dtype=np.int32,
    )


def _center_of_mass(model: mujoco.MjModel, data: mujoco.MjData, body_ids: np.ndarray) -> np.ndarray:
    masses = np.asarray(model.body_mass, dtype=float)[body_ids]
    positions = np.asarray(data.xipos, dtype=float)[body_ids]
    return np.sum(masses[:, None] * positions, axis=0) / float(np.sum(masses))


def _strut_lengths(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    lengths = []
    for start_name, end_name in NODE_PAIRS:
        start_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, start_name)
        end_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, end_name)
        lengths.append(float(np.linalg.norm(data.site_xpos[end_id] - data.site_xpos[start_id])))
    return np.asarray(lengths)


def _tensions(data: mujoco.MjData) -> np.ndarray:
    return -np.asarray(data.actuator_force, dtype=float).copy()


def analyze_demo(
    plugin_path: Path = DEFAULT_PLUGIN,
    model_path: Path = DEFAULT_MODEL,
    *,
    settle_steps: int = 6000,
    ramp_steps: int = 2000,
    hold_steps: int = 2000,
    release_steps: int = 8000,
    contraction_m: float = 0.06,
) -> dict[str, object]:
    mujoco.mj_loadPluginLibrary(str(Path(plugin_path).resolve()))
    model = mujoco.MjModel.from_xml_path(str(Path(model_path).resolve()))
    data = mujoco.MjData(model)
    body_ids = _body_ids(model)
    site_ids = _site_ids(model)

    mujoco.mj_forward(model, data)
    initial_lengths = np.asarray(data.ten_length, dtype=float).copy()
    initial_strut_lengths = _strut_lengths(model, data)
    initial_tensions = _tensions(data)
    initial_force_residual = float(np.linalg.norm(data.qfrc_actuator))
    ring_to_cross_ratio = float(np.mean(initial_tensions[:6]) / np.mean(initial_tensions[6:]))

    maximum_settle_contact_count = int(data.ncon)
    minimum_settle_contact_count = int(data.ncon)
    for _ in range(settle_steps):
        data.ctrl[:] = 0
        mujoco.mj_step(model, data)
        maximum_settle_contact_count = max(maximum_settle_contact_count, int(data.ncon))
        minimum_settle_contact_count = min(minimum_settle_contact_count, int(data.ncon))

    settled_lengths = np.asarray(data.ten_length, dtype=float).copy()
    settled_nodes = np.asarray(data.site_xpos, dtype=float)[site_ids].copy()
    settled_pairwise = np.linalg.norm(
        settled_nodes[:, None, :] - settled_nodes[None, :, :], axis=2
    )
    settled_tensions = _tensions(data)
    settled_contact_count = int(data.ncon)
    settled_speed = float(np.linalg.norm(data.qvel))
    settled_minimum_node_height = float(np.min(settled_nodes[:, 2]))
    settled_com = _center_of_mass(model, data, body_ids)

    controlled_actuator = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_cross_00"
    )
    maximum_cable_length_change = 0.0
    maximum_node_displacement = 0.0
    maximum_shape_change = 0.0
    maximum_strut_length_error = 0.0
    maximum_tension = float(np.max(_tensions(data)))
    maximum_com_drift = 0.0
    minimum_contact_count = int(data.ncon)
    maximum_contact_count = int(data.ncon)
    minimum_node_height = settled_minimum_node_height

    total_cycle_steps = ramp_steps + hold_steps + release_steps
    for step in range(total_cycle_steps):
        data.ctrl[:] = 0
        if step < ramp_steps:
            data.ctrl[controlled_actuator] = contraction_m * step / ramp_steps
        elif step < ramp_steps + hold_steps:
            data.ctrl[controlled_actuator] = contraction_m
        elif step < ramp_steps + hold_steps + ramp_steps:
            release_fraction = (step - ramp_steps - hold_steps) / ramp_steps
            data.ctrl[controlled_actuator] = contraction_m * (1.0 - release_fraction)
        mujoco.mj_step(model, data)
        maximum_cable_length_change = max(
            maximum_cable_length_change,
            float(np.max(np.abs(np.asarray(data.ten_length) - settled_lengths))),
        )
        maximum_node_displacement = max(
            maximum_node_displacement,
            float(np.max(np.linalg.norm(np.asarray(data.site_xpos)[site_ids] - settled_nodes, axis=1))),
        )
        current_nodes = np.asarray(data.site_xpos)[site_ids]
        current_pairwise = np.linalg.norm(
            current_nodes[:, None, :] - current_nodes[None, :, :], axis=2
        )
        maximum_shape_change = max(
            maximum_shape_change,
            float(np.max(np.abs(current_pairwise - settled_pairwise))),
        )
        maximum_strut_length_error = max(
            maximum_strut_length_error,
            float(np.max(np.abs(_strut_lengths(model, data) - initial_strut_lengths))),
        )
        maximum_tension = max(maximum_tension, float(np.max(_tensions(data))))
        maximum_com_drift = max(
            maximum_com_drift,
            float(np.linalg.norm(_center_of_mass(model, data, body_ids) - settled_com)),
        )
        minimum_contact_count = min(minimum_contact_count, int(data.ncon))
        maximum_contact_count = max(maximum_contact_count, int(data.ncon))
        minimum_node_height = min(minimum_node_height, float(np.min(current_nodes[:, 2])))

    final_length_error = float(np.max(np.abs(np.asarray(data.ten_length) - settled_lengths)))
    final_nodes = np.asarray(data.site_xpos)[site_ids]
    final_pairwise = np.linalg.norm(
        final_nodes[:, None, :] - final_nodes[None, :, :], axis=2
    )
    final_shape_error = float(np.max(np.abs(final_pairwise - settled_pairwise)))
    final_tensions = _tensions(data)
    final_speed = float(np.linalg.norm(data.qvel))
    finite = bool(
        np.all(np.isfinite(data.qpos))
        and np.all(np.isfinite(data.qvel))
        and np.all(np.isfinite(final_tensions))
    )

    structure_ok = bool(
        model.nq == 21
        and model.nv == 18
        and model.nu == 9
        and model.ntendon == 9
        and model.nsensor == 9
        and model.neq == 0
        and np.all(np.asarray(model.jnt_type) == int(mujoco.mjtJoint.mjJNT_FREE))
        and float(model.opt.gravity[2]) < -9.0
        and mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "grid_ground") >= 0
    )
    maximum_control_range = float(np.min(np.asarray(model.actuator_ctrlrange)[:, 1]))
    checks = {
        "structure": structure_ok,
        "all_cables_initially_taut": bool(np.all(initial_tensions > 0.1)),
        "self_stress_ratio": abs(ring_to_cross_ratio - 0.48405) < 5e-5,
        "initial_force_balance": initial_force_residual < 1e-4,
        "grounded_settle": settled_contact_count >= 3
        and settled_minimum_node_height > 0.005
        and settled_speed < 0.01,
        "large_control_range": maximum_control_range >= contraction_m,
        "visible_contraction_response": maximum_shape_change > 0.03,
        "cables_change_length": maximum_cable_length_change > 0.03,
        "rigid_struts": maximum_strut_length_error < 1e-10,
        "continuous_ground_contact": minimum_contact_count >= 3 and minimum_node_height > 0.005,
        "recovery": final_length_error < 1e-4
        and final_shape_error < 5e-4
        and final_speed < 0.01,
        "no_tension_saturation": maximum_tension < 80.0,
        "finite_state": finite,
    }

    return {
        "pass": bool(all(checks.values())),
        "checks": checks,
        "structure": {
            "free_bodies": len(body_ids),
            "generalized_coordinates": int(model.nq),
            "degrees_of_freedom": int(model.nv),
            "cables": int(model.ntendon),
            "plugin_force_transmissions": int(model.nu),
            "equalities": int(model.neq),
        },
        "initial": {
            "tensions_n": initial_tensions.tolist(),
            "ring_to_cross_tension_ratio": ring_to_cross_ratio,
            "generalized_force_residual": initial_force_residual,
            "cable_lengths_m": initial_lengths.tolist(),
            "strut_lengths_m": initial_strut_lengths.tolist(),
        },
        "ground_settle": {
            "duration_s": settle_steps * float(model.opt.timestep),
            "minimum_contact_count": minimum_settle_contact_count,
            "maximum_contact_count": maximum_settle_contact_count,
            "final_contact_count": settled_contact_count,
            "minimum_node_height_m": settled_minimum_node_height,
            "final_speed": settled_speed,
            "tensions_n": settled_tensions.tolist(),
            "cable_length_change_from_analytic_pose_m": float(
                np.max(np.abs(settled_lengths - initial_lengths))
            ),
        },
        "contraction_cycle": {
            "controlled_actuator": "act_cross_00",
            "commanded_contraction_m": contraction_m,
            "ramp_duration_s": ramp_steps * float(model.opt.timestep),
            "hold_duration_s": hold_steps * float(model.opt.timestep),
            "release_duration_s": release_steps * float(model.opt.timestep),
            "maximum_cable_length_change_m": maximum_cable_length_change,
            "maximum_node_displacement_m": maximum_node_displacement,
            "maximum_pairwise_shape_change_m": maximum_shape_change,
            "maximum_strut_length_error_m": maximum_strut_length_error,
            "maximum_center_of_mass_drift_m": maximum_com_drift,
            "minimum_contact_count": minimum_contact_count,
            "maximum_contact_count": maximum_contact_count,
            "minimum_node_height_m": minimum_node_height,
            "maximum_tension_n": maximum_tension,
            "final_cable_length_error_m": final_length_error,
            "final_pairwise_shape_error_m": final_shape_error,
            "final_speed": final_speed,
            "final_tensions_n": final_tensions.tolist(),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate Demo 17 three-strut nine-cable tensegrity")
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()

    report = analyze_demo(plugin_path=args.plugin, model_path=args.model)
    payload = json.dumps(report, indent=2)
    print(payload)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(payload + "\n", encoding="utf-8")
    return 1 if args.strict and not report["pass"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
