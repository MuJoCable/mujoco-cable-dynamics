#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

try:
    from view_cpp_plugin_demo import (
        _spool_debug_geometry,
        _spool_debug_specs,
        _unit,
    )
except ModuleNotFoundError:
    from scripts.view_cpp_plugin_demo import (
        _spool_debug_geometry,
        _spool_debug_specs,
        _unit,
    )


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = (
    ROOT
    / "cable_plugin_demos"
    / "21_cpp_plugin_wheel_axle_force_amplifier.xml"
)
DEFAULT_PLUGIN = ROOT / "build/cable_surface/plugin/libcable_unilateral.dylib"


def analyze(plugin: Path, model_path: Path) -> dict[str, Any]:
    mujoco.mj_loadPluginLibrary(str(plugin))
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rows: list[dict[str, Any]] = []

    for spec in _spool_debug_specs(model, model_path):
        geometry = _spool_debug_geometry(model, data, spec)
        center = geometry.center
        axis = geometry.cylinder_axis
        fixed = geometry.fixed
        exit_point = geometry.exit_point
        fixed_delta = fixed - center
        fixed_axial = float(np.dot(fixed_delta, axis))
        fixed_radial = fixed_delta - fixed_axial * axis
        exit_delta = exit_point - center
        exit_axial = float(np.dot(exit_delta, axis))
        exit_radial = exit_delta - exit_axial * axis

        initial_signed_angle = geometry.signed_wound_angle

        address = int(data.ten_wrapadr[spec.tendon_id])
        count = int(data.ten_wrapnum[spec.tendon_id])
        points = np.asarray(data.wrap_xpos, dtype=float).reshape(-1, 3)[
            address : address + count
        ]
        if count < 2:
            raise RuntimeError(f"{spec.label} tendon has fewer than two points")
        free_direction = _unit(points[1] - points[0])
        tangent_residual = abs(float(np.dot(_unit(exit_radial), free_direction)))
        dynamic_phase_errors = []
        for qdelta in (-0.35, 0.0, 0.35):
            data.qpos[spec.qpos_address] = spec.qpos0 + qdelta
            mujoco.mj_forward(model, data)
            dynamic_geometry = _spool_debug_geometry(model, data, spec)
            dynamic_phase_errors.append(
                float(
                    np.linalg.norm(
                        dynamic_geometry.expected_exit
                        - dynamic_geometry.exit_point
                    )
                )
            )
        data.qpos[spec.qpos_address] = spec.qpos0
        mujoco.mj_forward(model, data)
        row = {
            "label": spec.label,
            "fixed_site": (
                mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_SITE, spec.fixed_site_id
                )
                or ""
            ),
            "exit_site": (
                mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_SITE, spec.exit_site_id
                )
                or ""
            ),
            "spool_radius_m": spec.spool_radius,
            "reserve_length_m": spec.reserve_length,
            "initial_signed_turns": initial_signed_angle / (2.0 * np.pi),
            "fixed_radius_m": float(np.linalg.norm(fixed_radial)),
            "exit_radius_m": float(np.linalg.norm(exit_radial)),
            "fixed_radius_error_m": (
                float(np.linalg.norm(fixed_radial)) - spec.spool_radius
            ),
            "exit_radius_error_m": (
                float(np.linalg.norm(exit_radial)) - spec.spool_radius
            ),
            "axial_lane_error_m": exit_axial - fixed_axial,
            "tangent_residual": tangent_residual,
            "axis_alignment": geometry.axis_alignment,
            "phase_error_m": float(
                np.linalg.norm(geometry.expected_exit - exit_point)
            ),
            "dynamic_phase_error_max_m": max(dynamic_phase_errors),
            "fixed_is_attached_to_drum_body": (
                int(model.site_bodyid[spec.fixed_site_id])
                == int(model.geom_bodyid[spec.geom_id])
            ),
            "exit_is_world_fixed": int(model.site_bodyid[spec.exit_site_id]) == 0,
        }
        row["pass"] = (
            abs(float(row["fixed_radius_error_m"])) <= 1e-6
            and abs(float(row["exit_radius_error_m"])) <= 1e-9
            and abs(float(row["axial_lane_error_m"])) <= 1e-9
            and float(row["tangent_residual"]) <= 1e-6
            and float(row["phase_error_m"]) <= 1e-6
            and float(row["dynamic_phase_error_max_m"]) <= 1e-6
            and bool(row["fixed_is_attached_to_drum_body"])
            and bool(row["exit_is_world_fixed"])
        )
        rows.append(row)

    return {
        "pass": bool(rows) and all(bool(row["pass"]) for row in rows),
        "model": str(model_path),
        "plugin": str(plugin),
        "spools": rows,
        "visual_semantics": {
            "fixed": "body-fixed rope tie point; edit in drum-local coordinates",
            "exit": "world-fixed tangent point where the free cable begins",
            "expected_exit": "cyan point predicted from fixed phase, reserve, and shaft angle",
            "phase_error": "distance from predicted exit to the XML exit site",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    report = analyze(args.plugin.resolve(), args.model.resolve())
    text = json.dumps(report, indent=2)
    print(text)
    if args.out:
        args.out.resolve().parent.mkdir(parents=True, exist_ok=True)
        args.out.resolve().write_text(text + "\n", encoding="utf-8")
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
