from __future__ import annotations

import csv
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .config import load_config, resolve_path
from .unilateral_law import CableLawResult, UnilateralCableLaw


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEMO_CONFIGS = {
    "00": PROJECT_ROOT / "configs" / "demo00_two_body.yaml",
    "0": PROJECT_ROOT / "configs" / "demo00_two_body.yaml",
    "01": PROJECT_ROOT / "configs" / "demo01_single_pulley.yaml",
    "1": PROJECT_ROOT / "configs" / "demo01_single_pulley.yaml",
    "03": PROJECT_ROOT / "configs" / "demo03_warren_frame.yaml",
    "3": PROJECT_ROOT / "configs" / "demo03_warren_frame.yaml",
    "04": PROJECT_ROOT / "configs" / "demo04_tensegrity_path.yaml",
    "4": PROJECT_ROOT / "configs" / "demo04_tensegrity_path.yaml",
    "05": PROJECT_ROOT / "configs" / "demo05_cylinder_wrap.yaml",
    "5": PROJECT_ROOT / "configs" / "demo05_cylinder_wrap.yaml",
    "07": PROJECT_ROOT / "configs" / "demo07_moving_wrap.yaml",
    "7": PROJECT_ROOT / "configs" / "demo07_moving_wrap.yaml",
}


@dataclass
class CableRuntime:
    name: str
    tendon_id: int
    home_length: float
    pretension_offset: float
    spool_radius: float
    command: dict[str, Any]
    law: UnilateralCableLaw


def _name2id(model: mujoco.MjModel, objtype: int, name: str) -> int:
    obj_id = mujoco.mj_name2id(model, objtype, name)
    if obj_id < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return obj_id


def _command_contraction(command: dict[str, Any], time: float, spool_radius: float = 1.0) -> float:
    ctype = command.get("type", "none")
    if ctype == "none":
        return 0.0
    if ctype == "constant_contraction":
        return float(command.get("contraction", 0.0))
    if ctype == "constant_spool_angle":
        return spool_radius * float(command.get("angle", 0.0))
    if ctype == "ramp_contraction":
        start = float(command.get("start_time", 0.0))
        end = float(command.get("end_time", start))
        final = float(command.get("final_contraction", 0.0))
        if time <= start:
            return 0.0
        if end <= start or time >= end:
            return final
        return final * (time - start) / (end - start)
    if ctype == "ramp_spool_angle":
        start = float(command.get("start_time", 0.0))
        end = float(command.get("end_time", start))
        final_angle = float(command.get("final_angle", 0.0))
        if time <= start:
            return 0.0
        if end <= start or time >= end:
            return spool_radius * final_angle
        return spool_radius * final_angle * (time - start) / (end - start)
    raise ValueError(f"Unsupported command type: {ctype}")


def _apply_tendon_force(model: mujoco.MjModel, data: mujoco.MjData, tendon_id: int, scalar_force: float) -> None:
    del model
    ten_j = np.asarray(data.ten_J)
    if ten_j.ndim == 2:
        data.qfrc_applied[:] += scalar_force * ten_j[tendon_id]
        return

    rowadr = int(data.ten_J_rowadr[tendon_id])
    rownnz = int(data.ten_J_rownnz[tendon_id])
    for offset in range(rownnz):
        adr = rowadr + offset
        col = int(data.ten_J_colind[adr])
        data.qfrc_applied[col] += scalar_force * ten_j[adr]


def _wrap_geom_count(model: mujoco.MjModel, tendon_id: int) -> int:
    adr = int(model.tendon_adr[tendon_id])
    num = int(model.tendon_num[tendon_id])
    if num <= 0:
        return 0
    wrap_type = np.asarray(model.wrap_type)[adr : adr + num]
    return int(np.count_nonzero(wrap_type >= int(mujoco.mjtWrap.mjWRAP_SPHERE)))


def _site_position(model: mujoco.MjModel, data: mujoco.MjData, name: str | None) -> list[float] | None:
    if not name:
        return None
    site_id = _name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    return [float(x) for x in np.asarray(data.site_xpos)[site_id]]


def _shortest_angle(theta_a: float, theta_b: float) -> float:
    diff = abs(theta_a - theta_b) % (2.0 * np.pi)
    return float(min(diff, 2.0 * np.pi - diff))


def _active_wrap_rows(data: mujoco.MjData) -> list[tuple[np.ndarray, np.ndarray]]:
    rows: list[tuple[np.ndarray, np.ndarray]] = []
    wrap_obj = np.asarray(data.wrap_obj).reshape(-1, 2)
    wrap_xpos = np.asarray(data.wrap_xpos).reshape(-1, 2, 3)
    for obj_pair, xpos_pair in zip(wrap_obj, wrap_xpos, strict=False):
        if np.all(obj_pair == 0) and np.allclose(xpos_pair, 0):
            continue
        if any(np.allclose(point, 0, atol=1e-12, rtol=0.0) for point in xpos_pair):
            continue
        rows.append((np.asarray(obj_pair, dtype=int), np.asarray(xpos_pair, dtype=float)))
    return rows


def _wrap_diagnostics(model: mujoco.MjModel, data: mujoco.MjData, tension: float) -> dict[str, str]:
    segment_lengths: list[float] = []
    segment_tensions: list[float] = []
    wrap_angles: list[float] = []
    contact_points: list[list[float]] = []
    segment_types: list[str] = []
    contacts_by_geom: dict[int, list[np.ndarray]] = {}
    explicit_arc_geoms: set[int] = set()

    for obj_pair, xpos_pair in _active_wrap_rows(data):
        p0 = xpos_pair[0]
        p1 = xpos_pair[1]
        if int(obj_pair[0]) >= 0 and int(obj_pair[0]) == int(obj_pair[1]):
            geom_id = int(obj_pair[0])
            explicit_arc_geoms.add(geom_id)
            center = np.asarray(data.geom_xpos)[geom_id]
            radius = float(model.geom_size[geom_id, 0])
            theta0 = np.arctan2(float(p0[2] - center[2]), float(p0[0] - center[0]))
            theta1 = np.arctan2(float(p1[2] - center[2]), float(p1[0] - center[0]))
            angle = _shortest_angle(theta0, theta1)
            segment_lengths.append(radius * angle)
            wrap_angles.append(angle)
            contact_points.extend([[float(x) for x in p0], [float(x) for x in p1]])
            segment_types.append("arc")
        else:
            segment_lengths.append(float(np.linalg.norm(p1 - p0)))
            for obj_id, point in zip(obj_pair, xpos_pair, strict=False):
                if int(obj_id) >= 0:
                    contact_points.append([float(x) for x in point])
                    contacts_by_geom.setdefault(int(obj_id), []).append(np.asarray(point, dtype=float))
            segment_types.append("straight")
        segment_tensions.append(float(tension))

    for geom_id, points in contacts_by_geom.items():
        unique_points: list[np.ndarray] = []
        for point in points:
            if not any(np.linalg.norm(point - existing) <= 1e-9 for existing in unique_points):
                unique_points.append(point)
        if geom_id in explicit_arc_geoms or len(unique_points) < 2:
            continue
        p0, p1 = unique_points[0], unique_points[1]
        center = np.asarray(data.geom_xpos)[geom_id]
        radius = float(model.geom_size[geom_id, 0])
        theta0 = np.arctan2(float(p0[2] - center[2]), float(p0[0] - center[0]))
        theta1 = np.arctan2(float(p1[2] - center[2]), float(p1[0] - center[0]))
        angle = _shortest_angle(theta0, theta1)
        segment_lengths.append(radius * angle)
        segment_tensions.append(float(tension))
        segment_types.append("arc")
        wrap_angles.append(angle)

    return {
        "segment_lengths": json.dumps(segment_lengths),
        "segment_tensions": json.dumps(segment_tensions),
        "segment_types": json.dumps(segment_types),
        "wrap_angles": json.dumps(wrap_angles),
        "contact_points": json.dumps(contact_points),
    }


def _make_runtime(model: mujoco.MjModel, data: mujoco.MjData, cable: dict[str, Any]) -> CableRuntime:
    tendon_name = cable["tendon"]
    tendon_id = _name2id(model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name)
    home = cable.get("home_length", "auto_initial")
    home_length = float(data.ten_length[tendon_id]) if home == "auto_initial" else float(home)

    max_tension = cable.get("max_tension")
    law = UnilateralCableLaw(
        stiffness=float(cable.get("stiffness", 0.0)),
        damping=float(cable.get("damping", 0.0)),
        slack_threshold=float(cable.get("slack_threshold", 0.0)),
        max_tension=None if max_tension is None else float(max_tension),
    )
    return CableRuntime(
        name=str(cable.get("name", tendon_name)),
        tendon_id=tendon_id,
        home_length=home_length,
        pretension_offset=float(cable.get("pretension_offset", 0.0)),
        spool_radius=float(cable.get("spool_radius", 1.0)),
        command=dict(cable.get("command", {"type": "none"})),
        law=law,
    )


def _evaluate(runtime: CableRuntime, data: mujoco.MjData, time: float) -> tuple[float, float, CableLawResult]:
    contraction = _command_contraction(runtime.command, time, runtime.spool_radius)
    if "min_contraction" in runtime.command:
        contraction = max(float(runtime.command["min_contraction"]), contraction)
    if "max_contraction" in runtime.command:
        contraction = min(float(runtime.command["max_contraction"]), contraction)

    free_length = runtime.home_length - contraction - runtime.pretension_offset
    result = runtime.law.evaluate(
        path_length=float(data.ten_length[runtime.tendon_id]),
        free_length=free_length,
        path_velocity=float(data.ten_velocity[runtime.tendon_id]),
    )
    return contraction, free_length, result


def run_config(config_path: str | Path, out_dir: str | Path) -> dict[str, Any]:
    config_path = Path(config_path).resolve()
    config = load_config(config_path)
    model_path = resolve_path(config_path, config["model"])
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    simulation = config.get("simulation", {})
    dt = float(simulation.get("dt", model.opt.timestep))
    duration = float(simulation.get("duration", 1.0))
    if abs(dt - float(model.opt.timestep)) > 1e-12:
        model.opt.timestep = dt
    nstep = int(round(duration / model.opt.timestep))

    runtimes = [_make_runtime(model, data, cable) for cable in config.get("cables", [])]
    if not runtimes:
        raise ValueError("Config must define at least one cable")

    payload_site = config.get("payload_site")
    metrics_rows: list[dict[str, Any]] = []
    qpos = []
    qvel = []

    for _ in range(nstep + 1):
        time = float(data.time)
        data.qfrc_applied[:] = 0.0
        mujoco.mj_forward(model, data)

        payload_pos = _site_position(model, data, payload_site)
        qpos.append(np.array(data.qpos, copy=True))
        qvel.append(np.array(data.qvel, copy=True))

        for runtime in runtimes:
            contraction, free_length, result = _evaluate(runtime, data, time)
            _apply_tendon_force(model, data, runtime.tendon_id, -result.tension)
            diagnostics = _wrap_diagnostics(model, data, result.tension)
            metrics_rows.append(
                {
                    "time": time,
                    "cable": runtime.name,
                    "tendon_id": runtime.tendon_id,
                    "L_path": float(data.ten_length[runtime.tendon_id]),
                    "Ldot": float(data.ten_velocity[runtime.tendon_id]),
                    "L_home": runtime.home_length,
                    "L_free": free_length,
                    "contraction": contraction,
                    "spool_radius": runtime.spool_radius,
                    "extension": result.extension,
                    "positive_extension": result.positive_extension,
                    "tension": result.tension,
                    "taut": int(result.taut),
                    "saturation": int(result.saturated),
                    "wrap_geom_count": _wrap_geom_count(model, runtime.tendon_id),
                    "segment_lengths": diagnostics["segment_lengths"],
                    "segment_tensions": diagnostics["segment_tensions"],
                    "segment_types": diagnostics["segment_types"],
                    "wrap_angles": diagnostics["wrap_angles"],
                    "contact_points": diagnostics["contact_points"],
                    "energy_elastic": 0.5 * runtime.law.stiffness * result.positive_extension**2,
                    "winch_work": result.tension * contraction,
                    "friction_work": 0.0,
                    "payload_x": None if payload_pos is None else payload_pos[0],
                    "payload_y": None if payload_pos is None else payload_pos[1],
                    "payload_z": None if payload_pos is None else payload_pos[2],
                }
            )

        if data.time >= duration:
            break
        mujoco.mj_step(model, data)

    fieldnames = list(metrics_rows[0].keys())
    with (out_dir / "metrics.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(metrics_rows)

    np.savez_compressed(out_dir / "trajectory.npz", qpos=np.asarray(qpos), qvel=np.asarray(qvel))

    summary: dict[str, Any] = {
        "model": str(model_path),
        "duration": duration,
        "dt": float(model.opt.timestep),
        "steps": len(qpos),
        "cables": [],
    }
    for runtime in runtimes:
        rows = [row for row in metrics_rows if row["cable"] == runtime.name]
        summary["cables"].append(
            {
                "name": runtime.name,
                "home_length": runtime.home_length,
                "spool_radius": runtime.spool_radius,
                "max_tension": max(row["tension"] for row in rows),
                "final_tension": rows[-1]["tension"],
                "final_L_path": rows[-1]["L_path"],
                "final_L_free": rows[-1]["L_free"],
                "max_wrap_geom_count": max(row["wrap_geom_count"] for row in rows),
            }
        )

    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def resolve_demo_config(demo: str) -> Path:
    key = str(demo)
    if key in {"02", "2"}:
        raise ValueError("Demo 02 is intentionally skipped for this task.")
    if key in {"06", "6"}:
        raise ValueError("Demo 06 is an analytic Capstan sweep; use scripts/run_capstan_sweep.py.")
    if key not in DEMO_CONFIGS:
        choices = ", ".join(sorted(DEMO_CONFIGS))
        raise ValueError(f"Unsupported demo {key!r}. Choose one of: {choices}")
    return DEMO_CONFIGS[key]


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Run a MuJoCo unilateral cable demo")
    parser.add_argument("--config", help="Path to demo YAML config")
    parser.add_argument("--demo", help="Demo id such as 00, 01, 03, 04, 05, or 07")
    parser.add_argument(
        "--mode",
        default="python",
        choices=["python"],
        help="Compatibility option for TASK.md commands; only python mode is implemented here.",
    )
    parser.add_argument("--out", required=True, help="Output directory")
    args = parser.parse_args(argv)

    if args.config:
        config_path = Path(args.config)
    elif args.demo:
        config_path = resolve_demo_config(args.demo)
    else:
        parser.error("one of --config or --demo is required")

    summary = run_config(config_path, args.out)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
