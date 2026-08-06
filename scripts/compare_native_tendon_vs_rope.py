#!/usr/bin/env python3
"""Compare native MuJoCo tendon springs with the unilateral cable plugin."""

from __future__ import annotations

import argparse
import csv
import json
import sqlite3
import statistics
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLUGIN = ROOT / "build/plugin/libcable_unilateral.dylib"
DEFAULT_PLUGIN_MODEL = ROOT / "cable_plugin_demos" / "17_cpp_plugin_three_strut_nine_cable.xml"
DEFAULT_NATIVE_MODEL = ROOT / "cable_plugin_demos" / "18_native_tendon_three_strut_nine_cable.xml"
DEFAULT_OUT = ROOT / "outputs" / "native_vs_plugin_tendon_report"
CABLE_NAMES = (
    "bottom_01",
    "bottom_12",
    "bottom_20",
    "top_01",
    "top_12",
    "top_20",
    "cross_00",
    "cross_11",
    "cross_22",
)
NODE_NAMES = ("node_b0", "node_b1", "node_b2", "node_t0", "node_t1", "node_t2")
STRUT_NODE_PAIRS = (("node_b0", "node_t1"), ("node_b1", "node_t2"), ("node_b2", "node_t0"))
REPORT_SOURCE_SQL = """\
SELECT dataset, row_index, payload_json
FROM report_snapshot
ORDER BY dataset, row_index
"""


def _site_ids(model: mujoco.MjModel) -> np.ndarray:
    return np.asarray(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name) for name in NODE_NAMES],
        dtype=np.int32,
    )


def _pairwise(points: np.ndarray) -> np.ndarray:
    return np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)


def _tendon_jacobian(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    raw = np.asarray(data.ten_J)
    if raw.ndim == 2:
        return np.asarray(raw, dtype=float).copy()
    jacobian = np.zeros((model.ntendon, model.nv), dtype=float)
    for tendon_id in range(model.ntendon):
        row_address = int(data.ten_J_rowadr[tendon_id])
        row_nonzeros = int(data.ten_J_rownnz[tendon_id])
        for offset in range(row_nonzeros):
            address = row_address + offset
            jacobian[tendon_id, int(data.ten_J_colind[address])] = float(raw[address])
    return jacobian


def _plugin_parameters(model_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    root = ET.parse(model_path).getroot()
    instances = {
        item.attrib["name"]: {
            config.attrib["key"]: config.attrib["value"]
            for config in item.findall("config")
        }
        for item in root.findall("./extension/plugin/instance")
    }
    stiffness = []
    damping = []
    offsets = []
    maximum = []
    for cable_name in CABLE_NAMES:
        config = instances[f"cable_{cable_name}"]
        stiffness.append(float(config["stiffness"]))
        damping.append(float(config["damping"]))
        offsets.append(float(config["pretension_offset"]))
        maximum.append(float(config["max_tension"]))
    return tuple(np.asarray(values, dtype=float) for values in (stiffness, damping, offsets, maximum))


def _load_matched_plugin_model(model_path: Path) -> mujoco.MjModel:
    """Load the latest plugin model without optional command smoothing.

    The visual Demo 17 intentionally rate-limits contraction to suppress cable
    chatter. A native tendon rest-length update has no equivalent internal
    state, so retaining that filter would make this a controller comparison
    rather than a matched constitutive-law comparison.
    """
    root = ET.parse(model_path).getroot()
    for instance in root.findall("./extension/plugin/instance"):
        for config in list(instance.findall("config")):
            if config.attrib.get("key") in {"control_timeconstant", "max_contraction_rate"}:
                instance.remove(config)
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def _contraction(step: int, ramp_steps: int, hold_steps: int, release_steps: int, maximum: float) -> float:
    if step < ramp_steps:
        return maximum * step / ramp_steps
    if step < ramp_steps + hold_steps:
        return maximum
    release_step = step - ramp_steps - hold_steps
    if release_step < ramp_steps:
        return maximum * (1.0 - release_step / ramp_steps)
    if release_step < release_steps:
        return 0.0
    return 0.0


def _phase(step: int, ramp_steps: int, hold_steps: int) -> str:
    if step < ramp_steps:
        return "ramp_up"
    if step < ramp_steps + hold_steps:
        return "hold"
    if step < 2 * ramp_steps + hold_steps:
        return "ramp_down"
    return "recover"


def _settle(model: mujoco.MjModel, data: mujoco.MjData, steps: int, native_rest: np.ndarray | None) -> None:
    for _ in range(steps):
        if model.nu:
            data.ctrl[:] = 0
        if native_rest is not None:
            model.tendon_lengthspring[:, 0] = native_rest
            model.tendon_lengthspring[:, 1] = native_rest
        mujoco.mj_step(model, data)


def _simulate(
    kind: str,
    model: mujoco.MjModel,
    *,
    settle_steps: int,
    ramp_steps: int,
    hold_steps: int,
    release_steps: int,
    contraction_m: float,
    sample_every: int,
    plugin_params: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None,
) -> dict[str, object]:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    initial_lengths = np.asarray(data.ten_length, dtype=float).copy()
    initial_tensions = None
    native_rest = None
    if kind == "plugin":
        initial_tensions = -np.asarray(data.actuator_force, dtype=float).copy()
    else:
        native_rest = np.asarray(model.tendon_lengthspring[:, 0], dtype=float).copy()
        initial_tensions = (
            np.asarray(model.tendon_stiffness) * (np.asarray(data.ten_length) - native_rest)
            + np.asarray(model.tendon_damping) * np.asarray(data.ten_velocity)
        )

    _settle(model, data, settle_steps, native_rest)
    site_ids = _site_ids(model)
    baseline_nodes = np.asarray(data.site_xpos, dtype=float)[site_ids].copy()
    baseline_pairwise = _pairwise(baseline_nodes)
    baseline_lengths = np.asarray(data.ten_length, dtype=float).copy()
    baseline_tensions = None
    if kind == "plugin":
        baseline_tensions = -np.asarray(data.actuator_force, dtype=float).copy()
    else:
        baseline_tensions = (
            np.asarray(model.tendon_stiffness) * (np.asarray(data.ten_length) - native_rest)
            + np.asarray(model.tendon_damping) * np.asarray(data.ten_velocity)
        )

    records: list[dict[str, object]] = []
    node_samples: list[np.ndarray] = []
    tension_samples: list[np.ndarray] = []
    peak_tensions = np.asarray(baseline_tensions, dtype=float).copy()
    maximum_theory_error = 0.0
    maximum_force_mapping_residual = 0.0
    total_steps = ramp_steps + hold_steps + release_steps
    controlled_id = CABLE_NAMES.index("cross_00")

    for step in range(total_steps):
        command = _contraction(step, ramp_steps, hold_steps, release_steps, contraction_m)
        contraction_vector = np.zeros(model.ntendon, dtype=float)
        contraction_vector[controlled_id] = command
        if kind == "plugin":
            data.ctrl[:] = 0
            data.ctrl[controlled_id] = command
        else:
            current_rest = native_rest - contraction_vector
            model.tendon_lengthspring[:, 0] = current_rest
            model.tendon_lengthspring[:, 1] = current_rest

        mujoco.mj_step(model, data)
        # Align force arrays and tendon kinematics at the same end-of-step state.
        mujoco.mj_forward(model, data)
        lengths = np.asarray(data.ten_length, dtype=float).copy()
        velocities = np.asarray(data.ten_velocity, dtype=float).copy()
        jacobian = _tendon_jacobian(model, data)

        if kind == "plugin":
            stiffness, damping, offsets, maximum = plugin_params
            free_lengths = initial_lengths - offsets - contraction_vector
            raw = stiffness * (lengths - free_lengths) + damping * velocities
            theory = np.minimum(np.maximum(raw, 0.0), maximum)
            measured = -np.asarray(data.actuator_force, dtype=float).copy()
            actual_generalized = np.asarray(data.qfrc_actuator, dtype=float).copy()
        else:
            free_lengths = np.asarray(model.tendon_lengthspring[:, 0], dtype=float)
            theory = (
                np.asarray(model.tendon_stiffness) * (lengths - free_lengths)
                + np.asarray(model.tendon_damping) * velocities
            )
            measured = theory.copy()
            joint_damping_force = -np.asarray(model.dof_damping) * np.asarray(data.qvel)
            actual_generalized = (
                np.asarray(data.qfrc_spring)
                + np.asarray(data.qfrc_damper)
                - joint_damping_force
            )

        theory_error = float(np.max(np.abs(measured - theory)))
        expected_generalized = -jacobian.T @ measured
        force_mapping_residual = float(np.linalg.norm(actual_generalized - expected_generalized))
        maximum_theory_error = max(maximum_theory_error, theory_error)
        maximum_force_mapping_residual = max(maximum_force_mapping_residual, force_mapping_residual)
        peak_tensions = np.maximum(peak_tensions, measured)

        if step % sample_every != 0 and step != total_steps - 1:
            continue
        nodes = np.asarray(data.site_xpos, dtype=float)[site_ids].copy()
        shape_change = float(np.max(np.abs(_pairwise(nodes) - baseline_pairwise)))
        records.append(
            {
                "time_s": float((step + 1) * model.opt.timestep),
                "phase": _phase(step, ramp_steps, hold_steps),
                "contraction_m": command,
                "shape_change_m": shape_change,
                "maximum_cable_length_change_m": float(np.max(np.abs(lengths - baseline_lengths))),
                "cross_00_tension_n": float(measured[controlled_id]),
                "minimum_tension_n": float(np.min(measured)),
                "maximum_tension_n": float(np.max(measured)),
                "contact_count": int(data.ncon),
                "minimum_node_height_m": float(np.min(nodes[:, 2])),
                "theory_error_n": theory_error,
                "force_mapping_residual_n": force_mapping_residual,
            }
        )
        node_samples.append(nodes)
        tension_samples.append(measured)

    final_tensions = np.asarray(tension_samples[-1], dtype=float)
    return {
        "records": records,
        "nodes": node_samples,
        "tensions": tension_samples,
        "initial_tensions": np.asarray(initial_tensions, dtype=float),
        "settled_tensions": np.asarray(baseline_tensions, dtype=float),
        "peak_tensions": peak_tensions,
        "final_tensions": final_tensions,
        "initial_lengths": initial_lengths,
        "maximum_theory_error_n": maximum_theory_error,
        "maximum_force_mapping_residual_n": maximum_force_mapping_residual,
    }


def _self_stress(model: mujoco.MjModel) -> dict[str, object]:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name) for name in NODE_NAMES}
    points = np.asarray([data.site_xpos[ids[name]] for name in NODE_NAMES], dtype=float)
    members: list[tuple[int, int, str]] = []
    for index in range(3):
        members.append((index, (index + 1) % 3, "ring"))
    for index in range(3):
        members.append((3 + index, 3 + (index + 1) % 3, "ring"))
    for index in range(3):
        members.append((index, 3 + index, "cross"))
    for index in range(3):
        members.append((index, 3 + (index + 1) % 3, "strut"))
    equilibrium = np.zeros((18, 12), dtype=float)
    for member_id, (start, end, _) in enumerate(members):
        direction = points[end] - points[start]
        direction /= np.linalg.norm(direction)
        equilibrium[3 * start : 3 * start + 3, member_id] = direction
        equilibrium[3 * end : 3 * end + 3, member_id] = -direction
    _, singular_values, right_vectors = np.linalg.svd(equilibrium)
    stress = right_vectors[-1]
    if np.mean(stress[:9]) < 0:
        stress = -stress
    stress *= 3.0 / float(np.mean(stress[6:9]))
    return {
        "ring_tension_n": float(np.mean(stress[:6])),
        "cross_tension_n": float(np.mean(stress[6:9])),
        "strut_compression_n": float(-np.mean(stress[9:12])),
        "ring_to_cross_ratio": float(np.mean(stress[:6]) / np.mean(stress[6:9])),
        "smallest_singular_value": float(singular_values[-1]),
        "node_equilibrium_residual_n": float(np.linalg.norm(equilibrium @ stress)),
    }


def _benchmark(
    kind: str,
    model: mujoco.MjModel,
    *,
    steps: int,
    repeats: int,
    native_rest: np.ndarray | None,
) -> dict[str, object]:
    samples = []
    controlled_id = CABLE_NAMES.index("cross_00")
    for _ in range(repeats):
        data = mujoco.MjData(model)
        if native_rest is not None:
            model.tendon_lengthspring[:, 0] = native_rest - np.eye(model.ntendon)[controlled_id] * 0.03
            model.tendon_lengthspring[:, 1] = model.tendon_lengthspring[:, 0]
        else:
            data.ctrl[controlled_id] = 0.03
        for _ in range(1000):
            mujoco.mj_step(model, data)
        start = time.perf_counter()
        for _ in range(steps):
            if native_rest is None:
                data.ctrl[controlled_id] = 0.03
            mujoco.mj_step(model, data)
        samples.append((time.perf_counter() - start) * 1e6 / steps)
    return {
        "model": kind,
        "steps_per_repeat": steps,
        "repeats": repeats,
        "microseconds_per_step": float(statistics.median(samples)),
        "samples_microseconds_per_step": samples,
    }


def _slack_benchmark(plugin_path: Path) -> list[dict[str, object]]:
    native_xml = """
<mujoco model="native_slack"><option gravity="0 0 0"/><worldbody>
  <site name="anchor" pos="0 0 0"/>
  <body pos="0.5 0 0"><joint name="slide" type="slide" axis="1 0 0"/>
    <geom type="sphere" size="0.01" mass="0.1" contype="0" conaffinity="0"/>
    <site name="moving"/>
  </body></worldbody><tendon>
  <spatial name="cable" stiffness="100" damping="0" springlength="0.6">
    <site site="anchor"/><site site="moving"/>
  </spatial></tendon></mujoco>"""
    plugin_xml = """
<mujoco model="plugin_slack"><compiler autolimits="true"/><option gravity="0 0 0"/>
<extension><plugin plugin="mujoco.cable.unilateral"><instance name="cable_plugin">
  <config key="stiffness" value="100"/><config key="damping" value="0"/>
  <config key="slack" value="0"/><config key="home_length" value="0.6"/>
  <config key="pretension_offset" value="0"/><config key="max_tension" value="100"/>
</instance></plugin></extension><worldbody><site name="anchor" pos="0 0 0"/>
  <body pos="0.5 0 0"><joint name="slide" type="slide" axis="1 0 0"/>
    <geom type="sphere" size="0.01" mass="0.1" contype="0" conaffinity="0"/>
    <site name="moving"/>
  </body></worldbody><tendon><spatial name="cable"><site site="anchor"/><site site="moving"/></spatial></tendon>
<actuator><plugin name="cable_act" tendon="cable" instance="cable_plugin" ctrlrange="0 0.1"/></actuator>
</mujoco>"""
    mujoco.mj_loadPluginLibrary(str(plugin_path.resolve()))
    models = {
        "MuJoCo native": mujoco.MjModel.from_xml_string(native_xml),
        "Cable plugin": mujoco.MjModel.from_xml_string(plugin_xml),
    }
    rows = []
    for model_name, model in models.items():
        for state, qpos in (("shorter_than_rest", 0.0), ("longer_than_rest", 0.2)):
            data = mujoco.MjData(model)
            data.qpos[0] = qpos
            mujoco.mj_forward(model, data)
            length = float(data.ten_length[0])
            raw_tension = 100.0 * (length - 0.6)
            if model_name == "Cable plugin":
                applied_tension = float(-data.actuator_force[0])
                generalized_force = float(data.qfrc_actuator[0])
            else:
                applied_tension = raw_tension
                generalized_force = float(data.qfrc_spring[0])
            rows.append(
                {
                    "model": model_name,
                    "state": state,
                    "length_m": length,
                    "rest_length_m": 0.6,
                    "raw_linear_tension_n": raw_tension,
                    "applied_tension_n": applied_tension,
                    "generalized_force_n": generalized_force,
                }
            )
    return rows


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def _relative_source_path(out_dir: Path, filename: str) -> str:
    path = (out_dir / filename).resolve()
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return filename


def _materialize_report_snapshot(
    out_dir: Path,
    datasets: dict[str, list[dict[str, object]]],
) -> dict[str, list[dict[str, object]]]:
    """Round-trip report rows through the SQLite source declared in artifact.json."""
    database_path = out_dir / "report_snapshot.sqlite"
    database_path.unlink(missing_ok=True)
    with sqlite3.connect(database_path) as connection:
        connection.execute(
            "CREATE TABLE report_snapshot "
            "(dataset TEXT NOT NULL, row_index INTEGER NOT NULL, payload_json TEXT NOT NULL, "
            "PRIMARY KEY (dataset, row_index))"
        )
        connection.executemany(
            "INSERT INTO report_snapshot (dataset, row_index, payload_json) VALUES (?, ?, ?)",
            (
                (dataset, row_index, json.dumps(row, ensure_ascii=False, separators=(",", ":")))
                for dataset, rows in datasets.items()
                for row_index, row in enumerate(rows)
            ),
        )
        selected_rows = connection.execute(REPORT_SOURCE_SQL).fetchall()

    selected: dict[str, list[dict[str, object]]] = {name: [] for name in datasets}
    for dataset, row_index, payload_json in selected_rows:
        if row_index != len(selected[dataset]):
            raise RuntimeError(f"non-contiguous report row index for {dataset}: {row_index}")
        selected[dataset].append(json.loads(payload_json))
    if {name: len(rows) for name, rows in selected.items()} != {
        name: len(rows) for name, rows in datasets.items()
    }:
        raise RuntimeError("SQLite report snapshot row count changed during materialization")
    return selected


def _build_artifact(
    out_dir: Path,
    summary: dict[str, object],
    merged_rows: list[dict[str, object]],
    tension_rows: list[dict[str, object]],
    benchmark_rows: list[dict[str, object]],
    slack_rows: list[dict[str, object]],
) -> dict[str, object]:
    generated_at = datetime.now(timezone.utc).isoformat()
    comparison = summary["comparison"]
    plugin = summary["plugin"]
    native = summary["native"]
    self_stress = summary["self_stress"]
    runtime_ratio = summary["performance"]["plugin_to_native_ratio"]
    slack_push = abs(
        next(row["applied_tension_n"] for row in slack_rows if row["model"] == "MuJoCo native" and row["state"] == "shorter_than_rest")
    )
    benchmark_snapshot = [
        {
            "model": row["model"],
            "steps_per_repeat": row["steps_per_repeat"],
            "repeats": row["repeats"],
            "microseconds_per_step": row["microseconds_per_step"],
        }
        for row in benchmark_rows
    ]
    kpis = [{
        "max_cross_tension_difference_n": comparison["maximum_cross_tension_difference_n"],
        "max_shape_difference_mm": 1000.0 * comparison["maximum_shape_difference_m"],
        "plugin_constitutive_error_n": plugin["maximum_theory_error_n"],
        "runtime_ratio": runtime_ratio,
        "native_compressive_force_n": slack_push,
    }]

    trajectory_long = []
    error_rows = []
    for row in merged_rows:
        for model_name, prefix in (("MuJoCo native", "native"), ("Cable plugin", "plugin")):
            trajectory_long.append({
                "time_s": row["time_s"],
                "phase": row["phase"],
                "model": model_name,
                "contraction_mm": 1000.0 * row["contraction_m"],
                "cross_tension_n": row[f"{prefix}_cross_00_tension_n"],
                "shape_change_mm": 1000.0 * row[f"{prefix}_shape_change_m"],
                "contact_count": row[f"{prefix}_contact_count"],
            })
        error_rows.append({
            "time_s": row["time_s"],
            "tension_difference_n": row["cross_tension_difference_n"],
            "node_rmse_mm": 1000.0 * row["node_rmse_m"],
        })

    initial_rows = []
    for group, theory_value, plugin_value, native_value in (
        ("Ring cables", self_stress["ring_tension_n"], summary["plugin"]["initial_ring_tension_n"], summary["native"]["initial_ring_tension_n"]),
        ("Cross cables", self_stress["cross_tension_n"], summary["plugin"]["initial_cross_tension_n"], summary["native"]["initial_cross_tension_n"]),
    ):
        initial_rows.extend([
            {"group": group, "method": "Theory", "tension_n": theory_value},
            {"group": group, "method": "Cable plugin", "tension_n": plugin_value},
            {"group": group, "method": "MuJoCo native", "tension_n": native_value},
        ])

    snapshot_datasets = _materialize_report_snapshot(out_dir, {
        "kpis": kpis,
        "trajectory_long": trajectory_long,
        "error_series": error_rows,
        "initial_tension": initial_rows,
        "slack_benchmark": slack_rows,
        "benchmark": benchmark_snapshot,
        "cable_summary": tension_rows,
    })
    manifest_sources = [
        {"id": "comparison_data", "label": "Reproducible report snapshot", "path": _relative_source_path(out_dir, "report_snapshot.sqlite")},
        {"id": "plugin_model", "label": "Unilateral cable plugin model", "path": "cable_plugin_demos/17_cpp_plugin_three_strut_nine_cable.xml"},
        {"id": "native_model", "label": "Native tendon baseline model", "path": "cable_plugin_demos/18_native_tendon_three_strut_nine_cable.xml"},
        {"id": "mujoco_tendon_docs", "label": "MuJoCo tendon and actuator documentation", "href": "https://mujoco.readthedocs.io/en/stable/overview.html"},
    ]

    manifest = {
        "version": 1,
        "surface": "report",
        "title": "MuJoCo 原生 Tendon 与 Unilateral Rope 插件对比",
        "description": "三杆九索同构仿真、理论张力、自应力、松弛行为和执行性能的可复现实证报告。",
        "generatedAt": generated_at,
        "cards": [
            {"id": "trajectory_card", "description": "同一 60 mm 收缩周期内两模型的最大跨索张力差。", "dataset": "kpis", "sourceId": "comparison_data", "metrics": [{"label": "最大张力差", "field": "max_cross_tension_difference_n", "format": "number", "unit": "N"}]},
            {"id": "shape_card", "description": "六个节点世界坐标轨迹的最大 RMS 差。", "dataset": "kpis", "sourceId": "comparison_data", "metrics": [{"label": "最大形状轨迹差", "field": "max_shape_difference_mm", "format": "number", "unit": "mm"}]},
            {"id": "law_card", "description": "插件传感张力与其单向本构方程的最大误差。", "dataset": "kpis", "sourceId": "comparison_data", "metrics": [{"label": "插件本构误差", "field": "plugin_constitutive_error_n", "format": "number", "unit": "N"}]},
            {"id": "runtime_card", "description": "相同模型步数下插件与原生 tendon 的中位耗时比。", "dataset": "kpis", "sourceId": "comparison_data", "metrics": [{"label": "插件/原生耗时比", "field": "runtime_ratio", "format": "number"}]},
            {"id": "slack_card", "description": "原生线性 tendon spring 在长度短于 rest length 0.1 m 时产生的压缩标量力；插件为零。", "dataset": "kpis", "sourceId": "comparison_data", "metrics": [{"label": "原生受压推力", "field": "native_compressive_force_n", "format": "number", "unit": "N"}]},
        ],
        "charts": [
            {"id": "tension_chart", "title": "跨层索 cross_00 张力", "subtitle": "同一 0→60→0 mm rest-length 收缩周期；单位 N。", "type": "line", "dataset": "trajectory_long", "sourceId": "comparison_data", "encodings": {"x": {"field": "time_s", "type": "quantitative", "label": "周期时间 (s)"}, "y": {"field": "cross_tension_n", "type": "quantitative", "label": "张力 (N)"}, "color": {"field": "model", "type": "nominal", "label": "模型"}}},
            {"id": "shape_chart", "title": "结构形状变化", "subtitle": "相对各自落地稳态的最大节点对距离变化；单位 mm。", "type": "line", "dataset": "trajectory_long", "sourceId": "comparison_data", "encodings": {"x": {"field": "time_s", "type": "quantitative", "label": "周期时间 (s)"}, "y": {"field": "shape_change_mm", "type": "quantitative", "label": "形状变化 (mm)"}, "color": {"field": "model", "type": "nominal", "label": "模型"}}},
            {"id": "error_chart", "title": "两模型动态差异", "subtitle": "cross_00 张力差；同一路径和绷紧区间内应接近数值精度。", "type": "line", "dataset": "error_series", "sourceId": "comparison_data", "encodings": {"x": {"field": "time_s", "type": "quantitative", "label": "周期时间 (s)"}, "y": {"field": "tension_difference_n", "type": "quantitative", "label": "绝对张力差 (N)"}}},
            {"id": "initial_chart", "title": "初始自应力张力", "subtitle": "理论平衡解与两个仿真模型的初始值。", "type": "bar", "dataset": "initial_tension", "sourceId": "comparison_data", "encodings": {"x": {"field": "group", "type": "nominal", "label": "索组"}, "y": {"field": "tension_n", "type": "quantitative", "label": "张力 (N)"}, "color": {"field": "method", "type": "nominal", "label": "方法"}}},
            {"id": "slack_chart", "title": "松弛与受压基准", "subtitle": "正值表示受拉，负值表示原生线性 spring 进入压缩区。", "type": "bar", "dataset": "slack_benchmark", "sourceId": "comparison_data", "encodings": {"x": {"field": "state", "type": "nominal", "label": "长度状态"}, "y": {"field": "applied_tension_n", "type": "quantitative", "label": "标量索力 (N)"}, "color": {"field": "model", "type": "nominal", "label": "模型"}}},
            {"id": "runtime_chart", "title": "仿真步耗时", "subtitle": "相同三杆九索模型；Python 循环中位数，越低越快。", "type": "bar", "dataset": "benchmark", "sourceId": "comparison_data", "encodings": {"x": {"field": "model", "type": "nominal", "label": "模型"}, "y": {"field": "microseconds_per_step", "type": "quantitative", "label": "μs/step"}}},
        ],
        "tables": [
            {"id": "cable_table", "title": "九条索的张力审计", "subtitle": "理论初值、两个模型初值、落地稳态与周期峰值。", "dataset": "cable_summary", "sourceId": "comparison_data", "columns": [
                {"field": "cable", "label": "Cable", "type": "text"},
                {"field": "theory_initial_n", "label": "Theory (N)", "format": "number"},
                {"field": "plugin_initial_n", "label": "Plugin initial (N)", "format": "number"},
                {"field": "native_initial_n", "label": "Native initial (N)", "format": "number"},
                {"field": "plugin_settled_n", "label": "Plugin settled (N)", "format": "number"},
                {"field": "native_settled_n", "label": "Native settled (N)", "format": "number"},
                {"field": "plugin_peak_n", "label": "Plugin peak (N)", "format": "number"},
                {"field": "native_peak_n", "label": "Native peak (N)", "format": "number"},
            ]},
        ],
        "sources": manifest_sources,
        "blocks": [
            {"id": "title", "type": "markdown", "body": "# MuJoCo 原生 Tendon 与 Unilateral Rope 插件对比\n\n**结论先行：**在本三杆九索模型中，所有索始终绷紧且使用相同直线路径、刚度、阻尼和 rest length 命令时，插件与原生 tendon 应给出近乎相同的张力和运动。插件的物理增量不在这个绷紧线性区间，而在 **单向受拉、运行时自由绳长、张力传感、卷扬/Capstan 与 surface envelope**。", "sourceId": "comparison_data"},
            {"id": "metrics", "type": "metric-strip", "cardIds": ["trajectory_card", "shape_card", "law_card", "runtime_card", "slack_card"]},
            {"id": "finding_taut", "type": "markdown", "body": "## 绷紧直索区间，两种方法应当重合\n\n两者都使用 MuJoCo 编译出的 spatial-tendon 长度与 Jacobian，且本周期峰值低于 80 N 截止值。因而任何显著差异都意味着实现、采样阶段或参数没有真正对齐，而不是插件应该天然产生不同物理。", "sourceId": "comparison_data"},
            {"id": "tension", "type": "chart", "chartId": "tension_chart"},
            {"id": "shape", "type": "chart", "chartId": "shape_chart"},
            {"id": "error", "type": "chart", "chartId": "error_chart"},
            {"id": "theory_heading", "type": "markdown", "body": f"## 初始张力满足三棱柱自应力理论\n\n对 6 个节点、9 条索和 3 根杆建立 18×12 平衡矩阵。其最小奇异值为 `{self_stress['smallest_singular_value']:.3e}`，零空间给出 `T_ring/T_cross = {self_stress['ring_to_cross_ratio']:.5f}`。令跨层索为 3 N，则环索为 `{self_stress['ring_tension_n']:.5f} N`，杆轴向压力为 `{self_stress['strut_compression_n']:.5f} N`。落地后重力和地面反力加入平衡，因此稳态张力不再必须维持该纯自应力比例。", "sourceId": "comparison_data"},
            {"id": "initial", "type": "chart", "chartId": "initial_chart"},
            {"id": "cables", "type": "table", "tableId": "cable_table"},
            {"id": "unilateral_heading", "type": "markdown", "body": "## 松弛区才是 unilateral rope 的关键差异\n\nMuJoCo 原生 tendon `stiffness` 是关于 `springlength` 的线性 spring。当实际长度比 rest length 短时，线性模型进入负张力区并沿 tendon Jacobian 产生推力。插件使用 `max(0, ·)`，因此松绳不推压刚体。也可以用原生 position actuator 加单向 force clamp 人工构造类似行为，但原生 tendon spring 本身并不等价于单向绳。", "sourceId": "comparison_data"},
            {"id": "slack", "type": "chart", "chartId": "slack_chart"},
            {"id": "performance_heading", "type": "markdown", "body": "## 原生 tendon 是性能基线\n\n原生路径和 spring 在 MuJoCo 内核中直接求解；插件增加实例回调、张力传感与控制逻辑，因此小模型中通常更慢。性能差异不代表物理误差，只影响大规模模型和优化循环的吞吐量。", "sourceId": "comparison_data"},
            {"id": "runtime", "type": "chart", "chartId": "runtime_chart"},
            {"id": "scope", "type": "markdown", "body": "## 对比范围与方法\n\n- 两个模型均为 3 个 free body、9 条同名 spatial tendon、相同质量、接触、重力、`0.5 ms` 时间步和 `implicitfast`。\n- 原生模型用 `<spatial stiffness=\"1500\" damping=\"4\" springlength=\"…\">`；脚本通过更新 `mjModel.tendon_lengthspring` 施加同一 rest-length 命令。\n- 插件模型用 `home_length=auto_initial`、相同 pretension offset、`target_contraction` 和 `max_tension=80 N`。\n- 先落地 3 s，再对 `cross_00` 执行 1 s 上升、1 s 保持、1 s 回落和 3 s 恢复。\n- MuJoCo 文档说明 spatial tendon 是经过 site 或绕 sphere/cylinder 的最短路径，原生 tendon spring 关于 springlength 线性作用；actuator force 通过 transmission moment arm 映射到广义坐标：[Tendon overview](https://mujoco.readthedocs.io/en/stable/overview.html)、[XML reference](https://mujoco.readthedocs.io/en/latest/XMLreference.html)、[Computation](https://mujoco.readthedocs.io/en/3.3.5/computation/)。", "sourceId": "comparison_data"},
            {"id": "limits", "type": "markdown", "body": "## 真实性边界\n\n本报告验证的是 **数学和数值一致性**，不是实物标定。张力绝对值仍取决于 `k=1500 N/m`、`c=4 N·s/m`、预张量、端点位置和接触参数。两种模型都把绳视为无质量长度元件，不包含纵向波传播、真实截面压缩、弯曲/扭转、蠕变、磨损和完整 stick-slip 历史。插件 native route 在直索绷紧区不会比原生 tendon 更“真实”；它更可靠的地方是不会产生压缩推力，并提供运行时绳长和扩展路由能力。若要声称实验真实性，下一步必须用 load cell 标定刚度、阻尼、预张力和滑轮摩擦，并用独立运动捕捉验证构型。", "sourceId": "comparison_data"},
            {"id": "next", "type": "markdown", "body": "## 建议与进一步问题\n\n1. 对纯直线、始终绷紧且无需卷扬的索，优先使用原生 tendon：更简单、更快。\n2. 对会松弛、需要可变自由绳长、Capstan 或 mesh/cylinder surface envelope 的索，使用插件。\n3. 下一阶段加入实物拉伸试验和滑轮测力计数据，拟合非线性刚度、速度相关阻尼与摩擦参数。\n4. 进一步评估多索同时收缩、接触切换和更大时间步对两种实现差异的影响。", "sourceId": "comparison_data"},
        ],
    }
    return {
        "surface": "report",
        "manifest": manifest,
        "snapshot": {
            "version": 1,
            "generatedAt": generated_at,
            "status": "ready",
            "datasets": snapshot_datasets,
        },
        "sources": [{
            "id": "comparison_data",
            "query": {
                "engine": "sqlite",
                "sql": REPORT_SOURCE_SQL,
                "description": "Reads every ordered chart, KPI, and table row from the generated SQLite report snapshot.",
                "id": "native-vs-plugin-report-snapshot",
                "executed_at": generated_at,
            },
        }],
    }


def compare(
    plugin_path: Path,
    plugin_model_path: Path,
    native_model_path: Path,
    out_dir: Path,
    *,
    benchmark_steps: int = 20000,
    benchmark_repeats: int = 5,
) -> dict[str, object]:
    out_dir.mkdir(parents=True, exist_ok=True)
    mujoco.mj_loadPluginLibrary(str(plugin_path.resolve()))
    plugin_model = _load_matched_plugin_model(plugin_model_path.resolve())
    native_model = mujoco.MjModel.from_xml_path(str(native_model_path.resolve()))
    if (plugin_model.nq, plugin_model.nv, plugin_model.ntendon) != (native_model.nq, native_model.nv, native_model.ntendon):
        raise RuntimeError("Plugin and native models are not structurally aligned")

    plugin_params = _plugin_parameters(plugin_model_path)
    settle_steps = 6000
    ramp_steps = 2000
    hold_steps = 2000
    release_steps = 8000
    contraction_m = 0.06
    sample_every = 20
    plugin_run = _simulate(
        "plugin", plugin_model, settle_steps=settle_steps, ramp_steps=ramp_steps,
        hold_steps=hold_steps, release_steps=release_steps, contraction_m=contraction_m,
        sample_every=sample_every, plugin_params=plugin_params,
    )
    native_run = _simulate(
        "native", native_model, settle_steps=settle_steps, ramp_steps=ramp_steps,
        hold_steps=hold_steps, release_steps=release_steps, contraction_m=contraction_m,
        sample_every=sample_every, plugin_params=None,
    )

    merged_rows: list[dict[str, object]] = []
    maximum_node_rmse = 0.0
    maximum_cross_difference = 0.0
    maximum_shape_difference = 0.0
    for index, (plugin_record, native_record) in enumerate(zip(plugin_run["records"], native_run["records"], strict=True)):
        node_rmse = float(np.sqrt(np.mean((plugin_run["nodes"][index] - native_run["nodes"][index]) ** 2)))
        cross_difference = abs(float(plugin_record["cross_00_tension_n"]) - float(native_record["cross_00_tension_n"]))
        shape_difference = abs(float(plugin_record["shape_change_m"]) - float(native_record["shape_change_m"]))
        maximum_node_rmse = max(maximum_node_rmse, node_rmse)
        maximum_cross_difference = max(maximum_cross_difference, cross_difference)
        maximum_shape_difference = max(maximum_shape_difference, shape_difference)
        row = {
            "time_s": plugin_record["time_s"],
            "phase": plugin_record["phase"],
            "contraction_m": plugin_record["contraction_m"],
            "node_rmse_m": node_rmse,
            "cross_tension_difference_n": cross_difference,
            "shape_difference_m": shape_difference,
        }
        for prefix, record in (("plugin", plugin_record), ("native", native_record)):
            for key, value in record.items():
                if key not in ("time_s", "phase", "contraction_m"):
                    row[f"{prefix}_{key}"] = value
            for cable_id, cable_name in enumerate(CABLE_NAMES):
                row[f"{prefix}_{cable_name}_tension_n"] = float(
                    (plugin_run if prefix == "plugin" else native_run)["tensions"][index][cable_id]
                )
        merged_rows.append(row)

    self_stress = _self_stress(plugin_model)
    theory = np.asarray([self_stress["ring_tension_n"]] * 6 + [self_stress["cross_tension_n"]] * 3)
    tension_rows = []
    for cable_id, cable_name in enumerate(CABLE_NAMES):
        tension_rows.append({
            "cable": cable_name,
            "theory_initial_n": float(theory[cable_id]),
            "plugin_initial_n": float(plugin_run["initial_tensions"][cable_id]),
            "native_initial_n": float(native_run["initial_tensions"][cable_id]),
            "plugin_settled_n": float(plugin_run["settled_tensions"][cable_id]),
            "native_settled_n": float(native_run["settled_tensions"][cable_id]),
            "plugin_peak_n": float(plugin_run["peak_tensions"][cable_id]),
            "native_peak_n": float(native_run["peak_tensions"][cable_id]),
        })

    native_rest = np.asarray(native_model.tendon_lengthspring[:, 0], dtype=float).copy()
    benchmark_rows = [
        _benchmark("MuJoCo native", native_model, steps=benchmark_steps, repeats=benchmark_repeats, native_rest=native_rest),
        _benchmark("Cable plugin", plugin_model, steps=benchmark_steps, repeats=benchmark_repeats, native_rest=None),
    ]
    slack_rows = _slack_benchmark(plugin_path)
    native_time = benchmark_rows[0]["microseconds_per_step"]
    plugin_time = benchmark_rows[1]["microseconds_per_step"]

    summary = {
        "pass": bool(
            maximum_cross_difference < 0.05
            and maximum_shape_difference < 0.001
            and plugin_run["maximum_theory_error_n"] < 1e-9
            and plugin_run["maximum_force_mapping_residual_n"] < 1e-8
            and native_run["maximum_force_mapping_residual_n"] < 1e-8
        ),
        "models": {
            "plugin": str(plugin_model_path.relative_to(ROOT)),
            "native": str(native_model_path.relative_to(ROOT)),
            "timestep_s": float(plugin_model.opt.timestep),
            "settle_duration_s": settle_steps * float(plugin_model.opt.timestep),
            "contraction_m": contraction_m,
            "matched_control_override": "control smoothing disabled in memory for both-model equivalence",
        },
        "self_stress": self_stress,
        "comparison": {
            "maximum_cross_tension_difference_n": maximum_cross_difference,
            "maximum_shape_difference_m": maximum_shape_difference,
            "maximum_node_rmse_m": maximum_node_rmse,
        },
        "plugin": {
            "initial_ring_tension_n": float(np.mean(plugin_run["initial_tensions"][:6])),
            "initial_cross_tension_n": float(np.mean(plugin_run["initial_tensions"][6:])),
            "maximum_theory_error_n": plugin_run["maximum_theory_error_n"],
            "maximum_force_mapping_residual_n": plugin_run["maximum_force_mapping_residual_n"],
        },
        "native": {
            "initial_ring_tension_n": float(np.mean(native_run["initial_tensions"][:6])),
            "initial_cross_tension_n": float(np.mean(native_run["initial_tensions"][6:])),
            "maximum_theory_error_n": native_run["maximum_theory_error_n"],
            "maximum_force_mapping_residual_n": native_run["maximum_force_mapping_residual_n"],
        },
        "performance": {
            "rows": benchmark_rows,
            "plugin_to_native_ratio": float(plugin_time / native_time),
        },
        "slack_benchmark": slack_rows,
    }

    _write_csv(out_dir / "timeseries.csv", merged_rows)
    _write_csv(out_dir / "cable_tension_summary.csv", tension_rows)
    _write_csv(out_dir / "benchmark.csv", benchmark_rows)
    _write_csv(out_dir / "slack_benchmark.csv", slack_rows)
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    artifact = _build_artifact(out_dir, summary, merged_rows, tension_rows, benchmark_rows, slack_rows)
    (out_dir / "artifact.json").write_text(json.dumps(artifact, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare native MuJoCo tendon and unilateral cable plugin")
    parser.add_argument("--plugin", type=Path, default=DEFAULT_PLUGIN)
    parser.add_argument("--plugin-model", type=Path, default=DEFAULT_PLUGIN_MODEL)
    parser.add_argument("--native-model", type=Path, default=DEFAULT_NATIVE_MODEL)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--benchmark-steps", type=int, default=20000)
    parser.add_argument("--benchmark-repeats", type=int, default=5)
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    summary = compare(
        plugin_path=args.plugin,
        plugin_model_path=args.plugin_model,
        native_model_path=args.native_model,
        out_dir=args.out,
        benchmark_steps=args.benchmark_steps,
        benchmark_repeats=args.benchmark_repeats,
    )
    print(json.dumps(summary, indent=2))
    return 1 if args.strict and not summary["pass"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
