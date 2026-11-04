#!/usr/bin/env python3
"""Benchmark MuJoCo step time on the matched 18-joint SpiRobs models."""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
from pathlib import Path

import mujoco

import run_result2_experiments as result2


HERE = Path(__file__).resolve().parent
PLUGIN_MODEL = HERE / "result_2_2/models/result_2_2a_mujocable_mu_0p15.xml"
NATIVE_MODEL = HERE / "result_2_2/models/result_2_2b_native_tendon.xml"
RAW_OUTPUT = HERE / "result_2_2/data/result_2_2_runtime_samples.csv"
SUMMARY_OUTPUT = HERE / "result_2_2/data/result_2_2_runtime_summary.csv"


def write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", type=Path, required=True)
    parser.add_argument("--research-root", type=Path, default=HERE)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--repeats", type=int, default=7)
    args = parser.parse_args()

    if not args.plugin.is_file():
        parser.error(f"plugin not found: {args.plugin}")
    cycle_path = args.research_root.resolve() / "scripts"
    sys.path.insert(0, str(cycle_path))
    import run_tpu95hf_bidirectional_bending as cycle

    mujoco.set_mju_user_warning(lambda message: None)
    mujoco.mj_loadPluginLibrary(str(args.plugin.resolve()))
    result2.validate_matched_spirobot_models(PLUGIN_MODEL, NATIVE_MODEL)

    raw_rows: list[dict[str, object]] = []
    for repeat in range(args.repeats):
        order = ("MuJoCable", "Native tendon") if repeat % 2 == 0 else ("Native tendon", "MuJoCable")
        for model_name in order:
            if model_name == "MuJoCable":
                simulation = result2.simulate_plugin_spirobot(
                    cycle, PLUGIN_MODEL, model_name, args.duration, result2.SAMPLE_PERIOD
                )
            else:
                simulation = result2.simulate_native_spirobot(
                    NATIVE_MODEL, model_name, args.duration, result2.SAMPLE_PERIOD
                )
            for sample_index, row in enumerate(simulation.rows):
                raw_rows.append(
                    {
                        "model": model_name,
                        "repeat": repeat + 1,
                        "sample_index": sample_index,
                        "time_s": row["time_s"],
                        "step_time_us": 1000.0 * float(row["step_ms"]),
                    }
                )

    write_rows(RAW_OUTPUT, raw_rows)
    summary_rows: list[dict[str, object]] = []
    medians: dict[str, float] = {}
    for model_name in ("Native tendon", "MuJoCable"):
        samples = [float(row["step_time_us"]) for row in raw_rows if row["model"] == model_name]
        median = statistics.median(samples)
        medians[model_name] = median
        summary_rows.append(
            {
                "model": model_name,
                "repeats": args.repeats,
                "timed_samples": len(samples),
                "median_step_time_us": median,
                "mean_step_time_us": statistics.fmean(samples),
                "p95_step_time_us": sorted(samples)[int(0.95 * (len(samples) - 1))],
                "realtime_factor": 500.0 / median,
            }
        )
    for row in summary_rows:
        row["overhead_vs_native"] = float(row["median_step_time_us"]) / medians["Native tendon"]
    write_rows(SUMMARY_OUTPUT, summary_rows)

    for row in summary_rows:
        print(row)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
