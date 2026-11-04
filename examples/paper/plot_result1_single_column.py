#!/usr/bin/env python3
"""Build the single-column dimensionless agreement plot for Result 1.

The script reads the frozen per-case time-series CSV files, exports a unified
raw-data table and the exact normalized plotting table, and then draws every
valid sample. Marker placement is staggered for readability; it does not
remove samples from the plotted lines or exported data.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
OUTPUT = HERE / "figures" / "figure3_result1_pulley_validation_single_column"
DATA_DIR = HERE / "figure_data"
EXPERIMENTAL_DATA = DATA_DIR / "figure3_result1_experimental_data.csv"
PLOT_DATA = DATA_DIR / "figure3_result1_plot_data.csv"

mpl.rcParams.update(
    {
        "font.family": ["Times New Roman", "Times", "Arial", "serif"],
        "font.size": 10,
        "axes.labelsize": 10,
        "axes.titlesize": 10,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "legend.fontsize": 10,
        "mathtext.fontset": "custom",
        "mathtext.rm": "Times New Roman",
        "mathtext.it": "Times New Roman:italic",
        "mathtext.bf": "Times New Roman:bold",
        "axes.spines.right": False,
        "axes.spines.top": False,
        "axes.linewidth": 0.8,
        "legend.frameon": False,
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
    }
)


# Google palette sampled from the supplied color card. Repeated colors group
# related pulley families, while marker shapes distinguish cases. All data
# traces remain solid to avoid redundant and visually noisy style changes.
GOOGLE_BLUE = "#5383EC"
GOOGLE_GREEN = "#58A55D"
GOOGLE_YELLOW = "#F2BF42"
GOOGLE_RED = "#D85040"
GOOGLE_DARK = "#293238"


@dataclass(frozen=True)
class Comparison:
    case_id: str
    case_name: str
    path: str
    theory_field: str
    simulation_field: str
    color: str
    marker: str
    linestyle: object
    marker_offset: int
    filter_kind: str = "all"
    scale_path: str | None = None
    scale_field: str | None = None


COMPARISONS = (
    Comparison(
        "FP0",
        "Fixed pulley, mu=0",
        "result_1_1/data/result_1_1_mu_0_timeseries.csv",
        "theory_payload_lift_m",
        "payload_lift_m",
        GOOGLE_BLUE,
        "o",
        "-",
        0,
    ),
    Comparison(
        "FP15",
        "Fixed pulley, mu=0.15",
        "result_1_1/data/result_1_1_mu_0p15_timeseries.csv",
        "theory_capstan_ratio",
        "measured_capstan_ratio",
        GOOGLE_BLUE,
        "s",
        "-",
        3,
    ),
    Comparison(
        "FS0",
        "Free sheave, mu=0",
        "result_1_1_b/data/result_1_1_b_mu_0_timeseries.csv",
        "theory_pulley_omega_abs_radps",
        "measured_pulley_omega_abs_radps",
        GOOGLE_GREEN,
        "x",
        "-",
        6,
        scale_path="result_1_1_b/data/result_1_1_b_mu_0p15_timeseries.csv",
        scale_field="theory_pulley_omega_abs_radps",
    ),
    Comparison(
        "FS15",
        "Free sheave, mu=0.15",
        "result_1_1_b/data/result_1_1_b_mu_0p15_timeseries.csv",
        "theory_capstan_torque_abs_Nm",
        "measured_cable_torque_abs_Nm",
        GOOGLE_GREEN,
        "^",
        "-",
        9,
    ),
    Comparison(
        "WS",
        "Winch-sheave",
        "result_1_1_c/data/result_1_1_c_winch_timeseries.csv",
        "winch_rim_travel_m",
        "pulley_rim_travel_m",
        GOOGLE_YELLOW,
        "D",
        "-",
        12,
    ),
    Comparison(
        "MP",
        "Moving pulley",
        "result_1_2/data/result_1_2_moving_pulley_timeseries.csv",
        "theory_lift_m",
        "moving_load_lift_m",
        GOOGLE_RED,
        "v",
        "-",
        15,
    ),
    Comparison(
        "AT",
        "Atwood, pre-impact",
        "result_1_3/data/result_1_3_atwood_landing_timeseries.csv",
        "theory_speed_mps",
        "right_speed_down_mps",
        GOOGLE_RED,
        "P",
        "-",
        18,
        filter_kind="before_contact",
    ),
)


def read_rows(relative_path: str) -> list[dict[str, str]]:
    with (HERE / relative_path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def finite(value: float) -> bool:
    return math.isfinite(value)


def first_contact_time(rows: list[dict[str, str]]) -> float:
    contacts = [float(row["time_s"]) for row in rows if float(row["right_contact"]) > 0.5]
    if not contacts:
        raise ValueError("Atwood source data contain no contact sample")
    return min(contacts)


def included(comparison: Comparison, row: dict[str, str], contact_time: float | None) -> tuple[bool, str]:
    theory = float(row[comparison.theory_field])
    simulation = float(row[comparison.simulation_field])
    if not finite(theory) or not finite(simulation):
        return False, "non-finite theory or simulation value"
    if comparison.filter_kind == "before_contact" and float(row["time_s"]) >= float(contact_time):
        return False, "at or after first hardware-contact event in simulation"
    return True, ""


def normalization_scale(comparison: Comparison, cache: dict[str, list[dict[str, str]]]) -> tuple[float, str]:
    if comparison.scale_path and comparison.scale_field:
        values = np.asarray(
            [float(row[comparison.scale_field]) for row in cache[comparison.scale_path]],
            dtype=float,
        )
        scale = float(np.nanmax(np.abs(values)))
        rule = f"max_abs({comparison.scale_field}) from matched FS15 reference"
    else:
        rows = cache[comparison.path]
        contact_time = first_contact_time(rows) if comparison.filter_kind == "before_contact" else None
        values = [
            abs(float(row[comparison.theory_field]))
            for row in rows
            if included(comparison, row, contact_time)[0]
        ]
        scale = float(max(values))
        rule = f"max_abs({comparison.theory_field}) over included interval"
    if not finite(scale) or scale <= 0:
        raise ValueError(f"Non-positive normalization scale for {comparison.case_id}")
    return scale, rule


def build_data_tables() -> dict[str, list[dict[str, object]]]:
    source_paths = {comparison.path for comparison in COMPARISONS}
    source_paths.update(
        comparison.scale_path for comparison in COMPARISONS if comparison.scale_path is not None
    )
    cache = {path: read_rows(path) for path in source_paths}

    raw_rows: list[dict[str, object]] = []
    plot_rows: list[dict[str, object]] = []
    by_case: dict[str, list[dict[str, object]]] = {}

    for comparison in COMPARISONS:
        rows = cache[comparison.path]
        contact_time = first_contact_time(rows) if comparison.filter_kind == "before_contact" else None
        scale, scale_rule = normalization_scale(comparison, cache)
        current: list[dict[str, object]] = []

        for sample_index, row in enumerate(rows):
            theory = float(row[comparison.theory_field])
            simulation = float(row[comparison.simulation_field])
            use, reason = included(comparison, row, contact_time)
            raw_rows.append(
                {
                    "case_id": comparison.case_id,
                    "case_name": comparison.case_name,
                    "source_csv": comparison.path,
                    "sample_index": sample_index,
                    "time_s": float(row["time_s"]),
                    "theory_field": comparison.theory_field,
                    "simulation_field": comparison.simulation_field,
                    "theory_raw": theory,
                    "simulation_raw": simulation,
                    "included_in_plot": int(use),
                    "exclusion_reason": reason,
                    "normalization_scale": scale,
                    "normalization_rule": scale_rule,
                }
            )
            if not use:
                continue
            plotting_row = {
                "case_id": comparison.case_id,
                "case_name": comparison.case_name,
                "sample_index": sample_index,
                "time_s": float(row["time_s"]),
                "theory_raw": theory,
                "simulation_raw": simulation,
                "normalization_scale": scale,
                "theory_normalized": theory / scale,
                "simulation_normalized": simulation / scale,
                "line_style": str(comparison.linestyle),
                "marker": comparison.marker,
            }
            plot_rows.append(plotting_row)
            current.append(plotting_row)
        by_case[comparison.case_id] = current

    DATA_DIR.mkdir(parents=True, exist_ok=True)
    with EXPERIMENTAL_DATA.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(raw_rows[0]))
        writer.writeheader()
        writer.writerows(raw_rows)
    with PLOT_DATA.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(plot_rows[0]))
        writer.writeheader()
        writer.writerows(plot_rows)
    return by_case


def make_figure() -> None:
    by_case = build_data_tables()
    figure, axis = plt.subplots(figsize=(3.50, 3.75))
    figure.subplots_adjust(left=0.21, right=0.98, top=0.97, bottom=0.31)

    axis.plot(
        [0, 1.08],
        [0, 1.08],
        color=GOOGLE_DARK,
        linestyle=(0, (4, 3)),
        linewidth=1.15,
        zorder=1,
    )

    for index, comparison in enumerate(COMPARISONS):
        rows = by_case[comparison.case_id]
        theory = np.asarray([float(row["theory_normalized"]) for row in rows])
        simulation = np.asarray([float(row["simulation_normalized"]) for row in rows])
        marker_interval = max(1, len(rows) // 12)
        marker_offset = comparison.marker_offset % marker_interval
        line_width = 1.65 if comparison.case_id == "AT" else 1.35
        axis.plot(
            theory,
            simulation,
            color=comparison.color,
            linestyle=comparison.linestyle,
            linewidth=line_width,
            alpha=0.92,
            marker=comparison.marker,
            markersize=5.0,
            markevery=(marker_offset, marker_interval),
            markerfacecolor="white" if comparison.marker != "x" else comparison.color,
            markeredgecolor=comparison.color,
            markeredgewidth=1.0,
            label=comparison.case_id,
            zorder=3 + index,
        )

    axis.set_xlim(-0.035, 1.08)
    axis.set_ylim(-0.035, 1.08)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel(r"Normalized theory, $y_{\mathrm{th}}/s$")
    axis.set_ylabel(r"Normalized simulation, $y_{\mathrm{sim}}/s$")
    axis.set_xticks((0.0, 0.5, 1.0))
    axis.set_yticks((0.0, 0.5, 1.0))
    axis.text(0.045, 0.945, r"Ideal: $y_{\mathrm{sim}}=y_{\mathrm{th}}$", color=GOOGLE_DARK, transform=axis.transAxes)
    axis.legend(
        title="Case ID",
        loc="upper center",
        bbox_to_anchor=(0.5, -0.22),
        ncol=4,
        columnspacing=0.9,
        handlelength=2.3,
        handletextpad=0.45,
        borderaxespad=0,
        fontsize=10,
        title_fontsize=10,
    )

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(OUTPUT.with_suffix(".svg"), bbox_inches="tight")
    figure.savefig(OUTPUT.with_suffix(".pdf"), bbox_inches="tight")
    figure.savefig(OUTPUT.with_suffix(".png"), dpi=600, bbox_inches="tight")
    figure.savefig(OUTPUT.with_suffix(".tiff"), dpi=600, bbox_inches="tight")
    plt.close(figure)


if __name__ == "__main__":
    make_figure()
