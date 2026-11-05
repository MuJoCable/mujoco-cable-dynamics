#!/usr/bin/env python3
"""Plot the single-column SpiRobot comparison used in Result 2.

Core conclusion: similar robot-scale bending can conceal different cable
transmission states, and guide friction systematically changes those states.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib as mpl

mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FuncFormatter


HERE = Path(__file__).resolve().parent
FONT = Path("/System/Library/Fonts/Supplemental/Times New Roman.ttf")
COMPARISON_JSON = HERE / "result_2_2/data/result_2_2_summary.json"
FRICTION_JSON = HERE / "result_2_3/data/result_2_3_summary.json"
OUTPUT = HERE / "figures/figure5_result2_spirobot_single_column"
OUTPUT_PANEL_A = HERE / "figures/figure5a_result2_spirobot_native_comparison"
OUTPUT_PANEL_B = HERE / "figures/figure5b_result2_spirobot_friction_sweep"
SOURCE_DATA = HERE / "figure_data/figure5_result2_spirobot_single_column.csv"

# Google palette sampled from the supplied color card.
GOOGLE_BLUE = "#5383EC"
GOOGLE_BLUE_EDGE = "#4B76D5"
GOOGLE_GREEN = "#58A55D"
GOOGLE_GREEN_EDGE = "#4F9553"
GOOGLE_RED = "#D85040"
GOOGLE_DARK = "#293238"
GRID_COLOR = "#DFDFDF"


def configure_plotting() -> None:
    if FONT.exists():
        mpl.font_manager.fontManager.addfont(str(FONT))
    mpl.rcParams.update(
        {
            "font.family": "Times New Roman" if FONT.exists() else "serif",
            "font.serif": ["Times New Roman", "Times", "Helvetica"],
            "font.size": 10,
            "axes.labelsize": 10,
            "axes.titlesize": 10,
            "legend.fontsize": 10,
            "xtick.labelsize": 10,
            "ytick.labelsize": 10,
            "axes.linewidth": 0.8,
            "lines.linewidth": 1.5,
            "lines.markersize": 5.0,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "none",
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
        }
    )


def style_axis(axis: plt.Axes) -> None:
    axis.grid(axis="y", color=GRID_COLOR, linewidth=0.55, zorder=0)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)
    axis.tick_params(direction="out", length=3, width=0.7)


def load_json(path: Path) -> dict[str, dict[str, float]]:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def write_source_data(rows: list[dict[str, object]]) -> None:
    SOURCE_DATA.parent.mkdir(parents=True, exist_ok=True)
    with SOURCE_DATA.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def save_figure(figure: plt.Figure, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output.with_suffix(".pdf"), bbox_inches="tight")
    figure.savefig(output.with_suffix(".svg"), bbox_inches="tight")
    figure.savefig(output.with_suffix(".png"), dpi=600, bbox_inches="tight")
    figure.savefig(output.with_suffix(".tiff"), dpi=600, bbox_inches="tight")


def draw_native_comparison(
    axis: plt.Axes,
    metric_specs: list[tuple[str, str, str]],
    cable_ratios: np.ndarray,
) -> None:
    x = np.arange(len(metric_specs))
    width = 0.34
    axis.bar(
        x - width / 2,
        np.ones_like(x, dtype=float),
        width,
        color=GOOGLE_BLUE,
        edgecolor=GOOGLE_BLUE_EDGE,
        linewidth=0.6,
        label="Native tendon",
        zorder=2,
    )
    axis.bar(
        x + width / 2,
        cable_ratios,
        width,
        color=GOOGLE_GREEN,
        edgecolor=GOOGLE_GREEN_EDGE,
        linewidth=0.6,
        label="MuJoCable",
        zorder=3,
    )
    annotations = ("Nearly equal", "Nearly 4 times", "9% higher")
    for index, (ratio, annotation) in enumerate(zip(cable_ratios, annotations)):
        axis.text(
            index + width / 2,
            ratio + 0.08,
            annotation,
            ha="center",
            va="bottom",
            fontsize=10,
            color=GOOGLE_DARK,
        )
    axis.set_xticks(x, [label for label, _, _ in metric_specs])
    axis.set_ylabel("Ratio to native tendon")
    axis.set_ylim(0, 4.7)
    axis.legend(
        frameon=False,
        loc="lower center",
        bbox_to_anchor=(0.5, 1.005),
        ncol=2,
        columnspacing=0.8,
        handlelength=1.4,
        borderaxespad=0,
    )
    axis.text(-0.13, 1.03, "(c)", transform=axis.transAxes, fontweight="bold")
    style_axis(axis)


def draw_friction_sweep(
    axis: plt.Axes,
    mus: np.ndarray,
    friction_specs: list[tuple[str, str, str, str]],
    friction_ratios: dict[str, np.ndarray],
) -> None:
    friction_x = np.arange(len(mus))
    for label, _, color, marker in friction_specs:
        axis.plot(
            friction_x,
            friction_ratios[label],
            color=color,
            marker=marker,
            label=label,
            zorder=3,
        )
    axis.axhline(1.0, color="#888888", linestyle=":", linewidth=0.9, zorder=1)
    axis.set_yscale("log")
    axis.set_ylim(0.68, 4.8)
    axis.set_yticks([0.75, 1.0, 1.5, 2.0, 3.0, 4.0])
    axis.yaxis.set_major_formatter(FuncFormatter(lambda value, _: f"{value:g}"))
    axis.set_xticks(friction_x, [f"{mu:.2f}" if mu else "0" for mu in mus])
    axis.set_xlabel(r"Guide friction coefficient, $\mu$")
    axis.set_ylabel(r"Ratio to $\mu=0$")
    axis.legend(
        frameon=False,
        loc="lower center",
        bbox_to_anchor=(0.55, 1.01),
        ncol=2,
        columnspacing=0.8,
        handlelength=1.5,
        borderaxespad=0,
    )
    axis.text(-0.13, 1.03, "(d)", transform=axis.transAxes, fontweight="bold")
    style_axis(axis)


def main() -> None:
    configure_plotting()
    comparison = load_json(COMPARISON_JSON)
    friction = load_json(FRICTION_JSON)

    native = comparison["native_tendon_muscle"]
    cable = comparison["mujocable_mu_0.15"]
    metric_specs = [
        ("Steady bend", "hold_bend_deg", "deg"),
        ("Peak tension", "maximum_active_tension_N", "N"),
        ("Cable take-up", "maximum_active_cable_takeup_mm", "mm"),
    ]
    native_values = np.asarray([abs(float(native[key])) for _, key, _ in metric_specs])
    cable_values = np.asarray([abs(float(cable[key])) for _, key, _ in metric_specs])
    cable_ratios = cable_values / native_values

    friction_items = sorted(
        ((float(key.rsplit("_", 1)[-1]), value) for key, value in friction.items()),
        key=lambda item: item[0],
    )
    mus = np.asarray([item[0] for item in friction_items])
    friction_specs = [
        ("Bend magnitude", "hold_bend_deg", GOOGLE_BLUE, "o"),
        ("Peak tension", "maximum_active_tension_N", GOOGLE_RED, "s"),
        ("Cable take-up", "maximum_active_cable_takeup_mm", GOOGLE_GREEN, "^"),
    ]
    friction_ratios: dict[str, np.ndarray] = {}
    for label, key, _, _ in friction_specs:
        values = np.asarray([abs(float(item[1][key])) for item in friction_items])
        friction_ratios[label] = values / values[0]
    if np.any(np.concatenate(list(friction_ratios.values())) <= 0):
        raise ValueError("Log-scale ratios must be strictly positive")

    source_rows: list[dict[str, object]] = []
    for index, (label, _, unit) in enumerate(metric_specs):
        source_rows.extend(
            [
                {
                    "panel": "c",
                    "condition": "Native tendon",
                    "metric": label,
                    "value": native_values[index],
                    "unit": unit,
                    "ratio": 1.0,
                },
                {
                    "panel": "c",
                    "condition": "MuJoCable",
                    "metric": label,
                    "value": cable_values[index],
                    "unit": unit,
                    "ratio": cable_ratios[index],
                },
            ]
        )
    for mu_index, mu in enumerate(mus):
        for label, key, _, _ in friction_specs:
            source_rows.append(
                {
                    "panel": "d",
                    "condition": f"mu={mu:g}",
                    "metric": label,
                    "value": abs(float(friction_items[mu_index][1][key])),
                    "unit": "deg" if key == "hold_bend_deg" else ("N" if key.endswith("_N") else "mm"),
                    "ratio": friction_ratios[label][mu_index],
                }
            )
    write_source_data(source_rows)

    figure, axes = plt.subplots(2, 1, figsize=(3.5, 4.5), constrained_layout=True)
    draw_native_comparison(axes[0], metric_specs, cable_ratios)
    draw_friction_sweep(axes[1], mus, friction_specs, friction_ratios)
    save_figure(figure, OUTPUT)
    plt.close(figure)

    panel_a, axis_a = plt.subplots(figsize=(3.5, 2.15), constrained_layout=True)
    draw_native_comparison(axis_a, metric_specs, cable_ratios)
    save_figure(panel_a, OUTPUT_PANEL_A)
    plt.close(panel_a)

    panel_b, axis_b = plt.subplots(figsize=(3.5, 2.35), constrained_layout=True)
    draw_friction_sweep(axis_b, mus, friction_specs, friction_ratios)
    save_figure(panel_b, OUTPUT_PANEL_B)
    plt.close(panel_b)


if __name__ == "__main__":
    main()
