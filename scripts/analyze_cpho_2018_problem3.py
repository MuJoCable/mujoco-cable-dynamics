#!/usr/bin/env python3
"""Reproduce CPhO 2018 final theory problem 3 in MuJoCo.

The official problem describes a massive, inextensible rope that is picked up
from one floor pile, slides over a prescribed-speed rough pulley, and is
deposited into the other pile. MuJoCable's current cable law is massless, so
this benchmark integrates the official continuum equation through a unit-mass
MuJoCo transport coordinate instead of claiming plugin support for rope mass.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "cable_plugin_demos/30_cpho_2018_problem3_massive_rope.xml"
DEFAULT_OUTPUT = ROOT / "docs/results/demo30_cpho_2018_problem3"


@dataclass(frozen=True)
class ProblemParameters:
    linear_density: float
    radius: float
    height: float
    friction: float
    gravity: float


@dataclass(frozen=True)
class OfficialCoefficients:
    exponential: float
    drive: float
    effective_length: float
    sliding_speed_limit: float
    time_scale: float


def name2id(model: mujoco.MjModel, objtype: int, name: str) -> int:
    obj_id = mujoco.mj_name2id(model, objtype, name)
    if obj_id < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return obj_id


def numeric(model: mujoco.MjModel, name: str) -> float:
    numeric_id = name2id(model, mujoco.mjtObj.mjOBJ_NUMERIC, name)
    address = int(model.numeric_adr[numeric_id])
    size = int(model.numeric_size[numeric_id])
    if size != 1:
        raise ValueError(f"custom numeric {name!r} has size {size}, expected 1")
    return float(model.numeric_data[address])


def load_parameters(model: mujoco.MjModel) -> ProblemParameters:
    return ProblemParameters(
        linear_density=numeric(model, "rope_linear_density"),
        radius=numeric(model, "pulley_radius"),
        height=numeric(model, "axis_height"),
        friction=numeric(model, "kinetic_friction"),
        gravity=numeric(model, "gravity_acceleration"),
    )


def official_coefficients(parameters: ProblemParameters) -> OfficialCoefficients:
    mu = parameters.friction
    if mu <= 0:
        raise ValueError("the official closed form used here requires mu > 0")
    radius = parameters.radius
    height = parameters.height
    gravity = parameters.gravity
    exponential = math.exp(mu * math.pi)
    drive = (
        height * gravity * (exponential - 1.0)
        + radius
        * gravity
        * (2.0 * mu / (1.0 + mu * mu))
        * (exponential + 1.0)
    )
    effective_length = (
        height * (exponential + 1.0)
        + radius / mu * (exponential - 1.0)
    )
    sliding_speed_limit = math.sqrt(drive / exponential)
    time_scale = effective_length / math.sqrt(drive * exponential)
    return OfficialCoefficients(
        exponential=exponential,
        drive=drive,
        effective_length=effective_length,
        sliding_speed_limit=sliding_speed_limit,
        time_scale=time_scale,
    )


def official_sliding_acceleration(
    speed: float, coefficients: OfficialCoefficients
) -> float:
    return (
        coefficients.drive - coefficients.exponential * speed * speed
    ) / coefficients.effective_length


def official_transition_time(
    rim_speed: float, coefficients: OfficialCoefficients
) -> float | None:
    if rim_speed >= coefficients.sliding_speed_limit:
        return None
    ratio = rim_speed / coefficients.sliding_speed_limit
    return coefficients.time_scale * math.atanh(ratio)


def official_speed(
    time: float, rim_speed: float, coefficients: OfficialCoefficients
) -> float:
    sliding = coefficients.sliding_speed_limit * math.tanh(
        time / coefficients.time_scale
    )
    return min(sliding, rim_speed)


def segment_tensions(
    speed: float,
    acceleration: float,
    parameters: ProblemParameters,
) -> tuple[float, float]:
    line_mass = parameters.linear_density * parameters.height
    left = line_mass * (parameters.gravity - acceleration)
    right = line_mass * (parameters.gravity + acceleration) + (
        parameters.linear_density * speed * speed
    )
    return left, right


def kinetic_wrap_residual(
    speed: float,
    acceleration: float,
    left_tension: float,
    right_tension: float,
    parameters: ProblemParameters,
    coefficients: OfficialCoefficients,
) -> float:
    mu = parameters.friction
    density = parameters.linear_density
    radius = parameters.radius
    gravity = parameters.gravity
    exponential = coefficients.exponential
    lhs = right_tension - left_tension * exponential
    rhs = (
        density
        * radius
        * gravity
        * (2.0 * mu / (1.0 + mu * mu))
        * (exponential + 1.0)
        - density
        * radius
        / mu
        * (exponential - 1.0)
        * (acceleration + mu * speed * speed / radius)
    )
    return lhs - rhs


def simulate_case(
    model_path: Path,
    case_name: str,
    omega: float,
    duration: float,
    sample_stride: int,
) -> tuple[list[dict[str, float | str]], dict[str, Any]]:
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    data = mujoco.MjData(model)
    parameters = load_parameters(model)
    coefficients = official_coefficients(parameters)

    rope_joint = name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "rope_transport"
    )
    pulley_joint = name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "pulley_hinge")
    rope_dof = int(model.jnt_dofadr[rope_joint])
    pulley_dof = int(model.jnt_dofadr[pulley_joint])
    rope_qpos = int(model.jnt_qposadr[rope_joint])
    rim_speed = parameters.radius * omega
    predicted_maximum = min(rim_speed, coefficients.sliding_speed_limit)
    predicted_transition = official_transition_time(rim_speed, coefficients)

    mujoco.mj_forward(model, data)
    mass_matrix = np.zeros((model.nv, model.nv), dtype=float)
    mujoco.mj_fullM(model, mass_matrix, data.qM)
    transport_mass = float(mass_matrix[rope_dof, rope_dof])
    if not math.isclose(transport_mass, 1.0, rel_tol=0, abs_tol=1e-12):
        raise ValueError(
            f"rope transport generalized mass is {transport_mass}, expected 1"
        )

    data.qvel[pulley_dof] = -omega
    timestep = float(model.opt.timestep)
    steps = int(math.ceil(duration / timestep))
    rows: list[dict[str, float | str]] = []
    detected_transition: float | None = None

    for step in range(steps + 1):
        time = float(data.time)
        speed = max(0.0, float(data.qvel[rope_dof]))
        sticking = rim_speed < coefficients.sliding_speed_limit and (
            speed >= rim_speed - 1e-10
        )
        if sticking:
            speed = rim_speed
            data.qvel[rope_dof] = speed
            acceleration = 0.0
            regime = "stick"
            if detected_transition is None:
                detected_transition = time
        else:
            acceleration = official_sliding_acceleration(speed, coefficients)
            regime = "sliding"

        left_tension, right_tension = segment_tensions(
            speed, acceleration, parameters
        )
        analytic_speed = official_speed(time, rim_speed, coefficients)
        equation_11_residual = (
            coefficients.drive
            - coefficients.exponential * speed * speed
            - coefficients.effective_length * acceleration
        )
        wrap_residual = (
            kinetic_wrap_residual(
                speed,
                acceleration,
                left_tension,
                right_tension,
                parameters,
                coefficients,
            )
            if regime == "sliding"
            else float("nan")
        )
        tension_ratio = right_tension / max(left_tension, 1e-15)

        if step % sample_stride == 0 or step == steps:
            rows.append(
                {
                    "case": case_name,
                    "time_s": time,
                    "rope_speed_m_s": speed,
                    "analytic_speed_m_s": analytic_speed,
                    "speed_error_m_s": speed - analytic_speed,
                    "acceleration_m_s2": acceleration,
                    "pulley_rim_speed_m_s": rim_speed,
                    "left_tension_N": left_tension,
                    "right_tension_N": right_tension,
                    "tension_ratio": tension_ratio,
                    "capstan_static_upper_ratio": coefficients.exponential,
                    "transport_distance_m": float(data.qpos[rope_qpos]),
                    "equation_11_residual_m2_s2": equation_11_residual,
                    "kinetic_wrap_residual_N": wrap_residual,
                    "regime": regime,
                }
            )

        if step == steps:
            break
        data.qfrc_applied[:] = 0.0
        data.qfrc_applied[rope_dof] = transport_mass * acceleration
        data.qvel[pulley_dof] = -omega
        previous_speed = speed
        mujoco.mj_step(model, data)
        data.qvel[pulley_dof] = -omega
        if (
            predicted_transition is not None
            and previous_speed < rim_speed
            and data.qvel[rope_dof] >= rim_speed
        ):
            data.qvel[rope_dof] = rim_speed

    sliding_rows = [row for row in rows if row["regime"] == "sliding"]
    speed_errors = [abs(float(row["speed_error_m_s"])) for row in rows]
    equation_residuals = [
        abs(float(row["equation_11_residual_m2_s2"]))
        for row in sliding_rows
    ]
    wrap_residuals = [
        abs(float(row["kinetic_wrap_residual_N"]))
        for row in sliding_rows
    ]
    simulated_peak = max(float(row["rope_speed_m_s"]) for row in rows)
    final_row = rows[-1]
    transition_error = (
        abs(detected_transition - predicted_transition)
        if detected_transition is not None and predicted_transition is not None
        else 0.0
    )
    static_feasible = all(
        1.0 / coefficients.exponential - 1e-12
        <= float(row["tension_ratio"])
        <= coefficients.exponential + 1e-12
        for row in rows
        if row["regime"] == "stick"
    )
    checks = {
        "speed_matches_closed_form": max(speed_errors) <= 8e-4,
        "official_equation_11_closes": max(equation_residuals) <= 1e-11,
        "distributed_wrap_relation_closes": max(wrap_residuals) <= 1e-11,
        "maximum_speed_branch_matches": (
            abs(simulated_peak - predicted_maximum) / predicted_maximum
            <= (2e-3 if predicted_transition is None else 2e-4)
        ),
        "stick_transition_time_matches": transition_error <= timestep * 2.1,
        "static_tension_ratio_is_feasible": static_feasible,
        "tensions_remain_nonnegative": min(
            min(float(row["left_tension_N"]), float(row["right_tension_N"]))
            for row in rows
        )
        >= 0.0,
    }
    summary = {
        "case": case_name,
        "omega_rad_s": omega,
        "rim_speed_m_s": rim_speed,
        "predicted_branch": (
            "sliding_limited" if predicted_transition is None else "stick_limited"
        ),
        "predicted_sliding_limit_m_s": coefficients.sliding_speed_limit,
        "predicted_maximum_speed_m_s": predicted_maximum,
        "simulated_peak_speed_m_s": simulated_peak,
        "simulated_final_speed_m_s": float(final_row["rope_speed_m_s"]),
        "analytic_final_speed_m_s": float(final_row["analytic_speed_m_s"]),
        "maximum_absolute_speed_error_m_s": max(speed_errors),
        "predicted_stick_transition_s": predicted_transition,
        "detected_stick_transition_s": detected_transition,
        "stick_transition_error_s": transition_error,
        "maximum_equation_11_residual_m2_s2": max(equation_residuals),
        "maximum_kinetic_wrap_residual_N": max(wrap_residuals),
        "checks": checks,
        "pass": all(checks.values()),
    }
    return rows, summary


def simulate(
    model_path: Path = DEFAULT_MODEL,
    sliding_duration: float = 3.0,
    sticking_duration: float = 1.2,
    sample_stride: int = 10,
) -> tuple[list[dict[str, float | str]], dict[str, Any]]:
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    parameters = load_parameters(model)
    coefficients = official_coefficients(parameters)
    cases = (
        (
            "sliding_limited",
            numeric(model, "sliding_case_omega"),
            sliding_duration,
        ),
        (
            "stick_limited",
            numeric(model, "sticking_case_omega"),
            sticking_duration,
        ),
    )
    rows: list[dict[str, float | str]] = []
    case_summaries: list[dict[str, Any]] = []
    for case_name, omega, duration in cases:
        case_rows, case_summary = simulate_case(
            model_path, case_name, omega, duration, sample_stride
        )
        rows.extend(case_rows)
        case_summaries.append(case_summary)

    checks = {
        f"{summary['case']}_{name}": passed
        for summary in case_summaries
        for name, passed in summary["checks"].items()
    }
    summary: dict[str, Any] = {
        "pass": all(checks.values()),
        "model": str(model_path.resolve().relative_to(ROOT)),
        "source_problem": "35th CPhO (2018) final theory problem 3",
        "implementation_boundary": (
            "official massive-rope ODE applied to a MuJoCo transport DOF; "
            "not the current massless MuJoCable constitutive law"
        ),
        "timestep_s": float(model.opt.timestep),
        "parameters": {
            "linear_density_kg_m": parameters.linear_density,
            "pulley_radius_m": parameters.radius,
            "axis_height_m": parameters.height,
            "kinetic_friction": parameters.friction,
            "gravity_m_s2": parameters.gravity,
            "exp_mu_pi": coefficients.exponential,
            "official_sliding_limit_m_s": coefficients.sliding_speed_limit,
            "official_time_scale_s": coefficients.time_scale,
        },
        "cases": case_summaries,
        "checks": checks,
    }
    return rows, summary


def write_results(
    rows: list[dict[str, float | str]],
    summary: dict[str, Any],
    output_dir: Path,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "cpho_2018_problem3_timeseries.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output_dir / "cpho_2018_problem3_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def plot_results(
    rows: list[dict[str, float | str]], output_dir: Path
) -> None:
    os.environ.setdefault(
        "MPLCONFIGDIR",
        str(Path(tempfile.gettempdir()) / "mujocable_matplotlib_cache"),
    )
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
            "font.size": 10,
            "axes.labelsize": 10,
            "legend.fontsize": 9,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "lines.linewidth": 1.3,
        }
    )
    by_case = {
        case: [row for row in rows if row["case"] == case]
        for case in ("sliding_limited", "stick_limited")
    }
    fig, axes = plt.subplots(2, 2, figsize=(7.16, 5.2), constrained_layout=True)

    for axis, case, title in (
        (axes[0, 0], "sliding_limited", "(a) Persistent sliding"),
        (axes[0, 1], "stick_limited", "(b) Sliding-to-stick transition"),
    ):
        case_rows = by_case[case]
        time = np.asarray([float(row["time_s"]) for row in case_rows])
        speed = np.asarray([float(row["rope_speed_m_s"]) for row in case_rows])
        analytic = np.asarray(
            [float(row["analytic_speed_m_s"]) for row in case_rows]
        )
        rim = np.asarray(
            [float(row["pulley_rim_speed_m_s"]) for row in case_rows]
        )
        axis.plot(time, analytic, "-", label="Official solution")
        axis.plot(
            time[::25], speed[::25], "o", markersize=3, fillstyle="none",
            label="MuJoCo integration",
        )
        axis.plot(time, rim, "--", label=r"$R\omega$")
        axis.set_title(title)
        axis.set_xlabel("Time (s)")
        axis.set_ylabel("Rope speed (m/s)")
        axis.legend(frameon=False)

    sliding_rows = by_case["sliding_limited"]
    sliding_time = np.asarray(
        [float(row["time_s"]) for row in sliding_rows]
    )
    axes[1, 0].plot(
        sliding_time,
        [float(row["left_tension_N"]) for row in sliding_rows],
        "-",
        label=r"$T_1$",
    )
    axes[1, 0].plot(
        sliding_time,
        [float(row["right_tension_N"]) for row in sliding_rows],
        "--",
        label=r"$T_2$",
    )
    axes[1, 0].set_title("(c) Tangency tensions")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel("Tension (N)")
    axes[1, 0].legend(frameon=False)

    speed_squared = np.asarray(
        [float(row["rope_speed_m_s"]) ** 2 for row in sliding_rows]
    )
    acceleration = np.asarray(
        [float(row["acceleration_m_s2"]) for row in sliding_rows]
    )
    order = np.argsort(speed_squared)
    axes[1, 1].plot(
        speed_squared[order], acceleration[order], "-", label="Eq. (11)"
    )
    axes[1, 1].plot(
        speed_squared[::35],
        acceleration[::35],
        "s",
        markersize=3,
        fillstyle="none",
        label="MuJoCo states",
    )
    axes[1, 1].set_title("(d) Continuum dynamics")
    axes[1, 1].set_xlabel(r"Squared speed $v^2$ (m$^2$/s$^2$)")
    axes[1, 1].set_ylabel(r"Acceleration $\dot v$ (m/s$^2$)")
    axes[1, 1].legend(frameon=False)

    for axis in axes.flat:
        axis.grid(True, color="0.86", linewidth=0.6)
        axis.tick_params(direction="in")
    base = output_dir / "cpho_2018_problem3_validation"
    fig.savefig(base.with_suffix(".pdf"))
    fig.savefig(base.with_suffix(".svg"))
    fig.savefig(base.with_suffix(".png"), dpi=600)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--sliding-duration", type=float, default=3.0)
    parser.add_argument("--sticking-duration", type=float, default=1.2)
    parser.add_argument("--sample-stride", type=int, default=10)
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()

    rows, summary = simulate(
        args.model,
        sliding_duration=args.sliding_duration,
        sticking_duration=args.sticking_duration,
        sample_stride=args.sample_stride,
    )
    write_results(rows, summary, args.output_dir)
    if not args.no_plot:
        plot_results(rows, args.output_dir)
    print(json.dumps(summary, indent=2))
    return 1 if args.strict and not summary["pass"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
