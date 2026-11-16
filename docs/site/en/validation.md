# Validation and Reproduction

## Evidence Levels

1. **Analytic and unit checks**: unilateral law, cylinder tangency, virtual work, Capstan ratios and invalid configurations.
2. **Mechanism experiments**: pulleys, winches, Atwood, rolling contact and friction sensitivity.
3. **Videos**: useful observations of routes and motion, not substitutes for error, energy or runtime statistics.
4. **Hardware**: the public cases do not include new repeated-trial raw hardware records; simulation images alone establish no hardware accuracy.

## Public Current-Plugin Reproductions

The following values are recorded in `examples/paper/latest_plugin/`, dated 2026-11-04, using the optimized plugin. Building the manual does not rerun or rewrite these experiments. Source/library hashes are in the [run manifest](https://github.com/MuJoCable/mujoco-cable-dynamics/blob/main/examples/paper/latest_plugin/RUN_MANIFEST.json).

| Check | Recorded result | Interpretation |
|---|---|---|
| Fixed pulley, 0.2 kg, mu=0 | Load tension 1.962 N; lift RMSE 0.161 mm | Consistent with $mg=0.2\times9.81$ and 1:1 travel |
| Fixed pulley, mu=0.15 | Peak reference ratio 1.3674; measured 1.3614 | Approximately 0.45% difference for regularized sliding |
| Free inertial sheave | Peak 0.605 rad/s; torque RMSE $4.09\times10^{-5}$ N m | Passive rotation from tension difference |
| Ideal winch/sheave | Rim-travel RMSE 0.000473 mm | Equality-enforced baseline, not emergent friction-only no slip |
| Moving pulley | 20 mm take-up; 10.185 mm lift | Approximately 2:1 travel |
| Atwood landing | Landing-time error -9.38% | Retained ideal-model discrepancy, not a perfect match |
| Faive PIP | Virtual hinge 49.994 deg; cable 52.481 deg | Different input/transmission models |
| SpiRobs, mu=0.15 | Plugin total bend -522.970 deg; native -521.833 deg | Similar shape need not imply similar transmission loads |
| Calibrated bidirectional cycle | Right -524.052 deg; left 523.770 deg; final error 0.406 deg | Sum of 18 joint angles, not one joint angle |
| Two taut cables | Both sampled minima above 0.421 N | Specific to this trajectory and its sampled times |

The seven Result 1 runs and the listed Faive/friction cases report maximum route status 0. Their `contact_fraction` is rigid-body contact fraction, not route validity fraction. <a href="../downloads/latest-validation.zip">Download the full numerical records</a>.

<img class="example-poster" src="../_static/media/friction_summary.png" alt="Five guide-friction conditions: total bend, source tension and cable take-up">

This is deterministic sensitivity analysis without repeated-hardware confidence intervals. A change in simulated shape or transmission with friction is not an identified material coefficient.

## Reproduce Locally

After [installation](installation.md), run from the repository root:

```bash
python -m pip install -r examples/paper/requirements.txt
python examples/paper/run.py audit --check-videos
python examples/paper/run.py result1 --skip-videos
python examples/paper/run.py result2 --skip-videos
python examples/paper/run.py cycle --duration 24 --guide-friction-mu 0.015
python examples/paper/run.py benchmark --repeats 7 --duration 6
```

Omit `--skip-videos` to render new videos; this requires OpenGL and FFmpeg. New files default to `outputs/paper_latest/` without overwriting the archive. A load/short-step audit is not full-motion stability validation.

## Test a New Robot

| Check | Method | Important detail |
|---|---|---|
| No pushing while slack | Add reserve; inspect tension and force | Nonnegative source tension alone does not prove route validity |
| Virtual work | Frictionless central differences of $L(q)$ versus $-T\partial L/\partial q$ | Preserve branch; perturb freejoints in tangent space |
| Friction ratio | Known wrap angle versus $e^{\mu\Theta}$ | Near-zero-speed regularization need not reach the full ratio |
| No penetration | Segment intersection, surface distance and status | Do not rely only on transparent rendering |
| Timestep convergence | Halve timestep and compare paths/tensions | Match continuous control times, not step numbers |
| Speed and energy | Input work, load energy and dissipation | Include controller work and bearing losses |
| Runtime scaling | Isolated mesh/cable-count comparisons | Separate mean/p95, initialization and steady steps |

Keep failed conditions and residuals instead of hiding them with display smoothing. See [downloads](downloads.md) for the original derivations and statistics.
