# Demo 44/49 complete-bending friction pipeline

All cases use the same target cycle: right full bend, neutral, left full bend, and neutral. Only `guide_friction_mu` changes within each model. Both cables retain positive target extension through the geometry-following antagonist controller.

## Stroke ranking

The primary rank is the maximum absolute spool take-up from the neutral state at either full-bend hold. This is the minimum one-direction actuator excursion required by the simulated trajectory. The bidirectional span is the maximum-to-minimum cable range over the complete cycle.

| Rank | Model | Friction | Drive extension (mm) | One-way stroke (mm) | Bidirectional span (mm) | Completion | Both taut |
|---:|---|---:|---:|---:|---:|---:|:---:|
| 1 | TPU95HF connector calibration | 0.015 | 15.0 | 88.610 | 171.244 | 98.8% | yes |

For hardware, begin with the shortest-ranked stroke, add a measured safety margin rather than commanding the simulated limit directly, and stop on tension or contact thresholds. `guide_friction_mu` is a model parameter and must be identified against the printed guide, cable material, wrap angle, and preload.

## Reproduce

```bash
conda activate rope_plugin
mjpython scripts/run_full_bending_friction_pipeline.py \
  --plugin build/plugin/libcable_unilateral.dylib
```

Use `--skip-videos` for a fast CSV/figure-only run. Each case directory contains the portable generated MJCF, full time series, servo trajectory, vector/raster figures, raw video, annotated video, and a JSON summary.

Pipeline settings: reserve=90.0 mm, preload extension=1.0 mm, spool diameter=20 mm.
